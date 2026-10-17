"""Agents in their own processes (ADR-31, AB-6: tier 3 of the boundary's enforcement; docs/sdk/agents.md, "Isolation").

An MA's code runs in a process of its own and holds one thing: a channel to the range's process, through which it makes
its agent's calls - the calls of :class:`fsim.Agent` and :class:`fsim.AgentActivity`, nothing else. Its world, the
vehicles, the referee, the network and every other object stay in the range's process, out of its address space: it
cannot reach them even if it tries.

The range keeps the agents and the stepping::

    with fsim.isolation.AgentProcess(world, agent, "my_ma:Policy", path=["policies"]) as ma:
        for _ in range(steps):
            ma.step()          # the MA's Policy.step(), in its process; its agent calls come back here, one by one
            world.step(30)

Lockstep: the range asks one MA at a time to decide (`call`), serving that MA's agent calls until it answers, so the
calls reach the world in the same order as in one process - the isolated run reproduces the in-process run to the bit.

What crosses the channel: length-prefixed JSON (multiprocessing.connection's byte messages, HMAC-authenticated at
connection) with typed values - numbers (floats exactly: their shortest round-trip form), strings, bytes, lists, tuples,
dicts, fsim's named tuples, enums and batch commands, NumPy arrays of numbers, an activity by its id. Nothing is unpickled on either
side: an agent's message is data the range checks - a call it may make, on an activity it holds - never code it runs.

A run that claims isolation must give the MA no other way to the truth: a published world (its shared-memory segment
any local process may open) or one being recorded (its file) is refused unless `allow_side_channels`.

Mission episodes (docs/sdk/mission-training.md, section 16; M5 core) cross the same channel with less: an actor in its
own process holds a :class:`RemoteMissionEndpoint` - `observation_json`, `submit_json` and `receipts_json`, documents as
text, and nothing else - while :class:`MissionEndpointProcess` keeps the episode, its endpoint and its referee in the
range's process. A diagnostic episode (published or recorded) is refused, and neither a range, World, referee nor a
referee document is carried across. An endpoint is its generation's: after a reset the range re-binds the process
explicitly (`rebind`), or its calls fail as the stale endpoint's do.
"""
import inspect
import json
import os
import secrets
import subprocess
import sys
import threading
import time

from multiprocessing.connection import Listener

# The codec of what crosses and the actor-side proxies with their process's loop are fsim._adapters' (ARCH-07, P056);
# this module keeps the Range-side checks and process hosts and its public names, which stay its own. The codec carries
# two public value classes besides the named tuples: bound here, as it imports no public module.
from ._adapters import actor as _actor
from ._adapters import codec as _codec
from ._adapters.actor import RemoteAgent, RemoteMissionEndpoint
from ._adapters.codec import AgentError, _decode, _dump, _encode, _exception, _load, _rebuild
from .agent import AgentActivity as _AgentActivity
from .world import BatchCommand as _BatchCommand

_codec.bind(_BatchCommand, _AgentActivity)

__all__ = ["AgentProcess", "AgentError", "agent_calls", "activity_calls", "MissionEndpointProcess", "RemoteMissionEndpoint",
           "mission_endpoint_calls"]


# --- what may be called -----------------------------------------------------------------------------------------------

def _calls(cls):
    """A class's public members, by kind: ``property`` (read) or ``method`` (called)."""
    out = {}
    for name in dir(cls):
        if name.startswith("_"):
            continue
        member = inspect.getattr_static(cls, name)
        if isinstance(member, property) or type(member).__name__ in ("member_descriptor", "getset_descriptor"):
            out[name] = "property"  # (its identity's slots among them: vehicle_id, controller, agent_name)
        elif callable(member):
            out[name] = "method"
    return out


def agent_calls():
    """What an isolated MA may call on its agent: fsim.Agent's members (every one an agent's: docs/sdk/boundary.toml)."""
    from .agent import Agent
    return _calls(Agent)


def activity_calls():
    """What it may call on an activity it holds: fsim.AgentActivity's members."""
    from .agent import AgentActivity
    return _calls(AgentActivity)


def mission_endpoint_calls():
    """What an isolated actor may call on its mission endpoint: section 16's whitelist, and nothing else."""
    return {name: "method" for name in _MISSION_CALLS}


_MISSION_CALLS = ("observation_json", "submit_json", "receipts_json")
_REFEREE_SCHEMAS = ("ma.outcome.v1", "ma.truth.v1", "ma.costs.v1")  # (the referee's documents: never an actor's)


def _carried(value, depth=0):
    """What strict isolation lets cross to an actor: plain data - never a range, World, Agent, referee, endpoint or a
    referee document (section 16), whether as text, bytes or a parsed object, at any depth: a mapping's schema is
    checked before its members, and text that parses as JSON is checked as what it parses to."""
    from .agent import Agent, Referee
    from .mission import MissionBatch, MissionEndpoint, MissionEpisode, MissionReferee
    from .world import World
    if depth > 64:
        raise TypeError("isolation: an argument nested more than 64 deep does not cross to an actor")
    if isinstance(value, (MissionBatch, MissionEpisode, MissionReferee, MissionEndpoint, World, Referee, Agent)):
        raise TypeError("isolation: a %s does not cross to an actor" % type(value).__name__)
    if isinstance(value, (str, bytes, bytearray, memoryview)):
        try:
            document = json.loads(value if isinstance(value, (str, bytes)) else bytes(value))
        except (ValueError, TypeError):
            return value
        if isinstance(document, (dict, list)):
            _carried(document, depth + 1)
        return value
    if isinstance(value, dict):
        if value.get("schema") in _REFEREE_SCHEMAS:
            raise TypeError("isolation: a referee document (%s) does not cross to an actor" % value["schema"])
        for k, v in value.items():
            _carried(k, depth + 1), _carried(v, depth + 1)
    elif isinstance(value, (list, tuple, set, frozenset)):
        for v in value:
            _carried(v, depth + 1)
    return value


def _own_endpoint(episode, endpoint, service_id=None):
    """The endpoint the process may hold: its episode's own (the episode's handle opened it - an equal service id or
    episode key is no proof), of the service it serves, of the episode's current generation, and still open."""
    from .mission import MissionEndpoint
    if not isinstance(endpoint, MissionEndpoint):
        raise TypeError("isolation: an fsim.mission.MissionEndpoint is what crosses, nothing else")
    if not episode._owns(endpoint):
        raise ValueError("isolation: the endpoint is not one this episode opened, or it is closed")
    if service_id is not None and endpoint.service_id != service_id:
        raise ValueError("isolation: re-bound to an endpoint of the same service alone")
    import fsim
    try:
        endpoint.receipts_json()  # (an endpoint answers in its episode's current generation alone)
    except fsim.Error as e:
        if "busy" in str(e):
            raise
        raise ValueError("isolation: the endpoint is not of its episode's current generation: %s" % e) from None


# --- the range's side ----------------------------------------------------------------------------------------------------

class AgentProcess:
    """An MA in a process of its own (tier 3): `entry` ("module:Class") made there with its agent - a channel back to
    `agent` here - and `args`; `path` added to its import path. The range calls its methods (`step`, `call`) one MA at a
    time, serving its agent calls while it decides. `world` is the range's World: a published or recorded one is
    refused unless `allow_side_channels` (any local process could read its truth there)."""

    def __init__(self, world, agent, entry, *args, path=(), python=None, allow_side_channels=False, timeout_s=60.0):
        if not allow_side_channels and (world.published or world.recording):
            raise RuntimeError("isolation: the world is %s - any local process could read its truth there; make it with "
                               "publish=False and no record_path, or pass allow_side_channels=True"
                               % ("published" if world.published else "recorded"))
        from .agent import Agent
        if not isinstance(agent, Agent):
            raise TypeError("isolation: an fsim.Agent is what crosses, nothing else")
        self._agent, self._activities = agent, {}
        self._agent_calls, self._activity_calls = agent_calls(), activity_calls()
        self.calls = 0          # agent calls served
        self.channel_s = 0.0    # wall time spent in the channel and the MA's process
        key = secrets.token_bytes(32)
        self._listener = Listener(family="AF_PIPE" if os.name == "nt" else "AF_UNIX", authkey=key)
        env = dict(os.environ)
        package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env["PYTHONPATH"] = os.pathsep.join([package] + [os.path.abspath(p) for p in path] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
        # (the MA's process: the address and the key, on its standard input - not its command line)
        self._process = subprocess.Popen([python or sys.executable, "-m", "fsim.isolation"], stdin=subprocess.PIPE, env=env)
        self._process.stdin.write(_dump({"address": self._listener.address, "key": key.hex()}) + b"\n")
        self._process.stdin.close()
        accepted = []
        thread = threading.Thread(target=lambda: accepted.append(self._listener.accept()), daemon=True)
        thread.start()
        thread.join(timeout_s)
        if not accepted:
            self._process.kill()
            raise RuntimeError("isolation: the MA's process did not connect (exit code %s)" % self._process.poll())
        self._conn = accepted[0]
        self._exchange({"op": "init", "entry": entry, "args": [_encode(a) for a in args]})

    # the MA's methods, called in its process
    def call(self, method, *args, **kwargs):
        """Call the MA's `method` in its process; its agent's calls are served meanwhile. Its result, or its exception
        (AgentError, or the fsim exception its agent call raised and it did not catch)."""
        return self._exchange({"op": "call", "m": method, "a": [_encode(a) for a in args],
                               "k": {k: _encode(v) for k, v in kwargs.items()}})

    def step(self, *args, **kwargs):
        return self.call("step", *args, **kwargs)

    def _exchange(self, message):
        start = time.perf_counter()
        self._conn.send_bytes(_dump(message))
        try:
            while True:
                m = _load(self._conn.recv_bytes())
                op = m.get("op")
                if op == "return":
                    return _decode(m.get("v"), self._activity)
                if op == "raise":  # (the MA's own exception: AgentError, unless an agent call's fsim one it let through)
                    raise _rebuild(m["x"], builtins=False)
                if op in ("agent", "activity"):
                    self._conn.send_bytes(_dump(self._serve(m)))
                else:
                    raise RuntimeError("isolation: the MA's process sent %r" % op)
        finally:
            self.channel_s += time.perf_counter() - start

    def _activity(self, aid):
        held = self._activities.get(aid)
        if held is None:
            raise ValueError("isolation: activity %d is not one its agent holds" % aid)
        return held

    def _serve(self, m):
        """One agent call: a member of fsim.Agent or of an activity it holds - nothing else is reachable."""
        self.calls += 1
        try:
            name = m.get("m")
            if m["op"] == "agent":
                kinds, target = self._agent_calls, self._agent
            else:
                kinds, target = self._activity_calls, self._activity(int(m.get("id")))
            kind = kinds.get(name) if isinstance(name, str) else None
            if kind is None:
                raise AttributeError("isolation: %r is not an agent's call" % (name,))
            if kind == "property":
                value = getattr(target, name)
            else:
                args = [_decode(a, self._activity) for a in m.get("a", [])]
                kwargs = {str(k): _decode(v, self._activity) for k, v in m.get("k", {}).items()}
                value = getattr(target, name)(*args, **kwargs)
            return {"op": "result", "v": _encode(value, self._activities)}
        except Exception as exc:  # (its answer, as the in-process call would raise it)
            return {"op": "error", "x": _exception(exc)}

    def close(self):
        """End the MA's process."""
        if self._process is None:
            return
        try:
            self._conn.send_bytes(_dump({"op": "close"}))
            self._process.wait(10)
        except Exception:
            self._process.kill()
        finally:
            self._conn.close()
            self._listener.close()
            self._process = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


class MissionEndpointProcess:
    """An actor in a process of its own (section 16): `entry` ("module:Class") made there with a
    :class:`RemoteMissionEndpoint` - a channel back to `endpoint` here - and `args`; `path` added to its import path. The
    range calls the actor's methods (`step`, `call`) one at a time and serves its endpoint calls meanwhile; the episode
    and everything else stay in this process. A diagnostic episode is refused; so is any argument that is a range, a
    World, an Agent, a referee, an endpoint or a referee document."""

    def __init__(self, episode, endpoint, entry, *args, path=(), python=None, timeout_s=60.0):
        from .mission import MissionEndpoint, MissionEpisode
        if not isinstance(episode, MissionEpisode):
            raise TypeError("isolation: the range's fsim.mission.MissionEpisode is required")
        if episode.diagnostic:
            raise RuntimeError("isolation: a diagnostic episode (published or recorded) is labelled non-isolated and is refused")
        if not isinstance(endpoint, MissionEndpoint):
            raise TypeError("isolation: an fsim.mission.MissionEndpoint is what crosses, nothing else")
        _own_endpoint(episode, endpoint)
        for a in args:
            _carried(a)
        self._episode, self._endpoint = episode, endpoint
        self.calls = 0          # endpoint calls served
        self.channel_s = 0.0    # wall time spent in the channel and the actor's process
        key = secrets.token_bytes(32)
        self._listener = Listener(family="AF_PIPE" if os.name == "nt" else "AF_UNIX", authkey=key)
        env = dict(os.environ)
        package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env["PYTHONPATH"] = os.pathsep.join([package] + [os.path.abspath(p) for p in path] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
        self._process = subprocess.Popen([python or sys.executable, "-m", "fsim.isolation"], stdin=subprocess.PIPE, env=env)
        self._process.stdin.write(_dump({"address": self._listener.address, "key": key.hex(), "kind": "mission"}) + b"\n")
        self._process.stdin.close()
        accepted = []
        thread = threading.Thread(target=lambda: accepted.append(self._listener.accept()), daemon=True)
        thread.start()
        thread.join(timeout_s)
        if not accepted:
            self._process.kill()
            raise RuntimeError("isolation: the actor's process did not connect (exit code %s)" % self._process.poll())
        self._conn = accepted[0]
        self._exchange({"op": "init", "entry": entry, "args": [_encode(a) for a in args]})

    def rebind(self, endpoint):
        """Re-bind the process to a new generation's endpoint of the same service, explicitly (section 4): one its own
        episode opened, of that episode's current generation."""
        _own_endpoint(self._episode, endpoint, self._endpoint.service_id)
        self._endpoint = endpoint

    def call(self, method, *args, **kwargs):
        """Call the actor's `method` in its process, serving its endpoint calls meanwhile: its result, or its exception."""
        for a in list(args) + list(kwargs.values()):
            _carried(a)
        return self._exchange({"op": "call", "m": method, "a": [_encode(a) for a in args], "k": {k: _encode(v) for k, v in kwargs.items()}})

    def step(self, *args, **kwargs):
        return self.call("step", *args, **kwargs)

    def _exchange(self, message):
        start = time.perf_counter()
        self._conn.send_bytes(_dump(message))
        try:
            while True:
                m = _load(self._conn.recv_bytes())
                op = m.get("op")
                if op == "return":
                    return _decode(m.get("v"), _no_activity)
                if op == "raise":
                    raise _rebuild(m["x"], builtins=False)
                # (anything else it sends is refused as a call, never served: the endpoint's whitelist is all there is)
                self._conn.send_bytes(_dump(self._serve(m) if op == "endpoint" else
                                            {"op": "error", "x": _exception(AttributeError("isolation: %r is not a mission endpoint's call" % (op,)))}))
        finally:
            self.channel_s += time.perf_counter() - start

    def _serve(self, m):
        """One endpoint call of the whitelist; nothing else is reachable."""
        self.calls += 1
        try:
            name = m.get("m")
            if name not in _MISSION_CALLS:
                raise AttributeError("isolation: %r is not a mission endpoint's call" % (name,))
            args = [_decode(a, _no_activity) for a in m.get("a", [])]
            if name == "submit_json":
                if len(args) != 1 or not isinstance(args[0], (str, bytes)):
                    raise TypeError("submit_json(actions: str | bytes)")
            elif args:
                raise TypeError("%s() takes no argument" % name)
            return {"op": "result", "v": getattr(self._endpoint, name)(*args)}
        except Exception as exc:
            return {"op": "error", "x": _exception(exc)}

    def close(self):
        """End the actor's process."""
        if self._process is None:
            return
        try:
            self._conn.send_bytes(_dump({"op": "close"}))
            self._process.wait(10)
        except Exception:
            self._process.kill()
        finally:
            self._conn.close()
            self._listener.close()
            self._process = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def _no_activity(aid):
    raise ValueError("isolation: no activity crosses a mission endpoint's channel")


# --- the MA's side -------------------------------------------------------------------------------------------------------
# Its proxies (RemoteAgent, RemoteMissionEndpoint, an activity it holds) and its process's loop are fsim._adapters.actor's.

def _serve_ma():
    """The MA's process: connect, make the MA, answer the range's calls until it closes."""
    _actor.serve(agent_calls(), activity_calls())


if __name__ == "__main__":
    _serve_ma()
