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
"""
import base64
import enum
import importlib
import inspect
import json
import os
import secrets
import subprocess
import sys
import threading
import time
import traceback

from multiprocessing.connection import Client, Listener

__all__ = ["AgentProcess", "AgentError", "agent_calls", "activity_calls"]


class AgentError(RuntimeError):
    """The MA's code raised in its process: its exception's type, message and traceback, as text."""


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


# --- the values that cross ----------------------------------------------------------------------------------------------

_registry = None


def _types():
    """fsim's named tuples and enums, by qualified name: the only classes the channel rebuilds."""
    global _registry
    if _registry is None:
        import fsim
        named, enums = {}, {}
        for module in {sys.modules[n] for n in list(sys.modules) if n == "fsim" or n.startswith("fsim.")}:
            for name, obj in vars(module).items():
                if not isinstance(obj, type) or getattr(obj, "__module__", "").split(".")[0] != "fsim":
                    continue
                key = "%s.%s" % (obj.__module__, obj.__qualname__)
                if issubclass(obj, tuple) and hasattr(obj, "_fields"):
                    named[key] = obj
                elif issubclass(obj, enum.Enum):
                    enums[key] = obj
        _registry = (named, enums, fsim)
    return _registry


def _key(cls):
    return "%s.%s" % (cls.__module__, cls.__qualname__)


def _encode(value, activities=None):
    """A value as JSON's types, tagged where JSON has none."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, enum.Enum):  # (before int: an IntEnum is one)
        return {"$e": _key(type(value)), "v": value.value}
    if isinstance(value, (int, float)):
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"$b": base64.b64encode(bytes(value)).decode("ascii")}
    if isinstance(value, tuple) and hasattr(value, "_fields"):
        return {"$n": _key(type(value)), "v": [_encode(v, activities) for v in value]}
    if isinstance(value, tuple):
        return {"$t": [_encode(v, activities) for v in value]}
    if isinstance(value, list):
        return [_encode(v, activities) for v in value]
    if isinstance(value, dict):
        return {"$d": [[_encode(k, activities), _encode(v, activities)] for k, v in value.items()]}
    from .world import BatchCommand
    if isinstance(value, BatchCommand):  # (fsim's one value class besides its named tuples: a batch item, a setpoint)
        return {"$bc": [value.method, _encode(tuple(value.args), activities), _encode(value.kwargs, activities)]}
    from .agent import AgentActivity
    if isinstance(value, AgentActivity):
        if activities is None:
            raise TypeError("an activity crosses from the range alone")
        activities[value.id] = value
        return {"$a": value.id}
    if isinstance(value, _RemoteActivity):
        return {"$a": value.id}
    if type(value).__module__ == "numpy":
        import numpy as np
        if isinstance(value, np.ndarray):
            if value.dtype.kind not in "biuf" or value.dtype.names:
                raise TypeError("not transferable: an array of %s" % value.dtype)
            return {"$np": value.dtype.str, "s": list(value.shape), "b": base64.b64encode(np.ascontiguousarray(value).tobytes()).decode("ascii")}
        if isinstance(value, np.generic):
            return _encode(value.item(), activities)
    raise TypeError("not transferable: %s" % type(value).__name__)


def _decode(value, activity):
    """A value back from JSON's types: `activity(id)` resolves an activity reference (the range: one the agent holds)."""
    if isinstance(value, list):
        return [_decode(v, activity) for v in value]
    if not isinstance(value, dict):
        return value
    named, enums, _ = _types()
    if "$n" in value:
        cls = named.get(value["$n"])
        if cls is None:
            raise TypeError("not a type the channel carries: %s" % value["$n"])
        return cls(*[_decode(v, activity) for v in value["v"]])
    if "$e" in value:
        cls = enums.get(value["$e"])
        if cls is None:
            raise TypeError("not a type the channel carries: %s" % value["$e"])
        return cls(value["v"])
    if "$t" in value:
        return tuple(_decode(v, activity) for v in value["$t"])
    if "$d" in value:
        return {_decode(k, activity): _decode(v, activity) for k, v in value["$d"]}
    if "$b" in value:
        return base64.b64decode(value["$b"])
    if "$a" in value:
        return activity(int(value["$a"]))
    if "$bc" in value:
        from .world import BatchCommand
        method, args, kwargs = value["$bc"]
        return BatchCommand(str(method), *_decode(args, activity), **{str(k): v for k, v in _decode(kwargs, activity).items()})
    if "$np" in value:
        import numpy as np
        dtype = np.dtype(value["$np"])
        if dtype.kind not in "biuf" or dtype.names:
            raise TypeError("not transferable: an array of %s" % dtype)
        return np.frombuffer(base64.b64decode(value["b"]), dtype=dtype).reshape(value["s"]).copy()
    raise TypeError("not a value the channel carries")


def _dump(message):
    return json.dumps(message, separators=(",", ":")).encode("utf-8")


def _load(data):
    return json.loads(data.decode("utf-8"))


def _exception(exc):
    """An exception as data: its fsim type's name (or a builtin's), its arguments and attributes."""
    attrs = {}
    for k, v in getattr(exc, "__dict__", {}).items():
        try:
            attrs[k] = _encode(v)
        except TypeError:
            pass
    try:
        args = [_encode(a) for a in exc.args]
    except TypeError:
        args = [str(exc)]
    return {"type": _key(type(exc)), "args": args, "attrs": attrs, "text": "".join(traceback.format_exception_only(type(exc), exc)).strip()}


_BUILTIN = {c.__name__: c for c in (ValueError, TypeError, KeyError, IndexError, AttributeError, RuntimeError, OverflowError,
                                    ZeroDivisionError, NotImplementedError)}


def _rebuild(x, builtins=True):
    """The exception back - one of fsim's (fsim.Error, Rejected, MissionRejected ...) or, `builtins`, a builtin's (an agent
    call's: as the MA would catch it in one process) - else AgentError, with its traceback."""
    _, _, fsim = _types()
    module, _, name = x["type"].rpartition(".")
    cls = None
    if module == "builtins" and builtins:
        cls = _BUILTIN.get(name)
    elif module.split(".")[0] == "fsim":
        candidate = getattr(sys.modules.get(module), name, None)
        if isinstance(candidate, type) and issubclass(candidate, Exception):
            cls = candidate
    if cls is None:
        return AgentError(x["text"])
    exc = cls.__new__(cls)
    Exception.__init__(exc, *[_decode(a, lambda i: i) for a in x["args"]])
    for k, v in x["attrs"].items():
        try:
            setattr(exc, k, _decode(v, lambda i: i))
        except (AttributeError, TypeError):
            pass
    return exc


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


# --- the MA's side -------------------------------------------------------------------------------------------------------

class _RemoteActivity:
    """An activity its agent holds, in the MA's process: its calls go to the range's."""

    __slots__ = ("id", "_channel")

    def __init__(self, aid, channel):
        object.__setattr__(self, "id", aid)
        object.__setattr__(self, "_channel", channel)

    def __getattr__(self, name):
        kind = self._channel.activity_calls.get(name)
        if kind is None:
            raise AttributeError(name)
        if kind == "property":
            return self._channel.request({"op": "activity", "id": self.id, "m": name})
        return lambda *a, **k: self._channel.request({"op": "activity", "id": self.id, "m": name, "a": [_encode(v) for v in a],
                                                      "k": {key: _encode(v) for key, v in k.items()}})

    def __eq__(self, other):
        return isinstance(other, _RemoteActivity) and other.id == self.id

    def __hash__(self):
        return hash(self.id)

    def __repr__(self):
        return "AgentActivity(%d, remote)" % self.id


class RemoteAgent:
    """Its agent, as an MA in its own process holds it: fsim.Agent's calls, each made in the range's process."""

    __slots__ = ("_channel",)

    def __init__(self, channel):
        object.__setattr__(self, "_channel", channel)

    def __getattr__(self, name):
        kind = self._channel.agent_calls.get(name)
        if kind is None:
            raise AttributeError("%r is not an agent's call" % name)
        if kind == "property":
            return self._channel.request({"op": "agent", "m": name})
        return lambda *a, **k: self._channel.request({"op": "agent", "m": name, "a": [_encode(v) for v in a],
                                                      "k": {key: _encode(v) for key, v in k.items()}})

    def __repr__(self):
        return "RemoteAgent(%r)" % self.agent_name


class _Channel:
    def __init__(self, conn):
        self.conn = conn
        self.agent_calls, self.activity_calls = agent_calls(), activity_calls()

    def request(self, message):
        self.conn.send_bytes(_dump(message))
        reply = _load(self.conn.recv_bytes())
        if reply["op"] == "error":
            raise _rebuild(reply["x"])
        return _decode(reply["v"], lambda aid: _RemoteActivity(aid, self))


def _serve_ma():
    """The MA's process: connect, make the MA, answer the range's calls until it closes."""
    hello = json.loads(sys.stdin.readline())
    conn = Client(hello["address"], family="AF_PIPE" if os.name == "nt" else "AF_UNIX", authkey=bytes.fromhex(hello["key"]))
    channel = _Channel(conn)
    remote = RemoteAgent(channel)
    ma = None
    while True:
        m = _load(conn.recv_bytes())
        op = m.get("op")
        if op == "close":
            conn.close()
            return
        try:
            args = [_decode(a, lambda aid: _RemoteActivity(aid, channel)) for a in m.get("a", m.get("args", []))]
            if op == "init":
                module, _, name = m["entry"].partition(":")
                cls = getattr(importlib.import_module(module), name)
                ma = cls(remote, *args)
                result = None
            elif op == "call":
                kwargs = {k: _decode(v, lambda aid: _RemoteActivity(aid, channel)) for k, v in m.get("k", {}).items()}
                result = getattr(ma, m["m"])(*args, **kwargs)
            else:
                raise RuntimeError("isolation: the range sent %r" % op)
            conn.send_bytes(_dump({"op": "return", "v": _encode(result)}))
        except Exception as exc:
            x = _exception(exc)
            x["text"] = traceback.format_exc()
            conn.send_bytes(_dump({"op": "raise", "x": x}))


if __name__ == "__main__":
    _serve_ma()
