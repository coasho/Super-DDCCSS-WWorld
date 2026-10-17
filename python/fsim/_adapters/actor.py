"""The actor side of an isolated MA or actor (ADR-31, AB-6; docs/sdk/agents.md, "SDK access terminology" and
"Isolation"): the proxies its process holds - RemoteAgent, an agent's callable-method list; RemoteMissionEndpoint, a
mission endpoint's three calls - and the process's loop, which makes the MA from its "module:Class" entry and answers
the Range's calls. Each proxy call is answered in the Range's process.

fsim.isolation's actor-side implementation (ARCH-07, P056). It imports no public module: fsim.isolation passes the
callable-method lists (serve) and re-exports the proxies, whose owner it remains. Range-side hosts stay there."""
import importlib
import json
import os
import sys
import traceback

from multiprocessing.connection import Client

from .._contracts import public
from .codec import _RemoteActivity, _decode, _dump, _encode, _exception, _load, _rebuild

_public = public("fsim.isolation")


@_public
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


@_public
class RemoteMissionEndpoint:
    """Its mission endpoint, as an actor in its own process holds it: three calls, each answered in the range's process,
    documents as text - nothing else is there to reach."""

    __slots__ = ("_channel",)

    def __init__(self, channel):
        object.__setattr__(self, "_channel", channel)

    def observation_json(self):
        return self._channel.request({"op": "endpoint", "m": "observation_json"})

    def submit_json(self, actions):
        return self._channel.request({"op": "endpoint", "m": "submit_json", "a": [_encode(actions)]})

    def receipts_json(self):
        return self._channel.request({"op": "endpoint", "m": "receipts_json"})

    def __setattr__(self, name, value):
        raise AttributeError("a remote mission endpoint holds nothing to set")

    def __repr__(self):
        return "RemoteMissionEndpoint()"


class _Channel:
    def __init__(self, conn, agent_calls, activity_calls):
        self.conn = conn
        self.agent_calls, self.activity_calls = agent_calls, activity_calls

    def request(self, message):
        self.conn.send_bytes(_dump(message))
        reply = _load(self.conn.recv_bytes())
        if reply["op"] == "error":
            raise _rebuild(reply["x"])
        return _decode(reply["v"], lambda aid: _RemoteActivity(aid, self))


def serve(agent_calls, activity_calls):
    """The MA's process: connect, make the MA, answer the range's calls until it closes. `agent_calls` and
    `activity_calls` are fsim.isolation's callable-method lists, which its proxies answer."""
    hello = json.loads(sys.stdin.readline())
    conn = Client(hello["address"], family="AF_PIPE" if os.name == "nt" else "AF_UNIX", authkey=bytes.fromhex(hello["key"]))
    channel = _Channel(conn, agent_calls, activity_calls)
    remote = RemoteMissionEndpoint(channel) if hello.get("kind") == "mission" else RemoteAgent(channel)
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
