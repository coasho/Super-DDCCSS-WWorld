"""What crosses an isolated actor's local pipe (ADR-31, AB-6; docs/sdk/agents.md, "Isolation"): JSON with typed
values - fsim's named tuples and enums by qualified name, bytes, tuples, dicts, batch commands, NumPy arrays of numbers
and an activity by its id - and exceptions as data. Nothing is unpickled on either side.

fsim.isolation's codec (ARCH-07, P056). It imports no public module: fsim.isolation binds the two public value classes it
carries besides the named tuples (bind) and re-exports AgentError, whose owner it remains."""
import base64
import enum
import json
import sys
import traceback

from .._contracts import public

# fsim.world's BatchCommand and fsim.agent's AgentActivity, bound by fsim.isolation when it loads
_BatchCommand = _AgentActivity = None


def bind(batch_command, agent_activity):
    """fsim.isolation's BatchCommand and AgentActivity: the value classes the codec carries besides named tuples."""
    global _BatchCommand, _AgentActivity
    _BatchCommand, _AgentActivity = batch_command, agent_activity


@public("fsim.isolation")
class AgentError(RuntimeError):
    """The MA's code raised in its process: its exception's type, message and traceback, as text."""


# An activity its agent holds, as the MA's process holds it: what an activity reference decodes to there.
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


_registry = None


def _types():
    """fsim's named tuples and enums, by qualified name: the only classes the channel rebuilds."""
    global _registry
    if _registry is None:
        fsim = sys.modules["fsim"]  # (loaded: this module is reached through fsim.isolation)
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
    if isinstance(value, _BatchCommand):  # (fsim's one value class besides its named tuples: a batch item, a setpoint)
        return {"$bc": [value.method, _encode(tuple(value.args), activities), _encode(value.kwargs, activities)]}
    if isinstance(value, _AgentActivity):
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
        method, args, kwargs = value["$bc"]
        return _BatchCommand(str(method), *_decode(args, activity), **{str(k): v for k, v in _decode(kwargs, activity).items()})
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
