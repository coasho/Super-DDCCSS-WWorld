"""Example-only contracts of the composition example (docs/sdk/ma-composition.md, section 3).

These are conventions of this example, written against the installed public `fsim` package alone. They are not SDK
plug-in interfaces, carry no vtable, and nothing a production MA runtime must inherit from: the formal runtime - its
actors, behaviour trees, planners, blackboards and arbitration - belongs to the upper-layer project.
"""
from collections import namedtuple
import types

MASK64 = (1 << 64) - 1


class Proposal(namedtuple("Proposal", "kind operation parameters resources priority")):
    """One intent a component proposes: an `ma.actions.v1` request body (kind, operation, parameters), the resources it
    would hold (names the coordinator arbitrates: "flight", "radar", "c2_task:<from>:<id>", ...) and its priority."""

    __slots__ = ()


class EpisodeContext(namedtuple("EpisodeContext", "episode_key service_id service_seed component_seeds")):
    """What a new generation gives the system: its episode key (the schema's `episode_key`, as the snapshot carries it),
    the service's identity and the seed streams of mission-training.md, section 8."""

    __slots__ = ()

    def component(self, component_id):
        """A component's own context: the episode key, the service and that component's seed alone."""
        return ComponentContext(self.episode_key, self.service_id, self.component_seeds[component_id])


ComponentContext = namedtuple("ComponentContext", "episode_key service_id seed")


class MAComponent:
    """One component behind a service: proposes on ticks where tick % period_ticks == 0."""

    component_id = 0      # stable, unique within its service
    period_ticks = 1

    def reset(self, context):
        """A new generation: forget everything, re-seed from `context`."""
        raise NotImplementedError

    def propose(self, snapshot):
        """The tick's one snapshot (read-only) -> a list of Proposal."""
        raise NotImplementedError

    def feedback(self, receipts):
        """The SDK receipts of requests this component originated, as they change."""
        raise NotImplementedError

    def state(self):
        """Everything the component remembers, as plain data (what a reset must clear)."""
        raise NotImplementedError


class MASystem:
    """An MA system behind one endpoint: owns its blackboard, component timing, arbitration and lifecycle."""

    def begin_episode(self, context):
        raise NotImplementedError

    def tick(self, endpoint):
        raise NotImplementedError

    def end_episode(self, outcome):
        raise NotImplementedError


# --- seed streams (mission-training.md, section 8) ----------------------------------------------------------------------

def _next(x):
    """fsim::Rng(x).next(): one SplitMix64 step."""
    x = (x + 0x9E3779B97F4A7C15) & MASK64
    z = x
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return z ^ (z >> 31)


def stream(base_seed, env_id, local_episode_index, stream_class):
    return _next(_next(_next(_next(base_seed) ^ env_id) ^ local_episode_index) ^ stream_class)


def service_seed(base_seed, env_id, local_episode_index, service_id):
    return _next(stream(base_seed, env_id, local_episode_index, 1) ^ service_id)


def component_seed(base_seed, env_id, local_episode_index, service_id, component_id):
    return _next(service_seed(base_seed, env_id, local_episode_index, service_id) ^ component_id)


def context(base_seed, episode_key, service_id, component_ids):
    """The context of a generation, from its snapshot's episode key: seeds never depend on the generation itself."""
    env_id, local = int(episode_key["env_id"]), int(episode_key["local_episode_index"])
    seeds = {c: component_seed(base_seed, env_id, local, service_id, c) for c in component_ids}
    return EpisodeContext(dict(episode_key), str(service_id), service_seed(base_seed, env_id, local, service_id), seeds)


# --- one read-only snapshot ---------------------------------------------------------------------------------------------

def frozen(value):
    """A parsed document no component can change: mappings read-only, arrays tuples."""
    if isinstance(value, dict):
        return types.MappingProxyType({k: frozen(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(frozen(v) for v in value)
    return value
