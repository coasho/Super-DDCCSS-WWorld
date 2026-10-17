"""fsim.training's implementation (ARCH-07; docs/architecture/design/directory-layout.md, section 4): the training
adapter contract and section 8's seed streams, dependency-free - no learning framework, optimizer or tensor library.
fsim.training re-exports the public names, which keep it as their owner. The native Error comes from its owner,
fsim._native, never from the package root."""
from collections import namedtuple
import json
import time

from .. import _native
from .._contracts import public

_public = public("fsim.training")

MASK64 = (1 << 64) - 1


def _next(x):
    """fsim::Rng(x).next(): one SplitMix64 step."""
    x = (x + 0x9E3779B97F4A7C15) & MASK64
    z = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return z ^ (z >> 31)


def _stream(base_seed, env_id, local_episode_index, stream_class):
    return _next(_next(_next(_next(base_seed) ^ env_id) ^ local_episode_index) ^ stream_class)


@_public
def world_seed(base_seed, env_id, local_episode_index):
    """The section 8 world seed of a local episode."""
    return _stream(base_seed, env_id, local_episode_index, 0)


@_public
def service_seed(base_seed, env_id, local_episode_index, service_id):
    """The section 8 seed of a service, offered to the upper layer."""
    return _next(_stream(base_seed, env_id, local_episode_index, 1) ^ service_id)


@_public
def component_seed(base_seed, env_id, local_episode_index, service_id, component_id):
    """The section 8 seed of a component behind a service."""
    return _next(service_seed(base_seed, env_id, local_episode_index, service_id) ^ component_id)


ComponentContext = namedtuple("ComponentContext", "episode_key service_id seed", module="fsim.training")
ComponentContext.__doc__ = """A component's own context in a generation: the episode key, its service and its section 8 seed."""


@_public
class EpisodeContext(dict):
    """What begin_episode gets for a generation: a mapping with "slot", "service_id" (a decimal string) and
    "episode_key" (the snapshot's), the same as attributes, and the generation's section 8 seeds - `service_seed` and
    `component(component_id)`, a :class:`ComponentContext`. Seeds come from the slot's base seed, env_id and local
    episode index alone, never from the generation."""

    def __init__(self, slot, service_id, episode_key, base_seed):
        super().__init__(slot=int(slot), service_id=str(service_id), episode_key=dict(episode_key))
        self._seed = (int(base_seed), int(episode_key["env_id"]), int(episode_key["local_episode_index"]), int(service_id))

    slot = property(lambda self: self["slot"])
    service_id = property(lambda self: self["service_id"])
    episode_key = property(lambda self: self["episode_key"])

    @property
    def service_seed(self):
        return service_seed(*self._seed)

    def component(self, component_id):
        return ComponentContext(self.episode_key, self.service_id, component_seed(*self._seed, int(component_id)))


_ROLE_FLAGS = ("active", "terminated", "truncated", "final_observation", "bootstrap")


def _read(f):
    """f(), a read with no effect, retried while another thread's call holds the batch or the episode: what the
    adapter reads of a step that ran - all of it before anything is recorded."""
    while True:
        try:
            return f()
        except _native.Error as e:
            if "busy" not in str(e):
                raise
            time.sleep(0)


@_public
class TrainingAdapter:
    def __init__(self, batch):
        self.batch = batch
        self._systems = {}  # (slot, service id) -> system
        self._acting = {}   # (slot, service id) -> its role is active in the current generation
        self._open = set()  # (slot, service id) whose generation began and has not ended
        self._ticked = None  # the slots' (generation, tick) whose systems ticked and whose batch step is still to run
        self._actor, self._lifecycle, self._critic = [], [], []

    def register(self, slot, service_id, factory):
        """Register `factory(slot, service_id)`'s system for a configured service of a slot."""
        try:
            self.batch.final_observation_json(slot, service_id)  # (a service the slot's configuration has)
        except _native.Error as e:
            raise ValueError("training: slot %s has no service %s: %s" % (slot, service_id, e)) from None
        key = (int(slot), int(service_id))
        if key in self._systems:
            raise ValueError("training: slot %s service %s is registered already" % key)
        self._systems[key] = factory(*key)

    def _slots(self):
        return sorted({s for s, _ in self._systems})

    def _begin(self, slot):
        base = self.batch.base_seed(slot)
        for (s, service), system in sorted(self._systems.items()):
            if s == slot:
                key = json.loads(_read(lambda: self.batch.endpoint(s, service).observation_json()))["episode"]
                self._acting[(s, service)] = True
                self._open.add((s, service))
                system.begin_episode(EpisodeContext(s, service, key, base))

    def _interrupt(self, slots):
        """end_episode for each system whose generation a reset cuts short, with its outcome as it stands."""
        for slot in slots:
            if not any(s == slot for s, _ in self._open):
                continue
            o = json.loads(self.batch.referee(slot).outcome_json())
            for (s, service), system in sorted(self._systems.items()):
                if s == slot and (s, service) in self._open:
                    self._open.discard((s, service))
                    system.end_episode({"episode_done": o["episode_done"], "reason": o["reason"]})

    def reset(self):
        """A fresh generation in every slot; each registered system's begin_episode before it acts (its end_episode
        first where the reset cuts a generation short)."""
        self._interrupt(self._slots())
        self.batch.reset()
        for slot in self._slots():
            self._begin(slot)

    def reset_slot(self, slot):
        """A fresh generation in one slot (a "disabled" batch's ended slot, or any other); its systems' begin_episode."""
        self._interrupt([int(slot)])
        self.batch.reset_slot(slot)
        self._begin(int(slot))

    def _final(self, slot, service, tick, document):
        self._actor.append({"slot": slot, "service_id": str(service), "tick": tick, "final_observation": document})

    def step(self):
        """Each registered system whose role is active ticks on its endpoint (none in a slot whose ended episode waits
        for its reset), the batch steps, and the three streams take what the step produced. A role that ended alone
        stops acting and leaves its final snapshot once; where the episode ended, every role still acting leaves its
        final snapshot, every system gets end_episode, and begin_episode follows where the slot reset."""
        slots = self._slots()
        statuses = {slot: self.batch.status(slot) for slot in slots}
        marker = tuple((slot, statuses[slot].generation, statuses[slot].tick) for slot in slots)
        if self._ticked != marker:  # (not the retry of a batch step that failed busy: its systems ticked already)
            for (slot, service), system in sorted(self._systems.items()):
                status = statuses[slot]
                if (status.episode_done and not status.reset) or not self._acting.get((slot, service)):
                    continue  # (an ended episode, or an ended role, acts no more)
                endpoint = self.batch.endpoint(slot, service)
                self._actor.append({"slot": slot, "service_id": str(service), "tick": status.tick, "observation": endpoint.observation_json()})
                system.tick(endpoint)
            self._ticked = marker
        self.batch.step()  # (whole, or busy with nothing done: raised, and the next step() runs it again)
        self._ticked = None
        # What the step produced, read whole before anything is recorded or any hook runs.
        produced = {}
        for slot in slots:
            status = _read(lambda: self.batch.status(slot))
            if status.ended:
                docs = _read(lambda: (self.batch.final_outcome_json(slot), self.batch.final_truth_json(slot), self.batch.final_costs_json(slot)))
            else:
                docs = _read(lambda: (lambda r: (r.outcome_json(), r.truth_json(), r.costs_json()))(self.batch.referee(slot)))
            o = json.loads(docs[0])
            roles = {r["service_id"]: r for r in o["roles"]}
            finals = {}
            for (s, service) in sorted(self._systems):
                if s != slot or not self._acting.get((s, service)):
                    continue
                role = roles[str(service)]
                if status.ended:  # (the ended generation's: under SameStep the endpoint is already the next one's)
                    finals[service] = _read(lambda: self.batch.final_observation_json(slot, service))
                elif not role["active"] and role["final_observation"]:  # its own role ended - its platform lost
                    finals[service] = _read(lambda: self.batch.endpoint(slot, service).observation_json())
            produced[slot] = (status, docs, o, roles, finals)
        for slot in slots:
            status, (outcome, truth, costs), o, roles, finals = produced[slot]
            tick, generation = int(o["tick"]), o["episode"]["episode_generation"]
            self._critic.append({"slot": slot, "tick": tick, "generation": generation, "outcome": outcome, "truth": truth, "costs": costs})
            for (s, service), system in sorted(self._systems.items()):
                if s != slot:
                    continue
                role = roles[str(service)]
                record = {"slot": slot, "service_id": str(service), "tick": tick, "generation": generation,
                          "episode_done": o["episode_done"], "reason": o["reason"]}
                record.update({k: role[k] for k in _ROLE_FLAGS})
                self._lifecycle.append(record)
                if service in finals:
                    self._final(slot, service, tick, finals[service])
                    self._acting[(s, service)] = False
                if status.ended:
                    self._acting[(s, service)] = False
                    self._open.discard((s, service))
                    system.end_episode({"episode_done": o["episode_done"], "reason": o["reason"]})
            if status.reset:
                self._begin(slot)

    def actor_stream(self):
        return list(self._actor)

    def lifecycle_stream(self):
        return list(self._lifecycle)

    def critic_stream(self):
        return list(self._critic)

    def clear_streams(self):
        self._actor, self._lifecycle, self._critic = [], [], []

    def close(self):
        self.batch.close()
