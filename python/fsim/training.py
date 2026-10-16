"""The training adapter contract (docs/sdk/mission-training.md, section 18; M7 core).

Dependency-free: no learning framework, optimizer or tensor library. It runs MA systems an upper layer registers - one
per slot and service, made by its factory - over a :class:`fsim.mission.MissionBatch`, calls their lifecycle hooks
around every generation, and keeps three streams apart:

  actor_stream()      what each role's actors saw: its snapshots and, when its episode ended, its final snapshot
  lifecycle_stream()  the signals a learner needs per role and step: episode_done, reason, active, terminated,
                      truncated, final_observation, bootstrap - from the referee's outcome, never an observation
  critic_stream()     the referee's documents per slot and step: outcome, truth, costs - for a critic or a reward,
                      never for an actor

A registered system is any object with

    begin_episode(context)  # context: {"slot", "service_id", "episode_key"}: a new generation, before it acts
    tick(endpoint)          # read the endpoint's snapshot, submit its requests (fsim.mission.MissionEndpoint)
    end_episode(feedback)   # feedback: {"episode_done", "reason"}: the role's permitted mission feedback

- the composition example's MASystem is one. Reward definitions are not here: none is installed (section 15)."""
import json

__all__ = ["TrainingAdapter", "world_seed", "service_seed", "component_seed"]

MASK64 = (1 << 64) - 1


def _next(x):
    """fsim::Rng(x).next(): one SplitMix64 step."""
    x = (x + 0x9E3779B97F4A7C15) & MASK64
    z = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return z ^ (z >> 31)


def _stream(base_seed, env_id, local_episode_index, stream_class):
    return _next(_next(_next(_next(base_seed) ^ env_id) ^ local_episode_index) ^ stream_class)


def world_seed(base_seed, env_id, local_episode_index):
    """The section 8 world seed of a local episode."""
    return _stream(base_seed, env_id, local_episode_index, 0)


def service_seed(base_seed, env_id, local_episode_index, service_id):
    """The section 8 seed of a service, offered to the upper layer."""
    return _next(_stream(base_seed, env_id, local_episode_index, 1) ^ service_id)


def component_seed(base_seed, env_id, local_episode_index, service_id, component_id):
    """The section 8 seed of a component behind a service."""
    return _next(service_seed(base_seed, env_id, local_episode_index, service_id) ^ component_id)


_ROLE_FLAGS = ("active", "terminated", "truncated", "final_observation", "bootstrap")


class TrainingAdapter:
    def __init__(self, batch):
        self.batch = batch
        self._systems = {}  # (slot, service id) -> system
        self._actor, self._lifecycle, self._critic = [], [], []

    def register(self, slot, service_id, factory):
        """Register `factory(slot, service_id)`'s system for a configured service of a slot."""
        import fsim
        try:
            self.batch.final_observation_json(slot, service_id)  # (a service the slot's configuration has)
        except fsim.Error as e:
            raise ValueError("training: slot %s has no service %s: %s" % (slot, service_id, e)) from None
        key = (int(slot), int(service_id))
        if key in self._systems:
            raise ValueError("training: slot %s service %s is registered already" % key)
        self._systems[key] = factory(*key)

    def _begin(self, slot):
        for (s, service), system in sorted(self._systems.items()):
            if s == slot:
                key = json.loads(self.batch.endpoint(s, service).observation_json())["episode"]
                system.begin_episode({"slot": s, "service_id": str(service), "episode_key": key})

    def reset(self):
        """A fresh generation in every slot; each registered system's begin_episode before it acts."""
        self.batch.reset()
        for slot in sorted({s for s, _ in self._systems}):
            self._begin(slot)

    def step(self):
        """Each registered system ticks on its endpoint (not in a slot whose ended episode waits for its reset), the batch
        steps, and the three streams take what the step produced; then end_episode and begin_episode where it ended
        and reset."""
        for (slot, service), system in sorted(self._systems.items()):
            status = self.batch.status(slot)
            if status.episode_done and not status.reset:
                continue  # (an ended episode acts no more)
            endpoint = self.batch.endpoint(slot, service)
            self._actor.append({"slot": slot, "service_id": str(service), "tick": status.tick, "observation": endpoint.observation_json()})
            system.tick(endpoint)
        self.batch.step()
        for slot in sorted({s for s, _ in self._systems}):
            status = self.batch.status(slot)
            if status.ended:
                outcome, truth, costs = (self.batch.final_outcome_json(slot), self.batch.final_truth_json(slot),
                                         self.batch.final_costs_json(slot))
            else:
                referee = self.batch.referee(slot)
                outcome, truth, costs = referee.outcome_json(), referee.truth_json(), referee.costs_json()
            o = json.loads(outcome)
            self._critic.append({"slot": slot, "tick": int(o["tick"]), "generation": o["episode"]["episode_generation"],
                                 "outcome": outcome, "truth": truth, "costs": costs})
            roles = {r["service_id"]: r for r in o["roles"]}
            for (s, service), system in sorted(self._systems.items()):
                if s != slot:
                    continue
                role = roles[str(service)]
                record = {"slot": slot, "service_id": str(service), "tick": int(o["tick"]), "generation": o["episode"]["episode_generation"],
                          "episode_done": o["episode_done"], "reason": o["reason"]}
                record.update({k: role[k] for k in _ROLE_FLAGS})
                self._lifecycle.append(record)
                if status.ended:
                    self._actor.append({"slot": slot, "service_id": str(service), "tick": int(o["tick"]),
                                        "final_observation": self.batch.final_observation_json(slot, service)})
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
