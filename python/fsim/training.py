"""The training adapter contract (docs/sdk/mission-training.md, section 18; M7 core).

Dependency-free: no learning framework, optimizer or tensor library. It runs MA systems an upper layer registers - one
per slot and service, made by its factory - over a :class:`fsim.mission.MissionBatch`, calls their lifecycle hooks
around every generation, and keeps three streams apart:

  actor_stream()      what each role's actors saw: its snapshots and, once per generation, its final snapshot - when
                      its own role ended (a lost platform) or when the episode did
  lifecycle_stream()  the signals a learner needs per role and step: episode_done, reason, active, terminated,
                      truncated, final_observation, bootstrap - from the referee's outcome, never an observation
  critic_stream()     the referee's documents per slot and step: outcome, truth, costs - for a critic or a reward,
                      never for an actor

A registered system is any object with

    begin_episode(context)  # an EpisodeContext: a new generation, before it acts
    tick(endpoint)          # read the endpoint's snapshot, submit its requests (fsim.mission.MissionEndpoint)
    end_episode(feedback)   # feedback: {"episode_done", "reason"}: the role's permitted mission feedback

Each system gets one begin_episode and one end_episode per generation. A step whose batch step fails busy (another
thread's call held the batch or an episode) raises fsim.Error with nothing of its batch step done or recorded; its
systems have ticked, and the next step() runs that batch step again without ticking them twice. It ticks while its role is active: a role that
ended while its teammates go on (its platform lost) ticks no more in that generation, and its final snapshot is the
one its role ended in; its end_episode still comes when the generation does. The composition example's MASystem
(examples/training/ma_composition) is one such system: the context carries what it takes - the episode key, the service
and section 8's seeds per component. Reward definitions are not here: none is installed (section 15)."""
from ._training.adapter import (MASK64, ComponentContext, EpisodeContext, TrainingAdapter, component_seed, service_seed,
                                world_seed)

__all__ = ["TrainingAdapter", "EpisodeContext", "ComponentContext", "world_seed", "service_seed", "component_seed"]
