"""The example under the SDK's training adapter (fsim.training; docs/sdk/mission-training.md, section 18; M7 core).

The same lead MA system and scripted C2 as run_episode, registered with fsim.training.TrainingAdapter over a one-slot
fsim.mission.MissionBatch whose episodes end after `ticks` decision ticks, through its autoreset. The adapter's
EpisodeContext is what ReferenceSystem.begin_episode takes. Each generation's trace must equal run_episode's for the
same local episode index in a plain MissionEpisode: the adapter, the batch and its autoreset change nothing an MA
system sees or does. Example code only, on the installed public package."""
import json

from fsim import mission, training

from .scenario import CONFIG, CRC, LEAD_FLIGHT, C2Script, make_system, run_episode


def limited(ticks):
    """The scenario's configuration with an episode of `ticks` decision ticks (it then ends truncated)."""
    document = json.loads(json.dumps(CONFIG))
    document["task"]["criteria"] = [{"id": "time", "kind": "tick_limit", "outcome": "truncated", "max_ticks": ticks}]
    return json.dumps(document)


class Traced:
    """A system whose trace is kept at the end of each generation (begin_episode clears it for the next)."""

    def __init__(self, system):
        self.system, self.traces = system, []

    def begin_episode(self, context):
        self.system.begin_episode(context)

    def tick(self, endpoint):
        self.system.tick(endpoint)

    def end_episode(self, feedback):
        self.system.end_episode(feedback)
        self.traces.append(self.system.trace())


class ScriptedC2:
    """The range's scripted C2 as an adapter system: its request numbering starts again each generation."""

    def __init__(self):
        self.request = 0

    def begin_episode(self, context):
        self.request = 0

    def tick(self, endpoint):
        script = C2Script(endpoint)
        script.request = self.request
        script.tick()
        self.request = script.request

    def end_episode(self, feedback):
        pass


def under_training(ticks, local, autoreset, generations=2):
    """`generations` generations of the lead's system under the adapter; their traces."""
    traced = Traced(make_system())
    with mission.MissionBatch([limited(ticks)], autoreset=autoreset, first_local_episode_index=local) as batch:
        adapter = training.TrainingAdapter(batch)
        adapter.register(0, LEAD_FLIGHT, lambda slot, service: traced)
        adapter.register(0, CRC, lambda slot, service: ScriptedC2())
        adapter.reset()
        for _ in range(generations * (ticks + 1)):
            if len(traced.traces) == generations:
                break
            adapter.step()
    return traced.traces


def reference(ticks, local, generations=2):
    """The same generations by run_episode, one local episode index after another, in a plain MissionEpisode."""
    with mission.MissionEpisode(limited(ticks)) as episode:
        return [run_episode(episode, make_system(), local + g, ticks) for g in range(generations)]
