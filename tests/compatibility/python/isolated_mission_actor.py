"""An actor for test_mission.py: run in a process of its own (fsim.isolation.MissionEndpointProcess), holding only its
mission endpoint's channel - three calls, documents as text."""
import json


class Actor:
    """Reads snapshots, waits, and checks endpoint methods and unsupported-member refusals."""

    def __init__(self, endpoint, label):
        self.endpoint, self.label = endpoint, label

    def observe(self):
        return self.endpoint.observation_json()

    def wait(self):
        """An empty submission for the snapshot it holds: the acknowledgment."""
        snapshot = json.loads(self.endpoint.observation_json())
        return self.endpoint.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"],
                                                     "service_id": snapshot["service"]["service_id"], "tick": snapshot["tick"],
                                                     "requests": []}))

    def submit(self, text):
        return self.endpoint.submit_json(text)

    def receipts(self):
        return self.endpoint.receipts_json()

    def echo(self, value):
        return value

    def reach(self):
        """Range-owned members are absent from the actor's three-method endpoint proxy."""
        out = {}
        for name in ("episode", "referee", "world", "step", "reset", "outcome_json", "truth_json", "agent", "close"):
            try:
                getattr(self.endpoint, name)
                out[name] = "reached"
            except AttributeError:
                out[name] = "refused"
        return out

    def forge(self):
        """Requests outside the endpoint's callable-method list are refused in the Range process."""
        channel, out = object.__getattribute__(self.endpoint, "_channel"), {}
        for message in ({"op": "endpoint", "m": "step"}, {"op": "endpoint", "m": "referee"}, {"op": "endpoint", "m": "_endpoint"},
                        {"op": "agent", "m": "ownship"}, {"op": "endpoint", "m": "observation_json", "a": ["extra"]}):
            try:
                channel.request(message)
                out[message["op"] + "." + message["m"]] = "answered"
            except Exception as exc:  # (the range's refusal, rebuilt here)
                out[message["op"] + "." + message["m"]] = type(exc).__name__
        return out
