"""Mission episodes for MA training (docs/sdk/mission-training.md; M5 core).

A :class:`MissionEpisode` is the range's: made from an ``ma.config.v1`` document, it owns one World at a time - rebuilt
whole at every :meth:`~MissionEpisode.reset` - and hands out two kinds of handle, each bound to the generation it was
opened in. A :class:`MissionEndpoint` is one configured MA service's and the only thing an actor holds: it reads that
service's immutable snapshot of the decision tick (``ma.observation.v1``), submits ``ma.actions.v1`` requests and reads
their receipts (``ma.receipts.v1``). A :class:`MissionReferee` reads the range's documents (``ma.outcome.v1``,
``ma.truth.v1``), which never reach an endpoint::

    episode = fsim.mission.MissionEpisode(config_text)
    episode.reset(local_episode_index=0)
    wing = episode.endpoint(9007199254740995)
    snapshot = json.loads(wing.observation_json())          # the same text however often it is read
    wing.submit_json(actions_text)                          # validated and queued; nothing runs yet
    episode.step()                                          # dispatched in service id, then request id order

Every document is text (``str``), exactly as the contract writes it: identities are decimal strings, so they stay exact
whatever a caller parses them with. Errors are :class:`fsim.Error`: a malformed document, an unconfigured service, a
handle of an earlier generation or of a closed episode, an episode not reset, or one another call holds (busy). A
well-formed document for another generation, tick or service is not an error: its receipts say why nothing was queued.

The process boundary for actors is :class:`fsim.isolation.MissionEndpointProcess`.
"""
import json

from . import _native

__all__ = ["MissionEpisode", "MissionEndpoint", "MissionReferee"]


class MissionEndpoint:
    """One configured MA service's endpoint, of the generation it was opened in. Its three calls are all an actor has:
    nothing it returns reaches a World, a Vehicle, the range, the referee, the IPC segment or a recording."""

    __slots__ = ("_native", "_service_id")

    def __init__(self, native, service_id):
        self._native = native
        self._service_id = service_id

    @property
    def service_id(self):
        return self._service_id

    def observation_json(self):
        """This tick's snapshot (``ma.observation.v1``) as text; reading it consumes and changes nothing."""
        return self._live().observation_json()

    def submit_json(self, actions):
        """Validate an ``ma.actions.v1`` document (str or bytes) and queue its requests: the acknowledgment
        (``ma.receipts.v1``) as text. Nothing executes until the range steps the episode."""
        return self._live().submit_json(actions)

    def receipts_json(self):
        """The latest acknowledgment and the service's receipts, again; consumes nothing."""
        return self._live().receipts_json()

    def close(self):
        """Release the handle (also when it is already closed)."""
        if self._native is not None:
            self._native.close()
            self._native = None

    def _live(self):
        if self._native is None:
            raise _native.Error("this MissionEndpoint is closed")
        return self._native

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __repr__(self):
        return "MissionEndpoint(service %d%s)" % (self._service_id, "" if self._native is not None else ", closed")


class MissionReferee:
    """The referee's documents of the generation it was opened in: the range's and the trainer's, never an actor's."""

    __slots__ = ("_native",)

    def __init__(self, native):
        self._native = native

    def outcome_json(self):
        """``ma.outcome.v1`` of the current decision tick, as text."""
        return self._live().outcome_json()

    def truth_json(self):
        """``ma.truth.v1`` of the current decision tick, as text."""
        return self._live().truth_json()

    def close(self):
        if self._native is not None:
            self._native.close()
            self._native = None

    def _live(self):
        if self._native is None:
            raise _native.Error("this MissionReferee is closed")
        return self._native

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class MissionEpisode:
    """The range's episode, made from an ``ma.config.v1`` document (str or bytes). No World is built until
    :meth:`reset`."""

    __slots__ = ("_native", "_diagnostic")

    def __init__(self, config):
        self._native = _native.MissionEpisode(config)
        # (the configuration was accepted whole above: strict JSON, every rule of the contract)
        execution = json.loads(config).get("execution", {})
        self._diagnostic = bool(execution.get("diagnostic") or execution.get("publish") or execution.get("record_dir"))

    @property
    def diagnostic(self):
        """True for a diagnostic episode - published or recorded, labelled non-isolated: strict isolation refuses it."""
        return self._diagnostic

    def reset(self, local_episode_index):
        """Build a fresh World for local episode `local_episode_index`; every earlier handle is stale from here on. Its
        episode key is in each of the new generation's documents."""
        self._live().reset(local_episode_index)

    def step(self):
        """Close the decision tick: dispatch the queued requests, advance the World, freeze the next snapshot."""
        self._live().step()

    def endpoint(self, service_id):
        """The endpoint of configured service `service_id`, for the current generation."""
        return MissionEndpoint(self._live().endpoint(service_id), int(service_id))

    def referee(self):
        """The referee, for the current generation."""
        return MissionReferee(self._live().referee())

    def close(self):
        """End the episode: its handles answer no more (also when it is already closed)."""
        if self._native is not None:
            self._native.close()
            self._native = None

    def _live(self):
        if self._native is None:
            raise _native.Error("this MissionEpisode is closed")
        return self._native

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
