#pragma once

// fsim SDK: mission episodes for MA training (docs/sdk/mission-training.md; M5 core).
//
// A MissionEpisode is the range's: it is made from an `ma.config.v1` document, owns one World at a time - rebuilt whole
// at every reset - and hands out two kinds of child. A MissionEndpoint is one configured MA service's, and the only
// thing an actor holds: it reads that service's immutable snapshot of the current decision tick (`ma.observation.v1`),
// submits `ma.actions.v1` requests and reads their receipts (`ma.receipts.v1`). Nothing it returns reaches a World, a
// Vehicle, a Range or referee handle, a native callback, the IPC segment or a recording. A MissionReferee reads the
// range's documents - `ma.outcome.v1` and `ma.truth.v1` - which never reach an endpoint.
//
// Every document is UTF-8 JSON text, exactly as the contract defines it; identities are decimal strings. Each class
// holds one pointer to its implementation and nothing else.
//
// Errors (docs/sdk/mission-training.md, section 5): a malformed document, a configuration the contract refuses, or an
// endpoint for a service the configuration does not bind - std::invalid_argument, nothing changed. Before the first
// reset, after the episode's end, from a moved-from object or a child of an earlier generation, or while another call
// holds the episode's boundary - std::runtime_error, nothing changed. A well-formed document for another generation,
// tick or service is not an error: its receipts say why nothing was queued.

#include "fsim/Export.h"

#include <cstdint>
#include <string>
#include <string_view>

namespace fsim {

class MissionEpisode;

/// One configured MA service's endpoint, bound to the generation it was opened in.
class FSIM_API MissionEndpoint {
public:
    ~MissionEndpoint();
    MissionEndpoint(MissionEndpoint&& other) noexcept;
    MissionEndpoint& operator=(MissionEndpoint&& other) noexcept;
    MissionEndpoint(const MissionEndpoint&) = delete;
    MissionEndpoint& operator=(const MissionEndpoint&) = delete;

    /// The service's snapshot of the current decision tick. Reading it consumes nothing and changes nothing.
    std::string observationJsonV1() const;
    /// Validate an `ma.actions.v1` document and queue its requests for the next step: the acknowledgment
    /// (`ma.receipts.v1`). Nothing executes until the range steps the episode.
    std::string submitJsonV1(std::string_view actionsJsonV1);
    /// The latest acknowledgment and the service's receipts, again; consumes nothing.
    std::string receiptsJsonV1() const;

private:
    friend class MissionEpisode;
    struct Impl;
    explicit MissionEndpoint(Impl* impl) noexcept;
    Impl* impl_ = nullptr;
};

/// The referee's documents of the generation it was opened in: the range's and the trainer's, never an actor's.
class FSIM_API MissionReferee {
public:
    ~MissionReferee();
    MissionReferee(MissionReferee&& other) noexcept;
    MissionReferee& operator=(MissionReferee&& other) noexcept;
    MissionReferee(const MissionReferee&) = delete;
    MissionReferee& operator=(const MissionReferee&) = delete;

    std::string outcomeJsonV1() const; ///< `ma.outcome.v1` of the current decision tick
    std::string truthJsonV1() const;   ///< `ma.truth.v1` of the current decision tick
    /// `ma.costs.v1` of the current decision tick (M7 core): each service's requests and communication, this tick and
    /// in total - the referee's, never an actor's.
    std::string costsJsonV1() const;

private:
    friend class MissionEpisode;
    struct Impl;
    explicit MissionReferee(Impl* impl) noexcept;
    Impl* impl_ = nullptr;
};

class FSIM_API MissionEpisode {
public:
    /// Validate an `ma.config.v1` document. No World is built until the first reset.
    explicit MissionEpisode(std::string_view configJsonV1);
    /// Ends the episode: its children answer no more; a World a call still uses goes when that call returns.
    ~MissionEpisode();
    MissionEpisode(MissionEpisode&& other) noexcept;
    MissionEpisode& operator=(MissionEpisode&& other) noexcept;
    MissionEpisode(const MissionEpisode&) = delete;
    MissionEpisode& operator=(const MissionEpisode&) = delete;

    /// Build a fresh World for local episode `localEpisodeIndex`; every earlier child is stale from here on. Snapshot 0
    /// is frozen. The new generation's episode key, as the contract's `episode_key` object.
    std::string reset(std::uint64_t localEpisodeIndex);
    /// Close the decision tick: dispatch the queued requests in ascending service id, then request id, through each
    /// service's Agent, advance the World, read the native statuses and freeze the next snapshot.
    void step();
    /// The endpoint of a configured service, for the current generation.
    MissionEndpoint endpoint(std::uint64_t serviceId);
    /// The referee, for the current generation.
    MissionReferee referee();
    /// The range removes a platform (a scenario vehicle or C2 node, by name) from the current World, as the World's own
    /// removal does (M7 core): its services' roles end at the next tick. False when it is already gone;
    /// std::invalid_argument for a name the configuration does not have.
    bool removePlatform(std::string_view platform);

private:
    friend class MissionBatch; // (its slots' transitions: one operation per slot under the episode's boundary)
    struct Impl;
    Impl* impl_ = nullptr;
};

} // namespace fsim
