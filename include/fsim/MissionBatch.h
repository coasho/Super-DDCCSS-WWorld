#pragma once

// fsim SDK: batches of mission episodes for training (docs/sdk/mission-training.md, section 18; M7 core).
//
// A MissionBatch is the range's: one independent MissionEpisode per slot, each from its own `ma.config.v1` (slots may
// configure different platforms, services and C2 nodes - batch, service and actor counts are unrelated), each with its
// own World, task/judge, gateway and seed streams. Slot k's local episode index starts at the options' first index
// and counts that slot's resets alone; a slot's env_id is its configuration's and must be unique in the batch, so no
// random stream depends on another slot, on the worker count or on the order slots are stepped in.
//
// step() closes one decision tick in every slot. An episode ends when its referee outcome says `episode_done`; then:
//   Disabled  the slot holds its ended generation until resetSlot(): step() neither dispatches its requests nor
//             steps its World;
//   NextStep  the next step() resets the slot - before any request of the ended generation is dispatched and before
//             any World step - and does nothing else in it: no action, release, message or damage of the old
//             generation follows its end;
//   SameStep  the same step() keeps the ended generation's final documents and resets the slot at once.
// Final documents stay apart by principal: a final observation is its service's (an actor's), the final outcome and
// costs are the referee's. Nothing a referee reads enters an observation.
//
// Errors are MissionEpisode's: std::invalid_argument for a malformed configuration, a slot out of range or an
// unconfigured service; std::runtime_error for lifetime and busy failures.

#include "fsim/Export.h"
#include "fsim/MissionEpisode.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace fsim {

enum class MissionAutoreset : std::uint8_t { Disabled = 0, NextStep = 1, SameStep = 2 };

struct MissionBatchOptions {
    MissionAutoreset autoreset = MissionAutoreset::NextStep;
    unsigned workers = 1;                   ///< slots stepped side by side (each slot's World has its own workers)
    std::uint64_t firstLocalEpisodeIndex = 0;
};

/// A slot after the last reset or step.
struct MissionSlotStatus {
    std::uint64_t generation = 0, localEpisodeIndex = 0, tick = 0;
    bool episodeDone = false; ///< its current generation's outcome says episode_done
    bool ended = false;       ///< an episode of this slot ended in the last step
    bool reset = false;       ///< a new generation began in the last step (NextStep, SameStep) or reset
};

class FSIM_API MissionBatch {
public:
    /// Validate every configuration; no World is built until reset().
    MissionBatch(const std::vector<std::string>& configsJsonV1, const MissionBatchOptions& options);
    ~MissionBatch();
    MissionBatch(MissionBatch&& other) noexcept;
    MissionBatch& operator=(MissionBatch&& other) noexcept;
    MissionBatch(const MissionBatch&) = delete;
    MissionBatch& operator=(const MissionBatch&) = delete;

    std::size_t size() const;
    /// A fresh generation in every slot, each at its next local episode index.
    void reset();
    /// A fresh generation in one slot.
    void resetSlot(std::size_t slot);
    /// One decision tick in every slot, with its autoreset.
    void step();
    MissionSlotStatus status(std::size_t slot) const;
    /// The current generation's endpoint and referee of a slot (opened again after each of its resets).
    MissionEndpoint endpoint(std::size_t slot, std::uint64_t serviceId);
    MissionReferee referee(std::size_t slot);
    /// The last ended generation's final documents of a slot, kept until its next episode ends ("" before any):
    /// a service's last snapshot (`ma.observation.v1`) - the actor's - and the referee's outcome, truth and costs.
    std::string finalObservationJsonV1(std::size_t slot, std::uint64_t serviceId) const;
    std::string finalOutcomeJsonV1(std::size_t slot) const;
    std::string finalTruthJsonV1(std::size_t slot) const;
    std::string finalCostsJsonV1(std::size_t slot) const;
    /// The range removes a platform of a slot's current World (MissionEpisode::removePlatform).
    bool removePlatform(std::size_t slot, std::string_view platform);

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

} // namespace fsim
