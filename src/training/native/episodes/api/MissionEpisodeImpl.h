#pragma once

// The public mission episode's own state (fsim/MissionEpisode.h), shared within the SDK by MissionEpisode.cpp and the
// batch (MissionBatch.cpp: M7 core), which closes a slot's tick as one operation under the episode's boundary
// (env::EpisodeHold). Not installed.

#include "training/native/episodes/runtime/MissionEpisode.h"
#include "fsim/MissionEpisode.h"
#include "training/native/mission/MissionConfig.h"
#include "training/native/mission/MissionGateway.h"

#include <cstdint>
#include <memory>
#include <string>

namespace fsim {

namespace mission {

/// What one reset's native step made, for the judge factory that follows it in the same reset (under its boundary).
struct Minting {
    std::uint64_t generation = 0, localEpisodeIndex = 0;
    Minted minted;
};

/// A reset's hooks: the generation's minting inputs before the rebuild, and the new generation's episode key into
/// `key` after it - both under that reset's boundary, so no other call can change or fail the key.
env::ResetHooks resetHooks(const std::shared_ptr<Minting>& minting, std::uint64_t localEpisodeIndex, std::string& key);

} // namespace mission

struct MissionEpisode::Impl {
    std::shared_ptr<const mission::Config> config;
    std::shared_ptr<mission::Minting> minting;
    env::MissionEpisode episode;
    Impl(std::shared_ptr<const mission::Config> c, std::shared_ptr<mission::Minting> m, env::EpisodeConfig e)
        : config(std::move(c)), minting(std::move(m)), episode(std::move(e)) {}
};

} // namespace fsim
