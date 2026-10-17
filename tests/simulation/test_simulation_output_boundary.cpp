// IPC and recording are diagnostic outputs. Reader pacing, focus and reconnects must not change native state.
#include "training/native/episodes/runtime/MissionEpisode.h"
#include "fixtures/mission_training/NativeStateReset.h"
#include "foundation/transport/Recording.h"
#include "foundation/transport/WorldMirror.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace fixture = fsim::test::native_reset;
using fsim::env::EpisodeStatus;
using fsim::env::MissionEpisode;

template <class F>
void access(MissionEpisode& episode, F&& callback) {
    REQUIRE(episode.range([&](fsim::session::World& native) {
        const std::unique_ptr<fsim::World> world(fsim::World::borrowed(native));
        callback(*world);
    }) == EpisodeStatus::Ok);
}

fixture::Snapshot nativeState(MissionEpisode& episode, bool published) {
    fixture::Snapshot result;
    access(episode, [&](fsim::World& world) {
        REQUIRE(world.impl().published() == published);
        result = fixture::capture(world);
    });
    // This is the sole expected diagnostic difference; every captured authoritative field is compared unchanged.
    REQUIRE(result.erase("world/published") == 1);
    return result;
}

void equivalent(const fixture::Snapshot& expected, const fixture::Snapshot& actual) {
    std::ostringstream differences;
    unsigned count = 0;
    for (const auto& [key, value] : expected) {
        const auto found = actual.find(key);
        if (found == actual.end() || found->second != value) {
            if (count < 8) differences << key << ": " << value << " vs "
                                     << (found == actual.end() ? "<absent>" : found->second) << '\n';
            ++count;
        }
    }
    for (const auto& [key, value] : actual) {
        (void)value;
        if (!expected.count(key)) {
            if (count < 8) differences << "extra: " << key << '\n';
            ++count;
        }
    }
    INFO(differences.str());
    REQUIRE(count == 0);
}
} // namespace

TEST_CASE("simulation outputs: publication, recording and observer activity preserve native evolution and reset",
          "[boundary][simulation][diagnostic][reset]") {
    const std::filesystem::path directory = std::filesystem::path(FSIM_TEST_OUTPUT_DIR) / "recordings";
    std::filesystem::create_directories(directory);
    std::vector<std::filesystem::path> recordings;
    {
        fixture::Scenario quietScenario, observedScenario;
        fsim::env::EpisodeConfig config;
        config.name = "simulation-output-boundary";
        config.envId = 7, config.baseSeed = 99;
        config.dtS = 1.0 / 120.0, config.frameSkip = 4, config.workers = 1;
        config.native = [&](fsim::session::World& native) {
            const std::unique_ptr<fsim::World> world(fsim::World::borrowed(native));
            quietScenario = fixture::configure(*world);
        };
        MissionEpisode quiet(config);
        config.diagnostic = true, config.publish = true;
        config.recordDir = directory.string();
        config.native = [&](fsim::session::World& native) {
            const std::unique_ptr<fsim::World> world(fsim::World::borrowed(native));
            observedScenario = fixture::configure(*world);
        };
        MissionEpisode observed(config);
        fsim::ipc::WorldMirror mirror;
        unsigned snapshots = 0;
        for (const std::uint64_t localIndex : {0ULL, 1ULL}) {
            INFO("local episode " << localIndex);
            mirror.close();
            quiet.reset(localIndex);
            observed.reset(localIndex);
            recordings.emplace_back(observed.recordPath());
            equivalent(nativeState(quiet, false), nativeState(observed, true));
            REQUIRE(mirror.open(observed.worldName()));
            for (auto* episode : {&quiet, &observed}) {
                const auto& scenario = episode == &quiet ? quietScenario : observedScenario;
                access(*episode, [&](fsim::World& world) {
                    const auto dirty = fixture::contaminate(world, scenario);
                    fixture::requireDirty(world, scenario, dirty);
                });
            }
            equivalent(nativeState(quiet, false), nativeState(observed, true));
            for (unsigned tick = 0; tick < 24; ++tick) {
                REQUIRE(quiet.step() == EpisodeStatus::Ok);
                REQUIRE(observed.step() == EpisodeStatus::Ok);
                // Exercise the same IPC reader used by Viewer, without a renderer or GUI. Force different publication
                // timing and focus requests, then vary reader pacing and attachment while both Worlds keep stepping.
                const auto outputStatus = observed.range([&](fsim::session::World& world) {
                    REQUIRE(world.focusViewer(tick % 2 ? observedScenario.launcher : observedScenario.target));
                    world.publishNow();
                });
                REQUIRE(outputStatus == EpisodeStatus::Ok);
                if (tick % 3 == 0) {
                    mirror.pollTable();
                    fsim::sim::SnapshotBatch batch;
                    if (mirror.pollSnapshot(batch)) ++snapshots;
                    std::vector<fsim::ipc::EngagementEvent> events;
                    mirror.pollEvents(events);
                    (void)mirror.environment();
                }
                if (tick % 8 == 7) {
                    mirror.close();
                    REQUIRE(mirror.open(observed.worldName()));
                }
                equivalent(nativeState(quiet, false), nativeState(observed, true));
            }
        }
        CHECK(snapshots > 0);
        CHECK(recordings[0] != recordings[1]);
    } // Both episode Worlds and recorders are closed before files are read.
    for (const auto& path : recordings) {
        INFO(path.string());
        fsim::ipc::Recording recording;
        std::string error;
        REQUIRE(recording.load(path, &error));
        INFO(error);
        REQUIRE_FALSE(recording.frames().empty());
        CHECK(recording.header().dt == 1.0 / 120.0);
        CHECK(recording.header().frameSkip == 4);
    }
}
