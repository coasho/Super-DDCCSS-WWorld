// Same-build native fixture input for the independent episode reset gate; this is not an episode implementation.
#include "fixtures/mission_training/NativeStateReset.h"
#include "fsim/MAService.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <sstream>

using namespace fsim::test::native_reset;

namespace {
void equivalent(const Snapshot& first, const Snapshot& second) {
    std::ostringstream differences;
    unsigned count = 0;
    for (const auto& item : first) {
        const auto other = second.find(item.first);
        if (other == second.end() || other->second != item.second) {
            if (count < 6) differences << item.first << ": " << item.second << " vs "
                                     << (other == second.end() ? "<absent>" : other->second) << '\n';
            ++count;
        }
    }
    for (const auto& item : second) if (first.find(item.first) == first.end()) {
        if (count < 6) differences << "extra: " << item.first << '\n';
        ++count;
    }
    INFO(differences.str());
    REQUIRE(count == 0);
}
} // namespace

TEST_CASE("native reset fixture: every required state coexists", "[ma][native-reset-fixture]") {
    fsim::World world(options("native-reset-fixture"));
    const auto scenario = configure(world);
    const auto dirty = contaminate(world, scenario);
    CHECK(dirty.crewLost);
    CHECK(dirty.activeFire);
    CHECK(dirty.liveWeapon != 0);
    CHECK(dirty.liveDecoy != 0);
    CHECK(dirty.pendingTask != 0);
    CHECK(dirty.pendingConsent != 0);
    CHECK(dirty.queuedMessage != 0);
    CHECK(dirty.removedEntity == scenario.removed);
    CHECK(dirty.crewLossEvents > 0);
    CHECK(dirty.fireEvents > 0);
    const auto first = capture(world);
    requireDirty(world, scenario, dirty);
    equivalent(first, capture(world)); // Capturing and revalidating must consume nothing and advance no clock.
    CHECK_THROWS_AS(requireDeliveredMarker(world, scenario, dirty), fsim::Error);

    // The queued payload is tested through actual native delivery, including NUL/non-UTF-8 bytes.
    world.step(30);
    requireDeliveredMarker(world, scenario, dirty);
    const auto* inbox = world.impl().dms().inbox(scenario.launcher);
    REQUIRE(inbox);
    bool delivered = false;
    for (const auto& message : *inbox) if (message.source == dirty.c2System && message.message == dirty.queuedMessage) {
        delivered = true;
        CHECK(message.topic == fsim::comm::topic::kFreeForm + 73);
        CHECK(message.bytes == std::vector<std::uint8_t>{0x4d, 0x34, 0, 0xff, 0x52, 0x45, 0x53, 0x45, 0x54});
        CHECK(message.timeDelivered > message.timeSent);
    }
    CHECK(delivered);
    const auto afterDelivery = capture(world);
    requireDeliveredMarker(world, scenario, dirty);
    equivalent(afterDelivery, capture(world));
}

TEST_CASE("native reset fixture: contamination and replay are deterministic", "[ma][native-reset-fixture]") {
    fsim::World first(options("native-reset-repeat"));
    fsim::World second(options("native-reset-repeat"));
    const auto firstScenario = configure(first), secondScenario = configure(second);
    equivalent(capture(first), capture(second));
    const auto firstDirty = contaminate(first, firstScenario);
    const auto secondDirty = contaminate(second, secondScenario);
    CHECK(firstDirty.liveWeapon == secondDirty.liveWeapon);
    CHECK(firstDirty.pendingConsent == secondDirty.pendingConsent);
    CHECK(firstDirty.queuedMessage == secondDirty.queuedMessage);
    equivalent(capture(first), capture(second));
    for (const unsigned steps : {1u, 4u, 30u}) {
        first.step(steps);
        second.step(steps);
        equivalent(capture(first), capture(second));
    }
}

TEST_CASE("native reset fixture: legacy vehicle reset is not full fresh reconstruction", "[ma][native-reset-fixture]") {
    fsim::World world(options("native-reset-legacy"));
    const auto scenario = configure(world);
    const auto dirty = contaminate(world, scenario);
    const double timeBefore = world.time();
    REQUIRE(world.vehicle(scenario.launcher).reset());
    CHECK(world.time() == timeBefore);
    CHECK(world.weapons().size() == 1);
    CHECK(world.weapons().front().id == dirty.liveWeapon);
    CHECK(world.decoys().size() == 1);
    CHECK(fsim::maServiceBindings(world).size() == 2);
    const auto* tasks = world.impl().maCore().tasks(scenario.launcher);
    REQUIRE(tasks);
    REQUIRE(tasks->size() == 1);
    CHECK(tasks->front().order.taskId == dirty.pendingTask);
    const auto transmission = world.impl().dms().transmission(scenario.c2, dirty.queuedMessage);
    REQUIRE(transmission);
    REQUIRE(transmission->destinations.size() == 1);
    CHECK(transmission->destinations.front().second == fsim::comm::Delivery::Pending);
    CHECK_FALSE(world.vehicle(scenario.removed).valid());
    fsim::World fresh(options("native-reset-legacy"));
    configure(fresh);
    CHECK(capture(world) != capture(fresh));
}

TEST_CASE("native reset fixture: reconstructed owner matches fresh construction and replay", "[ma][native-reset-fixture]") {
    auto owned = std::make_unique<fsim::World>(options("native-reset-reconstruct"));
    const auto scenario = configure(*owned);
    const auto dirty = contaminate(*owned, scenario);
    requireDirty(*owned, scenario, dirty);
    // This is a fixture oracle control, NOT the pending MissionEpisode reset implementation.
    owned.reset();
    owned = std::make_unique<fsim::World>(options("native-reset-reconstruct"));
    const auto reconstructed = configure(*owned);
    fsim::World fresh(options("native-reset-reconstruct"));
    const auto reference = configure(fresh);
    CHECK(reconstructed.removed == reference.removed);
    CHECK(owned->vehicle(reconstructed.removed).valid());
    CHECK(owned->weapons().empty());
    CHECK(owned->decoys().empty());
    CHECK(owned->impl().damage(reconstructed.victim) == nullptr);
    CHECK(owned->impl().takeEngagementEvents().empty());
    CHECK(fresh.impl().takeEngagementEvents().empty());
    equivalent(capture(*owned), capture(fresh));
    for (const unsigned steps : {1u, 4u, 30u}) {
        owned->step(steps);
        fresh.step(steps);
        equivalent(capture(*owned), capture(fresh));
    }
    // Active replay exposes retained next-ID/RNG/queue/guidance state that idle stepping alone would miss.
    const auto replay = contaminate(*owned, reconstructed);
    const auto freshReplay = contaminate(fresh, reference);
    CHECK(replay.liveWeapon == freshReplay.liveWeapon);
    CHECK(replay.liveDecoy == freshReplay.liveDecoy);
    CHECK(replay.releasedActivity == freshReplay.releasedActivity);
    CHECK(replay.pendingTask == freshReplay.pendingTask);
    CHECK(replay.pendingConsent == freshReplay.pendingConsent);
    CHECK(replay.queuedMessage == freshReplay.queuedMessage);
    equivalent(capture(*owned), capture(fresh));
    owned->step(30);
    fresh.step(30);
    requireDeliveredMarker(*owned, reconstructed, replay);
    requireDeliveredMarker(fresh, reference, freshReplay);
    equivalent(capture(*owned), capture(fresh));
}

TEST_CASE("native reset fixture: same-build borrowed facade is reusable by a native owner", "[ma][native-reset-fixture]") {
    fsim::session::WorldOptions nativeOptions;
    nativeOptions.name = "native-reset-borrowed";
    nativeOptions.seed = 17;
    nativeOptions.publish = false;
    nativeOptions.workers = 1;
    nativeOptions.pinWorkers = false;
    fsim::session::World native(nativeOptions);
    {
        std::unique_ptr<fsim::World> facade(fsim::World::borrowed(native));
        const auto scenario = configure(*facade);
        requireDirty(*facade, scenario, contaminate(*facade, scenario));
    }
    CHECK(native.vehicleIds().size() > 0);
}

TEST_CASE("native reset fixture: another World is unchanged during contamination", "[ma][native-reset-fixture]") {
    fsim::World first(options("native-reset-independent-a"));
    fsim::World second(options("native-reset-independent-b"));
    const auto firstScenario = configure(first);
    configure(second);
    const auto untouched = capture(second);
    contaminate(first, firstScenario);
    equivalent(capture(second), untouched);
    first.step(10);
    equivalent(capture(second), untouched);
}

TEST_CASE("native reset fixture: explicit seeds and worker counts retain required native states", "[ma][native-reset-fixture]") {
    for (const auto seed : {std::uint64_t{0}, std::uint64_t{17}, std::uint64_t{0x123456789abcdef0ULL}}) {
        INFO(seed);
        auto configured = options("native-reset-seed-coverage", 2);
        configured.seed = seed;
        fsim::World world(configured);
        const auto scenario = configure(world);
        const auto dirty = contaminate(world, scenario);
        requireDirty(world, scenario, dirty);
        CHECK(dirty.crewLost);
        CHECK(dirty.activeFire);
    }
}
