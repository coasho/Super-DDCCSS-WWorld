// The neutral contracts alone (ARCH-02; docs/architecture/design/boundaries.md): this test is built against
// fsim::contracts and Catch2 and nothing else, so a contract that needed a domain or foundation library would not link
// here. It holds the definitions the contracts took over from the transport and the runtime to their frozen sizes,
// constants and defaults; tests/contracts/test_state_layout.cpp holds every field offset and the recording strides.

#include "contracts/recording/RecordingFormat.h"
#include "contracts/simulation/WorldOptions.h"
#include "contracts/vision_stream/VisionLayout.h"
#include "contracts/world_stream/DemoControl.h"
#include "contracts/world_stream/SnapshotBatch.h"
#include "contracts/world_stream/WorldLayout.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <type_traits>

namespace ipc = fsim::ipc;

static_assert(ipc::kMagic == 0x4D495346u && ipc::kLayoutVersion == 9);
static_assert(sizeof(ipc::VehicleSample) == 1000 && alignof(ipc::VehicleSample) == 8);
static_assert(sizeof(ipc::WorldHeader) == 384 && alignof(ipc::WorldHeader) == 64);
static_assert(sizeof(ipc::VehicleRecord) == 440);
static_assert(sizeof(ipc::MissionStore) == 40 && sizeof(ipc::EngagementEvent) == 176);
static_assert(ipc::kRecordingVersion == 7);
static_assert(sizeof(ipc::RecordingHeader) == 88 && sizeof(ipc::RecordedVehicle) == 440);
static_assert(sizeof(ipc::RecordingSnapshotHead) == 16);
static_assert(std::is_trivially_copyable_v<ipc::RecordingHeader> && std::is_trivially_copyable_v<ipc::RecordedVehicle>);
static_assert(ipc::kVisionMagic == 0x49565346u && ipc::kVisionVersion == 1 && ipc::kMaxVisionCameras == 64);
static_assert(sizeof(ipc::VisionCameraRecord) == 64 && sizeof(ipc::VisionHeader) == 4144);

TEST_CASE("the contracts build alone and keep their frozen definitions", "[architecture][contracts]") {
    REQUIRE(std::string(ipc::kRecordingMagic, 5) == "FSREC");

    const fsim::session::WorldOptions options;
    REQUIRE(options.name == "default");
    REQUIRE(options.dt == 1.0 / 120.0);
    REQUIRE(options.frameSkip == 4);
    REQUIRE(options.workers == 0);
    REQUIRE(options.pinWorkers);
    REQUIRE(options.seed == 0);
    REQUIRE(options.capacity == 256);
    REQUIRE(options.publish);
    REQUIRE(options.storeDrag);
    REQUIRE(options.publishIntervalSeconds == 1.0 / 60.0);
    REQUIRE(options.jsbsimRoot.empty());
    REQUIRE(options.ground == nullptr);
    REQUIRE_FALSE(options.terrain);
    REQUIRE(options.terrainUrl.empty());
    REQUIRE(options.terrainZoom == 15);
    REQUIRE(options.terrainPrefetchRadiusM == 1500.0);
    REQUIRE_FALSE(options.terrainOffline);
    REQUIRE(options.recordPath.empty());
    REQUIRE(options.recordIntervalSeconds == 0.0);

    const fsim::sim::SnapshotBatch batch;
    REQUIRE(batch.sequence == 0);
    REQUIRE(batch.states.empty());

    // the inline origin the viewer hangs a model from: the CG less its place from the origin, in body axes
    fsim::sim::VehicleState state;
    state.positionEcef[0] = 10.0;
    double derived[ipc::kDerivedCount] = {};
    derived[ipc::kDerivedCgFromOriginM] = 2.0;
    double origin[3];
    ipc::originEcef(state, derived, origin);
    REQUIRE(origin[0] == 10.0 - state.rotationBodyToEcef[0] * 2.0);
}

TEST_CASE("the demo control block and the control levels' names are stream contracts", "[architecture][contracts]") {
    // ARCH-08 (HC-16): a fresh block holds no command and the neutral time factor until the viewer stamps it
    const ipc::DemoControl block;
    REQUIRE(block.magic == 0u);
    REQUIRE(block.paused.load() == 0u);
    REQUIRE(block.quit.load() == 0u);
    REQUIRE(block.stepRequests.load() == 0u);
    REQUIRE(block.timeFactor.load() == 1.0);
    REQUIRE(ipc::kDemoControlMagic == 0x4C434446u);
    REQUIRE(ipc::demoControlName("demo-7") == "demo-7.control");
    STATIC_REQUIRE(std::is_standard_layout_v<ipc::DemoControl>);
    // (HC-17) the names a presentation shows for VehicleRecord::controlLevel; "?" past them
    REQUIRE(ipc::kControlLevels == 6);
    REQUIRE(std::string(ipc::controlLevelName(0)) == "actuator");
    REQUIRE(std::string(ipc::controlLevelName(5)) == "behavior");
    REQUIRE(std::string(ipc::controlLevelName(6)) == "?");
}
