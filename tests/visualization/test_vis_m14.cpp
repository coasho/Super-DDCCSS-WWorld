// VIS-M14 Visualization & Fleet VFX unit tests (CPU-only, no Vulkan device required):
// - B-57: 8-engine Exhaust::kPerSlot (B-52H 8x TF33), per-engine turbineExitK (F-22A / Mirage 2000)
// - B-59: Rotorcraft (UH-60, UH-1H) and turboprop (C-130J) placed exhaust heat-haze & rotor disk manifests
// - OfflineTiles::hasAnyChild & ElevatedTile::pruneOfflineLeaves (zero read_subtile warning spam)
// - CameraController Weapon & Cockpit modes

#include "contracts/world_stream/TacticalOverlayLayoutV1.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "visualization/scene/CameraController.h"
#include "visualization/scene/ElevatedTile.h"
#include "visualization/scene/OfflineTiles.h"
#include "visualization/scene/VehicleVisuals.h"
#include "visualization/vfx/effects/Exhaust.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <vsgXchange/all.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace fsim;

namespace {

sim::VehicleState sampleAirborneState(double latDeg, double lonDeg, double altM, double speedMs) {
    sim::VehicleState s{};
    s.simTime = 5.0;
    s.latitudeRad = latDeg * 3.14159265358979323846 / 180.0;
    s.longitudeRad = lonDeg * 3.14159265358979323846 / 180.0;
    s.altitudeMslM = altM;
    s.altitudeAglM = altM;
    auto ell = vsg::EllipsoidModel::create();
    const vsg::dvec3 p = ell->convertLatLongAltitudeToECEF(vsg::dvec3(latDeg, lonDeg, altM));
    s.positionEcef[0] = p.x;
    s.positionEcef[1] = p.y;
    s.positionEcef[2] = p.z;
    // Level north-facing body frame at equator/prime-ish orientation: build valid orthonormal ECEF basis.
    const vsg::dvec3 up = vsg::normalize(p);
    const vsg::dvec3 east = vsg::normalize(vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up));
    const vsg::dvec3 north = vsg::normalize(vsg::cross(up, east));
    const vsg::dvec3 down = -up;
    // Columns of rotationBodyToEcef are body X (north), body Y (east), body Z (down) in row-major order.
    s.rotationBodyToEcef[0] = north.x; s.rotationBodyToEcef[1] = east.x; s.rotationBodyToEcef[2] = down.x;
    s.rotationBodyToEcef[3] = north.y; s.rotationBodyToEcef[4] = east.y; s.rotationBodyToEcef[5] = down.y;
    s.rotationBodyToEcef[6] = north.z; s.rotationBodyToEcef[7] = east.z; s.rotationBodyToEcef[8] = down.z;
    s.velocityBodyMs[0] = speedMs;
    s.airspeedTrueMs = speedMs;
    s.mach = speedMs / 340.0;
    s.engineCount = 4;
    for (int i = 0; i < 4; ++i) {
        s.throttlePosition[i] = 0.85;
        s.engineN2[i] = 92.0;
        s.thrustN[i] = 60000.0;
        s.engineRpm[i] = 300.0;
    }
    return s;
}

} // namespace

TEST_CASE("VIS-M14 B-57 8-engine B-52H flames and per-engine turbineExitK manifest override", "[vis-m14][render][b57]") {
    static_assert(world::Exhaust::kPerSlot == 8u);

    // Per-engine turbineExitK raises reheat total temperature physically (Mirage 2000 M53-P2 1180 K vs default 1050 K).
    const double tDefault = world::Exhaust::reheatTotalK(0.36, 64300.0, 95100.0, world::Exhaust::kTurbineExitK);
    const double tM53 = world::Exhaust::reheatTotalK(0.36, 64300.0, 95100.0, 1180.0);
    CHECK(tM53 > tDefault + 100.0);
    CHECK(tM53 <= world::Exhaust::kMostReheatK);

    // Verify F-22A and Mirage 2000 manifests parse their custom turbineExitK values.
    {
        world::VehicleVisuals::Settings f22;
        f22.modelPath = (std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "f22a" / "f22a.glb").string();
        REQUIRE(world::VehicleVisuals::applyManifest(f22));
        REQUIRE(f22.engineExhaustK.count(0) == 1u);
        REQUIRE(f22.engineExhaustK.count(1) == 1u);
        CHECK(f22.engineExhaustK[0] == Catch::Approx(1280.0));
    }
    {
        world::VehicleVisuals::Settings m2k;
        m2k.modelPath = (std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "mirage2000" / "mirage2000.glb").string();
        REQUIRE(world::VehicleVisuals::applyManifest(m2k));
        REQUIRE(m2k.engineExhaustK.count(0) == 1u);
        CHECK(m2k.engineExhaustK[0] == Catch::Approx(1180.0));
    }

    // Load real B-52H model + manifest and drive all 8 engines through TacticalOverlayFrameV1.
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(FSIM_TEST_AIRCRAFT_DIR);
    world::VehicleVisuals visuals(1, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:b52h");
    const auto& shape = visuals.shape(0);
    REQUIRE(shape.valid);
    CHECK(shape.engines.size() == 8u);
    REQUIRE(shape.engineExhaustK.count(7) == 1u);
    CHECK(shape.engineExhaustK.at(7) == Catch::Approx(950.0));

    sim::VehicleState st = sampleAirborneState(37.62, -122.38, 9000.0, 240.0);
    ipc::TacticalOverlayFrameV1 tact;
    tact.simTime = st.simTime;
    ipc::TacticalPropulsionExtWireV1 prop{};
    prop.vehicleId = 1;
    prop.slot = 0;
    prop.engineCount = 8;
    prop.flags = 0u;
    for (int e = 0; e < 8; ++e) {
        prop.throttle[e] = 0.85f;
        prop.engineN2[e] = 91.0f;
        prop.thrustN[e] = 55000.0f;
        prop.fuelFlowKgS[e] = 0.72f;
    }
    tact.propulsion.push_back(prop);

    std::vector<sim::VehicleState> states{st};
    visuals.update(states, &tact);
    CHECK(visuals.flames().size() == 8u);

    // Cut engines 4..7 only: flames drop from 8 to 4, proving outboard engines are driven independently!
    for (int e = 4; e < 8; ++e) {
        tact.propulsion[0].engineN2[e] = 0.0f;
        tact.propulsion[0].thrustN[e] = 0.0f;
    }
    visuals.update(states, &tact);
    CHECK(visuals.flames().size() == 4u);
}

TEST_CASE("VIS-M14 B-59 rotorcraft and turboprop manifests drive placed exhaust heat haze and rotor geometry", "[vis-m14][render][b59]") {
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(FSIM_TEST_AIRCRAFT_DIR);
    world::VehicleVisuals visuals(3, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:uh60");
    visuals.setModel(1, {}, "jsbsim:uh1h");
    visuals.setModel(2, {}, "jsbsim:c130j");

    REQUIRE(visuals.shape(0).valid);
    CHECK(visuals.shape(0).rotors.size() == 2u);
    CHECK(visuals.shape(0).rotors[0].radiusM == Catch::Approx(8.18).margin(0.01));
    CHECK(visuals.shape(0).jets.size() == 2u);
    CHECK(visuals.shape(0).jets[0].exhaustStack);

    REQUIRE(visuals.shape(1).valid);
    CHECK(visuals.shape(1).rotors.size() == 2u);
    CHECK(visuals.shape(1).rotors[0].radiusM == Catch::Approx(7.315).margin(0.01));
    CHECK(visuals.shape(1).jets.size() == 1u);
    CHECK(visuals.shape(1).jets[0].exhaustStack);

    REQUIRE(visuals.shape(2).valid);
    CHECK(visuals.shape(2).rotors.size() == 4u);
    CHECK(visuals.shape(2).jets.size() == 4u);
    CHECK(visuals.shape(2).jets[0].exhaustStack);

    std::vector<sim::VehicleState> states{
        sampleAirborneState(37.62, -122.38, 12.0, 5.0),
        sampleAirborneState(37.63, -122.38, 12.0, 5.0),
        sampleAirborneState(37.64, -122.38, 1500.0, 140.0),
    };
    visuals.update(states);
    // 2 (UH-60) + 1 (UH-1H) + 4 (C-130J) = 7 placed exhaust heat-haze stacks!
    CHECK(visuals.flames().size() == 7u);
}

TEST_CASE("VIS-M14 OfflineTiles hasAnyChild and ElevatedTile leaf pruning stops missing subtile paging", "[vis-m14][render][terrain]") {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "fsim_vis_m14_offline_prune_test";
    std::error_code ec;
    fs::remove_all(root, ec);
    fs::create_directories(root / "img" / "0" / "0");
    fs::create_directories(root / "img" / "1" / "0");
    {
        std::ofstream(root / "img" / "0" / "0" / "0.jpg") << "x";
        std::ofstream(root / "img" / "1" / "0" / "0.jpg") << "x";
    }

    const std::string imgTmpl = (root / "img" / "{z}" / "{y}" / "{x}.jpg").generic_string();
    auto offline = world::OfflineTiles::create(imgTmpl, "", world::ElevationEncoding::Terrarium);
    CHECK(offline->hasAnyChild(0, 0, 0));
    CHECK_FALSE(offline->hasAnyChild(1, 0, 0));
    CHECK_FALSE(offline->hasAnyChild(1, 1, 1));

    // Build a synthetic tile group containing two PagedLODs: one with children on disk, one leaf without.
    auto group = vsg::Group::create();
    auto pParent = vsg::PagedLOD::create();
    pParent->bound = vsg::dsphere(0.0, 0.0, 0.0, 1000.0);
    pParent->filename = "0 0 0.tile";
    pParent->children[0] = vsg::PagedLOD::Child{0.25, {}};
    pParent->children[1] = vsg::PagedLOD::Child{0.0, vsg::Group::create()};

    auto pLeaf = vsg::PagedLOD::create();
    pLeaf->bound = vsg::dsphere(0.0, 0.0, 0.0, 500.0);
    pLeaf->filename = "0 0 1.tile";
    pLeaf->children[0] = vsg::PagedLOD::Child{0.25, {}};
    pLeaf->children[1] = vsg::PagedLOD::Child{0.0, vsg::Group::create()};

    group->addChild(pParent);
    group->addChild(pLeaf);

    const std::size_t pruned = world::ElevatedTile::pruneOfflineLeaves(group.get(), *offline);
    CHECK(pruned == 1u);
    CHECK(group->children[0].cast<vsg::PagedLOD>() != nullptr);
    CHECK(group->children[1].cast<vsg::PagedLOD>() == nullptr);
    auto collapsed = group->children[1].cast<vsg::CullGroup>();
    REQUIRE(collapsed != nullptr);
    CHECK(collapsed->bound.radius == Catch::Approx(500.0));

    // Direct ElevatedTile::read on a missing subtile must return empty silently (never a ReadError).
    auto tileSettings = vsg::createOpenStreetMapSettings(vsg::Options::create());
    tileSettings->imageLayer = imgTmpl;
    auto reader = world::ElevatedTile::create(tileSettings, vsg::Options::create(), offline);
    auto missing = reader->read("0 0 1.tile", vsg::Options::create());
    CHECK(missing == nullptr);

    fs::remove_all(root, ec);
}

TEST_CASE("VIS-M14 CameraController Weapon and Cockpit director modes", "[vis-m14][render][camera]") {
    auto ellipsoid = vsg::EllipsoidModel::create();
    auto lookAt = vsg::LookAt::create(vsg::dvec3(0.0, 0.0, 7000000.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 1.0, 0.0));
    auto persp = vsg::Perspective::create(30.0, 16.0 / 9.0, 1.0, 1e7);
    auto cam = vsg::Camera::create(persp, lookAt, vsg::ViewportState::create(VkExtent2D{1920, 1080}));
    auto ctrl = world::CameraController::create(cam, lookAt, ellipsoid);

    CHECK(std::string(world::CameraController::modeName(world::CameraController::Mode::Weapon)) == "weapon");
    CHECK(std::string(world::CameraController::modeName(world::CameraController::Mode::Cockpit)) == "cockpit");

    const sim::VehicleState jet = sampleAirborneState(37.62, -122.38, 4000.0, 260.0);
    const vsg::dvec3 jetPos(jet.positionEcef[0], jet.positionEcef[1], jet.positionEcef[2]);

    // Weapon director mode tracks close behind the target.
    ctrl->setMode(world::CameraController::Mode::Weapon);
    ctrl->setTargetRadius(2.0);
    ctrl->update(&jet, 0.016);
    const double wpnDist = vsg::length(lookAt->eye - jetPos);
    CHECK(wpnDist > 4.0);
    CHECK(wpnDist < 120.0);

    // Cockpit mode places the eye at the body-frame pilot eye position looking forward along nose (+X).
    ctrl->setMode(world::CameraController::Mode::Cockpit);
    ctrl->setCockpitEyeBodyM(vsg::dvec3(4.2, 0.0, -0.8));
    ctrl->update(&jet, 0.016);
    const double* R = jet.rotationBodyToEcef;
    const vsg::dvec3 expectedEye = jetPos + vsg::dvec3(
        R[0] * 4.2 + R[2] * (-0.8),
        R[3] * 4.2 + R[5] * (-0.8),
        R[6] * 4.2 + R[8] * (-0.8));
    CHECK(vsg::length(lookAt->eye - expectedEye) < 1e-4);
    const vsg::dvec3 fwd = vsg::normalize(lookAt->center - lookAt->eye);
    const vsg::dvec3 bodyX(R[0], R[3], R[6]);
    CHECK(vsg::dot(fwd, bodyX) > 0.999);
}
