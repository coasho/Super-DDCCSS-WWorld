// VIS-M14 UI & High-DPI Layout unit tests (CPU-only ImGui context, no Vulkan device needed):
// - B-43: MonitorGui fits cleanly inside 1600x900 @ 150% DPI without clipping or overlapping the vehicle list
// - KeyHandler: clean original 4 camera modes via [C] with no extraneous overlay/PiP hotkeys
// - TacticalOverlays: 0 draw vertices when disabled, clean anti-overlap micro-pills when enabled

#include "visualization/ui/KeyHandler.h"
#include "visualization/ui/MonitorGui.h"
#include "visualization/ui/TacticalOverlays.h"
#include "visualization/ui/ViewerControls.h"

#include <catch2/catch_test_macros.hpp>
#include <vsgImGui/imgui.h>
#include <vsgImGui/imgui_internal.h>

#include <cmath>
#include <memory>
#include <vector>

using namespace fsim;

namespace {

void initHeadlessImGui(float width, float height, float dpiScale) {
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(width, height);
    io.DeltaTime = 1.0f / 60.0f;
    io.FontGlobalScale = dpiScale;
    ImGui::GetStyle().ScaleAllSizes(dpiScale);
    unsigned char* pixels = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
}

} // namespace

TEST_CASE("VIS-M14 KeyHandler keeps clean 4 camera modes and no extraneous hotkeys", "[vis-m14][ui]") {
    auto controls = std::make_shared<ui::ViewerControls>();
    auto keys = ui::KeyHandler::create(controls);

    CHECK(controls->tacticalOverlayMask.load() == 0u);

    // [O] and [I] must remain unhandled (no extraneous hotkeys; overlays are controlled via MonitorGui / --overlays)
    for (const vsg::KeySymbol sym : {vsg::KEY_o, vsg::KEY_i}) {
        vsg::KeyPressEvent ev;
        ev.keyBase = sym;
        keys->apply(ev);
        CHECK_FALSE(ev.handled);
    }
    CHECK(controls->tacticalOverlayMask.load() == 0u);

    // [C] cycles strictly through the clean 4 camera modes (Chase -> Orbit -> Overview -> Free -> Chase)
    controls->cameraMode.store(0);
    for (int expected = 1; expected <= 4; ++expected) {
        vsg::KeyPressEvent ev;
        ev.keyBase = vsg::KEY_c;
        keys->apply(ev);
        CHECK(ev.handled);
        CHECK(controls->cameraMode.load() == (expected % 4));
    }
}

TEST_CASE("VIS-M14 B-43 MonitorGui fits 1600x900 @ 150% DPI without clipping or overlapping vehicle list", "[vis-m14][ui][b43]") {
    initHeadlessImGui(1600.0f, 900.0f, 1.5f);

    auto controls = std::make_shared<ui::ViewerControls>();
    auto gui = ui::MonitorGui::create(controls, "f16c");
    sim::SnapshotBatch batch;
    batch.simTime = 12.5;
    batch.states.resize(2);
    batch.states[0].altitudeMslM = 4500.0;
    batch.states[0].airspeedTrueMs = 240.0;
    batch.states[0].mach = 0.78;
    gui->setBatch(&batch);
    std::vector<ui::MonitorGui::VehicleMeta> vehs(2);
    vehs[0].name = "viper-1"; vehs[0].type = "f16c"; vehs[0].level = "HSA"; vehs[0].alive = true;
    vehs[1].name = "viper-2"; vehs[1].type = "f16c"; vehs[1].level = "HSA"; vehs[1].alive = true;
    gui->setVehicles(vehs);

    for (int f = 0; f < 2; ++f) {
        ImGui::NewFrame();
        vsg::CommandBuffer* dummy = nullptr;
        gui->record(*dummy);
        ImGui::Render();
    }

    ImGuiWindow* monitorWin = ImGui::FindWindowByName("flightsim");
    ImGuiWindow* listWin = ImGui::FindWindowByName("vehicles");
    REQUIRE(monitorWin != nullptr);
    REQUIRE(listWin != nullptr);

    CHECK(monitorWin->Pos.x >= 0.0f);
    CHECK(monitorWin->Pos.y >= 0.0f);
    CHECK(monitorWin->Pos.x + monitorWin->Size.x <= 1600.0f);
    CHECK(monitorWin->Pos.y + monitorWin->Size.y <= listWin->Pos.y + 1.0f);
    CHECK(listWin->Pos.x + listWin->Size.x <= 1600.0f);
    CHECK(listWin->Pos.y + listWin->Size.y <= 900.0f);

    ImGui::DestroyContext();
}

TEST_CASE("VIS-M14 TacticalOverlays emits zero vertices when OFF and clean pills when ON", "[vis-m14][ui]") {
    initHeadlessImGui(1920.0f, 1080.0f, 1.0f);

    auto controls = std::make_shared<ui::ViewerControls>();
    auto overlays = ui::TacticalOverlays::create(controls);

    auto ellipsoid = vsg::EllipsoidModel::create();
    const vsg::dvec3 jetPos = ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(37.65, -122.42, 4800.0));
    const vsg::dvec3 up = vsg::normalize(jetPos);
    const vsg::dvec3 east = vsg::normalize(vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up));
    const vsg::dvec3 north = vsg::normalize(vsg::cross(up, east));
    const vsg::dvec3 down = -up;

    sim::VehicleState st{};
    st.positionEcef[0] = jetPos.x; st.positionEcef[1] = jetPos.y; st.positionEcef[2] = jetPos.z;
    st.rotationBodyToEcef[0] = east.x;  st.rotationBodyToEcef[1] = -north.x; st.rotationBodyToEcef[2] = down.x;
    st.rotationBodyToEcef[3] = east.y;  st.rotationBodyToEcef[4] = -north.y; st.rotationBodyToEcef[5] = down.y;
    st.rotationBodyToEcef[6] = east.z;  st.rotationBodyToEcef[7] = -north.z; st.rotationBodyToEcef[8] = down.z;
    st.airspeedTrueMs = 250.0;
    st.altitudeMslM = 4800.0;

    const vsg::dvec3 eye = jetPos - east * 6000.0 + up * 4000.0;
    const vsg::dmat4 view = vsg::lookAt(eye, jetPos + east * 15000.0, up);
    const vsg::dmat4 proj = vsg::perspective(vsg::radians(35.0), 16.0 / 9.0, 1.0, 1e6);

    ipc::TacticalOverlayFrameV1 frame;
    frame.simTime = 5.0;
    ipc::TacticalRadarVolumeWireV1 r{};
    r.vehicleId = 1;
    r.slot = 0;
    r.rangeScaleM = 50000.0f;
    r.detectionRangeM = 45000.0f;
    r.burnThroughRangeM = 15000.0f;
    r.jammerRatioDb = 18.0f;
    r.azimuthHalfWidthRad = 0.45f;
    r.elevationHalfWidthRad = 0.2f;
    r.flags = ipc::kTactRadarSearching | ipc::kTactRadarJammed;
    frame.radars.push_back(r);

    std::vector<sim::VehicleState> states{st};
    std::vector<unsigned char> alive{1};
    std::vector<ui::MonitorGui::VehicleMeta> meta(1);
    meta[0].name = "viper-1"; meta[0].type = "f16c"; meta[0].level = "HSA"; meta[0].alive = true;
    vsg::CommandBuffer* dummy = nullptr;

    // 1. Overlays OFF -> zero draw vertices & 0.0 ms draw time
    controls->tacticalOverlayMask.store(0u);
    overlays->update(&frame, Span<const sim::VehicleState>(states), alive, meta, 0,
                     view, proj, eye, 1920, 1080, 5.0);
    ImGui::NewFrame();
    overlays->record(*dummy);
    ImGui::Render();
    CHECK(ImGui::GetDrawData()->TotalVtxCount == 0);
    CHECK(overlays->lastDrawMs() == 0.0);

    // 2. Overlays ON -> emits clean glassmorphic burn-through micro-pill
    controls->tacticalOverlayMask.store(ui::kOverlayAll);
    overlays->update(&frame, Span<const sim::VehicleState>(states), alive, meta, 0,
                     view, proj, eye, 1920, 1080, 5.0);
    ImGui::NewFrame();
    overlays->record(*dummy);
    ImGui::Render();
    CHECK(ImGui::GetDrawData()->TotalVtxCount > 20);

    ImGui::DestroyContext();
}
