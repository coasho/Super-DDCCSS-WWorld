// VIS-M14 UI, Focused-Vehicle Filtering & Replay Radar Binding unit tests (CPU-only ImGui context):
// - B-43: MonitorGui (Chinese UI) fits cleanly inside 1600x900 @ 150% DPI without clipping or overlapping vehicle list
// - KeyHandler: clean original 4 camera modes via [C] with no extraneous overlay/PiP hotkeys
// - TacticalOverlays: 0 draw vertices when disabled, strictly focused-vehicle filtered when enabled
// - MissionGui: force-binds Radar UI to radar-equipped aircraft in replay mode and hides on missiles / non-radar vehicles

#include "visualization/ui/KeyHandler.h"
#include "visualization/ui/MissionGui.h"
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

sim::VehicleState makeJetState(double latDeg, double lonDeg, double altM) {
    auto ellipsoid = vsg::EllipsoidModel::create();
    const vsg::dvec3 jetPos = ellipsoid->convertLatLongAltitudeToECEF(vsg::dvec3(latDeg, lonDeg, altM));
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
    st.altitudeMslM = altM;
    return st;
}

} // namespace

TEST_CASE("VIS-M14 KeyHandler keeps clean 4 camera modes and no extraneous hotkeys", "[vis-m14][ui]") {
    auto controls = std::make_shared<ui::ViewerControls>();
    auto keys = ui::KeyHandler::create(controls);

    CHECK(controls->tacticalOverlayMask.load() == 0u);

    for (const vsg::KeySymbol sym : {vsg::KEY_o, vsg::KEY_i}) {
        vsg::KeyPressEvent ev;
        ev.keyBase = sym;
        keys->apply(ev);
        CHECK_FALSE(ev.handled);
    }
    CHECK(controls->tacticalOverlayMask.load() == 0u);

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
    controls->fps.store(160.0);
    controls->frameMs.store(6.2);
    controls->replayStart.store(0.0);
    controls->replayEnd.store(18.0);
    controls->simTime.store(8.5);
    auto gui = ui::MonitorGui::create(controls, "回放: emso_tactical_showcase.fsrec");
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

    ImGuiWindow* monitorWin = ImGui::FindWindowByName("飞行监控###flightsim");
    ImGuiWindow* listWin = ImGui::FindWindowByName("载具列表###vehicles");
    ImGuiWindow* replayWin = ImGui::FindWindowByName("回放控制###replay_bar");
    REQUIRE(monitorWin != nullptr);
    REQUIRE(listWin != nullptr);
    REQUIRE(replayWin != nullptr);

    CHECK(monitorWin->Pos.x >= 0.0f);
    CHECK(monitorWin->Pos.y >= 0.0f);
    CHECK(monitorWin->Pos.x + monitorWin->Size.x <= 1600.0f);
    CHECK(monitorWin->Pos.y + monitorWin->Size.y <= listWin->Pos.y + 1.0f);
    CHECK(listWin->Pos.x + listWin->Size.x <= 1600.0f);
    CHECK(listWin->Pos.y + listWin->Size.y <= 900.0f);
    CHECK(replayWin->Pos.x >= listWin->Pos.x + listWin->Size.x - 1.0f);
    CHECK(replayWin->Pos.x + replayWin->Size.x <= 1600.0f);
    CHECK(replayWin->Pos.y + replayWin->Size.y <= 900.0f);
    CHECK(controls->topBarBottomPx.load() > 20.0f);
    CHECK(controls->leftDockRightPx.load() > 40.0f);
    CHECK(controls->leftDockTopPx.load() >= controls->topBarBottomPx.load());
    CHECK(controls->bottomBarTopPx.load() > 700.0f);

    ImGui::DestroyContext();
}

TEST_CASE("VIS-M14 WorkspacePresets map panels cleanly without forcing airborne radar", "[vis-m14][ui][workspace]") {
    using ui::WorkspacePreset;
    CHECK(ui::panelsForWorkspace(WorkspacePreset::Clean, 0u) == 0u);
    CHECK(ui::panelsForWorkspace(WorkspacePreset::AirRadar, 0u) == (ui::kPanelRadar | ui::kPanelSignal | ui::kPanelFleet));
    CHECK(ui::panelsForWorkspace(WorkspacePreset::Emso, 0u) == (ui::kPanelRadar | ui::kPanelSignal | ui::kPanelEw | ui::kPanelFleet));
    CHECK(ui::panelsForWorkspace(WorkspacePreset::Weapon, 0u) == (ui::kPanelRadar | ui::kPanelWeapon | ui::kPanelAero | ui::kPanelFleet));
    CHECK(ui::panelsForWorkspace(WorkspacePreset::Flight, 0u) == (ui::kPanelAero | ui::kPanelComms | ui::kPanelFleet));
    // Custom respects user choices strictly (never forces kPanelRadar).
    CHECK(ui::panelsForWorkspace(WorkspacePreset::Custom, ui::kPanelEw) == ui::kPanelEw);
    CHECK(ui::panelsForWorkspace(WorkspacePreset::Custom, 0u) == 0u);
}

TEST_CASE("VIS-M14 TacticalOverlays enforces focused-vehicle filtering and zero cost when OFF", "[vis-m14][ui]") {
    initHeadlessImGui(1920.0f, 1080.0f, 1.0f);

    auto controls = std::make_shared<ui::ViewerControls>();
    auto overlays = ui::TacticalOverlays::create(controls);

    const sim::VehicleState st0 = makeJetState(37.65, -122.42, 4800.0);
    const sim::VehicleState st1 = makeJetState(37.70, -122.35, 5000.0);
    const vsg::dvec3 jetPos(st0.positionEcef[0], st0.positionEcef[1], st0.positionEcef[2]);
    const vsg::dvec3 up = vsg::normalize(jetPos);
    const vsg::dvec3 east = vsg::normalize(vsg::cross(vsg::dvec3(0.0, 0.0, 1.0), up));

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

    std::vector<sim::VehicleState> states{st0, st1};
    std::vector<unsigned char> alive{1, 1};
    std::vector<ui::MonitorGui::VehicleMeta> meta(2);
    meta[0].name = "viper-1"; meta[0].type = "f16c"; meta[0].level = "HSA"; meta[0].alive = true;
    meta[1].name = "viper-1-aim120c"; meta[1].type = "aim120c"; meta[1].level = "WPN"; meta[1].alive = true;
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

    // 2. Overlays ON, focused on slot 0 (viper-1) -> emits burn-through callout
    controls->tacticalOverlayMask.store(ui::kOverlayAll);
    overlays->update(&frame, Span<const sim::VehicleState>(states), alive, meta, 0,
                     view, proj, eye, 1920, 1080, 5.0);
    ImGui::NewFrame();
    overlays->record(*dummy);
    ImGui::Render();
    CHECK(ImGui::GetDrawData()->TotalVtxCount > 20);

    // 3. Overlays ON, focused on slot 1 (missile with no radar) -> ZERO vertices from slot 0's radar!
    overlays->update(&frame, Span<const sim::VehicleState>(states), alive, meta, 1,
                     view, proj, eye, 1920, 1080, 5.0);
    ImGui::NewFrame();
    overlays->record(*dummy);
    ImGui::Render();
    CHECK(ImGui::GetDrawData()->TotalVtxCount == 0);

    ImGui::DestroyContext();
}

TEST_CASE("VIS-M14 MissionGui force-binds Radar UI to supported aircraft and hides on missiles", "[vis-m14][ui][radar]") {
    CHECK(ui::MissionGui::supportsAirborneRadar("f16c"));
    CHECK(ui::MissionGui::supportsAirborneRadar("jsbsim:ea18g"));
    CHECK(ui::MissionGui::supportsAirborneRadar("e3"));
    CHECK_FALSE(ui::MissionGui::supportsAirborneRadar("aim120c"));
    CHECK_FALSE(ui::MissionGui::supportsAirborneRadar("uh60"));
    CHECK_FALSE(ui::MissionGui::supportsAirborneRadar("c172"));

    initHeadlessImGui(1920.0f, 1080.0f, 1.0f);
    auto controls = std::make_shared<ui::ViewerControls>();
    auto missionGui = ui::MissionGui::create(controls);

    const sim::VehicleState st0 = makeJetState(37.65, -122.42, 4800.0);
    ipc::TacticalOverlayFrameV1 frame;
    frame.simTime = 6.0;
    ipc::TacticalRadarVolumeWireV1 r{};
    r.vehicleId = 1;
    r.slot = 0;
    r.rangeScaleM = 80000.0f;
    r.detectionRangeM = 65000.0f;
    r.burnThroughRangeM = 12000.0f;
    r.jammerRatioDb = 30.0f;
    r.azimuthHalfWidthRad = 0.52f;
    r.elevationHalfWidthRad = 0.2f;
    r.flags = ipc::kTactRadarSearching | ipc::kTactRadarJammed | ipc::kTactRadarElectronic;
    frame.radars.push_back(r);

    vsg::CommandBuffer* dummy = nullptr;

    // 1. Focused on radar-equipped F-16C in AirRadar preset (replay) -> renders Radar ONLY (never fabricates fake R-D!)
    missionGui->updateReplay(&frame, &st0, 0, "viper-1", "f16c", 6.0);
    for (int f = 0; f < 2; ++f) {
        ImGui::NewFrame();
        missionGui->record(*dummy);
        ImGui::Render();
    }
    CHECK(ImGui::GetDrawData()->TotalVtxCount > 60);
    ImGuiWindow* radarWin = ImGui::FindWindowByName("机载雷达 - viper-1###radar0");
    REQUIRE(radarWin != nullptr);
    CHECK(radarWin->WasActive);
    CHECK(ImGui::FindWindowByName("原生距离-多普勒 ###rd_panel") == nullptr);
    const float dockLeftStd = controls->rightDockLeftPx.load();
    CHECK(dockLeftStd > 1000.0f);

    // 1b. Panel size & right dock width setting dynamically resizes the right dock!
    controls->panelScale.store(1.18f);
    controls->rightDockWidthPx.store(450.0f);
    ImGui::NewFrame();
    missionGui->record(*dummy);
    ImGui::Render();
    CHECK(controls->rightDockLeftPx.load() < dockLeftStd - 40.0f);
    controls->panelScale.store(1.0f);
    controls->rightDockWidthPx.store(370.0f);

    // 1c. Clean preset hides ALL panels including Airborne Radar (no longer forced!)
    controls->workspacePreset.store(static_cast<int>(ui::WorkspacePreset::Clean));
    ImGui::NewFrame();
    missionGui->record(*dummy);
    ImGui::Render();
    CHECK(ImGui::GetDrawData()->TotalVtxCount == 0);
    CHECK(controls->rightDockLeftPx.load() == 0.0f);

    // 2. Focused on AIM-120C missile in AirRadar preset -> Radar is capability-gated OFF!
    controls->workspacePreset.store(static_cast<int>(ui::WorkspacePreset::AirRadar));
    missionGui->updateReplay(&frame, &st0, 4, "viper-1-aim120c-5", "aim120c", 6.0);
    ImGui::NewFrame();
    missionGui->record(*dummy);
    ImGui::Render();
    CHECK(ImGui::GetDrawData()->TotalVtxCount == 0);
    CHECK(controls->rightDockLeftPx.load() == 0.0f);

    // 3. Focused on UH-60 helicopter in Flight preset -> renders 6-DOF Aero/Propulsion panel, still NO Radar!
    controls->workspacePreset.store(static_cast<int>(ui::WorkspacePreset::Flight));
    missionGui->updateReplay(&frame, &st0, 1, "uh60-hover", "uh60", 6.0);
    for (int f = 0; f < 2; ++f) {
        ImGui::NewFrame();
        missionGui->record(*dummy);
        ImGui::Render();
    }
    CHECK(ImGui::GetDrawData()->TotalVtxCount > 40);
    ImGuiWindow* aeroWin = ImGui::FindWindowByName("气动舵面与推进###aero_panel");
    ImGuiWindow* radarWinAfter = ImGui::FindWindowByName("机载雷达 - uh60-hover###radar0");
    REQUIRE(aeroWin != nullptr);
    CHECK(aeroWin->WasActive);
    CHECK((radarWinAfter == nullptr || !radarWinAfter->WasActive));

    ImGui::DestroyContext();
}
