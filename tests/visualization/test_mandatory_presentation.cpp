#include "visualization/ui/KeyHandler.h"
#include "visualization/ui/MandatoryPanel.h"
#include "visualization/scene/Clouds.h"
#include "visualization/vfx/effects/Airflow.h"

#include <vsgImGui/imgui_internal.h>

#include <cstdio>
#include <memory>
#include <stdexcept>

int main() {
    auto controls = std::make_shared<fsim::ui::ViewerControls>();
    auto keys = fsim::ui::KeyHandler::create(controls);
    unsigned checks = 0, failures = 0;
    // Removed presentation shortcuts must not consume keys or hide any panel.
    for (auto key : {vsg::KEY_l, vsg::KEY_L, vsg::KEY_v, vsg::KEY_V,
                     vsg::KEY_p, vsg::KEY_P, vsg::KEY_m, vsg::KEY_n,
                     vsg::KEY_t, vsg::KEY_e, vsg::KEY_k,
                     vsg::KEY_Leftbracket, vsg::KEY_Rightbracket}) {
        vsg::KeyPressEvent event;
        event.keyBase = key;
        keys->apply(event);
        ++checks;
        if (event.handled) {
            ++failures;
            std::fprintf(stderr, "Removed presentation key %u was handled\n", unsigned(key));
        }
    }
    // ImGui needs no Vulkan device for layout. Seed a legacy collapsed layout, then
    // use the very same panel entry point as monitor/cameras/mission/radar.
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(3840, 2160);
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels = nullptr;
    int width = 0, height = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    ImGui::NewFrame();
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Always);
    ImGui::Begin("legacy collapsed panel");
    ImGui::End();
    ImGui::Render();
    ImGui::NewFrame();
    fsim::ui::beginMandatoryPanel("legacy collapsed panel");
    const auto* panel = ImGui::GetCurrentWindow();
    const auto check = [&](bool condition, const char* message) {
        ++checks;
        if (!condition) { ++failures; std::fprintf(stderr, "%s\n", message); }
    };
    check(!panel->Collapsed, "Legacy collapsed state was retained");
    check(!panel->HasCloseButton, "A required panel has a close button");
    check((panel->Flags & ImGuiWindowFlags_NoCollapse) != 0, "A required panel can collapse");
    ImGui::TextUnformatted("Visible SDK report");
    ImGui::End();
    ImGui::Render();
    check(ImGui::GetDrawData()->TotalVtxCount > 0, "Required panel produced no visible draw data");
    ImGui::DestroyContext();
    for (const auto& dir : {std::filesystem::path{}, std::filesystem::path("missing-required-cloud-assets")}) {
        fsim::world::Clouds clouds(vsg::Options::create(), dir);
        bool refused = false;
        try { clouds.build(3840, 2160, [](vsg::ref_ptr<vsg::Node>) { return true; }); }
        catch (const std::runtime_error&) { refused = true; }
        check(refused, "Missing required cloud assets silently produced a reduced scene");
        // Resource validation is CPU-only: no window/device is needed to refuse
        // a contrail renderer without its required shaders/noise/atmosphere.
        fsim::world::Airflow airflow(0, vsg::Options::create(), dir, fsim::world::AtmosphereTables{});
        check(!airflow.valid(), "Missing required contrail resources reported a ready renderer");
    }
    std::printf("Mandatory presentation: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
