#include "visualization/ui/FusionGui.h"
#include "visualization/ui/Colour.h"
#include "visualization/ui/MandatoryPanel.h"
#include <vsgImGui/imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim::ui {
void drawFusionV1(const ipc::FusionRecordV1& record, bool available, const std::string& name) {
    const auto display = ImGui::GetIO().DisplaySize;
    const float scale = uiScale();
    ImGui::SetNextWindowPos(ImVec2(12.0f * scale, 380.0f * scale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(560.0f * scale, display.x * 0.44f), 250.0f * scale), ImGuiCond_FirstUseEver);
    char title[160];
    std::snprintf(title, sizeof title, "融合航迹 - %s###fusion", name.empty() ? "?" : name.c_str());
    if (!beginMandatoryPanel(title)) {
        ImGui::End();
        return;
    }
    if (!available) {
        ImGui::TextDisabled("暂无融合航迹数据");
        ImGui::End();
        return;
    }
    ImGui::Text("已融合 %u / %u 目标  ·  雷达 / 数据链", record.count, record.total);
    ImGui::TextDisabled("采样 %.2f s  ·  拒收 %llu  ·  容量满 %llu", record.timeS,
                        static_cast<unsigned long long>(record.rejectedInputs),
                        static_cast<unsigned long long>(record.capacityRefusals));
    const auto flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY |
                       ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("fused", 7, flags)) {
        for (const char* label : {"编号", "状态", "来源", "延迟 s", "误差 m", "速度 m/s", "敌我识别"})
            ImGui::TableSetupColumn(label);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableHeadersRow();
        for (std::uint32_t i = 0; i < record.count; ++i) {
            const auto& e = record.entities[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%llu", static_cast<unsigned long long>(e.id));
            ImGui::TableSetColumnIndex(1);
            static const char* states[] = {"跟踪", "外推", "丢失"};
            ImGui::TextUnformatted(e.state < 3 ? states[e.state] : "未知");
            ImGui::TableSetColumnIndex(2);
            const char* kind = e.sourceKinds == 3 ? "雷达+链路"
                             : e.sourceKinds == 1 ? "雷达"
                             : e.sourceKinds == 2 ? "数据链"
                                                  : "未知";
            ImGui::Text("%u %s", e.sourceCount, kind);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.2f", record.timeS - e.lastUpdateS);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%.1f", std::sqrt(std::max(0.0, (e.covarianceEcef[0] + e.covarianceEcef[14] + e.covarianceEcef[28]) / 3.0)));
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.1f", std::sqrt(e.stateEcef[1] * e.stateEcef[1] + e.stateEcef[3] * e.stateEcef[3] + e.stateEcef[5] * e.stateEcef[5]));
            ImGui::TableSetColumnIndex(6);
            if (e.identityKnown) ImGui::Text("识别 %u", e.standardIdentity);
            else ImGui::TextDisabled("未知");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}
} // namespace fsim::ui
