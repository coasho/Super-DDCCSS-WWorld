#include "visualization/ui/FusionGui.h"
#include <vsgImGui/imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim::ui {
void drawFusionV1(const ipc::FusionRecordV1& record,bool available,const std::string& name) {
    const auto display=ImGui::GetIO().DisplaySize;const float scale=ImGui::GetIO().FontGlobalScale;
    // Below the status panel, above the fleet table; preserve both native reports.
    ImGui::SetNextWindowPos(ImVec2(12*scale,380*scale),ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(690*scale,display.x*.48f),280*scale),ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(.96f);
    char title[160];std::snprintf(title,sizeof title,"fused tracks - %s###fusion",name.empty()?"?":name.c_str());
    if(!ImGui::Begin(title,nullptr,ImGuiWindowFlags_NoCollapse)){ImGui::End();return;}
    if(!available){ImGui::TextDisabled("native fusion report unavailable");ImGui::End();return;}
    ImGui::Text("%u of %u entities | radar / delivered link",record.count,record.total);
    ImGui::TextDisabled("sample %.3f s | rejected %llu | capacity refusals %llu",record.timeS,
        static_cast<unsigned long long>(record.rejectedInputs),static_cast<unsigned long long>(record.capacityRefusals));
    const auto flags=ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_ScrollY;
    if(ImGui::BeginTable("fused",7,flags)) {
        for(const char* label:{"ID","state","sources","age s","sigma m","speed m/s","identity"})ImGui::TableSetupColumn(label);
        ImGui::TableSetupScrollFreeze(0,1);ImGui::TableHeadersRow();
        for(std::uint32_t i=0;i<record.count;++i) {
            const auto& e=record.entities[i];ImGui::TableNextRow();ImGui::TableSetColumnIndex(0);
            ImGui::Text("%llu",static_cast<unsigned long long>(e.id));ImGui::TableSetColumnIndex(1);
            static const char* states[]={"live","coasting","lost"};ImGui::TextUnformatted(e.state<3?states[e.state]:"unknown");
            ImGui::TableSetColumnIndex(2);const char* kind=e.sourceKinds==3?"radar+link":e.sourceKinds==1?"radar":e.sourceKinds==2?"link":"unknown";
            ImGui::Text("%u %s",e.sourceCount,kind);ImGui::TableSetColumnIndex(3);ImGui::Text("%.2f",record.timeS-e.lastUpdateS);
            ImGui::TableSetColumnIndex(4);ImGui::Text("%.1f",std::sqrt(std::max(0.,(e.covarianceEcef[0]+e.covarianceEcef[14]+e.covarianceEcef[28])/3)));
            ImGui::TableSetColumnIndex(5);ImGui::Text("%.1f",std::sqrt(e.stateEcef[1]*e.stateEcef[1]+e.stateEcef[3]*e.stateEcef[3]+e.stateEcef[5]*e.stateEcef[5]));
            ImGui::TableSetColumnIndex(6);if(e.identityKnown)ImGui::Text("reported %u",e.standardIdentity);else ImGui::TextDisabled("unknown");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}
}
