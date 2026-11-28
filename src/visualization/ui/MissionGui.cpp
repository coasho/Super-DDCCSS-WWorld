#include "visualization/ui/MissionGui.h"
#include "visualization/ui/FusionGui.h"
#include "visualization/ui/MandatoryPanel.h"

#include "visualization/ui/Colour.h"

#include "foundation/transport/WorldMirror.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace fsim::ui {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

const char* stateName(std::uint8_t s) {
    static const char* names[] = {"关闭", "启动中", "待机", "工作", "自检", "故障"};
    return s < 6 ? names[s] : "?";
}

ImVec4 stateColour(std::uint8_t s) {
    switch (s) {
    case 1: return tint(1.0f, 0.85f, 0.30f);  // starting
    case 2: return tint(0.45f, 0.80f, 1.00f); // standby
    case 3: return tint(0.35f, 0.94f, 0.54f); // operate
    case 4: return tint(1.0f, 0.62f, 0.22f);  // bit
    case 5: return tint(1.0f, 0.34f, 0.30f);  // failed
    default: return tint(0.58f, 0.64f, 0.72f); // off
    }
}

const char* familyName(std::uint8_t f) {
    static const char* names[] = {"空空雷达", "气象雷达", "合成孔径", "地面动目标", "电子侦察", "光电/红外", "电子干扰",
                                  "外挂弹药", "机炮", "诱饵对抗", "通信电台", "导航系统", "地形测绘"};
    static_assert(sizeof(names) / sizeof(names[0]) == fsim::ipc::kMissionFamilies, "a label for every family");
    return f < sizeof(names) / sizeof(names[0]) ? names[f] : "?";
}

const char* modeName(std::uint8_t m) {
    static const char* names[] = {"RWS", "VS", "TWS", "SURV", "STT"};
    return m < 5 ? names[m] : "?";
}

const char* modeLong(std::uint8_t m) {
    static const char* names[] = {"边搜索边测距", "速度搜索", "边扫描边跟踪", "全向警戒", "单目标锁定"};
    return m < 5 ? names[m] : "?";
}

struct Column {
    float right, width;
    float radarTop, radarHeight;
    float mapTop, mapHeight;
};

Column column(std::uint32_t payload, bool hasPayloadsAbove) {
    const float k = uiScale();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float gap = 8.0f * k;
    Column c;
    c.right = display.x - 12.0f * k;
    c.width = std::min(390.0f * k, display.x * 0.30f);
    c.radarTop = hasPayloadsAbove
                     ? (display.y * 0.32f + 24.0f * k * static_cast<float>(payload))
                     : (std::max(12.0f * k, display.y * 0.095f) + 24.0f * k * static_cast<float>(payload));
    const float room = std::max(180.0f * k, display.y - 12.0f * k - c.radarTop - gap);
    c.radarHeight = hasPayloadsAbove
                        ? std::min(440.0f * k, 0.56f * room)
                        : std::min(460.0f * k, display.y * 0.48f);
    c.mapTop = c.radarTop + c.radarHeight + gap;
    c.mapHeight = std::min(360.0f * k, std::max(120.0f * k, room - c.radarHeight));
    return c;
}

} // namespace

bool MissionGui::supportsAirborneRadar(const std::string& aircraftType) noexcept {
    if (aircraftType.empty()) return false;
    std::string lo;
    lo.reserve(aircraftType.size());
    for (char ch : aircraftType) {
        if (ch == '-' || ch == '_' || ch == ' ') continue;
        lo.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (lo.rfind("jsbsim:", 0) == 0) lo = lo.substr(7);

    // Explicit non-radar ordnance, helicopters, light GA, and close-air-support airframes.
    for (const char* deny : {"aim", "pl1", "pl2", "r73", "r77", "r27", "mk8", "gbu", "agm", "jd", "sdb",
                             "missile", "bomb", "rocket", "torpedo", "chaff", "flare",
                             "uh60", "uh1", "ah64", "ch47", "mi8", "mi24", "ka52", "ec135", "heli",
                             "c172", "cessna", "pa28", "a10", "mq9", "quad", "uav", "drone"}) {
        if (lo.find(deny) != std::string::npos) return false;
    }
    // Radar-equipped fighters, EW jets, AEW&C, bombers, maritime patrol, and tactical transports.
    for (const char* allow : {"f14", "f15", "f16", "f18", "fa18", "ea18", "f22", "f35", "f4", "f5",
                              "su27", "su30", "su33", "su34", "su35", "su57", "mig29", "mig31", "mig35",
                              "j10", "j11", "j15", "j16", "j20", "j35", "jf17",
                              "mirage", "rafale", "typhoon", "ef2000", "gripen", "jas39", "tornado",
                              "e2", "e3", "e7", "kj500", "kj2000", "awacs", "a50",
                              "b1", "b2", "b52", "h6", "tu95", "tu160", "p8", "c130"}) {
        if (lo.find(allow) != std::string::npos) return true;
    }
    return false;
}

MissionGui::MissionGui(std::shared_ptr<ViewerControls> controls)
    : controls_(std::move(controls)), record_(std::make_unique<ipc::MissionRecord>()),
      fusion_(std::make_unique<ipc::FusionRecordV1>()) {}

void MissionGui::update(const ipc::WorldMirror* mirror, int slot, const std::string& name, double simTime,
                        const ipc::TacticalOverlayFrameV1* tactical) {
    replayMode_ = false;
    replaySupportsRadar_ = false;
    replayRadars_.clear();
    hasOwnState_ = false;

    fusionValid_ = false;
    mirrored_ = mirror && mirror->valid();
    simTime_ = simTime;
    name_ = name;
    if (!mirrored_ || slot < 0) {
        valid_ = false;
        slot_ = -1;
        return;
    }
    if (slot != slot_) {
        valid_ = false;
        slot_ = slot;
    }
    if (mirror->mission(static_cast<std::uint32_t>(slot), *record_, valid_ ? record_->revision : ~0ull)) valid_ = true;
    fusionValid_ = mirror->fusion(static_cast<std::uint32_t>(slot), *fusion_);
    if (tactical) {
        for (const auto& r : tactical->radars) {
            if (static_cast<int>(r.slot) == slot) replayRadars_.push_back(r);
        }
    }
}

void MissionGui::updateReplay(const ipc::TacticalOverlayFrameV1* tactical,
                              const sim::VehicleState* ownState,
                              int slot,
                              const std::string& name,
                              const std::string& type,
                              double simTime) {
    mirrored_ = false;
    valid_ = false;
    fusionValid_ = false;
    replayRadars_.clear();

    slot_ = slot;
    name_ = name;
    type_ = type;
    simTime_ = simTime;
    hasOwnState_ = (ownState != nullptr);
    if (ownState) ownState_ = *ownState;

    replayMode_ = (slot >= 0);
    if (!replayMode_) {
        replaySupportsRadar_ = false;
        return;
    }
    if (tactical) {
        for (const auto& r : tactical->radars) {
            if (static_cast<int>(r.slot) == slot) replayRadars_.push_back(r);
        }
    }
    replaySupportsRadar_ = !replayRadars_.empty() || supportsAirborneRadar(type_);
}

void MissionGui::record(vsg::CommandBuffer&) const {
    if (mirrored_) {
        if (fusionValid_) drawFusionV1(*fusion_, fusionValid_, name_);
        if (valid_ && record_->payloadCount > 0) drawPayloads();
        if (valid_) {
            for (std::uint32_t i = 0; i < record_->radarCount && i < ipc::kMissionRadars; ++i) drawRadar(record_->radars[i]);
            if (record_->rangeDoppler.radar < record_->radarCount && record_->radars[record_->rangeDoppler.radar].signalLevel)
                drawRangeDoppler(record_->rangeDoppler);
        }
        return;
    }
    if (replayMode_) {
        if (!replayRadars_.empty()) {
            for (std::uint32_t i = 0; i < replayRadars_.size(); ++i) drawReplayRadar(replayRadars_[i], i);
        } else if (replaySupportsRadar_) {
            drawReplayStandbyRadar();
        }
    }
}

void MissionGui::drawPayloads() const {
    const float k = uiScale();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(display.x - 12.0f * k, 12.0f * k), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(std::min(430.0f * k, display.x * 0.34f), 0.0f), ImGuiCond_FirstUseEver);
    char title[128];
    std::snprintf(title, sizeof title, "机载载荷 - %s###mission", name_.empty() ? "?" : name_.c_str());
    if (!beginMandatoryPanel(title)) {
        ImGui::End();
        return;
    }
    if (!valid_ || record_->vehicleId == 0) {
        ImGui::TextDisabled("暂无该载具的任务系统数据");
        ImGui::End();
        return;
    }
    if (record_->payloadCount == 0) {
        ImGui::TextDisabled("无任务载荷配置");
        ImGui::End();
        return;
    }
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("payloads", 4, flags)) {
        ImGui::TableSetupColumn("型号", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("类型", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("能力", ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableHeadersRow();
        for (std::uint32_t i = 0; i < record_->payloadCount && i < ipc::kMissionPayloads; ++i) {
            const ipc::MissionPayload& p = record_->payloads[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", p.designation);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s%s", p.name, p.count > 1 ? "（多套）" : "");
            ImGui::TableSetColumnIndex(1);
            if (p.count > 1) ImGui::TextDisabled("%s x%u", familyName(p.family), static_cast<unsigned>(p.count));
            else ImGui::TextDisabled("%s", familyName(p.family));
            ImGui::TableSetColumnIndex(2);
            ImGui::TextColored(stateColour(p.state), "%s", stateName(p.state));
            if ((p.state == 1 || p.state == 4) && std::isfinite(p.percent)) {
                ImGui::SameLine();
                ImGui::TextDisabled("%.0f%%", static_cast<double>(p.percent));
            }
            ImGui::TableSetColumnIndex(3);
            ImGui::TextWrapped("%s", p.capabilities[0] ? p.capabilities : "仅状态");
        }
        ImGui::EndTable();
    }
    if (record_->storeCount > 0) {
        static constexpr const char* kStatus[] = {"已挂载", "通电中", "就绪", "释放中", "已释放", "已抛弃", "断电中", "损毁"};
        ImGui::Separator();
        ImGui::TextDisabled("外挂弹药:");
        for (std::uint32_t i = 0; i < record_->storeCount && i < ipc::kMissionStores; ++i) {
            const ipc::MissionStore& s = record_->stores[i];
            const char* status = s.status < 8 ? kStatus[s.status] : "?";
            const bool gone = s.status == 4 || s.status == 5 || s.status == 7;
            if (i > 0) ImGui::SameLine(0.0f, 12.0f * k);
            if (ImGui::GetContentRegionAvail().x < 100.0f * k) ImGui::NewLine();
            if (gone) ImGui::TextDisabled("%u %s", static_cast<unsigned>(s.station), s.mnemonic);
            else ImGui::TextColored(s.status == 2 ? tint(0.35f, 0.94f, 0.54f) : tint(0.86f, 0.90f, 0.95f), "%u %s", static_cast<unsigned>(s.station), s.mnemonic);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("挂点 %u: %s（%s）", static_cast<unsigned>(s.station), s.mnemonic, status);
        }
    }
    ImGui::End();
}

void MissionGui::drawRadar(const ipc::MissionRadar& r) const {
    const float k = uiScale();
    const ipc::MissionPayload* p = r.payload < record_->payloadCount ? &record_->payloads[r.payload] : nullptr;
    const Column place = column(r.payload, valid_ && record_->payloadCount > 0);
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(place.right, place.radarTop), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(place.width, 0.0f),
                                        ImVec2(place.width, std::max(240.0f * k, display.y - place.radarTop - 12.0f * k)));
    char title[128];
    std::snprintf(title, sizeof title, "机载雷达 %s - %s###radar%u",
                  p ? p->designation : "", name_.empty() ? "?" : name_.c_str(), static_cast<unsigned>(r.payload));
    if (!beginMandatoryPanel(title, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    const bool operating = p && p->state == 3;
    if (r.searching) {
        ImGui::TextColored(tint(0.35f, 0.94f, 0.54f), "%s", modeName(r.mode));
        ImGui::SameLine();
        ImGui::Text("%s  ·  量程 %.0f km", modeLong(r.mode), static_cast<double>(r.rangeScaleM) / 1e3);
        if (r.task == 1) {
            ImGui::SameLine();
            ImGui::TextColored(cautionText(), r.acquiring ? "截获确认" : "截获搜索");
        } else if (r.task == 2 && r.trackCount > 0 && (r.tracks[0].flags & ipc::kMissionTrackDesignated)) {
            ImGui::SameLine();
            ImGui::TextColored(cautionText(), r.illuminating ? "航迹 %u · 照射" : "航迹 %u", static_cast<unsigned>(r.tracks[0].id));
        }
        if (!r.rotodome) {
            ImGui::Text("方位 %+.0f° ±%.0f°  ·  %u 行  ·  俯仰 %+.1f°",
                        r.azimuthCentreRad / kDeg, r.azimuthHalfWidthRad / kDeg,
                        static_cast<unsigned>(r.bars), r.elevationCentreRad / kDeg);
        }
        const bool dwelling = r.framePeriodS < 0.2f;
        ImGui::TextDisabled(dwelling ? "驻留 %.0f ms  ·  %.2f GHz  ·  %s%s" : "周期 %.1f s  ·  %.2f GHz  ·  %s%s",
                            static_cast<double>(r.framePeriodS) * (dwelling ? 1e3 : 1.0), r.frequencyHz / 1e9,
                            r.rotodome ? "旋转雷达罩" : r.electronic ? "相控阵" : "机械扫描",
                            r.signalLevel ? " · 信号级" : "");
    } else {
        ImGui::TextColored(stateColour(p ? p->state : 0), "%s", operating ? "待机（未搜索）" : p ? stateName(p->state) : "?");
        ImGui::TextDisabled(operating ? "等待雷达扫描指令" : "雷达未开机");
    }

    ImVec2 avail = ImGui::GetContentRegionAvail();
    const float side = std::max(100.0f * k, std::min(avail.x, place.width * 0.78f));
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, side);
    ImGui::InvisibleButton("scope", size);
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(8, 17, 22, 252), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 140), 4.0f * k, 0, 1.0f);
    const ImU32 grid = colour(36, 78, 74, 210), labelCol = colour(116, 176, 158, 235);
    const double scale = r.rangeScaleM > 0.0f ? static_cast<double>(r.rangeScaleM) : 150e3;
    const double fade = std::max(3.0, 2.0 * static_cast<double>(r.framePeriodS));
    const ipc::MissionDetection* tip = nullptr;
    float tipDistance = 8.0f * k;

    auto mark = [&](float x, float y, const ipc::MissionDetection& d) {
        const double age = simTime_ - d.timeS;
        if (age > fade || age < -1.0) return;
        const float a = static_cast<float>(std::clamp(1.0 - age / fade, 0.15, 1.0));
        const ImU32 c = d.rangeRateMs < 0.0f ? colour(96, 245, 146, static_cast<int>(255 * a))
                                             : colour(255, 206, 84, static_cast<int>(255 * a));
        const float h = 3.2f * k;
        dl->AddRectFilled(ImVec2(x - h, y - h), ImVec2(x + h, y + h), c);
        if (hovered) {
            const float dist = std::hypot(mouse.x - x, mouse.y - y);
            if (dist < tipDistance) tipDistance = dist, tip = &d;
        }
    };

    const ipc::MissionTrack* tipTrack = nullptr;
    float tipTrackDistance = 10.0f * k;
    auto trackMark = [&](float x, float y, const ipc::MissionTrack& t) {
        const ImU32 c = t.sinceUpdateS > 2.0f * static_cast<float>(fade) ? colour(146, 186, 202, 210) : colour(112, 224, 255, 255);
        const float h = 5.0f * k, len = (8.0f + 22.0f * std::min(t.speedMs / 300.0f, 2.0f)) * k;
        dl->AddRect(ImVec2(x - h, y - h), ImVec2(x + h, y + h), c, 0.0f, 0, 1.5f * k);
        dl->AddLine(ImVec2(x, y), ImVec2(x + len * std::sin(t.headingRad), y - len * std::cos(t.headingRad)), c, 1.5f * k);
        if (t.flags & ipc::kMissionTrackDesignated) {
            const float d = 2.0f * h;
            const ImVec2 diamond[4] = {ImVec2(x, y - d), ImVec2(x + d, y), ImVec2(x, y + d), ImVec2(x - d, y)};
            dl->AddPolyline(diamond, 4, colour(255, 210, 72, 255), ImDrawFlags_Closed, 1.8f * k);
        }
        char id[16];
        std::snprintf(id, sizeof id, "%u", static_cast<unsigned>(t.id));
        dl->AddText(ImVec2(x + h + 3.0f * k, y - h - ImGui::GetTextLineHeight() * 0.45f), c, id);
        if (hovered) {
            const float dist = std::hypot(mouse.x - x, mouse.y - y);
            if (dist < tipTrackDistance) tipTrackDistance = dist, tipTrack = &t;
        }
    };

    char text[32];
    if (!r.rotodome) {
        const double lim = std::max(1e-3, static_cast<double>(r.azimuthLimitRad));
        auto x = [&](double az) { return p0.x + static_cast<float>((az / lim + 1.0) * 0.5) * size.x; };
        auto y = [&](double range) { return p1.y - static_cast<float>(std::clamp(range / scale, 0.0, 1.0)) * size.y; };
        if (r.searching) {
            const double lo = r.azimuthCentreRad - r.azimuthHalfWidthRad, hi = r.azimuthCentreRad + r.azimuthHalfWidthRad;
            dl->AddRectFilled(ImVec2(x(std::max(lo, -lim)), p0.y), ImVec2(x(std::min(hi, lim)), p1.y), colour(18, 52, 58, 155));
        }
        for (int q = 1; q < 4; ++q) {
            const float yy = y(scale * q / 4.0);
            dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
            std::snprintf(text, sizeof text, "%.0f km", scale * q / 4.0 / 1e3);
            dl->AddText(ImVec2(p0.x + 4.0f * k, yy - ImGui::GetTextLineHeight()), labelCol, text);
        }
        for (int deg = -60; deg <= 60; deg += 30) {
            if (std::fabs(deg * kDeg) > lim) continue;
            dl->AddLine(ImVec2(x(deg * kDeg), p0.y), ImVec2(x(deg * kDeg), p1.y), grid);
        }
        if (r.searching) {
            const float bx = x(r.beamAzimuthRad);
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y), colour(104, 240, 186, 200), 2.0f * k);
        }
        for (std::uint32_t i = 0; i < r.recentCount && i < ipc::kMissionDetections; ++i)
            mark(x(r.recent[i].azimuthRad), y(r.recent[i].rangeM), r.recent[i]);
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i)
            if (r.tracks[i].rangeM <= scale) trackMark(x(r.tracks[i].azimuthRad), y(r.tracks[i].rangeM), r.tracks[i]);
    } else {
        const ImVec2 c(p0.x + size.x * 0.5f, p0.y + size.y * 0.5f);
        const float radius = size.y * 0.46f;
        auto at = [&](double az, double range) {
            const float f = static_cast<float>(std::clamp(range / scale, 0.0, 1.0)) * radius;
            return ImVec2(c.x + f * static_cast<float>(std::sin(az)), c.y - f * static_cast<float>(std::cos(az)));
        };
        for (int q = 1; q <= 4; ++q) {
            dl->AddCircle(c, radius * static_cast<float>(q) / 4.0f, grid, 64);
            std::snprintf(text, sizeof text, "%.0f", scale * q / 4.0 / 1e3);
            dl->AddText(ImVec2(c.x + 4.0f * k, c.y - radius * static_cast<float>(q) / 4.0f), labelCol, text);
        }
        for (int deg = 0; deg < 360; deg += 30) dl->AddLine(c, at(deg * kDeg, scale), grid);
        if (r.searching) dl->AddLine(c, at(r.beamAzimuthRad, scale), colour(104, 240, 186, 210), 2.0f * k);
        dl->AddTriangleFilled(ImVec2(c.x, c.y - 6.0f * k), ImVec2(c.x - 4.0f * k, c.y + 4.0f * k), ImVec2(c.x + 4.0f * k, c.y + 4.0f * k),
                              colour(230, 238, 245, 255));
        for (std::uint32_t i = 0; i < r.recentCount && i < ipc::kMissionDetections; ++i) {
            const ImVec2 q = at(r.recent[i].azimuthRad, r.recent[i].rangeM);
            mark(q.x, q.y, r.recent[i]);
        }
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i)
            if (r.tracks[i].rangeM <= scale) {
                const ImVec2 q = at(r.tracks[i].azimuthRad, r.tracks[i].rangeM);
                trackMark(q.x, q.y, r.tracks[i]);
            }
    }
    if (tipTrack)
        ImGui::SetTooltip("航迹 %u%s（更新 %u 次，%.1f s 前）\n距离 %.1f km · 方位 %+.1f° · 俯仰 %+.1f°\n接近率 %+.0f m/s · 速度 %.0f m/s\n相对航向 %+.0f° · 误差 %.0f m",
                          static_cast<unsigned>(tipTrack->id), (tipTrack->flags & ipc::kMissionTrackDesignated) ? " [锁定]" : "",
                          static_cast<unsigned>(tipTrack->updates), static_cast<double>(tipTrack->sinceUpdateS),
                          static_cast<double>(tipTrack->rangeM) / 1e3, tipTrack->azimuthRad / kDeg, tipTrack->elevationRad / kDeg,
                          static_cast<double>(tipTrack->rangeRateMs), static_cast<double>(tipTrack->speedMs), tipTrack->headingRad / kDeg,
                          static_cast<double>(tipTrack->sigmaM));
    else if (tip)
        ImGui::SetTooltip("距离 %.1f km · 方位 %+.1f° · 俯仰 %+.1f°\n接近率 %+.0f m/s · 信噪比 %.1f dB（%.1f s 前）",
                          static_cast<double>(tip->rangeM) / 1e3, tip->azimuthRad / kDeg, tip->elevationRad / kDeg,
                          static_cast<double>(tip->rangeRateMs), static_cast<double>(tip->snrDb), simTime_ - tip->timeS);
    ImGui::TextDisabled("航迹 %u（暂态 %u） · 点迹 %llu · 虚警 %llu", static_cast<unsigned>(r.trackCount),
                        static_cast<unsigned>(r.tentativeTracks), static_cast<unsigned long long>(r.detections),
                        static_cast<unsigned long long>(r.falseAlarms));
    ImGui::TextDisabled("方框: 航迹与航向线   方块: 点迹（绿=接近）");

    // Expandable native radar physics & signal processing technical details (live mirror)
    if (ImGui::TreeNode("底层仿真技术细节")) {
        const double freqGhz = r.frequencyHz * 1e-9;
        const double lambdaMm = r.frequencyHz > 1e6 ? (299792458.0 / r.frequencyHz) * 1e3 : 0.0;
        const ipc::TacticalRadarVolumeWireV1* matchedTact = nullptr;
        for (const auto& tr : replayRadars_) {
            if (tr.payloadIndex == r.payload) { matchedTact = &tr; break; }
        }
        if (ImGui::BeginTable("radar_tech_live", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("参数", ImGuiTableColumnFlags_WidthStretch, 0.95f);
            ImGui::TableSetupColumn("数值", ImGuiTableColumnFlags_WidthStretch, 1.35f);
            auto row = [](const char* key, const char* fmt, auto... args) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("%s", key);
                ImGui::TableSetColumnIndex(1); ImGui::Text(fmt, args...);
            };
            row("天线体制", "%s", r.rotodome ? "360° 旋转雷达罩" : r.electronic ? "AESA 有源相控阵" : "机械扫描平板缝隙");
            row("载频 / 波长", "%.2f GHz  (λ = %.1f mm)", freqGhz, lambdaMm);
            row("波束宽度", "方位 %.2f° × 俯仰 %.2f°", r.beamwidthAzimuthRad / kDeg, r.beamwidthElevationRad / kDeg);
            row("视场极限", "方位 ±%.0f° · 俯仰 ±%.0f°", r.azimuthLimitRad / kDeg, r.elevationLimitRad / kDeg);
            row("仪表最大量程", "%.0f km（当前 %.0f km）", r.instrumentedRangeM * 1e-3, scale * 1e-3);
            row("瞬时波束指向", "方位 %+.1f° · 俯仰 %+.1f°", r.beamAzimuthRad / kDeg, r.beamElevationRad / kDeg);
            row("处理层级", "%s", r.signalLevel ? "信号级（距离-多普勒 + CFAR）" : "检测级（Swerling 雷达方程）");
            row("累计驻留 / 检出", "%llu 次 · %llu 点迹 / %llu 虚警",
                static_cast<unsigned long long>(r.dwells),
                static_cast<unsigned long long>(r.detections),
                static_cast<unsigned long long>(r.falseAlarms));
            if (r.signalLevel && record_->rangeDoppler.radar < record_->radarCount &&
                record_->radars[record_->rangeDoppler.radar].payload == r.payload) {
                const auto& rd = record_->rangeDoppler;
                row("信号级 PRF", "%.2f kHz", rd.prfHz * 1e-3);
                row("不模糊距离/速度", "%.1f km · %.0f m/s", rd.ambiguousRangeM * 1e-3, rd.ambiguousRateMs);
                if (rd.notchMs > 0.0f) row("杂波凹口宽度", "±%.0f m/s（峰值 %u）", rd.notchMs, static_cast<unsigned>(rd.peakCount));
            }
            if (matchedTact) {
                const double r0Km = static_cast<double>(matchedTact->detectionRangeM) * 1e-3;
                const double btKm = static_cast<double>(matchedTact->burnThroughRangeM) * 1e-3;
                const double jDb = static_cast<double>(matchedTact->jammerRatioDb);
                const double lin = std::pow(10.0, jDb / 10.0);
                const double keepPct = r0Km > 1e-3 ? std::clamp(btKm / r0Km * 100.0, 0.0, 100.0) : 100.0;
                row("标称探测 R0", "%.1f km", r0Km);
                if ((matchedTact->flags & ipc::kTactRadarJammed) != 0u) {
                    row("干扰压制比", "+%.1f dB（1+J0/N0 ≈ %.1e）", jDb, lin);
                    row("烧穿距离 Rbt", "%.1f km（保留 %.1f%%）", btKm, keepPct);
                }
            }
            if (r.trackCount > 0) {
                const auto& t0 = r.tracks[0];
                row("主航迹滤波误差", "航迹 #%u · σ = %.0f m · 更新 %u 次",
                    static_cast<unsigned>(t0.id), static_cast<double>(t0.sigmaM), static_cast<unsigned>(t0.updates));
            }
            ImGui::EndTable();
        }
        ImGui::TreePop();
    }
    ImGui::End();
}

void MissionGui::drawReplayRadar(const ipc::TacticalRadarVolumeWireV1& r, std::uint32_t index) const {
    const float k = uiScale();
    const Column place = column(index, false);
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(place.right, place.radarTop), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(place.width, 0.0f),
                                        ImVec2(place.width, std::max(260.0f * k, display.y - place.radarTop - 12.0f * k)));
    char title[128];
    std::snprintf(title, sizeof title, "机载雷达 - %s###radar%u",
                  name_.empty() ? "?" : name_.c_str(), static_cast<unsigned>(index));
    if (!beginMandatoryPanel(title, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    const bool searching = (r.flags & ipc::kTactRadarSearching) != 0u;
    const bool roto = (r.flags & ipc::kTactRadarRotodome) != 0u;
    const bool elec = (r.flags & ipc::kTactRadarElectronic) != 0u;
    const bool stt = (r.flags & ipc::kTactRadarSttLock) != 0u;
    const bool illum = (r.flags & ipc::kTactRadarIlluminating) != 0u;
    const bool jammed = (r.flags & ipc::kTactRadarJammed) != 0u;
    const bool hasTarget = (r.flags & ipc::kTactRadarHasTarget) != 0u;

    const double scale = r.rangeScaleM > 1000.0f
                             ? static_cast<double>(r.rangeScaleM)
                             : std::max(40000.0, static_cast<double>(r.detectionRangeM));

    if (searching) {
        ImGui::TextColored(stt || illum ? cautionText() : tint(0.35f, 0.94f, 0.54f), "%s", modeName(r.mode));
        ImGui::SameLine();
        ImGui::Text("%s  ·  量程 %.0f km", modeLong(r.mode), scale * 1e-3);
        if (illum) {
            ImGui::SameLine();
            ImGui::TextColored(warningText(), "照射引导");
        }
        if (!roto) {
            ImGui::TextDisabled("扫描方位 %+.0f° ±%.0f°  ·  %s",
                                static_cast<double>(r.azimuthCentreRad) / kDeg,
                                static_cast<double>(r.azimuthHalfWidthRad) / kDeg,
                                elec ? "相控阵" : "机械扫描");
        } else {
            ImGui::TextDisabled("360° 旋转雷达罩  ·  最大探测 %.0f km", static_cast<double>(r.detectionRangeM) * 1e-3);
        }
    } else {
        ImGui::TextColored(stateColour(2), "待机（未搜索）");
    }

    if (jammed && r.burnThroughRangeM > 100.0f) {
        ImGui::TextColored(cautionText(), "干扰压制 · 烧穿 %.1f km (+%.0f dB)",
                           static_cast<double>(r.burnThroughRangeM) * 1e-3,
                           static_cast<double>(r.jammerRatioDb));
    }

    ImVec2 avail = ImGui::GetContentRegionAvail();
    const float side = std::max(100.0f * k, std::min(avail.x, place.width * 0.78f));
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, side);
    ImGui::InvisibleButton("replay_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);

    dl->AddRectFilled(p0, p1, colour(8, 17, 22, 252), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 150), 4.0f * k, 0, 1.0f);
    const ImU32 grid = colour(36, 78, 74, 210);
    const ImU32 labelCol = colour(128, 184, 168, 240);

    // Compute relative target azimuth, elevation, and slant range from native targetEcef + ownship pose.
    bool tgtValid = false;
    double tgtAz = 0.0, tgtEl = 0.0, tgtRange = 0.0;
    if (hasTarget && hasOwnState_) {
        const double dx = r.targetEcef[0] - ownState_.positionEcef[0];
        const double dy = r.targetEcef[1] - ownState_.positionEcef[1];
        const double dz = r.targetEcef[2] - ownState_.positionEcef[2];
        const double* R = ownState_.rotationBodyToEcef;
        const double fwd = R[0] * dx + R[3] * dy + R[6] * dz;
        const double right = R[1] * dx + R[4] * dy + R[7] * dz;
        const double down = R[2] * dx + R[5] * dy + R[8] * dz;
        tgtRange = r.targetRangeM > 1.0f ? static_cast<double>(r.targetRangeM) : std::sqrt(dx * dx + dy * dy + dz * dz);
        if (tgtRange > 1.0) {
            tgtAz = std::atan2(right, fwd);
            tgtEl = std::atan2(-down, std::hypot(fwd, right));
            tgtValid = true;
        }
    }

    char text[48];
    if (!roto) {
        const double lim = std::max(60.0 * kDeg, std::abs(static_cast<double>(r.azimuthCentreRad)) +
                                                 static_cast<double>(r.azimuthHalfWidthRad) + 4.0 * kDeg);
        auto xOf = [&](double az) { return p0.x + static_cast<float>((std::clamp(az / lim, -1.0, 1.0) + 1.0) * 0.5) * size.x; };
        auto yOf = [&](double rng) { return p1.y - static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * size.y; };

        if (searching) {
            const double lo = static_cast<double>(r.azimuthCentreRad) - static_cast<double>(r.azimuthHalfWidthRad);
            const double hi = static_cast<double>(r.azimuthCentreRad) + static_cast<double>(r.azimuthHalfWidthRad);
            const float x0 = xOf(std::max(lo, -lim)), x1 = xOf(std::min(hi, lim));
            if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
                const float yBt = yOf(static_cast<double>(r.burnThroughRangeM));
                // Clean active burn-through pocket below + subtle jammed region above
                dl->AddRectFilled(ImVec2(x0, yBt), ImVec2(x1, p1.y), colour(18, 56, 66, 175));
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, yBt), colour(64, 22, 54, 110));
            } else {
                dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(x1, p1.y), colour(18, 52, 62, 160));
            }
        }
        // Range lines
        for (int q = 1; q < 4; ++q) {
            const float yy = yOf(scale * q / 4.0);
            dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
            std::snprintf(text, sizeof text, "%.0f km", scale * q / 4.0 * 1e-3);
            dl->AddText(ImVec2(p0.x + 4.0f * k, yy - ImGui::GetTextLineHeight()), labelCol, text);
        }
        // Azimuth lines
        for (int deg = -60; deg <= 60; deg += 30) {
            if (std::fabs(deg * kDeg) > lim + 1e-3) continue;
            const float xx = xOf(deg * kDeg);
            dl->AddLine(ImVec2(xx, p0.y), ImVec2(xx, p1.y), grid);
        }
        // Gold Burn-Through Range Limit Line
        if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
            const float yBt = yOf(static_cast<double>(r.burnThroughRangeM));
            dl->AddLine(ImVec2(p0.x, yBt), ImVec2(p1.x, yBt), colour(255, 206, 52, 255), 2.2f * k);
            std::snprintf(text, sizeof text, "烧穿 %.1f km", static_cast<double>(r.burnThroughRangeM) * 1e-3);
            const ImVec2 tsz = ImGui::CalcTextSize(text);
            dl->AddText(ImVec2(p1.x - tsz.x - 6.0f * k, std::max(p0.y + 2.0f * k, yBt - tsz.y - 2.0f * k)),
                        colour(255, 218, 92, 255), text);
        }
        // Instantaneous sweeping beam
        if (searching) {
            const float bx = xOf(static_cast<double>(r.beamAzimuthRad));
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y), colour(102, 240, 186, 215), 2.0f * k);
        }
        // Designated / Tracked Target
        if (tgtValid && std::abs(tgtAz) <= lim && tgtRange <= scale) {
            const float tx = xOf(tgtAz), ty = yOf(tgtRange);
            const float h = 5.5f * k;
            const ImU32 tCol = (stt || illum) ? colour(255, 212, 74, 255) : colour(112, 224, 255, 255);
            dl->AddRect(ImVec2(tx - h, ty - h), ImVec2(tx + h, ty + h), tCol, 0.0f, 0, 1.8f * k);
            if (stt || illum) {
                const float d = 1.9f * h;
                const ImVec2 dia[4] = {{tx, ty - d}, {tx + d, ty}, {tx, ty + d}, {tx - d, ty}};
                dl->AddPolyline(dia, 4, colour(255, 212, 74, 255), ImDrawFlags_Closed, 1.8f * k);
            }
            std::snprintf(text, sizeof text, "%.1f km", tgtRange * 1e-3);
            dl->AddText(ImVec2(tx + h + 4.0f * k, ty - ImGui::GetTextLineHeight() * 0.5f), tCol, text);
        }
    } else {
        const ImVec2 c(p0.x + size.x * 0.5f, p0.y + size.y * 0.5f);
        const float radius = std::min(size.x, size.y) * 0.46f;
        auto at = [&](double az, double rng) {
            const float f = static_cast<float>(std::clamp(rng / scale, 0.0, 1.0)) * radius;
            return ImVec2(c.x + f * static_cast<float>(std::sin(az)), c.y - f * static_cast<float>(std::cos(az)));
        };
        for (int q = 1; q <= 4; ++q) {
            dl->AddCircle(c, radius * static_cast<float>(q) / 4.0f, grid, 64);
            std::snprintf(text, sizeof text, "%.0f", scale * q / 4.0 * 1e-3);
            dl->AddText(ImVec2(c.x + 4.0f * k, c.y - radius * static_cast<float>(q) / 4.0f), labelCol, text);
        }
        for (int deg = 0; deg < 360; deg += 30) dl->AddLine(c, at(deg * kDeg, scale), grid);
        if (jammed && r.burnThroughRangeM > 100.0f && r.burnThroughRangeM < scale) {
            const float btR = static_cast<float>(r.burnThroughRangeM / scale) * radius;
            dl->AddCircle(c, btR, colour(255, 206, 52, 255), 64, 2.2f * k);
        }
        if (searching) dl->AddLine(c, at(static_cast<double>(r.beamAzimuthRad), scale), colour(102, 240, 186, 220), 2.0f * k);
        dl->AddTriangleFilled(ImVec2(c.x, c.y - 6.0f * k), ImVec2(c.x - 4.0f * k, c.y + 4.0f * k), ImVec2(c.x + 4.0f * k, c.y + 4.0f * k),
                              colour(230, 238, 245, 255));
        if (tgtValid && tgtRange <= scale) {
            const ImVec2 q = at(tgtAz, tgtRange);
            const float h = 5.5f * k;
            const ImU32 tCol = (stt || illum) ? colour(255, 212, 74, 255) : colour(112, 224, 255, 255);
            dl->AddRect(ImVec2(q.x - h, q.y - h), ImVec2(q.x + h, q.y + h), tCol, 0.0f, 0, 1.8f * k);
        }
    }

    if (jammed && r.burnThroughRangeM > 100.0f) {
        ImGui::TextDisabled("标称探测 %.0f km  ·  当前烧穿 %.1f km",
                            static_cast<double>(r.detectionRangeM) * 1e-3,
                            static_cast<double>(r.burnThroughRangeM) * 1e-3);
    } else {
        ImGui::TextDisabled("最大探测 %.0f km  ·  扫描量程 %.0f km",
                            static_cast<double>(r.detectionRangeM) * 1e-3, scale * 1e-3);
    }

    // Expandable native radar simulation technical details (ECCM burn-through equation & beam geometry)
    if (ImGui::TreeNode("底层仿真技术细节")) {
        if (ImGui::BeginTable("radar_tech_replay", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("参数", ImGuiTableColumnFlags_WidthStretch, 0.78f);
            ImGui::TableSetupColumn("数值", ImGuiTableColumnFlags_WidthStretch, 1.52f);
            auto row = [](const char* key, const char* fmt, auto... args) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("%s", key);
                ImGui::TableSetColumnIndex(1); ImGui::Text(fmt, args...);
            };
            row("天线体制", "%s", roto ? "360° 旋转雷达罩" : elec ? "AESA 有源相控阵" : "机械扫描平板缝隙");
            row("工作模式", "%s（%s）%s", modeName(r.mode), modeLong(r.mode), illum ? " · 照射" : "");
            row("瞬时波束指向", "方位 %+.1f° · 俯仰 %+.1f°",
                static_cast<double>(r.beamAzimuthRad) / kDeg, static_cast<double>(r.beamElevationRad) / kDeg);
            if (!roto) {
                row("扫描空域", "方位 %+.0f° ±%.0f° · 俯仰 %+.1f° ±%.1f°",
                    static_cast<double>(r.azimuthCentreRad) / kDeg, static_cast<double>(r.azimuthHalfWidthRad) / kDeg,
                    static_cast<double>(r.elevationCentreRad) / kDeg, static_cast<double>(r.elevationHalfWidthRad) / kDeg);
            } else {
                row("扫描空域", "方位 360° · 俯仰 %+.1f° ±%.1f°",
                    static_cast<double>(r.elevationCentreRad) / kDeg, static_cast<double>(r.elevationHalfWidthRad) / kDeg);
            }
            const double r0Km = static_cast<double>(r.detectionRangeM) * 1e-3;
            const double btKm = static_cast<double>(r.burnThroughRangeM) * 1e-3;
            const double jDb = static_cast<double>(r.jammerRatioDb);
            const double lin = std::pow(10.0, jDb / 10.0);
            const double keepPct = r0Km > 1e-3 ? std::clamp(btKm / r0Km * 100.0, 0.0, 100.0) : 100.0;
            row("标称探测 R0", "%.1f km（量程 %.0f km）", r0Km, scale * 1e-3);
            if (jammed) {
                row("干扰噪声比", "+%.1f dB（1+J0/N0 ≈ %.1e）", jDb, lin);
                row("烧穿距离 Rbt", "%.1f km（保留 %.1f%%）", btKm, keepPct);
                row("烧穿物理式", "R0 · (1 + J0/N0)^(-1/4)");
            } else {
                row("电子对抗状态", "无明显压制（+%.1f dB）", jDb);
            }
            if (tgtValid) {
                row("当前跟踪目标", "航迹 #%u · 斜距 %.2f km", static_cast<unsigned>(r.targetId), tgtRange * 1e-3);
                row("目标体轴角度", "相对方位 %+.1f° · 相对俯仰 %+.1f°", tgtAz / kDeg, tgtEl / kDeg);
            }
            ImGui::EndTable();
        }
        ImGui::TreePop();
    }
    ImGui::End();
}

void MissionGui::drawReplayStandbyRadar() const {
    const float k = uiScale();
    const Column place = column(0u, false);
    ImGui::SetNextWindowPos(ImVec2(place.right, place.radarTop), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(place.width, place.radarHeight * 0.85f), ImGuiCond_FirstUseEver);
    char title[128];
    std::snprintf(title, sizeof title, "机载雷达 - %s###radar0", name_.empty() ? "?" : name_.c_str());
    if (!beginMandatoryPanel(title)) {
        ImGui::End();
        return;
    }
    ImGui::TextColored(stateColour(2), "待机 / 雷达静默");
    ImGui::TextDisabled("当前未发射雷达波束");

    ImVec2 avail = ImGui::GetContentRegionAvail();
    const float side = std::max(90.0f * k, std::min(avail.x, avail.y - 1.2f * ImGui::GetTextLineHeightWithSpacing()));
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, side);
    ImGui::InvisibleButton("standby_scope", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(8, 17, 22, 245), 4.0f * k);
    dl->AddRect(p0, p1, colour(58, 96, 108, 120), 4.0f * k, 0, 1.0f);
    const ImU32 grid = colour(30, 62, 62, 165);
    for (int q = 1; q < 4; ++q) {
        const float yy = p1.y - size.y * static_cast<float>(q) / 4.0f;
        dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
    }
    for (int q = 1; q < 4; ++q) {
        const float xx = p0.x + size.x * static_cast<float>(q) / 4.0f;
        dl->AddLine(ImVec2(xx, p0.y), ImVec2(xx, p1.y), grid);
    }
    const char* msg = "雷达静默 / 待机";
    const ImVec2 msz = ImGui::CalcTextSize(msg);
    dl->AddText(ImVec2(p0.x + (size.x - msz.x) * 0.5f, p0.y + (size.y - msz.y) * 0.5f), colour(120, 158, 164, 210), msg);
    ImGui::End();
}

void MissionGui::drawRangeDoppler(const ipc::MissionRangeDoppler& rd) const {
    const float k = uiScale();
    const ipc::MissionRadar& radar = record_->radars[rd.radar];
    const ipc::MissionPayload* p = radar.payload < record_->payloadCount ? &record_->payloads[radar.payload] : nullptr;
    const Column place = column(radar.payload, valid_ && record_->payloadCount > 0);
    ImGui::SetNextWindowPos(ImVec2(place.right, place.mapTop), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(place.width, place.mapHeight), ImGuiCond_FirstUseEver);
    char title[128];
    std::snprintf(title, sizeof title, "距离-多普勒 %s###rd", p ? p->designation : "雷达");
    if (!beginMandatoryPanel(title)) {
        ImGui::End();
        return;
    }
    const int rows = rd.rows, cols = rd.columns;
    ImGui::TextDisabled("PRF %.1f kHz  ·  模糊距离 %.1f km × %.0f m/s", static_cast<double>(rd.prfHz) / 1e3,
                        static_cast<double>(rd.ambiguousRangeM) / 1e3, static_cast<double>(rd.ambiguousRateMs));
    if (rows <= 0 || cols <= 0) {
        ImGui::End();
        return;
    }
    const float axisW = ImGui::CalcTextSize("00.0").x + 4.0f * k, axisH = ImGui::GetTextLineHeightWithSpacing();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y -= ImGui::GetTextLineHeightWithSpacing();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(60.0f * k, avail.x - axisW), std::max(60.0f * k, avail.y - axisH));
    ImGui::InvisibleButton("rdmap", ImVec2(size.x + axisW, size.y + axisH));
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0(origin.x + axisW, origin.y), p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(0, 0, 0, 255));
    const float cw = size.x / static_cast<float>(cols), ch = size.y / static_cast<float>(rows);
    auto heat = [&](int q) {
        const float t = std::clamp(static_cast<float>(q) / 160.0f, 0.0f, 1.0f);
        const float r = std::clamp(2.0f * t - 0.8f, 0.0f, 1.0f), g = std::clamp(1.6f * t, 0.0f, 1.0f), b = std::clamp(3.0f * t - 2.0f, 0.0f, 1.0f);
        return colour(static_cast<int>(255 * r), static_cast<int>(255 * g), static_cast<int>(255 * b), 255);
    };
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
            if (q < 12) continue;
            const float x = p0.x + static_cast<float>(c) * cw, y = p1.y - static_cast<float>(r + 1) * ch;
            dl->AddRectFilled(ImVec2(x, y), ImVec2(x + cw, y + ch), heat(q));
        }
    const int filters = cols * rd.columnBin;
    const double va = rd.ambiguousRateMs;
    auto xAt = [&](double closing) {
        double f = closing / va;
        f -= std::floor(f + 0.5);
        return p0.x + static_cast<float>((f + 0.5) * filters / rd.columnBin) * cw;
    };
    auto yAt = [&](double rangeM) { return p1.y - static_cast<float>((rangeM - rd.firstRangeM) / rd.rowM) * ch; };
    const ImU32 grid = colour(60, 70, 80, 255), labelCol = colour(150, 160, 170, 255), notch = colour(90, 140, 255, 220);
    if (std::isfinite(rd.mainBeamRateMs) && rd.notchMs > 0.0f) {
        const double centre = -static_cast<double>(rd.mainBeamRateMs);
        for (double edge : {centre - rd.notchMs, centre + rd.notchMs}) {
            const float x = xAt(edge);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), notch, 1.0f * k);
        }
    }
    if (std::isfinite(rd.altitudeM) && rd.notchMs > 0.0f) {
        const double folded = std::fmod(static_cast<double>(rd.altitudeM), static_cast<double>(rd.ambiguousRangeM));
        const float y = yAt(folded);
        dl->AddRect(ImVec2(xAt(-rd.notchMs), y - 2.5f * static_cast<float>(rd.rowBin) * ch),
                    ImVec2(xAt(rd.notchMs), y + 2.5f * static_cast<float>(rd.rowBin) * ch),
                    notch, 0.0f, 0, 1.0f * k);
    }
    for (std::uint32_t i = 0; i < rd.peakCount && i < ipc::kRangeDopplerPeaks; ++i) {
        const ImVec2 c(p0.x + (static_cast<float>(rd.peakColumn[i]) + 0.5f) * cw, p1.y - (static_cast<float>(rd.peakRow[i]) + 0.5f) * ch);
        dl->AddCircle(c, std::max(4.0f * k, 0.8f * std::max(cw, ch)), colour(255, 80, 200, 255), 16, 1.5f * k);
    }
    char text[32];
    const double top = rd.firstRangeM + rows * static_cast<double>(rd.rowM);
    const double step = top > 20e3 ? 5e3 : top > 8e3 ? 2e3 : 1e3;
    for (double rr = step; rr < top; rr += step) {
        const float y = yAt(rr);
        if (y < p0.y || y > p1.y) continue;
        dl->AddLine(ImVec2(p0.x - 3.0f * k, y), ImVec2(p0.x, y), grid);
        std::snprintf(text, sizeof text, "%.0f", rr / 1e3);
        dl->AddText(ImVec2(origin.x, y - ImGui::GetTextLineHeight() * 0.5f), labelCol, text);
    }
    const float zx = xAt(0.0);
    dl->AddLine(ImVec2(zx, p1.y), ImVec2(zx, p1.y + 4.0f * k), grid);
    for (int side = -1; side <= 1; side += 2) {
        std::snprintf(text, sizeof text, "%+.0f", side * va / 2.0);
        const float w = ImGui::CalcTextSize(text).x;
        dl->AddText(ImVec2(side < 0 ? p0.x : p1.x - w, p1.y + 2.0f), labelCol, text);
    }
    dl->AddText(ImVec2(zx - ImGui::CalcTextSize("0").x * 0.5f, p1.y + 2.0f), labelCol, "0");
    if (hovered && mouse.x >= p0.x && mouse.x < p1.x && mouse.y >= p0.y && mouse.y < p1.y) {
        const int c = std::clamp(static_cast<int>((mouse.x - p0.x) / cw), 0, cols - 1);
        const int r = std::clamp(static_cast<int>((p1.y - mouse.y) / ch), 0, rows - 1);
        const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
        const double nu = ((static_cast<double>(c) + 0.5) * rd.columnBin - filters / 2) / filters;
        ImGui::SetTooltip("距离 %.2f km (+ n × %.2f)\n接近率 %+.0f m/s (+ n × %.0f)\n高于噪底 %.1f dB",
                          (rd.firstRangeM + (r + 0.5) * static_cast<double>(rd.rowM)) / 1e3, static_cast<double>(rd.ambiguousRangeM) / 1e3,
                          nu * va, va, q * 0.25);
    }
    ImGui::TextDisabled("纵轴: 距离 (km)  横轴: 接近率 (m/s)  圆圈: 检测峰值  蓝线: 杂波凹口");
    ImGui::End();
}

} // namespace fsim::ui
