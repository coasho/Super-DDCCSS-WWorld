#include "ui/MissionGui.h"

#include "fsim/MissionSystems.h"
#include "ipc/WorldMirror.h"

#include <vsgImGui/imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim::ui {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

/// Whether the ImGui renderer writes to an sRGB target, its style's colours made linear for it (the dark style's window
/// background, 0.06, is then some 0.005): colours given here are sRGB, as everyone picks them, and made linear likewise.
bool linearTarget() { return ImGui::GetStyle().Colors[ImGuiCol_WindowBg].x < 0.03f; }

float toLinear(float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); }

ImVec4 tint(float r, float g, float b, float a = 1.0f) {
    if (!linearTarget()) return ImVec4(r, g, b, a);
    return ImVec4(toLinear(r), toLinear(g), toLinear(b), a);
}

ImU32 colour(int r, int g, int b, int a = 255) {
    return ImGui::ColorConvertFloat4ToU32(tint(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f, static_cast<float>(b) / 255.0f,
                                               static_cast<float>(a) / 255.0f));
}

// (ms::PayloadState, ms::Family and ms::AmtiMode, as the record carries them)
const char* stateName(std::uint8_t s) {
    static const char* names[] = {"off", "starting", "standby", "operate", "bit", "failed"};
    return s < 6 ? names[s] : "?";
}

ImVec4 stateColour(std::uint8_t s) {
    switch (s) {
    case 1: return tint(1.0f, 0.85f, 0.3f);  // starting
    case 2: return tint(0.45f, 0.8f, 1.0f);  // standby
    case 3: return tint(0.4f, 1.0f, 0.45f);  // operate
    case 4: return tint(1.0f, 0.6f, 0.2f);   // bit
    case 5: return tint(1.0f, 0.3f, 0.3f);   // failed
    default: return tint(0.6f, 0.6f, 0.6f);  // off
    }
}

const char* familyName(std::uint8_t f) {
    static const char* names[] = {"air radar", "weather radar", "SAR", "SMTI", "ESM", "EO/IR", "jammer",
                                  "stores", "gun", "countermeasures", "comms", "navigation", "ground map radar"};
    static_assert(sizeof(names) / sizeof(names[0]) == static_cast<std::size_t>(fsim::ms::Family::Count), "a label for every family");
    return f < sizeof(names) / sizeof(names[0]) ? names[f] : "?";
}

const char* modeName(std::uint8_t m) {
    static const char* names[] = {"RWS", "VS", "TWS", "SURV"};
    return m < 4 ? names[m] : "?";
}

const char* modeLong(std::uint8_t m) {
    static const char* names[] = {"range while search", "velocity search", "track while scan", "surveillance"};
    return m < 4 ? names[m] : "?";
}

/// Where the radar windows first open: one column down the right edge, under the payloads window, out of the middle
/// where the followed aircraft is - the radar's display, and below it the room for its range-Doppler map (kept even
/// at the detection level: the map appears when the radar's search turns to the signal level, and must not open over
/// the display). Each radar's pair a little lower than the one before.
struct Column {
    float right, width;     ///< (anchored at its right edge)
    float radarTop, radarHeight;
    float mapTop, mapHeight;
};

Column column(std::uint32_t payload) {
    const float k = ImGui::GetIO().FontGlobalScale;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float gap = 8.0f * k;
    Column c;
    c.right = display.x - 12.0f * k;
    c.width = std::min(420.0f * k, display.x * 0.4f);
    c.radarTop = display.y * 0.35f + 30.0f * k * static_cast<float>(payload);
    const float room = std::max(160.0f * k, display.y - 12.0f * k - c.radarTop - gap); // (both windows, to the bottom edge)
    c.radarHeight = std::min(470.0f * k, 0.56f * room);
    c.mapTop = c.radarTop + c.radarHeight + gap;
    c.mapHeight = std::min(400.0f * k, room - c.radarHeight);
    return c;
}

} // namespace

MissionGui::MissionGui(std::shared_ptr<ViewerControls> controls)
    : controls_(std::move(controls)), record_(std::make_unique<ipc::MissionRecord>()) {}

void MissionGui::update(const ipc::WorldMirror* mirror, int slot, const std::string& name, double simTime) {
    mirrored_ = mirror && mirror->valid();
    simTime_ = simTime;
    name_ = name;
    if (!mirrored_ || slot < 0) {
        valid_ = false, slot_ = -1;
        return;
    }
    if (slot != slot_) valid_ = false, slot_ = slot; // (another vehicle: read it whole)
    if (mirror->mission(static_cast<std::uint32_t>(slot), *record_, valid_ ? record_->revision : ~0ull)) valid_ = true;
}

void MissionGui::record(vsg::CommandBuffer&) const {
    if (!mirrored_ || !controls_->showMission.load(std::memory_order_relaxed)) return;
    drawPayloads();
    if (valid_) {
        for (std::uint32_t i = 0; i < record_->radarCount && i < ipc::kMissionRadars; ++i) drawRadar(record_->radars[i]);
        if (record_->rangeDoppler.radar < record_->radarCount && record_->radars[record_->rangeDoppler.radar].signalLevel)
            drawRangeDoppler(record_->rangeDoppler);
    }
}

void MissionGui::drawPayloads() const {
    const float k = ImGui::GetIO().FontGlobalScale;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(display.x - 12.0f * k, 12.0f * k), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(std::min(470.0f * k, display.x * 0.45f), 0.0f), ImGuiCond_FirstUseEver);
    char title[128];
    std::snprintf(title, sizeof title, "mission systems - %s###mission", name_.empty() ? "?" : name_.c_str());
    if (!ImGui::Begin(title, nullptr)) {
        ImGui::End();
        return;
    }
    if (!valid_ || record_->vehicleId == 0) {
        ImGui::TextDisabled("no record for this vehicle yet");
        ImGui::End();
        return;
    }
    if (record_->payloadCount == 0) {
        ImGui::TextDisabled("no payloads: its aircraft has no fit");
        ImGui::End();
        return;
    }
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("payloads", 4, flags)) {
        ImGui::TableSetupColumn("payload", ImGuiTableColumnFlags_WidthStretch, 1.6f);
        ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("capabilities", ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableHeadersRow();
        for (std::uint32_t i = 0; i < record_->payloadCount && i < ipc::kMissionPayloads; ++i) {
            const ipc::MissionPayload& p = record_->payloads[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", p.designation);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s%s", p.name, p.count > 1 ? " (several)" : "");
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
            ImGui::TextWrapped("%s", p.capabilities[0] ? p.capabilities : "status only");
        }
        ImGui::EndTable();
    }
    // its stores (MS-7): station by station, the gone ones dimmed
    if (record_->storeCount > 0) {
        static constexpr const char* kStatus[] = {"loaded", "powering up", "ready", "releasing", "released", "jettisoned", "powering down"};
        ImGui::Text("stores");
        for (std::uint32_t i = 0; i < record_->storeCount && i < ipc::kMissionStores; ++i) {
            const ipc::MissionStore& s = record_->stores[i];
            const char* status = s.status < 7 ? kStatus[s.status] : "?";
            const bool gone = s.status == 4 || s.status == 5;
            if (i > 0) ImGui::SameLine(0.0f, 14.0f * ImGui::GetIO().FontGlobalScale);
            if (ImGui::GetContentRegionAvail().x < 120.0f * ImGui::GetIO().FontGlobalScale) ImGui::NewLine();
            if (gone) ImGui::TextDisabled("%u %s", static_cast<unsigned>(s.station), s.mnemonic);
            else ImGui::TextColored(s.status == 2 ? tint(0.4f, 1.0f, 0.45f) : tint(0.85f, 0.85f, 0.85f), "%u %s", static_cast<unsigned>(s.station), s.mnemonic);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("station %u: %s, %s", static_cast<unsigned>(s.station), s.mnemonic, status);
        }
    }
    ImGui::TextDisabled("as the SDK commands them; (MS-n): not modelled yet");
    ImGui::End();
}

void MissionGui::drawRadar(const ipc::MissionRadar& r) const {
    const float k = ImGui::GetIO().FontGlobalScale;
    const ipc::MissionPayload* p = r.payload < record_->payloadCount ? &record_->payloads[r.payload] : nullptr;
    const Column place = column(r.payload);
    ImGui::SetNextWindowPos(ImVec2(place.right, place.radarTop), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(place.width, place.radarHeight), ImGuiCond_FirstUseEver);
    char title[128];
    std::snprintf(title, sizeof title, "%s###radar%u", p ? p->designation : "radar", static_cast<unsigned>(r.payload));
    if (!ImGui::Begin(title, nullptr)) {
        ImGui::End();
        return;
    }
    const bool operating = p && p->state == 3;
    if (r.searching) {
        ImGui::TextColored(tint(0.4f, 1.0f, 0.45f), "%s", modeName(r.mode));
        ImGui::SameLine();
        ImGui::Text("%s  %.0f km", modeLong(r.mode), static_cast<double>(r.rangeScaleM) / 1e3);
        if (!r.rotodome) {
            ImGui::Text("az %+.0f +-%.0f deg  %u bar%s  el %+.1f deg", r.azimuthCentreRad / kDeg, r.azimuthHalfWidthRad / kDeg,
                        static_cast<unsigned>(r.bars), r.bars == 1 ? "" : "s", r.elevationCentreRad / kDeg);
        }
        ImGui::TextDisabled("frame %.1f s  %.2f GHz  %s%s", static_cast<double>(r.framePeriodS), r.frequencyHz / 1e9,
                            r.rotodome ? "rotodome" : r.electronic ? "AESA" : "mechanical", r.signalLevel ? "  signal level" : "");
    } else {
        ImGui::TextColored(stateColour(p ? p->state : 0), "%s", operating ? "not searching" : p ? stateName(p->state) : "?");
        ImGui::TextDisabled(operating ? "an AMTI command starts its search" : "the SDK powers it to operate, then commands a search");
    }

    ImGui::Checkbox("hits", &showHits_); // (raw detections, false alarms among them; off: the tracks alone)
    // the display
    ImVec2 avail = ImGui::GetContentRegionAvail();
    // (its height leaves room for the two lines under it - the counts and the legend - so the window needs no scrollbar;
    // a PPI centred across the whole width, its background too, not a square at the left with a gap beside it)
    const float side = std::max(80.0f * k, std::min(avail.x, avail.y - 2.0f * ImGui::GetTextLineHeightWithSpacing()));
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(avail.x, side);
    ImGui::InvisibleButton("scope", size);
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(6, 18, 10, 255));
    const ImU32 grid = colour(40, 90, 55, 255), label = colour(90, 160, 110, 255);
    const double scale = r.rangeScaleM > 0.0f ? static_cast<double>(r.rangeScaleM) : 150e3;
    const double fade = std::max(3.0, 2.0 * static_cast<double>(r.framePeriodS)); // (a detection shown for two frames)
    const ipc::MissionDetection* tip = nullptr;
    float tipDistance = 8.0f * k;

    auto mark = [&](float x, float y, const ipc::MissionDetection& d) {
        const double age = simTime_ - d.timeS;
        if (age > fade || age < -1.0) return;
        const float a = static_cast<float>(std::clamp(1.0 - age / fade, 0.15, 1.0));
        const ImU32 c = d.rangeRateMs < 0.0f ? colour(120, 255, 140, static_cast<int>(255 * a)) : colour(255, 210, 90, static_cast<int>(255 * a));
        const float h = 3.0f * k;
        dl->AddRectFilled(ImVec2(x - h, y - h), ImVec2(x + h, y + h), c);
        if (hovered) {
            const float dist = std::hypot(mouse.x - x, mouse.y - y);
            if (dist < tipDistance) tipDistance = dist, tip = &d;
        }
    };

    // a track (MS-5): an outlined box with its number, and a line along its course (up: the carrier's nose), its length
    // the speed
    const ipc::MissionTrack* tipTrack = nullptr;
    float tipTrackDistance = 10.0f * k;
    auto trackMark = [&](float x, float y, const ipc::MissionTrack& t) {
        const ImU32 c = t.sinceUpdateS > 2.0f * static_cast<float>(fade) ? colour(150, 190, 200, 200) : colour(140, 230, 255, 255);
        const float h = 5.0f * k, len = (8.0f + 22.0f * std::min(t.speedMs / 300.0f, 2.0f)) * k;
        dl->AddRect(ImVec2(x - h, y - h), ImVec2(x + h, y + h), c, 0.0f, 0, 1.5f * k);
        dl->AddLine(ImVec2(x, y), ImVec2(x + len * std::sin(t.headingRad), y - len * std::cos(t.headingRad)), c, 1.5f * k);
        char id[16];
        std::snprintf(id, sizeof id, "%u", static_cast<unsigned>(t.id));
        dl->AddText(ImVec2(x + h + 2.0f * k, y - h - ImGui::GetTextLineHeight() * 0.5f), c, id);
        if (hovered) {
            const float dist = std::hypot(mouse.x - x, mouse.y - y);
            if (dist < tipTrackDistance) tipTrackDistance = dist, tipTrack = &t;
        }
    };

    char text[32];
    if (!r.rotodome) { // B-scope: azimuth across its field of regard, range up
        const double lim = std::max(1e-3, static_cast<double>(r.azimuthLimitRad));
        auto x = [&](double az) { return p0.x + static_cast<float>((az / lim + 1.0) * 0.5) * size.x; };
        auto y = [&](double range) { return p1.y - static_cast<float>(std::clamp(range / scale, 0.0, 1.0)) * size.y; };
        if (r.searching) { // the volume searched
            const double lo = r.azimuthCentreRad - r.azimuthHalfWidthRad, hi = r.azimuthCentreRad + r.azimuthHalfWidthRad;
            dl->AddRectFilled(ImVec2(x(std::max(lo, -lim)), p0.y), ImVec2(x(std::min(hi, lim)), p1.y), colour(20, 50, 30, 255));
        }
        for (int q = 1; q < 4; ++q) {
            const float yy = y(scale * q / 4.0);
            dl->AddLine(ImVec2(p0.x, yy), ImVec2(p1.x, yy), grid);
            std::snprintf(text, sizeof text, "%.0f", scale * q / 4.0 / 1e3);
            dl->AddText(ImVec2(p0.x + 3.0f * k, yy - ImGui::GetTextLineHeight()), label, text);
        }
        for (int deg = -60; deg <= 60; deg += 30) {
            if (std::fabs(deg * kDeg) > lim) continue;
            dl->AddLine(ImVec2(x(deg * kDeg), p0.y), ImVec2(x(deg * kDeg), p1.y), grid);
        }
        if (r.searching) {
            const float bx = x(r.beamAzimuthRad);
            dl->AddLine(ImVec2(bx, p0.y), ImVec2(bx, p1.y), colour(140, 255, 160, 160), 2.0f * k);
        }
        for (std::uint32_t i = 0; showHits_ && i < r.recentCount && i < ipc::kMissionDetections; ++i)
            mark(x(r.recent[i].azimuthRad), y(r.recent[i].rangeM), r.recent[i]);
        for (std::uint32_t i = 0; i < r.trackCount && i < ipc::kMissionTracks; ++i)
            if (r.tracks[i].rangeM <= scale) trackMark(x(r.tracks[i].azimuthRad), y(r.tracks[i].rangeM), r.tracks[i]);
    } else { // PPI, heading up
        const ImVec2 c(p0.x + size.x * 0.5f, p0.y + size.y * 0.5f);
        const float radius = size.y * 0.48f;
        auto at = [&](double az, double range) {
            const float f = static_cast<float>(std::clamp(range / scale, 0.0, 1.0)) * radius;
            return ImVec2(c.x + f * static_cast<float>(std::sin(az)), c.y - f * static_cast<float>(std::cos(az)));
        };
        for (int q = 1; q <= 4; ++q) {
            dl->AddCircle(c, radius * static_cast<float>(q) / 4.0f, grid, 64);
            std::snprintf(text, sizeof text, "%.0f", scale * q / 4.0 / 1e3);
            dl->AddText(ImVec2(c.x + 3.0f * k, c.y - radius * static_cast<float>(q) / 4.0f), label, text);
        }
        for (int deg = 0; deg < 360; deg += 30) dl->AddLine(c, at(deg * kDeg, scale), grid);
        if (r.searching) dl->AddLine(c, at(r.beamAzimuthRad, scale), colour(140, 255, 160, 200), 2.0f * k);
        dl->AddTriangleFilled(ImVec2(c.x, c.y - 6.0f * k), ImVec2(c.x - 4.0f * k, c.y + 4.0f * k), ImVec2(c.x + 4.0f * k, c.y + 4.0f * k),
                              colour(220, 220, 220, 255));
        for (std::uint32_t i = 0; showHits_ && i < r.recentCount && i < ipc::kMissionDetections; ++i) {
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
        ImGui::SetTooltip("track %u (%u updates, %.1f s ago)\nrange %.1f km  az %+.1f deg  el %+.1f deg\nrange rate %+.0f m/s  speed %.0f m/s\n"
                          "course %+.0f deg off the nose\nuncertainty %.0f m",
                          static_cast<unsigned>(tipTrack->id), static_cast<unsigned>(tipTrack->updates), static_cast<double>(tipTrack->sinceUpdateS),
                          static_cast<double>(tipTrack->rangeM) / 1e3, tipTrack->azimuthRad / kDeg, tipTrack->elevationRad / kDeg,
                          static_cast<double>(tipTrack->rangeRateMs), static_cast<double>(tipTrack->speedMs), tipTrack->headingRad / kDeg,
                          static_cast<double>(tipTrack->sigmaM));
    else if (tip)
        ImGui::SetTooltip("range %.1f km\naz %+.1f deg  el %+.1f deg\nrange rate %+.0f m/s\nSNR %.1f dB\n%.1f s ago",
                          static_cast<double>(tip->rangeM) / 1e3, tip->azimuthRad / kDeg, tip->elevationRad / kDeg,
                          static_cast<double>(tip->rangeRateMs), static_cast<double>(tip->snrDb), simTime_ - tip->timeS);
    ImGui::TextDisabled("%u tracks (%u tentative)  %llu detections, %llu false alarms", static_cast<unsigned>(r.trackCount),
                        static_cast<unsigned>(r.tentativeTracks), static_cast<unsigned long long>(r.detections),
                        static_cast<unsigned long long>(r.falseAlarms));
    ImGui::TextDisabled("boxes: tracks (course line)  squares: hits, green closing");
    ImGui::End();
}

void MissionGui::drawRangeDoppler(const ipc::MissionRangeDoppler& rd) const {
    const float k = ImGui::GetIO().FontGlobalScale;
    const ipc::MissionRadar& radar = record_->radars[rd.radar];
    const ipc::MissionPayload* p = radar.payload < record_->payloadCount ? &record_->payloads[radar.payload] : nullptr;
    const Column place = column(radar.payload); // (under its radar's window)
    ImGui::SetNextWindowPos(ImVec2(place.right, place.mapTop), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(place.width, place.mapHeight), ImGuiCond_FirstUseEver);
    char title[128];
    std::snprintf(title, sizeof title, "%s range-Doppler###rd", p ? p->designation : "radar");
    if (!ImGui::Begin(title, nullptr)) {
        ImGui::End();
        return;
    }
    const int rows = rd.rows, cols = rd.columns;
    ImGui::TextDisabled("PRF %.1f kHz: folded to %.1f km x %.0f m/s", static_cast<double>(rd.prfHz) / 1e3,
                        static_cast<double>(rd.ambiguousRangeM) / 1e3, static_cast<double>(rd.ambiguousRateMs));
    if (rows <= 0 || cols <= 0) {
        ImGui::End();
        return;
    }
    const float axisW = ImGui::CalcTextSize("00.0").x + 4.0f * k, axisH = ImGui::GetTextLineHeightWithSpacing();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y -= ImGui::GetTextLineHeightWithSpacing(); // (the legend)
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(60.0f * k, avail.x - axisW), std::max(60.0f * k, avail.y - axisH));
    ImGui::InvisibleButton("rdmap", ImVec2(size.x + axisW, size.y + axisH));
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0(origin.x + axisW, origin.y), p1(p0.x + size.x, p0.y + size.y);
    dl->AddRectFilled(p0, p1, colour(0, 0, 0, 255));
    const float cw = size.x / static_cast<float>(cols), ch = size.y / static_cast<float>(rows);
    // a cell's power over the noise: black at the noise, through green and yellow to white at 40 dB
    auto heat = [&](int q) {
        const float t = std::clamp(static_cast<float>(q) / 160.0f, 0.0f, 1.0f);
        const float r = std::clamp(2.0f * t - 0.8f, 0.0f, 1.0f), g = std::clamp(1.6f * t, 0.0f, 1.0f), b = std::clamp(3.0f * t - 2.0f, 0.0f, 1.0f);
        return colour(static_cast<int>(255 * r), static_cast<int>(255 * g), static_cast<int>(255 * b), 255);
    };
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
            if (q < 12) continue; // (within 3 dB of the noise: black)
            const float x = p0.x + static_cast<float>(c) * cw, y = p1.y - static_cast<float>(r + 1) * ch;
            dl->AddRectFilled(ImVec2(x, y), ImVec2(x + cw, y + ch), heat(q));
        }
    // closing speed (m/s) at x, folded: a column's centre is (c + 1/2) col bins from -1/2 of the PRF
    const int filters = cols * rd.columnBin;
    const double va = rd.ambiguousRateMs;
    auto xAt = [&](double closing) { // (folded into [-va/2, va/2))
        double f = closing / va;
        f -= std::floor(f + 0.5);
        return p0.x + static_cast<float>((f + 0.5) * filters / rd.columnBin) * cw;
    };
    auto yAt = [&](double rangeM) { return p1.y - static_cast<float>((rangeM - rd.firstRangeM) / rd.rowM) * ch; };
    const ImU32 grid = colour(60, 70, 80, 255), label = colour(150, 160, 170, 255), notch = colour(90, 140, 255, 220);
    // the notches: the main beam's clutter (about its rate) and the altitude line's (zero rate, the altitude's range)
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
        dl->AddRect(ImVec2(xAt(-rd.notchMs), y - 2.5f * static_cast<float>(rd.rowBin) * ch), ImVec2(xAt(rd.notchMs), y + 2.5f * static_cast<float>(rd.rowBin) * ch),
                    notch, 0.0f, 0, 1.0f * k);
    }
    // the processor's peaks
    for (std::uint32_t i = 0; i < rd.peakCount && i < ipc::kRangeDopplerPeaks; ++i) {
        const ImVec2 c(p0.x + (static_cast<float>(rd.peakColumn[i]) + 0.5f) * cw, p1.y - (static_cast<float>(rd.peakRow[i]) + 0.5f) * ch);
        dl->AddCircle(c, std::max(4.0f * k, 0.8f * std::max(cw, ch)), colour(255, 80, 200, 255), 16, 1.5f * k);
    }
    // axes: folded range (km) up the left, closing speed across the bottom, zero Doppler marked
    char text[32];
    const double top = rd.firstRangeM + rows * static_cast<double>(rd.rowM);
    const double step = top > 20e3 ? 5e3 : top > 8e3 ? 2e3 : 1e3;
    for (double rr = step; rr < top; rr += step) {
        const float y = yAt(rr);
        if (y < p0.y || y > p1.y) continue;
        dl->AddLine(ImVec2(p0.x - 3.0f * k, y), ImVec2(p0.x, y), grid);
        std::snprintf(text, sizeof text, "%.0f", rr / 1e3);
        dl->AddText(ImVec2(origin.x, y - ImGui::GetTextLineHeight() * 0.5f), label, text);
    }
    const float zx = xAt(0.0);
    dl->AddLine(ImVec2(zx, p1.y), ImVec2(zx, p1.y + 4.0f * k), grid);
    for (int side = -1; side <= 1; side += 2) {
        std::snprintf(text, sizeof text, "%+.0f", side * va / 2.0);
        const float w = ImGui::CalcTextSize(text).x;
        dl->AddText(ImVec2(side < 0 ? p0.x : p1.x - w, p1.y + 2.0f), label, text);
    }
    dl->AddText(ImVec2(zx - ImGui::CalcTextSize("0").x * 0.5f, p1.y + 2.0f), label, "0");
    if (hovered && mouse.x >= p0.x && mouse.x < p1.x && mouse.y >= p0.y && mouse.y < p1.y) {
        const int c = std::clamp(static_cast<int>((mouse.x - p0.x) / cw), 0, cols - 1), r = std::clamp(static_cast<int>((p1.y - mouse.y) / ch), 0, rows - 1);
        const int q = rd.db[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(c)];
        const double nu = ((static_cast<double>(c) + 0.5) * rd.columnBin - filters / 2) / filters;
        ImGui::SetTooltip("range %.2f km (+ n x %.2f)\nclosing %+.0f m/s (+ n x %.0f)\n%.1f dB over the noise",
                          (rd.firstRangeM + (r + 0.5) * static_cast<double>(rd.rowM)) / 1e3, static_cast<double>(rd.ambiguousRangeM) / 1e3, nu * va, va,
                          q * 0.25);
    }
    ImGui::TextDisabled("km up, closing m/s across  circles: the processor's peaks  blue: notches");
    ImGui::End();
}

} // namespace fsim::ui
