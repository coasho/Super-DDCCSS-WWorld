// Operational geometry (docs/flight-autonomy.md, 4.42 to 4.45; A-GRA's OpPoint, OpZone, OpLine and OpVolume): the world's store of it,
// by id, for the commands that name it (a must fly). Set and asked for between steps, never stepped.
#include "session/World.h"

#include "simulation/flight_control/Lines.h"
#include "simulation/flight_control/Volumes.h"
#include "simulation/flight_control/Zones.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace fsim::session {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool code(double v, double count) noexcept { return control::isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return control::isHold(v) || std::isfinite(v); }

/// A datum as a viewer sees it (AB-2c): its own, else the common one (the range, viewer 0: the common).
template <typename T>
const T* resolve(const std::map<std::pair<std::uint32_t, std::uint64_t>, T>& kept, std::uint64_t id, std::uint32_t viewer) {
    if (viewer != 0)
        if (const auto own = kept.find({viewer, id}); own != kept.end()) return &own->second;
    const auto common = kept.find({0u, id});
    return common == kept.end() ? nullptr : &common->second;
}

/// The ids a viewer sees: its own and the common, each once.
template <typename T>
std::vector<std::uint64_t> seen(const std::map<std::pair<std::uint32_t, std::uint64_t>, T>& kept, std::uint32_t viewer) {
    std::vector<std::uint64_t> out;
    for (const auto& [key, datum] : kept)
        if (key.first == 0 || key.first == viewer) out.push_back(key.second);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

} // namespace

control::Reason World::setOpPoint(const control::OpPoint& point, std::uint32_t owner) {
    using control::isHold;
    const control::OpPoint& p = point;
    const bool place = !isHold(p.latitudeRad) || !isHold(p.longitudeRad), framed = !isHold(p.frame);
    const bool bad =
        p.id == 0 || place == framed || !finiteOr(p.latitudeRad) || !finiteOr(p.longitudeRad) || !finiteOr(p.altitudeM) || !finiteOr(p.frameXM) ||
        !finiteOr(p.frameYM) || !finiteOr(p.frameZM) || !finiteOr(p.ingressMinRad) || !finiteOr(p.ingressMaxRad) ||
        (place && (isHold(p.latitudeRad) || isHold(p.longitudeRad) || std::abs(p.latitudeRad) > 0.5 * kPi)) ||
        (framed && !(p.frame == std::floor(p.frame) && p.frame >= 1.0 && seesFrame(static_cast<control::FrameId>(p.frame), owner))) ||
        (!framed && (!isHold(p.frameRotation) || !isHold(p.frameOffsets) || !isHold(p.frameXM) || !isHold(p.frameYM) || !isHold(p.frameZM))) ||
        !code(p.frameRotation, static_cast<double>(control::FrameRotation::Count)) ||
        !code(p.frameOffsets, static_cast<double>(control::FrameOffsets::Count)) ||
        !code(p.altitudeReference, static_cast<double>(control::AltitudeReference::Count)) || (!isHold(p.altitudeReference) && isHold(p.altitudeM)) ||
        isHold(p.ingressMinRad) != isHold(p.ingressMaxRad) || (!isHold(p.ingressMinRad) && std::abs(p.ingressMinRad) > kPi) ||
        (!isHold(p.ingressMaxRad) && std::abs(p.ingressMaxRad) > kPi);
    if (bad) return control::Reason::InvalidParameter;
    control::OpPoint& kept = opPoints_[{owner, p.id}];
    const std::uint32_t revision = kept.revision + 1;
    kept = p;
    kept.revision = revision;
    return control::Reason::None;
}

bool World::removeOpPoint(control::OpPointId id, std::uint32_t owner) { return opPoints_.erase({owner, id}) > 0; }

std::vector<control::OpPointId> World::opPoints(std::uint32_t viewer) const { return seen(opPoints_, viewer); }

std::optional<control::OpPoint> World::opPoint(control::OpPointId id, std::uint32_t viewer) const {
    const control::OpPoint* found = resolve(opPoints_, id, viewer);
    return found ? std::optional<control::OpPoint>(*found) : std::nullopt;
}

bool World::Answers::opPoint(control::OpPointId id, control::OpPoint& out) const {
    const control::OpPoint* found = resolve(world_.opPoints_, id, vehicle_);
    if (!found) return false;
    out = *found;
    return true;
}

control::Reason World::setOpZone(const control::OpZone& zone, std::uint32_t owner) {
    const bool known = control::isHold(zone.frame) || (zone.frame == std::floor(zone.frame) && zone.frame >= 1.0 &&
                                                      seesFrame(static_cast<control::FrameId>(zone.frame), owner));
    if (zone.id == 0 || control::zones::fault(zone, known) >= 0) return control::Reason::InvalidParameter;
    control::OpZone& kept = opZones_[{owner, zone.id}];
    const std::uint32_t revision = kept.revision + 1;
    kept = zone;
    kept.revision = revision;
    if (!control::isHold(kept.northMs) && control::isHold(kept.timeS)) kept.timeS = simTime_; // (a moving zone's: from now)
    return control::Reason::None;
}

bool World::removeOpZone(control::OpZoneId id, std::uint32_t owner) { return opZones_.erase({owner, id}) > 0; }

std::vector<control::OpZoneId> World::opZones(std::uint32_t viewer) const { return seen(opZones_, viewer); }

std::optional<control::OpZone> World::opZone(control::OpZoneId id, std::uint32_t viewer) const {
    const control::OpZone* found = resolve(opZones_, id, viewer);
    return found ? std::optional<control::OpZone>(*found) : std::nullopt;
}

const control::OpZone* World::Answers::opZone(control::OpZoneId id) const { return resolve(world_.opZones_, id, vehicle_); }

control::Reason World::setOpLine(const control::OpLine& line, std::uint32_t owner) {
    const bool known = control::isHold(line.frame) || (line.frame == std::floor(line.frame) && line.frame >= 1.0 &&
                                                      seesFrame(static_cast<control::FrameId>(line.frame), owner));
    if (line.id == 0 || control::lines::fault(line, known) >= 0) return control::Reason::InvalidParameter;
    control::OpLine& kept = opLines_[{owner, line.id}];
    const std::uint32_t revision = kept.revision + 1;
    kept = line;
    kept.revision = revision;
    if (!control::isHold(kept.northMs) && control::isHold(kept.timeS)) kept.timeS = simTime_; // (a moving line's: from now)
    return control::Reason::None;
}

bool World::removeOpLine(control::OpLineId id, std::uint32_t owner) { return opLines_.erase({owner, id}) > 0; }

std::vector<control::OpLineId> World::opLines(std::uint32_t viewer) const { return seen(opLines_, viewer); }

std::optional<control::OpLine> World::opLine(control::OpLineId id, std::uint32_t viewer) const {
    const control::OpLine* found = resolve(opLines_, id, viewer);
    return found ? std::optional<control::OpLine>(*found) : std::nullopt;
}

const control::OpLine* World::Answers::opLine(control::OpLineId id) const { return resolve(world_.opLines_, id, vehicle_); }

control::Reason World::setOpVolume(const control::OpVolume& volume, std::uint32_t owner) {
    const bool known = control::isHold(volume.frame) || (volume.frame == std::floor(volume.frame) && volume.frame >= 1.0 &&
                                                        seesFrame(static_cast<control::FrameId>(volume.frame), owner));
    if (volume.id == 0 || control::volumes::fault(volume, known) >= 0) return control::Reason::InvalidParameter;
    control::OpVolume& kept = opVolumes_[{owner, volume.id}];
    const std::uint32_t revision = kept.revision + 1;
    kept = volume;
    kept.revision = revision;
    if (!control::isHold(kept.northMs) && control::isHold(kept.timeS)) kept.timeS = simTime_; // (a moving volume's: from now)
    return control::Reason::None;
}

bool World::removeOpVolume(control::OpVolumeId id, std::uint32_t owner) { return opVolumes_.erase({owner, id}) > 0; }

std::vector<control::OpVolumeId> World::opVolumes(std::uint32_t viewer) const { return seen(opVolumes_, viewer); }

std::optional<control::OpVolume> World::opVolume(control::OpVolumeId id, std::uint32_t viewer) const {
    const control::OpVolume* found = resolve(opVolumes_, id, viewer);
    return found ? std::optional<control::OpVolume>(*found) : std::nullopt;
}

const control::OpVolume* World::Answers::opVolume(control::OpVolumeId id) const { return resolve(world_.opVolumes_, id, vehicle_); }

} // namespace fsim::session
