// Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame): the world's store of them, and a frame as it
// is at a time. Set and asked for between steps, never stepped.
#include "session/World.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::session {

control::FrameId World::createFrame(const control::FrameSpec& spec, std::uint32_t owner) {
    const bool finite = std::isfinite(spec.latitudeRad) && std::abs(spec.latitudeRad) <= 0.5 * 3.14159265358979323846 &&
                        std::isfinite(spec.longitudeRad) && std::isfinite(spec.altitudeMslM) && std::isfinite(spec.yawRad) &&
                        std::isfinite(spec.pitchRad) && std::isfinite(spec.rollRad) && std::isfinite(spec.northMs) && std::isfinite(spec.eastMs) &&
                        std::isfinite(spec.downMs) && std::isfinite(spec.timeS);
    if (!finite || spec.origin >= control::FrameOrigin::Count) return 0;
    if (spec.origin == control::FrameOrigin::Vehicle) {
        if (!vehicleState(spec.vehicle)) return 0; // (a vehicle, or a designation: its track's estimate - AB-2b)
        // a vehicle's own (an MA's): following itself or its own designations, never another aircraft's truth
        if (owner != 0 && spec.vehicle != owner) {
            const Designation* d = designation(spec.vehicle);
            if (!d || d->owner != owner) return 0;
        }
    }
    frames_[++lastFrame_] = KeptFrame{spec, owner};
    return lastFrame_;
}

bool World::seesFrame(control::FrameId id, std::uint32_t viewer) const {
    const auto it = frames_.find(id);
    if (it == frames_.end() || (it->second.owner != 0 && it->second.owner != viewer)) return false;
    // A vehicle an MA flies sees no frame following another aircraft - that aircraft's truth (L4) - but its own designations
    const control::FrameSpec& spec = it->second.spec;
    if (viewer != 0 && spec.origin == control::FrameOrigin::Vehicle && spec.vehicle != viewer &&
        std::any_of(agents_.begin(), agents_.end(), [&](const AgentRecord& a) { return a.vehicle == viewer; })) {
        const Designation* d = designation(spec.vehicle);
        return d && d->owner == viewer;
    }
    return true;
}

bool World::removeFrame(control::FrameId id, std::uint32_t owner) {
    const auto it = frames_.find(id);
    if (it == frames_.end() || it->second.owner != owner) return false;
    frames_.erase(it);
    return true;
}

std::optional<control::FrameSpec> World::frame(control::FrameId id, std::uint32_t viewer) const {
    if (!seesFrame(id, viewer)) return std::nullopt;
    return frames_.at(id).spec;
}

std::vector<control::FrameId> World::frames(std::uint32_t viewer) const {
    std::vector<control::FrameId> out;
    for (const auto& [id, f] : frames_)
        if (f.owner == 0 || f.owner == viewer) out.push_back(id);
    return out;
}

std::optional<control::FramePose> World::framePose(control::FrameId id, double timeS, std::uint32_t viewer) const {
    if (!seesFrame(id, viewer)) return std::nullopt;
    const double t = std::isnan(timeS) ? simTime_ : timeS;
    const control::FrameSpec& spec = frames_.at(id).spec;
    if (spec.origin != control::FrameOrigin::Vehicle) return control::framePose(spec, t);
    // a vehicle's: where it is now, as it is turned and moving, carried on to the time (a designation's: its estimate)
    const sim::VehicleState* s = vehicleState(spec.vehicle);
    if (!s) return std::nullopt;
    return control::carried(control::vehiclePose(*s), t - simTime_);
}

std::optional<control::GeoPoint> World::framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS,
                                                   std::uint32_t viewer) const {
    if (offset.rotation >= control::FrameRotation::Count || offset.offsets >= control::FrameOffsets::Count) return std::nullopt;
    const std::optional<control::FramePose> pose = framePose(id, timeS, viewer);
    if (!pose) return std::nullopt;
    return control::framePoint(*pose, offset);
}

bool World::Answers::frame(control::FrameId id, control::FrameSpec& spec, control::FramePose& now) const {
    if (id > control::kVehicleFrames && id - control::kVehicleFrames <= 0xFFFFFFFFu) { // a vehicle's own (4.42): where it is now
        const auto vehicle = static_cast<std::uint32_t>(id - control::kVehicleFrames);
        // (a vehicle's state now; a designation's, its track's estimate - ADR-31, AB-2b)
        const sim::VehicleState* s = world_.vehicleState(vehicle);
        if (!s) return false;
        spec = control::FrameSpec{}, spec.origin = control::FrameOrigin::Vehicle, spec.vehicle = vehicle;
        now = control::vehiclePose(*s);
        return true;
    }
    const std::optional<control::FrameSpec> found = world_.frame(id, vehicle_);
    const std::optional<control::FramePose> pose = found ? world_.framePose(id, std::numeric_limits<double>::quiet_NaN(), vehicle_) : std::nullopt;
    if (!pose) return false;
    spec = *found, now = *pose;
    return true;
}

} // namespace fsim::session
