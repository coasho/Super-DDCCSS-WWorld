// C ABI adapter for recording readback.
#include "fsim/Recording.h"
#include "fsim/fsim_c.h"
#include "CAbiSupport.h"
#include <cstddef>
#include <cstring>
#include <vector>

// The C state struct must mirror fsim::VehicleState exactly.
static_assert(sizeof(fsim_vehicle_state) == sizeof(fsim::sim::VehicleState), "fsim_vehicle_state layout differs from VehicleState");
static_assert(offsetof(fsim_vehicle_state, rotation_body_to_ecef) == offsetof(fsim::sim::VehicleState, rotationBodyToEcef), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_count) == offsetof(fsim::sim::VehicleState, engineCount), "layout");
static_assert(offsetof(fsim_vehicle_state, speedbrake_position) == offsetof(fsim::sim::VehicleState, speedbrakePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, throttle_position) == offsetof(fsim::sim::VehicleState, throttlePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, on_ground) == offsetof(fsim::sim::VehicleState, onGround), "layout");
static_assert(offsetof(fsim_vehicle_state, crashed) == offsetof(fsim::sim::VehicleState, crashed), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_rpm) == offsetof(fsim::sim::VehicleState, engineRpm), "layout");
static_assert(offsetof(fsim_vehicle_state, nozzle_position) == offsetof(fsim::sim::VehicleState, nozzlePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, leading_edge_flap_rad) == offsetof(fsim::sim::VehicleState, leadingEdgeFlapRad), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_count) == offsetof(fsim::sim::VehicleState, wheelCount), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_compression_m) == offsetof(fsim::sim::VehicleState, wheelCompressionM), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_speed_ms) == offsetof(fsim::sim::VehicleState, wheelSpeedMs), "layout");
static_assert(FSIM_MAX_WHEELS == fsim::sim::VehicleState::kMaxWheels, "layout");
static_assert(sizeof(bool) == 1, "bool must be one byte for the C mirror of VehicleState");
static_assert(sizeof(fsim_control_inputs) == sizeof(fsim::ControlInputs), "fsim_control_inputs layout differs from ControlInputs");
static_assert(offsetof(fsim_control_inputs, brake_right) == offsetof(fsim::ControlInputs, brakeRight), "layout");

// --- Recordings ---------------------------------------------------------------------------

struct fsim_recording {
    fsim::Recording recording;
    struct Frame {
        std::vector<fsim_recorded_sample> samples;
        std::vector<fsim_recorded_event> events;
    };
    std::vector<Frame> frames; ///< C views of the frames; strings point into `recording`
};

FSIM_API int fsim_recording_load(const char* path, fsim_recording** out) {
    if (!path || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_recording_load: bad arguments");
    *out = nullptr;
    return guard("fsim_recording_load", [&] {
        auto r = new fsim_recording{fsim::Recording::load(path), {}};
        r->frames.reserve(r->recording.frames().size());
        for (const auto& f : r->recording.frames()) {
            fsim_recording::Frame cf;
            for (const auto& s : f.samples) {
                fsim_recorded_sample cs;
                cs.slot = s.slot;
                std::memcpy(&cs.state, &s.state, sizeof cs.state);
                std::memcpy(&cs.inputs, &s.inputs, sizeof cs.inputs);
                cf.samples.push_back(cs);
            }
            for (const auto& e : f.events) {
                fsim_recorded_event ce;
                ce.slot = e.slot;
                ce.id = e.id;
                ce.generation = e.generation;
                ce.alive = e.alive ? 1 : 0;
                ce.control_level = e.controlLevel;
                ce.name = e.name.c_str();
                ce.type = e.type.c_str();
                ce.model = e.model.c_str();
                ce.initial_latitude_deg = e.initialLatitudeDeg;
                ce.initial_longitude_deg = e.initialLongitudeDeg;
                ce.initial_altitude_msl_m = e.initialAltitudeMslM;
                ce.initial_heading_deg = e.initialHeadingDeg;
                cf.events.push_back(ce);
            }
            r->frames.push_back(std::move(cf));
        }
        *out = r;
        return static_cast<int>(FSIM_OK);
    });
}

FSIM_API void fsim_recording_destroy(fsim_recording* recording) { delete recording; }
FSIM_API uint32_t fsim_recording_frame_count(const fsim_recording* r) { return r ? static_cast<uint32_t>(r->frames.size()) : 0u; }
FSIM_API double fsim_recording_frame_time(const fsim_recording* r, uint32_t frame) {
    return r && frame < r->frames.size() ? r->recording.frames()[frame].simTime : 0.0;
}
FSIM_API const fsim_recorded_sample* fsim_recording_samples(const fsim_recording* r, uint32_t frame, uint32_t* count) {
    if (!r || frame >= r->frames.size()) {
        if (count) *count = 0;
        return nullptr;
    }
    if (count) *count = static_cast<uint32_t>(r->frames[frame].samples.size());
    return r->frames[frame].samples.data();
}
FSIM_API const fsim_recorded_event* fsim_recording_events(const fsim_recording* r, uint32_t frame, uint32_t* count) {
    if (!r || frame >= r->frames.size()) {
        if (count) *count = 0;
        return nullptr;
    }
    if (count) *count = static_cast<uint32_t>(r->frames[frame].events.size());
    return r->frames[frame].events.data();
}
FSIM_API const char* fsim_recording_world_name(const fsim_recording* r) { return r ? r->recording.worldName().c_str() : ""; }
FSIM_API double fsim_recording_dt(const fsim_recording* r) { return r ? r->recording.dt() : 0.0; }
FSIM_API int32_t fsim_recording_frame_skip(const fsim_recording* r) { return r ? r->recording.frameSkip() : 0; }
