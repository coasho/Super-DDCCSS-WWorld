#pragma once

// fsim SDK: the scenario document (design 9.13) - its values, and reading and writing it. fsim/Scenario.h includes this
// header and adds applying a scenario to a World and the VecEnv options it implies; include that one for the whole API.
// The document's values are the simulation's own: reading them needs neither the batch environment nor the World's
// execution (the "vecenv" section is plain values, mapped to VecEnvOptions by fsim::vecEnvOptions).
//
// Format (all keys optional unless noted; snake_case, SI units, degrees):
// {
//   "world": { "name", "dt", "frame_skip", "workers", "pin_workers", "seed", "capacity",
//              "publish", "publish_interval_s", "jsbsim_root", "terrain", "terrain_url",
//              "terrain_zoom", "record_path", "record_interval_s" },
//   "environment": { "utc": "2026-06-21T10:30:00Z" | "unix_seconds": N, "time_factor",
//                    "wind": { "direction_deg", "speed_ms", "gust_ms", "turbulence" },
//                    "atmosphere": { "temperature_sea_level_k", "pressure_sea_level_pa", "humidity" },
//                    "weather": { "visibility_m", "cloud_base_m", "cloud_cover", "precipitation" } },
//   "effects": [ { "id": "gaussian_sensor_noise", "position_sigma_m": 5 }, ... ],   // every vehicle
//   "vecenv": { "num_envs", "vehicles_per_env", "task", "observation", "action", "action_ranges", "max_episode_steps",
//               "jitter": { "lat_deg", "lon_deg", "alt_m", "heading_deg", "airspeed_ms" },
//               "target_altitude_delta_m", "target_heading_delta_deg" },          // batch layer (vecEnvOptions)
//   "vehicles": [ {
//       "name" (required), "type": "jsbsim:c172x", "model", "control_divider",
//       "count": 1, "spacing_m": 200,                 // count > 1: "<name>-1".. abreast, to the right of the heading
//       "initial": { "lat_deg", "lon_deg", "alt_msl_m", "heading_deg", "pitch_deg", "roll_deg", "airspeed_ms", "on_ground" },
//       "command": { "level": "actuator" | "attitude" | "acceleration" | "velocity" | "position" | "behavior", ... },
//       "effects": [ ... ]
//   } ]
// }
// Command fields by level (absent = the controller's default / hold):
//   actuator:     aileron, elevator, rudder, throttle, flaps, gear_down, brake_left, brake_right
//   attitude:     roll_deg, pitch_deg, heading_deg, max_bank_deg, throttle, airspeed_ms
//   acceleration: load_factor_g, roll_rate_deg_s, longitudinal_ms2, throttle
//   velocity:     airspeed_ms, vertical_speed_ms, heading_deg, turn_rate_deg_s
//   position:     lat_deg, lon_deg, alt_msl_m, airspeed_ms, capture_radius_m
//   behavior:     id (required), target (another vehicle's name), params { ... },
//                 points [ { lat_deg, lon_deg, alt_msl_m, airspeed_ms, capture_radius_m } ]

#include "fsim/Export.h"
#include "fsim/World.h"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim {

/// A built-in effect by id with named parameters (effects::createBuiltinEffect).
struct EffectSpec {
    std::string id;
    std::vector<std::pair<std::string, double>> params;
};

struct ScenarioVehicle {
    VehicleSpec spec;
    unsigned count = 1;          ///< instances; > 1 names them "<name>-1".."<name>-N"
    double spacingM = 200.0;     ///< lateral spacing between instances (positive: to the right of the heading)
    std::optional<control::Command> command; ///< issued after creation; its level becomes the active level
    std::string commandTarget;   ///< behaviour target by vehicle name (resolved when applied)
    std::vector<EffectSpec> effects;
};

/// The batch layer's section: how VecEnv samples episodes (see vecenv.md).
struct ScenarioVecEnv {
    bool present = false;
    unsigned numEnvs = 1, vehiclesPerEnv = 1;
    std::string task = "altitude_heading_hold", observation = "state", action = "surfaces";
    std::string actionRanges = "fixed"; ///< or "aircraft" (VecEnvOptions::actionRanges)
    unsigned maxEpisodeSteps = 2000;
    double latitudeJitterDeg = 0.02, longitudeJitterDeg = 0.02, altitudeJitterM = 150.0, headingJitterDeg = 180.0, airspeedJitterMs = 5.0;
    double targetAltitudeDeltaM = 300.0, targetHeadingDeltaDeg = 60.0;
};

struct Scenario {
    std::string source;          ///< file name, for messages
    std::filesystem::path path;  ///< the file it was loaded from (empty when parsed from memory)
    WorldOptions world;
    bool hasEnvironment = false;
    EnvironmentState environment; ///< applied when `hasEnvironment` (epochUtcSeconds 0 = leave the clock alone)
    std::vector<EffectSpec> effects; ///< for every vehicle, present and future
    std::vector<ScenarioVehicle> vehicles;
    ScenarioVecEnv vecenv;
};

/// Read and parse a scenario file. Throws fsim::Error with "<file>:<line>:<col>: ..." on a bad document.
FSIM_API Scenario loadScenario(const std::filesystem::path& path);
/// Parse a document held in memory.
FSIM_API Scenario parseScenario(std::string_view json, std::string_view source = "scenario");

/// The scenario back as a compact JSON document (a Scenario can be built or
/// edited in code and written out).
FSIM_API std::string dumpScenario(const Scenario& scenario);

} // namespace fsim
