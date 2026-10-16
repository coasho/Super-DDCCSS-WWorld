// The VecEnv options a scenario implies (design 9.13; vecenv.md): the public document's "world" and "vecenv" sections
// and its first vehicle as the episode template, mapped to the batch environment's options. Training's mapping, not the
// runtime's (ARCH-02): the runtime applies its native recipe (session/ScenarioRecipe.h) and never reads these sections.

#include "fsim/Scenario.h"

namespace fsim {

VecEnvOptions vecEnvOptions(const Scenario& sc) {
    VecEnvOptions o;
    const auto& w = sc.world;
    o.workers = w.workers;
    o.seed = w.seed;
    o.jsbsimRoot = w.jsbsimRoot;
    o.dt = w.dt;
    o.frameSkip = w.frameSkip;
    o.worldName = w.name;
    o.publish = w.publish;
    o.terrain = w.terrain;
    const auto& v = sc.vecenv;
    o.numEnvs = v.numEnvs;
    o.vehiclesPerEnv = v.vehiclesPerEnv;
    o.task = v.task;
    o.observation = v.observation;
    o.action = v.action;
    o.actionRanges = v.actionRanges;
    o.maxEpisodeSteps = v.maxEpisodeSteps;
    o.latitudeJitterDeg = v.latitudeJitterDeg;
    o.longitudeJitterDeg = v.longitudeJitterDeg;
    o.altitudeJitterM = v.altitudeJitterM;
    o.headingJitterDeg = v.headingJitterDeg;
    o.airspeedJitterMs = v.airspeedJitterMs;
    o.targetAltitudeDeltaM = v.targetAltitudeDeltaM;
    o.targetHeadingDeltaDeg = v.targetHeadingDeltaDeg;
    if (!sc.vehicles.empty()) {
        // The first vehicle is the episode template: its aircraft and initial state.
        const auto& t = sc.vehicles.front().spec;
        const auto colon = t.type.find(':');
        o.aircraft = colon == std::string::npos ? t.type : t.type.substr(colon + 1);
        o.latitudeDeg = t.initial.latitudeDeg;
        o.longitudeDeg = t.initial.longitudeDeg;
        o.altitudeM = t.initial.altitudeMslM;
        o.headingDeg = t.initial.headingDeg;
        o.airspeedMs = t.initial.airspeedTrueMs;
    }
    o.scenarioPath = sc.path.string();
    return o;
}

} // namespace fsim
