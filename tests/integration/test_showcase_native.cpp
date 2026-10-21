// Black-box native showcase acceptance: decode the actual recording, not only its evidence file.
#include "foundation/core/Json.h"
#include "fsim/MissionSystems.h"
#include "fsim/World.h"
#include "foundation/transport/Recording.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using fsim::core::Json;
namespace fs = std::filesystem;

void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

void write(const fs::path& path, const std::string& value) {
    std::ofstream output(path, std::ios::binary);
    output << value;
    require(output.good(), "cannot write fixture: " + path.string());
}

std::string read(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    require(input.good(), "cannot read: " + path.string());
    std::ostringstream text; text << input.rdbuf();
    return text.str();
}

std::string quoted(const fs::path& path) {
    const auto value = path.string();
#ifdef _WIN32
    require(value.find_first_of("\"\r\n%") == std::string::npos, "unsupported command path");
    return "\"" + value + "\"";
#else
    std::string out = "'";
    for (const char c : value) out += c == '\'' ? "'\\''" : std::string(1, c);
    return out + "'";
#endif
}

int execute(const fs::path& executable, const fs::path& scene, const fs::path& recording, const fs::path& log,
            const std::string& option = {}) {
    std::string command = quoted(executable) + " " + quoted(scene) + " " + quoted(recording);
    if (!option.empty()) command += " " + quoted(fs::path(option));
    command += " > " + quoted(log) + " 2>&1";
#ifdef _WIN32
    command = "\"" + command + "\"";
#endif
    return std::system(command.c_str());
}

Json scene() {
    return Json::parse(R"({
        "schema": "native-engagement-v1", "name": "native-showcase-test", "duration_s": 40,
        "rate_hz": 30, "seed": 17,
        "start": {"lat_deg": 36.25, "lon_deg": -115.03, "alt_m": 6000, "heading_deg": 90},
        "carrier": {"type": "jsbsim:f16c", "name": "launcher", "speed_ms": 250},
        "target": {"type": "jsbsim:f16c", "name": "opponent", "speed_ms": 250,
                   "heading_deg": 270, "range_m": 5000, "altitude_offset_m": 0},
        "missiles": [{"card": "aim-120c", "station_number": 1, "name": "native missile", "launch_s": 12}]
    })");
}

struct Run {
    Json evidence;
    fsim::ipc::Recording recording;
};

void functionalResponse(const Run& run) {
    using K=fsim::ipc::EngagementKind;
    const auto targetSlot=static_cast<std::uint32_t>(run.evidence.number("target_slot",-1));
    const auto& response=run.evidence.child("target_response");
    std::vector<const fsim::ipc::EngagementEvent*> losses;
    std::vector<std::pair<double,const fsim::ipc::VehicleSample*>> samples;
    for (const auto& frame : run.recording.frames()) {
        for (const auto& event : frame.events) if (event.slot==targetSlot &&
            (event.kind==static_cast<std::uint8_t>(K::Separated) || event.kind==static_cast<std::uint8_t>(K::EngineOut) ||
             event.kind==static_cast<std::uint8_t>(K::ControlLost) || event.kind==static_cast<std::uint8_t>(K::CrewLost))) losses.push_back(&event);
        for (const auto& [slot,sample] : frame.samples) if (slot==targetSlot) samples.push_back({frame.simTime,&sample});
    }
    require(!losses.empty(),"fixed showcase functional acceptance requires recorded structural separation or engine/control/crew loss; a Hit alone is insufficient");
    require(response.string("source","")=="native-recording" && response.child("functional_events").asArray().size()==losses.size(),
            "functional evidence must reproduce actual native component-loss events");
    double firstLoss=std::numeric_limits<double>::infinity();
    for (std::size_t i=0;i<losses.size();++i) {
        const auto& item=response.child("functional_events").asArray()[i];
        require(item.number("time_s",-1)==losses[i]->simTime && item.number("slot",-1)==targetSlot &&
                item.string("name","")==losses[i]->name && item.number("index",-1)==losses[i]->index,
                "functional event evidence differs from the authoritative recording");
        firstLoss=std::min(firstLoss,losses[i]->simTime);
    }
    require(response.number("first_loss_time_s",-1)==firstLoss && !samples.empty(),"functional response needs its native first-loss epoch and flight samples");
    const fsim::ipc::VehicleSample* before=nullptr;
    const fsim::ipc::VehicleSample* after=nullptr;
    double afterTime=0;
    const auto& evidenceSamples=response.child("samples").asArray();
    require(evidenceSamples.size()==samples.size(),"continued target evidence must retain every actual captured target sample");
    for (std::size_t i=0;i<samples.size();++i) {
        const auto [time,sample]=samples[i];
        const auto& item=evidenceSamples[i];
        require(item.number("time_s",-1)==time,"target evidence and native recording clocks disagree");
        for (std::size_t k=0;k<3;++k)
            require(item.child("position_ecef_m").asArray()[k].asNumber()==sample->state.positionEcef[k] &&
                    item.child("body_rates_rad_s").asArray()[k].asNumber()==sample->state.angularRateBodyRadS[k],
                    "continued target evidence must use recorded native poses and body rates");
        if (time<firstLoss) before=sample;
        else {after=sample;afterTime=time;}
    }
    require(before && after && afterTime-firstLoss>=5,"functional acceptance must record at least five seconds after native loss");
    require(!after->state.diverged,"continued damaged target must have finite native motion or a native crash");
    const double moved=std::hypot(after->state.positionEcef[0]-before->state.positionEcef[0],
                                  after->state.positionEcef[1]-before->state.positionEcef[1],
                                  after->state.positionEcef[2]-before->state.positionEcef[2]);
    bool consequence=false;
    for (const auto* loss : losses) {
        if (loss->kind==static_cast<std::uint8_t>(K::Separated))
            consequence=consequence || (after->derived[fsim::ipc::kDerivedMassKg]>0 &&
                                        after->derived[fsim::ipc::kDerivedMassKg]<before->derived[fsim::ipc::kDerivedMassKg]);
        if (loss->kind==static_cast<std::uint8_t>(K::EngineOut) || loss->kind==static_cast<std::uint8_t>(K::ControlLost))
            for (const auto& property : run.evidence.child("target_native_properties").asArray()) {
                const auto path=property.string("path","");
                if (property.number("value",0)!=1) continue;
                if (loss->kind==static_cast<std::uint8_t>(K::EngineOut) && path.find("/seized")!=std::string::npos) consequence=true;
                if (loss->kind==static_cast<std::uint8_t>(K::ControlLost) && path.find(loss->name)!=std::string::npos &&
                    path.find("/malfunction/fail_stuck")!=std::string::npos) consequence=true;
            }
        if (loss->kind==static_cast<std::uint8_t>(K::CrewLost)) {
            const fsim::ipc::VehicleSample* first=nullptr;
            for (const auto& [time,sample] : samples) if (time>=loss->simTime) {first=sample;break;}
            if (first && first->inputs.aileron==after->inputs.aileron && first->inputs.elevator==after->inputs.elevator &&
                first->inputs.rudder==after->inputs.rudder &&
                std::equal(first->inputs.throttle,first->inputs.throttle+fsim::ControlInputs::kMaxEngines,after->inputs.throttle)) consequence=true;
        }
    }
    require(consequence && (moved>0 || after->state.crashed),
            "functional loss must have actual native mass/engine/actuator/crew-control consequence and continued recorded motion or a native crash");
    std::cout << "FUNCTIONAL: recorded_losses=" << losses.size() << " first_loss_s=" << firstLoss
              << " last_sample_s=" << afterTime << " follow_s=" << afterTime-firstLoss << " moved_m=" << moved
              << " crashed=" << after->state.crashed << '\n';
}

bool destructiveEffect(const fsim::ipc::EngagementEvent& event) {
    using K = fsim::ipc::EngagementKind;
    return event.kind == static_cast<std::uint8_t>(K::Detonation) || event.kind == static_cast<std::uint8_t>(K::Hit) ||
           event.kind == static_cast<std::uint8_t>(K::Fire) || event.kind == static_cast<std::uint8_t>(K::Impact);
}

void healthyFlightState(const fsim::VehicleState& state) {
    require(!state.crashed && !state.diverged && !state.onGround, "unthreatened aircraft must remain in healthy flight");
    for (unsigned k = 0; k < 3; ++k)
        require(std::isfinite(state.positionEcef[k]) && std::isfinite(state.velocityNedMs[k]) && std::isfinite(state.eulerRad[k]),
                "unthreatened native flight contains non-finite position, velocity or attitude");
    require(std::isfinite(state.altitudeMslM) && state.altitudeMslM > 5000.0 && state.altitudeMslM < 7000.0 &&
            std::isfinite(state.airspeedTrueMs) && state.airspeedTrueMs > 180.0 && state.airspeedTrueMs < 320.0 &&
            std::isfinite(state.fuelKg) && state.fuelKg >= 0.0,
            "unthreatened native flight must retain its commanded altitude and speed with valid fuel");
}

void unthreatenedFlight(const fs::path& directory) {
    const auto output = directory / "unthreatened-flight.fsrec";
    fsim::WorldOptions options;
    options.name = "unthreatened-native-flight";
    options.publish = false;
    options.workers = 1;
    options.pinWorkers = false;
    options.seed = 17;
    options.recordPath = output.string();
    std::uint32_t aircraftId = 0;
    {
        // Use the public SDK, with its default integration and capture cadence. No weapons or damage inputs are submitted.
        fsim::World world(options);
        require(world.stepSeconds() == 1.0 / 30.0, "healthy-flight control must retain the default native cadence");
        fsim::VehicleSpec spec;
        spec.name = "unthreatened-aircraft";
        spec.type = "jsbsim:f16c";
        spec.initial.latitudeDeg = 36.25;
        spec.initial.longitudeDeg = -115.03;
        spec.initial.altitudeMslM = 6000.0;
        spec.initial.airspeedTrueMs = 250.0;
        spec.initial.headingDeg = 90.0;
        auto aircraft = world.createVehicle(spec);
        aircraftId = aircraft.id();
        fsim::control::VelocityCommand hold;
        hold.airspeedMs = 250.0;
        hold.verticalSpeedMs = 0.0;
        hold.headingRad = std::acos(-1.0) / 2.0;
        require(aircraft.submit(hold).accepted(), "public SDK rejected healthy-flight velocity control");
        const auto first = aircraft.state();
        healthyFlightState(first);
        while (world.time() < 40.0) {
            world.step();
            healthyFlightState(aircraft.state());
            require(world.vehicleCount() == 1 && world.weapons().empty() && world.engagementEvents().empty(),
                    "healthy public SDK flight produced an unsolicited entity, weapon or engagement");
        }
        const auto last = aircraft.state();
        require(std::hypot(last.positionEcef[0] - first.positionEcef[0], last.positionEcef[1] - first.positionEcef[1],
                           last.positionEcef[2] - first.positionEcef[2]) > 6000.0,
                "healthy-flight control must exercise an actual moving native aircraft");
    } // Close and flush the native World before reading its recording.
    fsim::ipc::Recording recording;
    std::string error;
    const bool decoded = recording.load(output, &error);
    require(decoded, "healthy public SDK recording failed to decode: " + error);
    require(recording.header().dt == 1.0 / 120.0 && recording.header().frameSkip == 4 &&
            recording.frames().size() >= 1200 && recording.frames().back().simTime >= 40.0,
            "healthy-flight recording must cover at least forty seconds at native cadence");
    bool created = false;
    for (const auto& frame : recording.frames()) {
        for (const auto& [slot, row] : frame.tableChanges) {
            (void)slot;
            require(row.id == aircraftId && row.alive, "healthy-flight recording contains an unsolicited creation or removal");
            created = true;
        }
        require(frame.samples.size() == 1, "healthy-flight recording must retain its one native aircraft");
        healthyFlightState(frame.samples.front().second.state);
        for (const auto& event : frame.events)
            require(!destructiveEffect(event), "unthreatened aircraft generated a recorded explosion, hit, fire or impact");
    }
    require(created, "healthy-flight recording lacks the native aircraft creation metadata");
    std::cout << "PASS: public SDK unthreatened flight remained healthy without recorded destructive effects\n";
}

Run successful(const fs::path& executable, const fs::path& directory, const std::string& name, double captureRate = 30.0,
               bool live = false, bool requireFunctional = false) {
    const auto input = directory / (name + ".json");
    const auto output = directory / (name + ".fsrec");
    const auto log = directory / (name + ".log");
    auto inputScene = scene(); inputScene.set("rate_hz", captureRate);
    write(input, inputScene.dump());
    const auto started = std::chrono::steady_clock::now();
    const int result = execute(executable, input, output, log, live ? "--live" : "");
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    require(result == 0, "native showcase failed: " + read(log));
    if (live) require(elapsed >= 39.5, "live native engagement must pace the complete forty-second run, including preparation");
    Run run;
    run.evidence = Json::parse(read(output.string() + ".engagement.json"));
    std::string error;
    const bool decoded = run.recording.load(output, &error);
    require(decoded, "native recording failed to decode: " + error);
    require(run.evidence.string("schema", "") == "native-engagement-v1" && run.evidence.string("source", "") == "native-sdk",
            "recording lacks native provenance");
    const double expectedFrames = 40.0 * captureRate;
    require(std::abs(static_cast<double>(run.recording.frames().size()) - expectedFrames) <= 1.0,
            "native recording frame count must match requested capture rate");
    require(run.evidence.number("native_frame_skip", 0) == 4 && run.evidence.number("native_dt_s", 0) == 1.0 / 120.0,
            "capture must preserve native SDK guidance and flight-model cadences");
    require(run.evidence.child("weapons").asArray().size() == 1, "consented native strike must release one missile");
    const auto& weapon = run.evidence.child("weapons").asArray().front();
    const auto& modelProfile=weapon.child("model_profile");
    require(modelProfile.number("execution",0)==1 && modelProfile.number("fuze",0)==1 && modelProfile.number("warhead",0)==1,
            "showcase must consume current versioned native model cards");
    const auto carrierId = static_cast<std::uint32_t>(run.evidence.number("carrier_id", 0));
    const auto targetId = static_cast<std::uint32_t>(run.evidence.number("target_id", 0));
    const auto weaponId = static_cast<std::uint32_t>(weapon.number("id", 0));
    const auto carrierSlot = static_cast<std::uint32_t>(run.evidence.number("carrier_slot", -1));
    const auto targetSlot = static_cast<std::uint32_t>(run.evidence.number("target_slot", -1));
    const auto weaponSlot = static_cast<std::uint32_t>(weapon.number("weapon_slot", -1));
    require(carrierId && targetId && weaponId && carrierSlot != targetSlot && targetSlot != weaponSlot,
            "native entities must have distinct IDs and recorded slots");
    require(weapon.number("launcher_id", 0) == carrierId && weapon.number("station", 0) == 1 &&
            weapon.string("store", "") == "aim-120c" && weapon.number("activity_id", 0) != 0,
            "release metadata must identify the native store and strike");
    require(weapon.number("target_reference", 0) == run.evidence.number("designation_id", 0) &&
            weapon.number("target_reference", 0) != targetId && run.evidence.number("radar_track_id", 0) != 0,
            "radar missile must use a measured designation instead of the target truth ID");
    require(weapon.boolean("acquired", false) && weapon.boolean("activated", false) && weapon.boolean("supported", false),
            "native launcher support and active seeker acquisition must occur");
    require(weapon.string("terminal_kind", "") == "detonated" && weapon.number("holding_id", 0) == targetId,
            "known head-on native fixture must detonate at its observed target");
    const double launch = weapon.number("launch_time_s", -1), end = weapon.number("end_time_s", -1);
    require(launch > 12.0 && end > launch && end < 40.0, "native warmup, consent and terminal times must be recorded");
    std::map<std::uint32_t, std::uint32_t> slots;
    unsigned bursts = 0, hits = 0, missileSamples = 0;
    double maximumSpeed = 0.0, firstMissileFuel = -1.0, lastMissileFuel = -1.0;
    std::vector<double> firstTargetPosition, lastTargetPosition;
    bool loaded = false, released = false, removed = false;
    const Json* nativeBurst = nullptr;
    bool nativeDamage = false;
    for (const auto& event : run.evidence.child("engagements").asArray()) {
        if (event.number("entity", 0) != weaponId) continue;
        if (event.string("kind", "") == "detonation") {
            require(nativeBurst == nullptr, "native weapon emitted multiple terminal bursts"); nativeBurst = &event;
        }
        if (event.string("kind", "") == "damage" && event.number("target", 0) == targetId) nativeDamage = true;
    }
    require(nativeBurst && nativeDamage && nativeBurst->number("time_s", -1) == end,
            "terminal record must correspond to genuine SDK detonation and damage events");
    if (live) {
        std::istringstream logText(read(log));
        std::string line;
        std::vector<std::string> subjects;
        while (std::getline(logText, line)) if (line.rfind("live focus ", 0) == 0) {
            std::istringstream request(line);
            std::string liveWord, focusWord, subject, entityWord, atWord, unit, status;
            std::uint32_t entity = 0;
            double time = -1.0;
            require(static_cast<bool>(request >> liveWord >> focusWord >> subject >> entityWord >> entity >> atWord >> time >> unit >> status) &&
                    liveWord == "live" && focusWord == "focus" && entityWord == "entity" && atWord == "at" && unit == "s" && status == "(requested)",
                    "live focus evidence must identify a successful native request, entity and simulation time");
            subjects.push_back(subject);
            if (subject == "carrier:") require(entity == carrierId && time == 0.0, "live view must start at its carrier before release");
            else if (subject == "missile:")
                require(entity == weaponId && std::abs(time - launch) < 1e-7, "live view must follow the actual missile at its native release");
            else if (subject == "target:")
                require(entity == targetId && std::abs(time - end) < 1e-7, "live target focus must follow the real native hit");
            else require(false, "live mode issued an unexpected camera focus request");
        }
        require(subjects == std::vector<std::string>{"carrier:", "missile:", "target:"},
                "live hit must request carrier, released missile and actually hit target once each, in order");
    }
    for (const auto& frame : run.recording.frames()) {
        for (const auto& [slot, row] : frame.tableChanges) {
            slots.emplace(row.id, slot);
            if (row.id == weaponId && !row.alive) { require(frame.simTime >= end, "weapon disappeared before its native fuze"); removed = true; }
        }
        for (const auto& [slot, stores] : frame.storeChanges) if (slot == carrierSlot) {
            for (const auto& store : stores) if (std::string(store.store) == "aim-120c") {
                if (store.status == static_cast<std::uint8_t>(fsim::ms::StoreStatus::Released)) released = true;
                else loaded = true;
            }
        }
        for (const auto& [slot, sample] : frame.samples) {
            const auto& state = sample.state;
            require(std::isfinite(state.positionEcef[0]) && std::isfinite(state.positionEcef[1]) && std::isfinite(state.positionEcef[2]),
                    "native recording contains invalid flight state");
            if (slot == weaponSlot) {
                require(frame.simTime >= launch, "released missile was recorded before its native launch");
                ++missileSamples; maximumSpeed = std::max(maximumSpeed, state.airspeedTrueMs);
                if (firstMissileFuel < 0.0) firstMissileFuel = state.fuelKg;
                lastMissileFuel = state.fuelKg;
            }
            if (slot == targetSlot && frame.simTime < end) {
                if (firstTargetPosition.empty()) firstTargetPosition.assign(state.positionEcef, state.positionEcef + 3);
                lastTargetPosition.assign(state.positionEcef, state.positionEcef + 3);
            }
        }
        for (const auto& event : frame.events) {
            if (destructiveEffect(event))
                require(event.simTime >= end, "recorded destructive effect preceded the genuine native weapon burst");
            if (event.kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Detonation)) {
                require(event.slot == weaponSlot, "aircraft or unrelated entity generated a detonation instead of the native weapon");
                ++bursts;
                require(event.simTime == end && std::string(event.name) == "aim-120c", "recorded burst did not match native terminal metadata");
                const auto& expected = nativeBurst->child("position_ecef_m").asArray();
                for (unsigned k = 0; k < 3; ++k)
                    require(std::abs(event.positionEcef[k] - expected[k].asNumber()) < 1e-6, "recorded burst position differs from SDK fuze position");
            }
            if (destructiveEffect(event))
                require(bursts > 0, "recorded damage or collision effect has no preceding native weapon burst");
            if (event.kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit) && event.slot == targetSlot) ++hits;
        }
    }
    require(slots.at(carrierId) == carrierSlot && slots.at(targetId) == targetSlot && slots.at(weaponId) == weaponSlot,
            "evidence slots must match actual recorded native IDs");
    if (!(bursts == 1 && hits > 0 && loaded && released && removed))
        std::cerr << "native fragment-Hit gate: bursts=" << bursts << ", fragment_hits=" << hits
                  << ", loaded=" << loaded << ", released=" << released << ", removed=" << removed
                  << "; contact/blast damage does not imply a fragment intersection\n";
    require(bursts == 1 && hits > 0 && loaded && released && removed, "native recording must contain inventory release, real burst/damage and weapon removal");
    require(missileSamples > 30 && maximumSpeed > 350.0 && lastMissileFuel < firstMissileFuel,
            "recorded native motor must consume fuel and accelerate the released missile");
    const double nativeMaximum = weapon.number("max_speed_m_s", 0);
    require(maximumSpeed <= nativeMaximum + 1e-6 && nativeMaximum > 350.0,
            "captured trajectory maximum cannot exceed the actual native motor maximum");
    if (captureRate == 30.0)
        require(std::abs(maximumSpeed - nativeMaximum) < 1e-6, "full native capture and evidence disagree about maximum speed");
    require(firstTargetPosition.size() == 3 && lastTargetPosition.size() == 3 &&
            std::hypot(firstTargetPosition[0] - lastTargetPosition[0], firstTargetPosition[1] - lastTargetPosition[1], firstTargetPosition[2] - lastTargetPosition[2]) > 500.0,
            "target must fly its native model before the encounter");
    if (requireFunctional) functionalResponse(run);
    return run;
}

void reject(const fs::path& executable, const fs::path& directory, Json input, const std::string& name) {
    const auto scenePath = directory / (name + ".json"), output = directory / (name + ".fsrec"), log = directory / (name + ".log");
    write(scenePath, input.dump());
    write(output, "previous recording");
    write(output.string() + ".engagement.json", R"({"schema":"native-engagement-v1","source":"native-sdk"})");
    require(execute(executable, scenePath, output, log) != 0, "native showcase accepted unsupported input: " + name);
    require(!fs::exists(output.string() + ".engagement.json"), "rejected input produced native provenance: " + name);
    require(!fs::exists(output), "rejected run preserved a partial or stale recording: " + name);
}
} // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 3 || (argc==4 && std::string(argv[3])=="functional"),
                "usage: fsim_showcase_native_test <flightsim-showcase> <output-directory> [functional]");
        const fs::path executable = fs::absolute(argv[1]), directory = fs::absolute(argv[2]);
        fs::create_directories(directory);
        if (argc==4) {
            successful(executable,directory,"functional",30,false,true);
            std::cout << "PASS: unchanged showcase native functional damage and continued flight\n";
            return 0;
        }
        unthreatenedFlight(directory);
        const auto first = successful(executable, directory, "first");
        functionalResponse(first);
        const auto second = successful(executable, directory, "second");
        require(first.evidence.dump() == second.evidence.dump(), "same seed and measured-track scene changed native events or outcome");
        const auto& a = first.recording.frames(); const auto& b = second.recording.frames();
        require(a.size() == b.size(), "repeat native recording changed frame count");
        for (std::size_t i = 0; i < a.size(); ++i) {
            require(a[i].simTime == b[i].simTime && a[i].samples.size() == b[i].samples.size(), "repeat native recording changed sample timing");
            for (std::size_t j = 0; j < a[i].samples.size(); ++j) {
                require(a[i].samples[j].first == b[i].samples[j].first, "repeat native recording changed entity slots");
                const auto& x = a[i].samples[j].second.state; const auto& y = b[i].samples[j].second.state;
                for (unsigned k = 0; k < 3; ++k)
                    require(x.positionEcef[k] == y.positionEcef[k] && x.velocityNedMs[k] == y.velocityNedMs[k], "repeat native trajectories differ");
            }
        }
        const auto live = successful(executable, directory, "live", 30.0, true);
        require(first.evidence.dump() == live.evidence.dump(), "live publication and pacing changed native events or outcome");
        const auto& liveFrames = live.recording.frames();
        require(a.size() == liveFrames.size(), "live publication changed native recording frame count");
        for (std::size_t i = 0; i < a.size(); ++i) {
            require(a[i].simTime == liveFrames[i].simTime && a[i].samples.size() == liveFrames[i].samples.size(),
                    "live pacing changed native sample timing");
            for (std::size_t j = 0; j < a[i].samples.size(); ++j) {
                require(a[i].samples[j].first == liveFrames[i].samples[j].first, "live publication changed native entity slots");
                const auto& x = a[i].samples[j].second.state; const auto& y = liveFrames[i].samples[j].second.state;
                for (unsigned k = 0; k < 3; ++k)
                    require(x.positionEcef[k] == y.positionEcef[k] && x.velocityNedMs[k] == y.velocityNedMs[k],
                            "live pacing changed native trajectories");
            }
        }
        const auto coarse = successful(executable, directory, "coarse-capture", 15.0);
        require(first.evidence.child("weapons").dump() == coarse.evidence.child("weapons").dump() &&
                first.evidence.child("engagements").dump() == coarse.evidence.child("engagements").dump(),
                "capture rate must not change native guidance, fuze or damage outcomes");
        for (const auto& frame : coarse.recording.frames()) {
            const auto found = std::find_if(a.begin(), a.end(), [&](const auto& original) { return original.simTime == frame.simTime; });
            require(found != a.end() && found->samples.size() == frame.samples.size(), "coarse capture must retain original native snapshot timing");
            for (std::size_t j = 0; j < frame.samples.size(); ++j) {
                require(found->samples[j].first == frame.samples[j].first, "capture rate changed native entity slots");
                for (unsigned k = 0; k < 3; ++k)
                    require(found->samples[j].second.state.positionEcef[k] == frame.samples[j].second.state.positionEcef[k],
                            "capture rate changed native flight trajectory");
            }
        }
        for (const auto* key : {"phases", "fin_deg", "detonate_s", "dive_s", "dive_deg", "station", "eject_ms", "ignition_delay_s", "cd", "launch", "bay"}) {
            auto input = scene(); auto missiles = input.child("missiles").asArray(); missiles.front().set(key, 1);
            input.set("missiles", Json(std::move(missiles))); reject(executable, directory, input, key);
        }
        auto truthAlignment = scene(); auto target = truthAlignment.child("target"); target.set("miss_m", Json(Json::Array{Json(0), Json(0), Json(0)}));
        truthAlignment.set("target", target); reject(executable, directory, truthAlignment, "forced-target");
        auto legacy = scene(); legacy.set("schema", "scripted-motion-v0"); reject(executable, directory, legacy, "legacy-schema");
        auto unsupportedRate = scene(); unsupportedRate.set("rate_hz", 60); reject(executable, directory, unsupportedRate, "capture-above-native");
        unsupportedRate.set("rate_hz", 7); reject(executable, directory, unsupportedRate, "capture-non-divisor");
        unsupportedRate.set("rate_hz", 2.5); reject(executable, directory, unsupportedRate, "capture-fractional-divisor");
        auto artificialThrust = scene(); auto carrier = artificialThrust.child("carrier"); carrier.set("thrust_n", 45000);
        artificialThrust.set("carrier", carrier); reject(executable, directory, artificialThrust, "forced-thrust");
        auto missing = scene(); auto missiles = missing.child("missiles").asArray(); missiles.front().set("card", "missing-native-store");
        missing.set("missiles", Json(std::move(missiles))); reject(executable, directory, missing, "missing-store");
        const auto invalidOption = directory / "invalid-option.json", invalidOutput = directory / "invalid-option.fsrec";
        const auto invalidLog = directory / "invalid-option.log";
        write(invalidOption, scene().dump());
        write(invalidOutput, "old recording");
        write(invalidOutput.string() + ".engagement.json", "old evidence");
        require(execute(executable, invalidOption, invalidOutput, invalidLog, "--unknown") != 0 && !fs::exists(invalidOutput) &&
                !fs::exists(invalidOutput.string() + ".engagement.json"),
                "unsupported CLI option was accepted or retained stale native output");
        const auto malformed = directory / "malformed.json", output = directory / "malformed.fsrec", log = directory / "malformed.log";
        write(malformed, "{"); write(output, "old recording");
        write(output.string() + ".engagement.json", R"({"source":"native-sdk"})");
        require(execute(executable, malformed, output, log) != 0 && !fs::exists(output) && !fs::exists(output.string() + ".engagement.json"),
                "JSON parse failure retained old native evidence");
        std::cout << "PASS: native showcase recording, real guidance/fuze/damage, determinism and legacy rejection\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
