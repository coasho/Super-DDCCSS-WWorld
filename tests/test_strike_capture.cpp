// The private viewer policy must preserve a complete native strike and take only its two requested captures.
#include "app/StrikeCapture.h"
#include "ipc/Recording.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using fsim::app::StrikeCapture;
using fsim::core::Json;
using K = fsim::ipc::EngagementKind;

void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

template<class Function> void rejected(Function function, const std::string& message) {
    bool threw = false;
    try { function(); } catch (const std::exception&) { threw = true; }
    require(threw, message);
}

Json evidence() {
    return Json::parse(R"({
        "schema":"native-engagement-v1", "source":"native-sdk", "duration_s":40,
        "native_dt_s":0.008333333333333333, "native_frame_skip":4,
        "carrier_id":1, "target_id":2, "carrier_slot":0, "target_slot":1,
        "designation_id":2147483649,
        "weapons":[{"id":3, "launcher_id":1, "weapon_slot":2, "store":"aim-120c",
            "activity_id":6442450949, "target_reference":2147483649,
            "launch_time_s":13, "end_time_s":18, "terminal_kind":"detonated",
            "seeker":"active_radar", "acquired":true, "holding_id":2}],
        "engagements":[
            {"kind":"launch", "entity":3, "launcher":1, "target":2147483649, "time_s":13},
            {"kind":"detonation", "entity":3, "launcher":1, "target":2, "time_s":18},
            {"kind":"damage", "entity":3, "launcher":1, "target":2, "time_s":18, "destroyed":true}
        ]
    })");
}

fsim::ipc::EngagementEvent event(K kind, std::uint32_t slot, double time,
                                std::uint32_t target = 0, std::uint32_t index = 0) {
    fsim::ipc::EngagementEvent value;
    value.kind = static_cast<std::uint8_t>(kind);
    value.slot = slot; value.simTime = time; value.target = target; value.index = index;
    const std::string name = "aim-120c";
    name.copy(value.name, name.size());
    return value;
}

void observeHit(StrikeCapture& policy, bool crewLost = true) {
    policy.observe(event(K::Detonation, 2, 18.0, 2)); // Detonation.target is the target slot + 1.
    policy.observe(event(K::Hit, 1, 18.0, 3, 1936)); // Hit.target is the weapon slot + 1.
    if (crewLost) policy.observe(event(K::CrewLost, 1, 18.0));
}

void fullStrike() {
    StrikeCapture policy(evidence());
    const auto& plan = policy.plan();
    require(plan.carrierId == 1 && plan.weaponId == 3 && plan.targetId == 2 &&
            plan.carrierSlot == 0 && plan.weaponSlot == 2 && plan.targetSlot == 1,
            "capture plan must retain the native entity IDs and recorded slots");
    require(plan.launchTime == 13 && plan.hitTime == 18 && plan.duration == 40 &&
            std::abs(plan.nativeStep - 1.0 / 30.0) < 1e-12,
            "capture plan must retain the native strike and complete duration");
    require(policy.selectedSlot(0) == 0 && policy.selectedSlot(12.99) == 0,
            "camera must stay on the carrier before release");
    require(policy.selectedSlot(13) == 2 && policy.selectedSlot(17.99) == 2,
            "camera must follow only the missile throughout flight");
    require(!policy.dueCapture(13.999), "first capture must not precede launch plus one second");
    require(policy.dueCapture(14) == 0, "exactly one capture must be due at launch plus one second");
    policy.acknowledge(0, 14);
    require(!policy.dueCapture(14) && !policy.dueCapture(17), "acknowledged missile capture must not repeat");
    require(policy.selectedSlot(18.1) == 2 && !policy.dueCapture(18.3),
            "planned terminal time cannot select the target or authorize a hit capture without live evidence");
    observeHit(policy);
    require(policy.selectedSlot(18) == 1 && policy.selectedSlot(39.99) == 1,
            "matching native damage must select the target and keep the camera there");
    require(!policy.dueCapture(18.299) && policy.dueCapture(18.3) == 1,
            "second capture must occur only at actual hit plus 0.3 seconds");
    policy.acknowledge(1, 18.3);
    require(!policy.dueCapture(18.3) && !policy.dueCapture(40), "second capture must not repeat");
    require(!policy.completed(18.3) && !policy.completed(39.99) && policy.completed(40),
            "two screenshots must not terminate the complete forty-second engagement");
    rejected([&] { policy.acknowledge(1, 18.3); }, "duplicate capture acknowledgement must fail");
}

void rejectedEvidence() {
    for (const std::string field : {"schema", "source"}) {
        auto input = evidence(); input.set(field, "legacy");
        rejected([&] { StrikeCapture policy(input); }, "legacy capture evidence must fail");
    }
    {
        auto input = evidence(); input.set("weapons", Json(Json::Array{}));
        rejected([&] { StrikeCapture policy(input); }, "a missing weapon must fail");
    }
    {
        auto input = evidence(); auto weapons = input.child("weapons").asArray(); weapons.push_back(weapons.front());
        input.set("weapons", Json(std::move(weapons)));
        rejected([&] { StrikeCapture policy(input); }, "multiple weapons must not change camera selection");
    }
    for (const auto& [field, replacement] : Json::Object{
             {"terminal_kind", "expired"}, {"holding_id", 4}, {"launcher_id", 4},
             {"seeker", ""}, {"acquired", false}, {"activity_id", 0}, {"weapon_slot", 1}}) {
        auto input = evidence(); auto weapons = input.child("weapons").asArray(); weapons.front().set(field, replacement);
        input.set("weapons", Json(std::move(weapons)));
        rejected([&] { StrikeCapture policy(input); }, "invalid native weapon metadata must fail: " + field);
    }
    for (const std::string kind : {"launch", "detonation", "damage"}) {
        auto input = evidence(); auto events = input.child("engagements").asArray();
        for (auto& item : events) if (item.string("kind", "") == kind) item.set("target", 99);
        input.set("engagements", Json(std::move(events)));
        rejected([&] { StrikeCapture policy(input); }, "mismatched native evidence target must fail: " + kind);
    }
    for (const std::string field : {"entity", "launcher", "time_s"}) {
        auto input = evidence(); auto events = input.child("engagements").asArray(); events.back().set(field, 99);
        input.set("engagements", Json(std::move(events)));
        rejected([&] { StrikeCapture policy(input); }, "mismatched native damage evidence must fail: " + field);
    }
    {
        auto input = evidence(); auto events = input.child("engagements").asArray(); events.pop_back();
        input.set("engagements", Json(std::move(events)));
        rejected([&] { StrikeCapture policy(input); }, "detonation without genuine native damage must fail");
    }
    {
        auto input = evidence(); auto events = input.child("engagements").asArray(); events.back().set("destroyed", false);
        input.set("engagements", Json(std::move(events)));
        rejected([&] { StrikeCapture policy(input); }, "damage without crew loss must not be presented as crew incapacitation");
    }
    for (const auto& [field, replacement] : Json::Object{
             {"carrier_slot", -1}, {"target_slot", 0}, {"native_frame_skip", 2},
             {"native_dt_s", 1.0 / 119.0}, {"duration_s", 18.2}}) {
        auto input = evidence(); input.set(field, replacement);
        rejected([&] { StrikeCapture policy(input); }, "invalid capture plan must fail: " + field);
    }
}

void rejectFalseDamage() {
    for (const K kind : {K::Detonation, K::Fire, K::Hit, K::Impact, K::Separated, K::EngineOut}) {
        StrikeCapture policy(evidence());
        rejected([&] { policy.observe(event(kind, 1, 17.9)); }, "early destructive aircraft effect must fail");
    }
    {
        StrikeCapture policy(evidence());
        rejected([&] { policy.observe(event(K::Fire, 99, 17.9)); }, "unrelated early fire in the isolated world must fail");
    }
    for (const auto& bad : {
             event(K::Detonation, 1, 18, 2), event(K::Detonation, 2, 18, 5),
             event(K::Detonation, 2, 18.1, 2), event(K::Hit, 1, 18, 4, 1936),
             event(K::Hit, 1, 18, 3, 0), event(K::Hit, 1, 18.1, 3, 1936)}) {
        StrikeCapture policy(evidence());
        rejected([&] { policy.observe(bad); }, "unmatched live weapon, target, time or fragment damage must fail");
    }
    for (const auto& bad : {event(K::Hit, 1, 18, 4, 1936), event(K::Hit, 1, 18, 3, 0),
                           event(K::Hit, 1, 18.1, 3, 1936)}) {
        StrikeCapture policy(evidence()); policy.observe(event(K::Detonation, 2, 18, 2));
        rejected([&] { policy.observe(bad); }, "a real burst must not authorize a mismatched target damage event");
    }
    {
        StrikeCapture policy(evidence());
        rejected([&] { policy.observe(event(K::Hit, 1, 18, 3, 1936)); },
                 "live hit without an observed preceding native burst must fail");
    }
    {
        StrikeCapture policy(evidence());
        policy.acknowledge(0, 14);
        policy.observe(event(K::Detonation, 2, 18, 2));
        require(policy.selectedSlot(19) == 2 && !policy.dueCapture(18.3) && !policy.completed(40),
                "burst without a damage event must not select or capture the target");
    }
}

void captureTiming() {
    {
        StrikeCapture policy(evidence());
        policy.acknowledge(0, 14); observeHit(policy, false); policy.acknowledge(1, 18.3);
        require(!policy.completed(40), "sidecar crew loss must not complete live acceptance without its native event");
    }
    {
        StrikeCapture policy(evidence());
        require(policy.dueCapture(14 + 1.0 / 30.0) == 0, "one native step of capture latency must be accepted");
        policy.acknowledge(0, 14 + 1.0 / 30.0);
        observeHit(policy);
        require(policy.dueCapture(18.3 + 1.0 / 30.0) == 1, "hit capture allows one native step of render latency");
        policy.acknowledge(1, 18.3 + 1.0 / 30.0);
        require(policy.completed(40), "allowed render latency must not prevent full completion");
    }
    {
        StrikeCapture policy(evidence());
        rejected([&] { policy.dueCapture(14.04); }, "missed missile capture must fail instead of capturing a later frame");
    }
    {
        StrikeCapture policy(evidence()); policy.acknowledge(0, 14); observeHit(policy);
        rejected([&] { policy.dueCapture(18.34); }, "missed explosion capture must fail instead of capturing a later frame");
    }
    {
        StrikeCapture policy(evidence());
        rejected([&] { policy.acknowledge(0, 13.99); }, "early capture acknowledgement must fail");
        rejected([&] { policy.acknowledge(1, 18.3); }, "target capture without an observed hit must fail");
    }
}

template<std::size_t Size> void finite(const double (&values)[Size]) {
    for (const auto value : values) require(std::isfinite(value), "pre-hit target contains a non-finite native state");
}

void healthyTarget(const fsim::sim::VehicleState& state) {
    require(!state.crashed && !state.diverged && !state.onGround, "native target crashed or diverged before the weapon hit");
    for (const auto value : {state.simTime, state.latitudeRad, state.longitudeRad, state.altitudeMslM, state.altitudeAglM,
             state.airspeedTrueMs, state.airspeedCalibratedMs, state.mach, state.alphaRad, state.betaRad, state.loadFactor,
             state.aileronRad, state.elevatorRad, state.rudderRad, state.flapsRad, state.gearPosition,
             static_cast<double>(state.speedbrakePosition), state.fuelKg, state.leadingEdgeFlapRad})
        require(std::isfinite(value), "pre-hit target contains non-finite native flight data");
    finite(state.positionEcef); finite(state.attitudeEcefToBody); finite(state.eulerRad);
    finite(state.velocityBodyMs); finite(state.velocityNedMs); finite(state.angularRateBodyRadS);
    finite(state.accelerationBodyMs2); finite(state.rotationBodyToEcef); finite(state.throttlePosition);
    finite(state.thrustN); finite(state.engineRpm); finite(state.engineN2); finite(state.afterburner);
    finite(state.nozzlePosition); finite(state.wheelCompressionM); finite(state.wheelSteerRad); finite(state.wheelSpeedMs);
}

bool sameInputs(const fsim::sim::ControlInputs& left, const fsim::sim::ControlInputs& right) {
    return left.aileron == right.aileron && left.elevator == right.elevator && left.rudder == right.rudder &&
        left.flaps == right.flaps && left.gearDown == right.gearDown &&
        left.brakeLeft == right.brakeLeft && left.brakeRight == right.brakeRight &&
        std::equal(std::begin(left.throttle), std::end(left.throttle), std::begin(right.throttle));
}

Json targetFlightOutcome(const std::vector<fsim::ipc::Recording::Frame>& frames,
                         const fsim::app::StrikeCapturePlan& plan) {
    bool crewLost = false, crashed = false, inputsFrozen = true;
    std::array<bool, fsim::sim::VehicleState::kMaxEngines> failedEngines{};
    std::array<double, fsim::sim::VehicleState::kMaxEngines> failureTimes{};
    Json::Array separations, controlLosses;
    unsigned tornPanels = 0, postHitSamples = 0;
    std::optional<fsim::sim::ControlInputs> heldInputs;
    std::optional<fsim::sim::VehicleState> before, after;
    double lastTime = 0.0;
    for (const auto& frame : frames) {
        for (const auto& value : frame.events) {
            if (value.slot != plan.targetSlot) continue;
            const auto kind = static_cast<K>(value.kind);
            if (kind == K::CrewLost) crewLost = true;
            if (kind == K::EngineOut) {
                require(value.index < failedEngines.size(), "native engine failure index is outside the recorded state");
                failedEngines[value.index] = true;
                failureTimes[value.index] = value.simTime;
            }
            if (kind == K::PanelTorn) ++tornPanels;
            const std::string name(std::begin(value.name), std::find(std::begin(value.name), std::end(value.name), '\0'));
            if (kind == K::Separated) separations.emplace_back(name);
            if (kind == K::ControlLost) controlLosses.emplace_back(name);
        }
        for (const auto& [slot, sample] : frame.samples) {
            if (slot != plan.targetSlot) continue;
            require(sample.state.engineCount >= 0 &&
                    static_cast<std::size_t>(sample.state.engineCount) <= failedEngines.size(),
                    "native target engine count is outside the recorded state");
            for (std::size_t engine = 0; engine < failedEngines.size(); ++engine)
                if (failedEngines[engine])
                    require(engine < static_cast<std::size_t>(sample.state.engineCount),
                            "native engine failure refers to an unused engine slot");
            if (frame.simTime <= plan.hitTime + 1e-7) {
                before = sample.state;
                heldInputs = sample.inputs;
                continue;
            }
            require(!sample.state.diverged && std::isfinite(sample.state.altitudeMslM) && std::isfinite(sample.state.airspeedTrueMs),
                    "post-hit flight contains non-finite native state");
            if (crewLost && heldInputs) inputsFrozen = inputsFrozen && sameInputs(sample.inputs, *heldInputs);
            for (std::size_t engine = 0; engine < failedEngines.size(); ++engine)
                if (failedEngines[engine] && frame.simTime > failureTimes[engine] + 1e-7)
                    require(sample.state.thrustN[engine] == 0.0 && sample.state.engineN2[engine] == 0.0,
                            "a recorded engine failure did not reach the native flight state");
            after = sample.state;
            lastTime = frame.simTime;
            crashed = crashed || sample.state.crashed;
            ++postHitSamples;
        }
    }
    require(before && after && heldInputs && postHitSamples > 0, "native target lacks a post-hit flight history");
    require(lastTime + 1e-7 >= plan.duration, "native target flight history ended before the full duration");
    require(!crewLost || inputsFrozen, "crew loss did not stop updates of SDK flight commands");
    Json report(Json::Object{});
    report.set("schema", "strike-target-outcome-v1").set("crew_lost", crewLost);
    report.set("engine_out_count", static_cast<double>(std::count(failedEngines.begin(), failedEngines.end(), true)));
    report.set("control_lost_channels", Json(std::move(controlLosses)));
    report.set("torn_panel_count", static_cast<double>(tornPanels)).set("separated_sections", Json(std::move(separations)));
    report.set("crashed", crashed).set("airborne_at_end", !after->crashed && !after->onGround);
    report.set("sdk_commands_frozen_after_crew_loss", crewLost && inputsFrozen);
    report.set("post_hit_samples", static_cast<double>(postHitSamples)).set("last_time_s", lastTime);
    report.set("pre_hit_airspeed_m_s", before->airspeedTrueMs).set("final_airspeed_m_s", after->airspeedTrueMs);
    report.set("pre_hit_altitude_m", before->altitudeMslM).set("final_altitude_m", after->altitudeMslM);
    return report;
}

void postHitOutcome() {
    StrikeCapture policy(evidence());
    std::vector<fsim::ipc::Recording::Frame> frames(2);
    frames[0].simTime = 18.0; frames[1].simTime = 40.0;
    fsim::ipc::VehicleSample sample;
    sample.state.engineCount = 1;
    sample.state.altitudeMslM = 6000.0;
    sample.state.airspeedTrueMs = 250.0;
    sample.state.thrustN[0] = 14000.0; sample.state.engineN2[0] = 82.0;
    frames[0].samples.emplace_back(1, sample);
    sample.state.airspeedTrueMs = 220.0; sample.state.altitudeMslM = 5980.0;
    sample.state.thrustN[0] = 0.0; sample.state.engineN2[0] = 0.0;
    frames[1].samples.emplace_back(1, sample);
    frames[1].events = {event(K::CrewLost, 1, 18.0), event(K::EngineOut, 1, 18.0)};
    const auto report = targetFlightOutcome(frames, policy.plan());
    require(report.boolean("crew_lost", false) && !report.boolean("crashed", true) &&
            report.boolean("airborne_at_end", false) && report.child("separated_sections").asArray().empty(),
            "crew incapacitation must not be reported as structural breakup or a crash");
    auto broken = frames;
    broken[1].samples.front().second.state.thrustN[0] = 14000.0;
    rejected([&] { targetFlightOutcome(broken, policy.plan()); }, "engine failure with retained thrust must fail");
    broken = frames;
    broken[1].samples.front().second.inputs.elevator = 0.5;
    rejected([&] { targetFlightOutcome(broken, policy.plan()); }, "updated SDK commands after crew loss must fail");
    broken = frames;
    broken[1].events[1].index = 3;
    rejected([&] { targetFlightOutcome(broken, policy.plan()); }, "an unused engine slot must not establish engine failure");
    broken = frames;
    broken[1].simTime = 39.0;
    rejected([&] { targetFlightOutcome(broken, policy.plan()); }, "airborne at end requires a target snapshot at the full duration");
    auto sameTick = frames;
    sameTick[1].events = {event(K::CrewLost, 1, 18.0)};
    auto failureTick = frames.front();
    failureTick.simTime = 20.0;
    failureTick.events = {event(K::EngineOut, 1, 20.0)};
    sameTick.insert(sameTick.begin() + 1, failureTick);
    require(targetFlightOutcome(sameTick, policy.plan()).number("engine_out_count", 0) == 1,
            "engine failure applies to flight snapshots after its native event, not the preceding integration");
    frames[1].samples.front().second.state.crashed = true;
    const auto crash = targetFlightOutcome(frames, policy.plan());
    require(crash.boolean("crashed", false) && !crash.boolean("airborne_at_end", true),
            "only an actual native crash may be reported as a crash");
}

void recordingReadback(const std::filesystem::path& evidencePath, const std::filesystem::path& recordingPath) {
    std::ifstream input(evidencePath, std::ios::binary);
    require(input.good(), "cannot read the actual native engagement evidence");
    std::ostringstream contents; contents << input.rdbuf();
    const auto nativeEvidence = Json::parse(contents.str(), evidencePath.string());
    StrikeCapture policy(nativeEvidence);
    const auto& plan = policy.plan();
    fsim::ipc::Recording recording;
    std::string error;
    const bool decoded = recording.load(recordingPath, &error);
    require(decoded, "cannot decode the actual native recording: " + error);
    require(std::abs(recording.header().dt - 1.0 / 120.0) < 1e-12 && recording.header().frameSkip == 4 &&
            nativeEvidence.number("rate_hz", 0) == 30,
            "live acceptance recording must retain the native 120 Hz integration and thirty snapshots per second");
    require(!recording.frames().empty() && recording.frames().front().simTime <= plan.nativeStep + 1e-7 &&
            recording.frames().back().simTime + 1e-7 >= plan.duration &&
            static_cast<double>(recording.frames().size()) >= std::floor(plan.duration / plan.nativeStep) - 1.0,
            "native recording must contain the complete engagement from startup through its full duration");
    std::map<std::uint32_t, std::uint32_t> slots;
    std::vector<std::uint32_t> subjects;
    unsigned captures = 0, preHitDestructive = 0, hitFragments = 0, healthySamples = 0;
    double previousTime = 0.0;
    for (const auto& frame : recording.frames()) {
        require(std::isfinite(frame.simTime) && frame.simTime > previousTime &&
                frame.simTime - previousTime <= plan.nativeStep + 1e-7,
                "native acceptance frames are missing, reversed or not recorded at the native cadence");
        previousTime = frame.simTime;
        for (const auto& [slot, row] : frame.tableChanges) {
            const auto [position, inserted] = slots.emplace(row.id, slot);
            require(inserted || position->second == slot, "native entity changed its recorded slot identity");
            if (row.id == plan.carrierId) require(slot == plan.carrierSlot, "native carrier ID differs from its evidence slot");
            if (row.id == plan.targetId) require(slot == plan.targetSlot, "native target ID differs from its evidence slot");
            if (row.id == plan.weaponId) require(slot == plan.weaponSlot, "native weapon ID differs from its evidence slot");
        }
        for (const auto& nativeEvent : frame.events) {
            if (nativeEvent.kind != static_cast<std::uint8_t>(K::None) && nativeEvent.simTime + 1e-7 < plan.hitTime)
                ++preHitDestructive;
            policy.observe(nativeEvent);
            if (nativeEvent.kind == static_cast<std::uint8_t>(K::Hit)) hitFragments += nativeEvent.index;
        }
        for (const auto& [slot, sample] : frame.samples)
            if (slot == plan.targetSlot && frame.simTime + 1e-7 < plan.hitTime) {
                healthyTarget(sample.state); ++healthySamples;
            }
        const auto selected = policy.selectedSlot(frame.simTime);
        if (subjects.empty() || subjects.back() != selected) subjects.push_back(selected);
        if (const auto due = policy.dueCapture(frame.simTime)) {
            require(*due == captures, "actual recording authorized an extra or out-of-order capture");
            policy.acknowledge(*due, frame.simTime); ++captures;
        }
    }
    require(slots.count(plan.carrierId) && slots.count(plan.weaponId) && slots.count(plan.targetId),
            "native recording omitted a required strike entity");
    require(healthySamples > 0 && preHitDestructive == 0 && hitFragments > 0,
            "complete native strike lacks healthy pre-hit flight or actual external fragment damage");
    require(subjects == std::vector<std::uint32_t>{plan.carrierSlot, plan.weaponSlot, plan.targetSlot},
            "native camera selection changed outside the carrier, missile and hit target sequence");
    require(captures == 2 && policy.completed(recording.frames().back().simTime),
            "native recording did not complete both requested captures and the full engagement duration");
    const auto outcome = targetFlightOutcome(recording.frames(), plan);
    require(outcome.boolean("crew_lost", false), "damage.destroyed was not backed by a native crew-loss event");
    std::cout << "target outcome: " << outcome.dump() << '\n';
    std::cout << "PASS: nativeFrames=" << recording.frames().size() << " preHitDestructiveEffects=" << preHitDestructive
              << " matchingHitFragments=" << hitFragments << " nativeCameraSubjects=" << subjects.size()
              << " captures=" << captures << " duration_s=" << recording.frames().back().simTime << '\n';
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 3) {
            recordingReadback(argv[1], argv[2]);
            return 0;
        }
        require(argc == 1, "usage: fsim_strike_capture_test [native.engagement.json native.fsrec]");
        fullStrike(); rejectedEvidence(); rejectFalseDamage(); captureTiming(); postHitOutcome();
        std::cout << "PASS: native strike camera order, two event-timed captures and complete duration\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
