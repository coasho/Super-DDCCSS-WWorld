// Native engagement recordings. World owns flight, sensors, release, guidance, fuze and damage.
// Usage: flightsim-showcase <scene.json> <out.fsrec> [--live]
#include "foundation/core/Json.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/transport/Recording.h"
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {
using fsim::core::Json;
constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180.0;
constexpr const char* kSchema = "native-engagement-v1";

void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

void keys(const Json& object, std::initializer_list<std::string_view> allowed, const std::string& path) {
    require(object.isObject(), path + " must be an object");
    std::set<std::string> seen;
    for (const auto& [key, value] : object.asObject()) {
        require(std::find(allowed.begin(), allowed.end(), key) != allowed.end(),
                path + "." + key + ": unknown field; scripted motion and forced outcomes are unsupported");
        require(seen.insert(key).second, path + "." + key + ": duplicate field");
        require(!value.isNull(), path + "." + key + ": null is unsupported");
    }
}

double number(const Json& object, const char* key, double fallback, double minimum, double maximum) {
    const double value = object.number(key, fallback);
    require(std::isfinite(value) && value >= minimum && value <= maximum,
            std::string(key) + " is outside its supported range");
    return value;
}

Json numeric(double value) { return std::isfinite(value) ? Json(value) : Json(nullptr); }
Json vector(const double* values) { return Json(Json::Array{Json(values[0]), Json(values[1]), Json(values[2])}); }

const char* functionalKind(std::uint8_t kind) {
    using K=fsim::ipc::EngagementKind;
    switch(static_cast<K>(kind)) {
        case K::Separated: return "separated";
        case K::EngineOut: return "engine_out";
        case K::ControlLost: return "control_lost";
        case K::CrewLost: return "crew_lost";
        default: return nullptr;
    }
}

// Referee evidence from the existing recording, never another flight/damage model.
Json targetResponse(const fsim::ipc::Recording& recording,std::uint32_t slot) {
    Json out(Json::Object{});
    Json::Array events,samples;
    double firstLoss=std::numeric_limits<double>::infinity();
    for (const auto& frame : recording.frames()) {
        for (const auto& event : frame.events) if (event.slot==slot) if (const char* kind=functionalKind(event.kind)) {
            Json item(Json::Object{});
            item.set("kind",kind).set("time_s",event.simTime).set("slot",static_cast<double>(event.slot));
            item.set("index",static_cast<double>(event.index)).set("name",event.name);
            item.set("position_ecef_m",vector(event.positionEcef)).set("velocity_ecef_m_s",vector(event.velocityEcef));
            events.push_back(std::move(item)); firstLoss=std::min(firstLoss,event.simTime);
        }
        for (const auto& [sampleSlot,sample] : frame.samples) if (sampleSlot==slot) {
            const auto& state=sample.state;
            Json item(Json::Object{});
            item.set("time_s",frame.simTime).set("position_ecef_m",vector(state.positionEcef));
            item.set("velocity_ned_m_s",vector(state.velocityNedMs)).set("euler_rad",vector(state.eulerRad));
            item.set("body_rates_rad_s",vector(state.angularRateBodyRadS));
            item.set("altitude_m",numeric(state.altitudeMslM)).set("speed_m_s",numeric(state.airspeedTrueMs));
            item.set("mass_kg",sample.derived[fsim::ipc::kDerivedMassKg]>0 ? numeric(sample.derived[fsim::ipc::kDerivedMassKg]) : Json(nullptr));
            item.set("crashed",state.crashed).set("diverged",state.diverged);
            item.set("axis_inputs",Json(Json::Array{Json(sample.inputs.aileron),Json(sample.inputs.elevator),Json(sample.inputs.rudder)}));
            Json::Array thrust,throttle;
            for (int k=0;k<fsim::sim::VehicleState::kMaxEngines;++k) {
                thrust.push_back(numeric(state.thrustN[k])); throttle.push_back(numeric(sample.inputs.throttle[k]));
            }
            item.set("thrust_n",Json(std::move(thrust))).set("throttle_inputs",Json(std::move(throttle)));
            samples.push_back(std::move(item));
        }
    }
    out.set("source","native-recording").set("first_loss_time_s",numeric(firstLoss));
    out.set("functional_events",Json(std::move(events))).set("samples",Json(std::move(samples)));
    return out;
}

std::string sceneHash(const std::string& text) {
    std::uint64_t hash = 14695981039346656037ull;
    for (const char byte : text) { hash ^= static_cast<unsigned char>(byte); hash *= 1099511628211ull; }
    std::ostringstream out;
    out << std::hex << std::setfill('0') << std::setw(16) << hash;
    return out.str();
}

struct Missile {
    std::string card, name;
    int station = 0;
    double launchS = 12.0, maximumSpeed = 0.0;
    fsim::ms::ActivityId activity = 0;
    std::uint16_t storeIndex = 0;
    std::uint32_t weapon = 0;
    bool consent = false, acquired = false, activated = false, supported = false, infrared = false;
};

class NativeStepper {
public:
    NativeStepper(fsim::session::World& world, bool live) : world_(world), live_(live), epoch_(Clock::now()), startedS_(world.simTime()) {}

    void step() {
        world_.step();
        if (live_) {
            const auto elapsed = std::chrono::duration<double>(world_.simTime() - startedS_);
            std::this_thread::sleep_until(epoch_ + std::chrono::duration_cast<Clock::duration>(elapsed));
        }
    }

private:
    using Clock = std::chrono::steady_clock;
    fsim::session::World& world_;
    bool live_;
    Clock::time_point epoch_;
    double startedS_;
};

std::uint16_t payload(const fsim::session::World& world, std::uint32_t id, fsim::ms::Family family) {
    const auto* host = world.missionSystems(id);
    require(host != nullptr, "carrier has no Mission Systems host");
    for (const auto& item : host->payloads()) if (item.family == family) return item.index;
    throw std::runtime_error("carrier needs native StoreStation and AirRadar payloads");
}

void accepted(const fsim::ms::CommandResult& result, const std::string& operation) {
    require(result.accepted(), operation + " rejected: " + fsim::ms::reasonName(result.reason));
}

void focus(fsim::session::World& world, std::uint32_t entity, const char* subject) {
    const bool requested = world.focusViewer(entity);
    std::printf("live focus %s: entity %u at %.9f s (%s)\n", subject, entity, world.simTime(), requested ? "requested" : "unavailable");
    std::fflush(stdout);
}

void operate(fsim::session::World& world, NativeStepper& steps, std::uint32_t id, std::uint16_t index) {
    const auto result = world.submit(id, fsim::ms::StateCommand{index, fsim::ms::PayloadState::Operate});
    require(result.accepted() || result.reason == fsim::ms::Reason::AlreadyInState,
            std::string("payload operation rejected: ") + fsim::ms::reasonName(result.reason));
    const double deadline = world.simTime() + 60.0;
    while (world.missionSystems(id)->payloads()[index].state != fsim::ms::PayloadState::Operate && world.simTime() < deadline)
        steps.step();
    require(world.missionSystems(id)->payloads()[index].state == fsim::ms::PayloadState::Operate,
            "native payload warmup did not reach OPERATE");
}

std::uint32_t aircraft(fsim::session::World& world, const Json& definition, const fsim::session::VehicleSpec& initial) {
    auto spec = initial;
    spec.type = definition.string("type", "jsbsim:f16c");
    spec.name = definition.string("name", initial.name);
    require(spec.type.rfind("jsbsim:", 0) == 0, "aircraft type must name a native jsbsim flight model");
    const auto id = world.createVehicle(spec);
    require(id != 0, "native aircraft flight model unavailable: " + spec.type);
    fsim::control::VelocityCommand hold;
    hold.airspeedMs = spec.initial.airspeedTrueMs;
    hold.verticalSpeedMs = 0.0;
    hold.headingRad = spec.initial.headingDeg * kDeg;
    require(world.submit(id, hold).accepted(), "native stable-flight command rejected for " + spec.name);
    return id;
}

void ensureStoreModel(const fsim::ms::StoreCard& card) {
    require(card.storeClass == "aam" && !card.seeker.empty(), "store needs native air-to-air seeker guidance: " + card.id);
    require(card.seeker == "active_radar" || card.seeker == "semi_active_radar" || card.seeker == "infrared" || card.seeker == "imaging_infrared",
            "native seeker guidance unsupported for store: " + card.id + " (" + card.seeker + ")");
    require(card.model.rfind("jsbsim:", 0) == 0 && card.model.size() > 7,
            "native store flight model unavailable: " + card.id);
    const fsim::io::AssetResolver assets;
    const auto root = assets.jsbsimRoot();
    require(root.has_value(), "native JSBSim runtime data unavailable");
    const auto name = card.model.substr(7);
    const auto directory = assets.findAircraft(name, *root);
    const auto path = (directory.empty() ? *root / "aircraft" : directory) / name / (name + ".xml");
    require(std::filesystem::is_regular_file(path), "native store flight model unavailable: " + card.model + " (" + path.string() + ")");
}

Json run(const Json& scene, const std::filesystem::path& output, const std::string& hash, bool live) {
    keys(scene, {"schema", "name", "duration_s", "rate_hz", "seed", "start", "carrier", "target", "missiles", "environment"}, "scene");
    require(scene.string("schema", "") == kSchema, "scene.schema must be native-engagement-v1; legacy scripted recordings are unsupported");
    const auto& start = scene.child("start");
    const auto& carrier = scene.child("carrier");
    const auto& target = scene.child("target");
    keys(start, {"lat_deg", "lon_deg", "alt_m", "heading_deg"}, "start");
    keys(carrier, {"type", "name", "speed_ms"}, "carrier");
    keys(target, {"type", "name", "speed_ms", "heading_deg", "range_m", "altitude_offset_m"}, "target");
    const double duration = number(scene, "duration_s", 40.0, 20.0, 3600.0);
    const fsim::session::WorldOptions nativeDefaults;
    const double nativeRate = 1.0 / (nativeDefaults.dt * nativeDefaults.frameSkip);
    const double rate = number(scene, "rate_hz", nativeRate, 1.0, nativeRate);
    require(rate == std::floor(rate), "rate_hz must be a positive integer divisor of the native 30 Hz world-step rate");
    const double captureDivider = nativeRate / rate;
    require(std::abs(captureDivider - std::round(captureDivider)) < 1e-9,
            "rate_hz must divide the native 30 Hz world-step rate; capture does not change guidance or fuze cadence");
    const double seed = number(scene, "seed", 17.0, 0.0, 9007199254740991.0);
    require(seed == std::floor(seed), "seed must be an integer");
    std::vector<Missile> missiles;
    require(scene.child("missiles").isArray() && !scene.child("missiles").asArray().empty(), "missiles must be a nonempty array");
    std::set<int> stations;
    for (const auto& item : scene.child("missiles").asArray()) {
        keys(item, {"card", "station_number", "name", "launch_s"}, "missiles[]");
        Missile missile;
        missile.card = item.string("card", ""); missile.name = item.string("name", missile.card);
        require(!missile.card.empty(), "each missile needs a native store card");
        const double station = number(item, "station_number", 1.0, 1.0, 255.0);
        require(station == std::floor(station), "station_number must be an integer");
        missile.station = static_cast<int>(station);
        require(stations.insert(missile.station).second, "each native station may carry one showcase missile");
        missile.launchS = number(item, "launch_s", 12.0, 0.0, duration);
        missiles.push_back(std::move(missile));
    }
    std::stable_sort(missiles.begin(), missiles.end(), [](const auto& a, const auto& b) { return a.launchS < b.launchS; });
    if (scene.has("environment"))
        keys(scene.child("environment"), {"humidity", "temperature_sea_level_k", "pressure_sea_level_pa", "wind_from_deg", "wind_ms", "cloud_cover", "cloud_base_m"}, "environment");
    fsim::session::VehicleSpec ownSpec;
    ownSpec.name = "F16C";
    ownSpec.initial.latitudeDeg = number(start, "lat_deg", 36.25, -89.0, 89.0);
    ownSpec.initial.longitudeDeg = number(start, "lon_deg", -115.03, -180.0, 180.0);
    ownSpec.initial.altitudeMslM = number(start, "alt_m", 6000.0, 100.0, 20000.0);
    ownSpec.initial.headingDeg = number(start, "heading_deg", 90.0, -360.0, 360.0);
    ownSpec.initial.airspeedTrueMs = number(carrier, "speed_ms", 250.0, 1.0, 1000.0);
    const double range = number(target, "range_m", 5000.0, 1000.0, 100000.0);
    const double targetOffset = number(target, "altitude_offset_m", 0.0, -19000.0, 19000.0);
    const double targetSpeed = number(target, "speed_ms", 250.0, 1.0, 1000.0);
    const double targetHeading = number(target, "heading_deg", std::fmod(ownSpec.initial.headingDeg + 540.0, 360.0), -360.0, 360.0);

    Json evidence(Json::Object{});
    evidence.set("schema", kSchema).set("source", "native-sdk").set("scene_hash", hash);
    evidence.set("name", scene.string("name", "native engagement")).set("duration_s", duration).set("rate_hz", rate);
    evidence.set("native_dt_s", nativeDefaults.dt).set("native_frame_skip", nativeDefaults.frameSkip);
    std::uint32_t own = 0, opponent = 0;
    {
        fsim::session::WorldOptions options;
        options.name = scene.string("name", "native engagement"); options.workers = 1; options.pinWorkers = false;
        options.publish = live; options.seed = static_cast<std::uint64_t>(seed);
        // Keep the SDK's default 120 Hz integration and 30 Hz World guidance/fuze steps.
        // Recording frequency only selects snapshots; it never retimes native flight or engagement logic.
        options.recordPath = output;
        options.recordIntervalSeconds = 1.0 / rate; options.terrainOffline = true;
        fsim::session::World world(options);
        require(world.recording(), "cannot open native recording: " + output.string());
        require(!live || world.published(), "cannot publish the native live world: " + options.name);
        NativeStepper steps(world, live);
        if (scene.has("environment")) {
            const auto& input = scene.child("environment"); auto environment = world.environment();
            environment.humidity = number(input, "humidity", environment.humidity, 0.0, 1.0);
            environment.temperatureSeaLevelK = number(input, "temperature_sea_level_k", environment.temperatureSeaLevelK, 150.0, 350.0);
            environment.pressureSeaLevelPa = number(input, "pressure_sea_level_pa", environment.pressureSeaLevelPa, 10000.0, 150000.0);
            environment.windDirectionDeg = number(input, "wind_from_deg", environment.windDirectionDeg, -360.0, 360.0);
            environment.windSpeedMs = number(input, "wind_ms", environment.windSpeedMs, 0.0, 100.0);
            environment.cloudCover = number(input, "cloud_cover", environment.cloudCover, 0.0, 1.0);
            environment.cloudBaseM = number(input, "cloud_base_m", environment.cloudBaseM, 0.0, 20000.0);
            world.setEnvironment(environment);
        }
        own = aircraft(world, carrier, ownSpec);
        if (live) focus(world, own, "carrier");
        const auto stationPayload = payload(world, own, fsim::ms::Family::StoreStation);
        const auto radar = payload(world, own, fsim::ms::Family::AirRadar);
        bool illuminate = false;
        for (auto& missile : missiles) {
            const auto reason = world.loadStore(own, stationPayload, missile.station, missile.card);
            require(reason == fsim::ms::Reason::None, "native store loading rejected for " + missile.card + ": " + fsim::ms::reasonName(reason));
            const auto* host = world.missionSystems(own);
            bool found = false;
            for (const auto& store : host->stores()) if (store.station == missile.station && store.store == missile.card) {
                missile.storeIndex = store.index; found = true;
                const auto* card = host->storeCard(store.index);
                require(card != nullptr, "native store card unavailable: " + missile.card);
                ensureStoreModel(*card);
                missile.infrared = card->seeker.find("infrared") != std::string::npos;
                illuminate = illuminate || card->seeker == "semi_active_radar";
                break;
            }
            require(found, "native loaded store inventory missing: " + missile.card);
        }
        operate(world, steps, own, stationPayload); operate(world, steps, own, radar);
        const double settled = world.simTime() + 10.0;
        while (world.simTime() < settled) steps.step();
        const auto* ownState = world.vehicleState(own);
        require(ownState && !ownState->crashed && !ownState->diverged, "carrier failed native stable-flight setup");
        fsim::session::VehicleSpec targetSpec;
        targetSpec.name = "TARGET";
        const double heading = ownSpec.initial.headingDeg * kDeg;
        targetSpec.initial.latitudeDeg = ownState->latitudeRad / kDeg + range * std::cos(heading) / 6371000.0 / kDeg;
        targetSpec.initial.longitudeDeg = ownState->longitudeRad / kDeg + range * std::sin(heading) / (6371000.0 * std::cos(ownState->latitudeRad)) / kDeg;
        targetSpec.initial.altitudeMslM = ownState->altitudeMslM + targetOffset;
        targetSpec.initial.airspeedTrueMs = targetSpeed; targetSpec.initial.headingDeg = targetHeading;
        opponent = aircraft(world, target, targetSpec);
        const double targetCreated = world.simTime();
        fsim::ms::AmtiCommand scan;
        scan.payload = radar; scan.azimuthHalfWidthRad = 5.0 * kDeg; scan.bars = 1;
        scan.rangeScaleM = std::max(60000.0, range * 1.2);
        scan.elevationCentreRad = std::atan2(targetOffset, range);
        accepted(world.submit(own, scan), "native radar search");
        std::uint64_t track = 0;
        const double searchDeadline = world.simTime() + 5.0;
        while (!track && world.simTime() < searchDeadline) {
            steps.step();
            double nearest = std::numeric_limits<double>::infinity();
            // Select from measured radar tracks; truth IDs never supply guidance or target alignment.
            for (const auto& observation : world.amtiTracks(own, radar))
                if (observation.updates >= 3 && observation.rangeM < nearest) { track = observation.id; nearest = observation.rangeM; }
        }
        require(track != 0, "native radar did not confirm a target track");
        const auto designation = world.designate(own, radar, track);
        require(designation >= fsim::session::World::kDesignations, "native radar-track designation rejected");
        fsim::ms::AmtiCommand stt;
        stt.payload = radar; stt.task = fsim::ms::AmtiTask::Track; stt.mode = fsim::ms::AmtiMode::SingleTargetTrack;
        stt.track = track; stt.rangeScaleM = scan.rangeScaleM; stt.illuminate = illuminate;
        accepted(world.submit(own, stt), "native single-target track");
        evidence.set("carrier_id", static_cast<double>(own)).set("target_id", static_cast<double>(opponent));
        evidence.set("radar_track_id", static_cast<double>(track)).set("designation_id", static_cast<double>(designation));
        evidence.set("target_created_time_s", targetCreated);
        std::size_t next = 0;
        std::uint32_t viewerWeapon = 0;
        bool viewerTargetFocused = false;
        while (world.simTime() < duration - 1e-9) {
            if (next < missiles.size()) {
                auto& missile = missiles[next];
                if (!missile.activity && world.simTime() >= missile.launchS - 1e-9) {
                    fsim::ms::StrikeCommand strike;
                    strike.payload = stationPayload; strike.store = missile.card; strike.stores = {missile.storeIndex};
                    // Infrared weapons preserve the SDK's observed on-rail contact, not a referee truth cue.
                    strike.target = missile.infrared ? world.seekerTarget(own, missile.card) : designation;
                    require(strike.target != 0, "native infrared seeker has no on-rail contact: " + missile.card);
                    const auto result = world.submit(own, strike); accepted(result, "native strike for " + missile.card);
                    missile.activity = result.activity;
                }
                if (missile.activity) {
                    const auto status = world.strikeStatus(missile.activity);
                    require(status.has_value(), "native strike status unavailable");
                    if (!missile.consent && status->phase == fsim::ms::StrikePhase::AwaitingConsent) {
                        require(status->weapons.empty(), "native strike released before explicit consent");
                        const auto reason = world.strikeConsent(missile.activity, true);
                        require(reason == fsim::ms::Reason::None, std::string("native release consent rejected: ") + fsim::ms::reasonName(reason));
                        missile.consent = true;
                    }
                    if (!status->weapons.empty()) {
                        require(status->weapons.size() == 1 && missile.consent, "native strike did not release exactly one consented store");
                        missile.weapon = status->weapons.front(); ++next;
                        if (live) {
                            focus(world, missile.weapon, "missile");
                            viewerWeapon = missile.weapon;
                            viewerTargetFocused = false;
                        }
                    } else if (const auto* activity = world.missionSystems(own)->activity(missile.activity); activity && !activity->live()) {
                        throw std::runtime_error(std::string("native strike ended without a weapon: ") + fsim::ms::reasonName(activity->reason));
                    }
                }
            }
            steps.step();
            for (auto& missile : missiles) if (missile.weapon) {
                for (const auto& record : world.weapons()) if (record.id == missile.weapon) {
                    missile.acquired = missile.acquired || record.tracking;
                    missile.supported = missile.supported || record.supported;
                    if (live && record.id == viewerWeapon && record.end == fsim::ms::WeaponEnd::Detonated && !viewerTargetFocused) {
                        const auto events = world.engagementEvents();
                        const auto hit = std::find_if(events.begin(), events.end(), [&](const auto& event) {
                            return event.entity == viewerWeapon && event.target == opponent && event.kind == "damage" && event.reason == "warhead";
                        });
                        if (hit != events.end()) {
                            focus(world, opponent, "target");
                            viewerTargetFocused = true;
                        }
                    }
                }
                if (const auto* state = world.vehicleState(missile.weapon))
                    missile.maximumSpeed = std::max(missile.maximumSpeed, state->airspeedTrueMs);
            }
        }
        require(next == missiles.size(), "scene ended before all native stores were released; increase duration_s or advance launch_s");
        Json::Array events;
        for (const auto& event : world.engagementEvents()) {
            Json item(Json::Object{});
            item.set("serial", static_cast<double>(event.serial)).set("time_s", event.timeS).set("kind", event.kind).set("reason", event.reason);
            item.set("entity", static_cast<double>(event.entity)).set("launcher", static_cast<double>(event.launcher));
            item.set("target", static_cast<double>(event.target)).set("destroyed", event.destroyed);
            item.set("position_ecef_m", vector(event.positionEcef)).set("velocity_ecef_m_s", vector(event.velocityEcef));
            events.push_back(std::move(item));
            for (auto& missile : missiles) if (event.entity == missile.weapon && event.kind == "activation") missile.activated = true;
        }
        evidence.set("engagements", Json(std::move(events)));
        Json::Array weapons;
        for (const auto& missile : missiles) {
            const auto records = world.weapons();
            const auto record = std::find_if(records.begin(), records.end(), [&](const auto& candidate) { return candidate.id == missile.weapon; });
            require(record != records.end(), "native weapon record unavailable");
            require(record->store == missile.card && record->launcher == own && !record->jettisoned && !record->seeker.empty(),
                    "native weapon release metadata did not match its armed store");
            Json item(Json::Object{});
            item.set("id", static_cast<double>(record->id)).set("launcher_id", static_cast<double>(record->launcher));
            item.set("requested_name", missile.name).set("native_name", record->name).set("store", record->store);
            item.set("station", record->station).set("activity_id", static_cast<double>(record->activity)).set("target_reference", static_cast<double>(record->target));
            item.set("launch_time_s", record->launchTimeS).set("end_time_s", record->end == fsim::ms::WeaponEnd::InFlight ? Json(nullptr) : numeric(record->endTimeS));
            item.set("terminal_kind", fsim::ms::weaponEndName(record->end));
            item.set("seeker", record->seeker).set("acquired", missile.acquired).set("activated", missile.activated).set("supported", missile.supported);
            if (const auto* card=world.missionSystems(own)->storeCard(missile.storeIndex)) {
                Json profile(Json::Object{});
                profile.set("execution",card->value("weapon_execution_version",0)).set("fuze",card->value("fuze_model_version",0))
                    .set("warhead",card->value("warhead_model_version",0));
                item.set("model_profile",std::move(profile));
            }
            item.set("max_speed_m_s", missile.maximumSpeed).set("miss_m", numeric(record->missM)).set("holding_id", static_cast<double>(record->holding));
            weapons.push_back(std::move(item));
            std::printf("native weapon %u: %s, launch %.6f s, end %s at %.6f s, maximum speed %.3f m/s\n",
                        record->id, record->store.c_str(), record->launchTimeS, fsim::ms::weaponEndName(record->end), record->endTimeS, missile.maximumSpeed);
        }
        evidence.set("weapons", Json(std::move(weapons)));
        Json::Array failures;
        // The same native paths are available through the SDK's property handles.
        // Quiet enumeration avoids creating optional properties or warning on absence.
        for (const char* prefix : {"fcs","propulsion"})
            for (const auto& [name,value] : world.model(opponent)->properties(prefix))
                if (name.find("/malfunction/fail_stuck")!=std::string::npos || name.find("/seized")!=std::string::npos) {
                    Json item(Json::Object{});
                    item.set("path",std::string(prefix)+"/"+name).set("value",numeric(value)); failures.push_back(std::move(item));
                }
        evidence.set("target_native_properties",Json(std::move(failures)));
    } // Close the native recorder before reading its authoritative entity slots.
    fsim::ipc::Recording recording;
    std::string error;
    const bool decoded = recording.load(output, &error);
    require(decoded, "native recording readback failed: " + error);
    std::map<std::uint32_t, std::uint32_t> slots;
    for (const auto& frame : recording.frames()) for (const auto& [slot, vehicle] : frame.tableChanges) slots.emplace(vehicle.id, slot);
    require(slots.count(own) && slots.count(opponent), "native recording omitted carrier or target metadata");
    evidence.set("carrier_slot", static_cast<double>(slots.at(own))).set("target_slot", static_cast<double>(slots.at(opponent)));
    Json::Array weapons = evidence.child("weapons").asArray();
    for (auto& weapon : weapons) {
        const auto id = static_cast<std::uint32_t>(weapon.number("id", 0.0));
        require(slots.count(id) != 0, "native recording omitted a released weapon");
        weapon.set("weapon_slot", static_cast<double>(slots.at(id)));
    }
    evidence.set("weapons", Json(std::move(weapons)));
    evidence.set("target_response",targetResponse(recording,slots.at(opponent)));
    evidence.set("recorded_frames", static_cast<double>(recording.frames().size()));
    return evidence;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: flightsim-showcase <scene.json> <out.fsrec> [--live]\n");
        return 2;
    }
    try {
        // Invalidate previous output even when the CLI or new scene is invalid.
        std::filesystem::remove(argv[2]);
        std::filesystem::remove(std::string(argv[2]) + ".engagement.json");
        if (argc != 3 && !(argc == 4 && std::string_view(argv[3]) == "--live")) {
            std::fprintf(stderr, "usage: flightsim-showcase <scene.json> <out.fsrec> [--live]\n");
            return 2;
        }
        std::ifstream input(argv[1], std::ios::binary);
        require(input.good(), std::string("cannot read scene: ") + argv[1]);
        std::ostringstream text; text << input.rdbuf();
        const auto raw = text.str();
        const auto evidence = run(Json::parse(raw, argv[1]), argv[2], sceneHash(raw), argc == 4);
        std::ofstream output(std::string(argv[2]) + ".engagement.json", std::ios::binary);
        require(output.good(), "cannot write native engagement evidence");
        output << evidence.dump() << '\n'; output.close();
        require(output.good(), "native engagement evidence write failed");
        return 0;
    } catch (const std::exception& error) {
        std::error_code ignored;
        std::filesystem::remove(argv[2], ignored);
        std::filesystem::remove(std::string(argv[2]) + ".engagement.json", ignored);
        std::fprintf(stderr, "flightsim-showcase: %s\n", error.what());
        return 1;
    }
}
