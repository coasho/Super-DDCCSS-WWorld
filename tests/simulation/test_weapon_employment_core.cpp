// Weapon-employment decisions exercise the native SDK path, not a test flight model.
#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/Cards.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT "D:/Projects/Cpp/Super-DDCCSS-WWorld/third_party/jsbsim"
#endif

namespace {
using namespace fsim;
constexpr double pi = 3.14159265358979323846;
constexpr double deg = pi / 180.0;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

session::WorldOptions options(const char* name) {
    session::WorldOptions out;
    out.name = name;
    out.publish = false;
    out.workers = 1;
    out.pinWorkers = false;
    out.seed = 713;
    out.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    return out;
}

std::uint32_t launcher(session::World& world) {
    session::VehicleSpec spec;
    spec.name = "launcher";
    spec.type = "jsbsim:f16c";
    spec.initial.altitudeMslM = 6000;
    spec.initial.airspeedTrueMs = 250;
    spec.initial.headingDeg = 90;
    const auto id = world.createVehicle(spec);
    require(id != 0, "Fixture launcher creation failed");
    control::VelocityCommand hold;
    hold.airspeedMs = 250;
    hold.verticalSpeedMs = 0;
    hold.headingRad = 90 * deg;
    require(world.submit(id, hold).accepted(), "Fixture flight hold was refused");
    return id;
}

std::uint16_t stations(const session::World& world, std::uint32_t id) {
    const auto* host = world.missionSystems(id);
    require(host != nullptr, "Fixture mission systems are missing");
    for (const auto& payload : host->payloads())
        if (payload.family == ms::Family::StoreStation) return payload.index;
    throw std::runtime_error("Fixture store stations are missing");
}

void operate(session::World& world, std::uint32_t id, std::uint16_t payload) {
    const auto result = world.submit(id, ms::StateCommand{payload, ms::PayloadState::Operate});
    require(result.accepted() || result.reason == ms::Reason::AlreadyInState,
            "Fixture store station operation was refused");
    for (unsigned k = 0; k < 1200 && world.missionSystems(id)->payloads()[payload].state != ms::PayloadState::Operate; ++k)
        world.step();
    require(world.missionSystems(id)->payloads()[payload].state == ms::PayloadState::Operate,
            "Fixture store station did not finish warming");
}

std::uint16_t storeIndex(const session::World& world, std::uint32_t id, int station) {
    for (const auto& store : world.stores(id)) if (store.station == station) return store.index;
    throw std::runtime_error("Fixture store index is missing");
}

sim::InitialConditions targetConditions(const session::World& world, std::uint32_t own,
                                        double angleDeg, double headingDeg = 90) {
    const auto state = *world.vehicleState(own);
    sim::InitialConditions initial;
    initial.latitudeDeg = state.latitudeRad / deg + 5000 * std::sin(angleDeg * deg) / 6371000 / deg;
    initial.longitudeDeg = state.longitudeRad / deg
        + 5000 * std::cos(angleDeg * deg) / (6371000 * std::cos(state.latitudeRad)) / deg;
    initial.altitudeMslM = state.altitudeMslM;
    initial.airspeedTrueMs = 250;
    initial.headingDeg = headingDeg;
    return initial;
}

std::uint32_t targetAt(session::World& world, std::uint32_t own, double angleDeg, double headingDeg = 90) {
    session::VehicleSpec spec;
    spec.name = "target";
    spec.type = "jsbsim:f16c";
    spec.initial = targetConditions(world, own, angleDeg, headingDeg);
    const auto id = world.createVehicle(spec);
    require(id != 0, "Fixture target creation failed");
    control::VelocityCommand hold;
    hold.airspeedMs = 250;
    hold.verticalSpeedMs = 0;
    hold.headingRad = headingDeg * deg;
    require(world.submit(id, hold).accepted(), "Fixture target flight hold was refused");
    return id;
}

void all_selected_aams_are_validated_atomically() {
    session::World world(options("selected-aam-atomicity"));
    const auto own = launcher(world);
    const auto payload = stations(world, own);
    require(world.loadStore(own, payload, 1, "aim-120c") == ms::Reason::None,
            "Fixture active radar missile load failed");
    require(world.loadStore(own, payload, 2, "aim-9x") == ms::Reason::None,
            "Fixture infrared missile load failed");
    operate(world, own, payload);
    const auto target = targetAt(world, own, 40);
    const auto before = world.stores(own);
    const auto revision = world.missionSystems(own)->storeRevision();
    const auto events = world.engagementEvents().size();
    const auto activities = world.missionSystems(own)->activities().size();
    ms::StrikeCommand strike;
    strike.payload = payload;
    strike.target = target;
    strike.stores = {storeIndex(world, own, 1), storeIndex(world, own, 2)};
    ms::CommandOptions command;
    command.commandId = 713;
    const auto result = world.submit(own, strike, command);
    require(!result.accepted() && result.reason == ms::Reason::OutOfReach,
            "A later selected infrared missile outside its own FOV must reject the entire strike");
    require(result.activity == 0 && result.commandId == command.commandId,
            "Atomic selection refusal must preserve the caller correlation without allocating an activity");
    const auto after = world.stores(own);
    require(after.size() == before.size(), "Atomic refusal must preserve the inventory");
    for (std::size_t k = 0; k < before.size(); ++k)
        require(after[k].status == before[k].status && after[k].weapon == before[k].weapon
                    && after[k].massKg == before[k].massKg && after[k].fuelKg == before[k].fuelKg,
                "Atomic refusal must not prepare, release or change an installed store");
    require(world.missionSystems(own)->storeRevision() == revision
                && world.missionSystems(own)->activities().size() == activities
                && !world.missionSystems(own)->releasing() && world.weapons().empty()
                && world.engagementEvents().size() == events,
            "Atomic refusal must preserve revisions, activities, release queues, flight and events");
}

void gone_aam_cannot_supply_another_stores_launch_rules() {
    session::World world(options("selected-aam-gone-store"));
    const auto own = launcher(world);
    const auto payload = stations(world, own);
    // Station 1's 192.8 kg wingtip limit cannot take the 225 kg Sparrow.
    // Stations 2 and 3 both admit these AAMs and preserve old-before-new order.
    require(world.loadStore(own, payload, 2, "aim-7m") == ms::Reason::None,
            "Fixture semi-active missile load failed");
    require(world.loadStore(own, payload, 3, "aim-120c") == ms::Reason::None,
            "Fixture active missile load failed");
    operate(world, own, payload);
    ms::JettisonCommand jettison;
    jettison.payload = payload;
    jettison.stores = {storeIndex(world, own, 2)};
    jettison.emergency = true;
    require(world.submit(own, jettison).accepted(), "Fixture old missile jettison failed");
    world.step();
    const auto inventory = world.stores(own);
    const auto old = std::find_if(inventory.begin(), inventory.end(), [](const auto& store) { return store.station == 2; });
    require(old != inventory.end() && old->status == ms::StoreStatus::Jettisoned,
            "Fixture first missile must already be gone");
    ms::StrikeCommand strike;
    strike.payload = payload;
    strike.target = targetAt(world, own, 0);
    strike.quantity = 1;
    const auto result = world.submit(own, strike);
    require(result.accepted(),
            "An automatic selection of an available active missile cannot inherit a gone SARH missile's illumination rule");
    const auto status = world.strikeStatus(result.activity);
    require(status && status->stores.size() == 1 && status->stores.front() == storeIndex(world, own, 3),
            "Launch checks and committed store selection must name the same available store");
}

void ready_infrared_store_retains_on_rail_observation() {
    session::World world(options("prepared-infrared-on-rail"));
    const auto own = launcher(world);
    const auto payload = stations(world, own);
    require(world.loadStore(own, payload, 1, "aim-9x") == ms::Reason::None,
            "Fixture infrared missile load failed");
    operate(world, own, payload);
    const auto target = targetAt(world, own, 0);
    require(world.seekerTarget(own, "aim-9x") == target,
            "Loaded infrared fixture must observe its on-axis target");
    ms::StrikeCommand strike;
    strike.payload = payload;
    strike.store = "aim-9x";
    strike.target = target;
    const auto request = world.submit(own, strike);
    require(request.accepted(), "Fixture infrared preparation was refused");
    for (unsigned k = 0; k < 1200; ++k) {
        const auto status = world.strikeStatus(request.activity);
        require(status.has_value(), "Prepared infrared activity disappeared");
        if (status->phase == ms::StrikePhase::AwaitingConsent) break;
        world.step();
    }
    const auto status = world.strikeStatus(request.activity);
    require(status && status->phase == ms::StrikePhase::AwaitingConsent && world.weapons().empty(),
            "Fixture must retain the prepared missile on its rail while waiting for consent");
    const auto inventory = world.stores(own);
    require(std::any_of(inventory.begin(), inventory.end(), [](const auto& store) {
                return store.store == std::string("aim-9x") && store.status == ms::StoreStatus::Ready;
            }), "Fixture infrared missile must be Ready");
    require(world.seekerTarget(own, "aim-9x") == target,
            "A Ready infrared missile awaiting consent must retain its current on-rail observation");
}

void awaitConsent(session::World& world, ms::ActivityId activity) {
    for (unsigned k = 0; k < 1200; ++k) {
        const auto status = world.strikeStatus(activity);
        require(status.has_value(), "Fixture strike disappeared before consent");
        if (status->phase == ms::StrikePhase::AwaitingConsent) return;
        world.step();
    }
    throw std::runtime_error("Fixture strike did not reach AwaitingConsent");
}

std::uint16_t radarPayload(const session::World& world, std::uint32_t own) {
    for (const auto& payload : world.missionSystems(own)->payloads())
        if (payload.family == ms::Family::AirRadar) return payload.index;
    throw std::runtime_error("Fixture radar is missing");
}

std::uint32_t measuredDesignation(session::World& world, std::uint32_t own,
                                  std::uint16_t radar, std::uint32_t target, bool illuminate) {
    ms::AmtiCommand scan;
    scan.payload = radar;
    scan.azimuthHalfWidthRad = 5 * deg;
    scan.bars = 1;
    scan.rangeScaleM = 15000;
    require(world.submit(own, scan).accepted(), "Fixture radar search was refused");
    std::uint64_t track = 0;
    for (unsigned k = 0; k < 300 && !track; ++k) {
        world.step();
        // Truth is used only to verify this controlled fixture's measured track.
        // The strike and its controller receive the designation, never this ID.
        for (const auto& observation : world.amtiTracks(own, radar))
            if (observation.truth == target && observation.updates >= 3) { track = observation.id; break; }
    }
    require(track != 0, "Fixture radar did not confirm the actual target");
    const auto designation = world.designate(own, radar, track);
    require(designation >= session::World::kDesignations, "Fixture measured designation was refused");
    ms::AmtiCommand stt;
    stt.payload = radar;
    stt.task = ms::AmtiTask::Track;
    stt.mode = ms::AmtiMode::SingleTargetTrack;
    stt.track = track;
    stt.illuminate = illuminate;
    stt.rangeScaleM = 15000;
    require(world.submit(own, stt).accepted(), "Fixture single-target radar track was refused");
    world.step();
    require(!illuminate || world.missionSystems(own)->radarStatus(radar).illuminating,
            "Fixture SARH transmitter must actually illuminate its own measured track");
    return designation;
}

void release_time_fov_loss_refuses_every_due_store_atomically() {
    session::World world(options("release-time-fov-loss"));
    const auto own = launcher(world);
    const auto payload = stations(world, own);
    require(world.loadStore(own, payload, 1, "aim-120c") == ms::Reason::None
                && world.loadStore(own, payload, 2, "aim-9x") == ms::Reason::None,
            "Fixture mixed due stores must load");
    operate(world, own, payload);
    const auto target = targetAt(world, own, 0);
    ms::StrikeCommand strike;
    strike.payload = payload;
    strike.target = target;
    strike.stores = {storeIndex(world, own, 1), storeIndex(world, own, 2)};
    strike.rippleS = 0;
    const auto request = world.submit(own, strike);
    require(request.accepted(), "Fixture on-axis mixed strike must be admitted");
    awaitConsent(world, request.activity);
    const auto moved = targetConditions(world, own, 40);
    require(world.resetVehicle(target, &moved), "Fixture target reset outside IR launch FOV failed");
    require(world.strikeConsent(request.activity, true) == ms::Reason::None,
            "Fixture release consent must reach native execution");
    world.step();
    const auto* result = world.missionSystems(own)->activity(request.activity);
    require(result && result->state == control::ActivityState::Failed && result->reason == ms::Reason::OutOfReach,
            "Execution must recheck current IR FOV and preserve its native OutOfReach refusal");
    require(world.weapons().empty() && world.strikeStatus(request.activity)->weapons.empty()
                && !world.missionSystems(own)->releasing(),
            "A later due IR refusal must not launch an earlier due ARH member of the same activity");
    for (const auto& store : world.stores(own))
        require(store.status == ms::StoreStatus::Loaded && store.weapon == 0,
                "A refused due group must remain aboard and free its stores");
    for (const auto& event : world.engagementEvents())
        require(event.kind != "launch" && event.kind != "acquisition",
                "A release refusal cannot emit a launch or acquisition side effect");
}

void release_time_illumination_loss_preserves_native_refusal() {
    session::World world(options("release-time-illumination-loss"));
    const auto own = launcher(world);
    const auto payload = stations(world, own);
    const auto radar = radarPayload(world, own);
    require(world.loadStore(own, payload, 2, "aim-7m") == ms::Reason::None,
            "Fixture SARH store must load on an admitted station");
    operate(world, own, payload);
    operate(world, own, radar);
    const auto target = targetAt(world, own, 0, 270);
    const auto designation = measuredDesignation(world, own, radar, target, true);
    ms::StrikeCommand strike;
    strike.payload = payload;
    strike.store = "aim-7m";
    strike.target = designation;
    const auto request = world.submit(own, strike);
    require(request.accepted(), "An illuminated SARH fixture must be admitted");
    awaitConsent(world, request.activity);
    require(world.submit(own, ms::StateCommand{radar, ms::PayloadState::Standby}).accepted(),
            "Fixture radar standby command failed");
    for (unsigned k = 0; k < 120 && world.missionSystems(own)->radarStatus(radar).illuminating; ++k) world.step();
    require(!world.missionSystems(own)->radarStatus(radar).illuminating,
            "Fixture transmitter must stop illuminating before release");
    require(world.strikeConsent(request.activity, true) == ms::Reason::None,
            "Fixture consent must reach native SARH execution");
    world.step();
    const auto* result = world.missionSystems(own)->activity(request.activity);
    require(result && result->state == control::ActivityState::Failed && result->reason == ms::Reason::UnknownTrack,
            "Execution must recheck current illumination and retain native UnknownTrack rather than StoreNotFlyable");
    require(world.weapons().empty() && !world.missionSystems(own)->releasing(),
            "A SARH release refused after illumination loss cannot spawn a weapon");
    const auto inventory = world.stores(own);
    require(inventory.size() == 1 && inventory.front().status == ms::StoreStatus::Loaded
                && inventory.front().weapon == 0,
            "A refused SARH release must remain attached and free");
}

void agent_designation_does_not_replace_first_infrared_observation_identity() {
    session::World world(options("agent-infrared-designation-cue"));
    const auto own = launcher(world);
    const auto payload = stations(world, own);
    const auto radar = radarPayload(world, own);
    require(world.loadStore(own, payload, 1, "aim-9x") == ms::Reason::None,
            "Fixture infrared store must load");
    operate(world, own, payload);
    operate(world, own, radar);
    const auto target = targetAt(world, own, 0, 270);
    const auto designation = measuredDesignation(world, own, radar, target, false);
    ms::StrikeCommand strike;
    strike.payload = payload;
    strike.store = "aim-9x";
    strike.target = designation;
    ms::CommandOptions actor;
    actor.controller = world.mintAgent(own, "infrared-actor");
    require(actor.controller != 0, "Fixture native Agent identity must be minted");
    const auto request = world.submit(own, strike, actor);
    require(request.accepted(), "A native Agent's own measured IR designation must be admitted");
    awaitConsent(world, request.activity);
    require(world.seekerTarget(own, "aim-9x") == target,
            "Fixture prepared seeker must actually observe its target before consent");
    // This is a native cue regression. The existing Range consent seam supplies
    // the fixture authorization; installed Agent/C2 acceptance is separate.
    require(world.strikeConsent(request.activity, true) == ms::Reason::None,
            "Fixture native release consent failed");
    for (unsigned k = 0; k < 30 && world.weapons().empty(); ++k) world.step();
    require(world.weapons().size() == 1, "Fixture authorized IR release must spawn exactly once");
    const auto weapon = world.weapons().front().id;
    world.step(); // The first actual Guidance update after the native release.
    const auto records = world.weapons();
    const auto result = std::find_if(records.begin(), records.end(),
        [&](const auto& record) { return record.id == weapon; });
    require(result != records.end() && result->target == designation && result->tracking
                && result->phase == ms::WeaponPhase::Homing && result->holding == target,
            "The first IR update must associate a current observation, never interpret the Agent designation ID as a sensor observation ID");
    const auto events = world.engagementEvents();
    require(std::any_of(events.begin(), events.end(),
        [&](const auto& event) { return event.entity == weapon && event.kind == "acquisition" && event.target == target; }),
            "First-update acquisition must be backed by a native observation event");
}

struct CardFile {
    std::filesystem::path directory;
    std::filesystem::path path;
    explicit CardFile(const std::string& capability) {
        directory = std::filesystem::temp_directory_path() / "fsim-weapon-employment-core";
        std::filesystem::create_directories(directory);
        path = directory / "synthetic-capability.json";
        std::ofstream file(path, std::ios::binary);
        file << "{\"id\":\"synthetic-capability\",\"designation\":\"Synthetic parser fixture\","
                "\"class\":\"aam\",\"model\":\"synthetic\",\"seeker\":\"infrared\","
                "\"sources\":{\"fixture\":{\"title\":\"Synthetic software fixture\","
                "\"publisher\":\"SDK regression\",\"url\":\"https://example.invalid/fixture\",\"date\":\"\"}},"
                "\"parameters\":{\"mass_kg\":{\"value\":1,\"unit\":\"kg\",\"uncertainty\":0,"
                "\"basis\":\"assumed\",\"note\":\"Synthetic parser input only\"}}"
             << capability << "}";
        require(static_cast<bool>(file), "Could not save the synthetic card fixture");
    }
    ~CardFile() {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
        std::filesystem::remove(directory, ignored);
    }
};

void malformed_explicit_guidance_metadata_is_refused() {
    for (const auto& metadata : {
            std::string{",\"guidance_capabilities\":{\"version\":2,\"inertial_midcourse\":true,\"launcher_updates\":false,\"basis\":\"assumed\",\"note\":\"Synthetic software profile\"}"},
            std::string{",\"guidance_capabilities\":{\"version\":1,\"inertial_midcourse\":\"true\",\"launcher_updates\":false,\"basis\":\"assumed\",\"note\":\"Synthetic software profile\"}"},
            std::string{",\"guidance_capabilities\":{\"version\":1,\"launcher_updates\":false,\"basis\":\"assumed\",\"note\":\"Synthetic software profile\"}"},
            std::string{",\"guidance_capabilities\":null"},
            std::string{",\"guidance_capabilities\":{\"version\":1,\"inertial_midcourse\":true,\"launcher_updates\":false,\"basis\":\"unknown\",\"source\":\"fixture\",\"note\":\"Synthetic software profile\"}"},
            std::string{",\"guidance_capabilities\":{\"version\":1,\"inertial_midcourse\":true,\"launcher_updates\":false,\"basis\":\"assumed\",\"source\":\"unlisted\",\"note\":\"Synthetic software profile\"}"},
            std::string{",\"guidance_capabilities\":{\"version\":1,\"inertial_midcourse\":true,\"launcher_updates\":false,\"basis\":\"assumed\",\"source\":\"/guidance\",\"note\":\"No authored guidance field exists in this fixture\"}"},
            std::string{",\"guidance_capabilities\":{\"version\":1,\"inertial_midcourse\":true,\"launcher_updates\":false,\"basis\":\"assumed\",\"source\":\"fixture\",\"note\":\"\"}"}}) {
        CardFile file(metadata);
        std::vector<std::string> errors;
        const auto card = ms::readStoreCard(file.path, errors);
        require(!card && !errors.empty(),
                "Malformed explicit guidance capability metadata must fail closed instead of silently becoming a legacy fit");
    }
}

void legacy_card_still_parses_without_new_equipment_claims() {
    CardFile file("");
    std::vector<std::string> errors;
    const auto card = ms::readStoreCard(file.path, errors);
    require(card && errors.empty() && card->seeker == "infrared",
            "A legacy external card without explicit metadata must remain readable");
    require(card->guidanceCapabilities.version == 0,
            "An unversioned external card must explicitly retain the legacy policy");
}

void explicit_guidance_metadata_preserves_independent_capabilities() {
    for (const bool inertial : {false, true}) for (const bool updates : {false, true}) {
        const std::string metadata = ",\"guidance_capabilities\":{\"version\":1,\"inertial_midcourse\":"
            + std::string(inertial ? "true" : "false") + ",\"launcher_updates\":"
            + (updates ? "true" : "false")
            + ",\"basis\":\"assumed\",\"source\":\"fixture\",\"note\":\"Synthetic execution declaration; no fitted link calibration\"}";
        CardFile file(metadata);
        std::vector<std::string> errors;
        const auto card = ms::readStoreCard(file.path, errors);
        require(card && errors.empty(), "A complete synthetic capability declaration must parse");
        const auto& profile = card->guidanceCapabilities;
        require(profile.version == 1 && profile.inertialMidcourse == inertial
                    && profile.launcherUpdates == updates && profile.basis == ms::Basis::Assumed
                    && profile.source == "fixture" && !profile.note.empty(),
                "Inertial steering and launcher updates must remain independent explicit facts with provenance");
    }
}

void maintained_aam_declarations_are_explicit_assumptions() {
    const auto root = std::filesystem::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();
    std::vector<std::string> errors;
    const auto library = ms::readStoreLibrary(root / "assets" / "stores", errors);
    require(errors.empty(), "Maintained store cards must parse cleanly");
    unsigned aams = 0, inertial = 0, updates = 0;
    for (const auto& item : library) {
        const auto& card = *item.second;
        if (card.storeClass != "aam") continue;
        ++aams;
        const auto& profile = card.guidanceCapabilities;
        require(profile.version == 1 && profile.basis == ms::Basis::Assumed
                    && profile.source == "/guidance" && !profile.note.empty(),
                "Every maintained AAM needs an explicit authored assumption, not an inferred fitted capability");
        inertial += profile.inertialMidcourse ? 1u : 0u;
        updates += profile.launcherUpdates ? 1u : 0u;
    }
    require(aams == 24 && inertial == 13 && updates == 11,
            "The reviewed authored capability inventory must cover the 24 maintained AAMs");
}

struct HostFixture {
    ms::Host host;
    std::uint16_t payload = 0;
    HostFixture() {
        const auto root = std::filesystem::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();
        std::vector<std::string> errors;
        const auto fit = ms::readFit(root / "aircraft" / "f16c" / "payloads.json", root / "assets" / "payloads", errors);
        require(fit && errors.empty(), "Private host fixture fit must parse");
        host.bind(7, fit, 713);
        while (payload < host.payloads().size() && host.payloads()[payload].family != ms::Family::StoreStation) ++payload;
        require(payload < host.payloads().size(), "Private host fixture needs store stations");
        require(host.load(payload, 1, "aim-120c") == ms::Reason::None
                    && host.load(payload, 2, "aim-120c") == ms::Reason::None
                    && host.load(payload, 3, "aim-9x") == ms::Reason::None,
                "Private host fixture stores must load");
    }
    void operate() {
        require(host.submit(ms::StateCommand{payload, ms::PayloadState::Operate}, {}, 0).accepted(),
                "Private host station activation failed");
        host.step(100);
        require(host.payloads()[payload].state == ms::PayloadState::Operate,
                "Private host station must be operating");
    }
    std::uint16_t index(int station) const {
        for (const auto& store : host.stores()) if (store.station == station) return store.index;
        throw std::runtime_error("Private host store index missing");
    }
};

void preview_is_pure_and_matches_committed_selection() {
    HostFixture fixture;
    ms::StrikeCommand strike;
    strike.payload = fixture.payload;
    strike.store = "aim-120c";
    strike.quantity = 2;
    const auto revision = fixture.host.storeRevision();
    std::vector<std::uint16_t> selection;
    require(fixture.host.previewStrike(strike, selection) == ms::Reason::None
                && selection == std::vector<std::uint16_t>{fixture.index(1), fixture.index(2)},
            "Selection preview must choose the actual available matching inventory");
    require(fixture.host.storeRevision() == revision && fixture.host.activities().empty()
                && !fixture.host.releasing(), "Selection preview cannot admit, prepare or queue a strike");
    require(fixture.host.submit(strike, {}, 0).reason == ms::Reason::PayloadOff,
            "A pure preview cannot bypass the host's unchanged master-arm admission");
    strike.stores = {fixture.index(2), fixture.index(1)};
    require(fixture.host.previewStrike(strike, selection) == ms::Reason::None
                && selection == strike.stores,
            "Explicit selection preview must preserve the host's request order");
    fixture.operate();
    const auto accepted = fixture.host.submit(strike, {}, 100);
    const auto status = fixture.host.strikeStatus(accepted.activity);
    require(accepted.accepted() && status && status->stores == selection,
            "Selection preview and committed strike must name exactly the same ordered stores");
    require(fixture.host.previewStrike(strike, selection) == ms::Reason::StoreBusy && selection.empty(),
            "A held explicit store must fail preview without returning a partial plan");
}

void refusal_preserves_reason_prior_weapons_and_unrelated_queue() {
    HostFixture fixture;
    fixture.operate();
    ms::StrikeCommand strike;
    strike.payload = fixture.payload;
    strike.stores = {fixture.index(1), fixture.index(2)};
    strike.consent = true;
    strike.rippleS = 10;
    const auto request = fixture.host.submit(strike, {}, 100);
    require(request.accepted(), "Private refused-release fixture must start its strike");
    fixture.host.step(101);
    const auto first = fixture.host.takeReleases();
    require(first.size() == 1 && first.front().store == fixture.index(1),
            "First ripple member must be due alone");
    fixture.host.released(first.front().store, 123, 101);
    fixture.host.step(111);
    ms::JettisonCommand independent;
    independent.payload = fixture.payload;
    independent.stores = {fixture.index(3)};
    independent.emergency = true;
    const auto other = fixture.host.submit(independent, {}, 111);
    require(other.accepted(), "Independent jettison fixture must be queued");
    require(fixture.host.releaseRefused(request.activity, ms::Reason::OutOfReach, 111),
            "A live native pre-launch refusal must be handled");
    const auto* activity = fixture.host.activity(request.activity);
    const auto status = fixture.host.strikeStatus(request.activity);
    require(activity && activity->state == control::ActivityState::Failed
                && activity->reason == ms::Reason::OutOfReach,
            "Native release refusal must retain its actual reason rather than StoreNotFlyable");
    require(status && status->weapons == std::vector<std::uint32_t>{123},
            "A failed later ripple must preserve all previously released weapon correlations");
    const auto remaining = fixture.host.takeReleases();
    require(remaining.size() == 1 && remaining.front().activity == other.activity && remaining.front().jettisoned,
            "Refusal cleanup must remove only its own unexecuted releases");
    for (const auto& store : fixture.host.stores()) {
        if (store.station == 1) require(store.status == ms::StoreStatus::Released && store.weapon == 123,
                "An earlier released store must remain released");
        if (store.station == 2) require(store.status == ms::StoreStatus::Loaded && store.weapon == 0,
                "A refused pending store must stay aboard and become free");
    }
    require(!fixture.host.releaseRefused(request.activity, ms::Reason::UnknownTrack, 112)
                && fixture.host.activity(request.activity)->reason == ms::Reason::OutOfReach,
            "Repeated execution refusal cannot rewrite a terminal activity");
}

void queued_cancel_and_jettison_contracts_are_unchanged() {
    HostFixture fixture;
    fixture.operate();
    ms::StrikeCommand strike;
    strike.payload = fixture.payload;
    strike.stores = {fixture.index(1), fixture.index(2)};
    strike.consent = true;
    strike.rippleS = 0;
    const auto request = fixture.host.submit(strike, {}, 100);
    require(request.accepted(), "Private cancel fixture must start");
    fixture.host.step(101);
    require(fixture.host.cancel(control::Caller{control::Source::Policy, 0}, request.activity, 101).status
                == control::CommandStatus::Canceled, "Private queued strike cancel failed");
    require(!fixture.host.releaseRefused(request.activity, ms::Reason::OutOfReach, 101),
            "An execution refusal must not rewrite a canceled activity");
    const auto queued = fixture.host.takeReleases();
    require(queued.size() == 2, "Already queued releases must retain their existing cancellation contract");
    ms::JettisonCommand jettison;
    jettison.payload = fixture.payload;
    jettison.stores = {fixture.index(3)};
    jettison.emergency = true;
    const auto other = fixture.host.submit(jettison, {}, 101);
    require(other.accepted() && !fixture.host.releaseRefused(other.activity, ms::Reason::OutOfReach, 101),
            "AAM eligibility refusal must not intercept native jettison");
    const auto inert = fixture.host.takeReleases();
    require(inert.size() == 1 && inert.front().jettisoned && inert.front().launch.inert,
            "Jettison must preserve its inert native release");
}
} // namespace

int main(int argc, char** argv) {
    const struct { const char* name; void (*run)(); } tests[] = {
        {"selected atomic", all_selected_aams_are_validated_atomically},
        {"selected gone", gone_aam_cannot_supply_another_stores_launch_rules},
        {"ready infrared", ready_infrared_store_retains_on_rail_observation},
        {"release fov loss", release_time_fov_loss_refuses_every_due_store_atomically},
        {"release illumination loss", release_time_illumination_loss_preserves_native_refusal},
        {"agent infrared cue", agent_designation_does_not_replace_first_infrared_observation_identity},
        {"metadata invalid", malformed_explicit_guidance_metadata_is_refused},
        {"metadata legacy", legacy_card_still_parses_without_new_equipment_claims},
        {"metadata explicit", explicit_guidance_metadata_preserves_independent_capabilities},
        {"metadata fleet", maintained_aam_declarations_are_explicit_assumptions},
        {"selection preview", preview_is_pure_and_matches_committed_selection},
        {"release refusal", refusal_preserves_reason_prior_weapons_and_unrelated_queue},
        {"queued cancellation", queued_cancel_and_jettison_contracts_are_unchanged},
    };
    bool selected = argc == 1;
    for (const auto& test : tests) {
        if (argc > 1 && std::string(argv[1]) != test.name) continue;
        selected = true;
        try { test.run(); std::cout << "PASS " << test.name << '\n'; }
        catch (const std::exception& error) {
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
            return 1;
        }
    }
    if (!selected) { std::cerr << "No matching weapon employment case\n"; return 2; }
    return 0;
}
