// Production-path finite missile control checks. Numerical limits are read
// from the model; this fixture admits no real equipment calibration.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/NativeStepObservationV1.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <filesystem>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT "D:/Projects/Cpp/Super-DDCCSS-WWorld/third_party/jsbsim"
#endif
#ifndef FSIM_TEST_AIRCRAFT_DIR
#define FSIM_TEST_AIRCRAFT_DIR "D:/Projects/Cpp/Super-DDCCSS-WWorld/aircraft"
#endif

using namespace fsim;

namespace {
constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180;
constexpr double kNumericalTolerance = 1e-10;
constexpr unsigned kSamples = 180;
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
session::WorldOptions options(unsigned workers) {
    session::WorldOptions o;
    o.name = "native-missile-control-" + std::to_string(workers);
    o.workers = workers; o.pinWorkers = false; o.publish = false; o.seed = 17;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    return o;
}
void advance(session::World& world, double seconds) {
    world.step(static_cast<unsigned>(std::lround(seconds / (world.dt() * world.frameSkip()))));
}
std::uint16_t payload(session::World& world, std::uint32_t id, ms::Family family) {
    const auto* host = world.missionSystems(id);
    require(host != nullptr, "vehicle mission systems missing");
    for (const auto& value : host->payloads()) if (value.family == family) return value.index;
    throw std::runtime_error("required native payload missing");
}
void operate(session::World& world, std::uint32_t id, std::uint16_t index) {
    const auto result = world.submit(id, ms::StateCommand{index, ms::PayloadState::Operate});
    require(result.accepted() || result.reason == ms::Reason::AlreadyInState, "payload operation rejected");
    for (unsigned i = 0; i < 1200 && world.missionSystems(id)->payloads()[index].state != ms::PayloadState::Operate; ++i)
        world.step();
    require(world.missionSystems(id)->payloads()[index].state == ms::PayloadState::Operate, "payload warmup timed out");
}
std::uint32_t createAircraft(session::World& world, const std::string& name, const sim::InitialConditions& initial) {
    session::VehicleSpec spec;
    spec.name = name; spec.type = "jsbsim:f16c"; spec.initial = initial;
    const auto id = world.createVehicle(spec);
    require(id != 0, "native F-16C creation failed");
    control::VelocityCommand hold;
    hold.airspeedMs = initial.airspeedTrueMs; hold.verticalSpeedMs = 0; hold.headingRad = initial.headingDeg * kDeg;
    require(world.submit(id, hold).accepted(), "native aircraft hold rejected");
    return id;
}

struct Fixture {
    session::World world;
    std::uint32_t missile = 0;
    explicit Fixture(unsigned workers,const std::string& store="aim-120c",double targetEastM=10000) : world(options(workers)) {
        sim::InitialConditions initial;
        initial.altitudeMslM = 6000; initial.airspeedTrueMs = 250; initial.headingDeg = 90;
        const auto launcher = createAircraft(world, "launcher", initial);
        const auto stations = payload(world, launcher, ms::Family::StoreStation);
        const auto radar = payload(world, launcher, ms::Family::AirRadar);
        require(world.loadStore(launcher, stations, 1, store) == ms::Reason::None, "fixture store load rejected");
        operate(world, launcher, stations); operate(world, launcher, radar);
        advance(world, 10);
        const auto launchState = *world.vehicleState(launcher);
        require(!launchState.crashed && !launchState.diverged, "launcher setup became invalid");
        initial.latitudeDeg = launchState.latitudeRad / kDeg + 1000 / 6371000.0 / kDeg;
        initial.longitudeDeg = launchState.longitudeRad / kDeg + targetEastM / (6371000 * std::cos(launchState.latitudeRad)) / kDeg;
        initial.headingDeg = 0;
        const auto target = createAircraft(world, "crossing-target", initial);
        ms::AmtiCommand scan;
        scan.payload = radar; scan.azimuthHalfWidthRad = 15 * kDeg; scan.bars = 1; scan.rangeScaleM = 60000;
        require(world.submit(launcher, scan).accepted(), "native radar scan rejected");
        std::uint64_t track = 0;
        for (unsigned i = 0; i < 360 && !track; ++i) {
            world.step();
            for (const auto& observation : world.amtiTracks(launcher, radar))
                if (observation.truth == target && observation.updates >= 3) { track = observation.id; break; }
        }
        require(track != 0, "crossing target did not produce a confirmed measured radar track");
        // Truth identifies this controlled fixture only. The weapon consumes
        // the measured track's designation through the normal release path.
        const auto designation = world.designate(launcher, radar, track);
        require(designation >= session::World::kDesignations, "measured track designation rejected");
        ms::AmtiCommand stt;
        stt.payload = radar; stt.task = ms::AmtiTask::Track; stt.mode = ms::AmtiMode::SingleTargetTrack;
        stt.track = track; stt.rangeScaleM = 60000;
        require(world.submit(launcher, stt).accepted(), "native target tracking rejected");
        ms::StrikeCommand strike;
        strike.payload = stations; strike.store = store; strike.target = designation;
        const auto request = world.submit(launcher, strike);
        require(request.accepted(), "native strike request rejected");
        for (unsigned i = 0; i < 240; ++i) {
            const auto status = world.strikeStatus(request.activity);
            require(status.has_value(), "native strike activity missing");
            if (status->phase == ms::StrikePhase::AwaitingConsent) break;
            world.step();
        }
        require(world.strikeStatus(request.activity)->phase == ms::StrikePhase::AwaitingConsent,
                "native strike did not reach consent within fixture bound");
        require(world.strikeConsent(request.activity, true) == ms::Reason::None, "native release consent rejected");
        for (unsigned i = 0; i < 30 && world.weapons().empty(); ++i) world.step();
        require(world.weapons().size() == 1, "native consent must release exactly one missile");
        const auto record = world.weapons().front();
        missile = record.id;
        require(!record.jettisoned && record.seeker == ((store=="aim-9x"||store=="pl-10")?"imaging_infrared":"active_radar"), "fixture did not release an armed guided missile");
    }
    sim::PropertyHandle property(const char* name) {
        auto* model = world.model(missile);
        require(model != nullptr, "released native model missing");
        const auto handle = model->property(name);
        require(handle.valid(), std::string("required missile control property missing: ") + name);
        return handle;
    }
};

struct Telemetry {
    struct Fin {
        sim::PropertyHandle position, demand, minimum, maximum, rate;
        std::array<sim::PropertyHandle, 3> force;
    };
    std::array<sim::PropertyHandle, 3> position, command;
    std::array<sim::PropertyHandle, 3> finForce, finMoment;
    std::array<sim::PropertyHandle, 2> external;
    sim::PropertyHandle limit, rate, lag;
    std::vector<Fin> fins;
    explicit Telemetry(Fixture& fixture) {
        position = {fixture.property("fcs/elevator-pos-rad"), fixture.property("fcs/rudder-pos-rad"),
                    fixture.property("fcs/left-aileron-pos-rad")};
        command = {fixture.property("fcs/elevator-cmd-norm"), fixture.property("fcs/rudder-cmd-norm"),
                   fixture.property("fcs/aileron-cmd-norm")};
        limit = fixture.property("fsim/missile-control/surface-limit-rad");
        rate = fixture.property("fsim/missile-control/surface-rate-rad_s");
        lag = fixture.property("fsim/missile-control/lag-coefficient-s-1");
        external = {fixture.property("external_reactions/fsim/magnitude"),
                    fixture.property("external_reactions/fsim/magnitude-lbsft")};
        finForce = {fixture.property("aero/force/fsim-fin-x-lbs"), fixture.property("aero/force/fsim-fin-y-lbs"),
                    fixture.property("aero/force/fsim-fin-z-lbs")};
        finMoment = {fixture.property("aero/moment/fsim-fin-l-lbsft"), fixture.property("aero/moment/fsim-fin-m-lbsft"),
                     fixture.property("aero/moment/fsim-fin-n-lbsft")};
        require(std::isfinite(limit.get()) && limit.get() > 0 && std::isfinite(rate.get()) && rate.get() > 0,
                "model must declare positive finite assumed surface stops and rate limits");
        require(rate.get() == 4 && lag.get() == 25,
                "native acceptance freezes the declared assumed 4 rad/s rate and 25 1/s lag coefficient");
        const double count = fixture.property("fsim/missile-control/fin-count").get();
        require(std::isfinite(count) && count > 0 && count <= std::numeric_limits<unsigned>::max() && std::floor(count) == count,
                "native model must declare a nonzero integral controlled-fin count");
        for (unsigned index = 0; index < static_cast<unsigned>(count); ++index) {
            const auto fcs = "fcs/fin-" + std::to_string(index);
            const auto metadata = "fsim/missile-control/fin-" + std::to_string(index);
            const auto output = "fsim/fin-control/fin-" + std::to_string(index);
            Fin fin;
            fin.position = fixture.property((fcs + "-delta-rad").c_str());
            fin.demand = fixture.property((fcs + "-demand-rad").c_str());
            fin.minimum = fixture.property((metadata + "-min-rad").c_str());
            fin.maximum = fixture.property((metadata + "-max-rad").c_str());
            fin.rate = fixture.property((metadata + "-rate-rad_s").c_str());
            fin.force = {fixture.property((output + "/force-x").c_str()), fixture.property((output + "/force-y").c_str()),
                         fixture.property((output + "/force-z").c_str())};
            require(std::isfinite(fin.minimum.get()) && std::isfinite(fin.maximum.get())
                        && fin.minimum.get() < fin.maximum.get() && fin.rate.get() == 4,
                    "every actual mixed fin needs its authored stops and the frozen assumed 4 rad/s rate");
            fins.push_back(fin);
        }
    }
    void noIdealActuation() const {
        for (const auto handle : external)
            require(std::isfinite(handle.get()) && std::abs(handle.get()) <= kNumericalTolerance,
                    "guided missile must not receive ideal external force or moment");
    }
    std::array<double, 3> positions() const { return {position[0].get(), position[1].get(), position[2].get()}; }
};

using Sample = std::vector<double>;
std::vector<Sample> healthyHistory(unsigned workers) {
    Fixture fixture(workers);
    Telemetry telemetry(fixture);
    auto previous = telemetry.positions();
    for (const double value : previous) require(std::abs(value) <= kNumericalTolerance,
            "release initialization must leave all actual control surfaces neutral");
    std::vector<double> previousFins;
    for (const auto& fin : telemetry.fins) {
        require(std::abs(fin.position.get()) <= kNumericalTolerance, "release must initialize every physical mixed fin neutral");
        previousFins.push_back(fin.position.get());
    }
    std::vector<Sample> history;
    double maximumPosition = 0, maximumCommand = 0, maximumFinForce = 0, maximumFinMoment = 0;
    const double dt = fixture.world.dt() * fixture.world.frameSkip();
    for (unsigned i = 0; i < kSamples; ++i) {
        fixture.world.step();
        require(fixture.world.alive(fixture.missile), "control fixture missile ended before bounded samples completed");
        telemetry.noIdealActuation();
        const auto current = telemetry.positions();
        for (std::size_t axis = 0; axis < current.size(); ++axis) {
            require(std::isfinite(current[axis]) && std::abs(current[axis]) <= telemetry.limit.get() + kNumericalTolerance,
                    "actual fin position must remain inside the model's declared stop");
            require(std::abs(current[axis] - previous[axis]) <= telemetry.rate.get() * dt + kNumericalTolerance,
                    "actual fin motion, including its first powered step, must respect the declared rate limit");
            maximumPosition = std::max(maximumPosition, std::abs(current[axis]));
            maximumCommand = std::max(maximumCommand, std::abs(telemetry.command[axis].get()));
            require(std::isfinite(telemetry.finForce[axis].get()) && std::isfinite(telemetry.finMoment[axis].get()),
                    "native fin aerodynamic force/moment must remain finite");
            maximumFinForce = std::max(maximumFinForce, std::abs(telemetry.finForce[axis].get()));
            maximumFinMoment = std::max(maximumFinMoment, std::abs(telemetry.finMoment[axis].get()));
        }
        const auto* state = fixture.world.vehicleState(fixture.missile);
        require(state && !state->crashed && !state->diverged, "finite control fixture flight became invalid");
        Sample sample(15);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            sample[axis] = current[axis];
            sample[3 + axis] = telemetry.command[axis].get();
            sample[6 + axis] = state->positionEcef[axis];
            sample[9 + axis] = state->velocityBodyMs[axis];
            sample[12 + axis] = state->angularRateBodyRadS[axis];
        }
        for (std::size_t index = 0; index < telemetry.fins.size(); ++index) {
            const auto& fin = telemetry.fins[index];
            const double position = fin.position.get();
            require(std::isfinite(position) && position >= fin.minimum.get() - kNumericalTolerance
                        && position <= fin.maximum.get() + kNumericalTolerance,
                    "actual mixed fin must remain inside its own authored stops");
            require(std::abs(position - previousFins[index]) <= fin.rate.get() * dt + kNumericalTolerance,
                    "each physical fin, after all axis mixing, must obey its own 4 rad/s rate limit");
            sample.push_back(position);
            sample.push_back(fin.demand.get());
            previousFins[index] = position;
        }
        history.push_back(sample);
        previous = current;
    }
    require(maximumCommand > 1e-6 && maximumPosition > 1e-6,
            "the measured crossing target must exercise actual guided fin motion");
    require(maximumFinForce > 1e-6 && maximumFinMoment > 1e-6,
            "moving guided fins must produce actual JSBSim aerodynamic force and moment");
    return history;
}
void finiteSurfaceResponse() { (void)healthyHistory(1); }
void stuckSurfacesCannotFollowGuidance() {
    Fixture fixture(1);
    Telemetry telemetry(fixture);
    const auto neutral = telemetry.positions();
    for (const double value : neutral) require(std::abs(value) <= kNumericalTolerance, "pre-failure fins must be neutral");
    auto* model = fixture.world.model(fixture.missile);
    require(model->failChannel("elevator") > 0 && model->failChannel("rudder") > 0 && model->failChannel("aileron") > 0,
            "native channel failure must find every generated missile actuator");
    double maximumCommand = 0;
    for (unsigned i = 0; i < kSamples; ++i) {
        fixture.world.step();
        require(fixture.world.alive(fixture.missile), "stuck-neutral fixture missile ended before bounded samples completed");
        telemetry.noIdealActuation();
        for (std::size_t axis = 0; axis < neutral.size(); ++axis) {
            require(std::abs(telemetry.position[axis].get() - neutral[axis]) <= kNumericalTolerance,
                    "native fail-stuck surfaces must not keep following guidance commands");
            require(std::isfinite(telemetry.finForce[axis].get()) && std::abs(telemetry.finForce[axis].get()) <= kNumericalTolerance
                    && std::isfinite(telemetry.finMoment[axis].get()) && std::abs(telemetry.finMoment[axis].get()) <= kNumericalTolerance,
                    "stuck-neutral fins cannot keep producing commanded control force or moment");
            maximumCommand = std::max(maximumCommand, std::abs(telemetry.command[axis].get()));
        }
    }
    require(maximumCommand > 1e-6, "stuck-neutral counterfactual must retain a nonzero guidance demand");
}
void individualPhysicalFinFailure() {
    Fixture fixture(1);
    Telemetry telemetry(fixture);
    require(telemetry.fins.size() > 1, "individual failure fixture requires multiple actual controlled fins");
    require(fixture.world.model(fixture.missile)->failChannel("fin-0") == 1,
            "individual fin failure must bind exactly one physical native actuator");
    bool otherFinMoved = false, failedFinDemanded = false;
    for (unsigned step = 0; step < kSamples; ++step) {
        fixture.world.step();
        require(fixture.world.alive(fixture.missile), "individual-fin fixture ended before bounded samples completed");
        telemetry.noIdealActuation();
        const auto& failed = telemetry.fins.front();
        require(std::abs(failed.position.get()) <= kNumericalTolerance,
                "failed physical fin must remain neutral despite live mixed demand");
        for (const auto force : failed.force)
            require(std::isfinite(force.get()) && std::abs(force.get()) <= kNumericalTolerance,
                    "a stuck-neutral individual fin cannot retain its control-induced force");
        failedFinDemanded = failedFinDemanded || std::abs(failed.demand.get()) > 1e-6;
        for (std::size_t index = 1; index < telemetry.fins.size(); ++index)
            otherFinMoved = otherFinMoved || std::abs(telemetry.fins[index].position.get()) > 1e-6;
    }
    require(failedFinDemanded && otherFinMoved,
            "individual failure must retain mixed command demand and the other native fins' response");
}
void workerDeterminism() {
    const auto one = healthyHistory(1);
    const auto two = healthyHistory(2);
    require(one == two, "one/two-worker native fin and trajectory histories must be identical");
}
void jammedPhysicalFinsReportActualPositions() {
    Fixture fixture(1);
    Telemetry telemetry(fixture);
    for (std::size_t index = 0; index < telemetry.fins.size(); ++index)
        require(fixture.world.model(fixture.missile)->failChannel("fin-" + std::to_string(index)) == 1,
                "all-fin reporting fixture must jam each actual native fin actuator");
    bool commanded = false;
    for (unsigned step = 0; step < 60; ++step) {
        fixture.world.step();
        telemetry.noIdealActuation();
        const auto* state = fixture.world.vehicleState(fixture.missile);
        require(state && !state->diverged, "all-fin reporting fixture must retain valid native state");
        for (std::size_t axis = 0; axis < 3; ++axis) {
            commanded = commanded || std::abs(telemetry.command[axis].get()) > 1e-6;
            require(std::abs(telemetry.position[axis].get()) <= kNumericalTolerance,
                    "reported standard positions must stay neutral when every force-driving physical fin is jammed neutral");
        }
        require(std::abs(state->elevatorRad) <= kNumericalTolerance && std::abs(state->rudderRad) <= kNumericalTolerance &&
                    std::abs(state->aileronRad) <= kNumericalTolerance,
                "VehicleState must report actual fin-equivalent positions rather than filtered channel demand");
    }
    require(commanded, "all-fin reporting fixture must exercise a nonzero controller request");
}
sim::PropertyHandle nativeProperty(sim::FlightModel& model, const char* name) {
    const auto handle = model.property(name);
    require(handle.valid(), std::string("required native plant property missing: ") + name);
    return handle;
}
sim::InitialConditions plantInitialConditions() {
    sim::InitialConditions initial;
    initial.altitudeMslM = 6000; initial.airspeedTrueMs = 350;
    initial.headingDeg = 90; initial.enginesRunning = false;
    return initial;
}
sim::AircraftSpec plantSpec(const std::string& name) {
    return {name, FSIM_TEST_JSBSIM_ROOT, std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "stores"};
}
struct FirstNativeForceSample {
    std::array<sim::PropertyHandle, 8> properties;
    std::array<double, 8> first{};
    unsigned samples = 0, expected = 0;
    bool ordered = true;
    explicit FirstNativeForceSample(sim::JsbsimModel& model) {
        const char* names[] = {"aero/qbar-psf", "fcs/elevator-pos-rad", "aero/force/fsim-fin-z-lbs",
                              "velocities/mach", "aero/alpha-rad", "aero/beta-rad",
                              "fcs/aileron-pos-rad", "fcs/rudder-pos-rad"};
        for (std::size_t i = 0; i < properties.size(); ++i) properties[i] = nativeProperty(model, names[i]);
        const double nativeDt = nativeProperty(model, "simulation/dt").get();
        require(std::isfinite(nativeDt) && nativeDt > 0, "qbar sample requires a valid native integration interval");
        const double count = model.dt() / nativeDt;
        require(count >= 1 && count <= 4096 && std::abs(count - std::round(count)) < 1e-8,
                "qbar sample requires a complete integer native-step schedule");
        expected = static_cast<unsigned>(std::lround(count));
    }
    static void observe(void* context, unsigned substep, unsigned substeps) noexcept {
        auto& sample = *static_cast<FirstNativeForceSample*>(context);
        sample.ordered = sample.ordered && substep == sample.samples + 1 && substeps == sample.expected;
        if (sample.samples == 0)
            for (std::size_t i = 0; i < sample.first.size(); ++i) sample.first[i] = sample.properties[i].get();
        ++sample.samples;
    }
    void advance(sim::JsbsimModel& model, const sim::ControlInputs& input) {
        {
            sim::ScopedNativeStepObserverV1 observer({&model, this, observe});
            model.step(input); // Execute the entire original public step, including all later native samples.
        }
        require(ordered && samples == expected, "qbar observation must retain every native integration step");
        for (const double value : first) require(std::isfinite(value), "first native force sample must be finite");
    }
};
void aerodynamicResponseDependsOnQbar() {
    const double dt = 1.0 / 120;
    auto ground = std::make_shared<sim::FlatGround>(0);
    sim::JsbsimModel ordinary(dt, ground), thin(dt, ground);
    const auto initial = plantInitialConditions();
    require(ordinary.load(plantSpec("aim120c"), initial) && thin.load(plantSpec("aim120c"), initial),
            "qbar comparison requires matched native control plants");
    // Both native plants have identical IC/neutral FCS priming and no motor.
    // Only density changes before the first same-command native force sample;
    // no evolved Mach/alpha/beta state or controller decision is compared.
    thin.setAtmosphere(288.15, 101325.0 / 2);
    sim::ControlInputs input;
    input.gearDown = 0; input.elevator = .05;
    FirstNativeForceSample ordinarySample(ordinary), thinSample(thin);
    ordinarySample.advance(ordinary, input); thinSample.advance(thin, input);
    for (std::size_t i = 3; i < ordinarySample.first.size(); ++i)
        require(ordinarySample.first[i] == thinSample.first[i],
                "qbar comparison requires exactly matched sampled Mach, alpha, beta and lateral fin positions");
    const double ordinaryQbar = ordinarySample.first[0];
    const double thinQbar = thinSample.first[0];
    require(std::isfinite(ordinaryQbar) && ordinaryQbar > 0 && std::isfinite(thinQbar)
                && thinQbar > 0 && thinQbar < .75 * ordinaryQbar,
            "native qbar must respond substantially to the synthetic pressure change");
    const auto ordinaryPosition = ordinarySample.first[1];
    const auto thinPosition = thinSample.first[1];
    require(ordinaryPosition != 0 && ordinaryPosition == thinPosition,
            "qbar comparison must use identical actual finite fin deflections");
    const double ordinaryForce = ordinarySample.first[2];
    const double thinForce = thinSample.first[2];
    require(std::isfinite(ordinaryForce) && std::abs(ordinaryForce) > 1e-6 && std::isfinite(thinForce),
            "qbar comparison must exercise actual native fin aerodynamic force");
    const double a = ordinaryForce / ordinaryQbar, b = thinForce / thinQbar;
    require(std::abs(a - b) <= 1e-8 * std::max(1.0, std::abs(a)),
            "matched first-step native fin force must scale with measured qbar");
    const double finalA = nativeProperty(ordinary, "aero/force/fsim-fin-z-lbs").get() /
                          nativeProperty(ordinary, "aero/qbar-psf").get();
    const double finalB = nativeProperty(thin, "aero/force/fsim-fin-z-lbs").get() /
                          nativeProperty(thin, "aero/qbar-psf").get();
    std::cout << "QBAR: first-state difference=" << std::abs(a - b)
              << "; evolved end-step difference=" << std::abs(finalA - finalB)
              << "; complete native samples=" << ordinarySample.samples << ',' << thinSample.samples << '\n';
    for (auto* model : {&ordinary, &thin}) {
        require(std::abs(nativeProperty(*model, "external_reactions/fsim/magnitude").get()) <= kNumericalTolerance
                    && std::abs(nativeProperty(*model, "external_reactions/fsim/magnitude-lbsft").get()) <= kNumericalTolerance,
                "matched qbar plants must not use ideal external reactions");
    }
}
void isolatedPlantSigns(const std::string& name) {
    const double dt = 1.0 / 120;
    auto ground = std::make_shared<sim::FlatGround>(0);
    sim::JsbsimModel model(dt, ground);
    const auto initial = plantInitialConditions();
    require(model.load(plantSpec(name), initial), name + ": native control plant failed to load");
    const auto position = nativeProperty(model, "fcs/elevator-pos-rad");
    const auto derivative = nativeProperty(model, "fsim/control/cm-elevator");
    const auto moment = nativeProperty(model, "aero/moment/fsim-fin-m-lbsft");
    const auto force = nativeProperty(model, "aero/force/fsim-fin-z-lbs");
    const auto rate = nativeProperty(model, "fsim/missile-control/surface-rate-rad_s");
    const auto limit = nativeProperty(model, "fsim/missile-control/surface-limit-rad");
    const auto lag = nativeProperty(model, "fsim/missile-control/lag-coefficient-s-1");
    const auto externalForce = nativeProperty(model, "external_reactions/fsim/magnitude");
    const auto externalMoment = nativeProperty(model, "external_reactions/fsim/magnitude-lbsft");
    require(std::isfinite(derivative.get()) && derivative.get() != 0, name + ": missing signed pitch authority");
    require(rate.get() == 4 && lag.get() == 25, name + ": assumed native actuator rate/lag changed outside acceptance");
    for (const double sign : {1.0, -1.0}) {
        require(model.reset(initial), name + ": native plant reset failed");
        require(std::abs(position.get()) <= kNumericalTolerance, name + ": reset must restore neutral actual fin position");
        sim::ControlInputs input;
        input.gearDown = 0; input.elevator = sign * .05;
        double previous = position.get();
        bool response = false;
        for (unsigned step = 0; step < 8; ++step) {
            model.step(input);
            const double current = position.get(), actualMoment = moment.get();
            require(std::isfinite(current) && current * sign > 0 && std::abs(current) <= limit.get() + kNumericalTolerance,
                    name + ": actual fin must follow the isolated command within its stop");
            require(std::abs(current - previous) <= rate.get() * dt + kNumericalTolerance,
                    name + ": direct native first/later surface steps must obey rate limits");
            if (step == 0) require(std::abs(current) < std::abs(input.elevator) * limit.get(),
                    name + ": finite native lag must prevent an instantaneous full small-command deflection");
            require(std::isfinite(actualMoment) && actualMoment * derivative.get() * current > 0,
                    name + ": pitch fin moment must have the declared native BODY derivative sign for both commands");
            require(std::isfinite(force.get()), name + ": actual fin force must be finite");
            response = response || std::abs(force.get()) > 1e-6;
            require(std::abs(externalForce.get()) <= kNumericalTolerance && std::abs(externalMoment.get()) <= kNumericalTolerance,
                    name + ": FCS-only plant must use aerodynamic force/moment rather than external reactions");
            sim::VehicleState state;
            model.state(state);
            require(!state.diverged && !state.crashed && std::isfinite(state.angularRateBodyRadS[1]),
                    name + ": isolated finite-control native flight must remain numerically valid");
            previous = current;
        }
        require(response, name + ": isolated native elevator must produce an actual aerodynamic force");
    }
}
void tailPlant() { isolatedPlantSigns("aim120c"); }
void canardPlant() { isolatedPlantSigns("r73"); }
void movingWingPlant() { isolatedPlantSigns("aim7m"); }
void currentFleetPlants() {
    // Maintained 24-AAM roster: software authority checks, not calibration.
    for (const char* name : {"aim120c", "aim120d", "aim7m", "aim9m", "aim9x", "asraam", "irist", "magic2",
            "meteor", "micaem", "micair", "pl10", "pl12", "pl15", "pl8", "r27er", "r27et", "r27r", "r27t",
            "r37m", "r60m", "r73", "r77m", "super530d"}) isolatedPlantSigns(name);
}

void nativeControlResetReplaysFreshModel() {
    const double dt = 1.0 / 120;
    auto ground = std::make_shared<sim::FlatGround>(0);
    sim::JsbsimModel used(dt, ground), fresh(dt, ground);
    const auto initial = plantInitialConditions();
    require(used.load(plantSpec("aim120c"), initial) && fresh.load(plantSpec("aim120c"), initial),
            "reset comparison requires two native AIM-120C control plants");
    sim::ControlInputs history;
    history.gearDown = 0; history.elevator = .3; history.rudder = -.2; history.aileron = .1;
    for (unsigned i = 0; i < 24; ++i) used.step(history);
    require(used.failChannel("elevator") > 0 && used.failChannel("rudder") > 0 && used.failChannel("aileron") > 0,
            "reset comparison must include real native actuator failures");
    require(used.failChannel("fin-0") == 1, "reset comparison must include an individual physical fin failure");
    for (unsigned i = 0; i < 4; ++i) used.step(history);
    require(used.reset(initial), "used native control plant reset failed");
    const char* paths[] = {"fcs/elevator-pos-rad", "fcs/rudder-pos-rad", "fcs/left-aileron-pos-rad",
                          "fcs/elevator-actuator/malfunction/fail_stuck", "fcs/rudder-actuator/malfunction/fail_stuck",
                          "fcs/aileron-actuator/malfunction/fail_stuck", "aero/force/fsim-fin-y-lbs", "aero/force/fsim-fin-z-lbs",
                          "aero/moment/fsim-fin-m-lbsft", "aero/moment/fsim-fin-n-lbsft"};
    std::vector<sim::PropertyHandle> a, b;
    for (const char* path : paths) { a.push_back(nativeProperty(used, path)); b.push_back(nativeProperty(fresh, path)); }
    const double finCount = nativeProperty(used, "fsim/missile-control/fin-count").get();
    require(std::isfinite(finCount) && finCount > 0 && finCount <= std::numeric_limits<unsigned>::max()
                && std::floor(finCount) == finCount, "reset comparison requires valid physical fin metadata");
    for (unsigned index = 0; index < static_cast<unsigned>(finCount); ++index) {
        const auto prefix = "fcs/fin-" + std::to_string(index);
        for (const auto& path : {prefix + "-delta-rad", prefix + "-actuator/malfunction/fail_stuck"}) {
            a.push_back(nativeProperty(used, path.c_str()));
            b.push_back(nativeProperty(fresh, path.c_str()));
            require(std::abs(a.back().get()) <= kNumericalTolerance && a.back().get() == b.back().get(),
                    "reset must clear physical mixed-fin positions and failures as a fresh native model");
        }
    }
    for (std::size_t index = 0; index < 6; ++index)
        require(std::abs(a[index].get()) <= kNumericalTolerance && a[index].get() == b[index].get(),
                "reset must clear actual fin positions and failure flags exactly as a fresh native model");
    std::array<double, 3> previous{};
    const auto rate = nativeProperty(used, "fsim/missile-control/surface-rate-rad_s");
    for (unsigned step = 0; step < 24; ++step) {
        sim::ControlInputs input;
        input.gearDown = 0;
        input.elevator = step < 12 ? .1 : -.1;
        input.rudder = step < 12 ? -.08 : .08;
        used.step(input); fresh.step(input);
        for (std::size_t index = 0; index < a.size(); ++index)
            require(std::isfinite(a[index].get()) && a[index].get() == b[index].get(),
                    "used/reset FCS output and actual aerodynamic response must replay the fresh native model exactly");
        for (std::size_t index = 0; index < previous.size(); ++index) {
            require(std::abs(a[index].get() - previous[index]) <= rate.get() * dt + kNumericalTolerance,
                    "reset must not reintroduce a first-run surface rate-limit bypass");
            previous[index] = a[index].get();
        }
        sim::VehicleState usedState, freshState;
        used.state(usedState); fresh.state(freshState);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            require(usedState.positionEcef[axis] == freshState.positionEcef[axis]
                    && usedState.velocityBodyMs[axis] == freshState.velocityBodyMs[axis]
                    && usedState.angularRateBodyRadS[axis] == freshState.angularRateBodyRadS[axis],
                    "native reset/replay kinematics must match the fresh control plant exactly");
        }
    }
}
}

namespace {
void localDerivativePlantRetainsActualFinTelemetry() {
    Fixture fixture(1,"pl-10",4500.);
    auto version=fixture.property("fsim/missile-control/version");
    require(version.get()==2,"Local incidence controller requires native derivative metadata version 2");
    for(unsigned i=0;i<30;++i) {
        fixture.world.step();const auto read=fixture.world.weaponControlsV1(fixture.missile);
        require(read && read->available && read->controllerAvailable && read->fins.size()==4,
            "Local derivative selection must retain all four actual fin identities and telemetry");
        for(const auto& fin:read->fins)require(fin.valid,"Local derivative model requires real valid native fin samples");
    }
    version.set(1);fixture.world.step();
    const auto rejected=fixture.world.weaponControlsV1(fixture.missile);
    require(rejected && !rejected->available && !rejected->controllerAvailable,
        "Wrong native metadata must not silently select version-1 telemetry/control");
}
void nativeVectoringNozzleConservesThrustAndProducesCurrentCgTorque() {
    Fixture fixture(1,"aim-9x",4500.);
    Telemetry telemetry(fixture);
    const auto pitch=fixture.property("propulsion/engine/pitch-angle-rad");
    const auto yaw=fixture.property("propulsion/engine/yaw-angle-rad");
    const auto limit=fixture.property("fsim/thrust-vector-control/limit_rad");
    const auto rate=fixture.property("fsim/thrust-vector-control/rate_rad_s");
    const auto origin=fixture.property("fsim/model/origin-x-in");
    const auto nozzle=fixture.property("propulsion/engine/x-position");
    const std::array<sim::PropertyHandle,3> force={fixture.property("forces/fbx-prop-lbs"),
        fixture.property("forces/fby-prop-lbs"),fixture.property("forces/fbz-prop-lbs")};
    const std::array<sim::PropertyHandle,3> moment={fixture.property("moments/l-prop-lbsft"),
        fixture.property("moments/m-prop-lbsft"),fixture.property("moments/n-prop-lbsft")};
    double previousPitch=pitch.get(),previousYaw=yaw.get(),maximum=0.;
    const double dt=fixture.world.dt()*fixture.world.frameSkip();
    for(unsigned i=0;i<24;++i) {
        fixture.world.step();telemetry.noIdealActuation();
        const auto* state=fixture.world.vehicleState(fixture.missile);
        require(state && !state->diverged && !state->crashed,"native vector fixture must remain physically live");
        const double p=pitch.get(),y=yaw.get();maximum=std::max({maximum,std::abs(p),std::abs(y)});
        require(std::abs(p)<=limit.get()+1e-12 && std::abs(y)<=limit.get()+1e-12,
            "actual native nozzle must obey its explicit finite stops");
        require(std::abs(p-previousPitch)<=rate.get()*dt+1e-10 && std::abs(y-previousYaw)<=rate.get()*dt+1e-10,
            "actual native nozzle motion must obey its finite rate");
        const double thrust=state->thrustN[0];
        const std::array<double,3> expected={thrust*std::cos(p)*std::cos(y),thrust*std::cos(p)*std::sin(y),-thrust*std::sin(p)};
        std::array<double,3> actual{};
        const double tolerance=1e-8*std::max(1.,thrust);
        for(std::size_t k=0;k<3;++k) {
            actual[k]=force[k].get()*4.4482216152605;
            require(std::abs(actual[k]-expected[k])<=tolerance,"nozzle orientation must rotate native thrust without adding energy or force");
        }
        double cg[3]{};require(fixture.world.model(fixture.missile)->cgFromOriginBodyM(cg),"current native CG required");
        const std::array<double,3> arm={-(nozzle.get()-origin.get())*.0254-cg[0],-cg[1],-cg[2]};
        const std::array<double,3> torque={arm[1]*actual[2]-arm[2]*actual[1],
            arm[2]*actual[0]-arm[0]*actual[2],arm[0]*actual[1]-arm[1]*actual[0]};
        for(std::size_t k=0;k<3;++k)
            require(std::abs(moment[k].get()*1.3558179483314-torque[k])<=tolerance,
                "native vector moment must equal the real nozzle arm cross thrust about current CG");
        previousPitch=p;previousYaw=y;
    }
    require(maximum>1e-6,"crossing-target native guidance must exercise real nozzle motion");
    fixture.property("fsim/thrust-vector-control/rate_rad_s").set(0);
    fixture.world.step();
    const auto read=fixture.world.weaponControlsV1(fixture.missile);
    require(read && !read->controllerAvailable && read->commands[0]==0 && read->commands[1]==0 && read->commands[2]==0,
        "mismatched actual vector metadata must refuse neutral control rather than silently selecting fin-only execution");
}
}

int main() {
    const struct { const char* name; void (*run)(); } cases[] = {
        {"finite surfaces and no ideal external actuation", finiteSurfaceResponse},
        {"native stuck-neutral surface counterfactual", stuckSurfacesCannotFollowGuidance},
        {"individual native physical fin failure", individualPhysicalFinFailure},
        {"native control worker determinism", workerDeterminism},
        {"jammed actual fins report neutral positions", jammedPhysicalFinsReportActualPositions},
        {"native aerodynamic control follows qbar", aerodynamicResponseDependsOnQbar},
        {"tail-control native plant signs", tailPlant},
        {"canard-control native plant signs", canardPlant},
        {"moving-wing native plant signs", movingWingPlant},
        {"all 24 current missile native control plants", currentFleetPlants},
        {"native actuator reset replays a fresh control plant", nativeControlResetReplaysFreshModel},
          {"native finite nozzle thrust and current-CG moment", nativeVectoringNozzleConservesThrustAndProducesCurrentCgTorque},
          {"local derivative plant retains actual fin telemetry", localDerivativePlantRetainsActualFinTelemetry},
    };
    unsigned passed = 0;
    for (const auto& test : cases) {
        try {
            test.run();
            ++passed;
            std::cout << "PASS: " << test.name << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n';
        }
    }
    std::cout << "CASES: " << passed << '/' << std::size(cases) << " passed\n";
    return passed == std::size(cases) ? 0 : 1;
}
