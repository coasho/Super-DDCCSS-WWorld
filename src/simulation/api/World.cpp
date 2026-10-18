// fsim SDK: World / Vehicle / Environment over session::World.

#include "fsim/World.h"
#include "fsim/MAService.h"

#include "foundation/core/Log.h"
#include "foundation/core/Time.h"
#include "simulation/runtime/services/AgentRules.h"
#include "simulation/runtime/world/World.h"
#include "simulation/api/WorldOptions.h"

#include <chrono>
#include <ctime>
#include <limits>

namespace fsim {

namespace {

const VehicleState& emptyState() {
    static const VehicleState s{};
    return s;
}
const effects::SensedState& emptySensed() {
    static const effects::SensedState s{};
    return s;
}
const ControlInputs& emptyInputs() {
    static const ControlInputs i{};
    return i;
}

} // namespace

double utcToUnixSeconds(const Utc& utc) noexcept {
    return core::unixSecondsFromCivil(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second);
}

// --- Environment --------------------------------------------------------------------

void Environment::setTime(const Utc& utc) { setTime(utcToUnixSeconds(utc)); }

void Environment::setTime(double unixSeconds) {
    auto s = world_->impl_->environment();
    s.epochUtcSeconds = unixSeconds - world_->impl_->simTime();
    world_->impl_->setEnvironment(s);
}

void Environment::setTimeFactor(double factor) {
    auto s = world_->impl_->environment();
    s.timeFactor = factor;
    world_->impl_->setEnvironment(s);
}

void Environment::setWind(const Wind& wind) {
    auto s = world_->impl_->environment();
    s.windDirectionDeg = wind.directionDeg;
    s.windSpeedMs = wind.speedMs;
    s.windGustMs = wind.gustMs;
    s.turbulence = wind.turbulence;
    world_->impl_->setEnvironment(s);
}

void Environment::setAtmosphere(const Atmosphere& atmosphere) {
    auto s = world_->impl_->environment();
    s.temperatureSeaLevelK = atmosphere.temperatureSeaLevelK;
    s.pressureSeaLevelPa = atmosphere.pressureSeaLevelPa;
    s.humidity = atmosphere.humidity;
    world_->impl_->setEnvironment(s);
}

void Environment::setWeather(const Weather& weather) {
    auto s = world_->impl_->environment();
    s.visibilityM = weather.visibilityM;
    s.cloudBaseM = weather.cloudBaseM;
    s.cloudCover = weather.cloudCover;
    s.precipitation = weather.precipitation;
    world_->impl_->setEnvironment(s);
}

void Environment::set(const EnvironmentState& state) { world_->impl_->setEnvironment(state); }
const EnvironmentState& Environment::state() const noexcept { return world_->impl_->environment(); }
double Environment::utcSeconds() const noexcept { return world_->impl_->environment().epochUtcSeconds + world_->impl_->simTime(); }

// --- Vehicle ------------------------------------------------------------------------

bool Vehicle::valid() const noexcept { return world_ && world_->impl_->alive(id_); }

std::string Vehicle::name() const {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->name : std::string();
}

std::string Vehicle::type() const {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->type : std::string();
}

ms::Role Vehicle::role() const noexcept {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->role : ms::Role::Aircraft;
}

std::uint32_t Vehicle::launcher() const noexcept {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->launcher : 0;
}

std::string Vehicle::model() const {
    const auto* i = world_ ? world_->impl_->info(id_) : nullptr;
    return i ? i->model : std::string();
}

const VehicleState& Vehicle::state() const {
    const auto* s = world_ ? world_->impl_->vehicleState(id_) : nullptr;
    return s ? *s : emptyState();
}

const effects::SensedState& Vehicle::sensed() const {
    const auto* s = world_ ? world_->impl_->sensedState(id_) : nullptr;
    return s ? *s : emptySensed();
}

const ControlInputs& Vehicle::inputs() const {
    const auto* s = world_ ? world_->impl_->inputs(id_) : nullptr;
    return s ? *s : emptyInputs();
}

bool Vehicle::command(const control::Command& command) { return world_ && world_->impl_->command(id_, command); }

control::CommandResult Vehicle::submit(const control::Command& command, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::SupportCommand& command, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::RouteCommand& route, Span<const control::Waypoint> waypoints, const control::CommandOptions& options,
                                       Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states,
                                       Span<const control::RoutePath> paths, Span<const control::RouteBranch> branches,
                                       Span<const control::RouteTerminator> terminators) {
    if (world_) return world_->impl_->submit(id_, route, waypoints, options, loiters, states, paths, branches, terminators);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                       const control::CommandOptions& options, const control::CurveShape* shape) {
    if (world_) return world_->impl_->submit(id_, curve, segments, options, shape);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                       const control::CommandOptions& options, const control::CurveShape* shape) {
    if (world_) return world_->impl_->submit(id_, curve, segments, options, shape);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::PatternCommand& pattern, const control::PatternShape& shape, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, pattern, shape, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::MarshallCommand& marshall, const control::PatternShape& shape, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, marshall, shape, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::InterceptCommand& intercept, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, intercept, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::MustFlyCommand& mustFly, const control::OpZone& zone, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, mustFly, zone, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::MustFlyCommand& mustFly, const control::OpLine& line, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, mustFly, line, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

control::CommandResult Vehicle::submit(const control::MustFlyCommand& mustFly, const control::OpVolume& volume, const control::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, mustFly, volume, options);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return r;
}

std::vector<control::CommandResult> Vehicle::submitBatch(Span<const control::BatchCommand> batch, std::vector<control::CommandDetails>* details) {
    if (world_) return world_->impl_->submitBatch(id_, batch, details);
    control::CommandResult r;
    r.reason = control::Reason::UnknownVehicle;
    return std::vector<control::CommandResult>(batch.size(), r);
}

control::CommandDetails Vehicle::commandDetails() const {
    const control::CommandDetails* d = world_ ? world_->impl_->commandDetails(id_) : nullptr;
    return d ? *d : control::CommandDetails{};
}

std::vector<control::ActivityRecord> Vehicle::activities() const {
    return world_ ? world_->impl_->activities(id_) : std::vector<control::ActivityRecord>{};
}

control::VehicleCommandState Vehicle::commanded() const { return world_ ? world_->impl_->commandState(id_) : control::VehicleCommandState{}; }

control::NavigationReport Vehicle::navigationReport() const {
    return world_ ? world_->impl_->navigationReport(id_) : control::NavigationReport{};
}

control::Reason Vehicle::setNavigation(const control::NavigationSettings& settings) {
    return world_ ? world_->impl_->setNavigation(id_, settings) : control::Reason::UnknownVehicle;
}

control::NavigationSettings Vehicle::navigation() const {
    return world_ ? world_->impl_->navigation(id_) : control::NavigationSettings{};
}

control::Reason Vehicle::setQnh(double qnhPa) { return world_ ? world_->impl_->setQnh(id_, qnhPa) : control::Reason::UnknownVehicle; }

double Vehicle::qnh() const { return world_ ? world_->impl_->qnh(id_) : std::numeric_limits<double>::quiet_NaN(); }

control::StateData Vehicle::stateData() const { return world_ ? world_->impl_->stateData(id_) : control::StateData{}; }

control::Reason Vehicle::performanceProfile(control::FlightMode mode, control::PerformanceProfile& out) const {
    return world_ ? world_->impl_->performanceProfile(id_, mode, out) : control::Reason::UnknownVehicle;
}

std::vector<control::CapabilityDescriptor> Vehicle::capabilities() const {
    return world_ ? world_->impl_->capabilities(id_) : std::vector<control::CapabilityDescriptor>{};
}

const control::VehicleProfile& Vehicle::profile() const {
    static const control::VehicleProfile none;
    const auto* p = world_ ? world_->impl_->profile(id_) : nullptr;
    return p ? *p : none;
}

control::Reason Vehicle::setVehicleDefault(control::VehicleDefault mode) {
    return world_ ? world_->impl_->setVehicleDefault(id_, mode) : control::Reason::UnknownVehicle;
}

control::VehicleDefault Vehicle::vehicleDefault() const {
    return world_ ? world_->impl_->vehicleDefault(id_) : control::VehicleDefault::Neutral;
}

control::Reason Vehicle::setProtection(control::ProtectionMode mode) {
    return world_ ? world_->impl_->setProtection(id_, mode) : control::Reason::UnknownVehicle;
}

control::ProtectionMode Vehicle::protection() const {
    return world_ ? world_->impl_->protection(id_) : control::ProtectionMode::Off;
}

control::EnvelopeStatus Vehicle::envelope() { return world_ ? world_->impl_->envelope(id_) : control::EnvelopeStatus{}; }

control::CapabilityStatus Vehicle::capabilityStatus(std::string_view capability) const {
    if (world_) return world_->impl_->capabilityStatus(id_, capability);
    control::CapabilityStatus none;
    none.availability = control::Availability::Unavailable;
    none.reason = control::Reason::UnknownVehicle;
    none.reasons = control::reasonBit(none.reason);
    none.description = control::reasonDescription(none.reason);
    return none;
}

const control::SupportInfo* Vehicle::support(std::string_view feature) const {
    return world_ ? world_->impl_->support(id_, feature) : nullptr;
}

std::vector<control::SupportInfo> Vehicle::supportTable() const {
    const control::SupportTable* t = world_ ? world_->impl_->supportTable(id_) : nullptr;
    return t ? t->rows() : std::vector<control::SupportInfo>{};
}

control::Performance Vehicle::performance() const {
    const control::Performance* p = world_ ? world_->impl_->performance(id_) : nullptr;
    return p ? *p : control::Performance{};
}

std::uint32_t Vehicle::controlRevision() const { return world_ ? world_->impl_->controlRevision(id_) : 0; }

control::Reason Vehicle::setControlMode(control::ControlMode mode) {
    return world_ ? world_->impl_->setControlMode(id_, mode) : control::Reason::UnknownVehicle;
}

control::ControlMode Vehicle::controlMode() const { return world_ ? world_->impl_->controlMode(id_) : control::ControlMode::Open; }

std::vector<ms::PayloadInfo> Vehicle::payloads() const {
    const ms::Host* host = world_ ? world_->impl_->missionSystems(id_) : nullptr;
    return host ? host->payloads() : std::vector<ms::PayloadInfo>{};
}

std::vector<ms::CapabilityInfo> Vehicle::missionCapabilities() const {
    const ms::Host* host = world_ ? world_->impl_->missionSystems(id_) : nullptr;
    return host ? host->capabilities() : std::vector<ms::CapabilityInfo>{};
}

ms::CapabilityStatus Vehicle::missionCapabilityStatus(std::string_view capability) const {
    if (world_) return world_->impl_->missionCapabilityStatus(id_, capability);
    ms::CapabilityStatus none;
    none.availability = control::Availability::Unavailable;
    none.reason = ms::Reason::UnknownVehicle;
    return none;
}

ms::CommandResult Vehicle::submit(const ms::StateCommand& command, const ms::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
}

ms::CommandResult Vehicle::submit(const ms::BitCommand& command, const ms::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
}

ms::CommandResult Vehicle::submit(const ms::AmtiCommand& command, const ms::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
}

std::vector<ms::StoreInfo> Vehicle::stores() const { return world_ ? world_->impl_->stores(id_) : std::vector<ms::StoreInfo>{}; }

ms::Reason Vehicle::loadStore(std::uint16_t payload, int station, const std::string& store, int count) {
    return world_ ? world_->impl_->loadStore(id_, payload, station, store, count) : ms::Reason::UnknownVehicle;
}

ms::Reason Vehicle::loadLoadout(const std::string& loadout) {
    return world_ ? world_->impl_->loadLoadout(id_, loadout) : ms::Reason::UnknownVehicle;
}

ms::CommandResult Vehicle::submit(const ms::StrikeCommand& command, const ms::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
}

ms::CommandResult Vehicle::submit(const ms::JettisonCommand& command, const ms::CommandOptions& options) {
    if (world_) return world_->impl_->submit(id_, command, options);
    return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
}

ms::Reason Vehicle::strikeConsent(ms::ActivityId activity, bool granted) {
    return world_ ? world_->impl_->strikeConsent(activity, granted) : ms::Reason::UnknownVehicle;
}

std::optional<ms::StrikeStatus> Vehicle::strikeStatus(ms::ActivityId activity) const {
    return world_ ? world_->impl_->strikeStatus(activity) : std::nullopt;
}

double Vehicle::massKg() const { return world_ ? world_->impl_->massKg(id_) : std::numeric_limits<double>::quiet_NaN(); }
std::array<double, 3> Vehicle::cgFromModelOriginM() const {
    std::array<double, 3> cg{0.0, 0.0, 0.0};
    if (world_) world_->impl_->cgFromOriginBodyM(id_, cg.data());
    return cg;
}

ms::RadarStatus Vehicle::radarStatus(std::uint16_t payload) const {
    return world_ ? world_->impl_->radarStatus(id_, payload) : ms::RadarStatus{};
}

std::vector<ms::AmtiDetection> Vehicle::amtiDetections(std::uint16_t payload, std::uint64_t since) const {
    return world_ ? world_->impl_->amtiDetections(id_, payload, since) : std::vector<ms::AmtiDetection>{};
}

std::vector<ms::AmtiTrack> Vehicle::amtiTracks(std::uint16_t payload) const {
    return world_ ? world_->impl_->amtiTracks(id_, payload) : std::vector<ms::AmtiTrack>{};
}

std::vector<ms::Contact> Vehicle::contacts(std::uint16_t payload) const {
    return world_ ? world_->impl_->contacts(id_, payload) : std::vector<ms::Contact>{};
}

ms::CommandResult Vehicle::cancelMission(ms::ActivityId activity, control::Caller caller) {
    if (world_) return world_->impl_->cancelMission(id_, caller, activity);
    return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, activity, 0};
}

std::optional<ms::ActivityRecord> Vehicle::missionActivity(ms::ActivityId activity) const {
    const ms::Host* host = world_ ? world_->impl_->missionSystems(id_) : nullptr;
    const ms::ActivityRecord* a = host ? host->activity(activity) : nullptr;
    return a ? std::optional<ms::ActivityRecord>(*a) : std::nullopt;
}

std::vector<ms::ActivityRecord> Vehicle::missionActivities() const {
    const ms::Host* host = world_ ? world_->impl_->missionSystems(id_) : nullptr;
    return host ? host->activities() : std::vector<ms::ActivityRecord>{};
}

ms::Reason Vehicle::failPayload(std::uint16_t payload, bool failed) {
    return world_ ? world_->impl_->failPayload(id_, payload, failed) : ms::Reason::UnknownVehicle;
}

control::Reason Vehicle::requestControl(std::string_view capability, control::ControllerId controller) {
    return world_ ? world_->impl_->requestControl(id_, capability, controller) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::releaseControl(std::string_view capability, control::ControllerId controller) {
    return world_ ? world_->impl_->releaseControl(id_, capability, controller) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::revokeControl(std::string_view capability, control::Reason reason) {
    return world_ ? world_->impl_->revokeControl(id_, capability, reason) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::setAllowed(std::string_view capability, bool allowed) {
    return world_ ? world_->impl_->setAllowed(id_, capability, allowed) : control::Reason::UnknownVehicle;
}

control::ControlStatus Vehicle::controlStatus(std::string_view capability) const {
    return world_ ? world_->impl_->controlStatus(id_, capability) : control::ControlStatus{false, false};
}

control::Reason Vehicle::storeTask(control::TaskId task, const control::Command& command, Span<const control::Waypoint> waypoints,
                                   Span<const control::BezierSegment> segments, control::TaskRepetition repetition, const control::PatternShape* shape) {
    return world_ ? world_->impl_->storeTask(id_, task, command, waypoints, segments, repetition, shape) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::storeTask(control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition) {
    return world_ ? world_->impl_->storeTask(id_, task, command, repetition) : control::Reason::UnknownVehicle;
}

control::CommandResult Vehicle::commandTask(control::TaskId task, const control::CommandOptions& options) {
    if (!world_) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->commandTask(id_, task, options);
}

control::CommandResult Vehicle::cancelTask(control::TaskId task, control::Caller caller) {
    if (!world_) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->cancelTask(id_, task, caller);
}

control::Reason Vehicle::removeTask(control::TaskId task) { return world_ ? world_->impl_->removeTask(id_, task) : control::Reason::UnknownVehicle; }

std::optional<control::TaskStatus> Vehicle::taskStatus(control::TaskId task) const {
    return world_ ? world_->impl_->taskStatus(id_, task) : std::nullopt;
}

std::vector<control::TaskStatus> Vehicle::tasks() const { return world_ ? world_->impl_->tasks(id_) : std::vector<control::TaskStatus>{}; }

control::Reason Vehicle::publishPlan(const control::RoutePlan& plan) {
    return world_ ? world_->impl_->publishPlan(id_, plan) : control::Reason::UnknownVehicle;
}

control::PlanCommandResult Vehicle::planCommand(control::PlanId plan, control::PlanCommand command, const control::CommandOptions& options) {
    if (!world_) {
        control::PlanCommandResult r;
        r.plan = plan, r.command = command, r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->planCommand(id_, plan, command, options);
}

control::PlanCommandResult Vehicle::abortPlan(control::PlanId plan, control::Reason reason) {
    if (!world_) {
        control::PlanCommandResult r;
        r.plan = plan, r.command = control::PlanCommand::Deactivate, r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->abortPlan(id_, plan, reason);
}

control::Reason Vehicle::removePlan(control::PlanId plan) { return world_ ? world_->impl_->removePlan(id_, plan) : control::Reason::UnknownVehicle; }

std::optional<control::PlanStatus> Vehicle::planStatus(control::PlanId plan) const {
    return world_ ? world_->impl_->planStatus(id_, plan) : std::nullopt;
}

std::vector<control::PlanStatus> Vehicle::plans() const { return world_ ? world_->impl_->plans(id_) : std::vector<control::PlanStatus>{}; }

std::optional<control::RoutePlan> Vehicle::plan(control::PlanId plan) const {
    return world_ ? world_->impl_->plan(id_, plan) : std::nullopt;
}

control::Reason Vehicle::loadPlan(const control::RoutePlan& plan) {
    return world_ ? world_->impl_->loadPlan(id_, plan) : control::Reason::UnknownVehicle;
}

control::Reason Vehicle::loadAirfield(const control::Airfield& airfield) {
    return world_ ? world_->impl_->loadAirfield(id_, airfield) : control::Reason::UnknownVehicle;
}

std::vector<control::Airfield> Vehicle::airfields() const {
    return world_ ? world_->impl_->airfields(id_) : std::vector<control::Airfield>{};
}

std::optional<control::Airfield> Vehicle::airfield(control::AirfieldId airfield) const {
    return world_ ? world_->impl_->airfield(id_, airfield) : std::nullopt;
}

control::PlanValidationResult Vehicle::validatePlan(const control::RoutePlan& plan, const control::PlanValidation& v) {
    if (!world_) {
        control::PlanValidationResult r;
        r.check.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->validatePlan(id_, plan, v);
}

control::PlanValidationResult Vehicle::validatePlan(control::PlanId plan, const control::PlanValidation& v) {
    if (!world_) {
        control::PlanValidationResult r;
        r.check.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return world_->impl_->validatePlan(id_, plan, v);
}

control::Reason Vehicle::setCapabilityPrecedence(std::string_view capability, std::uint32_t precedence) {
    return world_ ? world_->impl_->setCapabilityPrecedence(id_, capability, precedence) : control::Reason::UnknownVehicle;
}

std::uint32_t Vehicle::capabilityPrecedence(std::string_view capability) const {
    return world_ ? world_->impl_->capabilityPrecedence(id_, capability) : 0;
}

control::Reason Vehicle::setAvailability(std::string_view capability, control::Availability availability, control::Reason reason,
                                         std::uint64_t associated, double nextAvailableS) {
    return world_ ? world_->impl_->setAvailability(id_, capability, availability, reason, associated, nextAvailableS)
                  : control::Reason::UnknownVehicle;
}

control::ControlStack& Vehicle::controls() {
    auto* c = world_ ? world_->impl_->controls(id_) : nullptr;
    if (!c) throw Error("Vehicle::controls: invalid vehicle handle");
    return *c;
}

control::Level Vehicle::activeLevel() const {
    const auto* c = world_ ? world_->impl_->controls(id_) : nullptr;
    return c ? c->activeLevel() : control::Level::Actuator;
}

bool Vehicle::use(control::Level level, std::string_view controllerId) {
    auto* c = world_ ? world_->impl_->controls(id_) : nullptr;
    return c && c->use(level, controllerId);
}

bool Vehicle::reset() { return world_ && world_->impl_->resetVehicle(id_); }
bool Vehicle::reset(const InitialConditions& initial) { return world_ && world_->impl_->resetVehicle(id_, &initial); }
bool Vehicle::reset(const InitialConditions& initial, std::uint64_t seed) { return world_ && world_->impl_->resetVehicle(id_, &initial, seed); }
bool Vehicle::remove() { return world_ && world_->impl_->removeVehicle(id_); }
bool Vehicle::addEffect(std::unique_ptr<effects::Effect> effect) { return world_ && world_->impl_->addEffect(id_, std::move(effect)); }

PropertyHandle Vehicle::property(std::string_view path) {
    auto* m = world_ ? world_->impl_->model(id_) : nullptr;
    return m ? m->property(path) : PropertyHandle();
}

std::uint32_t Vehicle::designate(std::uint16_t payload, std::uint64_t track) {
    return world_ ? world_->impl_->designate(id_, payload, track) : 0;
}

bool Vehicle::undesignate(std::uint32_t target) { return world_ && world_->impl_->undesignate(id_, target); }

std::vector<Designation> Vehicle::designations() const {
    std::vector<Designation> out;
    if (world_)
        for (const auto& d : world_->impl_->designations(id_)) out.push_back({d.id, d.payload, d.track, d.valid, d.source});
    return out;
}

OwnShip Vehicle::ownship() const {
    const effects::SensedState* s = world_ ? world_->impl_->sensedState(id_) : nullptr;
    return s ? session::ownShipOf(*s) : OwnShip{};
}

Agent World::createAgent(const Vehicle& platform, std::string_view name) {
    const std::string n(name);
    const control::ControllerId controller = impl_->mintAgent(platform.id(), n);
    if (controller == 0) throw Error("createAgent: vehicle " + std::to_string(platform.id()) + " is not a live aircraft");
    return Agent(this, platform.id(), controller, n);
}

Agent World::createAgent(std::uint32_t entity, std::string_view name) {
    const std::string n(name);
    const control::ControllerId controller = impl_->mintAgent(entity, n);
    if (controller == 0) throw Error("createAgent: " + std::to_string(entity) + " is neither a live aircraft nor a C2 node");
    return Agent(this, entity, controller, n);
}

std::uint32_t World::createC2Node(const comm::C2NodeSpec& spec) {
    const std::uint32_t id = impl_->createC2Node(spec);
    if (id == 0) throw Error("createC2Node: '" + spec.name + "' has no valid position");
    return id;
}
bool World::removeC2Node(std::uint32_t node) { return impl_->removeC2Node(node); }
std::vector<std::uint32_t> World::c2Nodes() const { return impl_->c2Nodes(); }
comm::NetId World::createNet(const comm::NetSpec& spec) { return impl_->createNet(spec); }
bool World::joinNet(std::uint32_t entity, comm::NetId net) { return impl_->joinNet(entity, net); }
bool World::leaveNet(std::uint32_t entity, comm::NetId net) { return impl_->leaveNet(entity, net); }
bool World::setDissemination(std::uint32_t entity, const comm::DisseminationPlan& plan) { return impl_->setDissemination(entity, plan); }
void World::setRbac(std::uint32_t entity, const comm::RbacConfig& config) { impl_->maCore().setRbac(entity, config); }
comm::RbacConfig World::rbac(std::uint32_t entity) const { return impl_->maCore().rbac(entity); }
comm::SystemId World::assignSystem(std::uint32_t entity, comm::SystemId system, std::string_view tailNumber) {
    return impl_->assignSystem(entity, system, std::string(tailNumber));
}

comm::Node* Vehicle::node() { return world_ ? world_->impl_->network().node(id_) : nullptr; }

// --- World --------------------------------------------------------------------------

World::World(const WorldOptions& options) : environment_(*this) {
    impl_ = std::make_shared<session::World>(session::makeWorldOptions(options));
    // Default epoch: now, so the viewer's sun matches the wall clock unless told otherwise.
    auto s = impl_->environment();
    s.epochUtcSeconds = static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    impl_->setEnvironment(s);
}

World::World(Borrow, session::World& borrowed) : impl_(&borrowed, [](session::World*) {}), environment_(*this) {}

World::~World() = default;

Vehicle World::launchWeapon(const Vehicle& launcher, const ms::WeaponLaunch& launch) {
    // Numeric IDs are local to a native World; distinct facade wrappers may share that World.
    if (!launcher.world_ || &launcher.world_->impl() != impl_.get())
        throw Error("World::launchWeapon: the launcher belongs to another world or is empty");
    const std::uint32_t id = impl_->launchWeapon(launcher.id(), launch);
    if (!id) throw Error("World::launchWeapon: no '" + launch.type + "' released by vehicle " + std::to_string(launcher.id()) + " (see log)");
    return Vehicle(this, id);
}

std::vector<ms::WeaponRecord> World::weapons() const { return impl_->weapons(); }
ms::Reason World::dispense(std::uint32_t vehicle, bool flare) { return impl_->dispense(vehicle,flare); }
std::uint32_t World::seekerTarget(std::uint32_t vehicle,const std::string& store) const { return impl_->seekerTarget(vehicle,store); }
ms::CountermeasureInventory World::countermeasures(std::uint32_t vehicle) const { return impl_->countermeasures(vehicle); }
std::vector<ms::DecoyState> World::decoys() const { return impl_->decoys(); }
std::vector<ms::EngagementEvent> World::engagementEvents(std::uint64_t since) const { return impl_->engagementEvents(since); }
ms::LaunchZone World::launchZone(std::uint32_t vehicle,const std::string& store,std::uint32_t target) const {
    return impl_->launchZone(vehicle,store,target);
}

std::optional<ms::RadioPath> World::radioPath(const Vehicle& from, const Vehicle& to, const ms::RadioPathOptions& options) const {
    return impl_->radioPath(from.id(), to.id(), options);
}

std::optional<ms::RadarCrossSection> World::radarCrossSection(const Vehicle& observer, const Vehicle& target, const ms::RcsQuery& query) const {
    return impl_->radarCrossSection(observer.id(), target.id(), query);
}

Vehicle World::createVehicle(const VehicleSpec& spec) {
    session::VehicleSpec s;
    s.name = spec.name;
    s.type = spec.type;
    s.initial = spec.initial;
    s.model = spec.model;
    s.controlDivider = spec.controlDivider;
    s.profile = spec.profile;
    const std::uint32_t id = impl_->createVehicle(s);
    if (!id) throw Error("World::createVehicle: failed to create '" + spec.name + "' of type '" + spec.type + "' (see log)");
    return Vehicle(this, id);
}

Vehicle World::vehicle(std::uint32_t id) noexcept { return impl_->alive(id) ? Vehicle(this, id) : Vehicle(); }
Vehicle World::vehicle(std::string_view name) noexcept { return vehicle(impl_->find(std::string(name))); }

std::vector<Vehicle> World::vehicles() {
    std::vector<Vehicle> out;
    for (auto id : impl_->vehicleIds()) out.push_back(Vehicle(this, id));
    return out;
}

std::size_t World::vehicleCount() const noexcept { return impl_->vehicleCount(); }
control::CommandResult World::update(control::ActivityId activity, const control::Command& setpoint) {
    return impl_->update(activity, setpoint);
}

control::CommandResult World::update(control::ActivityId activity, const control::SupportCommand& setpoint) {
    return impl_->update(activity, setpoint);
}

control::CommandResult World::update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                     Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states,
                                     Span<const control::RoutePath> paths, Span<const control::RouteBranch> branches,
                                     Span<const control::RouteTerminator> terminators) {
    return impl_->update(activity, route, waypoints, loiters, states, paths, branches, terminators);
}

control::CommandResult World::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                     const control::CurveShape* shape) {
    return impl_->update(activity, curve, segments, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                     const control::CurveShape* shape) {
    return impl_->update(activity, curve, segments, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape) {
    return impl_->update(activity, pattern, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::MarshallCommand& marshall, const control::PatternShape& shape) {
    return impl_->update(activity, marshall, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::MarshallCommand& marshall) {
    return impl_->update(activity, marshall);
}

control::CommandResult World::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone) {
    return impl_->update(activity, mustFly, zone);
}

control::CommandResult World::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpLine& line) {
    return impl_->update(activity, mustFly, line);
}

control::CommandResult World::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpVolume& volume) {
    return impl_->update(activity, mustFly, volume);
}

control::CommandResult World::cancel(control::ActivityId activity) { return impl_->cancel(activity); }

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::Command& setpoint) {
    return impl_->update(caller, activity, setpoint);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::SupportCommand& setpoint) {
    return impl_->update(caller, activity, setpoint);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::RouteCommand& route,
                                     Span<const control::Waypoint> waypoints, Span<const control::RouteLoiter> loiters,
                                     Span<const control::RouteState> states, Span<const control::RoutePath> paths,
                                     Span<const control::RouteBranch> branches, Span<const control::RouteTerminator> terminators) {
    return impl_->update(caller, activity, route, waypoints, loiters, states, paths, branches, terminators);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                     Span<const control::BezierSegment> segments, const control::CurveShape* shape) {
    return impl_->update(caller, activity, curve, segments, shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                     Span<const control::NurbsSegment> segments, const control::CurveShape* shape) {
    return impl_->update(caller, activity, curve, segments, shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::PatternCommand& pattern,
                                     const control::PatternShape& shape) {
    return impl_->update(caller, activity, pattern, shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall,
                                     const control::PatternShape& shape) {
    return impl_->update(caller, activity, marshall, shape);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall) {
    return impl_->update(caller, activity, marshall);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                     const control::OpZone& zone) {
    return impl_->update(caller, activity, mustFly, zone);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                     const control::OpLine& line) {
    return impl_->update(caller, activity, mustFly, line);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                     const control::OpVolume& volume) {
    return impl_->update(caller, activity, mustFly, volume);
}

control::CommandResult World::cancel(control::Caller caller, control::ActivityId activity) { return impl_->cancel(caller, activity); }

control::CommandResult World::activityCommand(control::ActivityId activity, control::ActivityCommand command, control::Rank rank,
                                              control::Caller caller) {
    return impl_->activityCommand(caller, activity, command, rank);
}

control::CommandResult World::commandBranch(control::ActivityId activity, std::uint32_t branch, bool commanded, control::Caller caller) {
    return impl_->commandBranch(caller, activity, branch, commanded);
}

std::optional<control::ActivityRecord> World::activity(control::ActivityId activity) const {
    if (const auto* a = impl_->activity(activity)) return *a;
    return std::nullopt;
}

std::optional<control::Setpoint> World::activitySetpoint(control::ActivityId activity) const {
    control::Setpoint s;
    if (!impl_->activitySetpoint(activity, s)) return std::nullopt;
    return s;
}

std::optional<control::InterceptStatus> World::interceptStatus(control::ActivityId activity) const {
    control::InterceptStatus s;
    if (!impl_->interceptStatus(activity, s)) return std::nullopt;
    return s;
}

std::vector<control::EndPoint> World::endPoints(control::ActivityId activity, std::size_t max) const { return impl_->endPoints(activity, max); }

void World::step(unsigned n) { impl_->step(n); }
double World::time() const noexcept { return impl_->simTime(); }
double World::stepSeconds() const noexcept { return impl_->dt() * impl_->frameSkip(); }
comm::Network& World::network() { return impl_->network(); }

std::optional<double> World::terrainHeightM(double latitudeRad, double longitudeRad) const { return impl_->terrainHeightM(latitudeRad, longitudeRad); }

control::FrameId World::createFrame(const control::FrameSpec& spec) { return impl_->createFrame(spec); }
control::Reason World::setOpPoint(const control::OpPoint& point) { return impl_->setOpPoint(point); }
bool World::removeOpPoint(control::OpPointId id) { return impl_->removeOpPoint(id); }
std::vector<control::OpPointId> World::opPoints() const { return impl_->opPoints(); }
std::optional<control::OpPoint> World::opPoint(control::OpPointId id) const { return impl_->opPoint(id); }
control::Reason World::setOpZone(const control::OpZone& zone) { return impl_->setOpZone(zone); }
bool World::removeOpZone(control::OpZoneId id) { return impl_->removeOpZone(id); }
std::vector<control::OpZoneId> World::opZones() const { return impl_->opZones(); }
std::optional<control::OpZone> World::opZone(control::OpZoneId id) const { return impl_->opZone(id); }
control::Reason World::setOpLine(const control::OpLine& line) { return impl_->setOpLine(line); }
bool World::removeOpLine(control::OpLineId id) { return impl_->removeOpLine(id); }
std::vector<control::OpLineId> World::opLines() const { return impl_->opLines(); }
std::optional<control::OpLine> World::opLine(control::OpLineId id) const { return impl_->opLine(id); }
control::Reason World::setOpVolume(const control::OpVolume& volume) { return impl_->setOpVolume(volume); }
bool World::removeOpVolume(control::OpVolumeId id) { return impl_->removeOpVolume(id); }
std::vector<control::OpVolumeId> World::opVolumes() const { return impl_->opVolumes(); }
std::optional<control::OpVolume> World::opVolume(control::OpVolumeId id) const { return impl_->opVolume(id); }

bool World::removeFrame(control::FrameId id) { return impl_->removeFrame(id); }

std::optional<control::FrameSpec> World::frame(control::FrameId id) const { return impl_->frame(id); }

std::optional<control::FramePose> World::framePose(control::FrameId id, double timeS) const { return impl_->framePose(id, timeS); }

std::optional<control::GeoPoint> World::framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS) const {
    return impl_->framePoint(id, offset, timeS);
}
void World::addEffectToAll(std::function<std::unique_ptr<effects::Effect>()> factory) { impl_->addEffectToAll(std::move(factory)); }
const std::string& World::name() const noexcept { return impl_->name(); }
bool World::published() const noexcept { return impl_->published(); }
bool World::focusViewer(std::uint32_t id) noexcept { return impl_->focusViewer(id); }
std::uint64_t World::vehicleSteps() const noexcept { return impl_->vehicleSteps(); }
std::uint64_t World::worldSteps() const noexcept { return impl_->worldSteps(); }
void World::publishNow() { impl_->publishNow(); }

// --- MA services (fsim/MAService.h; docs/sdk/mission-training.md, section 9) --------------------------------------------

bool bindMAService(World& world, const MAServiceBindingV1& binding, std::string* error) {
    session::MaServiceBinding b;
    b.serviceId = binding.serviceId, b.platform = binding.platform, b.controller = binding.controller;
    b.capabilities = binding.commandCapabilities;
    b.ownsDmsIngress = binding.ownsDmsIngress, b.ownsDmsEgress = binding.ownsDmsEgress;
    std::string why;
    const bool bound = world.impl().bindMaService(b, why);
    if (error) *error = bound ? std::string() : why;
    return bound;
}

bool bindMAService(World& world, const Agent& agent, std::uint64_t serviceId, const std::vector<std::string>& commandCapabilities,
                   bool ownsDmsIngress, bool ownsDmsEgress, std::string* error) {
    // (two Worlds mint the same vehicle ids and controllers: whose an Agent is, only the World that made it says)
    if (!agent.world_ || &agent.world_->impl() != &world.impl()) {
        if (error) *error = "MA service " + std::to_string(serviceId) + ": the agent is not one this world made";
        return false;
    }
    return bindMAService(world, MAServiceBindingV1{serviceId, agent.vehicle(), agent.controller(), commandCapabilities, ownsDmsIngress, ownsDmsEgress},
                         error);
}

std::vector<MAServiceBindingV1> maServiceBindings(World& world) {
    std::vector<MAServiceBindingV1> out;
    for (const session::MaServiceBinding& b : world.impl().maServices().all())
        out.push_back({b.serviceId, b.platform, b.controller, b.capabilities, b.ownsDmsIngress, b.ownsDmsEgress});
    return out;
}

control::ControllerId maCapabilityOwner(World& world, std::uint32_t platform, const std::string& capability) {
    return world.impl().maServices().owner(platform, capability);
}

} // namespace fsim
