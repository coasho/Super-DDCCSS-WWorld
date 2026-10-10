// fsim SDK: Agent and Referee (ADR-31) over the range's World - the rules in session/AgentRules.h, the same the C ABI's
// agent handles are held to; what an agent's controller commands is checked again by the world (session::World).

#include "fsim/Agent.h"
#include "fsim/World.h"

#include "session/AgentRules.h"
#include "session/World.h"

namespace fsim {

namespace {

control::CommandResult refused(control::Reason reason, std::uint64_t commandId = 0) {
    control::CommandResult r;
    r.reason = reason, r.commandId = commandId;
    return r;
}

ms::CommandResult msRefused(ms::Reason reason, std::uint64_t commandId = 0) {
    return ms::CommandResult{control::CommandStatus::Rejected, reason, 0, commandId};
}

} // namespace

// --- Agent ---------------------------------------------------------------------------

namespace {

/// Count a call in the agent's audit (AB-2d): its record is its controller's place among the world's agents.
void auditOf(World* w, control::ControllerId controller, std::uint16_t call, bool refused) {
    if (w) w->impl().audit(controller - session::World::kAgentControllers - 1, call, refused);
}

} // namespace

/// Every call an agent makes, counted in its audit by its name (fsim::Agent::submit, ...).
#define FSIM_AGENT_AUDIT                                                                                                     \
    static const std::uint16_t fsim_call_ = session::auditCall((std::string("fsim::Agent::") + __func__).c_str());         \
    auditOf(world_, controller_, fsim_call_, false)
/// A refusal of a call counted.
#define FSIM_AGENT_REFUSED (world_ ? world_->impl().auditRefusal(controller_ - session::World::kAgentControllers - 1, fsim_call_) : void())

#define FSIM_AGENT_IDENTITY session::AgentIdentity{vehicle_, controller_, {}}
#define FSIM_AGENT_VEHICLE_OF world_->vehicle(vehicle_)
/// Its options as its own (session::agentOptions), else the call is refused.
#define FSIM_AGENT_OPTIONS(opts, refuse)                                                                                       \
    control::CommandOptions o = opts;                                                                                          \
    if (const auto why = session::agentOptions(FSIM_AGENT_IDENTITY, o); why != control::Reason::None) return FSIM_AGENT_REFUSED, refuse(why, o.commandId)
#define FSIM_AGENT_MS_OPTIONS(opts)                                                                                            \
    ms::CommandOptions o = opts;                                                                                               \
    if (const auto why = session::agentOptions(FSIM_AGENT_IDENTITY, o); why != ms::Reason::None) return FSIM_AGENT_REFUSED, msRefused(why, o.commandId)

bool Agent::own(control::ActivityId activity) const noexcept {
    return world_ && session::agentActivity(FSIM_AGENT_IDENTITY, activity, &world_->impl()); // (not a C2 command's: AB-5)
}
bool Agent::valid() const noexcept {
    FSIM_AGENT_AUDIT;
    return world_ && (FSIM_AGENT_VEHICLE_OF.valid() || world_->impl().c2Node(vehicle_));
}
std::string Agent::type() const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_VEHICLE_OF.type() : std::string();
}

double Agent::time() const noexcept {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->time() : 0.0;
}
OwnShip Agent::ownship() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.ownship();
}
control::NavigationReport Agent::navigationReport() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.navigationReport();
}
control::NavigationSettings Agent::navigation() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.navigation();
}
control::Reason Agent::setNavigation(const control::NavigationSettings& settings) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.setNavigation(settings);
}
control::Reason Agent::setQnh(double qnhPa) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.setQnh(qnhPa);
}
double Agent::qnh() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.qnh();
}
control::StateData Agent::stateData() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.stateData();
}
double Agent::massKg() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.massKg();
}
std::optional<double> Agent::terrainHeightM(double latitudeRad, double longitudeRad) const {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->terrainHeightM(latitudeRad, longitudeRad) : std::nullopt;
}

std::vector<control::CapabilityDescriptor> Agent::capabilities() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.capabilities();
}
control::CapabilityStatus Agent::capabilityStatus(std::string_view capability) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.capabilityStatus(capability);
}
const control::SupportInfo* Agent::support(std::string_view feature) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.support(feature);
}
std::vector<control::SupportInfo> Agent::supportTable() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.supportTable();
}
control::Performance Agent::performance() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.performance();
}
control::Reason Agent::performanceProfile(control::FlightMode mode, control::PerformanceProfile& out) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.performanceProfile(mode, out);
}
std::uint32_t Agent::capabilityPrecedence(std::string_view capability) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.capabilityPrecedence(capability);
}
control::ControlMode Agent::controlMode() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.controlMode();
}
control::VehicleDefault Agent::vehicleDefault() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.vehicleDefault();
}
control::ProtectionMode Agent::protection() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.protection();
}
std::uint32_t Agent::controlRevision() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.controlRevision();
}

control::Reason Agent::requestControl(std::string_view capability) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.requestControl(capability, controller_);
}
control::Reason Agent::releaseControl(std::string_view capability) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.releaseControl(capability, controller_);
}
control::ControlStatus Agent::controlStatus(std::string_view capability) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.controlStatus(capability);
}

control::CommandResult Agent::submit(const control::Command& command, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
control::CommandResult Agent::submit(const control::SupportCommand& command, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
control::CommandResult Agent::submit(const control::RouteCommand& route, Span<const control::Waypoint> waypoints, const control::CommandOptions& options,
                                     Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states,
                                     Span<const control::RoutePath> paths, Span<const control::RouteBranch> branches,
                                     Span<const control::RouteTerminator> terminators) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(route, waypoints, o, loiters, states, paths, branches, terminators);
}
control::CommandResult Agent::submit(const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                     const control::CommandOptions& options, const control::CurveShape* shape) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(curve, segments, o, shape);
}
control::CommandResult Agent::submit(const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                     const control::CommandOptions& options, const control::CurveShape* shape) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(curve, segments, o, shape);
}
control::CommandResult Agent::submit(const control::PatternCommand& pattern, const control::PatternShape& shape, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(pattern, shape, o);
}
control::CommandResult Agent::submit(const control::MarshallCommand& marshall, const control::PatternShape& shape, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(marshall, shape, o);
}
control::CommandResult Agent::submit(const control::InterceptCommand& intercept, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(intercept, o);
}
control::CommandResult Agent::submit(const control::MustFlyCommand& mustFly, const control::OpZone& zone, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(mustFly, zone, o);
}
control::CommandResult Agent::submit(const control::MustFlyCommand& mustFly, const control::OpLine& line, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(mustFly, line, o);
}
control::CommandResult Agent::submit(const control::MustFlyCommand& mustFly, const control::OpVolume& volume, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.submit(mustFly, volume, o);
}
std::vector<control::CommandResult> Agent::submitBatch(Span<const control::BatchCommand> batch, std::vector<control::CommandDetails>* details) {
    FSIM_AGENT_AUDIT;
    std::vector<control::BatchCommand> mine(batch.begin(), batch.end());
    std::vector<std::size_t> claimed; // (an item claiming another source or controller: refused alone)
    for (std::size_t i = 0; i < mine.size(); ++i)
        if (session::agentOptions(FSIM_AGENT_IDENTITY, mine[i].options) != control::Reason::None) claimed.push_back(i);
    if (claimed.empty()) return FSIM_AGENT_VEHICLE_OF.submitBatch(Span<const control::BatchCommand>(mine), details);
    std::vector<control::CommandResult> out(mine.size());
    if (details) details->assign(mine.size(), control::CommandDetails{});
    for (std::size_t i = 0, c = 0; i < mine.size(); ++i) {
        if (c < claimed.size() && claimed[c] == i) {
            out[i] = refused(control::Reason::NotAllowed, mine[i].options.commandId), ++c;
            continue;
        }
        std::vector<control::CommandDetails> one;
        out[i] = FSIM_AGENT_VEHICLE_OF.submitBatch(Span<const control::BatchCommand>(&mine[i], 1), details ? &one : nullptr).front();
        if (details && !one.empty()) (*details)[i] = one.front();
    }
    return out;
}

std::vector<control::ActivityRecord> Agent::activities() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.activities();
}
control::CommandDetails Agent::commandDetails() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.commandDetails();
}
control::VehicleCommandState Agent::commanded() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.commanded();
}
std::optional<control::ActivityRecord> Agent::activity(control::ActivityId activity) const {
    FSIM_AGENT_AUDIT;
    return own(activity) ? world_->activity(activity) : std::nullopt;
}
std::optional<control::Setpoint> Agent::activitySetpoint(control::ActivityId activity) const {
    FSIM_AGENT_AUDIT;
    return own(activity) ? world_->activitySetpoint(activity) : std::nullopt;
}
std::optional<control::InterceptStatus> Agent::interceptStatus(control::ActivityId activity) const {
    FSIM_AGENT_AUDIT;
    return own(activity) ? world_->interceptStatus(activity) : std::nullopt;
}
std::vector<control::EndPoint> Agent::endPoints(control::ActivityId activity, std::size_t max) const {
    FSIM_AGENT_AUDIT;
    return own(activity) ? world_->endPoints(activity, max) : std::vector<control::EndPoint>{};
}

#define FSIM_AGENT_ACTIVITY_OF(activity) \
    if (!own(activity)) return FSIM_AGENT_REFUSED, refused(control::Reason::NotAllowed); \
    const control::Caller caller{control::Source::Policy, controller_}

control::CommandResult Agent::update(control::ActivityId activity, const control::Command& setpoint) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, setpoint);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::SupportCommand& setpoint) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, setpoint);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                     Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states,
                                     Span<const control::RoutePath> paths, Span<const control::RouteBranch> branches,
                                     Span<const control::RouteTerminator> terminators) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, route, waypoints, loiters, states, paths, branches, terminators);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                     const control::CurveShape* shape) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, curve, segments, shape);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                     const control::CurveShape* shape) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, curve, segments, shape);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, pattern, shape);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::MarshallCommand& marshall, const control::PatternShape& shape) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, marshall, shape);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::MarshallCommand& marshall) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, marshall);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, mustFly, zone);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpLine& line) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, mustFly, line);
}
control::CommandResult Agent::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpVolume& volume) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->update(caller, activity, mustFly, volume);
}
control::CommandResult Agent::cancel(control::ActivityId activity) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->cancel(caller, activity);
}
control::CommandResult Agent::activityCommand(control::ActivityId activity, control::ActivityCommand command, control::Rank rank) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->activityCommand(activity, command, rank, caller);
}
control::CommandResult Agent::commandBranch(control::ActivityId activity, std::uint32_t branch, bool commanded) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_ACTIVITY_OF(activity);
    return world_->commandBranch(activity, branch, commanded, caller);
}

control::Reason Agent::storeTask(control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition) {
    FSIM_AGENT_AUDIT;
    if (const auto* c = std::get_if<control::Command>(&command.command))
        if (const auto why = session::agentCommand(FSIM_AGENT_IDENTITY, *c, world_ ? &world_->impl() : nullptr);
            why != control::Reason::None)
            return why;
    return FSIM_AGENT_VEHICLE_OF.storeTask(task, command, repetition);
}
control::CommandResult Agent::commandTask(control::TaskId task, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_OPTIONS(options, refused);
    return FSIM_AGENT_VEHICLE_OF.commandTask(task, o);
}
control::CommandResult Agent::cancelTask(control::TaskId task) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.cancelTask(task, control::Caller{control::Source::Policy, controller_});
}
control::Reason Agent::removeTask(control::TaskId task) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.removeTask(task);
}
std::optional<control::TaskStatus> Agent::taskStatus(control::TaskId task) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.taskStatus(task);
}
std::vector<control::TaskStatus> Agent::tasks() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.tasks();
}
control::Reason Agent::publishPlan(const control::RoutePlan& plan) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.publishPlan(plan);
}
control::PlanCommandResult Agent::planCommand(control::PlanId plan, control::PlanCommand command, const control::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    control::CommandOptions o = options;
    if (const auto why = session::agentOptions(FSIM_AGENT_IDENTITY, o); why != control::Reason::None) {
        control::PlanCommandResult r;
        r.reason = why;
        return r;
    }
    return FSIM_AGENT_VEHICLE_OF.planCommand(plan, command, o);
}
control::Reason Agent::removePlan(control::PlanId plan) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.removePlan(plan);
}
std::optional<control::PlanStatus> Agent::planStatus(control::PlanId plan) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.planStatus(plan);
}
std::vector<control::PlanStatus> Agent::plans() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.plans();
}
std::optional<control::RoutePlan> Agent::plan(control::PlanId plan) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.plan(plan);
}
control::PlanValidationResult Agent::validatePlan(const control::RoutePlan& plan, const control::PlanValidation& v) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.validatePlan(plan, v);
}
control::PlanValidationResult Agent::validatePlan(control::PlanId plan, const control::PlanValidation& v) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.validatePlan(plan, v);
}
std::vector<control::Airfield> Agent::airfields() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.airfields();
}
std::optional<control::Airfield> Agent::airfield(control::AirfieldId airfield) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.airfield(airfield);
}

std::vector<ms::PayloadInfo> Agent::payloads() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.payloads();
}
std::vector<ms::CapabilityInfo> Agent::missionCapabilities() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.missionCapabilities();
}
ms::CapabilityStatus Agent::missionCapabilityStatus(std::string_view capability) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.missionCapabilityStatus(capability);
}
ms::CommandResult Agent::submit(const ms::StateCommand& command, const ms::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_MS_OPTIONS(options);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
ms::CommandResult Agent::submit(const ms::BitCommand& command, const ms::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_MS_OPTIONS(options);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
ms::CommandResult Agent::submit(const ms::AmtiCommand& command, const ms::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_MS_OPTIONS(options);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
ms::CommandResult Agent::submit(const ms::StrikeCommand& command, const ms::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_MS_OPTIONS(options);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
ms::CommandResult Agent::submit(const ms::JettisonCommand& command, const ms::CommandOptions& options) {
    FSIM_AGENT_AUDIT;
    FSIM_AGENT_MS_OPTIONS(options);
    return FSIM_AGENT_VEHICLE_OF.submit(command, o);
}
ms::RadarStatus Agent::radarStatus(std::uint16_t payload) const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.radarStatus(payload);
}
std::uint32_t Agent::designate(std::uint16_t payload, std::uint64_t track) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.designate(payload, track);
}
bool Agent::undesignate(std::uint32_t target) {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.undesignate(target);
}
std::vector<Designation> Agent::designations() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.designations();
}
std::vector<ms::AmtiDetection> Agent::amtiDetections(std::uint16_t payload, std::uint64_t since) const {
    FSIM_AGENT_AUDIT;
    auto list = FSIM_AGENT_VEHICLE_OF.amtiDetections(payload, since);
    for (auto& d : list) session::agentReport(d);
    return list;
}
std::vector<ms::AmtiTrack> Agent::amtiTracks(std::uint16_t payload) const {
    FSIM_AGENT_AUDIT;
    auto list = FSIM_AGENT_VEHICLE_OF.amtiTracks(payload);
    for (auto& t : list) session::agentReport(t);
    return list;
}
ms::CommandResult Agent::cancelMission(ms::ActivityId activity) {
    FSIM_AGENT_AUDIT;
    if (!own(activity)) return msRefused(ms::Reason::NotAllowed);
    return FSIM_AGENT_VEHICLE_OF.cancelMission(activity, control::Caller{control::Source::Policy, controller_});
}
std::optional<ms::ActivityRecord> Agent::missionActivity(ms::ActivityId activity) const {
    FSIM_AGENT_AUDIT;
    return own(activity) ? FSIM_AGENT_VEHICLE_OF.missionActivity(activity) : std::nullopt;
}
std::vector<ms::ActivityRecord> Agent::missionActivities() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.missionActivities();
}
std::vector<ms::StoreInfo> Agent::stores() const {
    FSIM_AGENT_AUDIT;
    return FSIM_AGENT_VEHICLE_OF.stores();
}
std::optional<ms::StrikeStatus> Agent::strikeStatus(ms::ActivityId activity) const {
    FSIM_AGENT_AUDIT;
    return own(activity) ? FSIM_AGENT_VEHICLE_OF.strikeStatus(activity) : std::nullopt;
}

comm::SystemId Agent::system() const noexcept {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().systemOf(vehicle_) : comm::kNoSystem;
}
std::vector<comm::NetInfo> Agent::nets() const {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().netsOf(vehicle_) : std::vector<comm::NetInfo>{};
}
std::uint64_t Agent::send(const comm::DmsMessage& message, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().dms().send(vehicle_, message, world_->time(), reason) : 0;
    if (why) *why = reason;
    if (id == 0) FSIM_AGENT_REFUSED;
    return id;
}
bool Agent::subscribe(comm::Topic topic) {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().dms().subscribe(vehicle_, topic, true);
}
bool Agent::unsubscribe(comm::Topic topic) {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().dms().subscribe(vehicle_, topic, false);
}
bool Agent::subscribeAll(bool all) {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().dms().subscribeAll(vehicle_, all);
}
std::vector<comm::Topic> Agent::subscriptions() const {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().subscriptions(vehicle_) : std::vector<comm::Topic>{};
}
bool Agent::subscribedAll() const {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().dms().subscribedAll(vehicle_);
}
std::vector<comm::DmsReceived> Agent::inbox() const {
    FSIM_AGENT_AUDIT;
    const auto* in = world_ ? world_->impl().dms().inbox(vehicle_) : nullptr;
    return in ? std::vector<comm::DmsReceived>(in->begin(), in->end()) : std::vector<comm::DmsReceived>{};
}
std::vector<comm::DmsReceived> Agent::receive() {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().take(vehicle_) : std::vector<comm::DmsReceived>{};
}
std::optional<comm::DmsTransmission> Agent::transmission(std::uint64_t message) const {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().transmission(vehicle_, message) : std::nullopt;
}
std::uint64_t Agent::publishPosition(const std::vector<comm::SystemId>& destinations, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const auto ids = world_ ? world_->impl().publishNative(vehicle_, comm::topic::kPositionReport, 0, destinations, reason)
                            : std::vector<std::uint64_t>{};
    if (why) *why = reason;
    if (ids.empty()) FSIM_AGENT_REFUSED;
    return ids.empty() ? 0 : ids.front();
}
std::uint64_t Agent::publishStatus(const std::vector<comm::SystemId>& destinations, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const auto ids = world_ ? world_->impl().publishNative(vehicle_, comm::topic::kSystemStatus, 0, destinations, reason)
                            : std::vector<std::uint64_t>{};
    if (why) *why = reason;
    if (ids.empty()) FSIM_AGENT_REFUSED;
    return ids.empty() ? 0 : ids.front();
}
std::vector<std::uint64_t> Agent::publishTracks(std::uint16_t payload, const std::vector<comm::SystemId>& destinations, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    auto ids = world_ ? world_->impl().publishNative(vehicle_, comm::topic::kEntity, payload, destinations, reason)
                      : std::vector<std::uint64_t>{};
    if (why) *why = reason;
    return ids;
}
std::vector<comm::PeerReport> Agent::peers() const {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().peers(vehicle_) : std::vector<comm::PeerReport>{};
}
std::vector<comm::LinkTrack> Agent::linkTracks() const {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().dms().linkTracks(vehicle_) : std::vector<comm::LinkTrack>{};
}
std::uint32_t Agent::designateLink(comm::SystemId source, std::uint16_t payload, std::uint64_t track) {
    FSIM_AGENT_AUDIT;
    return world_ ? world_->impl().designateLink(vehicle_, source, payload, track) : 0;
}

// --- C2 authority (AB-5) -----------------------------------------------------------------------------------------------
C2Authority Agent::authority() const {
    FSIM_AGENT_AUDIT;
    C2Authority out;
    const auto* a = world_ ? world_->impl().maCore().authority(vehicle_) : nullptr;
    if (!a) return out;
    const comm::RbacConfig rbac = world_->impl().maCore().rbac(vehicle_);
    for (const auto& o : a->operators) {
        C2Authority::Operator op{o.system, o.roleId, {}, o.callsign};
        for (const auto& r : rbac.roles)
            if (r.id == o.roleId) op.role = r.name;
        out.operators.push_back(op);
    }
    out.designated = a->designated, out.designatedModes = a->designatedModes, out.designatedBy = a->designatedBy;
    out.designatedS = a->designatedS;
    for (const auto& c : a->c2) out.commands.push_back({c.activity, c.system, c.commandId});
    out.lostCommsSet = a->lostCommsSet, out.lostCommsThresholdS = a->lostComms.thresholdS;
    out.lostCommsPlan = a->lostComms.activatePlan ? a->lostComms.planId : 0;
    out.lost = a->lost, out.lostSinceS = a->lostSinceS, out.alerts = a->alerts;
    out.packageId = a->packageId, out.leader = a->leader, out.members = a->members;
    return out;
}
bool Agent::declareRole(std::uint32_t roleId, std::string_view callsign, const std::vector<comm::SystemId>& destinations, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const bool sent = world_ && world_->impl().maCore().declareRole(world_->impl(), vehicle_, roleId, std::string(callsign), destinations, reason);
    if (why) *why = reason;
    return sent;
}
std::uint64_t Agent::designateControl(comm::SystemId ma, std::uint32_t modes, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().designateControl(world_->impl(), vehicle_, ma, modes, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::uint64_t Agent::commandHsa(comm::SystemId ma, const control::HsaCommand& command, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::FlightCommand c;
    c.kind = comm::FlightCommand::Hsa;
    c.headingRad = command.headingRad, c.courseRad = command.courseRad, c.speed = command.speed;
    c.speedReference = command.speedReference, c.altitudeM = command.altitudeM, c.altitudeReference = command.altitudeReference;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().flightCommand(world_->impl(), vehicle_, ma, c, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::uint64_t Agent::commandRoute(comm::SystemId ma, Span<const control::Waypoint> waypoints, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    if (waypoints.size() == 0 || waypoints.size() > 16) {
        if (why) *why = comm::DmsReason::TooLarge;
        return 0;
    }
    comm::FlightCommand c;
    c.kind = comm::FlightCommand::Waypoints;
    c.pointCount = static_cast<std::uint32_t>(waypoints.size());
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const control::Waypoint& w = waypoints[i];
        c.points[i][0] = w.latitudeRad, c.points[i][1] = w.longitudeRad, c.points[i][2] = w.altitudeM, c.points[i][3] = w.speed;
    }
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().flightCommand(world_->impl(), vehicle_, ma, c, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::uint64_t Agent::cancelCommand(comm::SystemId ma, std::uint64_t commandId, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::FlightCommand c;
    c.kind = comm::FlightCommand::Cancel, c.commandId = commandId;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().flightCommand(world_->impl(), vehicle_, ma, c, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::vector<comm::CommandStatusEntry> Agent::commandStatuses() const {
    FSIM_AGENT_AUDIT;
    const auto* s = world_ ? world_->impl().dms().commandStatuses(vehicle_) : nullptr;
    return s ? *s : std::vector<comm::CommandStatusEntry>{};
}
std::vector<comm::ReceivedTask> Agent::c2Tasks() const {
    FSIM_AGENT_AUDIT;
    const auto* t = world_ ? world_->impl().maCore().tasks(vehicle_) : nullptr;
    return t ? *t : std::vector<comm::ReceivedTask>{};
}
bool Agent::acceptTask(comm::SystemId from, std::uint64_t taskId) {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().maCore().answerTask(world_->impl(), vehicle_, from, taskId, comm::ReceivedTask::Accepted, 0,
                                                       std::numeric_limits<double>::quiet_NaN());
}
bool Agent::rejectTask(comm::SystemId from, std::uint64_t taskId, std::uint32_t code) {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().maCore().answerTask(world_->impl(), vehicle_, from, taskId, comm::ReceivedTask::Rejected, code,
                                                       std::numeric_limits<double>::quiet_NaN());
}
bool Agent::reportTask(comm::SystemId from, std::uint64_t taskId, comm::ReceivedTask::State state, double percent) {
    FSIM_AGENT_AUDIT;
    return world_ && world_->impl().maCore().answerTask(world_->impl(), vehicle_, from, taskId, state, 0, percent);
}
std::vector<comm::ActiveOverride> Agent::overrides() const {
    FSIM_AGENT_AUDIT;
    const auto* o = world_ ? world_->impl().maCore().overrides(vehicle_) : nullptr;
    return o ? *o : std::vector<comm::ActiveOverride>{};
}
std::uint64_t Agent::sendTask(comm::SystemId ma, const comm::TaskOrder& task, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().sendTask(world_->impl(), vehicle_, ma, task, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::uint64_t Agent::setOverride(comm::SystemId ma, const comm::OverrideCommand& command, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().sendOverride(world_->impl(), vehicle_, ma, command, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::vector<comm::PendingConsent> Agent::consentRequests() const {
    FSIM_AGENT_AUDIT;
    const auto* c = world_ ? world_->impl().dms().consentsOf(vehicle_) : nullptr;
    return c ? *c : std::vector<comm::PendingConsent>{};
}
std::uint64_t Agent::setLostComms(comm::SystemId ma, double thresholdS, std::uint64_t planId, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::LostCommsSettings l;
    l.thresholdS = thresholdS, l.activatePlan = planId ? 1u : 0u, l.planId = planId;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().sendLostComms(world_->impl(), vehicle_, ma, l, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::uint64_t Agent::formPackage(std::uint32_t packageId, comm::SystemId leader, const std::vector<comm::SystemId>& members, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().formPackage(world_->impl(), vehicle_, packageId, leader, members, reason) : 0;
    if (why) *why = reason;
    return id;
}
std::uint64_t Agent::dissolvePackage(std::uint32_t packageId, const std::vector<comm::SystemId>& systems, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const std::uint64_t id = world_ ? world_->impl().maCore().dissolvePackage(world_->impl(), vehicle_, packageId, systems, reason) : 0;
    if (why) *why = reason;
    return id;
}
bool Agent::answerConsent(comm::SystemId ma, std::uint64_t requestId, bool granted, comm::DmsReason* why) {
    FSIM_AGENT_AUDIT;
    comm::DmsReason reason = comm::DmsReason::NoSystem;
    const bool sent = world_ && world_->impl().maCore().answerConsent(world_->impl(), vehicle_, ma, requestId, granted, reason);
    if (why) *why = reason;
    return sent;
}

// (its own, then the common: AB-2c - the world's mission data, as its vehicle sees it)
#define FSIM_AGENT_SESSION world_->impl()
control::Reason Agent::setOpPoint(const control::OpPoint& point) {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.setOpPoint(point, vehicle_) : control::Reason::UnknownVehicle;
}
bool Agent::removeOpPoint(control::OpPointId id) {
    FSIM_AGENT_AUDIT;
    return world_ && FSIM_AGENT_SESSION.removeOpPoint(id, vehicle_);
}
control::Reason Agent::setOpZone(const control::OpZone& zone) {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.setOpZone(zone, vehicle_) : control::Reason::UnknownVehicle;
}
bool Agent::removeOpZone(control::OpZoneId id) {
    FSIM_AGENT_AUDIT;
    return world_ && FSIM_AGENT_SESSION.removeOpZone(id, vehicle_);
}
control::Reason Agent::setOpLine(const control::OpLine& line) {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.setOpLine(line, vehicle_) : control::Reason::UnknownVehicle;
}
bool Agent::removeOpLine(control::OpLineId id) {
    FSIM_AGENT_AUDIT;
    return world_ && FSIM_AGENT_SESSION.removeOpLine(id, vehicle_);
}
control::Reason Agent::setOpVolume(const control::OpVolume& volume) {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.setOpVolume(volume, vehicle_) : control::Reason::UnknownVehicle;
}
bool Agent::removeOpVolume(control::OpVolumeId id) {
    FSIM_AGENT_AUDIT;
    return world_ && FSIM_AGENT_SESSION.removeOpVolume(id, vehicle_);
}
control::FrameId Agent::createFrame(const control::FrameSpec& spec) {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.createFrame(spec, vehicle_) : 0;
}
bool Agent::removeFrame(control::FrameId id) {
    FSIM_AGENT_AUDIT;
    return world_ && FSIM_AGENT_SESSION.removeFrame(id, vehicle_);
}
std::vector<control::FrameId> Agent::frames() const {
    FSIM_AGENT_AUDIT;
    std::vector<control::FrameId> out;
    if (world_)
        for (const control::FrameId id : FSIM_AGENT_SESSION.frames(vehicle_))
            if (frame(id)) out.push_back(id); // (not one following another aircraft)
    return out;
}
std::optional<control::FrameSpec> Agent::frame(control::FrameId id) const {
    FSIM_AGENT_AUDIT;
    if (!world_) return std::nullopt;
    const auto spec = FSIM_AGENT_SESSION.frame(id, vehicle_);
    if (spec && spec->vehicle != 0 && spec->vehicle != vehicle_) { // (L4: another aircraft's true pose - but its own designations)
        const auto* d = FSIM_AGENT_SESSION.designation(spec->vehicle);
        if (!d || d->owner != vehicle_) return std::nullopt;
    }
    return spec;
}
std::vector<control::OpPointId> Agent::opPoints() const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opPoints(vehicle_) : std::vector<control::OpPointId>{};
}
std::optional<control::OpPoint> Agent::opPoint(control::OpPointId id) const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opPoint(id, vehicle_) : std::nullopt;
}
std::vector<control::OpZoneId> Agent::opZones() const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opZones(vehicle_) : std::vector<control::OpZoneId>{};
}
std::optional<control::OpZone> Agent::opZone(control::OpZoneId id) const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opZone(id, vehicle_) : std::nullopt;
}
std::vector<control::OpLineId> Agent::opLines() const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opLines(vehicle_) : std::vector<control::OpLineId>{};
}
std::optional<control::OpLine> Agent::opLine(control::OpLineId id) const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opLine(id, vehicle_) : std::nullopt;
}
std::vector<control::OpVolumeId> Agent::opVolumes() const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opVolumes(vehicle_) : std::vector<control::OpVolumeId>{};
}
std::optional<control::OpVolume> Agent::opVolume(control::OpVolumeId id) const {
    FSIM_AGENT_AUDIT;
    return world_ ? FSIM_AGENT_SESSION.opVolume(id, vehicle_) : std::nullopt;
}
std::optional<control::GeoPoint> Agent::framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS) const {
    FSIM_AGENT_AUDIT;
    if (!frame(id)) return std::nullopt;
    return FSIM_AGENT_SESSION.framePoint(id, offset, timeS, vehicle_);
}
#undef FSIM_AGENT_SESSION

#undef FSIM_AGENT_REFUSED
#undef FSIM_AGENT_AUDIT
#undef FSIM_AGENT_ACTIVITY_OF
#undef FSIM_AGENT_MS_OPTIONS
#undef FSIM_AGENT_OPTIONS
#undef FSIM_AGENT_VEHICLE_OF
#undef FSIM_AGENT_IDENTITY

// --- Referee ---------------------------------------------------------------------------

double Referee::time() const noexcept { return world_ ? world_->time() : 0.0; }

std::uint32_t Referee::systemEntity(comm::SystemId system) const noexcept { return world_ ? const_cast<World*>(world_)->impl().dms().entityOf(system) : 0u; }
comm::SystemId Referee::entitySystem(std::uint32_t entity) const noexcept {
    return world_ ? const_cast<World*>(world_)->impl().dms().systemOf(entity) : comm::kNoSystem;
}
std::string Referee::tailNumber(comm::SystemId system) const { return world_ ? const_cast<World*>(world_)->impl().dms().tailNumber(system) : std::string(); }
std::uint32_t Referee::linkTrackTruth(comm::SystemId source, std::uint16_t payload, std::uint64_t track) const {
    return world_ ? trackTruth(const_cast<World*>(world_)->impl().dms().entityOf(source), payload, track) : 0u;
}
std::vector<comm::NetInfo> Referee::nets() const { return world_ ? const_cast<World*>(world_)->impl().dms().nets() : std::vector<comm::NetInfo>{}; }
comm::NetStats Referee::netStats(comm::NetId net) const noexcept { return world_ ? const_cast<World*>(world_)->impl().dms().stats(net) : comm::NetStats{}; }
std::vector<std::uint32_t> Referee::c2Nodes() const { return world_ ? const_cast<World*>(world_)->impl().c2Nodes() : std::vector<std::uint32_t>{}; }
std::optional<comm::C2NodeSpec> Referee::c2Node(std::uint32_t node) const {
    const comm::C2NodeSpec* n = world_ ? const_cast<World*>(world_)->impl().c2Node(node) : nullptr;
    return n ? std::optional<comm::C2NodeSpec>(*n) : std::nullopt;
}

std::vector<std::uint32_t> Referee::vehicles() const {
    std::vector<std::uint32_t> ids;
    if (!world_) return ids;
    const auto& w = const_cast<World*>(world_)->impl();
    ids.resize(w.vehicles(nullptr, 0));
    ids.resize(w.vehicles(ids.data(), static_cast<std::uint32_t>(ids.size())));
    return ids;
}

const VehicleState* Referee::state(std::uint32_t vehicle) const noexcept {
    return world_ ? const_cast<World*>(world_)->impl().vehicleState(vehicle) : nullptr;
}

std::vector<ms::WeaponRecord> Referee::weapons() const { return world_ ? world_->weapons() : std::vector<ms::WeaponRecord>{}; }

std::optional<ms::RadioPath> Referee::radioPath(std::uint32_t from, std::uint32_t to, const ms::RadioPathOptions& options) const {
    if (!world_) return std::nullopt;
    auto* w = const_cast<World*>(world_);
    return world_->radioPath(w->vehicle(from), w->vehicle(to), options);
}

std::optional<ms::RadarCrossSection> Referee::radarCrossSection(std::uint32_t observer, std::uint32_t target, const ms::RcsQuery& query) const {
    if (!world_) return std::nullopt;
    auto* w = const_cast<World*>(world_);
    return world_->radarCrossSection(w->vehicle(observer), w->vehicle(target), query);
}

std::uint32_t Referee::trackTruth(std::uint32_t vehicle, std::uint16_t payload, std::uint64_t track) const {
    if (!world_) return 0;
    for (const ms::AmtiTrack& t : const_cast<World*>(world_)->impl().amtiTracks(vehicle, payload))
        if (t.id == track) return t.truth;
    return 0;
}

std::uint32_t Referee::detectionTruth(std::uint32_t vehicle, std::uint16_t payload, std::uint64_t serial) const {
    if (!world_) return 0;
    for (const ms::AmtiDetection& d : const_cast<World*>(world_)->impl().amtiDetections(vehicle, payload, serial))
        if (d.serial == serial) return d.truth;
    return 0;
}

std::uint32_t Referee::designationTruth(std::uint32_t target) const {
    if (!world_) return 0;
    const auto* d = const_cast<World*>(world_)->impl().designation(target);
    if (!d || !d->valid) return 0;
    return d->source != comm::kNoSystem ? linkTrackTruth(d->source, d->payload, d->track) : trackTruth(d->owner, d->payload, d->track);
}

std::vector<Referee::AgentAudit> Referee::audit() const {
    std::vector<AgentAudit> out;
    if (!world_) return out;
    const std::vector<std::string> names = session::auditNames();
    for (const auto& a : const_cast<World*>(world_)->impl().agents()) {
        AgentAudit audit{{a.vehicle, a.controller, a.name}, {}};
        for (std::size_t i = 0; i < a.calls.size(); ++i)
            if (a.calls[i] || a.refused[i]) audit.calls.push_back({i < names.size() ? names[i] : std::string("?"), a.calls[i], a.refused[i]});
        out.push_back(std::move(audit));
    }
    return out;
}

std::vector<Referee::AgentInfo> Referee::agents() const {
    std::vector<AgentInfo> out;
    if (world_)
        for (const auto& a : const_cast<World*>(world_)->impl().agents()) out.push_back({a.vehicle, a.controller, a.name});
    return out;
}

} // namespace fsim
