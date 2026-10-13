#pragma once

// fsim SDK: agents and the referee (ADR-31, docs/agent-boundary.md; docs/sdk/boundary.md).
//
// An Agent is a mission autonomy's decisions bound to one aircraft: what an MA may read of its own platform and do with
// it, as itself. The range (whoever holds the World) makes one with World::createAgent and hands it to the MA's code;
// nothing an Agent returns leads back to the World, a Vehicle, the network or the referee. Its commands are the
// policy's, from a controller of its own: options or a caller claiming another source or controller are refused
// (NotAllowed), 0 meaning its own. It addresses its own vehicle's activities alone, names no other aircraft by its
// entity id (a behaviour's or a must fly's target; targets by track come with AB-2b), asks for release consent rather
// than giving it (the range stands in for C2, which gives it), and reads its detections and tracks without the entity
// behind them (truth 0: the referee's).
//
// A Referee is a read-only view of everything, truth among it - for scoring, rewards, a centralised critic.

#include "fsim/Capability.h"
#include "fsim/Comm.h"
#include "fsim/Control.h"
#include "fsim/Export.h"
#include "fsim/Frames.h"
#include "fsim/MAReports.h"
#include "fsim/MAService.h"
#include "fsim/MissionSystems.h"
#include "fsim/PerformanceProfile.h"
#include "fsim/Span.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace fsim {

class World;

/// Its own ship as its platform's navigation and air data report it (ADR-31, B3): the vehicle's sensed state - through
/// its sensor effects (GNSS degradation, noise, latency) - without what no platform reports (a crash or divergence
/// flag, the step count, an injected error's size). Equal to truth where the models are perfect.
struct OwnShip {
    double timeS = 0.0;
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeMslM = 0.0, altitudeAglM = 0.0;
    double velocityNedMs[3] = {0, 0, 0};       ///< north, east, down
    double eulerRad[3] = {0, 0, 0};            ///< roll, pitch, yaw
    double angularRateBodyRadS[3] = {0, 0, 0}; ///< p, q, r
    double accelerationBodyMs2[3] = {0, 0, 0};
    double airspeedTrueMs = 0.0, airspeedCalibratedMs = 0.0, mach = 0.0, alphaRad = 0.0, betaRad = 0.0, loadFactor = 1.0;
    double fuelKg = 0.0;
    bool onGround = false;
    bool gnssValid = true, airDataValid = true; ///< its navigation and air data sensors' own validity
    double throttle = 0.0; ///< its (first) engine's throttle as set, 0 .. 1 (AB-3; 0 without an engine)
};

/// A target designated by a vehicle's own radar track (ADR-31, AB-2b; A-GRA's target designation): its id - not an
/// entity's - is what a behaviour's, a must fly's or a strike's target may be. Flown on the track's estimate, never the
/// aircraft behind it; once the track is gone it lapses (`valid` false) and the behaviour has no target.
struct Designation {
    std::uint32_t target = 0;  ///< its id, from 0x80000001
    std::uint16_t payload = 0; ///< the radar whose track it is
    std::uint64_t track = 0;   ///< that radar's numbering
    bool valid = false;
    comm::SystemId source = comm::kNoSystem; ///< a network track's source system (AB-4b); 0: its own radar's track
};

/// The authority over an MA's platform, as its MA core keeps it (AB-5): the C2 systems that declared a role to it, C2's
/// control designation, and the direct commands C2 flies on it - while one flies, the agent's own commands on its axes
/// are refused (authority_held: its `other` the C2 command's activity) and it may not update or cancel it.
struct C2Authority {
    struct Operator {
        comm::SystemId system = comm::kNoSystem;
        std::uint32_t roleId = 0;
        std::string role;     ///< the role's name in its RBAC configuration
        std::string callsign;
    };
    struct Command {
        control::ActivityId activity = 0;
        comm::SystemId system = comm::kNoSystem; ///< the C2 system whose command it is
        std::uint64_t commandId = 0;
    };
    std::vector<Operator> operators;
    bool designated = false;           ///< C2 has designated its flight modes (the rest revoked; control granted to it)
    std::uint32_t designatedModes = 0; ///< 1 << control::FlightMode each
    comm::SystemId designatedBy = comm::kNoSystem;
    double designatedS = 0.0;
    std::vector<Command> commands;
    // AB-5c: the lost-communication contingency C2 configured, and whether it is raised; its team
    bool lostCommsSet = false;
    double lostCommsThresholdS = 0.0;
    std::uint64_t lostCommsPlan = 0;   ///< the route plan its response activates (0: the alert alone)
    bool lost = false;                 ///< nothing heard from C2 for the threshold: the contingency raised
    double lostSinceS = 0.0;           ///< when it last heard from C2, if lost
    std::uint32_t alerts = 0;          ///< contingency alerts raised
    std::uint32_t packageId = 0;       ///< its team (0: none)
    comm::SystemId leader = comm::kNoSystem;
    std::vector<comm::SystemId> members;
};

namespace ma {
/// What a platform's navigation reports: its own ship, and its endurance.
struct Navigation {
    OwnShip ownShip;
    control::NavigationReport endurance;
};
} // namespace ma

class FSIM_API Agent {
public:
    Agent() = default;
    /// Its vehicle (or C2 node) is alive.
    bool valid() const noexcept;
    /// Its own vehicle's id - or its C2 node's (AB-4) - and its controller, minted by the range.
    std::uint32_t vehicle() const noexcept { return vehicle_; }
    control::ControllerId controller() const noexcept { return controller_; }
    const std::string& name() const noexcept { return name_; }
    std::string type() const;

    // Own ship, through its platform's reports
    double time() const noexcept; ///< its on-board clock: the simulation's seconds
    /// Where it is and how it moves, as its navigation and air data report it.
    OwnShip ownship() const;
    control::NavigationReport navigationReport() const;
    control::NavigationSettings navigation() const;
    control::Reason setNavigation(const control::NavigationSettings& settings);
    control::Reason setQnh(double qnhPa);
    double qnh() const;
    control::StateData stateData() const;
    double massKg() const;
    /// The on-board terrain database (VI 1.2.6.9, A-GRA's elevation request).
    std::optional<double> terrainHeightM(double latitudeRad, double longitudeRad) const;

    // Discovery
    std::vector<control::CapabilityDescriptor> capabilities() const;
    control::CapabilityStatus capabilityStatus(std::string_view capability) const;
    const control::SupportInfo* support(std::string_view feature) const;
    std::vector<control::SupportInfo> supportTable() const;
    control::Performance performance() const;
    control::Reason performanceProfile(control::FlightMode mode, control::PerformanceProfile& out) const;
    std::uint32_t capabilityPrecedence(std::string_view capability) const;
    control::ControlMode controlMode() const;
    control::VehicleDefault vehicleDefault() const;
    control::ProtectionMode protection() const;
    std::uint32_t controlRevision() const;

    // Grants (its own controller's)
    control::Reason requestControl(std::string_view capability);
    control::Reason releaseControl(std::string_view capability);
    control::ControlStatus controlStatus(std::string_view capability) const;

    // NEW
    control::CommandResult submit(const control::Command& command, const control::CommandOptions& options = {});
    control::CommandResult submit(const control::SupportCommand& command, const control::CommandOptions& options = {});
    template <typename C>
    control::CommandResult submit(const C& c, const control::CommandOptions& options = {}) {
        if constexpr (std::is_constructible_v<control::SupportCommand, C> && !std::is_constructible_v<control::Command, C>)
            return submit(control::SupportCommand(c), options);
        else
            return submit(control::Command(c), options);
    }
    control::CommandResult submit(const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  const control::CommandOptions& options = {}, Span<const control::RouteLoiter> loiters = {},
                                  Span<const control::RouteState> states = {}, Span<const control::RoutePath> paths = {},
                                  Span<const control::RouteBranch> branches = {}, Span<const control::RouteTerminator> terminators = {});
    control::CommandResult submit(const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    control::CommandResult submit(const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    control::CommandResult submit(const control::PatternCommand& pattern, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    control::CommandResult submit(const control::MarshallCommand& marshall, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    control::CommandResult submit(const control::InterceptCommand& intercept, const control::CommandOptions& options = {});
    control::CommandResult submit(const control::MustFlyCommand& mustFly, const control::OpZone& zone, const control::CommandOptions& options = {});
    control::CommandResult submit(const control::MustFlyCommand& mustFly, const control::OpLine& line, const control::CommandOptions& options = {});
    control::CommandResult submit(const control::MustFlyCommand& mustFly, const control::OpVolume& volume, const control::CommandOptions& options = {});
    std::vector<control::CommandResult> submitBatch(Span<const control::BatchCommand> batch, std::vector<control::CommandDetails>* details = nullptr);

    // Its activities (its own vehicle's; another's is refused NotAllowed, or reads empty)
    std::vector<control::ActivityRecord> activities() const;
    control::CommandDetails commandDetails() const;
    control::VehicleCommandState commanded() const;
    std::optional<control::ActivityRecord> activity(control::ActivityId activity) const;
    std::optional<control::Setpoint> activitySetpoint(control::ActivityId activity) const;
    std::optional<control::InterceptStatus> interceptStatus(control::ActivityId activity) const;
    std::vector<control::EndPoint> endPoints(control::ActivityId activity, std::size_t max = 16) const;
    control::CommandResult update(control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::ActivityId activity, const control::SupportCommand& setpoint);
    template <typename C>
    control::CommandResult update(control::ActivityId activity, const C& c) {
        if constexpr (std::is_constructible_v<control::SupportCommand, C> && !std::is_constructible_v<control::Command, C>)
            return update(activity, control::SupportCommand(c));
        else
            return update(activity, control::Command(c));
    }
    control::CommandResult update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  Span<const control::RouteLoiter> loiters = {}, Span<const control::RouteState> states = {},
                                  Span<const control::RoutePath> paths = {}, Span<const control::RouteBranch> branches = {},
                                  Span<const control::RouteTerminator> terminators = {});
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape);
    control::CommandResult update(control::ActivityId activity, const control::MarshallCommand& marshall, const control::PatternShape& shape);
    control::CommandResult update(control::ActivityId activity, const control::MarshallCommand& marshall);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpLine& line);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpVolume& volume);
    control::CommandResult cancel(control::ActivityId activity);
    control::CommandResult activityCommand(control::ActivityId activity, control::ActivityCommand command, control::Rank rank = {});
    control::CommandResult commandBranch(control::ActivityId activity, std::uint32_t branch, bool commanded = true);

    // Tasks and route plans
    control::Reason storeTask(control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition = {});
    control::CommandResult commandTask(control::TaskId task, const control::CommandOptions& options = {});
    control::CommandResult cancelTask(control::TaskId task);
    control::Reason removeTask(control::TaskId task);
    std::optional<control::TaskStatus> taskStatus(control::TaskId task) const;
    std::vector<control::TaskStatus> tasks() const;
    control::Reason publishPlan(const control::RoutePlan& plan);
    control::PlanCommandResult planCommand(control::PlanId plan, control::PlanCommand command, const control::CommandOptions& options = {});
    control::Reason removePlan(control::PlanId plan);
    std::optional<control::PlanStatus> planStatus(control::PlanId plan) const;
    std::vector<control::PlanStatus> plans() const;
    std::optional<control::RoutePlan> plan(control::PlanId plan) const;
    control::PlanValidationResult validatePlan(const control::RoutePlan& plan, const control::PlanValidation& v = {});
    control::PlanValidationResult validatePlan(control::PlanId plan, const control::PlanValidation& v = {});
    std::vector<control::Airfield> airfields() const;
    std::optional<control::Airfield> airfield(control::AirfieldId airfield) const;

    // Mission Systems
    std::vector<ms::PayloadInfo> payloads() const;
    std::vector<ms::CapabilityInfo> missionCapabilities() const;
    ms::CapabilityStatus missionCapabilityStatus(std::string_view capability) const;
    ms::CommandResult submit(const ms::StateCommand& command, const ms::CommandOptions& options = {});
    ms::CommandResult submit(const ms::BitCommand& command, const ms::CommandOptions& options = {});
    ms::CommandResult submit(const ms::AmtiCommand& command, const ms::CommandOptions& options = {});
    /// A strike: its target 0 (by track: AB-2b) and its consent not given - the strike asks for it.
    ms::CommandResult submit(const ms::StrikeCommand& command, const ms::CommandOptions& options = {});
    ms::CommandResult submit(const ms::JettisonCommand& command, const ms::CommandOptions& options = {});
    ms::RadarStatus radarStatus(std::uint16_t payload) const;
    /// Designate one of its radar's tracks as a target (AB-2b): the target id its behaviours, must flies and strikes
    /// name, or 0 for a track it does not hold now.
    std::uint32_t designate(std::uint16_t payload, std::uint64_t track);
    bool undesignate(std::uint32_t target);
    std::vector<Designation> designations() const;
    /// Its radar's detections and tracks, `truth` 0: the entity behind each is the referee's.
    std::vector<ms::AmtiDetection> amtiDetections(std::uint16_t payload, std::uint64_t since = 0) const;
    std::vector<ms::AmtiTrack> amtiTracks(std::uint16_t payload) const;
    ms::CommandResult cancelMission(ms::ActivityId activity);
    std::optional<ms::ActivityRecord> missionActivity(ms::ActivityId activity) const;
    std::vector<ms::ActivityRecord> missionActivities() const;
    std::vector<ms::StoreInfo> stores() const;
    std::optional<ms::StrikeStatus> strikeStatus(ms::ActivityId activity) const;

    // Messages (AB-4): its system's DMS - over the nets the range put it on, stamped with its own SystemID; other systems
    // named by theirs. Its own topics from comm::topic::kFreeForm (a native type's is the MA core's: AB-4b).
    comm::SystemId system() const noexcept; ///< 0 until the range puts it on a net
    std::vector<comm::NetInfo> nets() const;
    /// Publish (no destinations: to every system subscribed to its topic, one attempt each) or send reliably to each
    /// destination: its message number, or 0 refused (`why`).
    std::uint64_t send(const comm::DmsMessage& message, comm::DmsReason* why = nullptr);
    /// What it receives of what is published (sent to it, it always receives): every topic (the default), or those it
    /// subscribed to once subscribeAll(false).
    bool subscribe(comm::Topic topic);
    bool unsubscribe(comm::Topic topic);
    bool subscribeAll(bool all);
    std::vector<comm::Topic> subscriptions() const;
    bool subscribedAll() const;
    /// What reached it - each once - and waits for it, oldest first (up to comm::kInboxCapacity: then its oldest go).
    std::vector<comm::DmsReceived> inbox() const;
    /// Take what waits: inbox(), emptied.
    std::vector<comm::DmsReceived> receive();
    /// A send's status: each destination's (reliable), how many it reached (published).
    std::optional<comm::DmsTransmission> transmission(std::uint64_t message) const;
    /// A-GRA's native messages (AB-4b), sent by its MA core from its platform's own reports - never its own bytes: its
    /// position report (its own ship as its navigation reports it), its heartbeat, its radar's tracks (each an entity
    /// report; `payload` 0xFFFF: every radar's). Published, or to `destinations`; the message numbers (none: refused).
    std::uint64_t publishPosition(const std::vector<comm::SystemId>& destinations = {}, comm::DmsReason* why = nullptr);
    std::uint64_t publishStatus(const std::vector<comm::SystemId>& destinations = {}, comm::DmsReason* why = nullptr);
    std::vector<std::uint64_t> publishTracks(std::uint16_t payload = 0xFFFF, const std::vector<comm::SystemId>& destinations = {},
                                             comm::DmsReason* why = nullptr);
    /// Its picture (AB-4b): each peer it heard (its last position report and heartbeat), each track received over a
    /// link - each dropped once older than its Data Dissemination Plan keeps them.
    std::vector<comm::PeerReport> peers() const;
    std::vector<comm::LinkTrack> linkTracks() const;
    /// Designate a received track as a target (B4: a network track): flown on its last report carried forward by its
    /// velocity, lapsing when its picture drops it. 0 for one it does not hold now.
    std::uint32_t designateLink(comm::SystemId source, std::uint16_t payload, std::uint64_t track);

    // C2 authority (AB-5). As an MA: the authority over its platform, as its MA core keeps it.
    C2Authority authority() const;
    // As a C2 system (a C2 node's agent - or any whose role the receiving MA authorizes): A-GRA's C2 messages, typed,
    // through its DMS; each MA's core authorizes them by the role declared to it, and answers (commandStatuses()).
    /// Declare its role (OperatorMT) to the MAs given (none: published to every system on its nets).
    bool declareRole(std::uint32_t roleId, std::string_view callsign, const std::vector<comm::SystemId>& destinations = {},
                     comm::DmsReason* why = nullptr);
    /// The flight modes an MA may command (1 << control::FlightMode each): its command id, 0 if not sent.
    std::uint64_t designateControl(comm::SystemId ma, std::uint32_t modes, comm::DmsReason* why = nullptr);
    /// A direct HSA/CSA command, or one along waypoints (each its latitude, longitude, altitude and speed - kHold: the
    /// aircraft's choice; at most 16): its command id, 0 if not sent.
    std::uint64_t commandHsa(comm::SystemId ma, const control::HsaCommand& command, comm::DmsReason* why = nullptr);
    std::uint64_t commandRoute(comm::SystemId ma, Span<const control::Waypoint> waypoints, comm::DmsReason* why = nullptr);
    /// Cancel a direct command it sent.
    std::uint64_t cancelCommand(comm::SystemId ma, std::uint64_t commandId, comm::DmsReason* why = nullptr);
    /// Its commands' last statuses, as the MAs answered.
    std::vector<comm::CommandStatusEntry> commandStatuses() const;

    // AB-5b. As an MA: C2's tasks - its to decide on - and the overrides in force on its flight (its own HSA and routes fly
    // them). Its strikes' release consent its MA core asks C2 for, as its platform's policy says.
    std::vector<comm::ReceivedTask> c2Tasks() const;
    /// Accept or reject a received task (`code`: its own reason, sent to C2); false for one it does not hold, or decided.
    bool acceptTask(comm::SystemId from, std::uint64_t taskId);
    bool rejectTask(comm::SystemId from, std::uint64_t taskId, std::uint32_t code);
    /// An accepted task's progress (`state` Accepted), or its end (Completed, Failed).
    bool reportTask(comm::SystemId from, std::uint64_t taskId, comm::ReceivedTask::State state, double percent);
    std::vector<comm::ActiveOverride> overrides() const;
    // As a C2 system: a task, an override (`cancel`: the kind's cleared), consent requests and their answers.
    std::uint64_t sendTask(comm::SystemId ma, const comm::TaskOrder& task, comm::DmsReason* why = nullptr);
    std::uint64_t setOverride(comm::SystemId ma, const comm::OverrideCommand& command, comm::DmsReason* why = nullptr);
    std::vector<comm::PendingConsent> consentRequests() const;
    bool answerConsent(comm::SystemId ma, std::uint64_t requestId, bool granted, comm::DmsReason* why = nullptr);
    // AB-5c, as a C2 system: the lost-communication contingency (`planId` 0: the alert alone); a team, sent to its leader
    // and each member (its members then accept what its leader sends as C2's), or its dissolution.
    std::uint64_t setLostComms(comm::SystemId ma, double thresholdS, std::uint64_t planId = 0, comm::DmsReason* why = nullptr);
    std::uint64_t formPackage(std::uint32_t packageId, comm::SystemId leader, const std::vector<comm::SystemId>& members,
                              comm::DmsReason* why = nullptr);
    std::uint64_t dissolvePackage(std::uint32_t packageId, const std::vector<comm::SystemId>& systems, comm::DmsReason* why = nullptr);

    // Mission data (AB-2c): its own - what it writes, and what the range loaded for it through it before handing it over -
    // and the range's common mission data, which it reads and names but cannot change. Its own shadows a common one of
    // the same id; no other MA's is there for it. A frame following another aircraft is not (that is its true pose) but
    // its own designations may be followed.
    control::Reason setOpPoint(const control::OpPoint& point);
    bool removeOpPoint(control::OpPointId id);
    control::Reason setOpZone(const control::OpZone& zone);
    bool removeOpZone(control::OpZoneId id);
    control::Reason setOpLine(const control::OpLine& line);
    bool removeOpLine(control::OpLineId id);
    control::Reason setOpVolume(const control::OpVolume& volume);
    bool removeOpVolume(control::OpVolumeId id);
    /// A frame of its own: fixed, moving, or following itself or one of its designations; 0 refused.
    control::FrameId createFrame(const control::FrameSpec& spec);
    bool removeFrame(control::FrameId id);
    std::vector<control::FrameId> frames() const;
    std::optional<control::FrameSpec> frame(control::FrameId id) const;
    std::vector<control::OpPointId> opPoints() const;
    std::optional<control::OpPoint> opPoint(control::OpPointId id) const;
    std::vector<control::OpZoneId> opZones() const;
    std::optional<control::OpZone> opZone(control::OpZoneId id) const;
    std::vector<control::OpLineId> opLines() const;
    std::optional<control::OpLine> opLine(control::OpLineId id) const;
    std::vector<control::OpVolumeId> opVolumes() const;
    std::optional<control::OpVolume> opVolume(control::OpVolumeId id) const;
    std::optional<control::GeoPoint> framePoint(control::FrameId id, const control::FrameOffset& offset, double timeS) const;

    // Legal reports with their provenance (fsim/MAReports.h; docs/sdk/mission-training.md, section 10). Nothing more
    // than the reads above give - the same values, by the same paths - each with where it came from, when, and how
    // old it is: an unknown stays unknown, and nothing is filled in from truth.
    /// What a payload's passive or mapping sensor holds, `truth` 0: the entity behind a contact is the referee's.
    std::vector<ms::Contact> contacts(std::uint16_t payload) const;
    /// Its own ship and its endurance (ownship(), navigationReport()): Invalid while a navigation or air data sensor
    /// of its own says so; NotApplicable for a C2 node.
    ma::Report<ma::Navigation> reportNavigation(const ma::ReportOptions& options = {}) const;
    /// Its air radars' tracks (amtiTracks() of each, in payload order): sampled at each track's last update.
    ma::Collection<ms::AmtiTrack> reportTracks(const ma::ReportOptions& options = {}) const;
    /// Its payloads' contacts (contacts() of each, in payload order): sampled when each was last seen.
    ma::Collection<ms::Contact> reportContacts(const ma::ReportOptions& options = {}) const;
    /// The tracks its links delivered (linkTracks()): each from its source system, sampled at the source's report time
    /// and received when it was delivered - present only after delivery, gone when its picture drops it.
    ma::Collection<comm::LinkTrack> reportLinkTracks(const ma::ReportOptions& options = {}) const;
    /// The peers it heard (peers()), each with the provenance of the position report it holds: the net that report
    /// came by and when it was delivered, sampled at its own time - whatever a later heartbeat came by (which is what
    /// comm::PeerReport's own `net` and `receivedS`, the last of either message, may say). A peer heard only by its
    /// heartbeat holds no position: that heartbeat's delivery, and no sample time.
    ma::Collection<comm::PeerReport> reportPeers(const ma::ReportOptions& options = {}) const;
    /// The same peers, index for index, each with the provenance of its last heartbeat: the net it came by, when it
    /// was delivered, sampled at the time it states. Validity Unknown, its times unknown, for a peer no heartbeat was
    /// heard from.
    ma::Collection<comm::PeerReport> reportPeerHeartbeats(const ma::ReportOptions& options = {}) const;
    /// C2's tasks (c2Tasks()): each from the system that sent it, received when it was delivered.
    ma::Collection<comm::ReceivedTask> reportTasks(const ma::ReportOptions& options = {}) const;
    /// The authority over its platform (authority()), as its MA core keeps it now.
    ma::Report<C2Authority> reportAuthority(const ma::ReportOptions& options = {}) const;

private:
    friend class World;
    /// (the range's binding by Agent reads which World made it: fsim/MAService.h)
    friend FSIM_API bool bindMAService(World& world, const Agent& agent, std::uint64_t serviceId,
                                       const std::vector<std::string>& commandCapabilities, bool ownsDmsIngress, bool ownsDmsEgress,
                                       std::string* error);
    Agent(World* world, std::uint32_t vehicle, control::ControllerId controller, std::string name)
        : world_(world), vehicle_(vehicle), controller_(controller), name_(std::move(name)) {}
    bool own(control::ActivityId activity) const noexcept;
    World* world_ = nullptr;
    std::uint32_t vehicle_ = 0;
    control::ControllerId controller_ = 0;
    std::string name_;
};

/// Everything, read only, truth among it: for scoring, rewards, a centralised critic (ADR-31's referee).
class FSIM_API Referee {
public:
    Referee() = default;
    double time() const noexcept;
    std::vector<std::uint32_t> vehicles() const;
    const VehicleState* state(std::uint32_t vehicle) const noexcept;  ///< truth; null for an unknown vehicle
    std::vector<ms::WeaponRecord> weapons() const;
    std::optional<ms::RadioPath> radioPath(std::uint32_t from, std::uint32_t to, const ms::RadioPathOptions& options = {}) const;
    std::optional<ms::RadarCrossSection> radarCrossSection(std::uint32_t observer, std::uint32_t target, const ms::RcsQuery& query = {}) const;
    /// The entity behind a radar track or detection now (0: a false alarm, or one it no longer has).
    std::uint32_t trackTruth(std::uint32_t vehicle, std::uint16_t payload, std::uint64_t track) const;
    std::uint32_t detectionTruth(std::uint32_t vehicle, std::uint16_t payload, std::uint64_t serial) const;
    /// The entity behind a designation's track now (0: lapsed, a false alarm, or no designation).
    std::uint32_t designationTruth(std::uint32_t target) const;
    // Systems and nets (AB-4)
    std::uint32_t systemEntity(comm::SystemId system) const noexcept; ///< the entity behind a SystemID (0: none)
    comm::SystemId entitySystem(std::uint32_t entity) const noexcept;
    std::string tailNumber(comm::SystemId system) const;
    /// The entity behind a received track: its source system's radar track's now (0: none, a false alarm).
    std::uint32_t linkTrackTruth(comm::SystemId source, std::uint16_t payload, std::uint64_t track) const;
    std::vector<comm::NetInfo> nets() const;
    comm::NetStats netStats(comm::NetId net) const noexcept;
    /// The C2 nodes the range made, and where one is.
    std::vector<std::uint32_t> c2Nodes() const;
    std::optional<comm::C2NodeSpec> c2Node(std::uint32_t node) const;
    /// Every agent the range made: its vehicle, controller and name.
    struct AgentInfo {
        std::uint32_t vehicle = 0;
        control::ControllerId controller = 0;
        std::string name;
    };
    std::vector<AgentInfo> agents() const;
    /// The audit (AB-2d): every call each agent made, by name - the C function's, or fsim::Agent's member's - and those
    /// refused it; the agents in the order the range made them.
    struct AuditCall {
        std::string call;
        std::uint64_t calls = 0, refused = 0;
    };
    struct AgentAudit {
        AgentInfo agent;
        std::vector<AuditCall> calls;
    };
    std::vector<AgentAudit> audit() const;

private:
    friend class World;
    explicit Referee(const World* world) : world_(world) {}
    const World* world_ = nullptr;
};

} // namespace fsim
