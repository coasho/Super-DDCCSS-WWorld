#pragma once

// The simulation session behind fsim::World (design 9.2): vehicles by name
// and type, lockstep stepping through the vehicle pool, control cascades and
// effects on the worker that owns each vehicle, the environment applied per
// step, the message fabric, and the shared-memory publisher for viewers.

#include "comm/Comm.h"
#include "control/Adapter.h"
#include "control/CapabilityHost.h"
#include "control/Catalog.h"
#include "control/ControlStack.h"
#include "ms/Host.h"
#include "ms/damage/State.h"
#include "ms/rcs/Table.h"
#include "control/Features.h"
#include "session/Dms.h"
#include "session/MaCore.h"
#include "fsim/VehicleProfile.h"
#include "effects/Effect.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Frames.h"
#include "fsim/InitialConditions.h"
#include "fsim/Rng.h"
#include "io/TerrainTiles.h"
#include "ipc/Recording.h"
#include "ipc/WorldPublisher.h"
#include "sim/GroundProvider.h"
#include "sim/VehiclePool.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace fsim::control {
struct PerformanceProfile; // fsim/PerformanceProfile.h
}

namespace fsim::session {

struct WorldOptions {
    std::string name = "default";       ///< shared-memory world name (viewers attach by it)
    double dt = 1.0 / 120.0;            ///< FDM step
    int frameSkip = 4;                  ///< FDM steps per world step
    unsigned workers = 0;               ///< 0 = physical cores - 2
    bool pinWorkers = true;             ///< one worker per physical core (throughput stability)
    std::uint64_t seed = 0;
    std::uint32_t capacity = 256;       ///< published vehicle slots
    bool publish = true;                ///< false = invisible to viewers
    bool storeDrag = true;              ///< carried stores' and their carriage's drag on the aircraft (MS-7g); false: their mass only
    double publishIntervalSeconds = 1.0 / 60.0;
    std::filesystem::path jsbsimRoot;   ///< empty = auto-detect
    std::shared_ptr<sim::GroundProvider> ground; ///< custom ground; null = `terrain` below or flat at 0 m
    bool terrain = false;               ///< physics ground from the public elevation tiles the viewer draws
    std::string terrainUrl;             ///< tile template; empty = AWS Terrarium
    unsigned terrainZoom = 15;          ///< ~3.8 m/px: the raster the viewer draws close up (io::TerrainTiles::Options::zoom)
    double terrainPrefetchRadiusM = 1500.0; ///< tiles loaded (blocking) around each new vehicle's position (then ahead of it, in the background)
    bool terrainOffline = false;        ///< the tiles from the cache alone, never fetched (io::TerrainTiles::Options::offline)
    std::filesystem::path recordPath;   ///< non-empty: write every vehicle's state to this file for replay in the viewer
    double recordIntervalSeconds = 0.0; ///< simulation time between recorded frames; 0 = every world step
};

struct VehicleSpec {
    std::string name;                   ///< unique in the world; empty = "<type>-<id>"
    std::string type = "jsbsim:c172x";  ///< "<flight model>:<aircraft>"; a bare aircraft name means JSBSim
    sim::InitialConditions initial;
    std::string model;                  ///< optional visual override for the viewer (glTF path)
    unsigned controlDivider = 1;        ///< run the control stack every N FDM steps
    /// Where its definition is (<dir>/<aircraft>/<aircraft>.xml); empty: found by its name (a piece's: its aircraft's
    /// pieces/, MS-7g)
    std::filesystem::path aircraftDir;
    /// Sections of the aircraft's profile to replace for this vehicle (those
    /// present(); docs/control-architecture.md, 7.4); null = the aircraft's own.
    std::shared_ptr<const control::VehicleProfile> profile;
};

struct VehicleInfo {
    std::uint32_t id = 0;
    std::string name, type, aircraft, model;
    sim::InitialConditions initial;
    bool alive = false;
    std::uint64_t generation = 0;
    ms::Role role = ms::Role::Aircraft; ///< a weapon in flight (ADR-30, MS-2): released by `launcher`
    std::uint32_t launcher = 0;
    double launchTimeS = 0.0;
};

/// A warhead going off (MS-7g's damage part): its store card's, where and how it moved (ECEF).
struct Detonation {
    std::string store;                    ///< its store card's id ("aim-120c")
    double positionEcef[3] = {0, 0, 0};   ///< the warhead's centre (m)
    double velocityEcef[3] = {0, 0, 0};   ///< the missile's (m/s)
    double axisEcef[3] = {1, 0, 0};       ///< its nose
    std::uint32_t weapon = 0;             ///< the weapon's vehicle (0: none in the world - a scripted one)
};

class World final : public control::WorldView {
public:
    explicit World(const WorldOptions& options);
    ~World() override;
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // --- Vehicles ------------------------------------------------------------
    /// Create (load) a vehicle. Returns its id, or 0 on failure (logged).
    std::uint32_t createVehicle(const VehicleSpec& spec);
    bool removeVehicle(std::uint32_t id);
    /// Re-apply initial conditions (`ic` null = the spec's) and reset controls/effects. With `seed`, the vehicle's
    /// random stream (its effects', JSBSim's turbulence and sensor noise) restarts from it, as a new vehicle's does from
    /// the world's seed and its id - the world's seed gives a new vehicle's, to the bit; without, it goes on (design 7.2).
    bool resetVehicle(std::uint32_t id, const sim::InitialConditions* ic = nullptr, std::optional<std::uint64_t> seed = std::nullopt);
    bool alive(std::uint32_t id) const noexcept;
    const VehicleInfo* info(std::uint32_t id) const noexcept;
    std::uint32_t find(const std::string& name) const noexcept;
    std::vector<std::uint32_t> vehicleIds() const;
    std::size_t vehicleCount() const noexcept { return liveCount_; }

    // --- State ---------------------------------------------------------------
    const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override;
    const effects::SensedState* sensedState(std::uint32_t id) const noexcept;
    sim::FlightModel* model(std::uint32_t id) noexcept;
    const sim::ControlInputs* inputs(std::uint32_t id) const noexcept;

    // --- Control (design 9.3, docs/control-architecture.md) ---------------------
    /// What every existing entry point calls: an update of the vehicle's own
    /// activity at the same level, else a new one (10.7). False if the vehicle
    /// is unknown or the command rejected (a behaviour nobody registered, an
    /// axis an autopilot or override activity holds).
    bool command(std::uint32_t id, const control::Command& command);
    /// The per-step path as a controller's (an agent's: VecEnv's actions, ADR-31 AB-3) - a behaviour or a must fly naming
    /// another aircraft by its entity id refused to an agent's, as its NEW would be.
    bool command(std::uint32_t id, const control::Command& command, control::ControllerId controller);
    /// command() with its answer (the C ABI reports why a command was refused).
    control::CommandResult commandResult(std::uint32_t id, const control::Command& command);
    /// NEW: a command becomes an activity, or is rejected with a reason.
    control::CommandResult submit(std::uint32_t id, const control::Command& command, const control::CommandOptions& options = {});
    /// The same, its agent's checks made (overrides applied).
    control::CommandResult submitChecked(std::uint32_t id, const control::Command& command, const control::CommandOptions& options);
    /// NEW for a support effector the vehicle has (gear, flaps, brakes, speedbrake, pitch trim) or its engines' throttles.
    control::CommandResult submit(std::uint32_t id, const control::SupportCommand& command, const control::CommandOptions& options = {});
    /// NEW of a route (fsim.guidance.route) with its waypoints (docs/vehicle-interface.md, 4.5), its loiter points'
    /// loiters (docs/flight-autonomy.md, 4.31), its planned states (4.34), its paths (4.36), its branches (4.37) and its
    /// civil path terminators' data (4.38).
    control::CommandResult submit(std::uint32_t id, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  const control::CommandOptions& options = {}, Span<const control::RouteLoiter> loiters = {},
                                  Span<const control::RouteState> states = {}, Span<const control::RoutePath> paths = {},
                                  Span<const control::RouteBranch> branches = {}, Span<const control::RouteTerminator> terminators = {});
    /// NEW of a curve (fsim.guidance.curve) with its segments (docs/vehicle-interface.md, 4.7): Bezier segments, or as
    /// A-GRA's schema gives them (docs/flight-autonomy.md, 4.26); its reference in a frame beside it, `shape` (4.27).
    control::CommandResult submit(std::uint32_t id, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    control::CommandResult submit(std::uint32_t id, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CommandOptions& options = {}, const control::CurveShape* shape = nullptr);
    /// NEW of a pattern (fsim.guidance.pattern) with its shape (docs/flight-autonomy.md, 4.23).
    /// NEW of a must fly with its zone given (docs/flight-autonomy.md, 4.43): the zone checked and laid out as it is given.
    control::CommandResult submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpZone& zone,
                                  const control::CommandOptions& options = {});
    /// NEW of a must fly with its corridor given (4.44): the line checked and laid out as it is given.
    control::CommandResult submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpLine& line,
                                  const control::CommandOptions& options = {});
    /// NEW of a must fly with its volume given (4.45): the volume checked and laid out as it is given.
    control::CommandResult submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpVolume& volume,
                                  const control::CommandOptions& options = {});
    control::CommandResult submit(std::uint32_t id, const control::PatternCommand& pattern, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    /// NEW of a marshall with its pattern's shape (docs/flight-autonomy.md, 4.46; Marshalls.cpp): its slot chosen first - the
    /// altitude asked for, else the lowest of its stack clear of every other aircraft's marshall round the same point - then
    /// submitted as any NEW is; StackFull (index 3) where none is. Flown as its pattern, its stack beside it.
    control::CommandResult submit(std::uint32_t id, const control::MarshallCommand& marshall, const control::PatternShape& shape,
                                  const control::CommandOptions& options = {});
    /// NEW of a route intercept (docs/flight-autonomy.md, 4.47; Intercepts.cpp): the vehicle's kept plan joined where its method
    /// chooses from where the aircraft is, and flown from there - the plan activated by it.
    control::CommandResult submit(std::uint32_t id, const control::InterceptCommand& intercept, const control::CommandOptions& options = {});
    /// Several NEWs at once (docs/flight-autonomy.md, 4.8), made in order at
    /// this simulation time, each answered on its own; `details`, if given,
    /// gets each answer's details (commandDetails()) in the same order.
    std::vector<control::CommandResult> submitBatch(std::uint32_t id, Span<const control::BatchCommand> batch,
                                                    std::vector<control::CommandDetails>* details = nullptr);
    /// Everything the checks found for the vehicle's last NEW, validation or
    /// UPDATE (docs/flight-autonomy.md, 4.8): every finding and every value
    /// flown other than asked. Null for an unknown vehicle.
    const control::CommandDetails* commandDetails(std::uint32_t id) const noexcept;
    /// UPDATE: a new setpoint for a live activity (the fast path).
    control::CommandResult update(control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::ActivityId activity, const control::SupportCommand& setpoint);
    /// UPDATE of a route: new options (kHold keeps one) and waypoints with their loiters, states, paths, branches and
    /// terminators (none: those it has); flown afresh from its start.
    control::CommandResult update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                  Span<const control::RouteLoiter> loiters = {}, Span<const control::RouteState> states = {},
                                  Span<const control::RoutePath> paths = {}, Span<const control::RouteBranch> branches = {},
                                  Span<const control::RouteTerminator> terminators = {});
    /// UPDATE of a curve: options, and segments appended (append 1) or a new curve.
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                  const control::CurveShape* shape = nullptr);
    /// UPDATE of a pattern with its shape: the fields given in either merged (kHold keeps one), flown afresh.
    control::CommandResult update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape);
    /// UPDATE of a marshall, with its pattern's shape or without: its slot chosen afresh where its stack moves (4.46). Its
    /// pattern takes an UPDATE through it alone (a PatternCommand's: wrong_command_type).
    control::CommandResult update(control::ActivityId activity, const control::MarshallCommand& marshall, const control::PatternShape& shape);
    control::CommandResult update(control::ActivityId activity, const control::MarshallCommand& marshall);
    /// A route intercept takes no UPDATE (4.47): not_updatable, a new one replacing it.
    control::CommandResult update(control::ActivityId activity, const control::InterceptCommand& intercept);
    /// UPDATE of a must fly with a zone given in place of its own (4.43), or a corridor (4.44).
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpLine& line);
    control::CommandResult update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpVolume& volume);
    /// CANCEL: the activity ends; its axes fly the vehicle default.
    control::CommandResult cancel(control::ActivityId activity);
    /// UPDATE and CANCEL declaring the caller's source, as a NEW's options do,
    /// and a policy's controller (docs/vehicle-interface.md, 6.1;
    /// docs/flight-autonomy.md, 4.12): under ControlMode::Granted a source
    /// below the activity's may not address it (AuthorityHeld, naming it), nor
    /// a controller another's. The calls above are the default policy's.
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::Command& setpoint);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::SupportCommand& setpoint);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::RouteCommand& route,
                                  Span<const control::Waypoint> waypoints, Span<const control::RouteLoiter> loiters = {},
                                  Span<const control::RouteState> states = {}, Span<const control::RoutePath> paths = {},
                                  Span<const control::RouteBranch> branches = {}, Span<const control::RouteTerminator> terminators = {});
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                  Span<const control::BezierSegment> segments, const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                  Span<const control::NurbsSegment> segments, const control::CurveShape* shape = nullptr);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpZone& zone);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpLine& line);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                  const control::OpVolume& volume);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::PatternCommand& pattern,
                                  const control::PatternShape& shape);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall,
                                  const control::PatternShape& shape);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall);
    control::CommandResult update(control::Caller caller, control::ActivityId activity, const control::InterceptCommand& intercept);
    control::CommandResult cancel(control::Caller caller, control::ActivityId activity);
    /// An activity command (docs/flight-autonomy.md, 4.10) - disable, enable,
    /// reset, delete, change its rank, unassign it - declaring the caller's
    /// source as UPDATE does; `rank` for ChangeRank.
    control::CommandResult activityCommand(control::Caller caller, control::ActivityId activity, control::ActivityCommand command,
                                           control::Rank rank = {});
    /// The operator's input to a route's conditional branch (docs/flight-autonomy.md, 4.37): branch `branch` commanded, or
    /// no longer - declaring the caller's source as UPDATE does.
    control::CommandResult commandBranch(control::Caller caller, control::ActivityId activity, std::uint32_t branch, bool commanded = true);
    control::CommandResult commandBranch(control::ActivityId activity, std::uint32_t branch, bool commanded = true) {
        return commandBranch(control::Source::Policy, activity, branch, commanded);
    }
    // --- Flight tasks (docs/flight-autonomy.md, 4.11): kept by id, flown on a task command ---
    control::Reason storeTask(std::uint32_t id, control::TaskId task, const control::Command& command, Span<const control::Waypoint> waypoints = {},
                              Span<const control::BezierSegment> segments = {}, control::TaskRepetition repetition = {},
                              const control::PatternShape* shape = nullptr);
    /// A task whose command is a batch item's - a route's waypoints, a curve's segments either way, a pattern's shape
    /// beside it (its options not kept: a task command gives them). InvalidParameter for a support command's.
    control::Reason storeTask(std::uint32_t id, control::TaskId task, const control::BatchCommand& command, control::TaskRepetition repetition = {});
    control::CommandResult commandTask(std::uint32_t id, control::TaskId task, const control::CommandOptions& options = {});
    control::CommandResult cancelTask(std::uint32_t id, control::TaskId task, control::Caller caller = {});
    control::Reason removeTask(std::uint32_t id, control::TaskId task);
    /// A task's status; empty for one not kept (or an unknown vehicle).
    std::optional<control::TaskStatus> taskStatus(std::uint32_t id, control::TaskId task);
    std::vector<control::TaskStatus> tasks(std::uint32_t id);
    // --- Route plans (docs/flight-autonomy.md, 4.39): kept by id, taken through the plan activation states ---
    /// A plan published (A-GRA's MA_RoutePlanMT): taken where FA listens for its id (None: A-GRA's notification
    /// CONFIRMED); WrongPlanState where it does not; InvalidParameter for id 0 or malformed metadata.
    control::Reason publishPlan(std::uint32_t id, const control::RoutePlan& plan);
    /// A plan activation command (A-GRA's), answered at once: completed or failed, and the plan's state after it.
    control::PlanCommandResult planCommand(std::uint32_t id, control::PlanId plan, control::PlanCommand command,
                                           const control::CommandOptions& options = {});
    /// FA's own deactivation (VI 1.2.5.7), the platform's: a plan ready for activation or activated is Deactivated, its
    /// live activity canceled with `reason`, its execution Canceled.
    control::PlanCommandResult abortPlan(std::uint32_t id, control::PlanId plan, control::Reason reason = control::Reason::Restricted);
    /// Forget a plan: UnknownPlan; WrongPlanState while its activity is live.
    control::Reason removePlan(std::uint32_t id, control::PlanId plan);
    /// A plan's status; empty for one not kept (or an unknown vehicle).
    std::optional<control::PlanStatus> planStatus(std::uint32_t id, control::PlanId plan) const;
    /// Every plan kept, in the order they were first prepared for upload (A-GRA's query for identifiers only).
    std::vector<control::PlanStatus> plans(std::uint32_t id) const;
    /// A plan's content as uploaded last, its metadata with it; empty for one not kept, or not yet uploaded.
    std::optional<control::RoutePlan> plan(std::uint32_t id, control::PlanId plan) const;
    // FA's own plans and the airfields (docs/flight-autonomy.md, 4.40): the platform's, read only to MA
    /// FA's own plan: kept Uploaded, in place of any plan by its id not flying. InvalidParameter, UnknownAirfield (a takeoff's
    /// or a landing's path naming an airfield or runway the vehicle does not keep), WrongPlanState, PlanStoreFull.
    control::Reason loadPlan(std::uint32_t id, const control::RoutePlan& plan);
    /// An airfield: kept in place of any by its id, its revision one more. InvalidParameter, PlanStoreFull.
    control::Reason loadAirfield(std::uint32_t id, const control::Airfield& airfield);
    /// Every airfield kept, as loaded (A-GRA's query for the airfields).
    std::vector<control::Airfield> airfields(std::uint32_t id) const;
    std::optional<control::Airfield> airfield(std::uint32_t id, control::AirfieldId airfield) const;
    /// A route plan validated without flying it (docs/flight-autonomy.md, 4.41): its route checked as its NEW would be, in
    /// the weather given, from its origin, its verdict over its parts (a patch's). Kept or not; nothing is kept or flown.
    /// commandDetails(id) has its findings and adjustments.
    control::PlanValidationResult validatePlan(std::uint32_t id, const control::RoutePlan& plan, const control::PlanValidation& v = {});
    /// A kept plan's, as uploaded last: UnknownPlan; WrongPlanState before its first upload.
    control::PlanValidationResult validatePlan(std::uint32_t id, control::PlanId plan, const control::PlanValidation& v = {});
    // --- Reports (docs/flight-autonomy.md, 4.12): what an activity flies, and where to ---
    /// What a live activity flies now, or waits to fly: its setpoint as
    /// updated, a route's waypoints, a curve's segments (appended ones too: its
    /// flyout curve). False for an activity not live.
    bool activitySetpoint(control::ActivityId activity, control::Setpoint& out) const;
    /// A live route's next point with a required time of arrival, as it estimates it now (docs/flight-autonomy.md, 4.33):
    /// when it will arrive, and that against its window; false where it has none.
    bool activityArrival(control::ActivityId activity, control::ArrivalEstimate& out) const;
    /// A route intercept's status (4.47; A-GRA's MA_RoutePlanInterceptStatusType): false for any other activity.
    bool interceptStatus(control::ActivityId activity, control::InterceptStatus& out) const;
    /// Where a live activity flies to (A-GRA's ActualEndPoint): the point it
    /// flies to now, then those after it, `max` at most.
    std::vector<control::EndPoint> endPoints(control::ActivityId activity, std::size_t max = 16) const;
    /// What the vehicle is commanded (A-GRA's VehicleCommandState): the
    /// cascade's levels, the acceleration they command in north, east and
    /// down, the altitude as its mode commanded it. NaN fields for an unknown vehicle.
    control::VehicleCommandState commandState(std::uint32_t id) const;
    // --- The navigation report (docs/flight-autonomy.md, 4.14; A-GRA's MA_NavigationReport) ---
    /// What the vehicle flies on, how much is left and for how long, its
    /// playtime to its recovery point and its contingency (Navigation.cpp).
    /// Energy::Unknown for an unknown vehicle, or one its flight model says
    /// nothing of.
    control::NavigationReport navigationReport(std::uint32_t id) const;
    /// Its recovery point and reserve: Reason::OutOfRange for a point off the
    /// Earth or a reserve outside [0, 1).
    control::Reason setNavigation(std::uint32_t id, const control::NavigationSettings& settings);
    control::NavigationSettings navigation(std::uint32_t id) const noexcept;
    // --- The terrain (docs/flight-autonomy.md, 4.19; VI 1.2.6.9, A-GRA's elevation request) ---
    /// The ground's height above the WGS-84 ellipsoid at a place, as the
    /// physics has it; empty where it has no data (a terrain tile it cannot load).
    std::optional<double> terrainHeightM(double latitudeRad, double longitudeRad) const {
        return ground_->knownHeightAboveEllipsoidM(latitudeRad, longitudeRad);
    }
    // --- The barometric altimeter (docs/flight-autonomy.md, 4.20; VI 1.2.6.5 and 1.2.6.8) ---
    /// The QNH a vehicle's altimeter is set to (A-GRA's QNH setting), Pa - what
    /// its barometric altitudes are read and flown at: applied (Reason::None),
    /// or OutOfRange outside 850 to 1,100 hPa (nothing changes); UnknownVehicle.
    /// Until set, the standard 1013.25 hPa: the altimeter reads pressure altitude.
    control::Reason setQnh(std::uint32_t id, double qnhPa);
    double qnh(std::uint32_t id) const noexcept; ///< NaN for an unknown vehicle
    /// What its altimeter reads now, and the air it reads it in (StateData;
    /// A-GRA's MA_AirDataType). NaN fields for an unknown vehicle.
    control::StateData stateData(std::uint32_t id) const;
    /// The world's air now (its environment's sea-level temperature and pressure).
    control::Air air() const noexcept { return control::Air{environment_.temperatureSeaLevelK, environment_.pressureSeaLevelPa}; }
    // --- Mission data's owners (ADR-31, AB-2c) -------------------------------------
    // Reference frames and operational geometry are each an owner's: 0, the range's common mission data - every MA reads
    // and names it, the range alone writes it - or a vehicle's, its MA's own, which no other vehicle sees. A vehicle
    // (`viewer`) sees its own, then the common: its own shadows a common one of the same id. The range (viewer 0) sees
    // the common. Each vehicle's commands name what it sees (its host's SessionView is its own).
    // --- Reference frames (docs/flight-autonomy.md, 4.21; A-GRA's ReferenceFrame) ---
    /// A frame by id (A-GRA's ReferenceFrameID), the owner's: fixed, moving at a constant velocity from a time, or
    /// following a vehicle or a designation (AB-2b). 0: refused - a value not finite, a latitude off the Earth, an origin
    /// not one, an unknown vehicle; for a vehicle's own, one following another vehicle than itself or its designations.
    control::FrameId createFrame(const control::FrameSpec& spec, std::uint32_t owner = 0);
    bool removeFrame(control::FrameId id, std::uint32_t owner = 0);
    std::optional<control::FrameSpec> frame(control::FrameId id, std::uint32_t viewer = 0) const;
    /// A frame as it is at a time (NaN: now; simulation seconds, simTime()): a
    /// vehicle's from its state now, carried on at its velocity to another
    /// time. Empty for an unknown frame, or one whose vehicle is gone.
    std::optional<control::FramePose> framePose(control::FrameId id, double timeS = std::numeric_limits<double>::quiet_NaN(),
                                                std::uint32_t viewer = 0) const;
    /// Where a point in a frame is at a time (NaN: now).
    std::optional<control::GeoPoint> framePoint(control::FrameId id, const control::FrameOffset& offset,
                                                double timeS = std::numeric_limits<double>::quiet_NaN(), std::uint32_t viewer = 0) const;
    /// The frames a viewer sees, by id.
    std::vector<control::FrameId> frames(std::uint32_t viewer = 0) const;
    // --- Operational geometry (docs/flight-autonomy.md, 4.42; A-GRA's OpPoint) ---
    /// An operational point, kept by its id in place of any by it, its revision one more (Geometry.cpp). InvalidParameter:
    /// id 0; neither a place nor a frame, or both; a latitude off the Earth, a value not finite, a code not one; a frame
    /// the world does not have, or offsets without one; an altitude reference without its altitude; a window of bearings
    /// given one way alone, or beyond half a turn.
    control::Reason setOpPoint(const control::OpPoint& point, std::uint32_t owner = 0);
    bool removeOpPoint(control::OpPointId id, std::uint32_t owner = 0);
    /// The operational points a viewer sees, by id.
    std::vector<control::OpPointId> opPoints(std::uint32_t viewer = 0) const;
    std::optional<control::OpPoint> opPoint(control::OpPointId id, std::uint32_t viewer = 0) const;
    /// An operational zone (4.43; A-GRA's OpZone), kept by its id in place of any by it, its revision one more; a moving one's
    /// time left out, now (Geometry.cpp). InvalidParameter for one A-GRA's schema would not take, or a frame the world does not
    /// have.
    control::Reason setOpZone(const control::OpZone& zone, std::uint32_t owner = 0);
    bool removeOpZone(control::OpZoneId id, std::uint32_t owner = 0);
    std::vector<control::OpZoneId> opZones(std::uint32_t viewer = 0) const;
    std::optional<control::OpZone> opZone(control::OpZoneId id, std::uint32_t viewer = 0) const;
    /// An operational line (4.44; A-GRA's OpLine), kept as a zone is (Geometry.cpp). InvalidParameter for one A-GRA's schema
    /// would not take, or a frame the world does not have.
    control::Reason setOpLine(const control::OpLine& line, std::uint32_t owner = 0);
    bool removeOpLine(control::OpLineId id, std::uint32_t owner = 0);
    std::vector<control::OpLineId> opLines(std::uint32_t viewer = 0) const;
    std::optional<control::OpLine> opLine(control::OpLineId id, std::uint32_t viewer = 0) const;
    /// An operational volume (4.45; A-GRA's OpVolume), kept as a zone is (Geometry.cpp). InvalidParameter for one A-GRA's schema
    /// would not take, or a frame the world does not have.
    control::Reason setOpVolume(const control::OpVolume& volume, std::uint32_t owner = 0);
    bool removeOpVolume(control::OpVolumeId id, std::uint32_t owner = 0);
    std::vector<control::OpVolumeId> opVolumes(std::uint32_t viewer = 0) const;
    std::optional<control::OpVolume> opVolume(control::OpVolumeId id, std::uint32_t viewer = 0) const;
    // --- The performance profile (docs/flight-autonomy.md, 4.15; A-GRA's MA_FlightControlModesPerformanceProfileType) ---
    /// A flight mode's performance profile at the vehicle's condition now - HSA/CSA, waypoint or curve following -
    /// into `out`, its vectors reused (PerformanceProfile.cpp). InvalidParameter for another mode (A-GRA profiles
    /// those three), the support table's refusal for one the vehicle does not offer, UnknownVehicle.
    control::Reason performanceProfile(std::uint32_t id, control::FlightMode mode, control::PerformanceProfile& out);
    /// What flies a vehicle's primary axes nobody owns: the neutral actuator
    /// command (as always) or a hold of the heading, airspeed and height each
    /// had when it was let go. Reason::None if set.
    control::Reason setVehicleDefault(std::uint32_t id, control::VehicleDefault mode);
    control::VehicleDefault vehicleDefault(std::uint32_t id) const noexcept;
    /// Envelope protection (docs/control-architecture.md, 11): Limit by default
    /// for an aircraft with an envelope, else Off. Reason::None if set.
    control::Reason setProtection(std::uint32_t id, control::ProtectionMode mode);
    control::ProtectionMode protection(std::uint32_t id) const noexcept;
    /// What protection saw since the last call (each call starts a new count).
    control::EnvelopeStatus envelope(std::uint32_t id);
    /// A live or recently ended activity; null if unknown.
    const control::ActivityRecord* activity(control::ActivityId activity) const noexcept;
    /// A vehicle's live activities, then the ended ones it remembers, newest first.
    std::vector<control::ActivityRecord> activities(std::uint32_t id) const;
    /// What a vehicle offers (empty for an unknown vehicle).
    const std::vector<control::CapabilityDescriptor>& capabilities(std::uint32_t id);
    /// A capability's availability as a policy is answered (docs/flight-autonomy.md,
    /// 4.1): for one the vehicle does not offer, Unavailable with NotSupported,
    /// NotImplemented or UnknownCapability.
    control::CapabilityStatus capabilityStatus(std::uint32_t id, std::string_view capability) const;
    /// Whether the vehicle's aircraft can do a public feature at all
    /// (docs/flight-autonomy.md, 4.2), by its identifier or a behaviour's id;
    /// null for an unknown vehicle or feature.
    const control::SupportInfo* support(std::uint32_t id, std::string_view feature) const noexcept;
    /// Every public feature's support on the vehicle, in supportFeature()'s order; null for an unknown vehicle.
    const control::SupportTable* supportTable(std::uint32_t id) const noexcept;
    /// What the vehicle flies with: its aircraft's profile, with the spec's sections over it.
    const control::VehicleProfile* profile(std::uint32_t id) const noexcept;
    control::ControlStack* controls(std::uint32_t id) noexcept;
    const control::ControlStack* controls(std::uint32_t id) const noexcept;
    /// What the vehicle can do, as its guidance plans with it (docs/vehicle-interface.md, 7.1): computed
    /// afresh first if its loops changed. Null for an unknown vehicle.
    const control::Performance* performance(std::uint32_t id) noexcept;

    // --- Authority and availability (docs/vehicle-interface.md, 6 and 7.2) ---------
    /// Open (as ADR-26, the default) or Granted: a policy's NEW then needs a
    /// grant for its capability, and what the policy flies without one ends.
    control::Reason setControlMode(std::uint32_t id, control::ControlMode mode);
    control::ControlMode controlMode(std::uint32_t id) const noexcept;
    /// A policy's controller asks for control of a capability (by id):
    /// Reason::None if granted; else NotAllowed, the reason it is unavailable,
    /// AuthorityHeld (another controller holds it), UnknownCapability.
    control::Reason requestControl(std::uint32_t id, std::string_view capability, control::ControllerId controller = 0);
    /// A controller lets go: the grant ends, and its live activities of the
    /// capability end Canceled(Released). NotGranted for another's grant.
    control::Reason releaseControl(std::uint32_t id, std::string_view capability, control::ControllerId controller = 0);
    /// The platform takes it back: the grant ends, and the policy's live activities of it end Canceled(`reason`).
    control::Reason revokeControl(std::uint32_t id, std::string_view capability, control::Reason reason = control::Reason::Revoked);
    /// Whether the policy may request the capability; a grant for one no longer allowed is revoked.
    control::Reason setAllowed(std::uint32_t id, std::string_view capability, bool allowed);
    control::ControlStatus controlStatus(std::uint32_t id, std::string_view capability) const;
    /// A capability's precedence (docs/flight-autonomy.md, 4.9; lower first, 0 until set): the platform's
    /// setting, by which two activities of one source contest axes before their ranks. UnknownCapability, or why
    /// the vehicle does not offer it.
    control::Reason setCapabilityPrecedence(std::uint32_t id, std::string_view capability, std::uint32_t precedence);
    std::uint32_t capabilityPrecedence(std::uint32_t id, std::string_view capability) const;
    /// The platform restricts a capability (or, Available, lifts it): a policy's NEW for it is refused with
    /// `reason`. The status reports the id it is about and when it is expected back (NaN: not known).
    control::Reason setAvailability(std::uint32_t id, std::string_view capability, control::Availability availability,
                                    control::Reason reason = control::Reason::Restricted, std::uint64_t associated = 0,
                                    double nextAvailableS = control::kUnknown);
    /// Counts every change to grants, what is allowed, the control mode, availability and the performance (6.3).
    std::uint32_t controlRevision(std::uint32_t id) noexcept;

    // --- Mission Systems (ADR-30, docs/mission-systems.md) ----------------------
    // Their grants and availability go through the calls above: a capability id
    // under "fsim.ms." is a Mission Systems capability's instance id.
    /// A vehicle's Mission Systems: its payloads, their capabilities and activities; null for an unknown vehicle.
    const ms::Host* missionSystems(std::uint32_t id) const noexcept;
    ms::CapabilityStatus missionCapabilityStatus(std::uint32_t id, std::string_view capability) const;
    ms::CommandResult submit(std::uint32_t id, const ms::StateCommand& command, const ms::CommandOptions& options = {});
    ms::CommandResult submit(std::uint32_t id, const ms::BitCommand& command, const ms::CommandOptions& options = {});
    /// Search with a radar (MS-4): the AMTI capability of its payload.
    ms::CommandResult submit(std::uint32_t id, const ms::AmtiCommand& command, const ms::CommandOptions& options = {});
    /// A radar payload's picture (empty for an unknown vehicle or a payload that is not a radar).
    ms::RadarStatus radarStatus(std::uint32_t id, std::uint16_t payload) const noexcept;
    /// A radar payload's detections with serial >= `since` that it still keeps (its last 2048), oldest first.
    std::vector<ms::AmtiDetection> amtiDetections(std::uint32_t id, std::uint16_t payload, std::uint64_t since = 0) const;
    /// A radar payload's confirmed tracks (MS-5), predicted to now.
    std::vector<ms::AmtiTrack> amtiTracks(std::uint32_t id, std::uint16_t payload) const;
    ms::CommandResult cancelMission(std::uint32_t id, control::Caller caller, ms::ActivityId activity);
    /// The platform fails a payload (a contingency), or repairs it (back to off).
    ms::Reason failPayload(std::uint32_t id, std::uint16_t payload, bool failed);
    // stores (MS-7): the inventory, loads (a scenario's), strike, consent, jettison
    std::vector<ms::StoreInfo> stores(std::uint32_t id) const;
    ms::Reason loadStore(std::uint32_t id, std::uint16_t payload, int station, const std::string& store, int count = 1);
    ms::Reason loadLoadout(std::uint32_t id, const std::string& loadout);
    ms::CommandResult submit(std::uint32_t id, const ms::StrikeCommand& command, const ms::CommandOptions& options = {});
    ms::CommandResult submit(std::uint32_t id, const ms::JettisonCommand& command, const ms::CommandOptions& options = {});
    /// Release consent for a strike asking for it (the activity's vehicle is in its id).
    ms::Reason strikeConsent(ms::ActivityId activity, bool granted);
    std::optional<ms::StrikeStatus> strikeStatus(ms::ActivityId activity) const;
    /// The vehicle's whole mass as its flight model last worked it out (kg; its fuel and stores among it): a load shows
    /// after the next step. NaN if unknown.
    double massKg(std::uint32_t id) const;

    // --- Weapons in flight and the radio path (ADR-30, MS-2) ------------------------
    /// Release a store from an aircraft: a new entity at its station, with its velocity, attitude and the
    /// ejection's push. 0 (logged) if the launcher is unknown or the store's model will not load.
    std::uint32_t launchWeapon(std::uint32_t launcher, const ms::WeaponLaunch& launch);
    /// (after each step) the stores the vehicles' store management released, flown as weapons (MS-7).
    void flyReleases();
    /// (before each step) each vehicle's station point masses set to the stores it carries, where they changed.
    void applyStoreMasses();
    /// Each aircraft's external tanks' fuel into its own tanks as they burn (before applyStoreMasses).
    void transferFuel();
    /// The weapons in flight (by id), then those that ended, newest first (the last 256).
    std::vector<ms::WeaponRecord> weapons() const;
    /// The radio path between two entities, over the world's ground (the physics terrain, else sea level).
    std::optional<ms::RadioPath> radioPath(std::uint32_t from, std::uint32_t to, const ms::RadioPathOptions& options) const;
    /// The radar cross section of `target` as `observer` sees it (MS-3): its type's table at the aspect between them.
    std::optional<ms::RadarCrossSection> radarCrossSection(std::uint32_t observer, std::uint32_t target, const ms::RcsQuery& query) const;
    /// An aircraft type's (or store's) RCS table: aircraft/<type>/rcs.bin, a stock aircraft's stand-in design's, or
    /// models/<type>.rcs.bin among the assets; null without one.
    std::shared_ptr<const ms::rcs::Table> rcsTable(const std::string& aircraft) const;

    // --- Effects ---------------------------------------------------------------
    bool addEffect(std::uint32_t id, std::unique_ptr<effects::Effect> effect);
    /// Give every vehicle (present and future) its own instance from `factory`.
    using EffectFactory = std::function<std::unique_ptr<effects::Effect>()>;
    void addEffectToAll(EffectFactory factory);
    void clearEffects(std::uint32_t id);

    // --- Environment, time, comm ---------------------------------------------
    const sim::EnvironmentState& environment() const noexcept override { return environment_; }
    bool navigation(std::uint32_t id, control::NavigationReport& out) const noexcept override;
    void setEnvironment(const sim::EnvironmentState& environment);
    double simTime() const noexcept override { return simTime_; }
    std::uint32_t vehicles(std::uint32_t* ids, std::uint32_t max) const noexcept override;
    double dt() const noexcept { return options_.dt; }
    int frameSkip() const noexcept { return options_.frameSkip; }
    comm::Network& network() noexcept { return network_; }
    const comm::Network& network() const noexcept { return network_; }
    Rng& rng() noexcept { return rng_; }

    // --- Systems, nets and the DMS (ADR-31, AB-4) -------------------------------
    /// C2 nodes' ids count from here: clear of the vehicles' and the designations'.
    static constexpr std::uint32_t kC2Nodes = 0x60000000u;
    /// A C2 node: a compute and communications node with a position (no flight model, no VI or MS). Its id, or 0 for a
    /// position that is not one.
    std::uint32_t createC2Node(const comm::C2NodeSpec& spec);
    bool removeC2Node(std::uint32_t id);
    const comm::C2NodeSpec* c2Node(std::uint32_t id) const noexcept;
    std::vector<std::uint32_t> c2Nodes() const;
    comm::NetId createNet(const comm::NetSpec& spec) { return dms_.createNet(spec); }
    /// Put a live vehicle or a C2 node on a net (false for anything else).
    bool joinNet(std::uint32_t entity, comm::NetId net);
    bool leaveNet(std::uint32_t entity, comm::NetId net) { return dms_.leave(entity, net); }
    /// Its SystemID (`want` 0: the next free) and tail number: 0 for another's id, or an entity that cannot have one.
    comm::SystemId assignSystem(std::uint32_t entity, comm::SystemId want, const std::string& tailNumber);
    Dms& dms() noexcept { return dms_; }
    const Dms& dms() const noexcept { return dms_; }
    /// The MA cores (AB-5): A-GRA's rules applied to what C2 sends each platform.
    MaCore& maCore() noexcept { return maCore_; }
    const MaCore& maCore() const noexcept { return maCore_; }
    /// A system's Data Dissemination Plan (AB-4b; the range's): false for an entity with no system yet.
    bool setDissemination(std::uint32_t entity, const comm::DisseminationPlan& plan) { return dms_.setPlan(entity, plan, simTime_); }
    /// A-GRA's native messages, built by the MA core from the platform's own reports (never an agent's bytes) and sent
    /// through its DMS: its position report (its own ship as its navigation reports it; a C2 node: where it is), its
    /// heartbeat, its radar's tracks (each its own entity report; `payload` 0xFFFF: every radar's). Published, or to
    /// `destinations`; the message numbers (none: refused, `why`).
    std::vector<std::uint64_t> publishNative(std::uint32_t entity, comm::Topic topic, std::uint16_t payload,
                                             const std::vector<comm::SystemId>& destinations, comm::DmsReason& why);

    // --- Agents (ADR-31, docs/agent-boundary.md) -------------------------------
    /// An agent the range made: an MA's decisions bound to one vehicle, commanding as a controller of its own.
    struct AgentRecord {
        std::uint32_t vehicle = 0;
        control::ControllerId controller = 0;
        std::string name;
        /// Its audit (AB-2d): each call it made, by its index (session::auditCall), and those refused it.
        std::vector<std::uint64_t> calls, refused;
    };
    /// Agents' controllers count from here: clear of the small numbers a program names its own controllers by.
    static constexpr control::ControllerId kAgentControllers = 0x40000000u;
    /// Make an agent for a live aircraft (not a weapon): its controller, or 0 for a vehicle that cannot have one.
    control::ControllerId mintAgent(std::uint32_t vehicle, const std::string& name);
    const std::vector<AgentRecord>& agents() const noexcept { return agents_; }
    /// Count an agent's call in its audit (its record's index, the call's), refused or not.
    void audit(std::size_t record, std::uint16_t call, bool refused) {
        if (record >= agents_.size()) return;
        AgentRecord& a = agents_[record];
        if (a.calls.size() <= call) a.calls.resize(call + 1u), a.refused.resize(call + 1u);
        ++a.calls[call];
        if (refused) ++a.refused[call];
    }
    /// A refusal of a call already counted (its claims: AB-2d).
    void auditRefusal(std::size_t record, std::uint16_t call) {
        if (record >= agents_.size()) return;
        AgentRecord& a = agents_[record];
        if (a.refused.size() <= call) a.calls.resize(call + 1u), a.refused.resize(call + 1u);
        ++a.refused[call];
    }

    // --- Designations (ADR-31, AB-2b; A-GRA's target designation) ----------------
    /// A target named by a vehicle's own radar track: an id clear of the vehicles' (from kDesignations), which a
    /// behaviour's, a must fly's or a strike's target may be. A behaviour reading it each step reads the track's
    /// estimate as that step began - not the aircraft behind it - and nothing once the track is gone (it lapses).
    static constexpr std::uint32_t kDesignations = 0x80000000u;
    struct Designation {
        std::uint32_t id = 0;
        std::uint32_t owner = 0;   ///< the vehicle whose track it is
        std::uint16_t payload = 0; ///< its radar
        std::uint64_t track = 0;   ///< the radar's own numbering
        comm::SystemId source = comm::kNoSystem; ///< a network track's source system (AB-4b); 0: the owner's own radar
        bool valid = false;        ///< false once the track is gone, or undesignated
        sim::VehicleState estimate{}; ///< the track's estimate, as a vehicle's state (position, velocity, heading)
    };
    /// Designate one of a vehicle's radar tracks: its target id, or 0 for a track it does not hold now.
    std::uint32_t designate(std::uint32_t owner, std::uint16_t payload, std::uint64_t track);
    /// Designate a track its system received over a link (AB-4b: a network track - its source system's radar track),
    /// flown on its last report carried forward by its velocity; 0 for one its picture does not hold now.
    std::uint32_t designateLink(std::uint32_t owner, comm::SystemId source, std::uint16_t payload, std::uint64_t track);
    /// End a designation of the vehicle's own (false for one not its own, or ended).
    bool undesignate(std::uint32_t owner, std::uint32_t target);
    const Designation* designation(std::uint32_t target) const noexcept;
    /// A vehicle's designations, live and lapsed.
    std::vector<Designation> designations(std::uint32_t owner) const;
    /// A command from an agent's controller (session/AgentRules.h): on its own vehicle, naming no other aircraft by its
    /// entity id - a behaviour's target, a must fly's entity (L4). Reason::None for any other controller's.
    control::Reason agentRefusal(std::uint32_t vehicle, control::ControllerId controller, const control::Command* command,
                                 const control::MustFlyCommand* mustFly = nullptr) const;
    const std::string& name() const noexcept { return options_.name; }
    bool published() const noexcept { return publisher_ && publisher_->active(); }
    /// A recording is being written (ADR-31, AB-6: a file another process could read the truth from).
    bool recording() const noexcept { return recorder_.isOpen(); }
    /// Ask the viewers to follow a vehicle, as a tab to it would (each request once; the user's choice stands after it):
    /// false if it is not alive or the world is not published.
    bool focusViewer(std::uint32_t id) noexcept {
        if (!published() || !alive(id)) return false;
        publisher_->requestFocus(id);
        return true;
    }
    /// The elevation tiles used for physics when `terrain` is on (null otherwise).
    io::TerrainTiles* terrain() noexcept { return terrain_.get(); }

    // --- Stepping ------------------------------------------------------------
    /// Advance every vehicle by `n` world steps (frameSkip FDM steps each).
    void step(unsigned n = 1);
    std::uint64_t vehicleSteps() const noexcept { return vehicleSteps_; }
    std::uint64_t worldSteps() const noexcept { return worldSteps_; }
    /// Push the current state to viewers now, regardless of the publish interval.
    void publishNow();

    // --- MS-7g's damage part (docs/mission-systems.md, G3 - G7) ------------------------------------------------
    /// A warhead goes off: what its fragments and blast break on each aircraft within kDamageReachM - its fuel's
    /// leaks and fires, an engine seized, a control channel stuck, the crew, the skin - drawn from each vehicle's own
    /// damage stream; the engagement's events that made (also kept for takeEngagementEvents).
    std::vector<ipc::EngagementEvent> detonate(const Detonation& detonation);
    /// The engagement's events since last taken: the detonations' and, as the damaged aircraft fly on, their fires'
    /// rates and their sections' separations.
    std::vector<ipc::EngagementEvent> takeEngagementEvents();
    /// What a vehicle's damage is (null: none).
    const ms::damage::VehicleDamage* damage(std::uint32_t id) const noexcept;
    /// A piece that left a hit aircraft flies on as a vehicle of its own (role Debris; its type
    /// "debris:<aircraft>:<section>:<x>,<y>,<z>" - its aircraft's model, that section alone, drawn so far from its
    /// centre: G6) from its section's own flight model (hangar's aircraft/<type>/pieces/).
    static constexpr double kDamageReachM = 150.0;
    /// A section's root failed outright (a test's or a scenario's structural failure, as failPayload is a payload's):
    /// it leaves as a hit's would. False: no such vehicle, damage data or section, or it is gone already.
    bool separateSection(std::uint32_t id, const std::string& section);

private:
    struct Entry {
        VehicleInfo info;
        std::size_t slot = 0;
        control::ControlStack stack;       ///< the control runtime
        control::CapabilityHost host;      ///< its contract layer (bound to `stack`)
        control::ActivityId commanded = 0; ///< the activity the last accepted command made or updated
        control::Level level = control::Level::Actuator; ///< as last published and recorded
        std::shared_ptr<const control::VehicleProfile> profile;
        std::shared_ptr<control::CapabilityCatalog> catalog; ///< its aircraft type's (or its own, with a profile of its own)
        std::shared_ptr<const control::SupportTable> support; ///< likewise: what it can do at all
        sim::PropertyHandle flapsPosition;                   ///< for flap activities that complete in position
        /// Its body's angular accelerations (p, q, r dot), for its state data's orientation accelerations (StateData.cpp).
        std::array<sim::PropertyHandle, 3> angularAcceleration;
        sim::EffectorInputs effectors;                       ///< as last written to the flight model
        std::vector<std::unique_ptr<effects::Effect>> effects;
        effects::SensedState sensed;
        Rng rng;
        sim::ControlInputs inputs;         ///< last inputs the cascade produced
        sim::VehicleState working;         ///< per-sub-step state for control/effects
        unsigned controlDivider = 1;
        bool forceApplied = false;         ///< external force pushed last step (needs zeroing)
        bool windApplied = false;
        control::NavigationSettings navigation; ///< its recovery point and reserve
        ms::Host ms;                       ///< its Mission Systems (ADR-30)
        std::uint64_t missionSignature = ~0ull; ///< what its published Mission record shows (~0: not yet written)
        bool weaponEnded = false;          ///< a weapon whose end is recorded (MS-2)
        /// Its damage (MS-7g), its own stream for its chances, what it reads to fly the damage on
        std::unique_ptr<ms::damage::VehicleDamage> damage;
        Rng damageRng;
        struct DamageReads {
            std::array<PropertyHandle, 3> aero, total;
            PropertyHandle weight, qbar;
            std::vector<PropertyHandle> tanks;
        } damageReads;
        std::map<int, double> fireRates;   ///< each burning tank's leak as last recorded (kg/s)
        bool crewLost = false;             ///< its controls held where they were
        ms::WeaponRecord origin;           ///< a weapon's: the store it was (MS-7; store, station, activity, target, jettisoned)
        /// Its stations' point masses in its flight model (MS-7: "station N", hangar's), each set to the stores on it.
        struct StationMass {
            std::uint16_t payload = 0;
            int station = 0;
            PropertyHandle weightLbs;
        };
        std::vector<StationMass> stationMass;
        std::uint64_t massRevision = ~0ull; ///< the store revision its point masses were last set for
        /// Where its stores and their carriage add drag (ms::Host::carriageDrag), for the store revision dragRevision;
        /// the dynamic pressure they read.
        std::vector<ms::Host::DragPoint> drag;
        std::uint64_t dragRevision = ~0ull;
        PropertyHandle dragQbar;
        /// The stations' mass revision its host was last told its weight for (control::CapabilityHost::setLoad).
        std::uint64_t loadRevision = ~0ull;
    };
    /// Counts the steps: a vehicle's host is told its weight every eighth, and the step after its stores change.
    unsigned loadTick_ = 0;

    /// What the control cascades see of other vehicles during a step: each
    /// one's state as the step began (Control.h, WorldView), whichever worker
    /// steps which vehicle and however far it has got.
    class StepView final : public control::WorldView {
    public:
        explicit StepView(const World& world) noexcept : world_(world) {}
        const sim::VehicleState* vehicleState(std::uint32_t id) const noexcept override;
        double simTime() const noexcept override { return world_.simTime_; }
        const sim::EnvironmentState& environment() const noexcept override { return world_.environment_; }
        std::uint32_t vehicles(std::uint32_t* ids, std::uint32_t max) const noexcept override { return world_.vehicles(ids, max); }
        /// Its own report, for the vehicle whose control update asks (4.37: a route's branch).
        bool navigation(std::uint32_t id, control::NavigationReport& out) const noexcept override { return world_.navigation(id, out); }

    private:
        const World& world_;
    };
    /// What the vehicles' hosts ask of the world when they check a command
    /// (docs/flight-autonomy.md, 4.18, 4.19): the energy on board, the
    /// ground's height. Asked at a NEW or an UPDATE, never while stepping.
    class Answers final : public control::SessionView {
    public:
        explicit Answers(const World& world, std::uint32_t vehicle = 0) noexcept : world_(world), vehicle_(vehicle) {}
        control::EnergyNow energyNow(std::uint32_t id) const override;
        double groundM(double latitudeRad, double longitudeRad) const override { return world_.ground_->heightAboveEllipsoidM(latitudeRad, longitudeRad); }
        double groundResolutionM() const override { return world_.ground_->resolutionM(); }
        double utcSeconds() const override { return world_.environment_.epochUtcSeconds + world_.simTime_; }
        double simTimeS() const override { return world_.simTime_; }
        bool frame(control::FrameId id, control::FrameSpec& spec, control::FramePose& now) const override;
        bool opPoint(control::OpPointId id, control::OpPoint& out) const override;
        const control::OpZone* opZone(control::OpZoneId id) const override;
        const control::OpLine* opLine(control::OpLineId id) const override;
        const control::OpVolume* opVolume(control::OpVolumeId id) const override;

    private:
        const World& world_;
        std::uint32_t vehicle_; ///< whose mission data it sees (AB-2c): its own, then the common
    };

    Entry* entry(std::uint32_t id) noexcept;
    const Entry* entry(std::uint32_t id) const noexcept;
    /// A marshall's slot (4.46; Marshalls.cpp): into `m`'s altitude - the one asked for, clear of the others, or the lowest of
    /// its stack clear of every other aircraft's live or waiting marshall round the same point - else StackFull.
    control::Reason slotMarshall(const Entry& e, control::MarshallCommand& m) const;
    /// A marshall's UPDATE: its slot chosen afresh where its stack moves or a slot is asked for, then the host's.
    control::CommandResult updateMarshall(control::Caller caller, control::ActivityId activity, const control::MarshallCommand& marshall,
                                          const control::PatternShape* shape);
    void preStep(std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs);
    void applyEnvironment(sim::FlightModel& model) const;
    void levelChanged(Entry& e);
    void publishVehicle(const Entry& e);
    static bool parseType(const std::string& type, std::string& family, std::string& aircraft);

    WorldOptions options_;
    std::filesystem::path jsbsimRoot_;
    std::shared_ptr<sim::GroundProvider> ground_;
    std::shared_ptr<io::TerrainTiles> terrain_;              ///< set when options.terrain (ground_ aliases it)
    std::unique_ptr<sim::VehiclePool> pool_;
    std::vector<std::unique_ptr<Entry>> entries_;               ///< by slot (null when never used)
    std::unordered_map<std::uint32_t, std::size_t> idToSlot_;
    std::vector<std::size_t> freeSlots_;
    std::vector<std::string> slotAircraft_;                     ///< loaded aircraft per slot (for reuse)
    /// Each loaded aircraft's profile (JSBSim properties under fsim/), read on its first load.
    std::unordered_map<std::string, std::shared_ptr<const control::VehicleProfile>> profiles_;
    /// What each aircraft type offers, from its profile through its adapter.
    std::unordered_map<std::string, std::shared_ptr<control::CapabilityCatalog>> catalogs_;
    /// What each aircraft type can do at all (docs/flight-autonomy.md, 4.2).
    std::unordered_map<std::string, std::shared_ptr<const control::SupportTable>> supports_;
    /// What each aircraft type carries (ADR-30: aircraft/<type>/payloads.json); null without a fit.
    std::unordered_map<std::string, std::shared_ptr<const ms::Fit>> fits_;
    /// Each aircraft type's damage data (MS-7g: hangar's <type>.damage.json beside its definition; none: it takes none)
    std::unordered_map<std::string, std::shared_ptr<const ms::damage::Airframe>> airframes_;
    std::map<std::string, std::shared_ptr<const ms::StoreCard>> storeCards_; ///< the store library, read when first needed
    std::vector<ipc::EngagementEvent> engagement_;                           ///< events not yet taken
    std::unordered_map<std::string, std::filesystem::path> aircraftDirs_; ///< each aircraft type's definition's directory
    /// One vehicle's station masses as its stores now are (applyStoreMasses's, at once).
    void applyStoreMasses(Entry& e);
    /// What a parting section takes off its aircraft (MS-7g): the aircraft as it was, where its CG was from its model's
    /// origin (body axes, m), its tanks' fuel (kg), each station's stores (number, kg).
    struct Parting {
        sim::VehicleState before;
        double cgFromOrigin[3] = {0.0, 0.0, 0.0};
        double fuelKg = 0.0;
        std::vector<std::pair<int, double>> stations;
    };
    std::uint32_t launchPiece(Entry& from, int section, const Parting& part);
    void stepDamage();
    bool ensureDamage(Entry& e);  ///< its damage begun (its data, its stream, what it reads); false: it takes none
    void separated(Entry& e, int section);
    void engage(const ipc::EngagementEvent& event); ///< kept for takeEngagementEvents, and recorded (MS-7g, G7)
    void damageFlight(Entry& e, ms::damage::Flight& out);
    ipc::EngagementEvent engagementEvent(const Entry& e, ipc::EngagementKind kind, const std::string& name) const;
    std::unordered_map<std::string, std::vector<std::string>> pointMasses_; ///< (MS-7) each aircraft type's point masses' names, in order
    /// Each type's RCS table (MS-3), read when first asked for; null without one.
    mutable std::unordered_map<std::string, std::shared_ptr<const ms::rcs::Table>> rcsTables_;
    /// The radars' path losses beyond free space, by the two entities and the frequency: the radio path's (P.528, the
    /// terrain, the gases) is worked out once a second of simulation time, the free space each look. A path's loss is
    /// the same both ways (reciprocity): two radars looking at each other share it, always worked out from the lower id
    /// to the higher, whoever asks first.
    struct PathKey {
        std::uint32_t lo = 0, hi = 0;
        double frequencyHz = 0.0;
        bool operator==(const PathKey& o) const noexcept { return lo == o.lo && hi == o.hi && frequencyHz == o.frequencyHz; }
    };
    struct PathKeyHash {
        std::size_t operator()(const PathKey& k) const noexcept {
            std::uint64_t f;
            static_assert(sizeof f == sizeof k.frequencyHz);
            std::memcpy(&f, &k.frequencyHz, sizeof f);
            std::uint64_t h = (static_cast<std::uint64_t>(k.lo) << 32 | k.hi) * 0x9E3779B97F4A7C15ull ^ f * 0xC2B2AE3D27D4EB4Full;
            return static_cast<std::size_t>(h ^ (h >> 29));
        }
    };
    struct PathExcess {
        double timeS = -1e300, excessDb = 0.0;
    };
    mutable std::unordered_map<PathKey, PathExcess, PathKeyHash> pathExcess_;
    mutable std::mutex pathExcessMutex_; ///< (the radars look in parallel)
    std::vector<std::size_t> sensing_;   ///< the slots whose sensors look this step
    std::unique_ptr<ipc::MissionRecord> missionScratch_; ///< publishMission's record, reused
    /// The sensors' looks after a step (MS-4).
    void sense();
    /// The vehicles' Mission Systems records for the viewers, those whose payloads or radars changed (MS-4).
    void publishMission();
    std::vector<sim::ControlInputs> poolInputs_;
    std::vector<ipc::FlightStatus> poolStatus_; ///< each slot's flight status, filled as a snapshot is published (fillStatus)
    void fillStatus();
    std::vector<sim::VehicleState> stepStates_;                 ///< by slot: every state as the current step began
    StepView stepView_{*this};
    std::vector<EffectFactory> worldEffects_;
    sim::EnvironmentState environment_;
    std::uint64_t appliedEnvironment_ = ~0ull;
    comm::Network network_;
    Dms dms_;      ///< (ADR-31, AB-4) systems, nets, each system's DMS
    MaCore maCore_; ///< (AB-5) each platform's MA core
    Rng dmsRng_;   ///< its draws (loss, jitter): a stream of its own, so a world without nets flies as before
    struct C2Node {
        comm::C2NodeSpec spec;
        bool alive = true;
    };
    std::vector<C2Node> c2Nodes_; ///< by id - kC2Nodes - 1, never reused
    std::vector<AgentRecord> agents_; ///< (ADR-31) every agent made, in order
    std::vector<Designation> designations_; ///< (ADR-31, AB-2b) by id - kDesignations - 1, never reused
    /// Each live designation's estimate from its track now (the step's start), or lapsed.
    void refreshDesignations();
    void stepDms();
    std::unique_ptr<ipc::WorldPublisher> publisher_;
    ipc::RecordWriter recorder_;
    double lastRecordedTime_ = -1.0;
    /// Per slot, the stores as last recorded ('M' records written when they change; kept apart from Entry, whose
    /// layout the step's speed is measured with)
    std::vector<std::vector<ipc::MissionStore>> recordedStores_;
    std::uint64_t recordedEnvironment_ = ~0ull; ///< the environment's revision last recorded ('E' records)
    void recordVehicle(const Entry& e);
    /// A weapon's flight over: its record kept (ADR-30, MS-2).
    void endWeapon(Entry& e, ms::WeaponEnd end, const sim::VehicleState& state);
    std::deque<ms::WeaponRecord> endedWeapons_;
    void recordFrame();
    Rng rng_;
    std::uint32_t nextId_ = 1;
    std::size_t liveCount_ = 0;
    double simTime_ = 0.0;
    std::uint64_t vehicleSteps_ = 0, worldSteps_ = 0;
    std::unordered_map<std::uint32_t, std::unique_ptr<Answers>> views_; ///< each vehicle's host's view (AB-2c: its own data)
    /// A frame and its owner (AB-2c).
    struct KeptFrame {
        control::FrameSpec spec;
        std::uint32_t owner = 0;
    };
    std::map<control::FrameId, KeptFrame> frames_; ///< the reference frames, by id (4.21)
    /// Is a frame there for a viewer (common, or its own)?
    bool seesFrame(control::FrameId id, std::uint32_t viewer) const;
    /// Mission data by (owner, id): 0 the range's common, else a vehicle's own (AB-2c).
    template <typename T>
    using Owned = std::map<std::pair<std::uint32_t, std::uint64_t>, T>;
    control::FrameId lastFrame_ = 0;
    Owned<control::OpPoint> opPoints_;   ///< the operational points (4.42)
    Owned<control::OpZone> opZones_;     ///< the operational zones (4.43)
    Owned<control::OpLine> opLines_;     ///< the operational lines (4.44)
    Owned<control::OpVolume> opVolumes_; ///< the operational volumes (4.45)
};

} // namespace fsim::session
