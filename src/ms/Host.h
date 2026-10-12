#pragma once

// A vehicle's Mission Systems (ADR-30, MS-1): its payloads from its aircraft's
// fit, the capabilities they give, their states, the commands that change them
// and the activities those make, and who may command them (grants, as the
// flight capabilities' under control::ControlMode::Granted). Stepped with the
// world, deterministic.

#include "ms/Cards.h"
#include "ms/Model.h"
#include "ms/radar/Radar.h"

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::ms {

class Host {
public:
    /// The vehicle's payloads from its fit (none without one); its radars' random streams from `seed`.
    void bind(std::uint32_t vehicle, std::shared_ptr<const Fit> fit, std::uint64_t seed = 0);
    /// Its radars' random streams restart from `seed` (as bind's).
    void reseed(std::uint64_t seed);

    const std::vector<PayloadInfo>& payloads() const noexcept { return payloads_; }
    const std::vector<CapabilityInfo>& capabilities() const noexcept { return capabilities_; }
    const Card* card(std::uint16_t payload) const noexcept;
    const StoreCard* storeCard(std::uint16_t index) const noexcept {
        return index < stores_.size() ? stores_[index].card.get() : nullptr;
    }
    /// A capability by its instance id ("fsim.ms.amti:radar"); -1 if none.
    int find(std::string_view id) const noexcept;
    /// Why a capability this vehicle does not offer is refused: NotSupported for
    /// a known type ("fsim.ms.amti"), UnknownCapability otherwise.
    static Reason refusal(std::string_view id) noexcept;
    CapabilityStatus status(std::size_t capability) const noexcept;

    // --- Commands ---
    CommandResult submit(const StateCommand& command, const CommandOptions& options, double simTime);
    CommandResult submit(const BitCommand& command, const CommandOptions& options, double simTime);
    /// Search with a radar (MS-4): its AMTI capability's activity.
    CommandResult submit(const AmtiCommand& command, const CommandOptions& options, double simTime);
    CommandResult cancel(control::Caller caller, ActivityId activity, double simTime);
    /// The platform fails a payload (a contingency: A-GRA's sensor and payload failures), or repairs it (back to Off).
    Reason setFailed(std::uint16_t payload, bool failed, double simTime);

    // --- Activities ---
    const ActivityRecord* activity(ActivityId id) const noexcept;
    /// Live activities, then ended ones it remembers, newest first.
    std::vector<ActivityRecord> activities() const;

    // --- Authority ---
    void setControlMode(control::ControlMode mode) noexcept { mode_ = mode; }
    Reason requestControl(std::size_t capability, control::ControllerId controller);
    Reason releaseControl(std::size_t capability, control::ControllerId controller, double simTime);
    Reason revokeControl(std::size_t capability, double simTime);
    Reason setAllowed(std::size_t capability, bool allowed, double simTime);
    control::ControlStatus controlStatus(std::size_t capability) const noexcept;
    /// The platform restricts a capability (or, Available, lifts it).
    Reason setAvailability(std::size_t capability, control::Availability availability);

    /// After each world step: warm-ups and BITs that finish.
    void step(double simTime);

    // --- Sensors (MS-4) ---
    /// A payload's radar; null if it is not one.
    const radar::Radar* radar(std::uint16_t payload) const noexcept;
    RadarStatus radarStatus(std::uint16_t payload) const noexcept;
    /// A radar payload's confirmed tracks (MS-5), predicted to `now`, relative to its carrier's state `own`.
    std::vector<AmtiTrack> amtiTracks(std::uint16_t payload, double now, const sim::VehicleState& own) const;
    /// Whether a sensor runs (sense() has work).
    bool sensing() const noexcept;
    /// After each world step: the sensors' looks up to `simTime`, at the carrier's state `own`, against `scene` -
    /// the radars', then the other payloads' models (MS-9a: a warning receiver's).
    void sense(double simTime, const sim::VehicleState& own, const Scene& scene);
    /// The radars alone, against a scene that has only what a radar needs.
    void sense(double simTime, const sim::VehicleState& own, const radar::Scene& scene);

    // --- The other payloads' models (MS-9 to MS-11; ms/Model.h) ---
    /// What the vehicle radiates now, as a receiver meets it: each radar that is on - the volume its main beam goes
    /// through, its power, what it does. Added to `out`; `own` (its carrier's state) must outlive what reads them.
    void emissions(const sim::VehicleState& own, double now, std::vector<Emission>& out) const;
    /// What a payload's sensor holds (a warning receiver's emitters): none for a payload with no such model, or not
    /// operating.
    std::vector<Contact> contacts(std::uint16_t payload) const;
    /// A payload's own figures (its model's report): none for a payload without a model.
    std::vector<PayloadValue> report(std::uint16_t payload) const;

    // --- Stores (MS-7; HostStores.cpp) ---
    /// The installed stores, station by station (A-GRA's MA_StrikeCapability inventory).
    const std::vector<StoreInfo>& stores() const noexcept { return storeInfo_; }
    /// Load a station (the ground crew's - a scenario's, not a Mission Systems command): `count` stores of a type, or
    /// none (an empty `store`: unloaded). Refused while its stores are in a strike or jettison, or where the station
    /// cannot take them.
    /// `roomKg`: what the aircraft may still take on before its maximum take-off weight (infinite: none held) - a
    /// load heavier than what it replaces by more is refused OverMaxWeight, nothing changed.
    Reason load(std::uint16_t payload, int station, const std::string& store, int count = 1,
                double roomKg = std::numeric_limits<double>::infinity());
    /// The fit's loadout of that name: each station it names loaded as it says (the others left as they are). Held to
    /// `roomKg` as a whole: all of it, or none.
    Reason loadLoadout(const std::string& name, double roomKg = std::numeric_limits<double>::infinity());
    /// What it carries now, kg: every store on its stations (a tank's with its fuel).
    double storesMassKg() const noexcept;
    /// Strike (A-GRA's Strike task): its activity powers the stores up, asks for consent, releases them.
    CommandResult submit(const StrikeCommand& command, const CommandOptions& options, double simTime);
    /// Jettison (A-GRA's Jettison task): its activity releases the stores unarmed.
    CommandResult submit(const JettisonCommand& command, const CommandOptions& options, double simTime);
    /// Release consent for a strike asking for it (StrikeConsentRequestStatus): granted, it releases; refused, it ends.
    Reason strikeConsent(ActivityId activity, bool granted, double simTime);
    /// A strike's or jettison's progress (while live, and after: its stores and weapons).
    std::optional<StrikeStatus> strikeStatus(ActivityId activity) const;
    /// Its strikes asking for release consent now (ADR-31, AB-5b: what its MA core asks C2 for), appended to `out`.
    void awaitingConsent(std::vector<ActivityId>& out) const;
    /// A store leaving its station: the world flies it (launchWeapon), then says which entity it became (released).
    struct Release {
        std::uint16_t store = 0;
        WeaponLaunch launch;
        ActivityId activity = 0;
        std::uint32_t target = 0;
        bool jettisoned = false;
    };
    /// This step's releases (taken: the queue empties).
    std::vector<Release> takeReleases();
    bool releasing() const noexcept { return !releases_.empty(); }
    /// A store released as `weapon` (0: it could not be flown - its strike fails).
    void released(std::uint16_t store, std::uint32_t weapon, double simTime);
    /// The mass a station carries now (kg): its stores still on it.
    double stationMassKg(std::uint16_t payload, int station) const noexcept;
    /// Bumped whenever a station's load changes (a load, a release): the world updates the flight model's masses.
    std::uint64_t storeRevision() const noexcept { return storeRevision_; }
    /// Its external tanks' fuel left (kg): what they can still give the aircraft's own tanks.
    double externalFuelKg() const noexcept;
    /// Fuel drawn from them (kg, as far as they hold it; from each alike): what was drawn. Their mass changes with it
    /// (massRevision).
    double drawExternalFuel(double kg);
    /// A store's fuel left (kg; a tank's).
    double storeFuelKg(std::uint16_t store) const noexcept;
    /// Bumped whenever a station's mass changes - its load, a release, a tank's fuel drawn: the world sets the flight
    /// model's masses again.
    std::uint64_t massRevision() const noexcept { return storeRevision_ + fuelRevision_; }
    /// The stores on a station gone with the part of the aircraft it was on (MS-7g): Lost, no longer its mass; theirs.
    double loseStation(int station, double simTime);
    /// A point where the stores and their carriage add drag (MS-7g): its station's place (body axes, from the model's
    /// origin - its empty CG) and its CD S below Mach 0.8 (m^2). A store's own (its card's drag_area_m2) while it is on,
    /// with its lugs under an ejector; the carriage itself - a launcher rail, a pylon and its rack - from the first time
    /// its station holds a store until the part of the aircraft it is on goes (a released store leaves its pylon);
    /// nothing in a weapon bay (its doors shut), for a wingtip's rail (the airframe's own) or a missile's fuselage well.
    /// In a sideslip its stores also turn and push the aircraft (their cards' figures, hangar storedrag.py; a store in a
    /// fuselage well is part of the fuselage's outline: none): their bodies' unstable moment, q munkM3 sin(2 beta), a
    /// couple that turns the nose away from the wind (Munk: why tanks cost directional stability); their fins' and
    /// bases' side force, q sideSlopeM2 sin(beta) - half their cards', as carried fins measure - acting sideArmM behind the
    /// station.
    struct DragPoint {
        double positionM[3] = {0.0, 0.0, 0.0};
        double areaM2 = 0.0;
        double munkM3 = 0.0;
        double sideSlopeM2 = 0.0;
        double sideArmM = 0.0;
    };
    void carriageDrag(std::vector<DragPoint>& out) const;
    /// A store's drag in carriage at that Mach number over its drag below Mach 0.8: its transonic rise.
    static double storeDragMachFactor(double mach) noexcept;
    /// Bumped whenever a weapon bay's doors move (StoreInfo::bayDoors): what a viewer draws, nothing the flight sees.
    std::uint64_t doorsRevision() const noexcept { return doorsRevision_; }

private:
    struct Payload {
        PayloadState state = PayloadState::Off;
        PayloadState target = PayloadState::Off; ///< where a warm-up goes
        PayloadState beforeBit = PayloadState::Off;
        double until = 0.0;       ///< a warm-up's or a BIT's end, simulation time
        double since = 0.0;       ///< its start
        ActivityId activity = 0;  ///< the live state or BIT activity
        ActivityId amti = 0;      ///< the live AMTI activity (its radar searching)
    };
    struct Control {
        bool allowed = true, granted = false;
        control::ControllerId holder = 0;
        control::Availability restricted = control::Availability::Available;
    };

    Reason admit(std::size_t capability, const CommandOptions& options) const noexcept;
    ActivityRecord& start(std::size_t capability, std::uint16_t payload, const CommandOptions& options, double simTime);
    void end(ActivityId id, control::ActivityState state, Reason reason, double simTime);
    ActivityRecord* live(ActivityId id) noexcept;
    void setState(std::uint16_t payload, PayloadState state);
    // stores (HostStores.cpp)
    struct StoreSlot {
        std::uint16_t payload = 0;
        const Station* station = nullptr;
        std::shared_ptr<const StoreCard> card;
        StoreStatus status = StoreStatus::Loaded;
        double until = 0.0;        ///< a power-up's or power-down's end
        ActivityId activity = 0;   ///< the strike or jettison holding it
        std::uint32_t weapon = 0;
        double fuelKg = 0.0;       ///< a tank's fuel left (its card's fuel_kg as loaded)
    };
    /// A store's mass now: its card's, and a tank's fuel left.
    static double massOf(const StoreSlot& s) noexcept;
    struct Strike {
        ActivityId id = 0;
        std::uint16_t payload = 0;
        std::vector<std::uint16_t> stores;
        std::size_t next = 0;      ///< the next to release
        std::size_t confirmed = 0; ///< releases the world has flown
        double nextAt = 0.0;
        StrikePhase phase = StrikePhase::PoweringUp;
        bool consent = false, jettison = false, emergency = false;
        std::uint32_t target = 0;
        double ripple = 0.25;
        std::vector<std::uint32_t> weapons;
    };
    void rebuildStoreInfo();
    void advanceStrikes(double simTime);
    void endStrike(ActivityId id);               ///< (from end(): its unreleased stores freed)
    void stationStateChanged(std::uint16_t payload, PayloadState state);
    Strike* strike(ActivityId id) noexcept;
    /// Why a payload in that state cannot do what operating would (PayloadOff, PayloadStandby ...).
    static Reason unavailableReason(PayloadState state) noexcept;

    std::uint32_t vehicle_ = 0;
    std::shared_ptr<const Fit> fit_;
    std::vector<PayloadInfo> payloads_;
    std::vector<Payload> state_;
    std::vector<const Card*> cards_;
    std::vector<std::unique_ptr<radar::Radar>> radars_; ///< by payload: a radar's model, null for the rest
    std::vector<std::unique_ptr<Model>> models_;        ///< by payload: its model beyond its state (MS-9a on), null for the rest
    std::vector<Rng> modelRng_;                         ///< by payload: its model's own stream
    std::vector<int> amtiCapability_;                  ///< by payload: its AMTI capability, -1 for none
    std::vector<CapabilityInfo> capabilities_;
    std::vector<std::string> ids_; ///< the capabilities' instance ids (their strings)
    std::vector<Control> control_;
    std::vector<std::size_t> statusCapability_; ///< each payload's status capability
    control::ControlMode mode_ = control::ControlMode::Open;
    std::vector<ActivityRecord> live_;
    std::deque<ActivityRecord> ended_;
    std::uint32_t serial_ = 0;
    double now_ = 0.0; ///< the last step's time (a search a state change ends, ends then)
    // stores
    std::vector<int> strikeCapability_, storeCapability_; ///< by payload: its strike and store management capabilities (-1: none)
    std::vector<StoreSlot> stores_;
    std::vector<StoreInfo> storeInfo_;
    std::vector<Strike> strikes_;
    std::deque<StrikeStatus> endedStrikes_; ///< (their last status, for strikeStatus after they end)
    std::vector<Release> releases_;
    std::uint64_t storeRevision_ = 0;
    std::uint64_t fuelRevision_ = 0; ///< bumped as its tanks give their fuel (massRevision)
    /// The stations that have held a store - their carriage on the aircraft from then on (carriageDrag) - and how it
    /// held it (a launcher rail, else a pylon and rack); and the stations gone with their part of the aircraft.
    enum class Mount : std::uint8_t { Pylon, Rail, Well };
    struct Carriage {
        std::uint16_t payload = 0;
        const Station* station = nullptr;
        Mount mount = Mount::Pylon;
    };
    std::vector<Carriage> carriage_;
    std::vector<int> lostStations_;
    /// A weapon bay's doors (MS-7e): how far open (0 shut .. 1), until when they are held open (a release from the bay
    /// holds them a moment after it). Kept apart from the store revision: moving doors change no mass.
    struct Doors {
        std::string bay;
        double position = 0.0, openUntil = -1.0;
    };
    std::vector<Doors> doors_;
    double doorsTime_ = -1.0;
    std::uint64_t doorsRevision_ = 0;
    Doors& doors(const std::string& bay);
    void advanceDoors(double simTime);
};

} // namespace fsim::ms
