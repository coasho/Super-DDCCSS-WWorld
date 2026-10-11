#include "session/World.h"

#include "session/AgentRules.h"

#include "core/Log.h"
#include "core/Units.h"
#include "io/AssetResolver.h"
#include "platform/Threads.h"
#include "control/Laws.h"
#include "control/Profile.h"
#include "fsim/ControllerRegistry.h"
#include "sim/JsbsimModel.h"
#include "ms/radar/Radar.h"
#include "ms/rf/Path.h"
#include "sim/Attitude.h"

#include <fstream>
#include <sstream>
#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

namespace fsim::session {

namespace {

/// The stream a vehicle's sensors draw from (its radars' looks), apart from its flight model's (stream 0).
constexpr std::uint64_t kSensorStream = 0x5345'4E53ull;

void copyName(char* dst, std::size_t capacity, const std::string& src) {
    std::memset(dst, 0, capacity);
    std::memcpy(dst, src.data(), std::min(src.size(), capacity - 1));
}

/// A radar's last signal-level interval for the viewer's range-Doppler display (ipc::MissionRangeDoppler): binned to
/// the record's cells (each the largest it holds, so a target's peak stays), Doppler turned so zero is in the middle
/// and closing to the right.
void publishRangeDoppler(const ms::radar::Radar& radar, std::uint8_t index, ipc::MissionRangeDoppler& out) {
    const ms::radar::signal::Snapshot& s = radar.snapshot();
    const int rows = s.rows, m = s.filters;
    if (rows <= 0 || m <= 0 || s.power.size() != static_cast<std::size_t>(rows) * static_cast<std::size_t>(m)) return;
    const int rowBin = (rows + static_cast<int>(ipc::kRangeDopplerRows) - 1) / static_cast<int>(ipc::kRangeDopplerRows);
    const int colBin = (m + static_cast<int>(ipc::kRangeDopplerColumns) - 1) / static_cast<int>(ipc::kRangeDopplerColumns);
    const int shownRows = (rows + rowBin - 1) / rowBin, shownCols = (m + colBin - 1) / colBin;
    out.radar = index;
    out.rows = static_cast<std::uint16_t>(shownRows), out.columns = static_cast<std::uint16_t>(shownCols);
    out.rowBin = static_cast<std::uint16_t>(rowBin), out.columnBin = static_cast<std::uint16_t>(colBin);
    out.prfHz = static_cast<float>(s.prfHz);
    out.firstRangeM = static_cast<float>(s.firstGate * s.gateM), out.rowM = static_cast<float>(rowBin * s.gateM);
    out.ambiguousRangeM = static_cast<float>(s.ambiguousRangeM), out.ambiguousRateMs = static_cast<float>(s.ambiguousRateMs);
    out.mainBeamRateMs = static_cast<float>(s.mainBeamRateMs), out.notchMs = static_cast<float>(s.notchMs);
    out.altitudeM = static_cast<float>(s.altitudeM);
    out.timeS = radar.snapshotTimeS();
    // filter k's frequency is k / m of the PRF, folded to [-1/2, 1/2): its column (k + m/2) mod m puts -1/2 at 0
    auto column = [&](int k) { return ((k + m / 2) % m) / colBin; };
    std::fill(std::begin(out.db), std::end(out.db), std::uint8_t{0});
    for (int r = 0; r < rows; ++r) {
        const float* row = s.power.data() + static_cast<std::size_t>(r) * static_cast<std::size_t>(m);
        std::uint8_t* to = out.db + static_cast<std::size_t>(r / rowBin) * static_cast<std::size_t>(shownCols);
        for (int k = 0; k < m; ++k) {
            const float v = row[k];
            if (!(v > 1.0f)) continue; // (at or below the noise's mean: 0)
            const int q = std::min(255, static_cast<int>(40.0f * std::log10(v))); // (0.25 dB a step)
            std::uint8_t& cell = to[column(k)];
            if (q > cell) cell = static_cast<std::uint8_t>(q);
        }
    }
    out.peakCount = 0;
    for (const auto& [r, k] : s.peaks) {
        if (out.peakCount >= ipc::kRangeDopplerPeaks) break;
        out.peakRow[out.peakCount] = static_cast<std::uint16_t>(r / rowBin), out.peakColumn[out.peakCount] = static_cast<std::uint16_t>(column(k));
        ++out.peakCount;
    }
}

/// The aircraft's profile (docs/control-architecture.md, 7): its properties
/// fsim/<section>/..., read once per aircraft type. Its gains are the control
/// section, fsim/control/<controller id>/<parameter> with the parameter's
/// dots written as slashes ("pid_attitude/pitch/kp" sets pid_attitude's
/// "pitch.kp"); a setting no built-in takes is reported once, here. The
/// applicability section's sources are in the header of the aircraft's
/// `file` (docs/flight-autonomy.md, 5.2).
std::shared_ptr<const control::VehicleProfile> readAircraftProfile(const std::string& aircraft, const sim::FlightModel& model,
                                                                   const std::filesystem::path& file) {
    std::vector<std::string> warnings;
    auto profile = std::make_shared<control::VehicleProfile>(
        control::readProfile(aircraft, [&model](std::string_view prefix) { return model.properties(prefix); }, warnings));
    if (profile->applicability.header.present()) control::readApplicabilitySources(file, profile->applicability);
    control::requireSources(aircraft, profile->applicability, warnings);
    for (const auto& w : warnings) LOG_WARN("session") << w;
    control::adapterFor(profile->identity.family).complete(*profile);
    if (control::completeControl(*profile)) // no gains of its own: the laws designed from its plant
        LOG_INFO("session") << "aircraft '" << aircraft << "': the loops designed from its identified plant";
    const auto& settings = profile->control.settings;
    if (!settings.empty()) {
        // a setting a controller of that id takes - whether or not a level flies it - is in use
        std::unique_ptr<control::Controller> probe;
        for (const auto& s : settings) {
            if (!probe || s.controller != probe->id()) probe = control::ControllerRegistry::instance().create(s.controller);
            if (!probe || !probe->setParameter(s.parameter, s.value))
                LOG_WARN("session") << "aircraft '" << aircraft << "': no built-in controller takes fsim/control setting " << s.controller << " "
                                    << s.parameter << "; kept for a controller registered under that id";
        }
        LOG_INFO("session") << "aircraft '" << aircraft << "' carries " << settings.size() << " controller settings";
    }
    return profile;
}

} // namespace

World::World(const WorldOptions& options) : options_(options), dmsRng_(Rng::forVehicle(options.seed, 0x444D53u, 0)), rng_(options.seed) {
    options_.frameSkip = std::max(1, options_.frameSkip);
    io::AssetResolver assets;
    if (auto root = assets.jsbsimRoot(options_.jsbsimRoot)) jsbsimRoot_ = *root;
    else LOG_WARN("session") << "JSBSim data root not found; vehicles of type jsbsim:* will fail to load";
    if (options_.ground) {
        ground_ = options_.ground;
    } else if (options_.terrain) {
        io::TerrainTiles::Options to;
        if (!options_.terrainUrl.empty()) to.urlTemplate = options_.terrainUrl;
        to.zoom = options_.terrainZoom;
        to.offline = options_.terrainOffline;
        terrain_ = std::make_shared<io::TerrainTiles>(to);
        ground_ = terrain_;
    } else {
        ground_ = std::make_shared<sim::FlatGround>(0.0);
    }

    const unsigned physical = platform::physicalCoreCount();
    const unsigned workers = options_.workers ? options_.workers : std::max(1u, physical > 2 ? physical - 2 : 1u);
    pool_ = std::make_unique<sim::VehiclePool>(workers, options_.pinWorkers && workers < physical);
    pool_->setPreStep([this](std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs) {
        preStep(slot, subStep, model, inputs);
    });

    if (options_.publish) {
        ipc::WorldPublisher::Options po;
        po.name = options_.name;
        po.capacity = options_.capacity;
        po.dt = options_.dt;
        po.frameSkip = options_.frameSkip;
        po.publishIntervalSeconds = options_.publishIntervalSeconds;
        publisher_ = std::make_unique<ipc::WorldPublisher>(po);
        publisher_->setEnvironment(environment_);
    }
    if (!options_.recordPath.empty()) {
        ipc::RecordingHeader rh;
        rh.capacity = options_.capacity;
        rh.dt = options_.dt;
        rh.frameSkip = options_.frameSkip;
        copyName(rh.name, ipc::kNameLength, options_.name);
        recorder_.open(options_.recordPath, rh);
    }
    LOG_INFO("session") << "world '" << options_.name << "': dt " << options_.dt << " s, frame skip " << options_.frameSkip << ", "
                        << workers << " worker(s)" << (published() ? ", published" : "");
}

World::~World() = default;

bool World::parseType(const std::string& type, std::string& family, std::string& aircraft) {
    const auto colon = type.find(':');
    if (colon == std::string::npos) {
        family = "jsbsim";
        aircraft = type;
    } else {
        family = type.substr(0, colon);
        aircraft = type.substr(colon + 1);
    }
    return !aircraft.empty();
}

namespace {

/// A Mission Systems capability's instance id (ADR-30): "fsim.ms.<type>:<payload>".
bool isMission(std::string_view capability) noexcept { return capability.rfind("fsim.ms.", 0) == 0; }

/// Its reason in the flight calls' terms: every one a grant or an availability call returns has a control::Reason of the same name.
control::Reason toControl(ms::Reason r) noexcept {
    switch (r) {
    case ms::Reason::None: return control::Reason::None;
    case ms::Reason::UnknownVehicle: return control::Reason::UnknownVehicle;
    case ms::Reason::NotSupported: return control::Reason::NotSupported;
    case ms::Reason::NotImplemented: return control::Reason::NotImplemented;
    case ms::Reason::NotGranted: return control::Reason::NotGranted;
    case ms::Reason::NotAllowed: return control::Reason::NotAllowed;
    case ms::Reason::AuthorityHeld: return control::Reason::AuthorityHeld;
    case ms::Reason::Restricted: return control::Reason::Restricted;
    default: return control::Reason::UnknownCapability;
    }
}

template <typename F>
control::Reason missionControl(const ms::Host& host, std::string_view capability, F&& call) {
    const int index = host.find(capability);
    if (index < 0) return toControl(ms::Host::refusal(capability));
    return toControl(call(static_cast<std::size_t>(index)));
}

} // namespace

std::uint32_t World::createVehicle(const VehicleSpec& spec) {
    std::string family, aircraft;
    if (!parseType(spec.type, family, aircraft)) {
        LOG_ERROR("session") << "bad vehicle type '" << spec.type << "'";
        return 0;
    }
    if (family != "jsbsim") {
        LOG_ERROR("session") << "unknown flight model family '" << family << "' (only jsbsim in this build)";
        return 0;
    }
    if (!spec.name.empty() && find(spec.name) != 0) {
        LOG_ERROR("session") << "vehicle name '" << spec.name << "' already exists";
        return 0;
    }

    // Reuse a freed slot with the same aircraft loaded, else load a new model.
    std::size_t slot = static_cast<std::size_t>(-1);
    for (auto it = freeSlots_.begin(); it != freeSlots_.end(); ++it)
        if (slotAircraft_[*it] == aircraft) {
            slot = *it;
            freeSlots_.erase(it);
            break;
        }
    // Terrain under and around the spawn point must be known before JSBSim
    // trims (on-ground spawns) or the first steps sample it.
    if (terrain_)
        terrain_->prefetch(units::degreesToRadians(spec.initial.latitudeDeg), units::degreesToRadians(spec.initial.longitudeDeg),
                           options_.terrainPrefetchRadiusM);
    if (slot == static_cast<std::size_t>(-1)) {
        auto model = std::make_unique<sim::JsbsimModel>(options_.dt, ground_);
        const sim::AircraftSpec files{aircraft, jsbsimRoot_,
                                      spec.aircraftDir.empty() ? io::AssetResolver{}.findAircraft(aircraft, jsbsimRoot_) : spec.aircraftDir};
        if (!model->load(files, spec.initial)) {
            LOG_ERROR("session") << "failed to load aircraft '" << aircraft << "'";
            return 0;
        }
        if (!profiles_.count(aircraft)) {
            const auto dir = files.aircraftDir.empty() ? files.jsbsimRoot / "aircraft" : files.aircraftDir;
            const auto profile = readAircraftProfile(aircraft, *model, dir / aircraft / (aircraft + ".xml"));
            profiles_[aircraft] = profile;
            catalogs_[aircraft] = std::make_shared<control::CapabilityCatalog>(*profile, control::adapterFor(profile->identity.family));
            supports_[aircraft] = std::make_shared<control::SupportTable>(*profile, *catalogs_[aircraft]);
            // what it carries (ADR-30): its fit beside its definition, its cards from the payload library
            std::vector<std::string> errors;
            const auto library = io::AssetResolver{}.find("payloads");
            fits_[aircraft] = library ? ms::readFit(dir / aircraft / "payloads.json", *library, errors) : nullptr;
            for (const auto& error : errors) LOG_WARN("session") << "payloads: " << error;
            // what a warhead breaks on it (MS-7g): its damage data beside its definition, if hangar wrote one; its
            // pieces' definitions beside it
            aircraftDirs_[aircraft] = dir / aircraft;
            {
                const auto file = dir / aircraft / (aircraft + ".damage.json");
                std::string error;
                if (std::filesystem::is_regular_file(file)) {
                    if (auto a = ms::damage::readAirframe(file, error)) airframes_[aircraft] = std::make_shared<const ms::damage::Airframe>(std::move(*a));
                    else LOG_WARN("session") << "damage data: " << error;
                }
            }
            // its point masses' names in order (their properties are indexed so): the stations' are hangar's "station N"
            {
                std::ifstream in(dir / aircraft / (aircraft + ".xml"), std::ios::binary);
                std::stringstream text;
                text << in.rdbuf();
                const std::string xml = text.str();
                auto& names = pointMasses_[aircraft];
                for (std::size_t at = xml.find("<pointmass"); at != std::string::npos; at = xml.find("<pointmass", at + 1)) {
                    const std::size_t q0 = xml.find("name=\"", at), close = xml.find('>', at);
                    if (q0 == std::string::npos || q0 > close) {
                        names.emplace_back();
                        continue;
                    }
                    const std::size_t q1 = xml.find('"', q0 + 6);
                    names.push_back(xml.substr(q0 + 6, q1 - q0 - 6));
                }
            }
        }
        slot = pool_->add(std::move(model));
        entries_.emplace_back();
        slotAircraft_.push_back(aircraft);
        poolInputs_.emplace_back();
        stepStates_.emplace_back();
    } else {
        pool_->vehicle(slot).reset(spec.initial);
    }

    const std::uint32_t id = nextId_++;
    auto e = std::make_unique<Entry>();
    e->info.id = id;
    e->info.name = spec.name.empty() ? aircraft + "-" + std::to_string(id) : spec.name;
    e->info.type = family + ":" + aircraft;
    e->info.aircraft = aircraft;
    e->info.model = spec.model;
    e->info.initial = spec.initial;
    e->info.alive = true;
    e->info.generation = 1;
    e->slot = slot;
    e->rng = Rng::forVehicle(options_.seed, 0, id);
    e->controlDivider = std::max(1u, spec.controlDivider);
    // Neutral defaults until the first command: cruise power airborne, idle on the ground.
    e->inputs.setThrottleAll(spec.initial.onGround ? 0.0 : 0.65);
    e->inputs.gearDown = spec.initial.onGround ? 1.0 : 0.0;
    e->stack.setInitialInputs(e->inputs);
    e->profile = profiles_[aircraft];
    e->catalog = catalogs_[aircraft];
    e->support = supports_[aircraft];
    if (spec.profile) {
        // sections of its own: its own profile, completed by its own family's adapter, and its own catalog
        auto own = std::make_shared<control::VehicleProfile>(control::mergeProfile(*e->profile, *spec.profile));
        std::vector<std::string> warnings;
        control::requireSources(e->info.name, own->applicability, warnings); // its own declarations need their sources too
        for (const auto& w : warnings) LOG_WARN("session") << w;
        const auto& adapter = control::adapterFor(own->identity.family);
        adapter.complete(*own);
        if (own->control.header.provenance == control::Provenance::Derived) own->control = {}; // designed again, from its plant
        control::completeControl(*own);
        e->catalog = std::make_shared<control::CapabilityCatalog>(*own, adapter);
        e->support = std::make_shared<control::SupportTable>(*own, *e->catalog);
        e->profile = std::move(own);
    }
    const auto& adapter = control::adapterFor(e->profile->identity.family);
    e->stack.setAdapter(adapter);
    if (!e->profile->control.settings.empty()) e->stack.setControllerSettings(e->profile->control.settings);
    for (const auto& [level, controller] : e->profile->control.controllers) e->stack.use(level, controller); // with the settings above
    e->host.bind(id, e->stack, *e->catalog, adapter, *e->profile, options_.dt * e->controlDivider);
    {
        // what it weighs as it is made: the weight its least airspeeds are its profile's at (docs/vehicle-interface.md, 7.3)
        const sim::EnergyOnBoard energy = pool_->vehicle(slot).energy();
        e->host.setLoad(energy.massKg, energy.fuelKg);
    }
    e->host.setSupport(e->support.get());
    // (its endurance and terrain checks: docs/flight-autonomy.md, 4.18, 4.19; the mission data it sees, its own: AB-2c)
    e->host.setSessionView((views_[id] = std::make_unique<Answers>(*this, id)).get());
    e->host.setAltimeter(control::Altimeter{air(), control::Altimeter::kStandardPa}); // (until its QNH is set: 4.20)
    e->ms.bind(id, fits_[aircraft], Rng::forVehicle(options_.seed, kSensorStream, id).next());
    // its stations' point masses (MS-7): hangar's "station N", found by name among its definition's
    e->stationMass.clear(), e->massRevision = ~0ull;
    if (const auto names = pointMasses_.find(aircraft); names != pointMasses_.end())
        for (const auto& p : e->ms.payloads()) {
            const ms::Card* card = e->ms.card(p.index);
            if (p.family != ms::Family::StoreStation || !card) continue;
            for (const auto& st : card->stations) {
                const auto it = std::find(names->second.begin(), names->second.end(), "station " + std::to_string(st.number));
                if (it == names->second.end()) continue;
                const std::size_t index = static_cast<std::size_t>(it - names->second.begin());
                PropertyHandle h = pool_->vehicle(slot).property(index ? "inertia/pointmass-weight-lbs[" + std::to_string(index) + "]"
                                                                       : std::string("inertia/pointmass-weight-lbs"));
                if (h.valid()) e->stationMass.push_back(Entry::StationMass{p.index, st.number, h});
            }
        }
    (void)rcsTable(aircraft); // (its RCS table read with the vehicle: the radars then only look it up, in parallel)
    e->flapsPosition = pool_->vehicle(slot).property("fcs/flap-pos-norm");
    for (int k = 0; k < 3; ++k) // (its state data's: docs/flight-autonomy.md, 4.21)
        e->angularAcceleration[static_cast<std::size_t>(k)] = pool_->vehicle(slot).property(k == 0 ? "accelerations/pdot-rad_sec2"
                                                                                                : k == 1 ? "accelerations/qdot-rad_sec2"
                                                                                                         : "accelerations/rdot-rad_sec2");
    for (auto& factory : worldEffects_) e->effects.push_back(factory());
    sim::FlightModel& model = pool_->vehicle(slot);
    model.seed(e->rng.next());
    applyEnvironment(model);
    model.state(e->working);
    e->sensed.state = e->working;
    pool_->refreshState(slot);
    pool_->setActive(slot, true);
    poolInputs_[slot] = e->inputs;

    network_.createNode(id, id);
    entries_[slot] = std::move(e);
    idToSlot_[id] = slot;
    ++liveCount_;
    publishVehicle(*entries_[slot]);
    if (publisher_ && slot >= publisher_->capacity())
        LOG_WARN("session") << "vehicle '" << entries_[slot]->info.name << "' is beyond the published capacity (" << publisher_->capacity()
                            << "); viewers will not show it";
    LOG_INFO("session") << "vehicle " << id << " '" << entries_[slot]->info.name << "' (" << entries_[slot]->info.type << ") created in slot " << slot;
    return id;
}

bool World::removeVehicle(std::uint32_t id) {
    Entry* e = entry(id);
    if (!e) return false;
    if (e->info.role == ms::Role::Weapon) endWeapon(*e, ms::WeaponEnd::Removed, pool_->states()[e->slot]);
    const std::size_t slot = e->slot;
    pool_->setActive(slot, false);
    network_.removeNode(id);
    dms_.removeEntity(id); // (off its nets; its SystemID stays the referee's)
    views_.erase(id); // (its host goes with it; its own mission data stays the referee's to read, named by nobody)
    if (publisher_) {
        publisher_->clearVehicle(static_cast<std::uint32_t>(slot));
        publisher_->setMission(static_cast<std::uint32_t>(slot), *std::make_unique<ipc::MissionRecord>()); // (none in the slot now)
    }
    if (recorder_.isOpen()) {
        ipc::RecordedVehicle r;
        r.id = id;
        r.generation = e->info.generation + 1;
        r.alive = 0;
        recorder_.writeVehicle(static_cast<std::uint32_t>(slot), r);
    }
    idToSlot_.erase(id);
    entries_[slot].reset();
    freeSlots_.push_back(slot);
    --liveCount_;
    return true;
}

bool World::resetVehicle(std::uint32_t id, const sim::InitialConditions* ic, std::optional<std::uint64_t> seed) {
    Entry* e = entry(id);
    if (!e) return false;
    if (ic) e->info.initial = *ic;
    if (terrain_ && ic)
        terrain_->prefetch(units::degreesToRadians(ic->latitudeDeg), units::degreesToRadians(ic->longitudeDeg), options_.terrainPrefetchRadiusM);
    sim::FlightModel& model = pool_->vehicle(e->slot);
    if (!model.reset(e->info.initial)) return false;
    if (seed) { // its random stream from `seed`, as a new vehicle's from the world's
        e->rng = Rng::forVehicle(*seed, 0, id);
        model.seed(e->rng.next());
        e->ms.reseed(Rng::forVehicle(*seed, kSensorStream, id).next());
    }
    // A flight model's reset returns its air to calm and standard: the world's, as a new vehicle's (design 7.2)
    applyEnvironment(model);
    e->inputs = sim::ControlInputs{};
    e->inputs.setThrottleAll(e->info.initial.onGround ? 0.0 : 0.65);
    e->inputs.gearDown = e->info.initial.onGround ? 1.0 : 0.0;
    e->stack.setInitialInputs(e->inputs);
    e->stack.reset();
    e->host.onReset();
    for (auto& fx : e->effects) fx->onReset();
    // whole again: what a hit or a ground strike broke, holed and set alight is forgotten with the flight it was in
    // (its flight model's reset put its mass and its inertia back)
    e->damage.reset();
    e->damageReads = {};
    e->fireRates.clear();
    e->wreckFireUntilS = 0.0;
    e->wreckFireName.clear();
    poolInputs_[e->slot] = e->inputs;
    model.state(e->working);
    e->sensed.state = e->working;
    pool_->refreshState(e->slot);
    ++e->info.generation;
    publishVehicle(*e);
    return true;
}

bool World::alive(std::uint32_t id) const noexcept { return entry(id) != nullptr; }

const VehicleInfo* World::info(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->info : nullptr;
}

std::uint32_t World::find(const std::string& name) const noexcept {
    for (const auto& e : entries_)
        if (e && e->info.name == name) return e->info.id;
    return 0;
}

std::vector<std::uint32_t> World::vehicleIds() const {
    std::vector<std::uint32_t> ids;
    for (const auto& e : entries_)
        if (e) ids.push_back(e->info.id);
    std::sort(ids.begin(), ids.end());
    return ids;
}

World::Entry* World::entry(std::uint32_t id) noexcept {
    const auto it = idToSlot_.find(id);
    return it == idToSlot_.end() ? nullptr : entries_[it->second].get();
}

const World::Entry* World::entry(std::uint32_t id) const noexcept {
    const auto it = idToSlot_.find(id);
    return it == idToSlot_.end() ? nullptr : entries_[it->second].get();
}

control::Reason World::agentRefusal(std::uint32_t vehicle, control::ControllerId controller, const control::Command* command,
                                    const control::MustFlyCommand* mustFly) const {
    for (const AgentRecord& a : agents_) {
        if (a.controller != controller) continue;
        if (a.vehicle != vehicle) return control::Reason::NotAllowed;
        const AgentIdentity who{a.vehicle, a.controller, {}};
        if (command) return agentCommand(who, *command, this);
        if (mustFly) return agentMustFly(who, *mustFly, this);
        return control::Reason::None;
    }
    return control::Reason::None;
}

control::ControllerId World::mintAgent(std::uint32_t vehicle, const std::string& name) {
    const VehicleInfo* i = info(vehicle);
    if ((!i || !i->alive || i->role != ms::Role::Aircraft) && !c2Node(vehicle)) return 0; // (an aircraft, or a C2 node: AB-4)
    const auto controller = kAgentControllers + static_cast<control::ControllerId>(agents_.size()) + 1;
    agents_.push_back({vehicle, controller, name, {}, {}});
    return controller;
}

// --- Systems, nets and the DMS (ADR-31, AB-4) -----------------------------------------------------------------------

std::uint32_t World::createC2Node(const comm::C2NodeSpec& spec) {
    if (!std::isfinite(spec.latitudeRad) || !std::isfinite(spec.longitudeRad) || !std::isfinite(spec.altitudeMslM) ||
        std::abs(spec.latitudeRad) > 1.5707963267948966)
        return 0;
    c2Nodes_.push_back({spec, true});
    const std::uint32_t id = kC2Nodes + static_cast<std::uint32_t>(c2Nodes_.size());
    if (c2Nodes_.back().spec.name.empty()) c2Nodes_.back().spec.name = "c2-" + std::to_string(c2Nodes_.size());
    return id;
}

bool World::removeC2Node(std::uint32_t id) {
    if (!c2Node(id)) return false;
    c2Nodes_[id - kC2Nodes - 1].alive = false;
    dms_.removeEntity(id);
    return true;
}

const comm::C2NodeSpec* World::c2Node(std::uint32_t id) const noexcept {
    if (id <= kC2Nodes || id - kC2Nodes > c2Nodes_.size()) return nullptr;
    const C2Node& n = c2Nodes_[id - kC2Nodes - 1];
    return n.alive ? &n.spec : nullptr;
}

std::vector<std::uint32_t> World::c2Nodes() const {
    std::vector<std::uint32_t> out;
    for (std::size_t i = 0; i < c2Nodes_.size(); ++i)
        if (c2Nodes_[i].alive) out.push_back(kC2Nodes + static_cast<std::uint32_t>(i) + 1);
    return out;
}

bool World::joinNet(std::uint32_t entity, comm::NetId net) {
    if (!entry(entity) && !c2Node(entity)) return false;
    return dms_.join(entity, net);
}

comm::SystemId World::assignSystem(std::uint32_t entity, comm::SystemId want, const std::string& tailNumber) {
    if (!entry(entity) && !c2Node(entity)) return comm::kNoSystem;
    return dms_.assign(entity, want, tailNumber);
}

std::vector<std::uint64_t> World::publishNative(std::uint32_t entity, comm::Topic topic, std::uint16_t payload,
                                                const std::vector<comm::SystemId>& destinations, comm::DmsReason& why) {
    std::vector<std::uint64_t> sent;
    why = comm::DmsReason::None;
    comm::DmsMessage m;
    m.topic = topic, m.destinations = destinations;
    auto send = [&]() {
        if (const std::uint64_t id = dms_.send(entity, m, simTime_, why, true)) sent.push_back(id);
    };
    if (topic == comm::topic::kPositionReport) {
        comm::PositionReport p;
        p.timeS = simTime_;
        if (const effects::SensedState* s = sensedState(entity)) { // (its own ship as its navigation reports it)
            const OwnShip o = ownShipOf(*s);
            p.latitudeRad = o.latitudeRad, p.longitudeRad = o.longitudeRad, p.altitudeMslM = o.altitudeMslM;
            for (int i = 0; i < 3; ++i) p.velocityNedMs[i] = o.velocityNedMs[i];
            p.headingRad = o.eulerRad[2];
            p.airspeedTrueMs = o.airspeedTrueMs, p.airspeedCalibratedMs = o.airspeedCalibratedMs, p.mach = o.mach;
        } else if (const comm::C2NodeSpec* n = c2Node(entity)) {
            p.latitudeRad = n->latitudeRad, p.longitudeRad = n->longitudeRad, p.altitudeMslM = n->altitudeMslM;
        } else {
            why = comm::DmsReason::NoSystem;
            return sent;
        }
        m.bytes = comm::encode(p);
        send();
    } else if (topic == comm::topic::kSystemStatus) {
        comm::SystemStatusReport h;
        h.timeS = simTime_;
        const std::string tail = dms_.tailNumber(dms_.systemOf(entity));
        std::memcpy(h.tailNumber, tail.data(), std::min(tail.size(), sizeof h.tailNumber - 1));
        m.bytes = comm::encode(h);
        send();
    } else if (topic == comm::topic::kEntity) {
        const Entry* e = entry(entity);
        if (!e) {
            why = comm::DmsReason::NoSystem;
            return sent;
        }
        for (const ms::PayloadInfo& p : e->ms.payloads()) {
            if (p.family != ms::Family::AirRadar || (payload != 0xFFFFu && p.index != payload)) continue;
            for (const ms::AmtiTrack& t : amtiTracks(entity, p.index)) { // (its numbering and estimate, never `truth`)
                comm::EntityReport r;
                r.track = t.id, r.payload = p.index, r.updates = t.updates, r.timeS = t.timeS;
                r.latitudeRad = t.latitudeRad, r.longitudeRad = t.longitudeRad, r.heightM = t.heightM;
                for (int i = 0; i < 3; ++i) r.velocityNedMs[i] = t.velocityNedMs[i];
                r.positionSigmaM = t.positionSigmaM, r.velocitySigmaMs = t.velocitySigmaMs;
                m.bytes = comm::encode(r);
                send();
            }
        }
    } else {
        why = comm::DmsReason::ReservedTopic;
    }
    return sent;
}

void World::stepDms() {
    // what each system's Data Dissemination Plan has due: its MA core publishes it from the platform's own reports
    comm::DmsReason why;
    for (const Dms::Due& d : dms_.due(simTime_)) {
        if (d.status) publishNative(d.entity, comm::topic::kSystemStatus, 0, {}, why);
        if (d.position) publishNative(d.entity, comm::topic::kPositionReport, 0, {}, why);
        if (d.tracks) publishNative(d.entity, comm::topic::kEntity, 0xFFFFu, {}, why);
    }
    // where each system is: a vehicle's true position, a C2 node's own (the delivery's physics, as a radar's echo)
    struct Where final : Dms::Locator {
        const World& world;
        explicit Where(const World& w) : world(w) {}
        bool locate(std::uint32_t entity, double& lat, double& lon, double& h) const override {
            if (const Entry* e = world.entry(entity)) {
                const sim::VehicleState& s = world.pool_->states()[e->slot];
                lat = s.latitudeRad, lon = s.longitudeRad, h = s.altitudeMslM;
                return true;
            }
            if (const comm::C2NodeSpec* n = world.c2Node(entity)) {
                lat = n->latitudeRad, lon = n->longitudeRad, h = n->altitudeMslM;
                return true;
            }
            return false;
        }
    };
    dms_.step(simTime_, Where(*this), ground_.get(), dmsRng_);
    maCore_.step(*this); // (C2's commands, as each platform's MA core takes them)
}

std::uint32_t World::vehicles(std::uint32_t* ids, std::uint32_t max) const noexcept {
    std::uint32_t n = 0; // (by slot: the same order every step)
    for (const auto& e : entries_)
        if (e) {
            if (n < max) ids[n] = e->info.id;
            ++n;
        }
    return n;
}

const sim::VehicleState* World::vehicleState(std::uint32_t id) const noexcept {
    if (id >= kDesignations) [[unlikely]] { // (a designation: its track's estimate, AB-2b)
        const Designation* d = designation(id);
        return d && d->valid ? &d->estimate : nullptr;
    }
    const Entry* e = entry(id);
    return e ? &pool_->states()[e->slot] : nullptr;
}

const sim::VehicleState* World::StepView::vehicleState(std::uint32_t id) const noexcept {
    if (id >= kDesignations) [[unlikely]] { // (its estimate as the step began: refreshed before it)
        const Designation* d = world_.designation(id);
        return d && d->valid ? &d->estimate : nullptr;
    }
    const Entry* e = world_.entry(id);
    return e ? &world_.stepStates_[e->slot] : nullptr;
}

namespace {

/// A radar track's estimate as a vehicle's state: where it is, how it moves, its heading and climb from its velocity.
sim::VehicleState estimateOf(const ms::AmtiTrack& t) {
    sim::VehicleState s;
    s.simTime = t.timeS;
    s.positionEcef[0] = t.stateEcef[0], s.positionEcef[1] = t.stateEcef[2], s.positionEcef[2] = t.stateEcef[4];
    s.latitudeRad = t.latitudeRad, s.longitudeRad = t.longitudeRad, s.altitudeMslM = t.heightM;
    for (int i = 0; i < 3; ++i) s.velocityNedMs[i] = t.velocityNedMs[i];
    const double ground = std::hypot(t.velocityNedMs[0], t.velocityNedMs[1]);
    s.eulerRad[2] = std::atan2(t.velocityNedMs[1], t.velocityNedMs[0]);
    s.eulerRad[1] = std::atan2(-t.velocityNedMs[2], ground);
    s.airspeedTrueMs = std::hypot(ground, t.velocityNedMs[2]);
    return s;
}

/// A network track's estimate now: its source's last report carried forward by its velocity (flat earth over the
/// report's age - seconds).
sim::VehicleState estimateOf(const comm::LinkTrack& l, double now) {
    const comm::EntityReport& r = l.report;
    constexpr double kEarthM = 6371000.0;
    const double age = std::max(0.0, now - r.timeS);
    sim::VehicleState s;
    s.simTime = now;
    s.latitudeRad = r.latitudeRad + r.velocityNedMs[0] * age / kEarthM;
    s.longitudeRad = r.longitudeRad + r.velocityNedMs[1] * age / (kEarthM * std::max(1e-6, std::cos(r.latitudeRad)));
    s.altitudeMslM = r.heightM - r.velocityNedMs[2] * age;
    ms::rf::geodeticToEcef(s.latitudeRad, s.longitudeRad, s.altitudeMslM, s.positionEcef);
    for (int i = 0; i < 3; ++i) s.velocityNedMs[i] = r.velocityNedMs[i];
    const double ground = std::hypot(r.velocityNedMs[0], r.velocityNedMs[1]);
    s.eulerRad[2] = std::atan2(r.velocityNedMs[1], r.velocityNedMs[0]);
    s.eulerRad[1] = std::atan2(-r.velocityNedMs[2], ground);
    s.airspeedTrueMs = std::hypot(ground, r.velocityNedMs[2]);
    return s;
}

} // namespace

std::uint32_t World::designateLink(std::uint32_t owner, comm::SystemId source, std::uint16_t payload, std::uint64_t track) {
    if (!entry(owner)) return 0;
    const comm::LinkTrack* t = dms_.linkTrack(owner, source, payload, track);
    if (!t) return 0;
    Designation d;
    d.id = kDesignations + static_cast<std::uint32_t>(designations_.size()) + 1;
    d.owner = owner, d.payload = payload, d.track = track, d.source = source, d.valid = true, d.estimate = estimateOf(*t, simTime_);
    designations_.push_back(d);
    return d.id;
}

std::uint32_t World::designate(std::uint32_t owner, std::uint16_t payload, std::uint64_t track) {
    if (!entry(owner)) return 0;
    for (const ms::AmtiTrack& t : amtiTracks(owner, payload))
        if (t.id == track) {
            Designation d;
            d.id = kDesignations + static_cast<std::uint32_t>(designations_.size()) + 1;
            d.owner = owner, d.payload = payload, d.track = track, d.valid = true, d.estimate = estimateOf(t);
            designations_.push_back(d);
            return d.id;
        }
    return 0;
}

bool World::undesignate(std::uint32_t owner, std::uint32_t target) {
    if (target <= kDesignations || target - kDesignations > designations_.size()) return false;
    Designation& d = designations_[target - kDesignations - 1];
    if (d.owner != owner || !d.valid) return false;
    d.valid = false;
    return true;
}

const World::Designation* World::designation(std::uint32_t target) const noexcept {
    if (target <= kDesignations || target - kDesignations > designations_.size()) return nullptr;
    return &designations_[target - kDesignations - 1];
}

std::vector<World::Designation> World::designations(std::uint32_t owner) const {
    std::vector<Designation> out;
    for (const Designation& d : designations_)
        if (d.owner == owner) out.push_back(d);
    return out;
}

void World::refreshDesignations() {
    for (Designation& d : designations_) {
        if (!d.valid) continue;
        d.valid = false; // (lapsed, unless its track is still held)
        if (!entry(d.owner)) continue;
        if (d.source != comm::kNoSystem) { // (a network track: its last report, carried forward)
            if (const comm::LinkTrack* t = dms_.linkTrack(d.owner, d.source, d.payload, d.track))
                d.estimate = estimateOf(*t, simTime_), d.valid = true;
            continue;
        }
        for (const ms::AmtiTrack& t : amtiTracks(d.owner, d.payload))
            if (t.id == d.track) {
                d.estimate = estimateOf(t), d.valid = true;
                break;
            }
    }
}

const effects::SensedState* World::sensedState(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->sensed : nullptr;
}

sim::FlightModel* World::model(std::uint32_t id) noexcept {
    Entry* e = entry(id);
    return e ? &pool_->vehicle(e->slot) : nullptr;
}

const sim::ControlInputs* World::inputs(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->inputs : nullptr;
}

bool World::command(std::uint32_t id, const control::Command& command) {
    Entry* e = entry(id);
    if (!e) return false;
    if (e->host.updateLegacy(command)) return true; // the per-step path: a new setpoint, same activity
    return commandResult(id, command).accepted();
}

bool World::command(std::uint32_t id, const control::Command& command, control::ControllerId controller) {
    Entry* e = entry(id);
    if (!e) return false;
    if (controller > kAgentControllers && (std::holds_alternative<control::BehaviorCommand>(command) ||
                                           std::holds_alternative<control::MustFlyCommand>(command)) &&
        agentRefusal(id, controller, &command) != control::Reason::None)
        return false;
    if (std::holds_alternative<control::BehaviorCommand>(command)) e->catalog->refresh();
    // (the per-step path: an update of the live activity - a new setpoint - else a NEW as the controller's)
    const control::CommandResult r = e->host.command(command, pool_->states()[e->slot], simTime_, controller);
    if (r.accepted() && r.activity != e->commanded) { // a new activity (an update keeps its level)
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r.accepted();
}

control::CommandResult World::commandResult(std::uint32_t id, const control::Command& command) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    if (std::holds_alternative<control::BehaviorCommand>(command)) e->catalog->refresh();
    const control::CommandResult r = e->host.command(command, pool_->states()[e->slot], simTime_);
    if (r.accepted() && r.activity != e->commanded) { // a new activity (an update keeps its level)
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::Command& command, const control::CommandOptions& options) {
    if (options.controller > kAgentControllers) [[unlikely]] { // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(id, options.controller, &command, nullptr); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = options.commandId;
            return r;
        }
        // C2's overrides in force: its HSA flies them (AB-5b)
        if (std::holds_alternative<control::HsaCommand>(command) && maCore_.hasOverrides(id)) {
            control::HsaCommand h = std::get<control::HsaCommand>(command);
            maCore_.applyOverrides(id, h);
            return submitChecked(id, control::Command(h), options);
        }
    }
    return submitChecked(id, command, options);
}

control::CommandResult World::submitChecked(std::uint32_t id, const control::Command& command, const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    if (std::holds_alternative<control::BehaviorCommand>(command)) e->catalog->refresh();
    control::CommandResult r = e->host.submit(command, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                     const control::CommandOptions& options, Span<const control::RouteLoiter> loiters,
                                     Span<const control::RouteState> states, Span<const control::RoutePath> paths,
                                     Span<const control::RouteBranch> branches, Span<const control::RouteTerminator> terminators) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    std::vector<control::Waypoint> overridden; // (an agent's route flies C2's overrides in force: AB-5b)
    if (options.controller > kAgentControllers && maCore_.hasOverrides(id)) [[unlikely]] {
        overridden.assign(waypoints.begin(), waypoints.end());
        maCore_.applyOverrides(id, overridden);
        waypoints = Span<const control::Waypoint>(overridden.data(), overridden.size());
    }
    control::CommandResult r =
        e->host.submit(route, waypoints, options, pool_->states()[e->slot], simTime_, loiters, states, paths, branches, terminators);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpZone& zone,
                                     const control::CommandOptions& options) {
    if (options.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(id, options.controller, nullptr, &mustFly); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = options.commandId;
            return r;
        }
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(mustFly, &zone, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpLine& line,
                                     const control::CommandOptions& options) {
    if (options.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(id, options.controller, nullptr, &mustFly); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = options.commandId;
            return r;
        }
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(mustFly, &line, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::MustFlyCommand& mustFly, const control::OpVolume& volume,
                                     const control::CommandOptions& options) {
    if (options.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(id, options.controller, nullptr, &mustFly); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = options.commandId;
            return r;
        }
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(mustFly, &volume, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::PatternCommand& pattern, const control::PatternShape& shape,
                                     const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(pattern, shape, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                     const control::CommandOptions& options, const control::CurveShape* shape) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(curve, segments, options, pool_->states()[e->slot], simTime_, shape);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                     const control::CommandOptions& options, const control::CurveShape* shape) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(curve, segments, options, pool_->states()[e->slot], simTime_, shape);
    r.commandId = options.commandId;
    if (r.accepted()) {
        e->commanded = r.activity;
        levelChanged(*e);
    }
    return r;
}

control::CommandResult World::submit(std::uint32_t id, const control::SupportCommand& command, const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    control::CommandResult r = e->host.submit(command, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted() && std::holds_alternative<control::EnginesCommand>(command)) levelChanged(*e); // thrust left the cascade
    return r;
}

control::Reason World::setVehicleDefault(std::uint32_t id, control::VehicleDefault mode) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    const control::Reason r = e->host.setVehicleDefault(mode);
    if (r == control::Reason::None) levelChanged(*e);
    return r;
}

control::VehicleDefault World::vehicleDefault(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->host.vehicleDefault() : control::VehicleDefault::Neutral;
}

control::Reason World::setProtection(std::uint32_t id, control::ProtectionMode mode) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    e->host.setProtection(mode);
    return control::Reason::None;
}

control::ProtectionMode World::protection(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->host.protection() : control::ProtectionMode::Off;
}

control::EnvelopeStatus World::envelope(std::uint32_t id) {
    Entry* e = entry(id);
    return e ? e->host.envelope() : control::EnvelopeStatus{};
}

namespace {

control::CommandResult unknownActivity(control::ActivityId activity) {
    control::CommandResult r;
    r.reason = control::Reason::UnknownActivity;
    r.activity = activity;
    return r;
}

} // namespace

namespace {

/// A refused UPDATE or CANCEL echoes the command id of the activity it addressed, as an accepted one does.
void echo(const control::CapabilityHost& host, control::CommandResult& r) noexcept {
    if (r.status == control::CommandStatus::Rejected && r.activity && !r.commandId)
        if (const control::ActivityRecord* a = host.activity(r.activity)) r.commandId = a->commandId;
}

} // namespace

std::vector<control::CommandResult> World::submitBatch(std::uint32_t id, Span<const control::BatchCommand> batch,
                                                       std::vector<control::CommandDetails>* details) {
    std::vector<control::CommandResult> out;
    out.reserve(batch.size());
    if (details) details->clear(), details->reserve(batch.size());
    for (const control::BatchCommand& b : batch) {
        if (b.marshall) { // (in place of its command: 4.46)
            out.push_back(submit(id, *b.marshall, b.shape ? *b.shape : control::PatternShape{}, b.options));
        } else if (b.intercept) { // (likewise: 4.47)
            out.push_back(submit(id, *b.intercept, b.options));
        } else if (const auto* support = std::get_if<control::SupportCommand>(&b.command)) {
            out.push_back(submit(id, *support, b.options));
        } else {
            const control::Command& c = std::get<control::Command>(b.command);
            if (const auto* route = std::get_if<control::RouteCommand>(&c))
                out.push_back(submit(id, *route, b.waypoints, b.options, b.loiters, b.states, b.paths, b.branches, b.terminators));
            else if (const auto* curve = std::get_if<control::CurveCommand>(&c))
                out.push_back(b.nurbs.empty() ? submit(id, *curve, b.segments, b.options, b.curveShape) : submit(id, *curve, b.nurbs, b.options, b.curveShape));
            else if (const auto* pattern = std::get_if<control::PatternCommand>(&c); pattern && b.shape) out.push_back(submit(id, *pattern, *b.shape, b.options));
            else if (const auto* mustFly = std::get_if<control::MustFlyCommand>(&c); mustFly && (b.zone || b.line || b.volume)) {
                if ((b.zone ? 1 : 0) + (b.line ? 1 : 0) + (b.volume ? 1 : 0) > 1) { // (a zone, a corridor or a volume: one)
                    control::CommandResult r;
                    r.reason = control::Reason::InvalidParameter, r.commandId = b.options.commandId;
                    out.push_back(r);
                } else {
                    out.push_back(b.zone   ? submit(id, *mustFly, *b.zone, b.options)
                                  : b.line ? submit(id, *mustFly, *b.line, b.options)
                                           : submit(id, *mustFly, *b.volume, b.options));
                }
            }
            else out.push_back(submit(id, c, b.options));
        }
        if (details) {
            const control::CommandDetails* d = commandDetails(id);
            details->push_back(d ? *d : control::CommandDetails{});
        }
    }
    return out;
}

const control::CommandDetails* World::commandDetails(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->host.details() : nullptr;
}

control::CommandResult World::update(control::ActivityId activity, const control::SupportCommand& setpoint) {
    return update(control::Source::Policy, activity, setpoint);
}

control::CommandResult World::update(control::ActivityId activity, const control::Command& setpoint) {
    return update(control::Source::Policy, activity, setpoint);
}

control::CommandResult World::update(control::ActivityId activity, const control::RouteCommand& route, Span<const control::Waypoint> waypoints,
                                     Span<const control::RouteLoiter> loiters, Span<const control::RouteState> states,
                                     Span<const control::RoutePath> paths, Span<const control::RouteBranch> branches,
                                     Span<const control::RouteTerminator> terminators) {
    return update(control::Source::Policy, activity, route, waypoints, loiters, states, paths, branches, terminators);
}

control::CommandResult World::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::BezierSegment> segments,
                                     const control::CurveShape* shape) {
    return update(control::Source::Policy, activity, curve, segments, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::CurveCommand& curve, Span<const control::NurbsSegment> segments,
                                     const control::CurveShape* shape) {
    return update(control::Source::Policy, activity, curve, segments, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::PatternCommand& pattern, const control::PatternShape& shape) {
    return update(control::Source::Policy, activity, pattern, shape);
}

control::CommandResult World::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpZone& zone) {
    return update(control::Source::Policy, activity, mustFly, zone);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                     const control::OpZone& zone) {
    if (caller.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(control::activityVehicle(activity), caller.controller, nullptr, &mustFly); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = 0;
            return r;
        }
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, mustFly, &zone, pool_->states()[e->slot], caller);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpLine& line) {
    return update(control::Source::Policy, activity, mustFly, line);
}

control::CommandResult World::update(control::ActivityId activity, const control::MustFlyCommand& mustFly, const control::OpVolume& volume) {
    return update(control::Source::Policy, activity, mustFly, volume);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                     const control::OpVolume& volume) {
    if (caller.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(control::activityVehicle(activity), caller.controller, nullptr, &mustFly); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = 0;
            return r;
        }
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, mustFly, &volume, pool_->states()[e->slot], caller);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::MustFlyCommand& mustFly,
                                     const control::OpLine& line) {
    if (caller.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(control::activityVehicle(activity), caller.controller, nullptr, &mustFly); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = 0;
            return r;
        }
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, mustFly, &line, pool_->states()[e->slot], caller);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::cancel(control::ActivityId activity) { return cancel(control::Source::Policy, activity); }

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::SupportCommand& setpoint) {
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, setpoint, caller);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::Command& setpoint) {
    if (caller.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's command names no other aircraft)
        if (const auto refused = agentRefusal(control::activityVehicle(activity), caller.controller, &setpoint, nullptr); refused != control::Reason::None) {
            control::CommandResult r;
            r.reason = refused, r.commandId = 0;
            return r;
        }
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, setpoint, pool_->states()[e->slot], caller);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::RouteCommand& route,
                                     Span<const control::Waypoint> waypoints, Span<const control::RouteLoiter> loiters,
                                     Span<const control::RouteState> states, Span<const control::RoutePath> paths,
                                     Span<const control::RouteBranch> branches, Span<const control::RouteTerminator> terminators) {
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r =
            e->host.update(activity, route, waypoints, pool_->states()[e->slot], caller, loiters, states, paths, branches, terminators);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                     Span<const control::BezierSegment> segments, const control::CurveShape* shape) {
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, curve, segments, pool_->states()[e->slot], caller, shape);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::CurveCommand& curve,
                                     Span<const control::NurbsSegment> segments, const control::CurveShape* shape) {
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, curve, segments, pool_->states()[e->slot], caller, shape);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::update(control::Caller caller, control::ActivityId activity, const control::PatternCommand& pattern,
                                     const control::PatternShape& shape) {
    if (Entry* e = entry(control::activityVehicle(activity))) {
        control::CommandResult r = e->host.update(activity, pattern, shape, pool_->states()[e->slot], caller);
        echo(e->host, r);
        return r;
    }
    return unknownActivity(activity);
}

control::CommandResult World::cancel(control::Caller caller, control::ActivityId activity) {
    Entry* e = entry(control::activityVehicle(activity));
    if (!e) return unknownActivity(activity);
    control::CommandResult r = e->host.cancel(activity, pool_->states()[e->slot], simTime_, caller);
    echo(e->host, r);
    if (r.status == control::CommandStatus::Canceled) levelChanged(*e);
    return r;
}

control::CommandResult World::activityCommand(control::Caller caller, control::ActivityId activity, control::ActivityCommand command,
                                              control::Rank rank) {
    Entry* e = entry(control::activityVehicle(activity));
    if (!e) return unknownActivity(activity);
    control::CommandResult r = e->host.activityCommand(activity, command, rank, pool_->states()[e->slot], simTime_, caller);
    echo(e->host, r);
    if (r.accepted()) levelChanged(*e); // (what flew may have stopped, or what waited started)
    return r;
}

control::CommandResult World::commandBranch(control::Caller caller, control::ActivityId activity, std::uint32_t branch, bool commanded) {
    Entry* e = entry(control::activityVehicle(activity));
    if (!e) return unknownActivity(activity);
    control::CommandResult r = e->host.commandBranch(activity, branch, commanded, caller);
    echo(e->host, r);
    return r;
}

control::Reason World::storeTask(std::uint32_t id, control::TaskId task, const control::Command& command, Span<const control::Waypoint> waypoints,
                                 Span<const control::BezierSegment> segments, control::TaskRepetition repetition, const control::PatternShape* shape) {
    control::BatchCommand item;
    item.command = command, item.waypoints = waypoints, item.segments = segments, item.shape = shape;
    return storeTask(id, task, item, repetition);
}

control::Reason World::storeTask(std::uint32_t id, control::TaskId task, const control::BatchCommand& item, control::TaskRepetition repetition) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    const auto* command = std::get_if<control::Command>(&item.command);
    if (!command) return control::Reason::InvalidParameter; // (a flight or guidance command is kept)
    if (item.marshall) return control::Reason::InvalidParameter; // (a marshall's slot is its stack's at its NEW: never a task - 4.46)
    if (item.intercept) return control::Reason::NotImplemented;  // (a route intercept kept as a task: not built - 4.47)
    if (std::holds_alternative<control::BehaviorCommand>(*command)) e->catalog->refresh();
    std::vector<control::NurbsSegment> made; // (Bezier segments, each made one)
    if (item.nurbs.empty())
        for (const control::BezierSegment& b : item.segments) made.push_back(control::NurbsSegment::of(b));
    const Span<const control::NurbsSegment> segments = item.nurbs.empty() ? Span<const control::NurbsSegment>(made) : item.nurbs;
    return e->host.storeTask(task, *command, item.waypoints, segments, repetition, item.shape, item.curveShape, item.loiters, item.states,
                             item.paths, item.branches, item.terminators, item.zone, item.line, item.volume);
}

control::CommandResult World::commandTask(std::uint32_t id, control::TaskId task, const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        r.commandId = options.commandId;
        return r;
    }
    e->catalog->refresh(); // (a behaviour's)
    control::CommandResult r = e->host.commandTask(task, options, pool_->states()[e->slot], simTime_);
    r.commandId = options.commandId;
    if (r.accepted()) levelChanged(*e);
    return r;
}

control::CommandResult World::cancelTask(std::uint32_t id, control::TaskId task, control::Caller caller) {
    Entry* e = entry(id);
    if (!e) {
        control::CommandResult r;
        r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    control::CommandResult r = e->host.cancelTask(task, pool_->states()[e->slot], simTime_, caller);
    if (r.status == control::CommandStatus::Canceled) levelChanged(*e);
    return r;
}

control::Reason World::removeTask(std::uint32_t id, control::TaskId task) {
    Entry* e = entry(id);
    return e ? e->host.removeTask(task) : control::Reason::UnknownVehicle;
}

std::optional<control::TaskStatus> World::taskStatus(std::uint32_t id, control::TaskId task) {
    Entry* e = entry(id);
    control::TaskStatus s;
    if (!e || !e->host.taskStatus(task, s)) return std::nullopt;
    return s;
}

std::vector<control::TaskStatus> World::tasks(std::uint32_t id) {
    Entry* e = entry(id);
    return e ? e->host.tasks() : std::vector<control::TaskStatus>{};
}

control::Reason World::publishPlan(std::uint32_t id, const control::RoutePlan& plan) {
    Entry* e = entry(id);
    return e ? e->host.publishPlan(plan) : control::Reason::UnknownVehicle;
}

control::PlanCommandResult World::planCommand(std::uint32_t id, control::PlanId plan, control::PlanCommand command,
                                              const control::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) {
        control::PlanCommandResult r;
        r.plan = plan, r.command = command, r.reason = control::Reason::UnknownVehicle;
        r.check.commandId = options.commandId;
        return r;
    }
    control::PlanCommandResult r = e->host.planCommand(plan, command, options, pool_->states()[e->slot], simTime_);
    r.check.commandId = options.commandId;
    if (r.completed && (command == control::PlanCommand::Activate || command == control::PlanCommand::Deactivate)) levelChanged(*e);
    return r;
}

control::PlanCommandResult World::abortPlan(std::uint32_t id, control::PlanId plan, control::Reason reason) {
    Entry* e = entry(id);
    if (!e) {
        control::PlanCommandResult r;
        r.plan = plan, r.command = control::PlanCommand::Deactivate, r.reason = control::Reason::UnknownVehicle;
        return r;
    }
    control::PlanCommandResult r = e->host.abortPlan(plan, reason, pool_->states()[e->slot], simTime_);
    if (r.completed) levelChanged(*e);
    return r;
}

control::Reason World::removePlan(std::uint32_t id, control::PlanId plan) {
    Entry* e = entry(id);
    return e ? e->host.removePlan(plan) : control::Reason::UnknownVehicle;
}

std::optional<control::PlanStatus> World::planStatus(std::uint32_t id, control::PlanId plan) const {
    const Entry* e = entry(id);
    control::PlanStatus s;
    if (!e || !e->host.planStatus(plan, s)) return std::nullopt;
    return s;
}

std::vector<control::PlanStatus> World::plans(std::uint32_t id) const {
    const Entry* e = entry(id);
    return e ? e->host.plans() : std::vector<control::PlanStatus>{};
}

std::optional<control::RoutePlan> World::plan(std::uint32_t id, control::PlanId plan) const {
    const Entry* e = entry(id);
    control::RoutePlan p;
    if (!e || !e->host.plan(plan, p)) return std::nullopt;
    return p;
}

control::Reason World::loadPlan(std::uint32_t id, const control::RoutePlan& plan) {
    Entry* e = entry(id);
    return e ? e->host.loadPlan(plan) : control::Reason::UnknownVehicle;
}

control::Reason World::loadAirfield(std::uint32_t id, const control::Airfield& airfield) {
    Entry* e = entry(id);
    return e ? e->host.loadAirfield(airfield) : control::Reason::UnknownVehicle;
}

std::vector<control::Airfield> World::airfields(std::uint32_t id) const {
    const Entry* e = entry(id);
    return e ? e->host.airfields() : std::vector<control::Airfield>{};
}

std::optional<control::Airfield> World::airfield(std::uint32_t id, control::AirfieldId airfield) const {
    const Entry* e = entry(id);
    control::Airfield a;
    if (!e || !e->host.airfield(airfield, a)) return std::nullopt;
    return a;
}

control::PlanValidationResult World::validatePlan(std::uint32_t id, const control::RoutePlan& plan, const control::PlanValidation& v) {
    Entry* e = entry(id);
    if (!e) {
        control::PlanValidationResult r;
        r.check.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return e->host.validatePlan(plan, v, pool_->states()[e->slot], simTime_);
}

control::PlanValidationResult World::validatePlan(std::uint32_t id, control::PlanId plan, const control::PlanValidation& v) {
    Entry* e = entry(id);
    if (!e) {
        control::PlanValidationResult r;
        r.check.reason = control::Reason::UnknownVehicle;
        return r;
    }
    return e->host.validatePlan(plan, v, pool_->states()[e->slot], simTime_);
}

bool World::activitySetpoint(control::ActivityId activity, control::Setpoint& out) const {
    const Entry* e = entry(control::activityVehicle(activity));
    return e && e->host.setpoint(activity, out);
}

bool World::activityArrival(control::ActivityId activity, control::ArrivalEstimate& out) const {
    const Entry* e = entry(control::activityVehicle(activity));
    return e && e->host.arrival(activity, out);
}

std::vector<control::EndPoint> World::endPoints(control::ActivityId activity, std::size_t max) const {
    const Entry* e = entry(control::activityVehicle(activity));
    return e ? e->host.endPoints(activity, max) : std::vector<control::EndPoint>{};
}

control::VehicleCommandState World::commandState(std::uint32_t id) const {
    const Entry* e = entry(id);
    return e ? e->host.commandState(pool_->states()[e->slot]) : control::VehicleCommandState{};
}

const control::ActivityRecord* World::activity(control::ActivityId activity) const noexcept {
    const Entry* e = entry(control::activityVehicle(activity));
    return e ? e->host.activity(activity) : nullptr;
}

std::vector<control::ActivityRecord> World::activities(std::uint32_t id) const {
    const Entry* e = entry(id);
    return e ? e->host.activities() : std::vector<control::ActivityRecord>{};
}

const std::vector<control::CapabilityDescriptor>& World::capabilities(std::uint32_t id) {
    static const std::vector<control::CapabilityDescriptor> none;
    Entry* e = entry(id);
    if (!e) return none;
    e->catalog->refresh();
    return e->catalog->descriptors();
}

control::CapabilityStatus World::capabilityStatus(std::uint32_t id, std::string_view capability) const {
    const Entry* e = entry(id);
    const int index = e ? e->catalog->find(capability) : -1;
    if (e && index >= 0) return e->host.status(static_cast<std::size_t>(index), pool_->states()[e->slot]);
    // what it does not offer: why (never Disabled, which means switched off)
    control::CapabilityStatus out;
    out.availability = control::Availability::Unavailable;
    out.reason = e ? e->support->refusal(capability) : control::Reason::UnknownVehicle;
    out.reasons = control::reasonBit(out.reason);
    out.description = control::reasonDescription(out.reason);
    return out;
}

const control::SupportInfo* World::support(std::uint32_t id, std::string_view feature) const noexcept {
    const Entry* e = entry(id);
    return e ? e->support->find(feature) : nullptr;
}

const control::SupportTable* World::supportTable(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->support.get() : nullptr;
}

const control::VehicleProfile* World::profile(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->profile.get() : nullptr;
}

void World::levelChanged(Entry& e) {
    const auto level = e.stack.activeLevel();
    if (publisher_) publisher_->setControlLevel(static_cast<std::uint32_t>(e.slot), static_cast<std::uint8_t>(level));
    if (level != e.level) {
        e.level = level;
        recordVehicle(e); // the level shows in replays too
    }
}

control::ControlStack* World::controls(std::uint32_t id) noexcept {
    Entry* e = entry(id);
    return e ? &e->stack : nullptr;
}

const control::ControlStack* World::controls(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->stack : nullptr;
}

const control::Performance* World::performance(std::uint32_t id) noexcept {
    Entry* e = entry(id);
    if (!e) return nullptr;
    e->host.refreshPerformance();
    return &e->host.performance();
}

control::Reason World::setControlMode(std::uint32_t id, control::ControlMode mode) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    e->host.setControlMode(mode, pool_->states()[e->slot], simTime_);
    e->ms.setControlMode(mode);
    levelChanged(*e); // (what the policy flew without a grant has ended)
    return control::Reason::None;
}

control::ControlMode World::controlMode(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->host.controlMode() : control::ControlMode::Open;
}

control::Reason World::requestControl(std::uint32_t id, std::string_view capability, control::ControllerId controller) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    if (isMission(capability)) return missionControl(e->ms, capability, [&](std::size_t i) { return e->ms.requestControl(i, controller); });
    const int index = e->catalog->find(capability);
    if (index < 0) return e->support->refusal(capability);
    return e->host.requestControl(static_cast<std::size_t>(index), pool_->states()[e->slot], controller);
}

control::Reason World::releaseControl(std::uint32_t id, std::string_view capability, control::ControllerId controller) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    if (isMission(capability))
        return missionControl(e->ms, capability, [&](std::size_t i) { return e->ms.releaseControl(i, controller, simTime_); });
    const int index = e->catalog->find(capability);
    if (index < 0) return e->support->refusal(capability);
    const control::Reason r = e->host.releaseControl(static_cast<std::size_t>(index), pool_->states()[e->slot], simTime_, controller);
    levelChanged(*e);
    return r;
}

control::Reason World::revokeControl(std::uint32_t id, std::string_view capability, control::Reason reason) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    if (isMission(capability)) return missionControl(e->ms, capability, [&](std::size_t i) { return e->ms.revokeControl(i, simTime_); });
    const int index = e->catalog->find(capability);
    if (index < 0) return e->support->refusal(capability);
    const control::Reason r = e->host.revokeControl(static_cast<std::size_t>(index), reason, pool_->states()[e->slot], simTime_);
    levelChanged(*e);
    return r;
}

control::Reason World::setAllowed(std::uint32_t id, std::string_view capability, bool allowed) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    if (isMission(capability)) return missionControl(e->ms, capability, [&](std::size_t i) { return e->ms.setAllowed(i, allowed, simTime_); });
    const int index = e->catalog->find(capability);
    if (index < 0) return e->support->refusal(capability);
    const control::Reason r = e->host.setAllowed(static_cast<std::size_t>(index), allowed, pool_->states()[e->slot], simTime_);
    levelChanged(*e);
    return r;
}

control::Reason World::setCapabilityPrecedence(std::uint32_t id, std::string_view capability, std::uint32_t precedence) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    const int index = e->catalog->find(capability);
    if (index < 0) return e->support->refusal(capability);
    const control::Reason r = e->host.setPrecedence(static_cast<std::size_t>(index), precedence, pool_->states()[e->slot], simTime_);
    levelChanged(*e); // (what waited may have started)
    return r;
}

std::uint32_t World::capabilityPrecedence(std::uint32_t id, std::string_view capability) const {
    const Entry* e = entry(id);
    const int index = e ? e->catalog->find(capability) : -1;
    return index < 0 ? 0 : e->host.precedence(static_cast<std::size_t>(index));
}

control::ControlStatus World::controlStatus(std::uint32_t id, std::string_view capability) const {
    const Entry* e = entry(id);
    if (e && isMission(capability)) {
        const int index = e->ms.find(capability);
        return index < 0 ? control::ControlStatus{false, false} : e->ms.controlStatus(static_cast<std::size_t>(index));
    }
    const int index = e ? e->catalog->find(capability) : -1;
    return index < 0 ? control::ControlStatus{false, false} : e->host.controlStatus(static_cast<std::size_t>(index));
}

control::Reason World::setAvailability(std::uint32_t id, std::string_view capability, control::Availability availability, control::Reason reason,
                                       std::uint64_t associated, double nextAvailableS) {
    Entry* e = entry(id);
    if (!e) return control::Reason::UnknownVehicle;
    if (isMission(capability)) return missionControl(e->ms, capability, [&](std::size_t i) { return e->ms.setAvailability(i, availability); });
    const int index = e->catalog->find(capability);
    if (index < 0) return e->support->refusal(capability);
    return e->host.setAvailability(static_cast<std::size_t>(index), availability, reason, associated, nextAvailableS);
}

std::uint32_t World::controlRevision(std::uint32_t id) noexcept {
    Entry* e = entry(id);
    if (!e) return 0;
    e->host.refreshPerformance();
    return e->host.controlRevision();
}

bool World::addEffect(std::uint32_t id, std::unique_ptr<effects::Effect> effect) {
    Entry* e = entry(id);
    if (!e || !effect) return false;
    e->effects.push_back(std::move(effect));
    return true;
}

void World::addEffectToAll(EffectFactory factory) {
    if (!factory) return;
    for (auto& e : entries_)
        if (e) e->effects.push_back(factory());
    worldEffects_.push_back(std::move(factory));
}

void World::clearEffects(std::uint32_t id) {
    if (Entry* e = entry(id)) e->effects.clear();
}

void World::setEnvironment(const sim::EnvironmentState& environment) {
    environment_ = environment;
    environment_.revision = appliedEnvironment_ == ~0ull ? 1 : appliedEnvironment_ + 1;
    if (publisher_) publisher_->setEnvironment(environment_);
    for (auto& e : entries_) // (the air every altimeter reads, and a barometric altitude is flown in, from the next step: 4.20)
        if (e) {
            control::Altimeter a = e->host.altimeter();
            a.air = air();
            e->host.setAltimeter(a);
        }
}

void World::applyEnvironment(sim::FlightModel& model) const {
    const auto& env = environment_;
    // Meteorological convention: the direction the wind blows FROM.
    const double dir = units::degreesToRadians(env.windDirectionDeg);
    model.setWindNed(-env.windSpeedMs * std::cos(dir), -env.windSpeedMs * std::sin(dir), 0.0);
    model.setTurbulence(env.turbulence, std::max(env.windSpeedMs, 1.0));
    model.setAtmosphere(env.temperatureSeaLevelK, env.pressureSeaLevelPa);
}

void World::preStep(std::size_t slot, int subStep, sim::FlightModel& model, sim::ControlInputs& inputs) {
    Entry* e = entries_[slot].get();
    if (!e) return;
    const bool needState = !e->effects.empty() || e->stack.activeLevel() != control::Level::Actuator;
    if (needState) model.state(e->working);
    else e->working = pool_->states()[slot];
    const double t = simTime_ + subStep * options_.dt;

    // Effects: sensed state starts as truth, wind/force contributions accumulate.
    e->sensed.state = e->working;
    e->sensed.gnssValid = e->sensed.airDataValid = true;
    e->sensed.positionErrorM = 0.0;
    // the force channel's push this sub-step: an effect's, and its damage's (MS-7g) - set once
    double push[3] = {0, 0, 0}, turn[3] = {0, 0, 0};
    bool pushed = false;
    if (!e->effects.empty()) {
        effects::EffectContext ctx(e->info.id, e->working, e->sensed, model, environment_, e->rng, options_.dt, t);
        for (auto& fx : e->effects)
            if (fx->enabled) fx->apply(ctx);
        if (ctx.windTouched()) {
            const double dir = units::degreesToRadians(environment_.windDirectionDeg);
            const double* w = ctx.windNed();
            if (ctx.windOverride()) model.setWindNed(w[0], w[1], w[2]);
            else model.setWindNed(w[0] - environment_.windSpeedMs * std::cos(dir), w[1] - environment_.windSpeedMs * std::sin(dir), w[2]);
            e->windApplied = true;
        } else if (e->windApplied) {
            applyEnvironment(model);
            e->windApplied = false;
        }
        if (ctx.forceTouched()) {
            for (int k = 0; k < 3; ++k) push[k] += ctx.force()[k], turn[k] += ctx.moment()[k];
            pushed = true;
        }
    }
    if (e->damage) { // (what its damage takes off the aerodynamic force)
        ms::damage::Flight f;
        damageFlight(*e, f);
        ms::damage::V3 F, M;
        e->damage->aeroLoss(f, F, M);
        // (its moment is about the model's origin; the force acts at the CG: about the CG, less the offset's moment)
        double cg[3];
        model.cgFromOriginBodyM(cg);
        M = M - ms::damage::cross(ms::damage::V3{cg[0], cg[1], cg[2]}, F);
        push[0] += F.x, push[1] += F.y, push[2] += F.z;
        turn[0] += M.x, turn[1] += M.y, turn[2] += M.z;
        pushed = true;
    }
    if (options_.storeDrag && !e->drag.empty() && e->dragQbar.valid()) {
        // its stores' and their carriage's drag (MS-7g): each against its airflow, at its station - its moment about the
        // CG (a store below it pitches the nose down; one wing's alone yaws it)
        constexpr double kPsfToPa = 47.880258888889;
        const sim::VehicleState& s = e->working;
        const double q0 = e->dragQbar.get() * kPsfToPa;
        const double q = q0 * ms::Host::storeDragMachFactor(s.mach);
        if (q > 0.0) {
            const double ca = std::cos(s.alphaRad), cb = std::cos(s.betaRad);
            const double wind[3] = {ca * cb, std::sin(s.betaRad), std::sin(s.alphaRad) * cb}; // (the air's way past it, body axes)
            double cg[3];
            model.cgFromOriginBodyM(cg);
            for (const auto& p : e->drag) {
                const double d = q * p.areaM2;
                const double F[3] = {-d * wind[0], -d * wind[1], -d * wind[2]};
                const double r[3] = {p.positionM[0] - cg[0], p.positionM[1] - cg[1], p.positionM[2] - cg[2]};
                for (int k = 0; k < 3; ++k) push[k] += F[k];
                turn[0] += r[1] * F[2] - r[2] * F[1];
                turn[1] += r[2] * F[0] - r[0] * F[2];
                turn[2] += r[0] * F[1] - r[1] * F[0];
                // in a sideslip: its stores' bodies turn the nose away from the wind (Munk's couple), their fins and
                // bases push across it, behind the station
                const double Y = -q0 * p.sideSlopeM2 * wind[1];
                push[1] += Y;
                turn[0] += -r[2] * Y;
                turn[2] += (r[0] - p.sideArmM) * Y - q0 * p.munkM3 * 2.0 * wind[1] * cb;
            }
            pushed = true;
        }
    }
    if (pushed) {
        model.setExternalForceBody(push, turn);
        e->forceApplied = true;
    } else if (e->forceApplied) {
        const double zero[3] = {0, 0, 0};
        model.setExternalForceBody(zero, zero);
        e->forceApplied = false;
    }

    // Control cascade at the FDM rate (or every `controlDivider` steps); a lost crew's controls held where they were.
    if (static_cast<unsigned>(subStep) % e->controlDivider == 0 && !e->crewLost) {
        control::ControlContext ctx{e->info.id, e->working, e->sensed.state, options_.dt * e->controlDivider, &stepView_, &e->rng};
        e->stack.update(ctx, e->inputs);
        // the effectors ControlInputs has no room for, when their demand changes
        const auto& fx = e->stack.effectors();
        auto differs = [](double a, double b) { return !(a == b || (std::isnan(a) && std::isnan(b))); };
        if (differs(fx.speedbrake, e->effectors.speedbrake) || differs(fx.pitchTrim, e->effectors.pitchTrim)) {
            model.setEffectors(fx);
            e->effectors = fx;
        }
    }
    inputs = e->inputs;
}

// --- Mission Systems (ADR-30) -------------------------------------------------

const ms::Host* World::missionSystems(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? &e->ms : nullptr;
}

ms::CapabilityStatus World::missionCapabilityStatus(std::uint32_t id, std::string_view capability) const {
    ms::CapabilityStatus out;
    const Entry* e = entry(id);
    if (!e) {
        out.availability = control::Availability::Unavailable, out.reason = ms::Reason::UnknownVehicle;
        return out;
    }
    const int index = e->ms.find(capability);
    if (index < 0) {
        out.availability = control::Availability::Unavailable, out.reason = ms::Host::refusal(capability);
        return out;
    }
    return e->ms.status(static_cast<std::size_t>(index));
}

ms::CommandResult World::submit(std::uint32_t id, const ms::StateCommand& command, const ms::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

ms::CommandResult World::submit(std::uint32_t id, const ms::BitCommand& command, const ms::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

ms::CommandResult World::submit(std::uint32_t id, const ms::AmtiCommand& command, const ms::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

std::vector<ms::StoreInfo> World::stores(std::uint32_t id) const {
    const Entry* e = entry(id);
    return e ? e->ms.stores() : std::vector<ms::StoreInfo>{};
}

double World::weightRoomKg(Entry& e) {
    const double most = e.profile ? e.profile->envelope.maxTakeoffKg : std::numeric_limits<double>::quiet_NaN();
    const double weighed = pool_->vehicle(e.slot).energy().massKg; // (as of its last step: loads since are not in it)
    if (!std::isfinite(most) || !std::isfinite(weighed)) return std::numeric_limits<double>::infinity();
    const double carried = e.ms.storesMassKg();
    if (e.storesStamp != loadTick_) e.storesKgAtStep = carried, e.storesStamp = loadTick_; // (the first load since that step)
    return most - (weighed - e.storesKgAtStep + carried);
}

ms::Reason World::loadStore(std::uint32_t id, std::uint16_t payload, int station, const std::string& store, int count) {
    Entry* e = entry(id);
    if (!e) return ms::Reason::UnknownVehicle;
    const ms::Reason r = e->ms.load(payload, station, store, count, weightRoomKg(*e));
    applyStoreMasses(); // (the flight model's masses at once: a load is between steps)
    return r;
}

ms::Reason World::loadLoadout(std::uint32_t id, const std::string& loadout) {
    Entry* e = entry(id);
    if (!e) return ms::Reason::UnknownVehicle;
    const ms::Reason r = e->ms.loadLoadout(loadout, weightRoomKg(*e));
    applyStoreMasses();
    return r;
}

ms::CommandResult World::submit(std::uint32_t id, const ms::StrikeCommand& command, const ms::CommandOptions& options) {
    if (options.controller > kAgentControllers) [[unlikely]] // (ADR-31: an agent's strike names no entity, and asks for consent)
        for (const AgentRecord& a : agents_)
            if (a.controller == options.controller &&
                (a.vehicle != id || agentStrike(AgentIdentity{a.vehicle, a.controller, {}}, command, this) != ms::Reason::None))
                return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::NotAllowed, 0, options.commandId};
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

ms::CommandResult World::submit(std::uint32_t id, const ms::JettisonCommand& command, const ms::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

double World::massKg(std::uint32_t id) const {
    const Entry* e = entry(id);
    return e ? pool_->vehicle(e->slot).energy().massKg : std::numeric_limits<double>::quiet_NaN();
}

bool World::cgFromOriginBodyM(std::uint32_t id, double out[3]) const {
    const Entry* e = entry(id);
    if (!e) return false;
    pool_->vehicle(e->slot).cgFromOriginBodyM(out);
    return true;
}

ms::Reason World::strikeConsent(ms::ActivityId activity, bool granted) {
    Entry* e = entry(static_cast<std::uint32_t>(activity >> 32));
    return e ? e->ms.strikeConsent(activity, granted, simTime_) : ms::Reason::UnknownActivity;
}

std::optional<ms::StrikeStatus> World::strikeStatus(ms::ActivityId activity) const {
    const Entry* e = entry(static_cast<std::uint32_t>(activity >> 32));
    return e ? e->ms.strikeStatus(activity) : std::nullopt;
}

void World::transferFuel() {
    // an aircraft's external tanks topping its own as it burns (their fuel the first it uses): as much as its tanks
    // have room for, at kTransferKgS at most - the mass moving from its stations into its tanks, none made or lost
    constexpr double kTransferKgS = 10.0; // (an estimate: above a fighter's fuel flow at full reheat, some 7 kg/s)
    for (auto& e : entries_) {
        if (!e || e->stationMass.empty() || !(e->ms.externalFuelKg() > 0.0)) continue;
        auto& model = pool_->vehicle(e->slot);
        const sim::EnergyOnBoard energy = model.energy();
        const double room = energy.fuelCapacityKg - energy.fuelKg;
        if (!(room > 0.01)) continue;
        const double drawn = e->ms.drawExternalFuel(std::min(room, kTransferKgS * options_.dt));
        const double put = model.refuel(drawn);
        (void)put;
    }
}

void World::applyStoreMasses() {
    for (auto& e : entries_)
        if (e) applyStoreMasses(*e);
}

void World::applyStoreMasses(Entry& e) {
    constexpr double kLbPerKg = 2.2046226218487757;
    if (e.dragRevision != e.ms.storeRevision()) { // (where its stores and carriage add drag: stepEntry pushes it)
        e.ms.carriageDrag(e.drag);
        e.dragRevision = e.ms.storeRevision();
        if (!e.drag.empty() && !e.dragQbar.valid()) e.dragQbar = pool_->vehicle(e.slot).property("aero/qbar-psf");
    }
    if (e.stationMass.empty() || e.massRevision == e.ms.massRevision()) return;
    for (auto& m : e.stationMass) m.weightLbs.set(e.ms.stationMassKg(m.payload, m.station) * kLbPerKg);
    e.massRevision = e.ms.massRevision();
}

void World::flyReleases() {
    bool any = false; // (most steps release nothing: nothing allocated then)
    for (const auto& e : entries_) any = any || (e && e->ms.releasing());
    if (!any) return;
    // (launching makes entities: the launchers' ids first, each looked up again after)
    std::vector<std::uint32_t> launchers;
    for (const auto& e : entries_)
        if (e) launchers.push_back(e->info.id);
    for (const std::uint32_t launcher : launchers) {
        Entry* e = entry(launcher);
        if (!e) continue;
        auto releases = e->ms.takeReleases();
        for (auto& r : releases) {
            // the station's place is from the carrier's model origin (its empty CG); the state is at its CG now
            double cg[3];
            pool_->vehicle(e->slot).cgFromOriginBodyM(cg);
            for (int k = 0; k < 3; ++k) r.launch.offsetBodyM[k] -= cg[k];
            const std::uint32_t weapon = launchWeapon(launcher, r.launch);
            if (Entry* w = weapon ? entry(weapon) : nullptr) {
                const auto& info = entry(launcher)->ms.stores();
                w->origin.store = r.store < info.size() ? info[r.store].store : "";
                w->origin.station = r.store < info.size() ? info[r.store].station : 0;
                w->origin.activity = r.activity;
                w->origin.target = r.target;
                w->origin.jettisoned = r.jettisoned;
                // a tank leaves with the fuel left in it (its model's "fuel" point mass; its own CG moved to it)
                const double fuel = entry(launcher)->ms.storeFuelKg(r.store);
                if (fuel > 0.0) {
                    auto& wm = pool_->vehicle(w->slot);
                    wm.beginMassChange();
                    wm.property("inertia/pointmass-weight-lbs").set(fuel * 2.2046226218487757);
                    wm.endMassChange(nullptr, nullptr);
                }
            }
            if (Entry* l = entry(launcher)) {
                // off the carrier at once: its station's mass, its CG and state moved to what is left, and the
                // ejector's push the other way on it (a rail's missile leaves under its own power: none)
                const auto& info = l->ms.stores();
                const double kg = weapon && r.store < info.size() ? info[r.store].massKg : 0.0;
                auto& model = pool_->vehicle(l->slot);
                model.beginMassChange();
                l->ms.released(r.store, weapon, simTime_);
                applyStoreMasses(*l);
                const double reaction[3] = {-kg * r.launch.ejectionBodyMs[0], -kg * r.launch.ejectionBodyMs[1], -kg * r.launch.ejectionBodyMs[2]};
                model.endMassChange(nullptr, reaction);
            }
        }
    }
}

ms::RadarStatus World::radarStatus(std::uint32_t id, std::uint16_t payload) const noexcept {
    const Entry* e = entry(id);
    return e ? e->ms.radarStatus(payload) : ms::RadarStatus{};
}

std::vector<ms::AmtiDetection> World::amtiDetections(std::uint32_t id, std::uint16_t payload, std::uint64_t since) const {
    std::vector<ms::AmtiDetection> out;
    const Entry* e = entry(id);
    const ms::radar::Radar* r = e ? e->ms.radar(payload) : nullptr;
    if (!r) return out;
    const auto& kept = r->detections();
    // (serials are consecutive: the first wanted is at its distance from the oldest kept)
    std::size_t first = 0;
    if (!kept.empty() && since > kept.front().serial) first = static_cast<std::size_t>(std::min<std::uint64_t>(since - kept.front().serial, kept.size()));
    out.reserve(kept.size() - first);
    for (std::size_t i = first; i < kept.size(); ++i) {
        const ms::radar::Detection& d = kept[i];
        ms::AmtiDetection a;
        a.serial = d.serial;
        a.payload = payload;
        a.timeS = d.timeS;
        a.truth = d.truth;
        a.rangeM = d.rangeM;
        a.azimuthRad = d.azimuthRad;
        a.elevationRad = d.elevationRad;
        a.rangeRateMs = d.rangeRateMs;
        a.snrDb = d.snrDb;
        a.latitudeRad = d.latitudeRad;
        a.longitudeRad = d.longitudeRad;
        a.heightM = d.heightM;
        out.push_back(a);
    }
    return out;
}

std::vector<ms::AmtiTrack> World::amtiTracks(std::uint32_t id, std::uint16_t payload) const {
    const Entry* e = entry(id);
    if (!e) return {};
    return e->ms.amtiTracks(payload, simTime_, pool_->states()[e->slot]);
}

void World::sense() {
    // the world each radar looks into: the other entities as the step left them
    struct Scene final : ms::radar::Scene {
        const World& world;
        std::uint32_t own = 0;
        Span<const sim::VehicleState> states;
        Scene(const World& w, Span<const sim::VehicleState> s) : world(w), states(s) {}
        std::size_t targets(std::uint32_t* ids, const sim::VehicleState** out, std::size_t max) const override {
            std::size_t n = 0;
            for (std::size_t s = 0; s < world.entries_.size() && n < max; ++s)
                if (const Entry* e = world.entries_[s].get(); e && e->info.id != own) ids[n] = e->info.id, out[n++] = &states[s];
            return n;
        }
        double sigma(std::uint32_t target, double frequencyHz, double polarizationRad, double& fluctuation) const override {
            ms::RcsQuery q;
            q.frequencyHz = frequencyHz;
            q.transmitPolarizationRad = polarizationRad;
            const auto rcs = world.radarCrossSection(own, target, q);
            if (!rcs || !rcs->modelled) return std::numeric_limits<double>::quiet_NaN();
            fluctuation = rcs->fluctuation;
            return rcs->sigmaM2;
        }
        double pathLossDb(std::uint32_t target, double frequencyHz) const override {
            const Entry* a = world.entry(own);
            const Entry* b = world.entry(target);
            const sim::VehicleState& sa = states[a->slot];
            const sim::VehicleState& sb = states[b->slot];
            const double d[3] = {sb.positionEcef[0] - sa.positionEcef[0], sb.positionEcef[1] - sa.positionEcef[1], sb.positionEcef[2] - sa.positionEcef[2]};
            const double range = std::max(1.0, std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
            const double freeSpace = 20.0 * std::log10(4.0 * 3.14159265358979323846 * range * frequencyHz / 299792458.0);
            const bool forward = own < target;
            const PathKey key{forward ? own : target, forward ? target : own, frequencyHz};
            {
                std::lock_guard<std::mutex> lock(world.pathExcessMutex_);
                const PathExcess& x = world.pathExcess_[key];
                if (world.simTime_ - x.timeS < 1.0) return freeSpace + x.excessDb;
            }
            // (worked out outside the lock: another radar of this step that finds it stale too works out the same - the
            // same two states, the same order - so whoever writes last, the answer is the one)
            ms::rf::PathOptions o;
            o.frequencyHz = frequencyHz;
            o.ground = world.ground_.get();
            const sim::VehicleState& lo = forward ? sa : sb;
            const sim::VehicleState& hi = forward ? sb : sa;
            const auto p = ms::rf::computePath({lo.latitudeRad, lo.longitudeRad, lo.altitudeMslM}, {hi.latitudeRad, hi.longitudeRad, hi.altitudeMslM}, o);
            const double excess = std::max(0.0, p.totalDb - p.freeSpaceDb);
            {
                std::lock_guard<std::mutex> lock(world.pathExcessMutex_);
                PathExcess& x = world.pathExcess_[key];
                x.timeS = world.simTime_, x.excessDb = excess;
            }
            return freeSpace + excess;
        }
        double surfaceHeightM(double latitudeRad, double longitudeRad) const override {
            // (the clutter's ground, out to the radar's horizon: read apart from the tiles the vehicles stand on - a
            // missing level's tile made from its ancestor for each point, and kept, cost 37 us a point and evicted theirs)
            return world.ground_ ? world.ground_->pointHeightAboveEllipsoidM(latitudeRad, longitudeRad) : std::numeric_limits<double>::quiet_NaN();
        }
    };
    const auto states = pool_->states();
    sensing_.clear();
    for (std::size_t s = 0; s < entries_.size(); ++s)
        if (Entry* e = entries_[s].get(); e && e->ms.sensing()) sensing_.push_back(s);
    if (sensing_.empty()) return;
    // each radar looks on its own (its stream, its detections), on the workers - a contiguous share of them each, in
    // order: the answers are the same for any number of workers
    pool_->forEach(sensing_.size(), [&](std::size_t k) {
        const std::size_t s = sensing_[k];
        Entry& e = *entries_[s];
        Scene scene(*this, states);
        scene.own = e.info.id;
        e.ms.sense(simTime_, states[s], scene);
    });
    if (pathExcess_.size() > 65536) pathExcess_.clear(); // (pairs that ended: worked out again where still wanted)
}

ms::CommandResult World::cancelMission(std::uint32_t id, control::Caller caller, ms::ActivityId activity) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, activity, 0};
    return e->ms.cancel(caller, activity, simTime_);
}

ms::Reason World::failPayload(std::uint32_t id, std::uint16_t payload, bool failed) {
    Entry* e = entry(id);
    if (!e) return ms::Reason::UnknownVehicle;
    return e->ms.setFailed(payload, failed, simTime_);
}

void World::step(unsigned n) {
    for (unsigned k = 0; k < n; ++k) {
        if (environment_.revision != appliedEnvironment_) {
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (entries_[s]) applyEnvironment(pool_->vehicle(s));
            appliedEnvironment_ = environment_.revision;
        }
        // behaviours that follow another vehicle read it as the step began (StepView); a designated track, its estimate
        if (!designations_.empty()) refreshDesignations();
        const auto before = pool_->states();
        std::copy(before.begin(), before.end(), stepStates_.begin());
        transferFuel();
        applyStoreMasses();
        pool_->step(Span<const sim::ControlInputs>(poolInputs_), options_.frameSkip);
        simTime_ += options_.dt * options_.frameSkip;
        {
            // the contract layer reads what the runtimes flew (docs/control-architecture.md, 6.4)
            const auto after = pool_->states();
            const bool weigh = (loadTick_++ & 7u) == 0;
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (Entry* e = entries_[s].get()) {
                    // its weight to its host, as its fuel burns (every eighth step) and once its stores have changed (their
                    // point masses were set before this step: the flight model's weight has them now) - 7.3
                    if (weigh || e->loadRevision != e->massRevision) {
                        const sim::EnergyOnBoard energy = pool_->vehicle(s).energy();
                        e->host.setLoad(energy.massKg, energy.fuelKg);
                        e->loadRevision = e->massRevision;
                    }
                    control::EffectorPositions positions;
                    if (e->host.awaitsPosition()) {
                        positions.gear = after[s].gearPosition;
                        if (e->flapsPosition.valid()) positions.flaps = e->flapsPosition.get();
                    }
                    if (e->host.afterStep(after[s], positions, simTime_)) levelChanged(*e); // (what waited started)
                    e->ms.step(simTime_);
                }
            sense();
            // a weapon that struck the ground has ended (its fuze and warhead: MS-7)
            std::vector<std::uint32_t> impacts;
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (Entry* e = entries_[s].get(); e && e->info.role == ms::Role::Weapon && after[s].crashed) {
                    sim::VehicleState at = after[s]; // (where it struck; its speed as the step began: the strike stopped it)
                    std::copy(std::begin(stepStates_[s].velocityNedMs), std::end(stepStates_[s].velocityNedMs), std::begin(at.velocityNedMs));
                    // (a store's strike is a strike: what it does - a tank's fuel left bursting into fire - is told
                    // before it ends there)
                    if (s < stepStates_.size() && !stepStates_[s].crashed) impacted(*e, stepStates_[s], after[s]);
                    endWeapon(*e, ms::WeaponEnd::Impact, at);
                    impacts.push_back(e->info.id);
                }
            for (const std::uint32_t id : impacts) removeVehicle(id);
            // an aircraft, or a piece of one, that struck the ground: what the strike does to it (MS-7g)
            for (std::size_t s = 0; s < entries_.size() && s < stepStates_.size(); ++s)
                if (Entry* e = entries_[s].get(); e && e->info.role != ms::Role::Weapon && after[s].crashed && !stepStates_[s].crashed)
                    impacted(*e, stepStates_[s], after[s]);
            flyReleases(); // (the stores the store management released this step)
            stepDamage();  // (the damaged: their leaks, fires and loads - MS-7g)
        }
        // Keep the tiles around every vehicle warm (background loaders) so the
        // workers seldom block on a download.
        if (terrain_ && worldSteps_ % 30 == 0) {
            const auto states = pool_->states();
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (entries_[s]) terrain_->requestAround(states[s].latitudeRad, states[s].longitudeRad);
        }
        vehicleSteps_ += liveCount_ * static_cast<std::uint64_t>(options_.frameSkip);
        ++worldSteps_;
        network_.step(simTime_, options_.dt * options_.frameSkip, this, rng_);
        if (dms_.active()) stepDms();
        recordFrame();
        if (publisher_) {
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (entries_[s]) poolInputs_[s] = entries_[s]->inputs;
            publisher_->addSteps(liveCount_ * static_cast<std::uint64_t>(options_.frameSkip), 1);
            if (publisher_->due()) { // (its vehicles' flight status worked out only as a snapshot is copied)
                fillStatus();
                if (publisher_->publish(simTime_, pool_->states(), Span<const sim::ControlInputs>(poolInputs_), false,
                                        Span<const ipc::FlightStatus>(poolStatus_)))
                    publishMission();
            }
        }
    }
}

void World::publishNow() {
    if (!publisher_) return;
    for (std::size_t s = 0; s < entries_.size(); ++s)
        if (entries_[s]) poolInputs_[s] = entries_[s]->inputs;
    fillStatus();
    publisher_->publish(simTime_, pool_->states(), Span<const sim::ControlInputs>(poolInputs_), true, Span<const ipc::FlightStatus>(poolStatus_));
    publishMission();
}

void World::fillStatus() {
    // what the viewer shows of each vehicle's Vehicle Interface (docs/vehicle-interface.md, 7.3): its weight, the least
    // airspeeds that leaves it, its speed floor, the bank its airspeed leaves a turn, its activities' flags
    if (poolStatus_.size() != entries_.size()) poolStatus_.resize(entries_.size());
    for (std::size_t s = 0; s < entries_.size(); ++s) {
        double* d = poolStatus_[s].derived;
        const Entry* e = entries_[s].get();
        if (!e) {
            std::fill(d, d + ipc::kDerivedCount, 0.0);
            continue;
        }
        const control::Performance& p = e->host.performance();
        const double mass = e->host.massKg();
        d[ipc::kDerivedMassKg] = std::isfinite(mass) ? mass : 0.0;
        d[ipc::kDerivedStallCasMs] = p.hovers ? std::numeric_limits<double>::quiet_NaN() : p.minCasMs;
        d[ipc::kDerivedLeastCasMs] = p.hovers ? std::numeric_limits<double>::quiet_NaN() : p.leastCasMs();
        d[ipc::kDerivedFloorCasMs] = e->stack.speedFloorCasMs();
        d[ipc::kDerivedBankCapRad] = e->stack.bankCapRad();
        d[ipc::kDerivedConstraints] = static_cast<double>(e->host.constraints());
        d[ipc::kDerivedLoadRatio] = p.loadSpeedRatio;
        // where its CG is from its model's origin: what the viewer hangs its picture from (its stores on their stations)
        const auto& model = pool_->vehicle(e->slot);
        model.cgFromOriginBodyM(d + ipc::kDerivedCgFromOriginM);
        // what each engine burns: what the viewer lays its contrails and its smoke from
        for (int k = 0; k < sim::VehicleState::kMaxEngines; ++k) d[ipc::kDerivedFuelFlowKgS + static_cast<std::size_t>(k)] = model.engineFuelFlowKgS(k);
    }
}

void World::publishMission() {
    for (std::size_t s = 0; s < entries_.size() && s < publisher_->capacity(); ++s) {
        Entry* e = entries_[s].get();
        if (!e) continue;
        const ms::Host& host = e->ms;
        // what the record would show: the payloads' states and progress, the radars' scans and detections
        std::uint64_t sig = 1469598103934665603ull;
        auto mix = [&](std::uint64_t v) { sig = (sig ^ v) * 1099511628211ull; };
        mix(e->info.id);
        bool progressing = false; // (a warm-up or a BIT: its progress shown)
        for (const auto& p : host.payloads()) {
            mix(static_cast<std::uint64_t>(p.state));
            progressing |= p.state == ms::PayloadState::Starting || p.state == ms::PayloadState::Bit;
        }
        if (progressing)
            for (const auto& a : host.activities())
                if (a.live() && std::isfinite(a.percent)) mix(static_cast<std::uint64_t>(a.percent));
        for (std::uint16_t i = 0; i < host.payloads().size(); ++i)
            if (const auto* r = host.radar(i)) {
                mix(r->dwells()), mix(r->detectionCount()), mix(r->active() ? 1u + static_cast<unsigned>(r->searching().mode) : 0u);
                mix(r->tracks().size()), mix(r->tentativeTracks());
            }
        mix(host.storeRevision()); // (its stores: loaded, released)
        mix(host.doorsRevision()); // (its bays' doors moving)
        for (const auto& st : host.stores()) mix(static_cast<std::uint64_t>(st.status));
        if (sig == e->missionSignature) continue;
        e->missionSignature = sig;
        if (!missionScratch_) missionScratch_ = std::make_unique<ipc::MissionRecord>(); // (some 9 KiB: off the stack, made once)
        ipc::MissionRecord& m = *missionScratch_;
        m.~MissionRecord(), new (&m) ipc::MissionRecord(); // (empty again: its SeqLock is not assignable)
        m.vehicleId = e->info.id;
        m.simTime = simTime_;
        const auto& caps = host.capabilities();
        for (const auto& p : host.payloads()) {
            if (m.payloadCount >= ipc::kMissionPayloads) break;
            ipc::MissionPayload& out = m.payloads[m.payloadCount++];
            copyName(out.name, sizeof out.name, p.name);
            copyName(out.designation, sizeof out.designation, p.designation);
            std::string list;
            for (const auto& c : caps) {
                if (c.payload != p.index || c.type == ms::CapabilityType::Status) continue;
                std::string id = ms::capabilityTypeId(c.type);
                id = id.substr(id.rfind('.') + 1);
                if (c.support != control::Support::Supported) id += " (MS-" + std::to_string(c.stage) + ")";
                list += (list.empty() ? "" : ", ") + id;
            }
            copyName(out.capabilities, sizeof out.capabilities, list);
            out.family = static_cast<std::uint8_t>(p.family);
            out.state = static_cast<std::uint8_t>(p.state);
            out.count = p.count;
            out.percent = std::numeric_limits<float>::quiet_NaN();
            for (const auto& a : host.activities())
                if (a.live() && a.payload == p.index && std::isfinite(a.percent)) out.percent = static_cast<float>(a.percent);
        }
        for (const auto& st : host.stores()) {
            if (m.storeCount >= ipc::kMissionStores) break;
            ipc::MissionStore& o = m.stores[m.storeCount++];
            o.station = static_cast<std::uint8_t>(st.station);
            o.status = static_cast<std::uint8_t>(st.status);
            o.bayDoors = static_cast<std::uint8_t>(std::lround(255.0 * std::clamp(st.bayDoors, 0.0, 1.0)));
            copyName(o.mnemonic, sizeof o.mnemonic, st.mnemonic);
            copyName(o.store, sizeof o.store, st.store);
        }
        for (std::uint16_t i = 0; i < host.payloads().size() && m.radarCount < ipc::kMissionRadars; ++i) {
            const auto* r = host.radar(i);
            if (!r) continue;
            const ms::RadarStatus st = host.radarStatus(i);
            ipc::MissionRadar& out = m.radars[m.radarCount++];
            out.payload = i;
            out.searching = st.searching ? 1 : 0, out.mode = static_cast<std::uint8_t>(st.mode);
            out.rotodome = st.rotodome ? 1 : 0, out.electronic = st.electronic ? 1 : 0, out.bars = static_cast<std::uint8_t>(st.bars);
            out.signalLevel = st.signalLevel ? 1 : 0;
            out.task = static_cast<std::uint8_t>(st.task), out.acquiring = st.acquiring ? 1 : 0, out.illuminating = st.illuminating ? 1 : 0;
            if (st.searching && st.signalLevel && r->snapshot().kept && m.rangeDoppler.radar == 0xFF)
                publishRangeDoppler(*r, static_cast<std::uint8_t>(m.radarCount - 1), m.rangeDoppler);
            out.beamAzimuthRad = static_cast<float>(st.beamAzimuthRad), out.beamElevationRad = static_cast<float>(st.beamElevationRad);
            out.framePeriodS = static_cast<float>(st.framePeriodS);
            out.azimuthCentreRad = static_cast<float>(st.azimuthCentreRad), out.azimuthHalfWidthRad = static_cast<float>(st.azimuthHalfWidthRad);
            out.elevationCentreRad = static_cast<float>(st.elevationCentreRad), out.rangeScaleM = static_cast<float>(st.rangeScaleM);
            out.azimuthLimitRad = static_cast<float>(st.azimuthLimitRad), out.elevationLimitRad = static_cast<float>(st.elevationLimitRad);
            out.instrumentedRangeM = static_cast<float>(st.instrumentedRangeM);
            out.beamwidthAzimuthRad = static_cast<float>(st.beamwidthAzimuthRad), out.beamwidthElevationRad = static_cast<float>(st.beamwidthElevationRad);
            out.frequencyHz = st.frequencyHz;
            out.dwells = st.dwells, out.detections = st.detections;
            const auto& kept = r->detections();
            const std::size_t first = kept.size() > ipc::kMissionDetections ? kept.size() - ipc::kMissionDetections : 0;
            for (std::size_t k = first; k < kept.size(); ++k) {
                const auto& d = kept[k];
                ipc::MissionDetection& o = out.recent[out.recentCount++];
                o.timeS = d.timeS;
                o.rangeM = static_cast<float>(d.rangeM), o.azimuthRad = static_cast<float>(d.azimuthRad);
                o.elevationRad = static_cast<float>(d.elevationRad), o.rangeRateMs = static_cast<float>(d.rangeRateMs);
                o.snrDb = static_cast<float>(d.snrDb), o.truth = d.truth;
            }
            // its tracks, from the carrier now
            out.falseAlarms = st.falseAlarms;
            out.tentativeTracks = st.tentativeTracks;
            const sim::VehicleState& own = pool_->states()[s];
            double rb[9];
            sim::attitude::bodyToEcefFromQuaternion(own.attitudeEcefToBody, rb);
            auto tracks = host.amtiTracks(i, simTime_, own);
            // (the track it follows first: shown whatever the display's count)
            const auto followed = std::find_if(tracks.begin(), tracks.end(), [&](const ms::AmtiTrack& t) { return st.designatedTrack && t.id == st.designatedTrack; });
            if (followed != tracks.end()) std::rotate(tracks.begin(), followed, followed + 1);
            for (const ms::AmtiTrack& t : tracks) {
                if (out.trackCount >= ipc::kMissionTracks) break;
                ipc::MissionTrack& o = out.tracks[out.trackCount++];
                o.id = static_cast<std::uint32_t>(t.id), o.updates = t.updates, o.truth = t.truth;
                o.flags = st.designatedTrack && t.id == st.designatedTrack ? ipc::kMissionTrackDesignated : 0u;
                o.rangeM = static_cast<float>(t.rangeM), o.azimuthRad = static_cast<float>(t.azimuthRad);
                o.elevationRad = static_cast<float>(t.elevationRad), o.rangeRateMs = static_cast<float>(t.rangeRateMs);
                const double v[3] = {t.stateEcef[1], t.stateEcef[3], t.stateEcef[5]};
                const double vb[2] = {rb[0] * v[0] + rb[3] * v[1] + rb[6] * v[2], rb[1] * v[0] + rb[4] * v[1] + rb[7] * v[2]};
                o.headingRad = static_cast<float>(std::atan2(vb[1], vb[0]));
                o.speedMs = static_cast<float>(std::hypot(t.velocityNedMs[0], t.velocityNedMs[1]));
                o.sigmaM = static_cast<float>(t.positionSigmaM);
                o.sinceUpdateS = static_cast<float>(simTime_ - t.lastUpdateS);
            }
        }
        publisher_->setMission(static_cast<std::uint32_t>(s), m);
    }
}

std::uint32_t World::launchWeapon(std::uint32_t launcher, const ms::WeaponLaunch& launch) {
    const Entry* from = entry(launcher);
    if (!from) {
        LOG_ERROR("session") << "launchWeapon: no vehicle " << launcher;
        return 0;
    }
    const sim::VehicleState& ls = pool_->states()[from->slot];
    // the launcher's body axes in NED (3-2-1: yaw, pitch, roll)
    const double cr = std::cos(ls.eulerRad[0]), sr = std::sin(ls.eulerRad[0]);
    const double cp = std::cos(ls.eulerRad[1]), sp = std::sin(ls.eulerRad[1]);
    const double cy = std::cos(ls.eulerRad[2]), sy = std::sin(ls.eulerRad[2]);
    const double r[3][3] = {{cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy},
                            {cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy},
                            {-sp, sr * cp, cr * cp}};
    auto toNed = [&](const double b[3], double n[3]) {
        for (int i = 0; i < 3; ++i) n[i] = r[i][0] * b[0] + r[i][1] * b[1] + r[i][2] * b[2];
    };
    double offset[3], push[3];
    toNed(launch.offsetBodyM, offset);
    toNed(launch.ejectionBodyMs, push);
    double lat, lon;
    geo::offsetLatLon(ls.latitudeRad, ls.longitudeRad, offset[0], offset[1], lat, lon);
    VehicleSpec spec;
    spec.type = launch.type;
    spec.initial.latitudeDeg = units::radiansToDegrees(lat);
    spec.initial.longitudeDeg = units::radiansToDegrees(lon);
    spec.initial.altitudeMslM = ls.altitudeMslM - offset[2];
    spec.initial.rollDeg = units::radiansToDegrees(ls.eulerRad[0]);
    spec.initial.pitchDeg = units::radiansToDegrees(ls.eulerRad[1]);
    spec.initial.headingDeg = units::radiansToDegrees(ls.eulerRad[2]);
    // as its point of the launcher moved (its velocity and its rotation's: omega x r), pushed clear, turning with it
    const double* w = ls.angularRateBodyRadS;
    const double* o = launch.offsetBodyM;
    const double spin[3] = {w[1] * o[2] - w[2] * o[1], w[2] * o[0] - w[0] * o[2], w[0] * o[1] - w[1] * o[0]};
    double spinNed[3];
    toNed(spin, spinNed);
    for (int i = 0; i < 3; ++i) spec.initial.velocityNedMs[i] = ls.velocityNedMs[i] + spinNed[i] + push[i];
    for (int i = 0; i < 3; ++i) spec.initial.bodyRatesRadS[i] = w[i];
    spec.initial.airspeedTrueMs = ls.airspeedTrueMs;
    spec.initial.enginesRunning = !launch.inert;
    std::string family, model;
    parseType(launch.type, family, model);
    spec.name = launch.name.empty() ? from->info.name + "-" + model + "-" + std::to_string(nextId_) : launch.name;
    const std::uint32_t id = createVehicle(spec);
    Entry* e = entry(id);
    if (!e) return 0;
    e->info.role = ms::Role::Weapon;
    e->info.launcher = launcher;
    e->info.launchTimeS = simTime_;
    // flown by its own model: no flight capability takes it (MS-7 gives it its seeker and guidance)
    for (std::size_t c = 0; c < e->catalog->size(); ++c)
        e->host.setAvailability(c, control::Availability::Unavailable, control::Reason::Restricted);
    e->inputs = sim::ControlInputs{};
    poolInputs_[e->slot] = e->inputs;
    LOG_INFO("session") << "weapon " << id << " '" << e->info.name << "' (" << e->info.type << ") released by vehicle " << launcher;
    return id;
}

void World::endWeapon(Entry& e, ms::WeaponEnd end, const sim::VehicleState& state) {
    if (e.weaponEnded) return;
    e.weaponEnded = true;
    ms::WeaponRecord w;
    w.id = e.info.id;
    w.launcher = e.info.launcher;
    w.name = e.info.name;
    w.type = e.info.type;
    w.launchTimeS = e.info.launchTimeS;
    w.store = e.origin.store, w.station = e.origin.station, w.activity = e.origin.activity;
    w.target = e.origin.target, w.jettisoned = e.origin.jettisoned;
    w.endTimeS = simTime_;
    w.end = end;
    w.endLatitudeRad = state.latitudeRad;
    w.endLongitudeRad = state.longitudeRad;
    w.endHeightM = state.altitudeMslM;
    w.endSpeedMs = std::sqrt(state.velocityNedMs[0] * state.velocityNedMs[0] + state.velocityNedMs[1] * state.velocityNedMs[1] +
                             state.velocityNedMs[2] * state.velocityNedMs[2]);
    endedWeapons_.push_front(std::move(w));
    if (endedWeapons_.size() > 256) endedWeapons_.pop_back();
    LOG_INFO("session") << "weapon " << e.info.id << " '" << e.info.name << "': " << ms::weaponEndName(end) << " at t = " << simTime_;
}

std::vector<ms::WeaponRecord> World::weapons() const {
    std::vector<ms::WeaponRecord> out;
    std::vector<const Entry*> live;
    for (const auto& e : entries_)
        if (e && e->info.role == ms::Role::Weapon) live.push_back(e.get());
    std::sort(live.begin(), live.end(), [](const Entry* a, const Entry* b) { return a->info.id < b->info.id; });
    for (const Entry* e : live) {
        ms::WeaponRecord w;
        w.id = e->info.id;
        w.launcher = e->info.launcher;
        w.name = e->info.name;
        w.type = e->info.type;
        w.launchTimeS = e->info.launchTimeS;
        w.store = e->origin.store, w.station = e->origin.station, w.activity = e->origin.activity;
        w.target = e->origin.target, w.jettisoned = e->origin.jettisoned;
        out.push_back(std::move(w));
    }
    out.insert(out.end(), endedWeapons_.begin(), endedWeapons_.end());
    return out;
}

std::optional<ms::RadioPath> World::radioPath(std::uint32_t from, std::uint32_t to, const ms::RadioPathOptions& options) const {
    const Entry* a = entry(from);
    const Entry* b = entry(to);
    if (!a || !b) return std::nullopt;
    const sim::VehicleState& sa = pool_->states()[a->slot];
    const sim::VehicleState& sb = pool_->states()[b->slot];
    ms::rf::PathOptions o;
    o.frequencyHz = options.frequencyHz;
    o.polarization = options.polarization;
    o.timePercent = options.timePercent;
    o.rainRateMmH = options.rainRateMmH;
    o.rainHeightM = options.rainHeightM;
    o.model = options.itm ? ms::rf::PathModel::Itm : ms::rf::PathModel::Auto;
    o.ground = ground_.get();
    const auto p = ms::rf::computePath({sa.latitudeRad, sa.longitudeRad, sa.altitudeMslM}, {sb.latitudeRad, sb.longitudeRad, sb.altitudeMslM}, o);
    ms::RadioPath r;
    r.code = p.code;
    r.viaItm = p.viaItm;
    r.mode = static_cast<int>(p.mode);
    r.groundRangeM = p.groundRangeM;
    r.slantRangeM = p.slantRangeM;
    for (int i = 0; i < 2; ++i) r.elevationRad[i] = p.elevationRad[i], r.azimuthRad[i] = p.azimuthRad[i];
    r.clearanceM = p.clearanceM;
    r.terrainMasked = p.terrainMasked;
    r.freeSpaceDb = p.freeSpaceDb;
    r.basicDb = p.basicDb;
    r.gasDb = p.gasDb;
    r.terrainDb = p.terrainDb;
    r.rainDb = p.rainDb;
    r.totalDb = p.totalDb;
    return r;
}

std::shared_ptr<const ms::rcs::Table> World::rcsTable(const std::string& aircraft) const {
    if (const auto it = rcsTables_.find(aircraft); it != rcsTables_.end()) return it->second;
    std::shared_ptr<const ms::rcs::Table> table;
    std::vector<std::filesystem::path> candidates;
    const io::AssetResolver assets;
    if (const auto dir = assets.findAircraft(aircraft, jsbsimRoot_); !dir.empty()) candidates.push_back(dir / aircraft / "rcs.bin");
    for (const auto& root : assets.aircraftDirs()) {
        candidates.push_back(root / aircraft / "rcs.bin");
        // a stock aircraft drawn by a design of ours (aircraft/models.txt): that design's table
        std::ifstream in(root / "models.txt");
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string stock, design;
            if (line.empty() || line[0] == '#' || !(fields >> stock >> design) || stock != aircraft) continue;
            candidates.push_back(root / design / "rcs.bin");
        }
    }
    if (const auto store = assets.find("models/" + aircraft + ".rcs.bin")) candidates.push_back(*store);
    for (const auto& file : candidates) {
        std::error_code ec;
        if (!std::filesystem::exists(file, ec)) continue;
        std::string error;
        table = ms::rcs::readTable(file, &error);
        if (!table) LOG_WARN("session") << "rcs: " << error;
        else break;
    }
    rcsTables_[aircraft] = table;
    return table;
}

std::optional<ms::RadarCrossSection> World::radarCrossSection(std::uint32_t observer, std::uint32_t target, const ms::RcsQuery& q) const {
    const Entry* o = entry(observer);
    const Entry* t = entry(target);
    if (!o || !t) return std::nullopt;
    const sim::VehicleState& so = pool_->states()[o->slot];
    const sim::VehicleState& st = pool_->states()[t->slot];
    // the line of sight from the target to the radar, and the radar's polarisation, in the target's body axes
    double los[3] = {so.positionEcef[0] - st.positionEcef[0], so.positionEcef[1] - st.positionEcef[1], so.positionEcef[2] - st.positionEcef[2]};
    const double range = std::sqrt(los[0] * los[0] + los[1] * los[1] + los[2] * los[2]);
    if (!(range > 0.0)) return std::nullopt;
    for (double& c : los) c /= range;
    const double cl = std::cos(st.latitudeRad);
    const double up[3] = {cl * std::cos(st.longitudeRad), cl * std::sin(st.longitudeRad), std::sin(st.latitudeRad)};
    double h[3] = {los[1] * up[2] - los[2] * up[1], los[2] * up[0] - los[0] * up[2], los[0] * up[1] - los[1] * up[0]};
    double hn = std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    if (hn < 1e-9) { // (straight up or down: any horizontal)
        const double east[3] = {-std::sin(st.longitudeRad), std::cos(st.longitudeRad), 0.0};
        std::copy(std::begin(east), std::end(east), std::begin(h)), hn = 1.0;
    }
    for (double& c : h) c /= hn;
    const double v[3] = {h[1] * los[2] - h[2] * los[1], h[2] * los[0] - h[0] * los[2], h[0] * los[1] - h[1] * los[0]};
    double r[9];
    sim::attitude::bodyToEcefFromQuaternion(st.attitudeEcefToBody, r); // ecef = r body: body = r^T ecef
    auto toBody = [&](const double e[3], double b[3]) {
        for (int i = 0; i < 3; ++i) b[i] = r[i] * e[0] + r[3 + i] * e[1] + r[6 + i] * e[2];
    };
    double d[3], vb[3], hb[3];
    toBody(los, d), toBody(v, vb), toBody(h, hb);
    ms::RadarCrossSection out;
    out.aspectAzimuthRad = std::atan2(d[1], d[0]);
    out.aspectElevationRad = std::atan2(-d[2], std::hypot(d[0], d[1]));
    const auto table = rcsTable(t->info.aircraft);
    if (!table) return out;
    // the polarisation angles in the table's theta-phi basis there
    const double ca = std::cos(out.aspectAzimuthRad), sa = std::sin(out.aspectAzimuthRad);
    const double ce = std::cos(out.aspectElevationRad), se = std::sin(out.aspectElevationRad);
    const double theta[3] = {se * ca, se * sa, ce}, phi[3] = {-sa, ca, 0.0};
    auto angle = [&](double psi) {
        double p[3];
        for (int i = 0; i < 3; ++i) p[i] = std::cos(psi) * vb[i] + std::sin(psi) * hb[i];
        return std::atan2(p[0] * phi[0] + p[1] * phi[1] + p[2] * phi[2], p[0] * theta[0] + p[1] * theta[1] + p[2] * theta[2]);
    };
    const double tx = angle(q.transmitPolarizationRad);
    const double rx = angle(std::isfinite(q.receivePolarizationRad) ? q.receivePolarizationRad : q.transmitPolarizationRad);
    out.modelled = true;
    out.sigmaM2 = table->sigma(d, q.frequencyHz, tx, rx);
    out.fluctuation = table->fluctuation(d, q.frequencyHz);
    return out;
}

void World::recordVehicle(const Entry& e) {
    if (!recorder_.isOpen()) return;
    ipc::RecordedVehicle r;
    r.id = e.info.id;
    r.generation = e.info.generation;
    r.alive = e.info.alive ? 1 : 0;
    r.controlLevel = static_cast<std::uint8_t>(e.stack.activeLevel());
    copyName(r.name, ipc::kNameLength, e.info.name);
    copyName(r.type, ipc::kTypeLength, e.info.type);
    copyName(r.model, ipc::kPathLength, e.info.model);
    r.initialLatitudeDeg = e.info.initial.latitudeDeg;
    r.initialLongitudeDeg = e.info.initial.longitudeDeg;
    r.initialAltitudeMslM = e.info.initial.altitudeMslM;
    r.initialHeadingDeg = e.info.initial.headingDeg;
    recorder_.writeVehicle(static_cast<std::uint32_t>(e.slot), r);
}

void World::recordFrame() {
    if (!recorder_.isOpen()) return;
    if (lastRecordedTime_ >= 0.0 && simTime_ - lastRecordedTime_ < options_.recordIntervalSeconds - 1e-9) return;
    lastRecordedTime_ = simTime_;
    const auto states = pool_->states();
    std::vector<std::pair<std::uint32_t, ipc::VehicleSample>> samples;
    samples.reserve(liveCount_);
    // (a replay shows of each vehicle what a live viewer is told: its flight status - its weight, its least airspeeds,
    // its floor, its flags - its model hung from its origin, what its engines burn)
    fillStatus();
    for (std::size_t s = 0; s < entries_.size(); ++s) {
        if (!entries_[s]) continue;
        ipc::VehicleSample v;
        v.state = states[s];
        v.inputs = entries_[s]->inputs;
        std::copy(poolStatus_[s].derived, poolStatus_[s].derived + ipc::kDerivedCount, v.derived);
        samples.emplace_back(static_cast<std::uint32_t>(s), v);
    }

    // the environment, where it changed since it was last recorded (a replay's air and weather)
    if (environment_.revision != recordedEnvironment_) {
        recorder_.writeEnvironment(environment_);
        recordedEnvironment_ = environment_.revision;
    }
    // the stores, where they changed since they were last recorded (in force from this snapshot)
    if (recordedStores_.size() < entries_.size()) recordedStores_.resize(entries_.size());
    for (std::size_t s = 0; s < entries_.size(); ++s) {
        std::vector<ipc::MissionStore> now;
        if (entries_[s])
            for (const auto& st : entries_[s]->ms.stores()) {
                if (now.size() >= ipc::kMissionStores) break;
                ipc::MissionStore& o = now.emplace_back();
                o.station = static_cast<std::uint8_t>(st.station);
                o.status = static_cast<std::uint8_t>(st.status);
                o.bayDoors = static_cast<std::uint8_t>(std::lround(255.0 * std::clamp(st.bayDoors, 0.0, 1.0)));
                copyName(o.mnemonic, sizeof o.mnemonic, st.mnemonic);
                copyName(o.store, sizeof o.store, st.store);
            }
        auto& was = recordedStores_[s];
        const bool same = was.size() == now.size() &&
                          (now.empty() || std::memcmp(was.data(), now.data(), now.size() * sizeof(ipc::MissionStore)) == 0);
        if (!same) {
            recorder_.writeStores(static_cast<std::uint32_t>(s), now);
            was = std::move(now);
        }
    }
    recorder_.writeSnapshot(simTime_, samples);
}

void World::publishVehicle(const Entry& e) {
    recordVehicle(e);
    if (!publisher_) return;
    ipc::VehicleRecord r;
    r.id = e.info.id;
    r.generation = e.info.generation;
    r.alive = e.info.alive ? 1 : 0;
    r.controlLevel = static_cast<std::uint8_t>(e.stack.activeLevel());
    copyName(r.name, ipc::kNameLength, e.info.name);
    copyName(r.type, ipc::kTypeLength, e.info.type);
    copyName(r.model, ipc::kPathLength, e.info.model);
    r.initialLatitudeDeg = e.info.initial.latitudeDeg;
    r.initialLongitudeDeg = e.info.initial.longitudeDeg;
    r.initialAltitudeMslM = e.info.initial.altitudeMslM;
    r.initialHeadingDeg = e.info.initial.headingDeg;
    publisher_->setVehicle(static_cast<std::uint32_t>(e.slot), r);
    publishNow();
}

} // namespace fsim::session
