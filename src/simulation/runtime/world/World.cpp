#include "simulation/runtime/world/World.h"
#include "simulation/runtime/world/WorldLiquidMaterials.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/weapon/OperatingPointControlV1.h"
#include "simulation/mission_systems/weapon/ControlObservationV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerLaunchNavigationV1.h"
#include "simulation/mission_systems/components/interfaces/GuidanceStateResetV1.h"

#include "simulation/runtime/services/AgentRules.h"

#include "foundation/core/Log.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include "foundation/core/Units.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/platform/Threads.h"
#include "simulation/flight_control/Laws.h"
#include "simulation/flight_control/RotorResponseV1.h"
#include "simulation/flight_control/Profile.h"
#include "fsim/ControllerRegistry.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/mission_systems/ea/Jammer.h"
#include "simulation/mission_systems/gun/Gun.h"
#include "simulation/mission_systems/radar/Radar.h"
#include "simulation/mission_systems/rcs/StoreRcs.h"
#include "simulation/mission_systems/rf/Path.h"
#include "simulation/mission_systems/weapon/LaunchEligibility.h"
#include "simulation/mission_systems/weapon/TrackKinematics.h"
#include "simulation/mission_systems/weapon/Autopilot.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/flight/Attitude.h"

#include <fstream>
#include <sstream>
#include "foundation/core/Geodesy.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>
#include <set>
#include <stdexcept>

namespace fsim::session {

namespace {

/// The stream a vehicle's sensors draw from (its radars' looks), apart from its flight model's (stream 0).
constexpr std::uint64_t kSensorStream = 0x5345'4E53ull;

void ecefToGeodeticWgs84(const double p[3], double& lat, double& lon, double& altM, double* upOut = nullptr) noexcept {
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    constexpr double b = a * (1.0 - f);
    constexpr double ep2 = (a * a - b * b) / (b * b);
    const double x = p[0], y = p[1], z = p[2];
    const double r = std::hypot(x, y);
    lon = std::atan2(y, x);
    if (r < 1e-6) {
        lat = z >= 0.0 ? 1.5707963267948966 : -1.5707963267948966;
        altM = std::abs(z) - b;
    } else {
        const double th = std::atan2(z * a, r * b);
        const double st = std::sin(th), ct = std::cos(th);
        lat = std::atan2(z + ep2 * b * st * st * st, r - e2 * a * ct * ct * ct);
        const double slat = std::sin(lat), clat = std::cos(lat);
        const double N = a / std::sqrt(1.0 - e2 * slat * slat);
        altM = (std::abs(clat) > 1e-6) ? (r / clat - N) : (std::abs(z) / std::abs(slat) - N * (1.0 - e2));
    }
    if (upOut) {
        const double cl = std::cos(lat);
        upOut[0] = cl * std::cos(lon);
        upOut[1] = cl * std::sin(lon);
        upOut[2] = std::sin(lat);
    }
}

std::optional<ms::weapon::Spec> legacyWeaponSpec(const ms::StoreCard& card,std::string& error) {
    ms::weapon::Spec spec;
    spec.seeker=card.seeker;
    const bool infrared=spec.seeker.find("infrared")!=std::string::npos;
    const bool semi=spec.seeker=="semi_active_radar";
    spec.acquireM=card.value("seeker_acquire_m",infrared ? 13000.0 : (semi ? 60000.0 : 17000.0));
    spec.gimbalRad=card.value("seeker_gimbal_deg",infrared ? 45.0 : 60.0)*units::kPi/180;
    spec.maxG=card.value("guidance_max_g",infrared ? 40.0 : 35.0);
    spec.lifeS=card.value("guidance_life_s",infrared ? 45.0 : 110.0);
    spec.loftRad=infrared ? 0 : 20*units::kPi/180;
    spec.fuzeM=card.value("fuze_radius_m",7+std::sqrt(card.value("warhead_kg",12)));
    const auto& capabilities=card.guidanceCapabilities;
    spec.capabilityVersion=capabilities.version;
    spec.inertialMidcourse=capabilities.inertialMidcourse; spec.launcherUpdates=capabilities.launcherUpdates;
    if (!ms::weapon::configureGuidance(card,spec,error)) return std::nullopt;
    if (card.parameters.count("warhead_model_version") && !ms::damage::warheadOf(card)) {
        error="explicit warhead model configuration is unavailable"; return std::nullopt;
    }
    return spec;
}

std::optional<ms::damage::V3> capturedCg(sim::FlightModel& model) {
    double cg[3];
    if (!model.cgFromOriginBodyM(cg) || !std::isfinite(cg[0]) || !std::isfinite(cg[1]) || !std::isfinite(cg[2]))
        return std::nullopt;
    return ms::damage::V3{cg[0],cg[1],cg[2]};
}

std::optional<ms::damage::V3> authoredContactNose(sim::FlightModel& model) {
    const auto version=model.property("fsim/model/nose-version");
    if (!version.valid() || version.get()!=1) return std::nullopt;
    const char* axes[]={"x","y","z"};
    double nose[3],origin[3];
    for (int k=0;k<3;++k) {
        const auto n=model.property(std::string("fsim/model/nose-")+axes[k]+"-in");
        const auto o=model.property(std::string("fsim/model/origin-")+axes[k]+"-in");
        if (!n.valid() || !o.valid() || !std::isfinite(n.get()) || !std::isfinite(o.get())) return std::nullopt;
        nose[k]=n.get(); origin[k]=o.get();
    }
    const ms::damage::V3 body{(origin[0]-nose[0])*.0254,(nose[1]-origin[1])*.0254,(origin[2]-nose[2])*.0254};
    if (!std::isfinite(body.x) || !std::isfinite(body.y) || !std::isfinite(body.z)) return std::nullopt;
    return body;
}

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

bool World::setPayloadProvidersV1(const ms::components::ProviderRegistryV1& providers,std::string& error) {
    if(nextId_!=1 || !storeCards_.empty()) {error="payload providers must be configured before the first native vehicle or store load";return false;}
    payloadProviders_=providers;error.clear();return true;
}

std::shared_ptr<const ms::components::ProfileV1> World::payloadProfile(const ms::StoreCard& card,std::string& error) const {
    return payloadBindings_.find(card,error);
}

bool World::bindPayloadProfile(std::shared_ptr<const ms::StoreCard> card,const std::filesystem::path& library,std::string& error) {
    error.clear();
    if (!card) {error="store card unavailable";return false;}
    const auto deny=[&] {payloadBindings_.refuse(card,library,error);return false;};
    if (card->id.empty() || card->id=="." || card->id==".." || card->id.find_first_of("/\\:")!=std::string::npos) {
        error="store identity is not a library filename";return deny();
    }
    if (library.empty()) {error="store component source was not retained with its loaded card";return deny();}
    ms::components::ProfileV1 resolved;
    if (!ms::components::readProfileV1(library/(card->id+".json"),*card,resolved,error)) return deny();
    std::error_code fileError; const auto origin=std::filesystem::weakly_canonical(library,fileError);
    if (fileError) {error="cannot canonicalize store component source";return deny();}
    auto profile=std::make_shared<const ms::components::ProfileV1>(std::move(resolved));
    return payloadBindings_.bind(std::move(card),std::move(profile),origin,error);
}

std::optional<ms::weapon::Spec> World::configuredWeaponSpec(const ms::StoreCard& card,std::string& error) const {
    auto configured=legacyWeaponSpec(card,error); if (!configured) return std::nullopt;
    const auto profile=payloadProfile(card,error); if (!profile) return std::nullopt;
    if (!profile->explicitProfile()) return configured;
    ms::weapon::Spec selected;
    if (!ms::weapon::configureCompositionV1(card,*profile,*configured,selected,error,payloadProviders_)) return std::nullopt;
    configured=std::move(selected);
    return configured;
}

ms::weapon::NativePlantIdentityV1 World::nativePlantIdentity(const Entry& e) {
    ms::weapon::NativePlantIdentityV1 identity;
    for (std::size_t k=0;k<identity.components.size();++k) {
        auto& row=identity.components[k]; const auto& reads=e.plantMetadata[k];
        if (reads[0].valid()) row.schemaVersion=reads[0].get();
        if (reads[1].valid()) row.implementationCode=reads[1].get();
        if (reads[2].valid()) row.fingerprint=reads[2].get();
    }
    return identity;
}

ms::weapon::NativePlantIdentityV1 World::nativePlantIdentity(sim::FlightModel& model,const ms::weapon::PlantAdapterV1& selected) {
    ms::weapon::NativePlantIdentityV1 identity;
    const char* kinds[]={"autopilot","allocator","actuator","propulsion","aerodynamics"};
    for (std::size_t k=0;k<5;++k) {
        if (!selected.required[k] || selected.availability[k]!=ms::components::AvailabilityV1::Available) continue;
        const auto base=std::string("fsim/components/")+kinds[k]+"/";
        const auto schema=model.property(base+"schema-version"),code=model.property(base+"implementation-code"),fingerprint=model.property(base+"profile-fingerprint");
        if (schema.valid()) identity.components[k].schemaVersion=schema.get();
        if (code.valid()) identity.components[k].implementationCode=code.get();
        if (fingerprint.valid()) identity.components[k].fingerprint=fingerprint.get();
    }
    return identity;
}

World::World(const WorldOptions& options)
    : options_(options), gunRng_(Rng::forVehicle(options.seed, 0x47554Eu, 0)), dmsRng_(Rng::forVehicle(options.seed, 0x444D53u, 0)), rng_(options.seed) {
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
    pool_->setNativeObserverFactoryV1([this](std::size_t slot,int subStep,sim::FlightModel& model) {
        auto* e=entries_[slot].get();
        return e?weaponNavigationObserverV2(*e,model,simTime_+subStep*options_.dt):sim::NativeStepObserverV1{};
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
        // Invalidate BEFORE either capture output is truncated, including
        // worlds without a publisher/diagnostic frame. A failed invalidation
        // leaves the previous files untouched and refuses this recording.
        std::string bindingError;
        const bool invalidated=ipc::invalidateWeaponDiagnosticCaptureBindingV1(options_.recordPath.string()+".wdiag",bindingError);
        ipc::RecordingHeader rh;
        rh.capacity = options_.capacity;
        rh.dt = options_.dt;
        rh.frameSkip = options_.frameSkip;
        copyName(rh.name, ipc::kNameLength, options_.name);
        if(invalidated)weaponDiagnosticCaptureBindingReadyV1_=recorder_.open(options_.recordPath, rh);
        else LOG_ERROR("session")<<"recording refused: cannot invalidate previous diagnostic capture: "<<bindingError;
    }
    debrisRng_ = Rng::forVehicle(options_.seed, 0x44454252ull, 0);
    LOG_INFO("session") << "world '" << options_.name << "': dt " << options_.dt << " s, frame skip " << options_.frameSkip << ", "
                        << workers << " worker(s)" << (published() ? ", published" : "");
}

World::~World() {
    stopStandbyPreloader();
    tacticalRecorder_.close();
    combatVfxRecorder_.close();
    debrisRecorder_.close();
    if(weaponLinkAccess_){std::lock_guard<std::mutex> lock(weaponLinkAccess_->mutex);weaponLinkAccess_->world=nullptr;}
    ipc::WeaponDiagnosticCaptureCompletionV1 completion;
    completion.recordingBytes=recorder_.bytesWritten();
    completion.recordingClosedSuccessfully=recorder_.finishV1();
    if(weaponDiagnosticsRecorderV1_) {
        const bool diagnosticsClosed=weaponDiagnosticsRecorderV1_->finishCaptureV1(completion);
        weaponDiagnosticsRecorderV1_.reset();
        if(weaponDiagnosticCaptureBindingReadyV1_&&diagnosticsClosed) {
            std::string error;
            if(!ipc::finalizeWeaponDiagnosticCaptureBindingV1(options_.recordPath,options_.recordPath.string()+".wdiag",completion,error))
                LOG_WARN("session")<<"diagnostic capture remains unavailable: "<<error;
        }
    }
}

void World::requestStandbyModels(const std::string& aircraft, std::size_t desiredReady) {
    if (!pool_ || pool_->workers() <= 1u || aircraft.empty() || desiredReady == 0u) return;
    const std::size_t cap = options_.publish ? 10u : 4u;
    desiredReady = std::min(desiredReady, cap);
    std::size_t toQueue = 0;
    {
        std::lock_guard<std::mutex> lock(standby_.mutex);
        if (standby_.stop) return;
        const std::size_t have = standby_.ready[aircraft].size() + standby_.queuedOrLoading[aircraft];
        if (have >= desiredReady) return;
        toQueue = desiredReady - have;
    }
    const sim::AircraftSpec files{aircraft, jsbsimRoot_, io::AssetResolver{}.findAircraft(aircraft, jsbsimRoot_)};
    bool needSpawn = false;
    {
        std::lock_guard<std::mutex> lock(standby_.mutex);
        if (standby_.stop) return;
        const std::size_t have = standby_.ready[aircraft].size() + standby_.queuedOrLoading[aircraft];
        if (have >= desiredReady) return;
        toQueue = desiredReady - have;
        for (std::size_t i = 0; i < toQueue; ++i) standby_.queue.push_back({aircraft, files});
        standby_.queuedOrLoading[aircraft] += toQueue;
        needSpawn = standby_.threads.empty();
        if (needSpawn) {
            const unsigned nThreads = options_.publish ? 2u : 1u;
            const double dt = options_.dt;
            const auto ground = ground_;
            standby_.threads.reserve(nThreads);
            for (unsigned t = 0; t < nThreads; ++t) {
                standby_.threads.emplace_back([this, dt, ground]() {
                    sim::InitialConditions dummyIc{};
                    dummyIc.latitudeDeg = 0.0;
                    dummyIc.longitudeDeg = 0.0;
                    dummyIc.altitudeMslM = 10000.0;
                    dummyIc.airspeedTrueMs = 300.0;
                    dummyIc.onGround = false;
                    dummyIc.enginesRunning = false;
                    for (;;) {
                        StandbyPreloadRequest req;
                        {
                            std::unique_lock<std::mutex> lk(standby_.mutex);
                            standby_.cv.wait(lk, [&] { return standby_.stop || !standby_.queue.empty(); });
                            if (standby_.stop) return;
                            req = std::move(standby_.queue.front());
                            standby_.queue.pop_front();
                        }
                        auto jsb = std::make_unique<sim::JsbsimModel>(dt, ground);
                        const bool ok = jsb->load(req.files, dummyIc);
                        std::lock_guard<std::mutex> lk(standby_.mutex);
                        auto& inFlight = standby_.queuedOrLoading[req.aircraft];
                        if (inFlight > 0u) --inFlight;
                        if (ok && !standby_.stop) standby_.ready[req.aircraft].push_back(std::move(jsb));
                    }
                });
            }
        }
    }
    standby_.cv.notify_all();
}

std::unique_ptr<sim::FlightModel> World::popStandbyModel(const std::string& aircraft) {
    std::unique_ptr<sim::FlightModel> out;
    {
        std::lock_guard<std::mutex> lock(standby_.mutex);
        auto it = standby_.ready.find(aircraft);
        if (it != standby_.ready.end() && !it->second.empty()) {
            out = std::move(it->second.back());
            it->second.pop_back();
        }
    }
    return out;
}

std::size_t World::standbyReadyCount(const std::string& aircraft) {
    std::lock_guard<std::mutex> lock(standby_.mutex);
    const auto it = standby_.ready.find(aircraft);
    return it != standby_.ready.end() ? it->second.size() : 0u;
}

void World::stopStandbyPreloader() noexcept {
    std::vector<std::thread> toJoin;
    {
        std::lock_guard<std::mutex> lock(standby_.mutex);
        standby_.stop = true;
        standby_.queue.clear();
        toJoin.swap(standby_.threads);
    }
    standby_.cv.notify_all();
    for (auto& t : toJoin) {
        if (t.joinable()) t.join();
    }
    std::lock_guard<std::mutex> lock(standby_.mutex);
    standby_.ready.clear();
    standby_.queuedOrLoading.clear();
}

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
    return createVehicleChecked(spec,{});
}

std::uint32_t World::createVehicleChecked(const VehicleSpec& spec,const std::function<bool(sim::FlightModel&)>& validate,
                                          std::unique_ptr<sim::FlightModel> preloaded) {
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
            break;
        }
    // Terrain under and around the spawn point must be known before JSBSim
    // trims (on-ground spawns) or the first steps sample it.
    if (terrain_)
        terrain_->prefetch(units::degreesToRadians(spec.initial.latitudeDeg), units::degreesToRadians(spec.initial.longitudeDeg),
                           options_.terrainPrefetchRadiusM);
    if (slot == static_cast<std::size_t>(-1)) {
        const sim::AircraftSpec files{aircraft, jsbsimRoot_,
                                      spec.aircraftDir.empty() ? io::AssetResolver{}.findAircraft(aircraft, jsbsimRoot_) : spec.aircraftDir};
        std::unique_ptr<sim::FlightModel> model;
        if (preloaded) {
            model = std::move(preloaded);
            if (!model->loaded()) {
                LOG_ERROR("session") << "failed to load aircraft '" << aircraft << "'";
                return 0;
            }
        } else {
            if (spec.aircraftDir.empty()) {
                if (auto warm = popStandbyModel(aircraft)) {
                    if (warm->reset(spec.initial)) {
                        model = std::move(warm);
                        requestStandbyModels(aircraft, options_.publish ? 4u : 2u);
                    }
                }
            }
            if (!model) {
                auto jsb = std::make_unique<sim::JsbsimModel>(options_.dt, ground_);
                if (!jsb->load(files, spec.initial)) {
                    LOG_ERROR("session") << "failed to load aircraft '" << aircraft << "'";
                    return 0;
                }
                model = std::move(jsb);
            }
        }
        if (validate && !validate(*model)) return 0;
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
            if (library && fits_[aircraft]) for (const auto& [storeId,card]:fits_[aircraft]->stores) {
                std::string error;
                if (bindPayloadProfile(card,library->parent_path()/"stores",error)) storeCards_.emplace(storeId,card);
                else errors.push_back(storeId+": "+error);
            }
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
                bool declared=false;std::string liquidError;
                const auto frame=airframes_.find(aircraft);
                auto liquid=readLiquidMaterialDatasetV1(dir/aircraft,aircraft,
                    frame==airframes_.end()?nullptr:frame->second.get(),declared,liquidError);
                if(liquid)liquidMaterials_[aircraft]=std::move(liquid);
                else if(declared)liquidMaterialErrors_[aircraft]=liquidError;
                else if(!liquidError.empty())LOG_WARN("session") << "legacy liquid declaration: " << liquidError;
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
        if(const auto refused=liquidMaterialErrors_.find(aircraft);refused!=liquidMaterialErrors_.end()) {
            LOG_ERROR("session") << "required liquid material input: " << refused->second;return 0;
        }
        slot = pool_->add(std::move(model));
        entries_.emplace_back();
        slotAircraft_.push_back(aircraft);
        poolInputs_.emplace_back();
        stepStates_.emplace_back();
    } else {
        if (!pool_->vehicle(slot).reset(spec.initial)) return 0;
        if (validate && !validate(pool_->vehicle(slot))) return 0;
        freeSlots_.erase(std::find(freeSlots_.begin(),freeSlots_.end(),slot));
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
    if(const auto liquid=liquidMaterials_.find(aircraft);liquid!=liquidMaterials_.end())e->liquidMaterials=liquid->second;
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
    const auto names = pointMasses_.find(aircraft);
    e->storeFlight.bind(e->ms, pool_->vehicle(slot), names != pointMasses_.end() ? &names->second : nullptr);
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
    maCore_.retireEntity(id);
    maServices_.release(id); // (its services' ownership ends with it: an entity made later binds afresh)
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
    if (e->dispenser && !e->dispenser->decoys().empty()) retiredDispensers_.push_back(std::move(e->dispenser));
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
    e->storeFlight.invalidateMasses();
    applyStoreMasses(*e);
    // Write retained inventory BEFORE reconciling the reset plant's derived
    // mass/CG/J. An empty supported transaction applies no physical impulse
    // or wrapper CG relocation; custom models keep their optional contract.
    if(e->storeFlight.hasStations()&&model.beginMassChange())model.endMassChange(nullptr,nullptr);
    e->inputs = sim::ControlInputs{};
    e->rotorAirframeTimeS=std::numeric_limits<double>::quiet_NaN();
    e->finControllerAvailable=false;
    e->guidanceDecisionTimeS=std::numeric_limits<double>::quiet_NaN();e->seekerResultV2={};
    e->finTelemetry = {};
    e->diagnosticPropertiesBoundV1=false;e->diagnosticPropertiesV1.clear();
    e->finControllerTimeS=std::numeric_limits<double>::quiet_NaN();
    e->finNavigationTimeS=std::numeric_limits<double>::quiet_NaN();
    e->onRailSeekers.clear();e->seekerAttributionsV3.clear();e->pulseManagerV1.reset();
    if(auto* sensor=dynamic_cast<ms::components::SeekerExecutionV2*>(e->seekerProvider.get()))sensor->resetTracking();
    if(auto* guidance=dynamic_cast<ms::components::GuidanceStateResetV1*>(e->guidance.get()))guidance->resetGuidanceStateV1();
    // A pose reset invalidates this tick's captured history, but it is not a
    // new release and does not rearm a consumed contact-initiation channel.
    e->engagementPath.clear();
    e->engagementStates.clear();
    e->destruction = {};
    e->slidePoseActive = false;
    e->slideSparkAccumS = 0.0;
    e->engagementCg.clear();
    e->engagementMatter.clear();
    // A pose-bound terrain initiation cannot survive teleportation. This
    // cancels its pending burst, without resetting the consumed contact latch.
    e->arrestedBurst.reset();
    e->groundBurstAgeS.reset();
    e->matterAtmosphere = {};
    e->matterFuelBound=false;e->matterFuelReads.clear();
    if (e->fuzeAdapter) e->fuzeAdapter->resetTurnHistory();
    else e->fuzeTurn.reset();
    e->proximityTurnsV1.clear();
    e->inputs.setThrottleAll(e->info.role == ms::Role::Weapon
                                ? (e->info.initial.enginesRunning ? 1.0 : 0.0)
                                : (e->info.initial.onGround ? 0.0 : 0.65));
    e->inputs.gearDown = e->info.initial.onGround ? 1.0 : 0.0;
    e->stack.setInitialInputs(e->inputs);
    e->stack.reset();
    e->host.onReset();
    for (auto& fx : e->effects) fx->onReset();
    // whole again: what a hit or a ground strike broke, holed and set alight is forgotten with the flight it was in
    // (its flight model's reset put its mass and its inertia back)
    e->damage.reset();
    e->crewLost = false;
    e->damageReads = {};
    e->fireRates.clear();
    e->wreckFireUntilS = 0.0;
    e->wreckFireName.clear();
    e->pntCoupling = {};
    const auto restoredEnergy=model.energy();
    e->host.setLoad(restoredEnergy.massKg,restoredEnergy.fuelKg);
    e->storeFlight.loadPublished();
    poolInputs_[e->slot] = e->inputs;
    model.state(e->working);
    e->sensed.state = e->working;
    pool_->refreshState(e->slot);
    resetWeaponNavigationV1(*e);
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

bool World::bindMaService(const MaServiceBinding& binding, std::string& error) {
    const auto refuse = [&](std::string why) {
        error = "MA service " + std::to_string(binding.serviceId) + ": " + std::move(why);
        return false;
    };
    const AgentRecord* agent = nullptr;
    for (const AgentRecord& a : agents_)
        if (a.controller == binding.controller) agent = &a;
    if (!agent) return refuse("controller " + std::to_string(binding.controller) + " is not one this world minted for an agent");
    if (agent->vehicle != binding.platform)
        return refuse("controller " + std::to_string(binding.controller) + " was minted for entity " + std::to_string(agent->vehicle) +
                      ", not for platform " + std::to_string(binding.platform));
    Entry* e = entry(binding.platform);
    if (!e && !c2Node(binding.platform)) return refuse("platform " + std::to_string(binding.platform) + " is gone");
    // What the platform offers for command: its flight capabilities and its Mission Systems capability instances.
    std::vector<std::string> offered;
    if (e) {
        for (const control::CapabilityDescriptor& d : e->catalog->descriptors())
            if ((d.interactions & control::kCommand) && d.kind != control::CapabilityKind::Status) offered.push_back(d.id);
        for (const ms::CapabilityInfo& c : e->ms.capabilities()) offered.emplace_back(c.id);
    }
    for (const std::string& c : binding.capabilities) {
        if (c == "*") {
            if (!e) return refuse("a C2 node has no command capability to bind");
            continue;
        }
        if (std::find(offered.begin(), offered.end(), c) == offered.end())
            return refuse(c + " is not a command capability of platform " + std::to_string(binding.platform));
    }
    if (std::string why = maServices_.refusal(binding); !why.empty()) return refuse(std::move(why));
    // Nothing above changed anything; nothing below can fail.
    maServices_.add(binding);
    if (!e) return true; // (a C2 node: its DMS sides alone)
    settleMaOwnership(*e, binding.platform, binding.controller);
    return true;
}

void World::settleMaOwnership(Entry& e, std::uint32_t platform, control::ControllerId bound) {
    // What stood before a binding gives way to it. For each capability a policy can hold a grant of - flight and
    // Mission Systems alike - a grant that is not its bound owner's, or a live or waiting policy activity that is not,
    // is taken back as the platform takes anything back: the grant revoked, the policy's activities of that capability
    // ending Canceled(Revoked). An unowned capability has no owner: no controller's grant or policy activity stands,
    // the default policy's (controller 0) no more than another's. What the newly bound service owns is then asked
    // for in its name. Its own activities, flown before the platform was under grants, go on under it on a capability
    // nothing was taken back from; where another controller held or acted on the same capability, the revoke - which
    // is of the capability, every controller's activities - ended the owner's with the other's. A capability the
    // platform does not allow or cannot give now stays ungranted, its owner's to ask for later. Last, the platform is
    // under grants. The platform's own sources (autopilot, override) need no grant and are not touched.
    const auto others = [&](const control::ControlStatus& status, control::ControllerId owner) {
        return status.granted && (owner == 0 || status.holder != owner);
    };
    const auto settle = [&](const std::string& id, bool taken, control::ControllerId owner) {
        if (taken) revokeControl(platform, id);
        if (owner != 0 && owner == bound) requestControl(platform, id, owner);
    };
    const std::vector<control::ActivityRecord> flown = e.host.activities(); // (flying, waiting, then ended)
    const auto& descriptors = e.catalog->descriptors();
    for (std::size_t i = 0; i < descriptors.size(); ++i) {
        if (!(descriptors[i].interactions & control::kCommand)) continue;
        const std::string id = descriptors[i].id;
        const control::ControllerId owner = maServices_.owner(platform, id);
        bool taken = others(e.host.controlStatus(i), owner);
        for (const control::ActivityRecord& r : flown)
            taken = taken || (r.live() && r.capability == i && r.source == control::Source::Policy && (owner == 0 || r.controller != owner));
        settle(id, taken, owner);
    }
    const std::vector<ms::ActivityRecord> working = e.ms.activities();
    const auto& instances = e.ms.capabilities();
    for (std::size_t i = 0; i < instances.size(); ++i) {
        const std::string id = instances[i].id;
        const control::ControllerId owner = maServices_.owner(platform, id);
        bool taken = others(e.ms.controlStatus(i), owner);
        for (const ms::ActivityRecord& r : working)
            taken = taken || (r.live() && r.capability == i && r.source == control::Source::Policy && (owner == 0 || r.controller != owner));
        settle(id, taken, owner);
    }
    setControlMode(platform, control::ControlMode::Granted);
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
    maCore_.retireEntity(id);
    maServices_.release(id);
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
            const auto* radar=e->ms.radar(p.index);
            if(!radar)continue;
            for (const auto& track : radar->tracks()) { // original measurement epoch, never refreshed by prediction
                const auto measured=radar->measurementReport(track,pool_->states()[e->slot]);
                if(!measured)continue;
                const auto& t=*measured;
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
        bool jammed(std::uint32_t from, std::uint32_t to, const comm::NetSpec& spec,
                    double rangeM, double) const override {
            const Entry* txE = world.entry(from);
            const Entry* rxE = world.entry(to);
            // Explicitly failed in-band CommTerminal on either endpoint breaks link closure.
            if ((txE && txE->ms.commLinkFailedForFrequency(spec.frequencyHz)) ||
                (rxE && rxE->ms.commLinkFailedForFrequency(spec.frequencyHz))) {
                return true;
            }
            if (world.emissions_.empty()) return false;
            double latB = 0.0, lonB = 0.0, hB = 0.0;
            if (!locate(to, latB, lonB, hB)) return false;
            double rxEcef[3];
            ms::rf::geodeticToEcef(latB, lonB, hB, rxEcef);
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kLight = 299792458.0;
            // Minimum post-despreading SIR for coded tactical QPSK/MSK packet decode (8 dB Eb/N0 threshold).
            constexpr double kRequiredSirDb = 8.0;

            // Resolve physical RF link parameters from operating CommTerminal payloads on tx/rx;
            // fall back to NetSpec tactical baseline ONLY for C2 nodes or aircraft without operating comm payloads.
            ms::CommLinkParams txParams{};
            const bool hasTxTerm = txE && txE->ms.commLinkForFrequency(spec.frequencyHz, txParams);
            ms::CommLinkParams rxParams{};
            const bool hasRxTerm = rxE && rxE->ms.commLinkForFrequency(spec.frequencyHz, rxParams);

            const double txPowerW = hasTxTerm ? txParams.txPowerW : 200.0;
            const double txGainDbi = hasTxTerm ? txParams.rxGainDbi : 0.0;
            const double rxGainDbi = hasRxTerm ? rxParams.rxGainDbi : 0.0;
            const double rxBwHz = hasRxTerm
                                      ? std::max(1.0e3, rxParams.channelBwHz)
                                      : std::clamp(spec.dataRateBps * 50.0, 250.0e3, 3.0e6);
            // Baseline compatibility margin: 18 dB processing gain - 8 dB SIR = 10 dB J/S margin (matches prior NetSpec).
            const double processingGainDb = hasRxTerm ? rxParams.processingGainDb : 18.0;

            const double sigLossDb = 20.0 * std::log10(4.0 * kPi * std::max(rangeM, 1.0) * std::max(spec.frequencyHz, 1.0) / kLight);
            const double rxSignalW = txPowerW * std::pow(10.0, (txGainDbi + rxGainDbi - sigLossDb) / 10.0);
            double jamPowerW = 0.0;
            for (const ms::Emission& em : world.emissions_) {
                if (em.mode != ms::EmitterMode::Jammer || !em.state || em.entity == to ||
                    em.deceptionMode != ms::EaDeceptionModeV1::Noise) continue;
                const double overlap = ms::ea::spectralOverlap(em.frequencyHz, em.bandwidthHz, spec.frequencyHz, rxBwHz);
                if (overlap <= 0.0) continue;
                const double dx = rxEcef[0] - em.state->positionEcef[0];
                const double dy = rxEcef[1] - em.state->positionEcef[1];
                const double dz = rxEcef[2] - em.state->positionEcef[2];
                const double jamRangeM = std::max(1.0, std::sqrt(dx * dx + dy * dy + dz * dz));
                double jamLossDb = 20.0 * std::log10(4.0 * kPi * jamRangeM * std::max(spec.frequencyHz, 1.0) / kLight);
                if (spec.lineOfSight) {
                    ms::rf::PathOptions po;
                    po.frequencyHz = spec.frequencyHz;
                    po.ground = world.ground_.get();
                    const auto path = ms::rf::computePath(
                        {em.state->latitudeRad, em.state->longitudeRad, em.state->altitudeMslM},
                        {latB, lonB, hB}, po);
                    if (std::isfinite(path.totalDb) && path.totalDb > jamLossDb) jamLossDb = path.totalDb;
                }
                const double jTxGainDbi = em.gainTowardsDbi(rxEcef);
                const double j0WHz = ms::ea::receivedJammerDensityWHz(em.powerW, jTxGainDbi, rxGainDbi, jamLossDb, em.bandwidthHz, overlap);
                jamPowerW += j0WHz * rxBwHz;
            }
            if (!(jamPowerW > 0.0)) return false;
            const double effJamW = jamPowerW / std::pow(10.0, std::max(0.0, processingGainDb) / 10.0);
            const double sirEffDb = 10.0 * std::log10(std::max(rxSignalW, 1e-30) / std::max(effJamW, 1e-35));
            return sirEffDb < kRequiredSirDb;
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
    if (!e) return nullptr;
    return e->slidePoseActive ? &e->slideState : &pool_->states()[e->slot];
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
std::optional<sim::VehicleState> estimateOf(const ms::AmtiTrack& t) {
    sim::VehicleState s;
    s.simTime = t.timeS;
    s.positionEcef[0] = t.stateEcef[0], s.positionEcef[1] = t.stateEcef[2], s.positionEcef[2] = t.stateEcef[4];
    s.latitudeRad = t.latitudeRad, s.longitudeRad = t.longitudeRad, s.altitudeMslM = t.heightM;
    for (int i = 0; i < 3; ++i) s.velocityNedMs[i] = t.velocityNedMs[i];
    const double ground = std::hypot(t.velocityNedMs[0], t.velocityNedMs[1]);
    s.eulerRad[2] = std::atan2(t.velocityNedMs[1], t.velocityNedMs[0]);
    s.eulerRad[1] = std::atan2(-t.velocityNedMs[2], ground);
    s.airspeedTrueMs = std::hypot(ground, t.velocityNedMs[2]);
    if (!ms::weapon::completeTrackKinematics(s)) return std::nullopt;
    return s;
}

/// A network track's estimate now: its source's last report carried forward by its velocity (flat earth over the
/// report's age - seconds).
std::optional<sim::VehicleState> estimateOf(const comm::LinkTrack& l, double now) {
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
    if (!ms::weapon::completeTrackKinematics(s)) return std::nullopt;
    return s;
}

} // namespace

std::uint32_t World::designateLink(std::uint32_t owner, comm::SystemId source, std::uint16_t payload, std::uint64_t track) {
    if (!entry(owner)) return 0;
    const comm::LinkTrack* t = dms_.linkTrack(owner, source, payload, track);
    if (!t) return 0;
    const auto estimate=estimateOf(*t,simTime_);
    if (!estimate) return 0;
    Designation d;
    d.id = kDesignations + static_cast<std::uint32_t>(designations_.size()) + 1;
    d.owner = owner, d.payload = payload, d.track = track, d.source = source, d.valid = true, d.estimate = *estimate;
    designations_.push_back(d);
    return d.id;
}

std::uint32_t World::designate(std::uint32_t owner, std::uint16_t payload, std::uint64_t track) {
    if (!entry(owner)) return 0;
    for (const ms::AmtiTrack& t : amtiTracks(owner, payload))
        if (t.id == track) {
            const auto estimate=estimateOf(t);
            if (!estimate) return 0;
            Designation d;
            d.id = kDesignations + static_cast<std::uint32_t>(designations_.size()) + 1;
            d.owner = owner, d.payload = payload, d.track = track, d.valid = true, d.estimate = *estimate;
            designations_.push_back(d);
            return d.id;
        }
    return 0;
}

bool World::undesignate(std::uint32_t owner, std::uint32_t target) {
    if (target <= kDesignations || target - kDesignations > designations_.size()) return false;
    Designation& d = designations_[target - kDesignations - 1];
    if (d.owner != owner || !d.valid) return false;
    d.valid = false;d.retiredV1=true;
    return true;
}

const World::Designation* World::designation(std::uint32_t target) const noexcept {
    if (target <= kDesignations || target - kDesignations > designations_.size()) return nullptr;
    return &designations_[target - kDesignations - 1];
}

std::vector<World::Designation> World::designations(std::uint32_t owner) const {
    std::vector<Designation> out;
    for (const Designation& d : designations_)
        if (d.owner == owner && !d.fusedEntityV1) out.push_back(d);
    return out;
}

void World::refreshDesignations() {
    for (Designation& d : designations_) {
        if (!d.valid) continue;
        d.valid = false; // (lapsed, unless its track is still held)
        if (!entry(d.owner)) continue;
        if(d.fusedEntityV1) {
            if(const auto fused=fusedWeaponInputV1(d.owner,d.id)) {
                sim::VehicleState state{};
                if(resolveWeaponTarget(d.owner,d.id,state))d.estimate=state,d.valid=true;
            }
            continue;
        }
        if (d.source != comm::kNoSystem) { // (a network track: its last report, carried forward)
            if (const comm::LinkTrack* t = dms_.linkTrack(d.owner, d.source, d.payload, d.track))
                if (const auto estimate=estimateOf(*t,simTime_)) d.estimate=*estimate,d.valid=true;
            continue;
        }
        for (const ms::AmtiTrack& t : amtiTracks(d.owner, d.payload))
            if (t.id == d.track) {
                if (const auto estimate=estimateOf(t)) d.estimate=*estimate,d.valid=true;
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
    static_assert(static_cast<int>(control::Level::Count) == ipc::kControlLevels,
                  "the stream names every control level (WorldLayout.h, kControlLevelNames)");
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
    // A platform with configured MA services stays under grants: open, every controller would command every capability.
    if (mode != control::ControlMode::Granted && maServices_.bound(id)) [[unlikely]] return control::Reason::NotAllowed;
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
    // On a platform with configured MA services a capability's grant is its bound owner's or no one's: not another
    // agent's, not the default policy's (controller 0), not any number's; an unowned capability's is no one's.
    // (A capability the platform does not have is refused as ever, by what follows.)
    if (maServices_.bound(id)) [[unlikely]] {
        const bool known = isMission(capability) ? e->ms.find(capability) >= 0 : e->catalog->find(capability) >= 0;
        const control::ControllerId owner = maServices_.owner(id, capability);
        if (known && (owner == 0 || owner != controller)) return control::Reason::NotAllowed;
    }
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
    const bool needState = captureEngagementMotion_ || subStep==0 || e->guidance || !e->effects.empty() || e->stack.activeLevel() != control::Level::Actuator;
    if (needState) model.state(e->working);
    else e->working = pool_->states()[slot];
    {
        // Boundary history is available to SDK historical queries even when
        // no weapon needs a complete native trajectory. Advance only ordinal
        // bookkeeping at the unsampled callbacks in that cheaper mode.
        const auto offset=e->engagementCaptureEpoch.capture(static_cast<unsigned>(subStep),options_.dt,
            e->working.crashed || e->working.diverged);
        if (offset && (captureEngagementMotion_ || subStep==0))
        {
            e->engagementPath.push_back({*offset,{e->working.positionEcef[0],e->working.positionEcef[1],e->working.positionEcef[2]}});
            e->engagementStates.push_back(e->working);
            e->engagementCg.push_back(capturedCg(model));
            e->engagementMatter.push_back(captureMatter(*e,model));
        }
    }
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
    applyPntCouplingToSensed(*e, t);
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
    if (options_.storeDrag && e->storeFlight.addDrag(e->working, model, push, turn)) pushed = true;
    if (e->guidance) {
        const bool navigationReadyV2=advanceWeaponNavigationReadV2(*e,e->working,t,e->sensed.gnssValid);
        if(navigationReadyV2)applyWeaponPulseV1(*e,model,simTime_+subStep*options_.dt);
        // Guidance requests specific acceleration. The finite native FCS and
        // fin aerodynamics alone realise it; no guidance external reaction.
        e->inputs.aileron = e->inputs.elevator = e->inputs.rudder = 0;
        e->finControllerAvailable=false;
        e->finControllerTimeS=simTime_+subStep*options_.dt;
        e->finNavigationTimeS=std::numeric_limits<double>::quiet_NaN();
        const auto& r = e->finControl;
        const auto& s = e->working;
        const bool totalAxialRequired=e->plantAdapter.configured&&e->plantAdapter.control.version==1;
        const double totalAxialN=totalAxialRequired&&r.totalAxialForce.valid()?
            r.totalAxialForce.get()*4.4482216152605:std::numeric_limits<double>::quiet_NaN();
        double cg[3]{};
        const double derivativeVersion=e->plantAdapter.configured && (e->plantAdapter.implementations[0]==2||e->plantAdapter.implementations[0]==3) ? 2.:1.;
        if (navigationReadyV2 && r.version.get() == derivativeVersion && model.cgFromOriginBodyM(cg) &&
            (!totalAxialRequired||std::isfinite(totalAxialN)) &&
            (!r.referenceVersion.valid() || (r.referenceVersion.get()==1 && r.aeroReferenceX.valid() && std::isfinite(r.aeroReferenceX.get()))) &&
            std::all_of(r.aeroForce.begin(),r.aeroForce.end(),[](const auto& p){return p.valid();}) &&
            std::all_of(r.aeroMoment.begin(),r.aeroMoment.end(),[](const auto& p){return p.valid();})) {
            ms::weapon::AutopilotInput in;
            ms::geom::toBody(s, e->guidanceDemand, in.demandBody.data());
            const double gravity[3] = {-9.80665 * std::cos(s.latitudeRad) * std::cos(s.longitudeRad),
                -9.80665 * std::cos(s.latitudeRad) * std::sin(s.longitudeRad), -9.80665 * std::sin(s.latitudeRad)};
            ms::geom::toBody(s, gravity, in.gravityBody.data());
            const auto navigationBinding=weaponLinksV1_.find(e->info.id);
            const bool strapdown=navigationBinding!=weaponLinksV1_.end()&&navigationBinding->second.strapdownNavigation;
            if(strapdown) {
                const auto pose=weaponNavigationPoseV1(*e);
                if(pose.valid)e->finNavigationTimeS=pose.timeS;
                const auto resolve=[&](const double* value,std::array<double,3>& body) {
                    if(!pose.valid){body.fill(std::numeric_limits<double>::quiet_NaN());return;}
                    for(std::size_t k=0;k<3;++k)body[k]=pose.bodyToEcef[k].x*value[0]+pose.bodyToEcef[k].y*value[1]+pose.bodyToEcef[k].z*value[2];
                };
                resolve(e->guidanceDemand,in.demandBody);
                const auto vertical=ms::weapon::navigationGravityVerticalV2(pose.position);
                const double estimatedGravity[3]={vertical.x,vertical.y,vertical.z};
                resolve(estimatedGravity,in.gravityBody);
            }
            in.massKg = model.energy().massKg;
            in.speedMs = s.airspeedTrueMs;
            in.alphaRad = s.alphaRad; in.betaRad = s.betaRad;
            in.actuatorLagS = 1 / r.lag.get();
            for (std::size_t k = 0; k < 3; ++k) {
                in.ratesRadS[k] = s.angularRateBodyRadS[k];
                if(strapdown) {
                    const auto& navigation=*navigationBinding->second.strapdownNavigation;
                    const auto imu=navigation.imuSample();const auto bias=navigation.gyroBiasBodyRadS();
                    const double gyro[3]={imu.gyroRadS.x-bias.x,imu.gyroRadS.y-bias.y,imu.gyroRadS.z-bias.z};
                    const auto pose=weaponNavigationPoseV1(*e);
                    in.ratesRadS[k]=imu.valid&&pose.valid?gyro[k]-7.292115e-5*pose.bodyToEcef[k].z:std::numeric_limits<double>::quiet_NaN();
                }
                in.inertiaKgM2[k] = r.inertia[k].get() * 1.3558179483314;
                in.limitsRad[k] = r.limits[k].get();
            }
            const double qS = r.qbar.get() * r.area.get() * 4.4482216152605;
            const double chord = r.diameter.get() * .3048,span=r.span.get()*.3048;
            const double qSc=qS*chord,qSb=qS*span;
            const double pitchDamping=qSc*chord/(2*std::max(in.speedMs,1.));
            const double lateralDamping=qSb*span/(2*std::max(in.speedMs,1.));
            const double lever=cg[0]-(r.referenceVersion.valid()?r.aeroReferenceX.get():0.);
            auto c = [&](std::size_t k) { return r.derivatives[k].get(); };
            in.pitch = {qS*c(0), qS*c(7), qSc*c(2)+lever*qS*c(0),
                        qSc*c(9)+lever*qS*c(7), pitchDamping*c(4)};
            in.yaw = {qS*c(1), qS*c(8), qSb*c(3)-lever*qS*c(1),
                      qSb*c(10)-lever*qS*c(8), lateralDamping*c(5)};
            in.rollMomentPerRad = qSb*c(11); in.rollDamping = lateralDamping*c(6);
            ms::weapon::AeroFeedback measured;
            measured.controlRad={s.aileronRad,s.elevatorRad,s.rudderRad};
            for (std::size_t k=0;k<3;++k) {
                measured.forceBodyN[k]=r.aeroForce[k].get()*4.4482216152605;
                // JSBSim already reports aerodynamic moments about current CG.
                measured.momentBodyNm[k]=r.aeroMoment[k].get()*1.3558179483314;
            }
            // Configured directional kinematics need the native total
            // non-gravity axial force, including thrust on non-TVC missiles.
            // Transverse and moment residuals retain their aero/TVC roles.
            if(totalAxialRequired)measured.forceBodyN[0]=totalAxialN;
            in.measured=measured;
            const auto& control=e->guidance->spec().control;
            bool vectorReady=!control.thrustVector && !r.vectorVersion.valid();
            if(control.thrustVector) {
                vectorReady=r.vectorVersion.valid() && r.vectorVersion.get()==1 &&
                    r.vectorLimit.valid() && r.vectorLimit.get()==control.thrustVectorLimitRad &&
                    r.vectorRate.valid() && r.vectorRate.get()==control.thrustVectorRateRadS &&
                    r.vectorPitchGain.valid() && r.vectorYawGain.valid() && r.vectorExitX.valid() &&
                    r.vectorPitch.valid() && r.vectorYaw.valid() &&
                    std::all_of(r.propulsionForce.begin(),r.propulsionForce.end(),[](const auto& h){return h.valid();}) &&
                    std::all_of(r.propulsionMoment.begin(),r.propulsionMoment.end(),[](const auto& h){return h.valid();});
                if(vectorReady) {
                    const double pitch=r.vectorPitch.get(),yaw=r.vectorYaw.get(),thrust=s.thrustN[0];
                    const double pitchGain=r.vectorPitchGain.get(),yawGain=r.vectorYawGain.get();
                    const double arm=r.vectorExitX.get()-cg[0];
                    vectorReady=std::isfinite(pitch)&&std::isfinite(yaw)&&std::isfinite(thrust)&&
                        std::isfinite(pitchGain)&&std::isfinite(yawGain)&&std::isfinite(arm)&&
                        std::abs(pitch)<=control.thrustVectorLimitRad+1e-12 &&
                        std::abs(yaw)<=control.thrustVectorLimitRad+1e-12;
                    if(vectorReady) {
                        const double fz=-thrust*std::cos(pitch)*pitchGain;
                        const double fy=thrust*std::cos(pitch)*std::cos(yaw)*yawGain;
                        in.pitch.forceControl+=fz;in.pitch.momentControl-=arm*fz;
                        in.yaw.forceControl+=fy;in.yaw.momentControl+=arm*fy;
                        for(std::size_t k=0;k<3;++k) {
                            if(k!=0||!totalAxialRequired)
                                in.measured->forceBodyN[k]+=r.propulsionForce[k].get()*4.4482216152605;
                            in.measured->momentBodyNm[k]+=r.propulsionMoment[k].get()*1.3558179483314;
                        }
                    }
                }
            }
            ms::weapon::ControlStopsV1 stops;
            const auto metadataVersion=[](const PropertyHandle& handle) -> unsigned {
                if (!handle.valid()) return 0;
                return handle.get()==1 ? 1u : 2u;
            };
            stops.version=metadataVersion(r.normalizationVersion);
            stops.actuatorVersion=metadataVersion(r.actuatorVersion);
            if (stops.version==1 &&
                (!std::all_of(r.lowerStops.begin(),r.lowerStops.end(),[](const auto& h){return h.valid();}) ||
                 !std::all_of(r.upperStops.begin(),r.upperStops.end(),[](const auto& h){return h.valid();}))) stops.version=2;
            const double count=r.finCount.get();
            if (stops.actuatorVersion==1 && (!r.finCount.valid() || !std::isfinite(count) || count<1 ||
                count>ipc::kWeaponControlMaxFins || count!=std::floor(count) || count!=static_cast<double>(r.finRates.size()+r.passiveFinCountV1)))
                stops.actuatorVersion=2;
            stops.channelRateRadS=r.channelRate.get();
            for (std::size_t k=0;k<3;++k) {
                stops.lowerRad[k]=r.lowerStops[k].get(); stops.upperRad[k]=r.upperStops[k].get();
            }
            for (const auto& rate : r.finRates) stops.finRateRadS.push_back(rate.get());
            ms::weapon::FinCommands commands;
            const auto observeControl=[&](const ms::weapon::OperatingPointInputV1* point,bool dispatched) noexcept {
                const auto& observer=ms::weapon::controlObserverV1;
                if(observer.model!=&model||!observer.observe)return;
                const unsigned code=e->plantAdapter.configured?e->plantAdapter.implementations[0]:0;
                const char* mechanism=code==3?"fsim.control.operating_point_bounded.v1":
                    code==2?"fsim.control.actuator_aware_cascade.v1":
                    code==1?"fsim.control.load_feedback_cascade":
                    e->plantAdapter.configured?"configured-unknown":"legacy-unconfigured-finCommands";
                const auto& selected=e->plantAdapter.configured?e->plantAdapter.control:control;
                observer.observe(observer.context,{model,e->info.id,e->info.generation,e->finControllerTimeS,
                    options_.dt,code,mechanism,dispatched,in,selected,stops,point,commands});
            };
            bool controlDispatched=false;
            if(vectorReady) {
                if(!e->plantAdapter.configured) {
                    commands=ms::weapon::finCommands(in,e->guidance->spec().control,stops);controlDispatched=true;
                } else if(e->plantAdapter.implementations[0]!=3) {
                    commands=e->plantProviders.commands(e->plantAdapter,in,stops,nativePlantIdentity(*e));
                    controlDispatched=true;
                }
            }
            if(!e->plantAdapter.configured||e->plantAdapter.implementations[0]!=3)observeControl(nullptr,controlDispatched);
            if(e->plantAdapter.configured&&e->plantAdapter.implementations[0]==3) {
                const auto identity=nativePlantIdentity(*e);
                const double substeps=r.operatingSubsteps.get();
                bool ready=vectorReady&&r.operatingVersion.valid()&&r.operatingVersion.get()==1&&
                    r.operatingSubsteps.valid()&&std::isfinite(substeps)&&substeps>=1&&substeps<=1024&&substeps==std::floor(substeps)&&
                    r.operatingFins.size()==r.finRates.size()&&!r.operatingFins.empty();
                ms::weapon::OperatingPointInputV1 point;point.plant=in;point.stops=stops;point.dtS=options_.dt;
                if(ready)point.actuatorSubsteps=static_cast<unsigned>(substeps);
                const auto readFailure=[](const std::array<PropertyHandle,3>& handles,ms::weapon::OperatingFailureV1& failure) {
                    for(const auto& handle:handles)if(!handle.valid()||(handle.get()!=0&&handle.get()!=1))return false;
                    failure={handles[0].get()!=0,handles[1].get()!=0,handles[2].get()!=0};return true;
                };
                for(std::size_t axis=0;ready&&axis<3;++axis) {
                    ready=readFailure(r.operatingFailure[axis],point.channelFailure[axis])&&r.operatingInput[axis].valid()&&r.operatingLag[axis].valid()&&r.operatingChannel[axis].valid();
                    if(ready){point.channelInputRad[axis]=r.operatingInput[axis].get();point.channelLagRad[axis]=r.operatingLag[axis].get();point.channelActualRad[axis]=r.operatingChannel[axis].get();}
                }
                for(const auto& source:r.operatingFins) {
                    if(!ready)break;
                    ready=source.actual.valid()&&source.minimum.valid()&&source.maximum.valid()&&source.rate.valid()&&source.loads.size()==source.angles.size();
                    if(!ready)break;
                    ms::weapon::OperatingFinV1 fin;ready=readFailure(source.failure,fin.failure);
                    fin.actualRad=source.actual.get();fin.minimumRad=source.minimum.get();fin.maximumRad=source.maximum.get();fin.rateRadS=source.rate.get();
                    for(std::size_t axis=0;axis<3;++axis){ready=ready&&source.mixer[axis].valid();if(ready)fin.mixer[axis]=source.mixer[axis].get();}
                    for(std::size_t knot=0;ready&&knot<source.angles.size();++knot) {
                        ready=source.angles[knot].valid()&&std::all_of(source.loads[knot].begin(),source.loads[knot].end(),[](const auto& h){return h.valid();});
                        if(!ready)break;
                        fin.anglesRad.push_back(source.angles[knot].get());
                        std::array<double,6> load{};for(std::size_t k=0;k<6;++k)load[k]=source.loads[knot][k].get()*(k<3?4.4482216152605:1.3558179483314);
                        fin.momentsNm.push_back(ms::weapon::operatingPointBodyMomentV1(load,s.alphaRad,s.betaRad,{lever,cg[1],cg[2]}));
                    }
                    point.fins.push_back(std::move(fin));
                }
                if(ready)commands=e->plantProviders.commandsOperatingPointV1(e->plantAdapter,point,identity);
                observeControl(&point,ready);
            }
            e->finControllerAvailable=commands.available;
            if(commands.available)e->finDemandBodyMs2=in.demandBody;
            e->inputs.aileron = commands.aileron;
            e->inputs.elevator = commands.elevator;
            e->inputs.rudder = commands.rudder;
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

    // Released weapons retain their launch/reset inputs. Their flight
    // capabilities are restricted; an empty aircraft cascade would replace
    // ignition throttle with neutral before the first native motor step.
    // Aircraft control still runs at the configured FDM rate; lost crews hold.
    if (e->info.role != ms::Role::Weapon && static_cast<unsigned>(subStep) % e->controlDivider == 0 && !e->crewLost) {
        control::ControlContext ctx{e->info.id, e->working, e->sensed.state, options_.dt * e->controlDivider, &stepView_, &e->rng};
        control::OwnRotorPlantV1 rotor;
        double acceleration[3],lag[3];
        if(auto* native=dynamic_cast<sim::JsbsimModel*>(&model);
           native && native->ownRotorAirframeAccelerationV1(acceleration,lag,rotor.specificForceBody,rotor.passiveForceIncluded)) {
            const double elapsed=e->working.simTime-e->rotorAirframeTimeS;
            const bool consecutive=std::isfinite(elapsed) && elapsed>0 && elapsed<=1.5*ctx.dt;
            rotor.available=true;
            for(unsigned k=0;k<3;++k) {
                // First-order actuator inverse of the modeled disturbance:
                // f + tau*f_dot. The secant uses consecutive native epochs;
                // a reset/gap starts without a derivative impulse.
                rotor.angularAcceleration[k]=acceleration[k]+(consecutive ? lag[k]*(acceleration[k]-e->rotorAirframeAcceleration[k])/elapsed : 0.0);
                rotor.available=rotor.available && std::isfinite(rotor.angularAcceleration[k]);
                e->rotorAirframeAcceleration[k]=acceleration[k];
            }
            e->rotorAirframeTimeS=e->working.simTime;
        } else e->rotorAirframeTimeS=std::numeric_limits<double>::quiet_NaN();
        const auto& runtime=e->stack.config();
        const auto courseOwner=runtime.owner[static_cast<std::size_t>(control::Axis::Roll)];
        bool completeCourseOwner=courseOwner<control::kSlotCount;
        for(auto axis:{control::Axis::Pitch,control::Axis::Yaw,control::Axis::Thrust})
            completeCourseOwner=completeCourseOwner && runtime.owner[static_cast<std::size_t>(axis)]==courseOwner;
        if(completeCourseOwner && e->profile && e->profile->hover.header.present()) {
            const auto* velocity=dynamic_cast<const control::RotorVelocity*>(e->stack.controller(control::Level::Velocity));
            const auto* attitude=dynamic_cast<const control::RotorAttitude*>(e->stack.controller(control::Level::Attitude));
            const auto* allocation=dynamic_cast<const control::RotorAllocation*>(e->stack.controller(control::Level::Acceleration));
            if(velocity && attitude && allocation) {
                const auto& hover=e->profile->hover;
                rotor.courseBandwidthRadS=control::rotorCourseResponseV1(*velocity,*attitude,*allocation,
                    hover.roll,hover.pitch,ctx.dt);
                rotor.coursePerformance=&e->stack.config().performance;
                rotor.positionController=dynamic_cast<const control::RotorPosition*>(e->stack.controller(control::Level::Position));
            }
        }
        control::OwnRotorPlantScopeV1 ownPlant(&stepView_,e->info.id,rotor);
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

ms::CommandResult World::submit(std::uint32_t id, const ms::PayloadTask& command, const ms::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

ms::CommandResult World::submit(std::uint32_t id, const ms::EaDeceptionCommandV1& command, const ms::CommandOptions& options) {
    Entry* e = entry(id);
    if (!e) return ms::CommandResult{control::CommandStatus::Rejected, ms::Reason::UnknownVehicle, 0, options.commandId};
    return e->ms.submit(command, options, simTime_);
}

void World::gatherActiveEaEmissions(Span<const sim::VehicleState> states, std::vector<ms::Emission>& out) const {
    out.clear();
    std::vector<ms::Emission> scratch;
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        const Entry* e = entries_[s].get();
        if (!e || states[s].crashed || states[s].diverged) continue;
        bool hasEa = false;
        for (const auto& p : e->ms.payloads()) {
            if (p.family == ms::Family::ElectronicAttack && p.state == ms::PayloadState::Operate) {
                hasEa = true;
                break;
            }
        }
        if (!hasEa) continue;
        scratch.clear();
        e->ms.emissions(states[s], simTime_, scratch);
        for (const auto& em : scratch) {
            if (em.mode == ms::EmitterMode::Jammer && em.state && em.powerW > 0.0) out.push_back(em);
        }
    }
}

void World::gatherActiveDircmBeams(std::vector<ms::DircmBeam>& out) const {
    out.clear();
    for (std::size_t s = 0; s < entries_.size(); ++s) {
        const Entry* e = entries_[s].get();
        if (!e) continue;
        e->ms.dircmBeams(out);
    }
}

void World::applyPntCouplingToSensed(Entry& e, double nowS) const noexcept {
    if (!e.pntCoupling.active) return;
    if (e.pntCoupling.gnssDenied) e.sensed.gnssValid = false;
    const double totalErrM = std::hypot(e.pntCoupling.horizErrorM, e.pntCoupling.vertErrorM);
    if (std::isfinite(totalErrM) && totalErrM > 0.0) {
        e.sensed.positionErrorM = std::max(e.sensed.positionErrorM, totalErrM);
    }
    if (e.pntCoupling.horizErrorM > 0.0 || e.pntCoupling.vertErrorM > 0.0 || e.pntCoupling.velErrorMs > 0.0) {
        // Deterministic Schuler-period (84.4 min) drift direction on sensed navigation only; truth pool_->states() untouched.
        constexpr double kSchulerPeriodS = 5063.5;
        const double phase = e.pntCoupling.driftAzRad + (2.0 * ms::geom::kPi / kSchulerPeriodS) * nowS;
        const double cPh = std::cos(phase);
        const double sPh = std::sin(phase);
        const double dNorth = e.pntCoupling.horizErrorM * cPh;
        const double dEast = e.pntCoupling.horizErrorM * sPh;
        const double dAlt = e.pntCoupling.vertErrorM * std::cos(0.7 * phase + 0.5);
        double lat = e.sensed.state.latitudeRad;
        double lon = e.sensed.state.longitudeRad;
        geo::offsetLatLon(lat, lon, dNorth, dEast, lat, lon);
        e.sensed.state.latitudeRad = lat;
        e.sensed.state.longitudeRad = lon;
        e.sensed.state.altitudeMslM += dAlt;
        ms::rf::geodeticToEcef(lat, lon, e.sensed.state.altitudeMslM, e.sensed.state.positionEcef);
        e.sensed.state.velocityNedMs[0] += e.pntCoupling.velErrorMs * cPh;
        e.sensed.state.velocityNedMs[1] += e.pntCoupling.velErrorMs * sPh;
    }
}

std::vector<ms::PayloadValue> World::payloadReport(std::uint32_t id, std::uint16_t payload) const {
    const Entry* e = entry(id);
    return e ? e->ms.report(payload) : std::vector<ms::PayloadValue>{};
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
    return e.storeFlight.weightRoomKg(most, weighed, carried, loadTick_);
}

ms::Reason World::loadStore(std::uint32_t id, std::uint16_t payload, int station, const std::string& store, int count) {
    Entry* e = entry(id);
    if (!e) return ms::Reason::UnknownVehicle;
    const ms::Reason r = e->ms.load(payload, station, store, count, weightRoomKg(*e));
    if(r==ms::Reason::None)for(const auto& installed:e->ms.stores())
        if(installed.payload==payload&&installed.station==station) {
            e->onRailSeekers.erase(installed.index);
            if (options_.publish) {
                if (const auto* card = e->ms.storeCard(installed.index); card && !card->model.empty()) {
                    std::string fam, ac;
                    if (parseType(card->model, fam, ac) && fam == "jsbsim") requestStandbyModels(ac, 8u);
                }
            }
        }
    applyStoreMasses(); // (the flight model's masses at once: a load is between steps)
    return r;
}

ms::Reason World::loadLoadout(std::uint32_t id, const std::string& loadout) {
    Entry* e = entry(id);
    if (!e) return ms::Reason::UnknownVehicle;
    const ms::Reason r = e->ms.loadLoadout(loadout, weightRoomKg(*e));
    if(r==ms::Reason::None) {
        e->onRailSeekers.clear();
        if (options_.publish) {
            for (const auto& installed : e->ms.stores()) {
                if (const auto* card = e->ms.storeCard(installed.index); card && !card->model.empty()) {
                    std::string fam, ac;
                    if (parseType(card->model, fam, ac) && fam == "jsbsim") requestStandbyModels(ac, 8u);
                }
            }
        }
    }
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
    std::vector<std::uint16_t> selected;
    const bool previewOk = e->ms.previewStrike(command, selected) == ms::Reason::None;
    if (command.target && previewOk) {
        for (const auto index : selected)
            if (const auto reason = weaponLaunchEligibility(*e, index, command.target); reason != ms::Reason::None)
                return ms::CommandResult{control::CommandStatus::Rejected, reason, 0, options.commandId};
    }
    const auto res = e->ms.submit(command, options, simTime_);
    if (res.status == control::CommandStatus::Accepted && previewOk && pool_->workers() > 1u) {
        for (const auto index : selected) {
            if (const auto* card = e->ms.storeCard(index); card && !card->model.empty()) {
                std::string fam, ac;
                if (parseType(card->model, fam, ac) && fam == "jsbsim")
                    requestStandbyModels(ac, options_.publish ? 6u : 2u);
            }
        }
    }
    return res;
}

bool World::resolveWeaponTarget(std::uint32_t launcher, std::uint32_t target, sim::VehicleState& out) const {
    if (target < kDesignations) {
        const auto* state = vehicleState(target);
        if (!state) return false;
        out = *state;
        return true;
    }
    const auto* d = designation(target);
    if (!d || d->owner != launcher || !entry(launcher)) return false;
    if(d->fusedEntityV1)return resolveFusedWeaponTargetV1(launcher,target,out);
    if(!d->valid)return false;
    // Resolve from current measured/delivered reports at this boundary. The
    // step-start designation cache can precede this tick's flight and sensing.
    if (d->source != comm::kNoSystem) {
        const auto* track = dms_.linkTrack(d->owner, d->source, d->payload, d->track);
        if (!track) return false;
        const auto estimate = estimateOf(*track, simTime_);
        if (!estimate) return false;
        out = *estimate;
        return true;
    }
    for (const auto& track : amtiTracks(d->owner, d->payload)) if (track.id == d->track) {
        const auto estimate = estimateOf(track);
        if (!estimate) return false;
        out = *estimate;
        return true;
    }
    return false;
}

ms::Reason World::weaponLaunchEligibility(const Entry& launcher, std::uint16_t store, std::uint32_t target) const {
    const auto* card = launcher.ms.storeCard(store);
    if (!card || card->storeClass != "aam") return ms::Reason::None;
    std::string configurationError;
    const auto spec=configuredWeaponSpec(*card,configurationError);
    if (!spec) {
        LOG_WARN("session") << card->id << ": " << configurationError;
        return ms::Reason::InvalidParameter;
    }
    if (!target) return ms::Reason::None;
    sim::VehicleState estimate{};
    const bool resolved = resolveWeaponTarget(launcher.info.id, target, estimate);
    const auto eligibility = ms::weapon::checkLaunchEligibility(*card, pool_->states()[launcher.slot], resolved ? &estimate : nullptr,spec->seekerModel);
    if (eligibility.reason != ms::Reason::None) return eligibility.reason;
    if (!eligibility.illuminationRequired) return ms::Reason::None;
    const auto* d = designation(target);
    const auto radar = d ? launcher.ms.radarStatus(d->payload) : ms::RadarStatus{};
    return ms::weapon::checkLaunchIllumination(launcher.info.id,
        {d != nullptr, d ? d->owner : 0, d ? d->track : 0, radar.illuminating,
         radar.designatedTrack, d && d->source == comm::kNoSystem});
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
    for (auto& e : entries_) {
        if (!e || !e->storeFlight.hasStations() || !(e->ms.externalFuelKg() > 0.0)) continue;
        auto& model = pool_->vehicle(e->slot);
        e->storeFlight.transferFuel(e->ms, model, options_.dt);
    }
}

void World::applyStoreMasses() {
    for (auto& e : entries_)
        if (e) applyStoreMasses(*e);
}

void World::applyStoreMasses(Entry& e) {
    e.storeFlight.applyMasses(e.ms, pool_->vehicle(e.slot));
}

void World::flyReleases() {
    bool any = false; // (most steps release nothing: nothing allocated then)
    for (const auto& e : entries_) any = any || (e && e->ms.releasing());
    if (!any) return;
    // (launching makes entities: the launchers' ids first, each looked up again after)
    std::vector<std::uint32_t> launchers;
    for (const auto& e : entries_)
        if (e && e->ms.releasing()) launchers.push_back(e->info.id);
    struct PendingLauncherReleases {
        std::uint32_t launcher = 0;
        std::vector<ms::Host::Release> releases;
        std::map<ms::ActivityId, ms::Reason> refusals;
    };
    std::vector<PendingLauncherReleases> pending;
    pending.reserve(launchers.size());
    for (const std::uint32_t launcher : launchers) {
        Entry* e = entry(launcher);
        if (!e) continue;
        PendingLauncherReleases batch;
        batch.launcher = launcher;
        batch.releases = e->ms.takeReleases();
        // Validate every due member of an activity before spawning any. A
        // native execution refusal preserves its actual reason and earlier
        // released weapons. Generic cancellation keeps its queued semantics.
        for (const auto& release : batch.releases) if (!release.jettisoned && !batch.refusals.count(release.activity)) {
            const auto reason = weaponLaunchEligibility(*e, release.store, release.target);
            if (reason != ms::Reason::None) batch.refusals.emplace(release.activity, reason);
        }
        for (auto it = batch.refusals.begin(); it != batch.refusals.end();) {
            if (!e->ms.releaseRefused(it->first, it->second, simTime_)) it = batch.refusals.erase(it);
            else ++it;
        }
        pending.push_back(std::move(batch));
    }
    // Preload fresh JSBSim weapon flight models concurrently on pool workers when
    // >= 2 due releases require new XML loads (no matching free slot). Serial
    // entity/ID/RNG/event creation and launcher mass updates remain in exact order.
    struct PreloadJob {
        std::size_t batchIdx = 0;
        std::size_t relIdx = 0;
        sim::AircraftSpec files;
        sim::InitialConditions ic;
        std::unique_ptr<sim::FlightModel> model;
        bool loaded = false;
    };
    std::vector<PreloadJob> preloadJobs;
    if (pool_->workers() > 1u) {
        std::unordered_map<std::string, std::size_t> freeCount;
        for (std::size_t s : freeSlots_) ++freeCount[slotAircraft_[s]];
        {
            std::lock_guard<std::mutex> lk(standby_.mutex);
            for (const auto& [ac, vec] : standby_.ready) freeCount[ac] += vec.size();
        }
        io::AssetResolver resolver;
        for (std::size_t bi = 0; bi < pending.size(); ++bi) {
            const auto& batch = pending[bi];
            const Entry* le = entry(batch.launcher);
            if (!le) continue;
            double cg[3];
            pool_->vehicle(le->slot).cgFromOriginBodyM(cg);
            for (std::size_t ri = 0; ri < batch.releases.size(); ++ri) {
                const auto& r = batch.releases[ri];
                if (batch.refusals.count(r.activity)) continue;
                std::string family, aircraft;
                if (!parseType(r.launch.type, family, aircraft) || family != "jsbsim") continue;
                if (auto fcIt = freeCount.find(aircraft); fcIt != freeCount.end() && fcIt->second > 0u) {
                    --fcIt->second;
                    continue;
                }
                ms::WeaponLaunch launch = r.launch;
                for (int k = 0; k < 3; ++k) launch.offsetBodyM[k] -= cg[k];
                VehicleSpec spec;
                if (!buildWeaponLaunchSpec(*le, launch, spec)) continue;
                if (terrain_)
                    terrain_->prefetch(units::degreesToRadians(spec.initial.latitudeDeg),
                                       units::degreesToRadians(spec.initial.longitudeDeg),
                                       options_.terrainPrefetchRadiusM);
                PreloadJob job;
                job.batchIdx = bi;
                job.relIdx = ri;
                job.files = sim::AircraftSpec{aircraft, jsbsimRoot_, resolver.findAircraft(aircraft, jsbsimRoot_)};
                job.ic = spec.initial;
                preloadJobs.push_back(std::move(job));
            }
        }
        if (preloadJobs.size() >= 2u) {
            pool_->forEach(preloadJobs.size(), [&](std::size_t j) {
                auto& job = preloadJobs[j];
                auto jsb = std::make_unique<sim::JsbsimModel>(options_.dt, ground_);
                if (jsb->load(job.files, job.ic)) {
                    job.model = std::move(jsb);
                    job.loaded = true;
                }
            });
        } else {
            preloadJobs.clear();
        }
    }
    std::size_t nextPreload = 0;
    for (std::size_t bi = 0; bi < pending.size(); ++bi) {
        const std::uint32_t launcher = pending[bi].launcher;
        Entry* e = entry(launcher);
        if (!e) continue;
        auto& releases = pending[bi].releases;
        auto& refusals = pending[bi].refusals;
        for (std::size_t ri = 0; ri < releases.size(); ++ri) {
            auto& r = releases[ri];
            std::unique_ptr<sim::FlightModel> preloadedModel;
            sim::InitialConditions preloadedIc{};
            bool hasPreloaded = false;
            if (nextPreload < preloadJobs.size() &&
                preloadJobs[nextPreload].batchIdx == bi &&
                preloadJobs[nextPreload].relIdx == ri) {
                if (preloadJobs[nextPreload].loaded) {
                    preloadedModel = std::move(preloadJobs[nextPreload].model);
                    preloadedIc = preloadJobs[nextPreload].ic;
                    hasPreloaded = true;
                }
                ++nextPreload;
            }
            if (refusals.count(r.activity)) continue;
            // the station's place is from the carrier's model origin (its empty CG); the state is at its CG now
            double cg[3];
            pool_->vehicle(e->slot).cgFromOriginBodyM(cg);
            for (int k = 0; k < 3; ++k) r.launch.offsetBodyM[k] -= cg[k];
            const ms::StoreCard* releasedCard = e->ms.storeCard(r.store);
            std::shared_ptr<const ms::components::ProfileV1> releaseProfile;
            ms::weapon::PlantAdapterV1 releasePlant;
            std::optional<ms::weapon::Spec> releaseGuidance;
            std::unique_ptr<ms::weapon::FuzeAdapter> releaseFuze;
            std::unique_ptr<ms::damage::WarheadAdapter> releaseWarhead;
            std::unique_ptr<ms::components::SeekerProviderV1> releaseSeeker;
            std::optional<ms::weapon::Observation> releaseSeekerEstimate;
            std::unique_ptr<ms::components::GuidanceProviderV1> releaseGuidanceProvider;
            ms::components::PlantProvidersV1 releasePlantProviders;
            std::string configurationError;
            bool configured=true;
            bool armedEffects=false;
            bool releaseTerrainContact=false;
            bool guided=!r.launch.inert && releasedCard && releasedCard->storeClass=="aam" && !releasedCard->seeker.empty();
            if (releasedCard) {
                releaseProfile=payloadProfile(*releasedCard,configurationError);
                configured=static_cast<bool>(releaseProfile);
                if(configured && !r.launch.inert && releaseProfile->explicitProfile())
                    guided=releaseProfile->find("guidance") && releaseProfile->find("guidance")->availability!=ms::components::AvailabilityV1::NotApplicable;
                if (configured && guided) {
                    releaseGuidance=configuredWeaponSpec(*releasedCard,configurationError);
                    configured=releaseGuidance.has_value();
                    if(configured && !releaseProfile->explicitProfile()) {
                        std::optional<ms::weapon::Observation> initialEstimate;
                        sim::VehicleState targetEstimate{};
                        if(designation(r.target) && resolveWeaponTarget(launcher,r.target,targetEstimate)) {
                            double v[3];ms::geom::velocityEcef(targetEstimate,v);
                            initialEstimate=ms::weapon::Observation{r.target,
                                {targetEstimate.positionEcef[0],targetEstimate.positionEcef[1],targetEstimate.positionEcef[2]},
                                {v[0],v[1],v[2]},false};
                        }
                        releaseSeeker=ms::components::nativeSeekerProviderV1(releaseGuidance->seekerModel);
                        releaseGuidanceProvider=ms::components::nativeGuidanceProviderV1(*releaseGuidance,initialEstimate);
                    }
                }
                if (configured && releaseProfile->explicitProfile()) {
                    const auto& profile=*releaseProfile;
                    const auto* fuze=profile.find("fuze");
                    const auto* warhead=profile.find("warhead");
                    armedEffects=!r.launch.inert &&
                        ((fuze && fuze->availability!=ms::components::AvailabilityV1::NotApplicable) ||
                         (warhead && warhead->availability!=ms::components::AvailabilityV1::NotApplicable));
                    ms::components::ProfileV1 validation=profile;
                    if(guided || armedEffects)configured=payloadProviders_.validationProfile(profile,validation,configurationError);
                    else {
                        const auto select=[&](const char* kind,const auto& table) {
                            const auto* d=profile.find(kind);
                            if(!d){configurationError="required native phase descriptor missing";return false;}
                            if(d->availability!=ms::components::AvailabilityV1::Available)return true;
                            return table.descriptor(*d,validation.components.at(kind),configurationError);
                        };
                        configured=select("aerodynamics",payloadProviders_.aerodynamics);
                        if(configured && !r.launch.inert)configured=select("propulsion",payloadProviders_.propulsion);
                    }
                    if (configured && guided) {
                        configured=ms::weapon::configurePlantV1(*validation.find("autopilot"),*validation.find("allocator"),
                            *validation.find("actuator"),*validation.find("propulsion"),*validation.find("aerodynamics"),
                            *releasedCard,releasePlant,configurationError);
                        if (configured) {
                            std::optional<ms::weapon::Observation> initialEstimate;
                            sim::VehicleState targetEstimate{};
                            if(designation(r.target) && resolveWeaponTarget(launcher,r.target,targetEstimate)) {
                                double v[3];ms::geom::velocityEcef(targetEstimate,v);
                                initialEstimate=ms::weapon::Observation{r.target,
                                    {targetEstimate.positionEcef[0],targetEstimate.positionEcef[1],targetEstimate.positionEcef[2]},
                                    {v[0],v[1],v[2]},false};
                            }
                            releaseSeekerEstimate=initialEstimate;
                            ms::components::ProviderContextV1 context{*releasedCard,profile,*releaseGuidance,initialEstimate};
                            const bool needsSeeker=profile.find("seeker")->availability!=ms::components::AvailabilityV1::NotApplicable;
                            if(needsSeeker)releaseSeeker=payloadProviders_.seeker.create(*profile.find("seeker"),context,configurationError);
                            if(!needsSeeker || releaseSeeker)releaseGuidanceProvider=payloadProviders_.guidance.create(*profile.find("guidance"),context,configurationError);
                            if(releaseGuidanceProvider)releaseFuze=payloadProviders_.fuze.create(*profile.find("fuze"),context,configurationError);
                            if(releaseFuze)releaseWarhead=payloadProviders_.warhead.create(*profile.find("warhead"),context,configurationError);
                            configured=(!needsSeeker || releaseSeeker) && releaseGuidanceProvider && releaseFuze && releaseWarhead;
                            if(configured&&profile.find("seeker")->bindings.count("measurement_model")&&
                               (!dynamic_cast<ms::components::SeekerExecutionV2*>(releaseSeeker.get())||
                                !dynamic_cast<ms::components::MeasuredGuidanceV2*>(releaseGuidanceProvider.get()))) {
                                configurationError="selected measured model lacks its execution interface";configured=false;
                            }
                        }
                    } else if (configured && r.launch.inert) configured=ms::weapon::configureBallisticPlantV1(*validation.find("aerodynamics"),*releasedCard,releasePlant,configurationError);
                    else if (configured) configured=ms::weapon::configureUnguidedPlantV1(*validation.find("propulsion"),*validation.find("aerodynamics"),*releasedCard,releasePlant,configurationError);
                    if(configured && armedEffects && !guided) {
                        releaseGuidance=configuredWeaponSpec(*releasedCard,configurationError);
                        configured=releaseGuidance.has_value();
                        if(configured) {
                            ms::components::ProviderContextV1 context{*releasedCard,profile,*releaseGuidance,std::nullopt};
                            releaseFuze=payloadProviders_.fuze.create(*profile.find("fuze"),context,configurationError);
                            if(releaseFuze)releaseWarhead=payloadProviders_.warhead.create(*profile.find("warhead"),context,configurationError);
                            configured=releaseFuze && releaseWarhead;
                        }
                    }
                    if(configured && releaseFuze) {
                        const auto* selected=validation.find("fuze");
                        releaseTerrainContact=selected && selected->bindings.count("terrain_contact") &&
                            selected->bindings.at("terrain_contact")=="native.structure_contact.v1";
                    }
                    if(configured) {
                        const char* kinds[]={"autopilot","allocator","actuator","propulsion","aerodynamics"};
                        for(std::size_t k=0;k<5;++k)if(releasePlant.required[k] && releasePlant.availability[k]==ms::components::AvailabilityV1::Available)
                            releasePlant.expected[k].fingerprint=ms::weapon::plantFingerprintV1(*profile.find(kinds[k]));
                        ms::components::ProviderContextV1 context{*releasedCard,profile,releaseGuidance.value_or(ms::weapon::Spec{}),std::nullopt};
                        configured=payloadProviders_.createPlant(profile,context,releasePlant,releasePlantProviders,configurationError);
                    }
                }
            }
            const auto refuse=[&] {
                LOG_WARN("session") << (releasedCard ? releasedCard->id : r.launch.type) << ": native component execution refused: " << configurationError;
                e->ms.releaseExecutionRefused(r.activity,ms::Reason::InvalidParameter,simTime_);
                refusals.emplace(r.activity,ms::Reason::InvalidParameter);
            };
            if (!configured) {refuse();continue;}
            bool nativeBindingRefused=false;
            std::function<bool(sim::FlightModel&)> validate;
            if (releasePlant.configured) validate=[&](sim::FlightModel& model) {
                bool engineMatches=true;
                if (releasePlant.required[3]) {
                    sim::VehicleState state;model.state(state);
                    engineMatches=releasePlant.availability[3]==ms::components::AvailabilityV1::Available
                        ? state.engineCount>=0 && releasePlantProviders.propulsion &&
                            releasePlantProviders.propulsion->acceptsEngineCount(static_cast<unsigned>(state.engineCount))
                        : state.engineCount==0;
                }
                if (engineMatches && releasePlantProviders.available(releasePlant,nativePlantIdentity(model,releasePlant))) return true;
                configurationError="loaded native component identity differs from the selected card profile";
                nativeBindingRefused=true;return false;
            };
            if (hasPreloaded && preloadedModel) {
                // If an earlier release from the same launcher shifted its CG/state,
                // apply the exact updated initial conditions via O(0.1ms) reset().
                VehicleSpec actualSpec;
                const auto sameIc = [](const sim::InitialConditions& a, const sim::InitialConditions& b) noexcept {
                    return a.latitudeDeg == b.latitudeDeg && a.longitudeDeg == b.longitudeDeg &&
                           a.altitudeMslM == b.altitudeMslM && a.headingDeg == b.headingDeg &&
                           a.pitchDeg == b.pitchDeg && a.rollDeg == b.rollDeg &&
                           a.airspeedTrueMs == b.airspeedTrueMs && a.onGround == b.onGround &&
                           a.enginesRunning == b.enginesRunning &&
                           a.velocityNedMs[0] == b.velocityNedMs[0] && a.velocityNedMs[1] == b.velocityNedMs[1] &&
                           a.velocityNedMs[2] == b.velocityNedMs[2] &&
                           a.bodyRatesRadS[0] == b.bodyRatesRadS[0] && a.bodyRatesRadS[1] == b.bodyRatesRadS[1] &&
                           a.bodyRatesRadS[2] == b.bodyRatesRadS[2];
                };
                if (buildWeaponLaunchSpec(*e, r.launch, actualSpec) && !sameIc(preloadedIc, actualSpec.initial)) {
                    if (!preloadedModel->reset(actualSpec.initial)) preloadedModel.reset();
                }
            }
            const std::uint32_t weapon = launchWeaponChecked(launcher,r.launch,validate,std::move(preloadedModel));
            if (nativeBindingRefused) {refuse();continue;}
            if (Entry* w = weapon ? entry(weapon) : nullptr) {
                const auto& info = entry(launcher)->ms.stores();
                w->origin.store = r.store < info.size() ? info[r.store].store : "";
                w->origin.station = r.store < info.size() ? info[r.store].station : 0;
                w->origin.activity = r.activity;
                w->origin.target = r.target;
                w->origin.jettisoned = r.jettisoned;
                w->payloadProfile=std::move(releaseProfile);
                w->plantAdapter=std::move(releasePlant);
                w->plantProviders=std::move(releasePlantProviders);
                w->seekerProvider=std::move(releaseSeeker);
                w->fuzeAdapter=std::move(releaseFuze);
                w->terrainContactEnabled=releaseTerrainContact;
                w->warheadAdapter=std::move(releaseWarhead);
                if (w->plantAdapter.configured) {
                    const char* kinds[]={"autopilot","allocator","actuator","propulsion","aerodynamics"};
                    const char* fields[]={"schema-version","implementation-code","profile-fingerprint"};
                    auto& model=pool_->vehicle(w->slot);
                    for (std::size_t k=0;k<5;++k) if (w->plantAdapter.required[k] &&
                        w->plantAdapter.availability[k]==ms::components::AvailabilityV1::Available) for (std::size_t j=0;j<3;++j)
                        w->plantMetadata[k][j]=model.property(std::string("fsim/components/")+kinds[k]+"/"+fields[j]);
                }
                if (releasedCard) w->contactNoseFromOriginBodyM=authoredContactNose(pool_->vehicle(w->slot));
                if (releasedCard && (guided || armedEffects)) {
                    const auto warhead=w->warheadAdapter ? std::optional<ms::damage::Warhead>{w->warheadAdapter->model()}
                                                        : ms::damage::warheadOf(*releasedCard);
                    if (warhead && (warhead->modelVersion==1 || warhead->modelVersion==2 || warhead->modelVersion==3 || warhead->modelVersion==4)) {
                        w->fixedBurstRequired=true;
                        if (w->contactNoseFromOriginBodyM && warhead->stationStructuralM) {
                            const auto station=*warhead->stationStructuralM;
                            w->burstFromOriginBodyM=*w->contactNoseFromOriginBodyM+ms::damage::V3{-station.x,station.y,-station.z};
                        }
                    }
                    const auto& spec=*releaseGuidance;
                    if (spec.fuzeModelVersion==1 && w->contactNoseFromOriginBodyM && spec.fuzeStationStructuralM) {
                        const auto station=*spec.fuzeStationStructuralM;
                        w->fuzeFromOriginBodyM=*w->contactNoseFromOriginBodyM+ms::damage::V3{-station.x,station.y,-station.z};
                    }
                }
                if (guided) {
                    const auto guidance=*releaseGuidance;
                    const bool infrared=guidance.seeker.find("infrared")!=std::string::npos;
                    w->launchSeekerCue = infrared ? onRailSeekerObservation(*e, *releasedCard,r.store) : 0;
                    if(infrared)if(auto* sensor=dynamic_cast<ms::components::SeekerExecutionV2*>(w->seekerProvider.get())) {
                        const auto cached=e->onRailSeekers.find(r.store);
                        if(cached!=e->onRailSeekers.end()) {
                            const auto* descriptor=w->payloadProfile?w->payloadProfile->find("seeker"):nullptr;
                            if(cached->second.card==releasedCard&&cached->second.provider&&descriptor&&
                               descriptor->bindings.count("receiver_model")&&cached->second.provider->config().engineering.version==3) {
                                // This is the same powered physical receiver, now
                                // separated from its rail. Retain its gimbal,
                                // exposure/processing queue, optical response,
                                // tracker and RNG state. A noisy bearing estimate
                                // is not the receiver's actual pointing state.
                                bool navigationReady=true;
                                if(releaseSeekerEstimate&&(guidance.inertialMidcourse||guidance.launcherUpdates)) {
                                    auto* initialization=dynamic_cast<ms::components::SeekerLaunchNavigationV1*>(cached->second.provider.get());
                                    navigationReady=initialization&&initialization->seedLaunchNavigationV1(*releaseSeekerEstimate);
                                }
                                if(navigationReady) {
                                    w->seekerProvider=std::move(cached->second.provider);
                                    // Pending measurements retain their actual
                                    // capture epochs and attribution indices.
                                    // Their separate referee join must follow
                                    // the receiver across this ownership seam.
                                    w->seekerAttributionsV3=std::move(cached->second.sceneAttributionsV3);
                                }
                                else if(cached->second.result.tracking&&cached->second.result.track)
                                    sensor->seedTrack(*cached->second.result.track);
                            } else if(cached->second.result.tracking&&cached->second.result.track)
                                sensor->seedTrack(*cached->second.result.track);
                        }
                        e->onRailSeekers.erase(r.store);
                    }
                    w->guidance=std::move(releaseGuidanceProvider);
                    auto& wm = pool_->vehicle(w->slot);
                    auto& f = w->finControl;
                    f.version = wm.property("fsim/missile-control/version");
                    f.qbar = wm.property("aero/qbar-psf"); f.area = wm.property("metrics/Sw-sqft");
                    f.diameter = wm.property("metrics/cbarw-ft");
                    f.span = wm.property("metrics/bw-ft");
                    f.referenceVersion=wm.property("fsim/missile-control/reference-version");
                    f.aeroReferenceX=wm.property("fsim/missile-control/aero-reference-x-m");
                    const char* forceAxes[]={"forces/fbx-aero-lbs","forces/fby-aero-lbs","forces/fbz-aero-lbs"};
                    const char* momentAxes[]={"moments/l-aero-lbsft","moments/m-aero-lbsft","moments/n-aero-lbsft"};
                    for (std::size_t k=0;k<3;++k) {
                        f.aeroForce[k]=wm.property(forceAxes[k]); f.aeroMoment[k]=wm.property(momentAxes[k]);
                    }
                    f.totalAxialForce=wm.property("forces/fbx-total-lbs");
                    if(!wm.properties("fsim/thrust-vector-control").empty()) {
                        const std::string prefix="fsim/thrust-vector-control/";
                        f.vectorVersion=wm.property(prefix+"version");f.vectorLimit=wm.property(prefix+"limit_rad");
                        f.vectorRate=wm.property(prefix+"rate_rad_s");f.vectorPitchGain=wm.property(prefix+"pitch_gain");
                        f.vectorYawGain=wm.property(prefix+"yaw_gain");f.vectorExitX=wm.property(prefix+"exit_x_body_m");
                        f.vectorPitch=wm.property("propulsion/engine/pitch-angle-rad");
                        f.vectorYaw=wm.property("propulsion/engine/yaw-angle-rad");
                        const char* force[]={"forces/fbx-prop-lbs","forces/fby-prop-lbs","forces/fbz-prop-lbs"};
                        const char* moment[]={"moments/l-prop-lbsft","moments/m-prop-lbsft","moments/n-prop-lbsft"};
                        for(std::size_t k=0;k<3;++k) {
                            f.propulsionForce[k]=wm.property(force[k]);f.propulsionMoment[k]=wm.property(moment[k]);
                        }
                    }
                    f.lag = wm.property("fsim/missile-control/lag-coefficient-s-1");
                    f.normalizationVersion=wm.property("fsim/missile-control/normalization-version");
                    f.actuatorVersion=wm.property("fsim/missile-control/actuator-version");
                    f.channelRate=wm.property("fsim/missile-control/channel-rate-rad_s");
                    f.finCount=wm.property("fsim/missile-control/fin-count");
                    const double finCount=f.finCount.get();
                    std::set<std::size_t> passiveFins;
                    for(const auto& property:wm.properties("fsim/passive-rolleron-v1")) {
                        const auto& key=property.first;
                        if(key.size()>10&&key.compare(key.size()-10,10,"/fin-index")==0&&
                           std::isfinite(property.second)&&property.second>=0&&property.second<finCount&&property.second==std::floor(property.second))
                            passiveFins.insert(static_cast<std::size_t>(property.second));
                    }
                    f.passiveFinCountV1=passiveFins.size();
                    if (std::isfinite(finCount) && finCount>=1 && finCount<=ipc::kWeaponControlMaxFins && std::floor(finCount)==finCount)
                        for (std::size_t k=0;k<static_cast<std::size_t>(finCount);++k)
                            if(!passiveFins.count(k))f.finRates.push_back(wm.property("fsim/missile-control/fin-"+std::to_string(k)+"-rate-rad_s"));
                    const char* axes[3] = {"aileron", "elevator", "rudder"};
                    const char* inertia[3] = {"ixx", "iyy", "izz"};
                    for (std::size_t k=0; k<3; ++k) {
                        f.inertia[k] = wm.property(std::string("inertia/")+inertia[k]+"-slugs_ft2");
                        f.limits[k] = wm.property(std::string("fsim/missile-control/")+axes[k]+"-limit-rad");
                        f.lowerStops[k]=wm.property(std::string("fsim/missile-control/")+axes[k]+"-min-rad");
                        f.upperStops[k]=wm.property(std::string("fsim/missile-control/")+axes[k]+"-max-rad");
                    }
                    const char* coefficients[12] = {"cz-alpha", "cy-beta", "cm-alpha", "cn-beta", "cm-q", "cn-r",
                        "cl-p", "cz-elevator", "cy-rudder", "cm-elevator", "cn-rudder", "cl-aileron"};
                    for (std::size_t k=0; k<12; ++k)
                        f.derivatives[k] = wm.property(std::string("fsim/control/")+coefficients[k]);
                    if(w->plantAdapter.implementations[0]==3) {
                        const std::string prefix="fsim/control-operating-point-v1/";
                        f.operatingVersion=wm.property(prefix+"version");f.operatingSubsteps=wm.property("fsim/integration-v1/substeps");
                        const char* failures[]={"fail_stuck","fail_zero","fail_hardover"};
                        for(std::size_t k=0;k<3;++k) {
                            for(std::size_t state=0;state<3;++state)
                                f.operatingFailure[k][state]=wm.property(std::string("fcs/")+axes[k]+"-actuator/malfunction/"+failures[state]);
                            f.operatingInput[k]=wm.property(std::string("fcs/")+axes[k]+"-control");
                            f.operatingLag[k]=wm.property(std::string("fcs/")+axes[k]+"-lag-state");
                            f.operatingChannel[k]=wm.property(std::string("fcs/")+axes[k]+"-response-rad");
                        }
                        const char* loads[]={"CD","CY","CL","Cl","Cm","Cn"};
                        for(std::size_t i=0;i<f.finRates.size();++i) {
                            Entry::FinControlReads::OperatingFin fin;const auto name="fin-"+std::to_string(i);
                            fin.actual=wm.property("fcs/"+name+"-delta-rad");fin.minimum=wm.property("fsim/missile-control/"+name+"-min-rad");
                            fin.maximum=wm.property("fsim/missile-control/"+name+"-max-rad");fin.rate=f.finRates[i];
                            for(std::size_t state=0;state<3;++state)fin.failure[state]=wm.property("fcs/"+name+"-actuator/malfunction/"+failures[state]);
                            for(std::size_t k=0;k<3;++k)fin.mixer[k]=wm.property(prefix+name+"/mix-"+axes[k]);
                            const auto countHandle=wm.property(prefix+name+"/knot-count");const double knots=countHandle.get();
                            if(countHandle.valid()&&std::isfinite(knots)&&knots>=2&&knots<=65&&knots==std::floor(knots))
                                for(std::size_t j=0;j<static_cast<std::size_t>(knots);++j) {
                                    const auto knot=prefix+name+"/knot-"+std::to_string(j);
                                    fin.angles.push_back(wm.property(knot+"-rad"));std::array<PropertyHandle,6> row;
                                    for(std::size_t k=0;k<6;++k)row[k]=wm.property(knot+"/"+loads[k]);
                                    fin.loads.push_back(std::move(row));
                                }
                            f.operatingFins.push_back(std::move(fin));
                        }
                    }
                    w->origin.seeker = guidance.seeker;
                    w->origin.phase = infrared ? ms::WeaponPhase::Seeking : ms::WeaponPhase::Midcourse;
                }
                if(releasedCard)bindWeaponLinkV1(*w,*e,*releasedCard,guided?*releaseGuidance:ms::weapon::Spec{});
                recordEngagement("launch", *w, r.target);
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
                // Later releases use their own current CG and the matching
                // native pose/velocity; the model origin remains fixed.
                pool_->refreshState(l->slot);
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

std::vector<ms::FusedEntityV1> World::fusedTracksV1(std::uint32_t id) const {
    const Entry* e=entry(id);return e?e->fusion.snapshot():std::vector<ms::FusedEntityV1>{};
}

ms::FusionStatusV1 World::fusionStatusV1(std::uint32_t id) const {
    ms::FusionStatusV1 status;status.timeS=simTime_;
    if(const auto* e=entry(id))status.rejectedInputs=e->fusion.rejected(),status.capacityRefusals=e->fusion.capacityRefusals();
    return status;
}

void World::fuseTracksV1() {
    const auto states=pool_->states();
    for(auto& pointer:entries_) {
        if(!pointer)continue;
        auto& e=*pointer;std::vector<ms::track::Fusion::Observation> observations;
        const auto system=dms_.systemOf(e.info.id);
        for(const auto& payload:e.ms.payloads()) {
            if(payload.family!=ms::Family::AirRadar)continue;
            for(const auto& track:e.ms.amtiTracks(payload.index,simTime_,states[e.slot])) {
                ms::track::Fusion::Observation o;
                o.source.system=system;o.source.payload=track.payload;o.source.track=track.id;
                o.source.sampleTimeS=o.source.receivedTimeS=track.lastUpdateS;
                o.estimate.t=track.timeS;
                std::copy(std::begin(track.stateEcef),std::end(track.stateEcef),o.estimate.x.a.begin());
                std::copy(std::begin(track.covarianceEcef),std::end(track.covarianceEcef),o.estimate.p.a.begin());
                observations.push_back(std::move(o));
            }
        }
        // Read the modeled DMS picture, never the consuming public inbox.
        if(const auto* links=dms_.linkTracksOf(e.info.id))for(const auto& link:*links) {
            ms::track::Fusion::Observation o;
            o.source.kind=ms::FusionSourceKindV1::DeliveredLink;o.source.system=link.source;
            o.source.payload=link.report.payload;o.source.track=link.report.track;
            o.source.sampleTimeS=link.report.timeS;o.source.receivedTimeS=link.receivedS;
            o.source.diagonalLinkCovariance=true;o.estimate.t=link.report.timeS;
            if(const auto state=estimateOf(link,link.report.timeS)) {
                double velocity[3];ms::geom::velocityEcef(*state,velocity);
                for(std::size_t i=0;i<3;++i) {
                    o.estimate.x.a[2*i]=state->positionEcef[i];o.estimate.x.a[2*i+1]=velocity[i];
                    const double p=link.report.positionSigmaM,v=link.report.velocitySigmaMs;
                    // The wire supplies three-axis RMS values, not isotropic
                    // covariances. With arbitrary anisotropy/cross-correlation,
                    // normalized trace(P)=6 gives P <= 6*diag(p^2,v^2,...).
                    // Fusion then propagates this bound from the sample epoch.
                    o.estimate.p(2*i,2*i)=p>0&&std::isfinite(p)?6*p*p:0;
                    o.estimate.p(2*i+1,2*i+1)=v>0&&std::isfinite(v)?6*v*v:0;
                }
            }
            observations.push_back(std::move(o)); // missing/invalid covariance is explicitly rejected by fusion
        }
        e.fusion.advance(simTime_,std::move(observations));
    }
}

std::vector<ms::Contact> World::contacts(std::uint32_t id, std::uint16_t payload) const {
    const Entry* e = entry(id);
    return e ? e->ms.contacts(payload) : std::vector<ms::Contact>{};
}

void World::sense() {
    // the world each radar looks into, and each of the other payloads' models listens to: the other entities as the
    // step left them, and what they radiate
    struct Scene final : ms::Scene {
        const World& world;
        std::uint32_t own = 0;
        Span<const sim::VehicleState> states;
        Scene(const World& w, Span<const sim::VehicleState> s) : world(w), states(s) {}
        std::size_t targets(std::uint32_t* ids, const sim::VehicleState** out, std::size_t max) const override {
            std::size_t n = 0;
            for (std::size_t s = 0; s < world.entries_.size() && n < max; ++s)
                if (const Entry* e = world.entries_[s].get(); e && e->info.id != own) ids[n] = e->info.id, out[n++] = &states[s];
            for (const auto& c : world.activeChaff_) {
                if (n >= max) break;
                ids[n] = c.id;
                out[n++] = &c.state;
            }
            return n;
        }
        double sigma(std::uint32_t target, double frequencyHz, double polarizationRad, double& fluctuation) const override {
            if (target >= ms::kDecoyIds) {
                for (const auto& c : world.activeChaff_) {
                    if (c.id == target) {
                        fluctuation = 1.0; // Swerling-2 Rayleigh independent dipole cloud scattering
                        const double rcs = ms::countermeasure::chaffRcsM2(c.decoy, c.ageS, frequencyHz);
                        return rcs > 1e-4 ? rcs : std::numeric_limits<double>::quiet_NaN();
                    }
                }
                return std::numeric_limits<double>::quiet_NaN();
            }
            ms::RcsQuery q;
            q.frequencyHz = frequencyHz;
            q.transmitPolarizationRad = polarizationRad;
            const auto rcs = world.radarCrossSection(own, target, q);
            if (!rcs || !rcs->modelled) return std::numeric_limits<double>::quiet_NaN();
            fluctuation = rcs->fluctuation;
            return rcs->sigmaM2;
        }
        double targetExtentRmsM(std::uint32_t target)const override {
            if (target >= ms::kDecoyIds) {
                for (const auto& c : world.activeChaff_)
                    if (c.id == target) return c.extentRmsM;
                return 0.0;
            }
            const auto* e=world.entry(target);
            if(!e||!e->damage)return 0.;
            const auto view=e->damage->attachedGeometry();const auto& frame=view.airframe();
            if(!frame.contactGeometry)return 0.;
            double lo[3]={INFINITY,INFINITY,INFINITY},hi[3]={-INFINITY,-INFINITY,-INFINITY};
            bool any=false;
            auto include=[&](const ms::damage::ContactMesh* mesh){
                if(!mesh)return;
                any=true;const double low[3]={mesh->low.x,mesh->low.y,mesh->low.z},high[3]={mesh->high.x,mesh->high.y,mesh->high.z};
                for(int k=0;k<3;++k){lo[k]=std::min(lo[k],low[k]);hi[k]=std::max(hi[k],high[k]);}
            };
            for(auto i:view.panelIndices())include(frame.contactGeometry->panel(i));
            for(auto i:view.componentIndices())include(frame.contactGeometry->component(i));
            if(!any)return 0.;
            double squared=0;for(int k=0;k<3;++k)squared+=(hi[k]-lo[k])*(hi[k]-lo[k]);
            return std::sqrt(squared/12.); // uniform bounding-volume RMS, explicitly an engineering geometry scale
        }
        double pathLossDb(std::uint32_t target, double frequencyHz) const override {
            const Entry* a = world.entry(own);
            if (!a) return 200.0;
            const sim::VehicleState& sa = states[a->slot];
            double bEcef[3] = {0.0, 0.0, 0.0};
            double bLat = 0.0, bLon = 0.0, bAlt = 0.0;
            if (const Entry* b = world.entry(target)) {
                const sim::VehicleState& sb = states[b->slot];
                bEcef[0] = sb.positionEcef[0], bEcef[1] = sb.positionEcef[1], bEcef[2] = sb.positionEcef[2];
                bLat = sb.latitudeRad, bLon = sb.longitudeRad, bAlt = sb.altitudeMslM;
            } else if (target >= ms::kDecoyIds) {
                bool foundChaff = false;
                for (const auto& c : world.activeChaff_) {
                    if (c.id == target) {
                        bEcef[0] = c.state.positionEcef[0], bEcef[1] = c.state.positionEcef[1], bEcef[2] = c.state.positionEcef[2];
                        bLat = c.state.latitudeRad, bLon = c.state.longitudeRad, bAlt = c.state.altitudeMslM;
                        foundChaff = true;
                        break;
                    }
                }
                if (!foundChaff) return 200.0;
            } else if (const comm::C2NodeSpec* c2 = world.c2Node(target)) {
                bLat = c2->latitudeRad, bLon = c2->longitudeRad, bAlt = c2->altitudeMslM;
                ms::rf::geodeticToEcef(bLat, bLon, bAlt, bEcef);
            } else {
                return 200.0;
            }
            const double d[3] = {bEcef[0] - sa.positionEcef[0], bEcef[1] - sa.positionEcef[1], bEcef[2] - sa.positionEcef[2]};
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
            const ms::rf::Terminal tA{sa.latitudeRad, sa.longitudeRad, sa.altitudeMslM};
            const ms::rf::Terminal tB{bLat, bLon, bAlt};
            const auto p = forward ? ms::rf::computePath(tA, tB, o) : ms::rf::computePath(tB, tA, o);
            const double excess = std::max(0.0, p.totalDb - p.freeSpaceDb);
            {
                std::lock_guard<std::mutex> lock(world.pathExcessMutex_);
                PathExcess& x = world.pathExcess_[key];
                x.timeS = world.simTime_, x.excessDb = excess;
            }
            return freeSpace + excess;
        }
        double jammingDensityWHz(double frequencyHz, double bandwidthHz,
                                 double beamAzimuthRad, double beamElevationRad,
                                 const ms::rf::Pattern& pattern, unsigned azimuthFaces,
                                 bool electronic, const sim::VehicleState& ownState) const override {
            if (world.emissions_.empty()) return 0.0;
            constexpr double kPi = 3.14159265358979323846;
            const double cb = std::cos(beamElevationRad);
            const double beamBody[3] = {cb * std::cos(beamAzimuthRad), cb * std::sin(beamAzimuthRad), -std::sin(beamElevationRad)};
            const double sy = beamBody[1], sz = beamBody[2];
            double totalJ0 = 0.0;
            for (const ms::Emission& e : world.emissions_) {
                if (e.mode != ms::EmitterMode::Jammer || !e.state || e.entity == own ||
                    e.deceptionMode != ms::EaDeceptionModeV1::Noise) continue;
                const double overlap = ms::ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, frequencyHz, bandwidthHz);
                if (overlap <= 0.0) continue;
                double los[3];
                const double rangeM = ms::geom::lineOfSight(ownState, e.state->positionEcef, los);
                if (!(rangeM > 1.0)) continue;
                double db[3];
                ms::geom::toBody(ownState, los, db);
                double rxGainDbi = 0.0;
                if (electronic) {
                    if (azimuthFaces <= 1) {
                        rxGainDbi = pattern.gainDbi(db, sy, sz);
                    } else {
                        const double spacing = 2.0 * kPi / static_cast<double>(azimuthFaces);
                        const double centre = std::round(std::atan2(beamBody[1], beamBody[0]) / spacing) * spacing;
                        const double c = std::cos(centre), s = std::sin(centre);
                        const double local[3] = {c * db[0] + s * db[1], -s * db[0] + c * db[1], db[2]};
                        rxGainDbi = 10.0 * std::log10(std::max(1e-12, pattern.gain(local, -s * beamBody[0] + c * beamBody[1], beamBody[2])));
                    }
                } else {
                    double y[3] = {-beamBody[1], beamBody[0], 0.0};
                    const double yl = std::hypot(y[0], y[1]);
                    if (yl < 1e-9) y[0] = 0.0, y[1] = 1.0, y[2] = 0.0;
                    else y[0] /= yl, y[1] /= yl;
                    const double z[3] = {
                        beamBody[1] * y[2] - beamBody[2] * y[1],
                        beamBody[2] * y[0] - beamBody[0] * y[2],
                        beamBody[0] * y[1] - beamBody[1] * y[0]
                    };
                    const double face[3] = {ms::geom::dot3(db, beamBody), ms::geom::dot3(db, y), ms::geom::dot3(db, z)};
                    rxGainDbi = pattern.gainDbi(face);
                }
                const double txGainDbi = e.gainTowardsDbi(ownState.positionEcef);
                const double lossDb = pathLossDb(e.entity, frequencyHz);
                totalJ0 += ms::ea::receivedJammerDensityWHz(e.powerW, txGainDbi, rxGainDbi, lossDb, e.bandwidthHz, overlap);
            }
            return totalJ0;
        }
        void deceptionEchoes(double frequencyHz, double bandwidthHz, double averagePowerW,
                             double dwellS, double noiseFigureDb, double systemLossDb,
                             double classCalibrationDb, double beamAzimuthRad, double beamElevationRad,
                             const ms::rf::Pattern& pattern, unsigned azimuthFaces, bool electronic,
                             const sim::VehicleState& ownState, const double rBodyToEcef[9],
                             const double vOwnEcef[3], std::vector<ms::radar::FalseEcho>& out) const override {
            if (world.emissions_.empty() || !(frequencyHz > 0.0) || !(averagePowerW > 0.0) || !(dwellS > 0.0)) return;
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kLight = 299792458.0;
            constexpr double kKT0DbW = -203.9772348283416; // 10*log10(1.380649e-23 * 290)
            const double lambda = kLight / frequencyHz;
            const double bwAz = pattern.beamwidthRad(false), bwEl = pattern.beamwidthRad(true);
            const double cb = std::cos(beamElevationRad);
            const double beamBody[3] = {cb * std::cos(beamAzimuthRad), cb * std::sin(beamAzimuthRad), -std::sin(beamElevationRad)};
            const double sy = beamBody[1], sz = beamBody[2];
            std::vector<ms::ea::DrfmGhost> ghosts;
            for (const ms::Emission& e : world.emissions_) {
                if (e.mode != ms::EmitterMode::Jammer || !e.state || e.entity == own ||
                    e.deceptionMode == ms::EaDeceptionModeV1::Noise) continue;
                const double overlap = ms::ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, frequencyHz, bandwidthHz);
                if (!(overlap > 0.0)) continue;
                double los[3];
                const double rangeM = ms::geom::lineOfSight(ownState, e.state->positionEcef, los);
                if (!(rangeM > 1.0)) continue;
                const double db[3] = {
                    rBodyToEcef[0] * los[0] + rBodyToEcef[3] * los[1] + rBodyToEcef[6] * los[2],
                    rBodyToEcef[1] * los[0] + rBodyToEcef[4] * los[1] + rBodyToEcef[7] * los[2],
                    rBodyToEcef[2] * los[0] + rBodyToEcef[5] * los[1] + rBodyToEcef[8] * los[2]};
                const double off = std::acos(std::clamp(ms::geom::dot3(db, beamBody), -1.0, 1.0));
                if (off > 3.0 * std::max(bwAz, bwEl)) continue;
                double rxGainDbi = 0.0;
                if (electronic) {
                    if (azimuthFaces <= 1) {
                        rxGainDbi = pattern.gainDbi(db, sy, sz);
                    } else {
                        const double spacing = 2.0 * kPi / static_cast<double>(azimuthFaces);
                        const double centre = std::round(std::atan2(beamBody[1], beamBody[0]) / spacing) * spacing;
                        const double c = std::cos(centre), sn = std::sin(centre);
                        const double local[3] = {c * db[0] + sn * db[1], -sn * db[0] + c * db[1], db[2]};
                        rxGainDbi = 10.0 * std::log10(std::max(1e-12, pattern.gain(local, -sn * beamBody[0] + c * beamBody[1], beamBody[2])));
                    }
                } else {
                    double y[3] = {-beamBody[1], beamBody[0], 0.0};
                    const double yl = std::hypot(y[0], y[1]);
                    if (yl < 1e-9) y[0] = 0.0, y[1] = 1.0, y[2] = 0.0;
                    else y[0] /= yl, y[1] /= yl;
                    const double z[3] = {
                        beamBody[1] * y[2] - beamBody[2] * y[1],
                        beamBody[2] * y[0] - beamBody[0] * y[2],
                        beamBody[0] * y[1] - beamBody[1] * y[0]};
                    const double face[3] = {ms::geom::dot3(db, beamBody), ms::geom::dot3(db, y), ms::geom::dot3(db, z)};
                    rxGainDbi = pattern.gainDbi(face);
                }
                const double lossDb = pathLossDb(e.entity, frequencyHz);
                const double freeSpaceDb = 20.0 * std::log10(4.0 * kPi * rangeM / lambda);
                const double excessDb = std::max(0.0, lossDb - freeSpaceDb);
                if (excessDb > 25.0) continue; // Terrain-masked DRFM repeater cannot receive/retransmit radar pulse
                const double txGainDbi = e.gainTowardsDbi(ownState.positionEcef);
                const double rxGainLin = std::pow(10.0, rxGainDbi / 10.0);
                const double excessLin = std::pow(10.0, excessDb / 10.0);
                ghosts.clear();
                ms::ea::evaluateDrfmGhosts(e, rangeM, averagePowerW, rxGainLin, excessLin, txGainDbi, overlap, ghosts);
                if (ghosts.empty()) continue;
                double vJam[3];
                ms::geom::velocityEcef(*e.state, vJam);
                const double trueRate = (vJam[0] - vOwnEcef[0]) * los[0] +
                                        (vJam[1] - vOwnEcef[1]) * los[1] +
                                        (vJam[2] - vOwnEcef[2]) * los[2];
                const double jamAz = std::atan2(db[1], db[0]);
                const double jamEl = std::atan2(-db[2], std::hypot(db[0], db[1]));
                for (const ms::ea::DrfmGhost& g : ghosts) {
                    const double snrDb = 10.0 * std::log10(averagePowerW) + 2.0 * rxGainDbi +
                                         10.0 * std::log10(4.0 * kPi * g.equivalentRcsM2 / (lambda * lambda)) -
                                         2.0 * lossDb + 10.0 * std::log10(dwellS) - kKT0DbW -
                                         noiseFigureDb - systemLossDb + classCalibrationDb;
                    ms::radar::FalseEcho fe;
                    fe.id = ms::ea::drfmGhostId(e.entity, e.payload, g.slot);
                    fe.rangeM = rangeM + g.rangeOffsetM;
                    fe.azimuthRad = jamAz;
                    fe.elevationRad = jamEl;
                    fe.rangeRateMs = trueRate + g.rangeRateOffsetMs;
                    fe.snrDb = snrDb;
                    fe.fluctuation = 1.0; // Coherent DRFM steady pulse train
                    out.push_back(fe);
                }
            }
        }
        double surfaceHeightM(double latitudeRad, double longitudeRad) const override {
            // (the clutter's ground, out to the radar's horizon: read apart from the tiles the vehicles stand on - a
            // missing level's tile made from its ancestor for each point, and kept, cost 37 us a point and evicted theirs)
            return world.ground_ ? world.ground_->pointHeightAboveEllipsoidM(latitudeRad, longitudeRad) : std::numeric_limits<double>::quiet_NaN();
        }
        const std::vector<ms::Emission>& emissions() const override { return world.emissions_; }
        const sim::EnvironmentState& environment() const override { return world.environment_; }
        ms::Role role(std::uint32_t id) const override {
            if (id >= ms::kDecoyIds) return ms::Role::Debris;
            const Entry* e = world.entry(id);
            return e ? e->info.role : ms::Role::Aircraft;
        }
        std::uint32_t launcher(std::uint32_t id) const override {
            const Entry* e = world.entry(id);
            return e ? e->info.launcher : 0;
        }
        bool weaponBurning(std::uint32_t id) const override {
            const Entry* e = world.entry(id);
            return e ? e->origin.burning : false;
        }
        bool iffTransponderActive(std::uint32_t id) const override {
            const Entry* e = world.entry(id);
            if (!e || e->slot >= states.size()) return false;
            return e->ms.iffResponderActive(e->crewLost, states[e->slot].crashed || states[e->slot].diverged);
        }
        const std::vector<ms::Decoy>& flares() const override { return world.activeFlares_; }
    };
    const auto states = pool_->states();
    sensing_.clear();
    for (std::size_t s = 0; s < entries_.size(); ++s)
        if (Entry* e = entries_[s].get(); e && e->ms.sensing()) sensing_.push_back(s);
    if (sensing_.empty()) return;
    // what radiates, gathered before the workers start: a receiver reads this, never a radar another worker steps
    // (so what it hears is the scan the step before left - the same for any number of workers)
    emissions_.clear();
    c2EmissionStates_.clear();
    for (std::size_t s = 0; s < entries_.size(); ++s)
        if (const Entry* e = entries_[s].get()) e->ms.emissions(states[s], simTime_, emissions_);
    if (dms_.active()) {
        std::vector<Dms::CarrierEmission> carriers;
        dms_.activeCarriers(simTime_, 1.0, carriers);
        c2EmissionStates_.reserve(carriers.size());
        for (const auto& c : carriers) {
            ms::Emission em;
            em.entity = c.entity;
            em.payload = 0;
            em.mode = ms::EmitterMode::Datalink;
            em.frequencyHz = c.frequencyHz;
            em.bandwidthHz = c.bandwidthHz;
            em.powerW = c.powerW;
            em.peakPowerW = 0.0;
            em.peakGainDbi = 0.0;
            em.sidelobeDbi = 0.0;
            em.allRound = true;
            if (const Entry* e = entry(c.entity)) {
                ms::CommLinkParams cp{};
                if (e->ms.commLinkForFrequency(c.frequencyHz, cp)) {
                    em.powerW = cp.txPowerW;
                    em.bandwidthHz = std::max(1.0e3, cp.channelBwHz);
                    em.peakGainDbi = cp.rxGainDbi;
                }
                em.state = &states[e->slot];
                emissions_.push_back(em);
            } else if (const comm::C2NodeSpec* c2 = c2Node(c.entity)) {
                sim::VehicleState st{};
                st.latitudeRad = c2->latitudeRad;
                st.longitudeRad = c2->longitudeRad;
                st.altitudeMslM = c2->altitudeMslM;
                ms::rf::geodeticToEcef(st.latitudeRad, st.longitudeRad, st.altitudeMslM, st.positionEcef);
                c2EmissionStates_.push_back(st);
                em.state = &c2EmissionStates_.back();
                emissions_.push_back(em);
            }
        }
    }
    for (const auto& e : entries_) if (e && e->guidance &&
            e->origin.phase != ms::WeaponPhase::Midcourse && e->origin.phase != ms::WeaponPhase::Unguided &&
            e->origin.seeker == "active_radar") {
        ms::Emission emission;
        emission.entity=e->info.id; emission.mode=ms::EmitterMode::Seeker;
        emission.state=&states[e->slot]; emission.target=e->origin.holding;
        // basis: assumed. Low-power seeker transmitter; the receiver measures its own bearing and strength.
        const auto seeker=e->seekerProvider ? e->seekerProvider->config() : e->guidance->spec().seekerModel;
        if (seeker.engineering.frequencyHz > 0.0 && std::isfinite(seeker.engineering.frequencyHz))
            emission.frequencyHz = seeker.engineering.frequencyHz;
        emission.powerW=seeker.emitterPowerW; emission.peakGainDbi=seeker.emitterPeakGainDbi; emission.sidelobeDbi=seeker.emitterSidelobeDbi;
        emission.azimuthHalfWidthRad=emission.elevationHalfWidthRad=seeker.emitterHalfWidthRad;
        emissions_.push_back(emission);
    }
    // Collect active pyrotechnic flare decoys (optical/IR) and chaff dipole clouds (airborne AMTI radar)
    // once before parallel worker sensing using physically calibrated cartridge models.
    activeFlares_.clear();
    activeChaff_.clear();
    for (const auto* dispenser : dispensers()) {
        if (!dispenser) continue;
        for (const auto& d : dispenser->decoys()) {
            const double ageS = std::max(0.0, simTime_ - d.launchTimeS);
            if (ageS >= d.lifeS) continue;
            if (d.kind == ms::countermeasure::Kind::Flare) {
                const double intensity = ms::countermeasure::flareIntensityWSr(d, ageS);
                if (!(intensity > 0.0)) continue;
                ms::Decoy flare;
                flare.id = d.id;
                flare.kind = ms::Decoy::Kind::Flare;
                flare.owner = d.owner;
                flare.bornS = d.launchTimeS;
                flare.state.positionEcef[0] = d.position.x;
                flare.state.positionEcef[1] = d.position.y;
                flare.state.positionEcef[2] = d.position.z;
                double lat = 0.0, lon = 0.0, alt = 0.0;
                ecefToGeodeticWgs84(flare.state.positionEcef, lat, lon, alt);
                flare.state.latitudeRad = lat;
                flare.state.longitudeRad = lon;
                flare.state.altitudeMslM = std::max(0.0, alt);
                flare.intensityWSr = intensity;
                activeFlares_.push_back(flare);
            } else {
                ActiveChaff ch;
                ch.id = d.id;
                ch.decoy = d;
                ch.ageS = ageS;
                ch.extentRmsM = ms::countermeasure::chaffBloomRadiusM(d, ageS) * 0.57735;
                ch.state.positionEcef[0] = d.position.x;
                ch.state.positionEcef[1] = d.position.y;
                ch.state.positionEcef[2] = d.position.z;
                // Identity body-to-ECEF frame so ms::geom::velocityEcef(st, v) yields exact cloud ECEF velocity.
                ch.state.rotationBodyToEcef[0] = 1.0;
                ch.state.rotationBodyToEcef[4] = 1.0;
                ch.state.rotationBodyToEcef[8] = 1.0;
                ch.state.velocityBodyMs[0] = d.velocity.x;
                ch.state.velocityBodyMs[1] = d.velocity.y;
                ch.state.velocityBodyMs[2] = d.velocity.z;
                double lat = 0.0, lon = 0.0, alt = 0.0;
                ecefToGeodeticWgs84(ch.state.positionEcef, lat, lon, alt);
                ch.state.latitudeRad = lat;
                ch.state.longitudeRad = lon;
                ch.state.altitudeMslM = std::max(0.0, alt);
                const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
                ch.state.velocityNedMs[0] = -sl * co * d.velocity.x - sl * so * d.velocity.y + cl * d.velocity.z;
                ch.state.velocityNedMs[1] = -so * d.velocity.x + co * d.velocity.y;
                ch.state.velocityNedMs[2] = -cl * co * d.velocity.x - cl * so * d.velocity.y - sl * d.velocity.z;
                ch.state.airspeedTrueMs = std::hypot(d.velocity.x, d.velocity.y, d.velocity.z);
                ch.state.altitudeAglM = ch.state.altitudeMslM;
                activeChaff_.push_back(ch);
            }
        }
    }
    // each radar looks on its own (its stream, its detections), on the workers - a contiguous share of them each, in
    // order: the answers are the same for any number of workers
    pool_->forEach(sensing_.size(), [&](std::size_t k) {
        const std::size_t s = sensing_[k];
        Entry& e = *entries_[s];
        Scene scene(*this, states);
        scene.own = e.info.id;
        e.ms.sense(simTime_, states[s], scene);
    });
    // Couple operating aircraft PNT solutions (GNSS lock/denial + Schuler INS drift) into sensed state.
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        Entry* e = entries_[s].get();
        if (!e || e->info.role != ms::Role::Aircraft) continue;
        ms::PntSolution pnt{};
        if (e->ms.pntSolution(pnt) && pnt.active) {
            e->pntCoupling.active = true;
            e->pntCoupling.gnssDenied = pnt.gnssCapable && !pnt.gnssLocked;
            e->pntCoupling.horizErrorM = pnt.horizErrorM;
            e->pntCoupling.vertErrorM = pnt.vertErrorM;
            e->pntCoupling.velErrorMs = pnt.velErrorMs;
            if (e->pntCoupling.driftAzRad == 0.0) {
                e->pntCoupling.driftAzRad = Rng::forVehicle(options_.seed, 0x504E5444ull, e->info.id).uniform(0.0, 2.0 * ms::geom::kPi);
            }
            e->sensed.state = states[s];
            e->sensed.gnssValid = true;
            e->sensed.positionErrorM = 0.0;
            applyPntCouplingToSensed(*e, simTime_);
        } else if (e->pntCoupling.active) {
            e->pntCoupling = {};
        }
    }
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
        sampleOnRailSeekers();
        prepareWeaponLinksV1();
        evaluateWeaponGnssJamming();
        prepareEngagements();
        const auto before = pool_->states();
        std::copy(before.begin(), before.end(), stepStates_.begin());
        captureEngagementMotion_=pendingDetonationEffects()||std::any_of(entries_.begin(),entries_.end(),
            [](const auto& e){return e && e->info.role==ms::Role::Weapon && !e->weaponEnded;});
        for (auto& e : entries_) if (e) {
            e->engagementCaptureEpoch={};
            e->engagementPath.clear();
            e->engagementStates.clear();
            e->engagementCg.clear();
            e->engagementMatter.clear();
            const auto samples=captureEngagementMotion_ ? static_cast<std::size_t>(options_.frameSkip)+1 : 2;
            e->engagementPath.reserve(samples);
            e->engagementStates.reserve(samples);
            e->engagementCg.reserve(samples);
            e->engagementMatter.reserve(samples);
            e->engagementModelTimeS=before[e->slot].simTime;
        }
        transferFuel();
        applyStoreMasses();
        pool_->step(Span<const sim::ControlInputs>(poolInputs_), options_.frameSkip);
        simTime_ += options_.dt * options_.frameSkip;
        {
            // the contract layer reads what the runtimes flew (docs/control-architecture.md, 6.4)
            const auto liveAfter = pool_->states();
            afterStates_.assign(liveAfter.begin(),liveAfter.end());
            const auto& after=afterStates_;
            for (auto& e : entries_) if (e) {
                const auto& state=after[e->slot];
                // In boundary-only mode a newly terminal final snapshot does
                // not prove a moving segment from the opening pose. Keep the
                // real opening boundary and refuse that unknown interval.
                const auto offset=pool_->active(e->slot) &&
                    (captureEngagementMotion_ || (!state.crashed && !state.diverged))
                    ? e->engagementCaptureEpoch.capture(static_cast<unsigned>(options_.frameSkip),options_.dt,state.crashed || state.diverged)
                    : std::nullopt;
                if (offset)
                {
                    e->engagementPath.push_back({*offset,{state.positionEcef[0],state.positionEcef[1],state.positionEcef[2]}});
                    e->engagementStates.push_back(state);
                    e->engagementCg.push_back(capturedCg(pool_->vehicle(e->slot)));
                    e->engagementMatter.push_back(captureMatter(*e,pool_->vehicle(e->slot)));
                }
            }
            const bool weigh = (loadTick_++ & 7u) == 0;
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (Entry* e = entries_[s].get()) {
                    // its weight to its host, as its fuel burns (every eighth step) and once its stores have changed (their
                    // point masses were set before this step: the flight model's weight has them now) - 7.3
                    if (weigh || e->storeFlight.loadChanged()) {
                        const sim::EnergyOnBoard energy = pool_->vehicle(s).energy();
                        e->host.setLoad(energy.massKg, energy.fuelKg);
                        e->storeFlight.loadPublished();
                    }
                    control::EffectorPositions positions;
                    if (e->host.awaitsPosition()) {
                        positions.gear = after[s].gearPosition;
                        if (e->flapsPosition.valid()) positions.flaps = e->flapsPosition.get();
                    }
                    if (e->host.afterStep(after[s], positions, simTime_)) levelChanged(*e); // (what waited started)
                    e->ms.step(simTime_);
                    const auto dispRequests = e->ms.takeDispenseRequests();
                    for (const auto& req : dispRequests) {
                        const ms::Reason dr = dispense(e->info.id, req.flare);
                        e->ms.dispenseResult(req.activity, req.flare, dr, simTime_);
                    }
                    e->ms.finalizeSalvos(simTime_);
                }
            advanceWeaponNavigationBoundaryV2();
            sense();
            stepGunProjectiles(options_.dt * options_.frameSkip);
            advanceWeaponNavigationV1();
            finishEngagements();
            // Boundary C: all engagement sources/contacts are collected. Their
            // B-to-C material response commits before ground/release ownership
            // and mass transitions; later stepDamage cannot publish C twice.
            advanceDetonationEffects();
            // a weapon that struck the ground has ended (its fuze and warhead: MS-7)
            std::vector<std::uint32_t> impacts;
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (Entry* e = entries_[s].get(); e && s < after.size() && e->info.role == ms::Role::Weapon && after[s].crashed) {
                    if(e->arrestedBurst)continue; // a native contact delay owns the stationary release
                    sim::VehicleState at = after[s]; // (where it struck; its speed as the step began: the strike stopped it)
                    const auto& history=e->engagementStates;
                    auto prior=captureEngagementMotion_ && history.size()>1 && history.back().crashed
                        ? history[history.size()-2] : stepStates_[s];
                    if (const auto* source = dynamic_cast<const sim::TerrainStrikeSourceV1*>(&pool_->vehicle(s)))
                        source->terrainStrikeStateV1(prior);
                    std::copy(std::begin(prior.velocityNedMs), std::end(prior.velocityNedMs), std::begin(at.velocityNedMs));
                    // (a store's strike is a strike: what it does - a tank's fuel left bursting into fire - is told
                    // before it ends there)
                    if (s < stepStates_.size() && !stepStates_[s].crashed) impacted(*e, prior, after[s]);
                    if(e->groundBurstAgeS) {
                        double cg[3];
                        if(pool_->vehicle(s).cgFromOriginBodyM(cg)) {
                            auto point=ms::weapon::materialPoint(after[s],{cg[0],cg[1],cg[2]},
                                e->burstFromOriginBodyM.value_or(ms::damage::V3{cg[0],cg[1],cg[2]}));
                            if(point.valid)e->arrestedBurst=point;
                        }
                    }
                    if(!e->arrestedBurst)endWeapon(*e, ms::WeaponEnd::Impact, at);
                    if(e->weaponEnded)impacts.push_back(e->info.id);
                }
            for (const std::uint32_t id : impacts) removeVehicle(id);
            // B-54 early gear-overload / belly-scrape impact trigger in LightweightDebris mode (before 1.0 m penetration).
            if (destructionMode_ == DestructionModeV1::LightweightDebris) {
                for (std::size_t s = 0; s < entries_.size() && s < after.size() && s < stepStates_.size(); ++s) {
                    Entry* e = entries_[s].get();
                    if (!e || e->info.role == ms::Role::Weapon || after[s].crashed || stepStates_[s].crashed) continue;
                    auto* contactSrc = dynamic_cast<sim::TerrainContactLoadSourceV1*>(&pool_->vehicle(s));
                    sim::TerrainContactLoadV1 load;
                    if (!contactSrc || !contactSrc->terrainContactLoadV1(load)) continue;
                    if (ms::damage::exceedsGearOrBellyContact(load.maxGearSinkMs, load.maxBellyDepthM, load.tangentSpeedMs)) {
                        contactSrc->arrestOnTerrainV1();
                        pool_->refreshState(s);
                        if (s < afterStates_.size()) afterStates_[s] = pool_->states()[s];
                    }
                }
            }
            // an aircraft, or a piece of one, that struck the ground: what the strike does to it (MS-7g)
            for (std::size_t s = 0; s < entries_.size() && s < after.size() && s < stepStates_.size(); ++s)
                if (Entry* e = entries_[s].get(); e && e->info.role != ms::Role::Weapon && after[s].crashed && !stepStates_[s].crashed)
                    impacted(*e, stepStates_[s], after[s]);
            flyReleases(); // (the stores the store management released this step)
            stepDamage();  // (the damaged: their leaks, fires and loads - MS-7g)
            debrisField_.step(options_.dt * options_.frameSkip, ground_.get(), environment_, simTime_);
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
        fuseTracksV1();
        finishWeaponLinksV1();
        sampleOnRailSeekers();
        recordFrame();
        if (publisher_) {
            for (std::size_t s = 0; s < entries_.size(); ++s)
                if (entries_[s]) poolInputs_[s] = entries_[s]->inputs;
            publisher_->addSteps(liveCount_ * static_cast<std::uint64_t>(options_.frameSkip), 1);
            if (publisher_->due()) { // (its vehicles' flight status worked out only as a snapshot is copied)
                fillStatus();
                const auto rawStates = pool_->states();
                std::vector<sim::VehicleState> pubStates(rawStates.begin(), rawStates.end());
                for (std::size_t s = 0; s < entries_.size() && s < pubStates.size(); ++s)
                    applySlidePose(s, pubStates[s]);
                publisher_->setDebris(collectDebrisFrame());
                publisher_->setCombatVfx(collectCombatVfxFrame());
                publisher_->setTacticalOverlay(collectTacticalOverlayFrame());
                if (publisher_->publish(simTime_, Span<const sim::VehicleState>(pubStates), Span<const sim::ControlInputs>(poolInputs_), false,
                                        Span<const ipc::FlightStatus>(poolStatus_),collectWeaponControls()))
                    {publishMission();publishWeaponDiagnosticsV1();}
            }
        }
    }
}

void World::publishNow() {
    if (!publisher_) return;
    for (std::size_t s = 0; s < entries_.size(); ++s)
        if (entries_[s]) poolInputs_[s] = entries_[s]->inputs;
    fillStatus();
    const auto rawStates = pool_->states();
    std::vector<sim::VehicleState> pubStates(rawStates.begin(), rawStates.end());
    for (std::size_t s = 0; s < entries_.size() && s < pubStates.size(); ++s)
        applySlidePose(s, pubStates[s]);
    publisher_->setDebris(collectDebrisFrame());
    publisher_->setCombatVfx(collectCombatVfxFrame());
    publisher_->setTacticalOverlay(collectTacticalOverlayFrame());
    publisher_->publish(simTime_, Span<const sim::VehicleState>(pubStates), Span<const sim::ControlInputs>(poolInputs_), true, Span<const ipc::FlightStatus>(poolStatus_),collectWeaponControls());
    publishMission();
    publishWeaponDiagnosticsV1();
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
        // Observational V1 sidecar: copy native fusion knowledge, never rebuild
        // association or sensor truth in a viewer/client.
        if(publisher_) {
            ipc::FusionRecordV1 fusion;
            fusion.vehicleId=e->info.id;fusion.generation=e->info.generation;fusion.timeS=simTime_;
            fusion.rejectedInputs=e->fusion.rejected();fusion.capacityRefusals=e->fusion.capacityRefusals();
            const auto picture=e->fusion.snapshot();fusion.total=static_cast<std::uint32_t>(picture.size());
            for(const auto& source:picture) {
                if(fusion.count>=ipc::kFusionDisplayEntitiesV1)break;
                auto& target=fusion.entities[fusion.count++];target.id=source.id;target.state=static_cast<std::uint32_t>(source.state);
                target.sourceCount=static_cast<std::uint32_t>(source.sources.size());
                for(const auto& contributor:source.sources)target.sourceKinds|=1u<<static_cast<unsigned>(contributor.kind);
                target.identityKnown=source.identityKnown?1u:0u;target.standardIdentity=source.standardIdentity;
                target.lastUpdateS=source.lastUpdateS;target.validUntilS=source.validUntilS;
                std::copy(std::begin(source.stateEcef),std::end(source.stateEcef),target.stateEcef);
                std::copy(std::begin(source.covarianceEcef),std::end(source.covarianceEcef),target.covarianceEcef);
            }
            publisher_->setFusion(static_cast<std::uint32_t>(s),fusion);
        }
        const control::Performance& p = e->host.performance();
        const double mass = e->host.massKg();
        d[ipc::kDerivedMassKg] = std::isfinite(mass) ? mass : 0.0;
        d[ipc::kDerivedStallCasMs] = p.hovers ? std::numeric_limits<double>::quiet_NaN() : p.minCasMs;
        d[ipc::kDerivedLeastCasMs] = p.hovers ? std::numeric_limits<double>::quiet_NaN() : p.leastCasMs();
        d[ipc::kDerivedFloorCasMs] = e->stack.speedFloorCasMs();
        d[ipc::kDerivedBankCapRad] = e->stack.bankCapRad();
        static_assert(control::kActivityLowEnergy == ipc::kConstraintLowEnergy &&
                          control::kActivityDeviating == ipc::kConstraintDeviating &&
                          control::kActivityUpset == ipc::kConstraintUpset,
                      "the published constraint bits are the wire contract's");
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
            static_assert(static_cast<std::uint32_t>(ms::Family::Count) == ipc::kMissionFamilies,
                          "the published family codes are the wire contract's");
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

bool World::buildWeaponLaunchSpec(const Entry& from, const ms::WeaponLaunch& launch, VehicleSpec& spec) const {
    const sim::VehicleState& ls = pool_->states()[from.slot];
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
    spec = VehicleSpec{};
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
    spec.name = launch.name.empty() ? from.info.name + "-" + model + "-" + std::to_string(nextId_) : launch.name;
    return true;
}

std::uint32_t World::launchWeapon(std::uint32_t launcher, const ms::WeaponLaunch& launch) {
    return launchWeaponChecked(launcher,launch,{});
}

std::uint32_t World::launchWeaponChecked(std::uint32_t launcher,const ms::WeaponLaunch& launch,
    const std::function<bool(sim::FlightModel&)>& validate,std::unique_ptr<sim::FlightModel> preloaded) {
    const Entry* from = entry(launcher);
    if (!from) {
        LOG_ERROR("session") << "launchWeapon: no vehicle " << launcher;
        return 0;
    }
    VehicleSpec spec;
    buildWeaponLaunchSpec(*from, launch, spec);
    const std::uint32_t id = createVehicleChecked(spec,validate,std::move(preloaded));
    Entry* e = entry(id);
    if (!e) return 0;
    e->info.role = ms::Role::Weapon;
    e->info.launcher = launcher;
    e->info.launchTimeS = simTime_;
    // flown by its own model: no flight capability takes it (MS-7 gives it its seeker and guidance)
    for (std::size_t c = 0; c < e->catalog->size(); ++c)
        e->host.setAvailability(c, control::Availability::Unavailable, control::Reason::Restricted);
    e->inputs = sim::ControlInputs{};
    e->inputs.gearDown=0;
    if (!launch.inert) e->inputs.setThrottleAll(1.0);
    poolInputs_[e->slot] = e->inputs;
    LOG_INFO("session") << "weapon " << id << " '" << e->info.name << "' (" << e->info.type << ") released by vehicle " << launcher;
    return id;
}

void World::endWeapon(Entry& e, ms::WeaponEnd end, const sim::VehicleState& state) {
    if (e.weaponEnded) return;
    if (end==ms::WeaponEnd::Impact) recordEngagement("impact",e,0,"ground_impact");
    if (end==ms::WeaponEnd::Removed) recordEngagement("removal",e,0,"removed");
    // A forced Range removal has no onboard shutdown transmitter event.
    if(end!=ms::WeaponEnd::Removed)sampleWeaponLinkV1(e,true);
    e.weaponEnded = true;e.pulseManagerV1.reset();e.proximityTurnsV1.clear();
    ms::WeaponRecord w = e.origin;
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
        ms::WeaponRecord w = e->origin;
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
    {
        std::lock_guard<std::mutex> lock(rcsTablesMutex_);
        if (const auto it = rcsTables_.find(aircraft); it != rcsTables_.end()) return it->second;
    }
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
    std::lock_guard<std::mutex> lock(rcsTablesMutex_);
    auto& slot = rcsTables_[aircraft];
    if (!slot) slot = table;
    return slot;
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
    // B-37 (SIM-M10): add radar cross section of externally carried stores; skip closed stealth weapon bays.
    for (const ms::StoreInfo& si : t->ms.stores()) {
        if (si.status != ms::StoreStatus::Loaded && si.status != ms::StoreStatus::PoweringUp &&
            si.status != ms::StoreStatus::Ready && si.status != ms::StoreStatus::PoweringDown) {
            continue;
        }
        const ms::Card* stCard = t->ms.card(si.payload);
        const ms::Station* stInfo = stCard ? stCard->station(si.station) : nullptr;
        if (stInfo && !stInfo->bay.empty() && si.bayDoors <= 0.1) continue;
        const ms::StoreCard* sc = t->ms.storeCard(si.index);
        if (!sc) continue;
        double storeSigma = 0.0;
        if (sc->model.rfind("jsbsim:", 0) == 0) {
            if (const auto storeTable = rcsTable(sc->model.substr(7))) {
                storeSigma = storeTable->sigma(d, q.frequencyHz, tx, rx);
            }
        }
        if (!(storeSigma > 0.0)) {
            storeSigma = ms::rcs::externalStoreSigmaM2(*sc, d, q.frequencyHz);
        }
        if (std::isfinite(storeSigma) && storeSigma > 0.0) out.sigmaM2 += storeSigma;
    }
    out.fluctuation = table->fluctuation(d, q.frequencyHz);
    return out;
}

void World::stepGunProjectiles(double dt) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    const auto states = pool_->states();

    // 1. Collect newly shed gun ammunition mass (B-33) and newly fired bursts from all alive aircraft.
    std::vector<ms::GunBurst> bursts;
    std::vector<std::pair<GunProjectile, double>> spawned;
    for (std::size_t s = 0; s < entries_.size(); ++s) {
        Entry* e = entries_[s].get();
        if (!e || e->info.role != ms::Role::Aircraft || s >= states.size()) continue;
        const std::uint64_t prevRev = e->ms.massRevision();
        e->ms.collectShedMass();
        if (e->ms.massRevision() != prevRev) applyStoreMasses(*e);
        bursts.clear();
        e->ms.takeBursts(bursts);
        if (bursts.empty()) continue;

        const sim::VehicleState& st = states[s];
        double carrierVel[3];
        ms::geom::velocityEcef(st, carrierVel);
        constexpr double kMuzzleOffsetM = 2.5;
        const double noseBody[3] = {kMuzzleOffsetM, 0.0, 0.0};
        double noseEcef[3];
        ms::geom::toEcef(st, noseBody, noseEcef);
        constexpr double kPi = 3.14159265358979323846;

        for (const ms::GunBurst& b : bursts) {
            if (b.rounds <= 0 || !(b.muzzleMs > 0.0) || !(b.projectileKg > 0.0)) continue;
            const double areaM2 = 0.25 * kPi * b.calibreM * b.calibreM;
            {
                const double fwdBody[3] = {1.0, 0.0, 0.0};
                double fwdEcef[3];
                ms::geom::toEcef(st, fwdBody, fwdEcef);
                ipc::CombatVfxEventWireV1 muzzle{};
                muzzle.kind = static_cast<std::uint8_t>(ipc::CombatVfxEventKindV1::MuzzleFlash);
                muzzle.sourceVehicleId = e->info.id;
                for (int k = 0; k < 3; ++k) {
                    muzzle.positionEcef[k] = st.positionEcef[k] + noseEcef[k];
                    muzzle.velocityEcef[k] = carrierVel[k];
                    muzzle.dirEcef[k] = static_cast<float>(fwdEcef[k]);
                }
                muzzle.magnitude = static_cast<float>(b.calibreM);
                muzzle.param = static_cast<float>(b.rounds);
                pushCombatVfxEvent(muzzle);
            }
            for (int r = 0; r < b.rounds; ++r) {
                const double dAz = gunRng_.normal() * b.dispersionRad;
                const double dEl = gunRng_.normal() * b.dispersionRad;
                const double el = b.elevationRad + dEl;
                const double ce = std::cos(el);
                const double muzzleBody[3] = {
                    b.muzzleMs * ce * std::cos(dAz),
                    b.muzzleMs * ce * std::sin(dAz),
                    -b.muzzleMs * std::sin(el)
                };
                double muzzleEcef[3];
                ms::geom::toEcef(st, muzzleBody, muzzleEcef);
                GunProjectile proj;
                proj.id = nextGunProjectileId_++;
                if (proj.id == 0u) proj.id = nextGunProjectileId_++;
                proj.shooter = e->info.id;
                proj.shooterGeneration = e->info.generation;
                for (int k = 0; k < 3; ++k) {
                    proj.positionEcef[k] = st.positionEcef[k] + noseEcef[k];
                    proj.velocityEcef[k] = carrierVel[k] + muzzleEcef[k];
                }
                proj.massKg = b.projectileKg;
                proj.areaM2 = areaM2;
                proj.calibreM = b.calibreM;
                proj.explosiveKg = b.explosiveKg;
                proj.dragK = b.dragK;
                proj.ageS = 0.0;
                proj.maxAgeS = 8.0;
                proj.tracer = ((gunRoundsFiredCount_++ % 3ull) == 0ull);
                const double subDt = dt * (1.0 - (static_cast<double>(r) + 0.5) / static_cast<double>(b.rounds));
                spawned.emplace_back(proj, std::max(subDt, 1e-4));
            }
        }
    }

    if (gunProjectiles_.empty() && spawned.empty()) return;

    // Helper: exact WGS-84 ECEF -> geodetic (lat, lon, altM) and local up unit vector.
    auto ecefToGeodetic = [](const double p[3], double& lat, double& lon, double& altM, double up[3]) noexcept {
        constexpr double a = 6378137.0;
        constexpr double f = 1.0 / 298.257223563;
        constexpr double e2 = f * (2.0 - f);
        constexpr double b = a * (1.0 - f);
        constexpr double ep2 = (a * a - b * b) / (b * b);
        const double x = p[0], y = p[1], z = p[2];
        const double r = std::hypot(x, y);
        lon = std::atan2(y, x);
        if (r < 1e-6) {
            lat = z >= 0.0 ? 1.5707963267948966 : -1.5707963267948966;
            altM = std::abs(z) - b;
        } else {
            const double th = std::atan2(z * a, r * b);
            const double st = std::sin(th), ct = std::cos(th);
            lat = std::atan2(z + ep2 * b * st * st * st, r - e2 * a * ct * ct * ct);
            const double slat = std::sin(lat), clat = std::cos(lat);
            const double N = a / std::sqrt(1.0 - e2 * slat * slat);
            altM = (std::abs(clat) > 1e-6) ? (r / clat - N) : (std::abs(z) / std::abs(slat) - N * (1.0 - e2));
        }
        const double cl = std::cos(lat);
        up[0] = cl * std::cos(lon);
        up[1] = cl * std::sin(lon);
        up[2] = std::sin(lat);
    };

    // Integrate and test one projectile over `stepDt`. Returns true if projectile remains alive.
    auto advanceOne = [&](GunProjectile& proj, double stepDt) -> bool {
        if (proj.ageS >= proj.maxAgeS) return false;
        double lat = 0.0, lon = 0.0, altM = 0.0, up[3] = {0.0, 0.0, 1.0};
        ecefToGeodetic(proj.positionEcef, lat, lon, altM, up);
        const double rhoRatio = ms::geom::densityRatio(altM);
        constexpr double kG0 = 9.80665;
        const double vMag = ms::geom::norm3(proj.velocityEcef);
        double acc[3];
        for (int k = 0; k < 3; ++k) {
            acc[k] = -proj.dragK * rhoRatio * vMag * proj.velocityEcef[k] - kG0 * up[k];
        }
        double p0[3], p1[3], v1[3];
        for (int k = 0; k < 3; ++k) {
            p0[k] = proj.positionEcef[k];
            p1[k] = p0[k] + proj.velocityEcef[k] * stepDt + 0.5 * acc[k] * stepDt * stepDt;
            v1[k] = proj.velocityEcef[k] + acc[k] * stepDt;
        }
        proj.ageS += stepDt;

        // Check swept intersection against alive aircraft (excluding the shooter).
        const double fracStart = std::clamp(1.0 - stepDt / dt, 0.0, 1.0);
        for (std::size_t s = 0; s < entries_.size(); ++s) {
            Entry* v = entries_[s].get();
            if (!v || v->info.id == proj.shooter || v->info.role != ms::Role::Aircraft || s >= states.size()) continue;
            const sim::VehicleState& stNow = states[s];
            const sim::VehicleState& stPrev = (s < stepStates_.size()) ? stepStates_[s] : stNow;

            // Broad-phase distance from target centre to segment p0 -> p1.
            double dSeg[3], toTgt[3];
            for (int k = 0; k < 3; ++k) {
                dSeg[k] = p1[k] - p0[k];
                toTgt[k] = stNow.positionEcef[k] - p0[k];
            }
            const double segLen2 = ms::geom::dot3(dSeg, dSeg);
            const double tProj = segLen2 > 1e-9 ? std::clamp(ms::geom::dot3(toTgt, dSeg) / segLen2, 0.0, 1.0) : 0.0;
            double closest[3];
            for (int k = 0; k < 3; ++k) closest[k] = (p0[k] + dSeg[k] * tProj) - stNow.positionEcef[k];
            if (ms::geom::dot3(closest, closest) > 40.0 * 40.0) continue;

            if (!ensureDamage(*v) || !v->damage) continue;
            double cg[3] = {0.0, 0.0, 0.0};
            auto& vModel = pool_->vehicle(v->slot);
            if (!vModel.cgFromOriginBodyM(cg)) continue;

            const double* R = stNow.rotationBodyToEcef;
            double cgEcef[3];
            ms::geom::toEcef(stNow, cg, cgEcef);
            double vStartPos[3], vEndPos[3];
            for (int k = 0; k < 3; ++k) {
                vStartPos[k] = stPrev.positionEcef[k] + fracStart * (stNow.positionEcef[k] - stPrev.positionEcef[k]) - cgEcef[k];
                vEndPos[k] = stNow.positionEcef[k] - cgEcef[k];
            }
            double vVel[3];
            ms::geom::velocityEcef(stNow, vVel);

            ms::damage::Target target;
            for (std::size_t k = 0; k < 3; ++k) target.bodyToEcef[k] = {R[k], R[3 + k], R[6 + k]};
            target.at = {vEndPos[0], vEndPos[1], vEndPos[2]};
            target.velocity = {vVel[0], vVel[1], vVel[2]};
            const double rhoTgt = 1.225 * ms::geom::densityRatio(stNow.altitudeMslM);
            const double tempTgt = std::max(216.65, 288.15 - 0.0065 * std::max(0.0, stNow.altitudeMslM));
            target.airDensity = rhoTgt;
            target.airTemperatureK = tempTgt;
            target.airPressurePa = rhoTgt * 287.05 * tempTgt;

            const ms::damage::V3 startRel = {p0[0] - vStartPos[0], p0[1] - vStartPos[1], p0[2] - vStartPos[2]};
            const ms::damage::V3 endRel = {p1[0] - vEndPos[0], p1[1] - vEndPos[1], p1[2] - vEndPos[2]};
            const ms::damage::V3 relVel = {proj.velocityEcef[0] - vVel[0], proj.velocityEcef[1] - vVel[1], proj.velocityEcef[2] - vVel[2]};

            const ms::damage::Impact impact = ms::damage::assessProjectileSegment(
                v->damage->attachedGeometry(), target, startRel, endRel, relVel,
                proj.massKg, proj.areaM2, proj.explosiveKg);
            if (impact.fragmentsStriking > 0) {
                Detonation det;
                det.store = "gun";
                det.weapon = proj.shooter;
                for (int k = 0; k < 3; ++k) {
                    det.positionEcef[k] = p1[k];
                    det.velocityEcef[k] = proj.velocityEcef[k];
                }
                const Entry* shooterEntry = entry(proj.shooter);
                const std::optional<std::uint32_t> sourceSlot =
                    shooterEntry ? std::optional<std::uint32_t>{static_cast<std::uint32_t>(shooterEntry->slot)} : std::nullopt;
                auto events = applyDamagePreparedV1(*v, det, impact, stNow, cg, target.airPressurePa, {},
                                                    sourceSlot, proj.shooterGeneration, false, false, nullptr);
                for (const auto& ev : events) engage(ev);
                if (shooterEntry) {
                    recordEngagement("damage", *shooterEntry, v->info.id, "gun", v->crewLost);
                } else {
                    recordEngagement("damage", *v, v->info.id, "gun", v->crewLost);
                }
                {
                    // Place BulletAirframeHit at the exact struck airframe skin/component coordinate rather than
                    // segment endpoint p1 (which can overshoot 10-25 m past the nose during a 0.02 s step!).
                    ms::damage::V3 hitPosSum{};
                    double hitWeight = 0.0;
                    const auto& af = v->damage->airframe();
                    for (const auto& ph : impact.panels) {
                        if (ph.hits > 0 && ph.panel >= 0 && static_cast<std::size_t>(ph.panel) < af.panels.size()) {
                            const double w = static_cast<double>(ph.hits) * 2.0;
                            hitPosSum += af.panels[static_cast<std::size_t>(ph.panel)].middle() * w;
                            hitWeight += w;
                        }
                    }
                    for (const auto& ch : impact.components) {
                        if (ch.hits > 0 && ch.component >= 0 && static_cast<std::size_t>(ch.component) < af.components.size()) {
                            const double w = static_cast<double>(ch.hits);
                            hitPosSum += af.components[static_cast<std::size_t>(ch.component)].centre * w;
                            hitWeight += w;
                        }
                    }
                    double impactEcef[3] = {p0[0] + dSeg[0] * tProj, p0[1] + dSeg[1] * tProj, p0[2] + dSeg[2] * tProj};
                    if (hitWeight > 0.0) {
                        const ms::damage::V3 hitBody = hitPosSum * (1.0 / hitWeight);
                        const double relBody[3] = {hitBody.x - cg[0], hitBody.y - cg[1], hitBody.z - cg[2]};
                        double relEcef[3] = {};
                        ms::geom::toEcef(stNow, relBody, relEcef);
                        for (int k = 0; k < 3; ++k) impactEcef[k] = stNow.positionEcef[k] + relEcef[k];
                    }
                    ipc::CombatVfxEventWireV1 hitEv{};
                    hitEv.kind = static_cast<std::uint8_t>(ipc::CombatVfxEventKindV1::BulletAirframeHit);
                    hitEv.sourceVehicleId = v->info.id;
                    const double spd = std::max(ms::geom::norm3(proj.velocityEcef), 1e-3);
                    const double relSpd2 = relVel.x * relVel.x + relVel.y * relVel.y + relVel.z * relVel.z;
                    for (int k = 0; k < 3; ++k) {
                        hitEv.positionEcef[k] = impactEcef[k];
                        hitEv.velocityEcef[k] = vVel[k];
                        hitEv.dirEcef[k] = static_cast<float>(proj.velocityEcef[k] / spd);
                    }
                    hitEv.magnitude = static_cast<float>(0.5 * proj.massKg * relSpd2);
                    hitEv.param = static_cast<float>(proj.explosiveKg);
                    pushCombatVfxEvent(hitEv);
                }
                return false; // projectile consumed on impact
            }
        }

        // Commit updated projectile state and check ground/range expiry.
        for (int k = 0; k < 3; ++k) {
            proj.positionEcef[k] = p1[k];
            proj.velocityEcef[k] = v1[k];
        }
        ecefToGeodetic(proj.positionEcef, lat, lon, altM, up);
        const double groundM = ground_ ? ground_->heightAboveEllipsoidM(lat, lon) : 0.0;
        if (altM <= groundM) {
            ipc::CombatVfxEventWireV1 surfEv{};
            surfEv.kind = static_cast<std::uint8_t>(ipc::CombatVfxEventKindV1::BulletSurfaceImpact);
            surfEv.sourceVehicleId = proj.shooter;
            const double spd = ms::geom::norm3(proj.velocityEcef);
            for (int k = 0; k < 3; ++k) {
                surfEv.positionEcef[k] = proj.positionEcef[k] + up[k] * std::max(0.0, groundM - altM);
                surfEv.velocityEcef[k] = 0.0;
                surfEv.dirEcef[k] = static_cast<float>(up[k]);
            }
            surfEv.magnitude = static_cast<float>(0.5 * proj.massKg * spd * spd);
            surfEv.param = static_cast<float>(proj.explosiveKg);
            const auto sk = impactSurfaceOverride_.has_value()
                                ? *impactSurfaceOverride_
                                : (groundM <= 1.5 ? ms::damage::SurfaceKind::Water : ms::damage::SurfaceKind::Soil);
            surfEv.flags = static_cast<std::uint8_t>(sk);
            pushCombatVfxEvent(surfEv);
            return false;
        }
        if (proj.ageS >= proj.maxAgeS || ms::geom::norm3(proj.velocityEcef) < 50.0) {
            return false;
        }
        return true;
    };

    // Advance existing projectiles by full `dt`.
    std::size_t writeIdx = 0;
    for (std::size_t i = 0; i < gunProjectiles_.size(); ++i) {
        if (advanceOne(gunProjectiles_[i], dt)) {
            if (writeIdx != i) gunProjectiles_[writeIdx] = gunProjectiles_[i];
            ++writeIdx;
        }
    }
    gunProjectiles_.resize(writeIdx);

    // Advance newly spawned projectiles by their staggered sub-step and retain surviving ones.
    for (auto& [proj, subDt] : spawned) {
        if (advanceOne(proj, subDt)) {
            gunProjectiles_.push_back(proj);
        }
    }
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
        applySlidePose(s, v.state);
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
    recorder_.writeWeaponControls(simTime_,collectWeaponControls());
    recorder_.writeSnapshot(simTime_, samples);
    // Optional companion .debris sidecar (VIS-M11): in LightweightDebris mode record 1-to-1 aligned frames from t=0
    // so replay seeking and detonation/impact transitions have exact zero-offset synchronization with .fsrec.
    const auto debrisFrame = collectDebrisFrame();
    const bool wantDebrisSidecar = destructionMode_ == DestructionModeV1::LightweightDebris ||
                                   !debrisFrame.bodies.empty() || !debrisFrame.vehicles.empty() || debrisRecorder_.isOpen();
    if (wantDebrisSidecar) {
        if (!debrisRecorder_.isOpen() && !options_.recordPath.empty())
            debrisRecorder_.open(options_.recordPath.string() + ".debris");
        if (debrisRecorder_.isOpen()) debrisRecorder_.write(debrisFrame);
    }
    // Optional companion .vfx sidecar (VIS-M13): lazy-opened on first combat projectile, decoy, or VFX event
    // so non-combat recordings produce zero extra files while combat replays have exact frame-aligned visuals.
    const auto vfxFrame = collectCombatVfxFrame();
    const bool wantVfxSidecar = !vfxFrame.projectiles.empty() || !vfxFrame.decoys.empty() ||
                                !vfxFrame.events.empty() || combatVfxRecorder_.isOpen();
    if (wantVfxSidecar) {
        if (!combatVfxRecorder_.isOpen() && !options_.recordPath.empty())
            combatVfxRecorder_.open(options_.recordPath.string() + ".vfx");
        if (combatVfxRecorder_.isOpen()) combatVfxRecorder_.write(vfxFrame);
    }
    // Optional companion .tact sidecar (VIS-M14): lazy-opened on first tactical/EMSO/extended-propulsion record
    // so legacy recordings produce zero extra files while tactical replays carry exact native overlay frames.
    const auto tactFrame = collectTacticalOverlayFrame();
    const bool wantTactSidecar = !tactFrame.empty() || tacticalRecorder_.isOpen();
    if (wantTactSidecar) {
        if (!tacticalRecorder_.isOpen() && !options_.recordPath.empty())
            tacticalRecorder_.open(options_.recordPath.string() + ".tact");
        if (tacticalRecorder_.isOpen()) tacticalRecorder_.write(tactFrame);
    }
}

void World::pushCombatVfxEvent(ipc::CombatVfxEventWireV1 ev) {
    ev.serial = nextCombatVfxSerial_++;
    if (ev.serial == 0u) ev.serial = nextCombatVfxSerial_++;
    if (!ipc::validCombatVfxEventV1(ev)) return;
    combatVfxEvents_.push_back(TimedCombatVfxEvent{simTime_, ev});
    while (!combatVfxEvents_.empty() &&
           (combatVfxEvents_.size() > ipc::kMaxCombatVfxEventsV1 || simTime_ - combatVfxEvents_.front().simTime > 0.30)) {
        combatVfxEvents_.pop_front();
    }
}

ipc::CombatVfxFrameV1 World::collectCombatVfxFrame() const {
    ipc::CombatVfxFrameV1 out;
    out.simTime = simTime_;

    out.projectiles.reserve(std::min<std::size_t>(gunProjectiles_.size(), ipc::kMaxProjectilesV1));
    for (const auto& p : gunProjectiles_) {
        if (out.projectiles.size() >= ipc::kMaxProjectilesV1) break;
        ipc::ProjectileWireV1 w{};
        for (int k = 0; k < 3; ++k) {
            w.positionEcef[k] = p.positionEcef[k];
            w.velocityEcef[k] = p.velocityEcef[k];
        }
        w.id = p.id;
        w.shooterVehicleId = p.shooter;
        w.calibreM = static_cast<float>(std::max(p.calibreM, 0.005));
        w.ageS = static_cast<float>(std::max(p.ageS, 0.0));
        w.maxAgeS = static_cast<float>(std::max(p.maxAgeS, 0.1));
        w.explosiveKg = static_cast<float>(std::max(p.explosiveKg, 0.0));
        w.flags = (p.tracer ? ipc::kProjectileFlagTracer : 0u) | (p.explosiveKg > 0.0 ? ipc::kProjectileFlagHe : 0u);
        if (ipc::validProjectileWireV1(w)) out.projectiles.push_back(w);
    }

    for (const auto* dispenser : dispensers()) {
        if (!dispenser) continue;
        for (const auto& d : dispenser->decoys()) {
            if (out.decoys.size() >= ipc::kMaxDecoysV1) break;
            const double ageS = std::max(0.0, simTime_ - d.launchTimeS);
            if (ageS >= d.lifeS) continue;
            ipc::DecoyWireV1 w{};
            w.positionEcef[0] = d.position.x;
            w.positionEcef[1] = d.position.y;
            w.positionEcef[2] = d.position.z;
            w.velocityEcef[0] = d.velocity.x;
            w.velocityEcef[1] = d.velocity.y;
            w.velocityEcef[2] = d.velocity.z;
            w.id = d.id;
            w.ownerVehicleId = d.owner;
            w.ageS = static_cast<float>(ageS);
            w.lifeS = static_cast<float>(std::max(d.lifeS, 0.1));
            if (d.kind == ms::countermeasure::Kind::Flare) {
                w.kind = static_cast<std::uint8_t>(ipc::DecoyKindV1::Flare);
                w.intensityOrSigma = static_cast<float>(ms::countermeasure::flareIntensityWSr(d, ageS));
                w.bloomRadiusM = static_cast<float>(ms::countermeasure::flareCoreRadiusM(d, ageS));
            } else {
                w.kind = static_cast<std::uint8_t>(ipc::DecoyKindV1::Chaff);
                w.intensityOrSigma = static_cast<float>(ms::countermeasure::chaffRcsM2(d, ageS, d.spec.chaffCenterFreqHz));
                w.bloomRadiusM = static_cast<float>(ms::countermeasure::chaffBloomRadiusM(d, ageS));
            }
            if (ipc::validDecoyWireV1(w)) out.decoys.push_back(w);
        }
        if (out.decoys.size() >= ipc::kMaxDecoysV1) break;
    }

    for (const auto& timed : combatVfxEvents_) {
        if (simTime_ - timed.simTime > 0.15) continue;
        if (out.events.size() >= ipc::kMaxCombatVfxEventsV1) break;
        out.events.push_back(timed.wire);
    }
    return out;
}

ipc::TacticalOverlayFrameV1 World::tacticalOverlayFrame() const {
    return collectTacticalOverlayFrame();
}

ipc::TacticalOverlayFrameV1 World::collectTacticalOverlayFrame() const {
    ipc::TacticalOverlayFrameV1 out;
    out.simTime = simTime_;
    if (!pool_) return out;

    const auto states = pool_->states();
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kLight = 299792458.0;
    constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;

    auto findReportVal = [](const std::vector<ms::PayloadValue>& rep, const char* key, double fallback) noexcept {
        for (const auto& v : rep) {
            if (v.name && std::strcmp(v.name, key) == 0)
                return std::isfinite(v.value) ? v.value : fallback;
        }
        return fallback;
    };

    auto entityEcef = [&](std::uint32_t entityId, double posOut[3], std::uint32_t& slotOut) noexcept -> bool {
        if (const Entry* ent = entry(entityId); ent && ent->slot < states.size()) {
            std::copy_n(states[ent->slot].positionEcef, 3, posOut);
            slotOut = static_cast<std::uint32_t>(ent->slot);
            return true;
        }
        if (const comm::C2NodeSpec* c2 = c2Node(entityId)) {
            ms::rf::geodeticToEcef(c2->latitudeRad, c2->longitudeRad, c2->altitudeMslM, posOut);
            slotOut = 0;
            return true;
        }
        return false;
    };

    auto pathLossReadOnlyDb = [&](std::uint32_t a, std::uint32_t b, double rangeM, double freqHz) -> double {
        const double freeSpace = 20.0 * std::log10(4.0 * kPi * std::max(rangeM, 1.0) * std::max(freqHz, 1.0) / kLight);
        const PathKey key{std::min(a, b), std::max(a, b), std::round(freqHz / 1e7) * 1e7};
        double excess = 0.0;
        {
            std::lock_guard<std::mutex> lock(pathExcessMutex_);
            if (const auto it = pathExcess_.find(key); it != pathExcess_.end() && std::isfinite(it->second.excessDb))
                excess = std::max(0.0, it->second.excessDb);
        }
        return freeSpace + excess;
    };

    // 1. Active Air Radars: scan volume, instantaneous beam, RCS-aware detection range, and live EA burn-through
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        const Entry* e = entries_[s].get();
        if (!e || !e->info.alive || states[s].crashed) continue;
        const sim::VehicleState& own = states[s];
        const auto& payloads = e->ms.payloads();
        for (const auto& p : payloads) {
            if (p.family != ms::Family::AirRadar) continue;
            if (out.radars.size() >= ipc::kMaxTacticalRadarsV1) break;
            const ms::RadarStatus st = e->ms.radarStatus(p.index);
            const ms::radar::Radar* rad = e->ms.radar(p.index);
            if (!st.searching || !rad) continue;

            ipc::TacticalRadarVolumeWireV1 w{};
            w.slot = static_cast<std::uint32_t>(s);
            w.vehicleId = e->info.id;
            w.payloadIndex = p.index;
            w.mode = static_cast<std::uint8_t>(st.mode);
            w.azimuthCentreRad = static_cast<float>(st.azimuthCentreRad);
            w.azimuthHalfWidthRad = static_cast<float>(st.rotodome ? kPi : std::max(0.02, st.azimuthHalfWidthRad));
            w.elevationCentreRad = static_cast<float>(st.elevationCentreRad);
            const double elHalf = std::isfinite(rad->searching().elevationHalfWidthRad)
                                      ? rad->searching().elevationHalfWidthRad
                                      : std::max(0.04, 0.5 * std::max(st.bars, 1) * std::max(st.beamwidthElevationRad, 0.03));
            w.elevationHalfWidthRad = static_cast<float>(std::clamp(elHalf, 0.02, 1.4));
            w.beamAzimuthRad = static_cast<float>(st.beamAzimuthRad);
            w.beamElevationRad = static_cast<float>(st.beamElevationRad);
            w.rangeScaleM = static_cast<float>(std::max(1000.0, st.rangeScaleM));
            w.flags = ipc::kTactRadarSearching;
            if (st.rotodome) w.flags |= ipc::kTactRadarRotodome;
            if (st.electronic) w.flags |= ipc::kTactRadarElectronic;
            if (st.mode == ms::AmtiMode::SingleTargetTrack) w.flags |= ipc::kTactRadarSttLock;
            if (st.illuminating) w.flags |= ipc::kTactRadarIlluminating;

            // Track / STT target & RCS-aware detection range (B-37 stealth contrast)
            double refSigmaM2 = 5.0;
            const auto tracks = e->ms.amtiTracks(p.index, simTime_, own);
            const ms::AmtiTrack* focusTrack = nullptr;
            if (st.designatedTrack != 0) {
                for (const auto& tr : tracks)
                    if (tr.id == st.designatedTrack) { focusTrack = &tr; break; }
            }
            if (!focusTrack && !tracks.empty()) focusTrack = &tracks.front();
            if (focusTrack) {
                w.flags |= ipc::kTactRadarHasTarget;
                w.targetId = static_cast<std::uint32_t>(focusTrack->id & 0xFFFFFFFFu);
                w.targetRangeM = static_cast<float>(std::max(0.0, focusTrack->rangeM));
                w.targetEcef[0] = focusTrack->stateEcef[0];
                w.targetEcef[1] = focusTrack->stateEcef[2];
                w.targetEcef[2] = focusTrack->stateEcef[4];
                if (focusTrack->truth != 0) {
                    const ms::RcsQuery rq{rad->spec().frequencyHz, rad->spec().polarizationRad, rad->spec().polarizationRad};
                    if (const auto rcs = radarCrossSection(e->info.id, focusTrack->truth, rq);
                        rcs && rcs->modelled && std::isfinite(rcs->sigmaM2) && rcs->sigmaM2 > 1e-6) {
                        refSigmaM2 = rcs->sigmaM2;
                    }
                }
            }
            const double detRaw = rad->detectionRangeM(refSigmaM2, 0.5, 1, st.mode);
            const double detM = std::isfinite(detRaw) && detRaw > 0.0
                                    ? std::min(detRaw, std::max(st.instrumentedRangeM, st.rangeScaleM))
                                    : st.rangeScaleM;
            w.detectionRangeM = static_cast<float>(std::max(0.0, detM));

            // Live EA jammer interference & burn-through range under (1 + J0/N0)^(-1/4)
            double totalJ0 = 0.0;
            if (!emissions_.empty()) {
                const double cb = std::cos(st.beamElevationRad);
                const double beamBody[3] = {cb * std::cos(st.beamAzimuthRad), cb * std::sin(st.beamAzimuthRad), -std::sin(st.beamElevationRad)};
                const double sy = beamBody[1], sz = beamBody[2];
                const double rxBwHz = std::max(1.0e5, 1.0 / std::max(rad->spec().dwellS, 1e-4));
                for (const ms::Emission& em : emissions_) {
                    if (em.mode != ms::EmitterMode::Jammer || !em.state || em.entity == e->info.id ||
                        em.deceptionMode != ms::EaDeceptionModeV1::Noise) continue;
                    const double overlap = ms::ea::spectralOverlap(em.frequencyHz, em.bandwidthHz, rad->spec().frequencyHz, rxBwHz);
                    if (overlap <= 0.0) continue;
                    double los[3];
                    const double rangeM = ms::geom::lineOfSight(own, em.state->positionEcef, los);
                    if (!(rangeM > 1.0)) continue;
                    double db[3];
                    ms::geom::toBody(own, los, db);
                    double rxGainDbi = 0.0;
                    if (st.electronic) {
                        if (rad->spec().azimuthFaces <= 1) {
                            rxGainDbi = rad->pattern().gainDbi(db, sy, sz);
                        } else {
                            const double spacing = 2.0 * kPi / static_cast<double>(rad->spec().azimuthFaces);
                            const double centre = std::round(std::atan2(beamBody[1], beamBody[0]) / spacing) * spacing;
                            const double c = std::cos(centre), sn = std::sin(centre);
                            const double local[3] = {c * db[0] + sn * db[1], -sn * db[0] + c * db[1], db[2]};
                            rxGainDbi = 10.0 * std::log10(std::max(1e-12, rad->pattern().gain(local, -sn * beamBody[0] + c * beamBody[1], beamBody[2])));
                        }
                    } else {
                        double y[3] = {-beamBody[1], beamBody[0], 0.0};
                        const double yl = std::hypot(y[0], y[1]);
                        if (yl < 1e-9) y[0] = 0.0, y[1] = 1.0, y[2] = 0.0;
                        else y[0] /= yl, y[1] /= yl;
                        const double z[3] = {
                            beamBody[1] * y[2] - beamBody[2] * y[1],
                            beamBody[2] * y[0] - beamBody[0] * y[2],
                            beamBody[0] * y[1] - beamBody[1] * y[0]};
                        const double face[3] = {ms::geom::dot3(db, beamBody), ms::geom::dot3(db, y), ms::geom::dot3(db, z)};
                        rxGainDbi = rad->pattern().gainDbi(face);
                    }
                    const double txGainDbi = em.gainTowardsDbi(own.positionEcef);
                    const double lossDb = pathLossReadOnlyDb(e->info.id, em.entity, rangeM, rad->spec().frequencyHz);
                    totalJ0 += ms::ea::receivedJammerDensityWHz(em.powerW, txGainDbi, rxGainDbi, lossDb, em.bandwidthHz, overlap);
                }
            }
            const double n0 = kBoltzmannT0 * std::pow(10.0, rad->spec().noiseFigureDb / 10.0);
            const double factor = 1.0 + totalJ0 / std::max(n0, 1e-30);
            w.jammerRatioDb = static_cast<float>(std::max(0.0, 10.0 * std::log10(factor)));
            w.burnThroughRangeM = static_cast<float>(detM * std::pow(factor, -0.25));
            if (w.jammerRatioDb > 0.5f) w.flags |= ipc::kTactRadarJammed;

            if (ipc::validTacticalRadarV1(w)) out.radars.push_back(w);

            // Additive radar tracks & recent raw detections (including electronic false targets / chaff / CFAR false alarms)
            // so replay Fog-of-War displays strictly radar-detected contacts without ever consulting omniscient truth.
            for (const auto& tr : tracks) {
                if (out.radarTracks.size() >= ipc::kMaxTacticalRadarTracksV1) break;
                ipc::TacticalRadarTrackWireV1 tk{};
                tk.slot = static_cast<std::uint32_t>(s);
                tk.vehicleId = e->info.id;
                tk.payloadIndex = p.index;
                tk.trackId = static_cast<std::uint32_t>(tr.id & 0xFFFFFFFFu);
                tk.positionEcef[0] = tr.stateEcef[0];
                tk.positionEcef[1] = tr.stateEcef[2];
                tk.positionEcef[2] = tr.stateEcef[4];
                tk.rangeM = static_cast<float>(std::max(0.0, tr.rangeM));
                tk.rangeRateMs = static_cast<float>(tr.rangeRateMs);
                tk.flags = ipc::kTactTrackConfirmed;
                if (st.designatedTrack != 0 && tr.id == st.designatedTrack) tk.flags |= ipc::kTactTrackDesignated;
                if (tr.truth == 0 || tr.truth >= ms::kDecoyIds) tk.flags |= ipc::kTactTrackFalseTarget;
                if (ipc::validTacticalRadarTrackV1(tk)) out.radarTracks.push_back(tk);
            }
            for (const auto& det : rad->detections()) {
                if (out.radarTracks.size() >= ipc::kMaxTacticalRadarTracksV1) break;
                if (!std::isfinite(det.timeS) || simTime_ - det.timeS > 2.5 || det.timeS > simTime_ + 0.1) continue;
                const double cEl = std::cos(det.elevationRad);
                const double bVec[3] = {
                    det.rangeM * cEl * std::cos(det.azimuthRad),
                    det.rangeM * cEl * std::sin(det.azimuthRad),
                    -det.rangeM * std::sin(det.elevationRad)};
                double relEcef[3] = {};
                ms::geom::toEcef(own, bVec, relEcef);
                const double px = own.positionEcef[0] + relEcef[0];
                const double py = own.positionEcef[1] + relEcef[1];
                const double pz = own.positionEcef[2] + relEcef[2];
                bool nearTrack = false;
                for (const auto& existing : out.radarTracks) {
                    if (existing.slot != static_cast<std::uint32_t>(s)) continue;
                    const double dx = existing.positionEcef[0] - px;
                    const double dy = existing.positionEcef[1] - py;
                    const double dz = existing.positionEcef[2] - pz;
                    if (dx * dx + dy * dy + dz * dz < 650.0 * 650.0) { nearTrack = true; break; }
                }
                if (nearTrack) continue;
                ipc::TacticalRadarTrackWireV1 dk{};
                dk.slot = static_cast<std::uint32_t>(s);
                dk.vehicleId = e->info.id;
                dk.payloadIndex = p.index;
                dk.trackId = 0u;
                dk.positionEcef[0] = px;
                dk.positionEcef[1] = py;
                dk.positionEcef[2] = pz;
                dk.rangeM = static_cast<float>(std::max(0.0, det.rangeM));
                dk.rangeRateMs = static_cast<float>(det.rangeRateMs);
                dk.snrDb = static_cast<float>(std::isfinite(det.snrDb) ? det.snrDb : 0.0);
                dk.flags = ipc::kTactTrackRawPlot | ((det.truth == 0 || det.truth >= ms::kDecoyIds) ? ipc::kTactTrackFalseTarget : 0u);
                if (ipc::validTacticalRadarTrackV1(dk)) out.radarTracks.push_back(dk);
            }
        }
    }

    // 2. Active EA Jammer Lobes
    for (const ms::Emission& em : emissions_) {
        if (out.jammers.size() >= ipc::kMaxTacticalJammersV1) break;
        if (em.mode != ms::EmitterMode::Jammer || !em.state || !(em.powerW > 0.0)) continue;
        ipc::TacticalJammerLobeWireV1 j{};
        j.vehicleId = em.entity;
        if (const Entry* je = entry(em.entity)) j.slot = static_cast<std::uint32_t>(je->slot);
        std::copy_n(em.state->positionEcef, 3, j.originEcef);
        const double cb = std::cos(em.elevationCentreRad);
        const double bodyDir[3] = {cb * std::cos(em.azimuthCentreRad), cb * std::sin(em.azimuthCentreRad), -std::sin(em.elevationCentreRad)};
        ms::geom::toEcef(*em.state, bodyDir, j.boresightEcef);
        const double bn = ms::geom::norm3(j.boresightEcef);
        if (bn > 1e-9) {
            for (double& v : j.boresightEcef) v /= bn;
        } else {
            j.boresightEcef[0] = 1.0, j.boresightEcef[1] = 0.0, j.boresightEcef[2] = 0.0;
        }
        j.halfAngleRad = static_cast<float>(em.allRound ? kPi : std::clamp(em.azimuthHalfWidthRad, 0.05, 1.4));
        const double erpDbw = 10.0 * std::log10(em.powerW) + em.peakGainDbi;
        j.erpDbw = static_cast<float>(erpDbw);
        j.rangeM = static_cast<float>(std::clamp(25000.0 * std::pow(10.0, (erpDbw - 40.0) / 40.0), 25000.0, 220000.0));
        j.frequencyGhz = static_cast<float>(em.frequencyHz * 1e-9);
        j.bandwidthMhz = static_cast<float>(em.bandwidthHz * 1e-6);
        j.flags = (em.allRound ? ipc::kTactJamAllRound : 0u) | (em.frequencyHz < 2.0e9 ? ipc::kTactJamComms : 0u);
        if (ipc::validTacticalJammerV1(j)) out.jammers.push_back(j);
    }

    // 3. ESM / RWR Bearing-Only DF Rays & 3-D Gauss-Newton FIM CRLB Geolocation Ellipsoids
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        const Entry* e = entries_[s].get();
        if (!e || !e->info.alive || states[s].crashed) continue;
        const sim::VehicleState& own = states[s];
        for (const auto& p : e->ms.payloads()) {
            if (p.family != ms::Family::Esm || p.state != ms::PayloadState::Operate) continue;
            const auto contacts = e->ms.contacts(p.index);
            for (const ms::Contact& c : contacts) {
                if (out.esmRays.size() >= ipc::kMaxTacticalEsmRaysV1) break;
                ipc::TacticalEsmRayWireV1 r{};
                r.observerSlot = static_cast<std::uint32_t>(s);
                r.observerVehicleId = e->info.id;
                std::copy_n(own.positionEcef, 3, r.originEcef);
                const double cb = std::cos(c.elevationRad);
                const double bDir[3] = {cb * std::cos(c.azimuthRad), cb * std::sin(c.azimuthRad), -std::sin(c.elevationRad)};
                double rayE[3];
                ms::geom::toEcef(own, bDir, rayE);
                const double rn = ms::geom::norm3(rayE);
                if (!(rn > 1e-9)) continue;
                for (int k = 0; k < 3; ++k) r.rayDirEcef[k] = static_cast<float>(rayE[k] / rn);
                r.receivedDbm = static_cast<float>(std::isfinite(c.strengthDb) ? c.strengthDb : -70.0);
                r.emitterMode = static_cast<std::uint8_t>(c.mode);
                r.flags = (c.warning ? ipc::kTactEsmWarning : 0u) | (c.mode == ms::EmitterMode::Datalink ? ipc::kTactEsmComint : 0u);
                if (std::isfinite(c.rangeM) && c.rangeM > 0.0 && std::isfinite(c.latitudeRad) && std::isfinite(c.longitudeRad)) {
                    r.flags |= ipc::kTactEsmHasGeo;
                    ms::rf::geodeticToEcef(c.latitudeRad, c.longitudeRad, std::isfinite(c.heightM) ? c.heightM : 0.0, r.geoPosEcef);
                    r.rayLengthM = static_cast<float>(std::clamp(c.rangeM, 500.0, 400000.0));
                    const double cep = (std::isfinite(c.sizeM) && c.sizeM > 0.0)
                                           ? c.sizeM
                                           : std::max(100.0, c.rangeM * std::max(c.bearingSigmaRad, 0.005));
                    r.crlbCep50M = static_cast<float>(cep);
                    r.crlbVertM = static_cast<float>(cep * 0.65);
                } else {
                    const double estLen = 120000.0 * std::pow(10.0, std::clamp((static_cast<double>(r.receivedDbm) + 75.0) / 40.0, -0.5, 0.5));
                    r.rayLengthM = static_cast<float>(std::clamp(estLen, 25000.0, 200000.0));
                }
                if (ipc::validTacticalEsmRayV1(r)) out.esmRays.push_back(r);
            }
        }
    }

    // 4. Link 16 / DMS Pulses, C2 Consent Arcs, Mode 5 IFF Rings, and PNT GNSS Drift Rings
    if (dms_.active()) {
        std::vector<Dms::CarrierEmission> carriers;
        dms_.activeCarriers(simTime_, 2.0, carriers);
        const auto dmsEnts = dms_.entities();
        for (const auto& c : carriers) {
            if (out.links.size() >= ipc::kMaxTacticalLinksV1) break;
            double fromPos[3];
            std::uint32_t fromSlot = 0;
            if (!entityEcef(c.entity, fromPos, fromSlot)) continue;
            const comm::SystemId txSys = dms_.systemOf(c.entity);
            bool emittedPeer = false;
            for (const std::uint32_t rxEnt : dmsEnts) {
                if (rxEnt == c.entity || out.links.size() >= ipc::kMaxTacticalLinksV1) continue;
                const auto* peers = dms_.peersOf(rxEnt);
                if (!peers) continue;
                for (const auto& pr : *peers) {
                    if (pr.system != txSys || !std::isfinite(pr.receivedS)) continue;
                    const double age = simTime_ - pr.receivedS;
                    if (age < 0.0 || age > 3.0) continue;
                    double toPos[3];
                    std::uint32_t toSlot = 0;
                    if (!entityEcef(rxEnt, toPos, toSlot)) continue;
                    ipc::TacticalLinkPulseWireV1 l{};
                    std::copy_n(fromPos, 3, l.fromEcef);
                    std::copy_n(toPos, 3, l.toEcef);
                    l.fromSlot = fromSlot;
                    l.toSlot = toSlot;
                    l.fromVehicleId = c.entity;
                    l.toVehicleId = rxEnt;
                    l.ageS = static_cast<float>(age);
                    l.durationS = 3.0f;
                    l.param = static_cast<float>(c.frequencyHz * 1e-6);
                    l.kind = static_cast<std::uint8_t>(ipc::TacticalLinkKindV1::Link16Dms);
                    if (ipc::validTacticalLinkPulseV1(l)) {
                        out.links.push_back(l);
                        emittedPeer = true;
                    }
                    break;
                }
            }
            if (!emittedPeer) {
                const auto txNets = dms_.netsOf(c.entity);
                for (const std::uint32_t rxEnt : dmsEnts) {
                    if (rxEnt == c.entity || out.links.size() >= ipc::kMaxTacticalLinksV1) continue;
                    const auto rxNets = dms_.netsOf(rxEnt);
                    bool shared = false;
                    for (const auto& tn : txNets)
                        for (const auto& rn : rxNets)
                            if (tn.id == rn.id) shared = true;
                    if (!shared) continue;
                    double toPos[3];
                    std::uint32_t toSlot = 0;
                    if (!entityEcef(rxEnt, toPos, toSlot)) continue;
                    ipc::TacticalLinkPulseWireV1 l{};
                    std::copy_n(fromPos, 3, l.fromEcef);
                    std::copy_n(toPos, 3, l.toEcef);
                    l.fromSlot = fromSlot;
                    l.toSlot = toSlot;
                    l.fromVehicleId = c.entity;
                    l.toVehicleId = rxEnt;
                    l.ageS = static_cast<float>(std::fmod(std::max(0.0, simTime_), 2.0));
                    l.durationS = 2.0f;
                    l.param = static_cast<float>(c.frequencyHz * 1e-6);
                    l.kind = static_cast<std::uint8_t>(ipc::TacticalLinkKindV1::Link16Dms);
                    if (ipc::validTacticalLinkPulseV1(l)) out.links.push_back(l);
                }
            }
        }
    }

    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        const Entry* e = entries_[s].get();
        if (!e || !e->info.alive || states[s].crashed) continue;
        const sim::VehicleState& own = states[s];

        // C2 consent requests awaiting approval
        std::vector<ms::ActivityId> awaiting;
        e->ms.awaitingConsent(awaiting);
        if (!awaiting.empty() && out.links.size() < ipc::kMaxTacticalLinksV1) {
            ipc::TacticalLinkPulseWireV1 l{};
            std::copy_n(own.positionEcef, 3, l.fromEcef);
            std::copy_n(own.positionEcef, 3, l.toEcef);
            l.fromSlot = static_cast<std::uint32_t>(s);
            l.fromVehicleId = e->info.id;
            for (std::size_t ci = 0; ci < c2Nodes_.size(); ++ci) {
                if (!c2Nodes_[ci].alive) continue;
                ms::rf::geodeticToEcef(c2Nodes_[ci].spec.latitudeRad, c2Nodes_[ci].spec.longitudeRad, c2Nodes_[ci].spec.altitudeMslM, l.toEcef);
                l.toVehicleId = kC2Nodes + static_cast<std::uint32_t>(ci) + 1u;
                break;
            }
            l.ageS = static_cast<float>(std::fmod(std::max(0.0, simTime_), 2.0));
            l.durationS = 2.0f;
            l.kind = static_cast<std::uint8_t>(ipc::TacticalLinkKindV1::C2Consent);
            if (ipc::validTacticalLinkPulseV1(l)) out.links.push_back(l);
        }

        for (const auto& p : e->ms.payloads()) {
            if (p.state != ms::PayloadState::Operate) continue;
            if (p.family == ms::Family::CommTerminal && out.links.size() < ipc::kMaxTacticalLinksV1) {
                const auto rep = e->ms.report(p.index);
                const bool mode5 = findReportVal(rep, "iff_mode5", 0.0) > 0.5;
                const double iffCount = findReportVal(rep, "iff_in_range", 0.0);
                const bool jammed = findReportVal(rep, "jammed", 0.0) > 0.5;
                if (mode5 && iffCount > 0.5) {
                    ipc::TacticalLinkPulseWireV1 ring{};
                    std::copy_n(own.positionEcef, 3, ring.fromEcef);
                    std::copy_n(own.positionEcef, 3, ring.toEcef);
                    ring.fromSlot = static_cast<std::uint32_t>(s);
                    ring.fromVehicleId = e->info.id;
                    ring.ageS = static_cast<float>(std::fmod(std::max(0.0, simTime_), 3.0));
                    ring.durationS = 3.0f;
                    ring.param = 90000.0f;
                    ring.kind = static_cast<std::uint8_t>(ipc::TacticalLinkKindV1::Mode5IffInterrogate);
                    ring.flags = jammed ? ipc::kTactLinkJammed : 0u;
                    if (ipc::validTacticalLinkPulseV1(ring)) out.links.push_back(ring);

                    int replies = 0;
                    for (std::size_t o = 0; o < entries_.size() && o < states.size() && replies < 4 && out.links.size() < ipc::kMaxTacticalLinksV1; ++o) {
                        const Entry* oe = entries_[o].get();
                        if (!oe || oe == e || !oe->info.alive || oe->info.role != ms::Role::Aircraft || states[o].crashed) continue;
                        double los[3];
                        const double r = ms::geom::lineOfSight(own, states[o].positionEcef, los);
                        if (!(r > 1.0 && r <= 180000.0)) continue;
                        ipc::TacticalLinkPulseWireV1 repPulse{};
                        std::copy_n(states[o].positionEcef, 3, repPulse.fromEcef);
                        std::copy_n(own.positionEcef, 3, repPulse.toEcef);
                        repPulse.fromSlot = static_cast<std::uint32_t>(o);
                        repPulse.toSlot = static_cast<std::uint32_t>(s);
                        repPulse.fromVehicleId = oe->info.id;
                        repPulse.toVehicleId = e->info.id;
                        repPulse.ageS = static_cast<float>(std::fmod(std::max(0.0, simTime_) + 0.35 * replies, 3.0));
                        repPulse.durationS = 3.0f;
                        repPulse.param = static_cast<float>(r);
                        repPulse.kind = static_cast<std::uint8_t>(ipc::TacticalLinkKindV1::Mode5IffReply);
                        if (ipc::validTacticalLinkPulseV1(repPulse)) {
                            out.links.push_back(repPulse);
                            ++replies;
                        }
                    }
                }
            } else if (p.family == ms::Family::Pnt && out.links.size() < ipc::kMaxTacticalLinksV1) {
                const auto rep = e->ms.report(p.index);
                if (findReportVal(rep, "valid", 0.0) > 0.5) {
                    const double errH = findReportVal(rep, "pos_error_h", 0.0);
                    const bool jammed = findReportVal(rep, "jammed", 0.0) > 0.5;
                    const bool locked = findReportVal(rep, "gnss_locked", 0.0) > 0.5;
                    if (errH > 0.1) {
                        ipc::TacticalLinkPulseWireV1 drift{};
                        std::copy_n(own.positionEcef, 3, drift.fromEcef);
                        std::copy_n(own.positionEcef, 3, drift.toEcef);
                        drift.fromSlot = static_cast<std::uint32_t>(s);
                        drift.fromVehicleId = e->info.id;
                        drift.ageS = 0.0f;
                        drift.durationS = 1.0f;
                        drift.param = static_cast<float>(std::clamp(errH, 0.5, 50000.0));
                        drift.kind = static_cast<std::uint8_t>(ipc::TacticalLinkKindV1::PntGnssDrift);
                        drift.flags = (jammed ? ipc::kTactLinkJammed : 0u) | (locked ? ipc::kTactLinkGnssLocked : 0u);
                        if (ipc::validTacticalLinkPulseV1(drift)) out.links.push_back(drift);
                    }
                }
            }
        }
    }

    // 5. Dynamic 5-D launchZoneV2 DLZ + McCoy/STANAG 4355 Gun LCOS + Wind-Compensated Bomb CCIP
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        if (out.weapons.size() >= ipc::kMaxTacticalWeaponsV1) break;
        const Entry* e = entries_[s].get();
        if (!e || !e->info.alive || e->info.role != ms::Role::Aircraft || states[s].crashed) continue;
        const sim::VehicleState& own = states[s];

        ipc::TacticalWeaponCueWireV1 w{};
        w.slot = static_cast<std::uint32_t>(s);
        w.vehicleId = e->info.id;

        // Resolve own tracked air target (strictly from own radar or fused tracks — never omniscient truth)
        std::uint32_t targetId = 0;
        double targetPos[3] = {0.0, 0.0, 0.0};
        double targetVel[3] = {0.0, 0.0, 0.0};
        bool haveTarget = false;
        for (const auto& p : e->ms.payloads()) {
            if (p.family != ms::Family::AirRadar) continue;
            const ms::RadarStatus st = e->ms.radarStatus(p.index);
            if (!st.searching) continue;
            const auto tracks = e->ms.amtiTracks(p.index, simTime_, own);
            const ms::AmtiTrack* best = nullptr;
            if (st.designatedTrack != 0) {
                for (const auto& tr : tracks)
                    if (tr.id == st.designatedTrack) { best = &tr; break; }
            }
            if (!best && !tracks.empty()) best = &tracks.front();
            if (best) {
                targetId = best->truth;
                targetPos[0] = best->stateEcef[0];
                targetPos[1] = best->stateEcef[2];
                targetPos[2] = best->stateEcef[4];
                targetVel[0] = best->stateEcef[1];
                targetVel[1] = best->stateEcef[3];
                targetVel[2] = best->stateEcef[5];
                haveTarget = true;
                break;
            }
        }
        if (!haveTarget) {
            const auto fused = e->fusion.snapshot();
            if (!fused.empty()) {
                const auto& fe = fused.front();
                targetPos[0] = fe.stateEcef[0];
                targetPos[1] = fe.stateEcef[2];
                targetPos[2] = fe.stateEcef[4];
                targetVel[0] = fe.stateEcef[1];
                targetVel[1] = fe.stateEcef[3];
                targetVel[2] = fe.stateEcef[5];
                haveTarget = true;
            }
        }

        // 5a. Flyout-derived 5-D launchZoneV2 DLZ
        if (haveTarget) {
            std::copy_n(targetPos, 3, w.targetEcef);
            if (targetId != 0) {
                for (const auto& stInfo : e->ms.stores()) {
                    if (!stInfo.storeClass || std::strcmp(stInfo.storeClass, "aam") != 0) continue;
                    if (stInfo.status != ms::StoreStatus::Loaded && stInfo.status != ms::StoreStatus::PoweringUp &&
                        stInfo.status != ms::StoreStatus::Ready)
                        continue;
                    const ms::LaunchZoneV2 v2 = launchZoneV2(e->info.id, stInfo.store, targetId);
                    if (!v2.valid) continue;
                    w.rAeroM = static_cast<float>(std::max(v2.maximumM * 1.15, v2.maximumM));
                    w.rMaxM = static_cast<float>(std::max(0.0, v2.maximumM));
                    w.rNeM = static_cast<float>(std::max(0.0, v2.noEscapeM));
                    w.rMinM = static_cast<float>(std::max(0.0, v2.minimumM));
                    copyName(w.storeName, sizeof w.storeName, stInfo.mnemonic && stInfo.mnemonic[0] ? stInfo.mnemonic : stInfo.store);
                    w.flags |= ipc::kTactWeaponHasDlz;
                    if (v2.inRange) w.flags |= ipc::kTactWeaponInRange;
                    if (v2.inNoEscape) w.flags |= ipc::kTactWeaponInNez;
                    break;
                }
            }
        }

        // 5b. Native McCoy / STANAG 4355 ballistic gun LCOS lead aimpoint
        for (const auto& p : e->ms.payloads()) {
            if (p.family != ms::Family::Gun) continue;
            const ms::Card* card = e->ms.card(p.index);
            if (!card) continue;
            const auto rep = e->ms.report(p.index);
            const double roundsLeft = findReportVal(rep, "rounds_remaining", 1.0);
            if (!(roundsLeft > 0.0)) continue;
            const ms::gun::Spec gspec = ms::gun::specOf(*card, p.count);
            const double dragK = ms::gun::dragRetardationPerM(gspec.calibreM, gspec.projectileKg, gspec.dragCd);
            w.gunMuzzleSpeedMs = static_cast<float>(gspec.muzzleMs);
            w.gunDragK = static_cast<float>(dragK);

            const double ownNorm = ms::geom::norm3(own.positionEcef);
            const double upHat[3] = {
                ownNorm > 1.0 ? own.positionEcef[0] / ownNorm : 0.0,
                ownNorm > 1.0 ? own.positionEcef[1] / ownNorm : 0.0,
                ownNorm > 1.0 ? own.positionEcef[2] / ownNorm : 1.0};
            double ownVel[3];
            ms::geom::velocityEcef(own, ownVel);
            const double rhoRatio = ms::geom::densityRatio(own.altitudeMslM);
            const double v0 = std::max(gspec.muzzleMs, 200.0);

            double refPos[3] = {};
            double refVel[3] = {};
            double slantM = 0.0;
            bool computeLcos = false;
            if (haveTarget) {
                double los[3];
                slantM = ms::geom::lineOfSight(own, targetPos, los);
                if (slantM >= 50.0 && slantM <= 4500.0) {
                    std::copy_n(targetPos, 3, refPos);
                    std::copy_n(targetVel, 3, refVel);
                    computeLcos = true;
                }
            }
            if (!computeLcos && !own.onGround && (p.state == ms::PayloadState::Operate || findReportVal(rep, "firing", 0.0) > 0.5)) {
                // Nominal 800 m boresight ballistic drop reticle when gun is armed without close air target
                const double fwdBody[3] = {std::cos(gspec.elevationRad), 0.0, -std::sin(gspec.elevationRad)};
                double fwdE[3];
                ms::geom::toEcef(own, fwdBody, fwdE);
                slantM = 800.0;
                for (int k = 0; k < 3; ++k) {
                    refPos[k] = own.positionEcef[k] + fwdE[k] * slantM;
                    refVel[k] = ownVel[k];
                }
                computeLcos = true;
            }
            if (computeLcos) {
                double tof = slantM / v0;
                for (int it = 0; it < 3; ++it) {
                    const double vAvg = v0 / (1.0 + 0.5 * dragK * rhoRatio * v0 * tof);
                    const double pred[3] = {
                        refPos[0] + (refVel[0] - ownVel[0]) * tof - own.positionEcef[0],
                        refPos[1] + (refVel[1] - ownVel[1]) * tof - own.positionEcef[1],
                        refPos[2] + (refVel[2] - ownVel[2]) * tof - own.positionEcef[2]};
                    tof = std::clamp(ms::geom::norm3(pred) / std::max(vAvg, 120.0), 0.02, 6.0);
                }
                for (int k = 0; k < 3; ++k)
                    w.lcosPipperEcef[k] = refPos[k] + (refVel[k] - ownVel[k]) * tof + 0.5 * 9.80665 * upHat[k] * tof * tof;
                w.flags |= ipc::kTactWeaponHasLcos;
            }
            break;
        }

        // 5c. Wind-compensated Mk 82 / gravity bomb CCIP terrain impact prediction
        if (!own.onGround && own.altitudeAglM > 20.0) {
            const ms::StoreInfo* bombStore = nullptr;
            for (const auto& stInfo : e->ms.stores()) {
                if (!stInfo.storeClass || std::strcmp(stInfo.storeClass, "bomb") != 0) continue;
                if (stInfo.status == ms::StoreStatus::Loaded || stInfo.status == ms::StoreStatus::PoweringUp ||
                    stInfo.status == ms::StoreStatus::Ready) {
                    bombStore = &stInfo;
                    break;
                }
            }
            if (bombStore) {
                double pos[3] = {own.positionEcef[0], own.positionEcef[1], own.positionEcef[2]};
                double vel[3];
                ms::geom::velocityEcef(own, vel);
                const double sl = std::sin(own.latitudeRad), cl = std::cos(own.latitudeRad);
                const double so = std::sin(own.longitudeRad), co = std::cos(own.longitudeRad);
                const double up[3] = {cl * co, cl * so, sl};
                const double east[3] = {-so, co, 0.0};
                const double north[3] = {-sl * co, -sl * so, cl};
                const double windRad = environment_.windDirectionDeg * (kPi / 180.0);
                const double windToN = -environment_.windSpeedMs * std::cos(windRad);
                const double windToE = -environment_.windSpeedMs * std::sin(windRad);
                const double windEcef[3] = {
                    windToN * north[0] + windToE * east[0],
                    windToN * north[1] + windToE * east[1],
                    windToN * north[2] + windToE * east[2]};
                const double dtB = 0.20;
                double altM = own.altitudeMslM;
                const double gndH = own.altitudeMslM - own.altitudeAglM;
                bool hit = false;
                for (int step = 0; step < 240; ++step) {
                    const double relV[3] = {vel[0] - windEcef[0], vel[1] - windEcef[1], vel[2] - windEcef[2]};
                    const double speed = ms::geom::norm3(relV);
                    const double kDrag = 0.00018 * ms::geom::densityRatio(altM);
                    double prevPos[3] = {pos[0], pos[1], pos[2]};
                    for (int k = 0; k < 3; ++k) {
                        const double acc = -9.80665 * up[k] - kDrag * speed * relV[k];
                        vel[k] += acc * dtB;
                        pos[k] += vel[k] * dtB;
                    }
                    constexpr double kA = 6378137.0, kB = 6356752.314245, kE2 = 1.0 - (kB * kB) / (kA * kA);
                    constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);
                    const double rhoP = std::hypot(pos[0], pos[1]);
                    const double theta = std::atan2(pos[2] * kA, std::max(rhoP, 1e-6) * kB);
                    const double stH = std::sin(theta), ctH = std::cos(theta);
                    const double lat = std::atan2(pos[2] + kEp2 * kB * stH * stH * stH,
                                                  std::max(rhoP, 1e-6) - kE2 * kA * ctH * ctH * ctH);
                    const double lon = std::atan2(pos[1], pos[0]);
                    const double slL = std::sin(lat), clL = std::cos(lat);
                    const double N = kA / std::sqrt(1.0 - kE2 * slL * slL);
                    const double h = (std::abs(clL) > 1e-4) ? (rhoP / clL - N) : (std::abs(pos[2]) / std::max(std::abs(slL), 1e-6) - N * (1.0 - kE2));
                    double terrainH = gndH;
                    if (ground_) {
                        const double th = ground_->pointHeightAboveEllipsoidM(lat, lon);
                        if (std::isfinite(th)) terrainH = th;
                    }
                    if (h <= terrainH) {
                        const double denom = std::max(1e-3, altM - h);
                        const double frac = std::clamp((altM - terrainH) / denom, 0.0, 1.0);
                        for (int k = 0; k < 3; ++k) w.ccipImpactEcef[k] = prevPos[k] + (pos[k] - prevPos[k]) * frac;
                        hit = true;
                        break;
                    }
                    altM = h;
                }
                if (hit) {
                    w.flags |= ipc::kTactWeaponHasCcip;
                    if (w.storeName[0] == '\0')
                        copyName(w.storeName, sizeof w.storeName, bombStore->mnemonic && bombStore->mnemonic[0] ? bombStore->mnemonic : bombStore->store);
                }
            }
        }

        if (w.flags != 0u && ipc::validTacticalWeaponCueV1(w)) out.weapons.push_back(w);
    }

    // 6. SAR Spotlight Footprints & Weather Radar Precipitation Cells
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        const Entry* e = entries_[s].get();
        if (!e || !e->info.alive || states[s].crashed) continue;
        for (const auto& p : e->ms.payloads()) {
            if (p.state != ms::PayloadState::Operate) continue;
            if (p.family != ms::Family::Sar && p.family != ms::Family::WeatherRadar) continue;
            const auto contacts = e->ms.contacts(p.index);
            const auto rep = e->ms.report(p.index);
            const bool sarJammed = (p.family == ms::Family::Sar) && (findReportVal(rep, "jam_degradation", 0.0) > 1.0);
            for (const ms::Contact& c : contacts) {
                if (out.swaths.size() >= ipc::kMaxTacticalSwathsV1) break;
                if (!std::isfinite(c.latitudeRad) || !std::isfinite(c.longitudeRad)) continue;
                ipc::TacticalSwathWireV1 sw{};
                sw.slot = static_cast<std::uint32_t>(s);
                sw.vehicleId = e->info.id;
                const double hM = std::isfinite(c.heightM) ? c.heightM : 0.0;
                ms::rf::geodeticToEcef(c.latitudeRad, c.longitudeRad, hM, sw.centerEcef);
                const double sl = std::sin(c.latitudeRad), cl = std::cos(c.latitudeRad);
                const double so = std::sin(c.longitudeRad), co = std::cos(c.longitudeRad);
                sw.axisEastEcef[0] = static_cast<float>(-so);
                sw.axisEastEcef[1] = static_cast<float>(co);
                sw.axisEastEcef[2] = 0.0f;
                sw.axisNorthEcef[0] = static_cast<float>(-sl * co);
                sw.axisNorthEcef[1] = static_cast<float>(-sl * so);
                sw.axisNorthEcef[2] = static_cast<float>(cl);
                if (p.family == ms::Family::Sar) {
                    sw.kind = static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::SarSpotlight);
                    const double res = std::isfinite(c.sizeM) && c.sizeM > 0.0 ? c.sizeM : 1.0;
                    const float half = static_cast<float>(std::clamp(std::max(800.0, res * 400.0), 800.0, 5000.0));
                    sw.halfWidthM = half;
                    sw.halfLengthM = half;
                    sw.metricDb = static_cast<float>(std::isfinite(c.strengthDb) ? c.strengthDb : -25.0);
                    sw.resolutionM = static_cast<float>(res);
                    sw.flags = sarJammed ? ipc::kTactSwathJammed : 0u;
                } else {
                    sw.kind = static_cast<std::uint8_t>(ipc::TacticalSwathKindV1::WeatherCell);
                    const float half = static_cast<float>(std::clamp(std::isfinite(c.sizeM) ? 0.5 * c.sizeM : 3000.0, 1200.0, 8000.0));
                    sw.halfWidthM = half;
                    sw.halfLengthM = half;
                    sw.metricDb = static_cast<float>(std::isfinite(c.strengthDb) ? c.strengthDb : 20.0);
                    sw.resolutionM = static_cast<float>(std::max(0.0, hM));
                }
                if (ipc::validTacticalSwathV1(sw)) out.swaths.push_back(sw);
            }
        }
    }

    // 7. Extended 8-Engine & Rotorcraft/Propeller Propulsion Telemetry (B-57 & B-59, zero break to VehicleState)
    for (std::size_t s = 0; s < entries_.size() && s < states.size(); ++s) {
        if (out.propulsion.size() >= ipc::kMaxTacticalPropulsionV1) break;
        const Entry* e = entries_[s].get();
        if (!e || !e->info.alive || e->info.role != ms::Role::Aircraft || states[s].crashed) continue;
        const sim::VehicleState& own = states[s];
        const auto* extSrc = dynamic_cast<const sim::PropulsionExtendedSourceV1*>(&pool_->vehicle(s));
        sim::PropulsionExtendedV1 ext{};
        const bool haveExt = extSrc && extSrc->propulsionExtendedV1(ext);

        const std::string& ac = e->info.aircraft;
        const bool isRotor = e->host.performance().hovers ||
                             ac.find("uh1") != std::string::npos || ac.find("uh60") != std::string::npos ||
                             ac.find("ah64") != std::string::npos || ac.find("ch47") != std::string::npos ||
                             ac.find("mi24") != std::string::npos || ac.find("ec135") != std::string::npos;
        const bool isProp = !isRotor &&
                            (ac.find("c130") != std::string::npos || ac.find("c172") != std::string::npos ||
                             ac.find("mq9") != std::string::npos || ac.find("p3") != std::string::npos ||
                             ac.find("e2") != std::string::npos || ac.find("tu95") != std::string::npos ||
                             ac.find("an26") != std::string::npos || ac.find("a400") != std::string::npos);

        const std::uint8_t nEng = haveExt ? static_cast<std::uint8_t>(std::clamp(ext.engineCount, 0, static_cast<int>(ipc::kTacticalMaxEnginesV1)))
                                          : static_cast<std::uint8_t>(std::clamp(own.engineCount, 0, sim::VehicleState::kMaxEngines));
        if (nEng <= sim::VehicleState::kMaxEngines && !isRotor && !isProp) continue;

        ipc::TacticalPropulsionExtWireV1 pr{};
        pr.slot = static_cast<std::uint32_t>(s);
        pr.vehicleId = e->info.id;
        pr.engineCount = nEng;
        if (haveExt) {
            for (std::uint32_t k = 0; k < nEng; ++k) {
                pr.throttle[k] = static_cast<float>(std::isfinite(ext.throttlePosition[k]) ? ext.throttlePosition[k] : 0.0);
                pr.thrustN[k] = static_cast<float>(std::isfinite(ext.thrustN[k]) ? ext.thrustN[k] : 0.0);
                pr.engineN2[k] = static_cast<float>(std::isfinite(ext.engineN2[k]) ? ext.engineN2[k] : 0.0);
                pr.afterburner[k] = static_cast<float>(std::isfinite(ext.afterburner[k]) ? ext.afterburner[k] : 0.0);
                pr.fuelFlowKgS[k] = static_cast<float>(std::isfinite(ext.fuelFlowKgS[k]) ? ext.fuelFlowKgS[k] : 0.0);
            }
            if (nEng > 0 && std::isfinite(ext.engineRpm[0]) && ext.engineRpm[0] > 0.0)
                pr.rotorRpm = static_cast<float>(ext.engineRpm[0]);
        } else {
            for (std::uint32_t k = 0; k < nEng; ++k) {
                pr.throttle[k] = static_cast<float>(own.throttlePosition[k]);
                pr.thrustN[k] = static_cast<float>(own.thrustN[k]);
                pr.engineN2[k] = static_cast<float>(own.engineN2[k]);
                pr.afterburner[k] = static_cast<float>(own.afterburner[k]);
            }
            if (nEng > 0 && std::isfinite(own.engineRpm[0]) && own.engineRpm[0] > 0.0)
                pr.rotorRpm = static_cast<float>(own.engineRpm[0]);
        }
        if (isRotor) {
            pr.flags |= ipc::kTactPropRotorcraft;
            if (ac.find("uh60") != std::string::npos) pr.rotorRadiusM = 8.18f;
            else if (ac.find("uh1") != std::string::npos) pr.rotorRadiusM = 7.32f;
            else if (ac.find("ch47") != std::string::npos) pr.rotorRadiusM = 9.14f;
            else pr.rotorRadiusM = 7.0f;
            if (pr.rotorRpm < 1.0f && own.engineCount > 0 && own.engineN2[0] > 20.0)
                pr.rotorRpm = static_cast<float>((ac.find("uh60") != std::string::npos ? 258.0 : 324.0) * std::clamp(own.engineN2[0] / 100.0, 0.2, 1.1));
            const double massKg = std::isfinite(e->host.massKg()) && e->host.massKg() > 100.0 ? e->host.massKg() : 4500.0;
            const double rr = static_cast<double>(pr.rotorRadiusM);
            pr.diskLoadingPa = static_cast<float>((massKg * 9.80665) / (kPi * rr * rr));
        } else if (isProp) {
            pr.flags |= ipc::kTactPropPropeller;
            if (ac.find("c130") != std::string::npos) pr.rotorRadiusM = 2.06f;
            else if (ac.find("c172") != std::string::npos) pr.rotorRadiusM = 0.95f;
            else pr.rotorRadiusM = 1.8f;
        }
        if (detectImpactSurface(own) == ms::damage::SurfaceKind::Water) pr.flags |= ipc::kTactPropOverWater;
        if (ipc::validTacticalPropulsionExtV1(pr)) out.propulsion.push_back(pr);
    }

    return out;
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
