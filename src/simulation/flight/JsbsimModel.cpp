#include "simulation/flight/JsbsimModel.h"

#include "foundation/core/Log.h"
#include "foundation/core/Units.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/flight/JsbsimLog.h"

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <input_output/FGGroundCallback.h>
#include <input_output/FGPropertyManager.h>
#include <input_output/FGXMLElement.h>
#include <math/FGColumnVector3.h>
#include <math/FGLocation.h>
#include <math/FGMatrix33.h>
#include <math/FGQuaternion.h>
#include <models/FGAccelerations.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGExternalReactions.h>
#include <models/atmosphere/FGWinds.h>
#include <models/FGFCS.h>
#include <models/FGGroundReactions.h>
#include <models/FGInertial.h>
#include <models/FGMassBalance.h>
#include <models/FGOutput.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGTank.h>
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGTurbine.h>
#include <simgear/misc/sg_path.hxx>

#include <algorithm>
#include <cmath>

namespace fsim::sim {

namespace {

/// JSBSim ground callback backed by a GroundProvider (design 7.3). Modelled on
/// FGDefaultGroundCallback but with the terrain height taken from the provider
/// at the queried geodetic position. Feet in, feet out (JSBSim internals).
class ProviderGroundCallback final : public JSBSim::FGGroundCallback {
public:
    ProviderGroundCallback(std::shared_ptr<const GroundProvider> provider, double semiMajorFt, double semiMinorFt)
        : provider_(std::move(provider)), a_(semiMajorFt), b_(semiMinorFt) {}

    // JSBSim asks once per contact point per step. The geodetic conversion
    // (iterative) is done for the first point of a step only; the others -
    // metres away, on the same aircraft - are placed in its local east-north-up
    // frame, which is exact to d^2 / 2R (under 0.1 mm within kLocalFt). The
    // terrain is still sampled under every point.
    double GetAGLevel(double t, const JSBSim::FGLocation& loc, JSBSim::FGLocation& contact,
                      JSBSim::FGColumnVector3& normal, JSBSim::FGColumnVector3& vel,
                      JSBSim::FGColumnVector3& angularVel) const override {
        vel.InitMatrix();
        angularVel.InitMatrix();
        if (t != depthT_) depthT_ = t, deepestFt_ = std::numeric_limits<double>::infinity(); // (a new step's queries)

        const double x = loc(1), y = loc(2), z = loc(3);
        double lat, lon, alt;
        const double dx = x - x0_, dy = y - y0_, dz = z - z0_;
        if (t == t0_ && dx * dx + dy * dy + dz * dz < kLocalFt * kLocalFt) {
            const double e = -sinLon0_ * dx + cosLon0_ * dy;
            const double n = -sinLat0_ * cosLon0_ * dx - sinLat0_ * sinLon0_ * dy + cosLat0_ * dz;
            const double u = cosLat0_ * cosLon0_ * dx + cosLat0_ * sinLon0_ * dy + sinLat0_ * dz;
            lat = lat0_ + n / feetPerRadLat_;
            lon = lon0_ + e / feetPerRadLon_;
            alt = alt0_ + u;
        } else {
            JSBSim::FGLocation l = loc;
            l.SetEllipse(a_, b_);
            lat = l.GetGeodLatitudeRad();
            lon = l.GetLongitude();
            alt = l.GetGeodAltitude();
            t0_ = t;
            x0_ = x, y0_ = y, z0_ = z;
            lat0_ = lat, lon0_ = lon, alt0_ = alt;
            sinLat0_ = std::sin(lat), cosLat0_ = std::cos(lat), sinLon0_ = std::sin(lon), cosLon0_ = std::cos(lon);
            // radii of curvature (feet): meridian M and prime vertical N, at the point's height
            const double e2 = 1.0 - (b_ * b_) / (a_ * a_);
            const double w = std::sqrt(1.0 - e2 * sinLat0_ * sinLat0_);
            feetPerRadLat_ = a_ * (1.0 - e2) / (w * w * w) + alt;
            feetPerRadLon_ = (a_ / w + alt) * cosLat0_;
        }
        const double cosLat = std::cos(lat);
        normal = JSBSim::FGColumnVector3(cosLat * std::cos(lon), cosLat * std::sin(lon), std::sin(lat));

        const double terrainFt = units::metresToFeet(provider_->heightAboveEllipsoidM(lat, lon));
        contact.SetEllipse(a_, b_);
        contact.SetPositionGeodetic(lon, lat, terrainFt);
        deepestFt_ = std::min(deepestFt_, alt - terrainFt);
        return alt - terrainFt;
    }

    /// The lowest a point it was asked about stood above the terrain under it, in the last step's queries (feet; + above):
    /// its CG's and every contact point's - the gear's and the structure's.
    double deepestFt() const noexcept { return deepestFt_; }

    /// Drops the step's frame: the next query starts a new one, as a new
    /// callback's first does.
    void forget() noexcept { t0_ = -1.0, depthT_ = -1.0, deepestFt_ = std::numeric_limits<double>::infinity(); }

    // JSBSim may try to move the terrain (IC files, scripts); the provider is
    // authoritative, so these are deliberately ignored.
    void SetTerrainElevation(double) override {}
    void SetEllipse(double semimajor, double semiminor) override {
        a_ = semimajor;
        b_ = semiminor;
    }

private:
    static constexpr double kLocalFt = 100.0; // points this close to a step's first share its frame

    std::shared_ptr<const GroundProvider> provider_;
    double a_, b_;
    // The step's first point (ECEF feet) and its local frame; one vehicle's
    // callback is only ever called from the thread stepping that vehicle.
    mutable double t0_ = -1.0, x0_ = 0.0, y0_ = 0.0, z0_ = 0.0;
    mutable double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
    mutable double sinLat0_ = 0.0, cosLat0_ = 1.0, sinLon0_ = 0.0, cosLon0_ = 1.0;
    mutable double feetPerRadLat_ = 1.0, feetPerRadLon_ = 1.0;
    mutable double depthT_ = -1.0, deepestFt_ = std::numeric_limits<double>::infinity();
};

bool finite(double v) noexcept { return std::isfinite(v); }

constexpr double kGroundSpawnCgHeightM = 1.5; // CG start height for on-ground spawns before ground trim
constexpr double kStrikeDepthM = 1.0;          // a point this far into the ground under it: the vehicle struck the terrain

#ifdef _WIN32
constexpr const char* kNullDevice = "NUL";
#else
constexpr const char* kNullDevice = "/dev/null";
#endif

} // namespace

/// A property's value as the aircraft was loaded.
struct JsbsimModel::InitialProperty {
    SGPropertyNode* node;
    simgear::props::Type type;
    double value; ///< a double's or float's, else an integer's or a bool's, exactly
};

namespace {

void collectPlain(SGPropertyNode* n, std::vector<SGPropertyNode*>& out) {
    for (int i = 0; i < n->nChildren(); ++i) {
        SGPropertyNode* c = n->getChild(i);
        if (!c->isTied()) switch (c->getType()) {
            case simgear::props::BOOL:
            case simgear::props::INT:
            case simgear::props::LONG:
            case simgear::props::FLOAT:
            case simgear::props::DOUBLE: out.push_back(c); break;
            default: break;
            }
        if (c->nChildren() > 0) collectPlain(c, out);
    }
}

} // namespace

// JSBSim's reset (each model's InitModel) returns the models' own state, and
// with it the properties they tie, to where a load leaves them - but not the
// plain properties: what the flight control system's components and the
// aircraft's functions last wrote stays, and the first pass of the next start
// reads it before they write it again. So they are kept as loaded, and put
// back on reset, with the few tied ones JSBSim's reset leaves.
void JsbsimModel::saveInitialProperties() {
    std::vector<SGPropertyNode*> nodes;
    collectPlain(fdm_->GetPropertyManager()->GetNode(), nodes);
    // The flight control system's commands its own reset leaves (FGFCS::InitModel):
    // the gear (a new model's is down), the brakes and the propeller levers.
    auto pm = fdm_->GetPropertyManager();
    for (const char* path : {"gear/gear-cmd-norm", "gear/gear-pos-norm", "fcs/left-brake-cmd-norm", "fcs/right-brake-cmd-norm",
                             "fcs/center-brake-cmd-norm"})
        if (auto* n = pm->GetNode(path)) nodes.push_back(n);
    for (unsigned i = 0; i < fdm_->GetPropulsion()->GetNumEngines(); ++i)
        for (const char* path : {"fcs/advance-cmd-norm", "fcs/feather-cmd-norm"})
            if (auto* n = pm->GetNode(path, static_cast<int>(i))) nodes.push_back(n);
    initialProperties_.clear();
    initialProperties_.reserve(nodes.size());
    for (SGPropertyNode* n : nodes) {
        const auto type = n->getType();
        const double v = type == simgear::props::BOOL   ? (n->getBoolValue() ? 1.0 : 0.0)
                         : type == simgear::props::INT  ? static_cast<double>(n->getIntValue())
                         : type == simgear::props::LONG ? static_cast<double>(n->getLongValue())
                                                        : n->getDoubleValue();
        initialProperties_.push_back({n, type, v});
    }
}

void JsbsimModel::restoreInitialProperties() {
    for (const auto& p : initialProperties_) switch (p.type) {
        case simgear::props::BOOL: p.node->setBoolValue(p.value != 0.0); break;
        case simgear::props::INT: p.node->setIntValue(static_cast<int>(p.value)); break;
        case simgear::props::LONG: p.node->setLongValue(static_cast<long>(p.value)); break;
        case simgear::props::FLOAT: p.node->setFloatValue(static_cast<float>(p.value)); break;
        default: p.node->setDoubleValue(p.value); break;
        }
}

JsbsimModel::JsbsimModel(double dt, std::shared_ptr<const GroundProvider> ground)
    : dt_(dt), ground_(std::move(ground)) {
    installJsbsimLogBridge();

    // Quiet JSBSim's own console chatter before the first instance prints its banner.
    JSBSim::FGJSBBase::debug_lvl = 0;

    fdm_ = std::make_unique<JSBSim::FGFDMExec>();
    fdm_->SetDebugLevel(0);
    fdm_->Setdt(dt_);
    fdm_->DisableOutput();

    auto inertial = fdm_->GetInertial();
    groundCallback_ = new ProviderGroundCallback(ground_, inertial->GetSemimajor(), inertial->GetSemiminor());
    inertial->SetGroundCallback(groundCallback_);
}

JsbsimModel::~JsbsimModel() = default;

bool JsbsimModel::load(const AircraftSpec& aircraft, const InitialConditions& ic) {
    loaded_ = false;
    diverged_ = crashed_ = false;
    stepCount_ = 0;

    const SGPath root(aircraft.jsbsimRoot.string());
    fdm_->SetRootDir(root);
    fdm_->SetAircraftPath(aircraft.aircraftDir.empty() ? SGPath("aircraft") : SGPath(aircraft.aircraftDir.string()));
    fdm_->SetEnginePath(SGPath("engine"));
    fdm_->SetSystemsPath(SGPath("systems"));

    try {
        if (!fdm_->LoadModel(aircraft.name)) {
            LOG_ERROR("sim") << "JSBSim failed to load aircraft '" << aircraft.name << "' from "
                             << (aircraft.aircraftDir.empty() ? aircraft.jsbsimRoot / "aircraft" : aircraft.aircraftDir).string();
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception loading '" << aircraft.name << "': " << e.what();
        return false;
    } catch (...) {
        LOG_ERROR("sim") << "JSBSim unknown exception loading '" << aircraft.name << "'";
        return false;
    }

    silenceOutputs();
    installExternalReaction();
    cacheCommandNodes();
    saveInitialProperties();

    try {
        applyInitialConditions(ic);
        if (!fdm_->RunIC()) {
            LOG_ERROR("sim") << "JSBSim RunIC failed for '" << aircraft.name << "'";
            return false;
        }
        if (ic.enginesRunning) startEngines();
        settleOnGround(ic);
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception in initial conditions: " << e.what();
        return false;
    }

    loaded_ = true;
    {
        // its empty CG - its model's origin - from what its mass balance sums (W cg = EW base + point masses' moments
        // + tanks' + gas's)
        auto mb = fdm_->GetMassBalance();
        const JSBSim::FGColumnVector3 moment = mb->GetXYZcg() * mb->GetWeight() - mb->GetPointMassMoment() -
                                               fdm_->GetPropulsion()->GetTanksMoment() -
                                               mb->in.GasMoment;
        for (unsigned k = 0; k < 3; ++k) baseCgIn_[k] = mb->GetEmptyWeight() > 0.0 ? moment(k + 1) / mb->GetEmptyWeight() : mb->GetXYZcg(static_cast<int>(k + 1));
        // unless its file says where its model's origin is (fsim/model/origin-*-in, its structural frame): a missile's
        // is its CG as it is launched, its motor full - what its picture is drawn about and what a station holds -
        // while its empty CG is its structure's, burnt out (hangar's storebodies.py)
        auto pm = fdm_->GetPropertyManager();
        static const char* const kOrigin[3] = {"fsim/model/origin-x-in", "fsim/model/origin-y-in", "fsim/model/origin-z-in"};
        if (pm->GetNode(kOrigin[0], false))
            for (unsigned k = 0; k < 3; ++k)
                if (auto node = pm->GetNode(kOrigin[k], false)) baseCgIn_[k] = node->getDoubleValue();
    }
    checkSpawnInTerrain(ic);
    LOG_DEBUG("sim") << "loaded '" << aircraft.name << "', " << fdm_->GetPropulsion()->GetNumEngines()
                     << " engine(s), dt=" << dt_;
    return true;
}

// A reset starts the vehicle as a load of the same aircraft with the same
// conditions does, to the bit, whatever the model flew before (design 7.2):
// the models are put back where a load leaves them, and then the start is a
// load's - the same conditions, the same passes, in the same order. A reused
// model then flies as a new one would, and an episode's start depends on its
// conditions alone.
bool JsbsimModel::reset(const InitialConditions& ic) {
    if (!loaded_) return false;
    diverged_ = crashed_ = false;
    stepCount_ = 0;
    try {
        // RunIC() reopens every output file; JSBSim only closes them when a
        // new output is started, so close them first (keeping the null name)
        // or the reopen fails and is logged on every reset.
        silenceOutputs();
        fdm_->GetOutput()->SetStartNewOutput();
        // Every model back to its initial state (InitModel), without the IC
        // pass; JSBSim is built with what its own reset missed (an actuator's
        // first run, the last lift, the stall hysteresis's limits and the
        // reference point's shift, the CG: cmake/JsbsimPatches.cmake).
        fdm_->ResetToInitialConditions(JSBSim::FGFDMExec::DONT_EXECUTE_RUN_IC);
        // What it leaves behind: a propeller's RPM (FGPropeller::ResetToIC
        // clears only the induced velocity; after a blow-up the start's passes
        // computed thrust from it), and properties (saveInitialProperties).
        auto propulsion = fdm_->GetPropulsion();
        for (unsigned i = 0; i < propulsion->GetNumEngines(); ++i)
            if (auto* thruster = propulsion->GetEngine(i)->GetThruster()) thruster->SetRPM(0.0);
        restoreInitialProperties();
        // what damage changed (an engine's seizure JSBSim's own reset clears)
        for (const auto& name : failedActuators_) fdm_->SetPropertyValue(name, 0.0);
        failedActuators_.clear();
        for (const int k : damageMasses_) fdm_->SetPropertyValue("inertia/pointmass-weight-lbs[" + std::to_string(k) + "]", 0.0);
        damageMassesUsed_ = 0;
        if (!loadedBaseJ_.empty()) {
            // (its own inertia back, as loaded)
            const auto& J = loadedBaseJ_;
            fdm_->GetMassBalance()->SetAircraftBaseInertias(JSBSim::FGMatrix33(J[0], J[1], J[2], J[3], J[4], J[5], J[6], J[7], J[8]));
            baseJ_ = loadedBaseJ_;
        }
        changing_ = false;
        applyInitialConditions(ic);
        if (!fdm_->RunIC()) {
            LOG_ERROR("sim") << "JSBSim RunIC failed on reset";
            return false;
        }
        if (ic.enginesRunning) startEngines();
        settleOnGround(ic);
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception on reset: " << e.what();
        return false;
    }
    checkSpawnInTerrain(ic);
    return true;
}

bool JsbsimModel::seizeEngine(int engine) {
    if (!loaded_ || engine < 0 || static_cast<unsigned>(engine) >= fdm_->GetPropulsion()->GetNumEngines()) return false;
    fdm_->SetPropertyValue("propulsion/engine[" + std::to_string(engine) + "]/seized", 1.0);
    return true;
}

int JsbsimModel::failChannel(const std::string& channel) {
    if (!loaded_ || channel.empty()) return 0;
    int failed = 0;
    for (const auto& [below, value] : properties("fcs")) {
        (void)value;
        const std::string name = "fcs/" + below, tail = "/malfunction/fail_stuck"; // (the listing's names are below its prefix)
        if (name.size() <= tail.size() || name.compare(name.size() - tail.size(), tail.size(), tail) != 0) continue;
        const std::string actuator = name.substr(0, name.size() - tail.size());
        if (actuator.find(channel) == std::string::npos) continue;
        fdm_->SetPropertyValue(name, 1.0);
        failedActuators_.push_back(name);
        ++failed;
    }
    return failed;
}

bool JsbsimModel::removeMass(double kg, const double at[3]) {
    if (!loaded_ || !(kg > 0.0)) return false;
    constexpr double kLbsPerKg = 2.20462262185, kInPerM = 39.3700787402;
    if (damageMassesUsed_ == damageMasses_.size()) {
        // a point mass of its own (JSBSim's AddPointMass, as its loader adds one), held at no weight until used
        auto leaf = [](const char* name, const std::string& value) {
            auto* e = new JSBSim::Element(name);
            e->AddData(value);
            return e;
        };
        JSBSim::Element_ptr pm = new JSBSim::Element("pointmass");
        pm->AddAttribute("name", "fsim damage " + std::to_string(damageMasses_.size()));
        auto* weight = leaf("weight", "0");
        weight->AddAttribute("unit", "LBS");
        pm->AddChildElement(weight);
        auto* location = new JSBSim::Element("location");
        location->AddAttribute("unit", "IN");
        location->AddChildElement(leaf("x", "0"));
        location->AddChildElement(leaf("y", "0"));
        location->AddChildElement(leaf("z", "0"));
        pm->AddChildElement(location);
        // (its index: the next after those the aircraft and earlier damage have - JSBSim numbers them in order)
        int next = 0;
        while (fdm_->GetPropertyManager()->HasNode("inertia/pointmass-weight-lbs[" + std::to_string(next) + "]")) ++next;
        damageMasses_.push_back(next);
        fdm_->GetMassBalance()->AddPointMass(pm);
    }
    const int k = damageMasses_[damageMassesUsed_++];
    const std::string base = "inertia/pointmass-";
    const std::string index = "[" + std::to_string(k) + "]";
    fdm_->SetPropertyValue(base + "location-X-inches" + index, at[0] * kInPerM);
    fdm_->SetPropertyValue(base + "location-Y-inches" + index, at[1] * kInPerM);
    fdm_->SetPropertyValue(base + "location-Z-inches" + index, at[2] * kInPerM);
    fdm_->SetPropertyValue(base + "weight-lbs" + index, -kg * kLbsPerKg);
    return true;
}

void JsbsimModel::runMassBalance() {
    // (as FGFDMExec::LoadInputs feeds it before its step's run; its gas cells' - none here - as the last step fed them)
    auto mb = fdm_->GetMassBalance();
    auto prop = fdm_->GetPropulsion();
    mb->in.TanksWeight = prop->GetTanksWeight();
    mb->in.TanksMoment = prop->GetTanksMoment();
    mb->in.TankInertia = prop->CalculateTankInertias();
    mb->in.WOW = fdm_->GetGroundReactions()->GetWOW();
    mb->Run(false);
}

JsbsimModel::Rigid JsbsimModel::rigid() const {
    Rigid r;
    if (!loaded_) return r;
    constexpr double kKgPerSlug = 14.5939029372, kMPerFt = 0.3048;
    auto mb = fdm_->GetMassBalance();
    auto p = fdm_->GetPropagate();
    r.massKg = mb->GetMass() * kKgPerSlug;
    const JSBSim::FGLocation& l = p->GetLocation();
    const JSBSim::FGColumnVector3 v = p->GetECEFVelocity();
    const JSBSim::FGMatrix33& T = p->GetTb2ec();
    for (unsigned k = 1; k <= 3; ++k) {
        r.cgEcef[k - 1] = l(k) * kMPerFt;
        r.velocityEcef[k - 1] = v(k) * kMPerFt;
        r.ratesBody[k - 1] = p->GetPQR(static_cast<int>(k));
        r.cgStructural[k - 1] = mb->GetXYZcg(static_cast<int>(k)) * 0.0254;
        for (unsigned c = 1; c <= 3; ++c) r.bodyToEcef[3 * (k - 1) + (c - 1)] = T(k, c);
    }
    return r;
}

void JsbsimModel::setBodyRates(const double ratesBody[3]) {
    if (!loaded_) return;
    for (unsigned k = 1; k <= 3; ++k) fdm_->GetPropagate()->SetPQR(k, ratesBody[k - 1]);
}

bool JsbsimModel::cgFromOriginBodyM(double out[3]) const {
    out[0] = out[1] = out[2] = 0.0;
    if (!loaded_) return false;
    constexpr double kMPerIn = 0.0254;
    const auto& cg = fdm_->GetMassBalance()->GetXYZcg();
    // (structural x aft, z up; body x forward, z down)
    out[0] = -(cg(1) - baseCgIn_[0]) * kMPerIn;
    out[1] = (cg(2) - baseCgIn_[1]) * kMPerIn;
    out[2] = -(cg(3) - baseCgIn_[2]) * kMPerIn;
    return true;
}

bool JsbsimModel::applyImpulse(const double linearBodyNs[3], const double angularBodyNms[3]) {
    if (!loaded_) return false;
    constexpr double kLbfSPerNs = 0.224808943099711, kLbfFtSPerNms = 0.737562149277;
    auto mb = fdm_->GetMassBalance();
    auto propagate = fdm_->GetPropagate();
    if (linearBodyNs && mb->GetMass() > 0.0)
        for (unsigned k = 1; k <= 3; ++k)
            propagate->SetUVW(k, propagate->GetUVW(static_cast<int>(k)) + linearBodyNs[k - 1] * kLbfSPerNs / mb->GetMass());
    if (angularBodyNms) {
        const JSBSim::FGColumnVector3 L(angularBodyNms[0] * kLbfFtSPerNms, angularBodyNms[1] * kLbfFtSPerNms, angularBodyNms[2] * kLbfFtSPerNms);
        const JSBSim::FGColumnVector3 dw = mb->GetJinv() * L;
        for (unsigned k = 1; k <= 3; ++k) propagate->SetPQR(k, propagate->GetPQR(static_cast<int>(k)) + dw(k));
    }
    return true;
}

double JsbsimModel::refuel(double kg) {
    if (!loaded_ || !(kg > 0.0)) return 0.0;
    constexpr double kLbPerKg = 2.20462262185;
    const auto& propulsion = fdm_->GetPropulsion();
    double room = 0.0;
    for (unsigned i = 0; i < propulsion->GetNumTanks(); ++i) {
        const auto tank = propulsion->GetTank(i);
        if (tank->GetType() == JSBSim::FGTank::ttFUEL) room += std::max(tank->GetCapacity() - tank->GetContents(), 0.0);
    }
    const double lbs = std::min(kg * kLbPerKg, room);
    if (!(lbs > 0.0)) return 0.0;
    for (unsigned i = 0; i < propulsion->GetNumTanks(); ++i) {
        const auto tank = propulsion->GetTank(i);
        if (tank->GetType() != JSBSim::FGTank::ttFUEL) continue;
        const double space = std::max(tank->GetCapacity() - tank->GetContents(), 0.0);
        tank->SetContents(tank->GetContents() + lbs * space / room);
    }
    return lbs / kLbPerKg;
}

bool JsbsimModel::beginMassChange() {
    if (!loaded_) return false;
    runMassBalance();
    const auto& cg = fdm_->GetMassBalance()->GetXYZcg();
    for (unsigned k = 0; k < 3; ++k) changeCgIn_[k] = cg(k + 1);
    changing_ = true;
    return true;
}

bool JsbsimModel::endMassChange(const double partInertia[6], const double impulseBodyNs[3]) {
    if (!loaded_ || !changing_) return false;
    changing_ = false;
    auto mb = fdm_->GetMassBalance();
    runMassBalance();
    if (partInertia) {
        // the part's own inertia about its centre off the aircraft's base inertia (its base: what the mass balance
        // sums with nothing of its own, against what it sums with it - JSBSim does not say)
        constexpr double kSlugFt2PerKgM2 = 0.737562149277;
        if (baseJ_.empty()) {
            // its base inertia as it was loaded, to the bit (a reset puts it back, and a vehicle reset after a part
            // left it must fly as a new one does): what the mass balance sums with nothing aboard - every point mass
            // at no weight, every tank empty - is the base inertia alone. (The whole less the rest, as it was taken
            // before, is the same number to a rounding - and a reset aircraft then flew a few bits off a new one.)
            auto pm = fdm_->GetPropertyManager();
            auto propulsion = fdm_->GetPropulsion();
            std::vector<double> weights, contents;
            for (int k = 0;; ++k) {
                const std::string name = "inertia/pointmass-weight-lbs[" + std::to_string(k) + "]";
                if (!pm->GetNode(name, false)) break;
                weights.push_back(fdm_->GetPropertyValue(name));
                fdm_->SetPropertyValue(name, 0.0);
            }
            for (unsigned i = 0; i < propulsion->GetNumTanks(); ++i) {
                contents.push_back(propulsion->GetTank(i)->GetContents());
                propulsion->GetTank(i)->SetContents(0.0);
            }
            runMassBalance();
            const JSBSim::FGMatrix33 base = mb->GetJ();
            for (unsigned r = 1; r <= 3; ++r)
                for (unsigned c = 1; c <= 3; ++c) loadedBaseJ_.push_back(base(r, c));
            for (std::size_t k = 0; k < weights.size(); ++k) fdm_->SetPropertyValue("inertia/pointmass-weight-lbs[" + std::to_string(k) + "]", weights[k]);
            for (unsigned i = 0; i < propulsion->GetNumTanks(); ++i) propulsion->GetTank(i)->SetContents(contents[i]);
            runMassBalance();
            baseJ_ = loadedBaseJ_;
        }
        const double* I = partInertia;
        // (the tensor: its moments on the diagonal, its plain products negated off it)
        const double part[9] = {I[0], -I[3], -I[4], -I[3], I[1], -I[5], -I[4], -I[5], I[2]};
        for (int k = 0; k < 9; ++k) baseJ_[static_cast<std::size_t>(k)] -= part[k] * kSlugFt2PerKgM2;
        const auto& J = baseJ_;
        mb->SetAircraftBaseInertias(JSBSim::FGMatrix33(J[0], J[1], J[2], J[3], J[4], J[5], J[6], J[7], J[8]));
        runMassBalance();
    }
    // the CG's shift (body axes, ft), the state point moved with it in the air (on the ground the mass balance did)
    constexpr double kFtPerIn = 1.0 / 12.0, kLbfSPerNs = 0.224808943099711;
    const auto& cg = mb->GetXYZcg();
    const JSBSim::FGColumnVector3 shift(-(cg(1) - changeCgIn_[0]) * kFtPerIn, (cg(2) - changeCgIn_[1]) * kFtPerIn,
                                        -(cg(3) - changeCgIn_[2]) * kFtPerIn);
    auto propagate = fdm_->GetPropagate();
    if (!fdm_->GetHoldDown() && !fdm_->GetGroundReactions()->GetWOW()) propagate->NudgeBodyLocation(-1.0 * shift);
    // the rest moving as that point of it did, and the reaction on it
    JSBSim::FGColumnVector3 dv = propagate->GetPQR() * shift;
    if (impulseBodyNs && mb->GetMass() > 0.0)
        for (unsigned k = 1; k <= 3; ++k) dv(k) += impulseBodyNs[k - 1] * kLbfSPerNs / mb->GetMass();
    for (unsigned k = 1; k <= 3; ++k) propagate->SetUVW(k, propagate->GetUVW(static_cast<int>(k)) + dv(k));
    return true;
}

// Aircraft files may declare their own <output> (CSV/socket). The platform
// owns logging and recording, so those are disabled - and, because JSBSim
// opens output files in RunIC() even when disabled, redirected to the null
// device so no stray files appear in the working directory.
void JsbsimModel::silenceOutputs() {
    auto output = fdm_->GetOutput();
    for (unsigned i = 0; !output->GetOutputName(i).empty(); ++i) output->SetOutputName(i, kNullDevice);
    fdm_->DisableOutput();
}

// A generic body-frame force and moment at the CG, defined in memory so that
// effects can push on any stock aircraft without editing its XML (design 9.5).
void JsbsimModel::installExternalReaction() {
    auto leaf = [](const char* name, const char* value) {
        auto* e = new JSBSim::Element(name);
        e->AddData(value);
        return e;
    };
    auto triplet = [&](const char* name, const char* x, const char* y, const char* z) {
        auto* e = new JSBSim::Element(name);
        e->AddChildElement(leaf("x", x));
        e->AddChildElement(leaf("y", y));
        e->AddChildElement(leaf("z", z));
        return e;
    };
    // Elements are reference counted (SGSharedPtr): JSBSim's loader takes and
    // drops references while parsing, so hold the root ourselves.
    JSBSim::Element_ptr reactions = new JSBSim::Element("external_reactions");
    auto* force = new JSBSim::Element("force");
    force->AddAttribute("name", "fsim");
    force->AddAttribute("frame", "BODY");
    auto* location = triplet("location", "0", "0", "0");
    location->AddAttribute("unit", "IN");
    force->AddChildElement(location);
    force->AddChildElement(triplet("direction", "1", "0", "0"));
    reactions->AddChildElement(force);
    auto* moment = new JSBSim::Element("moment");
    moment->AddAttribute("name", "fsim");
    moment->AddAttribute("frame", "BODY");
    moment->AddChildElement(triplet("direction", "1", "0", "0"));
    reactions->AddChildElement(moment);
    try {
        // An aircraft with its own <external_reactions> (f16: pushback, hook) has
        // already tied the summary properties; Load() ties them again and logs an
        // error for each, so untie first (the same object re-ties them).
        auto pm = fdm_->GetPropertyManager();
        for (const char* tied : {"moments/l-external-lbsft", "moments/m-external-lbsft", "moments/n-external-lbsft",
                                 "forces/fbx-external-lbs", "forces/fby-external-lbs", "forces/fbz-external-lbs"})
            if (pm->HasNode(tied)) pm->Untie(tied);
        fdm_->GetExternalReactions()->Load(reactions);
    } catch (const std::exception& e) {
        LOG_WARN("sim") << "external reaction not installed: " << e.what();
    }
    extForceMag_ = property("external_reactions/fsim/magnitude");
    extForceX_ = property("external_reactions/fsim/x");
    extForceY_ = property("external_reactions/fsim/y");
    extForceZ_ = property("external_reactions/fsim/z");
    extMomentMag_ = property("external_reactions/fsim/magnitude-lbsft");
    extMomentL_ = property("external_reactions/fsim/l");
    extMomentM_ = property("external_reactions/fsim/m");
    extMomentN_ = property("external_reactions/fsim/n");
    extLocX_ = property("external_reactions/fsim/location-x-in");
    extLocY_ = property("external_reactions/fsim/location-y-in");
    extLocZ_ = property("external_reactions/fsim/location-z-in");
    cgX_ = property("inertia/cg-x-in");
    cgY_ = property("inertia/cg-y-in");
    cgZ_ = property("inertia/cg-z-in");
    extForceMag_.set(0.0);
    extMomentMag_.set(0.0);
}

// Engines running with full mixture from the first step (an airborne start
// with a dead engine is never what a scenario means); the throttle is the
// caller's from the next step on.
void JsbsimModel::startEngines() {
    auto propulsion = fdm_->GetPropulsion();
    if (propulsion->GetNumEngines() == 0) return;
    try {
        propulsion->InitRunning(-1);
    } catch (const std::exception& e) {
        LOG_WARN("sim") << "engine start: " << e.what();
    }
}

void JsbsimModel::setWindNed(double north, double east, double down) {
    if (!loaded_) return;
    fdm_->GetWinds()->SetWindNED(units::metresToFeet(north), units::metresToFeet(east), units::metresToFeet(down));
}

void JsbsimModel::setTurbulence(double intensity, double windSpeed20ftMs) {
    if (!loaded_) return;
    auto winds = fdm_->GetWinds();
    if (intensity <= 0.0) {
        winds->SetTurbType(JSBSim::FGWinds::ttNone);
        return;
    }
    // Milspec (Dryden) turbulence: severity index 0..7 (3 light, 4 moderate, 6 severe).
    winds->SetTurbType(JSBSim::FGWinds::ttMilspec);
    winds->SetProbabilityOfExceedence(std::clamp(static_cast<int>(std::lround(1.0 + intensity * 6.0)), 1, 7));
    winds->SetWindspeed20ft(units::metresToFeet(std::max(windSpeed20ftMs, 1.0)));
}

void JsbsimModel::setAtmosphere(double temperatureSeaLevelK, double pressureSeaLevelPa) {
    if (!loaded_) return;
    auto atmosphere = fdm_->GetAtmosphere();
    atmosphere->SetTemperatureSL(temperatureSeaLevelK, JSBSim::FGAtmosphere::eKelvin);
    atmosphere->SetPressureSL(JSBSim::FGAtmosphere::ePascals, pressureSeaLevelPa);
}

void JsbsimModel::setExternalForceBody(const double forceN[3], const double momentNm[3]) {
    if (!loaded_ || !extForceMag_.valid()) return;
    // At the centre of gravity (structural frame, inches), so a pure force makes no moment.
    extLocX_.set(cgX_.get());
    extLocY_.set(cgY_.get());
    extLocZ_.set(cgZ_.get());
    const double f = std::sqrt(forceN[0] * forceN[0] + forceN[1] * forceN[1] + forceN[2] * forceN[2]);
    if (f > 1e-9) {
        extForceX_.set(forceN[0] / f);
        extForceY_.set(forceN[1] / f);
        extForceZ_.set(forceN[2] / f);
    }
    extForceMag_.set(f * units::kNewtonsToPoundsForce);
    const double m = std::sqrt(momentNm[0] * momentNm[0] + momentNm[1] * momentNm[1] + momentNm[2] * momentNm[2]);
    if (m > 1e-9) {
        extMomentL_.set(momentNm[0] / m);
        extMomentM_.set(momentNm[1] / m);
        extMomentN_.set(momentNm[2] / m);
    }
    constexpr double kNewtonMetresToPoundFeet = units::kNewtonsToPoundsForce * units::kMetresToFeet;
    extMomentMag_.set(m * kNewtonMetresToPoundFeet);
}

void JsbsimModel::seed(std::uint64_t value) {
    fdm_->GetRandomGenerator()->seed(static_cast<unsigned int>(value ^ (value >> 32)));
}

void JsbsimModel::applyInitialConditions(const InitialConditions& ic) {
    // JSBSim's IC setters each start from what was set before them: the
    // latitude re-places the position at the altitude last set, the angles
    // turn the velocity last set. So the conditions start from the IC a new
    // model has, and the terrain's queries from a new callback's (no frame
    // kept from the last query) - from where a reused model's last vehicle
    // left them, the same conditions gave another start in the last bits
    // (design 7.2).
    static_cast<ProviderGroundCallback*>(groundCallback_)->forget();
    auto IC = fdm_->GetIC();
    IC->InitializeIC();
    IC->SetGeodLatitudeDegIC(ic.latitudeDeg); // geodetic; SetLatitudeDegIC would be geocentric
    IC->SetLongitudeDegIC(ic.longitudeDeg);
    IC->SetPsiDegIC(ic.headingDeg);
    IC->SetThetaDegIC(ic.pitchDeg);
    IC->SetPhiDegIC(ic.rollDeg);
    if (std::isfinite(ic.velocityNedMs[0]) && std::isfinite(ic.velocityNedMs[1]) && std::isfinite(ic.velocityNedMs[2])) {
        IC->SetVNorthFpsIC(units::metresToFeet(ic.velocityNedMs[0])); // (after the angles: they turn the velocity last set)
        IC->SetVEastFpsIC(units::metresToFeet(ic.velocityNedMs[1]));
        IC->SetVDownFpsIC(units::metresToFeet(ic.velocityNedMs[2]));
    } else {
        IC->SetVtrueKtsIC(units::metresPerSecondToKnots(ic.airspeedTrueMs));
    }
    if (std::isfinite(ic.bodyRatesRadS[0]) && std::isfinite(ic.bodyRatesRadS[1]) && std::isfinite(ic.bodyRatesRadS[2])) {
        IC->SetPRadpsIC(ic.bodyRatesRadS[0]);
        IC->SetQRadpsIC(ic.bodyRatesRadS[1]);
        IC->SetRRadpsIC(ic.bodyRatesRadS[2]);
    }

    if (ic.onGround) {
        // Start with the CG a gear-height above the provider's terrain; the
        // ground trim after RunIC() then settles the gear onto the local slope.
        // (AGL 0 would bury the gear and launch the aircraft off its springs.)
        const double terrainM = ground_->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg),
                                                               units::degreesToRadians(ic.longitudeDeg));
        IC->SetTerrainElevationFtIC(units::metresToFeet(terrainM));
        IC->SetAltitudeAGLFtIC(units::metresToFeet(kGroundSpawnCgHeightM));
    } else {
        IC->SetAltitudeASLFtIC(units::metresToFeet(ic.altitudeMslM));
        // flying: its gear up, where its command puts it (a load's model has it down, and it took 5 s to come up - with its
        // drag, the first seconds of every start in the air: docs/flight-autonomy.md, 4.62)
        auto pm = fdm_->GetPropertyManager();
        for (const char* path : {"gear/gear-cmd-norm", "gear/gear-pos-norm"})
            if (auto* n = pm->GetNode(path)) n->setDoubleValue(0.0);
    }
    if (std::isfinite(ic.fuelFraction)) {
        // its fuel tanks each that full (a load's and a reset's own are its design's: FGTank::ResetToIC)
        const double fraction = std::clamp(ic.fuelFraction, 0.0, 1.0);
        const auto& propulsion = fdm_->GetPropulsion();
        for (unsigned i = 0; i < propulsion->GetNumTanks(); ++i) {
            const auto tank = propulsion->GetTank(i);
            if (tank->GetType() == JSBSim::FGTank::ttFUEL) tank->SetContents(fraction * tank->GetCapacity());
        }
    }
}

void JsbsimModel::settleOnGround(const InitialConditions& ic) {
    if (!ic.onGround) return;
    // JSBSim's ground trim: adjusts altitude, pitch and roll until the gear
    // forces balance the weight on the terrain under each wheel.
    try {
        fdm_->DoTrim(JSBSim::tGround);
    } catch (const std::exception& e) {
        LOG_WARN("sim") << "ground trim failed (" << e.what() << "); the vehicle will settle dynamically";
    }
}

void JsbsimModel::cacheCommandNodes() {
    auto pm = fdm_->GetPropertyManager();
    auto node = [&](const char* path) { return PropertyHandle(pm->GetNode(path, true)); };

    aileronCmd_ = node("fcs/aileron-cmd-norm");
    elevatorCmd_ = node("fcs/elevator-cmd-norm");
    rudderCmd_ = node("fcs/rudder-cmd-norm");
    flapCmd_ = node("fcs/flap-cmd-norm");
    gearCmd_ = node("gear/gear-cmd-norm");
    leftBrakeCmd_ = node("fcs/left-brake-cmd-norm");
    rightBrakeCmd_ = node("fcs/right-brake-cmd-norm");
    speedbrakeCmd_ = node("fcs/speedbrake-cmd-norm");
    pitchTrimCmd_ = node("fcs/pitch-trim-cmd-norm");

    // read-only: an aircraft whose FCS has no leading-edge flaps reports none
    lefPosDeg_ = PropertyHandle(pm->GetNode("fcs/lef-pos-deg", false));
    // likewise a battery: one without it has none
    batteryCharge_ = PropertyHandle(pm->GetNode("fsim/battery/charge-j", false));
    batteryCapacity_ = PropertyHandle(pm->GetNode("fsim/battery/capacity-j", false));
    batteryPower_ = PropertyHandle(pm->GetNode("fsim/battery/power-w", false));

    throttleCmd_.clear();
    const auto engines = std::min<std::size_t>(fdm_->GetPropulsion()->GetNumEngines(), ControlInputs::kMaxEngines);
    for (std::size_t i = 0; i < engines; ++i) {
        throttleCmd_.push_back(PropertyHandle(pm->GetNode("fcs/throttle-cmd-norm", static_cast<int>(i), true)));
    }
    rotorRpm_.clear();
    for (std::size_t i = 0; i < std::min<std::size_t>(engines, VehicleState::kMaxEngines); ++i) {
        const auto thruster = fdm_->GetPropulsion()->GetEngine(static_cast<unsigned>(i))->GetThruster();
        const bool ownSpeed = thruster && thruster->GetType() != JSBSim::FGThruster::ttDirect;
        rotorRpm_.push_back(ownSpeed ? PropertyHandle() : PropertyHandle(pm->GetNode("propulsion/engine[" + std::to_string(i) + "]/rotor-rpm", false)));
    }
}

void JsbsimModel::step(const ControlInputs& in) {
    if (!loaded_ || diverged_ || crashed_) return;

    aileronCmd_.set(in.aileron);
    elevatorCmd_.set(in.elevator);
    rudderCmd_.set(in.rudder);
    flapCmd_.set(in.flaps);
    gearCmd_.set(in.gearDown);
    leftBrakeCmd_.set(in.brakeLeft);
    rightBrakeCmd_.set(in.brakeRight);
    for (std::size_t i = 0; i < throttleCmd_.size(); ++i) throttleCmd_[i].set(in.throttle[i]);

    fdm_->Run();
    ++stepCount_;

    if (checkDivergence()) checkTerrainStrike();
}

void JsbsimModel::checkTerrainStrike() {
    // Struck the terrain: a point of it - its CG, a wheel, a wingtip, its nose - more than kStrikeDepthM into the ground
    // under it, deeper than any gear's or structure's give. JSBSim's ground pushes up only, under each point: flown into
    // a hillside the C172 went 330 m into the rock and flew on inside it, the F-16C through the ridge and out of its far
    // side. Stopped where it struck instead, lifted back onto the surface by that depth, its activities ended, until it
    // is reset (docs/flight-autonomy.md, 4.68)
    const auto* ground = static_cast<const ProviderGroundCallback*>(groundCallback_);
    const double depthM = -units::feetToMetres(ground->deepestFt());
    if (!(depthM > kStrikeDepthM)) return;
    const double speed = stopOnTerrain(depthM);
    LOG_WARN("sim") << "vehicle struck the terrain at step " << stepCount_ << ": " << depthM << " m into it at lat "
                    << crashState_.latitudeRad / units::kDegreesToRadians << " lon " << crashState_.longitudeRad / units::kDegreesToRadians
                    << ", " << speed << " m/s; stopped there";
}

void JsbsimModel::checkSpawnInTerrain(const InitialConditions& ic) {
    // Started in the air inside the terrain - its CG under the ground beneath it: struck it where it starts, stood on
    // its surface a gear's height up (as a start on the ground begins: kGroundSpawnCgHeightM). Flown from there, its
    // gear's springs launched it from 900 m down and its state blew up in its first step (4.68)
    if (ic.onGround || !loaded_) return;
    VehicleState s;
    state(s);
    const double depthM = -s.altitudeAglM;
    if (!(depthM > kStrikeDepthM)) return;
    stopOnTerrain(depthM + kGroundSpawnCgHeightM);
    LOG_WARN("sim") << "vehicle started " << depthM << " m inside the terrain at lat " << ic.latitudeDeg << " lon " << ic.longitudeDeg
                    << " (altitude " << ic.altitudeMslM << " m, the ground " << ic.altitudeMslM + depthM << " m); stopped on its surface";
}

double JsbsimModel::stopOnTerrain(double liftM) {
    state(crashState_);
    const double cl = std::cos(crashState_.latitudeRad);
    const double up[3] = {cl * std::cos(crashState_.longitudeRad), cl * std::sin(crashState_.longitudeRad), std::sin(crashState_.latitudeRad)};
    const double speed = std::sqrt(crashState_.velocityNedMs[0] * crashState_.velocityNedMs[0] +
                                   crashState_.velocityNedMs[1] * crashState_.velocityNedMs[1] +
                                   crashState_.velocityNedMs[2] * crashState_.velocityNedMs[2]);
    for (int i = 0; i < 3; ++i) {
        crashState_.positionEcef[i] += up[i] * liftM;
        crashState_.velocityBodyMs[i] = crashState_.velocityNedMs[i] = crashState_.angularRateBodyRadS[i] = 0.0;
        crashState_.accelerationBodyMs2[i] = 0.0;
    }
    crashState_.altitudeMslM += liftM, crashState_.altitudeAglM += liftM;
    crashState_.airspeedTrueMs = crashState_.airspeedCalibratedMs = crashState_.mach = 0.0;
    crashState_.alphaRad = crashState_.betaRad = 0.0;
    crashState_.loadFactor = 1.0;
    for (double& t : crashState_.thrustN) t = 0.0;
    crashState_.onGround = crashState_.crashed = true;
    crashed_ = true;
    return speed;
}

void JsbsimModel::setEffectors(const EffectorInputs& e) {
    if (!loaded_) return;
    if (!std::isnan(e.speedbrake)) speedbrakeCmd_.set(e.speedbrake);
    if (!std::isnan(e.pitchTrim)) pitchTrimCmd_.set(e.pitchTrim);
}

bool JsbsimModel::checkDivergence() {
    // A NaN anywhere in position/velocity poisons everything downstream;
    // mark the vehicle so the environment terminates and resets it (design 13).
    const auto prop = fdm_->GetPropagate();
    const auto& loc = prop->GetLocation();
    const auto& uvw = prop->GetUVW();
    const double altM = units::feetToMetres(prop->GetAltitudeASL());
    const double speedMs = units::feetToMetres(uvw.Magnitude());
    // Non-finite, or physically impossible for an aircraft: catches the onset of
    // a numerical blow-up a few steps in instead of after the values overflow.
    const bool ok = finite(loc(1)) && finite(loc(2)) && finite(loc(3)) && finite(uvw(1)) && finite(uvw(2)) &&
                    finite(uvw(3)) && finite(altM) && speedMs < 5000.0 && altM > -1000.0 && altM < 200000.0;
    if (!ok && !diverged_) {
        diverged_ = true;
        LOG_WARN("sim") << "vehicle diverged (non-finite state) at step " << stepCount_ << "; last good state: lat "
                        << lastGood_.latDeg << " lon " << lastGood_.lonDeg << " alt " << lastGood_.altM << " m, agl "
                        << lastGood_.aglM << " m, |v| " << lastGood_.speedMs << " m/s, on ground " << lastGood_.onGround;
    } else if (ok) {
        // Keep a compact copy of the last finite state for the divergence report.
        lastGood_.latDeg = prop->GetGeodLatitudeDeg();
        lastGood_.lonDeg = prop->GetLongitudeDeg();
        lastGood_.altM = altM;
        lastGood_.aglM = units::feetToMetres(prop->GetDistanceAGL());
        lastGood_.speedMs = speedMs;
        lastGood_.onGround = fdm_->GetGroundReactions()->GetWOW();
    }
    return !diverged_;
}

void JsbsimModel::state(VehicleState& out) const {
    using namespace units;
    if (crashed_) { // (stopped where it struck)
        out = crashState_;
        return;
    }
    out = VehicleState{};
    out.simTime = fdm_->GetSimTime();
    out.stepCount = stepCount_;
    out.diverged = diverged_;
    if (!loaded_) return;

    const auto prop = fdm_->GetPropagate();
    const auto aux = fdm_->GetAuxiliary();
    const auto fcs = fdm_->GetFCS();
    const auto accel = fdm_->GetAccelerations();
    const auto propulsion = fdm_->GetPropulsion();

    const JSBSim::FGLocation& loc = prop->GetLocation();
    for (int i = 0; i < 3; ++i) out.positionEcef[i] = feetToMetres(loc(static_cast<unsigned>(i + 1)));

    const JSBSim::FGQuaternion q = prop->GetQuaternionECEF(); // ECEF -> body
    for (int i = 0; i < 4; ++i) out.attitudeEcefToBody[i] = q(static_cast<unsigned>(i + 1));

    out.latitudeRad = loc.GetGeodLatitudeRad();
    out.longitudeRad = loc.GetLongitude();
    out.altitudeMslM = feetToMetres(prop->GetAltitudeASL());
    out.altitudeAglM = feetToMetres(prop->GetDistanceAGL());

    const auto& euler = prop->GetEuler();
    const auto& uvw = prop->GetUVW();
    const auto& ned = prop->GetVel();
    const auto& pqr = prop->GetPQR();
    const auto& uvwDot = accel->GetUVWdot();
    for (unsigned i = 1; i <= 3; ++i) {
        out.eulerRad[i - 1] = euler(i);
        out.velocityBodyMs[i - 1] = feetToMetres(uvw(i));
        out.velocityNedMs[i - 1] = feetToMetres(ned(i));
        out.angularRateBodyRadS[i - 1] = pqr(i);
        out.accelerationBodyMs2[i - 1] = feetToMetres(uvwDot(i));
    }

    out.airspeedTrueMs = feetToMetres(aux->GetVtrueFPS());
    out.airspeedCalibratedMs = knotsToMetresPerSecond(aux->GetVcalibratedKTS());
    out.mach = aux->GetMach();
    out.alphaRad = aux->Getalpha();
    out.betaRad = aux->Getbeta();
    out.loadFactor = aux->GetNlf();

    out.aileronRad = fcs->GetDaLPos(JSBSim::ofRad);
    out.elevatorRad = fcs->GetDePos(JSBSim::ofRad);
    out.rudderRad = fcs->GetDrPos(JSBSim::ofRad);
    out.flapsRad = fcs->GetDfPos(JSBSim::ofRad);
    out.gearPosition = fcs->GetGearPos();
    out.speedbrakePosition = static_cast<float>(fcs->GetDsbPos(JSBSim::ofNorm)); // (fcs/speedbrake-pos-norm)

    const auto engines = std::min<std::size_t>(propulsion->GetNumEngines(), VehicleState::kMaxEngines);
    out.engineCount = static_cast<int>(engines);
    for (std::size_t i = 0; i < engines; ++i) {
        out.throttlePosition[i] = fcs->GetThrottlePos(static_cast<int>(i));
        const auto engine = propulsion->GetEngine(static_cast<unsigned>(i));
        out.thrustN[i] = poundsForceToNewtons(engine->GetThrust());
        out.engineRpm[i] = i < rotorRpm_.size() && rotorRpm_[i].valid() ? rotorRpm_[i].get()
                           : engine->GetThruster()                      ? engine->GetThruster()->GetRPM()
                                                                        : 0.0;
        out.engineN2[i] = out.afterburner[i] = out.nozzlePosition[i] = 0.0;
        if (engine->GetType() == JSBSim::FGEngine::etTurbine) {
            const auto* turbine = static_cast<const JSBSim::FGTurbine*>(engine.get());
            out.engineN2[i] = turbine->GetN2();
            out.nozzlePosition[i] = turbine->GetNozzle();
            // lit: as far as the nozzle has opened with the reheat (a staged
            // one's lit share, cmake/JsbsimPatches.cmake; a throttle-gated
            // one's opening) - what burns, not where the lever is
            if (turbine->GetAugmentation()) out.afterburner[i] = std::min(std::max(turbine->GetNozzle(), 0.0), 1.0);
        }
    }
    out.leadingEdgeFlapRad = lefPosDeg_.valid() ? lefPosDeg_.get() * 3.14159265358979323846 / 180.0 : 0.0;
    // the wheels: each strut's compression, its steering, the wheel's roll
    const auto& ground = fdm_->GetGroundReactions();
    out.wheelCount = 0;
    for (int i = 0; i < ground->GetNumGearUnits() && out.wheelCount < VehicleState::kMaxWheels; ++i) {
        const auto unit = ground->GetGearUnit(i);
        if (!unit->IsBogey()) continue;
        const int k = out.wheelCount++;
        out.wheelCompressionM[k] = feetToMetres(unit->GetCompLen());
        out.wheelSteerRad[k] = unit->GetSteerAngleDeg() * 3.14159265358979323846 / 180.0;
        out.wheelSpeedMs[k] = feetToMetres(unit->GetWheelRollVel());
    }
    for (int k = out.wheelCount; k < VehicleState::kMaxWheels; ++k)
        out.wheelCompressionM[k] = out.wheelSteerRad[k] = out.wheelSpeedMs[k] = 0.0;
    out.fuelKg = propulsion->GetTanksWeight() * 0.45359237; // tank contents, lbs -> kg

    out.onGround = fdm_->GetGroundReactions()->GetWOW();

    const JSBSim::FGMatrix33& tb2ec = prop->GetTb2ec(); // body -> ECEF
    for (unsigned r = 1; r <= 3; ++r)
        for (unsigned c = 1; c <= 3; ++c) out.rotationBodyToEcef[(r - 1) * 3 + (c - 1)] = tb2ec(r, c);
}

double JsbsimModel::engineFuelFlowKgS(int engine) const {
    if (!fdm_ || engine < 0) return 0.0;
    const auto& propulsion = fdm_->GetPropulsion();
    if (static_cast<unsigned>(engine) >= propulsion->GetNumEngines()) return 0.0;
    const auto e = propulsion->GetEngine(static_cast<unsigned>(engine));
    // (a rocket's propellant: its fuel's and its oxidiser's, of which a solid motor's grain is all the first)
    double flow = e->GetFuelFlowRate();
    if (e->GetType() == JSBSim::FGEngine::etRocket) {
        // The tied property reads the same native value without coupling the
        // adapter to the generated rocket subclass header.
        const auto path = "propulsion/engine[" + std::to_string(engine) + "]/oxi-flow-rate-pps";
        if (const auto* node = fdm_->GetPropertyManager()->GetNode(path, false)) flow += node->getDoubleValue();
    }
    constexpr double kKgPerLb = 0.45359237;
    return std::isfinite(flow) ? std::max(flow, 0.0) * kKgPerLb : 0.0;
}

EnergyOnBoard JsbsimModel::energy() const {
    EnergyOnBoard e;
    if (!fdm_) return e;
    constexpr double kKgPerLb = 0.45359237;
    const auto& propulsion = fdm_->GetPropulsion();
    double contents = 0.0, capacity = 0.0;
    for (unsigned i = 0; i < propulsion->GetNumTanks(); ++i) {
        const auto tank = propulsion->GetTank(i);
        if (tank->GetType() != JSBSim::FGTank::ttFUEL) continue;
        contents += tank->GetContents();
        capacity += tank->GetCapacity();
    }
    if (capacity > 0.0) {
        double flow = 0.0;
        for (unsigned i = 0; i < propulsion->GetNumEngines(); ++i) {
            const auto engine = propulsion->GetEngine(i);
            flow += engine->GetFuelFlowRate(); // lb/s
            // (JSBSim counts an engine no tank feeds as starved: a tail rotor's drive burns nothing)
            if (engine->GetNumSourceTanks() > 0 && engine->GetStarved()) e.starved = true;
        }
        e.fuelKg = contents * kKgPerLb, e.fuelCapacityKg = capacity * kKgPerLb, e.fuelFlowKgS = flow * kKgPerLb;
    }
    if (batteryCapacity_.valid() && batteryCharge_.valid() && batteryPower_.valid())
        e.chargeJ = batteryCharge_.get(), e.chargeCapacityJ = batteryCapacity_.get(), e.powerW = batteryPower_.get();
    e.massKg = fdm_->GetMassBalance()->GetWeight() * kKgPerLb;
    return e;
}

PropertyHandle JsbsimModel::property(std::string_view path) {
    auto* node = fdm_->GetPropertyManager()->GetNode(std::string(path), false);
    if (!node) LOG_WARN("sim") << "unknown JSBSim property '" << path << "'";
    return PropertyHandle(node);
}

std::vector<std::pair<std::string, double>> JsbsimModel::properties(std::string_view prefix) const {
    std::vector<std::pair<std::string, double>> out;
    if (!fdm_) return out;
    const SGPropertyNode* root = fdm_->GetPropertyManager()->GetNode(std::string(prefix), false);
    if (!root) return out;
    // depth first, in the tree's order; a leaf's path joins its ancestors' names below the prefix
    auto walk = [&out](const SGPropertyNode* node, const std::string& path, auto& self) -> void {
        for (int i = 0; i < node->nChildren(); ++i) {
            const SGPropertyNode* child = node->getChild(i);
            std::string name = path.empty() ? child->getNameString() : path + "/" + child->getNameString();
            if (child->getIndex() > 0) name += "[" + std::to_string(child->getIndex()) + "]";
            if (child->nChildren() > 0) self(child, name, self);
            else if (child->hasValue()) out.emplace_back(std::move(name), child->getDoubleValue());
        }
    };
    walk(root, "", walk);
    return out;
}

} // namespace fsim::sim
