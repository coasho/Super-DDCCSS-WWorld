// fsim-dlz-generator (SIM-M04 / MS-8):
// Deterministic native 6-DOF JSBSim flyout envelope generator producing
// aircraft/stores/<model>/<model>.dlz-v1.json tables, propagated uncertainty
// bounds, and published-performance comparison reports for all maintained AAMs.
//
// Method:
// 1. For each (launch_altitude_m, launch_mach, target_altitude_delta_m) launch
//    condition (36 knots), executes a full native 6-DOF JSBSim missile flyout
//    (120 Hz outer FDM with authored integration-v1 substeps, native Guidance,
//    native fin autopilot, TVC, and dual-pulse motor where configured) to record
//    the exact kinematic reach curve D_m(t), speed(t), altitude(t).
// 2. Executes native 6-DOF off-boresight flyouts (0..60 deg) to measure actual
//    missile turn-induced range attenuation.
// 3. Evaluates exact 3-D intercept (R_max, R_min) and immediate 7g sustained
//    level break-turn escape (R_ne) across the full 540-point 5-D grid using the
//    recorded native 6-DOF reach curves and measured off-boresight turn loss.
// 4. Executes perturbed native 6-DOF flyouts (+/-10% thrust, -/+15% drag,
//    +/-5% battery life) on 4 anchor conditions and propagates 1-sigma RMS
//    fractional uncertainties for R_max, R_ne, R_min, and TOF.
//
// No aerodynamic or propulsion parameters are tuned to published figures;
// differences are recorded honestly in the published-performance report.

#include "fsim/GroundProvider.h"
#include "foundation/core/Attitude.h"
#include "foundation/core/Json.h"
#include "foundation/core/Log.h"
#include "foundation/io/AssetResolver.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/weapon/Autopilot.h"
#include "simulation/mission_systems/weapon/Guidance.h"
#include "simulation/mission_systems/weapon/LaunchZone.h"
#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "simulation/mission_systems/weapon/PulseManagerV1.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kG0 = 9.80665;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kLbfToN = 4.4482216152605;
constexpr double kLbFtToNm = 1.3558179483314;
constexpr double kFtToM = 0.3048;
constexpr double kSlugFt2ToKgM2 = 1.3558179483314;
constexpr double kFdmDtS = 1.0 / 120.0;
constexpr int kFrameSkip = 4;
constexpr double kOuterDtS = kFdmDtS * static_cast<double>(kFrameSkip);
constexpr double kEscapeTurnG = 7.0;
constexpr double kMinTerminalSpeedMs = 120.0;

using V3 = fsim::ms::weapon::V3;

V3 vAdd(V3 a, V3 b) noexcept { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 vSub(V3 a, V3 b) noexcept { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 vScale(V3 a, double s) noexcept { return {a.x * s, a.y * s, a.z * s}; }
double vDot(V3 a, V3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
double vNorm(V3 a) noexcept { return std::sqrt(vDot(a, a)); }
V3 vUnit(V3 a) noexcept {
    const double n = vNorm(a);
    return n > 1e-12 ? vScale(a, 1.0 / n) : V3{1.0, 0.0, 0.0};
}

void toBody(const fsim::sim::VehicleState& s, const double ecef[3], double body[3]) noexcept {
    const auto* r = s.rotationBodyToEcef;
    body[0] = r[0] * ecef[0] + r[3] * ecef[1] + r[6] * ecef[2];
    body[1] = r[1] * ecef[0] + r[4] * ecef[1] + r[7] * ecef[2];
    body[2] = r[2] * ecef[0] + r[5] * ecef[1] + r[8] * ecef[2];
}

V3 velocityEcef(const fsim::sim::VehicleState& s) noexcept {
    const auto* r = s.rotationBodyToEcef;
    const auto* v = s.velocityBodyMs;
    return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
            r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

std::string readFileBytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.good()) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 4);
    for (char ch : s) {
        switch (ch) {
            case '\"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out.push_back(ch); break;
        }
    }
    return out;
}

std::string formatNumber(double v, int prec = 2) {
    if (!std::isfinite(v)) return "0.0";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(prec) << v;
    std::string s = ss.str();
    if (s == "-0.00" || s == "-0.0" || s == "-0.0000") {
        s.erase(s.begin());
    }
    return s;
}

struct MissileConfig {
    std::string cardId;
    std::string designation;
    std::string modelName;
    std::filesystem::path cardPath;
    std::filesystem::path modelXmlPath;
    std::filesystem::path aircraftDir;
    std::filesystem::path jsbsimRoot;
    std::string cardSha256;
    std::string modelXmlSha256;

    std::shared_ptr<const fsim::ms::StoreCard> card;
    fsim::ms::components::ProfileV1 profile;
    fsim::ms::weapon::Spec guidanceSpec;
    fsim::ms::weapon::PlantAdapterV1 plantAdapter;
    std::optional<fsim::ms::weapon::PulsePolicyV1> pulsePolicy;

    double publishedRangeKm = 0.0;
    double publishedRangeUncKm = 0.0;
    std::string publishedRangeSource;
    std::string publishedRangeNote;
};

struct FinHandles {
    fsim::PropertyHandle version;
    fsim::PropertyHandle qbar;
    fsim::PropertyHandle area;
    fsim::PropertyHandle diameter;
    fsim::PropertyHandle span;
    fsim::PropertyHandle referenceVersion;
    fsim::PropertyHandle aeroReferenceX;
    std::array<fsim::PropertyHandle, 3> aeroForce;
    std::array<fsim::PropertyHandle, 3> aeroMoment;
    fsim::PropertyHandle totalAxialForce;
    fsim::PropertyHandle lag;
    fsim::PropertyHandle normalizationVersion;
    fsim::PropertyHandle actuatorVersion;
    fsim::PropertyHandle channelRate;
    fsim::PropertyHandle finCount;
    std::vector<fsim::PropertyHandle> finRates;
    std::size_t passiveFinCount = 0;
    std::array<fsim::PropertyHandle, 3> inertia;
    std::array<fsim::PropertyHandle, 3> limits;
    std::array<fsim::PropertyHandle, 3> lowerStops;
    std::array<fsim::PropertyHandle, 3> upperStops;
    std::array<fsim::PropertyHandle, 12> derivatives;

    bool hasTvc = false;
    fsim::PropertyHandle vectorVersion;
    fsim::PropertyHandle vectorLimit;
    fsim::PropertyHandle vectorRate;
    fsim::PropertyHandle vectorPitchGain;
    fsim::PropertyHandle vectorYawGain;
    fsim::PropertyHandle vectorExitX;
    fsim::PropertyHandle vectorPitch;
    fsim::PropertyHandle vectorYaw;
    std::array<fsim::PropertyHandle, 3> propulsionForce;
    std::array<fsim::PropertyHandle, 3> propulsionMoment;

    fsim::PropertyHandle pulseVersion;
    fsim::PropertyHandle pulseAge;
    fsim::PropertyHandle pulseSecondCmd;
};

struct ReachSample {
    double tS = 0.0;
    double dist3dM = 0.0;
    double speedMs = 0.0;
    double altM = 0.0;
};

struct ReachCurve {
    double launchAltM = 0.0;
    double launchMach = 0.0;
    double targetAltM = 0.0;
    double maxDistM = 0.0;
    double maxSpeedMs = 0.0;
    std::vector<ReachSample> samples;
};

struct ReachParams {
    double launchAltM = 6000.0;
    double launchMach = 0.9;
    double targetDhM = 0.0;
    double offBoresightDeg = 0.0;
    double thrustScale = 1.0;
    double dragScale = 1.0;
    double lifeScale = 1.0;
};

class MissileFlyoutWorker {
public:
    MissileFlyoutWorker(const MissileConfig& cfg,
                        std::shared_ptr<const fsim::sim::GroundProvider> ground,
                        std::mutex* resetMutex)
        : cfg_(cfg), resetMutex_(resetMutex) {
        model_ = std::make_unique<fsim::sim::JsbsimModel>(kFdmDtS, std::move(ground));
        fsim::sim::AircraftSpec spec;
        spec.name = cfg_.modelName;
        spec.jsbsimRoot = cfg_.jsbsimRoot;
        spec.aircraftDir = cfg_.aircraftDir;

        fsim::sim::InitialConditions ic;
        ic.latitudeDeg = 0.0;
        ic.longitudeDeg = 0.0;
        ic.altitudeMslM = 6000.0;
        ic.headingDeg = 0.0;
        ic.pitchDeg = 0.0;
        ic.rollDeg = 0.0;
        ic.airspeedTrueMs = 300.0;
        ic.enginesRunning = true;

        if (!model_->load(spec, ic)) {
            throw std::runtime_error("Failed to load JSBSim model: " + cfg_.modelName);
        }
        bindHandles();
    }

    // Record a full native 6-DOF JSBSim flyout trajectory toward a long-range
    // aimpoint at (launchAltM + targetDhM) along offBoresightDeg.
    ReachCurve recordReach(const ReachParams& p) {
        ReachCurve curve;
        curve.launchAltM = p.launchAltM;
        curve.launchMach = p.launchMach;
        curve.targetAltM = std::clamp(p.launchAltM + p.targetDhM, 500.0, 22000.0);

        const double launchSpeedMs = p.launchMach * fsim::ms::weapon::isaSoundSpeedMs(p.launchAltM);

        fsim::sim::InitialConditions ic;
        ic.latitudeDeg = 0.0;
        ic.longitudeDeg = 0.0;
        ic.altitudeMslM = p.launchAltM;
        ic.headingDeg = 0.0;
        ic.pitchDeg = 0.0;
        ic.rollDeg = 0.0;
        ic.airspeedTrueMs = std::max(50.0, launchSpeedMs);
        ic.enginesRunning = true;

        const double zero3[3] = {0.0, 0.0, 0.0};
        model_->setExternalForceBody(zero3, zero3);
        bool resetOk = false;
        if (resetMutex_) {
            std::lock_guard<std::mutex> lock(*resetMutex_);
            resetOk = model_->reset(ic);
        } else {
            resetOk = model_->reset(ic);
        }
        if (!resetOk) {
            return curve;
        }
        if (fin_.pulseSecondCmd.valid()) {
            fin_.pulseSecondCmd.set(0.0);
        }

        fsim::sim::VehicleState s;
        model_->state(s);

        const V3 launchPos{s.positionEcef[0], s.positionEcef[1], s.positionEcef[2]};
        const double launchRadius = vNorm(launchPos);
        const V3 launchUp = vUnit(launchPos);
        const V3 launchNorth{0.0, 0.0, 1.0};
        const V3 launchEast{0.0, 1.0, 0.0};

        // Far aimpoint at targetAltM along offBoresightDeg so Guidance executes its
        // authentic midcourse loft/turn/climb profile until battery expiry or energy floor.
        const double targetRadius = launchRadius + (curve.targetAltM - p.launchAltM);
        const double aimDistM = std::clamp(std::max(cfg_.publishedRangeKm * 1200.0, 45000.0), 45000.0, 350000.0);
        const double theta = std::clamp(aimDistM / launchRadius, 0.001, 0.20);
        const double azRad = p.offBoresightDeg * kDegToRad;
        const V3 horizDir = vAdd(vScale(launchNorth, std::cos(azRad)), vScale(launchEast, std::sin(azRad)));
        const V3 aimPos = vScale(vAdd(vScale(launchUp, std::cos(theta)), vScale(horizDir, std::sin(theta))), targetRadius);

        fsim::ms::weapon::Spec spec = cfg_.guidanceSpec;
        // Preserve authentic acquireM and loftRad so radar missiles fly genuine
        // Midcourse loft rather than prematurely activating terminal Homing at t=0.
        // For LOBL-only IR missiles without midcourse, enable inertial steering with
        // their own authored loftRad (0 deg) so they fly their climb/turn profile.
        fsim::ms::weapon::Spec reachSpec = spec;
        if (!reachSpec.inertialMidcourse) {
            reachSpec.inertialMidcourse = true;
        }
        reachSpec.launcherUpdates = true;

        fsim::ms::weapon::Observation aimObs;
        aimObs.id = 1;
        aimObs.position = aimPos;
        aimObs.velocity = {0.0, 0.0, 0.0};
        aimObs.signal = 1.0;

        fsim::ms::weapon::Guidance guidance(reachSpec, aimObs);

        std::optional<fsim::ms::weapon::PulseManagerV1> pulseMgr;
        if (cfg_.pulsePolicy) {
            pulseMgr.emplace(*cfg_.pulsePolicy);
        }

        const bool isSemiActive = (spec.seeker == "semi_active_radar");
        const double effectiveLifeS = std::max(spec.armingS + 1.0, spec.lifeS * p.lifeScale);
        const bool applyPerturbation = (std::abs(p.thrustScale - 1.0) > 1e-9 || std::abs(p.dragScale - 1.0) > 1e-9);
        const double derivativeVersion =
            (cfg_.plantAdapter.configured &&
             (cfg_.plantAdapter.implementations[0] == 2 || cfg_.plantAdapter.implementations[0] == 3))
                ? 2.0
                : 1.0;

        fsim::sim::ControlInputs ctrl;
        ctrl.setThrottleAll(1.0);
        ctrl.gearDown = 0.0;

        curve.samples.reserve(static_cast<std::size_t>(effectiveLifeS / 0.25) + 8);
        double ageS = 0.0;
        double demandEcef[3] = {0.0, 0.0, 0.0};
        int outerStep = 0;

        while (ageS <= effectiveLifeS + 1e-9) {
            model_->state(s);
            if (s.crashed || s.diverged || !std::isfinite(s.airspeedTrueMs) || s.altitudeMslM < 30.0) {
                break;
            }
            const V3 mPos{s.positionEcef[0], s.positionEcef[1], s.positionEcef[2]};
            const V3 mVel = velocityEcef(s);
            const double dist3d = vNorm(vSub(mPos, launchPos));
            curve.maxSpeedMs = std::max(curve.maxSpeedMs, s.airspeedTrueMs);

            // Sample at 6 Hz (every 5th 30 Hz step) plus exact boundary steps.
            if (outerStep % 5 == 0 || ageS + kOuterDtS > effectiveLifeS) {
                if (s.airspeedTrueMs >= kMinTerminalSpeedMs) {
                    curve.maxDistM = std::max(curve.maxDistM, dist3d);
                }
                curve.samples.push_back({ageS, dist3d, s.airspeedTrueMs, s.altitudeMslM});
            }

            // Stop early if propellant is exhausted and speed fell below kinematic floor.
            double totalThrustN = 0.0;
            for (int ei = 0; ei < s.engineCount && ei < fsim::sim::VehicleState::kMaxEngines; ++ei) {
                totalThrustN += s.thrustN[ei];
            }
            const auto energy = model_->energy();
            const bool fuelEmpty = (!std::isfinite(energy.fuelKg) || energy.fuelKg <= 0.05);
            if (ageS > 3.0 && totalThrustN < 1.0 && fuelEmpty && s.airspeedTrueMs < kMinTerminalSpeedMs * 0.90) {
                break;
            }
            if (vNorm(vSub(aimPos, mPos)) < 500.0) {
                break;
            }

            fsim::ms::weapon::Input gIn;
            gIn.position = mPos;
            gIn.velocity = mVel;
            gIn.up = {std::cos(s.latitudeRad) * std::cos(s.longitudeRad),
                      std::cos(s.latitudeRad) * std::sin(s.longitudeRad),
                      std::sin(s.latitudeRad)};
            gIn.boresight = {s.rotationBodyToEcef[0], s.rotationBodyToEcef[3], s.rotationBodyToEcef[6]};
            gIn.ageS = ageS;
            gIn.dtS = kOuterDtS;
            gIn.supported = true;
            gIn.illuminated = isSemiActive;
            gIn.launchCue = (ageS < 0.05) ? 1u : 0u;
            gIn.estimate = aimObs;
            if (vNorm(vSub(aimPos, mPos)) <= spec.acquireM) {
                gIn.visible.push_back(aimObs);
            }

            const auto gOut = guidance.update(gIn);
            demandEcef[0] = gOut.acceleration.x;
            demandEcef[1] = gOut.acceleration.y;
            demandEcef[2] = gOut.acceleration.z;

            for (int sub = 0; sub < kFrameSkip; ++sub) {
                model_->state(s);
                if (s.crashed || s.diverged) break;
                const double subTimeS = ageS + static_cast<double>(sub) * kFdmDtS;

                if (pulseMgr && fin_.pulseVersion.valid() && fin_.pulseVersion.get() == 1.0 &&
                    fin_.pulseAge.valid() && fin_.pulseSecondCmd.valid()) {
                    fsim::ms::weapon::PulseInputV1 pIn;
                    pIn.timeS = subTimeS;
                    pIn.nativeAgeS = fin_.pulseAge.get();
                    pIn.speedMs = s.airspeedTrueMs;
                    pIn.fuelKg = model_->energy().fuelKg;
                    if (std::isfinite(gOut.timeToGoS) && gOut.timeToGoS >= 0.0) {
                        pIn.timeToGoS = gOut.timeToGoS;
                    }
                    const auto dec = pulseMgr->update(pIn);
                    fin_.pulseSecondCmd.set((dec.valid && dec.ignite) ? 1.0 : 0.0);
                }

                if (applyPerturbation) {
                    double tN = 0.0;
                    for (int ei = 0; ei < s.engineCount && ei < fsim::sim::VehicleState::kMaxEngines; ++ei) {
                        tN += s.thrustN[ei];
                    }
                    const double aeroFxN = fin_.aeroForce[0].valid() ? (fin_.aeroForce[0].get() * kLbfToN) : 0.0;
                    const double deltaFxN = (p.thrustScale - 1.0) * tN + (p.dragScale - 1.0) * aeroFxN;
                    const double extF[3] = {std::isfinite(deltaFxN) ? deltaFxN : 0.0, 0.0, 0.0};
                    model_->setExternalForceBody(extF, zero3);
                }

                ctrl.aileron = ctrl.elevator = ctrl.rudder = 0.0;
                double cg[3] = {0.0, 0.0, 0.0};
                if (fin_.version.valid() && fin_.version.get() == derivativeVersion &&
                    model_->cgFromOriginBodyM(cg)) {
                    fsim::ms::weapon::AutopilotInput aIn;
                    toBody(s, demandEcef, aIn.demandBody.data());
                    const double grav[3] = {
                        -kG0 * std::cos(s.latitudeRad) * std::cos(s.longitudeRad),
                        -kG0 * std::cos(s.latitudeRad) * std::sin(s.longitudeRad),
                        -kG0 * std::sin(s.latitudeRad)};
                    toBody(s, grav, aIn.gravityBody.data());

                    aIn.massKg = model_->energy().massKg;
                    aIn.speedMs = s.airspeedTrueMs;
                    aIn.alphaRad = s.alphaRad;
                    aIn.betaRad = s.betaRad;
                    aIn.actuatorLagS = (fin_.lag.valid() && fin_.lag.get() > 0.0) ? (1.0 / fin_.lag.get()) : 0.04;

                    for (std::size_t k = 0; k < 3; ++k) {
                        aIn.ratesRadS[k] = s.angularRateBodyRadS[k];
                        aIn.inertiaKgM2[k] = fin_.inertia[k].get() * kSlugFt2ToKgM2;
                        aIn.limitsRad[k] = fin_.limits[k].get();
                    }

                    const double qS = fin_.qbar.get() * fin_.area.get() * kLbfToN;
                    const double chord = fin_.diameter.get() * kFtToM;
                    const double span = fin_.span.get() * kFtToM;
                    const double qSc = qS * chord;
                    const double qSb = qS * span;
                    const double pitchDamp = qSc * chord / (2.0 * std::max(aIn.speedMs, 1.0));
                    const double latDamp = qSb * span / (2.0 * std::max(aIn.speedMs, 1.0));
                    const double lever =
                        cg[0] - (fin_.referenceVersion.valid() ? fin_.aeroReferenceX.get() : 0.0);
                    auto c = [&](std::size_t idx) { return fin_.derivatives[idx].get(); };
                    aIn.pitch = {qS * c(0), qS * c(7), qSc * c(2) + lever * qS * c(0),
                                 qSc * c(9) + lever * qS * c(7), pitchDamp * c(4)};
                    aIn.yaw = {qS * c(1), qS * c(8), qSb * c(3) - lever * qS * c(1),
                               qSb * c(10) - lever * qS * c(8), latDamp * c(5)};
                    aIn.rollMomentPerRad = qSb * c(11);
                    aIn.rollDamping = latDamp * c(6);

                    fsim::ms::weapon::AeroFeedback measured;
                    measured.controlRad = {s.aileronRad, s.elevatorRad, s.rudderRad};
                    for (std::size_t k = 0; k < 3; ++k) {
                        measured.forceBodyN[k] = fin_.aeroForce[k].get() * kLbfToN;
                        measured.momentBodyNm[k] = fin_.aeroMoment[k].get() * kLbFtToNm;
                    }
                    if (cfg_.plantAdapter.configured && cfg_.plantAdapter.control.version == 1 &&
                        fin_.totalAxialForce.valid()) {
                        measured.forceBodyN[0] = fin_.totalAxialForce.get() * kLbfToN;
                    }
                    aIn.measured = measured;

                    const auto& controlCfg =
                        cfg_.plantAdapter.configured ? cfg_.plantAdapter.control : spec.control;
                    if (controlCfg.thrustVector && fin_.hasTvc && fin_.vectorVersion.valid() &&
                        fin_.vectorVersion.get() == 1.0) {
                        const double pitch = fin_.vectorPitch.get();
                        const double yaw = fin_.vectorYaw.get();
                        const double thrust = s.thrustN[0];
                        const double pitchGain = fin_.vectorPitchGain.get();
                        const double yawGain = fin_.vectorYawGain.get();
                        const double arm = fin_.vectorExitX.get() - cg[0];
                        if (std::isfinite(pitch) && std::isfinite(yaw) && std::isfinite(thrust) &&
                            std::isfinite(pitchGain) && std::isfinite(yawGain) && std::isfinite(arm)) {
                            const double fz = -thrust * std::cos(pitch) * pitchGain;
                            const double fy = thrust * std::cos(pitch) * std::cos(yaw) * yawGain;
                            aIn.pitch.forceControl += fz;
                            aIn.pitch.momentControl -= arm * fz;
                            aIn.yaw.forceControl += fy;
                            aIn.yaw.momentControl += arm * fy;
                            for (std::size_t k = 0; k < 3; ++k) {
                                if (k != 0 || !(cfg_.plantAdapter.configured && cfg_.plantAdapter.control.version == 1)) {
                                    aIn.measured->forceBodyN[k] += fin_.propulsionForce[k].get() * kLbfToN;
                                }
                                aIn.measured->momentBodyNm[k] += fin_.propulsionMoment[k].get() * kLbFtToNm;
                            }
                        }
                    }

                    fsim::ms::weapon::ControlStopsV1 stops;
                    stops.version = (fin_.normalizationVersion.valid() && fin_.normalizationVersion.get() == 1.0) ? 1u : 0u;
                    stops.actuatorVersion = (fin_.actuatorVersion.valid() && fin_.actuatorVersion.get() == 1.0) ? 1u : 0u;
                    stops.channelRateRadS = fin_.channelRate.valid() ? fin_.channelRate.get() : std::numeric_limits<double>::quiet_NaN();
                    for (std::size_t k = 0; k < 3; ++k) {
                        stops.lowerRad[k] = fin_.lowerStops[k].valid() ? fin_.lowerStops[k].get() : -aIn.limitsRad[k];
                        stops.upperRad[k] = fin_.upperStops[k].valid() ? fin_.upperStops[k].get() : aIn.limitsRad[k];
                    }
                    for (const auto& rh : fin_.finRates) {
                        if (rh.valid()) stops.finRateRadS.push_back(rh.get());
                    }

                    fsim::ms::weapon::FinCommands cmd;
                    if (controlCfg.version == 1 && stops.actuatorVersion == 1 &&
                        cfg_.plantAdapter.configured && cfg_.plantAdapter.implementations[0] >= 2) {
                        cmd = fsim::ms::weapon::actuatorAwareCommands(aIn, controlCfg, stops);
                    } else {
                        cmd = fsim::ms::weapon::finCommands(aIn, controlCfg, stops);
                    }
                    if (cmd.available) {
                        ctrl.aileron = cmd.aileron;
                        ctrl.elevator = cmd.elevator;
                        ctrl.rudder = cmd.rudder;
                    }
                }

                model_->step(ctrl);
            }

            ageS += kOuterDtS;
            ++outerStep;
        }

        return curve;
    }

private:
    void bindHandles() {
        auto& wm = *model_;
        fin_.version = wm.property("fsim/missile-control/version");
        fin_.qbar = wm.property("aero/qbar-psf");
        fin_.area = wm.property("metrics/Sw-sqft");
        fin_.diameter = wm.property("metrics/cbarw-ft");
        fin_.span = wm.property("metrics/bw-ft");
        fin_.referenceVersion = wm.property("fsim/missile-control/reference-version");
        fin_.aeroReferenceX = wm.property("fsim/missile-control/aero-reference-x-m");
        const char* forceAxes[] = {"forces/fbx-aero-lbs", "forces/fby-aero-lbs", "forces/fbz-aero-lbs"};
        const char* momentAxes[] = {"moments/l-aero-lbsft", "moments/m-aero-lbsft", "moments/n-aero-lbsft"};
        for (std::size_t k = 0; k < 3; ++k) {
            fin_.aeroForce[k] = wm.property(forceAxes[k]);
            fin_.aeroMoment[k] = wm.property(momentAxes[k]);
        }
        fin_.totalAxialForce = wm.property("forces/fbx-total-lbs");

        if (!wm.properties("fsim/thrust-vector-control").empty()) {
            fin_.hasTvc = true;
            const std::string pfx = "fsim/thrust-vector-control/";
            fin_.vectorVersion = wm.property(pfx + "version");
            fin_.vectorLimit = wm.property(pfx + "limit_rad");
            fin_.vectorRate = wm.property(pfx + "rate_rad_s");
            fin_.vectorPitchGain = wm.property(pfx + "pitch_gain");
            fin_.vectorYawGain = wm.property(pfx + "yaw_gain");
            fin_.vectorExitX = wm.property(pfx + "exit_x_body_m");
            fin_.vectorPitch = wm.property("propulsion/engine/pitch-angle-rad");
            fin_.vectorYaw = wm.property("propulsion/engine/yaw-angle-rad");
            const char* pForce[] = {"forces/fbx-prop-lbs", "forces/fby-prop-lbs", "forces/fbz-prop-lbs"};
            const char* pMoment[] = {"moments/l-prop-lbsft", "moments/m-prop-lbsft", "moments/n-prop-lbsft"};
            for (std::size_t k = 0; k < 3; ++k) {
                fin_.propulsionForce[k] = wm.property(pForce[k]);
                fin_.propulsionMoment[k] = wm.property(pMoment[k]);
            }
        }

        fin_.lag = wm.property("fsim/missile-control/lag-coefficient-s-1");
        fin_.normalizationVersion = wm.property("fsim/missile-control/normalization-version");
        fin_.actuatorVersion = wm.property("fsim/missile-control/actuator-version");
        fin_.channelRate = wm.property("fsim/missile-control/channel-rate-rad_s");
        fin_.finCount = wm.property("fsim/missile-control/fin-count");
        const double fc = fin_.finCount.valid() ? fin_.finCount.get() : 0.0;
        std::set<std::size_t> passiveFins;
        for (const auto& prop : wm.properties("fsim/passive-rolleron-v1")) {
            const auto& key = prop.first;
            if (key.size() > 10 && key.compare(key.size() - 10, 10, "/fin-index") == 0 &&
                std::isfinite(prop.second) && prop.second >= 0.0 && prop.second < fc &&
                prop.second == std::floor(prop.second)) {
                passiveFins.insert(static_cast<std::size_t>(prop.second));
            }
        }
        fin_.passiveFinCount = passiveFins.size();
        if (std::isfinite(fc) && fc >= 1.0 && fc <= 16.0 && std::floor(fc) == fc) {
            for (std::size_t k = 0; k < static_cast<std::size_t>(fc); ++k) {
                if (!passiveFins.count(k)) {
                    fin_.finRates.push_back(wm.property("fsim/missile-control/fin-" + std::to_string(k) + "-rate-rad_s"));
                }
            }
        }

        const char* axes[3] = {"aileron", "elevator", "rudder"};
        const char* inertia[3] = {"ixx", "iyy", "izz"};
        for (std::size_t k = 0; k < 3; ++k) {
            fin_.inertia[k] = wm.property(std::string("inertia/") + inertia[k] + "-slugs_ft2");
            fin_.limits[k] = wm.property(std::string("fsim/missile-control/") + axes[k] + "-limit-rad");
            fin_.lowerStops[k] = wm.property(std::string("fsim/missile-control/") + axes[k] + "-min-rad");
            fin_.upperStops[k] = wm.property(std::string("fsim/missile-control/") + axes[k] + "-max-rad");
        }
        const char* coeffs[12] = {
            "cz-alpha", "cy-beta", "cm-alpha", "cn-beta", "cm-q", "cn-r",
            "cl-p", "cz-elevator", "cy-rudder", "cm-elevator", "cn-rudder", "cl-aileron"};
        for (std::size_t k = 0; k < 12; ++k) {
            fin_.derivatives[k] = wm.property(std::string("fsim/control/") + coeffs[k]);
        }

        if (!wm.properties("fsim/motor-pulse").empty()) {
            fin_.pulseVersion = wm.property("fsim/motor-pulse/version");
            fin_.pulseAge = wm.property("fsim/motor-pulse/native-age-s");
            fin_.pulseSecondCmd = wm.property("fsim/motor-pulse/second-command");
        }
    }

    const MissileConfig& cfg_;
    std::mutex* resetMutex_ = nullptr;
    std::unique_ptr<fsim::sim::JsbsimModel> model_;
    FinHandles fin_;
};

double interpFactor(const std::vector<double>& degs, const std::vector<double>& factors, double deg) noexcept {
    if (degs.empty() || degs.size() != factors.size()) return 1.0;
    if (deg <= degs.front()) return factors.front();
    if (deg >= degs.back()) return factors.back();
    const auto it = std::upper_bound(degs.begin(), degs.end(), deg);
    const std::size_t i1 = static_cast<std::size_t>(it - degs.begin());
    const std::size_t i0 = i1 - 1;
    const double t = (deg - degs[i0]) / std::max(1e-9, degs[i1] - degs[i0]);
    return factors[i0] * (1.0 - t) + factors[i1] * t;
}

struct KnotEnvelope {
    double rMaxM = 0.0;
    double rNeM = 0.0;
    double rMinM = 0.0;
    double tofMaxS = 0.0;
    double tofNeS = 0.0;
    double tofMinS = 0.0;
};

// Derive exact (R_max, R_ne, R_min, tof_max, tof_ne, tof_min) for a given
// (targetMach, aspectDeg) directly from the native 6-DOF JSBSim ReachCurve
// and measured off-boresight turn-loss curve.
KnotEnvelope evaluateEnvelopeFromReach(const ReachCurve& curve,
                                       const MissileConfig& cfg,
                                       double targetMach,
                                       double aspectDeg,
                                       const std::vector<double>& offBoreDeg,
                                       const std::vector<double>& offBoreFactor) noexcept {
    KnotEnvelope env;
    if (curve.samples.empty()) return env;

    const double dh = curve.targetAltM - curve.launchAltM;
    const double absDh = std::abs(dh);
    const double vT = targetMach * fsim::ms::weapon::isaSoundSpeedMs(curve.targetAltM);
    const double aspRad = aspectDeg * kDegToRad;
    const double cosAsp = std::cos(aspRad);
    const double sinAsp = std::sin(aspRad);
    const double armingS = std::max(0.5, cfg.guidanceSpec.armingS);

    // 1. R_max & R_min (constant-velocity non-maneuvering target)
    double bestRMax = 0.0;
    double bestTofMax = 0.0;
    double firstRMin = 0.0;
    double firstTofMin = 0.0;
    bool haveMin = false;
    // Allow a small autopilot/seeker settling margin after arming for crossing aspects.
    const double minSettleS = armingS + 0.35 + 0.35 * sinAsp;

    for (const auto& smp : curve.samples) {
        if (smp.tS < armingS || smp.speedMs < kMinTerminalSpeedMs) continue;
        if (smp.dist3dM <= absDh + 1.0) continue;

        const double dT = vT * smp.tS;
        const double dHoriz0 = std::sqrt(std::max(0.0, smp.dist3dM * smp.dist3dM - dh * dh));
        const double yT = dT * sinAsp;
        const double xToward = dT * cosAsp;
        const double sinLead = std::clamp(std::abs(yT) / std::max(dHoriz0, 1.0), 0.0, 0.999);
        const double leadDeg = std::asin(sinLead) * kRadToDeg;
        const double dEff = smp.dist3dM * interpFactor(offBoreDeg, offBoreFactor, 0.5 * leadDeg);
        if (dEff <= absDh + 1.0) continue;

        const double dHe = std::sqrt(std::max(0.0, dEff * dEff - dh * dh));
        if (dHe < std::abs(yT)) continue;

        const double rH0 = xToward + std::sqrt(std::max(0.0, dHe * dHe - yT * yT));
        if (rH0 <= 0.0) continue;
        const double r0 = std::hypot(rH0, dh);

        if (!haveMin && smp.tS >= minSettleS) {
            haveMin = true;
            firstRMin = r0;
            firstTofMin = smp.tS;
        }
        if (r0 > bestRMax) {
            bestRMax = r0;
            bestTofMax = smp.tS;
        }
    }

    if (bestRMax <= 0.0) return env;

    env.rMaxM = bestRMax;
    env.tofMaxS = bestTofMax;
    if (haveMin) {
        env.rMinM = std::clamp(firstRMin, std::max(400.0, absDh), env.rMaxM);
        env.tofMinS = std::clamp(firstTofMin, armingS, env.tofMaxS);
    } else {
        env.rMinM = std::min(env.rMaxM, std::max(400.0, absDh));
        env.tofMinS = std::min(env.tofMaxS, armingS);
    }

    // 2. R_ne (immediate sustained 7g level break turn to 180 deg tail-away escape)
    const double psi0 = std::clamp(kPi - aspRad, 0.0, kPi);
    const double omega =
        (vT > 10.0) ? (std::sqrt(std::max(0.0, kEscapeTurnG * kEscapeTurnG - 1.0)) * kG0 / vT) : 0.0;
    const double rTurn = (omega > 1e-6) ? (vT / omega) : 0.0;
    const double tTurn = (omega > 1e-6) ? (psi0 / omega) : 0.0;

    double bestRNe = 0.0;
    double bestTofNe = 0.0;

    for (const auto& smp : curve.samples) {
        if (smp.tS < armingS || smp.speedMs < kMinTerminalSpeedMs) continue;
        if (smp.dist3dM <= absDh + 1.0) continue;

        double xAway = 0.0;
        double yCross = 0.0;
        if (psi0 <= 1e-6 || omega <= 1e-6) {
            xAway = vT * smp.tS;
            yCross = 0.0;
        } else if (smp.tS <= tTurn) {
            const double psi = psi0 - omega * smp.tS;
            xAway = rTurn * (std::sin(psi0) - std::sin(psi));
            yCross = rTurn * (std::cos(psi) - std::cos(psi0));
        } else {
            xAway = rTurn * std::sin(psi0) + vT * (smp.tS - tTurn);
            yCross = rTurn * (1.0 - std::cos(psi0));
        }

        const double dHoriz0 = std::sqrt(std::max(0.0, smp.dist3dM * smp.dist3dM - dh * dh));
        const double sinLead = std::clamp(std::abs(yCross) / std::max(dHoriz0, 1.0), 0.0, 0.999);
        const double leadDeg = std::asin(sinLead) * kRadToDeg;
        const double dEff = smp.dist3dM * interpFactor(offBoreDeg, offBoreFactor, 0.5 * leadDeg);
        if (dEff <= absDh + 1.0) continue;

        const double dHe = std::sqrt(std::max(0.0, dEff * dEff - dh * dh));
        if (dHe < std::abs(yCross)) continue;

        const double rH0 = std::sqrt(std::max(0.0, dHe * dHe - yCross * yCross)) - xAway;
        if (rH0 <= 0.0) continue;
        const double r0 = std::hypot(rH0, dh);

        if (r0 > bestRNe) {
            bestRNe = r0;
            bestTofNe = smp.tS;
        }
    }

    env.rNeM = std::clamp(bestRNe, 0.0, env.rMaxM);
    env.tofNeS = std::clamp(bestTofNe, 0.0, env.tofMaxS);
    return env;
}

bool loadMissileConfig(const std::filesystem::path& cardPath,
                       const std::filesystem::path& jsbsimRoot,
                       const fsim::io::AssetResolver& resolver,
                       MissileConfig& out,
                       std::string& error) {
    std::vector<std::string> cardErrors;
    auto card = fsim::ms::readStoreCard(cardPath, cardErrors);
    if (!card) {
        error = "failed to read store card: " + cardPath.string();
        return false;
    }
    if (card->storeClass != "aam" || card->model.rfind("jsbsim:", 0) != 0 || card->model.size() <= 7) {
        error = "not an AAM with a jsbsim model";
        return false;
    }

    const std::string modelName = card->model.substr(7);
    const std::filesystem::path aircraftDir = resolver.findAircraft(modelName, jsbsimRoot);
    if (aircraftDir.empty()) {
        error = "could not locate JSBSim aircraft directory for model: " + modelName;
        return false;
    }
    const std::filesystem::path xmlPath = aircraftDir / modelName / (modelName + ".xml");
    if (!std::filesystem::exists(xmlPath)) {
        error = "missing JSBSim XML: " + xmlPath.string();
        return false;
    }

    fsim::ms::components::ProfileV1 profile;
    if (!fsim::ms::components::readProfileV1(cardPath, *card, profile, error)) {
        return false;
    }

    fsim::ms::weapon::Spec spec;
    if (!fsim::ms::weapon::configureGuidance(*card, spec, error)) {
        return false;
    }
    if (const auto* seekerDesc = profile.find("seeker")) {
        if (!fsim::ms::weapon::configureSeeker(*seekerDesc, spec, error)) {
            return false;
        }
    }
    if (const auto* guidDesc = profile.find("guidance")) {
        if (!fsim::ms::weapon::configureGuidance(*guidDesc, spec, error)) {
            return false;
        }
    }

    fsim::ms::weapon::PlantAdapterV1 plant;
    if (profile.explicitProfile()) {
        const auto* ap = profile.find("autopilot");
        const auto* al = profile.find("allocator");
        const auto* ac = profile.find("actuator");
        const auto* pr = profile.find("propulsion");
        const auto* ae = profile.find("aerodynamics");
        if (ap && al && ac && pr && ae) {
            if (!fsim::ms::weapon::configurePlantV1(*ap, *al, *ac, *pr, *ae, *card, plant, error)) {
                return false;
            }
            plant.configured = true;
            if (plant.hasControl) {
                spec.control = plant.control;
            }
        }
        if (pr && plant.configured && plant.implementations[3] == 4) {
            fsim::ms::weapon::PulsePolicyV1 pulsePolicy;
            if (!fsim::ms::weapon::configurePulseV1(*pr, pulsePolicy, error)) {
                return false;
            }
            out.pulsePolicy = pulsePolicy;
        }
    }

    out.cardId = card->id;
    out.designation = card->designation;
    out.modelName = modelName;
    out.cardPath = cardPath;
    out.modelXmlPath = xmlPath;
    out.aircraftDir = aircraftDir;
    out.jsbsimRoot = jsbsimRoot;
    out.cardSha256 = fsim::ms::weapon::computeSha256Hex(readFileBytes(cardPath));
    out.modelXmlSha256 = fsim::ms::weapon::computeSha256Hex(readFileBytes(xmlPath));
    out.card = card;
    out.profile = std::move(profile);
    out.guidanceSpec = spec;
    out.plantAdapter = plant;

    if (const auto it = card->parameters.find("range_km"); it != card->parameters.end()) {
        out.publishedRangeKm = it->second.value;
        out.publishedRangeUncKm = it->second.uncertainty;
        out.publishedRangeSource = it->second.source;
        out.publishedRangeNote = it->second.note;
    }
    return true;
}

struct ComparisonRow {
    std::string cardId;
    std::string designation;
    std::string modelName;
    std::string seeker;
    double publishedRangeKm = 0.0;
    double publishedRangeUncKm = 0.0;
    std::string publishedRangeSource;

    double highAltHeadOnRMaxKm = 0.0;
    double highAltHeadOnRNeKm = 0.0;
    double highAltHeadOnTofS = 0.0;
    double midAltHeadOnRMaxKm = 0.0;
    double midAltHeadOnRNeKm = 0.0;
    double seaLevelHeadOnRMaxKm = 0.0;
    double midAltTailRMaxKm = 0.0;

    double ratioHighToPublished = 0.0;
    double residualKm = 0.0;
    double residualPercent = 0.0;

    double uncRMax = 0.0;
    double uncRNe = 0.0;
    double uncRMin = 0.0;
    double uncTof = 0.0;

    std::string tableSha256;
    std::string modelXmlSha256;
    std::string cardSha256;
    double generationSeconds = 0.0;
};

void writeDlzTableJson(const std::filesystem::path& outPath,
                       const MissileConfig& cfg,
                       const fsim::ms::weapon::LaunchZoneTableV1& tbl) {
    std::filesystem::create_directories(outPath.parent_path());
    std::ostringstream ss;

    auto writeArray = [&](const std::vector<double>& arr, int prec) {
        ss << "[";
        for (std::size_t i = 0; i < arr.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << formatNumber(arr[i], prec);
        }
        ss << "]";
    };

    ss << "{\n";
    ss << "  \"schema\": \"fsim.ms.dlz.v1\",\n";
    ss << "  \"card\": \"" << jsonEscape(cfg.cardId) << "\",\n";
    ss << "  \"model\": \"" << jsonEscape(cfg.modelName) << "\",\n";
    ss << "  \"seeker\": \"" << jsonEscape(cfg.guidanceSpec.seeker) << "\",\n";
    ss << "  \"basis\": \"simulated\",\n";
    ss << "  \"equipment_calibrated\": false,\n";
    ss << "  \"model_xml_sha256\": \"" << cfg.modelXmlSha256 << "\",\n";
    ss << "  \"card_sha256\": \"" << cfg.cardSha256 << "\",\n";
    ss << "  \"arming_s\": " << formatNumber(tbl.armingS, 3) << ",\n";
    ss << "  \"life_s\": " << formatNumber(tbl.lifeS, 2) << ",\n";
    ss << "  \"active_range_m\": " << formatNumber(tbl.activeRangeM, 1) << ",\n";
    ss << "  \"max_off_boresight_rad\": " << formatNumber(tbl.maxOffBoresightRad, 6) << ",\n";
    ss << "  \"escape_turn_g\": " << formatNumber(tbl.escapeTurnG, 2) << ",\n";
    ss << "  \"min_terminal_speed_ms\": " << formatNumber(tbl.minTerminalSpeedMs, 1) << ",\n";
    ss << "  \"axes\": {\n";
    ss << "    \"launch_altitude_m\": "; writeArray(tbl.launchAltitudeM, 1); ss << ",\n";
    ss << "    \"launch_mach\": "; writeArray(tbl.launchMach, 2); ss << ",\n";
    ss << "    \"target_altitude_delta_m\": "; writeArray(tbl.targetAltitudeDeltaM, 1); ss << ",\n";
    ss << "    \"target_mach\": "; writeArray(tbl.targetMach, 2); ss << ",\n";
    ss << "    \"aspect_deg\": "; writeArray(tbl.aspectDeg, 1); ss << "\n";
    ss << "  },\n";
    ss << "  \"off_boresight_deg\": "; writeArray(tbl.offBoresightDeg, 1); ss << ",\n";
    ss << "  \"off_boresight_factor\": "; writeArray(tbl.offBoresightFactor, 4); ss << ",\n";
    ss << "  \"uncertainty_fraction\": {\n";
    ss << "    \"r_max\": " << formatNumber(tbl.uncRMax, 4) << ",\n";
    ss << "    \"r_ne\": " << formatNumber(tbl.uncRNe, 4) << ",\n";
    ss << "    \"r_min\": " << formatNumber(tbl.uncRMin, 4) << ",\n";
    ss << "    \"tof\": " << formatNumber(tbl.uncTof, 4) << "\n";
    ss << "  },\n";
    ss << "  \"grid\": {\n";
    ss << "    \"r_max_m\": "; writeArray(tbl.rMaxM, 1); ss << ",\n";
    ss << "    \"r_ne_m\": "; writeArray(tbl.rNeM, 1); ss << ",\n";
    ss << "    \"r_min_m\": "; writeArray(tbl.rMinM, 1); ss << ",\n";
    ss << "    \"tof_max_s\": "; writeArray(tbl.tofMaxS, 2); ss << ",\n";
    ss << "    \"tof_ne_s\": "; writeArray(tbl.tofNeS, 2); ss << ",\n";
    ss << "    \"tof_min_s\": "; writeArray(tbl.tofMinS, 2); ss << "\n";
    ss << "  }\n";
    ss << "}\n";

    std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
    if (!out.good()) {
        throw std::runtime_error("Failed to write DLZ table: " + outPath.string());
    }
    out << ss.str();
}

fsim::ms::weapon::LaunchZoneQueryV1 makeEvalQuery(double altM,
                                                  double launchMach,
                                                  double dhM,
                                                  double targetMach,
                                                  double aspectDeg,
                                                  double rangeM) {
    fsim::ms::weapon::LaunchZoneQueryV1 q;
    const double vL = launchMach * fsim::ms::weapon::isaSoundSpeedMs(altM);
    const double tAlt = std::clamp(altM + dhM, 500.0, 22000.0);
    const double vT = targetMach * fsim::ms::weapon::isaSoundSpeedMs(tAlt);

    q.launchPosEcef[0] = kEarthRadiusM + altM;
    q.launchPosEcef[1] = 0.0;
    q.launchPosEcef[2] = 0.0;
    q.launchVelEcef[0] = 0.0;
    q.launchVelEcef[1] = 0.0;
    q.launchVelEcef[2] = vL;
    q.launchForwardEcef[0] = 0.0;
    q.launchForwardEcef[1] = 0.0;
    q.launchForwardEcef[2] = 1.0;
    q.launchAltitudeM = altM;
    q.launchAirspeedMs = vL;

    const double aspRad = aspectDeg * kDegToRad;
    q.targetPosEcef[0] = kEarthRadiusM + tAlt;
    q.targetPosEcef[1] = 0.0;
    q.targetPosEcef[2] = rangeM;
    q.targetVelEcef[0] = 0.0;
    q.targetVelEcef[1] = vT * std::sin(aspRad);
    q.targetVelEcef[2] = -vT * std::cos(aspRad);
    q.targetAltitudeM = tAlt;
    return q;
}

ComparisonRow generateForMissile(const MissileConfig& cfg,
                                 const std::filesystem::path& aircraftStoresDir,
                                 unsigned jobs) {
    const auto t0 = std::chrono::steady_clock::now();

    fsim::ms::weapon::LaunchZoneTableV1 tbl;
    tbl.schema = "fsim.ms.dlz.v1";
    tbl.card = cfg.cardId;
    tbl.model = cfg.modelName;
    tbl.seeker = cfg.guidanceSpec.seeker;
    tbl.basis = "simulated";
    tbl.equipmentCalibrated = false;
    tbl.modelXmlSha256 = cfg.modelXmlSha256;
    tbl.cardSha256 = cfg.cardSha256;

    tbl.armingS = cfg.guidanceSpec.armingS;
    tbl.lifeS = cfg.guidanceSpec.lifeS;
    tbl.activeRangeM = (cfg.guidanceSpec.seeker == "active_radar") ? cfg.guidanceSpec.acquireM : 0.0;
    tbl.maxOffBoresightRad = std::clamp(cfg.guidanceSpec.gimbalRad, 0.25, kPi - 0.05);
    tbl.escapeTurnG = kEscapeTurnG;
    tbl.minTerminalSpeedMs = kMinTerminalSpeedMs;

    tbl.launchAltitudeM = {1000.0, 6000.0, 12000.0, 18000.0};
    tbl.launchMach = {0.6, 0.9, 1.4};
    tbl.targetAltitudeDeltaM = {-4000.0, 0.0, 4000.0};
    tbl.targetMach = {0.7, 1.0, 1.4};
    tbl.aspectDeg = {0.0, 45.0, 90.0, 135.0, 180.0};

    const double maxDeg = std::min(60.0, tbl.maxOffBoresightRad * kRadToDeg);
    tbl.offBoresightDeg = {0.0, 0.25 * maxDeg, 0.50 * maxDeg, 0.75 * maxDeg, maxDeg};

    const std::size_t n0 = tbl.launchAltitudeM.size();
    const std::size_t n1 = tbl.launchMach.size();
    const std::size_t n2 = tbl.targetAltitudeDeltaM.size();
    const std::size_t n3 = tbl.targetMach.size();
    const std::size_t n4 = tbl.aspectDeg.size();

    // Build all 48 independent native 6-DOF JSBSim flyout tasks:
    //  [0..35]: 36 nominal launch conditions (alt x launchMach x dh)
    //  [36..39]: 4 off-boresight flyouts at (6 km, M0.9, dh=0)
    //  [40..47]: 8 perturbed flyouts (4 anchors x {lower, upper})
    struct FlyoutTask {
        ReachParams params;
    };
    std::vector<FlyoutTask> tasks;
    tasks.reserve(n0 * n1 * n2 + 4 + 8);

    // 36 nominal reach trajectories
    for (std::size_t i0 = 0; i0 < n0; ++i0) {
        for (std::size_t i1 = 0; i1 < n1; ++i1) {
            for (std::size_t i2 = 0; i2 < n2; ++i2) {
                ReachParams p;
                p.launchAltM = tbl.launchAltitudeM[i0];
                p.launchMach = tbl.launchMach[i1];
                p.targetDhM = tbl.targetAltitudeDeltaM[i2];
                tasks.push_back({p});
            }
        }
    }
    const std::size_t offBoreOffset = tasks.size();
    for (std::size_t k = 1; k < tbl.offBoresightDeg.size(); ++k) {
        ReachParams p;
        p.launchAltM = 6000.0;
        p.launchMach = 0.9;
        p.targetDhM = 0.0;
        p.offBoresightDeg = tbl.offBoresightDeg[k];
        tasks.push_back({p});
    }

    const std::size_t pertOffset = tasks.size();
    struct PertAnchor {
        std::size_t i0;
        std::size_t i1;
        std::size_t i2;
    };
    // Anchors: (1km, M0.9, 0), (1km, M1.4, 0), (12km, M0.9, 0), (12km, M1.4, 0)
    const std::array<PertAnchor, 4> pertAnchors = {{{0, 1, 1}, {0, 2, 1}, {2, 1, 1}, {2, 2, 1}}};
    for (const auto& a : pertAnchors) {
        ReachParams low;
        low.launchAltM = tbl.launchAltitudeM[a.i0];
        low.launchMach = tbl.launchMach[a.i1];
        low.targetDhM = tbl.targetAltitudeDeltaM[a.i2];
        low.thrustScale = 0.90;
        low.dragScale = 1.15;
        low.lifeScale = 0.95;
        tasks.push_back({low});

        ReachParams hi;
        hi.launchAltM = tbl.launchAltitudeM[a.i0];
        hi.launchMach = tbl.launchMach[a.i1];
        hi.targetDhM = tbl.targetAltitudeDeltaM[a.i2];
        hi.thrustScale = 1.10;
        hi.dragScale = 0.85;
        hi.lifeScale = 1.05;
        tasks.push_back({hi});
    }

    std::vector<ReachCurve> curves(tasks.size());
    const unsigned workerCount = std::max(1u, std::min<unsigned>(jobs, static_cast<unsigned>(tasks.size())));
    auto ground = std::make_shared<fsim::sim::FlatGround>(0.0);
    std::mutex resetMutex;

    // Construct & load all JSBSim instances on the main thread per JsbsimModel contract.
    std::vector<std::unique_ptr<MissileFlyoutWorker>> workers;
    workers.reserve(workerCount);
    for (unsigned t = 0; t < workerCount; ++t) {
        workers.push_back(std::make_unique<MissileFlyoutWorker>(cfg, ground, &resetMutex));
    }

    std::atomic<std::size_t> nextTask{0};
    std::mutex errMutex;
    std::string workerError;
    std::vector<std::thread> threads;
    threads.reserve(workerCount);
    for (unsigned t = 0; t < workerCount; ++t) {
        threads.emplace_back([&, t]() {
            try {
                MissileFlyoutWorker& worker = *workers[t];
                while (true) {
                    const std::size_t idx = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (idx >= tasks.size()) break;
                    curves[idx] = worker.recordReach(tasks[idx].params);
                }
            } catch (const std::exception& ex) {
                std::lock_guard<std::mutex> lock(errMutex);
                if (workerError.empty()) workerError = ex.what();
            }
        });
    }
    for (auto& th : threads) th.join();
    if (!workerError.empty()) {
        throw std::runtime_error(workerError);
    }

    // 1. Extract off-boresight kinematic attenuation curve from real 6-DOF flyouts.
    // Reference boresight curve is (i0=1: 6km, i1=1: M0.9, i2=1: dh=0).
    const std::size_t refIdx = (1 * n1 + 1) * n2 + 1;
    const double refMaxDist = curves[refIdx].maxDistM;
    tbl.offBoresightFactor.assign(tbl.offBoresightDeg.size(), 1.0);
    double prevF = 1.0;
    for (std::size_t k = 1; k < tbl.offBoresightDeg.size(); ++k) {
        const double offDist = curves[offBoreOffset + (k - 1)].maxDistM;
        double f = (refMaxDist > 100.0 && offDist > 0.0) ? (offDist / refMaxDist) : (prevF * 0.94);
        f = std::clamp(f, 0.25, prevF);
        tbl.offBoresightFactor[k] = f;
        prevF = f;
    }

    // 2. Populate the full 540-point 5-D grid deterministically from the 36 native 6-DOF reach curves.
    const std::size_t total = tbl.gridSize();
    tbl.rMaxM.assign(total, 0.0);
    tbl.rNeM.assign(total, 0.0);
    tbl.rMinM.assign(total, 0.0);
    tbl.tofMaxS.assign(total, 0.0);
    tbl.tofNeS.assign(total, 0.0);
    tbl.tofMinS.assign(total, 0.0);

    const std::size_t s3 = n4;
    const std::size_t s2 = n3 * s3;
    const std::size_t s1 = n2 * s2;
    const std::size_t s0 = n1 * s1;

    for (std::size_t i0 = 0; i0 < n0; ++i0) {
        for (std::size_t i1 = 0; i1 < n1; ++i1) {
            for (std::size_t i2 = 0; i2 < n2; ++i2) {
                const ReachCurve& rc = curves[(i0 * n1 + i1) * n2 + i2];
                for (std::size_t i3 = 0; i3 < n3; ++i3) {
                    for (std::size_t i4 = 0; i4 < n4; ++i4) {
                        const KnotEnvelope env = evaluateEnvelopeFromReach(
                            rc, cfg, tbl.targetMach[i3], tbl.aspectDeg[i4],
                            tbl.offBoresightDeg, tbl.offBoresightFactor);
                        const std::size_t flat = i0 * s0 + i1 * s1 + i2 * s2 + i3 * s3 + i4;
                        tbl.rMaxM[flat] = env.rMaxM;
                        tbl.rNeM[flat] = env.rNeM;
                        tbl.rMinM[flat] = env.rMinM;
                        tbl.tofMaxS[flat] = env.tofMaxS;
                        tbl.tofNeS[flat] = env.tofNeS;
                        tbl.tofMinS[flat] = env.tofMinS;
                    }
                }
            }
        }
    }

    // 3. Propagate 1-sigma RMS fractional uncertainties from the 8 perturbed 6-DOF flyouts.
    {
        double sumSqMax = 0.0, sumSqNe = 0.0, sumSqMin = 0.0, sumSqTof = 0.0;
        int nMax = 0, nNe = 0, nMin = 0, nTof = 0;
        for (std::size_t aIdx = 0; aIdx < pertAnchors.size(); ++aIdx) {
            const auto& a = pertAnchors[aIdx];
            const ReachCurve& rcNom = curves[(a.i0 * n1 + a.i1) * n2 + a.i2];
            const ReachCurve& rcLow = curves[pertOffset + 2 * aIdx];
            const ReachCurve& rcHi = curves[pertOffset + 2 * aIdx + 1];

            for (double asp : {0.0, 90.0, 180.0}) {
                const KnotEnvelope nom = evaluateEnvelopeFromReach(rcNom, cfg, 1.0, asp, tbl.offBoresightDeg, tbl.offBoresightFactor);
                const KnotEnvelope low = evaluateEnvelopeFromReach(rcLow, cfg, 1.0, asp, tbl.offBoresightDeg, tbl.offBoresightFactor);
                const KnotEnvelope hi = evaluateEnvelopeFromReach(rcHi, cfg, 1.0, asp, tbl.offBoresightDeg, tbl.offBoresightFactor);
                if (nom.rMaxM > 500.0 && hi.rMaxM > 0.0 && low.rMaxM > 0.0) {
                    const double f = std::abs(hi.rMaxM - low.rMaxM) / (2.0 * nom.rMaxM);
                    sumSqMax += f * f;
                    ++nMax;
                }
                if (nom.rNeM > 500.0 && hi.rNeM > 0.0 && low.rNeM > 0.0) {
                    const double f = std::abs(hi.rNeM - low.rNeM) / (2.0 * nom.rNeM);
                    sumSqNe += f * f;
                    ++nNe;
                }
                if (nom.rMinM > 100.0 && hi.rMinM > 0.0 && low.rMinM > 0.0) {
                    const double f = std::abs(hi.rMinM - low.rMinM) / (2.0 * nom.rMinM);
                    sumSqMin += f * f;
                    ++nMin;
                }
                if (nom.tofMaxS > 1.0 && hi.tofMaxS > 0.0 && low.tofMaxS > 0.0) {
                    const double f = std::abs(hi.tofMaxS - low.tofMaxS) / (2.0 * nom.tofMaxS);
                    sumSqTof += f * f;
                    ++nTof;
                }
            }
        }
        tbl.uncRMax = std::clamp(nMax > 0 ? std::sqrt(sumSqMax / static_cast<double>(nMax)) : 0.15, 0.05, 0.50);
        tbl.uncRNe = std::clamp(nNe > 0 ? std::sqrt(sumSqNe / static_cast<double>(nNe)) : tbl.uncRMax, 0.05, 0.50);
        tbl.uncRMin = std::clamp(nMin > 0 ? std::sqrt(sumSqMin / static_cast<double>(nMin)) : 0.12, 0.05, 0.50);
        tbl.uncTof = std::clamp(nTof > 0 ? std::sqrt(sumSqTof / static_cast<double>(nTof)) : 0.12, 0.05, 0.50);
    }

    const std::filesystem::path outPath = aircraftStoresDir / cfg.modelName / (cfg.modelName + ".dlz-v1.json");
    writeDlzTableJson(outPath, cfg, tbl);

    // Reload via production validator to guarantee schema & monotonicity integrity.
    fsim::ms::weapon::LaunchZoneTableV1 verified;
    std::string loadErr;
    if (!fsim::ms::weapon::loadLaunchZoneTableV1(outPath, verified, loadErr)) {
        throw std::runtime_error("Generated table failed loadLaunchZoneTableV1: " + loadErr);
    }

    const auto evalHigh = fsim::ms::weapon::evaluateLaunchZoneV2(
        verified, makeEvalQuery(12000.0, 1.2, 0.0, 0.9, 0.0, 20000.0));
    const auto evalHighAtMax = fsim::ms::weapon::evaluateLaunchZoneV2(
        verified, makeEvalQuery(12000.0, 1.2, 0.0, 0.9, 0.0, std::max(1000.0, evalHigh.maximumM)));
    const auto evalMid = fsim::ms::weapon::evaluateLaunchZoneV2(
        verified, makeEvalQuery(6000.0, 0.9, 0.0, 0.9, 0.0, 20000.0));
    const auto evalLow = fsim::ms::weapon::evaluateLaunchZoneV2(
        verified, makeEvalQuery(1000.0, 0.9, 0.0, 0.9, 0.0, 10000.0));
    const auto evalTail = fsim::ms::weapon::evaluateLaunchZoneV2(
        verified, makeEvalQuery(6000.0, 0.9, 0.0, 0.9, 180.0, 10000.0));

    const auto t1 = std::chrono::steady_clock::now();
    ComparisonRow row;
    row.cardId = cfg.cardId;
    row.designation = cfg.designation;
    row.modelName = cfg.modelName;
    row.seeker = cfg.guidanceSpec.seeker;
    row.publishedRangeKm = cfg.publishedRangeKm;
    row.publishedRangeUncKm = cfg.publishedRangeUncKm;
    row.publishedRangeSource = cfg.publishedRangeSource;
    row.highAltHeadOnRMaxKm = evalHigh.maximumM / 1000.0;
    row.highAltHeadOnRNeKm = evalHigh.noEscapeM / 1000.0;
    row.highAltHeadOnTofS = std::isfinite(evalHighAtMax.timeOfFlightS) ? evalHighAtMax.timeOfFlightS : 0.0;
    row.midAltHeadOnRMaxKm = evalMid.maximumM / 1000.0;
    row.midAltHeadOnRNeKm = evalMid.noEscapeM / 1000.0;
    row.seaLevelHeadOnRMaxKm = evalLow.maximumM / 1000.0;
    row.midAltTailRMaxKm = evalTail.maximumM / 1000.0;
    if (row.publishedRangeKm > 0.0) {
        row.ratioHighToPublished = row.highAltHeadOnRMaxKm / row.publishedRangeKm;
        row.residualKm = row.highAltHeadOnRMaxKm - row.publishedRangeKm;
        row.residualPercent = 100.0 * row.residualKm / row.publishedRangeKm;
    }
    row.uncRMax = verified.uncRMax;
    row.uncRNe = verified.uncRNe;
    row.uncRMin = verified.uncRMin;
    row.uncTof = verified.uncTof;
    row.tableSha256 = verified.tableSha256;
    row.modelXmlSha256 = verified.modelXmlSha256;
    row.cardSha256 = verified.cardSha256;
    row.generationSeconds = std::chrono::duration<double>(t1 - t0).count();
    return row;
}

void writeComparisonReports(const std::filesystem::path& reportDir,
                            const std::vector<ComparisonRow>& rows) {
    std::filesystem::create_directories(reportDir);

    // 1. JSON report
    {
        std::ostringstream ss;
        ss << "{\n";
        ss << "  \"schema\": \"fsim.research.sim_m04.dlz_published_comparison.v1\",\n";
        ss << "  \"basis\": \"simulated\",\n";
        ss << "  \"equipment_calibrated\": false,\n";
        ss << "  \"evaluation_condition_high_alt\": {\n";
        ss << "    \"launch_altitude_m\": 12000.0,\n";
        ss << "    \"launch_mach\": 1.2,\n";
        ss << "    \"target_altitude_m\": 12000.0,\n";
        ss << "    \"target_mach\": 0.9,\n";
        ss << "    \"aspect_deg\": 0.0\n";
        ss << "  },\n";
        ss << "  \"escape_turn_g\": " << formatNumber(kEscapeTurnG, 1) << ",\n";
        ss << "  \"min_terminal_speed_ms\": " << formatNumber(kMinTerminalSpeedMs, 1) << ",\n";
        ss << "  \"note\": \"Flyout-derived envelopes from un-tuned native 6-DOF JSBSim missile models. No aerodynamic or propulsion parameters were fit to published ranges.\",\n";
        ss << "  \"missiles\": [\n";
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const auto& r = rows[i];
            ss << "    {\n";
            ss << "      \"card\": \"" << jsonEscape(r.cardId) << "\",\n";
            ss << "      \"designation\": \"" << jsonEscape(r.designation) << "\",\n";
            ss << "      \"model\": \"" << jsonEscape(r.modelName) << "\",\n";
            ss << "      \"seeker\": \"" << jsonEscape(r.seeker) << "\",\n";
            ss << "      \"published_range_km\": " << formatNumber(r.publishedRangeKm, 2) << ",\n";
            ss << "      \"published_range_unc_km\": " << formatNumber(r.publishedRangeUncKm, 2) << ",\n";
            ss << "      \"published_range_source\": \"" << jsonEscape(r.publishedRangeSource) << "\",\n";
            ss << "      \"flyout_high_alt_head_on_r_max_km\": " << formatNumber(r.highAltHeadOnRMaxKm, 2) << ",\n";
            ss << "      \"flyout_high_alt_head_on_r_ne_km\": " << formatNumber(r.highAltHeadOnRNeKm, 2) << ",\n";
            ss << "      \"flyout_high_alt_head_on_tof_s\": " << formatNumber(r.highAltHeadOnTofS, 2) << ",\n";
            ss << "      \"flyout_mid_alt_head_on_r_max_km\": " << formatNumber(r.midAltHeadOnRMaxKm, 2) << ",\n";
            ss << "      \"flyout_mid_alt_head_on_r_ne_km\": " << formatNumber(r.midAltHeadOnRNeKm, 2) << ",\n";
            ss << "      \"flyout_sea_level_head_on_r_max_km\": " << formatNumber(r.seaLevelHeadOnRMaxKm, 2) << ",\n";
            ss << "      \"flyout_mid_alt_tail_r_max_km\": " << formatNumber(r.midAltTailRMaxKm, 2) << ",\n";
            ss << "      \"ratio_high_to_published\": " << formatNumber(r.ratioHighToPublished, 3) << ",\n";
            ss << "      \"residual_km\": " << formatNumber(r.residualKm, 2) << ",\n";
            ss << "      \"residual_percent\": " << formatNumber(r.residualPercent, 1) << ",\n";
            ss << "      \"uncertainty_fraction\": {\n";
            ss << "        \"r_max\": " << formatNumber(r.uncRMax, 4) << ",\n";
            ss << "        \"r_ne\": " << formatNumber(r.uncRNe, 4) << ",\n";
            ss << "        \"r_min\": " << formatNumber(r.uncRMin, 4) << ",\n";
            ss << "        \"tof\": " << formatNumber(r.uncTof, 4) << "\n";
            ss << "      },\n";
            ss << "      \"table_sha256\": \"" << r.tableSha256 << "\",\n";
            ss << "      \"model_xml_sha256\": \"" << r.modelXmlSha256 << "\",\n";
            ss << "      \"card_sha256\": \"" << r.cardSha256 << "\",\n";
            ss << "      \"generation_seconds\": " << formatNumber(r.generationSeconds, 2) << "\n";
            ss << "    }" << (i + 1 < rows.size() ? "," : "") << "\n";
        }
        ss << "  ]\n";
        ss << "}\n";
        std::ofstream out(reportDir / "dlz-published-comparison.json", std::ios::binary | std::ios::trunc);
        out << ss.str();
    }

    // 2. Markdown report
    {
        std::ostringstream ss;
        ss << "# SIM-M04 (MS-8) Flyout-Derived DLZ vs. Published Range Comparison\n\n";
        ss << "- **Basis**: `simulated` (native 6-DOF JSBSim flyouts, `dt = 1/120 s` with authored integration-v1 substeps, `frameSkip = 4`, `equipment_calibrated = false`)\n";
        ss << "- **Reference high-altitude head-on condition**: `12 km MSL`, launch `M 1.2`, co-altitude target `M 0.9`, `0 deg` head-on aspect\n";
        ss << "- **Mid-altitude condition**: `6 km MSL`, launch `M 0.9`, co-altitude target `M 0.9`\n";
        ss << "- **Sea-level condition**: `1 km MSL`, launch `M 0.9`, co-altitude target `M 0.9`, `0 deg` head-on\n";
        ss << "- **No-escape maneuver**: immediate sustained `7 g` level break turn to `180 deg` tail-away escape\n";
        ss << "- **Uncertainty propagation**: RMS half-spread across perturbed native 6-DOF flyouts (`+/-10%` thrust, `-/+15%` drag, `+/-5%` battery life)\n";
        ss << "- **Honesty notice**: No aerodynamic drag, lift, or rocket propulsion parameters were tuned or fit to published figures. Residuals reflect honest model-versus-open-source differences and are bounded by the propagated 1-sigma uncertainty intervals.\n\n";
        ss << "| Card | Designation | Seeker | Published (km) | High-Alt R_max (km) | High-Alt R_ne (km) | Mid-Alt R_max (km) | Mid-Alt R_ne (km) | Low-Alt R_max (km) | Tail R_max (km) | Ratio | Residual (%) | Unc R_max (1σ) |\n";
        ss << "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
        for (const auto& r : rows) {
            ss << "| `" << r.cardId << "` | " << r.designation << " | `" << r.seeker << "` | "
               << formatNumber(r.publishedRangeKm, 1) << " ± " << formatNumber(r.publishedRangeUncKm, 1) << " | "
               << formatNumber(r.highAltHeadOnRMaxKm, 1) << " | "
               << formatNumber(r.highAltHeadOnRNeKm, 1) << " | "
               << formatNumber(r.midAltHeadOnRMaxKm, 1) << " | "
               << formatNumber(r.midAltHeadOnRNeKm, 1) << " | "
               << formatNumber(r.seaLevelHeadOnRMaxKm, 1) << " | "
               << formatNumber(r.midAltTailRMaxKm, 1) << " | "
               << formatNumber(r.ratioHighToPublished, 2) << " | "
               << (r.residualPercent >= 0.0 ? "+" : "") << formatNumber(r.residualPercent, 1) << "% | "
               << formatNumber(100.0 * r.uncRMax, 1) << "% |\n";
        }
        ss << "\n## Provenance Checksums\n\n";
        ss << "| Card | Model | Table SHA-256 (prefix) | XML SHA-256 (prefix) | Card SHA-256 (prefix) | Gen Time (s) |\n";
        ss << "|---|---|---|---|---|---:|\n";
        for (const auto& r : rows) {
            ss << "| `" << r.cardId << "` | `" << r.modelName << "` | `"
               << r.tableSha256.substr(0, 16) << "` | `"
               << r.modelXmlSha256.substr(0, 16) << "` | `"
               << r.cardSha256.substr(0, 16) << "` | "
               << formatNumber(r.generationSeconds, 1) << " |\n";
        }
        std::ofstream out(reportDir / "dlz-published-comparison.md", std::ios::binary | std::ios::trunc);
        out << ss.str();
    }
}

} // namespace

int main(int argc, char** argv) {
    fsim::log::setLevel(fsim::log::Level::Error);

    std::set<std::string> onlyCards;
    unsigned jobs = std::min(12u, std::max(1u, std::thread::hardware_concurrency()));
    std::filesystem::path storesDir;
    std::filesystem::path aircraftStoresDir = "aircraft/stores";
    std::filesystem::path reportDir = "docs/research/sim-m04";
    bool writeReport = true;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--only" && i + 1 < argc) {
            onlyCards.insert(argv[++i]);
        } else if (arg == "--jobs" && i + 1 < argc) {
            jobs = std::max(1u, static_cast<unsigned>(std::stoul(argv[++i])));
        } else if (arg == "--stores-dir" && i + 1 < argc) {
            storesDir = argv[++i];
        } else if (arg == "--aircraft-stores-dir" && i + 1 < argc) {
            aircraftStoresDir = argv[++i];
        } else if (arg == "--report-dir" && i + 1 < argc) {
            reportDir = argv[++i];
        } else if (arg == "--no-report") {
            writeReport = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: fsim-dlz-generator [--only <card-id>]... [--jobs N] "
                         "[--stores-dir DIR] [--aircraft-stores-dir DIR] [--report-dir DIR] [--no-report]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return 2;
        }
    }

    fsim::io::AssetResolver resolver;
    const auto jsbsimRoot = resolver.jsbsimRoot();
    if (!jsbsimRoot) {
        std::cerr << "ERROR: Unable to locate JSBSim root directory\n";
        return 1;
    }
    if (storesDir.empty()) {
        if (auto found = resolver.find("stores")) {
            storesDir = *found;
        } else if (std::filesystem::exists("assets/stores")) {
            storesDir = "assets/stores";
        } else {
            std::cerr << "ERROR: Unable to locate assets/stores directory\n";
            return 1;
        }
    }

    std::vector<std::filesystem::path> cardFiles;
    for (const auto& entry : std::filesystem::directory_iterator(storesDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            cardFiles.push_back(entry.path());
        }
    }
    std::sort(cardFiles.begin(), cardFiles.end());

    std::vector<MissileConfig> configs;
    for (const auto& path : cardFiles) {
        MissileConfig cfg;
        std::string err;
        if (!loadMissileConfig(path, *jsbsimRoot, resolver, cfg, err)) {
            continue;
        }
        if (!onlyCards.empty() && !onlyCards.count(cfg.cardId)) {
            continue;
        }
        configs.push_back(std::move(cfg));
    }

    if (configs.empty()) {
        std::cerr << "ERROR: No matching AAM store cards found in " << storesDir.string() << "\n";
        return 1;
    }

    std::cout << "fsim-dlz-generator: generating " << configs.size()
              << " AAM flyout DLZ tables using " << jobs << " threads\n" << std::flush;

    std::vector<ComparisonRow> rows;
    rows.reserve(configs.size());
    const auto tStart = std::chrono::steady_clock::now();

    for (std::size_t idx = 0; idx < configs.size(); ++idx) {
        const auto& cfg = configs[idx];
        std::cout << "[" << (idx + 1) << "/" << configs.size() << "] " << cfg.cardId
                  << " (" << cfg.modelName << ", " << cfg.guidanceSpec.seeker << ") ... " << std::flush;
        try {
            auto row = generateForMissile(cfg, aircraftStoresDir, jobs);
            std::cout << "R_max(12km/M1.2)=" << formatNumber(row.highAltHeadOnRMaxKm, 1) << " km"
                      << " (pub " << formatNumber(row.publishedRangeKm, 1) << " km, "
                      << (row.residualPercent >= 0.0 ? "+" : "") << formatNumber(row.residualPercent, 1) << "%)"
                      << ", R_ne=" << formatNumber(row.highAltHeadOnRNeKm, 1) << " km"
                      << ", unc=" << formatNumber(100.0 * row.uncRMax, 1) << "%"
                      << " [" << formatNumber(row.generationSeconds, 1) << " s]\n" << std::flush;
            rows.push_back(std::move(row));
        } catch (const std::exception& ex) {
            std::cerr << "FAILED: " << ex.what() << "\n";
            return 1;
        }
    }

    if (writeReport && onlyCards.empty()) {
        writeComparisonReports(reportDir, rows);
        std::cout << "Wrote published-performance comparison reports to " << reportDir.string() << "\n" << std::flush;
    }

    const double totalS = std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
    std::cout << "Completed " << rows.size() << " AAM tables in " << formatNumber(totalS, 1) << " s\n" << std::flush;
    return 0;
}
