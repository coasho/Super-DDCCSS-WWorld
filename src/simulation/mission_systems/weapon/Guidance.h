#pragma once

#include "fsim/MissionSystems.h"
#include "Autopilot.h"
#include "Seeker.h"
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ms { struct StoreCard; }
namespace fsim::ms::weapon {
// All positions/velocities/commands use ECEF metres, metres/second and
// metres/second squared. up is the local outward unit direction. This core
// commands acceleration including gravity compensation; JSBSim remains the
// sole flight integrator. The caller owns expiry (lifeS), impact, damage and
// terminal events; Guidance must only be updated for a live armed release.
// Acquisition/range/loft and the legacy 3.5 navigation gain/10-degree cue
// are engineering assumptions. Versioned cards select explicit execution
// settings; the standard gravity conversion is 9.80665 m/s^2.
// visible must contain only actual sensor observations after range, gimbal,
// radar notch and infrared aspect/decoy applicability filtering by the caller.
// The core defensively reapplies geometric range/gimbal limits. It never
// reads target truth or rolls decoy probabilities. Equal acquisition scores
// resolve by ascending observation ID, independent of input ordering.
struct V3 { double x = 0, y = 0, z = 0; };
struct Spec {
    std::string seeker;
    double acquireM = 17000, gimbalRad = 1.047, maxG = 35;
    double lifeS = 110, armingS = 1, fuzeM = 12, loftRad = .349;
    /// 0 retains unversioned callers' legacy execution. Version 1 uses both
    /// explicit authored facts; neither implies a fitted modeled transport.
    unsigned capabilityVersion = 0;
    bool inertialMidcourse = false, launcherUpdates = false;
    unsigned executionVersion = 0, guidanceAlgorithm = 1;
    double navigationGain = 3.5, cueRad = .17453292519943295;
    double activationRatio = 1.15, loftAccelerationG = 2;
    FinControlConfigV1 control{};
    unsigned fuzeModelVersion = 0, fuzeAlgorithm = 0;
    double fuzeDelayS = 0;
    std::optional<V3> fuzeStationStructuralM{}; ///< x aft from nose, y right, z up.
    SeekerConfigV1 seekerModel{}; ///< Version zero leaves legacy seeker execution unchanged.
};
/// Parse independent optional execution/fuze blocks from the existing numeric
/// card parameters. An explicit invalid block leaves `spec` unchanged.
bool configureGuidance(const StoreCard& card, Spec& spec, std::string& error);
bool configureSeeker(const components::DescriptorV1& descriptor, Spec& spec, std::string& error);
bool configureGuidance(const components::DescriptorV1& descriptor, Spec& spec, std::string& error);
struct Observation {
    std::uint32_t id = 0;
    V3 position, velocity;
    bool decoy = false;
    double signal = 1.0; ///< Relative sensor strength after caller filtering; assumed discrimination model.
    /// Current observation's admitted sensor reach, after caller filtering.
    /// NaN preserves the model's base acquireM; other nonfinite or negative
    /// values invalidate the observation. Geometry is checked again here.
    double admittedRangeM = std::numeric_limits<double>::quiet_NaN();
};
struct Input {
    V3 position, velocity, up;
    double ageS = 0, dtS = 0; // Absolute launch age drives estimate extrapolation.
    bool supported = false, illuminated = false; // Support authorizes estimate updates.
    std::optional<Observation> estimate;
    std::uint32_t launchCue = 0; // One-time on-rail observation ID; only a current eligible observation can authorize it.
    std::vector<Observation> visible;
    /// Body-fixed seeker forward direction in ECEF, when the caller has it.
    /// Absent preserves the legacy velocity-axis approximation; explicitly
    /// supplied directions must be finite and nonzero.
    std::optional<V3> boresight;
};
struct Output {
    WeaponPhase phase = WeaponPhase::Unguided;
    bool active = false, tracking = false, supported = false;
    std::uint32_t holding = 0; // Last held observation survives Lost; tracking says if current.
    V3 acceleration;
    double timeToGoS = std::numeric_limits<double>::quiet_NaN();
    /// Zero after active-radar activation; NaN without a closing prediction or
    /// for seekers that have no active-radar transition.
    double timeToActiveS = std::numeric_limits<double>::quiet_NaN();
};
class Guidance {
public:
    explicit Guidance(Spec spec);
    /// A permitted prelaunch estimate, predicted to release time (launch age 0).
    /// This supplies an acquisition cue, not a current seeker observation.
    Guidance(Spec spec, std::optional<Observation> initialEstimate);
    Output update(const Input& input);
    Spec spec() const noexcept;
private:
    Spec spec_;
    bool active_ = false;
    bool initialized_ = false;
    std::uint32_t holding_ = 0;
    std::optional<Observation> estimate_;
    double estimateAgeS_ = 0;
    double lastAgeS_ = 0;
};
// Closest relative separation over one linear fixed-step sweep. fraction, if
// supplied, receives the normalized step position [0,1] of closest approach.
double sweptMissDistance(V3 ownBefore, V3 ownAfter, V3 targetBefore,
                         V3 targetAfter, double* fraction = nullptr) noexcept;
// Arming alone does not authorize detonation. The caller must also require
// an armed release (never jettison), current applicable target observations,
// proximity/impact and a live weapon. Terminal event/damage ownership is external.
bool armed(double ageS, const Spec& spec) noexcept;
}
