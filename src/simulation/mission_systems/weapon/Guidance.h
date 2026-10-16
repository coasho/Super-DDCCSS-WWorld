#pragma once

#include "fsim/MissionSystems.h"
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ms::weapon {
// All positions/velocities/commands use ECEF metres, metres/second and
// metres/second squared. up is the local outward unit direction. This core
// commands acceleration including gravity compensation; JSBSim remains the
// sole flight integrator. The caller owns expiry (lifeS), impact, damage and
// terminal events; Guidance must only be updated for a live armed release.
// basis: assumed. Acquisition range, navigation gain (3.5), 10-degree cue
// gate, loft and gravity (9.80665 m/s^2) are engineering/game estimates.
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
};
struct Observation {
    std::uint32_t id = 0;
    V3 position, velocity;
    bool decoy = false;
    double signal = 1.0; ///< Relative sensor strength after caller filtering; assumed discrimination model.
};
struct Input {
    V3 position, velocity, up;
    double ageS = 0, dtS = 0; // Absolute launch age drives estimate extrapolation.
    bool supported = false, illuminated = false; // Support authorizes estimate updates.
    std::optional<Observation> estimate;
    std::uint32_t launchCue = 0; // One-time on-rail observation ID; only a current eligible observation can authorize it.
    std::vector<Observation> visible;
};
struct Output {
    WeaponPhase phase = WeaponPhase::Unguided;
    bool active = false, tracking = false, supported = false;
    std::uint32_t holding = 0; // Last held observation survives Lost; tracking says if current.
    V3 acceleration;
    double timeToGoS = 0, timeToActiveS = 0; // Zero when unavailable/non-closing.
};
class Guidance {
public:
    explicit Guidance(Spec spec);
    Output update(const Input& input);
    Spec spec() const noexcept;
private:
    Spec spec_;
    bool active_ = false;
    bool initialized_ = false;
    std::uint32_t holding_ = 0;
    std::optional<Observation> estimate_;
    double estimateAgeS_ = 0;
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
