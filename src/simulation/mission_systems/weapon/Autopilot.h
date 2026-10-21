#pragma once

#include <array>
#include <optional>
#include <limits>
#include <vector>

namespace fsim::ms::weapon {
// Private reduced-order engineering controller. Derivatives are signed BODY
// forces (N/rad) and CG moments (Nm/rad, Nm/(rad/s)), at the current native
// flight condition. They are not real-equipment identification or calibration.
struct ControlAxis {
    double forceAngle = 0, forceControl = 0;
    double momentAngle = 0, momentControl = 0, momentRate = 0;
};
// Same-frame native aerodynamic loads ABOUT THE CURRENT CG and measured actual
// fin-equivalent deflections. These are not demands or forces to be applied.
struct AeroFeedback {
    std::array<double,3> forceBodyN{}, momentBodyNm{}, controlRad{};
};
struct AutopilotInput {
    std::array<double, 3> demandBody{}, gravityBody{}, ratesRadS{};
    std::array<double, 3> inertiaKgM2{};
    double massKg = 0, speedMs = 0, alphaRad = 0, betaRad = 0;
    std::array<double, 3> limitsRad{}; // aileron, elevator, rudder
    double actuatorLagS = 0;
    ControlAxis pitch, yaw;
    double rollMomentPerRad = 0, rollDamping = 0;
    std::optional<AeroFeedback> measured;
};
struct FinCommands {
    double aileron = 0, elevator = 0, rudder = 0;
    bool available = false;
};
/// Versioned private execution settings. Version zero preserves legacy callers.
/// Algorithm/allocator 1 select the reduced-order cascade/authored fin mixer.
struct FinControlConfigV1 {
    unsigned version = 0, algorithm = 1, allocator = 1;
    double maxIncidenceRad = .2617993877991494;
    double rateTimeMinS = .12, rateLagMultiplier = 3;
    double incidenceTimeMinS = .4, incidenceRateMultiplier = 3.333333333333333;
    double actuatorLagS = .04, actuatorRateRadS = 4;
};
/// Optional native zero-centred normalized-command mapping, in axis order
/// aileron/elevator/rudder. Zero means genuinely absent legacy metadata.
struct ControlStopsV1 {
    unsigned version = 0;
    std::array<double,3> lowerRad{}, upperRad{};
    unsigned actuatorVersion = 0;
    double channelRateRadS = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> finRateRadS;
};
// Only normalized commands. There is no force/moment output or force fallback.
// Stateless feedback avoids integrator windup and cross-release/reset history.
FinCommands finCommands(const AutopilotInput& in) noexcept;
FinCommands finCommands(const AutopilotInput& in, const FinControlConfigV1& config,
                         const ControlStopsV1& stops = {}) noexcept;
}
