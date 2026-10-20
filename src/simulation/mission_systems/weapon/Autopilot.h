#pragma once

#include <array>

namespace fsim::ms::weapon {
// Private reduced-order engineering controller. Derivatives are signed BODY
// forces (N/rad) and CG moments (Nm/rad, Nm/(rad/s)), at the current native
// flight condition. They are not real-equipment identification or calibration.
struct ControlAxis {
    double forceAngle = 0, forceControl = 0;
    double momentAngle = 0, momentControl = 0, momentRate = 0;
};
struct AutopilotInput {
    std::array<double, 3> demandBody{}, gravityBody{}, ratesRadS{};
    std::array<double, 3> inertiaKgM2{};
    double massKg = 0, speedMs = 0, alphaRad = 0, betaRad = 0;
    std::array<double, 3> limitsRad{}; // aileron, elevator, rudder
    double actuatorLagS = 0;
    ControlAxis pitch, yaw;
    double rollMomentPerRad = 0, rollDamping = 0;
};
struct FinCommands {
    double aileron = 0, elevator = 0, rudder = 0;
    bool available = false;
};
// Only normalized commands. There is no force/moment output or force fallback.
// Stateless feedback avoids integrator windup and cross-release/reset history.
FinCommands finCommands(const AutopilotInput& in) noexcept;
}
