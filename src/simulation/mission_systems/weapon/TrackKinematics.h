#pragma once

#include "fsim/VehicleState.h"

#include <cmath>

namespace fsim::ms::weapon {

/// Complete a measured track's synthetic state for existing geometry consumers.
/// NED velocity and geodetic position are measured/estimated inputs. The caller's
/// pitch and heading are already derived from that velocity, as in estimateOf.
/// The completed orientation is a derived zero-roll velocity frame: it supplies
/// no measured target attitude, roll, angle of attack or air-relative velocity.
/// Invalid or unrepresentable inputs leave the entire state unchanged.
inline bool completeTrackKinematics(sim::VehicleState& state) noexcept {
    constexpr double halfPi = 1.5707963267948966;
    const double latitude = state.latitudeRad, longitude = state.longitudeRad;
    const double pitch = state.eulerRad[1], heading = state.eulerRad[2];
    if (!std::isfinite(latitude) || std::abs(latitude) > halfPi || !std::isfinite(longitude)
        || !std::isfinite(pitch) || !std::isfinite(heading)) return false;
    for (double value : state.velocityNedMs) if (!std::isfinite(value)) return false;
    if (!std::isfinite(std::hypot(state.velocityNedMs[0], state.velocityNedMs[1], state.velocityNedMs[2])))
        return false;

    const double slat = std::sin(latitude), clat = std::cos(latitude);
    const double slon = std::sin(longitude), clon = std::cos(longitude);
    const double sp = std::sin(pitch), cp = std::cos(pitch);
    const double sh = std::sin(heading), ch = std::cos(heading);
    // Columns are the local north/east/down axes in ECEF and the synthetic
    // forward/right/down axes in NED respectively. Neither assumes flat Earth.
    const double nedToEcef[9] = {-slat * clon, -slon, -clat * clon,
                                -slat * slon,  clon, -clat * slon,
                                 clat,        0,    -slat};
    const double bodyToNed[9] = {cp * ch, -sh, sp * ch,
                                cp * sh,  ch, sp * sh,
                                -sp,       0, cp};
    double rotation[9]{}, bodyVelocity[3]{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column)
            for (int k = 0; k < 3; ++k)
                rotation[3 * row + column] += nedToEcef[3 * row + k] * bodyToNed[3 * k + column];
        for (int k = 0; k < 3; ++k)
            bodyVelocity[row] += bodyToNed[3 * k + row] * state.velocityNedMs[k];
        if (!std::isfinite(bodyVelocity[row])) return false;
    }
    // Verify the same arithmetic existing body-state consumers will perform.
    for (int row = 0; row < 3; ++row)
        if (!std::isfinite(rotation[3 * row] * bodyVelocity[0]
                          + rotation[3 * row + 1] * bodyVelocity[1]
                          + rotation[3 * row + 2] * bodyVelocity[2])) return false;

    // Compose Rz(longitude) Ry(-latitude-pi/2) Rz(heading) Ry(pitch).
    // This quaternion convention matches VehicleState/JSBSim's ECEF->body
    // quaternion and its redundant body->ECEF matrix.
    const double cLongitude = std::cos(longitude / 2), sLongitude = std::sin(longitude / 2);
    const double cLatitude = std::cos((-latitude - halfPi) / 2), sLatitude = std::sin((-latitude - halfPi) / 2);
    const double cHeading = std::cos(heading / 2), sHeading = std::sin(heading / 2);
    const double cPitch = std::cos(pitch / 2), sPitch = std::sin(pitch / 2);
    const double a[4] = {cLongitude * cLatitude, -sLongitude * sLatitude,
                         cLongitude * sLatitude, sLongitude * cLatitude};
    const double b[4] = {cHeading * cPitch, -sHeading * sPitch,
                         cHeading * sPitch, sHeading * cPitch};
    double quaternion[4] = {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    const double norm = std::hypot(std::hypot(quaternion[0], quaternion[1]),
                                   std::hypot(quaternion[2], quaternion[3]));
    if (!(norm > 0) || !std::isfinite(norm)) return false;
    for (int k = 0; k < 4; ++k) state.attitudeEcefToBody[k] = quaternion[k] / norm;
    for (int k = 0; k < 9; ++k) state.rotationBodyToEcef[k] = rotation[k];
    for (int k = 0; k < 3; ++k) state.velocityBodyMs[k] = bodyVelocity[k];
    state.eulerRad[0] = 0;
    return true;
}

} // namespace fsim::ms::weapon
