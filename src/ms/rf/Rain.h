#pragma once
// Rain's attenuation of a radio path: ITU-R P.838-3's specific attenuation
// (its equations (1)-(5) and the constants of its tables 1-4), along the part
// of the path below the rain height.

namespace fsim::ms::rf {

/// P.838-3's power-law coefficients at a frequency, path elevation and
/// polarisation tilt (0 horizontal, pi/2 vertical, pi/4 circular).
struct RainCoefficients {
    double k = 0.0;
    double alpha = 0.0;
};

/// k_H, alpha_H, k_V, alpha_V from equations (2) and (3), 1 .. 1000 GHz.
void rainCoefficientsHV(double fGhz, double& kH, double& alphaH, double& kV, double& alphaV) noexcept;
/// Equations (4) and (5): for a path at `elevationRad`, polarised at `tiltRad` from the horizontal.
RainCoefficients rainCoefficients(double fGhz, double elevationRad, double tiltRad) noexcept;
/// Equation (1): the specific attenuation (dB/km) in rain of `rateMmH`.
double rainSpecificDbPerKm(double fGhz, double rateMmH, double elevationRad, double tiltRad) noexcept;

} // namespace fsim::ms::rf
