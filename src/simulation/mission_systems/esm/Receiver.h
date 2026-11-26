#pragma once

// A warning receiver (ADR-30, MS-9a): the payload family `esm`. It listens. Every transmitter in the scene whose
// signal reaches its antennas above what it can detect is an emitter contact: a bearing with its error, the power
// it arrives with, and what the emitter is doing as far as its signal says - scanning past, locked on, lighting the
// carrier for a missile. It takes no task: it works while its payload operates.

#include "simulation/mission_systems/Model.h"

#include <cstddef>
#include <limits>

namespace fsim::ms::esm {

/// A single passive bearing measurement ray in ECEF used for moving-platform triangulation.
struct BearingSample {
    double originEcef[3] = {0.0, 0.0, 0.0};
    double rayEcef[3] = {1.0, 0.0, 0.0};
    double sigmaRad = 0.02; ///< 1-sigma angular DF uncertainty (rad)
};

/// Result of 3-D bearing-only triangulation (Stansfield/orthogonal-projection init +
/// damped Gauss-Newton angular Maximum Likelihood refinement + Fisher Information Matrix CRLB).
struct TriangulationResult {
    bool valid = false;
    double positionEcef[3] = {0.0, 0.0, 0.0};
    double crlbRmsM = std::numeric_limits<double>::quiet_NaN();   ///< 3-D 1-sigma CRLB RMS error sqrt(tr(F^-1)) (m)
    double crlbCep50M = std::numeric_limits<double>::quiet_NaN(); ///< Horizontal (ENU) elliptical CEP50 approximation (m)
};

/// Deterministic 3-D bearing-only triangulation aligned with O'Donoughue (2019) / ewgeo.triang:
/// weighted orthogonal-projection initialization, damped Gauss-Newton ML refinement on unit-sphere
/// angular residuals, and exact 3-D FIM/CRLB uncertainty estimation.
TriangulationResult triangulateBearings(const BearingSample* samples, std::size_t count,
                                        const double ownPos[3], const double latestRay[3]) noexcept;

/// The power (dBm) a receiver on a carrier at `own` takes from an emission, over a path that loses `pathLossDb`
/// one way, through an antenna of `antennaGainDbi`: a pulse's power where the emitter pulses, else its average.
double receivedDbm(const Emission& emission, const sim::VehicleState& own, double pathLossDb, double antennaGainDbi) noexcept;

/// A warning receiver from its card. What the card does not give is its class's, the same for every type
/// (docs/sdk/mission-systems.md, "The warning receiver").
std::unique_ptr<Model> makeReceiver(const Card& card);

} // namespace fsim::ms::esm
