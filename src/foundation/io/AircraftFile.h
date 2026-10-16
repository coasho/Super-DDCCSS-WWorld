#pragma once

#include <filesystem>
#include <optional>

namespace fsim::io {

/// What a JSBSim aircraft definition says of its size and weight: <metrics><wingspan>, <mass_balance><emptywt> and
/// the sum of its <pointmass> weights (crew, payload), in SI (JSBSim's defaults, ft and lbs, where no unit is given).
struct AircraftMetrics {
    double wingspanM = 0.0;
    double emptyMassKg = 0.0;
    double pointMassKg = 0.0;
};

/// Read from the definition's file; nullopt when it cannot be read or says neither.
std::optional<AircraftMetrics> readAircraftMetrics(const std::filesystem::path& xml);

} // namespace fsim::io
