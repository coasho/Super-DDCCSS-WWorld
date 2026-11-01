#pragma once

#include "foundation/core/Json.h"
#include <array>
#include <filesystem>
#include <memory>
#include <limits>
#include <vector>

namespace fsim::session {

/// The common aircraft table representation, evaluated for a stowed store.
/// Released flight uses the same data through the common JSBSim XML writer.
class StoreAerodynamicsV1 {
public:
    static std::shared_ptr<const StoreAerodynamicsV1> load(const std::filesystem::path& path);
    std::array<double, 6> coefficients(double mach, double alpha, double beta,
                                      double speed, const double rates[3],
                                      double heightOverSpan = std::numeric_limits<double>::infinity()) const;
    double area = 0., span = 0., chord = 0.;
    std::array<double, 3> aeroPoint{};
private:
    std::vector<double> alpha_, beta_, mach_;
    std::array<std::vector<double>, 6> base_;
    std::array<std::array<std::vector<double>, 6>, 3> rates_;
    std::vector<double> inducedBase_;
    std::array<std::vector<double>,3> inducedRates_;
    std::vector<double> lift_, side_, pitch_, drag_, induced_;
    std::vector<double> height_, groundLift_, groundDrag_;
};

} // namespace fsim::session
