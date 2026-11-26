#include "simulation/mission_systems/rcs/StoreRcs.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace fsim::ms::rcs {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kC = 299792458.0;
} // namespace

double externalStoreSigmaM2(const StoreCard& card,
                            const double directionBody[3],
                            double frequencyHz) noexcept {
    const double lengthM = std::max(card.value("length_m", 2.5), 0.25);
    const double diamM = std::max(card.value("diameter_m", 0.25), 0.05);
    const double radiusM = 0.5 * diamM;
    const double lambdaM = kC / std::max(frequencyHz, 100e6);

    const double dx = directionBody[0];
    const double dy = directionBody[1];
    const double dz = directionBody[2];
    const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(norm > 1e-12)) return 0.0;

    // Angle theta from the store's longitudinal x-axis (0 = nose-on, pi/2 = broadside).
    const double cosTheta = std::clamp(std::fabs(dx) / norm, 0.0, 1.0);
    const double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));

    // 1. Finite circular cylinder broadside physical-optics return (Knott, Radar Cross Section, Ch. 5):
    //    Broadside specular peak sigma_cyl = 2*pi*a*L^2 / lambda, averaged over a 3-degree aspect bin
    //    (~sinc^2 envelope with characteristic width lambda / (2*L)) to match binned rcs.bin tables.
    const double rawBroadside = (2.0 * kPi * radiusM * lengthM * lengthM) / lambdaM;
    const double binAveragingFactor = std::min(1.0, lambdaM / (0.105 * lengthM)); // ~3 deg bin averaging
    const double broadsideM2 = rawBroadside * binAveragingFactor;
    const double broadsideWeight = std::pow(sinTheta, 4.0);

    // 2. Nose/tail ogive & seeker-cavity / pylon-wing dihedral return (Ruck et al.):
    //    External stores mounted on pylons/rails form right-angle metal dihedrals with the
    //    pylon/wing surface plus non-LO seeker domes / intake scoops (RAM turbines on pods).
    const double crossAreaM2 = kPi * radiusM * radiusM;
    const std::string_view cls = card.storeClass;
    double noseFactor = 0.45; // baseline non-LO nose/pylon scatter
    double finFactor = 0.15;
    if (cls == "tank") {
        noseFactor = 0.85; // large blunt nose + pylon dihedral
        finFactor = 0.25;
    } else if (cls == "pod") {
        noseFactor = 1.15; // optics window / RAM-air turbine cavity + box pylon
        finFactor = 0.30;
    } else if (cls == "bomb") {
        noseFactor = 0.65; // bluff nose fuze + conical tail fins
        finFactor = 0.25;
    } else if (cls == "aam") {
        noseFactor = 0.55; // radome/seeker bulkhead cavity + 4 cruciform fins
        finFactor = 0.20;
    }

    const double noseM2 = noseFactor * crossAreaM2 * (0.35 + 0.65 * cosTheta * cosTheta);
    // Pylon/fin corner reflector contribution peaking at oblique aspects (~30-60 deg)
    const double obliqueWeight = 4.0 * sinTheta * sinTheta * cosTheta * cosTheta;
    const double pylonCornerM2 = finFactor * (lengthM * diamM) * (0.25 + 0.75 * obliqueWeight);

    const double sigma = broadsideM2 * broadsideWeight + noseM2 + pylonCornerM2;
    return std::clamp(sigma, 1e-4, 25.0);
}

} // namespace fsim::ms::rcs
