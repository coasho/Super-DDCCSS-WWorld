#pragma once
#include "ThermalReticleReceiverV1.h"
#include <functional>

namespace fsim::ms::components::moving_radiator_v2 {
using V=weapon::V3;
using Shape=std::array<double,7>; ///< xFirst,yFirst,xLast,yLast,width,height,cutoff.
struct RayPath {
    V readDirection{},readDerivative{};
    bool stationary=false,domainBoundary=false;
    unsigned branch=0;
    double endpointRoundoffRad=0; ///< Certified endpoint reconstruction arithmetic radius.
};
/// Independent analytic seam: times are relative to the read epoch.
/// Only physically canonical, finite, renderer-admitted rates are returned.
std::optional<RayPath> endpointPath(V first,V last,double firstRelativeReadS,
    double lastRelativeReadS,double integrationTimeS,unsigned branch) noexcept;
V directionAt(const RayPath&,double relativeReadS) noexcept;
V derivativeAt(const RayPath&,double relativeReadS) noexcept;

/// Borrowed only for this synchronous pure inverse call. No state is retained.
struct Optics {
    const ThermalReticleConfigV1& policy;
    const std::vector<double>& weights;
    const std::vector<double>& timeWeights;
    const std::vector<double>& solidAngles;
    const std::vector<double>& flat;
    const std::vector<std::vector<double>>& nuisance;
    std::function<std::vector<double>(const std::array<double,4>&)> staticSourceRaw;
};
/// Pure numeric calibration of the production fixed-mode certificate. All
/// error radii and the native-score-derived allowed norm are caller bounds.
/// False means uncertain/not refused and requires the full native likelihood.
bool subsetCertificateArithmeticV2() noexcept;
bool certifiedSubsetRefusalV2(const std::vector<double>& target,
    const std::array<std::vector<double>,2>& columns,double targetError,
    const std::array<double,2>& columnErrors,double allowedResidualNorm);
void infer(const Optics&,ThermalReticleResultV1&,V pointing,V up,ThermalReticleMovingDiagnosticV2*);
std::optional<std::array<std::vector<double>,2>> calibrateBasis(const Optics&,
    const ThermalReticleResultV1&,V pointing,V up,const Shape&,unsigned branch);
/// Explicit V4 continuous numerical operator; retained calibration stays V2 exact.
std::optional<std::array<std::vector<double>,2>> calibrateBasisV4(const Optics&,
    const ThermalReticleResultV1&,V pointing,V up,const Shape&,unsigned branch);
/// Explicit version5 fast arithmetic; the V2/V4 calibration operators stay retained.
std::optional<std::array<std::vector<double>,2>> calibrateBasisV5(const Optics&,
    const ThermalReticleResultV1&,V pointing,V up,const Shape&,unsigned branch);
}
