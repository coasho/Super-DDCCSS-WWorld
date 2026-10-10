// NTIA's Irregular Terrain Model (Longley-Rice; third_party/ntia/itm, public
// domain; third_party/ntia/README.md): one translation unit, inside a
// namespace of its own, then the wrapper Ntia.h declares.
#include "ms/rf/Ntia.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <math.h>
#include <stdlib.h>
#include <vector>

namespace fsim_ntia_itm {
#include "itm/src/ComputeDeltaH.cpp"
#include "itm/src/DiffractionLoss.cpp"
#include "itm/src/FindHorizons.cpp"
#include "itm/src/FreeSpaceLoss.cpp"
#include "itm/src/FresnelIntegral.cpp"
#include "itm/src/H0Function.cpp"
#include "itm/src/InitializeArea.cpp"
#include "itm/src/InitializePointToPoint.cpp"
#include "itm/src/InverseComplementaryCumulativeDistributionFunction.cpp"
#include "itm/src/KnifeEdgeDiffraction.cpp"
#include "itm/src/LineOfSightLoss.cpp"
#include "itm/src/LinearLeastSquaresFit.cpp"
#include "itm/src/LongleyRice.cpp"
#include "itm/src/QuickPfl.cpp"
#include "itm/src/SigmaHFunction.cpp"
#include "itm/src/SmoothEarthDiffraction.cpp"
#include "itm/src/TerrainRoughness.cpp"
#include "itm/src/TroposcatterLoss.cpp"
#include "itm/src/ValidateInputs.cpp"
#include "itm/src/Variability.cpp"
#include "itm/src/itm_area.cpp"
#include "itm/src/itm_p2p.cpp"
} // namespace fsim_ntia_itm

namespace fsim::ms::ntia {

int itmP2p(double hTxM, double hRxM, const double* pfl, int climate, double n0, double fMhz, int pol, double epsilon,
           double conductivity, int mdvar, double timePercent, double locationPercent, double situationPercent, ItmResult& out) {
    fsim_ntia_itm::IntermediateValues iv{};
    double a = 0.0;
    long warnings = 0;
    const int rc = fsim_ntia_itm::ITM_P2P_TLS_Ex(hTxM, hRxM, pfl, climate, n0, fMhz, pol, epsilon, conductivity, mdvar, timePercent,
                                                 locationPercent, situationPercent, &a, &warnings, &iv);
    out.lossDb = a;
    out.warnings = warnings;
    for (int i = 0; i < 2; ++i) {
        out.thetaHorizonRad[i] = iv.theta_hzn[i];
        out.horizonDistanceM[i] = iv.d_hzn__meter[i];
        out.effectiveHeightM[i] = iv.h_e__meter[i];
    }
    out.deltaHM = iv.delta_h__meter;
    out.referenceDb = iv.A_ref__db;
    out.freeSpaceDb = iv.A_fs__db;
    out.dKm = iv.d__km;
    out.mode = iv.mode;
    return rc;
}

} // namespace fsim::ms::ntia
