#include "simulation/mission_systems/Model.h"

#include "simulation/mission_systems/esm/Receiver.h"

namespace fsim::ms {

double Emission::gainTowardsDbi(const double placeEcef[3]) const noexcept {
    if (!state) return sidelobeDbi;
    double los[3], body[3], azimuth = 0.0, elevation = 0.0;
    if (!(geom::lineOfSight(*state, placeEcef, los) > 0.0)) return sidelobeDbi;
    geom::toBody(*state, los, body);
    geom::angles(body, azimuth, elevation);
    // in the volume its main beam goes through (every azimuth, for one that turns all round): its main beam's gain
    const bool inAzimuth = allRound || std::fabs(std::remainder(azimuth - azimuthCentreRad, 2.0 * geom::kPi)) <= azimuthHalfWidthRad;
    return inAzimuth && std::fabs(elevation - elevationCentreRad) <= elevationHalfWidthRad ? peakGainDbi : sidelobeDbi;
}

std::unique_ptr<Model> makeModel(const Card& card, std::uint16_t count) {
    (void)count;
    switch (card.family) {
    case Family::Esm: return esm::makeReceiver(card);
    default: return nullptr; // (the other families' models: MS-9 to MS-11, not built yet)
    }
}

} // namespace fsim::ms
