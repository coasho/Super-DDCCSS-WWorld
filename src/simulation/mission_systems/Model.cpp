#include "simulation/mission_systems/Model.h"

#include "simulation/mission_systems/comm/CommTerminal.h"
#include "simulation/mission_systems/ea/Dircm.h"
#include "simulation/mission_systems/ea/Jammer.h"
#include "simulation/mission_systems/esm/Receiver.h"
#include "simulation/mission_systems/ground_map/GroundMapRadar.h"
#include "simulation/mission_systems/gun/Gun.h"
#include "simulation/mission_systems/optical/OpticalSensor.h"
#include "simulation/mission_systems/pnt/PntReceiver.h"
#include "simulation/mission_systems/sar/SarRadar.h"
#include "simulation/mission_systems/smti/SmtiRadar.h"
#include "simulation/mission_systems/weather/WeatherRadar.h"

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
    switch (card.family) {
    case Family::Esm: return esm::makeReceiver(card);
    case Family::ElectronicAttack:
        if (auto jammer = ea::makeJammer(card, count)) return jammer;
        return ea::makeDircm(card, count);
    case Family::PassiveOptical: return optical::makeOpticalSensor(card, count);
    case Family::WeatherRadar: return weather::makeWeatherRadar(card, count);
    case Family::Sar: return sar::makeSarRadar(card, count);
    case Family::Smti: return smti::makeSmtiRadar(card, count);
    case Family::Pnt: return pnt::makePntReceiver(card, count);
    case Family::CommTerminal: return comm_payload::makeCommTerminal(card, count);
    case Family::GroundMapRadar: return ground_map::makeGroundMapRadar(card, count);
    case Family::Gun: return gun::makeGun(card, count);
    default: return nullptr; // (store stations, air radars handled separately)
    }
}

} // namespace fsim::ms
