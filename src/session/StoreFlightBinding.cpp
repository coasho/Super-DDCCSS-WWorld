#include "session/StoreFlightBinding.h"

#include "sim/FlightModel.h"

#include <algorithm>
#include <cmath>

namespace fsim::session {

void StoreFlightBinding::bind(const ms::Host& host, sim::FlightModel& model, const std::vector<std::string>* names) {
    // its stations' point masses (MS-7): hangar's "station N", found by name among its definition's
    stationMass.clear(), massRevision = ~0ull;
    if (names)
        for (const auto& p : host.payloads()) {
            const ms::Card* card = host.card(p.index);
            if (p.family != ms::Family::StoreStation || !card) continue;
            for (const auto& st : card->stations) {
                const auto it = std::find((*names).begin(), (*names).end(), "station " + std::to_string(st.number));
                if (it == (*names).end()) continue;
                const std::size_t index = static_cast<std::size_t>(it - (*names).begin());
                PropertyHandle h = model.property(index ? "inertia/pointmass-weight-lbs[" + std::to_string(index) + "]"
                                                                       : std::string("inertia/pointmass-weight-lbs"));
                if (h.valid()) stationMass.push_back(StationMass{p.index, st.number, h});
            }
        }
}

void StoreFlightBinding::applyMasses(const ms::Host& host, sim::FlightModel& model) {
    constexpr double kLbPerKg = 2.2046226218487757;
    if (dragRevision != host.storeRevision()) { // (where its stores and carriage add drag: stepEntry pushes it)
        host.carriageDrag(drag);
        dragRevision = host.storeRevision();
        if (!drag.empty() && !dragQbar.valid()) dragQbar = model.property("aero/qbar-psf");
    }
    if (stationMass.empty() || massRevision == host.massRevision()) return;
    for (auto& m : stationMass) m.weightLbs.set(host.stationMassKg(m.payload, m.station) * kLbPerKg);
    massRevision = host.massRevision();
}

double StoreFlightBinding::weightRoomKg(double maximumKg, double weighedKg, double carriedKg, unsigned tick) {
    if (storesStamp != tick) storesKgAtStep = carriedKg, storesStamp = tick; // (the first load since that step)
    return maximumKg - (weighedKg - storesKgAtStep + carriedKg);
}

bool StoreFlightBinding::hasStations() const noexcept {
    return !stationMass.empty();
}

bool StoreFlightBinding::loadChanged() const noexcept {
    return loadRevision != massRevision;
}

void StoreFlightBinding::loadPublished() noexcept {
    loadRevision = massRevision;
}

void StoreFlightBinding::transferFuel(ms::Host& host, sim::FlightModel& model, double dt) {
    // an aircraft's external tanks topping its own as it burns (their fuel the first it uses): as much as its tanks
    // have room for, at kTransferKgS at most - the mass moving from its stations into its tanks, none made or lost
    constexpr double kTransferKgS = 10.0; // (an estimate: above a fighter's fuel flow at full reheat, some 7 kg/s)
    const sim::EnergyOnBoard energy = model.energy();
    const double room = energy.fuelCapacityKg - energy.fuelKg;
    if (!(room > 0.01)) return;
    const double drawn = host.drawExternalFuel(std::min(room, kTransferKgS * dt));
    const double put = model.refuel(drawn);
    (void)put;
}

bool StoreFlightBinding::addDrag(const sim::VehicleState& s, sim::FlightModel& model, double push[3], double turn[3]) const {
    if (!drag.empty() && dragQbar.valid()) {
        // its stores' and their carriage's drag (MS-7g): each against its airflow, at its station - its moment about the
        // CG (a store below it pitches the nose down; one wing's alone yaws it)
        constexpr double kPsfToPa = 47.880258888889;
        const double q0 = dragQbar.get() * kPsfToPa;
        const double q = q0 * ms::Host::storeDragMachFactor(s.mach);
        if (q > 0.0) {
            const double ca = std::cos(s.alphaRad), cb = std::cos(s.betaRad);
            const double wind[3] = {ca * cb, std::sin(s.betaRad), std::sin(s.alphaRad) * cb}; // (the air's way past it, body axes)
            double cg[3];
            model.cgFromOriginBodyM(cg);
            for (const auto& p : drag) {
                const double d = q * p.areaM2;
                const double F[3] = {-d * wind[0], -d * wind[1], -d * wind[2]};
                const double r[3] = {p.positionM[0] - cg[0], p.positionM[1] - cg[1], p.positionM[2] - cg[2]};
                for (int k = 0; k < 3; ++k) push[k] += F[k];
                turn[0] += r[1] * F[2] - r[2] * F[1];
                turn[1] += r[2] * F[0] - r[0] * F[2];
                turn[2] += r[0] * F[1] - r[1] * F[0];
                // in a sideslip: its stores' bodies turn the nose away from the wind (Munk's couple), their fins and
                // bases push across it, behind the station
                const double Y = -q0 * p.sideSlopeM2 * wind[1];
                push[1] += Y;
                turn[0] += -r[2] * Y;
                turn[2] += (r[0] - p.sideArmM) * Y - q0 * p.munkM3 * 2.0 * wind[1] * cb;
            }
            return true;
        }
    }
    return false;
}

} // namespace fsim::session
