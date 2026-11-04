#include "simulation/runtime/stores/StoreFlightBinding.h"

#include "simulation/flight/FlightModel.h"
#include "foundation/io/AssetResolver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsim::session {

void StoreFlightBinding::bind(const ms::Host& host, sim::FlightModel& model, const std::vector<std::string>* names) {
    // its stations' point masses (MS-7): hangar's "station N", found by name among its definition's
    stationMass.clear(), massRevision = ~0ull;
    airframes.clear(); drag.clear(); dragRevision=~0ull; dragQbar={}; dragDensity={};
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
        std::vector<ms::Host::CarriedAirframeV1> selected;
        host.carriedAirframesV1(selected);
        std::vector<Airframe> next;
        io::AssetResolver resolver;
        for(const auto& store:selected) {
            const auto& modelName=store.card->model;
            if(modelName.rfind("jsbsim:",0)!=0) throw std::runtime_error("Carried store requires an authored VLM model");
            const auto name=modelName.substr(7);
            if(name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")!=std::string::npos)
                throw std::runtime_error("Invalid carried store identity");
            const auto marker=store.card->parameters.find("airframe_aerodynamics_version");
            if(marker==store.card->parameters.end() || marker->second.value!=1. || marker->second.unit!="1")
                throw std::runtime_error("Missing shared aircraft VLM store binding");
            const auto provenance=store.card->sources.find(marker->second.source);
            if(provenance==store.card->sources.end()) throw std::runtime_error("Missing VLM table provenance");
            const auto& relative=provenance->second.url;
            const std::string prefix="aero/vlm-carriage-";
            if(relative.size()!=prefix.size()+64+5 || relative.rfind(prefix,0)!=0 ||
                relative.compare(relative.size()-5,5,".json")!=0 ||
                relative.substr(prefix.size(),64).find_first_not_of("0123456789abcdef")!=std::string::npos)
                throw std::runtime_error("Invalid content-addressed VLM table path");
            const auto directory=resolver.findAircraft(name,{});
            const auto path=directory/name/relative;
            if(directory.empty() || !std::filesystem::is_regular_file(path))
                throw std::runtime_error("Missing shared VLM table for "+name);
            next.push_back({StoreAerodynamicsV1::load(path),{store.positionM[0],store.positionM[1],store.positionM[2]}});
        }
        airframes=std::move(next);
        dragRevision = host.storeRevision();
        if ((!drag.empty() || !airframes.empty()) && !dragQbar.valid()) {
            dragQbar = model.property("aero/qbar-psf");
            dragDensity = model.property("atmosphere/rho-slugs_ft3");
        }
    }
    if (stationMass.empty() || massRevision == host.massRevision()) return;
    for (auto& m : stationMass) m.weightLbs.set(host.stationMassKg(m.payload, m.station) * kLbPerKg);
    massRevision = host.massRevision();
}

void StoreFlightBinding::invalidateMasses() noexcept {
    // Physical reset restores the plant's authored properties, while Host
    // retains its current inventory. The same revision must be written again.
    massRevision=~0ull;loadRevision=~0ull;storesStamp=~0u;
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
    bool applied=false;
    if(!airframes.empty() && dragDensity.valid()) {
        const double density=dragDensity.get()*515.3788183931961;
        double cg[3];model.cgFromOriginBodyM(cg);
        const auto& omega=s.angularRateBodyRadS;
        for(const auto& store:airframes) {
            const auto& t=*store.table;
            double arm[3];for(unsigned k=0;k<3;++k)arm[k]=store.position[k]+t.aeroPoint[k]-cg[k];
            const double velocity[]={s.airspeedTrueMs*std::cos(s.alphaRad)*std::cos(s.betaRad)+omega[1]*arm[2]-omega[2]*arm[1],
                s.airspeedTrueMs*std::sin(s.betaRad)+omega[2]*arm[0]-omega[0]*arm[2],
                s.airspeedTrueMs*std::sin(s.alphaRad)*std::cos(s.betaRad)+omega[0]*arm[1]-omega[1]*arm[0]};
            const double speed=std::sqrt(velocity[0]*velocity[0]+velocity[1]*velocity[1]+velocity[2]*velocity[2]);
            if(speed<1.e-9)continue;
            const double alpha=std::atan2(velocity[2],velocity[0]),beta=std::asin(std::clamp(velocity[1]/speed,-1.,1.));
            const double mach=s.airspeedTrueMs>1.e-9?s.mach*speed/s.airspeedTrueMs:0.;
            const auto c=t.coefficients(mach,alpha,beta,speed,omega,s.altitudeAglM/t.span);
            const double scale=.5*density*speed*speed*t.area;
            const double ca=std::cos(alpha),sa=std::sin(alpha),cb=std::cos(beta),sb=std::sin(beta);
            const double force[]={scale*(-c[0]*ca*cb-c[1]*ca*sb+c[2]*sa),
                scale*(-c[0]*sb+c[1]*cb),scale*(-c[0]*sa*cb-c[1]*sa*sb-c[2]*ca)};
            for(unsigned k=0;k<3;++k) {push[k]+=force[k];turn[k]+=scale*c[k+3]*(k==1?t.chord:t.span);}
            turn[0]+=arm[1]*force[2]-arm[2]*force[1];
            turn[1]+=arm[2]*force[0]-arm[0]*force[2];
            turn[2]+=arm[0]*force[1]-arm[1]*force[0];
            applied=true;
        }
    }
    if (!drag.empty() && dragQbar.valid()) {
        // its stores' and their carriage's drag (MS-7g): each against its airflow, at its station - its moment about the
        // CG (a store below it pitches the nose down; one wing's alone yaws it)
        constexpr double kPsfToPa = 47.880258888889;
        const double q0 = dragQbar.get() * kPsfToPa;
        const double q = q0;
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
            }
            return true;
        }
    }
    return applied;
}

} // namespace fsim::session
