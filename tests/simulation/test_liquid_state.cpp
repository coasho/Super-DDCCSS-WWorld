#include "simulation/mission_systems/damage/State.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>

namespace {
using namespace fsim::ms::damage;
std::shared_ptr<Airframe> fuelFrame() {
    auto frame=std::make_shared<Airframe>();Airframe::Component tank;
    tank.kind="fuel";tank.tank=0;tank.fuelKg=100;frame->components.push_back(tank);return frame;
}
Impact energy(double value,int hits=0) {
    Impact result;Impact::ComponentHits row;row.component=0;row.energyJ=value;row.hits=hits;
    result.components.push_back(row);return result;
}
}

TEST_CASE("Late liquid drag energy ignites by the existing energy hazard without another physical hit", "[liquid-state]") {
    using namespace fsim::ms::damage;
    constexpr double deposited=16000;
    for(const double pressure:{0.,25331.25,101325.})for(std::uint64_t seed=0;seed<128;++seed) {
        fsim::Rng random(seed),reference(seed);VehicleDamage state(fuelFrame());
        const bool expected=reference.uniform() < -std::expm1(-deposited*std::sqrt(pressure/101325.)/kIgnitionJ);
        const auto changes=state.applyLiquidEnergyV1(energy(deposited),pressure,random);
        REQUIRE(state.tanks().count(0)==1);CHECK(state.tanks().at(0).burning==expected);
        CHECK(state.tanks().at(0).holeM2==0);CHECK(changes.size()==(expected?1u:0u));
        if(expected)CHECK(changes[0].kind==Change::Kind::Fire);
        CHECK(random.uniform()==reference.uniform()); // Exactly one energy opportunity.
    }
}

TEST_CASE("Liquid continuation preserves punctures and consumes neither initial-hit energy nor repeated ignition draws", "[liquid-state]") {
    using namespace fsim::ms::damage;
    auto frame=fuelFrame();VehicleDamage state(frame);fsim::Rng random(17),reference(17);
    auto puncture=energy(0,1);puncture.components[0].areaM2=1e-5;puncture.components[0].fastestMs=500;
    state.apply(puncture,101325,random);reference.uniform(); // Existing zero-energy physical-hit behavior.
    REQUIRE(state.tanks().count(0)==1);const auto hole=state.tanks().at(0).holeM2;REQUIRE(hole>0);
    CHECK(state.applyLiquidEnergyV1(energy(1e9,1),101325,random).empty());
    CHECK(random.uniform()==reference.uniform());
    const auto changes=state.applyLiquidEnergyV1(energy(1e9),101325,random);reference.uniform();
    REQUIRE(changes.size()==1);CHECK(state.tanks().at(0).holeM2==hole);
    CHECK(state.applyLiquidEnergyV1(energy(1e9),101325,random).empty());
    CHECK(state.tanks().at(0).holeM2==hole);CHECK(random.uniform()==reference.uniform());
    VehicleDamage retained(frame);fsim::Rng legacy(17);
    CHECK(retained.apply(energy(1e9),101325,legacy).empty());
    CHECK_FALSE(retained.tanks().at(0).burning);CHECK(retained.tanks().at(0).holeM2==0);
}
