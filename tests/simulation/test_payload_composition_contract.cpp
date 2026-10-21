#include "simulation/mission_systems/weapon/Composition.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>

namespace {
using namespace fsim::ms;
namespace cmp=fsim::ms::components;
struct Fixture {
    StoreCard card;
    cmp::ProfileV1 profile;
    weapon::Spec baseline;
};
weapon::Spec baselineFor(const StoreCard& card) {
    weapon::Spec spec;spec.seeker=card.seeker;
    const bool heat=card.seeker.find("infrared")!=std::string::npos;
    spec.acquireM=card.value("seeker_acquire_m",heat?13000:card.seeker=="semi_active_radar"?60000:17000);
    spec.gimbalRad=card.value("seeker_gimbal_deg",heat?45:60)*3.14159265358979323846/180;
    spec.maxG=card.value("guidance_max_g",heat?40:35);spec.lifeS=card.value("guidance_life_s",heat?45:110);
    spec.loftRad=heat?0:20*3.14159265358979323846/180;
    spec.fuzeM=card.value("fuze_radius_m",7+std::sqrt(card.value("warhead_kg",12)));
    spec.capabilityVersion=card.guidanceCapabilities.version;
    spec.inertialMidcourse=card.guidanceCapabilities.inertialMidcourse;spec.launcherUpdates=card.guidanceCapabilities.launcherUpdates;
    std::string error;REQUIRE(weapon::configureGuidance(card,spec,error));
    return spec;
}
Fixture fixture(const char* name="aim-120c") {
    std::vector<std::string> errors;
    const auto file=std::filesystem::path(FSIM_TEST_STORE_DIR)/(std::string(name)+".json");
    const auto card=readStoreCard(file,errors);
    for(const auto& error:errors) INFO(error);
    REQUIRE(errors.empty());REQUIRE(card);
    Fixture f;f.card=*card;f.baseline=baselineFor(f.card);std::string error;
    REQUIRE(cmp::readProfileV1(file,f.card,f.profile,error));
    return f;
}
weapon::Spec sentinel() {
    weapon::Spec spec{"untouched"};spec.lifeS=432;spec.acquireM=987;spec.control.actuatorLagS=.321;
    spec.fuzeDelayS=.25;spec.fuzeStationStructuralM=weapon::V3{1,2,3};return spec;
}
void refusesUnchanged(const Fixture& f) {
    auto output=sentinel();std::string error;
    REQUIRE_FALSE(weapon::configureCompositionV1(f.card,f.profile,f.baseline,output,error));
    REQUIRE_FALSE(error.empty());REQUIRE(output.seeker=="untouched");REQUIRE(output.lifeS==432);
    REQUIRE(output.acquireM==987);REQUIRE(output.control.actuatorLagS==.321);
    REQUIRE(output.fuzeDelayS==.25);REQUIRE(output.fuzeStationStructuralM);
    REQUIRE(output.fuzeStationStructuralM->x==1);REQUIRE(output.fuzeStationStructuralM->z==3);
}
}

TEST_CASE("Composition preserves absent legacy execution without upgrading any component", "[payload-composition-contract]") {
    const auto baseline=sentinel();weapon::Spec output;cmp::ProfileV1 legacy;StoreCard card;std::string error;
    REQUIRE(weapon::configureCompositionV1(card,legacy,baseline,output,error));
    REQUIRE(error.empty());REQUIRE(output.seeker==baseline.seeker);REQUIRE(output.lifeS==baseline.lifeS);
    REQUIRE(output.seekerModel.version==0);REQUIRE(output.executionVersion==0);REQUIRE(output.control.version==0);
    REQUIRE(output.control.actuatorLagS==baseline.control.actuatorLagS);REQUIRE(output.fuzeDelayS==baseline.fuzeDelayS);
    REQUIRE(output.fuzeStationStructuralM->y==2);
}

TEST_CASE("Composition resolves every maintained guided card without changing its retained execution values", "[payload-composition-contract]") {
    std::vector<std::string> errors;const std::filesystem::path library=FSIM_TEST_STORE_DIR;
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());REQUIRE(cards.size()==53);
    unsigned guided=0;
    for(const auto& [id,card]:cards) if(!card->seeker.empty()) {
        ++guided;INFO(id);cmp::ProfileV1 profile;std::string error;
        REQUIRE(cmp::readProfileV1(library/(id+".json"),*card,profile,error));
        const auto baseline=baselineFor(*card);weapon::Spec selected;
        REQUIRE(weapon::configureCompositionV1(*card,profile,baseline,selected,error));
        INFO(error);REQUIRE(error.empty());REQUIRE(selected.seeker==baseline.seeker);
        REQUIRE(selected.seekerModel.version==1);REQUIRE(selected.executionVersion==1);
        REQUIRE(selected.acquireM==baseline.acquireM);REQUIRE(selected.gimbalRad==baseline.gimbalRad);
        REQUIRE(selected.maxG==baseline.maxG);REQUIRE(selected.lifeS==baseline.lifeS);REQUIRE(selected.loftRad==baseline.loftRad);
        REQUIRE(selected.navigationGain==baseline.navigationGain);REQUIRE(selected.cueRad==baseline.cueRad);
        REQUIRE(selected.activationRatio==baseline.activationRatio);REQUIRE(selected.loftAccelerationG==baseline.loftAccelerationG);
        REQUIRE(selected.capabilityVersion==baseline.capabilityVersion);
        REQUIRE(selected.inertialMidcourse==baseline.inertialMidcourse);REQUIRE(selected.launcherUpdates==baseline.launcherUpdates);
        REQUIRE(selected.control.version==1);
        REQUIRE(selected.control.algorithm==baseline.control.algorithm);
        REQUIRE(selected.control.allocator==baseline.control.allocator);
        REQUIRE(selected.control.maxIncidenceRad==baseline.control.maxIncidenceRad);
        REQUIRE(selected.control.rateTimeMinS==baseline.control.rateTimeMinS);
        REQUIRE(selected.control.rateLagMultiplier==baseline.control.rateLagMultiplier);
        REQUIRE(selected.control.incidenceTimeMinS==baseline.control.incidenceTimeMinS);
        REQUIRE(selected.control.incidenceRateMultiplier==baseline.control.incidenceRateMultiplier);
        REQUIRE(selected.control.actuatorLagS==baseline.control.actuatorLagS);
        REQUIRE(selected.control.actuatorRateRadS==baseline.control.actuatorRateRadS);
        REQUIRE(selected.fuzeModelVersion==baseline.fuzeModelVersion);
        REQUIRE(selected.armingS==baseline.armingS);REQUIRE(selected.fuzeM==baseline.fuzeM);REQUIRE(selected.fuzeDelayS==baseline.fuzeDelayS);
    }
    REQUIRE(guided==24);
}

TEST_CASE("Composition rejects disagreement between legacy seeker identity and selected mode atomically", "[payload-composition-contract]") {
    auto f=fixture();f.profile.components["seeker"].bindings["mode"]="infrared";refusesUnchanged(f);
    f=fixture();f.card.seeker.clear();refusesUnchanged(f);
}

TEST_CASE("Composition rejects shared guidance and fuze lifetime disagreement atomically", "[payload-composition-contract]") {
    auto f=fixture();f.profile.components["guidance"].parameters["life_s"].value+=1;refusesUnchanged(f);
    f=fixture();f.profile.components["fuze"].parameters["life_s"].value+=1;refusesUnchanged(f);
}

TEST_CASE("Composition rejects duplicate authored capability contradictions atomically", "[payload-composition-contract]") {
    auto f=fixture();f.card.guidanceCapabilities.launcherUpdates=false;refusesUnchanged(f);
    f=fixture();f.card.guidanceCapabilities.inertialMidcourse=false;refusesUnchanged(f);
    f=fixture();f.profile.components["guidance"].parameters["launcher_updates"].value=0;refusesUnchanged(f);
}

TEST_CASE("Composition validates selected V1 fuze dimensions against canonical card geometry atomically", "[payload-composition-contract]") {
    auto f=fixture();f.profile.components["fuze"].parameters["body_length_m"].value+=.1;refusesUnchanged(f);
    f=fixture();f.profile.components["fuze"].parameters["body_diameter_m"].value*=1.1;refusesUnchanged(f);
    f=fixture();f.card.parameters["length_m"].unit="ft";refusesUnchanged(f);
    f=fixture();f.card.parameters["diameter_m"].value=std::numeric_limits<double>::quiet_NaN();refusesUnchanged(f);
}

TEST_CASE("Composition requires a complete matching profile and armed control and effect providers", "[payload-composition-contract]") {
    auto f=fixture();f.profile.schemaVersion=2;refusesUnchanged(f);
    f=fixture();f.profile.storeId="another-store";refusesUnchanged(f);
    f=fixture();f.profile.components.erase("allocator");refusesUnchanged(f);
    f=fixture();f.profile.components["extra"]=f.profile.components["allocator"];refusesUnchanged(f);
    f=fixture();f.profile.components["guidance"].kind="seeker";refusesUnchanged(f);
    f=fixture();
    for(const char* role:{"autopilot","allocator","actuator"}) {
        auto& d=f.profile.components[role];d.availability=cmp::AvailabilityV1::NotApplicable;
        d.reason="Synthetic missing armed control";d.parameters.clear();d.parameterRefs.clear();d.bindings.clear();
    }
    refusesUnchanged(f);
    f=fixture();f.profile.components["fuze"].availability=cmp::AvailabilityV1::Unavailable;refusesUnchanged(f);
    f=fixture();f.profile.components["warhead"].availability=cmp::AvailabilityV1::Unavailable;refusesUnchanged(f);
}
