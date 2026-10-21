#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/weapon/Guidance.h"
#include "simulation/mission_systems/weapon/LaunchEligibility.h"
#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "simulation/mission_systems/weapon/FuzeAdapter.h"
#include "simulation/mission_systems/damage/WarheadAdapter.h"
#include "foundation/core/Json.h"
#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <sstream>

namespace {
using namespace fsim::ms;
namespace cmp=fsim::ms::components;
fsim::core::Json read(const std::filesystem::path& path) {
    std::ifstream in(path,std::ios::binary); REQUIRE(in.good());
    std::stringstream text; text<<in.rdbuf(); return fsim::core::Json::parse(text.str(),path.string());
}
}

TEST_CASE("Every maintained payload declares all component roles and resolves actual providers", "[payload-components]") {
    std::vector<std::string> errors;
    const std::filesystem::path library=FSIM_TEST_STORE_DIR;
    const auto cards=readStoreLibrary(library,errors);
    for (const auto& error:errors) INFO(error);
    REQUIRE(errors.empty()); REQUIRE(cards.size()==53);
    unsigned guided=0, inert=0;
    for (const auto& [id,card]:cards) {
        INFO(id); cmp::ProfileV1 profile; std::string error;
        REQUIRE(cmp::readProfileV1(library/(id+".json"),*card,profile,error));
        INFO(error); REQUIRE(profile.explicitProfile()); REQUIRE(profile.components.size()==9);
        weapon::PlantAdapterV1 plant;
        REQUIRE(weapon::configurePlantV1(*profile.find("autopilot"),*profile.find("allocator"),*profile.find("actuator"),
            *profile.find("propulsion"),*profile.find("aerodynamics"),*card,plant,error));
        if (card->seeker.empty()) {++inert;continue;}
        ++guided; weapon::Spec spec; spec.seeker=card->seeker;
        REQUIRE(weapon::configureSeeker(*profile.find("seeker"),spec,error));
        REQUIRE(weapon::configureGuidance(*profile.find("guidance"),spec,error));
        const auto fuze=weapon::makeFuzeAdapter(*profile.find("fuze"),spec,error);
        INFO(error); REQUIRE(fuze);
        const auto warhead=damage::makeWarheadAdapter(*card,*profile.find("warhead"),error);
        INFO(error); REQUIRE(warhead);
        const auto existing=damage::warheadOf(*card); REQUIRE(existing);
        REQUIRE(warhead->model().modelVersion==existing->modelVersion);
        REQUIRE(warhead->model().fragments==existing->fragments);
        REQUIRE(warhead->model().speedMs==existing->speedMs);
        REQUIRE(fuze->spec().fuzeModelVersion==(card->parameters.count("fuze_model_version")?1:0));
        REQUIRE(plant.control.version==(card->parameters.count("weapon_execution_version")?1:0));
        auto wrong=*profile.find("seeker"); wrong.implementationId="unsupported.provider";
        auto unchanged=spec; REQUIRE_FALSE(weapon::configureSeeker(wrong,spec,error));
        REQUIRE(spec.acquireM==unchanged.acquireM); REQUIRE(spec.seeker==unchanged.seeker);
    }
    REQUIRE(guided==24); REQUIRE(inert==29);
}

TEST_CASE("Configured release field of view consumes the selected seeker rather than a card fallback", "[payload-components]") {
    StoreCard c; c.seeker="infrared";
    fsim::sim::VehicleState own{},target{}; own.rotationBodyToEcef[0]=own.rotationBodyToEcef[4]=own.rotationBodyToEcef[8]=1;
    target.positionEcef[0]=100;target.positionEcef[1]=50;
    weapon::SeekerConfigV1 config; config.version=1; config.mode=weapon::SeekerModeV1::Infrared;
    config.launchFovRad=.1;
    REQUIRE(weapon::checkLaunchEligibility(c,own,&target,config).reason==Reason::OutOfReach);
    config.launchFovRad=.6;
    REQUIRE(weapon::checkLaunchEligibility(c,own,&target,config).reason==Reason::None);
    config.mode=weapon::SeekerModeV1::SemiActiveRadar;
    REQUIRE(weapon::checkLaunchEligibility(c,own,&target,config).illuminationRequired);
    config.version=2;
    REQUIRE(weapon::checkLaunchEligibility(c,own,&target,config).reason==Reason::InvalidParameter);
}

TEST_CASE("The store reader rejects malformed explicit composition before the store enters its library", "[payload-components]") {
    const auto source=std::filesystem::path(FSIM_TEST_STORE_DIR)/"aim-120c.json";
    auto doc=read(source); auto block=doc.child("component_models");block.set("schema_version",2);doc.set("component_models",block);
    const auto directory=std::filesystem::temp_directory_path()/"fsim-payload-profile-refusal";
    std::filesystem::create_directories(directory);const auto path=directory/"aim-120c.json";
    {std::ofstream out(path,std::ios::binary);out<<doc.dump();}
    std::vector<std::string> errors; REQUIRE_FALSE(readStoreCard(path,errors)); REQUIRE_FALSE(errors.empty());
    std::filesystem::remove(path);std::filesystem::remove(directory);
}
