#include "simulation/mission_systems/weapon/NavigationConfigurationV2.h"
#include <catch2/catch_test_macros.hpp>
#include <limits>
using namespace fsim::ms;
namespace {
StoreCard actual() {
    std::vector<std::string> errors;
    const auto card=readStoreCard(std::filesystem::path(FSIM_TEST_STORE_DIR)/"aim-120d.json",errors);
    REQUIRE(errors.empty());REQUIRE(card);return *card;
}
}
TEST_CASE("Actual guided roster explicitly admits body-IMU navigation without inventing GNSS fits", "[weapon-navigation-policy]") {
    std::vector<std::string> errors;const auto cards=readStoreLibrary(FSIM_TEST_STORE_DIR,errors);REQUIRE(errors.empty());
    unsigned fitted=0,selected=0;
    for(const auto& item:cards) {
        std::optional<weapon::NavigationStrapdownConfigV2> config;std::string error;
        INFO(item.first);REQUIRE(weapon::navigationConfigurationV2(*item.second,config,error));
        if(config){++selected;if(config->gnssAiding){++fitted;CHECK(item.first=="aim-120d");}}
    }
    CHECK(selected==25);CHECK(fitted==1);
}
TEST_CASE("Malformed selected onboard IMU policy refuses atomically instead of using defaults", "[weapon-navigation-policy]") {
    auto card=actual();std::optional<weapon::NavigationStrapdownConfigV2> config;std::string error;
    REQUIRE(weapon::navigationConfigurationV2(card,config,error));REQUIRE(config);
    const auto key="own_nav_gyro_noise_density";const auto saved=card.parameters.at(key);
    card.parameters.erase(key);CHECK_FALSE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);
    card.parameters[key]=saved;card.parameters[key].unit="rad/s";
    CHECK_FALSE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);
    card.parameters[key]=saved;card.parameters[key].value=std::numeric_limits<double>::infinity();
    CHECK_FALSE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);
    card.parameters[key]=saved;card.parameters["own_nav_gyro_noise_typo"]=saved;
    CHECK_FALSE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);
    card.parameters.erase("own_nav_gyro_noise_typo");
    card.parameters[key]=saved;card.parameters["own_nav_maximum_integration_step_s"].value=.1;
    CHECK_FALSE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);
}
TEST_CASE("Unselected legacy store retains its old navigation policy", "[weapon-navigation-policy]") {
    StoreCard card;std::optional<weapon::NavigationStrapdownConfigV2> config=weapon::NavigationStrapdownConfigV2{};std::string error;
    REQUIRE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);CHECK(error.empty());
    card.parameters["own_navigation_version"].value=3;card.parameters["own_navigation_version"].unit="1";
    CHECK_FALSE(weapon::navigationConfigurationV2(card,config,error));CHECK_FALSE(config);
}
