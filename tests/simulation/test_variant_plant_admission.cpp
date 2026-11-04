#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "simulation/mission_systems/components/implementations/propulsion/JsbsimDuctedRocketPropulsionV1.h"
#include <catch2/catch_test_macros.hpp>
#include <tuple>
#include <limits>
using namespace fsim::ms;using namespace fsim::ms::weapon;
namespace {
void parameter(components::DescriptorV1& d,const char* role,double value,const char* unit){Parameter p;p.value=value;p.unit=unit;p.note="Independent synthetic admission fixture";p.uncertainty=.05;d.parameters[role]=p;d.parameterRefs[role]=role;}
}
TEST_CASE("Finite inlet V2 needs its complete residence and distinct native implementation", "[variant-actuation-native][ducted-inlet-admission]") {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    components::DescriptorV1 aero;aero.kind="aerodynamics";aero.implementationId="fsim.aerodynamics.jsbsim_generated_store";aero.availability=components::AvailabilityV1::Available;
    aero.bindings={{"model",card.model},{"design","aircraft/synthetic/synthetic.toml"},{"source","vlm"}};
    components::DescriptorV1 prop;prop.kind="propulsion";prop.implementationId="fsim.propulsion.jsbsim_ducted_rocket.v2";prop.availability=components::AvailabilityV1::Available;
    prop.bindings={{"model",card.model},{"design","aircraft/synthetic/synthetic.toml"},{"motor","Engines/synthetic_motor.xml"},{"nozzle","Engines/synthetic_nozzle.xml"},{"smoke","reduced"}};
    for(const auto& [role,unit,value]:{
        std::tuple{"isp_s","s",245.},{"coast_thrust_n","N",2.},{"buildup_s","s",.05},
        {"grain_density_kg_m3","kg/m^3",1750.},{"grain_radius_fraction","1",.9},
        {"grain_port_area_fraction","1",.15},{"grain_aft_clearance_m","m",.03},{"phase_count","1",2.},
        {"exit_x_m","m",1.},{"exit_y_m","m",0.},{"exit_z_m","m",0.},{"exit_diameter_m","m",.1},
        {"throat_diameter_m","m",.04},{"cone_length_m","m",.12},
        {"phase_0_duration_s","s",2.},{"phase_0_thrust_kn","kN",25.},
        {"phase_1_duration_s","s",25.},{"phase_1_thrust_kn","kN",3.},
        {"capture_area_m2","m^2",.0119},{"fuel_flow_max_kg_s","kg/s",.3},
        {"heating_value_j_kg","J/kg",40e6},{"combustion_efficiency","1",.9},{"pressure_recovery","1",.85},
        {"air_cp_j_kg_k","J/(kg*K)",1004.5},{"gas_cp_j_kg_k","J/(kg*K)",1150.},{"gas_gamma","1",1.33},
        {"fuel_air_ratio_max","1",.08},{"lightoff_mach","Mach",1.5},{"target_mach","Mach",3.5},
        {"valve_minimum","1",.15},{"speed_gain","1",.5},{"valve_lag_s","s",.1},{"valve_rate_s_1","1/s",10.}})
        parameter(prop,role,value,unit);
    parameter(prop,"inlet_residence_s",.1,"s");
    PlantAdapterV1 selected;std::string error;REQUIRE(configureUnguidedPlantV1(prop,aero,card,selected,error));
    CHECK(selected.implementations[3]==5);NativePlantIdentityV1 actual;actual.components[3]=selected.expected[3];actual.components[4]=selected.expected[4];
    REQUIRE(plantAvailableV1(selected,actual));actual.components[3].implementationCode=2;CHECK_FALSE(plantAvailableV1(selected,actual));
    const auto retained=selected;
    for(double value:{0.,-1.,1e308,std::numeric_limits<double>::denorm_min()}) {
        parameter(prop,"inlet_residence_s",value,"s");CHECK_FALSE(configureUnguidedPlantV1(prop,aero,card,selected,error));
        CHECK(selected.expected[3].fingerprint==retained.expected[3].fingerprint);
    }
    components::JsbsimDuctedRocketPropulsionV2 provider(card.model);CHECK(provider.acceptsEngineCount(2));CHECK_FALSE(provider.acceptsEngineCount(1));
    CHECK_FALSE(provider.burning(std::numeric_limits<double>::infinity()));
}
