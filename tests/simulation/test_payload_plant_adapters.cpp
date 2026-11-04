#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "simulation/mission_systems/components/implementations/propulsion/JsbsimDuctedRocketPropulsionV1.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>
using namespace fsim::ms;
using namespace fsim::ms::weapon;
using components::DescriptorV1;
using components::AvailabilityV1;
namespace {
void require(bool value,const char* reason) {if(!value) throw std::runtime_error(reason);}
DescriptorV1 descriptor(const char* kind,const char* implementation) {
    DescriptorV1 d; d.kind=kind;d.implementationId=implementation;d.availability=AvailabilityV1::Available;
    d.bindings["model"]="jsbsim:synthetic";return d;
}
void parameter(DescriptorV1& d,const char* role,const char* ref,double value,const char* unit) {
    Parameter p;p.value=value;p.unit=unit;p.note="Synthetic engineering fixture";d.parameters[role]=p;d.parameterRefs[role]=ref;
}
void adapterDispatchAndActualIdentityAreRequired() {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    auto a=descriptor("autopilot","fsim.control.load_feedback_cascade");a.bindings["execution_mode"]="legacy";
    auto m=descriptor("allocator","fsim.allocator.jsbsim_authored_fin_mixer");m.bindings["design"]="aircraft/synthetic/synthetic.toml";
    auto s=descriptor("actuator","fsim.actuator.jsbsim_lag_rate_stops");
    parameter(s,"lag_s","lag",.08,"s");parameter(s,"rate_rad_s","rate",3.25,"rad/s");
    require(plantFingerprintV1(s)==3348686125u,"Python and native profile identity encoding must agree exactly");
    auto p=descriptor("propulsion","");p.availability=AvailabilityV1::NotApplicable;p.reason="Unpowered synthetic plant";p.bindings.clear();
    auto d=descriptor("aerodynamics","fsim.aerodynamics.jsbsim_generated_store");
    d.bindings["design"]="aircraft/synthetic/synthetic.toml";d.bindings["source"]="vlm";
    PlantAdapterV1 adapter;std::string error;
    require(configurePlantV1(a,m,s,p,d,card,adapter,error),"Supported native backend must configure");
    NativePlantIdentityV1 actual;actual.components=adapter.expected;
    require(plantAvailableV1(adapter,actual),"Actual metadata must match the frozen selected implementation");
    AutopilotInput in;in.massKg=100;in.speedMs=500;in.actuatorLagS=.08;in.limitsRad.fill(.3);
    in.inertiaKgM2={1,100,100};in.pitch={-10000,-2000,-5000,-10000,-2000};
    in.yaw={-10000,2000,5000,-10000,-2000};in.rollMomentPerRad=1000;in.rollDamping=-100;
    in.ratesRadS[0]=.3;
    ControlStopsV1 stops;stops.actuatorVersion=1;stops.channelRateRadS=3.25;stops.finRateRadS={3.25,3.25};
    const auto expected=finCommands(in),command=plantCommandsV1(adapter,in,stops,actual);
    require(command.available && command.aileron!=0 && command.aileron==expected.aileron &&
            command.elevator==expected.elevator && command.rudder==expected.rudder,
            "Selected legacy backend must execute the identical real finite normalized control law");
    in.actuatorLagS=.04;
    require(!plantCommandsV1(adapter,in,stops,actual).available,"Legacy control cannot conceal a mismatched configured native plant");
    in.actuatorLagS=.08;
    actual.components[4].fingerprint+=1;
    require(!plantCommandsV1(adapter,in,stops,actual).available,"Different loaded aerodynamic asset must refuse control");
    auto retained=adapter;m.implementationId="unknown";
    require(!configurePlantV1(a,m,s,p,d,card,adapter,error) && !error.empty() && adapter.expected[0].fingerprint==retained.expected[0].fingerprint,
            "Unknown compiled adapter must fail atomically without selecting a default");
}
void ballisticPhaseRequiresOnlyGenuineNativeAerodynamics() {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    auto aero=descriptor("aerodynamics","fsim.aerodynamics.jsbsim_generated_store");
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="vlm";
    const auto original=aero;
    PlantAdapterV1 selected;std::string error;
    require(configureBallisticPlantV1(aero,card,selected,error),"Ballistic release must select its available native aerodynamic adapter");
    require(!selected.hasControl && selected.required[4] && !selected.required[0] && !selected.required[3],
            "Ballistic selection must explicitly disable armed control and propulsion dependencies");
    require(selected.availability[0]==AvailabilityV1::Unavailable && selected.availability[3]==AvailabilityV1::Unavailable &&
            aero.implementationId==original.implementationId && aero.availability==original.availability,
            "Selecting a phase cannot claim omitted armed providers exist or mutate original profile availability");
    NativePlantIdentityV1 actual;actual.components[4]=selected.expected[4];
    require(plantAvailableV1(selected,actual),"Actual ballistic aerodynamics must be sufficient even when armed metadata is absent");
    actual.components[4].fingerprint+=1;
    require(!plantAvailableV1(selected,actual),"Stale native aerodynamics must reject even an inert released body");
    const auto retained=selected;
    aero.implementationId="unknown";
    require(!configureBallisticPlantV1(aero,card,selected,error) && !error.empty() && selected.expected[4].fingerprint==retained.expected[4].fingerprint,
            "Unknown ballistic aerodynamic implementation must fail atomically without choosing legacy flight");
    aero=original;aero.bindings["design"]="aircraft/other/other.toml";
    require(!configureBallisticPlantV1(aero,card,selected,error),"Ballistic release cannot bypass authored native asset binding checks");
    aero=original;aero.availability=AvailabilityV1::Unavailable;aero.reason="Native aerodynamics not built";
    require(!configureBallisticPlantV1(aero,card,selected,error) && selected.expected[4].fingerprint==retained.expected[4].fingerprint,
            "An unavailable aerodynamic implementation cannot become an available ballistic body");
}
void unguidedNonInertPhaseStillRequiresNativePropulsion() {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    auto aero=descriptor("aerodynamics","fsim.aerodynamics.jsbsim_generated_store");
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="vlm";
    auto prop=descriptor("propulsion","fsim.propulsion.jsbsim_generated_solid_rocket");
    prop.bindings.insert({{"design","aircraft/synthetic/synthetic.toml"},{"motor","Engines/synthetic_motor.xml"},
                         {"nozzle","Engines/synthetic_nozzle.xml"},{"smoke","reduced"}});
    for(const auto& [role,unit,value]:{
            std::tuple{"isp_s","s",245.}, {"coast_thrust_n","N",2.}, {"buildup_s","s",.05},
            {"grain_density_kg_m3","kg/m^3",1750.}, {"grain_radius_fraction","1",.9},
            {"grain_port_area_fraction","1",.15}, {"grain_aft_clearance_m","m",.03},
            {"phase_count","1",1.}, {"exit_x_m","m",1.}, {"exit_y_m","m",0.}, {"exit_z_m","m",0.},
            {"exit_diameter_m","m",.1}, {"throat_diameter_m","m",.04}, {"cone_length_m","m",.12},
            {"phase_0_duration_s","s",1.}, {"phase_0_thrust_kn","kN",.001}})
        parameter(prop,role,role,value,unit);
    PlantAdapterV1 selected;std::string error;
    require(configureUnguidedPlantV1(prop,aero,card,selected,error),
            "A non-inert unguided release must select genuine native propulsion and aerodynamics");
    require(!selected.hasControl && !selected.required[0] && selected.required[3] && selected.required[4],
            "The unguided non-inert phase must retain both executing plant dependencies");
    NativePlantIdentityV1 actual;actual.components[3]=selected.expected[3];actual.components[4]=selected.expected[4];
    require(plantAvailableV1(selected,actual),"Matching native propulsion and aerodynamics must admit the declared plant");
    actual.components[3].fingerprint+=1;
    require(!plantAvailableV1(selected,actual),"Unguided release cannot bypass stale actual native propulsion");
    const auto retained=selected;
    prop.availability=AvailabilityV1::Unavailable;prop.reason="Native motor not built";
    require(!configureUnguidedPlantV1(prop,aero,card,selected,error) && !error.empty() &&
            selected.expected[3].fingerprint==retained.expected[3].fingerprint,
            "An unavailable declared propulsion provider must refuse atomically");
    prop.availability=AvailabilityV1::NotApplicable;prop.reason="Unpowered released body";
    prop.implementationId.clear();prop.parameters.clear();prop.parameterRefs.clear();prop.bindings.clear();
    require(configureUnguidedPlantV1(prop,aero,card,selected,error),"An unpowered body may explicitly declare propulsion not applicable");
    actual={};actual.components[4]=selected.expected[4];
    require(plantAvailableV1(selected,actual) && selected.required[3] && selected.availability[3]==AvailabilityV1::NotApplicable,
            "Unpowered release retains the explicit not-applicable status without inventing a motor");
}
void ductedRocketRequiresItsOwnCycleIdentityAndTwoEngines() {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    auto aero=descriptor("aerodynamics","fsim.aerodynamics.jsbsim_generated_store");
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="vlm";
    auto prop=descriptor("propulsion","fsim.propulsion.jsbsim_ducted_rocket.v1");
    prop.bindings.insert({{"design","aircraft/synthetic/synthetic.toml"},{"motor","Engines/synthetic_motor.xml"},
        {"nozzle","Engines/synthetic_nozzle.xml"},{"smoke","reduced"}});
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
        parameter(prop,role,role,value,unit);
    PlantAdapterV1 selected;std::string error;
    require(configureUnguidedPlantV1(prop,aero,card,selected,error),"Complete ducted cycle must configure independently of a solid rocket");
    require(selected.implementations[3]==2 && selected.expected[3].implementationCode==2,"Ducted cycle needs a distinct actual native identity");
    NativePlantIdentityV1 actual;actual.components[3]=selected.expected[3];actual.components[4]=selected.expected[4];
    require(plantAvailableV1(selected,actual),"Matching ducted native metadata must be admitted");
    actual.components[3].implementationCode=1;
    require(!plantAvailableV1(selected,actual),"A solid rocket cannot satisfy a selected ducted-cycle profile");
    components::JsbsimDuctedRocketPropulsionV1 provider(card.model);
    require(provider.acceptsEngineCount(2) && !provider.acceptsEngineCount(1) && !provider.acceptsEngineCount(3),
        "Ducted propulsion needs its distinct booster and sustainer engines");
    const auto retained=selected;prop.parameters.erase("capture_area_m2");
    require(!configureUnguidedPlantV1(prop,aero,card,selected,error) && selected.expected[3].fingerprint==retained.expected[3].fingerprint,
        "Missing atmosphere-capture input must refuse atomically without a solid fallback");
}
void operatingPointIdentityRequiresItsVersionedActualAutopilot() {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    auto autopilot=descriptor("autopilot","fsim.control.operating_point_bounded.v1");
    autopilot.bindings["execution_mode"]="configured";
    for(const auto& [role,unit,value]:{std::tuple{"max_incidence_deg","deg",15.},
        {"rate_time_min_s","s",.12},{"rate_lag_multiplier","1",3.},
        {"incidence_time_min_s","s",.4},{"incidence_rate_multiplier","1",3.333333333333333}})
        parameter(autopilot,role,role,value,unit);
    auto mixer=descriptor("allocator","fsim.allocator.jsbsim_authored_fin_mixer");
    mixer.bindings["design"]="aircraft/synthetic/synthetic.toml";
    auto actuator=descriptor("actuator","fsim.actuator.jsbsim_lag_rate_stops");
    parameter(actuator,"lag_s","lag",.04,"s");parameter(actuator,"rate_rad_s","rate",4.,"rad/s");
    auto propulsion=descriptor("propulsion","");propulsion.availability=AvailabilityV1::NotApplicable;
    propulsion.reason="Unpowered synthetic plant";propulsion.bindings.clear();
    auto aero=descriptor("aerodynamics","fsim.aerodynamics.jsbsim_generated_store");
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="vlm";
    PlantAdapterV1 selected;std::string error;
    require(configurePlantV1(autopilot,mixer,actuator,propulsion,aero,card,selected,error),"Explicit operating-point provider must configure");
    require(selected.implementations[0]==3 && selected.expected[0].implementationCode==3,"New autopilot requires distinct native code 3");
    NativePlantIdentityV1 actual;actual.components=selected.expected;
    require(plantAvailableV1(selected,actual),"Matching operating-point native metadata must be admitted");
    auto wrong=actual;wrong.components[0].implementationCode=1;
    require(!plantAvailableV1(selected,wrong),"A retained controller cannot satisfy new operating-point selection");
    wrong=actual;wrong.components[0].fingerprint+=1;
    require(!plantAvailableV1(selected,wrong),"New provider cannot accept a different actual control profile");
    // Code 3 also has its existing propulsion meaning; it must authorize no
    // arbitrary allocator, actuator or aerodynamic implementation.
    for(std::size_t slot:{1u,2u,4u}) {
        auto forged=selected;forged.implementations[slot]=3;forged.expected[slot]={1,3,123};
        wrong.components=forged.expected;
        require(!plantAvailableV1(forged,wrong),"Operating-point code cannot authorize another component kind");
    }
    const auto retained=selected;autopilot.bindings["execution_mode"]="legacy";
    require(!configurePlantV1(autopilot,mixer,actuator,propulsion,aero,card,selected,error) &&
        selected.expected[0].fingerprint==retained.expected[0].fingerprint,"New operating-point input cannot silently select legacy mode");
}
void commandedPulseIdentityIsAvailableOnlyForPropulsion() {
    StoreCard card;card.id="synthetic";card.model="jsbsim:synthetic";
    auto aero=descriptor("aerodynamics","fsim.aerodynamics.jsbsim_generated_store");
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="vlm";
    auto prop=descriptor("propulsion","fsim.propulsion.jsbsim_commanded_dual_pulse.v1");
    prop.bindings.insert({{"design","aircraft/synthetic/synthetic.toml"},{"motor","Engines/synthetic_motor.xml"},
        {"nozzle","Engines/synthetic_nozzle.xml"},{"smoke","reduced"}});
    for(const auto& [role,unit,value]:{
        std::tuple{"isp_s","s",245.},{"coast_thrust_n","N",2.},{"buildup_s","s",.05},
        {"grain_density_kg_m3","kg/m^3",1750.},{"grain_radius_fraction","1",.9},
        {"grain_port_area_fraction","1",.15},{"grain_aft_clearance_m","m",.03},{"phase_count","1",3.},
        {"exit_x_m","m",1.},{"exit_y_m","m",0.},{"exit_z_m","m",0.},{"exit_diameter_m","m",.1},
        {"throat_diameter_m","m",.04},{"cone_length_m","m",.12},
        {"phase_0_duration_s","s",4.},{"phase_0_thrust_kn","kN",25.},
        {"phase_1_duration_s","s",6.},{"phase_1_thrust_kn","kN",0.},
        {"phase_2_duration_s","s",4.},{"phase_2_thrust_kn","kN",12.},
        {"pulse_hold_limit_s","s",30.},{"pulse_reserve_speed_m_s","m/s",450.},{"pulse_terminal_lead_s","s",6.}})
        parameter(prop,role,role,value,unit);
    for(const auto* role:{"pulse_hold_limit_s","pulse_reserve_speed_m_s","pulse_terminal_lead_s"})
        prop.parameters.at(role).uncertainty=.5*prop.parameters.at(role).value;
    PlantAdapterV1 selected;std::string error;
    require(configureUnguidedPlantV1(prop,aero,card,selected,error),"Complete commanded pulse profile must configure");
    require(selected.implementations[3]==4 && selected.expected[3].implementationCode==4,
        "Commanded pulse must retain its distinct actual implementation identity");
    NativePlantIdentityV1 actual;actual.components=selected.expected;
    require(plantAvailableV1(selected,actual),"Matching native commanded pulse code 4 must admit the propulsion plant");
    auto stale=actual;stale.components[3].implementationCode=1;
    require(!plantAvailableV1(selected,stale),"A retained timed motor cannot satisfy commanded pulse selection");
    stale=actual;stale.components[3].fingerprint+=1;
    require(!plantAvailableV1(selected,stale),"A different actual commanded pulse profile must refuse");
    for(std::size_t slot=0;slot<5;++slot)if(slot!=3) {
        auto wrong=selected;auto wrongActual=actual;
        wrong.required[slot]=true;wrong.availability[slot]=AvailabilityV1::Available;wrong.implementations[slot]=4;
        wrong.expected[slot]={1,4,123};wrongActual.components[slot]=wrong.expected[slot];
        require(!plantAvailableV1(wrong,wrongActual),"Commanded pulse code 4 cannot authorize another component kind");
    }
    selected.implementations[3]=6;selected.expected[3].implementationCode=6;actual.components=selected.expected;
    require(!plantAvailableV1(selected,actual),"An unknown propulsion implementation cannot be admitted by matching metadata");
}
}
int main() {
    try {adapterDispatchAndActualIdentityAreRequired();ballisticPhaseRequiresOnlyGenuineNativeAerodynamics();
        unguidedNonInertPhaseStillRequiresNativePropulsion();ductedRocketRequiresItsOwnCycleIdentityAndTwoEngines();
        commandedPulseIdentityIsAvailableOnlyForPropulsion();operatingPointIdentityRequiresItsVersionedActualAutopilot();
        std::cout<<"6 payload plant adapter cases passed\n";}
    catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
