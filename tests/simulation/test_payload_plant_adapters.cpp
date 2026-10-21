#include "simulation/mission_systems/weapon/PlantAdapter.h"
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
    d.bindings["design"]="aircraft/synthetic/synthetic.toml";d.bindings["source"]="component_build_up";
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
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="component_build_up";
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
    aero.bindings["design"]="aircraft/synthetic/synthetic.toml";aero.bindings["source"]="component_build_up";
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
}
int main() {
    try {adapterDispatchAndActualIdentityAreRequired();ballisticPhaseRequiresOnlyGenuineNativeAerodynamics();
        unguidedNonInertPhaseStillRequiresNativePropulsion();std::cout<<"3 payload plant adapter cases passed\n";}
    catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
