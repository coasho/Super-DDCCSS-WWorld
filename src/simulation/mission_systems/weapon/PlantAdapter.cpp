#include "PlantAdapter.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <iterator>
#include <set>
#include <stdexcept>
#include <utility>

namespace fsim::ms::weapon {
namespace {
using components::DescriptorV1;
using components::AvailabilityV1;
constexpr const char* kinds[]={"autopilot","allocator","actuator","propulsion","aerodynamics"};
constexpr const char* ids[]={"fsim.control.load_feedback_cascade","fsim.allocator.jsbsim_authored_fin_mixer",
    "fsim.actuator.jsbsim_lag_rate_stops","fsim.propulsion.jsbsim_generated_solid_rocket","fsim.aerodynamics.jsbsim_generated_store"};
double number(const DescriptorV1& d,const std::string& role,const char* unit) {
    double value;std::string error;
    if(!components::parameterV1(d,role,unit,value,error)) throw std::runtime_error(error);
    return value;
}
void slots(const DescriptorV1& d,std::initializer_list<const char*> parameters,std::initializer_list<const char*> bindings) {
    std::string error;if(!components::slotsV1(d,parameters,bindings,error)) throw std::runtime_error(error);
    if(d.parameters.size()!=parameters.size() || d.parameterRefs.size()!=parameters.size() || d.bindings.size()!=bindings.size())
        throw std::runtime_error(d.kind+" has missing parameter/binding slots");
    for(const auto* role:parameters) if(!d.parameters.count(role) || !d.parameterRefs.count(role))
        throw std::runtime_error(d.kind+" has missing parameter role");
    for(const auto* role:bindings) if(!d.bindings.count(role)) throw std::runtime_error(d.kind+" has missing binding");
}
bool path(const std::string& value) {
    if(value.empty() || value.front()=='/' || value.find('\\')!=std::string::npos || value.find(':')!=std::string::npos) return false;
    std::size_t first=0;
    do {
        const auto last=value.find('/',first);auto piece=value.substr(first,last-first);
        std::transform(piece.begin(),piece.end(),piece.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
        if(piece.empty() || piece=="." || piece==".." || piece==".codex") return false;
        if(last==std::string::npos) break;
        first=last+1;
    } while(true);
    return true;
}
bool same(double a,double b) noexcept {
    return std::isfinite(a)&&std::isfinite(b)&&a>0&&b>0&&
        std::abs(a-b)<=16*std::numeric_limits<double>::epsilon()*std::max(a,b);
}
}
std::uint32_t plantFingerprintV1(const components::DescriptorV1& d) noexcept {
    std::uint32_t hash=2166136261u;
    const auto byte=[&](unsigned char b){hash=(hash^b)*16777619u;};
    const auto integer=[&](std::uint32_t n){for(unsigned k=0;k<4;++k)byte(static_cast<unsigned char>(n>>(8*k)));};
    const auto text=[&](const std::string& s){integer(static_cast<std::uint32_t>(s.size()));for(char c:s)byte(static_cast<unsigned char>(c));};
    text(d.kind);text(d.implementationId);integer(d.configurationVersion);
    integer(static_cast<std::uint32_t>(d.parameters.size()));
    for(const auto& [role,p]:d.parameters) {
        text(role);text(p.unit);std::uint64_t bits=0;static_assert(sizeof bits==sizeof p.value);
        std::memcpy(&bits,&p.value,sizeof bits);for(unsigned k=0;k<8;++k)byte(static_cast<unsigned char>(bits>>(8*k)));
    }
    for(const auto* values:{&d.parameterRefs,&d.bindings}) {
        integer(static_cast<std::uint32_t>(values->size()));
        for(const auto& [role,value]:*values){text(role);text(value);}
    }
    return hash;
}
bool configurePlantV1(const DescriptorV1& autopilot,const DescriptorV1& allocator,const DescriptorV1& actuator,
    const DescriptorV1& propulsion,const DescriptorV1& aerodynamics,const StoreCard& card,PlantAdapterV1& output,std::string& error) {
    try {
        PlantAdapterV1 next;const DescriptorV1* descriptors[]={&autopilot,&allocator,&actuator,&propulsion,&aerodynamics};
        const bool executing=std::any_of(std::begin(descriptors),std::end(descriptors),
            [](const auto* d){return d->availability==AvailabilityV1::Available;});
        if(executing && (card.model.rfind("jsbsim:",0)!=0 || card.model.size()==7))
            throw std::runtime_error("executing plant needs a native JSBSim model identity");
        const auto model=card.model.size()>7 ? card.model.substr(7) : std::string{};
        const bool ducted=propulsion.implementationId=="fsim.propulsion.jsbsim_ducted_rocket.v1";
        const bool turbofan=propulsion.implementationId=="fsim.propulsion.jsbsim_turbofan.v1";
        const bool actuatorAware=autopilot.implementationId=="fsim.control.actuator_aware_cascade.v1";
        for(std::size_t i=0;i<5;++i) {
            const auto& d=*descriptors[i];
            if(d.kind!=kinds[i] || d.configurationVersion!=1) throw std::runtime_error("unsupported plant component kind/version");
            next.availability[i]=d.availability;
            if(d.availability!=AvailabilityV1::Available) {
                if(d.availability!=AvailabilityV1::Unavailable && d.availability!=AvailabilityV1::NotApplicable)
                    throw std::runtime_error(d.kind+" has unknown availability");
                if(d.reason.empty()) throw std::runtime_error(d.kind+" needs an availability reason");
                continue;
            }
            if(d.implementationId!=ids[i] && !(i==3&&(ducted||turbofan)) && !(i==0&&actuatorAware)) throw std::runtime_error(d.kind+" selects an unsupported compiled adapter");
            if(d.bindings.find("model")==d.bindings.end() || d.bindings.at("model")!=card.model)
                throw std::runtime_error(d.kind+" native model identity mismatch");
            for(const auto* role:{"design","motor","nozzle","table"}) if(d.bindings.count(role) && !path(d.bindings.at(role)))
                throw std::runtime_error(d.kind+" has an invalid asset binding");
            if(d.bindings.count("design") && d.bindings.at("design")!="aircraft/"+model+"/"+model+".toml")
                throw std::runtime_error(d.kind+" authored design identity mismatch");
            const unsigned code=i==3&&turbofan ? 3 : (i==3&&ducted)||(i==0&&actuatorAware)?2:1;
            next.implementations[i]=code;next.expected[i]={1,static_cast<double>(code),static_cast<double>(plantFingerprintV1(d))};
        }
        next.hasControl=autopilot.availability==AvailabilityV1::Available;
        if(next.hasControl!=(allocator.availability==AvailabilityV1::Available) || next.hasControl!=(actuator.availability==AvailabilityV1::Available))
            throw std::runtime_error("autopilot/allocator/actuator applicability is inconsistent");
        if(next.hasControl) {
            const auto mode=autopilot.bindings.find("execution_mode");
            if(mode==autopilot.bindings.end()) throw std::runtime_error("autopilot execution_mode missing");
            if(mode->second=="legacy") {
                if(actuatorAware)throw std::runtime_error("actuator-aware control requires configured native dynamics");
                slots(autopilot,{}, {"model","execution_mode"});
                if(card.parameters.count("weapon_execution_version")) throw std::runtime_error("explicit execution cannot silently select legacy control");
            } else if(mode->second=="configured") {
                const auto vector=autopilot.bindings.find("thrust_vector");
                if(vector!=autopilot.bindings.end()) {
                    if(vector->second!="jsbsim_nozzle_v1")throw std::runtime_error("unsupported thrust-vector control binding");
                    slots(autopilot,{"max_incidence_deg","rate_time_min_s","rate_lag_multiplier","incidence_time_min_s","incidence_rate_multiplier",
                        "tvc_limit_rad","tvc_rate_rad_s"},{"model","execution_mode","thrust_vector"});
                    next.control.thrustVector=true;
                    next.control.thrustVectorLimitRad=number(autopilot,"tvc_limit_rad","rad");
                    next.control.thrustVectorRateRadS=number(autopilot,"tvc_rate_rad_s","rad/s");
                    if(next.control.thrustVectorLimitRad<=0 || next.control.thrustVectorLimitRad>=1.5707963267948966 ||
                        next.control.thrustVectorRateRadS<=0)throw std::runtime_error("thrust-vector angle/rate outside finite actuator domain");
                } else slots(autopilot,{"max_incidence_deg","rate_time_min_s","rate_lag_multiplier","incidence_time_min_s","incidence_rate_multiplier"}, {"model","execution_mode"});
                next.control.version=1;
                next.control.maxIncidenceRad=number(autopilot,"max_incidence_deg","deg")*(3.14159265358979323846/180);
                next.control.rateTimeMinS=number(autopilot,"rate_time_min_s","s");
                next.control.rateLagMultiplier=number(autopilot,"rate_lag_multiplier","1");
                next.control.incidenceTimeMinS=number(autopilot,"incidence_time_min_s","s");
                next.control.incidenceRateMultiplier=number(autopilot,"incidence_rate_multiplier","1");
                if(next.control.maxIncidenceRad<=0 || next.control.maxIncidenceRad>=1.5707963267948966 ||
                    next.control.rateTimeMinS<=0 || next.control.rateLagMultiplier<=0 || next.control.incidenceTimeMinS<=0 || next.control.incidenceRateMultiplier<=0)
                    throw std::runtime_error("autopilot configuration outside supported model domain");
            } else throw std::runtime_error("unsupported autopilot execution mode");
            slots(allocator,{}, {"model","design"});slots(actuator,{"lag_s","rate_rad_s"},{"model"});
            next.control.actuatorLagS=number(actuator,"lag_s","s");next.control.actuatorRateRadS=number(actuator,"rate_rad_s","rad/s");
            if(next.control.actuatorLagS<=0 || !std::isfinite(1/next.control.actuatorLagS) || next.control.actuatorRateRadS<=0)
                throw std::runtime_error("actuator configuration outside native finite lag/rate domain");
        }
        if(propulsion.availability==AvailabilityV1::Available && turbofan) {
            slots(propulsion,{"thrust_dry_kn","bypass_ratio","tsfc_dry","fuel_capacity_kg","fuel_cg_x_m",
                "dry_mass_kg","cruise_speed_ms","governor_time_s"},{"model","design","engine","nozzle"});
            const std::pair<const char*,const char*> values[]={{"thrust_dry_kn","kN"},{"bypass_ratio","1"},
                {"tsfc_dry","lb/(lbf*h)"},{"fuel_capacity_kg","kg"},{"fuel_cg_x_m","m"},
                {"dry_mass_kg","kg"},{"cruise_speed_ms","m/s"},{"governor_time_s","s"}};
            for(const auto& [role,unit]:values)if(number(propulsion,role,unit)<=0)
                throw std::runtime_error("turbofan needs finite positive sizing, fuel and governor inputs");
            if(propulsion.bindings.at("engine")!="Engines/"+model+"_engine0.xml" ||
               propulsion.bindings.at("nozzle")!="Engines/"+model+"_nozzle0.xml")
                throw std::runtime_error("turbofan engine/nozzle asset binding mismatch");
        } else if(propulsion.availability==AvailabilityV1::Available) {
            const double n=number(propulsion,"phase_count","1");
            if(n<1 || n>1024 || n!=std::floor(n)) throw std::runtime_error("propulsion phase_count invalid");
            std::map<std::string,std::string> fixed={{"isp_s","s"},{"coast_thrust_n","N"},{"buildup_s","s"},
                {"grain_density_kg_m3","kg/m^3"},{"grain_radius_fraction","1"},{"grain_port_area_fraction","1"},
                {"grain_aft_clearance_m","m"},{"phase_count","1"},{"exit_x_m","m"},{"exit_y_m","m"},{"exit_z_m","m"},
                {"exit_diameter_m","m"},{"throat_diameter_m","m"},{"cone_length_m","m"}};
            if(ducted) {
                if(n!=2)throw std::runtime_error("ducted rocket requires booster and fuel-ledger phases");
                const std::map<std::string,std::string> cycle={{"capture_area_m2","m^2"},{"fuel_flow_max_kg_s","kg/s"},
                    {"heating_value_j_kg","J/kg"},{"combustion_efficiency","1"},{"pressure_recovery","1"},
                    {"air_cp_j_kg_k","J/(kg*K)"},{"gas_cp_j_kg_k","J/(kg*K)"},{"gas_gamma","1"},
                    {"fuel_air_ratio_max","1"},{"lightoff_mach","Mach"},{"target_mach","Mach"},
                    {"valve_minimum","1"},{"speed_gain","1"},{"valve_lag_s","s"},{"valve_rate_s_1","1/s"}};
                fixed.insert(cycle.begin(),cycle.end());
                const auto value=[&](const char* key){return number(propulsion,key,cycle.at(key).c_str());};
                if(value("combustion_efficiency")>1 || value("pressure_recovery")>1 ||
                    value("gas_gamma")<=1 || value("gas_gamma")>=2 || value("fuel_air_ratio_max")>=1 ||
                    value("valve_minimum")>1 || value("lightoff_mach")<=1 || value("target_mach")<=value("lightoff_mach"))
                    throw std::runtime_error("ducted rocket cycle/controller inputs outside supported domain");
            }
            std::set<std::string> roles;
            for(const auto& [role,unit]:fixed) {
                const double value=number(propulsion,role,unit.c_str());roles.insert(role);
                if(role.rfind("exit_",0)!=0 && role!="grain_aft_clearance_m" && value<=0)
                    throw std::runtime_error("propulsion positive input invalid");
            }
            for(unsigned i=0;i<static_cast<unsigned>(n);++i) {
                const auto first="phase_"+std::to_string(i)+"_duration_s", second="phase_"+std::to_string(i)+"_thrust_kn";
                if(number(propulsion,first,"s")<=0 || number(propulsion,second,"kN")<0) throw std::runtime_error("propulsion phase invalid");
                roles.insert(first);roles.insert(second);
            }
            if(propulsion.parameters.size()!=roles.size() || propulsion.parameterRefs.size()!=roles.size()) throw std::runtime_error("propulsion parameter slots invalid");
            for(const auto& [role,value]:propulsion.parameters) { (void)value;if(!roles.count(role) || !propulsion.parameterRefs.count(role)) throw std::runtime_error("propulsion unknown role"); }
            if(propulsion.bindings.size()!=5 || propulsion.bindings.count("design")!=1 || propulsion.bindings.count("smoke")!=1 ||
               propulsion.bindings.find("motor")==propulsion.bindings.end() || propulsion.bindings.at("motor")!="Engines/"+model+"_motor.xml" ||
               propulsion.bindings.find("nozzle")==propulsion.bindings.end() || propulsion.bindings.at("nozzle")!="Engines/"+model+"_nozzle.xml")
                throw std::runtime_error("propulsion native asset binding mismatch");
            const auto smoke=propulsion.bindings.at("smoke");
            if(smoke!="minimum" && smoke!="reduced" && smoke!="aluminised") throw std::runtime_error("propulsion smoke binding invalid");
            if(number(propulsion,"grain_radius_fraction","1")>1 || number(propulsion,"grain_port_area_fraction","1")>=1 ||
               number(propulsion,"grain_aft_clearance_m","m")<0 || number(propulsion,"exit_diameter_m","m")<=0 ||
               number(propulsion,"exit_y_m","m")!=0 || number(propulsion,"exit_z_m","m")!=0 ||
               number(propulsion,"throat_diameter_m","m")>=number(propulsion,"exit_diameter_m","m"))
                throw std::runtime_error("propulsion geometry/fractions invalid");
        }
        if(aerodynamics.availability==AvailabilityV1::Available) {
            const auto found=aerodynamics.bindings.find("source");
            if(found==aerodynamics.bindings.end()) throw std::runtime_error("aerodynamics source missing");
            slots(aerodynamics,{}, {"model","design","source"});
            if(found->second!="vlm") throw std::runtime_error("aircraft VLM aerodynamic source required");
        }
        next.configured=true;output=std::move(next);error.clear();return true;
    } catch(const std::exception& ex) {error=ex.what();return false;}
}
bool configureBallisticPlantV1(const DescriptorV1& aerodynamics,const StoreCard& card,PlantAdapterV1& output,std::string& error) {
    if(aerodynamics.availability!=AvailabilityV1::Available) {
        error="ballistic phase requires an available native aerodynamic provider";return false;
    }
    // These placeholders express unselected providers, never replacements for
    // the original separately owned ProfileV1 or its availability reasons.
    const auto omitted=[](const char* kind) {
        DescriptorV1 d;d.kind=kind;d.availability=AvailabilityV1::Unavailable;
        d.reason="Armed provider is not selected by the ballistic execution phase";return d;
    };
    PlantAdapterV1 next;
    if(!configurePlantV1(omitted("autopilot"),omitted("allocator"),omitted("actuator"),
                        omitted("propulsion"),aerodynamics,card,next,error)) return false;
    next.required.fill(false);next.required[4]=true;next.hasControl=false;
    output=std::move(next);error.clear();return true;
}
bool configureUnguidedPlantV1(const DescriptorV1& propulsion,const DescriptorV1& aerodynamics,
    const StoreCard& card,PlantAdapterV1& output,std::string& error) {
    if(aerodynamics.availability!=AvailabilityV1::Available ||
       (propulsion.availability!=AvailabilityV1::Available && propulsion.availability!=AvailabilityV1::NotApplicable)) {
        error="unguided non-inert phase requires available aerodynamics and available or explicitly unpowered propulsion";
        return false;
    }
    const auto omitted=[](const char* kind) {
        DescriptorV1 d;d.kind=kind;d.availability=AvailabilityV1::Unavailable;
        d.reason="Control provider is not selected by the unguided execution phase";return d;
    };
    PlantAdapterV1 next;
    if(!configurePlantV1(omitted("autopilot"),omitted("allocator"),omitted("actuator"),
                        propulsion,aerodynamics,card,next,error)) return false;
    next.required.fill(false);next.required[3]=next.required[4]=true;next.hasControl=false;
    output=std::move(next);error.clear();return true;
}
bool plantAvailableV1(const PlantAdapterV1& selected,const NativePlantIdentityV1& actual) noexcept {
    if(!selected.configured) return false;
    for(std::size_t i=0;i<5;++i) {
        if(!selected.required[i]) continue;
        if(selected.availability[i]==AvailabilityV1::Unavailable) return false;
        if(selected.availability[i]==AvailabilityV1::NotApplicable) continue;
        switch(selected.implementations[i]) {
        case 1: break; // the compiled JSBSim/readback adapters admitted above
        case 2: if(i!=3&&i!=0)return false;break; // finite-actuator control or ducted sustainer
        case 3: if(i!=3)return false;break; // native liquid-fuel turbofan
        default: return false;
        }
        const auto& a=actual.components[i];const auto& e=selected.expected[i];
        if(a.schemaVersion!=e.schemaVersion || a.implementationCode!=e.implementationCode || a.fingerprint!=e.fingerprint) return false;
    }
    return true;
}
bool plantCommandInputsAvailableV1(const PlantAdapterV1& selected,const AutopilotInput& input,const ControlStopsV1& stops,
    const NativePlantIdentityV1& actual) noexcept {
    if(!selected.hasControl || !plantAvailableV1(selected,actual) || stops.actuatorVersion!=1 ||
       !same(input.actuatorLagS,selected.control.actuatorLagS) || !same(stops.channelRateRadS,selected.control.actuatorRateRadS) ||
       stops.finRateRadS.empty() || !std::all_of(stops.finRateRadS.begin(),stops.finRateRadS.end(),
           [&](double rate){return same(rate,selected.control.actuatorRateRadS);})) return false;
    return true;
}
FinCommands plantCommandsV1(const PlantAdapterV1& selected,const AutopilotInput& input,const ControlStopsV1& stops,
    const NativePlantIdentityV1& actual) noexcept {
    if(!plantCommandInputsAvailableV1(selected,input,stops,actual))return {};
    switch(selected.implementations[0]) {
    case 1: return finCommands(input,selected.control,stops);
    case 2: return actuatorAwareCommands(input,selected.control,stops);
    default: return {};
    }
}
}
