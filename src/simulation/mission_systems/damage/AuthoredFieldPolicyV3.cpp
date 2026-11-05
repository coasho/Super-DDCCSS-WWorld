#include "AuthoredFieldPolicyV3.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fsim::ms::damage {
namespace {
constexpr double authoredPi=3.14159265358979323846;
std::map<std::string,std::string> commonRoles() {
    return {{"warhead_model_version","1"},{"warhead_kg","kg"},{"explosive_kg","kg"},
        {"warhead_case_kg","kg"},{"warhead_structure_kg","kg"},{"warhead_tnt_equivalent_kg","kg"},
        {"diameter_m","m"},{"length_m","m"},{"warhead_case_outer_radius_m","m"},
        {"warhead_case_inner_radius_m","m"},{"warhead_case_length_m","m"},
        {"warhead_fragment_count","1"},{"warhead_fragment_area_m2","m^2"},
        {"warhead_source_speed_ms","m/s"},{"warhead_fragment_cd","1"},
        {"warhead_station_x_m","m"},{"warhead_station_y_m","m"},{"warhead_station_z_m","m"}};
}
double required(const StoreCard& card,const std::string& name,const char* unit) {
    const auto found=card.parameters.find(name);
    if(found==card.parameters.end()||found->second.unit!=unit||!std::isfinite(found->second.value))
        throw std::runtime_error("authored source requires finite '"+name+"' in "+unit);
    return found->second.value;
}
std::optional<Warhead> read(const StoreCard& card,bool ring,std::string& error) {
    error.clear();
    try {
        if(card.warheadKind!="fragmentation"&&card.warheadKind!="rod")throw std::runtime_error("authored source requires explicit existing kind");
        if(ring&&card.warheadKind!="rod")throw std::runtime_error("authored ring requires rod kind");
        const double version=required(card,"warhead_model_version","1");
        if(version!=(ring?4.:3.))throw std::runtime_error("authored source requires its exact policy version");
        Warhead out;out.modelVersion=ring?4u:3u;out.kind=card.warheadKind=="rod"?Warhead::Kind::Rod:Warhead::Kind::Fragmentation;
        out.fragmentSource=Warhead::FragmentSource::FiniteZonedCaseV2;
        out.gurneyMs=0; // No Gurney driver is owned by these direct GAME source policies.
        out.totalKg=required(card,"warhead_kg","kg");out.explosiveKg=required(card,"explosive_kg","kg");
        out.caseKg=required(card,"warhead_case_kg","kg");const double structure=required(card,"warhead_structure_kg","kg");
        out.tntKg=required(card,"warhead_tnt_equivalent_kg","kg");
        const double diameter=required(card,"diameter_m","m"),modelLength=required(card,"length_m","m");
        const double outer=required(card,"warhead_case_outer_radius_m","m");
        out.caseInnerRadiusM=required(card,"warhead_case_inner_radius_m","m");
        out.lengthM=required(card,"warhead_case_length_m","m");
        const double count=required(card,"warhead_fragment_count","1");
        out.fragmentAreaM2=required(card,"warhead_fragment_area_m2","m^2");
        out.speedMs=required(card,"warhead_source_speed_ms","m/s");out.fragmentCd=required(card,"warhead_fragment_cd","1");
        const V3 station{required(card,"warhead_station_x_m","m"),required(card,"warhead_station_y_m","m"),required(card,"warhead_station_z_m","m")};
        if(!(out.totalKg>0)||out.explosiveKg<0||!(out.caseKg>0)||structure<0||out.tntKg<0||
           !(diameter>0)||!(modelLength>0)||!(outer>0)||out.caseInnerRadiusM<0||!(out.caseInnerRadiusM<outer)||
           !(out.lengthM>0)||!(out.fragmentAreaM2>0)||!(out.speedMs>0)||!(out.fragmentCd>0)||
           count!=std::floor(count)||count<(ring?3:1)||count>1000000)
            throw std::runtime_error("authored source outside declared numeric domain");
        const double owned=(out.explosiveKg+out.caseKg)+structure;
        const double scale=std::max(out.totalKg,owned);
        if(!std::isfinite(owned)||!std::isfinite(scale))throw std::runtime_error("authored source mass sum overflow");
        const double upward=std::nextafter(scale,std::numeric_limits<double>::infinity());
        const double ulp=std::isfinite(upward)?upward-scale:scale-std::nextafter(scale,0.);
        if(std::abs(owned-out.totalKg)>64*ulp)throw std::runtime_error("authored source mass roles do not close");
        const double half=ring?0.:out.lengthM/2;
        if(station.x-half<0||station.x+half>modelLength||std::hypot(station.y,station.z)+outer>diameter/2)
            throw std::runtime_error("authored source outside containing model envelope");
        out.diameterM=2*outer;out.caseThicknessM=outer-out.caseInnerRadiusM;
        out.fragments=static_cast<int>(count);out.fragmentKg=out.caseKg/count;out.stationStructuralM=station;
        const double energyRoot=(out.speedMs*std::sqrt(out.caseKg))*std::sqrt(.5);
        if(!std::isfinite(out.diameterM)||!(out.fragmentKg>0)||!std::isfinite(out.fragmentKg)||
           !std::isfinite(energyRoot)||energyRoot>std::sqrt(std::numeric_limits<double>::max()))
            throw std::runtime_error("authored source derived arithmetic overflow");
        if(ring) {
            const double maximumRadius=count*out.lengthM/(2*authoredPi);
            if(!std::isfinite(maximumRadius)||!(maximumRadius>outer))throw std::runtime_error("authored ring lacks finite unfolding material");
            const double expansion=out.speedMs/outer,breakup=(maximumRadius-outer)/out.speedMs;
            const double opening=2*authoredPi/count;
            const V3 first{outer,0,0},last{outer*std::cos(opening),outer*std::sin(opening),0};
            const double chord=length(last-first),growth=1+expansion*breakup,materialLength=chord*growth;
            if(!(expansion>0)||!std::isfinite(expansion)||!(breakup>0)||!std::isfinite(breakup)||
               !(chord>0)||!std::isfinite(chord)||!(growth>1)||!std::isfinite(growth)||
               !(materialLength>0)||!std::isfinite(materialLength))
                throw std::runtime_error("authored ring derived state is not representable");
        } else {
            const double zones=required(card,"warhead_zone_count","1");
            if(zones!=std::floor(zones)||zones<1||zones>32)throw std::runtime_error("authored source mixture count outside resource bound");
            const auto size=static_cast<unsigned>(zones);double maximum=0;
            for(unsigned i=0;i<size;++i) {
                const auto prefix="warhead_zone_"+std::to_string(i);
                Zone z{required(card,prefix+"_from_deg","deg"),required(card,prefix+"_to_deg","deg"),required(card,prefix+"_weight","1")};
                if(z.fromDeg<0||!(z.fromDeg<z.toDeg)||z.toDeg>180||z.share<0)throw std::runtime_error("authored source invalid solid-angle mixture");
                maximum=std::max(maximum,z.share);out.zones.push_back(z);
            }
            if(!(maximum>0))throw std::runtime_error("authored source empty mixture");
            // Relative probabilities have no absolute scale. Exact binary
            // exponent shifting avoids subnormal/overflowed CDF arithmetic;
            // normal legacy mixtures keep the same comparisons and RNG draws.
            const int exponent=std::ilogb(maximum);double total=0;
            for(auto& zone:out.zones) {
                if(zone.share==0)continue;
                zone.share=std::scalbn(zone.share,-exponent);
                if(!std::isnormal(zone.share))throw std::runtime_error("authored source positive mixture weight is not representable normally");
                const double next=total+zone.share;
                if(!(next>total)||!std::isfinite(next))throw std::runtime_error("authored source positive mixture CDF increment collapsed");
                total=next;
            }
            if(!(total>0)||!std::isnormal(total))throw std::runtime_error("authored source mixture total is not representable normally");
            for(const auto& zone:out.zones)if(zone.share>0&&!(total+zone.share>total))
                throw std::runtime_error("authored source positive mixture weight is below total CDF resolution");
            // Zero-weight entries have no measure. Put them before the stable
            // positive order so the unchanged sampler's defensive last-entry
            // fallback always belongs to a positive component.
            std::stable_partition(out.zones.begin(),out.zones.end(),[](const Zone& zone){return zone.share==0;});
        }
        const auto expected=authoredSourceRoleUnitsV3(out);
        for(const auto& [key,value]:card.parameters) {
            (void)value;
            if(key.rfind("warhead_",0)==0&&!expected.count(key))throw std::runtime_error("authored source unowned role '"+key+"'");
        }
        return out;
    } catch(const std::exception& ex) {error=ex.what();return std::nullopt;}
}
}
std::map<std::string,std::string> authoredSourceRoleUnitsV3(const Warhead& model) {
    auto roles=commonRoles();
    if(model.modelVersion==3) {
        roles.emplace("warhead_zone_count","1");
        for(std::size_t i=0;i<model.zones.size();++i) {
            const auto prefix="warhead_zone_"+std::to_string(i);
            roles.emplace(prefix+"_from_deg","deg");roles.emplace(prefix+"_to_deg","deg");roles.emplace(prefix+"_weight","1");
        }
    }
    return roles;
}
std::optional<Warhead> readAuthoredFieldPolicyV3(const StoreCard& card,std::string& error) {return read(card,false,error);}
std::optional<Warhead> readAuthoredRingPolicyV4(const StoreCard& card,std::string& error) {return read(card,true,error);}
}
