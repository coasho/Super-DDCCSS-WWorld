#include "WarheadAdapter.h"
#include "simulation/mission_systems/components/registration/EffectFactoriesV1.h"

#include <stdexcept>
#include <utility>

namespace fsim::ms::damage {
namespace {


bool sameParameter(const Parameter& a, const Parameter& b) {
    return a.value==b.value && a.unit==b.unit && a.uncertainty==b.uncertainty &&
           a.basis==b.basis && a.source==b.source && a.note==b.note;
}

void declaration(const StoreCard& card, const components::DescriptorV1& d) {
    if (d.kind!="warhead" || d.availability!=components::AvailabilityV1::Available || d.configurationVersion!=1)
        throw std::runtime_error("warhead provider requires an available warhead configuration version 1");
    if (d.note.empty() || d.validity.empty() ||
        (d.basis!=Basis::Published && d.basis!=Basis::Derived && d.basis!=Basis::Assumed) ||
        (d.basis!=Basis::Assumed && d.source.empty()) || (!d.source.empty() && !card.sources.count(d.source)))
        throw std::runtime_error("warhead provider requires explicit valid provenance and validity");
    if (d.parameterRefs.size()!=d.parameters.size())
        throw std::runtime_error("warhead provider has inconsistent resolved parameter references");
    for (const auto& [role,p]:d.parameters) {
        const auto ref=d.parameterRefs.find(role);
        if (ref==d.parameterRefs.end() || ref->second.empty())
            throw std::runtime_error("warhead provider missing reference for role '"+role+"'");
        const auto resolved=card.parameters.find(ref->second), native=card.parameters.find(role);
        if (resolved==card.parameters.end() || native==card.parameters.end() ||
            !sameParameter(p,resolved->second) || !sameParameter(p,native->second))
            throw std::runtime_error("warhead provider role '"+role+"' differs from its native card snapshot");
    }
}

void binding(const components::DescriptorV1& d, const char* role, const char* expected) {
    const auto found=d.bindings.find(role);
    if (found==d.bindings.end() || found->second!=expected)
        throw std::runtime_error(std::string("warhead provider needs binding '")+role+"' = '"+expected+"'");
}

} // namespace

std::unique_ptr<WarheadAdapter> makeWarheadAdapter(
    const StoreCard& card, const components::DescriptorV1& d, std::string& error) {
    error.clear();
    try {
        declaration(card,d);
        const bool legacy=d.implementationId=="native.warhead.legacy.v1" || d.implementationId=="native.warhead.zoned_case.v1";
        const bool finiteZoned=d.implementationId=="native.warhead.finite_zoned_case.v2";
        if (!legacy && !finiteZoned && d.implementationId!="native.warhead.side_case.v1")
            throw std::runtime_error("unknown warhead implementation '"+d.implementationId+"'");
        const std::initializer_list<const char*> bindings=finiteZoned
            ? std::initializer_list<const char*>{"burst_reference","blast","penetration","assessment","angular_profile"}
            : std::initializer_list<const char*>{"burst_reference","blast","penetration","assessment"};
        if (legacy) {
            if (!components::slotsV1(d,{"warhead_kg","explosive_kg","gurney_kms","diameter_m"},bindings,error))
                return nullptr;
            if (card.parameters.count("warhead_model_version"))
                throw std::runtime_error("legacy warhead provider cannot execute a versioned source");
        } else if (!components::slotsV1(d,{"warhead_model_version","warhead_fragment_source","warhead_kg","explosive_kg",
                    "warhead_tnt_equivalent_kg","diameter_m","length_m","gurney_kms","warhead_structure_fraction",
                    "warhead_diameter_ratio","warhead_case_density_kg_m3","warhead_explosive_density_kg_m3",
                    "warhead_structure_density_kg_m3","warhead_fragment_cd","warhead_station_x_m",
                    "warhead_station_y_m","warhead_station_z_m"},bindings,error)) return nullptr;
        binding(d,"burst_reference",legacy?"captured_weapon_cg":"captured_authored_station_from_nose");
        binding(d,"blast","kingery_bulmash.swisdak_1994.free_air.v1");
        binding(d,"penetration","thor.aluminium_2024_t3.v1");
        binding(d,"assessment","native.fragment_blast.v1");
        if(finiteZoned)binding(d,"angular_profile","wt.aam.seven_zones.ea96f3af75af.v1");
        const auto number=[&](const char* role,const char* unit) {
            double value=0;
            if (!components::parameterV1(d,role,unit,value,error)) throw std::runtime_error(error);
            return value;
        };
        const double total=number("warhead_kg","kg"), charge=number("explosive_kg","kg");
        const double gurney=number("gurney_kms","km/s"), diameter=number("diameter_m","m");
        if (legacy) {
            if ((card.warheadKind!="fragmentation" && card.warheadKind!="rod") ||
                !(total>0) || charge<0 || charge>total || !(gurney>0) || !(diameter>0))
                throw std::runtime_error("legacy warhead configuration is outside its supported native domain");
        } else {
            const double expectedVersion=finiteZoned?2:1;
            if (number("warhead_model_version","1")!=expectedVersion || number("warhead_fragment_source","1")!=expectedVersion)
                throw std::runtime_error("finite warhead provider requires its exact native source/model version");
            number("warhead_tnt_equivalent_kg","kg"); number("length_m","m");
            number("warhead_structure_fraction","1"); number("warhead_diameter_ratio","1");
            number("warhead_case_density_kg_m3","kg/m^3"); number("warhead_explosive_density_kg_m3","kg/m^3");
            number("warhead_structure_density_kg_m3","kg/m^3"); number("warhead_fragment_cd","1");
            number("warhead_station_x_m","m"); number("warhead_station_y_m","m"); number("warhead_station_z_m","m");
        }
        auto model=warheadOf(card);
        if (!model || model->modelVersion!=(legacy?0u:finiteZoned?2u:1u))
            throw std::runtime_error("selected native warhead construction refused its card");
        return makeNativeWarheadAdapterV1(d.implementationId,std::move(*model));
    } catch (const std::exception& ex) {
        error=ex.what(); return nullptr;
    }
}

} // namespace fsim::ms::damage
