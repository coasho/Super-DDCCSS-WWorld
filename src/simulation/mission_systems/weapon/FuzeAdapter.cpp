#include "FuzeAdapter.h"
#include "adapters/NativeFuze.h"

#include <stdexcept>
#include <utility>

namespace fsim::ms::weapon {
namespace {


void declaration(const components::DescriptorV1& d) {
    if (d.kind!="fuze" || d.availability!=components::AvailabilityV1::Available || d.configurationVersion!=1)
        throw std::runtime_error("fuze provider requires an available fuze configuration version 1");
    if (d.note.empty() || d.validity.empty() ||
        (d.basis!=Basis::Published && d.basis!=Basis::Derived && d.basis!=Basis::Assumed) ||
        (d.basis!=Basis::Assumed && d.source.empty()))
        throw std::runtime_error("fuze provider requires explicit valid provenance and validity");
    if (d.parameterRefs.size()!=d.parameters.size())
        throw std::runtime_error("fuze provider has inconsistent resolved parameter references");
    for (const auto& [role,p]:d.parameters) {
        (void)p;
        const auto ref=d.parameterRefs.find(role);
        if (ref==d.parameterRefs.end() || ref->second.empty())
            throw std::runtime_error("fuze provider missing reference for role '"+role+"'");
    }
}

void binding(const components::DescriptorV1& d, const char* role, const char* expected) {
    const auto found=d.bindings.find(role);
    if (found==d.bindings.end() || found->second!=expected)
        throw std::runtime_error(std::string("fuze provider needs binding '")+role+"' = '"+expected+"'");
}

} // namespace

std::unique_ptr<FuzeAdapter> makeFuzeAdapter(
    const components::DescriptorV1& d, const Spec& spec, std::string& error) {
    error.clear();
    try {
        declaration(d);
        const bool legacy=d.implementationId=="native.fuze.legacy.v1" || d.implementationId=="native.fuze.swept_path.v1";
        if (!legacy && d.implementationId!="native.fuze.turning_cpa.v1")
            throw std::runtime_error("unknown fuze implementation '"+d.implementationId+"'");
        const auto bindings={"sensor_reference","target_reference","contact_reference","contact","history","delay"};
        if (legacy) {
            if (!components::slotsV1(d,{"model_version","algorithm","arming_s","life_s","radius_m"},bindings,error))
                return nullptr;
        } else if (!components::slotsV1(d,{"model_version","algorithm","arming_s","life_s","radius_m","delay_s",
                       "station_x_m","station_y_m","station_z_m","body_length_m","body_diameter_m"},bindings,error)) return nullptr;
        binding(d,"sensor_reference",legacy?"captured_weapon_cg":"captured_authored_station_from_nose");
        binding(d,"target_reference","captured_target_cg"); binding(d,"contact_reference","captured_authored_nose");
        binding(d,"contact","native.authored_point_contact.v1"); binding(d,"history","native.captured_piecewise_linear.v1");
        binding(d,"delay",legacy?"none":"native.captured_fixed_delay.v1");
        const auto number=[&](const char* role,const char* unit) {
            double value=0;
            if (!components::parameterV1(d,role,unit,value,error)) throw std::runtime_error(error);
            return value;
        };
        const double model=number("model_version","1"), algorithm=number("algorithm","1");
        Spec selected=spec;
        selected.armingS=number("arming_s","s"); selected.lifeS=number("life_s","s");
        selected.fuzeM=number("radius_m","m");
        if (model!=(legacy?0:1) || algorithm!=(legacy?0:1))
            throw std::runtime_error("fuze implementation differs from its explicit native model/algorithm selection");
        if (legacy) {
            if (selected.armingS<0 || !(selected.lifeS>selected.armingS) || selected.fuzeM<0)
                throw std::runtime_error("legacy fuze configuration is outside its native timing/radius domain");
            selected.fuzeModelVersion=0; selected.fuzeAlgorithm=0;
            selected.fuzeDelayS=0; selected.fuzeStationStructuralM.reset();
        } else {
            // Feed the selected descriptor into the existing native parser.
            // Its authored-body, timing and finite-domain checks remain the
            // single definition of V1 fuze configuration acceptance.
            StoreCard native;
            const auto put=[&](const char* role,const char* key,const char* unit) {
                number(role,unit); native.parameters.emplace(key,d.parameters.at(role));
            };
            put("model_version","fuze_model_version","1"); put("algorithm","fuze_proximity_algorithm","1");
            put("arming_s","fuze_arming_s","s"); put("radius_m","fuze_radius_m","m");
            put("delay_s","fuze_delay_s","s"); put("station_x_m","fuze_station_x_m","m");
            put("station_y_m","fuze_station_y_m","m"); put("station_z_m","fuze_station_z_m","m");
            put("body_length_m","length_m","m"); put("body_diameter_m","diameter_m","m");
            if (!configureGuidance(native,selected,error)) return nullptr;
        }
        return makeNativeFuzeAdapterV1(d.implementationId,std::move(selected));
    } catch (const std::exception& ex) {
        error=ex.what(); return nullptr;
    }
}

} // namespace fsim::ms::weapon
