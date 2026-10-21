#include "Composition.h"
#include "FuzeAdapter.h"
#include "PlantAdapter.h"
#include "simulation/mission_systems/damage/WarheadAdapter.h"
#include "simulation/mission_systems/components/Providers.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace fsim::ms::weapon {
bool configureCompositionV1(const StoreCard& card, const components::ProfileV1& profile,
                            const Spec& baseline, Spec& output, std::string& error) {
    return configureCompositionV1(card,profile,baseline,output,error,components::builtInProvidersV1());
}
bool configureCompositionV1(const StoreCard& card, const components::ProfileV1& selectedProfile,
                            const Spec& baseline, Spec& output, std::string& error,
                            const components::ProviderRegistryV1& providers) {
    error.clear();
    components::ProfileV1 profile;
    if(!providers.validationProfile(selectedProfile,profile,error))return false;
    try {
        Spec candidate=baseline;
        if(profile.schemaVersion==0) {output=std::move(candidate);return true;}
        if(profile.schemaVersion!=1 || card.id.empty() || profile.storeId!=card.id)
            throw std::runtime_error("unsupported composition version or mismatched store identity");
        constexpr const char* kinds[]={"seeker","guidance","autopilot","allocator","actuator",
                                      "fuze","warhead","propulsion","aerodynamics"};
        if(profile.components.size()!=9) throw std::runtime_error("explicit composition needs exactly nine component roles");
        for(const auto* kind:kinds) {
            const auto* d=profile.find(kind);
            if(!d || d->kind!=kind || d->configurationVersion!=1)
                throw std::runtime_error(std::string("composition has a missing or mismatched component role '")+kind+"'");
            if(d->availability!=components::AvailabilityV1::Available && d->availability!=components::AvailabilityV1::Unavailable &&
                    d->availability!=components::AvailabilityV1::NotApplicable)
                throw std::runtime_error(std::string("composition has unknown availability for '")+kind+"'");
        }
        // This helper composes an armed guided release. Inert/ballistic stores
        // retain their separate applicable plant checks in the World owner.
        if(card.storeClass!="aam" || card.seeker.empty())
            throw std::runtime_error("armed guided composition needs an authored AAM seeker identity");
        if(!configureSeeker(*profile.find("seeker"),candidate,error) ||
                !configureGuidance(*profile.find("guidance"),candidate,error)) return false;
        if(candidate.seeker!=card.seeker)
            throw std::runtime_error("selected seeker mode disagrees with the store's authored seeker identity");
        const auto& capabilities=card.guidanceCapabilities;
        if(capabilities.version>1 || (capabilities.version==1 &&
                (candidate.inertialMidcourse!=capabilities.inertialMidcourse || candidate.launcherUpdates!=capabilities.launcherUpdates)))
            throw std::runtime_error("selected guidance capabilities disagree with the store's authored capability declaration");

        PlantAdapterV1 plant;
        if(!configurePlantV1(*profile.find("autopilot"),*profile.find("allocator"),*profile.find("actuator"),
                *profile.find("propulsion"),*profile.find("aerodynamics"),card,plant,error)) return false;
        if(!plant.hasControl) throw std::runtime_error("armed guided composition needs its available finite control providers");
        for(const auto availability:plant.availability) if(availability==components::AvailabilityV1::Unavailable)
            throw std::runtime_error("armed guided composition has an unavailable native plant component");
        candidate.control=plant.control;

        const double guidanceLifeS=candidate.lifeS;
        const auto fuze=makeFuzeAdapter(*profile.find("fuze"),candidate,error);
        if(!fuze) return false;
        if(fuze->spec().lifeS!=guidanceLifeS)
            throw std::runtime_error("selected guidance and fuze lifetimes disagree");
        if(fuze->spec().fuzeModelVersion==1) {
            const auto canonical=[&](const char* name) {
                const auto found=card.parameters.find(name);
                if(found==card.parameters.end() || found->second.unit!="m" || !std::isfinite(found->second.value) || found->second.value<=0)
                    throw std::runtime_error(std::string("V1 fuze needs finite positive canonical card geometry in metres: ")+name);
                return found->second.value;
            };
            double bodyLength=0,bodyDiameter=0;
            if(!components::parameterV1(*profile.find("fuze"),"body_length_m","m",bodyLength,error) ||
                    !components::parameterV1(*profile.find("fuze"),"body_diameter_m","m",bodyDiameter,error)) return false;
            if(bodyLength!=canonical("length_m") || bodyDiameter!=canonical("diameter_m"))
                throw std::runtime_error("selected V1 fuze body dimensions disagree with canonical card geometry");
        }
        const auto warhead=damage::makeWarheadAdapter(card,*profile.find("warhead"),error);
        if(!warhead) return false;
        candidate=fuze->spec();
        output=std::move(candidate);error.clear();return true;
    } catch(const std::exception& ex) {error=ex.what();return false;}
}
} // namespace fsim::ms::weapon
