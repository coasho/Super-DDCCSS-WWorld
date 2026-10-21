#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/seeker/EncounterSeekerV1.h"

namespace fsim::ms::components {

std::unique_ptr<SeekerProviderV1> nativeSeekerProviderV1(weapon::SeekerConfigV1 config) {return std::make_unique<EncounterSeekerV1>(config);}
void registerNativeSeekerV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.seeker.registerFactory("fsim.seeker.encounter.v1","fsim.seeker.encounter.v1",
        [](const DescriptorV1& d,const ProviderContextV1&,std::string& e) -> std::unique_ptr<SeekerProviderV1> {
            weapon::SeekerConfigV1 config;if(!weapon::configureSeeker(d,config,e))return {};
            return nativeSeekerProviderV1(config);
        },false,error);
}

} // namespace fsim::ms::components
