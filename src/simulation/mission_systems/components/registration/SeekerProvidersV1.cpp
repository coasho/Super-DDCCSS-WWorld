#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/seeker/EncounterSeekerV1.h"
#include "simulation/mission_systems/components/implementations/seeker/NonImagingIrSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ImagingIrSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/SemiActiveRadarSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ActiveRadarSeekerV2.h"

namespace fsim::ms::components {

std::unique_ptr<SeekerProviderV1> nativeSeekerProviderV1(weapon::SeekerConfigV1 config) {return std::make_unique<EncounterSeekerV1>(config);}
void registerNativeSeekerV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.seeker.registerFactory("fsim.seeker.encounter.v1","fsim.seeker.encounter.v1",
        [](const DescriptorV1& d,const ProviderContextV1&,std::string& e) -> std::unique_ptr<SeekerProviderV1> {
            weapon::SeekerConfigV1 config;if(!weapon::configureSeeker(d,config,e))return {};
            return nativeSeekerProviderV1(config);
        },false,error);
    for(const auto* id:{"fsim.seeker.nonimaging_ir.v2","fsim.seeker.imaging_ir.v2",
                       "fsim.seeker.semi_active_radar.v2","fsim.seeker.active_radar.v2"}) {
        registry.seeker.registerFactory(id,id,
            [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e)->std::unique_ptr<SeekerProviderV1> {
                auto spec=ctx.spec;if(!weapon::configureSeeker(d,spec,e))return {};
                if(spec.seeker=="infrared")return std::make_unique<NonImagingIrSeekerV2>(spec);
                if(spec.seeker=="imaging_infrared")return std::make_unique<ImagingIrSeekerV2>(spec);
                if(spec.seeker=="semi_active_radar")return std::make_unique<SemiActiveRadarSeekerV2>(spec);
                if(spec.seeker=="active_radar")return std::make_unique<ActiveRadarSeekerV2>(spec,ctx.initialEstimate);
                e="unsupported measured seeker mode";return {};
            },false,error);
    }
}

} // namespace fsim::ms::components
