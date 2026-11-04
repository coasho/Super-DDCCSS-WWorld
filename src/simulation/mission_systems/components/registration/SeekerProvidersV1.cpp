#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/seeker/EncounterSeekerV1.h"
#include "simulation/mission_systems/components/implementations/seeker/NonImagingIrSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ImagingIrSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/SemiActiveRadarSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ActiveRadarSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV1.h"

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
                       "fsim.seeker.semi_active_radar.v2","fsim.seeker.active_radar.v2",
                       "fsim.seeker.nonimaging_ir.v3","fsim.seeker.imaging_ir.v3",
                       "fsim.seeker.semi_active_radar.v3","fsim.seeker.active_radar.v3"}) {
        registry.seeker.registerFactory(id,id,
            [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e)->std::unique_ptr<SeekerProviderV1> {
                auto spec=ctx.spec;if(!weapon::configureSeeker(d,spec,e))return {};
                if(!weapon::seekerVariantMatchesV2(d,ctx.card.id)) {
                    e="measured seeker variant does not belong to the provider's store";return {};
                }
                std::unique_ptr<MeasuredSeekerV2> sensor;
                if(spec.seeker=="active_radar")sensor=std::make_unique<ActiveRadarSeekerV2>(spec,ctx.initialEstimate);
                if(spec.seeker=="infrared")sensor=std::make_unique<NonImagingIrSeekerV2>(spec);
                if(spec.seeker=="imaging_infrared")sensor=std::make_unique<ImagingIrSeekerV2>(spec);
                if(spec.seeker=="semi_active_radar")sensor=std::make_unique<SemiActiveRadarSeekerV2>(spec);
                if(sensor) {
                    if(spec.seekerModel.engineering.version==3) {
                        auto finite=std::make_unique<EngineeringSeekerV3>(spec,std::move(sensor));
                        DescriptorV1 normalized;ReceiverConfigurationV3 receiver;
                        if(!receiverConfigurationV3(d,normalized,receiver,e))return {};
                        if((receiver.reticle||receiver.inherited.dual||receiver.inherited.single.kind!=PhysicalReceiverKindV1::Scalar)&&!finite->configureReceiverV3(receiver,e))return {};
                        // Physical receiver configuration resets tracking/navigation.
                        // Apply the authorized launch estimate after that boundary.
                        if(ctx.initialEstimate)finite->seedLaunchNavigationV1(*ctx.initialEstimate);
                        return finite;
                    }
                    if(ctx.initialEstimate)sensor->seedNavigationEstimate(*ctx.initialEstimate);
                    return sensor;
                }
                e="unsupported measured seeker mode";return {};
            },false,error);
    }
}

} // namespace fsim::ms::components
