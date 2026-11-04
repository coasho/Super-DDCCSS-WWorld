#include "BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/implementations/guidance/VelocityNormalPnGuidanceV1.h"
#include "simulation/mission_systems/components/implementations/guidance/MeasuredPnGuidanceV2.h"
#include "simulation/mission_systems/weapon/NavigationConfigurationV2.h"
#include <algorithm>
#include <cmath>

namespace fsim::ms::components {
namespace {
class InertialWaypointV1 final : public GuidanceProviderV1 {
public:
    InertialWaypointV1(weapon::Spec spec,weapon::V3 point,double low,double high)
        :spec_(std::move(spec)),point_(point),minimum_(low),maximum_(high){}
    weapon::Spec spec() const noexcept override{return spec_;}
    weapon::Output update(const weapon::Input& in) override {
        using V=weapon::V3;
        const auto finite=[](V a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);};
        const auto dot=[](V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;};
        const auto norm=[](V a){return std::hypot(a.x,a.y,a.z);};
        weapon::Output out;
        if(!finite(in.position)||!finite(in.velocity)||!finite(in.up)||
           !std::isfinite(in.ageS)||in.ageS<0||in.ageS>=spec_.lifeS||
           !std::isfinite(in.dtS)||in.dtS<=0)return out;
        const V displacement{point_.x-in.position.x,point_.y-in.position.y,point_.z-in.position.z};
        if(!finite(displacement))return out;
        const double distance=norm(displacement),speed=norm(in.velocity),upLength=norm(in.up);
        if(!(distance>0 && speed>0 && upLength>0) || !std::isfinite(distance)||!std::isfinite(speed)||!std::isfinite(upLength))return out;
        const V desired{displacement.x/distance,displacement.y/distance,displacement.z/distance};
        const V forward{in.velocity.x/speed,in.velocity.y/speed,in.velocity.z/speed};
        const double time=std::clamp(distance/speed,minimum_,maximum_);
        const double along=dot(desired,forward);
        V turn{desired.x-along*forward.x,desired.y-along*forward.y,desired.z-along*forward.z};
        if(along<0 && norm(turn)<1e-12) {
            const V up{in.up.x/upLength,in.up.y/upLength,in.up.z/upLength};
            const double projection=dot(up,forward);
            turn={up.x-projection*forward.x,up.y-projection*forward.y,up.z-projection*forward.z};
            if(norm(turn)<1e-12) {
                const V axis=std::abs(forward.x)<std::abs(forward.y) ? V{1,0,0}:V{0,1,0};
                const double axisProjection=dot(axis,forward);
                turn={axis.x-axisProjection*forward.x,axis.y-axisProjection*forward.y,axis.z-axisProjection*forward.z};
            }
            const double scale=norm(turn);turn={turn.x/scale,turn.y/scale,turn.z/scale};
        }
        // Velocity-direction tracking: d(vhat)/dt=(dhat-(dhat.vhat)vhat)/T.
        // Gravity compensation is specific acceleration, never a force or a
        // client-side integrator. Every demand still passes native finite fins.
        V demand{speed/time*turn.x+9.80665*in.up.x/upLength,
                 speed/time*turn.y+9.80665*in.up.y/upLength,
                 speed/time*turn.z+9.80665*in.up.z/upLength};
        if(!finite(demand)||!std::isfinite(norm(demand)))return out;
        const double scale=std::min(1.,spec_.maxG*9.80665/std::max(norm(demand),1e-12));
        out.acceleration={scale*demand.x,scale*demand.y,scale*demand.z};
        out.phase=WeaponPhase::Midcourse;
        const double closing=dot(in.velocity,desired);
        if(closing>0)out.timeToGoS=distance/closing;
        // No seeker activation/tracking/holding, support update, or target truth.
        return out;
    }
private:
    weapon::Spec spec_;const weapon::V3 point_;const double minimum_,maximum_;
};
}

std::unique_ptr<GuidanceProviderV1> nativeGuidanceProviderV1(weapon::Spec spec,std::optional<weapon::Observation> estimate) {
    return std::make_unique<VelocityNormalPnGuidanceV1>(std::move(spec),std::move(estimate));
}
void registerNativeGuidanceV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.guidance.registerFactory("fsim.guidance.inertial_waypoint.v1","fsim.guidance.inertial_waypoint.v1",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e)->std::unique_ptr<GuidanceProviderV1> {
            std::optional<weapon::NavigationStrapdownConfigV2> navigation;
            if(!weapon::navigationConfigurationV2(ctx.card,navigation,e))return {};
            auto spec=ctx.spec;if(!weapon::configureGuidance(d,spec,e))return {};
            if(!ctx.initialEstimate || ctx.initialEstimate->decoy || !ctx.initialEstimate->id) {
                e="inertial waypoint release needs an authorized static aimpoint";return {};
            }
            const auto& aim=*ctx.initialEstimate;
            for(double value:{aim.position.x,aim.position.y,aim.position.z})
                if(!std::isfinite(value)){e="inertial aimpoint must be finite";return {};}
            // The authority-checked observed POSITION defines a static last-
            // known coordinate. Its track velocity is neither used nor replaced
            // by an invented observation, and no moving-target track is held.
            double low,high;
            if(!parameterV1(d,"response_min_s","s",low,e)||!parameterV1(d,"response_max_s","s",high,e))return {};
            e.clear();return std::make_unique<InertialWaypointV1>(std::move(spec),aim.position,low,high);
        },false,error);
    registry.guidance.registerFactory("fsim.guidance.velocity_normal_pn.v1","fsim.guidance.velocity_normal_pn.v1",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<GuidanceProviderV1> {
            std::optional<weapon::NavigationStrapdownConfigV2> navigation;
            if(!weapon::navigationConfigurationV2(ctx.card,navigation,e))return {};
            auto spec=ctx.spec;if(!weapon::configureGuidance(d,spec,e))return {};
            return nativeGuidanceProviderV1(std::move(spec),ctx.initialEstimate);
        },false,error);
    registry.guidance.registerFactory("fsim.guidance.measured_pn.v2","fsim.guidance.measured_pn.v2",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e)->std::unique_ptr<GuidanceProviderV1> {
            std::optional<weapon::NavigationStrapdownConfigV2> navigation;
            if(!weapon::navigationConfigurationV2(ctx.card,navigation,e))return {};
            auto spec=ctx.spec;if(!weapon::configureGuidance(d,spec,e))return {};
            return std::make_unique<MeasuredPnGuidanceV2>(std::move(spec));
        },false,error);
}

} // namespace fsim::ms::components
