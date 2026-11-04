#include "ZonedCaseWarheadV1.h"
#include <utility>

namespace fsim::ms::damage {

Impact ZonedCaseWarheadV1::assess(const Burst& burst,const AttachedGeometryView& geometry,const Target& target,Rng& rng) const {
    if(id_=="native.warhead.connected_ring.v1") {
        const auto field=emitScene(burst,rng);SceneEnvironmentV1 environment;environment.airDensity=target.airDensity;
        const auto scene=assessSceneV1(field,{{1,geometry,target}},environment);
        return scene.valid&&!scene.victims.empty()?scene.victims.front().impact:Impact{};
    }
    return damage::assess(model_,burst,geometry,target,rng);
}

} // namespace fsim::ms::damage
