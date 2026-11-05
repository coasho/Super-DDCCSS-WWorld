#include "AuthoredFieldWarheadV3.h"
#include <cmath>
namespace fsim::ms::damage {
namespace {
FragmentFieldV1 checkedRingEmission(const Warhead& model,const Burst& burst,Rng& rng) {
    auto temporary=rng;
    auto field=emitFragmentFieldV1(model,burst,temporary,true);if(!field.valid)return {};
    // The actual phase/captured pose affects rounded chord geometry. Validate
    // every produced material state before publishing or committing its RNG.
    for(const auto& p:field.projectiles) {
        if(!p.segment||!(p.massKg>0)||!std::isfinite(p.massKg)||
           !(p.expansionPerS>0)||!std::isfinite(p.expansionPerS)||
           !(p.breakupAgeS>0)||!std::isfinite(p.breakupAgeS)||
           !(p.materialLengthM>0)||!std::isfinite(p.materialLengthM))return {};
    }
    rng=temporary;return field;
}
Impact transactionalAssessment(const WarheadSceneExecutionV2& execution,const Burst& burst,
    const AttachedGeometryView& geometry,const Target& target,Rng& rng) {
    auto temporary=rng;
    const auto field=execution.emitScene(burst,temporary);if(!field.valid)return {};
    SceneEnvironmentV1 environment;environment.airDensity=target.airDensity;
    const auto result=assessSceneV1(field,{{1,geometry,target}},environment);
    if(!result.valid||result.victims.size()!=1)return {};
    Impact impact=result.victims.front().impact;
    rng=temporary;return impact;
}
}
FragmentFieldV1 AuthoredFieldWarheadV3::emitScene(const Burst& burst,Rng& rng) const {return emitFragmentFieldV1(model_,burst,rng,false);}
Impact AuthoredFieldWarheadV3::assess(const Burst& burst,const AttachedGeometryView& geometry,const Target& target,Rng& rng) const {
    return transactionalAssessment(*this,burst,geometry,target,rng);
}
FragmentFieldV1 AuthoredRingWarheadV4::emitScene(const Burst& burst,Rng& rng) const {return checkedRingEmission(model_,burst,rng);}
Impact AuthoredRingWarheadV4::assess(const Burst& burst,const AttachedGeometryView& geometry,const Target& target,Rng& rng) const {
    return transactionalAssessment(*this,burst,geometry,target,rng);
}
}
