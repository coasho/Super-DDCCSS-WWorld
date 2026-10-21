#include "SideCaseWarheadV1.h"
#include <utility>

namespace fsim::ms::damage {

Impact SideCaseWarheadV1::assess(const Burst& burst,const AttachedGeometryView& geometry,const Target& target,Rng& rng) const {
    return damage::assess(model_,burst,geometry,target,rng);
}

} // namespace fsim::ms::damage
