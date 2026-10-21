#include "FiniteZonedCaseWarheadV2.h"

namespace fsim::ms::damage {

Impact FiniteZonedCaseWarheadV2::assess(const Burst& burst,const AttachedGeometryView& geometry,
    const Target& target,Rng& rng) const {
    return damage::assess(model_,burst,geometry,target,rng);
}

} // namespace fsim::ms::damage
