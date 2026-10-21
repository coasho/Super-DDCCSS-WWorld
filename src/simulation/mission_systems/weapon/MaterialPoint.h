#pragma once

#include "fsim/VehicleState.h"
#include "simulation/mission_systems/damage/Damage.h"

namespace fsim::ms::weapon {

struct MaterialPointSample {
    bool valid=false;
    damage::V3 position, velocity, axis, angularVelocity;
};

/// A fixed material point from the captured CG/state epoch, never a final-CG
/// substitution. Body axes: forward/right/down. Native rigid-body velocity at
/// the point is v_CG + omega cross(point-CG). No trajectory is integrated.
inline MaterialPointSample materialPoint(const sim::VehicleState& state, damage::V3 cgFromOrigin,
                                         damage::V3 pointFromOrigin) noexcept {
    const auto rotate=[&](damage::V3 v) {
        const auto* r=state.rotationBodyToEcef;
        return damage::V3{r[0]*v.x+r[1]*v.y+r[2]*v.z,r[3]*v.x+r[4]*v.y+r[5]*v.z,
                          r[6]*v.x+r[7]*v.y+r[8]*v.z};
    };
    const damage::V3 arm=pointFromOrigin-cgFromOrigin;
    MaterialPointSample out;
    out.position=damage::V3{state.positionEcef[0],state.positionEcef[1],state.positionEcef[2]}+rotate(arm);
    out.velocity=rotate(damage::V3{state.velocityBodyMs[0],state.velocityBodyMs[1],state.velocityBodyMs[2]}+
        damage::cross({state.angularRateBodyRadS[0],state.angularRateBodyRadS[1],state.angularRateBodyRadS[2]},arm));
    out.axis=rotate({1,0,0});
    out.angularVelocity=rotate({state.angularRateBodyRadS[0],state.angularRateBodyRadS[1],state.angularRateBodyRadS[2]});
    out.valid=true;
    for (const auto v : {out.position,out.velocity,out.axis,out.angularVelocity})
        out.valid=out.valid && std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    return out;
}

} // namespace fsim::ms::weapon
