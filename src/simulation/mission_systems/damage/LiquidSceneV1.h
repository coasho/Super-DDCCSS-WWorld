#pragma once

// Additive private material-scene input. All capacity geometry and free-surface
// planes are in the recipient's captured structural body frame (metres).
// Capacity cells partition volume; none of their boundaries is a material wall.
#include "Damage.h"
#include "FluidTransitV1.h"

namespace fsim::ms::damage {

struct LiquidCapacityCellV1 {
    std::uint64_t stableCellId=0;
    int nativeTank=-1;
    std::size_t componentIndex=0;
    int section=-1;
    fluid::Aabb bodyBounds;
};
struct LiquidTankStateV1 {
    int nativeTank=-1;
    double densityKgM3=0;
    double dragCoefficient=0; // Explicit fluid coefficient, never the air Cd.
    double actualContentsKg=0;
    fluid::Plane freeSurfaceBody;
};
struct SceneLiquidRecipientV1 {
    std::uint64_t identity=0,generation=0;
    std::vector<LiquidCapacityCellV1> cells;
    std::vector<LiquidTankStateV1> tanks;
};
struct SceneLiquidControlsV1 {
    // Numerical budgets only; none is a density, thickness or lethality input.
    double maxStepS=.00025;
    double minStepS=1e-10;
    double absolutePositionToleranceM=1e-7;
    double absoluteVelocityToleranceMs=1e-5;
    double relativeTolerance=1e-6;
    double absoluteVolumeToleranceM3=1e-9;
    double relativeVolumeTolerance=1e-6;
    unsigned maxSteps=100000;
    fluid::QuadratureControls quadrature{1e-6,1e-6,16,4096};
};
struct SceneLiquidInputsV1 {
    std::vector<SceneLiquidRecipientV1> recipients;
    SceneLiquidControlsV1 controls;
};

// Transactional: malformed/stale/incomplete opted-in recipient material,
// overlapping capacity, inconsistent actual volume or exhausted integration
// budgets returns an invalid assessment, with no caller state/RNG mutation.
// Recipients not listed retain original solid-component semantics. For each
// listed recipient every attached declared fuel component's native tank must
// have a state. Multiple old probes may refer to one native tank. A zero-current
// orphan tank may have no local cells after its complete owner departs.
// Negligible bladder walls admit one per-tank hole only on an actual exposed
// capacity-union face contact, never initial volume occupancy or internal cell
// cuts. Actual liquid work/torque is a material encounter even without a hole
// or net linear impulse. Deposited energy is drag work, never incoming energy.
// Actual wet occupancy excludes ambient-air drag over the same point or rod
// material. Rod air loads integrate the exact dry material-interval complement;
// liquid comoving with material still excludes air despite zero liquid load.
SceneAssessmentV1 assessSceneLiquidsV1(const FragmentFieldV1&,
    const std::vector<SceneVictimV1>&,const SceneEnvironmentV1&,
    const SceneLiquidInputsV1&);

// Geometry-free material recipients. These have real capacity and native tank
// contents, but no authored skin, structural components, blast area or failure
// model. They share ONE traversal and residual field with structural victims.
// No existing victim, Target, assessment or provider layout is changed.
struct SceneLiquidOnlyVictimV2 {
    std::uint64_t identity=0,generation=0;
    Target target;
    std::vector<SceneVictimV1::MotionSample> motion;
};
struct SceneLiquidOnlyWorkV2 {
    std::uint64_t identity=0,generation=0;
    int nativeTank=-1;
    double energyJ=0;
    unsigned wallContacts=0;
};
struct SceneLiquidAssessmentV2 {
    SceneAssessmentV1 scene;
    std::vector<SceneLiquidOnlyWorkV2> liquidOnlyWork;
};
SceneLiquidAssessmentV2 assessSceneLiquidsV2(const FragmentFieldV1&,
    const std::vector<SceneVictimV1>&,const std::vector<SceneLiquidOnlyVictimV2>&,
    const SceneEnvironmentV1&,const SceneLiquidInputsV1&);

} // namespace fsim::ms::damage
