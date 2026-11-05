#pragma once

// Internal native contact-episode state, not an installed/public SDK value.
#include "simulation/mission_systems/damage/LiquidSceneV1.h"

namespace fsim::ms::damage {
struct MaterialFacetKeyV3 {
    std::uint64_t victim=0,generation=0;
    std::size_t panel=0,triangle=0;
    std::shared_ptr<const ContactGeometryV1> geometry;
    double thickness=0;
    Target boundaryTarget;
    bool freshBoundary=false,paid=true;
    bool operator==(const MaterialFacetKeyV3& other) const noexcept {
        return victim==other.victim&&generation==other.generation&&panel==other.panel&&triangle==other.triangle&&
            geometry==other.geometry&&thickness==other.thickness;
    }
};
struct MaterialContactHistoryV3 {
    struct Particle {std::vector<MaterialFacetKeyV3> active;};
    bool initialized=false;
    FragmentFieldV1 expected;
    std::vector<Particle> particles;
};
struct MaterialOwnerHandoffV3 {
    std::uint64_t from=0,fromGeneration=0,to=0,toGeneration=0;
    std::shared_ptr<const ContactGeometryV1> fromGeometry,toGeometry;
    std::vector<std::size_t> panels;
};
bool rebindMaterialContactOwnerV3(MaterialContactHistoryV3&,const MaterialOwnerHandoffV3&,const Target&);
// Both entrypoints stage all history and publish it ONLY with a valid result.
// Historical V1 crossedFaces remains an observed-face list, not active state.
SceneAssessmentV1 assessSceneMaterialContactsV3(const FragmentFieldV1&,const std::vector<SceneVictimV1>&,
    const SceneEnvironmentV1&,MaterialContactHistoryV3&);
SceneLiquidAssessmentV2 assessSceneLiquidsMaterialContactsV3(const FragmentFieldV1&,
    const std::vector<SceneVictimV1>&,const std::vector<SceneLiquidOnlyVictimV2>&,
    const SceneEnvironmentV1&,const SceneLiquidInputsV1&,MaterialContactHistoryV3&);
} // namespace fsim::ms::damage
