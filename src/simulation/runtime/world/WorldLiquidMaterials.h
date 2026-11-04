#pragma once

// Private producer-to-native binding. This is separate from the frozen
// Airframe/Target/provider contracts and from actor reports.
#include "simulation/mission_systems/damage/LiquidSceneV1.h"
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace fsim::session {
struct LiquidMaterialDatasetV1 {
    struct Tank {
        int index=-1,section=-1;
        std::string sectionName;
        double capacityKg=0,densityKgM3=0;
        ms::damage::V3 anchorBodyM;
    };
    std::string design;
    double dragCoefficient=0; ///< Explicit producer-selected liquid policy, separate from air drag.
    std::vector<Tank> tanks;
    std::vector<ms::damage::LiquidCapacityCellV1> cells;
};

// Null with declared=false is retained legacy input. Declared current data
// cannot silently fall back when absent, malformed or inconsistent.
std::shared_ptr<const LiquidMaterialDatasetV1> readLiquidMaterialDatasetV1(
    const std::filesystem::path& directory,const std::string& design,
    const ms::damage::Airframe* airframe,bool& declared,std::string& error);

// Current contents are captured native kg, indexed by native tank ID. The
// caller supplies the current attached geometry and captured body-frame up.
// A piece's origin shift is measured from its parent model's fixed origin.
std::optional<ms::damage::SceneLiquidRecipientV1> freezeLiquidRecipientV1(
    const LiquidMaterialDatasetV1&,const ms::damage::AttachedGeometryView&,
    std::uint64_t identity,std::uint64_t generation,
    const std::vector<std::optional<double>>& contentsKg,
    ms::damage::V3 originShiftBodyM,ms::damage::V3 upBody,
    const ms::damage::SceneLiquidControlsV1&,std::string& error);

// Standalone unsectioned material has real native tanks/cells and deliberately
// has no Airframe, structural component index or detached-owner inference.
std::optional<ms::damage::SceneLiquidRecipientV1> freezeUnsectionedLiquidRecipientV2(
    const LiquidMaterialDatasetV1&,std::uint64_t identity,std::uint64_t generation,
    const std::vector<std::optional<double>>& contentsKg,ms::damage::V3 upBody,
    const ms::damage::SceneLiquidControlsV1&,std::string& error);

// Opt-in referee diagnostics owned by ONE internal World. Default null; never
// available through an actor endpoint. One immutable capture only, no history.
struct LiquidMaterialDiagnosticV1 {
    double sampleTimeS=0;
    std::uint64_t source=0,sourceGeneration=0;
    ms::damage::SceneLiquidRecipientV1 recipient;
    bool unsectioned=false,assessed=false;
    std::vector<ms::damage::SceneLiquidOnlyWorkV2> tankWork;
    ms::damage::V3 impulseBody,angularImpulseBody;
    double fieldAgeS=0;
    std::size_t projectileCount=0;
    std::optional<ms::damage::FragmentFieldV1::Projectile> firstProjectile;
};
struct LiquidMaterialDiagnosticStateV1 {
    std::shared_ptr<const LiquidMaterialDiagnosticV1> last;
};
}
