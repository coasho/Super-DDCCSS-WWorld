#pragma once

#include "simulation/mission_systems/components/support/WarheadSnapshotV1.h"

namespace fsim::ms::damage {

/// Explicit generic game-reference angular field over a finite annular source.
/// Source provenance and equipment-calibration limits belong to its selection.
class FiniteZonedCaseWarheadV2 final : public WarheadSnapshotV1 {
public:
    using WarheadSnapshotV1::WarheadSnapshotV1;
    Impact assess(const Burst&,const AttachedGeometryView&,const Target&,Rng&) const override;
};

} // namespace fsim::ms::damage
