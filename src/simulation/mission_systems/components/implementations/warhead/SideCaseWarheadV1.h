#pragma once

#include "simulation/mission_systems/components/support/WarheadSnapshotV1.h"

namespace fsim::ms::damage {

class SideCaseWarheadV1 final : public WarheadSnapshotV1 {
public:
    using WarheadSnapshotV1::WarheadSnapshotV1;
    Impact assess(const Burst& burst,const AttachedGeometryView& geometry,const Target& target,Rng& rng) const override;
};

} // namespace fsim::ms::damage
