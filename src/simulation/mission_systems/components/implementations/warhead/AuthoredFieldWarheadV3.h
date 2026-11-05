#pragma once
#include "simulation/mission_systems/components/support/WarheadSnapshotV1.h"
namespace fsim::ms::damage {
class AuthoredFieldWarheadV3 final : public WarheadSnapshotV1 {
public:
    using WarheadSnapshotV1::WarheadSnapshotV1;
    FragmentFieldV1 emitScene(const Burst&,Rng&) const override;
    Impact assess(const Burst&,const AttachedGeometryView&,const Target&,Rng&) const override;
};
// Separate typed connected topology; never routes through the discrete-rod field.
class AuthoredRingWarheadV4 final : public WarheadSnapshotV1 {
public:
    using WarheadSnapshotV1::WarheadSnapshotV1;
    FragmentFieldV1 emitScene(const Burst&,Rng&) const override;
    Impact assess(const Burst&,const AttachedGeometryView&,const Target&,Rng&) const override;
};
}
