#include "WarheadSnapshot.h"
namespace fsim::ms::damage {
namespace {
class SideCaseWarheadV1 final : public WarheadSnapshotV1 {
public:
    using WarheadSnapshotV1::WarheadSnapshotV1;
    Impact assess(const Burst& burst,const AttachedGeometryView& geometry,const Target& target,Rng& rng) const override {
        return damage::assess(model_,burst,geometry,target,rng);
    }
};
}
std::unique_ptr<WarheadAdapter> makeSideCaseWarheadV1(std::string id,Warhead model) {
    return std::make_unique<SideCaseWarheadV1>(std::move(id),std::move(model));
}
}
