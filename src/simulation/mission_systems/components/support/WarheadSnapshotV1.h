#pragma once
#include "simulation/mission_systems/components/interfaces/WarheadAdapter.h"
#include <string>
#include <utility>
namespace fsim::ms::damage {
class WarheadSnapshotV1 : public WarheadAdapter, public WarheadSceneExecutionV2 {
public:
    WarheadSnapshotV1(std::string id, Warhead model) : id_(std::move(id)), model_(std::move(model)) {}
    std::string_view implementationId() const noexcept override { return id_; }
    const Warhead& model() const noexcept override { return model_; }
    FragmentFieldV1 emitScene(const Burst& burst,Rng& rng) const override {
        return emitFragmentFieldV1(model_,burst,rng,id_=="native.warhead.connected_ring.v1");
    }
protected:
    std::string id_;
    Warhead model_;
};

}
