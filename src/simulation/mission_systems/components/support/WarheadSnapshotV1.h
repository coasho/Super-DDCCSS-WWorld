#pragma once
#include "simulation/mission_systems/components/interfaces/WarheadAdapter.h"
#include <string>
#include <utility>
namespace fsim::ms::damage {
class WarheadSnapshotV1 : public WarheadAdapter {
public:
    WarheadSnapshotV1(std::string id, Warhead model) : id_(std::move(id)), model_(std::move(model)) {}
    std::string_view implementationId() const noexcept override { return id_; }
    const Warhead& model() const noexcept override { return model_; }
protected:
    std::string id_;
    Warhead model_;
};

}
