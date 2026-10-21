#pragma once

#include "simulation/mission_systems/components/interfaces/FuzeAdapter.h"
#include "simulation/mission_systems/components/interfaces/WarheadAdapter.h"
#include <memory>
#include <string>

namespace fsim::ms::weapon {
std::unique_ptr<FuzeAdapter> makeNativeFuzeAdapterV1(std::string id, Spec spec);
std::unique_ptr<FuzeAdapter> makeSweptPathFuzeV1(std::string, Spec);
std::unique_ptr<FuzeAdapter> makeTurningCpaFuzeV1(std::string, Spec);
}

namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeNativeWarheadAdapterV1(std::string id, Warhead model);
std::unique_ptr<WarheadAdapter> makeZonedCaseWarheadV1(std::string, Warhead);
std::unique_ptr<WarheadAdapter> makeSideCaseWarheadV1(std::string, Warhead);
}
