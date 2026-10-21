#pragma once
#include "simulation/mission_systems/weapon/FuzeAdapter.h"
namespace fsim::ms::weapon {
std::unique_ptr<FuzeAdapter> makeNativeFuzeAdapterV1(std::string id, Spec spec);
std::unique_ptr<FuzeAdapter> makeSweptPathFuzeV1(std::string, Spec);
std::unique_ptr<FuzeAdapter> makeTurningCpaFuzeV1(std::string, Spec);
}
