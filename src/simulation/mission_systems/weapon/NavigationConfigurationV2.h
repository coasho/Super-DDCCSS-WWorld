#pragma once
#include "NavigationStrapdownV2.h"
#include "simulation/mission_systems/Cards.h"
#include <optional>
#include <string>
namespace fsim::ms::weapon {
/// Absent selection preserves legacy cards; explicit malformed policies refuse.
bool navigationConfigurationV2(const StoreCard&,std::optional<NavigationStrapdownConfigV2>&,std::string&);
}
