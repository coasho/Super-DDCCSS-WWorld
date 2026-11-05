#pragma once
#include "simulation/mission_systems/damage/Damage.h"
#include <map>
#include <string>

namespace fsim::ms::damage {
// Private additive policies; existing public layouts and source enums stay fixed.
std::optional<Warhead> readAuthoredFieldPolicyV3(const StoreCard&,std::string& error);
std::optional<Warhead> readAuthoredRingPolicyV4(const StoreCard&,std::string& error);
std::map<std::string,std::string> authoredSourceRoleUnitsV3(const Warhead&);
}
