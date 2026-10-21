#pragma once
#include "simulation/mission_systems/damage/WarheadAdapter.h"
namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeNativeWarheadAdapterV1(std::string id, Warhead model);
std::unique_ptr<WarheadAdapter> makeZonedCaseWarheadV1(std::string, Warhead);
std::unique_ptr<WarheadAdapter> makeSideCaseWarheadV1(std::string, Warhead);
}
