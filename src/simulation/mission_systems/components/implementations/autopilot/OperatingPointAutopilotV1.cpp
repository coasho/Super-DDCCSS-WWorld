#include "OperatingPointAutopilotV1.h"
#include <utility>
namespace fsim::ms::components {
OperatingPointAutopilotV1::OperatingPointAutopilotV1(std::string model,weapon::FinControlConfigV1 control)
    : NativePlantBindingV1(std::move(model)),control_(control) {}
weapon::FinCommands OperatingPointAutopilotV1::commands(const weapon::AutopilotInput&,const weapon::ControlStopsV1&) const noexcept {
    return {}; // This version requires its actual operating-point sibling snapshot.
}
weapon::OperatingPointResultV1 OperatingPointAutopilotV1::commandsOperatingPointV1(const weapon::OperatingPointInputV1& in) const noexcept {
    return weapon::operatingPointInputCommandsV1(in,control_);
}
}
