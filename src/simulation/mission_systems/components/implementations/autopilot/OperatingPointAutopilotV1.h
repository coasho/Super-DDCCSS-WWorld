#pragma once
#include "simulation/mission_systems/components/interfaces/AutopilotProviderV1.h"
#include "simulation/mission_systems/components/interfaces/OperatingPointControlV1.h"
#include "simulation/mission_systems/components/support/NativePlantBindingV1.h"
namespace fsim::ms::components {
class OperatingPointAutopilotV1 final : public NativePlantBindingV1<AutopilotProviderV1>,public OperatingPointControlV1 {
public:
    OperatingPointAutopilotV1(std::string model,weapon::FinControlConfigV1 control);
    weapon::FinCommands commands(const weapon::AutopilotInput&,const weapon::ControlStopsV1&) const noexcept override;
    weapon::OperatingPointResultV1 commandsOperatingPointV1(const weapon::OperatingPointInputV1&) const noexcept override;
private:
    weapon::FinControlConfigV1 control_;
};
}
