#include "EquipmentModels.h"
#include "contracts/equipment/EquipmentModels.h"
#include <stdexcept>

namespace fsim::ms::components {
void registerEquipmentModels(ProviderRegistryV1& registry) {
    const auto registerRole=[](const char* kind,auto& table) {
        for(const auto& model:equipment::components)if(model.kind==kind) {
            std::string error;
            if(!table.registerEquipmentModel(model.selectionId,error))
                throw std::logic_error(std::string(model.selectionId)+": "+error);
        }
    };
    registerRole("seeker",registry.seeker);registerRole("guidance",registry.guidance);
    registerRole("autopilot",registry.autopilot);registerRole("allocator",registry.allocator);
    registerRole("actuator",registry.actuator);registerRole("fuze",registry.fuze);
    registerRole("warhead",registry.warhead);registerRole("propulsion",registry.propulsion);
    registerRole("aerodynamics",registry.aerodynamics);
}
}
