#pragma once
#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/Cards.h"
#include "contracts/equipment/EquipmentModels.h"

namespace fsim::ms::components {
// Decode only equipment identities. Legacy/custom selection semantics belong
// to their existing decoders or provider registry and remain unchanged.
inline bool equipmentSelectionV1(const DescriptorV1& input,std::string_view kind,
    const StoreCard* card,DescriptorV1& output,std::string& error) {
    const auto* part=equipment::component(input.implementationId);
    if(!part||part->kind!=kind||input.kind!=kind) {
        error="unknown equipment selection or mismatched component role";return false;
    }
    if(card) {
        const auto* model=equipment::model(part->storeId);
        if(!model||card->id!=model->storeId||card->model!=model->nativeModel||
           card->designation!=model->cardDesignation) {
            error="equipment selection belongs to another card or native model";return false;
        }
    }
    output=input;output.implementationId=std::string(part->mechanismId);return true;
}
}
