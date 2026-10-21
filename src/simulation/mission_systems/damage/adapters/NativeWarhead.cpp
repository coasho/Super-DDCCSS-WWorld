#include "NativeWarhead.h"
namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeNativeWarheadAdapterV1(std::string id,Warhead model) {
    if(model.modelVersion==0)return makeZonedCaseWarheadV1(std::move(id),std::move(model));
    if(model.modelVersion==1)return makeSideCaseWarheadV1(std::move(id),std::move(model));
    return {};
}
}
