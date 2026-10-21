#include "NativeFuze.h"
namespace fsim::ms::weapon {
std::unique_ptr<FuzeAdapter> makeNativeFuzeAdapterV1(std::string id,Spec spec) {
    if(spec.fuzeModelVersion==0)return makeSweptPathFuzeV1(std::move(id),std::move(spec));
    if(spec.fuzeModelVersion==1)return makeTurningCpaFuzeV1(std::move(id),std::move(spec));
    return {};
}
}
