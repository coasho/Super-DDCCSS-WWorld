#include "EffectFactoriesV1.h"
#include <utility>
#include "simulation/mission_systems/components/implementations/fuze/SweptPathFuzeV1.h"
#include "simulation/mission_systems/components/implementations/fuze/TurningCpaFuzeV1.h"
#include "simulation/mission_systems/components/implementations/warhead/ZonedCaseWarheadV1.h"
#include "simulation/mission_systems/components/implementations/warhead/SideCaseWarheadV1.h"
#include "simulation/mission_systems/components/implementations/warhead/FiniteZonedCaseWarheadV2.h"

namespace fsim::ms::weapon {
std::unique_ptr<FuzeAdapter> makeNativeFuzeAdapterV1(std::string id,Spec spec) {
    if(spec.fuzeModelVersion==0)return makeSweptPathFuzeV1(std::move(id),std::move(spec));
    if(spec.fuzeModelVersion==1)return makeTurningCpaFuzeV1(std::move(id),std::move(spec));
    return {};
}
}

namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeNativeWarheadAdapterV1(std::string id,Warhead model) {
    if(model.modelVersion==0)return makeZonedCaseWarheadV1(std::move(id),std::move(model));
    if(model.modelVersion==1)return makeSideCaseWarheadV1(std::move(id),std::move(model));
    if(model.modelVersion==2)return makeFiniteZonedCaseWarheadV2(std::move(id),std::move(model));
    return {};
}
}

namespace fsim::ms::weapon {
std::unique_ptr<FuzeAdapter> makeSweptPathFuzeV1(std::string id,Spec spec) {
    return std::make_unique<SweptPathFuzeV1>(std::move(id),std::move(spec));
}
}

namespace fsim::ms::weapon {
std::unique_ptr<FuzeAdapter> makeTurningCpaFuzeV1(std::string id,Spec spec) {
    return std::make_unique<TurningCpaFuzeV1>(std::move(id),std::move(spec));
}
}

namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeZonedCaseWarheadV1(std::string id,Warhead model) {
    return std::make_unique<ZonedCaseWarheadV1>(std::move(id),std::move(model));
}
}

namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeSideCaseWarheadV1(std::string id,Warhead model) {
    return std::make_unique<SideCaseWarheadV1>(std::move(id),std::move(model));
}
std::unique_ptr<WarheadAdapter> makeFiniteZonedCaseWarheadV2(std::string id,Warhead model) {
    return std::make_unique<FiniteZonedCaseWarheadV2>(std::move(id),std::move(model));
}
}
