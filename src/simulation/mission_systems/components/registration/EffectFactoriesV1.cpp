#include "EffectFactoriesV1.h"
#include <utility>
#include "simulation/mission_systems/components/implementations/fuze/SweptPathFuzeV1.h"
#include "simulation/mission_systems/components/implementations/fuze/TurningCpaFuzeV1.h"
#include "simulation/mission_systems/components/implementations/warhead/ZonedCaseWarheadV1.h"
#include "simulation/mission_systems/components/implementations/warhead/SideCaseWarheadV1.h"
#include "simulation/mission_systems/components/implementations/warhead/FiniteZonedCaseWarheadV2.h"
#include "simulation/mission_systems/components/implementations/warhead/AuthoredFieldWarheadV3.h"

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
    if(model.modelVersion==3&&id=="native.warhead.authored_field.v3")return makeAuthoredFieldWarheadV3(std::move(id),std::move(model));
    if(model.modelVersion==4&&id=="native.warhead.authored_ring.v4")return makeAuthoredRingWarheadV4(std::move(id),std::move(model));
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

namespace fsim::ms::damage {
std::unique_ptr<WarheadAdapter> makeAuthoredFieldWarheadV3(std::string id,Warhead model) {
    if(id!="native.warhead.authored_field.v3"||model.modelVersion!=3)return {};
    return std::make_unique<AuthoredFieldWarheadV3>(std::move(id),std::move(model));
}
std::unique_ptr<WarheadAdapter> makeAuthoredRingWarheadV4(std::string id,Warhead model) {
    if(id!="native.warhead.authored_ring.v4"||model.modelVersion!=4)return {};
    return std::make_unique<AuthoredRingWarheadV4>(std::move(id),std::move(model));
}
}
