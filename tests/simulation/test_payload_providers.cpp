#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/components/implementations/seeker/EncounterSeekerV1.h"
#include "simulation/mission_systems/components/implementations/guidance/VelocityNormalPnGuidanceV1.h"
#include "simulation/mission_systems/components/implementations/autopilot/LoadFeedbackCascadeAutopilotV1.h"
#include "simulation/mission_systems/components/implementations/allocator/JsbsimAuthoredFinMixerV1.h"
#include "simulation/mission_systems/components/implementations/actuator/JsbsimLagRateStopsActuatorV1.h"
#include "simulation/mission_systems/components/implementations/propulsion/JsbsimSolidRocketPropulsionV1.h"
#include "simulation/mission_systems/components/implementations/aerodynamics/JsbsimGeneratedStoreAerodynamicsV1.h"
#include "simulation/mission_systems/components/implementations/fuze/SweptPathFuzeV1.h"
#include "simulation/mission_systems/components/implementations/fuze/TurningCpaFuzeV1.h"
#include "simulation/mission_systems/components/implementations/warhead/ZonedCaseWarheadV1.h"
#include "simulation/mission_systems/components/implementations/warhead/SideCaseWarheadV1.h"
#include <catch2/catch_test_macros.hpp>
#include <type_traits>

namespace {
using namespace fsim::ms;
using namespace fsim::ms::components;
class SyntheticGuidance final : public GuidanceProviderV1 {
public:
    explicit SyntheticGuidance(double value) : value_(value) {}
    weapon::Spec spec() const noexcept override {return {};}
    weapon::Output update(const weapon::Input&) override {weapon::Output out;out.acceleration.x=value_;return out;}
private:
    double value_;
};
DescriptorV1 selection(const char* id) {
    DescriptorV1 d;d.kind="guidance";d.implementationId=id;d.availability=AvailabilityV1::Available;return d;
}
template<class Interface,class Concrete> constexpr bool concreteProviderV1 =
    std::is_abstract_v<Interface> && std::is_base_of_v<Interface,Concrete> &&
    !std::is_abstract_v<Concrete> && std::is_final_v<Concrete>;
}
TEST_CASE("Named provider headers expose concrete models while shared helpers remain abstract", "[payload-components][providers]") {
    STATIC_REQUIRE((concreteProviderV1<SeekerProviderV1,EncounterSeekerV1>));
    STATIC_REQUIRE((concreteProviderV1<GuidanceProviderV1,VelocityNormalPnGuidanceV1>));
    STATIC_REQUIRE((concreteProviderV1<AutopilotProviderV1,LoadFeedbackCascadeAutopilotV1>));
    STATIC_REQUIRE((concreteProviderV1<AllocatorProviderV1,JsbsimAuthoredFinMixerV1>));
    STATIC_REQUIRE((concreteProviderV1<ActuatorProviderV1,JsbsimLagRateStopsActuatorV1>));
    STATIC_REQUIRE((concreteProviderV1<PropulsionProviderV1,JsbsimSolidRocketPropulsionV1>));
    STATIC_REQUIRE((concreteProviderV1<AerodynamicsProviderV1,JsbsimGeneratedStoreAerodynamicsV1>));
    STATIC_REQUIRE((concreteProviderV1<weapon::FuzeAdapter,weapon::SweptPathFuzeV1>));
    STATIC_REQUIRE((concreteProviderV1<weapon::FuzeAdapter,weapon::TurningCpaFuzeV1>));
    STATIC_REQUIRE((concreteProviderV1<damage::WarheadAdapter,damage::ZonedCaseWarheadV1>));
    STATIC_REQUIRE((concreteProviderV1<damage::WarheadAdapter,damage::SideCaseWarheadV1>));
    STATIC_REQUIRE(std::is_abstract_v<NativeBindingProviderV1>);
    STATIC_REQUIRE(std::is_abstract_v<NativePlantBindingV1<AutopilotProviderV1>>);
    STATIC_REQUIRE(std::is_abstract_v<weapon::CapturedFuzeLifecycleV1>);
    STATIC_REQUIRE(std::is_abstract_v<damage::WarheadSnapshotV1>);
}
TEST_CASE("Native payload catalogue covers every role and both existing effect algorithms", "[payload-components][providers]") {
    const auto registry=builtInProvidersV1();
    CHECK(registry.seeker.size()>0);CHECK(registry.guidance.size()>0);
    CHECK(registry.fuze.size()>=2);CHECK(registry.warhead.size()>=2);
    CHECK(registry.autopilot.size()>0);CHECK(registry.allocator.size()>0);CHECK(registry.actuator.size()>0);
    CHECK(registry.propulsion.size()>0);CHECK(registry.aerodynamics.size()>0);
    STATIC_REQUIRE(std::is_abstract_v<AutopilotProviderV1>);
    STATIC_REQUIRE(std::is_abstract_v<AllocatorProviderV1>);
    STATIC_REQUIRE(std::is_abstract_v<ActuatorProviderV1>);
    STATIC_REQUIRE(std::is_abstract_v<PropulsionProviderV1>);
    STATIC_REQUIRE(std::is_abstract_v<AerodynamicsProviderV1>);
    STATIC_REQUIRE_FALSE(std::is_same_v<AutopilotProviderV1,AllocatorProviderV1>);
}
TEST_CASE("Typed provider replacement affects new snapshots without mutating retained factories", "[payload-components][providers]") {
    ProviderRegistryV1 registry;std::string error;
    auto factory=[](double value) {return [value](const DescriptorV1&,const ProviderContextV1&,std::string& e) {
        e.clear();return std::make_unique<SyntheticGuidance>(value);
    };};
    REQUIRE(registry.guidance.registerFactory("synthetic", "synthetic-schema",factory(1),false,error));
    const auto snapshot=registry;
    CHECK_FALSE(registry.guidance.registerFactory("synthetic","synthetic-schema",factory(2),false,error));
    REQUIRE(registry.guidance.registerFactory("synthetic","synthetic-schema",factory(2),true,error));
    StoreCard card;ProfileV1 profile;ProviderContextV1 context{card,profile,{},std::nullopt};
    auto first=snapshot.guidance.create(selection("synthetic"),context,error);REQUIRE(first);
    auto second=registry.guidance.create(selection("synthetic"),context,error);REQUIRE(second);
    CHECK(first->update({}).acceleration.x==1);CHECK(second->update({}).acceleration.x==2);
}
TEST_CASE("Provider creation rejects wrong role version unknown identity null and throwing factories", "[payload-components][providers]") {
    ProviderRegistryV1 registry;std::string error;StoreCard card;ProfileV1 profile;
    ProviderContextV1 context{card,profile,{},std::nullopt};
    REQUIRE(registry.guidance.registerFactory("null","schema",[](const auto&,const auto&,auto&) -> std::unique_ptr<GuidanceProviderV1> {return {};},false,error));
    REQUIRE(registry.guidance.registerFactory("throws","schema",[](const auto&,const auto&,auto&) -> std::unique_ptr<GuidanceProviderV1> {throw std::runtime_error("synthetic failure");},false,error));
    for(const auto* id:{"unknown","null","throws"}) {
        CHECK_FALSE(registry.guidance.create(selection(id),context,error));CHECK_FALSE(error.empty());
    }
    auto d=selection("null");d.kind="seeker";CHECK_FALSE(registry.guidance.create(d,context,error));
    d=selection("null");d.configurationVersion=2;CHECK_FALSE(registry.guidance.create(d,context,error));
    d=selection("null");d.availability=AvailabilityV1::Unavailable;CHECK_FALSE(registry.guidance.create(d,context,error));
    CHECK_FALSE(registry.guidance.registerFactory("bad","schema",{},false,error));
}
