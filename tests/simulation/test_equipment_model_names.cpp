#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "contracts/equipment/EquipmentModels.h"
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <set>

namespace {
using namespace fsim::ms;
using namespace fsim::ms::components;
struct Inputs {
    std::map<std::string,std::shared_ptr<const StoreCard>> cards;
    Inputs() {
        std::vector<std::string> errors;
        const auto loaded=readStoreLibrary(std::filesystem::path(FSIM_TEST_STORE_DIR),errors);
        REQUIRE(errors.empty());
        for(const auto& [id,card]:loaded)cards.emplace(id,card);
    }
    ProfileV1 named(const std::string& store) const {
        ProfileV1 profile;std::string error;
        REQUIRE(readProfileV1(std::filesystem::path(FSIM_TEST_STORE_DIR)/(store+".json"),*cards.at(store),profile,error));
        for(auto& [kind,d]:profile.components)if(d.availability==AvailabilityV1::Available) {
            const auto* model=fsim::equipment::component(store,kind);REQUIRE(model);
            d.implementationId=std::string(model->selectionId);
        }
        return profile;
    }
};
}

TEST_CASE("Every equipment model selects its actual component with an unchanged mechanism schema", "[equipment-model-names]") {
    Inputs source;const auto registry=builtInProvidersV1();std::set<std::string> identities;
    REQUIRE(fsim::equipment::models.size()==source.cards.size());
    for(const auto& model:fsim::equipment::models) {
        const auto store=std::string(model.storeId);INFO(store);REQUIRE(identities.insert(store).second);
        REQUIRE(source.cards.at(store)->model==model.nativeModel);
        ProfileV1 normalized;std::string error;const auto selected=source.named(store);
        INFO(error);REQUIRE(registry.validationProfile(selected,normalized,error));
        for(const auto& [kind,d]:normalized.components)if(d.availability==AvailabilityV1::Available) {
            const auto* actual=fsim::equipment::component(store,kind);REQUIRE(actual);
            CHECK(d.implementationId==actual->mechanismId);
            CHECK(d.parameters.size()==selected.components.at(kind).parameters.size());
            CHECK(weapon::plantFingerprintV1(d)==weapon::plantFingerprintV1(selected.components.at(kind)));
        }
    }
    CHECK(fsim::equipment::model("aim-120c")->designation=="AIM-120C-7");
    CHECK(fsim::equipment::model("aim-9x")->designation=="AIM-9X Block I");
}

TEST_CASE("Wrong named equipment and component roles refuse without replacing accepted input", "[equipment-model-names]") {
    Inputs source;const auto registry=builtInProvidersV1();auto selected=source.named("aim-120d");
    ProfileV1 accepted;std::string error;REQUIRE(registry.validationProfile(selected,accepted,error));
    const auto original=accepted.components.at("seeker").implementationId;
    for(const auto* wrong:{"equipment.aim-120c-7.seeker","equipment.aim-120d.warhead","equipment.aim-120d-3.seeker"}) {
        auto changed=selected;changed.components.at("seeker").implementationId=wrong;
        CHECK_FALSE(registry.validationProfile(changed,accepted,error));
        CHECK(accepted.components.at("seeker").implementationId==original);
    }
    auto custom=registry;
    const auto factory=[](const DescriptorV1&,const ProviderContextV1&,std::string&) {
        return std::unique_ptr<SeekerProviderV1>{};
    };
    for(const auto* wrong:{"equipment.aim-120d-3.seeker","equipment.aim-120d.warhead"}) {
        const auto size=custom.seeker.size();
        CHECK_FALSE(custom.seeker.registerFactory(wrong,"fsim.seeker.active_radar.v3",factory,false,error));
        CHECK(custom.seeker.size()==size);
    }
}

TEST_CASE("Named model construction keeps catalogue copies and backend replacement independent", "[equipment-model-names]") {
    Inputs source;auto registry=builtInProvidersV1();const auto original=registry;
    auto profile=source.named("aim-9m");const auto& card=*source.cards.at("aim-9m");
    weapon::Spec spec;std::string error;REQUIRE(weapon::configureCompositionV1(card,profile,spec,spec,error,registry));
    const auto* model=fsim::equipment::component("aim-9m","seeker");REQUIRE(model);
    unsigned calls=0;const auto previous=registry.seeker;
    REQUIRE(registry.seeker.registerFactory(std::string(model->mechanismId),std::string(model->mechanismId),
        [previous,&calls](const auto& d,const auto& context,auto& e){++calls;return previous.create(d,context,e);},true,error));
    ProviderContextV1 context{card,profile,spec,std::nullopt};
    REQUIRE(registry.seeker.create(profile.components.at("seeker"),context,error));CHECK(calls==1);
    REQUIRE(original.seeker.create(profile.components.at("seeker"),context,error));CHECK(calls==1);
    auto wrong=card;wrong.model="jsbsim:aim120d";
    ProviderContextV1 wrongContext{wrong,profile,spec,std::nullopt};
    CHECK_FALSE(registry.seeker.create(profile.components.at("seeker"),wrongContext,error));CHECK(calls==1);
}
