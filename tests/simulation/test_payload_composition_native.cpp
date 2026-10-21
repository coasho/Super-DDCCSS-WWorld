#include "simulation/runtime/world/World.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <atomic>

namespace {
using namespace fsim;
class StaleAsset {
public:
    explicit StaleAsset(const std::string& model) : model_(model) {
        root_=std::filesystem::temp_directory_path()/("fsim-payload-stale-"+
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        const auto source=std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"stores"/model;
        std::filesystem::create_directories(root_/model/"Engines");
        for (const auto& file:{model+".xml","Engines/"+model+"_motor.xml","Engines/"+model+"_nozzle.xml"}) {
            if (!std::filesystem::exists(source/file)) continue;
            std::filesystem::copy_file(source/file,root_/model/file);
            files_.push_back(root_/model/file);
        }
        const auto xml=root_/model/(model+".xml"); std::ifstream in(xml,std::ios::binary);
        std::stringstream stream;stream<<in.rdbuf();in.close(); auto text=stream.str();
        const auto tag=text.find(">fsim/components/aerodynamics/profile-fingerprint</property>");
        REQUIRE(tag!=std::string::npos); const auto begin=text.rfind("value=\"",tag);
        REQUIRE(begin!=std::string::npos); const auto end=text.find('"',begin+7);
        text.replace(begin+7,end-(begin+7),"0");
        {std::ofstream out(xml,std::ios::binary|std::ios::trunc);out<<text;}
        if (const auto* old=std::getenv("FSIM_AIRCRAFT_PATH")) old_=old;
        const auto selected=root_.string()+(old_.empty()?"":";"+old_);
#ifdef _WIN32
        REQUIRE(_putenv_s("FSIM_AIRCRAFT_PATH",selected.c_str())==0);
#else
        REQUIRE(setenv("FSIM_AIRCRAFT_PATH",selected.c_str(),1)==0);
#endif
    }
    ~StaleAsset() {
#ifdef _WIN32
        _putenv_s("FSIM_AIRCRAFT_PATH",old_.c_str());
#else
        if (old_.empty()) unsetenv("FSIM_AIRCRAFT_PATH");else setenv("FSIM_AIRCRAFT_PATH",old_.c_str(),1);
#endif
        std::error_code error;
        for (const auto& file:files_) std::filesystem::remove(file,error);
        std::filesystem::remove(root_/model_/"Engines",error);
        std::filesystem::remove(root_/model_,error);std::filesystem::remove(root_,error);
    }
private:
    std::string model_,old_;
    std::filesystem::path root_;
    std::vector<std::filesystem::path> files_;
};

void refusedRelease(const char* store,const char* model,int station,bool inert,
                    const fsim::ms::components::ProviderRegistryV1* providers=nullptr) {
    std::unique_ptr<StaleAsset> stale;
    if(!providers)stale=std::make_unique<StaleAsset>(model);
    session::WorldOptions options; options.name="payload-native-identity-refusal";options.publish=false;
    options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World world(options);
    if(providers){std::string error;REQUIRE(world.setPayloadProvidersV1(*providers,error));}
    session::VehicleSpec spec;
    spec.type="jsbsim:f16c";spec.name="carrier";
    spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=250;spec.initial.headingDeg=90;
    const auto own=world.createVehicle(spec);REQUIRE(own==1);
    std::uint16_t stations=0;bool found=false;
    for (const auto& p:world.missionSystems(own)->payloads()) if (p.family==ms::Family::StoreStation) {stations=p.index;found=true;}
    REQUIRE(found); REQUIRE(world.loadStore(own,stations,station,store)==ms::Reason::None);
    if (!inert) {
        REQUIRE(world.submit(own,ms::StateCommand{stations,ms::PayloadState::Operate}).accepted());
        for (unsigned i=0;i<1000 && world.missionSystems(own)->payloads()[stations].state!=ms::PayloadState::Operate;++i) world.step();
        REQUIRE(world.missionSystems(own)->payloads()[stations].state==ms::PayloadState::Operate);
    }
    ms::CommandResult submitted;
    if (inert) {
        ms::JettisonCommand command;command.payload=stations;command.emergency=true;
        submitted=world.submit(own,command);
    } else {
        ms::StrikeCommand command;command.payload=stations;command.store=store;command.consent=true;
        submitted=world.submit(own,command);
    }
    REQUIRE(submitted.accepted());
    bool failed=false;
    for (unsigned i=0;i<600 && !failed;++i) {
        world.step();
        for (const auto& a:world.missionSystems(own)->activities()) if (a.id==submitted.activity && a.state==control::ActivityState::Failed) {
            REQUIRE(a.reason==ms::Reason::InvalidParameter);failed=true;
        }
    }
    REQUIRE(failed);
    REQUIRE(world.weapons().empty());
    REQUIRE(world.engagementEvents().empty());
    const auto inventory=world.stores(own); REQUIRE(inventory.size()==1);
    REQUIRE(inventory.front().status==ms::StoreStatus::Loaded);
    std::uint32_t ids[4]{}; REQUIRE(world.vehicles(ids,4)==1);
    spec.name="after-refusal";REQUIRE(world.createVehicle(spec)==2);
}
}

TEST_CASE("Stale armed native assets refuse without publishing a weapon or consuming an identity", "[payload-components][native-engagement]") {
    refusedRelease("aim-9x","aim9x",1,false);
}
TEST_CASE("Inert missile jettison validates the selected native aerodynamic asset", "[payload-components][native-engagement]") {
    refusedRelease("aim-9x","aim9x",1,true);
}
TEST_CASE("Non-guided store jettison validates the selected native aerodynamic asset", "[payload-components][native-engagement]") {
    refusedRelease("mk82","mk82",3,true);
}
TEST_CASE("A selected native factory refusing creation preserves inventory history and identities", "[payload-components][native-engagement]") {
    auto registry=fsim::ms::components::builtInProvidersV1();std::string error;
    REQUIRE(registry.guidance.registerFactory("fsim.guidance.velocity_normal_pn.v1","fsim.guidance.velocity_normal_pn.v1",
        [](const auto&,const auto&,auto&) -> std::unique_ptr<fsim::ms::components::GuidanceProviderV1> {return {};},true,error));
    refusedRelease("aim-9x","aim9x",1,false,&registry);
}
TEST_CASE("Inert jettison requires only the selected aerodynamic provider catalogue", "[payload-components][native-engagement]") {
    using namespace fsim;
    ms::components::ProviderRegistryV1 registry;
    registry.aerodynamics=ms::components::builtInProvidersV1().aerodynamics;
    session::WorldOptions options;options.name="payload-inert-provider-scope";options.publish=false;
    options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World world(options);std::string error;REQUIRE(world.setPayloadProvidersV1(registry,error));
    session::VehicleSpec spec;spec.type="jsbsim:f16c";spec.name="carrier";
    spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=250;
    const auto own=world.createVehicle(spec);REQUIRE(own);
    std::uint16_t stations=0;bool found=false;
    for(const auto& p:world.missionSystems(own)->payloads())if(p.family==ms::Family::StoreStation){stations=p.index;found=true;}
    REQUIRE(found);REQUIRE(world.loadStore(own,stations,1,"aim-9x")==ms::Reason::None);
    ms::JettisonCommand command;command.payload=stations;command.emergency=true;
    REQUIRE(world.submit(own,command).accepted());
    for(unsigned i=0;i<30 && world.weapons().empty();++i)world.step();
    REQUIRE(world.weapons().size()==1);
    CHECK(world.weapons().front().jettisoned);
    CHECK_FALSE(world.weapons().front().burning);
    CHECK_FALSE(world.weapons().front().tracking);
}

TEST_CASE("All maintained native store assets load and match their selected component configuration", "[payload-components][native-engagement]") {
    session::WorldOptions options;options.name="payload-all-native-assets";options.publish=false;
    options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World world(options);std::vector<std::string> errors;
    const std::filesystem::path library=FSIM_TEST_STORE_DIR;
    const auto cards=ms::readStoreLibrary(library,errors);REQUIRE(errors.empty());REQUIRE(cards.size()==53);
    unsigned full=0,ballistic=0;
    for (const auto& [name,card]:cards) {
        INFO(name); ms::components::ProfileV1 profile;std::string error;
        REQUIRE(ms::components::readProfileV1(library/(name+".json"),*card,profile,error));
        ms::weapon::PlantAdapterV1 selected;
        if (card->seeker.empty()) {
            REQUIRE(ms::weapon::configureBallisticPlantV1(*profile.find("aerodynamics"),*card,selected,error));++ballistic;
        } else {
            REQUIRE(ms::weapon::configurePlantV1(*profile.find("autopilot"),*profile.find("allocator"),*profile.find("actuator"),
                *profile.find("propulsion"),*profile.find("aerodynamics"),*card,selected,error));++full;
        }
        session::VehicleSpec spec;spec.type=card->model;spec.name=name;
        spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=250;spec.initial.enginesRunning=false;
        const auto id=world.createVehicle(spec);REQUIRE(id!=0);auto* model=world.model(id);REQUIRE(model);
        ms::weapon::NativePlantIdentityV1 actual;
        const char* kinds[]={"autopilot","allocator","actuator","propulsion","aerodynamics"};
        for (std::size_t k=0;k<5;++k) if (selected.required[k]) {
            const auto base=std::string("fsim/components/")+kinds[k]+"/";
            const auto schema=model->property(base+"schema-version"),code=model->property(base+"implementation-code"),fp=model->property(base+"profile-fingerprint");
            REQUIRE(schema.valid());REQUIRE(code.valid());REQUIRE(fp.valid());
            actual.components[k]={schema.get(),code.get(),fp.get()};
        }
        REQUIRE(ms::weapon::plantAvailableV1(selected,actual));
        REQUIRE(std::isfinite(model->energy().massKg));REQUIRE(model->energy().massKg>0);
        REQUIRE(world.removeVehicle(id));
    }
    REQUIRE(full==24);REQUIRE(ballistic==29);REQUIRE(world.weapons().empty());
}

namespace {
using namespace fsim::ms::components;
struct ProviderCounts {
    std::array<std::atomic<unsigned>,9> created{},executed{};
    void made(std::size_t role) {created[role].fetch_add(1,std::memory_order_relaxed);}
    void used(std::size_t role) {executed[role].fetch_add(1,std::memory_order_relaxed);}
};
class ObservedSeeker final : public SeekerProviderV1 {
public:
    ObservedSeeker(std::unique_ptr<SeekerProviderV1> inner,std::shared_ptr<ProviderCounts> counts):inner_(std::move(inner)),counts_(std::move(counts)){}
    const fsim::ms::weapon::SeekerConfigV1& config() const noexcept override {return inner_->config();}
    double infraredRange(double a,double b,bool c) const noexcept override {counts_->used(0);return inner_->infraredRange(a,b,c);}
    double radarRange(double a,double b,bool c) const noexcept override {counts_->used(0);return inner_->radarRange(a,b,c);}
    bool notched(double a,double b) const noexcept override {counts_->used(0);return inner_->notched(a,b);}
    double decoySignal(double a,bool b) const noexcept override {counts_->used(0);return inner_->decoySignal(a,b);}
    bool illuminated(const double a[3],double b,double c,double d,double e) const noexcept override {counts_->used(0);return inner_->illuminated(a,b,c,d,e);}
private:
    std::unique_ptr<SeekerProviderV1> inner_;std::shared_ptr<ProviderCounts> counts_;
};
class ObservedGuidance final : public GuidanceProviderV1 {
public:
    ObservedGuidance(std::unique_ptr<GuidanceProviderV1> inner,std::shared_ptr<ProviderCounts> counts):inner_(std::move(inner)),counts_(std::move(counts)){}
    fsim::ms::weapon::Spec spec() const noexcept override {return inner_->spec();}
    fsim::ms::weapon::Output update(const fsim::ms::weapon::Input& input) override {counts_->used(1);return inner_->update(input);}
private:
    std::unique_ptr<GuidanceProviderV1> inner_;std::shared_ptr<ProviderCounts> counts_;
};
template<class Interface> class ObservedBinding : public Interface {
public:
    ObservedBinding(std::unique_ptr<Interface> inner,std::shared_ptr<ProviderCounts> counts,std::size_t role)
        :inner_(std::move(inner)),counts_(std::move(counts)),role_(role){}
    std::string_view nativeModel() const noexcept override {return inner_->nativeModel();}
    bool available(const fsim::ms::weapon::NativePlantMetadataV1& a,const fsim::ms::weapon::NativePlantMetadataV1& b) const noexcept override {
        counts_->used(role_);return inner_->available(a,b);
    }
protected:
    std::unique_ptr<Interface> inner_;std::shared_ptr<ProviderCounts> counts_;std::size_t role_;
};
class ObservedAutopilot final : public ObservedBinding<AutopilotProviderV1> {
public:
    using ObservedBinding::ObservedBinding;
    fsim::ms::weapon::FinCommands commands(const fsim::ms::weapon::AutopilotInput& a,const fsim::ms::weapon::ControlStopsV1& b) const noexcept override {
        counts_->used(role_);return inner_->commands(a,b);
    }
};
class ObservedAllocator final : public ObservedBinding<AllocatorProviderV1> {
public:
    using ObservedBinding::ObservedBinding;
    fsim::ms::weapon::FinCommands allocate(fsim::ms::weapon::FinCommands a,const fsim::ms::weapon::ControlStopsV1& b) const noexcept override {
        counts_->used(role_);return inner_->allocate(a,b);
    }
};
class ObservedActuator final : public ObservedBinding<ActuatorProviderV1> {
public:
    using ObservedBinding::ObservedBinding;
    bool accepts(const fsim::ms::weapon::AutopilotInput& a,const fsim::ms::weapon::ControlStopsV1& b) const noexcept override {
        counts_->used(role_);return inner_->accepts(a,b);
    }
};
class ObservedPropulsion final : public ObservedBinding<PropulsionProviderV1> {
public:
    using ObservedBinding::ObservedBinding;
    bool acceptsEngineCount(unsigned a) const noexcept override {counts_->used(role_);return inner_->acceptsEngineCount(a);}
    bool burning(double a) const noexcept override {counts_->used(role_);return inner_->burning(a);}
};
class ObservedAerodynamics final : public ObservedBinding<AerodynamicsProviderV1> {
public:
    using ObservedBinding::ObservedBinding;
    std::string_view coefficientSource() const noexcept override {return inner_->coefficientSource();}
};
template<class Wrapper,class Interface>
bool observeBinding(ProviderTableV1<Interface>& target,const ProviderTableV1<Interface>& original,const std::string& id,
    std::shared_ptr<ProviderCounts> counts,std::size_t role,std::string& error) {
    return target.registerFactory(id,id,[table=original,counts,role](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<Interface> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(role);
        return std::make_unique<Wrapper>(std::move(inner),counts,role);
    },true,error);
}
class ObservedWarhead final : public fsim::ms::damage::WarheadAdapter {
public:
    ObservedWarhead(std::unique_ptr<fsim::ms::damage::WarheadAdapter> inner,std::shared_ptr<ProviderCounts> counts):inner_(std::move(inner)),counts_(std::move(counts)){}
    std::string_view implementationId() const noexcept override {return inner_->implementationId();}
    const fsim::ms::damage::Warhead& model() const noexcept override {return inner_->model();}
    fsim::ms::damage::Impact assess(const fsim::ms::damage::Burst& a,const fsim::ms::damage::AttachedGeometryView& b,
        const fsim::ms::damage::Target& c,fsim::Rng& d) const override {counts_->used(8);return inner_->assess(a,b,c,d);}
private:
    std::unique_ptr<fsim::ms::damage::WarheadAdapter> inner_;std::shared_ptr<ProviderCounts> counts_;
};
}

TEST_CASE("Native World executes replacement providers from a frozen per-World catalogue", "[payload-components][native-engagement]") {
    using namespace fsim;
    auto registry=builtInProvidersV1();const auto defaults=registry;
    const auto counts=std::make_shared<ProviderCounts>();std::string error;
    std::vector<std::string> errors;
    const auto card=ms::readStoreCard(std::filesystem::path(FSIM_TEST_STORE_DIR)/"aim-9x.json",errors);
    REQUIRE(card);REQUIRE(errors.empty());ProfileV1 profile;
    REQUIRE(readProfileV1(std::filesystem::path(FSIM_TEST_STORE_DIR)/"aim-9x.json",*card,profile,error));
    const auto id=[&](const char* role){return profile.find(role)->implementationId;};
    REQUIRE(registry.seeker.registerFactory(id("seeker"),id("seeker"),[table=defaults.seeker,counts](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<SeekerProviderV1> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(0);return std::make_unique<ObservedSeeker>(std::move(inner),counts);
    },true,error));
    REQUIRE(registry.guidance.registerFactory(id("guidance"),id("guidance"),[table=defaults.guidance,counts](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<GuidanceProviderV1> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(1);return std::make_unique<ObservedGuidance>(std::move(inner),counts);
    },true,error));
    REQUIRE(observeBinding<ObservedAutopilot>(registry.autopilot,defaults.autopilot,id("autopilot"),counts,2,error));
    REQUIRE(observeBinding<ObservedAllocator>(registry.allocator,defaults.allocator,id("allocator"),counts,3,error));
    REQUIRE(observeBinding<ObservedActuator>(registry.actuator,defaults.actuator,id("actuator"),counts,4,error));
    REQUIRE(observeBinding<ObservedPropulsion>(registry.propulsion,defaults.propulsion,id("propulsion"),counts,5,error));
    REQUIRE(observeBinding<ObservedAerodynamics>(registry.aerodynamics,defaults.aerodynamics,id("aerodynamics"),counts,6,error));
    REQUIRE(registry.fuze.registerFactory(id("fuze"),id("fuze"),[table=defaults.fuze,counts](const auto& d,const auto& ctx,auto& e) {
        auto inner=table.create(d,ctx,e);if(inner)counts->made(7);return inner;
    },true,error));
    REQUIRE(registry.warhead.registerFactory(id("warhead"),id("warhead"),[table=defaults.warhead,counts](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<ms::damage::WarheadAdapter> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(8);return std::make_unique<ObservedWarhead>(std::move(inner),counts);
    },true,error));
    session::WorldOptions options;options.name="payload-provider-dispatch";options.publish=false;
    options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World world(options);REQUIRE(world.setPayloadProvidersV1(registry,error));
    // Mutation of the caller's catalogue cannot replace this World's snapshot.
    REQUIRE(registry.guidance.registerFactory(id("guidance"),id("guidance"),[](const auto&,const auto&,auto&) -> std::unique_ptr<GuidanceProviderV1> {return {};},true,error));
    session::VehicleSpec spec;spec.type="jsbsim:f16c";spec.name="carrier";
    spec.initial.latitudeDeg=36.25;spec.initial.longitudeDeg=-115.03;
    spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=250;spec.initial.headingDeg=90;
    const auto own=world.createVehicle(spec);REQUIRE(own);
    CHECK_FALSE(world.setPayloadProvidersV1(registry,error));
    spec.name="candidate";spec.initial.longitudeDeg+=.06;spec.initial.headingDeg=270;
    const auto target=world.createVehicle(spec);REQUIRE(target);
    std::uint16_t stations=0;bool found=false;
    for(const auto& p:world.missionSystems(own)->payloads())if(p.family==ms::Family::StoreStation){stations=p.index;found=true;}
    REQUIRE(found);REQUIRE(world.loadStore(own,stations,1,"aim-9x")==ms::Reason::None);
    REQUIRE(world.submit(own,ms::StateCommand{stations,ms::PayloadState::Operate}).accepted());
    for(unsigned i=0;i<1000 && world.missionSystems(own)->payloads()[stations].state!=ms::PayloadState::Operate;++i)world.step();
    REQUIRE(world.missionSystems(own)->payloads()[stations].state==ms::PayloadState::Operate);
    ms::StrikeCommand strike;strike.payload=stations;strike.store="aim-9x";strike.target=target;strike.consent=true;
    REQUIRE(world.submit(own,strike).accepted());
    for(unsigned i=0;i<150 && world.weapons().empty();++i)world.step();
    REQUIRE(world.weapons().size()==1);world.step(10);
    for(std::size_t k=0;k<9;++k){INFO(k);CHECK(counts->created[k].load()>0);}
    for(std::size_t k=0;k<7;++k){INFO(k);CHECK(counts->executed[k].load()>0);}
    // Independent referee dispatch fixture, not an intercept or damage gate.
    // No assertion here claims a hit, loss or completion of either fixed scene.
    session::Detonation burst;burst.store="aim-9x";burst.weapon=world.weapons().front().id;
    const auto* state=world.vehicleState(target);REQUIRE(state);
    std::copy(state->positionEcef,state->positionEcef+3,burst.positionEcef);
    world.detonate(burst);CHECK(counts->executed[8].load()>0);
}
