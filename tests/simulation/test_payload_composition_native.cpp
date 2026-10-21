#include "simulation/runtime/world/World.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>

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

void refusedRelease(const char* store,const char* model,int station,bool inert) {
    StaleAsset stale(model);
    session::WorldOptions options; options.name="payload-native-identity-refusal";options.publish=false;
    options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World world(options); session::VehicleSpec spec;
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
