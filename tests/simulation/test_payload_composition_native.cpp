#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include <cmath>
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
        // A stale native identity is the fault under test. Supply the genuine
        // carriage table so admission reaches that guard rather than failing
        // earlier because the fixture omitted a required physical asset.
        if(std::filesystem::is_directory(source/"aero")) {
            std::filesystem::create_directories(root_/model/"aero");
            for(const auto& file:std::filesystem::directory_iterator(source/"aero"))
                if(file.is_regular_file() && file.path().extension()==".json" &&
                    file.path().filename().string().rfind("vlm-carriage-",0)==0) {
                    const auto destination=root_/model/"aero"/file.path().filename();
                    std::filesystem::copy_file(file.path(),destination);files_.push_back(destination);
                }
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
        std::filesystem::remove(root_/model_/"aero",error);
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

TEST_CASE("Armed ballistic stores use native terrain contact while unarmed releases remain inert", "[payload-components][native-engagement]") {
    using namespace fsim;
    for(const auto mode:{0,1,2}) {
        INFO("0=armed contact, 1=pre-arming contact, 2=inert jettison; mode="<<mode);
        session::WorldOptions options;options.name="ballistic-native-contact";options.publish=false;
        options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
        session::World world(options);
        session::VehicleSpec spec;spec.type="jsbsim:f16c";spec.name="carrier";
        spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=250;
        const auto own=world.createVehicle(spec);REQUIRE(own);
        std::uint16_t stations=0;bool found=false;
        for(const auto& p:world.missionSystems(own)->payloads())if(p.family==ms::Family::StoreStation){stations=p.index;found=true;}
        REQUIRE(found);REQUIRE(world.loadStore(own,stations,3,"mk82")==ms::Reason::None);
        if(mode==2) {
            ms::JettisonCommand c;c.payload=stations;c.emergency=true;REQUIRE(world.submit(own,c).accepted());
        } else {
            REQUIRE(world.submit(own,ms::StateCommand{stations,ms::PayloadState::Operate}).accepted());
            for(unsigned k=0;k<1000 && world.missionSystems(own)->payloads()[stations].state!=ms::PayloadState::Operate;++k)world.step();
            REQUIRE(world.missionSystems(own)->payloads()[stations].state==ms::PayloadState::Operate);
            ms::StrikeCommand c;c.payload=stations;c.store="mk82";c.consent=true;REQUIRE(world.submit(own,c).accepted());
        }
        for(unsigned k=0;k<600 && world.weapons().empty();++k)world.step();
        REQUIRE(world.weapons().size()==1);const auto weapon=world.weapons().front().id;
        // Vehicle.reset deliberately preserves release age and the fuze latch.
        // Wait only the armed cases, then provide an explicit physical impact.
        if(mode!=1)for(unsigned k=0;k<90;++k)world.step();
        sim::InitialConditions impact;impact.altitudeMslM=2;impact.airspeedTrueMs=0;
        impact.velocityNedMs[0]=0;impact.velocityNedMs[1]=0;impact.velocityNedMs[2]=40;
        impact.enginesRunning=false;REQUIRE(world.resetVehicle(weapon,&impact));
        for(unsigned k=0;k<300 && world.alive(weapon);++k)world.step();
        if(mode==0)REQUIRE_FALSE(world.alive(weapon));
        else if(world.alive(weapon)) {
            // A spring-arrested body need not cross the terminal crash depth.
            // Prove that the earlier unarmed initiation cannot be rearmed by
            // resetting its pose after the arming time and striking again.
            if(mode==1) {
                const auto* source=dynamic_cast<const sim::TerrainContactSourceV1*>(world.model(weapon));
                REQUIRE(source);sim::TerrainContactSnapshotV1 contact;REQUIRE(source->terrainContactSnapshotV1(contact));
                REQUIRE(contact.state.simTime<1.0);
            }
            impact.velocityNedMs[2]=80;
            REQUIRE(world.resetVehicle(weapon,&impact));
            for(unsigned k=0;k<300 && world.alive(weapon);++k)world.step();
        }
        const auto records=world.weapons();REQUIRE(records.size()==1);
        if(mode==0)CHECK(records.front().end==ms::WeaponEnd::Detonated);
        else CHECK(records.front().end!=ms::WeaponEnd::Detonated);
        unsigned detonations=0;
        for(const auto& event:world.engagementEvents())if(event.entity==weapon && event.kind=="detonation") {
            ++detonations;for(double coordinate:event.positionEcef)CHECK(std::isfinite(coordinate));
        }
        CHECK(detonations==(mode==0?1:0));
    }
}
TEST_CASE("Terrain initiation uses captured native ordinals at large clocks and exact window boundaries", "[payload-components]") {
    sim::TerrainContactSnapshotV1 contact;contact.state.simTime=1e12;
    const std::uint64_t opening=1000000000;
    contact.outerStep=opening+4;contact.substep=contact.substeps=8;
    REQUIRE(sim::terrainContactOffsetV1(contact,opening,4,1./120)==4./120);
    contact.substep=1;REQUIRE(sim::terrainContactOffsetV1(contact,opening,4,1./120)==(3.+1./8)/120);
    contact.outerStep=opening;CHECK_FALSE(sim::terrainContactOffsetV1(contact,opening,4,1./120));
    contact.outerStep=opening+5;CHECK_FALSE(sim::terrainContactOffsetV1(contact,opening,4,1./120));
    contact.outerStep=opening+4;contact.substep=9;CHECK_FALSE(sim::terrainContactOffsetV1(contact,opening,4,1./120));
}

TEST_CASE("A selected native factory refusing creation preserves inventory history and identities", "[payload-components][native-engagement]") {
    auto registry=fsim::ms::components::builtInProvidersV1();std::string error;
    REQUIRE(registry.guidance.registerFactory("fsim.guidance.measured_pn.v2","fsim.guidance.measured_pn.v2",
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
        if (profile.find("guidance")->availability!=ms::components::AvailabilityV1::Available) {
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
    REQUIRE(full==25);REQUIRE(ballistic==28);REQUIRE(world.weapons().empty());
}

TEST_CASE("Inertial waypoint guidance freezes authorized input and cannot consume target truth", "[payload-components]") {
    using namespace fsim;
    std::vector<std::string> errors;const auto cards=ms::readStoreLibrary(FSIM_TEST_STORE_DIR,errors);REQUIRE(errors.empty());
    unsigned tested=0;
    for(const auto& [name,card]:cards) {
        ms::components::ProfileV1 profile;std::string error;
        REQUIRE(ms::components::readProfileV1(std::filesystem::path(FSIM_TEST_STORE_DIR)/(name+".json"),*card,profile,error));
        const auto* descriptor=profile.find("guidance");
        if(!descriptor || equipment::mechanism(descriptor->implementationId)!="fsim.guidance.inertial_waypoint.v1")continue;
        ++tested;ms::weapon::Spec spec;REQUIRE(ms::weapon::configureCompositionV1(*card,profile,spec,spec,error));
        ms::weapon::Observation aim{123,{6379137,10000,1000},{0,0,0},false};
        ms::components::ProviderContextV1 context{*card,profile,spec,aim};
        const auto& registry=ms::components::builtInProvidersV1();
        auto guidance=registry.guidance.create(*descriptor,context,error);REQUIRE(guidance);
        ms::weapon::Input input;input.position={6379137,0,0};input.velocity={0,200,0};input.up={1,0,0};input.dtS=1./30;
        const auto baseline=guidance->update(input);REQUIRE(baseline.phase==ms::WeaponPhase::Midcourse);
        CHECK_FALSE(baseline.tracking);CHECK_FALSE(baseline.active);CHECK(baseline.holding==0);
        input.estimate=ms::weapon::Observation{456,{NAN,NAN,NAN},{NAN,NAN,NAN},true};
        input.visible.push_back(*input.estimate);input.supported=true;input.illuminated=true;
        const auto ignored=guidance->update(input);
        CHECK(ignored.acceleration.x==baseline.acceleration.x);CHECK(ignored.acceleration.y==baseline.acceleration.y);CHECK(ignored.acceleration.z==baseline.acceleration.z);
        CHECK(std::hypot(ignored.acceleration.x,ignored.acceleration.y,ignored.acceleration.z)<=spec.maxG*9.80665);
        context.initialEstimate.reset();CHECK_FALSE(registry.guidance.create(*descriptor,context,error));CHECK_FALSE(error.empty());
        aim.velocity.y=1.;context.initialEstimate=aim;auto movingCoordinate=registry.guidance.create(*descriptor,context,error);REQUIRE(movingCoordinate);
        CHECK(movingCoordinate->update(input).acceleration.x==baseline.acceleration.x);
        input.position={6379137,0,1000};input.velocity={0,-200,0};
        const auto opposite=guidance->update(input);CHECK(opposite.phase==ms::WeaponPhase::Midcourse);CHECK(opposite.acceleration.x>9.80665);
        const double huge=std::numeric_limits<double>::max();input.velocity={huge,huge,huge};const auto overflow=guidance->update(input);
        CHECK(overflow.phase==ms::WeaponPhase::Unguided);CHECK(std::isfinite(overflow.acceleration.x));CHECK(std::isfinite(overflow.acceleration.y));CHECK(std::isfinite(overflow.acceleration.z));
    }
    REQUIRE(tested==1);
}

TEST_CASE("Liquid-fuel inertial stores fly an authorized linked coordinate without terminal tracking", "[payload-components][native-engagement]") {
    using namespace fsim;
    std::vector<std::string> errors;const auto cards=ms::readStoreLibrary(FSIM_TEST_STORE_DIR,errors);REQUIRE(errors.empty());
    unsigned tested=0;
    for(const auto& [name,card]:cards) {
        ms::components::ProfileV1 profile;std::string error;
        REQUIRE(ms::components::readProfileV1(std::filesystem::path(FSIM_TEST_STORE_DIR)/(name+".json"),*card,profile,error));
        if(equipment::mechanism(profile.find("propulsion")->implementationId)!="fsim.propulsion.jsbsim_turbofan.v1")continue;
        ++tested;INFO(name);
        session::WorldOptions options;options.name="native-linked-coordinate-cruise";options.publish=false;
        options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
        session::World world(options);
        session::VehicleSpec aircraft;aircraft.name="carrier";aircraft.type="jsbsim:h6k";
        aircraft.initial.latitudeDeg=0;aircraft.initial.longitudeDeg=0;aircraft.initial.altitudeMslM=6000;
        aircraft.initial.headingDeg=90;aircraft.initial.airspeedTrueMs=200;
        const auto carrier=world.createVehicle(aircraft);REQUIRE(carrier);
        aircraft.name="sensor donor";aircraft.type="jsbsim:f16c";
        const auto donor=world.createVehicle(aircraft);REQUIRE(donor);
        aircraft.name="observed coordinate source";aircraft.initial.latitudeDeg=.01;aircraft.initial.longitudeDeg=.18;
        aircraft.initial.headingDeg=270;
        const auto target=world.createVehicle(aircraft);REQUIRE(target);
        for(const auto id:{carrier,donor,target}) {
            control::HsaCommand hold;hold.headingRad=(id==target?270.:90.)*3.14159265358979323846/180;
            hold.speed=200;hold.speedReference=static_cast<double>(control::SpeedReference::TrueAirspeed);
            REQUIRE(world.submit(id,hold).accepted());
        }
        comm::NetSpec net;net.name="modeled targeting link";net.latencyS=.01;net.jitterS=0;net.dataRateBps=1e6;
        const auto network=world.createNet(net);REQUIRE(network);
        const auto source=world.assignSystem(donor,0x4001,"DONOR");REQUIRE(source);
        REQUIRE(world.joinNet(donor,network));REQUIRE(world.joinNet(carrier,network));
        comm::DisseminationPlan plan;plan.tracksPeriodS=.2;REQUIRE(world.setDissemination(donor,plan));
        REQUIRE(world.loadStore(carrier,0,1,name)==ms::Reason::None);
        REQUIRE(world.submit(donor,ms::StateCommand{0,ms::PayloadState::Operate}).accepted());
        REQUIRE(world.submit(carrier,ms::StateCommand{0,ms::PayloadState::Operate}).accepted());
        ms::AmtiCommand search;search.payload=0;search.azimuthHalfWidthRad=.2;search.bars=1;search.rangeScaleM=60000;
        REQUIRE(world.submit(donor,search).accepted());
        std::uint32_t designation=0;std::uint64_t observed=0;
        for(unsigned k=0;k<1200 && !designation;++k) {
            world.step();
            // Referee attribution selects the fixture's measured track; only
            // its genuinely delivered report reaches the launcher's guidance.
            for(const auto& track:world.amtiTracks(donor,0))if(track.truth==target && track.updates>=3)observed=track.id;
            if(observed)for(const auto& report:world.dms().linkTracks(carrier))
                if(report.source==source && report.report.track==observed && report.report.payload==0)
                    designation=world.designateLink(carrier,source,0,observed);
        }
        REQUIRE(designation>=session::World::kDesignations);
        REQUIRE(world.vehicleState(designation));const auto aim=*world.vehicleState(designation);
        ms::StrikeCommand strike;strike.payload=0;strike.store=name;strike.target=designation;strike.consent=true;
        REQUIRE(world.submit(carrier,strike).accepted());
        for(unsigned k=0;k<600 && world.weapons().empty();++k)world.step();
        REQUIRE(world.weapons().size()==1);const auto id=world.weapons().front().id;
        auto* model=world.model(id);REQUIRE(model);
        const auto initial=model->energy();double cgBefore[3];REQUIRE(model->cgFromOriginBodyM(cgBefore));
        REQUIRE(std::abs(initial.massKg-card->value("mass_kg",0))<.02);REQUIRE(initial.fuelKg>0);
        const auto distance=[&] {
            const auto* s=world.vehicleState(id);REQUIRE(s);
            return std::hypot(s->positionEcef[0]-aim.positionEcef[0],s->positionEcef[1]-aim.positionEcef[1],s->positionEcef[2]-aim.positionEcef[2]);
        };
        const double before=distance();
        REQUIRE(world.undesignate(carrier,designation));
        REQUIRE(world.removeVehicle(target));REQUIRE(world.removeVehicle(donor));REQUIRE(world.removeVehicle(carrier));
        double maximumThrust=0;
        for(unsigned k=0;k<900;++k) {
            world.step();REQUIRE(world.alive(id));const auto& s=*world.vehicleState(id);
            REQUIRE_FALSE(s.crashed);REQUIRE_FALSE(s.diverged);
            for(double value:s.thrustN)maximumThrust=std::max(maximumThrust,value);
            const auto record=world.weapons().front();REQUIRE_FALSE(record.tracking);REQUIRE_FALSE(record.supported);
            REQUIRE(record.holding==0);REQUIRE(record.phase==ms::WeaponPhase::Midcourse);
        }
        REQUIRE(maximumThrust>0);CHECK(distance()<before-5000);
        const auto supplied=model->energy();CHECK(supplied.fuelKg<initial.fuelKg);CHECK(supplied.massKg<initial.massKg);
        double cgAfter[3];REQUIRE(model->cgFromOriginBodyM(cgAfter));CHECK(cgAfter[0]>cgBefore[0]);
        const auto* propulsion=profile.find("propulsion");double cruise=0;
        REQUIRE(ms::components::parameterV1(*propulsion,"cruise_speed_ms","m/s",cruise,error));
        INFO("native speed="<<world.vehicleState(id)->airspeedTrueMs<<", configured="<<cruise);
        CHECK(std::abs(world.vehicleState(id)->airspeedTrueMs-cruise)<.03*cruise);
        auto tank=model->property("propulsion/tank[0]/contents-lbs");REQUIRE(tank.valid());tank.set(0);
        world.step(300);REQUIRE(world.alive(id));CHECK(model->energy().fuelKg==0);CHECK(model->energy().starved);
        CHECK(model->energy().fuelFlowKgS==0);
        for(const auto& event:world.engagementEvents())if(event.entity==id) {
            CHECK(event.kind!="acquisition");CHECK(event.kind!="activation");CHECK(event.kind!="detonation");
        }
    }
    REQUIRE(tested==1);
}

namespace {
using namespace fsim::ms::components;
struct ProviderCounts {
    std::array<std::atomic<unsigned>,9> created{},executed{};
    void made(std::size_t role) {created[role].fetch_add(1,std::memory_order_relaxed);}
    void used(std::size_t role) {executed[role].fetch_add(1,std::memory_order_relaxed);}
};
class ObservedSeeker final : public SeekerProviderV1,public SeekerExecutionV2 {
public:
    ObservedSeeker(std::unique_ptr<SeekerProviderV1> inner,std::shared_ptr<ProviderCounts> counts):inner_(std::move(inner)),counts_(std::move(counts)){}
    const fsim::ms::weapon::SeekerConfigV1& config() const noexcept override {return inner_->config();}
    double infraredRange(double a,double b,bool c) const noexcept override {counts_->used(0);return inner_->infraredRange(a,b,c);}
    double radarRange(double a,double b,bool c) const noexcept override {counts_->used(0);return inner_->radarRange(a,b,c);}
    bool notched(double a,double b) const noexcept override {counts_->used(0);return inner_->notched(a,b);}
    double decoySignal(double a,bool b) const noexcept override {counts_->used(0);return inner_->decoySignal(a,b);}
    bool illuminated(const double a[3],double b,double c,double d,double e) const noexcept override {counts_->used(0);return inner_->illuminated(a,b,c,d,e);}
    SeekerResultV2 sample(const SeekerFrameV2& f) override {
        auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get());REQUIRE(p);counts_->used(0);return p->sample(f);
    }
    void seedTrack(const SeekerTrackV2& t) override {auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get());REQUIRE(p);p->seedTrack(t);}
    void resetTracking() noexcept override {if(auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get()))p->resetTracking();}
private:
    std::unique_ptr<SeekerProviderV1> inner_;std::shared_ptr<ProviderCounts> counts_;
};
class ObservedGuidance final : public GuidanceProviderV1,public MeasuredGuidanceV2 {
public:
    ObservedGuidance(std::unique_ptr<GuidanceProviderV1> inner,std::shared_ptr<ProviderCounts> counts):inner_(std::move(inner)),counts_(std::move(counts)){}
    fsim::ms::weapon::Spec spec() const noexcept override {return inner_->spec();}
    fsim::ms::weapon::Output update(const fsim::ms::weapon::Input& input) override {counts_->used(1);return inner_->update(input);}
    fsim::ms::weapon::Output updateMeasured(const SeekerFrameV2& f,const SeekerResultV2& r) override {
        auto* p=dynamic_cast<MeasuredGuidanceV2*>(inner_.get());REQUIRE(p);counts_->used(1);return p->updateMeasured(f,r);
    }
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
    return target.registerFactory(id,std::string(fsim::equipment::mechanism(id)),[table=original,counts,role](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<Interface> {
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
    REQUIRE(registry.seeker.registerFactory(id("seeker"),std::string(equipment::mechanism(id("seeker"))),[table=defaults.seeker,counts](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<SeekerProviderV1> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(0);return std::make_unique<ObservedSeeker>(std::move(inner),counts);
    },true,error));
    REQUIRE(registry.guidance.registerFactory(id("guidance"),std::string(equipment::mechanism(id("guidance"))),[table=defaults.guidance,counts](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<GuidanceProviderV1> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(1);return std::make_unique<ObservedGuidance>(std::move(inner),counts);
    },true,error));
    REQUIRE(observeBinding<ObservedAutopilot>(registry.autopilot,defaults.autopilot,id("autopilot"),counts,2,error));
    REQUIRE(observeBinding<ObservedAllocator>(registry.allocator,defaults.allocator,id("allocator"),counts,3,error));
    REQUIRE(observeBinding<ObservedActuator>(registry.actuator,defaults.actuator,id("actuator"),counts,4,error));
    REQUIRE(observeBinding<ObservedPropulsion>(registry.propulsion,defaults.propulsion,id("propulsion"),counts,5,error));
    REQUIRE(observeBinding<ObservedAerodynamics>(registry.aerodynamics,defaults.aerodynamics,id("aerodynamics"),counts,6,error));
    REQUIRE(registry.fuze.registerFactory(id("fuze"),std::string(equipment::mechanism(id("fuze"))),[table=defaults.fuze,counts](const auto& d,const auto& ctx,auto& e) {
        auto inner=table.create(d,ctx,e);if(inner)counts->made(7);return inner;
    },true,error));
    REQUIRE(registry.warhead.registerFactory(id("warhead"),std::string(equipment::mechanism(id("warhead"))),[table=defaults.warhead,counts](const auto& d,const auto& ctx,auto& e) -> std::unique_ptr<ms::damage::WarheadAdapter> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};counts->made(8);return std::make_unique<ObservedWarhead>(std::move(inner),counts);
    },true,error));
    session::WorldOptions options;options.name="payload-provider-dispatch";options.publish=false;
    options.pinWorkers=false;options.workers=1;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World world(options);REQUIRE(world.setPayloadProvidersV1(registry,error));
    // Mutation of the caller's catalogue cannot replace this World's snapshot.
    REQUIRE(registry.guidance.registerFactory(id("guidance"),std::string(equipment::mechanism(id("guidance"))),[](const auto&,const auto&,auto&) -> std::unique_ptr<GuidanceProviderV1> {return {};},true,error));
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
