#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/components/interfaces/MeasuredGuidanceV2.h"
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace {
using namespace fsim;
using namespace fsim::ms::components;
struct Counts { unsigned sensor=0,guidance=0,created=0; };
class ObservedSensor final : public SeekerProviderV1,public SeekerExecutionV2 {
public:
    ObservedSensor(std::unique_ptr<SeekerProviderV1> p,std::shared_ptr<Counts> c):inner_(std::move(p)),counts_(std::move(c)) {++counts_->created;}
    const ms::weapon::SeekerConfigV1& config() const noexcept override {return inner_->config();}
    double infraredRange(double a,double b,bool c) const noexcept override {return inner_->infraredRange(a,b,c);}
    double radarRange(double a,double b,bool c) const noexcept override {return inner_->radarRange(a,b,c);}
    bool notched(double a,double b) const noexcept override {return inner_->notched(a,b);}
    double decoySignal(double a,bool b) const noexcept override {return inner_->decoySignal(a,b);}
    bool illuminated(const double a[3],double b,double c,double d,double e) const noexcept override {return inner_->illuminated(a,b,c,d,e);}
    SeekerResultV2 sample(const SeekerFrameV2& f) override {
        if(config().mode!=ms::weapon::SeekerModeV1::ActiveRadar)CHECK_FALSE(f.launcherEstimate);
        ++counts_->sensor;return execution().sample(f);
    }
    void seedTrack(const SeekerTrackV2& t) override {execution().seedTrack(t);}
    void resetTracking() noexcept override {if(auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get()))p->resetTracking();}
private:
    SeekerExecutionV2& execution() {auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get());REQUIRE(p);return *p;}
    std::unique_ptr<SeekerProviderV1> inner_;std::shared_ptr<Counts> counts_;
};
class ObservedController final : public GuidanceProviderV1,public MeasuredGuidanceV2 {
public:
    ObservedController(std::unique_ptr<GuidanceProviderV1> p,std::shared_ptr<Counts> c):inner_(std::move(p)),counts_(std::move(c)) {}
    ms::weapon::Spec spec() const noexcept override {return inner_->spec();}
    ms::weapon::Output update(const ms::weapon::Input& f) override {return inner_->update(f);}
    ms::weapon::Output updateMeasured(const SeekerFrameV2& f,const SeekerResultV2& r) override {
        ++counts_->guidance;auto* p=dynamic_cast<MeasuredGuidanceV2*>(inner_.get());REQUIRE(p);return p->updateMeasured(f,r);
    }
private:
    std::unique_ptr<GuidanceProviderV1> inner_;std::shared_ptr<Counts> counts_;
};
ProviderRegistryV1 observed(const char* seeker,std::shared_ptr<Counts> c) {
    const auto defaults=builtInProvidersV1();auto registry=defaults;std::string error;
    REQUIRE(registry.seeker.registerFactory(seeker,seeker,[table=defaults.seeker,c](const auto& d,const auto& ctx,auto& e)->std::unique_ptr<SeekerProviderV1> {
        auto inner=table.create(d,ctx,e);if(!inner)return {};return std::make_unique<ObservedSensor>(std::move(inner),c);
    },true,error));
    REQUIRE(registry.guidance.registerFactory("fsim.guidance.measured_pn.v2","fsim.guidance.measured_pn.v2",
        [table=defaults.guidance,c](const auto& d,const auto& ctx,auto& e)->std::unique_ptr<GuidanceProviderV1> {
            auto inner=table.create(d,ctx,e);if(!inner)return {};return std::make_unique<ObservedController>(std::move(inner),c);
        },true,error));
    return registry;
}
session::WorldOptions options(const std::string& label="on-rail") {
    session::WorldOptions o;o.name="representative-weapon-native-"+label;o.publish=false;o.workers=1;o.pinWorkers=false;
    if(const auto* directory=std::getenv("FSIM_REPRESENTATIVE_RECORD_DIR"))
        o.recordPath=std::filesystem::path(directory)/(label+".fsrec");
    o.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;return o;
}
std::uint16_t stations(session::World& world,std::uint32_t own) {
    for(const auto& p:world.missionSystems(own)->payloads())if(p.family==ms::Family::StoreStation)return p.index;
    FAIL("Store station payload missing");return 0;
}
session::VehicleSpec carrier(const char* name) {
    session::VehicleSpec s;s.name=name;s.type="jsbsim:f16c";
    s.initial.latitudeDeg=36.25;s.initial.longitudeDeg=-115.03;s.initial.altitudeMslM=6000;
    s.initial.airspeedTrueMs=250;s.initial.headingDeg=90;return s;
}
}

TEST_CASE("All four selected weapon models execute measured providers after actual native release", "[representative-weapons][native-engagement]") {
    const std::pair<const char*,const char*> models[]={
        {"aim-9m","fsim.seeker.nonimaging_ir.v2"},{"aim-9x","fsim.seeker.imaging_ir.v2"},
        {"aim-7m","fsim.seeker.semi_active_radar.v2"},{"aim-120c","fsim.seeker.active_radar.v2"}};
    std::ofstream trace;
    if(const auto* directory=std::getenv("FSIM_REPRESENTATIVE_RECORD_DIR"))
        trace.open(std::filesystem::path(directory)/"summary.jsonl");
    for(const auto& [store,seeker]:models) {
        INFO(store);auto counts=std::make_shared<Counts>();session::World world(options(store));std::string error;
        REQUIRE(world.setPayloadProvidersV1(observed(seeker,counts),error));
        auto spec=carrier("launcher");if(std::string(store)=="aim-7m")spec.type="jsbsim:fa18c";
        const auto own=world.createVehicle(spec);REQUIRE(own);
        const auto payload=stations(world,own);
        REQUIRE(world.loadStore(own,payload,std::string(store)=="aim-7m"?4:1,store)==ms::Reason::None);
        REQUIRE(world.submit(own,ms::StateCommand{payload,ms::PayloadState::Operate}).accepted());
        for(unsigned i=0;i<1000&&world.missionSystems(own)->payloads()[payload].state!=ms::PayloadState::Operate;++i)world.step();
        REQUIRE(world.missionSystems(own)->payloads()[payload].state==ms::PayloadState::Operate);
        const auto* ownState=world.vehicleState(own);REQUIRE(ownState);
        spec=carrier("target");spec.initial.latitudeDeg=ownState->latitudeRad*180/3.141592653589793;
        spec.initial.longitudeDeg=ownState->longitudeRad*180/3.141592653589793+.055;
        const auto target=world.createVehicle(spec);REQUIRE(target);
        ms::StrikeCommand strike;strike.payload=payload;strike.store=store;strike.consent=true;
        REQUIRE(world.submit(own,strike).accepted());
        for(unsigned i=0;i<150&&world.weapons().empty();++i)world.step();
        REQUIRE(world.weapons().size()==1);const auto missile=world.weapons().front().id;
        world.step(4);
        CHECK(counts->sensor>0);CHECK(counts->guidance>0);
        const auto records=world.weapons();REQUIRE(records.size()==1);
        CHECK(records.front().id==missile);CHECK_FALSE(records.front().jettisoned);
        if(std::string(store)=="aim-7m")CHECK_FALSE(records.front().tracking);
        else CHECK(records.front().tracking);
        if(trace)trace<<"{\"store\":"<<std::quoted(store)<<",\"seeker\":"<<std::quoted(seeker)
            <<",\"tracking\":"<<(records.front().tracking?"true":"false")
            <<",\"sensor_calls\":"<<counts->sensor<<",\"guidance_calls\":"<<counts->guidance<<"}\n";
    }
    if(const auto* directory=std::getenv("FSIM_REPRESENTATIVE_RECORD_DIR"))for(const auto& [store,seeker]:models) {
        (void)seeker;ipc::Recording recording;std::string error;
        REQUIRE(recording.load(std::filesystem::path(directory)/(std::string(store)+".fsrec"),&error));
        REQUIRE_FALSE(recording.frames().empty());
        CHECK(std::any_of(recording.frames().begin(),recording.frames().end(),[](const auto& f){return !f.weaponControls.empty();}));
    }
}

TEST_CASE("On-rail queries retain one sampled seeker and reset invalidates its history", "[representative-weapons][native-engagement]") {
    auto counts=std::make_shared<Counts>();session::World world(options());std::string error;
    REQUIRE(world.setPayloadProvidersV1(observed("fsim.seeker.imaging_ir.v2",counts),error));
    auto spec=carrier("launcher");const auto own=world.createVehicle(spec);REQUIRE(own);
    const auto payload=stations(world,own);REQUIRE(world.loadStore(own,payload,1,"aim-9x")==ms::Reason::None);
    spec=carrier("target");spec.initial.longitudeDeg+=.055;const auto target=world.createVehicle(spec);REQUIRE(target);
    REQUIRE(world.seekerTarget(own,"aim-9x")==target);
    const auto sampled=counts->sensor,created=counts->created;CHECK(sampled>0);
    for(int i=0;i<10;++i)CHECK(world.seekerTarget(own,"aim-9x")==target);
    CHECK(counts->sensor==sampled);CHECK(counts->created==created);
    REQUIRE(world.resetVehicle(own));CHECK(world.seekerTarget(own,"aim-9x")==target);
    CHECK(counts->sensor>sampled);CHECK(counts->created>created);
}

TEST_CASE("Native AIM-7M tracking follows actual illumination interruption and restoration", "[representative-weapons][native-engagement]") {
    session::World world(options("sarh-support"));auto spec=carrier("illuminator");spec.type="jsbsim:fa18c";
    auto counts=std::make_shared<Counts>();std::string error;
    REQUIRE(world.setPayloadProvidersV1(observed("fsim.seeker.semi_active_radar.v2",counts),error));
    const auto own=world.createVehicle(spec);REQUIRE(own);const auto station=stations(world,own);
    std::optional<std::uint16_t> radar;
    for(const auto& p:world.missionSystems(own)->payloads())if(p.family==ms::Family::AirRadar)radar=p.index;
    REQUIRE(radar);REQUIRE(world.loadStore(own,station,4,"aim-7m")==ms::Reason::None);
    for(const auto payload:{station,*radar}) {
        const auto command=world.submit(own,ms::StateCommand{payload,ms::PayloadState::Operate});
        REQUIRE((command.accepted()||command.reason==ms::Reason::AlreadyInState));
        for(unsigned i=0;i<1200&&world.missionSystems(own)->payloads()[payload].state!=ms::PayloadState::Operate;++i)world.step();
        REQUIRE(world.missionSystems(own)->payloads()[payload].state==ms::PayloadState::Operate);
    }
    const auto* state=world.vehicleState(own);REQUIRE(state);
    spec=carrier("opponent");spec.initial.latitudeDeg=state->latitudeRad*180/3.141592653589793;
    spec.initial.longitudeDeg=state->longitudeRad*180/3.141592653589793+.18;spec.initial.headingDeg=270;
    const auto target=world.createVehicle(spec);REQUIRE(target);
    ms::AmtiCommand scan;scan.payload=*radar;scan.azimuthHalfWidthRad=5*3.141592653589793/180;
    scan.bars=1;scan.rangeScaleM=60000;REQUIRE(world.submit(own,scan).accepted());
    std::uint64_t track=0;
    for(unsigned i=0;i<300&&!track;++i) {
        world.step();for(const auto& t:world.amtiTracks(own,*radar))if(t.truth==target&&t.updates>=3)track=t.id;
    }
    REQUIRE(track);const auto designation=world.designate(own,*radar,track);REQUIRE(designation>=session::World::kDesignations);
    ms::AmtiCommand stt;stt.payload=*radar;stt.task=ms::AmtiTask::Track;
    stt.mode=ms::AmtiMode::SingleTargetTrack;stt.track=track;stt.rangeScaleM=60000;stt.illuminate=true;
    REQUIRE(world.submit(own,stt).accepted());world.step(3);
    ms::StrikeCommand strike;strike.payload=station;strike.store="aim-7m";strike.target=designation;strike.consent=true;
    REQUIRE(world.submit(own,strike).accepted());
    for(unsigned i=0;i<150&&world.weapons().empty();++i)world.step();
    REQUIRE(world.weapons().size()==1);world.step(2);CHECK(world.weapons().front().tracking);
    stt.illuminate=false;REQUIRE(world.submit(own,stt).accepted());world.step(2);CHECK_FALSE(world.weapons().front().tracking);
    stt.illuminate=true;REQUIRE(world.submit(own,stt).accepted());world.step(2);CHECK(world.weapons().front().tracking);
}
