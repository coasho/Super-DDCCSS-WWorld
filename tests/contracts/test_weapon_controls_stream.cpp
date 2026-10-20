#include "foundation/transport/WeaponControlsSegment.h"
#include "foundation/transport/Recording.h"
#include "foundation/transport/WorldPublisher.h"
#include "foundation/transport/WorldMirror.h"
#include "foundation/core/Log.h"
#include "simulation/runtime/world/World.h"
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <chrono>

TEST_CASE("unsupported weapon fin queries and publication are quiet and explicitly unavailable", "[ipc][weapon_controls][quiet_optional]") {
    fsim::session::WorldOptions options;
    options.name="fin-unsupported"; options.publish=true; options.terrain=false;
    options.workers=1; options.pinWorkers=false; options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    fsim::session::World world(options);
    fsim::session::VehicleSpec spec;
    spec.name="carrier"; spec.type="jsbsim:f16c";
    spec.initial.latitudeDeg=40; spec.initial.altitudeMslM=6000; spec.initial.airspeedTrueMs=250;
    const auto carrier=world.createVehicle(spec); REQUIRE(carrier!=0);
    fsim::ms::WeaponLaunch launch; launch.type="jsbsim:mk82";
    const auto bomb=world.launchWeapon(carrier,launch); REQUIRE(bomb!=0);
    unsigned warnings=0;
    const auto previous=fsim::log::level();
    struct Restore {
        fsim::log::Level previous;
        ~Restore() { fsim::log::setSink({}); fsim::log::setLevel(previous); }
    } restore{previous};
    fsim::log::setLevel(fsim::log::Level::Warn);
    fsim::log::setSink([&](const fsim::log::Record& record) {
        if(record.message.find("missile-control")!=std::string_view::npos) ++warnings;
    });
    for(unsigned i=0;i<20;++i) {
        const auto sample=world.weaponControlsV1(bomb); REQUIRE(sample);
        CHECK_FALSE(sample->available); CHECK(sample->fins.empty());
        world.publishNow();
    }
    CHECK(warnings==0);
}

TEST_CASE("fin sidecar rejects wrong incarnations sequences and clears removed entities", "[ipc][weapon_controls]") {
    const auto suffix=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const std::string name="fin-test-"+suffix;
    fsim::ipc::WeaponControlsSegment publisher,reader,foreign;
    REQUIRE(publisher.create(name,2,77,101));
    REQUIRE(reader.open(name,77,101));
    CHECK_FALSE(foreign.open(name,77,102));
    CHECK_FALSE(foreign.open(name,78,101));
    fsim::ms::WeaponControlSnapshotV1 sample;
    sample.entity=42; sample.generation=8; sample.slot=1; sample.available=true;
    sample.sampleTimeS=3; sample.controllerTimeS=2.99; sample.controllerAvailable=true;
    fsim::ms::WeaponFinStateV1 fin;
    fin.id=1234; fin.valid=true; fin.actualRad=.12; fin.demandRad=.2;
    fin.minimumRad=-.3; fin.maximumRad=.3; fin.rateRadS=4; fin.failedStuck=true;
    sample.fins.push_back(fin);
    REQUIRE(publisher.publish(11,{sample}));
    std::vector<fsim::ms::WeaponControlSnapshotV1> out;
    REQUIRE(reader.read(11,out)); REQUIRE(out.size()==1);
    CHECK(out[0].entity==42); CHECK(out[0].generation==8);
    CHECK(out[0].fins[0].id==1234); CHECK(out[0].fins[0].actualRad==.12);
    CHECK(out[0].fins[0].failedStuck);
    CHECK_FALSE(reader.read(12,out)); CHECK(out.empty());
    REQUIRE(publisher.publish(12,{})); REQUIRE(reader.read(12,out)); CHECK(out.empty());
    REQUIRE(publisher.create(name,2,77,102));
    CHECK_FALSE(reader.read(11,out)); CHECK(out.empty());
}

TEST_CASE("recorded fin snapshots retain jam state without altering base sample stride", "[recording][weapon_controls]") {
    const auto path=std::filesystem::temp_directory_path()/
        ("fin-record-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".fsrec");
    fsim::ipc::RecordingHeader header; header.capacity=1; header.dt=1./120;
    fsim::ipc::RecordWriter writer; REQUIRE(writer.open(path,header));
    fsim::ms::WeaponControlSnapshotV1 sample;
    sample.entity=42; sample.generation=9; sample.available=true; sample.sampleTimeS=1;
    fsim::ms::WeaponFinStateV1 fin; fin.id=1234; fin.valid=true; fin.actualRad=-.07; fin.demandRad=0;
    fin.minimumRad=-.3; fin.maximumRad=.3; fin.rateRadS=4; fin.failedStuck=true;
    sample.fins.push_back(fin);
    writer.writeWeaponControls(1,{sample});
    fsim::ipc::VehicleSample base; base.state.simTime=1;
    writer.writeSnapshot(1,{{0,base}});
    writer.writeWeaponControls(2,{}); writer.writeSnapshot(2,{{0,base}});
    sample.sampleTimeS=3;
    writer.writeWeaponControls(3,{sample}); writer.writeSnapshot(4,{{0,base}});
    writer.close();
    fsim::ipc::Recording recording; REQUIRE(recording.load(path));
    REQUIRE(recording.frames().size()==3);
    REQUIRE(recording.frames()[0].weaponControls.size()==1);
    CHECK(recording.frames()[0].weaponControls[0].fins[0].actualRad==-.07);
    CHECK(recording.frames()[0].weaponControls[0].fins[0].failedStuck);
    CHECK(recording.frames()[1].weaponControls.empty());
    CHECK(recording.frames()[2].weaponControls.empty()); // a mismatched extension never carries into another frame
    CHECK(recording.frames()[0].samples[0].second.state.simTime==1);
    std::error_code error; std::filesystem::remove(path,error);
}

TEST_CASE("fin wire refuses unknown flags duplicate identity and invalid available data", "[ipc][weapon_controls]") {
    fsim::ms::WeaponControlSnapshotV1 sample;
    sample.entity=42; sample.generation=8; sample.sampleTimeS=3; sample.available=true;
    fsim::ms::WeaponFinStateV1 fin;
    fin.id=1234; fin.valid=true; fin.actualRad=.12; fin.demandRad=.2;
    fin.minimumRad=-.3; fin.maximumRad=.3; fin.rateRadS=4;
    sample.fins.push_back(fin);
    fsim::ipc::WeaponControlsWireV1 wire;
    REQUIRE(fsim::ipc::encodeWeaponControls(sample,wire));
    fsim::ms::WeaponControlSnapshotV1 decoded;
    auto invalid=wire; invalid.flags|=4;
    CHECK_FALSE(fsim::ipc::decodeWeaponControls(invalid,decoded));
    invalid=wire; invalid.fins[0].flags|=16;
    CHECK_FALSE(fsim::ipc::decodeWeaponControls(invalid,decoded));
    invalid=wire; invalid.fins[0].reserved=1;
    CHECK_FALSE(fsim::ipc::decodeWeaponControls(invalid,decoded));
    invalid=wire; invalid.count=2; invalid.fins[1]=invalid.fins[0];
    CHECK_FALSE(fsim::ipc::decodeWeaponControls(invalid,decoded));
    sample.fins[0].rateRadS=0;
    CHECK_FALSE(fsim::ipc::encodeWeaponControls(sample,wire));
}

TEST_CASE("world mirror accepts actual fins only for the same base entity generation and frame", "[ipc][weapon_controls]") {
    const auto suffix=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    fsim::ipc::WorldPublisher::Options options;
    options.name="fin-mirror-"+suffix; options.capacity=2;
    fsim::ipc::WorldPublisher publisher(options);
    REQUIRE(publisher.active());
    fsim::ipc::VehicleRecord row;
    row.id=42; row.generation=8; row.alive=1;
    publisher.setVehicle(1,row);
    fsim::ipc::WorldMirror mirror; REQUIRE(mirror.open(options.name)); REQUIRE(mirror.pollTable());
    std::vector<fsim::sim::VehicleState> states(2);
    std::vector<fsim::sim::ControlInputs> inputs(2);
    fsim::ms::WeaponControlSnapshotV1 control;
    control.entity=42; control.generation=9; control.slot=1; control.sampleTimeS=3;
    fsim::ms::WeaponFinStateV1 fin; fin.id=1234; control.fins.push_back(fin);
    fsim::sim::SnapshotBatch batch;
    REQUIRE(publisher.publish(3,states,inputs,true,{}, {control}));
    REQUIRE(mirror.pollSnapshot(batch)); CHECK(mirror.weaponControls().empty());
    control.generation=8; control.sampleTimeS=4;
    REQUIRE(publisher.publish(4,states,inputs,true,{}, {control}));
    REQUIRE(mirror.pollSnapshot(batch)); REQUIRE(mirror.weaponControls().size()==1);
    CHECK(mirror.weaponControls()[0].fins[0].id==1234);
    control.sampleTimeS=4; // base frame advances while a stale extension remains
    REQUIRE(publisher.publish(5,states,inputs,true,{}, {control}));
    REQUIRE(mirror.pollSnapshot(batch)); CHECK(mirror.weaponControls().empty());
    publisher.clearVehicle(1); REQUIRE(mirror.pollTable()); control.sampleTimeS=6;
    REQUIRE(publisher.publish(6,states,inputs,true,{}, {control}));
    REQUIRE(mirror.pollSnapshot(batch)); CHECK(mirror.weaponControls().empty());
}
