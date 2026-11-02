#include "foundation/transport/FusionSegment.h"
#include "foundation/transport/WorldMirror.h"
#include "foundation/transport/WorldPublisher.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <limits>

TEST_CASE("fusion stream V1: exact snapshots reject foreign publishers old frames and recycled vehicles", "[ipc][fusion][sim-m01]") {
    const auto name="fusion-wire-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    fsim::ipc::WorldPublisher::Options options;options.name=name;options.capacity=2;
    fsim::ipc::WorldPublisher publisher(options);REQUIRE(publisher.active());
    fsim::ipc::VehicleRecord vehicle;vehicle.id=42;vehicle.generation=1;vehicle.alive=1;publisher.setVehicle(1,vehicle);
    fsim::ipc::FusionRecordV1 record;record.vehicleId=42;record.generation=1;record.timeS=2;record.count=1;record.total=1;
    record.entities[0].id=123;record.entities[0].lastUpdateS=1.5;record.entities[0].validUntilS=13.5;
    record.entities[0].sourceCount=2;record.entities[0].sourceKinds=3;record.entities[0].stateEcef[0]=4e6;
    record.entities[0].covarianceEcef[0]=100;publisher.setFusion(1,record);
    std::vector<fsim::sim::VehicleState> states(2);states[1].simTime=2;
    REQUIRE(publisher.publish(2,fsim::Span<const fsim::sim::VehicleState>(states),{},true));
    fsim::ipc::WorldMirror mirror;REQUIRE(mirror.open(name));mirror.pollTable();fsim::sim::SnapshotBatch snapshot;
    REQUIRE(mirror.pollSnapshot(snapshot));fsim::ipc::FusionRecordV1 observed;REQUIRE(mirror.fusion(1,observed));
    CHECK(observed.entities[0].id==123);CHECK(observed.entities[0].sourceKinds==3);
    CHECK(observed.entities[0].stateEcef[0]==4e6);CHECK(observed.entities[0].covarianceEcef[0]==100);
    CHECK_FALSE(observed.entities[0].identityKnown);CHECK(observed.entities[0].lastUpdateS==1.5);
    CHECK_FALSE(mirror.fusion(0,observed));CHECK(observed.vehicleId==0);
    ++vehicle.generation;publisher.setVehicle(1,vehicle);CHECK_FALSE(mirror.fusion(1,observed));
    publisher.clearVehicle(1);CHECK_FALSE(mirror.fusion(1,observed));
    fsim::ipc::FusionSegment foreign;CHECK_FALSE(foreign.open(name,1,1));
}

TEST_CASE("fusion stream V1: loss identity covariance and truncation remain explicit", "[ipc][fusion][sim-m01]") {
    const auto name="fusion-sidecar-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    fsim::ipc::FusionSegment writer,reader;REQUIRE(writer.create(name,1,77,101));REQUIRE(reader.open(name,77,101));
    std::vector<std::unique_ptr<fsim::ipc::FusionRecordV1>> rows(1);rows[0]=std::make_unique<fsim::ipc::FusionRecordV1>();
    auto& r=*rows[0];r.vehicleId=42;r.generation=1;r.timeS=20;r.count=1;r.total=200;
    r.entities[0].id=123;r.entities[0].state=2;r.entities[0].lastUpdateS=1;r.entities[0].validUntilS=13;
    writer.publish(7,rows);fsim::ipc::FusionRecordV1 out;REQUIRE(reader.read(7,0,out));
    CHECK(out.total==200);CHECK(out.count==1);CHECK(out.entities[0].state==2);
    CHECK_FALSE(reader.read(8,0,out));CHECK(out.vehicleId==0);
    r.entities[0].covarianceEcef[0]=std::numeric_limits<double>::infinity();writer.publish(8,rows);
    CHECK_FALSE(reader.read(8,0,out));CHECK(out.vehicleId==0);
    REQUIRE(writer.create(name,1,77,102));CHECK_FALSE(reader.read(7,0,out));
}
