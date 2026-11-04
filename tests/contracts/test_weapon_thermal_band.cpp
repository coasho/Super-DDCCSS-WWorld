#include "foundation/transport/WeaponDiagnosticsV1.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>

#if __has_include("foundation/transport/WeaponThermalBandV1.h")
#include "foundation/transport/WeaponThermalBandV1.h"
namespace {
std::string bandWorld(){return "thermal-band-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());}
}
TEST_CASE("Thermal display selection binds channel publisher entity generation and completed frame", "[weapon-thermal-band][ipc]") {
    using namespace fsim::ipc;
    const auto name=bandWorld();const auto pid=fsim::platform::currentProcessId();
    WeaponThermalBandV1 owner,reader,foreign;
    REQUIRE(owner.create(name,pid,7));REQUIRE(reader.open(name,pid,7));CHECK_FALSE(foreign.open(name,pid,8));
    WeaponThermalBandV1 duplicate;CHECK_FALSE(duplicate.create(name,pid,7));
    REQUIRE(owner.publish(100,42,9,2));
    WeaponThermalBandAvailabilityV1 available;REQUIRE(reader.availability(100,available));
    CHECK(available.worldFrame==100);CHECK(available.entity==42);CHECK(available.generation==9);CHECK(available.bandCount==2);
    REQUIRE(reader.request(100,42,9,1));unsigned selected=99;
    REQUIRE(owner.selection(100,42,9,selected));CHECK(selected==1);
    CHECK_FALSE(reader.request(101,42,9,0));
    REQUIRE(owner.selection(100,42,9,selected));CHECK(selected==1);
    CHECK_FALSE(owner.selection(99,42,9,selected));CHECK_FALSE(owner.selection(100,42,10,selected));
    CHECK_FALSE(owner.selection(100,43,9,selected));CHECK_FALSE(owner.selection(401,42,9,selected));
    CHECK_FALSE(reader.request(100,42,9,2));CHECK_FALSE(reader.request(100,42,0,0));CHECK_FALSE(owner.request(100,42,9,0));
    REQUIRE(owner.selection(100,42,9,selected));CHECK(selected==1);
    REQUIRE(reader.availability(100,available));CHECK(available.bandCount==2);
    owner.close();CHECK_FALSE(reader.valid());CHECK_FALSE(reader.availability(100,available));
}
TEST_CASE("Thermal display availability and selections reject malformed or stale updates atomically", "[weapon-thermal-band][ipc]") {
    using namespace fsim::ipc;
    const auto name=bandWorld();const auto pid=fsim::platform::currentProcessId();
    WeaponThermalBandV1 owner,reader;REQUIRE(owner.create(name,pid,3));REQUIRE(reader.open(name,pid,3));
    REQUIRE(owner.publish(10,4,5,2));REQUIRE(reader.request(10,4,5,1));
    CHECK_FALSE(owner.publish(9,4,5,2));CHECK_FALSE(owner.publish(11,4,0,2));CHECK_FALSE(owner.publish(11,4,5,3));
    WeaponThermalBandAvailabilityV1 available;REQUIRE(reader.availability(10,available));CHECK(available.bandCount==2);
    REQUIRE(owner.publish(11,4,5,1));unsigned selected=99;CHECK_FALSE(owner.selection(11,4,5,selected));CHECK(selected==99);
    REQUIRE(reader.request(11,4,5,0));REQUIRE(owner.selection(11,4,5,selected));CHECK(selected==0);
    CHECK_FALSE(reader.availability(10,available));CHECK_FALSE(reader.availability(312,available));
    REQUIRE(owner.publish(12,0,0,0));REQUIRE(reader.availability(12,available));CHECK(available.bandCount==0);
    CHECK_FALSE(owner.selection(12,4,5,selected));
}
#else
TEST_CASE("Both native thermal bands require an additive display selection contract", "[weapon-thermal-band][ipc]") {
    FAIL("The normal native diagnostic window cannot yet select both executing thermal channels without changing frozen V1 frames");
}
#endif
