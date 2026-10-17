// MA lifecycle records must retain the storage borrowed by the C ABI.
#include "simulation/runtime/world/World.h"
#include "simulation/runtime/services/MaConsentLedger.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

TEST_CASE("MA consent ledger: first completion retires all approvers and duplicate answers", "[ma][lifecycle][ledger]") {
    fsim::session::MaConsentLedger ledger;
    ledger.record({100, 7, {11, 12}});
    REQUIRE(ledger.size() == 1);
    const auto* request = ledger.findRequest(7);
    REQUIRE(request != nullptr);
    CHECK(request == ledger.findStrike(100));
    CHECK(request->strike == 100);
    REQUIRE(request->asked.size() == 2);
    CHECK(request->asked[0] == 11);
    CHECK(request->asked[1] == 12);
    REQUIRE(ledger.complete(7));
    CHECK(ledger.findRequest(7) == nullptr);
    CHECK(ledger.findStrike(100) == nullptr);
    CHECK_FALSE(ledger.complete(7));
    CHECK(ledger.size() == 0);
}

TEST_CASE("MA consent ledger: unresolved requests survive unrelated answers and automatic policies", "[ma][lifecycle][ledger]") {
    fsim::session::MaConsentLedger ledger;
    ledger.record({100, 7, {11, 12}});
    ledger.record({200, 8, {11}});
    ledger.record({300, 0, {}}); // An automatic policy has no approving sender to authorize.
    REQUIRE(ledger.size() == 2);
    CHECK(ledger.findStrike(300) == nullptr);
    REQUIRE(ledger.complete(8));
    const auto* unanswered = ledger.findRequest(7);
    REQUIRE(unanswered != nullptr);
    CHECK(unanswered->strike == 100);
    CHECK(unanswered->asked.size() == 2);
    CHECK_FALSE(ledger.complete(99));
    CHECK(ledger.size() == 1);
    ledger.record({100, 9, {12}});
    CHECK(ledger.size() == 1);
    CHECK(ledger.findRequest(7) == nullptr);
    REQUIRE(ledger.findRequest(9) != nullptr);
    CHECK(ledger.findRequest(9)->asked[0] == 12);
}

TEST_CASE("MA consent ledger: answered churn does not accumulate and retirement releases capacity", "[ma][lifecycle][ledger]") {
    fsim::session::MaConsentLedger ledger;
    for (std::uint64_t id = 1; id <= 512; ++id) {
        ledger.record({id, id, {77}});
        REQUIRE(ledger.complete(id));
        CHECK(ledger.size() == 0);
        CHECK(ledger.findRequest(id) == nullptr);
    }
    ledger.record({513, 513, {77, 78}});
    REQUIRE(ledger.capacity() > 0);
    ledger.release();
    CHECK(ledger.size() == 0);
    CHECK(ledger.capacity() == 0);
    CHECK(ledger.findRequest(513) == nullptr);
    ledger.release();
    ledger.record({514, 514, {79}});
    REQUIRE(ledger.findStrike(514) != nullptr);
    CHECK(ledger.findStrike(514)->asked[0] == 79);
}

TEST_CASE("MA consent ledger: repeated retirement releases unresolved storage and permits reuse", "[ma][lifecycle][ledger]") {
    fsim::session::MaConsentLedger ledger;
    for (std::uint64_t id = 1; id <= 512; ++id) {
        ledger.record({id, id, {77, 78}});
        REQUIRE(ledger.findRequest(id) != nullptr);
        REQUIRE(ledger.capacity() > 0);
        ledger.release();
        CHECK(ledger.size() == 0);
        CHECK(ledger.capacity() == 0);
        CHECK(ledger.findRequest(id) == nullptr);
    }
}

TEST_CASE("MA lifecycle: retirement of an unknown entity does not create query records", "[ma][lifecycle]") {
    fsim::session::MaCore ma;
    REQUIRE(ma.authority(1234) == nullptr);
    ma.retireEntity(1234);
    CHECK(ma.authority(1234) == nullptr);
    CHECK(ma.tasks(1234) == nullptr);
    CHECK(ma.overrides(1234) == nullptr);
}

TEST_CASE("MA lifecycle: removed systems retain borrowed RBAC storage", "[ma][lifecycle]") {
    fsim::session::WorldOptions options;
    options.name = "ma-rbac-retention";
    options.publish = false;
    options.workers = 1;
    options.pinWorkers = false;
    fsim::session::World world(options);
    const auto entity = world.createC2Node({});
    auto config = fsim::comm::RbacConfig::agraDefaults();
    config.roles[1].name = "Retained approving operator role";
    world.maCore().setRbac(entity, config);
    const auto& borrowed = world.maCore().rbacOf(entity);
    const char* roleName = borrowed.roles[1].name.c_str();

    REQUIRE(world.removeC2Node(entity));
    world.step(3);
    CHECK(&world.maCore().rbacOf(entity) == &borrowed);
    CHECK(world.maCore().rbacOf(entity).roles[1].name.c_str() == roleName);
    CHECK(std::string(roleName) == "Retained approving operator role");
}
