// Pure synthetic geometry/timing cases; no equipment fuze is calibrated here.
#include "simulation/mission_systems/weapon/ContactFuze.h"
#include "simulation/mission_systems/damage/State.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <memory>

namespace {
namespace dmg = fsim::ms::damage;
namespace weapon = fsim::ms::weapon;

weapon::ContactWindow window() { return {0, 1, 0, 10, 1, false}; }

dmg::Airframe boxedFrame() {
    dmg::Airframe frame;
    dmg::Airframe::Panel box;
    box.surface = false;
    box.half = {1, 1, 1};
    box.areaM2 = 24;
    frame.panels.push_back(box);
    return frame;
}

dmg::Airframe::Panel face(double x, double y0 = -1, double y1 = 1) {
    dmg::Airframe::Panel panel;
    panel.corners = {dmg::V3{x, y0, -1}, dmg::V3{x, y1, -1}, dmg::V3{x, y1, 1}, dmg::V3{x, y0, 1}};
    panel.areaM2 = 2 * (y1 - y0);
    return panel;
}

weapon::ContactFuzeResult sweep(const dmg::Airframe& frame, const std::vector<weapon::ContactSample>& path,
                                weapon::ContactWindow timing = window()) {
    return weapon::contactFuze(path, dmg::AttachedGeometryView(frame), timing);
}

bool near(double a, double b) { return std::abs(a - b) < 1e-12; }
} // namespace

TEST_CASE("contact fuze uses first box entry rather than exit or closest approach", "[contact-fuze]") {
    const auto hit = sweep(boxedFrame(), {{0, {-2, 0, 0}}, {1, {2, 0, 0}}});
    REQUIRE(hit.valid);
    REQUIRE(hit.contact);
    CHECK(hit.triggered);
    CHECK_FALSE(hit.initialOverlap);
    CHECK(near(hit.offsetS, .25));
    CHECK(near(hit.fraction, .25));
    CHECK(hit.kind == weapon::ContactKind::Panel);
    CHECK(hit.index == 0);
}

TEST_CASE("contact fuze sweeps finite panel faces and their real gaps", "[contact-fuze]") {
    dmg::Airframe frame;
    frame.panels = {face(0, -2, -1), face(0, 1, 2)};
    const auto gap = sweep(frame, {{0, {-2, 0, 0}}, {1, {2, 0, 0}}});
    REQUIRE(gap.valid);
    CHECK_FALSE(gap.contact);
    CHECK_FALSE(gap.triggered);
    const auto panel = sweep(frame, {{0, {-2, 1.5, 0}}, {1, {2, 1.5, 0}}});
    REQUIRE(panel.contact);
    CHECK(panel.index == 1);
    CHECK(near(panel.fraction, .5));
    const auto edge = sweep(frame, {{0, {-2, 1, 0}}, {1, {2, 1, 0}}});
    CHECK(edge.contact);
    CHECK(edge.triggered);
}

TEST_CASE("contact fuze handles coplanar face entry without a radius", "[contact-fuze]") {
    dmg::Airframe frame;
    frame.panels.push_back(face(0));
    const auto entry = sweep(frame, {{0, {0, -2, 0}}, {1, {0, 2, 0}}});
    REQUIRE(entry.contact);
    CHECK(near(entry.fraction, .25));
    const auto offset = sweep(frame, {{0, {1e-6, -2, 0}}, {1, {1e-6, 2, 0}}});
    REQUIRE(offset.valid);
    CHECK_FALSE(offset.contact);
}

TEST_CASE("contact fuze preserves native piecewise paths instead of outer chords", "[contact-fuze]") {
    const auto frame = boxedFrame();
    const auto avoided = sweep(frame, {{0, {-2, 0, 0}}, {.25, {-2, 2, 0}}, {.75, {2, 2, 0}}, {1, {2, 0, 0}}});
    REQUIRE(avoided.valid);
    CHECK_FALSE(avoided.contact);
    const auto crossing = sweep(frame, {{0, {-2, 2, 0}}, {.5, {0, 0, 0}}, {1, {2, 2, 0}}});
    REQUIRE(crossing.contact);
    CHECK(near(crossing.fraction, .25));
}

TEST_CASE("contact fuze retains the first unarmed crossing without arming on its exit", "[contact-fuze]") {
    auto timing = window();
    timing.armingS = .5;
    const auto hit = sweep(boxedFrame(), {{0, {-2, 0, 0}}, {1, {2, 0, 0}}}, timing);
    REQUIRE(hit.valid);
    REQUIRE(hit.contact);
    CHECK_FALSE(hit.triggered);
    CHECK(near(hit.fraction, .25));
    timing.armingS = .25;
    CHECK(sweep(boxedFrame(), {{0, {-2, 0, 0}}, {1, {2, 0, 0}}}, timing).triggered);
}

TEST_CASE("contact fuze excludes exact life and native arrest boundaries", "[contact-fuze]") {
    const auto frame = boxedFrame();
    const std::vector<weapon::ContactSample> path{{0, {-2, 0, 0}}, {1, {2, 0, 0}}};
    auto timing = window();
    timing.lifeS = .25;
    const auto expiry = sweep(frame, path, timing);
    REQUIRE(expiry.valid);
    CHECK_FALSE(expiry.contact);
    CHECK_FALSE(expiry.triggered);
    timing = window();
    timing.cutoffS = .25;
    timing.exclusiveCutoff = true; // Native terrain contact or divergence.
    const auto stopped = sweep(frame, path, timing);
    REQUIRE(stopped.valid);
    CHECK_FALSE(stopped.contact);
    timing.exclusiveCutoff = false;
    CHECK(sweep(frame, path, timing).triggered);
    timing.cutoffS = std::nextafter(.25, 1.);
    timing.exclusiveCutoff = true;
    CHECK(sweep(frame, path, timing).triggered);
}

TEST_CASE("contact fuze initial overlap never manufactures a new armed encounter", "[contact-fuze]") {
    auto timing = window();
    timing.armingS = .5;
    const auto inside = sweep(boxedFrame(), {{0, {0, 0, 0}}, {1, {2, 0, 0}}}, timing);
    REQUIRE(inside.valid);
    CHECK(inside.initialOverlap);
    CHECK_FALSE(inside.contact);
    CHECK_FALSE(inside.triggered);
    const auto boundary = sweep(boxedFrame(), {{0, {-1, 0, 0}}, {1, {2, 0, 0}}});
    CHECK(boundary.initialOverlap);
    CHECK_FALSE(boundary.triggered);
    dmg::Airframe panel;
    panel.panels.push_back(face(0));
    CHECK(sweep(panel, {{0, {0, 0, 0}}, {1, {2, 0, 0}}}).initialOverlap);
}

TEST_CASE("contact fuze never bridges an unavailable sample or extrapolates a short history", "[contact-fuze]") {
    const auto frame = boxedFrame();
    const auto gap = sweep(frame, {{0, {-2, 0, 0}}, {.5, {}, false}, {1, {2, 0, 0}}});
    CHECK_FALSE(gap.valid);
    CHECK_FALSE(gap.triggered);
    const auto shortPath = sweep(frame, {{0, {-2, 0, 0}}, {.1, {-1.5, 0, 0}}});
    REQUIRE(shortPath.valid);
    CHECK_FALSE(shortPath.contact);
}

TEST_CASE("contact fuze stable ties retain immutable panel and component identities", "[contact-fuze]") {
    auto frame = boxedFrame();
    frame.panels.push_back(frame.panels.front());
    dmg::Airframe::Component component;
    component.half = {1, 1, 1};
    frame.components.push_back(component);
    const auto hit = sweep(frame, {{0, {-2, 0, 0}}, {1, {2, 0, 0}}});
    REQUIRE(hit.contact);
    CHECK(hit.kind == weapon::ContactKind::Panel);
    CHECK(hit.index == 0);
    frame.panels.clear();
    const auto exposedComponent = sweep(frame, {{0, {-2, 0, 0}}, {1, {2, 0, 0}}});
    REQUIRE(exposedComponent.contact);
    CHECK(exposedComponent.kind == weapon::ContactKind::Component);
    CHECK(exposedComponent.index == 0);
}

TEST_CASE("contact fuze earliest geometry takes precedence over primitive kind", "[contact-fuze]") {
    auto frame = boxedFrame();
    frame.panels[0].centre.x = 2;
    dmg::Airframe::Component component;
    component.half = {1, 1, 1};
    frame.components.push_back(component);
    auto timing = window();
    timing.armingS = .5;
    const auto hit = sweep(frame, {{0, {-2, 0, 0}}, {1, {2, 0, 0}}}, timing);
    REQUIRE(hit.valid);
    REQUIRE(hit.contact);
    CHECK(hit.kind == weapon::ContactKind::Component);
    CHECK(hit.index == 0);
    CHECK(near(hit.offsetS, .25));
    CHECK_FALSE(hit.triggered); // The later armed panel entry cannot replace it.
}

TEST_CASE("contact fuze omits separated geometry without modifying shared airframes", "[contact-fuze]") {
    auto frame = std::make_shared<dmg::Airframe>(boxedFrame());
    frame->sections.resize(1);
    frame->sections[0].hasRoot = true;
    frame->panels[0].section = 0;
    dmg::VehicleDamage separated(frame), intact(frame);
    REQUIRE(separated.separate(0));
    const std::vector<weapon::ContactSample> path{{0, {-2, 0, 0}}, {1, {2, 0, 0}}};
    CHECK_FALSE(weapon::contactFuze(path, separated.attachedGeometry(), window()).contact);
    CHECK(weapon::contactFuze(path, intact.attachedGeometry(), window()).contact);
    CHECK(frame->panels.size() == 1);
}

TEST_CASE("contact fuze rejects malformed timing motion and active geometry", "[contact-fuze]") {
    const auto frame = boxedFrame();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<weapon::ContactSample> path{{0, {-2, 0, 0}}, {1, {2, 0, 0}}};
    for (const auto& bad : std::vector<std::vector<weapon::ContactSample>>{{}, {{0, {0, 0, 0}}},
             {{.1, {-2, 0, 0}}, {1, {2, 0, 0}}}, {{0, {-2, 0, 0}}, {0, {2, 0, 0}}},
             {{0, {-2, 0, 0}}, {inf, {2, 0, 0}}}, {{0, {-2, nan, 0}}, {1, {2, 0, 0}}}}) {
        const auto rejected = sweep(frame, bad);
        CHECK_FALSE(rejected.valid);
        CHECK_FALSE(rejected.triggered);
    }
    for (int field = 0; field < 5; ++field) {
        auto timing = window();
        (field == 0 ? timing.ageBeforeS : field == 1 ? timing.durationS : field == 2 ? timing.armingS :
         field == 3 ? timing.lifeS : timing.cutoffS) = nan;
        CHECK_FALSE(sweep(frame, path, timing).valid);
    }
    auto negative = frame;
    negative.panels[0].half.x = -1;
    CHECK_FALSE(sweep(negative, path).valid);
    auto unavailable = frame;
    unavailable.panels[0].centre.x = inf;
    CHECK_FALSE(sweep(unavailable, path).valid);
}

TEST_CASE("contact fuze consumes an unarmed first crossing across decision steps", "[contact-fuze]") {
    const auto frame = boxedFrame();
    const std::vector<weapon::ContactSample> crossing{{0, {-2, 0, 0}}, {1, {2, 0, 0}}};
    auto timing = window();
    timing.armingS = .5;
    const auto unarmed = sweep(frame, crossing, timing);
    REQUIRE(unarmed.contact);
    REQUIRE_FALSE(unarmed.triggered);
    weapon::ContactFuzeState state;
    CHECK_FALSE(state.commit(unarmed));
    CHECK(state.consumed());
    timing.ageBeforeS = 1;
    const auto laterArmed = sweep(frame, crossing, timing);
    REQUIRE(laterArmed.triggered);
    CHECK_FALSE(state.commit(laterArmed));
    state.reset(); // A distinct release/generation owns a fresh channel.
    CHECK_FALSE(state.consumed());
    CHECK(state.commit(laterArmed));
    CHECK_FALSE(state.commit(laterArmed));
}

TEST_CASE("contact fuze candidate evaluation does not consume a target iteration order", "[contact-fuze]") {
    const auto frame = boxedFrame();
    weapon::ContactFuzeState state;
    const auto later = sweep(frame, {{0, {-4, 0, 0}}, {1, {0, 0, 0}}});
    const auto first = sweep(frame, {{0, {-2, 0, 0}}, {1, {2, 0, 0}}});
    REQUIRE(later.contact);
    REQUIRE(first.contact);
    REQUIRE(first.offsetS < later.offsetS);
    CHECK_FALSE(state.consumed());
    CHECK_FALSE(state.commit({}));
    CHECK_FALSE(state.commit(sweep(frame, {{0, {0, 0, 0}}, {1, {2, 0, 0}}})));
    CHECK_FALSE(state.commit(sweep(frame, {{0, {-2, 2, 0}}, {1, {2, 2, 0}}})));
    CHECK_FALSE(state.consumed());
    CHECK(state.commit(first));
    CHECK_FALSE(state.commit(later));
}

TEST_CASE("contact fuze clips to captured time and preserves a folded finite quad", "[contact-fuze]") {
    auto timing = window();
    timing.ageBeforeS = 9.8;
    const auto expired = sweep(boxedFrame(), {{0, {-2, 0, 0}}, {1, {2, 0, 0}}}, timing);
    REQUIRE(expired.valid);
    CHECK_FALSE(expired.contact);
    timing = window();
    timing.cutoffS = .2;
    CHECK_FALSE(sweep(boxedFrame(), {{0, {-2, 0, 0}}, {1, {2, 0, 0}}}, timing).contact);

    dmg::Airframe frame;
    auto folded = face(0);
    folded.corners[3].x = 1;
    frame.panels.push_back(folded);
    // This point belongs to triangle (0,2,3), whose plane is x=(z-y)/2.
    const auto hit = sweep(frame, {{0, {-1, -.5, .5}}, {1, {1, -.5, .5}}});
    REQUIRE(hit.valid);
    REQUIRE(hit.contact);
    CHECK(near(hit.offsetS, .75));
    // A collapsed tip triangle contributes no fabricated volume or radius.
    frame.panels[0].corners.fill({0, 0, 0});
    const auto collapsed = sweep(frame, {{0, {-1, 0, 0}}, {1, {1, 0, 0}}});
    REQUIRE(collapsed.valid);
    CHECK_FALSE(collapsed.contact);
}
