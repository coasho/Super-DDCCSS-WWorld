// Pure synthetic geometry/timing cases; no equipment fuze is calibrated here.
#include "simulation/mission_systems/weapon/ContactFuze.h"
#include "simulation/mission_systems/damage/State.h"
#include "foundation/core/Json.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

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

// These synthetic fixtures deliberately describe exact boxes. Declare their
// closed surfaces explicitly; production readers never infer physical shape
// from a damage-only bounding box.
bool declareExactFixtureGeometry(dmg::Airframe& frame) {
    std::vector<dmg::ContactMesh> meshes;
    const auto append=[&](dmg::ContactMesh::Kind kind,std::size_t index,dmg::V3 centre,dmg::V3 half) {
        if (!std::isfinite(centre.x) || !std::isfinite(centre.y) || !std::isfinite(centre.z) ||
            !(half.x>0) || !(half.y>0) || !(half.z>0)) return false;
        dmg::ContactMesh mesh;
        mesh.kind=kind; mesh.index=index;
        for (const auto sign : std::vector<dmg::V3>{{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
                                                   {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}})
            mesh.vertices.push_back({centre.x+sign.x*half.x,centre.y+sign.y*half.y,centre.z+sign.z*half.z});
        mesh.triangles={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                        {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
        mesh.triangleNeighbors.assign(mesh.triangles.size(),std::nullopt); // Every exact fixture face is authored.
        meshes.push_back(std::move(mesh));
        return true;
    };
    for (std::size_t i=0;i<frame.panels.size();++i)
        if (!frame.panels[i].surface && !append(dmg::ContactMesh::Kind::Panel,i,frame.panels[i].centre,frame.panels[i].half)) return false;
    for (std::size_t i=0;i<frame.components.size();++i)
        if (!append(dmg::ContactMesh::Kind::Component,i,frame.components[i].centre,frame.components[i].half)) return false;
    std::string error;
    return dmg::setContactGeometryV1(frame,std::move(meshes),error);
}

weapon::ContactFuzeResult sweep(const dmg::Airframe& frame, const std::vector<weapon::ContactSample>& path,
                                weapon::ContactWindow timing = window()) {
    auto fixture=frame;
    if (!fixture.contactGeometry) declareExactFixtureGeometry(fixture);
    return weapon::contactFuze(path, dmg::AttachedGeometryView(fixture), timing);
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
    REQUIRE(declareExactFixtureGeometry(*frame));
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

namespace {
// A closed square pyramid: tip x=0, base x=-1. This is an independent
// analytic shape, rather than a saved trajectory or a changed equipment card.
const std::string taperedMesh = R"({"version":1,"meshes":[{
    "kind":"panel","index":0,
    "vertices":[[0,0,0],[-1,-1,-1],[-1,1,-1],[-1,1,1],[-1,-1,1]],
    "triangles":[[0,1,2],[0,2,3],[0,3,4],[0,4,1],[1,4,3],[1,3,2]],
    "triangle_neighbors":[null,null,null,null,null,null]}]})";

struct ContactFile {
    std::filesystem::path path;
    explicit ContactFile(const std::string& metadata,bool split=false) {
        static unsigned serial = 0;
        path = std::filesystem::temp_directory_path() /
            ("fsim-contact-geometry-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
             "-" + std::to_string(++serial) + ".json");
        std::ofstream output(path);
        output << R"({"aircraft":"analytical-pyramid","n_limit":[-1,1],"n_ultimate":[-1.5,1.5],
            "sections":[{"name":"nose","cg":[0,0,0],"inertia":[],"tanks":[]})";
        if(split) output << R"(,{"name":"centre","cg":[0,0,0],"inertia":[],"tanks":[]})";
        output << R"(],"panels":[{"kind":"body","section":"nose","centre":[-0.5,0,0],"half":[0.5,1,1],"area_m2":10})";
        if(split) output << R"(,{"kind":"body","section":"centre","centre":[0.5,0,0],"half":[0.5,1,1],"area_m2":10})";
        output << R"(],"components":[])";
        if (!metadata.empty()) output << ",\"contact_geometry\":" << metadata;
        output << '}';
        REQUIRE(output.good());
    }
    ~ContactFile() { std::error_code ignored; std::filesystem::remove(path, ignored); }
    std::optional<dmg::Airframe> read() const {
        std::string error;
        auto frame = dmg::readAirframe(path, error);
        INFO(error);
        return frame;
    }
};
} // namespace

TEST_CASE("authored contact mesh rejects the empty corner of a tapered damage box", "[contact-fuze][contact-mesh]") {
    const ContactFile file(taperedMesh);
    const auto frame = file.read();
    REQUIRE(frame);
    // At y=.5,z=0, the pyramid's first real surface is x=-.5. The
    // captured nose ends at x=-.25 and has not touched it; the box begins x=0.
    const auto avoided = sweep(*frame, {{0, {1, .5, 0}}, {1, {-.25, .5, 0}}});
    REQUIRE(avoided.valid);
    CHECK_FALSE(avoided.contact);
    CHECK_FALSE(avoided.triggered);
    const auto gap = sweep(*frame, {{0, {1, 1.000001, 0}}, {1, {-1, 1.000001, 0}}});
    REQUIRE(gap.valid);
    CHECK_FALSE(gap.contact);
}

TEST_CASE("authored contact mesh initiates at the true tapered face with stable source identity", "[contact-fuze][contact-mesh]") {
    const ContactFile file(taperedMesh);
    const auto frame = file.read();
    REQUIRE(frame);
    const auto hit = sweep(*frame, {{0, {1, .5, 0}}, {1, {-1, .5, 0}}});
    REQUIRE(hit.valid);
    REQUIRE(hit.contact);
    CHECK(hit.triggered);
    CHECK(hit.kind == weapon::ContactKind::Panel);
    CHECK(hit.index == 0);
    CHECK(near(hit.fraction, .75));
    // A one-micrometre gap beyond the true cut/base cannot be closed by a
    // broad-phase bound or an intersection tolerance.
    const auto pastBase = sweep(*frame, {{0, {-1.000001, -2, 0}}, {1, {-1.000001, 2, 0}}});
    REQUIRE(pastBase.valid);
    CHECK_FALSE(pastBase.contact);
}

TEST_CASE("authored closed contact volume distinguishes inside from an empty bound corner", "[contact-fuze][contact-mesh]") {
    const ContactFile file(taperedMesh);
    const auto frame = file.read();
    REQUIRE(frame);
    const auto inside = sweep(*frame, {{0, {-.75, .25, 0}}, {1, {1, .25, 0}}});
    REQUIRE(inside.valid);
    CHECK(inside.initialOverlap);
    CHECK_FALSE(inside.contact);
    const auto outside = sweep(*frame, {{0, {-.25, .5, 0}}, {1, {-.75, .5, 0}}});
    REQUIRE(outside.valid);
    CHECK_FALSE(outside.initialOverlap);
    REQUIRE(outside.contact);
    CHECK(near(outside.fraction, .5));
    const auto boundary = sweep(*frame, {{0, {-.5, .5, 0}}, {1, {1, .5, 0}}});
    CHECK(boundary.initialOverlap);
    CHECK_FALSE(boundary.contact);
}

TEST_CASE("authored contact volume follows attached panel membership", "[contact-fuze][contact-mesh]") {
    const ContactFile file(taperedMesh);
    const auto loaded = file.read();
    REQUIRE(loaded);
    auto frame = std::make_shared<dmg::Airframe>(*loaded);
    frame->sections[0].hasRoot = true;
    dmg::VehicleDamage separated(frame), intact(frame);
    REQUIRE(separated.separate(0));
    const std::vector<weapon::ContactSample> path{{0, {1, .5, 0}}, {1, {-1, .5, 0}}};
    const auto absent = weapon::contactFuze(path, separated.attachedGeometry(), window());
    REQUIRE(absent.valid);
    CHECK_FALSE(absent.contact);
    const auto present = weapon::contactFuze(path, intact.attachedGeometry(), window());
    REQUIRE(present.contact);
    CHECK(present.index == 0);
    CHECK(near(present.fraction, .75));
}

TEST_CASE("legacy damage data remains readable without authorizing box contact", "[contact-fuze][contact-mesh]") {
    const ContactFile file("");
    const auto frame = file.read();
    REQUIRE(frame);
    REQUIRE(frame->panels.size() == 1);
    CHECK(frame->panels[0].half.x == .5);
    const auto unavailable = weapon::contactFuze({{0, {1, .5, 0}}, {1, {-1, .5, 0}}},dmg::AttachedGeometryView(*frame),window());
    CHECK_FALSE(unavailable.valid);
    CHECK_FALSE(unavailable.contact);
}

TEST_CASE("explicit malformed contact geometry cannot fall back to damage boxes", "[contact-fuze][contact-mesh]") {
    for (const auto& metadata : std::vector<std::string>{
             R"({"version":2,"meshes":[]})",
             R"({"version":1,"meshes":[{"kind":"panel","index":0,"vertices":[[0,0,0]],"triangles":[[0,1,2]]}]})",
             R"({"version":1,"meshes":[{"kind":"panel","index":0,"vertices":[[0,0,0],[1,0,0],[0,1,0]],"triangles":[[0,1,2]]}]})"}) {
        const ContactFile file(metadata);
        INFO(metadata);
        CHECK_FALSE(file.read());
    }
}

TEST_CASE("contact mesh bounds come from vertices and retain first-crossing timing", "[contact-fuze][contact-mesh]") {
    const ContactFile file(taperedMesh);
    auto frame=file.read();
    REQUIRE(frame);
    // Authored skin can extend beyond an old midpoint damage box. That box
    // must neither reject the true face nor authorize its empty corner.
    frame->panels[0].half={.01,.01,.01};
    auto timing=window(); timing.armingS=.8;
    const auto first=sweep(*frame,{{0,{1,.5,0}},{1,{-1,.5,0}}},timing);
    REQUIRE(first.valid);
    REQUIRE(first.contact);
    CHECK_FALSE(first.triggered);
    CHECK(near(first.fraction,.75));
    timing=window();timing.lifeS=.75;
    CHECK_FALSE(sweep(*frame,{{0,{1,.5,0}},{1,{-1,.5,0}}},timing).contact);
    timing=window();timing.cutoffS=.75;timing.exclusiveCutoff=true;
    CHECK_FALSE(sweep(*frame,{{0,{1,.5,0}},{1,{-1,.5,0}}},timing).contact);
}

TEST_CASE("sealed contact meshes preserve thin volumes and gaps between separate bodies", "[contact-fuze][contact-mesh]") {
    auto thin=boxedFrame(); thin.panels[0].half.x=1e-6;
    REQUIRE(declareExactFixtureGeometry(thin));
    const auto hit=sweep(thin,{{0,{-1,0,0}},{1,{1,0,0}}});
    REQUIRE(hit.contact);
    CHECK(near(hit.fraction,.4999995));
    auto gap=boxedFrame();
    gap.panels[0].centre.x=-1.000001;
    gap.panels.push_back(gap.panels[0]);gap.panels[1].centre.x=1.000001;
    REQUIRE(declareExactFixtureGeometry(gap));
    const auto empty=sweep(gap,{{0,{0,-2,0}},{1,{0,2,0}}});
    REQUIRE(empty.valid);
    CHECK_FALSE(empty.initialOverlap);
    CHECK_FALSE(empty.contact);
}

TEST_CASE("contact mesh validation rejects broken topology without replacing sealed geometry", "[contact-fuze][contact-mesh]") {
    const ContactFile file(taperedMesh);
    auto frame=file.read();
    REQUIRE(frame);
    const auto sealed=frame->contactGeometry;
    const auto source=*sealed->panel(0);
    for (int defect=0;defect<5;++defect) {
        auto mesh=source;
        if (defect==0) std::swap(mesh.triangles[0][0],mesh.triangles[0][1]);
        if (defect==1) mesh.triangles.pop_back();
        if (defect==2) mesh.vertices.push_back({42,42,42});
        if (defect==3) mesh.triangles[0][0]=mesh.vertices.size();
        if (defect==4) mesh.vertices[0]=mesh.vertices[1];
        std::string error;
        CHECK_FALSE(dmg::setContactGeometryV1(*frame,{std::move(mesh)},error));
        CHECK_FALSE(error.empty());
        CHECK(frame->contactGeometry==sealed);
    }
    auto noninteger=taperedMesh;
    const auto index=noninteger.find("\"index\":0");
    REQUIRE(index!=std::string::npos);
    noninteger.replace(index,std::string("\"index\":0").size(),"\"index\":0.5");
    CHECK_FALSE(ContactFile(noninteger).read());
}

namespace {
std::string splitContactMetadata(bool includeRight=true) {
    using Json=fsim::core::Json;
    Json::Array meshes;
    for(int owner=0;owner<(includeRight?2:1);++owner) {
        Json mesh;
        mesh.set("kind","panel").set("index",owner);
        Json::Array vertices,triangles,neighbors;
        for(const auto v:std::vector<dmg::V3>{{-1,-1,-1},{0,-1,-1},{0,1,-1},{-1,1,-1},
                                             {-1,-1,1},{0,-1,1},{0,1,1},{-1,1,1}})
            vertices.emplace_back(Json::Array{v.x+owner,v.y,v.z});
        // Put the left volume's ownership cap first: an exact rim tie must
        // choose authored skin, not this arbitrary source-triangle order.
        const std::vector<std::array<int,3>> faces={{1,2,6},{1,6,5},{0,2,1},{0,3,2},{4,5,6},{4,6,7},
                                                   {0,1,5},{0,5,4},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
        for(std::size_t i=0;i<faces.size();++i) {
            const auto t=faces[i];triangles.emplace_back(Json::Array{t[0],t[1],t[2]});
            if((owner==0 && i<2) || (owner==1 && i>=10)) {
                Json neighbor;neighbor.set("kind","panel").set("index",1-owner);neighbors.push_back(std::move(neighbor));
            } else neighbors.emplace_back(nullptr);
        }
        mesh.set("vertices",std::move(vertices)).set("triangles",std::move(triangles)).set("triangle_neighbors",std::move(neighbors));
        meshes.push_back(std::move(mesh));
    }
    Json metadata;metadata.set("version",1).set("meshes",std::move(meshes));
    return metadata.dump();
}

void checkContactVector(dmg::V3 actual,dmg::V3 expected) {
    CHECK(near(actual.x,expected.x));CHECK(near(actual.y,expected.y));CHECK(near(actual.z,expected.z));
}
} // namespace

TEST_CASE("contact at an ownership rim reports the authored outer face point and normal", "[contact-fuze][contact-normal]") {
    const ContactFile file(splitContactMetadata(),true);
    const auto frame=file.read();REQUIRE(frame);
    const auto hit=sweep(*frame,{{0,{0,2,0}},{1,{0,0,0}}});
    REQUIRE(hit.valid);REQUIRE(hit.contact);
    CHECK(near(hit.offsetS,.5));CHECK(hit.index==0);
    checkContactVector(hit.pointBody,{0,1,0});
    checkContactVector(hit.normalBody,{0,1,0});
    CHECK(near(dmg::length(hit.normalBody),1));
    CHECK(dmg::dot(hit.normalBody,{0,-2,0})<0);
}

TEST_CASE("an ownership cap stays hidden while its adjacent source remains attached", "[contact-fuze][contact-normal]") {
    // The right source owner is admitted but has no available contact mesh.
    // Missing narrow-phase data does not expose its shared ownership cut.
    const ContactFile file(splitContactMetadata(false),true);
    const auto frame=file.read();REQUIRE(frame);
    const auto hit=sweep(*frame,{{0,{.5,0,0}},{1,{-.5,0,0}}});
    REQUIRE(hit.valid);CHECK_FALSE(hit.initialOverlap);
    CHECK_FALSE(hit.contact);CHECK_FALSE(hit.triggered);
}

TEST_CASE("removing the neighbor exposes its cut while true skin still wins a rim tie", "[contact-fuze][contact-normal]") {
    const ContactFile file(splitContactMetadata(),true);
    const auto loaded=file.read();REQUIRE(loaded);
    auto frame=std::make_shared<dmg::Airframe>(*loaded);frame->sections[1].hasRoot=true;
    dmg::VehicleDamage intact(frame),separated(frame);
    const std::vector<weapon::ContactSample> cut{{0,{.5,0,0}},{1,{-.5,0,0}}};
    CHECK(weapon::contactFuze(cut,intact.attachedGeometry(),window()).initialOverlap);
    REQUIRE(separated.separate(1));
    const auto exposed=weapon::contactFuze(cut,separated.attachedGeometry(),window());
    REQUIRE(exposed.contact);CHECK(exposed.index==0);CHECK(near(exposed.offsetS,.5));
    checkContactVector(exposed.pointBody,{0,0,0});checkContactVector(exposed.normalBody,{1,0,0});
    const auto rim=weapon::contactFuze({{0,{0,2,0}},{1,{0,0,0}}},separated.attachedGeometry(),window());
    REQUIRE(rim.contact);checkContactVector(rim.pointBody,{0,1,0});checkContactVector(rim.normalBody,{0,1,0});
}

TEST_CASE("folded contact faces return their own unit normal against incoming motion", "[contact-fuze][contact-normal]") {
    dmg::Airframe frame;auto folded=face(0);folded.corners[3].x=1;frame.panels.push_back(folded);
    const double unit=1/std::sqrt(6.);
    const auto forward=sweep(frame,{{0,{-1,-.5,.5}},{1,{1,-.5,.5}}});
    REQUIRE(forward.contact);CHECK(near(forward.offsetS,.75));
    checkContactVector(forward.pointBody,{.5,-.5,.5});checkContactVector(forward.normalBody,{-2*unit,-unit,unit});
    const auto reverse=sweep(frame,{{0,{1,-.5,.5}},{1,{-1,-.5,.5}}});
    REQUIRE(reverse.contact);CHECK(near(reverse.offsetS,.25));
    checkContactVector(reverse.pointBody,{.5,-.5,.5});checkContactVector(reverse.normalBody,{2*unit,unit,-unit});
}

TEST_CASE("coplanar contact has a face normal but zero normal closing motion", "[contact-fuze][contact-normal]") {
    dmg::Airframe frame;frame.panels.push_back(face(0));
    const auto tangent=sweep(frame,{{0,{0,-2,0}},{1,{0,2,0}}});
    REQUIRE(tangent.contact);CHECK(near(tangent.offsetS,.25));
    checkContactVector(tangent.pointBody,{0,-1,0});CHECK(near(dmg::length(tangent.normalBody),1));
    CHECK(near(dmg::dot(tangent.normalBody,{0,4,0}),0));
    frame.panels[0].corners.fill({0,0,0});
    const auto collapsed=sweep(frame,{{0,{-1,0,0}},{1,{1,0,0}}});
    REQUIRE(collapsed.valid);CHECK_FALSE(collapsed.contact);
    CHECK(std::isnan(collapsed.pointBody.x));CHECK(std::isnan(collapsed.normalBody.x));
}

TEST_CASE("explicit contact triangle provenance rejects missing and invalid neighbors", "[contact-fuze][contact-normal]") {
    using Json=fsim::core::Json;
    for(int defect=0;defect<5;++defect) {
        auto metadata=Json::parse(splitContactMetadata());
        auto meshes=metadata.child("meshes").asArray();
        auto mesh=meshes[0];
        if(defect==0) mesh.set("triangle_neighbors",nullptr);
        if(defect==1) mesh.set("triangle_neighbors",Json::Array{});
        auto neighbors=meshes[0].child("triangle_neighbors").asArray();
        if(defect>=2) {
            Json invalid;invalid.set("kind","panel").set("index",defect==2?0:defect==3?999:.5);
            neighbors[0]=std::move(invalid);mesh.set("triangle_neighbors",std::move(neighbors));
        }
        meshes[0]=std::move(mesh);metadata.set("meshes",std::move(meshes));
        const ContactFile file(metadata.dump(),true);CHECK_FALSE(file.read());
    }
}
