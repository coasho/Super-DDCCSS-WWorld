// Moving control surfaces (VSG scene graph, no Vulkan): the nodes a model
// names fsim:<channel> turn with each vehicle's own deflections, the way
// JSBSim means them, and the geometry under them stays shared. The engines'
// exhaust hangs under the afterburner nodes, and the airflow effects follow
// the flight state by rules of their own.
#include "world/Airflow.h"
#include "world/WingVapour.h"
#include "world/Clouds.h"
#include "world/Exhaust.h"
#include "world/Interpolator.h"
#include "world/Propellants.h"
#include "core/Json.h"
#include "world/VehicleVisuals.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>

#include <vsgXchange/all.h>

#include <cmath>
#include <set>
#include <filesystem>
#include <functional>
#include <fstream>
#include <optional>
#include <vector>

using namespace fsim;
using Joint = world::VehicleVisuals::Joint;

namespace {

/// The matrix from a graph's root down to one transform in it (inclusive).
class MatrixTo : public vsg::Visitor {
public:
    explicit MatrixTo(const vsg::Node* target) : target_(target) {}
    std::optional<vsg::dmat4> found;

    void apply(vsg::Object& o) override {
        if (!found) o.traverse(*this);
    }
    void apply(vsg::Transform& t) override {
        if (found) return;
        const vsg::dmat4 outer = m_;
        m_ = t.transform(m_);
        if (&t == target_) found = m_;
        else t.traverse(*this);
        m_ = outer;
    }

private:
    const vsg::Node* target_;
    vsg::dmat4 m_;
};

vsg::dmat4 matrixTo(const vsg::ref_ptr<vsg::Node>& root, const vsg::Node* target) {
    MatrixTo v(target);
    root->accept(v);
    REQUIRE(v.found);
    return *v.found;
}

vsg::dvec3 point(const vsg::dmat4& m, const vsg::dvec3& p) {
    const vsg::dvec4 r = m * vsg::dvec4(p.x, p.y, p.z, 1.0);
    return vsg::dvec3(r.x, r.y, r.z);
}

} // namespace

TEST_CASE("a weapon bay's cover is the radar's: the viewer draws the doors, not it", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "f22a" / "f22a.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(1, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:f22a");
    struct Find : vsg::Visitor {
        int covers = 0, drawnCovers = 0, doors = 0;
        void apply(vsg::Object& o) override { o.traverse(*this); }
        void apply(vsg::MatrixTransform& t) override {
            std::string name;
            if (t.getValue("name", name)) {
                if (name.rfind("fsim:cover:", 0) == 0) ++covers, drawnCovers += t.children.empty() ? 0 : 1;
                if (name.rfind("fsim:bay:", 0) == 0 && !t.children.empty()) ++doors;
            }
            t.traverse(*this);
        }
    } find;
    visuals.node()->accept(find);
    CHECK(find.covers >= 3);
    CHECK(find.drawnCovers == 0);
    CHECK(find.doors >= 4);
}

TEST_CASE("a vehicle that appears is drawn where it is from its first frame", "[render][visuals]") {
    // (a store released mid-replay: its slot empty until then - interpolated from the Earth's centre, its first frame
    // was drawn deep inside the Earth and its smoke trail began there)
    world::Interpolator interpolator(2);
    sim::SnapshotBatch batch;
    batch.states.resize(2);
    const double at[3] = {1000.0, 2000.0, 6356752.0};
    for (int k = 0; k < 6; ++k) {
        batch.simTime = k / 60.0;
        batch.sequence = static_cast<std::uint64_t>(k);
        for (auto& s : batch.states) s.simTime = batch.simTime;
        for (int i = 0; i < 3; ++i) batch.states[0].positionEcef[i] = at[i];
        if (k >= 3) // the second appears
            for (int i = 0; i < 3; ++i) batch.states[1].positionEcef[i] = at[i] + 10.0;
        interpolator.push(batch);
        if (!interpolator.update(1.0 / 60.0, 1.0, false)) continue;
        const auto& s = interpolator.states()[1];
        const double r = std::sqrt(s.positionEcef[0] * s.positionEcef[0] + s.positionEcef[1] * s.positionEcef[1] + s.positionEcef[2] * s.positionEcef[2]);
        INFO("frame " << k << ": " << r << " m from the Earth's centre");
        CHECK((r == 0.0 || std::abs(r - 6.37e6) < 2e4)); // not there yet, or where it is
    }
}

TEST_CASE("a replay's frame is drawn at its time, however long the frames between take", "[render][visuals]") {
    // (screenshots on a replay's clock: on the interpolator's own clock, paced by the window's frames, a slow frame -
    // a screenshot saved - left it extrapolating, then snapping back: a missile drawn ahead of its rail, then behind)
    world::Interpolator interpolator(1);
    sim::SnapshotBatch batch;
    batch.states.resize(1);
    const auto x = [](double t) { return 6378137.0 + 230.0 * t + 50.0 * t * t; }; // accelerating
    int pushed = 0;
    for (const double at : {0.02, 0.04, 0.05, 0.09, 0.1, 0.13}) { // each at its time, between its frames
        // as the viewer feeds a replay: its frames to the one after the time
        const int k = static_cast<int>(at * 60.0);
        for (; pushed <= k + 1; ++pushed) {
            batch.simTime = pushed / 60.0;
            batch.states[0].simTime = batch.simTime;
            batch.states[0].positionEcef[0] = x(batch.simTime);
            interpolator.push(batch);
        }
        REQUIRE(interpolator.updateAt(at));
        const double a = x(k / 60.0), b = x((k + 1) / 60.0);
        const double drawn = interpolator.states()[0].positionEcef[0];
        INFO("at " << at << " s: drawn " << drawn - x(at) << " m from where it was");
        CHECK(std::abs(drawn - (a + (b - a) * (at * 60.0 - k))) < 1e-6);
        CHECK(std::abs(drawn - x(at)) < 0.01); // within the frames' chord of the path
    }
}

TEST_CASE("control surfaces turn with each vehicle's own deflections", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "c172" / "c172.glb")); // written by tools/hangar

    world::VehicleVisuals::Settings settings; // no default model: the placeholder
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(2, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:c172");
    visuals.setModel(1, {}, "jsbsim:c172");
    const auto& a = visuals.pose(0);
    const auto& b = visuals.pose(1);

    // flaps, ailerons and elevator both sides, one rudder - and the propeller
    const auto count = [](const auto& joints, Joint::Kind kind) {
        return std::count_if(joints.begin(), joints.end(), [kind](const Joint& j) { return j.kind == kind; });
    };
    REQUIRE(count(a.joints, Joint::Surface) == 7);
    REQUIRE(count(a.joints, Joint::Propeller) == 1);
    REQUIRE(b.joints.size() == a.joints.size());
    REQUIRE(a.normal);
    REQUIRE(a.normal != b.normal);
    // (whether a node holds one of a vehicle's joints: on the path down to it, its own copy too)
    const std::function<bool(const vsg::Node*, const decltype(a.transforms)&)> holds = [&](const vsg::Node* n, const auto& joints) {
        if (std::any_of(joints.begin(), joints.end(), [n](const auto& t) { return t.get() == n; })) return true;
        const auto* g = dynamic_cast<const vsg::Group*>(n);
        return g && std::any_of(g->children.begin(), g->children.end(), [&](const auto& c) { return holds(c.get(), joints); });
    };
    for (std::size_t j = 0; j < a.transforms.size(); ++j) {
        REQUIRE(a.transforms[j] != b.transforms[j]); // each vehicle turns its own
        const auto& ka = a.transforms[j]->children;
        const auto& kb = b.transforms[j]->children;
        REQUIRE(ka.size() == kb.size());
        for (std::size_t k = 0; k < ka.size(); ++k) {
            // a joint nested in it (a wheel in its oleo, a control surface on its breakup section) is the vehicle's
            // own too, and what holds one; everything else, the geometry, is shared
            const auto nested = std::find_if(a.transforms.begin(), a.transforms.end(),
                                             [&](const auto& t) { return t.get() == ka[k].get(); });
            if (nested != a.transforms.end())
                REQUIRE(kb[k].get() == b.transforms[static_cast<std::size_t>(nested - a.transforms.begin())].get());
            else if (holds(ka[k].get(), a.transforms)) {
                REQUIRE(ka[k] != kb[k]);
                REQUIRE(holds(kb[k].get(), b.transforms));
            } else REQUIRE(ka[k] == kb[k]);
        }
    }

    // At rest, then vehicle 0 with every channel positive; vehicle 1 stays at rest.
    std::vector<sim::VehicleState> states(2);
    visuals.update(Span<const sim::VehicleState>(states));
    std::vector<vsg::dmat4> rest;
    for (const auto& t : a.transforms) rest.push_back(matrixTo(a.normal, t.get()));
    states[0].aileronRad = states[0].elevatorRad = states[0].rudderRad = states[0].flapsRad = 0.25;
    visuals.update(Span<const sim::VehicleState>(states));

    int left = 0, right = 0;
    for (std::size_t j = 0; j < a.transforms.size(); ++j) {
        const vsg::dmat4 before = rest[j];
        const vsg::dmat4 after = matrixTo(a.normal, a.transforms[j].get());
        if (a.joints[j].kind != Joint::Surface) { // the propeller turns with the clock, not the controls
            REQUIRE(after == before);
            continue;
        }
        // A point on the surface behind the hinge (body axes: x forward, y right,
        // z down): the local direction that points aft at rest, off the hinge axis.
        const vsg::dmat4 inv = vsg::inverse(before);
        vsg::dvec3 aft = point(inv, point(before, vsg::dvec3()) + vsg::dvec3(-1.0, 0.0, 0.0));
        aft.x = 0.0; // local x is the hinge
        const vsg::dvec3 probe = vsg::normalize(aft) * 0.3;
        const vsg::dvec3 moved = point(after, probe) - point(before, probe);
        const bool onLeft = point(before, vsg::dvec3()).y < 0.0;
        CAPTURE(j, a.joints[j].channel, onLeft, moved.x, moved.y, moved.z);
        switch (a.joints[j].channel) {
        case Joint::Elevator:
        case Joint::Flaps: REQUIRE(moved.z > 0.05); break;                  // trailing edge down
        case Joint::Aileron:                                                 // left down, right up
            REQUIRE((onLeft ? moved.z : -moved.z) > 0.05);
            ++(onLeft ? left : right);
            break;
        case Joint::Rudder: REQUIRE(moved.y < -0.05); break;                 // trailing edge left
        case Joint::Speedbrake: REQUIRE(vsg::length(moved) > 0.05); break; // a panel opening (it moves out of the skin)
        }
        // vehicle 1 did not move
        REQUIRE(matrixTo(b.normal, b.transforms[j].get()) == before);
    }
    REQUIRE(left == 1);
    REQUIRE(right == 1);
}

TEST_CASE("a stock aircraft is drawn with the model registered to stand in for it", "[render][visuals]") {
    // models.txt (fsim hangar register): jsbsim:<type> drawn with a design's
    // model, moved so its wheels stand where the stock aircraft's do
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    const auto reg = std::filesystem::temp_directory_path() / "fsim-test-standins";
    std::filesystem::create_directories(reg);
    {
        std::ofstream f(reg / "models.txt");
        f << "# test register\nstocktype c172 0.5 0 -0.25\nbroken line\n";
    }
    world::VehicleVisuals::Settings settings;
    settings.modelDirs = {reg, dir};
    world::VehicleVisuals visuals(2, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:stocktype");
    visuals.setModel(1, {}, "jsbsim:c172");
    const std::string key = visuals.modelOf(0);
    REQUIRE(key.find("c172.glb") != std::string::npos);
    REQUIRE(key.find('|') != std::string::npos);
    REQUIRE(visuals.modelOf(1).find('|') == std::string::npos); // the design itself: as built
    // the model's own root carries the offset (body axes: forward, right, down)
    auto root = visuals.pose(0).normal.cast<vsg::MatrixTransform>();
    REQUIRE(root);
    REQUIRE(std::abs(root->matrix[3][0] - 0.5) < 1e-9);
    REQUIRE(std::abs(root->matrix[3][2] + 0.25) < 1e-9);
    REQUIRE(visuals.pose(0).joints.size() == visuals.pose(1).joints.size());
    std::filesystem::remove_all(reg);
}

TEST_CASE("control surface names parse, and anything else is left alone", "[render][visuals]") {
    Joint j;
    REQUIRE(Joint::parse("fsim:elevator", j));
    REQUIRE(j.channel == Joint::Elevator);
    REQUIRE(j.gain == 1.0);
    REQUIRE(Joint::parse("fsim:aileron:-0.5", j));
    REQUIRE(j.channel == Joint::Aileron);
    REQUIRE(j.gain == -0.5);
    REQUIRE(Joint::parse("fsim:flap", j));
    REQUIRE(j.channel == Joint::Flaps);
    REQUIRE(j.mix.empty());
    // a stabilator that rolls too, a flaperon
    REQUIRE(Joint::parse("fsim:elevator+aileron:-0.25", j));
    REQUIRE(j.channel == Joint::Elevator);
    REQUIRE(j.gain == 1.0);
    REQUIRE(j.mix.size() == 1);
    REQUIRE(j.mix[0].first == Joint::Aileron);
    REQUIRE(j.mix[0].second == -0.25);
    REQUIRE(Joint::parse("fsim:aileron+flaps:-1", j));
    REQUIRE(j.mix.size() == 1);
    sim::VehicleState s;
    s.aileronRad = 0.1;
    s.flapsRad = 0.3;
    j.rest = vsg::dmat4();
    const vsg::dmat4 m = j.matrix(s); // turned by 0.1 - 0.3 about x
    REQUIRE(std::abs(std::atan2(m[1][2], m[1][1]) - (-0.2)) < 1e-12);
    // stops: a canard that travels further than the elevons on its channel,
    // an elevon held at its own when pitch and roll add up
    REQUIRE(Joint::parse("fsim:elevator:-1@-50,20", j));
    REQUIRE(j.gain == -1.0);
    s = sim::VehicleState();
    s.elevatorRad = 0.8; // 46 deg nose down: the canard turns -46, within its -50
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - (-0.8)) < 1e-12);
    REQUIRE(Joint::parse("fsim:elevator+aileron:-1@-25,25", j));
    s.elevatorRad = 0.3;
    s.aileronRad = -0.3; // 0.6 rad asked, held at 25 deg
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - 25.0 * 3.14159265358979323846 / 180.0) < 1e-12);
    REQUIRE(Joint::parse("fsim:elevator", j)); // no stops: turns as far as asked
    s.elevatorRad = 0.9;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - 0.9) < 1e-12);
    REQUIRE_FALSE(Joint::parse("fsim:canopy", j));
    REQUIRE_FALSE(Joint::parse("fsim:rudder:x", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator+", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator+canopy", j));
    // landing gear: a leg that turns 95 deg as the gear goes up (1 -> 0), a
    // door that opens in the first fifth of the way down
    REQUIRE(Joint::parse("fsim:gear:95", j));
    REQUIRE(j.kind == Joint::Gear);
    s = sim::VehicleState();
    s.gearPosition = 1.0;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1])) < 1e-12);
    s.gearPosition = 0.5;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - 47.5 * 3.14159265358979323846 / 180.0) < 1e-9);
    REQUIRE(Joint::parse("fsim:gear:-80:0:0.2", j));
    s.gearPosition = 0.1;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) + 40.0 * 3.14159265358979323846 / 180.0) < 1e-9);
    s.gearPosition = 1.0;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) + 80.0 * 3.14159265358979323846 / 180.0) < 1e-9);
    // everything else moves with what the simulation reports, not with a
    // formula of the viewer's: an afterburner plume by the engine's afterburner
    // (hidden when it is out), full length when it is full
    const auto turn = [](const vsg::dmat4& mat) { return std::atan2(mat[1][2], mat[1][1]); };
    constexpr double kRad = 3.14159265358979323846 / 180.0;
    REQUIRE(Joint::parse("fsim:afterburner:1", j));
    REQUIRE(j.kind == Joint::Afterburner);
    REQUIRE(j.engine == 1);
    s = sim::VehicleState();
    s.engineCount = 2;
    s.throttlePosition[1] = 2.0; // the lever alone lights nothing
    REQUIRE(j.matrix(s)[0][0] == 0.0);
    s.afterburner[1] = 1.0;
    REQUIRE(std::abs(j.matrix(s)[0][0] - 1.0) < 1e-12);
    REQUIRE(Joint::parse("fsim:afterburner", j));
    REQUIRE(j.engine == 0);
    // a leading-edge flap as the flight controls move it, held within its stops
    REQUIRE(Joint::parse("fsim:lef@-2,25", j));
    REQUIRE(j.kind == Joint::LeadingEdge);
    s = sim::VehicleState();
    s.alphaRad = 0.5; // not a guess from the angle of attack
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.leadingEdgeFlapRad = 10.0 * kRad;
    REQUIRE(std::abs(turn(j.matrix(s)) - 10.0 * kRad) < 1e-9);
    s.leadingEdgeFlapRad = 40.0 * kRad;
    REQUIRE(std::abs(turn(j.matrix(s)) - 25.0 * kRad) < 1e-9);
    REQUIRE(Joint::parse("fsim:lef:-1", j));
    REQUIRE(std::abs(turn(j.matrix(s)) + 40.0 * kRad) < 1e-9);
    // a propeller at its engine's rpm: 600 rpm is a quarter turn in 0.025 s;
    // the turn holds while the clock stops, and does not jump when it jumps
    REQUIRE(Joint::parse("fsim:propeller:0", j));
    REQUIRE(j.kind == Joint::Propeller);
    s = sim::VehicleState();
    s.engineCount = 1;
    s.engineRpm[0] = 600.0;
    s.simTime = 1.0;
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.simTime = 1.025;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    s.simTime = 50.0;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    s.engineRpm[0] = 0.0; // a stopped engine: a stopped propeller
    s.simTime = 50.5;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    // a nozzle petal by the engine's nozzle: shut, half open
    REQUIRE(Joint::parse("fsim:nozzle:1:8", j));
    REQUIRE(j.kind == Joint::Nozzle);
    s = sim::VehicleState();
    s.engineCount = 2;
    s.throttlePosition[1] = 2.0;
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.nozzlePosition[1] = 0.5;
    REQUIRE(std::abs(turn(j.matrix(s)) - 4.0 * kRad) < 1e-9);
    REQUIRE_FALSE(Joint::parse("fsim:nozzle:0", j));
    REQUIRE_FALSE(Joint::parse("fsim:propeller:0:10", j));
    REQUIRE_FALSE(Joint::parse("fsim:propeller:7", j));
    REQUIRE_FALSE(Joint::parse("fsim:lef:1:2", j));
    REQUIRE_FALSE(Joint::parse("fsim:lef@25,-2", j));
    // a wheel unit's parts: the oleo slides by the strut's compression, the
    // steering turns by its angle, the wheel rolls at its speed
    REQUIRE(Joint::parse("fsim:oleo:2:1.1", j));
    REQUIRE(j.kind == Joint::Oleo);
    REQUIRE(j.wheel == 2);
    s = sim::VehicleState();
    s.wheelCount = 3;
    s.wheelCompressionM[2] = 0.1;
    REQUIRE(std::abs(j.matrix(s)[3][0] - 0.11) < 1e-12);
    s.wheelCount = 2; // no such wheel: at rest
    REQUIRE(j.matrix(s)[3][0] == 0.0);
    REQUIRE(Joint::parse("fsim:steer:0", j));
    REQUIRE(j.kind == Joint::Steer);
    s.wheelSteerRad[0] = 0.3;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.3) < 1e-12);
    REQUIRE(Joint::parse("fsim:wheel:1:0.25", j));
    REQUIRE(j.kind == Joint::Wheel);
    s.wheelSpeedMs[1] = 0.5 * 3.14159265358979323846; // a turn a second
    s.simTime = 2.0;
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.simTime = 2.25;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    REQUIRE_FALSE(Joint::parse("fsim:oleo:8", j));
    REQUIRE_FALSE(Joint::parse("fsim:steer:0:1", j));
    REQUIRE_FALSE(Joint::parse("fsim:wheel:0", j));
    REQUIRE_FALSE(Joint::parse("fsim:wheel:0:0", j));
    REQUIRE_FALSE(Joint::parse("fsim:gear", j));
    REQUIRE_FALSE(Joint::parse("fsim:gear:x", j));
    REQUIRE_FALSE(Joint::parse("fsim:gear:90:0.5:0.5", j));
    REQUIRE_FALSE(Joint::parse("fsim:afterburner:9", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator@20,-50", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator@-50", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator@-50,20x", j));
    REQUIRE_FALSE(Joint::parse("elevator", j));
}

TEST_CASE("an engine's exhaust hangs under its afterburner node and follows its engine", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "f16c" / "f16c.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(2, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:f16c");
    visuals.setModel(1, {}, "jsbsim:f16c");
    REQUIRE(visuals.exhaust()); // the shaders build (no Vulkan device needed for that)

    const auto& pose = visuals.pose(0);
    std::size_t j = 0;
    while (j < pose.joints.size() && pose.joints[j].kind != Joint::Afterburner) ++j;
    REQUIRE(j < pose.joints.size());
    REQUIRE(pose.flames[j] == 0);
    // sized by the model's own flame: the F100's nozzle, 1.08 m across
    REQUIRE(std::abs(pose.joints[j].radius - 0.538) < 0.03);
    // the node holds a switch between the model's own flame and the exhaust,
    // each vehicle its own
    const auto& node = pose.transforms[j];
    REQUIRE(node->children.size() == 1);
    auto sw = node->children[0].cast<vsg::Switch>();
    REQUIRE(sw);
    REQUIRE(sw == pose.flameSwitches[j]);
    REQUIRE(sw != visuals.pose(1).flameSwitches[j]);
    REQUIRE(sw->children.size() == 2);
    REQUIRE(sw->children[0].mask == vsg::MASK_OFF);

    std::vector<sim::VehicleState> states(2);
    for (auto& s : states) {
        s.engineCount = 1;
        s.engineN2[0] = 0.0; // stopped: nothing to see
    }
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask == vsg::MASK_OFF);
    REQUIRE(node->matrix == pose.joints[j].rest); // never scaled: the exhaust sizes itself
    states[0].engineN2[0] = 60.0; // idle: running - a dim glow in the nozzle, the hot gas behind it
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask != vsg::MASK_OFF);
    REQUIRE(visuals.pose(1).flameSwitches[j]->children[1].mask == vsg::MASK_OFF); // vehicle 1 still stopped
    states[0].engineN2[0] = 100.0; // military power: the nozzle glows deeper, the air shimmers
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask != vsg::MASK_OFF);
    states[0].afterburner[0] = 1.0;
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask != vsg::MASK_OFF);
    REQUIRE(node->matrix == pose.joints[j].rest);

    // switched off: the model's flame, stretched as before
    visuals.setExhaust(false);
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[0].mask != vsg::MASK_OFF);
    REQUIRE(sw->children[1].mask == vsg::MASK_OFF);
    Joint stretched = pose.joints[j];
    REQUIRE(node->matrix == stretched.matrix(states[0]));
    visuals.setExhaust(true);
    REQUIRE(sw->children[0].mask == vsg::MASK_OFF);

    // what the airflow effects hang on: the jet aft, pointing aft; the wing's
    // tip from the manifest
    const auto& shape = visuals.shape(0);
    REQUIRE(shape.valid);
    REQUIRE(shape.jets.size() == 1);
    REQUIRE(std::abs(shape.jets[0].exit.x + 6.0) < 0.05);
    REQUIRE(shape.jets[0].direction.x < -0.99);
    REQUIRE(std::abs(shape.jets[0].radius - 0.538) < 0.03);
    REQUIRE(std::abs(shape.tip.y - 4.572) < 1e-6);
    REQUIRE(shape.tip.x < -2.0); // its trailing edge
    REQUIRE(std::any_of(shape.surfaces.begin(), shape.surfaces.end(), [](const auto& s) { return s.kind == "strake"; }));
    REQUIRE(shape.hi.y > 4.5);
}

TEST_CASE("a model's manifest names its lifting surfaces and its jets' kinds", "[render][visuals]") {
    const auto dir = std::filesystem::temp_directory_path() / "fsim-test-manifest";
    std::filesystem::create_directories(dir);
    const auto model = dir / "m.glb";
    {
        std::ofstream f(model.string() + ".manifest");
        f << "forward -y\nup +z\nscale 1.0\n# surfaces\n"
             "wing 2.7 0 0 4.965 -1.136 4.572 0 1.131\n"
             "strake 6.4 0.45 0 4.078 1.554 1.366 0 0.05\n"
             "canard 1 2 3\n" // not sections of four: left out
             "engine 0 6.0 0\nengine 1 0.36 1\nengine 2 abc\n" // (the last malformed: left out)
             "rocket 3 aluminised 0.12 30.0\nrocket 2 sooty 0.1 5\n"; // (an unknown smoke class: left out)
    }
    world::VehicleVisuals::Settings s;
    s.modelPath = model.string();
    REQUIRE(world::VehicleVisuals::applyManifest(s));
    REQUIRE(s.surfaces.size() == 2);
    REQUIRE(s.surfaces[0].kind == "wing");
    REQUIRE(s.surfaces[0].sections.size() == 2);
    REQUIRE(s.surfaces[0].sections[1].y == 4.572);
    REQUIRE(s.surfaces[0].sections[1].w == 1.131);
    REQUIRE(s.surfaces[1].kind == "strake");
    // the jets' kinds: a high-bypass jet without an afterburner, a fighter's augmented one
    REQUIRE(s.engines.size() == 2);
    REQUIRE(s.engines.at(0) == std::pair<double, bool>(6.0, false));
    REQUIRE(s.engines.at(1) == std::pair<double, bool>(0.36, true));
    // a missile's motor: its smoke class, its nozzle's exit and its boost's thrust
    REQUIRE(s.rockets.size() == 1);
    REQUIRE(s.rockets.at(3).smoke == 2);
    REQUIRE(s.rockets.at(3).exitDiameterM == 0.12);
    REQUIRE(s.rockets.at(3).peakThrustN == 30000.0);
    std::filesystem::remove_all(dir);
}

TEST_CASE("the propellant classes are NASA CEA's (tools/propellants/classes.json)", "[render][visuals]") {
    const std::filesystem::path file = std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR).parent_path() / "tools" / "propellants" / "classes.json";
    REQUIRE(std::filesystem::exists(file));
    std::ifstream in(file);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto doc = core::Json::parse(text, file.string());
    for (const auto& [name, index] : {std::pair<const char*, int>{"reduced", 1}, {"aluminised", 2}}) {
        CAPTURE(name);
        const core::Json& c = doc.child("classes").child(name);
        const world::Propellant& p = world::propellant(index);
        CHECK(p.computed);
        const auto near = [](double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(1.0, std::abs(b)); };
        CHECK(near(p.exitK, c.number("exit_k", 0)));
        CHECK(near(p.gamma, c.number("effective_gamma", 0)));
        CHECK(near(p.exitMach, std::round(c.number("exit_mach", 0) * 1000.0) / 1000.0));
        CHECK(near(p.molarMass, c.number("exit_gas_molar_mass", 0)));
        CHECK(near(p.exitPressure, c.number("exit_over_sea_level_pressure", 0)));
        CHECK(near(p.water, c.number("water_after_afterburning", 0)));
        CHECK(near(p.afterburning, c.number("afterburning_heat_j_per_kg", 0)));
        CHECK(near(p.hcl, c.number("hcl", 0)));
        CHECK(near(p.alumina, c.number("alumina", 0)));
        CHECK(near(p.ispS, 0.95 * c.number("isp_exit_pressure_s", 0)));
    }
    CHECK_FALSE(world::propellant(0).computed); // (the minimum-smoke class: an estimate)
}

TEST_CASE("a rocket's plume by its propellant: an aluminised motor's hotter and its core held, a reduced-smoke one's carbon burnt",
          "[render][visuals]") {
    // (launch photographs: an AIM-7's aluminised boost saturated white from its nozzle for 1.6 - 2.3 missile lengths, an
    // AIM-120's reduced-smoke glow brightest at its nozzle - docs/sdk/viewer.md; the jet's numbers that draw them)
    world::Exhaust exhaust(1, vsg::Options::create());
    REQUIRE(exhaust.valid());
    sim::VehicleState s;
    s.engineCount = 1;
    s.altitudeMslM = 6000.0, s.airspeedTrueMs = 340.0, s.thrustN[0] = 25000.0;
    const auto rowsOf = [&](world::Exhaust::Kind kind) {
        REQUIRE(exhaust.set(0, 0, 0.053, 0.053, s, 0, kind));
        std::vector<vsg::vec4> r;
        for (std::uint32_t i = 0; i < world::Exhaust::kRows; ++i) r.push_back(exhaust.rows()->at(i));
        return r;
    };
    world::Exhaust::Kind rocket;
    rocket.rocket = true, rocket.peakThrustN = 25000.0;
    rocket.smoke = world::Exhaust::Kind::Aluminised;
    const auto al = rowsOf(rocket);
    rocket.smoke = world::Exhaust::Kind::Reduced;
    const auto rs = rowsOf(rocket);
    rocket.smoke = world::Exhaust::Kind::Minimum;
    const auto ms = rowsOf(rocket);
    INFO("the expanded jet: aluminised " << al[1].x << " K, reduced " << rs[1].x << " K, minimum " << ms[1].x << " K");
    // its gas: an aluminised motor's hottest, its alumina's heat holding it through the expansion; a double base's coolest
    CHECK(al[1].x > rs[1].x + 300.0f);
    CHECK(rs[1].x > ms[1].x);
    // through the shock train every rocket's core holds its exit's temperature (no dark stretch behind the nozzle);
    // carbon burns in the shear layer, alumina does not
    for (const auto* r : {&al, &rs, &ms}) CHECK((*r)[9].y == 1.0f);
    CHECK(al[9].z == 0.0f);
    CHECK(rs[9].z == 1.0f);
    CHECK(ms[9].z == 1.0f);
    // its particles: alumina's glow densest, its cool alumina left to the smoke trail (white there, not dark here)
    CHECK(al[3].x > 5.0f * rs[3].x);
    CHECK(al[3].y < 0.5f * rs[3].y); // (what survives cooling: the thin air's quench leaves a little)
    // a jet engine is drawn as it was
    sim::VehicleState jet;
    jet.engineCount = 1, jet.altitudeMslM = 6000.0, jet.airspeedTrueMs = 250.0, jet.engineN2[0] = 100.0, jet.afterburner[0] = 1.0;
    REQUIRE(exhaust.set(0, 0, 0.5, 0.5, jet, 0, world::Exhaust::Kind{}));
    CHECK(exhaust.rows()->at(9).y == 0.0f);
    CHECK(exhaust.rows()->at(9).z == 0.0f);
}

TEST_CASE("a missile's motor shows its plume while it makes thrust, from its nozzle's exit", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "aim120c" / "aim120c.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(1, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:aim120c");
    REQUIRE(visuals.exhaust());
    const auto& shape = visuals.shape(0);
    REQUIRE(shape.rockets.size() == 1); // hangar's [motor], through the manifest
    REQUIRE(shape.rockets.at(0).smoke == 1); // reduced smoke (the card: HTPB)
    REQUIRE(shape.rockets.at(0).peakThrustN > 0.0);
    const auto& pose = visuals.pose(0);
    std::size_t j = 0;
    while (j < pose.joints.size() && pose.joints[j].kind != Joint::Afterburner) ++j;
    REQUIRE(j < pose.joints.size()); // the motor's node at the nozzle's exit, aft
    REQUIRE(shape.jets.size() == 1);
    REQUIRE(shape.jets[0].direction.x < -0.99);
    const auto& sw = pose.flameSwitches[j];

    std::vector<sim::VehicleState> states(1);
    states[0].engineCount = 1;
    states[0].altitudeMslM = 6000.0, states[0].airspeedTrueMs = 300.0;
    visuals.update(Span<const sim::VehicleState>(states)); // carried, unlit: nothing
    REQUIRE(sw->children[1].mask == vsg::MASK_OFF);
    states[0].engineN2[0] = 100.0; // (a turbine's core speed means nothing to a rocket)
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask == vsg::MASK_OFF);
    states[0].thrustN[0] = shape.rockets.at(0).peakThrustN; // the boost
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask != vsg::MASK_OFF);
    // its plume from its own nozzle: the jet the exhaust works out is the exit's size, not a default
    REQUIRE(visuals.exhaustData());
    const auto rows = visuals.exhaustData()->rows();
    const float exitRadius = 0.5f * static_cast<float>(shape.rockets.at(0).exitDiameterM);
    const float boostRadius = rows->at(0).x, boostReach = rows->at(0).z;
    REQUIRE(boostRadius > exitRadius);        // under-expanded at 6 km: wider than the exit
    REQUIRE(boostRadius < 4.0f * exitRadius); // but its own, not a jet engine's default half metre
    states[0].thrustN[0] = 0.3 * shape.rockets.at(0).peakThrustN; // the sustain: the chamber's pressure down
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask != vsg::MASK_OFF);
    REQUIRE(rows->at(0).x < boostRadius); // less under-expanded: a narrower jet
    REQUIRE(rows->at(0).z <= boostReach);
    states[0].thrustN[0] = 0.0; // burnt out
    visuals.update(Span<const sim::VehicleState>(states));
    REQUIRE(sw->children[1].mask == vsg::MASK_OFF);
}

namespace {
/// An F-16's shape as its manifest gives it (aircraft/f16c/f16c.glb.manifest): its wing and strake, right half.
world::VehicleVisuals::Shape f16Shape() {
    world::VehicleVisuals::Shape shape;
    shape.valid = true;
    shape.lo = vsg::dvec3(-8.6, -4.8, -2.6);
    shape.hi = vsg::dvec3(6.4, 4.8, 1.6);
    shape.tip = vsg::dvec3(-2.267, 4.572, 0.0);
    shape.surfaces.push_back({"wing", {vsg::dvec4(2.7, 0.0, 0.0, 4.965), vsg::dvec4(-1.136, 4.572, 0.0, 1.131)}});
    shape.surfaces.push_back({"strake", {vsg::dvec4(6.4, 0.0, 0.0, 3.7), vsg::dvec4(6.4, 0.45, 0.0, 4.078), vsg::dvec4(4.48, 0.75, 0.0, 2.409),
                                         vsg::dvec4(1.554, 1.366, 0.0, 0.05)}});
    return shape;
}
/// The strongest pressure deficit across a square `half` m either side of a point (body frame), square to the flight.
double strongest(const world::VehicleVisuals::Shape& shape, const world::WingVapour::Flight& flight, const world::WingVapour::Air& air,
                 const vsg::dvec3& at, double half) {
    double most = 0.0;
    for (int i = -8; i <= 8; ++i)
        for (int j = -8; j <= 8; ++j)
            most = std::max(most, world::WingVapour::deficit(shape, flight, air, at + vsg::dvec3(0.0, half * i / 8.0, half * j / 8.0)));
    return most;
}
} // namespace

TEST_CASE("the vapour round an aircraft follows the flight and the air", "[render][visuals]") {
    using world::WingVapour;
    constexpr double deg = 3.14159265358979323846 / 180.0;
    // the air: the standard atmosphere, its vapour condensing only once a parcel is expanded past its dew point
    const auto humid = WingVapour::moistAir(300.0, 0.85, 0.0), dry = WingVapour::moistAir(300.0, 0.3, 0.0);
    REQUIRE(std::abs(WingVapour::moistAir(0.0, 0.5, 0.0).temperatureK - 288.15) < 1e-9);
    REQUIRE(std::abs(WingVapour::moistAir(0.0, 0.5, 0.0).pressurePa - 101325.0) < 1e-6);
    REQUIRE(std::abs(WingVapour::moistAir(11000.0, 0.5, 0.0).temperatureK - 216.65) < 1e-6);
    REQUIRE(WingVapour::liquidWater(1.0, humid) == 0.0);                          // the free air: clear
    REQUIRE(WingVapour::liquidWater(0.85, humid) > 0.0);                          // expanded 15 %: cloud
    REQUIRE(WingVapour::liquidWater(0.85, humid) < WingVapour::liquidWater(0.7, humid)); // more, the deeper
    REQUIRE(WingVapour::saturationDeficit(humid) < WingVapour::saturationDeficit(dry));    // moist air condenses sooner
    REQUIRE(WingVapour::saturationDeficit(humid) < 0.05);
    REQUIRE(WingVapour::defaultHumidity(0.0) == 0.85);                                     // aeronautic's default day

    const auto shape = f16Shape();
    const auto flight = [](double tas, double alphaDeg) {
        WingVapour::Flight f;
        f.airspeedMs = tas;
        f.alphaRad = alphaDeg * deg;
        return f;
    };
    const double needed = WingVapour::saturationDeficit(humid);
    // the tip vortices: just behind the tips, in a hard pull, fast, in humid air - and not in a gentle one
    const vsg::dvec3 tip(-2.6, 4.5, 0.0);
    REQUIRE(strongest(shape, flight(230.0, 12.0), humid, tip, 0.4) > needed);
    REQUIRE(strongest(shape, flight(230.0, 2.0), humid, tip, 0.4) < needed);
    REQUIRE(strongest(shape, flight(230.0, 12.0), humid, tip, 0.4) > strongest(shape, flight(120.0, 12.0), humid, tip, 0.4)); // faster, deeper
    // over the wing: its upper surface's suction in a pull, felt more the nearer Mach 1
    const vsg::dvec3 overWing(0.0, 2.2, -0.15); // (a quarter of the chord back, just over the surface)
    REQUIRE(strongest(shape, flight(250.0, 10.0), humid, overWing, 0.05) > needed);
    REQUIRE(strongest(shape, flight(150.0, 2.0), humid, overWing, 0.05) < needed);
    // the strakes' vortices at a high angle of attack, over the wing's root
    const vsg::dvec3 overStrake(-0.5, 1.4, -0.8);
    REQUIRE(strongest(shape, flight(140.0, 24.0), humid, overStrake, 0.8) > strongest(shape, flight(140.0, 6.0), humid, overStrake, 0.8));
    // the cone: the transonic pocket round the fuselage, just under Mach 1
    const vsg::dvec3 shoulder(-1.5, 0.0, -1.6);
    const double sound = humid.soundMs;
    REQUIRE(strongest(shape, flight(0.98 * sound, 1.0), humid, shoulder, 0.3) > strongest(shape, flight(0.8 * sound, 1.0), humid, shoulder, 0.3));
    // the box round what can fog: a hard pull in humid air has one, cruising in dry air none
    vsg::dvec3 lo, hi;
    REQUIRE(WingVapour::bounds(shape, flight(230.0, 12.0), humid, lo, hi));
    REQUIRE(lo.x < -20.0); // the tips' trails, behind
    REQUIRE(!WingVapour::bounds(shape, flight(200.0, 2.0), dry, lo, hi));
    // contrails (the Schmidt-Appleman criterion, world::cocip): in the standard atmosphere at cruise, not at 7 km,
    // not on a hot day; in a typical day's air (the world's humidity not given) as in air that is given
    const world::Airflow::Air standard;
    REQUIRE(!world::cocip::formation(world::Airflow::airAt(standard, 7000.0)).forms);
    REQUIRE(world::cocip::formation(world::Airflow::airAt(standard, 10500.0)).forms);
    world::Airflow::Air hot;
    hot.seaLevelK = 318.15;
    REQUIRE(!world::cocip::formation(world::Airflow::airAt(hot, 10500.0)).forms);
    world::Airflow::Air humidAir;
    humidAir.humidity = 0.7;
    const auto a = world::Airflow::airAt(humidAir, 10500.0);
    REQUIRE(world::cocip::rhi(a.specificHumidity, a.temperature, a.pressure) > 1.0); // 70 % over water: supersaturated over ice
    const auto i = world::Airflow::airAt(standard, 10500.0); // (a typical day: 60 % over ice, no persistent contrails)
    REQUIRE(std::abs(world::cocip::rhi(i.specificHumidity, i.temperature, i.pressure) - world::Airflow::typicalHumidityOverIce(10500.0)) < 1e-12);
    // the air streaks: faster, stronger
    // the exhaust: dry power from the core's speed, the flame's length from the afterburner
    REQUIRE(world::Exhaust::dryPower(60.0) == 0.0);
    REQUIRE(world::Exhaust::dryPower(100.0) == 1.0);
    REQUIRE(world::Exhaust::running(0.0) == 0.0);  // stopped
    REQUIRE(world::Exhaust::running(60.0) == 1.0); // idle: running
    REQUIRE(world::Exhaust::running(40.0) > 0.0);  // spooling up
    REQUIRE(world::Exhaust::flameLength(0.5, 0.0) == 0.0);
    REQUIRE(world::Exhaust::flameLength(0.5, 0.2) < world::Exhaust::flameLength(0.5, 1.0));
    REQUIRE(std::abs(world::Exhaust::flameLength(0.5, 1.0) - 5.8) < 1e-9); // some six diameters
}

TEST_CASE("the clouds follow the weather: takram's layers on its base, its cover as their coverage, the sun dimmed only under a deck", "[render][visuals]") {
    using world::Clouds;
    Clouds::Weather fair;
    fair.cover = 0.4;
    fair.baseM = 1500.0;
    const auto l = Clouds::layersFor(fair);
    REQUIRE(l[0].altitudeM == 1500.0);                               // takram's lowest layer on the cloud base
    REQUIRE(l[0].heightM == 650.0);
    REQUIRE(l[1].altitudeM - l[0].altitudeM == 250.0);               // their second, as far above it as theirs
    REQUIRE(l[1].heightM == 1200.0);
    REQUIRE(l[2].altitudeM >= 7500.0);                               // the cirrus high above
    REQUIRE(l[2].altitudeM > l[1].altitudeM + l[1].heightM);
    REQUIRE(l[3].heightM == 0.0);                                    // the fourth empty, as theirs
    // the cover as takram's coverage: the measured curve, rising with the cover, none for a clear sky
    REQUIRE(Clouds::coverageForCover(0.0) == 0.0);
    REQUIRE(std::abs(Clouds::coverageForCover(0.187) - 0.3) < 1e-9); // their default coverage covers 19 % of the sky
    REQUIRE(std::abs(Clouds::coverageForCover(0.481) - 0.4) < 1e-9);
    double last = 0.0;
    for (double c = 0.01; c <= 1.0; c += 0.01) {
        REQUIRE(Clouds::coverageForCover(c) > last);
        last = Clouds::coverageForCover(c);
    }
    REQUIRE(last <= 0.65);
    // the sun: a scattered sky takes none of it (its shadows are its own), an overcast most of it below, none above
    Clouds::Weather grey = fair;
    grey.cover = 1.0;
    const auto g = Clouds::layersFor(grey);
    REQUIRE(Clouds::overcast(fair, 500.0) == 0.0);
    REQUIRE(Clouds::overcast(grey, 500.0) > 0.6);
    REQUIRE(Clouds::overcast(grey, 9000.0) == 0.0);
    const double inside = Clouds::overcast(grey, 0.5 * (g[0].altitudeM + g[1].altitudeM + g[1].heightM));
    REQUIRE(inside > 0.0);
    REQUIRE(inside < Clouds::overcast(grey, 500.0));
    REQUIRE(Clouds::overcast(Clouds::Weather{}, 500.0) == 0.0); // a clear sky
}

TEST_CASE("the vapour cone's bounds hold all of it: nothing that fogs is cut off", "[render][visuals]") {
    using world::WingVapour;
    constexpr double deg = 3.14159265358979323846 / 180.0;
    const auto shape = f16Shape();
    for (double humidity : {0.85, 0.95})
        for (double mach : {0.95, 0.97, 0.99, 1.01}) {
            const auto air = WingVapour::moistAir(300.0, humidity, 0.0);
            WingVapour::Flight f;
            f.airspeedMs = mach * air.soundMs;
            f.alphaRad = 1.0 * deg;
            const double needed = WingVapour::saturationDeficit(air);
            vsg::dvec3 lo, hi;
            if (!WingVapour::bounds(shape, f, air, lo, hi)) continue;
            // every point that fogs, along the whole body and behind it, round it: inside the box (else it is cut off)
            int outside = 0;
            double worst = 0.0;
            vsg::dvec3 at;
            for (double x = lo.x - 5.0; x <= hi.x + 5.0; x += 0.25)
                for (double r = 0.5; r <= 60.0; r += 0.25)
                    for (double a : {0.0, 45.0, 90.0, 135.0, 180.0}) {
                        const vsg::dvec3 p(x, r * std::cos(a * deg), r * std::sin(a * deg));
                        const double beyond = std::max({lo.x - p.x, p.x - hi.x, lo.y - p.y, p.y - hi.y, lo.z - p.z, p.z - hi.z});
                        if (beyond > 0.05 && WingVapour::deficit(shape, f, air, p) > needed) {
                            ++outside;
                            if (beyond > worst) { worst = beyond; at = p; }
                        }
                    }
            INFO("humidity " << humidity << ", Mach " << mach << ": " << outside << " points fog outside the box, up to " << worst
                 << " m out, at " << at.x << ", " << at.y << ", " << at.z << " (box " << lo.x << ".." << hi.x << ", " << lo.y << ".."
                 << hi.y << ", " << lo.z << ".." << hi.z << ")");
            REQUIRE(outside == 0); // (what fogs past the bounds is cut off: a circle round the cone)
        }
}

TEST_CASE("a store station's carriage shows once a store has been there, and the store hangs on it until it leaves", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "f16c" / "f16c.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(1, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:f16c");
    const auto& shape = visuals.shape(0);
    REQUIRE(shape.stations.size() == 9); // the F-16's nine (its store-station card, through the manifest)
    // its vulnerable components (hangar/vulnerability.py, through the manifest): its engine, its four tanks, its
    // pilot ahead of the CG under the canopy, its controls' actuators - each on its breakup section
    const auto count = [&](const char* kind) {
        return std::count_if(shape.components.begin(), shape.components.end(), [&](const auto& c) { return c.kind == kind; });
    };
    CHECK(count("engine") == 1);
    std::set<std::string> tanks; // (a long wing tank a chain of boxes under its name)
    for (const auto& c : shape.components)
        if (c.kind == "fuel") tanks.insert(c.name);
    CHECK(tanks.size() == 4);
    CHECK(count("crew") == 1);
    CHECK(count("control") >= 6);
    for (const auto& c : shape.components) {
        INFO(c.kind << " " << c.name << " on " << c.section);
        CHECK((c.section == "nose" || c.section == "centre" || c.section == "left-wing" || c.section == "right-wing" || c.section == "tail"));
        if (c.kind == "crew") CHECK(c.at.x > 2.0);           // ahead of the CG
        if (c.kind == "crew") CHECK(c.at.z < 0.0);           // above it
        if (c.name == "left wing") CHECK(c.section == "left-wing");
        if (c.kind == "engine") CHECK(c.shape == "cylinder");
    }
    // its dispensers (hangar/dispensers.py, from its photographs): a pair under its aft fuselage, behind the CG,
    // firing down and outboard
    REQUIRE(shape.dispensers.size() == 2);
    for (const auto& d : shape.dispensers) {
        INFO(d.name << " on " << d.section);
        CHECK(d.at.x < -2.0);                                 // aft of the CG
        CHECK(d.at.z > 0.0);                                  // below it
        CHECK(d.direction.z > 0.8);                           // down
        CHECK(d.direction.y * d.at.y > 0.0);                  // and outboard, its own side
    }
    REQUIRE(shape.stations.at(7).at.y > 2.5);
    REQUIRE(shape.stations.at(7).bay == 0);
    const auto& pose = visuals.pose(0);
    std::vector<std::size_t> stations;
    for (std::size_t j = 0; j < pose.joints.size(); ++j)
        if (pose.joints[j].kind == Joint::Station) stations.push_back(j);
    REQUIRE(stations.size() >= 6); // the wing pylons and the centreline rack (the wingtips' are the airframe's)
    const auto scaledAway = [&](std::size_t j) { return std::abs(pose.transforms[j]->matrix[0][0]) < 1e-3; };
    std::vector<sim::VehicleState> states(1);
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : stations) CHECK(scaledAway(j)); // clean: no carriage
    REQUIRE(world::VehicleVisuals::storeType("aim-120c") == "jsbsim:aim120c");
    visuals.setStores(0, {{7, "jsbsim:aim120c"}});
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : stations) CHECK(scaledAway(j) == (pose.joints[j].station != 7));
    visuals.setStores(0, {}); // launched: the pylon and its rail stay
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : stations) CHECK(scaledAway(j) == (pose.joints[j].station != 7));
}

TEST_CASE("a weapon bay's doors turn as the store management opens them, and its store hangs inside", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "f22a" / "f22a.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(1, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:f22a");
    const auto& shape = visuals.shape(0);
    REQUIRE(shape.stations.at(5).bay > 0); // main bay 2 (its manifest's bay)
    REQUIRE(shape.stations.at(1).bay == 0);
    const int main = shape.stations.at(5).bay;
    const auto& pose = visuals.pose(0);
    std::vector<std::size_t> doors, others;
    for (std::size_t j = 0; j < pose.joints.size(); ++j)
        if (pose.joints[j].kind == Joint::BayDoor) (pose.joints[j].bay == main ? doors : others).push_back(j);
    REQUIRE(doors.size() == 2); // the main bay's pair
    REQUIRE(others.size() == 2); // a side bay's door each
    std::vector<sim::VehicleState> states(1);
    const auto rest = [&](std::size_t j) { return pose.transforms[j]->matrix == pose.joints[j].rest; };
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : doors) CHECK(rest(j)); // shut
    visuals.setStores(0, {{5, "jsbsim:aim120c"}});
    visuals.setBayDoors(0, {{5, 1.0}, {3, 0.0}});
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : doors) CHECK_FALSE(rest(j)); // open
    for (std::size_t j : others) CHECK(rest(j));      // the side bays' stay shut
    // the open door has turned its whole travel, as the node's name gives it
    const auto& d = pose.joints[doors[0]];
    CHECK(d.gearRad > 1.5);
    CHECK(d.opening == 1.0);
    visuals.setBayDoors(0, {{5, 0.0}});
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : doors) CHECK(rest(j)); // shut again
}

TEST_CASE("a missile's rollerons spin with the airstream, blurring into discs; a free-rolling tail turns as it flies", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "aim9m" / "aim9m.glb"));
    REQUIRE(std::filesystem::exists(dir / "magic2" / "magic2.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(2, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:aim9m");
    visuals.setModel(1, {}, "jsbsim:magic2");
    const auto of = [&](std::size_t slot, Joint::Kind kind) {
        std::vector<std::size_t> out;
        for (std::size_t j = 0; j < visuals.pose(slot).joints.size(); ++j)
            if (visuals.pose(slot).joints[j].kind == kind) out.push_back(j);
        return out;
    };
    const auto wheels = of(0, Joint::Rolleron), discs = of(0, Joint::RolleronDisc), tail = of(1, Joint::FreeRoll);
    REQUIRE(wheels.size() == 4); // one at each tail fin's tip
    REQUIRE(discs.size() == 4);
    REQUIRE(tail.size() == 1);   // the four fins and their ring, one part
    const auto hidden = [&](std::size_t slot, std::size_t j) { return std::abs(visuals.pose(slot).transforms[j]->matrix[0][0]) < 1e-3; };
    std::vector<sim::VehicleState> states(2);
    // carried slowly (taxiing): the wheels turn, seen as wheels
    for (auto& s : states) s.airspeedTrueMs = 1.0, s.simTime = 0.0;
    visuals.update(Span<const sim::VehicleState>(states));
    for (auto& s : states) s.simTime = 0.5;
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : wheels) CHECK_FALSE(hidden(0, j));
    for (std::size_t j : discs) CHECK(hidden(0, j));
    CHECK(visuals.pose(0).transforms[wheels[0]]->matrix != visuals.pose(0).joints[wheels[0]].rest); // (turned)
    // flying: too fast to see the notches - the discs instead
    for (auto& s : states) s.airspeedTrueMs = 300.0, s.simTime = 1.0;
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j : wheels) CHECK(hidden(0, j));
    for (std::size_t j : discs) CHECK_FALSE(hidden(0, j));
    // the tail: 0.002 turns a metre, 300 m/s for a quarter second - 0.15 of a turn
    const double before = visuals.pose(1).joints[tail[0]].phase;
    for (auto& s : states) s.simTime = 1.25;
    visuals.update(Span<const sim::VehicleState>(states));
    const double turned = visuals.pose(1).joints[tail[0]].phase - before;
    CHECK(std::abs(turned - 0.15) < 1e-6);
}

TEST_CASE("an airframe's breakup sections are joints at their breaks, what is mounted on them below them", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "f16c" / "f16c.glb"));
    world::VehicleVisuals::Settings settings;
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(1, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:f16c");
    const auto& pose = visuals.pose(0);
    std::vector<std::string> sections;
    for (const auto& j : pose.joints)
        if (j.kind == Joint::Section) sections.push_back(j.section);
    std::sort(sections.begin(), sections.end());
    CHECK(sections == std::vector<std::string>{"centre", "left-wing", "nose", "right-wing", "tail"});
    // whole, every section at rest; the jet's exhaust still where the nozzle is (now below the tail's section)
    std::vector<sim::VehicleState> states(1);
    visuals.update(Span<const sim::VehicleState>(states));
    for (std::size_t j = 0; j < pose.joints.size(); ++j)
        if (pose.joints[j].kind == Joint::Section) CHECK(pose.transforms[j]->matrix == pose.joints[j].rest);
    const auto& shape = visuals.shape(0);
    REQUIRE(shape.jets.size() == 1);
    CHECK(std::abs(shape.jets[0].exit.x + 6.0) < 0.05);
}
