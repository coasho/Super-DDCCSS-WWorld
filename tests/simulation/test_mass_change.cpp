// A part leaving an aircraft, made physically consistent (sim::FlightModel's mass change; MS-7a's releases, MS-7g's
// parting sections): the airframe stays where it is, its CG and state move to what is left, momentum is kept.
#include "mode_flights.h"
#include "foundation/io/AssetResolver.h"
#include "contracts/world_stream/WorldLayout.h"
#include "foundation/transport/WorldMirror.h"
#include "simulation/mission_systems/Host.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/mission_systems/damage/Damage.h"
#include <FGFDMExec.h>
#include <models/FGPropagate.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;

TEST_CASE("Native rigid mutations expose coherent body and NED velocity without stepping",
          "[mass][impulse][native-state-coherence]") {
    for (const bool massChange : {false, true}) {
        INFO("mutation=" << (massChange ? "nonzero CG mass change" : "body impulse"));
        session::World world(options("same-epoch-native-state"));
        session::VehicleSpec spec;
        spec.name = "rotated rigid-state witness";
        spec.type = "jsbsim:mig29a";
        spec.initial.altitudeMslM = 6000;
        spec.initial.airspeedTrueMs = 220;
        spec.initial.headingDeg = 123;
        spec.initial.pitchDeg = 7;
        spec.initial.rollDeg = -18;
        const auto id = world.createVehicle(spec);
        REQUIRE(id);
        auto* native = dynamic_cast<sim::JsbsimModel*>(world.model(id));
        REQUIRE(native);
        sim::VehicleState before;
        native->state(before);
        const auto rigidBefore = native->rigid();
        REQUIRE(rigidBefore.massKg > 1000);
        double oldCg[3];
        REQUIRE(native->cgFromOriginBodyM(oldCg));
        const double impulse[3] = {17000, -9000, 13000};
        if (massChange) {
            const double rates[3] = {.31, -.27, .19};
            native->setBodyRates(rates);
            const double station[3] = {rigidBefore.cgStructural[0] + .5,
                                       rigidBefore.cgStructural[1] - 3,
                                       rigidBefore.cgStructural[2] - .2};
            REQUIRE(native->beginMassChange());
            REQUIRE(native->removeMass(250, station));
            REQUIRE(native->endMassChange(nullptr, impulse));
            double newCg[3];
            REQUIRE(native->cgFromOriginBodyM(newCg));
            CHECK(std::hypot(newCg[0] - oldCg[0], std::hypot(newCg[1] - oldCg[1], newCg[2] - oldCg[2])) > 1e-4);
        } else {
            REQUIRE(native->applyImpulse(impulse, nullptr));
        }
        sim::VehicleState after;
        native->state(after);
        CHECK(after.simTime == before.simTime);
        CHECK(after.stepCount == before.stepCount);
        CHECK(std::hypot(after.velocityBodyMs[0] - before.velocityBodyMs[0],
                         std::hypot(after.velocityBodyMs[1] - before.velocityBodyMs[1],
                                    after.velocityBodyMs[2] - before.velocityBodyMs[2])) > 1e-3);
        // Independent Rz(heading) Ry(pitch) Rx(roll) expansion. No simulation
        // step, native cached NED value or SDK NED value supplies the reference.
        const double cp = std::cos(after.eulerRad[0]), sp = std::sin(after.eulerRad[0]);
        const double ct = std::cos(after.eulerRad[1]), st = std::sin(after.eulerRad[1]);
        const double cy = std::cos(after.eulerRad[2]), sy = std::sin(after.eulerRad[2]);
        const double u = after.velocityBodyMs[0], v = after.velocityBodyMs[1], w = after.velocityBodyMs[2];
        const double expected[3] = {ct * cy * u + (sp * st * cy - cp * sy) * v + (cp * st * cy + sp * sy) * w,
                                    ct * sy * u + (sp * st * sy + cp * cy) * v + (cp * st * sy - sp * cy) * w,
                                    -st * u + sp * ct * v + cp * ct * w};
        const auto propagate = native->exec().GetPropagate();
        const auto direct = propagate->GetTb2l() * propagate->GetUVW();
        for (unsigned axis = 0; axis < 3; ++axis) {
            const double nativeDirect = direct(axis + 1) * .3048;
            INFO("axis=" << axis << " Euler=" << expected[axis] << " native Tb2l*UVW=" << nativeDirect
                 << " cached native GetVel=" << propagate->GetVel(static_cast<int>(axis + 1)) * .3048
                 << " SDK NED=" << after.velocityNedMs[axis]);
            CHECK(std::abs(nativeDirect - expected[axis]) < 1e-9);
            CHECK(std::abs(after.velocityNedMs[axis] - expected[axis]) < 1e-9);
        }
    }
}

namespace {

struct V {
    double x = 0.0, y = 0.0, z = 0.0;
    V operator+(const V& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V operator-(const V& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V operator*(double k) const { return {x * k, y * k, z * k}; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
};
V cross(const V& a, const V& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

/// An aircraft's rigid body now: its mass, CG, velocity (ECEF) and its model's origin (ECEF: its CG less its offset)
struct Body {
    double kg = 0.0;
    V cg, velocity, origin;
    sim::JsbsimModel::Rigid r;
};

Body body(sim::FlightModel& m) {
    auto& j = dynamic_cast<sim::JsbsimModel&>(m);
    Body b;
    b.r = j.rigid();
    b.kg = b.r.massKg;
    b.cg = {b.r.cgEcef[0], b.r.cgEcef[1], b.r.cgEcef[2]};
    b.velocity = {b.r.velocityEcef[0], b.r.velocityEcef[1], b.r.velocityEcef[2]};
    double off[3];
    m.cgFromOriginBodyM(off);
    const double* T = b.r.bodyToEcef;
    const V offset{T[0] * off[0] + T[1] * off[1] + T[2] * off[2], T[3] * off[0] + T[4] * off[1] + T[5] * off[2],
                   T[6] * off[0] + T[7] * off[1] + T[8] * off[2]};
    b.origin = b.cg - offset;
    return b;
}

} // namespace

TEST_CASE("Physical reset preserves authoritative store inventory and native loaded mass",
          "[stores][mass][world][reset-store-mass]") {
    for(const bool ground:{false,true}) {
        INFO("on ground="<<ground);
        session::World active(options("reset-stores-active")),fresh(options("reset-stores-fresh"));
        session::VehicleSpec spec;spec.name="retained store";spec.type="jsbsim:f16c";
        spec.initial.onGround=ground;spec.initial.altitudeMslM=ground?0:6000;
        spec.initial.airspeedTrueMs=ground?0:220;spec.initial.enginesRunning=!ground;
        spec.initial.headingDeg=90;
        if(!ground){spec.initial.bodyRatesRadS[0]=.1;spec.initial.bodyRatesRadS[1]=-.05;spec.initial.bodyRatesRadS[2]=.02;}
        const auto id=active.createVehicle(spec),reference=fresh.createVehicle(spec);REQUIRE(id);REQUIRE(reference);
        const auto payload=[](session::World& world,std::uint32_t vehicle) {
            for(const auto& p:world.missionSystems(vehicle)->payloads())if(p.family==ms::Family::StoreStation)return p.index;
            FAIL("Native F16 store station is unavailable");return std::uint16_t{0};
        };
        REQUIRE(active.loadStore(id,payload(active,id),3,"mk82")==ms::Reason::None);
        REQUIRE(fresh.loadStore(reference,payload(fresh,reference),3,"mk82")==ms::Reason::None);
        // Reconcile the independent fresh model's just-written real station
        // mass without advancing its IC or applying an external impulse.
        REQUIRE(fresh.model(reference)->beginMassChange());REQUIRE(fresh.model(reference)->endMassChange(nullptr,nullptr));
        active.step(3);
        struct Inventory {std::uint16_t index,payload;int station;std::string store;ms::StoreStatus status;double mass;};
        std::vector<Inventory> inventory;
        for(const auto& row:active.stores(id))inventory.push_back({row.index,row.payload,row.station,row.store,row.status,row.massKg});
        REQUIRE(active.resetVehicle(id));
        const auto retained=active.stores(id);REQUIRE(retained.size()==inventory.size());
        for(std::size_t i=0;i<inventory.size();++i) {
            const auto& expected=inventory[i];const auto& actual=retained[i];
            CHECK(actual.index==expected.index);CHECK(actual.payload==expected.payload);CHECK(actual.station==expected.station);
            CHECK(std::string(actual.store)==expected.store);CHECK(actual.status==expected.status);CHECK(actual.massKg==expected.mass);
        }
        const auto compare=[&] {
            std::map<std::string,double> expected;
            for(const auto& p:fresh.model(reference)->properties("inertia"))if(p.first.rfind("pointmass-",0)==0)expected.emplace(p);
            unsigned checked=0;
            for(const auto& p:active.model(id)->properties("inertia"))if(p.first.rfind("pointmass-",0)==0) {
                const auto found=expected.find(p.first);REQUIRE(found!=expected.end());CHECK(p.second==found->second);++checked;
            }
            REQUIRE(checked==expected.size());REQUIRE(checked>4);
            const auto a=body(*active.model(id)),b=body(*fresh.model(reference));CHECK(std::abs(a.kg-b.kg)<1e-8);
            double acg[3],bcg[3];REQUIRE(active.cgFromOriginBodyM(id,acg));REQUIRE(fresh.cgFromOriginBodyM(reference,bcg));
            for(unsigned k=0;k<3;++k)CHECK(std::abs(acg[k]-bcg[k])<1e-10);
            sim::VehicleState as,bs;active.model(id)->state(as);fresh.model(reference)->state(bs);
            for(unsigned k=0;k<3;++k){CHECK(std::abs(as.positionEcef[k]-bs.positionEcef[k])<1e-7);CHECK(std::abs(as.velocityBodyMs[k]-bs.velocityBodyMs[k])<1e-8);}
            CHECK(as.onGround==bs.onGround);CHECK_FALSE(as.diverged);CHECK_FALSE(as.crashed);
        };
        compare();active.step();fresh.step();compare();
    }
}

TEST_CASE("a mass change: the airframe stays where it is, its CG and state move to what is left (v + w x d), a reaction on it",
          "[damage][stores][mass]") {
    session::World w(options("mass-change"));
    const auto id = wing(w, "mig29a", 6000.0, 220.0);
    w.step(stepsFor(w, 1.0));
    sim::FlightModel& m = *w.model(id);
    auto& j = dynamic_cast<sim::JsbsimModel&>(m);
    // turning (so the part's point moves as more than the CG does)
    const double rates[3] = {0.5, 0.2, -0.1};
    j.setBodyRates(rates);
    const Body before = body(m);
    double cg0[3];
    m.cgFromOriginBodyM(cg0);
    const double* c0 = before.r.cgStructural;
    // 500 kg off beside the left wing's tip (structural frame: x aft, y right, z up), a sideways push on what is left
    const double kg = 500.0, at[3] = {c0[0] + 0.5, c0[1] - 5.0, c0[2] - 0.3}, impulse[3] = {0.0, 2000.0, -500.0};
    REQUIRE(m.beginMassChange());
    REQUIRE(m.removeMass(kg, at));
    REQUIRE(m.endMassChange(nullptr, impulse));
    const Body after = body(m);
    // the airframe where it was
    CHECK((after.origin - before.origin).length() < 1e-6);
    // the CG where the rest's is: d = -m r / (M - m), r the part's place from the CG (body axes: x and z flipped)
    double cg1[3];
    m.cgFromOriginBodyM(cg1);
    const double M = before.kg;
    const double r[3] = {-(at[0] - c0[0]), at[1] - c0[1], -(at[2] - c0[2])};
    for (int k = 0; k < 3; ++k) CHECK(std::abs((cg1[k] - cg0[k]) - (-kg * r[k] / (M - kg))) < 1e-4);
    CHECK(std::abs(after.kg - (M - kg)) < 0.05);
    // its velocity: the new CG's (w x d) and the reaction's, in body axes turned to ECEF
    const V d{cg1[0] - cg0[0], cg1[1] - cg0[1], cg1[2] - cg0[2]};
    const V dvBody = cross(V{rates[0], rates[1], rates[2]}, d) + V{impulse[0], impulse[1], impulse[2]} * (1.0 / after.kg);
    const double* T = before.r.bodyToEcef;
    const V dv{T[0] * dvBody.x + T[1] * dvBody.y + T[2] * dvBody.z, T[3] * dvBody.x + T[4] * dvBody.y + T[5] * dvBody.z,
               T[6] * dvBody.x + T[7] * dvBody.y + T[8] * dvBody.z};
    CHECK(((after.velocity - before.velocity) - dv).length() < 1e-6);
}

TEST_CASE("a MiG-29A's left wing parts with its missiles: it and its piece carry the aircraft's momentum, its airframe does not jump",
          "[damage][stores][mass][world]") {
    session::World w(options("mass-wing"));
    const auto id = wing(w, "mig29a", 6000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "air-to-air") == ms::Reason::None);
    w.step(stepsFor(w, 2.0));
    const Body before = body(*w.model(id));
    REQUIRE(w.separateSection(id, "left wing"));
    std::uint32_t piece = 0;
    for (const auto v : w.vehicleIds())
        if (w.info(v)->role == ms::Role::Debris) piece = v;
    REQUIRE(piece != 0);
    const Body rest = body(*w.model(id)), part = body(*w.model(piece));
    // the mass: the aircraft's, the piece's - its wing and its three missiles (two R-73s, an R-27R)
    CHECK(std::abs(rest.kg + part.kg - before.kg) < 0.1);
    CHECK(part.kg > 600.0);
    // the momentum (the break's spring pushes them apart: nothing from outside)
    const V p0 = before.velocity * before.kg, p1 = rest.velocity * rest.kg + part.velocity * part.kg;
    CHECK((p1 - p0).length() < 1e-5 * p0.length());
    // the airframe where it was, and their centre of mass where the aircraft's was
    CHECK((rest.origin - before.origin).length() < 1e-4);
    const V centre = (rest.cg * rest.kg + part.cg * part.kg) * (1.0 / (rest.kg + part.kg));
    CHECK((centre - before.cg).length() < 0.01);
    // the stores went with it: the aircraft's lost, none released
    int lost = 0;
    for (const auto& s : w.stores(id)) lost += s.status == ms::StoreStatus::Lost ? 1 : 0;
    CHECK(lost == 3);
}

TEST_CASE("a loaded departing section conserves linear and angular momentum at its actual CG",
          "[damage][stores][mass][world][loaded-conservation]") {
    session::World w(options("mass-loaded-angular"));
    const auto id = wing(w, "mig29a", 6000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "air-to-air") == ms::Reason::None);
    w.step(stepsFor(w, 2.0));
    // MassBalance precedes propulsion in a native tick. Reconcile fuel already
    // burnt by the last tick before defining this zero-integration baseline.
    REQUIRE(w.model(id)->beginMassChange());
    REQUIRE(w.model(id)->endMassChange(nullptr,nullptr));
    // Zero initial spin isolates the internal separation impulse. No native
    // integration occurs between these snapshots: external loads do no work.
    const double zeroRates[3]{};
    dynamic_cast<sim::JsbsimModel&>(*w.model(id)).setBodyRates(zeroRates);
    const Body before = body(*w.model(id));
    const auto* coordinateSource = dynamic_cast<const sim::NativePositionSourceV1*>(w.model(id));
    REQUIRE(coordinateSource != nullptr);
    sim::NativePositionV1 nativePosition;
    REQUIRE(coordinateSource->nativePositionV1(before.r.cgEcef,nativePosition));
    sim::VehicleState nativeState;
    w.model(id)->state(nativeState);
    CHECK(std::abs(nativePosition.latitudeRad-nativeState.latitudeRad) < 1e-14);
    CHECK(std::abs(nativePosition.longitudeRad-nativeState.longitudeRad) < 1e-14);
    CHECK(std::abs(nativePosition.altitudeMslM-nativeState.altitudeMslM) < 1e-8);
    const auto angularMomentum = [&](sim::FlightModel& model, const Body& value, V frameVelocity) {
        const auto* source = dynamic_cast<const sim::RigidMassSourceV1*>(&model);
        REQUIRE(source != nullptr);
        sim::RigidMassPropertiesV1 mass;
        REQUIRE(source->rigidMassPropertiesV1(mass));
        const auto* inertia = mass.inertiaBodyKgM2;
        const auto* rate = value.r.ratesBody;
        const V spinBody{inertia[0]*rate[0]+inertia[1]*rate[1]+inertia[2]*rate[2],
                         inertia[3]*rate[0]+inertia[4]*rate[1]+inertia[5]*rate[2],
                         inertia[6]*rate[0]+inertia[7]*rate[1]+inertia[8]*rate[2]};
        const auto* r = value.r.bodyToEcef;
        const V spin{r[0]*spinBody.x+r[1]*spinBody.y+r[2]*spinBody.z,
                     r[3]*spinBody.x+r[4]*spinBody.y+r[5]*spinBody.z,
                     r[6]*spinBody.x+r[7]*spinBody.y+r[8]*spinBody.z};
        return cross(value.cg-before.cg,(value.velocity-frameVelocity)*value.kg)+spin;
    };
    // Use the frame translating with the original CG. Subtracting a common
    // velocity does not change conservation when mass, first moment and linear
    // momentum are conserved; those are checked separately below. In the fixed
    // Earth frame a nanometre of coordinate roundoff multiplies the aircraft's
    // entire bulk momentum, obscuring the much smaller separation impulse.
    const V initialAngular = angularMomentum(*w.model(id),before,before.velocity);
    const V initialEarthAngular = angularMomentum(*w.model(id),before,{});
    REQUIRE(w.separateSection(id, "left wing"));
    std::uint32_t piece = 0;
    for (const auto candidate : w.vehicleIds())
        if (w.info(candidate)->role == ms::Role::Debris) piece = candidate;
    REQUIRE(piece != 0);
    const Body rest = body(*w.model(id)), part = body(*w.model(piece));
    REQUIRE(part.kg > 600);
    REQUIRE((part.cg-part.origin).length() > .01);
    const V initialLinear = before.velocity*before.kg;
    const V finalLinear = rest.velocity*rest.kg+part.velocity*part.kg;
    const V finalAngular = angularMomentum(*w.model(id),rest,before.velocity)+
                           angularMomentum(*w.model(piece),part,before.velocity);
    const V finalEarthAngular = angularMomentum(*w.model(id),rest,{})+angularMomentum(*w.model(piece),part,{});
    const V firstMoment = (rest.cg-before.cg)*rest.kg+(part.cg-before.cg)*part.kg;
    const V centreResidual = firstMoment*(1/(rest.kg+part.kg));
    const auto print = [](const char* name, V value) {
        INFO(name << " = " << value.x << ", " << value.y << ", " << value.z);
        std::printf("LOADED-CONSERVATION %s %.17g %.17g %.17g\n",name,value.x,value.y,value.z);
    };
    std::printf("LOADED-CONSERVATION mass before %.17g rest %.17g part %.17g residual %.17g\n",
                before.kg,rest.kg,part.kg,rest.kg+part.kg-before.kg);
    print("centre_residual",centreResidual);
    print("rest_origin_residual",rest.origin-before.origin);
    print("initial_velocity",before.velocity);
    print("rest_velocity_delta",rest.velocity-before.velocity);
    print("part_velocity_delta",part.velocity-before.velocity);
    print("initial_rates",{before.r.ratesBody[0],before.r.ratesBody[1],before.r.ratesBody[2]});
    print("rest_rates",{rest.r.ratesBody[0],rest.r.ratesBody[1],rest.r.ratesBody[2]});
    print("part_rates",{part.r.ratesBody[0],part.r.ratesBody[1],part.r.ratesBody[2]});
    print("linear_residual",finalLinear-initialLinear);
    print("angular_residual",finalAngular-initialAngular);
    print("earth_frame_angular_residual",finalEarthAngular-initialEarthAngular);
    print("earth_frame_translation_term",cross(firstMoment,before.velocity));
    for (const auto candidate : {id,piece}) {
        sim::VehicleState state;
        w.model(candidate)->state(state);
        const auto* r=state.rotationBodyToEcef;
        const auto* v=state.velocityBodyMs;
        const V fromBody{r[0]*v[0]+r[1]*v[1]+r[2]*v[2],r[3]*v[0]+r[4]*v[1]+r[5]*v[2],r[6]*v[0]+r[7]*v[1]+r[8]*v[2]};
        print(candidate==id ? "rest_native_velocity_consistency" : "part_native_velocity_consistency",
              fromBody-body(*w.model(candidate)).velocity);
    }
    CAPTURE((finalLinear-initialLinear).length(), (finalAngular-initialAngular).length());
    CHECK((finalLinear-initialLinear).length() < .001);
    CHECK((finalAngular-initialAngular).length() < .001);
    CHECK(std::abs(rest.kg+part.kg-before.kg) < 1e-8);
    CHECK(centreResidual.length() < 1e-8);
}

TEST_CASE("stores jettisoned: the carrier's airframe flies on without a jump, though its CG moved", "[stores][mass][world]") {
    session::World w(options("mass-release"));
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "mk82 x4") == ms::Reason::None);
    w.step(stepsFor(w, 1.0));
    std::uint16_t station = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) station = pl.index;
    ms::JettisonCommand all;
    all.payload = station;
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    // the origin's path step by step: its second difference its acceleration's (under a millimetre a step), not the CG's
    // shift (centimetres): what the ejectors push it by aside
    std::vector<V> path;
    double cgMoved = 0.0, last[3];
    w.model(id)->cgFromOriginBodyM(last);
    for (int k = 0; k < 40; ++k) {
        w.step(1);
        path.push_back(body(*w.model(id)).origin);
        double cg[3];
        w.model(id)->cgFromOriginBodyM(cg);
        cgMoved = std::max(cgMoved, std::sqrt((cg[0] - last[0]) * (cg[0] - last[0]) + (cg[1] - last[1]) * (cg[1] - last[1]) +
                                              (cg[2] - last[2]) * (cg[2] - last[2])));
        for (int a = 0; a < 3; ++a) last[a] = cg[a];
    }
    CHECK(cgMoved > 0.03); // (four Mk 82s off: the CG moved centimetres)
    double worst = 0.0;
    for (std::size_t k = 2; k < path.size(); ++k) worst = std::max(worst, (path[k] - path[k - 1] * 2.0 + path[k - 2]).length());
    if (std::getenv("FSIM_MASS_SHOW")) std::printf("the CG moved %.4f m; the origin's worst second difference %.5f m\n", cgMoved, worst);
    // (the ejectors' reaction changes its velocity at once - some 2.3 mm a step - and its acceleration adds under one:
    // well under a tenth of the CG's shift, which the airframe jumped by before)
    CHECK(worst < 0.1 * cgMoved);
}

TEST_CASE("an F-16's external tanks feed its own as it burns - fuel moved, none made or lost; jettisoned, each flies on with what was left in it",
          "[stores][mass][fuel][world]") {
    session::World w(options("mass-tanks"));
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(1);
    auto external = [&] {
        double kg = 0.0;
        for (const auto& s : w.stores(id))
            if (s.status == ms::StoreStatus::Loaded) kg += s.fuelKg;
        return kg;
    };
    const sim::EnergyOnBoard e0 = w.model(id)->energy();
    const double x0 = external(), m0 = body(*w.model(id)).kg;
    CHECK(std::abs(x0 - 0.8 * (2 * 370 + 300) * 3.785411784) < 0.5); // (three tanks full: the cards' 0.80 kg/L)
    w.step(stepsFor(w, 30.0));
    const sim::EnergyOnBoard e1 = w.model(id)->energy();
    const double x1 = external(), m1 = body(*w.model(id)).kg;
    const double burnt = (e0.fuelKg + x0) - (e1.fuelKg + x1);
    if (std::getenv("FSIM_MASS_SHOW"))
        std::printf("internal %.2f -> %.2f of %.2f kg, external %.2f -> %.2f kg, mass %.2f -> %.2f kg\n", e0.fuelKg, e1.fuelKg, e1.fuelCapacityKg, x0, x1,
                    m0, m1);
    CHECK(burnt > 1.0);                           // (its engine burnt some)
    CHECK(x1 < x0 - 1.0);                         // the tanks gave it ...
    CHECK(e1.fuelCapacityKg - e1.fuelKg < 1.0);   // ... its own kept topped up (it started full) ...
    CHECK(std::abs((m0 - m1) - burnt) < 0.05);    // ... and the aircraft is lighter by the fuel burnt alone
    // the wing tanks drawn alike (a symmetric load stays so)
    double left = -1.0, right = -2.0;
    for (const auto& s : w.stores(id)) {
        if (s.station == 4) left = s.fuelKg;
        if (s.station == 6) right = s.fuelKg;
    }
    CHECK(std::abs(left - right) < 1e-6);
    // jettisoned: each tank is a vehicle of its own, as heavy as it was on its station; the aircraft that much lighter
    std::vector<double> carried;
    for (const auto& s : w.stores(id)) carried.push_back(s.massKg);
    std::uint16_t station = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) station = pl.index;
    ms::JettisonCommand all;
    all.payload = station;
    all.emergency = true;
    const double before = body(*w.model(id)).kg;
    REQUIRE(w.submit(id, all).accepted());
    w.step(stepsFor(w, 0.5));
    double gone = 0.0;
    std::vector<std::uint32_t> tanks;
    const auto stores = w.stores(id);
    for (std::size_t k = 0; k < stores.size(); ++k) {
        const auto& s = stores[k];
        REQUIRE(s.status == ms::StoreStatus::Jettisoned);
        REQUIRE(s.weapon != 0);
        const double kg = body(*w.model(s.weapon)).kg;
        CHECK(std::abs(kg - carried[k]) < 0.05);
        gone += kg;
        tanks.push_back(s.weapon);
    }
    const double after = body(*w.model(id)).kg;
    CHECK(std::abs((before - after) - gone) < 0.5); // (and half a second's burn)
    // it falls away below the aircraft (some 40 m in 3 s), behind it as its drag slows it
    w.step(stepsFor(w, 3.0));
    const Body tank = body(*w.model(tanks[0])), plane = body(*w.model(id));
    const double drop = plane.cg.length() - tank.cg.length();
    const V ahead = plane.velocity * (1.0 / plane.velocity.length());
    const V apart = plane.cg - tank.cg;
    const double behind = apart.x * ahead.x + apart.y * ahead.y + apart.z * ahead.z;
    if (std::getenv("FSIM_MASS_SHOW")) std::printf("3 s after: the tank %.1f m below the aircraft, %.1f m behind it\n", drop, behind);
    CHECK(drop > 20.0);
    CHECK(behind > 1.0);
}

TEST_CASE("the viewer's picture: an airframe hung from its model's origin has each store on its station where the simulation "
          "releases it - and the published sample says where that origin is",
          "[stores][mass][world][viewer]") {
    // what the viewer draws (app/viewer_main.cpp, drawnState): each model's origin at its state's position less its
    // CG's place from the origin (ipc::originEcef, the sample's `derived`), each carried store's model at its
    // aircraft's manifest's `station` line. Released, the store is a vehicle of its own: it must be where it hung.
    session::WorldOptions o = options("mass-picture");
    o.publish = true;
    session::World w(o);
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "mk82 x4") == ms::Reason::None);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(stepsFor(w, 1.0));
    // loaded, its CG is not at its origin: decimetres
    double cg[3];
    w.model(id)->cgFromOriginBodyM(cg);
    CHECK(std::sqrt(cg[0] * cg[0] + cg[1] * cg[1] + cg[2] * cg[2]) > 0.1);
    // the sample the viewer reads says so, and its origin is the model's
    w.publishNow();
    ipc::WorldMirror mirror;
    REQUIRE(mirror.open("mass-picture"));
    sim::SnapshotBatch batch;
    REQUIRE(mirror.pollSnapshot(batch));
    REQUIRE(!mirror.status().empty());
    const double* d = mirror.status()[0].derived;
    for (int k = 0; k < 3; ++k) CHECK(std::abs(d[ipc::kDerivedCgFromOriginM + static_cast<std::size_t>(k)] - cg[k]) < 1e-9);
    double origin[3];
    ipc::originEcef(batch.states[0], d, origin);
    const Body before = body(*w.model(id));
    CHECK((V{origin[0], origin[1], origin[2]} - before.origin).length() < 1e-6);

    // the stations' places as the viewer has them: its model's manifest
    std::map<int, V> stations;
    for (const auto& dir : io::AssetResolver{}.aircraftDirs()) {
        std::ifstream in(dir / "f16c" / "f16c.glb.manifest");
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string key;
            int number = 0;
            V at;
            if (fields >> key >> number >> at.x >> at.y >> at.z && key == "station") stations.emplace(number, at);
        }
        if (!stations.empty()) break;
    }
    REQUIRE(stations.size() == 9);

    std::uint16_t payload = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) payload = pl.index;
    ms::JettisonCommand all;
    all.payload = payload;
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    std::map<std::uint32_t, bool> seen;
    int released = 0;
    double worst = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 4.0) && released < 5; ++k) {
        w.step(1);
        for (const auto& rec : w.weapons()) {
            if (rec.launcher != id || seen[rec.id]) continue;
            seen[rec.id] = true;
            ++released;
            // the store's origin from its aircraft's, in the aircraft's body axes, the step it became a vehicle
            const Body a = body(*w.model(id)), s = body(*w.model(rec.id));
            const V r = s.origin - a.origin;
            const double* T = a.r.bodyToEcef;
            const V at{T[0] * r.x + T[3] * r.y + T[6] * r.z, T[1] * r.x + T[4] * r.y + T[7] * r.z, T[2] * r.x + T[5] * r.y + T[8] * r.z};
            REQUIRE(stations.count(rec.station));
            INFO("station " << rec.station << " (" << rec.store << "): " << at.x << ", " << at.y << ", " << at.z);
            // within what its ejector moved it in that step (some 5 m/s for a 120th of a second) and a centimetre
            const double off = (at - stations.at(rec.station)).length();
            CHECK(off < 0.08);
            worst = std::max(worst, off);
        }
    }
    CHECK(released == 5); // (two bombs, three tanks: the tanks' loadout had the inboard stations first)
    // (hung from its CG instead - as the viewer drew it before - each would have left its station's picture by the
    // CG's place from the origin as it was loaded: the decimetres checked above)
    if (std::getenv("FSIM_MASS_SHOW")) std::printf("stores from their stations as they leave: %.4f m at most\n", worst);
}

TEST_CASE("a missile's model origin is its launch CG: full, its CG is at its origin; as its motor burns its CG moves ahead of it",
          "[stores][mass][world][viewer]") {
    // its file's empty CG is its structure's, burnt out - some decimetres ahead of its CG as it hangs on a station.
    // Its picture's origin and what a station holds is its launch CG, which its file says (fsim/model/origin-x-in:
    // hangar's storebodies.py): a missile jettisoned full is drawn where it is, not a third of a metre off
    session::World w(options("mass-missile-origin"));
    const auto lit = wing(w, "aim120c", 8000.0, 250.0);
    // A standalone flight-model fixture is an aircraft endpoint whose neutral
    // default commands zero throttle. Request ignition explicitly; model load
    // itself must not burn propellant, and changing neutral defaults is invalid.
    control::ActuatorCommand ignition;
    ignition.throttle = 1;
    REQUIRE(w.submit(lit, ignition).accepted());
    const double initialFuel = w.vehicleState(lit)->fuelKg;
    double cg[3];
    w.model(lit)->cgFromOriginBodyM(cg);
    CHECK(std::abs(cg[0]) < 0.002);
    CHECK(std::abs(cg[1]) < 1e-6);
    CHECK(std::abs(cg[2]) < 1e-6);
    w.step(stepsFor(w, 10.0)); // (its 8 s of burn, and on)
    REQUIRE(w.vehicleState(lit)->fuelKg < initialFuel);
    w.model(lit)->cgFromOriginBodyM(cg);
    // burnt out: its structure's CG, 1.497 m from its nose where its launch CG was 1.83 m (its design's and its grain's)
    CHECK(cg[0] > 0.31);
    CHECK(cg[0] < 0.36);
    // a tank's, a bomb's: its CG is its origin (nothing burns)
    for (const char* type : {"tank370gal", "mk82", "463lpallet"}) {
        const auto id = wing(w, type, 6000.0, 200.0, 2);
        w.model(id)->cgFromOriginBodyM(cg);
        INFO(type);
        CHECK(std::sqrt(cg[0] * cg[0] + cg[1] * cg[1] + cg[2] * cg[2]) < 0.002);
        w.removeVehicle(id);
        w.step();
    }
}

TEST_CASE("a load is held to the aircraft's maximum take-off weight: what would take it past is refused, nothing changed; "
          "a bomber carries its bombs",
          "[stores][mass][world]") {
    session::World w(options("mass-most"));
    // a C-17A as it spawns: 235,996 kg of its 265,352 kg. Eighteen 463L pallets (40,824 kg) would take it past: refused
    // as a whole, none loaded. Its airdrop bundles (17,964 kg) fit.
    const auto c17 = wing(w, "c17a", 6000.0, 200.0);
    w.step(stepsFor(w, 0.5));
    REQUIRE(w.profile(c17) != nullptr);
    const double most = w.profile(c17)->envelope.maxTakeoffKg;
    CHECK(std::abs(most - 265352.0) < 1.0);
    const double spawned = w.massKg(c17), full = w.navigationReport(c17).fuelKg;
    REQUIRE(spawned + 40824.0 > most);
    CHECK(w.loadLoadout(c17, "pallets") == ms::Reason::OverMaxWeight);
    CHECK(std::string(ms::reasonName(ms::Reason::OverMaxWeight)) == "over_max_weight");
    for (const auto& s : w.stores(c17)) CHECK(std::string(s.store).empty());
    REQUIRE(w.loadLoadout(c17, "airdrop") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    CHECK(std::abs(w.massKg(c17) - spawned - 17964.0) < 30.0); // (less the fuel of a second)
    // station by station to the limit: pallets go on until the next would pass it (two loads between steps are both
    // counted: the flight model has not weighed the first yet)
    REQUIRE(!w.stores(c17).empty());
    const std::uint16_t hold = w.stores(c17).front().payload;
    int taken = 0;
    for (int station = 1; station <= 18; ++station) { // (each pallet replaces a 998 kg bundle: 1,270 kg more)
        const ms::Reason r = w.loadStore(c17, hold, station, "463l-pallet");
        if (r == ms::Reason::None) ++taken;
        else CHECK(r == ms::Reason::OverMaxWeight);
    }
    w.step(stepsFor(w, 0.5));
    CHECK(w.massKg(c17) <= most + 1.0);
    CHECK(w.massKg(c17) > most - 1300.0);
    CHECK(taken > 0);
    CHECK(taken < 18);
    // with half its fuel it takes all eighteen: what a start's tanks hold is the scenario's to say
    // (InitialConditions::fuelFraction - each tank that fraction of what it holds)
    const auto light = wing(w, "c17a", 6000.0, 200.0, 2, 0.5);
    w.step(stepsFor(w, 0.5));
    const auto nav = w.navigationReport(light);
    CHECK(std::abs(nav.fuelKg - 0.5 * nav.capacity) < 0.001 * nav.capacity);
    CHECK(std::abs((spawned - w.massKg(light)) - (full - nav.fuelKg)) < 30.0); // (lighter by the fuel it has not)
    REQUIRE(w.loadLoadout(light, "pallets") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    CHECK(w.massKg(light) < most);
    REQUIRE(w.resetVehicle(light)); // (a reset is to the same start: its tanks as they were given)
    w.step(stepsFor(w, 0.5));
    CHECK(std::abs(w.navigationReport(light).fuelKg - nav.fuelKg) < 30.0);

    // the B-52H: six bombs on each pylon and eight on its bay's launcher, 4,536 kg - one pylon's drag a pylon
    const auto buff = wing(w, "b52h", 6000.0, 220.0);
    w.step(stepsFor(w, 0.5));
    const double clean = w.massKg(buff);
    REQUIRE(w.loadLoadout(buff, "mk82 x20") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    CHECK(std::abs(w.massKg(buff) - clean - 20.0 * 226.8) < 60.0);
    int carried = 0;
    for (const auto& s : w.stores(buff)) carried += std::string(s.store) == "mk82" ? 1 : 0;
    CHECK(carried == 20);
}

TEST_CASE("F16 sequential nose tail and wing separation leaves physical positive inertia", "[mass-partition][mass][damage][world]") {
    session::World world(options("f16-mass-partition"));
    const auto id=wing(world,"f16c",6000.,250.);
    REQUIRE(world.model(id)!=nullptr);
    REQUIRE(world.model(id)->beginMassChange());
    REQUIRE(world.model(id)->endMassChange(nullptr,nullptr));
    const auto positive=[](const double* j) {
        for(unsigned k=0;k<9;++k)if(!std::isfinite(j[k]))return false;
        const double determinant=j[0]*(j[4]*j[8]-j[5]*j[7])-j[1]*(j[3]*j[8]-j[5]*j[6])+j[2]*(j[3]*j[7]-j[4]*j[6]);
        return j[0]>0&&j[4]>0&&j[8]>0&&j[0]*j[4]-j[1]*j[3]>0&&
            j[0]*j[8]-j[2]*j[6]>0&&j[4]*j[8]-j[5]*j[7]>0&&determinant>0;
    };
    for(const auto* section:{"nose","tail","left wing","right wing"}) {
        INFO(section);
        REQUIRE(world.separateSection(id,section));
        for(const auto entity:world.vehicleIds()) {
            const auto* source=dynamic_cast<const sim::RigidMassSourceV1*>(world.model(entity));
            REQUIRE(source!=nullptr);
            sim::RigidMassPropertiesV1 properties;
            REQUIRE(source->rigidMassPropertiesV1(properties));
            INFO("entity="<<entity<<" mass="<<properties.massKg<<" Ixx="<<properties.inertiaBodyKgM2[0]
                <<" Iyy="<<properties.inertiaBodyKgM2[4]<<" Izz="<<properties.inertiaBodyKgM2[8]);
            CHECK(properties.massKg>0);
            CHECK(positive(properties.inertiaBodyKgM2));
        }
    }
}

TEST_CASE("Queued release material leaves with a lost station without a duplicate weapon",
          "[stores][mass][world][queued-station-loss]") {
    // A same-activity surviving-station control proves that only the lost
    // material's queued Release is removed, not every committed queue member.
    for (const bool retainedQueuedStore : {false, true}) {
        INFO("same-activity surviving queued station=" << retainedQueuedStore);
        session::World world(options("queued-station-loss"));
        const auto parent = wing(world, "f16c", 6000, 220);
        auto* host = world.missionSystems(parent);
        REQUIRE(host);
        std::uint16_t payload = 0;
        bool found = false;
        for (const auto& row : host->payloads()) if (row.family == ms::Family::StoreStation) {
            payload = row.index;
            found = true;
        }
        REQUIRE(found);
        REQUIRE(world.loadStore(parent, payload, 8, "aim-9x") == ms::Reason::None);
        if (retainedQueuedStore) REQUIRE(world.loadStore(parent, payload, 1, "aim-9x") == ms::Reason::None);
        std::string error;
        const auto geometry = ms::damage::readAirframe(
            std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "f16c/f16c.damage.json", error);
        INFO(error);
        REQUIRE(geometry);
        const int sectionIndex = geometry->section("right wing");
        REQUIRE(sectionIndex >= 0);
        const auto& section = geometry->sections[static_cast<std::size_t>(sectionIndex)];
        const auto station = std::find(section.stations.begin(), section.stations.end(), 8);
        REQUIRE(station != section.stations.end());
        REQUIRE(std::find(section.stations.begin(), section.stations.end(), 1) == section.stations.end());
        const auto stationOffset = static_cast<std::size_t>(station - section.stations.begin());
        double lostMass = 0;
        std::uint16_t lostIndex = 0, retainedIndex = 0;
        ms::JettisonCommand command;
        command.payload = payload;
        command.emergency = true;
        for (const auto& row : host->stores()) if (std::string(row.store) == "aim-9x") {
            REQUIRE(std::string(row.carriage) == "rail");
            command.stores.push_back(row.index);
            if (row.station == 8) { lostMass = row.massKg; lostIndex = row.index; }
            else { REQUIRE(row.station == 1); retainedIndex = row.index; }
        }
        REQUIRE(lostMass > 0);
        REQUIRE(command.stores.size() == (retainedQueuedStore ? 2u : 1u));
        // Define a current-mass zero-integration baseline exactly as the
        // existing loaded-section conservation fixture does. No warmup step.
        auto& model = *world.model(parent);
        REQUIRE(model.beginMassChange());
        REQUIRE(model.endMassChange(nullptr, nullptr));
        const double beforeMass = model.energy().massKg;
        double sectionFuel = 0;
        for (const auto& tank : section.tanks) {
            REQUIRE(tank.index >= 0);
            const auto contents = model.property("propulsion/tank[" + std::to_string(tank.index) + "]/contents-lbs");
            REQUIRE(contents.valid());
            sectionFuel += contents.get() * .45359237;
        }
        const auto submission = world.submit(parent, command);
        REQUIRE(submission.accepted());
        REQUIRE(world.weapons().empty());
        const auto statusAt = [&](std::uint16_t index) {
            const auto& stores = host->stores();
            const auto foundStore = std::find_if(stores.begin(), stores.end(), [&](const auto& row) { return row.index == index; });
            REQUIRE(foundStore != stores.end());
            return foundStore->status;
        };
        REQUIRE(statusAt(lostIndex) == ms::StoreStatus::Releasing);
        REQUIRE(host->stationMassKg(payload, 8) == lostMass);
        REQUIRE(host->releasing());
        const double epoch = world.simTime();
        REQUIRE(world.separateSection(parent, "right wing"));
        REQUIRE(world.simTime() == epoch);
        REQUIRE(statusAt(lostIndex) == ms::StoreStatus::Lost);
        REQUIRE(host->stationMassKg(payload, 8) == 0);
        const auto* activity = host->activity(submission.activity);
        REQUIRE(activity);
        CHECK(activity->state == control::ActivityState::Failed);
        CHECK(activity->reason == ms::Reason::PayloadFailed);
        REQUIRE(host->releasing() == retainedQueuedStore);
        if (retainedQueuedStore) {
            REQUIRE(statusAt(retainedIndex) == ms::StoreStatus::Releasing);
            REQUIRE(host->stationMassKg(payload, 1) == lostMass);
        }
        std::uint32_t piece = 0;
        for (const auto id : world.vehicleIds()) {
            const auto* info = world.info(id);
            if (info->role == ms::Role::Debris && info->launcher == parent && info->type.find(":right wing:") != std::string::npos) {
                REQUIRE(piece == 0);
                piece = id;
            }
        }
        REQUIRE(piece != 0);
        const auto carried = world.model(piece)->property("inertia/pointmass-weight-lbs[" + std::to_string(stationOffset + 1) + "]");
        REQUIRE(carried.valid());
        const double epsilon = std::numeric_limits<double>::epsilon();
        CHECK(std::abs(carried.get() * .45359237 - lostMass) <= 512 * epsilon * std::max(1., lostMass));
        const double departedMass = section.massKg + sectionFuel + lostMass;
        CHECK(std::abs(beforeMass - model.energy().massKg - departedMass) <= 512 * epsilon * std::max(1., beforeMass));
        REQUIRE(world.weapons().empty()); // The real piece owns the store; no weapon child exists yet.
        world.step();
        const auto weapons = world.weapons();
        REQUIRE(weapons.size() == (retainedQueuedStore ? 1u : 0u));
        for (const auto& weapon : weapons) {
            CHECK(weapon.launcher == parent);
            CHECK(weapon.store == "aim-9x");
            CHECK(weapon.station == 1);
            CHECK(weapon.jettisoned);
        }
        CHECK(statusAt(lostIndex) == ms::StoreStatus::Lost);
        CHECK(host->stationMassKg(payload, 8) == 0);
        CHECK_FALSE(host->releasing());
        activity = host->activity(submission.activity);
        REQUIRE(activity);
        CHECK(activity->state == control::ActivityState::Failed);
        CHECK(activity->reason == ms::Reason::PayloadFailed);
    }
}


// Exact identity of an empty supported native mass transaction. This is a
// numerical-method witness, independent of World reset, damage or geometry.
#include <models/FGMassBalance.h>
#include <cstring>

TEST_CASE("An exact-zero native mass transaction preserves rigid state bits",
          "[mass][native-mass-zero-identity]") {
    for (const unsigned warmupSteps : {0u, 5u}) {
        for (const bool explicitZeroImpulse : {false, true}) {
            INFO("native warmup steps=" << warmupSteps << " explicit zero impulse=" << explicitZeroImpulse);
            session::World world(options("exact-zero-native-mass"));
            session::VehicleSpec spec;
            spec.name = "nontrivial rigid identity witness";
            spec.type = "jsbsim:f16c";
            spec.initial.latitudeDeg = 37.3;
            spec.initial.longitudeDeg = -122.1;
            spec.initial.altitudeMslM = 6000;
            spec.initial.airspeedTrueMs = 220;
            spec.initial.headingDeg = 123;
            spec.initial.pitchDeg = 7;
            spec.initial.rollDeg = -18;
            spec.initial.bodyRatesRadS[0] = .31;
            spec.initial.bodyRatesRadS[1] = -.27;
            spec.initial.bodyRatesRadS[2] = .19;
            const auto id = world.createVehicle(spec);
            REQUIRE(id);
            auto* native = dynamic_cast<sim::JsbsimModel*>(world.model(id));
            REQUIRE(native);
            sim::ControlInputs controls;
            controls.aileron = .11;
            controls.elevator = -.07;
            controls.rudder = .03;
            controls.setThrottleAll(.3);
            for (unsigned step = 0; step < warmupSteps; ++step) native->step(controls);
            // Synchronize native mass inputs first. In particular, a late fuel
            // publication is not misclassified as an empty end operation.
            REQUIRE(native->beginMassChange());
            const auto snapshot = [&]() {
                std::vector<std::pair<std::string, double>> values;
                const auto propagation = native->exec().GetPropagate();
                const auto balance = native->exec().GetMassBalance();
                const auto& state = propagation->GetVState();
                const auto vector = [&](const char* name, const auto& value) {
                    for (unsigned axis = 1; axis <= 3; ++axis)
                        values.emplace_back(std::string(name) + std::to_string(axis), value(axis));
                };
                vector("location", state.vLocation);
                vector("bodyVelocity", state.vUVW);
                vector("bodyRate", state.vPQR);
                vector("inertialBodyRate", state.vPQRi);
                vector("inertialVelocity", state.vInertialVelocity);
                vector("inertialPosition", state.vInertialPosition);
                vector("cachedNedVelocity", propagation->GetVel());
                vector("structuralCg", balance->GetXYZcg());
                values.emplace_back("mass", balance->GetMass());
                for (unsigned element = 1; element <= 4; ++element) {
                    values.emplace_back("localQuaternion" + std::to_string(element), state.qAttitudeLocal(element));
                    values.emplace_back("inertialQuaternion" + std::to_string(element), state.qAttitudeECI(element));
                    values.emplace_back("quaternionDerivative" + std::to_string(element), state.vQtrndot(element));
                }
                for (unsigned row = 1; row <= 3; ++row) for (unsigned column = 1; column <= 3; ++column) {
                    const auto index = std::to_string(row) + std::to_string(column);
                    values.emplace_back("inertia" + index, balance->GetJ()(row, column));
                    values.emplace_back("bodyToLocal" + index, propagation->GetTb2l()(row, column));
                    values.emplace_back("bodyToInertial" + index, propagation->GetTb2i()(row, column));
                }
                sim::VehicleState sdk;
                native->state(sdk);
                for (unsigned axis = 0; axis < 3; ++axis) {
                    const auto index = std::to_string(axis);
                    values.emplace_back("sdkPosition" + index, sdk.positionEcef[axis]);
                    values.emplace_back("sdkBodyVelocity" + index, sdk.velocityBodyMs[axis]);
                    values.emplace_back("sdkNedVelocity" + index, sdk.velocityNedMs[axis]);
                    values.emplace_back("sdkBodyRate" + index, sdk.angularRateBodyRadS[axis]);
                }
                values.emplace_back("simulationTime", sdk.simTime);
                values.emplace_back("stepCount", static_cast<double>(sdk.stepCount));
                return values;
            };
            const auto before = snapshot();
            const double zeroImpulse[3] = {0, 0, 0};
            REQUIRE(native->endMassChange(nullptr, explicitZeroImpulse ? zeroImpulse : nullptr));
            const auto after = snapshot();
            REQUIRE(after.size() == before.size());
            for (std::size_t index = 0; index < before.size(); ++index) {
                INFO(before[index].first << " before=" << before[index].second << " after=" << after[index].second);
                REQUIRE(before[index].first == after[index].first);
                REQUIRE(std::isfinite(before[index].second));
                REQUIRE(std::isfinite(after[index].second));
                CHECK(std::memcmp(&before[index].second, &after[index].second, sizeof(double)) == 0);
            }
        }
    }
}


// Independent parallel-axis reference; no mb->GetPointmassInertia or native
// CalculateTankInertias supplies this expected tensor.
#include <models/FGPropulsion.h>
#include <models/propulsion/FGTank.h>
#include <array>
#include <limits>

TEST_CASE("Synchronous native mass reconciliation uses one CG for tank and whole inertia",
          "[mass][native-mass-tank-epoch]") {
    session::World world(options("native-mass-tank-epoch"));
    const auto id = wing(world, "f16c", 6000, 220);
    auto* native = dynamic_cast<sim::JsbsimModel*>(world.model(id));
    REQUIRE(native);
    const auto balance = native->exec().GetMassBalance();
    const auto propulsion = native->exec().GetPropulsion();
    REQUIRE(propulsion->GetNumTanks() > 0);
    REQUIRE(propulsion->GetNumTanks() <= std::numeric_limits<unsigned>::max());
    unsigned selected = static_cast<unsigned>(propulsion->GetNumTanks());
    for (unsigned index = 0; index < propulsion->GetNumTanks(); ++index)
        if (propulsion->GetTank(index)->GetContents() > 0) { selected = index; break; }
    REQUIRE(selected < propulsion->GetNumTanks());
    const auto oldCg = balance->GetXYZcg();
    const auto tank = propulsion->GetTank(selected);
    // A deliberate unit-only mass input change, not fuel burn, native time
    // advancement or a change to an aircraft file/encounter acceptance scene.
    tank->SetContents(tank->GetContents() * .5);
    REQUIRE(native->beginMassChange());
    const auto newCg = balance->GetXYZcg();
    CHECK((newCg - oldCg).Magnitude() > 0);
    const auto independentTank = [&](const auto& cg) {
        std::array<long double, 9> result{};
        for (unsigned index = 0; index < propulsion->GetNumTanks(); ++index) {
            const auto item = propulsion->GetTank(index);
            const auto position = item->GetXYZ();
            const long double mass = static_cast<long double>(item->GetContents()) * (1.0 / 32.174049);
            const long double r[3] = {(static_cast<long double>(cg(1)) - position(1)) / 12,
                                     (static_cast<long double>(position(2)) - cg(2)) / 12,
                                     (static_cast<long double>(cg(3)) - position(3)) / 12};
            const long double square = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
            for (unsigned row = 0; row < 3; ++row) for (unsigned column = 0; column < 3; ++column)
                result[3*row+column] += mass*((row == column ? square : 0)-r[row]*r[column]);
            result[0] += item->GetIxx(); result[4] += item->GetIyy(); result[8] += item->GetIzz();
        }
        return result;
    };
    const auto oldReference = independentTank(oldCg);
    const auto expected = independentTank(newCg);
    const auto beforeJ = balance->GetJ();
    const auto suppliedTank = balance->in.TankInertia;
    const long double epsilon = std::numeric_limits<double>::epsilon();
    // Numerical reference tolerances below apply ONLY to this independently
    // reordered tensor comparison, not the original 78 exact identity checks.
    long double largest = 1;
    for (const auto value : expected) largest = std::max(largest, std::abs(value));
    for (const auto value : oldReference) largest = std::max(largest, std::abs(value));
    for (unsigned row = 1; row <= 3; ++row) for (unsigned column = 1; column <= 3; ++column) {
        const auto index = 3*(row-1)+column-1;
        INFO("tank=" << selected << " row=" << row << " column=" << column
             << " supplied=" << suppliedTank(row,column) << " old-CG reference=" << oldReference[index]
             << " published-CG reference=" << expected[index]);
        CHECK(std::abs(static_cast<long double>(suppliedTank(row,column))-expected[index]) <= 512*epsilon*largest);
    }
    REQUIRE(native->endMassChange(nullptr, nullptr));
    // Independent causal control: the change of WHOLE J must be explained by
    // replacing precisely the tank tensor. This does not call a J drift a
    // velocity-setter error or demand skipping required mass reconciliation.
    const auto afterJ = balance->GetJ();
    for (unsigned row = 1; row <= 3; ++row) for (unsigned column = 1; column <= 3; ++column) {
        const auto index = 3*(row-1)+column-1;
        const long double observed = static_cast<long double>(afterJ(row,column))-beforeJ(row,column);
        const long double predicted = expected[index]-suppliedTank(row,column);
        const long double scale = std::max(largest, std::abs(static_cast<long double>(beforeJ(row,column))));
        INFO("whole J row=" << row << " column=" << column << " observed=" << observed << " tank prediction=" << predicted);
        CHECK(std::abs(observed-predicted) <= 1024*epsilon*scale);
        CHECK(std::abs(static_cast<long double>(balance->in.TankInertia(row,column))-expected[index]) <= 512*epsilon*largest);
    }
}


TEST_CASE("Committed native mass change publishes tank and public inertia about final CG",
          "[mass][native-mass-committed-epoch]") {
    session::World world(options("native-mass-committed-epoch"));
    const auto id = wing(world, "f16c", 6000, 220);
    auto* native = dynamic_cast<sim::JsbsimModel*>(world.model(id));
    REQUIRE(native);
    const auto balance = native->exec().GetMassBalance();
    const auto propulsion = native->exec().GetPropulsion();
    REQUIRE(propulsion->GetNumTanks() > 0);
    REQUIRE(propulsion->GetNumTanks() <= std::numeric_limits<unsigned>::max());
    unsigned selected = static_cast<unsigned>(propulsion->GetNumTanks());
    for (unsigned index = 0; index < propulsion->GetNumTanks(); ++index)
        if (propulsion->GetTank(index)->GetContents() > 0) { selected = index; break; }
    REQUIRE(selected < propulsion->GetNumTanks());
    REQUIRE(native->beginMassChange());
    const auto initialCg = balance->GetXYZcg();
    sim::VehicleState initialState;
    native->state(initialState);
    const auto tank = propulsion->GetTank(selected);
    // An actual mass input changes DURING the owned transaction. This asserts
    // the contracted committed endpoint, not an intermediate begin snapshot.
    tank->SetContents(tank->GetContents() * .5);
    REQUIRE(native->endMassChange(nullptr, nullptr));
    const auto finalCg = balance->GetXYZcg();
    CHECK((finalCg - initialCg).Magnitude() > 0);
    sim::VehicleState finalState;
    native->state(finalState);
    CHECK(finalState.simTime == initialState.simTime);
    CHECK(finalState.stepCount == initialState.stepCount);
    // Independent central tensor sum using current native input scalars only.
    // No native point-mass inertia helper supplies the expected tensor.
    std::array<long double, 9> expectedTank{};
    for (unsigned index = 0; index < propulsion->GetNumTanks(); ++index) {
        const auto item = propulsion->GetTank(index);
        const auto position = item->GetXYZ();
        const long double mass = static_cast<long double>(item->GetContents()) * (1.0 / 32.174049);
        const long double r[3] = {(static_cast<long double>(finalCg(1)) - position(1)) / 12,
                                 (static_cast<long double>(position(2)) - finalCg(2)) / 12,
                                 (static_cast<long double>(finalCg(3)) - position(3)) / 12};
        const long double square = r[0]*r[0]+r[1]*r[1]+r[2]*r[2];
        for (unsigned row = 0; row < 3; ++row) for (unsigned column = 0; column < 3; ++column)
            expectedTank[3*row+column] += mass*((row == column ? square : 0)-r[row]*r[column]);
        expectedTank[0] += item->GetIxx(); expectedTank[4] += item->GetIyy(); expectedTank[8] += item->GetIzz();
    }
    sim::RigidMassPropertiesV1 published;
    REQUIRE(native->rigidMassPropertiesV1(published));
    CHECK(published.massKg == balance->GetMass()*14.5939029372064);
    const long double epsilon = std::numeric_limits<double>::epsilon();
    long double scale = 1;
    for (const auto value : expectedTank) scale = std::max(scale, std::abs(value));
    for (unsigned row = 1; row <= 3; ++row) for (unsigned column = 1; column <= 3; ++column) {
        const auto index = 3*(row-1)+column-1;
        const long double supplied = balance->in.TankInertia(row,column);
        INFO("committed row=" << row << " column=" << column << " supplied tank=" << supplied
             << " final-CG independent tank=" << expectedTank[index]);
        CHECK(std::abs(supplied-expectedTank[index]) <= 512*epsilon*scale);
        // All other whole-body terms are retained as native computed them;
        // replace only the independently checked tank contribution. This
        // checks the public tensor consequence, not merely its native input.
        const long double expectedWhole = static_cast<long double>(balance->GetJ()(row,column))
                                          - supplied + expectedTank[index];
        const long double expectedPublic = expectedWhole*1.3558179483314;
        const long double publicScale = std::max(1.0L, std::abs(expectedPublic));
        CHECK(std::isfinite(published.inertiaBodyKgM2[index]));
        CHECK(std::abs(static_cast<long double>(published.inertiaBodyKgM2[index])-expectedPublic)
              <= 1024*epsilon*publicScale);
    }
}


TEST_CASE("Native single-run mass refresh preserves null and holding paths",
          "[mass][native-mass-refresh-method]") {
    session::World legacy(options("native-mass-refresh-legacy"));
    session::World explicitNull(options("native-mass-refresh-null"));
    const auto legacyId = wing(legacy, "f16c", 6000, 220);
    const auto nullId = wing(explicitNull, "f16c", 6000, 220);
    auto* legacyModel = dynamic_cast<sim::JsbsimModel*>(legacy.model(legacyId));
    auto* nullModel = dynamic_cast<sim::JsbsimModel*>(explicitNull.model(nullId));
    REQUIRE(legacyModel); REQUIRE(nullModel);
    const auto supply = [](sim::JsbsimModel& model) {
        const auto balance = model.exec().GetMassBalance();
        const auto propulsion = model.exec().GetPropulsion();
        balance->in.TanksWeight = propulsion->GetTanksWeight();
        balance->in.TanksMoment = propulsion->GetTanksMoment();
        balance->in.TankInertia = propulsion->CalculateTankInertias();
    };
    supply(*legacyModel); supply(*nullModel);
    const auto first = legacyModel->exec().GetMassBalance();
    const auto second = nullModel->exec().GetMassBalance();
    REQUIRE_FALSE(first->Run(false));
    REQUIRE_FALSE(second->RunWithInputRefreshV1(false, nullptr, nullptr));
    CHECK(first->GetMass() == second->GetMass());
    for (unsigned row = 1; row <= 3; ++row) {
        CHECK(first->GetXYZcg(static_cast<int>(row)) == second->GetXYZcg(static_cast<int>(row)));
        for (unsigned column = 1; column <= 3; ++column) {
            const double a = first->GetJ()(row,column), b = second->GetJ()(row,column);
            const double ai = first->GetJinv()(row,column), bi = second->GetJinv()(row,column);
            CHECK(std::memcmp(&a, &b, sizeof(double)) == 0);
            CHECK(std::memcmp(&ai, &bi, sizeof(double)) == 0);
        }
    }
    struct Counter { mutable unsigned calls = 0; } counter;
    const auto observe = +[](JSBSim::FGMassBalance&, const void* context) {
        const auto& value = *static_cast<const Counter*>(context);
        ++value.calls;
    };
    const auto heldJ = second->GetJ();
    const auto heldCg = second->GetXYZcg();
    const double heldTime = nullModel->exec().GetSimTime();
    REQUIRE_FALSE(second->RunWithInputRefreshV1(true, observe, &counter));
    CHECK(counter.calls == 0);
    CHECK(nullModel->exec().GetSimTime() == heldTime);
    for (unsigned row = 1; row <= 3; ++row) {
        CHECK(second->GetXYZcg(static_cast<int>(row)) == heldCg(row));
        for (unsigned column = 1; column <= 3; ++column) CHECK(second->GetJ()(row,column) == heldJ(row,column));
    }
}

TEST_CASE("Native mass refresh runs once at computed CG before inertia composition",
          "[mass][native-mass-refresh-method]") {
    session::World plain(options("native-mass-refresh-plain"));
    session::World refreshed(options("native-mass-refresh-one"));
    const auto plainId = wing(plain, "f16c", 6000, 220);
    const auto refreshedId = wing(refreshed, "f16c", 6000, 220);
    auto* firstModel = dynamic_cast<sim::JsbsimModel*>(plain.model(plainId));
    auto* secondModel = dynamic_cast<sim::JsbsimModel*>(refreshed.model(refreshedId));
    REQUIRE(firstModel); REQUIRE(secondModel);
    const auto changeInput = [](sim::JsbsimModel& model) {
        const auto propulsion = model.exec().GetPropulsion();
        REQUIRE(propulsion->GetNumTanks() > 0);
        const auto tank = propulsion->GetTank(0);
        tank->SetContents(tank->GetContents() * .5);
        const auto balance = model.exec().GetMassBalance();
        balance->in.TanksWeight = propulsion->GetTanksWeight();
        balance->in.TanksMoment = propulsion->GetTanksMoment();
        balance->in.TankInertia = propulsion->CalculateTankInertias();
    };
    changeInput(*firstModel); changeInput(*secondModel);
    const auto first = firstModel->exec().GetMassBalance();
    const auto second = secondModel->exec().GetMassBalance();
    const auto oldCg = second->GetXYZcg();
    struct Counter {
        mutable unsigned calls = 0;
        mutable double seenCg[3] = {};
    } counter;
    const auto refresh = +[](JSBSim::FGMassBalance& balance, const void* context) noexcept {
        const auto& value = *static_cast<const Counter*>(context);
        ++value.calls;
        for (unsigned axis = 1; axis <= 3; ++axis) value.seenCg[axis-1] = balance.GetXYZcg(static_cast<int>(axis));
        // Positive diagonal synthetic input, restricted to this numerical
        // method test. No aircraft/scene/production inertia is authored here.
        balance.in.TankInertia(1,1) += .25;
        balance.in.TankInertia(2,2) += .5;
        balance.in.TankInertia(3,3) += .75;
    };
    const double time = secondModel->exec().GetSimTime();
    REQUIRE_FALSE(first->Run(false));
    REQUIRE_FALSE(second->RunWithInputRefreshV1(false, refresh, &counter));
    CHECK(counter.calls == 1);
    CHECK(secondModel->exec().GetSimTime() == time);
    CHECK((second->GetXYZcg()-oldCg).Magnitude() > 0);
    CHECK(second->GetMass() == first->GetMass());
    for (unsigned row = 1; row <= 3; ++row) {
        CHECK(counter.seenCg[row-1] == second->GetXYZcg(static_cast<int>(row)));
        CHECK(second->GetXYZcg(static_cast<int>(row)) == first->GetXYZcg(static_cast<int>(row)));
        for (unsigned column = 1; column <= 3; ++column) {
            const double delta = row == column ? .25*row : 0;
            const double scale = std::max(1.0, std::abs(first->GetJ()(row,column)));
            CHECK(std::abs(second->GetJ()(row,column)-first->GetJ()(row,column)-delta)
                  <= 16*std::numeric_limits<double>::epsilon()*scale);
        }
    }
}
