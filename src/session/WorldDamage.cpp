// MS-7g's damage part in the world (docs/mission-systems.md, G3 - G7): a warhead going off and what it breaks on the
// aircraft near it, then, step by step, what that does as they fly on - their holed tanks leaking (and burning),
// what the damage takes off the aerodynamic force (World::preStep), each section's root against its strength left,
// a failed one leaving with its mass. The physics is ms/damage's; the chances are each vehicle's own damage stream.

#include "session/World.h"

#include "simulation/mission_systems/damage/Debris.h"
#include "simulation/mission_systems/damage/Impact.h"

#include "foundation/core/Log.h"
#include "foundation/io/AssetResolver.h"

#include <algorithm>
#include <cstring>

namespace fsim::session {

namespace {

constexpr std::uint64_t kDamageStream = 0x444D'4147ull; // "DMAG": a stream of its own, so no other draw moves
constexpr double kLbsToKg = 0.45359237, kLbfToN = 4.4482216152605, kPsfToPa = 47.880258888889;
constexpr double kSlugFt3ToKgM3 = 515.378818;
constexpr double kG = 9.80665;
constexpr double kRateReported = 0.1; ///< a fire's rate recorded again when it has moved by a tenth
constexpr double kSeparationMs = 5.0; ///< a failed section's push away from its root (an estimate: the break's spring)
/// A section a hit breaks off leaves with its own share of the hit's push, no faster than this from its aircraft (an
/// estimate: a light panel's share of a near burst would otherwise be hundreds of metres a second - it tears, it is
/// not thrown whole)
constexpr double kThrownMostMs = 60.0;
/// A holed tank's fuel is thrown out the way the fragments went through it (hydrodynamic ram: the spray leaves the far
/// wall) at this share of their speed (an estimate: some tens of metres a second behind fragments at 1.5 - 2 km/s)
constexpr double kRamSprayShare = 0.03;

ms::damage::V3 v3(const double* a) { return {a[0], a[1], a[2]}; }


// a section's place in its model's structural frame (m: x aft, y right, z up) from body axes off its origin
void structural(const ms::damage::Airframe& a, const ms::damage::V3& body, double out[3]) {
    out[0] = a.originDesign.x - body.x;
    out[1] = a.originDesign.y + body.y;
    out[2] = a.originDesign.z - body.z;
}

} // namespace

ipc::EngagementEvent World::engagementEvent(const Entry& e, ipc::EngagementKind kind, const std::string& name) const {
    ipc::EngagementEvent ev;
    ev.kind = static_cast<std::uint8_t>(kind);
    ev.slot = static_cast<std::uint32_t>(e.slot);
    ev.simTime = simTime_;
    const sim::VehicleState& s = pool_->states()[e.slot];
    for (int k = 0; k < 3; ++k) ev.positionEcef[k] = s.positionEcef[k];
    // (its velocity in ECEF: the body's, turned by its attitude - the rotation's columns are the body axes)
    const double* R = s.rotationBodyToEcef;
    for (int r = 0; r < 3; ++r) ev.velocityEcef[r] = R[3 * r] * s.velocityBodyMs[0] + R[3 * r + 1] * s.velocityBodyMs[1] + R[3 * r + 2] * s.velocityBodyMs[2];
    std::snprintf(ev.name, sizeof ev.name, "%s", name.c_str());
    return ev;
}

void World::damageFlight(Entry& e, ms::damage::Flight& f) {
    auto& r = e.damageReads;
    const double weightN = r.weight.get() * kLbfToN;
    for (int k = 0; k < 3; ++k) {
        (&f.aeroForceN.x)[k] = r.aero[static_cast<std::size_t>(k)].get() * kLbfToN;
        (&f.specificForceMs2.x)[k] = weightN > 0.0 ? r.total[static_cast<std::size_t>(k)].get() * kLbfToN / (weightN / kG) : 0.0;
        (&f.rateDotRadS2.x)[k] = e.angularAcceleration[static_cast<std::size_t>(k)].get();
    }
    const sim::VehicleState& s = e.working;
    f.rateRadS = v3(s.angularRateBodyRadS);
    f.dynamicPressurePa = r.qbar.get() * kPsfToPa;
    f.tankKg.resize(r.tanks.size());
    for (std::size_t t = 0; t < r.tanks.size(); ++t) f.tankKg[t] = r.tanks[t].get() * kLbsToKg;
}

std::vector<ipc::EngagementEvent> World::detonate(const Detonation& d) {
    std::vector<ipc::EngagementEvent> out;
    if (storeCards_.empty()) {
        std::vector<std::string> errors;
        if (const auto library = io::AssetResolver{}.find("stores")) storeCards_ = ms::readStoreLibrary(*library, errors);
        for (const auto& error : errors) LOG_WARN("session") << "stores: " << error;
    }
    const auto card = storeCards_.find(d.store);
    const auto warhead = card == storeCards_.end() ? std::nullopt : ms::damage::warheadOf(*card->second);
    if (!warhead) {
        LOG_WARN("session") << "detonate: '" << d.store << "' has no warhead on its card";
        return out;
    }
    const Entry* weapon = entry(d.weapon);
    ipc::EngagementEvent burst;
    burst.kind = static_cast<std::uint8_t>(ipc::EngagementKind::Detonation);
    burst.slot = weapon ? static_cast<std::uint32_t>(weapon->slot) : 0u;
    burst.simTime = simTime_;
    for (int k = 0; k < 3; ++k) burst.positionEcef[k] = d.positionEcef[k], burst.velocityEcef[k] = d.velocityEcef[k];
    burst.amountKg = warhead->tntKg, burst.massKg = warhead->totalKg;
    std::snprintf(burst.name, sizeof burst.name, "%s", d.store.c_str());
    const std::size_t burstAt = out.size();
    out.push_back(burst);
    double nearest = kDamageReachM;

    ms::damage::Burst b;
    b.at = v3(d.positionEcef), b.velocity = v3(d.velocityEcef), b.axis = v3(d.axisEcef);
    const auto liveStates = pool_->states();
    std::vector<sim::VehicleState> states(liveStates.begin(), liveStates.end());
    if (std::isfinite(d.stepFraction)) {
        const double f=std::clamp(d.stepFraction,0.0,1.0);
        for(std::size_t slot=0;slot<states.size() && slot<stepStates_.size();++slot)
            for(int k=0;k<3;++k) states[slot].positionEcef[k]=stepStates_[slot].positionEcef[k]+f*(states[slot].positionEcef[k]-stepStates_[slot].positionEcef[k]);
    }
    // A burst may create debris. Freeze the victim set and its states before any slot can be appended.
    for (std::size_t slot = 0; slot < states.size(); ++slot) {
        Entry* e = entries_[slot].get();
        if (!e || e == weapon || e->info.role != ms::Role::Aircraft) continue;
        const auto af = airframes_.find(e->info.aircraft);
        if (af == airframes_.end()) continue;
        const sim::VehicleState& s = states[slot];
        const ms::damage::V3 at = v3(s.positionEcef);
        const double miss = ms::damage::length(b.at - at);
        if (!(miss < kDamageReachM)) continue;
        auto& model = pool_->vehicle(slot);
        if (!ensureDamage(*e)) continue;
        ms::damage::Target t;
        const double* R = s.rotationBodyToEcef;
        for (std::size_t k = 0; k < 3; ++k) t.bodyToEcef[k] = {R[k], R[3 + k], R[6 + k]};
        // (its model's origin, which its airframe's places are from: the state is at its CG)
        double cg[3];
        model.cgFromOriginBodyM(cg);
        t.at = at - ms::damage::V3{R[0] * cg[0] + R[1] * cg[1] + R[2] * cg[2], R[3] * cg[0] + R[4] * cg[1] + R[5] * cg[2],
                                   R[6] * cg[0] + R[7] * cg[1] + R[8] * cg[2]};
        for (int r = 0; r < 3; ++r)
            (&t.velocity.x)[r] = R[3 * r] * s.velocityBodyMs[0] + R[3 * r + 1] * s.velocityBodyMs[1] + R[3 * r + 2] * s.velocityBodyMs[2];
        t.airPressurePa = model.property("atmosphere/P-psf").get() * kPsfToPa;
        t.airTemperatureK = model.property("atmosphere/T-R").get() / 1.8;
        t.airDensity = model.property("atmosphere/rho-slugs_ft3").get() * kSlugFt3ToKgM3;
        const ms::damage::Impact impact = ms::damage::assess(*warhead, b, *af->second, t, e->damageRng);
        // where the burst was from its origin (body axes), and what struck each section's own panels: the fragments'
        // momentum, and the blast's impulse on each face, away from the burst
        const ms::damage::V3 burstBody = impact.fromBody * impact.missM;
        std::vector<ms::damage::V3> sectionImpulse(af->second->sections.size());
        for (const auto& ph : impact.panels) {
            if (ph.panel < 0 || static_cast<std::size_t>(ph.panel) >= af->second->panels.size()) continue;
            const auto& panel = af->second->panels[static_cast<std::size_t>(ph.panel)];
            if (panel.section < 0 || static_cast<std::size_t>(panel.section) >= sectionImpulse.size()) continue;
            sectionImpulse[static_cast<std::size_t>(panel.section)] +=
                ph.momentumNs + ms::damage::normalized(panel.middle() - burstBody) * (ph.impulsePas * panel.areaM2);
        }
        const double hitKg = model.energy().massKg;
        {
            // the blast and the fragments push and turn it (about its CG: the impulse's moment about the origin less
            // the CG's offset's)
            const ms::damage::V3 J = impact.impulseBody;
            const ms::damage::V3 L = impact.angularImpulseBody - ms::damage::cross(v3(cg), J);
            const double linear[3] = {J.x, J.y, J.z}, angular[3] = {L.x, L.y, L.z};
            model.applyImpulse(linear, angular);
        }
        if (impact.fragmentsStriking > 0) {
            auto hit = engagementEvent(*e, ipc::EngagementKind::Hit, d.store);
            hit.target = weapon ? static_cast<std::uint32_t>(weapon->slot) + 1 : 0u;
            hit.index = static_cast<std::uint32_t>(impact.fragmentsStriking);
            out.push_back(hit);
        }
        if (miss < nearest) nearest = miss, out[burstAt].target = static_cast<std::uint32_t>(slot) + 1;
        ms::damage::Flight f;
        damageFlight(*e, f);
        const auto& a = *af->second;
        for (const auto& c : e->damage->apply(impact, t.airPressurePa, e->damageRng)) {
            using K = ms::damage::Change::Kind;
            const std::string component = c.component >= 0 ? a.components[static_cast<std::size_t>(c.component)].name : std::string();
            switch (c.kind) {
            case K::Fire: {
                auto ev = engagementEvent(*e, ipc::EngagementKind::Fire, component);
                if (c.component >= 0) { // (where its component is on it)
                    const auto& centre = a.components[static_cast<std::size_t>(c.component)].centre;
                    for (int r = 0; r < 3; ++r) ev.positionEcef[r] += R[3 * r] * centre.x + R[3 * r + 1] * centre.y + R[3 * r + 2] * centre.z;
                }
                ev.amountKg = c.spillKg;
                if (c.spillKg > 0.0 && c.component >= 0) {
                    // what it throws out at once goes the way the fragments went through it - from the burst, through
                    // the tank, out of its far side - on top of the aircraft's own motion: the fireball is laid along
                    // the aircraft's path, and off it to the side away from the burst
                    double fastest = 0.0;
                    for (const auto& h : impact.components)
                        if (h.component == c.component) fastest = std::max(fastest, h.fastestMs);
                    const ms::damage::V3 through = ms::damage::normalized(a.components[static_cast<std::size_t>(c.component)].centre - burstBody) *
                                                   (kRamSprayShare * fastest);
                    for (int r = 0; r < 3; ++r) ev.velocityEcef[r] += R[3 * r] * through.x + R[3 * r + 1] * through.y + R[3 * r + 2] * through.z;
                }
                if (c.tank >= 0 && static_cast<std::size_t>(c.tank) < f.tankKg.size()) {
                    // what it throws out at once leaves its tank; what leaks on burns on
                    auto& h = e->damageReads.tanks[static_cast<std::size_t>(c.tank)];
                    const double kg = std::max(f.tankKg[static_cast<std::size_t>(c.tank)] - c.spillKg, 0.0);
                    h.set(kg / kLbsToKg);
                    ev.massKg = e->damage->leakKgS(c.tank, kg, s.loadFactor, f.dynamicPressurePa);
                    ev.rateKgS = std::min(ev.massKg, ms::damage::kBurnMostKgS); // (what burns of it; the rest its mist)
                    e->fireRates[c.tank] = ev.massKg;
                } else if (c.engine >= 0) {
                    // an engine's fire: the fuel its feed was flowing as it failed (an estimate: until it is shut off)
                    ev.rateKgS = model.property("propulsion/engine[" + std::to_string(c.engine) + "]/fuel-flow-rate-pps").get() * kLbsToKg;
                    ev.massKg = ev.rateKgS;
                }
                out.push_back(ev);
                break;
            }
            case K::EngineOut: {
                model.seizeEngine(c.engine);
                auto ev = engagementEvent(*e, ipc::EngagementKind::EngineOut, component);
                ev.index = static_cast<std::uint32_t>(std::max(c.engine, 0));
                out.push_back(ev);
                break;
            }
            case K::ControlLost: {
                const int n = model.failChannel(c.channel);
                if (n == 0) LOG_WARN("session") << e->info.name << ": no actuator of its '" << c.channel << "' channel to fail";
                out.push_back(engagementEvent(*e, ipc::EngagementKind::ControlLost, c.channel));
                break;
            }
            case K::CrewLost:
                e->crewLost = true;
                out.push_back(engagementEvent(*e, ipc::EngagementKind::CrewLost, component));
                break;
            case K::Separated: { // (the blast's impulse broke its root at once)
                Thrown thrown;
                const auto& J = sectionImpulse[static_cast<std::size_t>(c.section)];
                thrown.impulseBody[0] = J.x, thrown.impulseBody[1] = J.y, thrown.impulseBody[2] = J.z;
                if (std::isfinite(hitKg) && hitKg > 0.0)
                    for (int k = 0; k < 3; ++k) thrown.sharedBody[k] = (&impact.impulseBody.x)[k] / hitKg;
                separated(*e, c.section, nullptr, &thrown);
                break;
            }
            case K::PanelTorn: {
                auto ev = engagementEvent(*e, ipc::EngagementKind::PanelTorn, a.sections[static_cast<std::size_t>(c.section)].name);
                ev.index = static_cast<std::uint32_t>(c.panel);
                out.push_back(ev);
                break;
            }
            default:
                break;
            }
        }
        LOG_INFO("session") << e->info.name << " hit by " << d.store << " at " << miss << " m: " << impact.fragmentsStriking << " of "
                            << impact.fragmentsThrown << " fragments struck";
        for (std::size_t k = 0; k < a.sections.size(); ++k)
            if (a.sections[k].hasRoot)
                LOG_INFO("session") << "  its " << a.sections[k].name << ": strength left " << e->damage->sections()[k].strength
                                    << ", the blast's and the fragments' kick " << e->damage->sections()[k].kick << " of it";
    }
    for (const auto& ev : out) engage(ev);
    return out;
}

void World::stepDamage() {
    const double dt = options_.dt * options_.frameSkip;
    // a ground strike's fire, burnt out
    for (auto& entry : entries_)
        if (Entry* e = entry.get(); e && e->wreckFireUntilS > 0.0 && simTime_ >= e->wreckFireUntilS) {
            engage(engagementEvent(*e, ipc::EngagementKind::Fire, e->wreckFireName)); // (its rate 0: it is out)
            e->wreckFireUntilS = 0.0;
        }
    for (std::size_t slot = 0; slot < entries_.size(); ++slot) {
        Entry* e = entries_[slot].get();
        if (!e || !e->damage) continue;
        auto& model = pool_->vehicle(slot);
        (void)model;
        e->working = pool_->states()[slot];
        ms::damage::Flight f;
        damageFlight(*e, f);
        // the holed tanks leak; a burning one's rate recorded as it changes, its end when it is dry
        for (const auto& [tank, state] : e->damage->tanks()) {
            if (tank < 0 || static_cast<std::size_t>(tank) >= f.tankKg.size()) continue;
            const double kg = f.tankKg[static_cast<std::size_t>(tank)];
            const double leak = e->damage->leakKgS(tank, kg, e->working.loadFactor, f.dynamicPressurePa);
            if (leak > 0.0) e->damageReads.tanks[static_cast<std::size_t>(tank)].set(std::max(kg - leak * dt, 0.0) / kLbsToKg);
            if (!state.burning) continue;
            const auto last = e->fireRates.find(tank);
            const double before = last == e->fireRates.end() ? 0.0 : last->second;
            if (std::abs(leak - before) > kRateReported * std::max(before, 1e-3) && !(before == 0.0 && leak == 0.0)) {
                const auto& comp = e->damage->airframe().components[static_cast<std::size_t>(std::max(state.component, 0))];
                auto ev = engagementEvent(*e, ipc::EngagementKind::Fire, comp.name);
                ev.massKg = leak;
                ev.rateKgS = std::min(leak, ms::damage::kBurnMostKgS);
                engage(ev);
                e->fireRates[tank] = leak;
            }
        }
        // each section's root against its strength left: one that fails leaves, with its mass and its tanks' fuel
        for (const auto& c : e->damage->loads(f)) separated(*e, c.section);
    }
}

bool World::ensureDamage(Entry& e) {
    if (e.damage) return true;
    const auto af = airframes_.find(e.info.aircraft);
    if (af == airframes_.end() || e.info.role != ms::Role::Aircraft) return false;
    auto& model = pool_->vehicle(e.slot);
    e.damage = std::make_unique<ms::damage::VehicleDamage>(af->second);
    e.damageRng = Rng::forVehicle(options_.seed, kDamageStream, e.info.id);
    auto& r = e.damageReads;
    const char* axes[3] = {"x", "y", "z"};
    for (std::size_t k = 0; k < 3; ++k) {
        r.aero[k] = model.property(std::string("forces/fb") + axes[k] + "-aero-lbs");
        r.total[k] = model.property(std::string("forces/fb") + axes[k] + "-total-lbs");
    }
    r.weight = model.property("inertia/weight-lbs");
    r.qbar = model.property("aero/qbar-psf");
    int tanks = 0; // (its tanks: as many as it lists - the first's name has no index)
    for (const auto& [name, value] : model.properties("propulsion")) { // (names below the prefix)
        (void)value;
        if (name == "tank/contents-lbs") tanks = std::max(tanks, 1);
        const auto close = name.find(']');
        if (name.rfind("tank[", 0) == 0 && close != std::string::npos && name.compare(close, std::string::npos, "]/contents-lbs") == 0)
            tanks = std::max(tanks, std::stoi(name.substr(5, close - 5)) + 1);
    }
    for (int k = 0; k < tanks; ++k) r.tanks.push_back(model.property("propulsion/tank[" + std::to_string(k) + "]/contents-lbs"));
    return true;
}

void World::separated(Entry& e, int section, const sim::VehicleState* as, const Thrown* thrown) {
    const auto& a = e.damage->airframe();
    const auto& sec = a.sections[static_cast<std::size_t>(section)];
    auto& model = pool_->vehicle(e.slot);
    // the aircraft as it was, and where its CG is from its model's origin (the section's place is from that)
    Parting part;
    part.before = as ? *as : pool_->states()[e.slot];
    model.cgFromOriginBodyM(part.cgFromOrigin);
    // what leaves with it: its structure, its tanks' fuel, its stations' stores - off the aircraft at once, so its CG,
    // its inertia and its state move to what is left (FlightModel::endMassChange)
    model.beginMassChange();
    double at[3];
    structural(a, sec.cg, at);
    model.removeMass(sec.massKg, at);
    for (const auto& t : sec.tanks)
        if (t.index >= 0 && static_cast<std::size_t>(t.index) < e.damageReads.tanks.size()) {
            auto& contents = e.damageReads.tanks[static_cast<std::size_t>(t.index)];
            part.fuelKg += contents.get() * kLbsToKg;
            contents.set(0.0);
        }
    for (const int station : sec.stations) part.stations.emplace_back(station, e.ms.loseStation(station, simTime_));
    applyStoreMasses(e);
    // pushed off its root by the break's spring, and the aircraft the other way
    double partKg = sec.massKg + part.fuelKg;
    for (const auto& [station, kg] : part.stations) partKg += kg;
    ms::damage::V3 push = ms::damage::normalized(sec.cg - sec.root) * kSeparationMs;
    if (thrown && partKg > 0.0) {
        // a hit broke it off: what struck its own panels pushed it, where what pushed the whole aircraft is already in
        // the aircraft's motion - it leaves with the difference, away from the burst (and the aircraft the other way)
        ms::damage::V3 extra = v3(thrown->impulseBody) * (1.0 / partKg) - v3(thrown->sharedBody);
        if (const double speed = ms::damage::length(extra); speed > kThrownMostMs) extra = extra * (kThrownMostMs / speed);
        push = push + extra;
    }
    part.pushBody[0] = push.x, part.pushBody[1] = push.y, part.pushBody[2] = push.z;
    const double reaction[3] = {-partKg * push.x, -partKg * push.y, -partKg * push.z};
    model.endMassChange(sec.inertia.data(), reaction);
    engage(engagementEvent(e, ipc::EngagementKind::Separated, sec.name));
    LOG_INFO("session") << e.info.name << ": its " << sec.name << " failed at its root and left";
    // it flies on as a piece of its own, its flight model its section's (hangar's pieces), its fuel and stores with it
    launchPiece(e, section, part);
}

bool World::separateSection(std::uint32_t id, const std::string& section) {
    Entry* e = entry(id);
    if (!e || !ensureDamage(*e)) return false;
    const int s = e->damage->airframe().section(section);
    if (s < 0 || !e->damage->separate(s)) return false;
    separated(*e, s);
    return true;
}

std::uint32_t World::launchPiece(Entry& from, int section, const Parting& part) {
    const auto& a = from.damage->airframe();
    const auto& sec = a.sections[static_cast<std::size_t>(section)];
    const auto dir = aircraftDirs_.find(from.info.aircraft);
    std::string slug = sec.name;
    for (char& c : slug)
        if (c == ' ') c = '-';
    const std::string piece = from.info.aircraft + "-" + slug;
    if (dir == aircraftDirs_.end() || !std::filesystem::is_regular_file(dir->second / "pieces" / piece / (piece + ".xml"))) {
        // (it has left all the same - its mass, fuel and stores off the aircraft - flown by nothing)
        LOG_WARN("session") << from.info.name << ": no flight model for its " << sec.name << " (hangar's pieces/" << piece << ")";
        return 0;
    }
    const sim::VehicleState& s = part.before;
    const double* R = s.rotationBodyToEcef;
    auto toEcef = [&](const ms::damage::V3& b) {
        return ms::damage::V3{R[0] * b.x + R[1] * b.y + R[2] * b.z, R[3] * b.x + R[4] * b.y + R[5] * b.z, R[6] * b.x + R[7] * b.y + R[8] * b.z};
    };
    // where its centre was (from the aircraft's CG, which the state is at: the section's place is from the model's
    // origin), as it moved there (the aircraft's velocity and its rotation's), pushed off its root
    const ms::damage::V3 r = sec.cg - v3(part.cgFromOrigin);
    const ms::damage::V3 at = v3(s.positionEcef) + toEcef(r);
    const ms::damage::V3 rate = v3(s.angularRateBodyRadS);
    const ms::damage::V3 dv = toEcef(ms::damage::cross(rate, r) + v3(part.pushBody));
    double lat, lon, h;
    ms::damage::geodetic(at, lat, lon, h);
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const ms::damage::V3 north{-sl * co, -sl * so, cl}, east{-so, co, 0.0}, down{-cl * co, -cl * so, -sl};
    VehicleSpec spec;
    spec.name = from.info.name + " " + sec.name;
    spec.type = "jsbsim:" + piece;
    spec.aircraftDir = dir->second / "pieces";
    spec.model = from.info.model;
    spec.initial.latitudeDeg = lat * 180.0 / 3.14159265358979323846;
    spec.initial.longitudeDeg = lon * 180.0 / 3.14159265358979323846;
    spec.initial.altitudeMslM = h;
    spec.initial.rollDeg = s.eulerRad[0] * 180.0 / 3.14159265358979323846;
    spec.initial.pitchDeg = s.eulerRad[1] * 180.0 / 3.14159265358979323846;
    spec.initial.headingDeg = s.eulerRad[2] * 180.0 / 3.14159265358979323846;
    spec.initial.velocityNedMs[0] = s.velocityNedMs[0] + ms::damage::dot(dv, north);
    spec.initial.velocityNedMs[1] = s.velocityNedMs[1] + ms::damage::dot(dv, east);
    spec.initial.velocityNedMs[2] = s.velocityNedMs[2] + ms::damage::dot(dv, down);
    spec.initial.airspeedTrueMs = s.airspeedTrueMs;
    for (int k = 0; k < 3; ++k) spec.initial.bodyRatesRadS[k] = s.angularRateBodyRadS[k];
    const std::uint32_t id = createVehicle(spec);
    Entry* p = entry(id);
    if (!p) return 0;
    p->info.role = ms::Role::Debris;
    p->info.launcher = from.info.id;
    p->info.launchTimeS = simTime_;
    for (std::size_t c = 0; c < p->catalog->size(); ++c)
        p->host.setAvailability(c, control::Availability::Unavailable, control::Reason::Restricted);
    p->inputs = sim::ControlInputs{};
    poolInputs_[p->slot] = p->inputs;
    // what it took with it: its tanks' fuel (at their middle), each station's stores (at its place) - its own CG and
    // state then moved to all of it, as the aircraft's were (each part moving as its point of the aircraft did)
    auto& model = pool_->vehicle(p->slot);
    model.beginMassChange();
    model.property("inertia/pointmass-weight-lbs").set(part.fuelKg / kLbsToKg); // ("fuel")
    double storesKg = 0.0;
    for (std::size_t k = 0; k < part.stations.size(); ++k) {
        model.property("inertia/pointmass-weight-lbs[" + std::to_string(k + 1) + "]").set(part.stations[k].second / kLbsToKg);
        storesKg += part.stations[k].second;
    }
    model.endMassChange(nullptr, nullptr);
    // drawn as its aircraft's model, its section alone: the model's origin from the piece's CG (its structure's
    // centre and what it carries)
    double shift[3];
    model.cgFromOriginBodyM(shift);
    char offset[96];
    std::snprintf(offset, sizeof offset, ":%.4f,%.4f,%.4f", -(sec.cg.x + shift[0]), -(sec.cg.y + shift[1]), -(sec.cg.z + shift[2]));
    p->info.type = "debris:" + from.info.aircraft + ":" + sec.name + offset;
    publishVehicle(*p); // (recorded, and its row told to a live viewer: its type is what it is drawn by)
    LOG_INFO("session") << "piece " << id << " '" << spec.name << "': " << sec.massKg + part.fuelKg + storesKg << " kg (" << part.fuelKg
                        << " of fuel, " << storesKg << " of stores)";
    return id;
}

void World::impacted(Entry& e, const sim::VehicleState& before, const sim::VehicleState& after) {
    namespace dmg = ms::damage;
    auto& model = pool_->vehicle(e.slot);
    // the local axes there (ECEF), and the ground's normal: its slope over its data's spacing
    const double lat = after.latitudeRad, lon = after.longitudeRad;
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const dmg::V3 north{-sl * co, -sl * so, cl}, east{-so, co, 0.0}, down{-cl * co, -cl * so, -sl};
    dmg::V3 normal = down * -1.0;
    if (ground_ && std::isfinite(ground_->resolutionM())) {
        const double h = std::clamp(ground_->resolutionM(), 5.0, 60.0);
        const double dn = h / 6371000.0, de = h / (6371000.0 * std::max(cl, 1e-6));
        const double slopeN = (ground_->heightAboveEllipsoidM(lat + dn, lon) - ground_->heightAboveEllipsoidM(lat - dn, lon)) / (2.0 * h);
        const double slopeE = (ground_->heightAboveEllipsoidM(lat, lon + de) - ground_->heightAboveEllipsoidM(lat, lon - de)) / (2.0 * h);
        if (std::isfinite(slopeN) && std::isfinite(slopeE)) normal = dmg::normalized(north * -slopeN + east * -slopeE + down * -1.0);
    }
    auto ecef = [&](const double ned[3]) { return north * ned[0] + east * ned[1] + down * ned[2]; };
    dmg::GroundStrike strike;
    strike.velocity = ecef(before.velocityNedMs);
    strike.normal = normal;
    const sim::EnergyOnBoard aboard = model.energy();
    strike.massKg = std::isfinite(aboard.massKg) ? aboard.massKg : 0.0;
    strike.fuelKg = std::isfinite(aboard.fuelKg) ? std::max(aboard.fuelKg, 0.0) : 0.0;
    // a store that was released ends where it strikes (World removes it): nothing of it burns on. A tank's fuel is the
    // point mass its flight model names "fuel" (hangar's storemodels.py), set as it left its station
    const bool stays = e.info.role != ms::Role::Weapon;
    if (!stays)
        if (const auto fuel = model.property("inertia/pointmass-weight-lbs"); fuel.valid()) strike.fuelKg = std::max(fuel.get() * kLbsToKg, 0.0);
    const dmg::StrikeOutcome o = dmg::groundStrike(strike);

    // its sections against the strike's load, as against a flight's: what an accelerometer read through the crush, in
    // its body axes as it struck - a root that cannot hold its section against that fails, and the section goes on
    // with what the strike left it (the wreck itself is stopped where it lies)
    int broke = 0;
    if (o.normalMs > 0.0 && e.info.role == ms::Role::Aircraft && ensureDamage(e)) {
        const double* R = before.rotationBodyToEcef; // (row-major, columns the body axes: its transpose turns ECEF to body)
        const dmg::V3 f = o.specificForce;
        dmg::Flight flight;
        flight.specificForceMs2 = {R[0] * f.x + R[3] * f.y + R[6] * f.z, R[1] * f.x + R[4] * f.y + R[7] * f.z, R[2] * f.x + R[5] * f.y + R[8] * f.z};
        flight.rateRadS = v3(before.angularRateBodyRadS);
        flight.tankKg.resize(e.damageReads.tanks.size());
        for (std::size_t t = 0; t < e.damageReads.tanks.size(); ++t) flight.tankKg[t] = e.damageReads.tanks[t].get() * kLbsToKg;
        // (thrown from where it lies, a little clear of the ground, as it flew in but with the strike's velocity)
        sim::VehicleState thrown = before;
        const double clear = 0.5 * o.crushM;
        for (int k = 0; k < 3; ++k) thrown.positionEcef[k] = after.positionEcef[k] + (&normal.x)[k] * clear;
        thrown.latitudeRad = after.latitudeRad, thrown.longitudeRad = after.longitudeRad;
        thrown.velocityNedMs[0] = dmg::dot(o.after, north), thrown.velocityNedMs[1] = dmg::dot(o.after, east), thrown.velocityNedMs[2] = dmg::dot(o.after, down);
        thrown.airspeedTrueMs = dmg::length(o.after);
        for (const auto& c : e.damage->loads(flight)) {
            separated(e, c.section, &thrown);
            ++broke;
        }
    }

    ipc::EngagementEvent ev = engagementEvent(e, ipc::EngagementKind::Impact, "ground");
    for (int k = 0; k < 3; ++k) {
        ev.velocityEcef[k] = (&strike.velocity.x)[k];
        ev.normalEcef[k] = (&normal.x)[k];
    }
    ev.massKg = strike.massKg;
    ev.energyJ = o.energyJ;
    ev.angleRad = o.angleRad;
    ev.loadG = o.decelMs2 / kG;
    ev.index = static_cast<std::uint32_t>(broke);
    ev.amountKg = o.fireballKg;
    ev.rateKgS = stays ? std::min(o.poolKgS, dmg::kBurnMostKgS) : 0.0;
    engage(ev);
    LOG_INFO("session") << e.info.name << " struck the ground at " << o.speedMs << " m/s, " << o.angleRad * 57.29577951308232
                        << " deg below it: " << ev.loadG << " g across it, " << broke << " section(s) broke off, " << o.fireballKg
                        << " kg of fuel burnt at once, " << o.poolKg << " kg burns on";

    // its fuel's fire: the mist's fireball, moving as what left the strike does, and what burns on where it lies - from
    // a tank of its own where its damage data names one (the viewer hangs the flame there)
    if (o.fireballKg > 0.0 || ev.rateKgS > 0.0) {
        std::string component = "fuel";
        if (e.damage) {
            for (const auto& c : e.damage->airframe().components)
                if (c.kind == "fuel" && (c.section < 0 || !e.damage->sections()[static_cast<std::size_t>(c.section)].separated)) {
                    component = c.name;
                    break;
                }
        } else if (e.info.type.rfind("debris:", 0) == 0) {
            // a piece (debris:<aircraft>:<section>:<offset>): a tank of its section, as its aircraft's damage data has it
            const auto second = e.info.type.find(':', 7);
            const auto third = second == std::string::npos ? second : e.info.type.find(':', second + 1);
            const auto af = second == std::string::npos ? airframes_.end() : airframes_.find(e.info.type.substr(7, second - 7));
            if (af != airframes_.end() && third != std::string::npos) {
                const std::string section = e.info.type.substr(second + 1, third - second - 1);
                for (const auto& c : af->second->components)
                    if (c.kind == "fuel" && c.section >= 0 && af->second->sections[static_cast<std::size_t>(c.section)].name == section) {
                        component = c.name;
                        break;
                    }
            }
        }
        ipc::EngagementEvent fire = engagementEvent(e, ipc::EngagementKind::Fire, component);
        for (int k = 0; k < 3; ++k) {
            fire.velocityEcef[k] = (&o.after.x)[k];
            fire.normalEcef[k] = (&normal.x)[k]; // (on the ground: the viewer's fireball a hemisphere, rolling on along it)
        }
        fire.amountKg = o.fireballKg;
        fire.rateKgS = ev.rateKgS;
        fire.massKg = ev.rateKgS; // (all of it burns: nothing blows off a wreck as mist)
        engage(fire);
        if (ev.rateKgS > 0.0) {
            e.wreckFireName = component;
            e.wreckFireUntilS = simTime_ + o.poolKg / ev.rateKgS;
        }
    }
}

void World::engage(const ipc::EngagementEvent& event) {
    engagement_.push_back(event);
    if (publisher_) publisher_->publishEvent(event); // (a live viewer draws it as it happens)
    // (recorded as it happens: the reader gives it to the frame that follows - G7)
    if (recorder_.isOpen()) recorder_.writeEvent(event);
}

std::vector<ipc::EngagementEvent> World::takeEngagementEvents() {
    std::vector<ipc::EngagementEvent> out;
    out.swap(engagement_);
    return out;
}

const ms::damage::VehicleDamage* World::damage(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->damage.get() : nullptr;
}

} // namespace fsim::session
