#include "world/Airflow.h"

#include "world/Blackbody.h"
#include "world/ShadingRate.h"

#include "core/Log.h"
#include "world/Frames.h"
#include "world/Propellants.h"
#include "world/Sky.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace fsim::world {

namespace {

constexpr double kRange = 60000.0;        // m from the eye: aircraft whose trails are made
constexpr double kInterval = 0.25;        // s between samples as they are laid
constexpr double kEngineEfficiency = 0.3; // pycontrails' SACParams.engine_efficiency
constexpr double kQFuel = 43.13e6;        // J/kg, pycontrails' JetA
// s: the jets' ice is seen from half this age on, all of it from this age: an ATTAS's contrail was seen 25-35 m behind
// its engines at 200 m/s (Busen and Schumann 1995; Schumann et al. 2017, 3.1.5), within one to two spans behind an
// airliner (Kaercher 2018; the LES of Unterstrasser and co-workers)
constexpr double kCondense = 0.3;
constexpr double kBottomRadius = 6360000.0;
constexpr double kExposure = 10.0;        // takram's examples'
constexpr int kStride = 7;                // rows per sample (assets/clouds/shaders/contrail.glsl)
// A contrail's plume is gone once it can no longer be seen: its optical depth under 0.02, the threshold the
// contrail-cirrus literature takes for one to be visible (Kaercher et al. 2009, "Factors controlling contrail cirrus
// optical depth"); CoCiP's own limits (an optical depth of 1e-6, 20 hours) would keep it drawn long after
constexpr double kVisibleTau = 0.02;
constexpr double kRollUpSpans = 10.0;     // the wake rolls up within some ten spans
constexpr double kCrowGrowth = 0.83;      // the Crow mode's growth rate, w0 / b0 (Crow 1970)

double finite(double v, double fallback = 0.0) { return std::isfinite(v) ? v : fallback; }

// A missile's smoke. Its motor's mass flow is the thrust over its propellant class's delivered specific impulse
// (world/Propellants.h: NASA CEA's ideal at the exit's pressure, 241 s aluminised and 232 s reduced smoke, of which 95 %
// is delivered - the efficiencies an assumption), laid along the trail at its speed.
constexpr double kG0 = 9.80665;
// Primary smoke, the propellant's particles. An aluminised propellant's 18 % aluminium (a tactical motor's usual
// loading) leaves 34 % of the exhaust's mass as alumina (NASA CEA: 0.340), particles some 3 um across: their
// mass extinction in visible light 3 Q / (2 rho D) = 250 m2/kg (Q 2, their geometric limit; rho 3,990 kg/m3). A
// reduced-smoke propellant (no aluminium) leaves its burn-rate catalysts' oxides and some carbon - 0.25 % of its mass,
// the middle of the 0.1 - 0.5 % such catalysts are loaded at, smaller particles, 3,000 m2/kg (assumed); a minimum-smoke
// (nitramine, double-base) propellant a tenth of that.
constexpr double kAluminaExtinction = 250.0;
constexpr double kResidueShare[3] = {0.00025, 0.0025, 0.0}, kResidueExtinction = 3000.0;
// Secondary smoke: ammonium perchlorate's chlorine leaves as HCl - 21 % of an aluminised motor's exhaust (68 % AP), 27 %
// of a reduced-smoke one's (86 % AP; NASA CEA); a minimum-smoke propellant has none. It
// takes up the air's water into droplets of acid, as much as their equilibrium with the air's humidity gives, where the
// air is humid enough (secondarySmokeOnset); droplets some 0.5 um in radius scatter 3 Q / (4 rho r) = 3,400 m2/kg
// (Q 2.5, rho 1,100 kg/m3). How much of the HCl is in visible droplets within the trail's first seconds is not
// modelled: a tenth (an engineering assumption, until a published secondary-smoke method - AGARD AR-287's, STANAG
// 6016's - is to hand)
constexpr double kAcidExtinction = 3400.0, kAcidCondensed = 0.1;
// The trail's start: the jet where its afterburning flame ends, some 28 exit radii across (world::Exhaust's jet at its
// visible reach); its primary smoke there at once - its particles are there from the nozzle, and launch photographs
// show the haze within a missile length of it (0.01 s, a missile length at launch speed; over 0.05 s it was
// missing for five) - the droplets over the first tenths of a second as the plume mixes
constexpr double kSmokeStartRadii = 28.0, kPrimaryShows = 0.01, kSecondaryShows = 0.3;
// The ice contrail. A solid motor's exhaust leaves its nozzle with water and hydrogen, which burns in the air into more
// (world/Propellants.h, NASA CEA: an aluminised motor's 5.7 % water and 2.6 % H2 come to 0.29 kg of water per kg, a
// reduced-smoke motor's 26 % and 0.8 % to 0.33). The heat it brings the plume per kg (Airflow::rocketHeat): its
// sensible heat from its exit's temperature (2,327 K, 1,648 K; c_p some 1,800 J/kg/K), the afterburning of its hydrogen
// and its carbon monoxide (5.8 MJ/kg, 2.3 MJ/kg) and the jet's kinetic energy relative to the air, (v_e - V)^2 / 2
// (v_e its delivered specific impulse's: what is not work on the missile). Its ice crystals: the exhaust is rich in particles (alumina, catalysts' oxides,
// soot, the HCl's droplets) - 1e15 per kg activated, CoCiP's default for a jet's soot per kg of fuel (an assumption),
// with CoCiP's activation near the threshold.
constexpr double kExhaustCp = 1800.0, kRocketIceN = 1e15;

// A sample drawn further along the path than it was laid (the head, the young trail's): at that height, its plumes
// with it - left at the sample's, a climbing missile's ice was drawn up to 15 m below its path, each stretch of the
// young trail a step
void moveAlong(Airflow::Sample& s, double flightAltitude) {
    const double dz = flightAltitude - s.flightAltitude;
    s.flightAltitude = flightAltitude;
    s.plume.altitude += dz;
    s.ice.altitude += dz;
}

double smoothstep(double e0, double e1, double x) {
    const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// How often a sample's CoCiP state is stepped as it ages (CoCiP's own step is 30 minutes; its formulas hold for any).
double stepInterval(double age) { return age < 30.0 ? 0.5 : age < 300.0 ? 5.0 : 30.0; }

} // namespace

Airflow::Sample Airflow::lay(const Emitter& e, const cocip::AirAt& air, const vsg::dvec3& at, double now) {
    const auto d = cocip::downwash(e.aircraft, e.altitude, air, e.shear);
    const double V = std::max(e.aircraft.trueAirspeed, 1.0);
    Sample s;
    s.at = at;
    s.birth = now;
    s.flightAltitude = e.altitude;
    s.vortexTime = d.vortexSpeed > 0.0 ? d.dzMax / d.vortexSpeed : 0.0;
    s.fuelDist = e.aircraft.fuelFlow / V;
    s.youngN = d.young.nIcePerM;
    s.separation = d.separation;
    s.dzMax = d.dzMax;
    s.descentSpeed = d.vortexSpeed;
    s.linkTime = cocip::crowBateLinkTime(d.epsilonStar, d.separation, d.vortexSpeed);
    s.rollUp = kRollUpSpans * e.aircraft.wingspan / V;
    s.jetHalf = e.jetHalf;
    s.jetRadius = e.jetRadius;
    s.jetWidth = 2.0 * (e.jetHalf + e.jetRadius);
    s.jetDepth = 2.0 * e.jetRadius;
    s.plume = d.plume;
    s.shear = e.shear;
    s.lastStep = now;
    return s;
}

double Airflow::secondarySmokeOnset(double T) {
    // HCl stays a gas over its droplets the more readily the warmer the air (its vapour pressure over a 20 % solution
    // some 30 Pa at 20 C, falling tenfold every 25 K or so), so the dilute plume needs more of the air's water to form
    // them warm than cold: 40 % at -30 C, rising to 90 % by +12 C (an engineering estimate of the trend, said so)
    return std::clamp(0.4 + 0.012 * (T - 243.15), 0.3, 0.9);
}

double Airflow::acidMassFraction(double rh) {
    // hydrochloric acid in equilibrium with water vapour at a relative humidity (its water activity), at room
    // temperature (approximate, from the solutions' published water vapour pressures): 35 % HCl at 25 %, 20 % at 73 %,
    // 5 % at 97 %
    static constexpr double a[] = {0.25, 0.40, 0.58, 0.73, 0.84, 0.92, 0.97}, w[] = {0.35, 0.30, 0.25, 0.20, 0.15, 0.10, 0.05};
    if (rh <= a[0]) return w[0];
    for (std::size_t i = 1; i < std::size(a); ++i)
        if (rh <= a[i]) return w[i - 1] + (w[i] - w[i - 1]) * (rh - a[i - 1]) / (a[i] - a[i - 1]);
    return w[std::size(w) - 1];
}

double Airflow::Smoke::massFlow() const {
    return flowKgS > 0.0 ? flowKgS : thrustN / (propellant(kind).ispS * kG0);
}

double Airflow::engineEfficiency(double thrustN, double speedMs, double fuelFlowKgS) {
    if (!(fuelFlowKgS > 0.0) || !(thrustN > 0.0) || !(speedMs > 0.0)) return kEngineEfficiency;
    return std::clamp(thrustN * speedMs / (fuelFlowKgS * kQFuel), 0.1, 0.5);
}

Airflow::SmokeExtinction Airflow::smokeExtinction(const Smoke& m, const cocip::Air& air) {
    SmokeExtinction e;
    if (!(m.thrustN > 0.0) || !(m.speed > 1.0)) return e;
    const int kind = std::clamp(m.kind, 0, 2);
    const Propellant& p = propellant(kind);
    const double perMetre = m.massFlow() / m.speed; // kg of exhaust per metre of trail
    e.primary = perMetre * (kind == 2 ? p.alumina * kAluminaExtinction : kResidueShare[kind] * kResidueExtinction);
    if (!(p.hcl > 0.0)) return e; // (no perchlorate: no HCl)
    const double rh = cocip::rh(air.specificHumidity, air.temperature, air.pressure);
    const double onset = secondarySmokeOnset(air.temperature);
    const double forms = smoothstep(onset, onset + 0.1, rh);
    e.secondary = perMetre * p.hcl * kAcidCondensed * kAcidExtinction * forms / acidMassFraction(rh);
    return e;
}

double Airflow::rocketWater(int smokeClass) { return propellant(smokeClass).water; }

double Airflow::rocketHeat(int smokeClass, double V, double T) {
    const Propellant& p = propellant(smokeClass);
    const double ve = p.ispS * kG0;
    return kExhaustCp * std::max(p.exitK - T, 0.0) + p.afterburning + 0.5 * (ve - V) * (ve - V);
}

cocip::Formation Airflow::iceFormation(const Smoke& m, const cocip::Air& air) {
    if (!(m.thrustN > 0.0) || !(m.speed > 1.0)) return {};
    return cocip::formationOf(air, rocketWater(m.kind), rocketHeat(m.kind, m.speed, air.temperature));
}

Airflow::Sample Airflow::laySmoke(const Smoke& m, double altitude, const cocip::Shear& shear, const cocip::AirAt& air, const vsg::dvec3& at,
                                  double now) {
    Sample s;
    s.smoke = true;
    s.at = at;
    s.birth = now;
    s.flightAltitude = altitude;
    s.shear = shear;
    s.lastStep = now;
    const SmokeExtinction e = smokeExtinction(m, air(altitude));
    s.smokePrimaryE = e.primary;
    s.smokeSecondaryE = e.secondary;
    s.plume.altitude = altitude;
    s.plume.width = s.plume.depth = std::max(0.5, kSmokeStartRadii * m.exitRadius);
    s.plume.alive = true;
    // its ice contrail, where the exhaust's water freezes: CoCiP's initial ice over the trail's section, on its particles
    const cocip::Air a = air(altitude);
    const cocip::Formation f = iceFormation(m, a);
    if (f.forms) {
        const double perMetre = m.massFlow() / m.speed;
        s.ice.altitude = altitude;
        s.ice.width = s.plume.width;
        s.ice.depth = s.plume.depth;
        s.ice.iwc = cocip::initialIwcOfWater(a, rocketWater(m.kind) * perMetre, s.ice.width, s.ice.depth);
        s.ice.nIcePerM = perMetre * kRocketIceN * cocip::iceActivation(a.temperature, f.tCritical);
        s.ice.alive = s.ice.iwc > 0.0;
        if (s.ice.alive) cocip::properties(s.ice, a, cocip::potentialTemperatureGradient(air, altitude), shear);
    }
    return s;
}

void Airflow::age(Sample& s, const cocip::AirAt& air, double now) {
    if (s.smoke) {
        if (now - s.lastStep >= stepInterval(now - s.birth)) {
            cocip::disperse(s.plume, air, s.shear, now - s.lastStep);
            if (s.ice.alive) cocip::step(s.ice, air, s.shear, now - s.lastStep, false);
            s.lastStep = now;
        }
        return;
    }
    if (s.plume.alive && now - s.lastStep >= stepInterval(now - s.birth)) {
        cocip::step(s.plume, air, s.shear, now - s.lastStep);
        s.lastStep = now;
    }
}

Airflow::Drawn Airflow::drawn(const Sample& s, const cocip::AirAt& air, double now) {
    const double age = now - s.birth;
    if (s.smoke) {
        // a rocket's smoke: the plume part alone, from its start, its section as it has spread; its ice seen as a jet's
        // is, its section the ice's own (fallen, spread with it) as far as it outweighs the smoke
        Drawn w;
        double smokeE = s.smokePrimaryE * smoothstep(0.0, kPrimaryShows, age) + s.smokeSecondaryE * smoothstep(0.0, kSecondaryShows, age);
        if (s.mist) smokeE = s.smokePrimaryE * std::exp(-age / kMistLifeS); // (a fuel mist evaporating)
        const double iceE = s.ice.alive ? std::max(cocip::extinctionPerMetre(s.ice), 0.0) * smoothstep(0.0, kCondense, age) : 0.0;
        const double k = smokeE + iceE > 0.0 ? iceE / (smokeE + iceE) : 0.0;
        w.plumeE = smokeE + iceE;
        w.plumeSyy = 0.125 * ((1.0 - k) * s.plume.width * s.plume.width + k * s.ice.width * s.ice.width);
        w.plumeSzz = 0.125 * ((1.0 - k) * s.plume.depth * s.plume.depth + k * s.ice.depth * s.ice.depth);
        w.plumeSyz = (1.0 - k) * s.plume.sigmaYz + k * s.ice.sigmaYz;
        w.plumeDrop = k * (s.flightAltitude - s.ice.altitude);
        if (s.fire) {
            w.albedo = s.albedo;
            w.flameK = s.handOver > 0.0 ? 0.0 : flameAt(age, s.fireScale);
            // (a near field drawn elsewhere: the share of its soot it has lost by now - the two together the soot's)
            if (s.handOver > 0.0) w.plumeE *= -std::expm1(-s.handOver * std::max(age, 0.0));
        }
        return w;
    }
    const double f = s.vortexTime > 0.0 ? smoothstep(0.0, s.vortexTime, age) : 1.0;
    const double width = s.jetWidth + (s.plume.width - s.jetWidth) * f;
    const double depth = s.jetDepth + (s.plume.depth - s.jetDepth) * f;
    const double plumeE = s.plume.alive ? cocip::extinctionPerMetre(s.plume) : 0.0;
    // through the vortex phase: CoCiP's initial ice for the plume's size as drawn
    double youngE = 0.0;
    if (width > 0.0 && depth > 0.0) {
        const cocip::Air a = air(s.flightAltitude);
        cocip::Plume y;
        y.width = width;
        y.depth = depth;
        y.iwc = cocip::initialIwc(a, s.fuelDist, width, depth);
        y.nIcePerM = s.youngN;
        cocip::properties(y, a, 0.0, s.shear);
        youngE = cocip::extinctionPerMetre(y);
    }
    const double condensed = smoothstep(0.0, kCondense, age);
    const double E = (youngE + (plumeE - youngE) * f) * condensed;
    // the wake: rolled up within kRollUpSpans spans, the pair descending at its speed to CoCiP's descent, the
    // Crow wave growing to touch at the lifespan, then the rings, which dissolve over another
    const double rolled = s.rollUp > 0.0 ? smoothstep(0.0, s.rollUp, age) : 1.0;
    const double b0 = s.separation;
    const double halfSpacing = s.jetHalf + (0.5 * b0 - s.jetHalf) * rolled;
    // (the pair's descent slowed by the buoyancy of the air it carries down, as in stably stratified air: its starting
    // speed easing to rest at CoCiP's descent, a quarter of the buoyant oscillation; at its starting speed to a stop at
    // the bottom, the trail's sinking part met its level part at a corner)
    const double descent = s.dzMax > 0.0 ? s.dzMax * std::sin(std::min(s.descentSpeed * age / s.dzMax, 1.5707963267948966)) : 0.0;
    const double link = std::max(s.linkTime, 1e-3);
    const double crow = 0.5 * b0 * std::exp(std::min(kCrowGrowth * s.descentSpeed / std::max(b0, 1e-3) * (age - link), 0.0));
    const double rings = smoothstep(0.85 * link, link, age);
    const double young = 1.0 - smoothstep(link, 2.0 * link, age);
    const double tube = s.jetRadius + (0.2 * b0 - s.jetRadius) * rolled;
    const double plumeDrop = s.flightAltitude - s.plume.altitude;
    // the parts: the primary wake carries the ice down, where it sublimates as CoCiP's descent has it; the secondary
    // wake stays at the flight level and keeps its ice while the air there is supersaturated over ice - behind a
    // B747-400 in air at about 101 % it persisted after the primary wake had sublimated at 170 s (Sussmann and Gierens
    // 1999) - until the wake has dissolved into the plume
    const cocip::Air flight = air(s.flightAltitude);
    const bool supersaturated = flight.specificHumidity >= cocip::qSatIce(flight.temperature, flight.pressure);
    Drawn w;
    w.primaryE = std::max(E, 0.0) * young * (1.0 - kSecondaryShare);
    w.secondaryE = (supersaturated ? youngE * condensed : std::max(E, 0.0)) * young * kSecondaryShare;
    w.plumeE = std::max(E, 0.0) * (1.0 - young);
    w.plumeSyy = 0.125 * s.plume.width * s.plume.width;
    w.plumeSzz = 0.125 * s.plume.depth * s.plume.depth;
    w.plumeSyz = s.plume.sigmaYz;
    w.plumeDrop = plumeDrop;
    w.b0 = b0;
    w.halfSpacing = halfSpacing;
    w.descent = descent;
    w.crow = crow;
    w.rings = rings;
    w.tube = tube;
    w.young = young;
    return w;
}

double Airflow::flameAt(double age, double scale) {
    if (age < 0.0) return 0.0;
    const double hold = kFlameS * scale, cool = kFlameCoolS * scale;
    const double T = age < hold ? kFlameK : kFlameK * std::exp(-(age - hold) / cool);
    return T >= 700.0 ? T : 0.0;
}

Airflow::Sample Airflow::layFire(double rateKgS, double airspeedMs, double altitude, const cocip::Shear& shear, const cocip::AirAt& air,
                                 const vsg::dvec3& at, double now) {
    (void)air;
    Sample s;
    s.smoke = true;
    s.fire = true;
    s.albedo = kSootAlbedo;
    s.at = at;
    s.birth = now;
    s.flightAltitude = altitude;
    s.shear = shear;
    s.lastStep = now;
    // its soot per metre of trail: what burns each second over the metres flown in it
    s.smokePrimaryE = std::max(rateKgS, 0.0) * kFireSootYield * kSootAbsorption / std::max(airspeedMs, 1.0);
    s.plume.altitude = altitude;
    s.fireScale = fireScale(rateKgS);
    s.plume.width = s.plume.depth = kFireWidth * s.fireScale;
    s.plume.alive = true;
    return s;
}

void Airflow::ignite(std::size_t slot, const std::string& component, double rateKgS, double now, double mistKgS) {
    for (auto& f : fires_)
        if (f.slot == slot && f.component == component) {
            f.rateKgS = rateKgS, f.mistKgS = std::max(mistKgS, 0.0);
            return;
        }
    if (rateKgS > 0.0) fires_.push_back(Fire{slot, component, rateKgS, now, std::max(mistKgS, 0.0)});
}

void Airflow::smokeFrom(std::size_t i, const sim::VehicleState& st, const VehicleVisuals::Shape& shape, const cocip::AirAt& air, double now) {
    // the motor (the first the manifest names): its thrust, its nozzle's exit where the model's node has it
    const int engine = shape.rockets.begin()->first;
    const auto& rocket = shape.rockets.begin()->second;
    const double thrust = engine < st.engineCount && engine < sim::VehicleState::kMaxEngines ? std::max(finite(st.thrustN[engine]), 0.0) : 0.0;
    const double V = finite(st.airspeedTrueMs), altitude = finite(st.altitudeMslM);
    vsg::dvec3 exit(0.0, 0.0, 0.0);
    for (const auto& jet : shape.jets)
        if (jet.engine == engine) exit = jet.exit;
    const bool burning = thrust > 0.0 && V > 30.0;
    Source* src = source(static_cast<int>(i), burning);
    if (!src) return;
    src->used = true;
    const vsg::dvec3 at = bodyToEcef(st) * exit;
    if (burning && (!src->forming || now - src->lastCommit >= kInterval)) {
        if (src->samples.size() >= kSamples - 1) { // full: the oldest half thinned, as a contrail's
            std::vector<Sample> kept;
            kept.reserve(kSamples);
            const std::size_t half = src->samples.size() / 2;
            for (std::size_t k = 0; k < src->samples.size(); ++k)
                if (k >= half || k % 2 == 0) kept.push_back(src->samples[k]);
            src->samples.swap(kept);
        }
        // (its propellant flow the simulation's, where it says it)
        const double burnt = i < flows_.size() && engine >= 0 && engine < sim::VehicleState::kMaxEngines ? flows_[i][static_cast<std::size_t>(engine)] : 0.0;
        const Smoke m{rocket.smoke, thrust, V, 0.5 * rocket.exitDiameterM, std::isfinite(burnt) ? std::max(burnt, 0.0) : 0.0};
        const cocip::Shear shear{kShear, kShear * 2.0 / 3.14159265358979323846};
        Sample s = laySmoke(m, altitude, shear, air, at, now);
        s.flown = src->samples.empty() ? 0.0 : src->samples.back().flown + vsg::length(at - src->samples.back().at);
        if (src->samples.empty()) src->phase = static_cast<double>(i % 7) * 0.9;
        s.gapBefore = !src->forming;
        s.up = -bodyAxisEcef(st, 2);
        src->samples.push_back(s);
        src->lastCommit = now;
    }
    src->forming = burning;
    src->headAt = at;
    src->headUp = -bodyAxisEcef(st, 2);
    src->headAltitude = altitude;
}

cocip::Air Airflow::airAt(const Air& a, double h) {
    // the world's standard atmosphere from its sea level's temperature and pressure (to the tropopause, isothermal
    // above), its humidity relative to water; none given, a typical day's (typicalHumidity)
    constexpr double g = 9.80665, Rd = 287.05, L = 0.0065, eps = 287.05 / 461.51;
    const double T0 = a.seaLevelK > 150.0 ? a.seaLevelK : 288.15, p0 = a.seaLevelPa > 1000.0 ? a.seaLevelPa : 101325.0;
    double T, p;
    if (h <= 11000.0) {
        T = T0 - L * h;
        p = p0 * std::pow(T / T0, g / (Rd * L));
    } else {
        const double T11 = T0 - L * 11000.0, p11 = p0 * std::pow(T11 / T0, g / (Rd * L));
        T = T11;
        p = p11 * std::exp(-g * (h - 11000.0) / (Rd * T11));
    }
    const double q = a.humidity > 0.0 ? std::min(a.humidity, 1.0) * eps * cocip::eSatLiquid(T) / p
                                      : typicalHumidityOverIce(h) * cocip::qSatIce(T, p);
    return {T, p, q};
}

double Airflow::typicalHumidityOverIce(double h) {
    // a typical day (an assumption: the world gives no humidity): near the ground 70 % over water, drying to 50 % by
    // 5 km; through the upper troposphere 60 % over ice - short of the ice saturation persistent contrails need, as
    // the air at cruise levels is most of the time; above the tropopause (11 km) the stratosphere's dry air, 30 %
    const auto overIce = [](double rhWater, double z) {
        const double T = 288.15 - 0.0065 * std::min(z, 11000.0);
        return rhWater * cocip::eSatLiquid(T) / cocip::eSatIce(T);
    };
    if (h <= 5000.0) return overIce(0.7 - 0.2 * std::clamp(h / 5000.0, 0.0, 1.0), std::max(h, 0.0));
    const double low = overIce(0.5, 5000.0), upper = 0.6, strat = 0.3;
    if (h <= 8000.0) return low + (upper - low) * (h - 5000.0) / 3000.0;
    if (h <= 11000.0) return upper;
    return upper + (strat - upper) * std::clamp((h - 11000.0) / 1000.0, 0.0, 1.0);
}


Airflow::Airflow(std::size_t slots, vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders, const AtmosphereTables& tables,
                 bool coarseShading) {
    (void)options;
    root_ = vsg::Group::create();
    switch_ = vsg::Switch::create();
    root_->addChild(switch_);
    aircraft_.assign(slots, vsg::dvec2(0.0, 0.0));
    const CloudNoise noise = shaders.empty() ? CloudNoise{} : loadCloudNoise(shaders.parent_path());
    if (shaders.empty() || !tables.valid() || !noise.valid()) {
        LOG_WARN("world") << "no contrails: the clouds' shaders, noises or the atmosphere's tables are missing";
        return;
    }
    const std::uint32_t table = static_cast<std::uint32_t>(kSources * kDrawn * kStride);
    const std::string cap = "#define CONTRAIL_CAP " + std::to_string(kDrawn) + "\n#define CONTRAIL_STRIDE " + std::to_string(kStride) +
                            "\n#define BLACKBODY_AT " + std::to_string(table) + "\n#define BLACKBODY_ROWS " +
                            std::to_string(SunlitBlackbody::kRows) + "\n";
    auto vertex = loadCloudShader(shaders, "contrail.vert", VK_SHADER_STAGE_VERTEX_BIT, cap);
    auto fragment = loadCloudShader(shaders, "contrail.frag", VK_SHADER_STAGE_FRAGMENT_BIT, cap);
    vsg::ShaderStages stages{vertex, fragment};
    auto compiler = vsg::ShaderCompiler::create();
    if (!vertex || !fragment || !compiler->supported() || !compiler->compile(stages)) {
        LOG_WARN("world") << "contrail shaders did not build; no contrails";
        return;
    }

    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr}};
    for (std::uint32_t b : {3u, 4u, 6u, 7u, 8u, 20u}) bindings.push_back({b, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, both, nullptr});
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
    samples_ = vsg::vec4Array::create(table + SunlitBlackbody::kRows, vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
    blackbody_.write(*samples_, table);
    frame_ = vsg::vec4Array::create(4, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    for (auto& d : {samples_, frame_}) d->properties.dataVariance = vsg::DYNAMIC_DATA;
    vsg::Descriptors descriptors{vsg::DescriptorBuffer::create(samples_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                 vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)};
    descriptors.push_back(vsg::DescriptorImage::create(noise.shape, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER));
    descriptors.push_back(vsg::DescriptorImage::create(noise.detail, 4, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER));
    for (const auto& d : tables.descriptors()) descriptors.push_back(d);
    auto set = vsg::DescriptorSet::create(setLayout, descriptors);

    // premultiplied over, depth tested and not written (a contrail hides nothing); each segment's box by its far faces
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_FRONT_BIT;
    auto assembly = vsg::InputAssemblyState::create();
    assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
    auto depth = vsg::DepthStencilState::create();
    depth->depthWriteEnable = VK_FALSE;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), assembly, raster, vsg::MultisampleState::create(), blend, depth};
    // shaded a fragment to 2 x 2 pixels on a large window (setScreen), the edges, depth and coverage still each pixel's:
    // the march is the costliest thing the frame draws, each pixel through every stretch of trail in front of it -
    // fullscreen at 3840 x 2160, inside a fighter's trail, 7.2 ms of an 11.4 ms frame - and the ice is soft
    if (coarseShading) {
        states.push_back(vsg::DynamicState::create(VK_DYNAMIC_STATE_FRAGMENT_SHADING_RATE_KHR));
        shadingRate_ = ShadingRate::create();
    }
    auto trails = vsg::StateGroup::create();
    trails->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, stages, states)));
    trails->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, set));
    ribbonDraw_ = vsg::Draw::create(14, 0, 0, 0); // (a box a segment, one strip)
    if (shadingRate_) trails->addChild(shadingRate_);
    trails->addChild(ribbonDraw_);
    ribbonAt_ = vsg::MatrixTransform::create();
    ribbonAt_->addChild(trails);
    switch_->addChild(true, ribbonAt_);
    state_ = trails;
    sources_.resize(kSources);
}

void Airflow::setVisible(bool on) {
    if (on == visible_) return;
    visible_ = on;
    switch_->setAllChildren(on);
    if (!on)
        for (auto& s : sources_) s = Source{};
}

void Airflow::setScreen(std::uint32_t width, std::uint32_t height) {
    screen_ = vsg::vec2(static_cast<float>(std::max<std::uint32_t>(width, 1)), static_cast<float>(std::max<std::uint32_t>(height, 1)));
    // a fragment to 2 x 2 pixels from 2 million pixels up (at 1600 x 900 each its own)
    coarse_ = shadingRate_ && static_cast<double>(width) * static_cast<double>(height) >= 2.0e6 ? 2u : 1u;
    if (shadingRate_) static_cast<ShadingRate*>(shadingRate_.get())->size = coarse_;
}

void Airflow::setAircraft(std::size_t slot, double wingspanM, double massWithoutFuelKg) {
    if (slot >= aircraft_.size()) aircraft_.resize(slot + 1, vsg::dvec2(0.0, 0.0));
    aircraft_[slot] = vsg::dvec2(std::max(wingspanM, 0.0), std::max(massWithoutFuelKg, 0.0));
}

Airflow::Source* Airflow::source(int slot, bool create) {
    Source* free = nullptr;
    for (auto& s : sources_) {
        if (s.slot == slot) return &s;
        if (!free && s.slot < 0) free = &s;
    }
    if (!free || !create) return nullptr;
    *free = Source{};
    free->slot = slot;
    free->samples.reserve(kSamples);
    return free;
}

void Airflow::update(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                     const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid, const vsg::dvec3& sunEcef) {
    if (!state_ || !visible_) return;
    const std::size_t n = std::min({states.size(), alive.size(), visuals.count()});

    // the world's clock: from the vehicles, the same for all of them
    double now = clock_;
    for (std::size_t i = 0; i < n && !std::isfinite(now); ++i)
        if (alive[i] && std::isfinite(states[i].simTime)) now = states[i].simTime;
    if (!std::isfinite(now)) now = lastTime_ >= 0.0 ? lastTime_ : 0.0;
    if (now < lastTime_ - 0.5) // a reset or a replay's seek: the trails were somewhere else
        for (auto& s : sources_) s = Source{};
    const double frameDt = lastTime_ >= 0.0 ? std::clamp(now - lastTime_, 0.0, 1.0) : 0.0;
    lastTime_ = now;

    const cocip::AirAt air = [this](double h) { return airAt(air_, h); };
    const double fromRad = vsg::radians(air_.windFromDeg);

    // nearest first: they get the trails when there are more than can be kept
    std::vector<std::pair<double, std::size_t>> order;
    for (std::size_t i = 0; i < n; ++i)
        if (alive[i] && visuals.visible(i) && !states[i].diverged && visuals.shape(i).valid)
            order.emplace_back(vsg::length(positionEcef(states[i]) - eye), i);
    std::sort(order.begin(), order.end());
    for (auto& s : sources_) s.used = false;
    for (const auto& [distance, i] : order) {
        if (distance >= kRange) break;
        const auto& st = states[i];
        const auto& shape = visuals.shape(i);
        if (!shape.rockets.empty()) {
            smokeFrom(i, st, shape, air, now);
            continue;
        }
        if (shape.jets.empty()) continue;
        const double V = finite(st.airspeedTrueMs);
        const double altitude = finite(st.altitudeMslM);
        // the jets: their middle, how wide they lie and how wide each is
        vsg::dvec3 mid(0.0, 0.0, 0.0);
        double lo = 1e9, hi = -1e9, radius = 0.0, thrust = 0.0;
        for (const auto& jet : shape.jets) {
            mid += jet.exit + jet.direction * 0.5;
            lo = std::min(lo, jet.exit.y);
            hi = std::max(hi, jet.exit.y);
            radius = std::max(radius, jet.radius);
            const int e = jet.engine;
            if (e >= 0 && e < st.engineCount && e < sim::VehicleState::kMaxEngines) thrust += std::max(finite(st.thrustN[e]), 0.0);
        }
        mid = mid / static_cast<double>(shape.jets.size());
        // the fuel flow: what its engines burn as the simulation has it; not told (a recording from before it was),
        // the thrust's power over the fuel's energy at pycontrails' engine efficiency
        double burnt = 0.0;
        if (i < flows_.size())
            for (int e = 0; e < std::min(st.engineCount, sim::VehicleState::kMaxEngines); ++e)
                burnt += std::isfinite(flows_[i][static_cast<std::size_t>(e)]) ? std::max(flows_[i][static_cast<std::size_t>(e)], 0.0) : 0.0;
        // (engines past the state's four burn as the four do)
        if (st.engineCount > sim::VehicleState::kMaxEngines) burnt *= static_cast<double>(st.engineCount) / sim::VehicleState::kMaxEngines;
        const double fuelFlow = burnt > 0.0 ? burnt : thrust * V / (kEngineEfficiency * kQFuel);
        const vsg::dvec2 known = i < aircraft_.size() ? aircraft_[i] : vsg::dvec2(0.0, 0.0);
        const double span = known.x > 0.0 ? known.x : std::max(2.0 * std::abs(shape.tip.y), 2.0);
        const double mass = (known.y > 0.0 ? known.y : 120.0 * span * span) + std::max(finite(st.fuelKg), 0.0);
        // the shear: its part across the track as the world's wind lies to it (2/pi of it, the mean, without wind)
        const vsg::dvec2 track(finite(st.velocityNedMs[1]), finite(st.velocityNedMs[0])); // east, north
        double across = 2.0 / 3.14159265358979323846;
        if (air_.windMs > 0.1 && vsg::length(track) > 1.0) {
            const vsg::dvec2 wind(-std::sin(fromRad), -std::cos(fromRad));
            const vsg::dvec2 t = vsg::normalize(track);
            across = std::abs(t.x * wind.y - t.y * wind.x);
        }
        const cocip::Shear shear{kShear, kShear * across};
        const cocip::Aircraft ac{span, mass, V, fuelFlow};
        // (the criterion at the efficiency it flies at: its thrust's power over what it burns, where that is told)
        const auto sac = cocip::formation(air(altitude), engineEfficiency(thrust, V, burnt));
        const bool forms = sac.forms && fuelFlow > 0.0 && V > 30.0;
        Source* src = source(static_cast<int>(i), forms);
        if (!src) continue;
        src->used = true;
        const vsg::dvec3 at = bodyToEcef(st) * mid;
        const vsg::dvec3 up = -bodyAxisEcef(st, 2);
        if (forms && (!src->forming || now - src->lastCommit >= kInterval)) {
            if (src->samples.size() >= kSamples - 1) { // (one left for the head)
                // full: the oldest half thinned to every other sample (their stretch of trail drawn coarser)
                std::vector<Sample> kept;
                kept.reserve(kSamples);
                const std::size_t half = src->samples.size() / 2;
                for (std::size_t k = 0; k < src->samples.size(); ++k)
                    if (k >= half || k % 2 == 0) kept.push_back(src->samples[k]);
                src->samples.swap(kept);
            }
            Sample s = lay(Emitter{ac, altitude, 0.5 * (hi - lo), radius, shear}, air, at, now);
            s.flown = src->samples.empty() ? 0.0 : src->samples.back().flown + vsg::length(at - src->samples.back().at);
            if (src->samples.empty()) src->phase = static_cast<double>(i % 7) * 0.9;
            s.gapBefore = !src->forming;
            s.up = up;
            src->samples.push_back(s);
            src->lastCommit = now;
        }
        src->forming = forms;
        src->headAt = at;
        src->headUp = up;
        src->headAltitude = altitude;
    }

    // the fires: each a trail of its own from where its component is on the vehicle - while it burns and the vehicle
    // is there
    for (std::size_t f = 0; f < fires_.size(); ++f) {
        const Fire& fire = fires_[f];
        const std::size_t i = fire.slot;
        const VehicleVisuals::Component* part = nullptr;
        if (i < n && alive[i] && visuals.shape(i).valid)
            for (const auto& c : visuals.shape(i).components)
                if (c.name == fire.component) { part = &c; break; }
        const bool burning = part && fire.rateKgS > 0.0 && now >= fire.since;
        // its unburnt fuel's mist: a trail of its own from the same place, while the fire's leak throws one out
        const bool misting = burning && fire.mistKgS > 0.0 && fireHandOver_ <= 0.0;
        if (Source* mist = source(kFireKey + 4096 + static_cast<int>(f), misting)) {
            mist->used = true;
            if (!misting) mist->forming = false;
            else {
                const auto& st = states[i];
                const vsg::dvec3 at = bodyToEcef(st) * part->at;
                const double V = finite(st.airspeedTrueMs), altitude = finite(st.altitudeMslM);
                if (!mist->forming || now - mist->lastCommit >= kInterval) {
                    const cocip::Shear shear{kShear, kShear * 2.0 / 3.14159265358979323846};
                    Sample m = layFire(fire.rateKgS, V, altitude, shear, air, at, now);
                    m.fire = false, m.mist = true, m.albedo = 1.0; // (white: droplets that scatter)
                    m.smokePrimaryE = fire.mistKgS * kMistExtinction / std::max(V, 1.0);
                    m.plume.width = m.plume.depth = kFireWidth * fireScale(fire.rateKgS + fire.mistKgS); // (all the leak's spray)
                    m.flown = mist->samples.empty() ? 0.0 : mist->samples.back().flown + vsg::length(at - mist->samples.back().at);
                    m.gapBefore = !mist->forming;
                    m.up = -bodyAxisEcef(st, 2);
                    if (mist->samples.size() >= kSamples - 1) mist->samples.erase(mist->samples.begin());
                    mist->samples.push_back(m);
                    mist->lastCommit = now;
                }
                mist->forming = true;
                mist->headAt = at, mist->headUp = -bodyAxisEcef(st, 2), mist->headAltitude = altitude;
            }
        }
        Source* src = source(kFireKey + static_cast<int>(f), burning);
        if (!src) continue;
        src->used = true;
        if (!burning) {
            src->forming = false;
            continue;
        }
        const auto& st = states[i];
        const vsg::dvec3 at = bodyToEcef(st) * part->at;
        const double V = finite(st.airspeedTrueMs), altitude = finite(st.altitudeMslM);
        if (!src->forming || now - src->lastCommit >= kInterval) {
            if (src->samples.size() >= kSamples - 1) { // full: the oldest half thinned, as a contrail's
                std::vector<Sample> kept;
                kept.reserve(kSamples);
                const std::size_t half = src->samples.size() / 2;
                for (std::size_t k = 0; k < src->samples.size(); ++k)
                    if (k >= half || k % 2 == 0) kept.push_back(src->samples[k]);
                src->samples.swap(kept);
            }
            const cocip::Shear shear{kShear, kShear * 2.0 / 3.14159265358979323846};
            Sample s = layFire(fire.rateKgS, V, altitude, shear, air, at, now);
            s.handOver = fireHandOver_;
            s.flown = src->samples.empty() ? 0.0 : src->samples.back().flown + vsg::length(at - src->samples.back().at);
            if (src->samples.empty()) src->phase = static_cast<double>(f % 7) * 0.9 + 0.4;
            s.gapBefore = !src->forming;
            s.up = -bodyAxisEcef(st, 2);
            src->samples.push_back(s);
            src->lastCommit = now;
        }
        src->forming = true;
        src->headAt = at;
        src->headUp = -bodyAxisEcef(st, 2);
        src->headAltitude = altitude;
    }

    // every sample: carried by the wind (the world's, east and north where the eye is), its CoCiP state stepped as it
    // ages; gone when its plume is
    vsg::dvec3 drift(0.0, 0.0, 0.0);
    if (air_.windMs > 0.0 && frameDt > 0.0) {
        const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(eye);
        const double lat = vsg::radians(lla.x), lon = vsg::radians(lla.y);
        const vsg::dvec3 E(-std::sin(lon), std::cos(lon), 0.0);
        const vsg::dvec3 N(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
        drift = (E * -std::sin(fromRad) + N * -std::cos(fromRad)) * (air_.windMs * frameDt);
    }
    for (auto& src : sources_) {
        if (src.slot < 0) continue;
        std::vector<Sample> kept;
        kept.reserve(src.samples.size());
        bool gap = false;
        for (auto& s : src.samples) {
            const double age = now - s.birth;
            s.at += drift;
            Airflow::age(s, air, now);
            // gone: its plume died (CoCiP), or thinned past seeing once the wake had dissolved into it; smoke, spread
            // past seeing, and its ice with it
            const bool faded = s.plume.alive && age > 2.0 * s.linkTime && age > s.vortexTime && s.plume.tau < kVisibleTau;
            const double iceTau = s.ice.alive ? s.ice.tau : 0.0;
            const bool evaporated = s.mist && s.smokePrimaryE * std::exp(-age / kMistLifeS) / std::max(s.plume.width, 1e-3) < kVisibleTau;
            const bool gone = evaporated ? true : s.smoke ? age > kSecondaryShows &&
                                            (s.smokePrimaryE + s.smokeSecondaryE) / std::max(s.plume.width, 1e-3) + iceTau < kVisibleTau
                                      : (!s.plume.alive || faded) && age > s.vortexTime + kCondense;
            if (gone) {
                gap = true;
                continue;
            }
            if (gap) s.gapBefore = true;
            gap = false;
            kept.push_back(s);
        }
        src.samples.swap(kept);
        if (src.samples.empty() && !src.used) src = Source{};
    }

    // the frame: the eye, a pixel's angle, the altitude correction (as the clouds'), the sun, the night light
    {
        const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(eye);
        const vsg::dvec3 surface = ellipsoid.convertLatLongAltitudeToECEF(vsg::dvec3(lla.x, lla.y, 0.0));
        const double a2 = ellipsoid.radiusEquator() * ellipsoid.radiusEquator(), b2 = ellipsoid.radiusPolar() * ellipsoid.radiusPolar();
        const vsg::dvec3 normal = vsg::normalize(vsg::dvec3(surface.x / a2, surface.y / a2, surface.z / b2));
        const vsg::dvec3 correction = -(surface - normal * kBottomRadius);
        const vsg::dvec3 sun = vsg::normalize(sunEcef);
        const double pixel = 1.0 / (screen_.y * 0.5 * std::abs(projection[1][1]));
        const Daylight d = daylightAt(sunEcef, normal);
        const double sunUp = vsg::dot(sun, vsg::normalize(eye));
        const double night = (1.0 - std::clamp((sunUp + 0.10) / 0.25, 0.0, 1.0)) * d.ambient;
        const auto F = [](double v) { return static_cast<float>(v); };
        frame_->set(0, vsg::vec4(F(eye.x), F(eye.y), F(eye.z), F(pixel * coarse_))); // (a fragment's: coarse_ pixels)
        frame_->set(1, vsg::vec4(F(correction.x), F(correction.y), F(correction.z), F(kBottomRadius)));
        frame_->set(2, vsg::vec4(F(sun.x), F(sun.y), F(sun.z), F(std::fmod(std::max(now, 0.0), 3600.0))));
        frame_->set(3, vsg::vec4(F(d.ambientColour.x * night), F(d.ambientColour.y * night), F(d.ambientColour.z * night), F(kExposure)));
        frame_->dirty();
    }

    // the samples as drawn: the jets rolled up into the wake's pair, the pair descending, bending and pinching into
    // rings, dissolving into CoCiP's plume. The young trail - where the jets' ice condenses and the wake rolls up, in
    // a fraction of a second - is drawn from samples at fixed ages (kYoungAges) between those laid, along the path
    // between them: it was drawn between samples laid 0.25 s apart, whose ages moved on frame after frame until the
    // next was laid, and the trail behind the jets faded and filled again four times a second
    std::size_t used = 0;
    std::vector<Sample> drawnSamples;
    drawnSamples.reserve(kDrawn);
    for (std::size_t k = 0; k < sources_.size(); ++k) {
        const Source& src = sources_[k];
        double sequence = 0.0;
        drawnSamples.clear();
        if (src.slot >= 0) {
            // the head: the newest sample again, at the jets now and just born
            const bool head = src.used && src.forming && !src.samples.empty();
            Sample h;
            if (head) {
                h = src.samples.back();
                h.flown += vsg::length(src.headAt - h.at);
                h.at = src.headAt;
                h.up = src.headUp;
                moveAlong(h, src.headAltitude);
                h.birth = now;
                h.gapBefore = false;
            }
            const std::size_t laid = src.samples.size() + (head ? 1 : 0);
            for (std::size_t j = 0; j < laid; ++j) {
                const Sample& a = j < src.samples.size() ? src.samples[j] : h;
                drawnSamples.push_back(a);
                if (j + 1 >= laid) break;
                const Sample& b = j + 1 < src.samples.size() ? src.samples[j + 1] : h;
                const double ta = now - a.birth, tb = now - b.birth;
                if (b.gapBefore || ta <= tb) continue;
                for (std::size_t g = std::size(kYoungAges); g-- > 0;) { // (oldest first)
                    const double age = kYoungAges[g];
                    if (age <= tb || age >= ta || drawnSamples.size() + (laid - j) >= kDrawn) continue;
                    const double w = (ta - age) / (ta - tb);
                    Sample v = a;
                    v.at = a.at + (b.at - a.at) * w;
                    v.flown = a.flown + (b.flown - a.flown) * w;
                    moveAlong(v, a.flightAltitude + (b.flightAltitude - a.flightAltitude) * w);
                    v.up = a.up + (b.up - a.up) * w;
                    v.birth = now - age;
                    v.gapBefore = false;
                    drawnSamples.push_back(v);
                }
            }
        }
        const std::size_t count = drawnSamples.size();
        for (std::size_t j = 0; j < kDrawn; ++j) {
            const auto at = static_cast<std::uint32_t>(kStride * (k * kDrawn + j));
            if (j >= count) {
                samples_->set(at, vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
                continue;
            }
            const Sample& s = drawnSamples[j];
            if (s.gapBefore) sequence += 1.0;
            const Drawn w = drawn(s, air, now);
            const vsg::dvec3 rel = s.at - eye;
            const auto F = [](double v) { return static_cast<float>(v); };
            samples_->set(at, vsg::vec4(F(rel.x), F(rel.y), F(rel.z), F(sequence)));
            samples_->set(at + 1, vsg::vec4(F(w.plumeSyy), F(w.plumeSzz), F(w.plumeSyz), F(w.plumeE)));
            samples_->set(at + 2, vsg::vec4(F(w.halfSpacing), F(w.descent), F(w.crow), F(w.rings)));
            samples_->set(at + 3, vsg::vec4(F(w.tube), F(s.flown), F(w.primaryE), F(w.plumeDrop)));
            samples_->set(at + 4, vsg::vec4(F(w.b0), F(src.phase), F(w.secondaryE), F(w.total())));
            samples_->set(at + 5, vsg::vec4(F(s.up.x), F(s.up.y), F(s.up.z), F(w.albedo)));
            // the flame: its soot's mean temperature, fluctuating by kFlicker (0: none)
            const double flameK = w.flameK > 0.0 ? SunlitBlackbody::meanOfFlicker(w.flameK, kFlicker) : 0.0;
            samples_->set(at + 6, vsg::vec4(F(flameK), F(kFlicker), 0.0f, 0.0f));
            sequence += 1.0;
        }
        if (src.slot >= 0) used = k + 1;
    }
    samples_->dirty();
    ribbonDraw_->instanceCount = static_cast<std::uint32_t>(used * kDrawn);
    ribbonAt_->matrix = vsg::translate(eye);
}

} // namespace fsim::world
