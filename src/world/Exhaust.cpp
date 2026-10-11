#include "world/Exhaust.h"

#include "core/Log.h"
#include "world/Propellants.h"

#include <algorithm>
#include <cmath>

// The engines' exhaust, ported from aeronautic's @aeronautic/afterburner (MIT, (c) 2026 Romain Le Gall,
// https://github.com/Romainlg29/aeronautic - see THIRD_PARTY_NOTICES.md): the jet worked out from the nozzle and the
// gas (packages/afterburner/src/plume-profile.ts), its field and its radiative transfer (src/tsl/plume.ts and
// src/afterburner-material.ts), translated from TypeScript and TSL to C++ and GLSL. What it does is theirs; what is
// changed is said where it is.

namespace fsim::world {

namespace {

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double smoothstepd(double e0, double e1, double x) {
    const double t = clampd((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// --- the engine and the camera: a large military turbofan at reheat, aeronautic's `afterburner` preset with its
// `JET_PROFILE` (presets.ts) - what its own fighter example flies (apps/docs/examples/fighter.tsx) ----------------
struct Engine {
    double exitMach = 1.45, pressureRatio = 1.35;        // a convergent-divergent nozzle, a little underexpanded
    double exitTemperatureK = 1700.0, dryTemperatureK = 1000.0;
    double gamma = 1.3, molarMass = 28.8;
    double sootPerM = 0.5, sootSurvival = 0.03;          // a trace of soot from the spray bars
    double afterburningK = 450.0;                        // what burns in the shear layer
    double bandStrength = 0.06;                          // CH* / C2*: the lavender sheath, the violet up the nozzle
    double turbulence = 0.12, meander = 0.02; // (the preset's: "almost no turbulence to break it up")
    double trainMean = 0.0; // fsim: a rocket's: how much of its core holds the exit's temperature through its shock train
    double oxidised = 0.0;  // fsim: a rocket's: how much of its particles burn where its shear layer burns (carbon; not alumina)
};
struct Profile {
    double adaptation = 0.6;
    double visibleTemperatureK = 900.0, maxLengthD = 40.0;
    double burnerThreshold = 1.0, idlePressure = 0.65, idleTemperature = 0.4;
    double coreScale = 1.0, spreadScale = 1.0;
    double shockFloor = 0.1, shockPersistence = 1.0;
};

constexpr double kMaxThrottle = 1.1, kMinReheat = 0.35, kLightOff = 0.03;
constexpr double kPhotopicK = 25900.0, kMaxAdaptation = 1e8, kNozzleMaxMach = 2.2, kMaxCoflow = 0.85;
constexpr double kFarSpread = 0.11, kShockDecays = 3.0, kQuenchDensity = 0.03, kSootPressureExponent = 1.5;
constexpr double kFieldExtent = 2.8, kEddyReach = 0.35, kAirMolarMass = 28.97, kGasConstant = 8314.46;
// The camera's exposure as aeronautic's examples set it (apps/docs/examples/stage.tsx): at night the profile's own
// (types.ts default_afterburner_profile: 120), by day stopped down to what the sky leaves the camera - a blackbody at
// 2000 K (4.8e5 cd/m2) against a clear sky (6000 cd/m2) drawn at its background's luminance, #6f9fd8 - eased in stops
// (exposure_under)
constexpr double kNightExposure = 120.0, kDayExposure = 26.4616;

// fsim: the engine of a kind. An augmented turbofan is the preset above (a fighter's, its mixed exhaust at some 1000 K
// at military power behind a convergent-divergent nozzle). One without an afterburner has a convergent nozzle, choked
// at most (exit Mach 1), and its core's gas leaves cooler the more of the work its turbines take drives a fan: some
// 1000 K out of a turbojet's or a low-bypass engine's, 850 K out of a high-bypass one's (the CFM56's core exhaust at
// take-off power) - and at cruise less, its turbine's face below 900 K, a glow no eye sees by day. Drawn as the
// preset, a tanker's or a bomber's nozzles glowed orange: the preset's turbine at 1,200 K at cruise
Engine engineOf(const Exhaust::Kind& kind) {
    Engine e;
    if (kind.augmented) {
        // its reheat as hot as its own ratings make it (Exhaust::reheatTotalK): the gas at its exit, where the preset's
        // 1,700 K is 2,236 K of total temperature at its exit's Mach number - an engine that adds less thrust burns
        // cooler and shows a shorter, redder flame; one that adds more, a longer and whiter one
        const double total = Exhaust::reheatTotalK(kind.bypassRatio, kind.dryThrustN, kind.reheatThrustN);
        if (total > 0.0) e.exitTemperatureK = total / (1.0 + 0.5 * (e.gamma - 1.0) * e.exitMach * e.exitMach);
        return e;
    }
    const double bypass = std::isfinite(kind.bypassRatio) ? std::max(kind.bypassRatio, 0.0) : 0.0;
    e.exitMach = 1.0;
    e.pressureRatio = 1.1; // (a convergent nozzle a little underexpanded, at most)
    e.dryTemperatureK = 1000.0 - 150.0 * smoothstepd(1.0, 4.0, bypass);
    e.exitTemperatureK = e.dryTemperatureK; // (no burner)
    e.afterburningK = 0.0;
    return e;
}

// fsim: a missile's solid rocket motor. Its gas leaves its nozzle as NASA's equilibrium code has its propellant class
// leave it (world/Propellants.h: an aluminised motor's at 2,327 K and Mach 2.88, gamma 1.135 with its alumina; a
// reduced-smoke motor's at 1,648 K and Mach 2.79, gamma 1.20 - from 3,456 K and 2,962 K chambers at 10 MPa), its
// pressure 3.1 - 3.6 times sea level's at its full thrust and following the chamber's - the thrust's - and, fixed by the
// nozzle, under-expanded more the higher it flies (as the model has a dry jet's: no burner, `breathes` 0).
// Its exhaust is rich in H2 and CO that burn in the air round it (the afterburning a low plume shows, quenched high up
// by the thin air as the model has it). Its particles by its propellant: an aluminised motor's alumina, glowing and
// dense; a reduced-smoke motor's little (no aluminium: its trail is HCl's condensation, the trail's business); a
// minimum-smoke motor's next to none. The alumina that cools leaves the plume as the smoke trail, drawn there: here
// only what still glows. (Brown, "Spacecraft propulsion"; Simmons, "Rocket exhaust plume phenomenology", 2000: the
// numbers are those classes' typical values, not one motor's)
Engine rocketOf(const Exhaust::Kind& kind) {
    Engine e;
    const Propellant& p = propellant(static_cast<int>(kind.smoke));
    e.exitMach = p.exitMach;
    e.pressureRatio = p.exitPressure; // (the exit's static pressure over sea level's at the full thrust)
    e.exitTemperatureK = e.dryTemperatureK = p.exitK;
    e.gamma = p.gamma;
    e.molarMass = p.molarMass;
    e.afterburningK = 700.0;
    e.bandStrength = 0.03;
    e.turbulence = 0.2;
    e.meander = 0.04;
    // (a reduced-smoke motor's core still glows: its fuel-rich HTPB's carbon and its burn-rate catalyst's oxide - a
    // bright core of a few metres in launch photographs; a minimum-smoke motor's, nitramine and double base, far less)
    //
    // Held to launch photographs (docs/sdk/viewer.md, "A missile's plume held to launch photographs"): measured from its
    // nozzle in missile lengths L, three of an AIM-7's (aluminised) boost show it saturated white from the nozzle to
    // 1.6 - 2.3 L, 1.5 - 2.0 body diameters wide at 0.1 L and 3.2 - 4.1 at 1 L, closing like a tongue; an AIM-120's
    // (reduced smoke) in daylight a glow at the nozzle some 1.5 times the sky, fading by 0.5 L into a white haze - and
    // none a dark gap behind the nozzle, which the jet drawn as an ideal gas expanding (at 6 km its core 1,400 K, a
    // thirtieth of the light at 1,700 K) left for half a missile length, nor shock cells. By class:
    // - aluminised: hotter (its chamber 3,456 K against a reduced-smoke motor's 2,962 K, NASA CEA), its exhaust a third
    //   alumina by mass (0.34), whose heat - the liquid's freezing at 2,327 K - holds the gas's temperature through the
    //   expansion (the two phases' gamma 1.135) - its core at its exit's temperature through the unresolved shock train
    //   (trainMean); its alumina, not burnt in the shear layer, glowing
    //   there - 12 /m, half the extinction its 34 % at the exit's density gives (0.27 kg/m3 x 0.34 x 250 m2/kg: alumina
    //   scatters more than it absorbs); once cool it is the smoke trail's, white (world::Airflow), not drawn here;
    // - reduced smoke: its core too held at the exit's temperature (the recompression behind each Mach disk, unresolved
    //   at a photograph's or a screen's scale, keeps the cells' light near the exit's), its carbon burnt where the shear
    //   layer burns (oxidised) - so its light is the core's near the nozzle, as the photographs show it, not a flame
    //   half a missile length aft;
    // - minimum smoke: a double base's or a nitramine's cooler exit (an estimate: Propellants.h), the same.
    switch (kind.smoke) {
    case Exhaust::Kind::Aluminised: e.sootPerM = 12.0, e.sootSurvival = 0.0, e.trainMean = 1.0; break;
    case Exhaust::Kind::Reduced: e.sootPerM = 1.2, e.sootSurvival = 0.05, e.trainMean = 1.0, e.oxidised = 1.0; break;
    case Exhaust::Kind::Minimum: e.sootPerM = 0.15, e.sootSurvival = 0.02, e.trainMean = 1.0, e.oxidised = 1.0; break;
    }
    return e;
}
// Its profile: the gas as hot at any thrust (only the chamber's pressure follows it), no burner to light, the visible
// flame as long as a rocket's (some tens of exit diameters), its shear layer spreading 2.2 times a jet's - an AIM-7's
// flame widening some 0.08 of its length each side in launch photographs, the compressible layer's 0.036 (its heat
// release dilating it: held to the photographs, said so). Its shock train dies out within a few cells: high up, an
// under-expanded rocket jet (Mach 3.6 out of the AIM-120's nozzle at 6 km) gave the train the strength of a jet's
// reheat and 12 m of length, cold gas between its cells - a string of separate hot beads, which the aluminised motors'
// launch photographs do not show (a continuous flame, their alumina glowing between the cells; the diamonds of a
// booster near the ground are not lost: they are its first cells). A reduced-smoke motor's may: an AIM-120 off an F-14A
// shows a string of beads behind its nozzle, an F-35A's its glow dipping to 0.4 of its peak within half a missile
// length (docs/sdk/viewer.md, "At the launch") - not drawn, the open difference
Profile rocketProfile() {
    Profile p;
    p.idlePressure = 0.0, p.idleTemperature = 1.0, p.burnerThreshold = 0.0;
    p.maxLengthD = 80.0, p.visibleTemperatureK = 1000.0;
    p.shockPersistence = 0.3;
    p.spreadScale = 2.2;
    return p;
}

// The International Standard Atmosphere (to 32 km): temperature (K), pressure and density over sea level's.
struct Air {
    double temperatureK, pressure, density, sound, ram;
};
Air air(double altitudeM, double airspeed) {
    const double h = clampd(altitudeM, 0.0, 32000.0);
    const double g0 = 9.80665, R = 287.05287, HYD = g0 / R;
    double T = 288.15, p = 1.0;
    const double layers[3][2] = {{0.0, -0.0065}, {11000.0, 0.0}, {20000.0, 0.001}};
    for (int i = 0; i < 3; ++i) {
        const double base = layers[i][0], lapse = layers[i][1], top = i < 2 ? layers[i + 1][0] : 1e9;
        const double rise = std::min(h, top) - base;
        if (rise <= 0.0) break;
        const double next = T + lapse * rise;
        p *= lapse == 0.0 ? std::exp(-HYD * rise / T) : std::pow(next / T, -HYD / lapse);
        T = next;
    }
    Air a;
    a.temperatureK = T;
    a.pressure = p;
    a.density = p * 288.15 / T;
    a.sound = std::sqrt(1.4 * R * T);
    // the stagnation over static pressure an inlet recovers (aeronautic's ram_pressure)
    const double m = std::max(airspeed, 0.0) / a.sound;
    const double recovery = m > 1.0 ? 1.0 - 0.075 * std::pow(m - 1.0, 1.35) : 1.0;
    a.ram = std::pow(1.0 + 0.2 * m * m, 3.5) * std::max(recovery, 0.2);
    return a;
}

// How much of the burner is lit at a throttle travel (0 .. 1 dry, to 1.1 at full reheat): its first zone lights with a
// thump, the rest stage in behind it (plume-profile.ts burner_lit)
double burnerLit(double travel, const Profile& p) {
    if (p.burnerThreshold <= 0.0) return 1.0;
    const double lit = smoothstepd(p.burnerThreshold, p.burnerThreshold + kLightOff, travel);
    const double staged = smoothstepd(p.burnerThreshold, std::max(p.burnerThreshold + kLightOff, kMaxThrottle), travel);
    return lit * (kMinReheat + (1.0 - kMinReheat) * staged);
}

// Where the axis stops being supersonic, in potential cores (plume-profile.ts sonic_distance)
double sonicDistance(double excess, double jetK, double ambientK, double gammaR) {
    const double heat = gammaR * (jetK - ambientK);
    const double speed = std::max(excess * excess, 1.0);
    const double share = clampd((heat + std::sqrt(heat * heat + 4.0 * speed * gammaR * ambientK)) / (2.0 * speed), 0.05, 1.0);
    return std::pow(std::max(std::pow(share, -4.0) - 1.0, 0.0), 0.25);
}

// The nozzle's outline as a superellipse fitted to a round exit of the same area (nozzle-outline.ts)
void outlineFit(double aspect, double squareness, double& areaScale, double& reach) {
    const double n = clampd(squareness, 0.5, 24.0);
    const double stretch = std::sqrt(std::max(aspect, 1e-3));
    areaScale = std::sqrt(3.14159265358979 * std::tgamma(1.0 + 2.0 / n) / (4.0 * std::pow(std::tgamma(1.0 + 1.0 / n), 2.0)));
    reach = areaScale * std::max(stretch, 1.0 / stretch) * std::pow(2.0, std::max(0.0, 0.5 - 1.0 / n));
}

// The jet one nozzle makes (plume-profile.ts jet_state, and what tsl/plume.ts plume_jet adds for the fragment)
struct Jet {
    double burner, breathes, radius, velocity, coreLength, reach, temperature, afterburning, kinetic, reheat, thinning;
    double lipTemperature, compression, sootSurvival, sootFormed, adaptation, glow, pipeFlame, glowTemperature;
    double shockHeat, shockLength, shockSpacing, firstDisk, spreadNear, spreadFar, outerNear, outerFar, ambientK;
};
Jet jetState(const Engine& e, const Profile& pr, double nozzleRadius, double throttle, const Air& a, double airspeed) {
    Jet j{};
    const double travel = clampd(throttle, 0.0, kMaxThrottle);
    const double setting = std::min(travel, 1.0);
    j.burner = burnerLit(travel, pr);
    const double dryK = a.temperatureK + (e.dryTemperatureK - a.temperatureK) * (pr.idleTemperature + (1.0 - pr.idleTemperature) * setting);
    const double exitK = dryK + (e.exitTemperatureK - dryK) * j.burner;
    j.breathes = smoothstepd(0.0, 0.2, 1.0 - e.dryTemperatureK / std::max(e.exitTemperatureK, 1.0));
    const double ambientScale = j.breathes * a.ram + (1.0 - j.breathes) / a.pressure;
    const double builtRatio = std::max(e.pressureRatio * (pr.idlePressure + (1.0 - pr.idlePressure) * setting) * ambientScale, 0.05);
    const double gamma = std::max(e.gamma, 1.05), k = 0.5 * (gamma - 1.0), exponent = gamma / (gamma - 1.0);
    const double exitMach = std::max(e.exitMach, 0.2);
    const double exitStag = 1.0 + k * exitMach * exitMach;
    const double totalOverAmbient = std::pow(exitStag, exponent) * builtRatio;
    const double widest = 1.0 + k * kNozzleMaxMach * kNozzleMaxMach;
    const double lipStag = std::max(std::min(exitStag * std::pow(std::max(a.ram, 1.0), j.breathes / exponent), widest), exitStag);
    j.lipTemperature = exitK * exitStag / lipStag;
    const double pressureRatio = std::max(totalOverAmbient / std::pow(lipStag, exponent), 0.05);
    const double mach = std::max(std::sqrt(std::max(std::pow(totalOverAmbient, 1.0 / exponent) - 1.0, 0.0) / k), 0.2);
    const double jetStag = 1.0 + k * mach * mach;
    j.temperature = exitK * exitStag / jetStag;
    const double area = (exitMach / mach) * std::pow(jetStag / exitStag, (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    const double expanded = clampd(area, 0.25, 36.0);
    j.radius = nozzleRadius * std::sqrt(expanded);
    const double gasR = kGasConstant / std::max(e.molarMass, 1.0);
    const double sound = std::sqrt(gamma * gasR * j.temperature);
    j.velocity = mach * sound;
    const double exitVelocity = exitMach * std::sqrt(gamma * gasR * exitK);
    j.thinning = exitVelocity / std::max(expanded * j.velocity, 1e-3);
    j.ambientK = a.temperatureK;
    const double as = std::max(airspeed, 0.0);
    const double convective = std::max(j.velocity - as, 0.0) / (sound + a.sound);
    const double coflow = clampd(as / j.velocity, 0.0, kMaxCoflow);
    const double shear = (1.0 - coflow) / (1.0 + coflow);
    const double compressibility = 0.23 + 0.77 * std::exp(-3.4 * convective * convective);
    const double density = (a.temperatureK / std::max(j.temperature, 1.0)) * (e.molarMass / kAirMolarMass);
    const double diameter = 2.0 * j.radius;
    j.coreLength = pr.coreScale * diameter * (4.2 + 1.1 * mach * mach) * std::pow(clampd(density, 0.01, 10.0), 0.28) / shear;
    j.spreadFar = pr.spreadScale * kFarSpread * shear;
    j.spreadNear = j.spreadFar * compressibility;
    const double supersonic = smoothstepd(1.0, 1.2, mach);
    j.shockSpacing = 1.306 * diameter * std::sqrt(std::max(mach * mach - 1.0, 0.04));
    const double mismatch = 1.0 - std::exp(-4.0 * std::abs(std::log(pressureRatio)));
    const double diamonds = 1.0 + (j.burner - 1.0) * j.breathes; // (a dry jet's train is too cool to glow)
    const double strength = (pr.shockFloor + (1.0 - pr.shockFloor) * mismatch) * supersonic * diamonds;
    j.shockHeat = strength * (jetStag - 1.0);
    const double sonic = j.coreLength * sonicDistance(j.velocity - as, j.temperature, a.temperatureK, gamma * gasR);
    j.shockLength = pr.shockPersistence * sonic / kShockDecays;
    const double normal = (gamma + 1.0) * mach * mach / ((gamma - 1.0) * mach * mach + 2.0);
    j.compression = 1.0 + (normal - 1.0) * strength;
    j.firstDisk = 0.5 + 0.3 * smoothstepd(0.9, 1.3, pressureRatio);
    const double visibleExcess = std::max(pr.visibleTemperatureK - a.temperatureK, 1.0);
    const double quench = a.density * (1.0 + kQuenchDensity) / (a.density + kQuenchDensity);
    j.afterburning = std::max(e.afterburningK * j.burner, 0.0) * quench;
    j.reheat = j.afterburning * j.breathes;
    j.sootSurvival = 1.0 + (clampd(e.sootSurvival, 0.0, 1.0) - 1.0) * quench;
    const double pressed = std::min(std::pow(std::max(a.pressure * a.ram, 0.0), kSootPressureExponent), 3.0);
    const double flame = smoothstepd(1200.0, 1500.0, exitK);
    j.sootFormed = 1.0 + (pressed * flame - 1.0) * j.breathes;
    const double excessSpeed = std::max(j.velocity - as, 0.0);
    j.kinetic = excessSpeed * excessSpeed * (gamma - 1.0) / (8.0 * gamma * gasR);
    const double hottestExcess = std::max(j.temperature - a.temperatureK, 0.0) + 4.0 * (j.afterburning + j.kinetic);
    const double thermal = j.coreLength / std::max(visibleExcess / std::max(hottestExcess, 1e-3), 1e-3);
    const double extinction = std::max(e.sootPerM, 0.0) * j.sootFormed * j.sootSurvival * j.thinning;
    const double smokeDepth = extinction * j.coreLength * j.spreadFar * 2.13;
    const double shortest = std::max(8.0 * j.radius, (j.firstDisk + 1.5) * j.shockSpacing);
    const double longest = std::max(pr.maxLengthD * 2.0 * nozzleRadius, shortest);
    j.reach = clampd(std::max(thermal, smoothstepd(0.01, 0.06, smokeDepth) * longest), shortest, longest);
    // what shows up the nozzle: the turbine and the pipe dry, the burner's flame lit - at the gas's total temperature
    // there (the turbine's exit, the gas still slow: some 650 K at idle, 1150 K at military power), glowing as hot metal
    // does, Planck's law at that temperature: nothing to see at idle, a dull orange deep in the pipe at full dry power
    const double lighting = std::min(j.burner / kMinReheat, 1.0);
    j.glow = j.breathes * (1.0 - lighting);
    j.pipeFlame = j.breathes * lighting * std::max(e.sootPerM, 0.0) * j.sootFormed;
    j.glowTemperature = exitK * exitStag;
    // how far the camera opens up for a burner dimmer than at full power
    j.adaptation = clampd(std::exp(clampd(pr.adaptation, 0.0, 1.0) * lighting * kPhotopicK *
                                   (1.0 / std::max(exitK, 1.0) - 1.0 / std::max(e.exitTemperatureK, 1.0))),
                          1.0, kMaxAdaptation);
    const double extent = (kFieldExtent + std::max(e.turbulence, 0.0) * kEddyReach + std::max(e.meander, 0.0)) * 1.05;
    const double farWidth = j.radius + j.spreadNear * std::min(j.reach, j.coreLength) + j.spreadFar * std::max(j.reach - j.coreLength, 0.0);
    j.outerNear = j.radius * extent;
    j.outerFar = farWidth * extent;
    return j;
}

// Planck's law through the CIE 1931 observer (Wyman, Sloan and Shirley's fit) into linear sRGB, per kelvin
// (blackbody.ts): its chromaticity and its log2 luminance against 2000 K's, from 500 to 6000 K
void blackbody(double T, double& r, double& g, double& b, double& y) {
    const auto lobe = [](double nm, double mean, double below, double above) {
        const double t = (nm - mean) / (nm < mean ? below : above);
        return std::exp(-0.5 * t * t);
    };
    double X = 0.0, Y = 0.0, Z = 0.0;
    for (double nm = 380.0; nm <= 780.0; nm += 2.0) {
        const double m = nm * 1e-9;
        const double L = 3.7418e-16 / std::pow(m, 5.0) / std::expm1(0.014388 / (m * T));
        X += L * (1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7) - 0.065 * lobe(nm, 501.1, 20.4, 26.2));
        Y += L * (0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1));
        Z += L * (1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8));
    }
    r = std::max(0.0, 3.2406 * X - 1.5372 * Y - 0.4986 * Z);
    g = std::max(0.0, -0.9689 * X + 1.8758 * Y + 0.0415 * Z);
    b = std::max(0.0, 0.0557 * X - 0.204 * Y + 1.057 * Z);
    y = Y;
}

constexpr std::uint32_t kBlackbody = 256;

// The march (afterburner-material.ts, the fragment shader of a hull per engine as it was): a function of the ray in the
// flame's own frame, for the pass that marches the gas round the aircraft (WingVapour). Its numbers at bindings 11
// (the rows, kRows per flame), 12 (the blackbody table) and 13 (time, daylight, exposure).
const char* kMarch = R"(
layout(std430, set = 0, binding = 11) readonly buffer ExhaustRows { vec4 rows[]; };
layout(std430, set = 0, binding = 12) readonly buffer Blackbody { vec4 bb[]; };
layout(set = 0, binding = 13) uniform ExhaustFrame { vec4 frame; };
int inst;
const int EXHAUST_ROWS = 10;
const float EPS = 1e-5;
const float FIELD_EXTENT = 2.8, EDDY_REACH = 0.35;
const float BARREL_RIM = 0.55, BARREL_BOW = 0.35, BARREL_WIDTH = 0.06, BARREL_HEAT = 0.55;
const vec3 SOOT_SPECTRUM = vec3(550.0 / 610.0, 1.0, 550.0 / 465.0);
const vec3 BAND_COLOR = vec3(0.153, 0.188, 1.0); // #6d78ff, linear
const vec3 LUMINANCE = vec3(0.2126, 0.7152, 0.0722);
const float SHOCK_RELAXATION = 3.2, ACTIVATION_K = 18000.0, EDDY_STRETCH = 2.5, SHUTTER = 1.0 / 60.0, MEANDER_SCALE = 5.0;
const float STEP_SAFETY = 0.8, DITHER_SCALE = 37.0, GAUSSIAN_DEPTH = 2.13, PLUME_OPAQUE = 0.01;
const float SAMPLES_PER_CELL = 5.0, SAMPLES_ACROSS_CORE = 4.0, SHOCK_TRAIN_LENGTHS = 3.0;
const float TURBINE_DEPTH = 2.0, PIPE_GLOW_LIP = 0.55, PIPE_GLOW = 0.8;
const float PIPE_EMISSIVITY = 0.85; // (oxidised nickel alloy, the turbine's coated blades)
const int NEAR_STEPS = 64, MID_STEPS = 24;

// the jet, as the CPU worked it out (Exhaust::set)
float radius, coreLength, reach, velocity, temperature, afterburning, shockHeat, shockLength, spacing, firstDisk;
float spreadNear, spreadFar, soot, sootSurvival, reheat, thinning, lipT, compression, adaptation, band;
float turbulence, meander, seed, outerFar, glow, exitRadius, glowT, pipeFlame, aspect, squareness, areaScale, outlineReach;
float ambient, ambientDensity, meanTrain, oxidised;
void load() {
    vec4 r0 = rows[EXHAUST_ROWS * inst], r1 = rows[EXHAUST_ROWS * inst + 1], r2 = rows[EXHAUST_ROWS * inst + 2], r3 = rows[EXHAUST_ROWS * inst + 3];
    vec4 r4 = rows[EXHAUST_ROWS * inst + 4], r5 = rows[EXHAUST_ROWS * inst + 5], r6 = rows[EXHAUST_ROWS * inst + 6], r7 = rows[EXHAUST_ROWS * inst + 7];
    vec4 r8 = rows[EXHAUST_ROWS * inst + 8];
    radius = r0.x; coreLength = r0.y; reach = r0.z; velocity = r0.w;
    temperature = r1.x; afterburning = r1.y; shockHeat = r1.z; shockLength = r1.w;
    spacing = r2.x; firstDisk = r2.y; spreadNear = r2.z; spreadFar = r2.w;
    soot = r3.x; sootSurvival = r3.y; reheat = r3.z; thinning = r3.w;
    lipT = r4.x; compression = r4.y; adaptation = r4.z; band = r4.w;
    turbulence = r5.x; meander = r5.y; seed = r5.z; outerFar = r5.w;
    glow = r6.x; exitRadius = r6.y; glowT = r6.z; pipeFlame = r6.w;
    aspect = r7.x; squareness = r7.y; areaScale = r7.z; outlineReach = r7.w;
    ambient = r8.x;
    ambientDensity = rows[EXHAUST_ROWS * inst + 9].x;
    meanTrain = rows[EXHAUST_ROWS * inst + 9].y;
    oxidised = rows[EXHAUST_ROWS * inst + 9].z;
}

// fsim: the bend the hot gas gives a ray - heat haze. Air's index of refraction is n = 1 + K rho (Gladstone and Dale;
// K = 0.226 cm^3/g in visible light), the jet's density rho_a T_a / T at the air's pressure: its mean profile across
// the mixing layer a lens, its eddies' fluctuations (their rms a share SIGMA_T of the deficit, as a heated jet's shear
// layer has) the shimmer; a ray turns by the integral of the index's gradient across it.
const float GLADSTONE_DALE = 2.26e-4, SIGMA_T = 0.15;
// fsim: the shimmer drawn HAZE_SHOWN times what the eddies give. As they give it, an engine flying is next to
// invisible: the turn its eddies give a ray behind it some 0.05 px a frame at a 40 m chase (p95 0.2), where the eye
// sees a shimmer from some 0.5 px; a running engine should show it is hot. Drawn so, an Su-57 at idle seen from 25 m
// astern at 1,000 m shimmers 0.3 px (p95 1.1) over its nozzles, at military power 0.7 (p95 1.9). (The lens - the mean
// turn - is as given)
const float HAZE_SHOWN = 8.0;

// Temperature to radiance, a luminance of one at 2000 K
vec3 blackbody(float T) {
    float u = clamp((T - 500.0) / 5500.0, 0.0, 1.0) * 255.0;
    int i = int(floor(u));
    vec4 v = mix(bb[i], bb[min(i + 1, 255)], u - float(i));
    return v.rgb * exp2(v.a);
}

// three.js r180 tonemapping_pars_fragment: AgX (Filament's, after Blender's), linear sRGB in and out, exposure 1
vec3 plumeAgx(vec3 color) {
    const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(vec3(0.6274, 0.0691, 0.0164), vec3(0.3293, 0.9195, 0.0880), vec3(0.0433, 0.0113, 0.8956));
    const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(vec3(1.6605, -0.1246, -0.0182), vec3(-0.5876, 1.1329, -0.1006), vec3(-0.0728, -0.0083, 1.1187));
    const mat3 AgXInsetMatrix = mat3(vec3(0.856627153315983, 0.137318972929847, 0.11189821299995),
                                     vec3(0.0951212405381588, 0.761241990602591, 0.0767994186031903),
                                     vec3(0.0482516061458583, 0.101439036467562, 0.811302368396859));
    const mat3 AgXOutsetMatrix = mat3(vec3(1.1271005818144368, -0.1413297634984383, -0.14132976349843826),
                                      vec3(-0.11060664309660323, 1.157823702216272, -0.11060664309660294),
                                      vec3(-0.016493938717834573, -0.016493938717834257, 1.2519364065950405));
    const float AgxMinEv = -12.47393, AgxMaxEv = 4.026069;
    color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
    color = AgXInsetMatrix * color;
    color = max(color, 1e-10);
    color = log2(color);
    color = (color - AgxMinEv) / (AgxMaxEv - AgxMinEv);
    color = clamp(color, 0.0, 1.0);
    vec3 x2 = color * color, x4 = x2 * x2;
    color = 15.5 * x4 * x2 - 40.14 * x4 * color + 31.96 * x4 - 6.868 * x2 * color + 0.4298 * x2 + 0.1191 * color - 0.00232;
    color = AgXOutsetMatrix * color;
    color = pow(max(vec3(0.0), color), vec3(2.2));
    color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
    return clamp(color, 0.0, 1.0);
}

float plumeHash(vec3 c) {
    vec3 p = fract(c * 0.3183099 + vec3(0.1, 0.2, 0.3)) * 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}
float plumeNoise(vec3 x) {
    vec3 c = floor(x), o = fract(x), e = o * o * (3.0 - 2.0 * o);
    return mix(mix(mix(plumeHash(c), plumeHash(c + vec3(1, 0, 0)), e.x), mix(plumeHash(c + vec3(0, 1, 0)), plumeHash(c + vec3(1, 1, 0)), e.x), e.y),
               mix(mix(plumeHash(c + vec3(0, 0, 1)), plumeHash(c + vec3(1, 0, 1)), e.x), mix(plumeHash(c + vec3(0, 1, 1)), plumeHash(c + vec3(1, 1, 1)), e.x), e.y), e.z);
}
float turbulenceNoise(vec3 p) { return plumeNoise(p) * 0.65 + plumeNoise(p * 2.17 + vec3(5.3, 1.7, 9.1)) * 0.35; }
vec2 wander(float along, float s) {
    float c = floor(along), o = fract(along), e = o * o * (3.0 - 2.0 * o);
    return vec2(mix(plumeHash(vec3(c, s, 0.0)), plumeHash(vec3(c + 1.0, s, 0.0)), e),
                mix(plumeHash(vec3(c, s + 7.3, 3.1)), plumeHash(vec3(c + 1.0, s + 7.3, 3.1)), e));
}

float halfWidth(float x) { x = max(x, 0.0); return radius + spreadNear * min(x, coreLength) + spreadFar * max(x - coreLength, 0.0); }
float centreline(float x) { float r = max(x, 0.0) / max(coreLength, EPS); float s = r * r; return pow(s * s + 1.0, -0.25); }
float stationOf(float x) {
    float c = max(coreLength, EPS), s = max(x, 0.0);
    return s < c ? s : c + (s * s - c * c) / (2.0 * c);
}
float streaked(float size, float speed) { return (size + speed * 0.6 * SHUTTER) * velocity / max(speed, EPS); }

// what the bound and the field share at a point (plume_frame)
struct Fr { float x, core, hw, centre, speed, station, layer, radial, spread; vec2 lateral; };
Fr frameAt(vec3 p, float time) {
    Fr f;
    f.x = p.x;
    f.core = clamp(f.x / max(coreLength, EPS), 0.0, 1.0);
    f.hw = halfWidth(f.x);
    f.centre = centreline(f.x);
    f.speed = velocity * f.centre;
    f.station = stationOf(f.x);
    f.layer = max(f.hw - radius * (1.0 - f.core), radius * 0.02);
    // the far plume meanders as its large structures roll up, rigid at the lip
    float freedom = smoothstep(0.0, coreLength * 2.0, f.x);
    float phase = (f.station - time * velocity * 0.6) / streaked(f.hw * MEANDER_SCALE, f.speed);
    vec2 drift = (wander(phase, seed) - 0.5) * 1.3587 * (meander * f.hw * freedom * 2.0);
    f.lateral = p.yz - drift;
    float round = length(f.lateral);
    f.radial = round;
    f.spread = 1.0;
    if (abs(aspect - 1.0) > 1e-3 || abs(squareness - 2.0) > 1e-3) {
        // a two-dimensional nozzle: measured against its superellipse at the lip, relaxing to round downstream
        float stretch = sqrt(aspect);
        float across = abs(f.lateral.x) / stretch, up = abs(f.lateral.y) * stretch;
        float larger = max(max(across, up), EPS);
        float shaped = larger * pow(pow(across / larger, squareness) + pow(up / larger, squareness), 1.0 / squareness) / areaScale;
        float relaxed = smoothstep(0.0, max(coreLength, EPS), f.x);
        f.radial = mix(shaped, round, relaxed);
        f.spread = 1.0 / mix(1.0 / max(outlineReach, 1.0), 1.0, relaxed);
    }
    return f;
}

float boundFine(Fr f, vec3 p) {
    float capped = max(-p.x, p.x - reach);
    float extent = f.hw * FIELD_EXTENT + f.layer * max(turbulence, 0.0) * EDDY_REACH;
    return max(capped, length(f.lateral) - extent * f.spread);
}
float boundCoarse(vec3 p) {
    float x = p.x;
    float capped = max(-x, x - reach);
    float hw = halfWidth(x);
    float core = clamp(x / max(coreLength, EPS), 0.0, 1.0);
    float layer = max(hw - radius * (1.0 - core), radius * 0.02);
    float extent = hw * FIELD_EXTENT + layer * max(turbulence, 0.0) * EDDY_REACH;
    float m = meander * hw * smoothstep(0.0, coreLength * 2.0, x) * 2.0 * 0.6793 * 1.4142;
    return max(capped, length(p.yz) - extent * max(outlineReach, 1.0) - m);
}

// what the plume is at a point (plume_field): its temperature, how much of it is exhaust, its extinction and emission
void plumeField(Fr f, vec2 footprint, bool turbulent, float time, out float T, out float mixture, out vec3 extinction, out vec3 emission) {
    float displaced = f.radial;
    float coreRadius = radius * (1.0 - f.core);
    float shearWidth = max(f.hw - coreRadius, radius * 0.04);
    float push = turbulence * f.layer * EDDY_REACH;
    bool reachesLayer = f.radial + push > coreRadius && f.radial - push < coreRadius + shearWidth * 3.0;
    float eddy = f.layer * 0.3 + radius * 0.03;
    float blur = smoothstep(0.35, 1.2, footprint.x / eddy);
    if (turbulent && reachesLayer && blur < 1.0) {
        float along = (f.station - time * velocity * 0.6) / streaked(eddy * EDDY_STRETCH, f.speed);
        vec3 flow = vec3(along, f.lateral / eddy) + seed;
        float eddies = mix(turbulenceNoise(flow), 0.5, blur);
        displaced += (eddies - 0.5) * 2.0 * turbulence * f.layer * EDDY_REACH;
    }
    float intoLayer = max(displaced - coreRadius, 0.0) / shearWidth;
    float shape = exp2(-intoLayer * intoLayer);
    // fsim: a rocket's flame closes towards its end, its edge first - cooled from outside by the air it takes in (the
    // tongue its launch photographs show, not a jet's width cut off)
    float rim = clamp(f.radial / max(f.hw, EPS), 0.0, 1.0);
    float closes = reach * (1.0 - 0.45 * step(0.5, meanTrain) * rim * rim);
    float window = (1.0 - smoothstep(closes * 0.7, closes, f.x)) * (f.x < 0.0 ? 0.0 : 1.0);
    mixture = f.centre * shape * window;
    float sp = max(spacing, EPS);
    float expanding = 1.0 - smoothstep(0.0, max(firstDisk * sp, EPS), f.x);
    float coreT = mix(temperature, max(lipT, temperature), max(expanding, meanTrain));
    float mixed = ambient + (coreT - ambient) * mixture + afterburning * mixture * (1.0 - mixture) * 4.0;
    // the shock train: hot behind each Mach disk, cold through the fan; the barrel shock a ring round each cell
    float cells = f.x / sp - firstDisk;
    float behind = fract(cells);
    float inviscid = radius * clamp(1.0 - f.x / max(shockLength, EPS) * 0.65, 0.12, 1.0);
    float radialShare = f.radial / max(inviscid, EPS);
    float hotLength = clamp(2.5 / max(SHOCK_RELAXATION, 0.5), 0.05, 0.9);
    float lozenge = abs(behind - hotLength * 0.5) / (hotLength * 0.5) + radialShare / 0.85;
    float recovering = 1.0 - clamp(behind / hotLength, 0.0, 1.0) * 0.55;
    float lozengeHot = (1.0 - smoothstep(0.2, 1.05, lozenge)) * recovering;
    float barrel = BARREL_RIM + behind * (1.0 - behind) * 4.0 * BARREL_BOW;
    float sideways = sqrt(max(footprint.x * footprint.x - footprint.y * footprint.y, 0.0)) / max(inviscid, EPS);
    float sheetWidth = max(BARREL_WIDTH, sideways * 0.5);
    float sheet = exp2(-pow((radialShare - barrel) / sheetWidth, 2.0)) * (BARREL_WIDTH / sheetWidth) * BARREL_HEAT;
    float hot = max(lozengeHot, sheet);
    float started = smoothstep(-0.15, 0.0, cells);
    float resolved = 1.0 - smoothstep(0.15, 0.6, footprint.y / sp);
    float swing = shockHeat * exp(-max(cells, 0.0) * sp / max(shockLength, EPS)) * resolved;
    float cell = hot * started - (1.0 - hot) * (1.0 - expanding) * 0.6 * (1.0 - meanTrain);
    float disked = hot * started * (swing / max(shockHeat, EPS));
    float relit = reheat * disked * mixture;
    float dense = min(mix(thinning, 1.0, expanding) * mix(1.0, compression, disked), 1.0);
    float shocked = mixed + (mixed - ambient) * (pow(swing + 1.0, cell) - 1.0) + relit;
    T = max(shocked, 1.0);
    // soot glows only where it is hot enough; the rest is drawn burnt out
    float flame = smoothstep(1200.0, 1500.0, shocked);
    float s = soot * dense * mixture * mix(sootSurvival, 1.0, flame) * (1.0 - oxidised * mixture * (1.0 - mixture) * 4.0);
    vec3 sootRgb = SOOT_SPECTRUM * s;
    extinction = sootRgb;
    vec3 thermal = sootRgb * blackbody(T);
    float activated = exp(min(ACTIVATION_K * (1.0 / 2000.0 - 1.0 / T), 6.0));
    emission = thermal + BAND_COLOR * band * dense * mixture * activated;
}

// One flame along a ray in its frame (from `origin`, `dir` a unit vector), ending at `solid` (in its frame's units), at
// its detail (`tier`: 0 near, 1, 2 far) and its share of the near steps (`budget`), dithered by `pixel`: its light as
// aeronautic's material leaves it (premultiplied, tone mapped) and its opacity; `first` where the gas starts (-1 none).
// `bend` the turn the gas gives the ray (in its frame, an angle per axis), `hot` where the hot gas starts (-1 none).
vec4 marchFlame(int which, vec3 origin, vec3 dir, float tier, float budgetIn, vec2 pixel, float solid, out float first, out vec3 bend,
                out float hot) {
    first = -1.0;
    bend = vec3(0.0);
    hot = -1.0;
    inst = which;
    load();
    float time = frame.x;
    // the field's bounding cylinder (plume_span), its far side the only one drawn: the near side has nothing to do
    float rc = outerFar;
    float capA = (0.0 - origin.x) / (abs(dir.x) > EPS ? dir.x : EPS), capB = (reach - origin.x) / (abs(dir.x) > EPS ? dir.x : EPS);
    float nearCaps = max(min(capA, capB), 0.0), farCaps = max(capA, capB);
    float qa = dot(dir.yz, dir.yz), qb = 2.0 * dot(origin.yz, dir.yz), qc = dot(origin.yz, origin.yz) - rc * rc;
    float disc = qb * qb - 4.0 * qa * qc;
    float t0, t1;
    if (qa < EPS) { t0 = nearCaps; t1 = qc > 0.0 ? -1.0 : farCaps; }
    else if (disc < 0.0) { t0 = 0.0; t1 = -1.0; }
    else {
        float root = sqrt(disc);
        t0 = max(nearCaps, (-qb - root) / (2.0 * qa));
        t1 = min(farCaps, (-qb + root) / (2.0 * qa));
    }
    t1 = min(t1, solid); // (the airframe in the way: the ray ends on it)
    if (t1 <= t0) return vec4(0.0);

    bool nearTier = tier < 0.5, farTier = tier > 1.5;
    float crossed = t1 - t0;
    float lateral = max(length(dir.yz), EPS);
    float closest = qa > EPS ? clamp(-dot(origin.yz, dir.yz) / qa, t0, t1) : 0.5 * (t0 + t1);
    float farDepth = min(halfWidth(origin.x + dir.x * closest) * GAUSSIAN_DEPTH / lateral, crossed);
    int steps = farTier ? 1 : (nearTier ? int(max(float(NEAR_STEPS) * budgetIn, float(MID_STEPS))) : MID_STEPS);
    float stride = farTier ? farDepth : crossed / float(steps);
    // the shock train, marched at so many samples a cell, tied to the plume and not the view, so the diamonds do not
    // slide as the camera moves
    float sp = max(spacing, EPS);
    float axial = abs(dir.x);
    float trainEnd = min(reach, (firstDisk + 1.0) * sp + shockLength * SHOCK_TRAIN_LENGTHS);
    float cylinder = radius * 1.05 * max(outlineReach, 1.0) + meander * halfWidth(trainEnd);
    float ca = max(dot(dir.yz, dir.yz), EPS), cb = dot(origin.yz, dir.yz), cc = dot(origin.yz, origin.yz) - cylinder * cylinder;
    float cdisc = cb * cb - ca * cc, croot = sqrt(max(cdisc, 0.0));
    float dx = axial > EPS ? dir.x : EPS;
    float xlo = -origin.x / dx, xhi = (trainEnd - origin.x) / dx;
    float segIn = max(max(t0, (-cb - croot) / ca), min(xlo, xhi));
    float segOut = min(min(t1, (-cb + croot) / ca), max(xlo, xhi));
    float segLength = cdisc > 0.0 ? max(segOut - segIn, 0.0) : 0.0;
    float fineSteps = farTier ? 0.0 : min(ceil(max(segLength * axial / sp * SAMPLES_PER_CELL, segLength / cylinder * SAMPLES_ACROSS_CORE * 0.5)),
                                         float(NEAR_STEPS) * 2.0);
    bool hasFine = fineSteps > 0.0;
    float fineStride = min(stride, segLength / max(fineSteps, 1.0));
    int budget = steps + int(fineSteps) + 2;
    // a dither held still on the plume: seeded in whole cells of its own frame, split per pixel
    float interleaved = gasDither(pixel); // (fsim: the gas pass's blue noise, not interleaved gradient noise's lattice)
    float jitter = fract(plumeHash(floor((origin + dir * t0) * DITHER_SCALE)) + interleaved);
    float segStart = segIn + jitter * fineStride;
    float firstStride = (hasFine && segIn <= t0) ? fineStride : stride;
    float t = farTier ? closest : t0 + jitter * firstStride;
    bool turbulent = nearTier && turbulence > 0.0;
    vec3 light = vec3(0.0), Tr = vec3(1.0);
    bool outside = true;
    for (int i = 0; i < 300; ++i) {
        if (i >= budget || t > t1) break;
        vec3 p = origin + dir * t;
        bool beforeTrain = hasFine && t < segStart;
        bool inTrain = hasFine && t >= segStart && t < segOut;
        float toTrain = max(segStart - t, 1e-4);
        float here = inTrain ? fineStride : (beforeTrain ? min(stride, toTrain) : stride);
        vec2 footprint = vec2(here, here * axial);
        if (outside && !farTier) {
            float coarse = boundCoarse(p);
            if (coarse > 0.0) {
                float leap = max(coarse * STEP_SAFETY, here);
                t += beforeTrain ? min(leap, toTrain) : leap;
                continue;
            }
        }
        Fr f = frameAt(p, time);
        float gap = boundFine(f, p);
        outside = gap > 0.0;
        if (gap > 0.0 && !farTier) {
            float leap = max(gap * STEP_SAFETY, here);
            t += beforeTrain ? min(leap, toTrain) : leap;
            continue;
        }
        float T, mixture;
        vec3 ext, em;
        plumeField(f, footprint, turbulent, time, T, mixture, ext, em);
        if (mixture > 0.003 && T > ambient + 1.0 && ambientDensity > 0.0) {
            // the lens: grad rho = -rho_a T_a / T^2 grad T, T falling across the mixing layer as the field's profile has it
            float coreRadius = radius * (1.0 - f.core);
            float shearWidth = max(f.hw - coreRadius, radius * 0.04);
            float into = max(f.radial - coreRadius, 0.0) / shearWidth;
            vec2 outward = length(f.lateral) > 1e-4 ? normalize(f.lateral) : vec2(0.0);
            float dTdr = -(T - ambient) * 1.3863 * into / shearWidth;
            vec3 grad = vec3(0.0, outward * (-ambientDensity * ambient / (T * T) * dTdr));
            // the shimmer: the eddies the flame's turbulence is drawn with, their density's fluctuation, carried down the
            // jet. A step longer than an eddy - as long, along the ray, as the eddies are drawn: streaked down the jet,
            // round across it - crosses here / ell of them, their turns adding as a random walk: the turn the step takes
            // is one eddy's gradient times sqrt(ell here), not times here - and not left out, as it was when the steps
            // were coarser than the eddies (nearly always: the march takes a few steps across the jet), when the hot gas
            // only bent the scene behind it, still, and an engine at low power showed nothing. (Seen up the jet from
            // astern, a ray runs along an eddy's whole streaked length: one eddy, not as many as round ones would be)
            float eddy = f.layer * 0.3 + radius * 0.03;
            float eddyLength = streaked(eddy * EDDY_STRETCH, f.speed);
            float ell = 1.0 / max(length(vec2(dir.x / eddyLength, length(dir.yz) / eddy)), EPS);
            float along = (f.station - time * velocity * 0.6) / eddyLength;
            vec3 q = vec3(along, f.lateral / eddy) + seed;
            const float h = 0.3;
            float n0 = turbulenceNoise(q);
            vec2 dN = vec2(turbulenceNoise(q + vec3(0.0, h, 0.0)) - n0, turbulenceNoise(q + vec3(0.0, 0.0, h)) - n0) / (h * eddy);
            float deficit = ambientDensity * (1.0 - ambient / T);
            vec3 shimmer = vec3(0.0, dN * (2.0 * SIGMA_T * deficit)) * (GLADSTONE_DALE * HAZE_SHOWN);
            grad *= GLADSTONE_DALE;
            bend += (grad - dir * dot(grad, dir)) * here + (shimmer - dir * dot(shimmer, dir)) * min(here, sqrt(ell * here));
            if (hot < 0.0) hot = t;
        }
        // one step of the transfer equation, exact for a constant source over the step
        vec3 depth = ext * here;
        vec3 through = exp(-depth);
        vec3 share = mix(1.0 - 0.5 * depth, (1.0 - through) / max(depth, vec3(1e-3)), step(vec3(1e-3), depth));
        light += Tr * em * share * here;
        Tr *= through;
        if (first < 0.0 && mixture > 0.02) first = t;
        if (max(max(Tr.x, Tr.y), Tr.z) < PLUME_OPAQUE) break;
        t += here;
    }
    // up the nozzle from astern: the turbine and the pipe glowing dry, the burner's flame filling the pipe lit
    if (max(glow, pipeFlame) > 0.0 && dir.x < -EPS) {
        float er = max(exitRadius, EPS);
        float toExit = origin.x / -dir.x;
        float toTurbine = (origin.x + er * TURBINE_DEPTH) / -dir.x;
        float atExit = frameAt(vec3(0.0, origin.yz + dir.yz * toExit), time).radial / er;
        float atTurbine = frameAt(vec3(0.0, origin.yz + dir.yz * toTurbine), time).radial / er;
        // (seen where the ray passes the exit before anything solid: the nozzle's inside lit by its gas, as the material
        // has it; an airframe outside the exit stands in front of it)
        float throughLip = (1.0 - smoothstep(0.97, 1.0, atExit)) * step(0.0, toExit) * step(toExit, solid);
        float blades = smoothstep(0.25, 0.5, atTurbine) * 0.5 + 0.5;
        float turbine = (1.0 - smoothstep(0.85, 1.0, atTurbine)) * blades;
        float deep = clamp((1.0 - atExit) / max(atTurbine - atExit, EPS), 0.0, 1.0);
        float wall = mix(PIPE_GLOW_LIP, PIPE_GLOW, deep);
        float seen = mix(wall, 1.0, turbine) * throughLip;
        vec3 hue = blackbody(glowT);
        float path = max(toTurbine - toExit, 0.0) * deep;
        vec3 fl = hue * (1.0 - exp(-pipeFlame * path)) * throughLip;
        light += Tr * (hue * (PIPE_EMISSIVITY * glow * seen) + fl);
        if (first < 0.0 && throughLip > 0.0) first = toExit;
    }
    // exposed (Exhaust::setFrame), then through three.js's AgX tone mapping, as aeronautic's examples draw it
    // (stage.tsx). (Theirs maps the whole frame, the scene behind included; here the plume's own light is mapped
    // and laid over the scene, which is not)
    vec3 exposed = light * frame.z * adaptation;
    float coverage = 1.0 - dot(Tr, LUMINANCE);
    if (coverage < 1e-4 && max(max(exposed.x, exposed.y), exposed.z) < 1e-5) {
        first = -1.0;
        return vec4(0.0);
    }
    // where the gas starts: what stands in front of that hides it
    first = max(first < 0.0 ? t0 : first, 0.0);
    return vec4(plumeAgx(exposed), clamp(coverage, 0.0, 1.0));
}
)";

} // namespace

double Exhaust::reheatTotalK(double bypassRatio, double dryThrustN, double reheatThrustN) {
    if (!(dryThrustN > 0.0) || !(reheatThrustN > dryThrustN)) return 0.0;
    const double bypass = std::isfinite(bypassRatio) ? std::max(bypassRatio, 0.0) : 0.0;
    const double dryK = (kTurbineExitK + bypass * kBypassK) / (1.0 + bypass);
    const double speed = reheatThrustN / dryThrustN / (1.0 + kFuelMassShare); // (the jet's speed with reheat over dry)
    return std::clamp(dryK * speed * speed, kLeastReheatK, kMostReheatK);
}

double Exhaust::flameLength(double radius, double afterburner) {
    const double a = std::clamp(afterburner, 0.0, 1.0);
    return a > 0.0 ? 2.0 * radius * (1.2 + 4.6 * a) : 0.0;
}

double Exhaust::running(double n2) {
    // from a turbine starting (some 20 % of its core speed) to its idle (about 60 %, JSBSim's)
    if (!std::isfinite(n2)) return 0.0;
    const double s = std::clamp((n2 - 20.0) / 40.0, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

double Exhaust::dryPower(double n2) {
    // JSBSim's turbines idle at about 60 % of their core speed: nothing to see
    // below three quarters, the hottest dry exhaust at full speed
    if (!std::isfinite(n2)) return 0.0;
    const double s = std::clamp((n2 - 75.0) / 25.0, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

Exhaust::Exhaust(std::size_t slots, vsg::ref_ptr<const vsg::Options> options) {
    (void)options;
    rows_ = vsg::vec4Array::create(static_cast<std::uint32_t>(std::max<std::size_t>(slots, 1) * kPerSlot * kRows), vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    rows_->properties.dataVariance = vsg::DYNAMIC_DATA;
    frame_ = vsg::vec4Value::create(vsg::vec4(0.0f, 1.0f, static_cast<float>(kNightExposure), 0.0f));
    frame_->properties.dataVariance = vsg::DYNAMIC_DATA;
    // Planck's law, worked out once: chromaticity and log2 luminance against 2000 K's, from 500 to 6000 K
    auto bb = vsg::vec4Array::create(kBlackbody);
    {
        double r, g, b, y;
        blackbody(2000.0, r, g, b, y);
        const double reference = y;
        for (std::uint32_t i = 0; i < kBlackbody; ++i) {
            const double T = 500.0 + 5500.0 * i / (kBlackbody - 1);
            blackbody(T, r, g, b, y);
            const double l = std::max(y, 1e-30);
            bb->set(i, vsg::vec4(static_cast<float>(r / l), static_cast<float>(g / l), static_cast<float>(b / l), static_cast<float>(std::log2(l / reference))));
        }
    }
    blackbody_ = bb;

    flames_.resize(std::max<std::size_t>(slots, 1) * kPerSlot);
}

vsg::ref_ptr<vsg::Node> Exhaust::node(std::size_t slot, std::size_t k) {
    if (!rows_ || k >= kPerSlot) return {};
    const std::size_t i = slot * kPerSlot + k;
    if (i >= flames_.size()) return {};
    Flame& f = flames_[i];
    if (!f.node) f.node = vsg::Group::create();
    return f.node;
}

const char* Exhaust::marchSource() { return kMarch; }

bool Exhaust::set(std::size_t slot, std::size_t k, double radiusY, double radiusZ, const sim::VehicleState& s, int engine, const Kind& kind) {
    const std::size_t i = slot * kPerSlot + k;
    if (!rows_ || k >= kPerSlot || i >= flames_.size()) return false;
    const bool ok = engine >= 0 && engine < s.engineCount && engine < sim::VehicleState::kMaxEngines;
    const double ab = ok && std::isfinite(s.afterburner[engine]) ? std::clamp(s.afterburner[engine], 0.0, 1.0) : 0.0;
    const double dry = ok ? dryPower(s.engineN2[engine]) : 0.0;
    const double run = ok ? running(s.engineN2[engine]) : 0.0;
    const double ry = std::isfinite(radiusY) && radiusY > 0.0 ? radiusY : 0.5, rz = std::isfinite(radiusZ) && radiusZ > 0.0 ? radiusZ : ry;
    // a rocket burns while it makes thrust, its plume at the share of its boost's it makes (its chamber's pressure)
    const double thrust = ok && std::isfinite(s.thrustN[engine]) ? std::max(s.thrustN[engine], 0.0) : 0.0;
    const double burning = kind.rocket && thrust > 0.0 ? (kind.peakThrustN > 0.0 ? std::min(thrust / kind.peakThrustN, 1.0) : 1.0) : 0.0;
    const bool shows = kind.rocket ? burning > 0.0 : run > 0.0;

    // The throttle's travel as aeronautic has it: idle 0, military power 1, reheat on to 1.1. The nozzle's size as the
    // radius of the round exit of its area, its outline a rounded rectangle when it is two-dimensional.
    const double travel = kind.rocket ? burning : (ab > 0.0 ? 1.0 + 0.1 * ab : dry * run);
    const double radius = std::sqrt(ry * rz);
    const double aspect = ry / rz;
    const double squareness = std::abs(aspect - 1.0) > 0.05 ? 4.0 : 2.0;
    double areaScale = 1.0, outlineReach = 1.0;
    outlineFit(aspect, squareness, areaScale, outlineReach);
    const double altitude = std::isfinite(s.altitudeMslM) ? s.altitudeMslM : 0.0;
    const double airspeed = std::isfinite(s.airspeedTrueMs) ? s.airspeedTrueMs : 0.0;
    const Air a = air(altitude, airspeed);
    const Engine e = kind.rocket ? rocketOf(kind) : engineOf(kind);
    const Profile pr = kind.rocket ? rocketProfile() : Profile{};
    Jet j = jetState(e, pr, radius, travel, a, airspeed);
    if (kind.rocket && shows) {
        // up its nozzle from astern: the throat at the chamber's temperature, white hot (the model's dry glow, which a
        // jet with no burner shows only at full power)
        j.glow = 1.0, j.pipeFlame = 0.0;
    }
    const double reachOut = std::max(outlineReach, 1.0);

    const auto F = [](double v) { return static_cast<float>(v); };
    // a seed per engine, so two nozzles side by side do not move in step
    const double seed = std::fmod(static_cast<double>(i) * 0.618033988749895, 1.0) * 97.0;
    const vsg::vec4 rows[kRows] = {
        {F(j.radius), F(j.coreLength), F(j.reach), F(j.velocity)},
        {F(j.temperature), F(j.afterburning + j.kinetic), F(j.shockHeat), F(j.shockLength)},
        {F(j.shockSpacing), F(j.firstDisk), F(j.spreadNear), F(j.spreadFar)},
        {F(e.sootPerM * j.sootFormed), F(j.sootSurvival), F(j.reheat), F(j.thinning)},
        {F(j.lipTemperature), F(j.compression), F(j.adaptation), F(e.bandStrength * (1.0 + (j.burner - 1.0) * j.breathes))},
        {F(e.turbulence), F(e.meander), F(seed), F(j.outerFar * reachOut)},
        {F(j.glow), F(radius), F(j.glowTemperature), F(j.pipeFlame)},
        {F(aspect), F(squareness), F(areaScale), F(outlineReach)},
        {F(j.ambientK), F(j.outerNear * reachOut), shows ? 1.0f : 0.0f, 0.0f},
        {F(1.225 * a.density), F(e.trainMean), F(e.oxidised), 0.0f}}; // (the air's density, kg/m^3: the haze)
    for (std::uint32_t r = 0; r < kRows; ++r) {
        const auto at = static_cast<std::uint32_t>(kRows * i + r);
        if (rows_->at(at) != rows[r]) {
            rows_->set(at, rows[r]);
            changed_ = true;
        }
    }
    return shows;
}

void Exhaust::setFrame(double seconds, float daylight) {
    if (!frame_) return;
    // the shaders see seconds as a float: wrapped, so the eddies keep their speed however long the world has run
    // (they travel at the jet's speed, so the wrap is short)
    const float t = static_cast<float>(std::fmod(std::isfinite(seconds) ? seconds : 0.0, 600.0));
    const float day = std::clamp(daylight, 0.0f, 1.0f);
    // the exposure (stage.tsx exposure_under): the night's, coming down in stops to the sky's as the day comes up
    const float exposure = static_cast<float>(kNightExposure * std::pow(std::min(kNightExposure, kDayExposure) / kNightExposure, static_cast<double>(day)));
    const vsg::vec4 v(t, day, exposure, 0.0f);
    if (frame_->value() != v) {
        frame_->value() = v;
        frame_->dirty();
    }
}

void Exhaust::commit() {
    if (changed_ && rows_) rows_->dirty();
    changed_ = false;
}

} // namespace fsim::world
