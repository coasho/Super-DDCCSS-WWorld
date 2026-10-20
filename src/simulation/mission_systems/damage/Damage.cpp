#include "simulation/mission_systems/damage/Damage.h"

#include "foundation/core/Json.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>

namespace fsim::ms::damage {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSteel = 7850.0, kExplosive = 1700.0, kAluminium = 2780.0; // kg/m^3
constexpr double kTntGurney = 2440.0;   // m/s: TNT's sqrt(2E) (Cooper)
// The rule every warhead is made by, from its card (estimates: no source gives a modern missile's fragments):
constexpr double kStructure = 0.15;     // of what is not explosive: its fuze, safe-arm, booster and end plates
constexpr double kDiameter = 0.9;       // of the missile's: the warhead's, inside its skin
constexpr double kRodSpeedMs = 1400.0;  // a continuous rod's speed at most (Talos's, JHU APL Technical Digest 3(2))
constexpr double kFragmentCd = 1.3;     // a tumbling cube's drag on its presented area, supersonic (Hoerner, ch. 16)

// The spray: War Thunder's datamine gives every air-to-air missile the same seven zones (a game reference - no
// public source gives a modern warhead's), 60 % of the fragments between 75 and 100 degrees from the nose; a rod
// warhead's ring a narrow belt about its middle (an estimate)
const std::vector<Zone> kFragmentZones = {{20.0, 60.0, 0.075}, {60.0, 80.0, 0.10}, {75.0, 100.0, 0.60},
                                          {100.0, 120.0, 0.10}, {120.0, 160.0, 0.075}};
const std::vector<Zone> kRodZones = {{85.0, 95.0, 1.0}};

double cardValue(const StoreCard& card, const char* name, double fallback) { return card.value(name, fallback); }

V3 readV3(const core::Json& j) {
    const auto& a = j.asArray();
    if (a.size() != 3) throw std::runtime_error("a point needs three numbers");
    return {a[0].asNumber(), a[1].asNumber(), a[2].asNumber()};
}

// Kingery-Bulmash (Swisdak 1994): exp(A + B L + C L^2 + D L^3 + E L^4), L = ln Z
struct Fit {
    double from, to, c[5];
};
const Fit kPressure[] = {{0.2, 2.9, {7.2106, -2.1069, -0.3229, 0.1117, 0.0685}},
                         {2.9, 23.8, {7.5938, -3.0523, 0.40977, 0.0261, -0.01267}},
                         {23.8, 198.5, {6.0536, -1.4066, 0.0, 0.0, 0.0}}};
const Fit kImpulse[] = {{0.2, 0.96, {5.522, 1.117, 0.6, -0.292, -0.087}},
                        {0.96, 2.38, {5.465, -0.308, -1.464, 1.362, -0.432}},
                        {2.38, 33.7, {5.2749, -0.4677, -0.2499, 0.0588, -0.00554}},
                        {33.7, 158.7, {5.9825, -1.062, 0.0, 0.0, 0.0}}};

template <std::size_t N>
double fit(const Fit (&f)[N], double z) {
    z = std::clamp(z, f[0].from, f[N - 1].to);
    const Fit* use = &f[N - 1];
    for (const Fit& x : f)
        if (z <= x.to) {
            use = &x;
            break;
        }
    const double L = std::log(z);
    return std::exp(use->c[0] + L * (use->c[1] + L * (use->c[2] + L * (use->c[3] + L * use->c[4]))));
}

V3 toBody(const Target& t, const V3& v) { return {dot(v, t.bodyToEcef[0]), dot(v, t.bodyToEcef[1]), dot(v, t.bodyToEcef[2])}; }

// a ray's stretch in an axis-aligned box (body axes): [enter, leave] and each crossed face
bool slab(const V3& p, const V3& u, const V3& centre, const V3& half, double& enter, double& leave,
          V3& normal, V3& exitNormal) {
    const double pc[3] = {p.x - centre.x, p.y - centre.y, p.z - centre.z}, uc[3] = {u.x, u.y, u.z}, h[3] = {half.x, half.y, half.z};
    enter = -std::numeric_limits<double>::infinity();
    leave = std::numeric_limits<double>::infinity();
    int face = 0, exitFace = 0;
    double sign = 1.0, exitSign = 1.0;
    for (int k = 0; k < 3; ++k) {
        if (std::abs(uc[k]) < 1e-12) {
            if (std::abs(pc[k]) > h[k]) return false;
            continue;
        }
        double a = (-h[k] - pc[k]) / uc[k], b = (h[k] - pc[k]) / uc[k];
        double s = -1.0;
        if (a > b) std::swap(a, b), s = 1.0;
        if (a > enter) enter = a, face = k, sign = s;
        if (b < leave) leave = b, exitFace = k, exitSign = -s;
    }
    if (leave < std::max(enter, 0.0)) return false;
    normal = face == 0 ? V3{sign, 0, 0} : face == 1 ? V3{0, sign, 0} : V3{0, 0, sign};
    exitNormal = exitFace == 0 ? V3{exitSign, 0, 0} : exitFace == 1 ? V3{0, exitSign, 0} : V3{0, 0, exitSign};
    return true;
}

// a ray through a convex quad (its plane, inside all four edges): the distance along it and the plane's normal
bool quad(const V3& p, const V3& u, const std::array<V3, 4>& c, double& s, V3& normal) {
    normal = normalized(cross(c[2] - c[0], c[3] - c[1]));
    const double den = dot(u, normal);
    if (std::abs(den) < 1e-9) return false;
    s = dot(c[0] - p, normal) / den;
    if (s < 0.0) return false;
    const V3 q = p + u * s;
    int pos = 0, neg = 0;
    for (std::size_t k = 0; k < 4; ++k) {
        const double side = dot(cross(c[(k + 1) % 4] - c[k], q - c[k]), normal);
        (side >= 0.0 ? pos : neg) += 1;
    }
    return pos == 4 || neg == 4;
}

} // namespace

double gurneyCylinder(double gurneyMs, double caseKg, double chargeKg) noexcept {
    if (!(chargeKg > 0.0)) return 0.0;
    return gurneyMs / std::sqrt(std::max(caseKg, 0.0) / chargeKg + 0.5);
}

std::optional<Warhead> warheadOf(const StoreCard& card) {
    if (card.warheadKind.empty()) return std::nullopt;
    Warhead w;
    w.kind = card.warheadKind == "rod" ? Warhead::Kind::Rod : Warhead::Kind::Fragmentation;
    w.totalKg = cardValue(card, "warhead_kg", 0.0);
    w.explosiveKg = std::clamp(cardValue(card, "explosive_kg", 0.0), 0.0, w.totalKg);
    w.gurneyMs = 1000.0 * cardValue(card, "gurney_kms", 2.6);
    w.tntKg = w.explosiveKg * (w.gurneyMs / kTntGurney) * (w.gurneyMs / kTntGurney);
    const double inert = w.totalKg - w.explosiveKg;
    w.caseKg = (1.0 - kStructure) * inert;
    w.diameterM = kDiameter * cardValue(card, "diameter_m", 0.15);
    // as long as a cylinder of its diameter holding its charge, its case and its structure
    const double volume = w.explosiveKg / kExplosive + w.caseKg / kSteel + kStructure * inert / kAluminium;
    w.lengthM = volume / (0.25 * kPi * w.diameterM * w.diameterM);
    w.caseThicknessM = w.caseKg / (kSteel * kPi * w.diameterM * std::max(w.lengthM, 1e-3));
    const double t = w.caseThicknessM;
    if (w.kind == Warhead::Kind::Rod) {
        w.fragmentKg = kSteel * t * t * w.lengthM;
        w.fragmentAreaM2 = 0.25 * (2.0 * t * t + 4.0 * t * w.lengthM);
        w.zones = kRodZones;
    } else {
        w.fragmentKg = kSteel * t * t * t;
        w.fragmentAreaM2 = 1.5 * t * t;
        w.zones = kFragmentZones;
    }
    w.fragments = w.fragmentKg > 0.0 ? static_cast<int>(std::lround(w.caseKg / w.fragmentKg)) : 0;
    w.speedMs = gurneyCylinder(w.gurneyMs, w.caseKg, w.explosiveKg);
    if (w.kind == Warhead::Kind::Rod) w.speedMs = std::min(w.speedMs, kRodSpeedMs);
    double total = 0.0;
    for (const Zone& z : w.zones) total += z.share;
    for (Zone& z : w.zones) z.share /= total;
    return w;
}

Blast blastAt(double tntKg, double rangeM, double ambientPa, double ambientK) noexcept {
    Blast b;
    if (!(tntKg > 0.0) || !(rangeM > 0.0)) return b;
    const double w = 0.5 * tntKg; // (free air: the surface burst's fits at half the charge)
    const double cw = std::cbrt(w);
    const double p = std::max(ambientPa, 1.0) / 101325.0;
    const double z = rangeM / cw * std::cbrt(p);                // Sachs's scaled distance
    b.overpressurePa = 1000.0 * p * fit(kPressure, z);          // kPa -> Pa
    b.impulsePas = cw * std::pow(p, 2.0 / 3.0) * std::sqrt(288.15 / std::max(ambientK, 100.0)) * fit(kImpulse, z); // kPa ms = Pa s
    return b;
}

double reflected(double ps, double pa) noexcept { return ps > 0.0 ? 2.0 * ps * (7.0 * pa + 4.0 * ps) / (7.0 * pa + ps) : 0.0; }

double thorResidual(double plateM, double areaM2, double massKg, double speedMs, double obliquityRad) noexcept {
    if (!(speedMs > 0.0)) return 0.0;
    if (!(plateM > 0.0)) return speedMs;
    // THOR TR-47's aluminium 2024-T3: c, alpha, beta, gamma, lambda (inches, square inches, grains, ft/s)
    constexpr double c = 7.047, alpha = 1.029, beta = -1.072, gamma = 1.251, lambda = -0.139;
    const double h = plateM / 0.0254, A = areaM2 / (0.0254 * 0.0254), m = massKg / 6.479891e-5, v = speedMs / 0.3048;
    const double sec = 1.0 / std::max(std::cos(std::min(std::abs(obliquityRad), 1.45)), 0.12);
    const double loss = std::pow(10.0, c) * std::pow(h * A, alpha) * std::pow(m, beta) * std::pow(sec, gamma) * std::pow(v, lambda);
    return std::max(v - loss, 0.0) * 0.3048;
}

V3 Airframe::Panel::middle() const noexcept {
    if (!surface) return centre;
    return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25;
}

int Airframe::section(const std::string& name) const noexcept {
    for (std::size_t i = 0; i < sections.size(); ++i)
        if (sections[i].name == name) return static_cast<int>(i);
    return -1;
}

double Airframe::skinThicknessM(int s) const noexcept {
    if (s < 0 || s >= static_cast<int>(sections.size())) return 0.002;
    const Section& x = sections[static_cast<std::size_t>(s)];
    return x.skinAreaM2 > 0.0 ? std::clamp(x.skinKg / (kAluminium * x.skinAreaM2), 0.0005, 0.02) : 0.002;
}

AttachedGeometryView::AttachedGeometryView(const Airframe& airframe) : airframe_(&airframe) {
    auto validSection = [&](int section) { return section < 0 || static_cast<std::size_t>(section) < airframe.sections.size(); };
    panels_.reserve(airframe.panels.size());
    components_.reserve(airframe.components.size());
    for (std::size_t k = 0; k < airframe.panels.size(); ++k)
        if (validSection(airframe.panels[k].section)) panels_.push_back(k);
    for (std::size_t k = 0; k < airframe.components.size(); ++k)
        if (validSection(airframe.components[k].section)) components_.push_back(k);
}

V3 panelBlastImpulse(const Airframe::Panel& panel, const V3& burstBody, double impulsePas) noexcept {
    if (!(impulsePas > 0.0) || !std::isfinite(impulsePas) || !(panel.areaM2 > 0.0) || !std::isfinite(panel.areaM2)) return {};
    const V3 to = burstBody - panel.middle();
    double facing = 1.0;
    if (panel.surface)
        facing = std::abs(dot(normalized(cross(panel.corners[2] - panel.corners[0], panel.corners[3] - panel.corners[1])), normalized(to)));
    return normalized(to) * (-impulsePas * 0.5 * panel.areaM2 * facing);
}

std::optional<Airframe> readAirframe(const std::filesystem::path& file, std::string& error) {
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        error = file.string() + ": cannot be read";
        return std::nullopt;
    }
    std::stringstream text;
    text << in.rdbuf();
    try {
        const core::Json doc = core::Json::parse(text.str(), file.string());
        Airframe a;
        a.aircraft = doc.string("aircraft", "");
        a.designMassKg = doc.number("design_mass_kg", 0.0);
        if (doc.child("origin_design_m").isArray()) a.originDesign = readV3(doc.child("origin_design_m"));
        const auto& nl = doc.child("n_limit").asArray();
        const auto& nu = doc.child("n_ultimate").asArray();
        a.nLimit[0] = nl.at(0).asNumber(), a.nLimit[1] = nl.at(1).asNumber();
        a.nUltimate[0] = nu.at(0).asNumber(), a.nUltimate[1] = nu.at(1).asNumber();
        for (const core::Json& s : doc.child("sections").asArray()) {
            Airframe::Section x;
            x.name = s.string("name", "");
            x.massKg = s.number("mass_kg", 0.0);
            x.cg = readV3(s.child("cg"));
            const auto& in6 = s.child("inertia").asArray();
            for (std::size_t k = 0; k < 6 && k < in6.size(); ++k) x.inertia[k] = in6[k].asNumber();
            for (const core::Json& t : s.child("tanks").asArray())
                x.tanks.push_back({static_cast<int>(t.number("index", 0.0)), t.string("name", ""), t.number("capacity_kg", 0.0), readV3(t.child("centre"))});
            if (s.child("stations").isArray())
                for (const core::Json& st : s.child("stations").asArray()) x.stations.push_back(static_cast<int>(st.number("number", 0.0)));
            x.normalShare = s.number("normal_share", 0.0);
            x.sideShare = s.number("side_share", 0.0);
            if (s.child("normal_cp").isArray()) x.normalCp = readV3(s.child("normal_cp"));
            if (s.child("side_cp").isArray()) x.sideCp = readV3(s.child("side_cp"));
            if (s.child("root").isObject()) {
                x.hasRoot = true;
                x.root = readV3(s.child("root").child("at"));
                for (const core::Json& ax : s.child("root").child("axes").asArray()) x.axes.push_back(readV3(ax));
                for (const core::Json& v : s.child("strength_nm").asArray()) x.strengthNm.push_back(v.asNumber());
            }
            x.skinAreaM2 = s.number("skin_area_m2", 0.0);
            x.skinKg = s.number("skin_kg", 0.0);
            a.sections.push_back(std::move(x));
        }
        for (const core::Json& p : doc.child("panels").asArray()) {
            Airframe::Panel x;
            x.section = a.section(p.string("section", ""));
            x.surface = p.string("kind", "surface") == "surface";
            if (x.surface) {
                const auto& c = p.child("corners").asArray();
                for (std::size_t k = 0; k < 4 && k < c.size(); ++k) x.corners[k] = readV3(c[k]);
                x.thicknessM = p.number("thickness_m", 0.0);
            } else {
                x.centre = readV3(p.child("centre"));
                x.half = readV3(p.child("half"));
            }
            x.areaM2 = p.number("area_m2", 0.0);
            a.panels.push_back(x);
        }
        for (const core::Json& c : doc.child("components").asArray()) {
            Airframe::Component x;
            x.kind = c.string("kind", "");
            x.shape = c.string("shape", "box");
            x.name = c.string("name", "");
            x.section = a.section(c.string("section", ""));
            x.centre = readV3(c.child("centre"));
            x.half = readV3(c.child("half"));
            x.engine = static_cast<int>(c.number("engine", -1.0));
            x.tank = static_cast<int>(c.number("tank", -1.0));
            x.channel = c.string("channel", "");
            x.fuelKg = c.number("fuel_kg", 0.0);
            a.components.push_back(std::move(x));
        }
        return a;
    } catch (const std::exception& ex) {
        error = file.string() + ": " + ex.what();
        return std::nullopt;
    }
}

Impact assess(const Warhead& w, const Burst& burst, const AttachedGeometryView& geometry, const Target& target, Rng& rng) {
    const Airframe& af = geometry.airframe();
    Impact out;
    const V3 p0 = toBody(target, burst.at - target.at);
    out.missM = length(p0);
    out.fromBody = normalized(p0);
    // the missile's frame: its nose and two directions across it
    const V3 nose = normalized(burst.axis);
    V3 e1 = normalized(cross(nose, std::abs(nose.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0}));
    const V3 e2 = cross(nose, e1);
    std::vector<Impact::ComponentHits> comp(af.components.size());
    std::vector<Impact::PanelHits> pan(af.panels.size());
    const double decayPerM = target.airDensity * kFragmentCd * w.fragmentAreaM2 / (2.0 * std::max(w.fragmentKg, 1e-9));
    out.fragmentsThrown = w.fragments;
    struct Event {
        double s;
        bool skin;
        int index;
        V3 normal;
        bool done;
        int containmentDelta;
    };
    std::vector<Event> events;
    for (int i = 0; i < w.fragments; ++i) {
        // its zone, its direction in it (uniform over the zone's solid angle)
        double pick = rng.uniform(), acc = 0.0;
        const Zone* zone = &w.zones.back();
        for (const Zone& z : w.zones) {
            acc += z.share;
            if (pick < acc) {
                zone = &z;
                break;
            }
        }
        const double c0 = std::cos(zone->fromDeg * kPi / 180.0), c1 = std::cos(zone->toDeg * kPi / 180.0);
        const double ct = c1 + (c0 - c1) * rng.uniform(), st = std::sqrt(std::max(1.0 - ct * ct, 0.0));
        const double phi = 2.0 * kPi * rng.uniform();
        const V3 d = nose * ct + (e1 * std::cos(phi) + e2 * std::sin(phi)) * st;
        // thrown from the moving missile, met by the moving target
        const V3 rel = burst.velocity + d * w.speedMs - target.velocity;
        double v = length(rel);
        if (!(v > 1.0)) continue;
        const V3 u = toBody(target, rel * (1.0 / v));
        events.clear();
        std::size_t bornInside = 0;
        for (const std::size_t k : geometry.panelIndices()) {
            const auto& p = af.panels[k];
            double s0, s1;
            V3 n, exitNormal;
            if (p.surface) {
                if (quad(p0, u, p.corners, s0, n)) {
                    events.push_back({std::max(s0 - 1e-3, 0.0), true, static_cast<int>(k), n, false, 1}); // in through one skin,
                    events.push_back({s0 + 1e-3, true, static_cast<int>(k), n, false, -1}); // out through the other
                }
            } else if (slab(p0, u, p.centre, p.half, s0, s1, n, exitNormal)) {
                if (s0 < 0.0) ++bornInside;
                if (s0 >= 0.0) events.push_back({s0, true, static_cast<int>(k), n, false, 1});
                if (s1 >= 0.0) events.push_back({s1, true, static_cast<int>(k), exitNormal, false, -1});
            }
        }
        for (const std::size_t k : geometry.componentIndices()) {
            double s0, s1;
            V3 n, exitNormal;
            if (slab(p0, u, af.components[k].centre, af.components[k].half, s0, s1, n, exitNormal) && s1 > 0.0)
                events.push_back({std::max(s0, 0.0), false, static_cast<int>(k), n, false, 0});
        }
        if (events.empty()) continue;
        // Preserve source order at ties: panel crossings were added before
        // component hits, and each panel's entry before its exit.
        std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.s < b.s; });
        bool struck = false;
        std::size_t containment = bornInside;
        double at = 0.0;
        double transferredMomentum = 0.0;
        auto skin = [&](Event& e) {
            v *= std::exp(-decayPerM * std::max(e.s - at, 0.0));
            at = std::max(e.s, at);
            const auto& p = af.panels[static_cast<std::size_t>(e.index)];
            auto& h = pan[static_cast<std::size_t>(e.index)];
            h.panel = e.index;
            h.hits += 1;
            const double obliquity = std::acos(std::min(std::abs(dot(u, e.normal)), 1.0));
            const double after = thorResidual(af.skinThicknessM(p.section), w.fragmentAreaM2, w.fragmentKg, v, obliquity);
            if (after > 0.0) h.perforations += 1, h.holeAreaM2 += w.fragmentAreaM2;
            h.momentumNs += u * (w.fragmentKg * (v - after));
            transferredMomentum += w.fragmentKg * (v - after);
            v = after;
            e.done = true;
            struck = true;
            if (e.containmentDelta > 0) ++containment;
            else if (e.containmentDelta < 0 && containment > 0) --containment;
        };
        for (std::size_t k = 0; k < events.size() && v > 0.0; ++k) {
            Event& e = events[k];
            if (e.done) continue;
            if (e.skin) {
                skin(e);
                continue;
            }
            // a component inside: through the skin over it first (the next skin within a metre)
            if (containment == 0)
                for (std::size_t j = k + 1; j < events.size(); ++j)
                    if (events[j].skin && !events[j].done && events[j].s - e.s < 1.0) {
                        skin(events[j]);
                        break;
                    }
            if (!(v > 0.0)) break;
            v *= std::exp(-decayPerM * std::max(e.s - at, 0.0));
            at = std::max(e.s, at);
            auto& c = comp[static_cast<std::size_t>(e.index)];
            c.component = e.index;
            c.hits += 1;
            c.energyJ += 0.5 * w.fragmentKg * v * v;
            c.fastestMs = std::max(c.fastestMs, v);
            c.areaM2 += w.fragmentAreaM2;
            transferredMomentum += w.fragmentKg * v;
            v = 0.0; // (it stops in what it strikes)
            struck = true;
        }
        if (struck) {
            out.fragmentsStriking += 1;
            // Only momentum transferred at the target belongs to its impulse;
            // slowing in the air before/between strikes transfers to the air.
            const V3 j = u * transferredMomentum;
            out.impulseBody += j;
            out.angularImpulseBody += cross(p0, j); // (along its line from the burst: any point of it the same)
        }
    }
    // the blast on each panel: side-on, reflected as far as its face looks at the burst
    for (const std::size_t k : geometry.panelIndices()) {
        const auto& p = af.panels[k];
        const V3 m = p.middle();
        const V3 to = p0 - m;
        const double r = length(to);
        const Blast b = blastAt(w.tntKg, r, target.airPressurePa, target.airTemperatureK);
        if (b.overpressurePa < kBlastSeenPa && pan[k].hits == 0) continue;
        double facing = 1.0;
        if (p.surface) facing = std::abs(dot(normalized(cross(p.corners[2] - p.corners[0], p.corners[3] - p.corners[1])), normalized(to)));
        const double pr = reflected(b.overpressurePa, target.airPressurePa);
        const double peak = b.overpressurePa + (pr - b.overpressurePa) * facing * facing;
        auto& h = pan[k];
        h.panel = static_cast<int>(k);
        h.overpressurePa = peak;
        h.impulsePas = b.overpressurePa > 0.0 ? b.impulsePas * peak / b.overpressurePa : 0.0;
        // (it pushes the face that looks at it: half a body box's girth, one side of a surface panel)
        const V3 j = panelBlastImpulse(p, p0, h.impulsePas);
        out.impulseBody += j;
        out.angularImpulseBody += cross(m, j);
    }
    for (auto& c : comp)
        if (c.component >= 0) out.components.push_back(c);
    for (auto& p : pan)
        if (p.panel >= 0) out.panels.push_back(p);
    return out;
}

Impact assess(const Warhead& w, const Burst& burst, const Airframe& airframe, const Target& target, Rng& rng) {
    return assess(w, burst, AttachedGeometryView(airframe), target, rng);
}

} // namespace fsim::ms::damage
