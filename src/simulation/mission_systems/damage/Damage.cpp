#include "simulation/mission_systems/damage/Damage.h"
#include "simulation/mission_systems/damage/LiquidSceneV1.h"

#include "foundation/core/Json.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <set>
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

// V0's historical five-sector approximation, normalized below. Preserve its
// established outputs; it omitted the reference's forward and aft sectors.
const std::vector<Zone> kFragmentZones = {{20.0, 60.0, 0.075}, {60.0, 80.0, 0.10}, {75.0, 100.0, 0.60},
                                          {100.0, 120.0, 0.10}, {120.0, 160.0, 0.075}};
const std::vector<Zone> kRodZones = {{85.0, 95.0, 1.0}};
// Complete game-reference angular shares, not measured equipment data:
// War-Thunder-Datamine ea96f3af75afe1d858f80ac13f212db218bc69ca,
// aces.vromfs.bin_u/gamedata/weapons/rocketguns/us_aim_120c_7.blkx,
// rocket.damage.shatter.segment[*].angles/countPortion. No damage/radius
// multipliers are imported. Finite source geometry remains our explicit model.
const std::vector<Zone> kFiniteZonesV2 = {{0,20,.0375},{20,60,.075},{60,80,.10},{75,100,.60},
                                         {100,120,.10},{120,160,.075},{160,180,.0125}};

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
        // Only an exactly zero component is parallel. Treating a finite
        // component as zero can both erase a thin-box entry and admit a ray
        // which has already left that slab before reaching the other axes.
        if (uc[k] == 0.0) {
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
    // A shallow finite crossing still belongs to the finite quad. Its edge
    // tests below reject crossings outside the face; an angular cutoff must
    // not replace that geometric test.
    if (!std::isfinite(den) || den == 0.0) return false;
    s = dot(c[0] - p, normal) / den;
    if (!std::isfinite(s) || s < 0.0) return false;
    const V3 q = p + u * s;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) return false;
    int pos = 0, neg = 0;
    for (std::size_t k = 0; k < 4; ++k) {
        const double side = dot(cross(c[(k + 1) % 4] - c[k], q - c[k]), normal);
        if (!std::isfinite(side)) return false;
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
    if (card.parameters.find("warhead_model_version")==card.parameters.end())
        for (const auto& parameter : card.parameters)
            if (parameter.first.compare(0,8,"warhead_")==0 && parameter.first!="warhead_kg") return std::nullopt;
    if (const auto version=card.parameters.find("warhead_model_version"); version!=card.parameters.end()) {
        if ((version->second.value!=1.0 && version->second.value!=2.0) || version->second.unit!="1" || card.warheadKind!="fragmentation") return std::nullopt;
        const auto required=[&](const char* name,const char* unit) {
            const auto found=card.parameters.find(name);
            return found==card.parameters.end() || found->second.unit!=unit ?
                std::numeric_limits<double>::quiet_NaN() : found->second.value;
        };
        const double source=required("warhead_fragment_source","1"), total=required("warhead_kg","kg"), charge=required("explosive_kg","kg");
        const double tnt=required("warhead_tnt_equivalent_kg","kg");
        const double diameter=required("diameter_m","m"), modelLength=required("length_m","m"), gurney=required("gurney_kms","km/s");
        const double structure=required("warhead_structure_fraction","1"), ratio=required("warhead_diameter_ratio","1");
        const double caseDensity=required("warhead_case_density_kg_m3","kg/m^3"), chargeDensity=required("warhead_explosive_density_kg_m3","kg/m^3");
        const double structureDensity=required("warhead_structure_density_kg_m3","kg/m^3"), cd=required("warhead_fragment_cd","1");
        const V3 station{required("warhead_station_x_m","m"),required("warhead_station_y_m","m"),required("warhead_station_z_m","m")};
        for (const double value : {source,total,charge,tnt,diameter,modelLength,gurney,structure,ratio,caseDensity,chargeDensity,
                                   structureDensity,cd,station.x,station.y,station.z})
            if (!std::isfinite(value)) return std::nullopt;
        if (source!=version->second.value || !(total>charge) || !(charge>0) || !(tnt>0) || !(diameter>0) || !(modelLength>0) || !(gurney>0) ||
            structure<0 || structure>=1 || !(ratio>0) || ratio>1 || !(caseDensity>0) || !(chargeDensity>0) ||
            !(structureDensity>0) || !(cd>0)) return std::nullopt;
        Warhead w;
        w.modelVersion=static_cast<unsigned>(version->second.value);
        w.fragmentSource=w.modelVersion==1 ? Warhead::FragmentSource::CylindricalSideCase : Warhead::FragmentSource::FiniteZonedCaseV2;
        if(w.modelVersion==2)w.zones=kFiniteZonesV2;
        w.stationStructuralM=station;w.fragmentCd=cd;w.totalKg=total;w.explosiveKg=charge;w.gurneyMs=1000*gurney;
        const double inert=total-charge;
        w.caseKg=(1-structure)*inert;w.diameterM=ratio*diameter;
        const double outer=w.diameterM/2;
        const double volume=charge/chargeDensity+w.caseKg/caseDensity+structure*inert/structureDensity;
        w.lengthM=volume/(kPi*outer*outer);
        const double annulus=w.caseKg/(caseDensity*kPi*w.lengthM);
        const double innerSquared=outer*outer-annulus;
        if (!(outer>0) || !(innerSquared>0) || !(w.lengthM>0) || !std::isfinite(w.lengthM)) return std::nullopt;
        w.caseInnerRadiusM=std::sqrt(innerSquared);
        // Stable exact-annulus thickness, rather than the thin-wall estimate.
        w.caseThicknessM=annulus/(outer+w.caseInnerRadiusM);
        if (station.x-w.lengthM/2<0 || station.x+w.lengthM/2>modelLength ||
            std::hypot(station.y,station.z)+outer>diameter/2) return std::nullopt;
        const double cubeKg=caseDensity*w.caseThicknessM*w.caseThicknessM*w.caseThicknessM;
        if (!(cubeKg>0) || !std::isfinite(cubeKg)) return std::nullopt;
        const double count=std::round(w.caseKg/cubeKg);
        if (!(count>=1) || count>std::numeric_limits<int>::max() || !std::isfinite(count)) return std::nullopt;
        w.fragments=static_cast<int>(count);
        // Integerizing the equivalent-cube rule must not create/lose case mass.
        w.fragmentKg=w.caseKg/count;
        w.fragmentAreaM2=1.5*std::pow(w.fragmentKg/caseDensity,2.0/3.0);
        w.speedMs=gurneyCylinder(w.gurneyMs,w.caseKg,charge);
        w.tntKg=tnt; // Blast equivalence cannot be inferred from fragment-driving Gurney energy.
        if (!(w.speedMs>0) || !std::isfinite(w.speedMs) || !(w.fragmentAreaM2>0) || !std::isfinite(w.fragmentAreaM2) ||
            !std::isfinite(w.tntKg) || !std::isfinite(w.caseThicknessM) || !std::isfinite(w.fragmentKg)) return std::nullopt;
        return w;
    }
    Warhead w;
    w.kind = card.warheadKind == "rod" ? Warhead::Kind::Rod : Warhead::Kind::Fragmentation;
    w.totalKg = cardValue(card, "warhead_kg", 0.0);
    w.explosiveKg = std::clamp(cardValue(card, "explosive_kg", 0.0), 0.0, w.totalKg);
    w.gurneyMs = 1000.0 * cardValue(card, "gurney_kms", 2.6);
    w.tntKg = w.explosiveKg * (w.gurneyMs / kTntGurney) * (w.gurneyMs / kTntGurney);
    const double inert = w.totalKg - w.explosiveKg;
    w.caseKg = (1.0 - kStructure) * inert;
    w.diameterM = kDiameter * cardValue(card, "diameter_m", 0.15);
    if((card.warheadKind!="fragmentation"&&card.warheadKind!="rod")||!(w.totalKg>0)||!(w.explosiveKg>0)||
       !(w.totalKg>w.explosiveKg)||!std::isfinite(w.totalKg)||!std::isfinite(w.explosiveKg)||
       !(w.gurneyMs>0)||!std::isfinite(w.gurneyMs)||!(w.diameterM>0)||!std::isfinite(w.diameterM))return std::nullopt;
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
    const double count=w.caseKg/w.fragmentKg;
    if(!(w.fragmentKg>0)||!std::isfinite(w.fragmentKg)||!(w.fragmentAreaM2>0)||!std::isfinite(w.fragmentAreaM2)||
       !(count>=.5)||!std::isfinite(count)||count>std::numeric_limits<int>::max()-.5||
       !(w.lengthM>0)||!std::isfinite(w.lengthM)||!std::isfinite(w.caseThicknessM))return std::nullopt;
    w.fragments = static_cast<int>(std::lround(count));
    w.speedMs = gurneyCylinder(w.gurneyMs, w.caseKg, w.explosiveKg);
    if (w.kind == Warhead::Kind::Rod) w.speedMs = std::min(w.speedMs, kRodSpeedMs);
    if(!(w.speedMs>0)||!std::isfinite(w.speedMs)||!std::isfinite(w.tntKg))return std::nullopt;
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

const ContactMesh* ContactGeometryV1::panel(std::size_t index) const noexcept {
    return index < panels_.size() && panels_[index] < meshes_.size() ? &meshes_[panels_[index]] : nullptr;
}
const ContactMesh* ContactGeometryV1::component(std::size_t index) const noexcept {
    return index < components_.size() && components_[index] < meshes_.size() ? &meshes_[components_[index]] : nullptr;
}

bool setContactGeometryV1(Airframe& frame, std::vector<ContactMesh> meshes, std::string& error) {
    try {
        constexpr std::size_t maxVertices = 1000000, maxTriangles = 2000000;
        auto geometry = std::shared_ptr<ContactGeometryV1>(new ContactGeometryV1);
        geometry->panels_.assign(frame.panels.size(), meshes.size());
        geometry->components_.assign(frame.components.size(), meshes.size());
        std::size_t totalVertices = 0, totalTriangles = 0;
        for (std::size_t i = 0; i < meshes.size(); ++i) {
            auto& mesh = meshes[i];
            auto* lookup = mesh.kind == ContactMesh::Kind::Panel ? &geometry->panels_ :
                           mesh.kind == ContactMesh::Kind::Component ? &geometry->components_ : nullptr;
            if (!lookup || mesh.index >= lookup->size() || (*lookup)[mesh.index] != meshes.size())
                throw std::runtime_error("contact geometry has an invalid or duplicate source identity");
            if (mesh.kind == ContactMesh::Kind::Panel && frame.panels[mesh.index].surface)
                throw std::runtime_error("a finite surface panel cannot also be a closed contact volume");
            if (mesh.vertices.size() < 4 || mesh.triangles.size() < 4 ||
                mesh.vertices.size() > maxVertices - totalVertices || mesh.triangles.size() > maxTriangles - totalTriangles)
                throw std::runtime_error("contact mesh size is invalid or exceeds the geometry limit");
            if (mesh.triangleNeighbors.size()!=mesh.triangles.size())
                throw std::runtime_error("contact triangle provenance must be explicit and parallel to its faces");
            for (const auto& neighbor : mesh.triangleNeighbors) if(neighbor) {
                const auto size=neighbor->kind==ContactMesh::Kind::Panel ? frame.panels.size() :
                                neighbor->kind==ContactMesh::Kind::Component ? frame.components.size() : 0;
                if (neighbor->index>=size || (neighbor->kind==mesh.kind && neighbor->index==mesh.index))
                    throw std::runtime_error("contact ownership cut has an invalid or self neighbor");
            }
            totalVertices += mesh.vertices.size(); totalTriangles += mesh.triangles.size();
            mesh.low = mesh.high = mesh.vertices.front();
            std::set<std::array<double, 3>> positions;
            for (const auto v : mesh.vertices) {
                if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
                    !positions.insert({v.x, v.y, v.z}).second)
                    throw std::runtime_error("contact mesh vertices must be finite and welded");
                mesh.low.x=std::min(mesh.low.x,v.x); mesh.low.y=std::min(mesh.low.y,v.y); mesh.low.z=std::min(mesh.low.z,v.z);
                mesh.high.x=std::max(mesh.high.x,v.x); mesh.high.y=std::max(mesh.high.y,v.y); mesh.high.z=std::max(mesh.high.z,v.z);
            }
            using Edge = std::pair<std::size_t, std::size_t>;
            std::map<Edge, std::pair<unsigned, int>> edges;
            std::vector<std::map<std::size_t,std::vector<std::size_t>>> links(mesh.vertices.size());
            std::vector<bool> used(mesh.vertices.size(), false);
            long double volume = 0;
            const auto origin = mesh.vertices.front();
            for (const auto& triangle : mesh.triangles) {
                for (const auto index : triangle) {
                    if (index >= mesh.vertices.size()) throw std::runtime_error("contact triangle index is out of range");
                    used[index]=true;
                }
                const auto a=mesh.vertices[triangle[0]], b=mesh.vertices[triangle[1]], c=mesh.vertices[triangle[2]];
                const long double ux=static_cast<long double>(b.x)-a.x, uy=static_cast<long double>(b.y)-a.y, uz=static_cast<long double>(b.z)-a.z;
                const long double vx=static_cast<long double>(c.x)-a.x, vy=static_cast<long double>(c.y)-a.y, vz=static_cast<long double>(c.z)-a.z;
                const long double nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
                const long double square=nx*nx+ny*ny+nz*nz;
                if (!(square>0) || !std::isfinite(square)) throw std::runtime_error("contact mesh has a collapsed or overflowing triangle");
                volume += (static_cast<long double>(a.x)-origin.x)*nx +
                          (static_cast<long double>(a.y)-origin.y)*ny + (static_cast<long double>(a.z)-origin.z)*nz;
                for (int k=0;k<3;++k) {
                    const auto from=triangle[static_cast<std::size_t>(k)], to=triangle[static_cast<std::size_t>((k+1)%3)];
                    auto& edge=edges[{std::min(from,to),std::max(from,to)}];
                    ++edge.first; edge.second += from<to ? 1 : -1;
                    const auto other=triangle[static_cast<std::size_t>((k+2)%3)];
                    links[from][to].push_back(other); links[from][other].push_back(to);
                }
            }
            if (!std::isfinite(volume) || volume==0 || std::find(used.begin(),used.end(),false)!=used.end())
                throw std::runtime_error("contact mesh has no closed volume or has unused vertices");
            for (const auto& edge : edges)
                if (edge.second.first!=2 || edge.second.second!=0)
                    throw std::runtime_error("contact mesh must be a closed consistently wound two-manifold");
            for (const auto& link : links) {
                std::set<std::size_t> visited;
                std::vector<std::size_t> pending{link.begin()->first};
                while (!pending.empty()) {
                    const auto vertex=pending.back();pending.pop_back();
                    if (!visited.insert(vertex).second) continue;
                    const auto& neighbours=link.at(vertex);
                    if (neighbours.size()!=2) throw std::runtime_error("contact mesh has a non-manifold vertex");
                    pending.insert(pending.end(),neighbours.begin(),neighbours.end());
                }
                if (visited.size()!=link.size()) throw std::runtime_error("contact mesh has disconnected vertex fans");
            }
            (*lookup)[mesh.index]=i;
        }
        geometry->meshes_=std::move(meshes);
        frame.contactGeometry=std::move(geometry);
        error.clear();
        return true;
    } catch (const std::exception& ex) { error=ex.what(); return false; }
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
        if (doc.has("contact_geometry")) {
            const auto& contact=doc.child("contact_geometry");
            if (!contact.isObject() || !contact.child("version").isNumber() || contact.child("version").asNumber()!=1)
                throw std::runtime_error("unsupported explicit contact geometry version");
            const auto index=[](const core::Json& value,std::size_t limit) {
                const double n=value.asNumber();
                if (!std::isfinite(n) || n<0 || n>=static_cast<double>(limit) || std::floor(n)!=n)
                    throw std::runtime_error("invalid contact geometry index");
                return static_cast<std::size_t>(n);
            };
            std::vector<ContactMesh> meshes;
            for (const auto& serialized : contact.child("meshes").asArray()) {
                if (meshes.size()>=a.panels.size()+a.components.size()) throw std::runtime_error("too many contact meshes");
                ContactMesh mesh;
                const auto kind=serialized.child("kind").asString();
                if (kind!="panel" && kind!="component") throw std::runtime_error("invalid contact geometry source kind");
                mesh.kind=kind=="panel" ? ContactMesh::Kind::Panel : ContactMesh::Kind::Component;
                mesh.index=index(serialized.child("index"),kind=="panel" ? a.panels.size() : a.components.size());
                const auto& vertices=serialized.child("vertices").asArray();
                const auto& triangles=serialized.child("triangles").asArray();
                if (vertices.size()>1000000 || triangles.size()>2000000) throw std::runtime_error("contact mesh exceeds reader size limit");
                for (const auto& vertex : vertices) mesh.vertices.push_back(readV3(vertex));
                for (const auto& triangle : triangles) {
                    const auto& t=triangle.asArray();
                    if (t.size()!=3) throw std::runtime_error("a contact triangle needs three indices");
                    mesh.triangles.push_back({index(t[0],mesh.vertices.size()),index(t[1],mesh.vertices.size()),index(t[2],mesh.vertices.size())});
                }
                const auto& neighbors=serialized.child("triangle_neighbors").asArray();
                if(neighbors.size()!=mesh.triangles.size()) throw std::runtime_error("contact triangle provenance count mismatch");
                for(const auto& neighbor : neighbors) {
                    if(neighbor.isNull()) {mesh.triangleNeighbors.emplace_back(std::nullopt);continue;}
                    if(!neighbor.isObject()) throw std::runtime_error("invalid contact triangle provenance");
                    const auto neighborKind=neighbor.child("kind").asString();
                    if(neighborKind!="panel" && neighborKind!="component") throw std::runtime_error("invalid contact neighbor kind");
                    mesh.triangleNeighbors.emplace_back(ContactMesh::Neighbor{
                        neighborKind=="panel" ? ContactMesh::Kind::Panel : ContactMesh::Kind::Component,
                        index(neighbor.child("index"),neighborKind=="panel" ? a.panels.size() : a.components.size())});
                }
                meshes.push_back(std::move(mesh));
            }
            if (!setContactGeometryV1(a,std::move(meshes),error)) throw std::runtime_error(error);
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
    const bool sideCase=w.modelVersion==1 && w.fragmentSource==Warhead::FragmentSource::CylindricalSideCase;
    const bool finiteZoned=w.modelVersion==2 && w.fragmentSource==Warhead::FragmentSource::FiniteZonedCaseV2;
    const bool finiteSource=sideCase || finiteZoned;
    // Explicit versions/sources cannot fall back to the legacy point model.
    if (w.modelVersion>2 || (w.modelVersion>0 && (!finiteSource || !(w.lengthM>0) || !(w.diameterM>0) ||
        !(w.caseInnerRadiusM>0) || !(w.caseInnerRadiusM<w.diameterM/2) || !(w.fragmentCd>0) ||
        !std::isfinite(w.lengthM) || !std::isfinite(w.diameterM) || !std::isfinite(w.caseInnerRadiusM) ||
        !std::isfinite(w.fragmentCd) || !(length(nose)>0) ||
        !std::isfinite(burst.angularVelocityEcef.x) || !std::isfinite(burst.angularVelocityEcef.y) ||
        !std::isfinite(burst.angularVelocityEcef.z)))) return out;
    if(finiteZoned) {
        double share=0;
        if(w.zones.empty())return out;
        for(const auto& zone:w.zones) {
            if(!std::isfinite(zone.fromDeg) || !std::isfinite(zone.toDeg) || !std::isfinite(zone.share) ||
                zone.fromDeg<0 || zone.toDeg>180 || zone.fromDeg>zone.toDeg || !(zone.share>0))return out;
            share+=zone.share;
        }
        if(!std::isfinite(share) || std::abs(share-1)>8*std::numeric_limits<double>::epsilon())return out;
    }
    const double decayPerM = target.airDensity * (finiteSource?w.fragmentCd:kFragmentCd) * w.fragmentAreaM2 / (2.0 * std::max(w.fragmentKg, 1e-9));
    out.fragmentsThrown = w.fragments;
    struct Event {
        double s;
        bool skin;
        int index;
        V3 normal;
    };
    std::vector<Event> events;
    for (int i = 0; i < w.fragments; ++i) {
        V3 d, rayFromBody=p0, sourceVelocity=burst.velocity;
        if (finiteSource) {
            // V1's three draws stay intact. V2 then draws zone and polar
            // cosine; source position and ejection retain a common azimuth.
            const double axial=w.lengthM*(rng.uniform()-.5);
            const double phi=2*kPi*rng.uniform();
            const double outer=w.diameterM/2, inner=w.caseInnerRadiusM;
            const double radius=std::sqrt(inner*inner+rng.uniform()*(outer*outer-inner*inner));
            d=e1*std::cos(phi)+e2*std::sin(phi);
            const V3 sourceOffset=nose*axial+d*radius;
            if(finiteZoned) {
                const double pick=rng.uniform();double cumulative=0;
                const Zone* zone=&w.zones.back();
                for(const auto& candidate:w.zones) {
                    cumulative+=candidate.share;if(pick<cumulative){zone=&candidate;break;}
                }
                const double c0=std::cos(zone->fromDeg*kPi/180),c1=std::cos(zone->toDeg*kPi/180);
                const double cosine=c1+(c0-c1)*rng.uniform();
                d=nose*cosine+d*std::sqrt(std::max(0.0,1-cosine*cosine));
            }
            const V3 source=burst.at+sourceOffset;
            rayFromBody=toBody(target,source-target.at);
            sourceVelocity+=cross(burst.angularVelocityEcef,sourceOffset);
        } else {
            // Legacy draw order, point source and weighted angular zones.
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
            d = nose * ct + (e1 * std::cos(phi) + e2 * std::sin(phi)) * st;
        }
        // thrown from the moving missile, met by the moving target
        const V3 rel = sourceVelocity + d * w.speedMs - target.velocity;
        double v = length(rel);
        if (!(v > 1.0) || (finiteSource && !std::isfinite(v))) continue;
        const V3 u = toBody(target, rel * (1.0 / v));
        events.clear();
        for (const std::size_t k : geometry.panelIndices()) {
            const auto& p = af.panels[k];
            double s0, s1;
            V3 n, exitNormal;
            if (p.surface) {
                if (quad(rayFromBody, u, p.corners, s0, n)) {
                    events.push_back({std::max(s0 - 1e-3, 0.0), true, static_cast<int>(k), n}); // in through one skin,
                    events.push_back({s0 + 1e-3, true, static_cast<int>(k), n}); // out through the other
                }
            } else if (slab(rayFromBody, u, p.centre, p.half, s0, s1, n, exitNormal)) {
                if (s0 >= 0.0) events.push_back({s0, true, static_cast<int>(k), n});
                if (s1 >= 0.0) events.push_back({s1, true, static_cast<int>(k), exitNormal});
            }
        }
        for (const std::size_t k : geometry.componentIndices()) {
            double s0, s1;
            V3 n, exitNormal;
            if (slab(rayFromBody, u, af.components[k].centre, af.components[k].half, s0, s1, n, exitNormal) && s1 > 0.0)
                events.push_back({std::max(s0, 0.0), false, static_cast<int>(k), n});
        }
        if (events.empty()) continue;
        // Preserve source order at ties: panel crossings were added before
        // component hits, and each panel's entry before its exit.
        std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.s < b.s; });
        bool struck = false;
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
            struck = true;
        };
        for (std::size_t k = 0; k < events.size() && v > 0.0; ++k) {
            Event& e = events[k];
            if (e.skin) {
                skin(e);
                continue;
            }
            // Only earlier crossed skins can shield this component. A later
            // intersection cannot be moved ahead of the component to infer
            // an unauthored enclosure, including a surface's mid-plane.
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
            out.angularImpulseBody += cross(rayFromBody, j); // Any point on this fragment's actual ray gives its moment.
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

namespace {
bool finiteScene(V3 v) {return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
bool validSceneTarget(const Target& target) {
    if(!finiteScene(target.at)||!finiteScene(target.velocity)||!finiteScene(target.angularVelocityEcef)||
       !std::isfinite(length(target.at))||!std::isfinite(length(target.velocity))||!std::isfinite(length(target.angularVelocityEcef))||
       !std::isfinite(target.airPressurePa)||!(target.airPressurePa>0)||!std::isfinite(target.airTemperatureK)||
       !(target.airTemperatureK>0)||!std::isfinite(target.airDensity)||target.airDensity<0)return false;
    for(std::size_t i=0;i<3;++i) {
        if(!finiteScene(target.bodyToEcef[i]))return false;
        for(std::size_t j=0;j<3;++j)
            if(std::abs(dot(target.bodyToEcef[i],target.bodyToEcef[j])-(i==j?1:0))>1e-8)return false;
    }
    return dot(cross(target.bodyToEcef[0],target.bodyToEcef[1]),target.bodyToEcef[2])>=1-1e-8;
}
struct MaterialSceneVictimV2 {
    std::uint64_t identity=0,generation=0;
    const AttachedGeometryView* geometry=nullptr;
    Target target;
    std::vector<SceneVictimV1::MotionSample> motion;
};
bool sceneTargetAt(const MaterialSceneVictimV2& victim,double age,double& nextBoundary,Target& target) {
    target=victim.target;
    if(victim.motion.empty()){target.at+=target.velocity*age;return true;}
    if(age<0||age>victim.motion.back().offsetS)return false;
    const auto after=std::upper_bound(victim.motion.begin(),victim.motion.end(),age,
        [](double time,const SceneVictimV1::MotionSample& sample){return time<sample.offsetS;});
    if(after==victim.motion.begin())return false;
    const auto& sample=*std::prev(after);target=sample.target;
    if(after!=victim.motion.end()) {
        target.velocity=(after->target.at-sample.target.at)*(1/(after->offsetS-sample.offsetS));
        nextBoundary=std::min(nextBoundary,after->offsetS);
    }
    target.at+=target.velocity*(age-sample.offsetS);return true;
}
struct FlightInterval {V3 at,velocity;};
FlightInterval fragmentFlight(V3 at,V3 velocity,double duration,double drag,V3 air,V3 gravity) {
    // Symmetric gravity kicks surround the exact constant-air drag solve.
    // Vacuum gravity is exact; drag/gravity coupling converges quadratically.
    const V3 halfVelocity=velocity+gravity*(duration/2);
    const V3 relative=halfVelocity-air;const double speed=length(relative),k=drag*speed;
    if(k==0)return {at+velocity*duration+gravity*(duration*duration/2),velocity+gravity*duration};
    const double attenuation=1/(1+k*duration);
    const double distanceFactor=std::log1p(k*duration)/k;
    return {at+air*duration+relative*distanceFactor,air+relative*attenuation+gravity*(duration/2)};
}
// A finite expanding line against convex geometry. With d=b-a and
// b'(t)-a'(t)=k*d, x=a+v*t+d*q, 0<=q<=1+k*t. Every face is a half-plane in
// (t,q); clipping their intersection finds the first material contact, including
// a hit between endpoints. No endpoint-only or bounding-sphere substitution.
struct TimeCoordinate {double time,along;};
struct ScenePlane {V3 normal;double bound;};
bool finiteLineContact(V3 a,V3 velocity,V3 d,double k,double duration,
                       const std::vector<ScenePlane>& planes,double& time,double& along,V3& normal,bool exit=false,
                       bool closedSkin=false) {
    std::vector<TimeCoordinate> polygon={{0,0},{duration,0},{duration,1+k*duration},{0,1}};
    for(const auto& plane:planes) {
        if(polygon.empty())return false;
        const double offset=dot(plane.normal,a)-plane.bound;
        const double rate=dot(plane.normal,velocity),slope=dot(plane.normal,d);
        const auto value=[&](TimeCoordinate p){return offset+rate*p.time+slope*p.along;};
        std::vector<TimeCoordinate> next;
        auto before=polygon.back();double vb=value(before);
        for(const auto after:polygon) {
            const double va=value(after);
            if((vb<=0)!=(va<=0)) {
                const double fraction=vb/(vb-va);
                next.push_back({before.time+fraction*(after.time-before.time),before.along+fraction*(after.along-before.along)});
            }
            if(va<=0)next.push_back(after);
            before=after;vb=va;
        }
        polygon=std::move(next);
    }
    if(polygon.empty())return false;
    const auto ordering=[](auto left,auto right){return left.time<right.time || (left.time==right.time&&left.along<right.along);};
    const auto first=exit?std::max_element(polygon.begin(),polygon.end(),ordering):std::min_element(polygon.begin(),polygon.end(),ordering);
    time=first->time;along=first->along;normal={};double nearest=std::numeric_limits<double>::infinity();
    if(closedSkin&&!exit&&time==0) {
        // Feasible occupancy at birth is contact for a solid component, but
        // is not an entry through an enclosing shell. A wholly interior line
        // must reach its actual exit face before paying penetration work.
        // Retain initial straddles and an inward-moving material endpoint on
        // a boundary; an outward boundary is handled only by the exit query.
        bool contained=true,inwardBoundary=false;
        for(const auto& plane:planes) {
            const double start=dot(plane.normal,a)-plane.bound;
            const double end=dot(plane.normal,a+d)-plane.bound;
            if(start>0||end>0)contained=false;
            const double rate=dot(plane.normal,velocity);
            if((start==0&&rate<0)||(end==0&&rate+k*dot(plane.normal,d)<0))inwardBoundary=true;
        }
        if(contained&&!inwardBoundary)return false;
    }
    if(exit&&time>=duration)return false; // An interval endpoint does not prove geometric exit.
    const V3 point=a+velocity*time+d*along;
    for(const auto& plane:planes) {
        const double error=std::abs(dot(plane.normal,point)-plane.bound);
        if(error<nearest){nearest=error;normal=plane.normal;}
    }
    return std::isfinite(time)&&std::isfinite(along)&&time>=0&&time<=duration;
}
std::vector<ScenePlane> boxPlanes(V3 centre,V3 half) {
    return {{{1,0,0},centre.x+half.x},{{-1,0,0},-centre.x+half.x},
            {{0,1,0},centre.y+half.y},{{0,-1,0},-centre.y+half.y},
            {{0,0,1},centre.z+half.z},{{0,0,-1},-centre.z+half.z}};
}
std::vector<ScenePlane> facePlanes(const Airframe::Panel& panel) {
    const auto& corners=panel.corners;const V3 normal=normalized(cross(corners[2]-corners[0],corners[3]-corners[1]));
    if(length(normal)==0)return {};
    // A numerical face thickness, many orders below the authored skin. It
    // resolves roundoff in half-plane clipping and is not an explosive radius.
    constexpr double faceRoundoffM=1e-10;
    std::vector<ScenePlane> planes={{normal,dot(normal,corners[0])+faceRoundoffM},
                                   {normal*(-1),-dot(normal,corners[0])+faceRoundoffM}};
    const V3 middle=panel.middle();
    for(std::size_t i=0;i<4;++i) {
        V3 outward=normalized(cross(corners[(i+1)%4]-corners[i],normal));
        if(dot(outward,middle-corners[i])>0)outward=outward*(-1);
        planes.push_back({outward,dot(outward,corners[i])});
    }
    return planes;
}
Airframe::Panel surfaceFace(const Airframe::Panel& panel,int side) {
    auto face=panel;
    const auto normal=normalized(cross(panel.corners[2]-panel.corners[0],panel.corners[3]-panel.corners[1]));
    const auto shift=normal*(.5*panel.thicknessM*(side==0?-1:1));
    for(auto& corner:face.corners)corner+=shift;
    return face;
}
struct SceneSkinTriangle {
    std::array<V3,3> vertices;
    V3 normal;
    int face=0;
};
std::vector<SceneSkinTriangle> authoredSkinTriangles(const ContactMesh& mesh) {
    // Keep distinct surfaces of a nonconvex shell. Only adjacent coplanar
    // triangles share a crossing identity, so a triangulation diagonal cannot
    // charge penetration work twice. Ownership caps are never material skin,
    // including after the neighboring section has departed.
    std::vector<std::size_t> roots(mesh.triangles.size());
    for(std::size_t i=0;i<roots.size();++i)roots[i]=i;
    const auto root=[&](std::size_t i){while(roots[i]!=i)i=roots[i];return i;};
    std::map<std::pair<std::size_t,std::size_t>,std::size_t> edges;
    for(std::size_t i=0;i<mesh.triangles.size();++i)if(!mesh.triangleNeighbors[i]) {
        const auto& triangle=mesh.triangles[i];
        for(std::size_t k=0;k<3;++k) {
            const auto a=triangle[k],b=triangle[(k+1)%3];
            const auto edge=std::make_pair(std::min(a,b),std::max(a,b));
            const auto previous=edges.find(edge);
            if(previous==edges.end()){edges.emplace(edge,i);continue;}
            const auto& other=mesh.triangles[previous->second];std::size_t opposite=other[0];
            for(const auto vertex:other)if(vertex!=a&&vertex!=b)opposite=vertex;
            const V3 e=mesh.vertices[b]-mesh.vertices[a],c=mesh.vertices[triangle[(k+2)%3]]-mesh.vertices[a],
                     d=mesh.vertices[opposite]-mesh.vertices[a];
            const double determinant=dot(e,cross(c,d));
            const double rounding=64*std::numeric_limits<double>::epsilon()*
                (std::abs(e.x)*(std::abs(c.y*d.z)+std::abs(c.z*d.y))+
                 std::abs(e.y)*(std::abs(c.z*d.x)+std::abs(c.x*d.z))+
                 std::abs(e.z)*(std::abs(c.x*d.y)+std::abs(c.y*d.x)));
            if(std::abs(determinant)<=rounding) {
                const auto left=root(i),right=root(previous->second);
                roots[std::max(left,right)]=std::min(left,right);
            }
        }
    }
    std::vector<SceneSkinTriangle> result;
    for(std::size_t i=0;i<mesh.triangles.size();++i)if(!mesh.triangleNeighbors[i]) {
        const auto& indices=mesh.triangles[i];
        const std::array<V3,3> vertices={mesh.vertices[indices[0]],mesh.vertices[indices[1]],mesh.vertices[indices[2]]};
        result.push_back({vertices,normalized(cross(vertices[1]-vertices[0],vertices[2]-vertices[0])),static_cast<int>(root(i))+2});
    }
    return result;
}
std::vector<ScenePlane> trianglePlanes(const SceneSkinTriangle& triangle) {
    const auto& a=triangle.vertices[0];const auto& n=triangle.normal;
    std::vector<ScenePlane> planes={{n,dot(n,a)},{n*(-1),-dot(n,a)}};
    for(std::size_t i=0;i<3;++i) {
        const auto& start=triangle.vertices[i];
        const V3 outward=cross(triangle.vertices[(i+1)%3]-start,n);
        planes.push_back({outward,dot(outward,start)});
    }
    return planes;
}
bool overlaps(fluid::Aabb a,fluid::Aabb b,bool strict=false) {
    if(strict)return a.low.x<b.high.x&&b.low.x<a.high.x&&a.low.y<b.high.y&&b.low.y<a.high.y&&a.low.z<b.high.z&&b.low.z<a.high.z;
    return a.low.x<=b.high.x&&b.low.x<=a.high.x&&a.low.y<=b.high.y&&b.low.y<=a.high.y&&a.low.z<=b.high.z&&b.low.z<=a.high.z;
}
fluid::Aabb unite(fluid::Aabb a,fluid::Aabb b) {
    return {{std::min(a.low.x,b.low.x),std::min(a.low.y,b.low.y),std::min(a.low.z,b.low.z)},
            {std::max(a.high.x,b.high.x),std::max(a.high.y,b.high.y),std::max(a.high.z,b.high.z)}};
}
double capacityAxis(V3 value,unsigned axis) {return axis==0?value.x:axis==1?value.y:value.z;}
V3 capacityNormal(unsigned axis,double sign=1) {return axis==0?V3{sign,0,0}:axis==1?V3{0,sign,0}:V3{0,0,sign};}
struct LiquidFacePatch {
    unsigned axis=0;
    double coordinate=0,u0=0,u1=0,v0=0,v1=0;
    bool positive=false;
};
std::vector<ScenePlane> capacityFacePlanes(const LiquidFacePatch& face) {
    const V3 normal=capacityNormal(face.axis),u=capacityNormal((face.axis+1)%3),v=capacityNormal((face.axis+2)%3);
    return {{normal,face.coordinate},{normal*(-1),-face.coordinate},{u,face.u1},{u*(-1),-face.u0},{v,face.v1},{v*(-1),-face.v0}};
}
struct LiquidTree {
    struct Node {fluid::Aabb bounds;std::size_t begin=0,end=0,left=0,right=0;};
    const SceneLiquidRecipientV1* source=nullptr;
    std::vector<std::size_t> order;
    std::vector<Node> nodes;
    std::map<int,const LiquidTankStateV1*> tanks;
    mutable std::map<std::size_t,std::vector<LiquidFacePatch>> boundaryCache;
    mutable std::size_t boundaryPatchCount=0;
    mutable bool boundaryValid=true;
    std::size_t build(std::size_t begin,std::size_t end) {
        const std::size_t index=nodes.size();nodes.push_back({source->cells[order[begin]].bodyBounds,begin,end,0,0});
        for(std::size_t i=begin+1;i<end;++i)nodes[index].bounds=unite(nodes[index].bounds,source->cells[order[i]].bodyBounds);
        if(end-begin>8) {
            const V3 span=nodes[index].bounds.high-nodes[index].bounds.low;
            const unsigned axis=span.x>=span.y&&span.x>=span.z?0u:span.y>=span.z?1u:2u;
            const auto coordinate=[&](std::size_t i){const auto& b=source->cells[i].bodyBounds;const V3 c=b.low*.5+b.high*.5;return axis==0?c.x:axis==1?c.y:c.z;};
            const auto middle=begin+(end-begin)/2;
            std::nth_element(order.begin()+static_cast<std::ptrdiff_t>(begin),order.begin()+static_cast<std::ptrdiff_t>(middle),order.begin()+static_cast<std::ptrdiff_t>(end),
                [&](std::size_t a,std::size_t b){const double x=coordinate(a),y=coordinate(b);return x<y||(x==y&&source->cells[a].stableCellId<source->cells[b].stableCellId);});
            const auto left=build(begin,middle),right=build(middle,end);nodes[index].left=left;nodes[index].right=right;
        }
        return index;
    }
    void query(fluid::Aabb bounds,std::vector<std::size_t>& out,std::size_t index=0) const {
        if(nodes.empty()||!overlaps(bounds,nodes[index].bounds))return;
        const auto& node=nodes[index];
        if(node.left){query(bounds,out,node.left);query(bounds,out,node.right);}
        else for(std::size_t i=node.begin;i<node.end;++i)if(overlaps(bounds,source->cells[order[i]].bodyBounds))out.push_back(order[i]);
    }
    const std::vector<LiquidFacePatch>& boundary(std::size_t cellIndex) const {
        const auto retained=boundaryCache.find(cellIndex);if(retained!=boundaryCache.end())return retained->second;
        auto& output=boundaryCache[cellIndex];const auto& cell=source->cells[cellIndex];
        std::vector<std::size_t> neighbors;query(cell.bodyBounds,neighbors);
        std::sort(neighbors.begin(),neighbors.end(),[&](auto a,auto b){return source->cells[a].stableCellId<source->cells[b].stableCellId;});
        // Explicit finite topology budget, independent of physical coefficients.
        // Construction is lazy; a 65k-cell input does not rebuild all faces for
        // a projectile whose actual swept bounds touch only a few cells.
        const std::size_t budget=source->cells.size()*32+6;
        for(unsigned axis=0;axis<3;++axis)for(const bool positive:{false,true}) {
            const unsigned u=(axis+1)%3,v=(axis+2)%3;
            const double coordinate=capacityAxis(positive?cell.bodyBounds.high:cell.bodyBounds.low,axis);
            std::vector<LiquidFacePatch> patches={{axis,coordinate,capacityAxis(cell.bodyBounds.low,u),capacityAxis(cell.bodyBounds.high,u),
                capacityAxis(cell.bodyBounds.low,v),capacityAxis(cell.bodyBounds.high,v),positive}};
            for(const auto index:neighbors) {
                const auto& neighbor=source->cells[index];
                if(index==cellIndex||neighbor.nativeTank!=cell.nativeTank||
                   capacityAxis(positive?neighbor.bodyBounds.low:neighbor.bodyBounds.high,axis)!=coordinate)continue;
                const double u0=capacityAxis(neighbor.bodyBounds.low,u),u1=capacityAxis(neighbor.bodyBounds.high,u),
                             v0=capacityAxis(neighbor.bodyBounds.low,v),v1=capacityAxis(neighbor.bodyBounds.high,v);
                std::vector<LiquidFacePatch> next;
                for(const auto& patch:patches) {
                    const double lowU=std::max(patch.u0,u0),highU=std::min(patch.u1,u1),
                                 lowV=std::max(patch.v0,v0),highV=std::min(patch.v1,v1);
                    if(!(lowU<highU&&lowV<highV)){next.push_back(patch);continue;}
                    if(patch.u0<lowU)next.push_back({axis,coordinate,patch.u0,lowU,patch.v0,patch.v1,positive});
                    if(highU<patch.u1)next.push_back({axis,coordinate,highU,patch.u1,patch.v0,patch.v1,positive});
                    if(patch.v0<lowV)next.push_back({axis,coordinate,lowU,highU,patch.v0,lowV,positive});
                    if(highV<patch.v1)next.push_back({axis,coordinate,lowU,highU,highV,patch.v1,positive});
                    if(next.size()>budget){boundaryValid=false;return output;}
                }
                patches=std::move(next);if(patches.empty())break;
            }
            if(patches.size()>budget-boundaryPatchCount||output.size()>budget-boundaryPatchCount-patches.size()) {
                boundaryValid=false;return output;
            }
            output.insert(output.end(),patches.begin(),patches.end());
        }
        boundaryPatchCount+=output.size();return output;
    }
};
struct PreparedLiquids {
    SceneLiquidControlsV1 controls;
    std::vector<std::optional<LiquidTree>> recipients;
};
bool prepareLiquids(const SceneLiquidInputsV1& input,const std::vector<const MaterialSceneVictimV2*>& victims,PreparedLiquids& output) {
    const auto& c=input.controls;
    if(!std::isfinite(c.maxStepS)||!(c.maxStepS>0)||!std::isfinite(c.minStepS)||!(c.minStepS>0)||c.minStepS>c.maxStepS||
       !std::isfinite(c.absolutePositionToleranceM)||!(c.absolutePositionToleranceM>0)||
       !std::isfinite(c.absoluteVelocityToleranceMs)||!(c.absoluteVelocityToleranceMs>0)||
       !std::isfinite(c.relativeTolerance)||c.relativeTolerance<0||
       !std::isfinite(c.absoluteVolumeToleranceM3)||c.absoluteVolumeToleranceM3<0||
       !std::isfinite(c.relativeVolumeTolerance)||c.relativeVolumeTolerance<0||
       !(c.absoluteVolumeToleranceM3>0||c.relativeVolumeTolerance>0)||!c.maxSteps||
       !std::isfinite(c.quadrature.absoluteForceToleranceN)||c.quadrature.absoluteForceToleranceN<0||
       !std::isfinite(c.quadrature.relativeTolerance)||c.quadrature.relativeTolerance<0||
       !(c.quadrature.absoluteForceToleranceN>0||c.quadrature.relativeTolerance>0)||
       !c.quadrature.maxDepth||c.quadrature.maxDepth>30||c.quadrature.maxEvaluations<5)return false;
    output.controls=c;output.recipients.resize(victims.size());
    for(const auto& supplied:input.recipients) {
        const auto found=std::lower_bound(victims.begin(),victims.end(),supplied.identity,[](auto victim,std::uint64_t id){return victim->identity<id;});
        if(found==victims.end()||(*found)->identity!=supplied.identity||(*found)->generation!=supplied.generation||supplied.cells.size()>65536)return false;
        const auto vi=static_cast<std::size_t>(found-victims.begin());if(output.recipients[vi])return false;
        LiquidTree tree;tree.source=&supplied;
        for(const auto& tank:supplied.tanks) {
            if(tank.nativeTank<0||!std::isfinite(tank.densityKgM3)||!(tank.densityKgM3>0)||
               !std::isfinite(tank.dragCoefficient)||!(tank.dragCoefficient>0)||
               !std::isfinite(tank.actualContentsKg)||tank.actualContentsKg<0||!finiteScene(tank.freeSurfaceBody.normal)||
               !(length(tank.freeSurfaceBody.normal)>0)||!std::isfinite(tank.freeSurfaceBody.offset)||
               !tree.tanks.emplace(tank.nativeTank,&tank).second)return false;
        }
        const auto* geometry=(*found)->geometry;
        if(geometry)for(const auto index:geometry->componentIndices())if(geometry->airframe().components[index].kind=="fuel")
            if(!tree.tanks.count(geometry->airframe().components[index].tank))return false;
        std::set<std::uint64_t> ids;std::map<int,double> volume,capacity;
        for(std::size_t i=0;i<supplied.cells.size();++i) {
            const auto& cell=supplied.cells[i];
            if(!cell.stableCellId||!ids.insert(cell.stableCellId).second||!tree.tanks.count(cell.nativeTank)||
               !fluid::detail::box(cell.bodyBounds))return false;
            if(geometry) {
                if(!std::binary_search(geometry->componentIndices().begin(),geometry->componentIndices().end(),cell.componentIndex))return false;
                const auto& component=geometry->airframe().components[cell.componentIndex];
                if(component.kind!="fuel"||component.tank!=cell.nativeTank||component.section!=cell.section)return false;
            } else if(cell.section!=-1||cell.componentIndex!=static_cast<std::size_t>(cell.nativeTank))return false;
            const auto wet=fluid::immersedVolume(cell.bodyBounds,tree.tanks.at(cell.nativeTank)->freeSurfaceBody);
            if(!wet)return false;
            volume[cell.nativeTank]+=*wet;capacity[cell.nativeTank]+=fluid::detail::box(cell.bodyBounds)->volume;
            if(!std::isfinite(volume[cell.nativeTank])||!std::isfinite(capacity[cell.nativeTank]))return false;
            tree.order.push_back(i);
        }
        if(!tree.order.empty())tree.build(0,tree.order.size());
        for(std::size_t i=0;i<supplied.cells.size();++i) {
            std::vector<std::size_t> neighbors;tree.query(supplied.cells[i].bodyBounds,neighbors);
            for(const auto j:neighbors)if(j>i&&overlaps(supplied.cells[i].bodyBounds,supplied.cells[j].bodyBounds,true))return false;
        }
        for(const auto& [id,tank]:tree.tanks) {
            const double actual=tank->actualContentsKg/tank->densityKgM3;
            if(!std::isfinite(actual)||(actual>0&&!capacity.count(id)))return false;
            const double tolerance=c.absoluteVolumeToleranceM3+c.relativeVolumeTolerance*capacity[id];
            if(!std::isfinite(tolerance)||std::abs(volume[id]-actual)>tolerance)return false;
        }
        output.recipients[vi]=std::move(tree);
    }
    return true;
}
struct LiquidLoad {V3 impulse,angular;double energy=0;};
using LiquidLoadMap=std::map<std::pair<std::size_t,std::size_t>,LiquidLoad>;
struct LiquidDerivative {
    V3 position0,position1,velocity0,velocity1,airForce;
    LiquidLoadMap loads;
    bool valid=true;
};
struct LiquidMotion {V3 position0,position1,velocity0,velocity1;};
using LiquidPointPhase=std::vector<std::pair<std::size_t,std::size_t>>;
struct LiquidAdvance {
    bool valid=false;
    LiquidMotion motion;
    V3 airImpulse;
    LiquidLoadMap loads;
    std::optional<double> firstInteractionS;
};
fluid::Interval pointWetInterval(fluid::Aabb bounds,fluid::Plane surface,V3 start,V3 end) {
    // Clip against the original shared endpoint planes, not each cell's
    // separately rounded centre/half representation. Adjacent cells then
    // produce identical cuts on their common source plane, while a genuine
    // finite gap remains a gap without a geometric merging tolerance.
    fluid::Interval out;out.valid=true;out.first=0;out.last=1;
    const auto clip=[&](V3 normal,double bound) {
        const long double a=fluid::detail::scalar(normal,start)-bound,b=fluid::detail::scalar(normal,end)-bound;
        if(!std::isfinite(a)||!std::isfinite(b)){out.valid=false;return false;}
        if(a<=0&&b<=0)return true;
        if(a>0&&b>0)return false;
        const double cut=static_cast<double>(a/(a-b));
        if(a>0)out.first=std::max(out.first,cut);else out.last=std::min(out.last,cut);
        return out.first<=out.last;
    };
    if(!clip({1,0,0},bounds.high.x)||!clip({-1,0,0},-bounds.low.x)||
       !clip({0,1,0},bounds.high.y)||!clip({0,-1,0},-bounds.low.y)||
       !clip({0,0,1},bounds.high.z)||!clip({0,0,-1},-bounds.low.z)||
       !clip(surface.normal,surface.offset))return out;
    out.intersects=true;return out;
}
bool directedInside(const LiquidCapacityCellV1& cell,const LiquidTankStateV1& tank,V3 point,V3 direction,
    const V3* acceleration=nullptr,bool* tangent=nullptr) {
    const auto& b=cell.bodyBounds;
    const double p[]{point.x,point.y,point.z},v[]{direction.x,direction.y,direction.z},lo[]{b.low.x,b.low.y,b.low.z},hi[]{b.high.x,b.high.y,b.high.z};
    const double a[]{acceleration?acceleration->x:0,acceleration?acceleration->y:0,acceleration?acceleration->z:0};
    for(unsigned i=0;i<3;++i) {
        if(tangent&&(p[i]==lo[i]||p[i]==hi[i])&&v[i]==0)*tangent=true;
        if(p[i]<lo[i]||p[i]>hi[i]||(p[i]==lo[i]&&(v[i]<0||(v[i]==0&&a[i]<0)))||
           (p[i]==hi[i]&&(v[i]>0||(v[i]==0&&a[i]>0))))return false;
    }
    const double height=dot(tank.freeSurfaceBody.normal,point)-tank.freeSurfaceBody.offset;
    const double rate=dot(tank.freeSurfaceBody.normal,direction);
    if(tangent&&height==0&&rate==0)*tangent=true;
    return height<0||(height==0&&(rate<0||(rate==0&&(!acceleration||dot(tank.freeSurfaceBody.normal,*acceleration)<=0))));
}
struct LiquidOwnedInterval {double first=0,last=0;std::size_t cell=0;};
std::optional<std::vector<LiquidOwnedInterval>> liquidOwnedIntervals(const LiquidTree& tree,
    const std::vector<std::size_t>& candidates,V3 a,V3 b,V3 direction0,V3 direction1) {
    struct Span {double first=0,last=0;std::size_t cell=0;std::vector<std::pair<long double,long double>> directions;};
    struct Event {double at=0;std::size_t span=0;bool opening=false;};
    std::vector<Span> spans;std::vector<Event> events;
    const auto noSpan=std::numeric_limits<std::size_t>::max();
    for(const auto ci:candidates) {
        const auto& cell=tree.source->cells[ci];const auto& tank=*tree.tanks.at(cell.nativeTank);
        if(tank.actualContentsKg==0)continue;
        const auto wet=pointWetInterval(cell.bodyBounds,tank.freeSurfaceBody,a,b);
        if(!wet.valid)return std::nullopt;
        if(!wet.intersects||wet.first==wet.last)continue;
        Span span{wet.first,wet.last,ci,{}};
        const auto& bounds=cell.bodyBounds;
        const std::array<std::pair<V3,double>,7> planes{{{{1,0,0},bounds.high.x},{{-1,0,0},-bounds.low.x},
            {{0,1,0},bounds.high.y},{{0,-1,0},-bounds.low.y},{{0,0,1},bounds.high.z},{{0,0,-1},-bounds.low.z},
            {tank.freeSurfaceBody.normal,tank.freeSurfaceBody.offset}}};
        for(const auto& plane:planes) {
            if(fluid::detail::scalar(plane.first,a)-plane.second!=0||
               fluid::detail::scalar(plane.first,b)-plane.second!=0)continue;
            const long double start=fluid::detail::scalar(plane.first,direction0),
                change=fluid::detail::scalar(plane.first,direction1)-start;
            span.directions.emplace_back(start,change);
            if(change!=0) {
                const double zero=static_cast<double>(-start/change);
                if(zero>wet.first&&zero<wet.last)events.push_back({zero,noSpan,false});
            }
        }
        const auto index=spans.size();spans.push_back(std::move(span));
        events.push_back({wet.first,index,true});events.push_back({wet.last,index,false});
    }
    std::sort(events.begin(),events.end(),[](const auto& x,const auto& y){return x.at<y.at;});
    std::set<std::pair<std::uint64_t,std::size_t>> active;std::vector<LiquidOwnedInterval> owned;
    for(std::size_t cursor=0;cursor<events.size();) {
        const double first=events[cursor].at;
        do {
            const auto& event=events[cursor++];if(event.span==noSpan)continue;
            const auto key=std::make_pair(tree.source->cells[spans[event.span].cell].stableCellId,event.span);
            if(event.opening)active.insert(key);else active.erase(key);
        }while(cursor<events.size()&&events[cursor].at==first);
        if(cursor==events.size())break;
        const double last=events[cursor].at;const long double middle=static_cast<long double>(first)+
            (static_cast<long double>(last)-first)/2;
        for(const auto& entry:active) {
            const auto& span=spans[entry.second];bool admitted=true;
            for(const auto& direction:span.directions)
                if(direction.first+direction.second*middle>0){admitted=false;break;}
            if(!admitted)continue;
            // A material coordinate has one medium. Entering motion selects
            // the occupied side; an exact tangent tie uses stable cell ID,
            // including interfaces between different native tanks/probes.
            if(!owned.empty()&&owned.back().cell==span.cell&&owned.back().last==first)owned.back().last=last;
            else owned.push_back({first,last,span.cell});
            break;
        }
    }
    return owned;
}
LiquidDerivative liquidDerivative(const FragmentFieldV1::Projectile& p,const LiquidMotion& state,double time,
    const SceneEnvironmentV1& environment,const PreparedLiquids& liquids,const std::vector<Target>& targets,
    const LiquidPointPhase* pointPhase=nullptr) {
    LiquidDerivative result;result.position0=state.velocity0;result.position1=state.velocity1;
    result.velocity0=result.velocity1=environment.gravityEcef;
    if(!p.segment&&pointPhase) {
        if(pointPhase->empty()) {
            const V3 v=state.velocity0-environment.airVelocity;
            const V3 force=v*(-.5*environment.airDensity*p.cd*p.areaM2*length(v));
            result.velocity0+=force*(1/p.massKg);result.airForce=force*(-1);
        }
        for(const auto& owner:*pointPhase) {
            const auto& tree=*liquids.recipients[owner.first];const auto& target=targets[owner.first];
            const auto& cell=tree.source->cells[owner.second];const auto& tank=*tree.tanks.at(cell.nativeTank);
            const V3 origin=target.at+target.velocity*time;
            const V3 v=state.velocity0-target.velocity-cross(target.angularVelocityEcef,state.position0-origin);
            const V3 force=v*(-.5*tank.densityKgM3*tank.dragCoefficient*p.areaM2*length(v));
            result.velocity0+=force*(1/p.massKg);auto& load=result.loads[{owner.first,cell.componentIndex}];
            load.impulse=force*(-1);load.angular=cross(state.position0-origin,force)*(-1);load.energy=-dot(v,force);
        }
        result.valid=finiteScene(result.velocity0)&&finiteScene(result.airForce);return result;
    }
    const auto apply=[&](V3 force0,V3 force1){
        if(p.segment){result.velocity0+=(force0*4-force1*2)*(1/p.massKg);result.velocity1+=(force1*4-force0*2)*(1/p.massKg);}
        else result.velocity0+=force0*(1/p.massKg);
    };
    bool pointWet=false;
    std::vector<std::pair<double,double>> wetMaterial;
    for(std::size_t vi=0;vi<liquids.recipients.size();++vi)if(liquids.recipients[vi]) {
        const auto& tree=*liquids.recipients[vi];const auto& target=targets[vi];const V3 origin=target.at+target.velocity*time;
        const V3 a=toBody(target,state.position0-origin),b=p.segment?toBody(target,state.position1-origin):a;
        std::vector<std::size_t> candidates;tree.query(unite({a,a},{b,b}),candidates);
        std::sort(candidates.begin(),candidates.end(),[&](auto x,auto y){return tree.source->cells[x].stableCellId<tree.source->cells[y].stableCellId;});
        const V3 medium0=target.velocity+cross(target.angularVelocityEcef,state.position0-origin);
        const V3 medium1=target.velocity+cross(target.angularVelocityEcef,state.position1-origin);
        const auto record=[&](const LiquidCapacityCellV1& cell,V3 f0,V3 f1,double relativePower) {
            apply(f0,f1);auto& load=result.loads[{vi,cell.componentIndex}];load.impulse+=(f0+f1)*(-1);
            load.angular+=(cross(state.position0-origin,f0)+cross(state.position1-origin,f1))*(-1);load.energy-=relativePower;
        };
        if(p.segment) {
            const auto owned=liquidOwnedIntervals(tree,candidates,a,b,toBody(target,state.velocity0-medium0),toBody(target,state.velocity1-medium1));
            if(!owned){result.valid=false;return result;}
            for(const auto& wet:*owned) {
                const auto& cell=tree.source->cells[wet.cell];const auto& tank=*tree.tanks.at(cell.nativeTank);
                wetMaterial.emplace_back(wet.first,wet.last);
                const auto load=fluid::rodLoads({state.position0,state.position1,state.velocity0-medium0+target.velocity,
                    state.velocity1-medium1+target.velocity,p.massKg},{target.velocity,tank.densityKgM3,tank.dragCoefficient,p.areaM2},
                    wet.first,wet.last,liquids.controls.quadrature);
                if(!load.valid||!load.converged){result.valid=false;return result;}
                record(cell,load.force0,load.force1,load.relativePowerW);
            }
        } else for(const auto ci:candidates) {
            const auto& cell=tree.source->cells[ci];const auto& tank=*tree.tanks.at(cell.nativeTank);
            if(tank.actualContentsKg==0||!directedInside(cell,tank,a,toBody(target,state.velocity0-medium0)))continue;
            pointWet=true;const V3 v=state.velocity0-medium0;
            const V3 force=v*(-.5*tank.densityKgM3*tank.dragCoefficient*p.areaM2*length(v));record(cell,force,{},dot(v,force));
            break;
        }
    }
    // Liquid replaces ambient air over the actually immersed material, even
    // when it is comoving with that liquid and its instantaneous load is zero.
    // The exact interval union retains true dry gaps and integrates varying
    // rod velocities with their original full-rod shape functions.
    if(p.segment) {
        std::sort(wetMaterial.begin(),wetMaterial.end());
        const auto air=[&](double first,double last) {
            if(first==last)return true;
            const auto load=fluid::rodLoads({state.position0,state.position1,state.velocity0,state.velocity1,p.massKg},
                {environment.airVelocity,environment.airDensity,p.cd,p.areaM2},first,last,liquids.controls.quadrature);
            if(!load.valid||!load.converged)return false;
            apply(load.force0,load.force1);result.airForce+=load.force*(-1);return true;
        };
        double dryStart=0;
        for(const auto& interval:wetMaterial) {
            if(interval.first>dryStart&&!air(dryStart,interval.first)){result.valid=false;return result;}
            dryStart=std::max(dryStart,interval.second);
        }
        if(dryStart<1&&!air(dryStart,1)){result.valid=false;return result;}
    } else if(!pointWet) {
        const V3 v=state.velocity0-environment.airVelocity;
        const V3 force=v*(-.5*environment.airDensity*p.cd*p.areaM2*length(v));apply(force,{});result.airForce=force*(-1);
    }
    if(!finiteScene(result.velocity0)||!finiteScene(result.velocity1)||!finiteScene(result.airForce))result.valid=false;
    return result;
}
LiquidMotion increment(LiquidMotion s,const LiquidDerivative& d,double h) {
    s.position0+=d.position0*h;s.position1+=d.position1*h;s.velocity0+=d.velocity0*h;s.velocity1+=d.velocity1*h;return s;
}
void accumulate(LiquidAdvance& out,const LiquidDerivative& derivative,double scale) {
    out.airImpulse+=derivative.airForce*scale;
    for(const auto& [key,value]:derivative.loads){auto& load=out.loads[key];load.impulse+=value.impulse*scale;load.angular+=value.angular*scale;load.energy+=value.energy*scale;}
}
LiquidAdvance liquidRk4(const FragmentFieldV1::Projectile& p,LiquidMotion state,double time,double h,
    const SceneEnvironmentV1& environment,const PreparedLiquids& liquids,const std::vector<Target>& targets,
    const LiquidPointPhase* pointPhase=nullptr) {
    LiquidAdvance out;
    if(!p.segment&&pointPhase&&length(environment.gravityEcef)==0&&pointPhase->size()<=1) {
        fluid::Medium medium{environment.airVelocity,environment.airDensity,p.cd,p.areaM2};
        bool uniform=true;
        if(!pointPhase->empty()) {
            const auto owner=pointPhase->front();const auto& tree=*liquids.recipients[owner.first];const auto& target=targets[owner.first];
            const auto& cell=tree.source->cells[owner.second];const auto& tank=*tree.tanks.at(cell.nativeTank);
            uniform=length(target.angularVelocityEcef)==0;medium={target.velocity,tank.densityKgM3,tank.dragCoefficient,p.areaM2};
        }
        if(uniform) {
            const auto step=fluid::advancePoint({state.position0,state.velocity0,p.massKg},medium,h);if(!step)return out;
            out.motion=state;out.motion.position0=step->position;out.motion.velocity0=step->velocity;
            if(pointPhase->empty())out.airImpulse=step->mediumImpulse;
            else {
                const auto owner=pointPhase->front();const auto& target=targets[owner.first];const auto& cell=liquids.recipients[owner.first]->source->cells[owner.second];
                auto& load=out.loads[{owner.first,cell.componentIndex}];load.impulse=step->mediumImpulse;load.energy=step->dissipatedEnergyJ;
                load.angular=cross(state.position0-(target.at+target.velocity*time),load.impulse);
            }
            out.valid=true;return out;
        }
    }
    const auto a=liquidDerivative(p,state,time,environment,liquids,targets,pointPhase);if(!a.valid)return out;
    const auto b=liquidDerivative(p,increment(state,a,h/2),time+h/2,environment,liquids,targets,pointPhase);if(!b.valid)return out;
    const auto c=liquidDerivative(p,increment(state,b,h/2),time+h/2,environment,liquids,targets,pointPhase);if(!c.valid)return out;
    const auto d=liquidDerivative(p,increment(state,c,h),time+h,environment,liquids,targets,pointPhase);if(!d.valid)return out;
    out.motion=increment(increment(increment(increment(state,a,h/6),b,h/3),c,h/3),d,h/6);
    accumulate(out,a,h/6);accumulate(out,b,h/3);accumulate(out,c,h/3);accumulate(out,d,h/6);out.valid=true;return out;
}
LiquidAdvance advanceLiquidPoint(const FragmentFieldV1::Projectile& p,double duration,const SceneEnvironmentV1& environment,
    const PreparedLiquids& liquids,const std::vector<Target>& targets) {
    LiquidAdvance out;out.motion={p.at,p.end,p.velocity,p.endVelocity};
    const auto phaseAt=[&](const LiquidMotion& state,double time,const V3* acceleration=nullptr,bool* tangent=nullptr) {
        LiquidPointPhase phase;
        for(std::size_t vi=0;vi<liquids.recipients.size();++vi)if(liquids.recipients[vi]) {
            const auto& tree=*liquids.recipients[vi];const auto& target=targets[vi];const V3 origin=target.at+target.velocity*time;
            const V3 at=toBody(target,state.position0-origin),direction=toBody(target,state.velocity0-target.velocity-
                cross(target.angularVelocityEcef,state.position0-origin));
            const V3 relativeAcceleration=acceleration?toBody(target,*acceleration-
                cross(target.angularVelocityEcef,state.velocity0-target.velocity)):V3{};
            std::vector<std::size_t> candidates;tree.query({at,at},candidates);
            std::sort(candidates.begin(),candidates.end(),[&](auto a,auto b){return tree.source->cells[a].stableCellId<tree.source->cells[b].stableCellId;});
            for(const auto ci:candidates) {
                const auto& cell=tree.source->cells[ci];const auto& tank=*tree.tanks.at(cell.nativeTank);
                if(tank.actualContentsKg>0&&directedInside(cell,tank,at,direction,acceleration?&relativeAcceleration:nullptr,tangent)){
                    phase.emplace_back(vi,ci);break;
                }
            }
        }
        return phase;
    };
    double time=0,h=std::min(duration,liquids.controls.maxStepS);unsigned steps=0;
    while(time<duration) {
        if(++steps>liquids.controls.maxSteps)return {};
        h=std::min(h,duration-time);bool tangent=false;auto phase=phaseAt(out.motion,time,nullptr,&tangent);
        if(tangent) {
            // At a true face with zero normal velocity, its instantaneous
            // physical acceleration selects the right-hand temporal phase.
            // No coordinate is displaced and a future wet region cannot
            // make the departing region act across a finite dry gap.
            const auto opening=liquidDerivative(p,out.motion,time,environment,liquids,targets,&phase);if(!opening.valid)return {};
            phase=phaseAt(out.motion,time,&opening.velocity0);
        }
        const auto whole=liquidRk4(p,out.motion,time,h,environment,liquids,targets,&phase);
        const auto left=liquidRk4(p,out.motion,time,h/2,environment,liquids,targets,&phase);
        if(!whole.valid||!left.valid)return {};
        const auto right=liquidRk4(p,left.motion,time+h/2,h/2,environment,liquids,targets,&phase);if(!right.valid)return {};
        const auto& controls=liquids.controls;
        if(length(whole.motion.position0-right.motion.position0)>controls.absolutePositionToleranceM+
                controls.relativeTolerance*length(right.motion.position0-out.motion.position0)||
           length(whole.motion.velocity0-right.motion.velocity0)>controls.absoluteVelocityToleranceMs+
                controls.relativeTolerance*length(right.motion.velocity0)) {
            if(h/2<controls.minStepS)return {};
            h/=2;continue;
        }
        double eventTime=h;bool event=false;
        for(std::size_t vi=0;vi<liquids.recipients.size();++vi)if(liquids.recipients[vi]) {
            const auto& tree=*liquids.recipients[vi];const auto& target=targets[vi];
            const V3 a=toBody(target,out.motion.position0-(target.at+target.velocity*time)),
                b=toBody(target,right.motion.position0-(target.at+target.velocity*(time+h)));
            const V3 direction=(b-a)*(1/h);std::vector<std::size_t> candidates;tree.query(unite({a,a},{b,b}),candidates);
            const auto owned=liquidOwnedIntervals(tree,candidates,a,b,direction,direction);if(!owned)return {};
            const auto current=std::find_if(phase.begin(),phase.end(),[&](const auto& owner){return owner.first==vi;});
            double cut=1;std::size_t cell=0;
            if(current==phase.end()){if(owned->empty())continue;cut=owned->front().first;cell=owned->front().cell;}
            else {
                const auto& initial=tree.source->cells[current->second];double end=0;cell=current->second;
                for(const auto& span:*owned) {
                    const auto& next=tree.source->cells[span.cell];
                    if(span.first!=end||next.nativeTank!=initial.nativeTank||next.componentIndex!=initial.componentIndex)break;
                    end=span.last;cell=span.cell;
                }
                cut=end;
            }
            if(!(cut>0&&cut<1))continue;
            const auto& part=tree.source->cells[cell];const auto& tank=*tree.tanks.at(part.nativeTank);const auto& bounds=part.bodyBounds;
            const std::array<std::pair<V3,double>,7> planes{{{{1,0,0},bounds.high.x},{{-1,0,0},-bounds.low.x},
                {{0,1,0},bounds.high.y},{{0,-1,0},-bounds.low.y},{{0,0,1},bounds.high.z},{{0,0,-1},-bounds.low.z},
                {tank.freeSurfaceBody.normal,tank.freeSurfaceBody.offset}}};
            for(const auto& plane:planes) {
                const long double start=fluid::detail::scalar(plane.first,a)-plane.second,
                    finish=fluid::detail::scalar(plane.first,b)-plane.second;
                if(start==finish||static_cast<double>(start/(start-finish))!=cut)continue;
                double low=0,high=h;
                for(unsigned iteration=0;iteration<56;++iteration) {
                    const double middle=low+(high-low)*.5;
                    const auto trial=liquidRk4(p,out.motion,time,middle,environment,liquids,targets,&phase);if(!trial.valid)return {};
                    const V3 at=toBody(target,trial.motion.position0-(target.at+target.velocity*(time+middle)));
                    const long double gap=fluid::detail::scalar(plane.first,at)-plane.second;
                    if((start>0&&gap>0)||(start<0&&gap<0))low=middle;else high=middle;
                }
                if(high<eventTime){eventTime=high;event=true;}
            }
        }
        LiquidAdvance accepted;
        if(event) {
            double end=time+eventTime;
            if(end-time<eventTime)end=std::nextafter(end,std::numeric_limits<double>::infinity());
            eventTime=std::min(end-time,h);
            accepted=liquidRk4(p,out.motion,time,eventTime,environment,liquids,targets,&phase);if(!accepted.valid)return {};
        } else {
            accepted.motion=right.motion;accepted.airImpulse=left.airImpulse+right.airImpulse;
            for(const auto* part:{&left,&right})for(const auto& [key,value]:part->loads){auto& load=accepted.loads[key];
                load.impulse+=value.impulse;load.angular+=value.angular;load.energy+=value.energy;}
            accepted.valid=true;
        }
        out.motion=accepted.motion;out.airImpulse+=accepted.airImpulse;
        for(const auto& [key,value]:accepted.loads){auto& load=out.loads[key];load.impulse+=value.impulse;load.angular+=value.angular;load.energy+=value.energy;}
        if(!out.firstInteractionS)for(const auto& [key,value]:accepted.loads) {
            (void)key;
            if(value.energy>0||length(value.impulse)>0||length(value.angular)>0){out.firstInteractionS=time;break;}
        }
        time+=eventTime;h=std::min(h*2,controls.maxStepS);
    }
    out.valid=finiteScene(out.motion.position0)&&finiteScene(out.motion.velocity0);return out;
}
LiquidAdvance advanceLiquid(const FragmentFieldV1::Projectile& p,double duration,const SceneEnvironmentV1& environment,
    const PreparedLiquids& liquids,const std::vector<Target>& targets) {
    LiquidAdvance out;out.motion={p.at,p.end,p.velocity,p.endVelocity};
    if(duration==0){out.valid=true;return out;}
    if(!p.segment)return advanceLiquidPoint(p,duration,environment,liquids,targets);
    double time=0,h=std::min(duration,liquids.controls.maxStepS);unsigned steps=0;
    while(time<duration) {
        if(++steps>liquids.controls.maxSteps)return {};
        h=std::min(h,duration-time);
        const auto whole=liquidRk4(p,out.motion,time,h,environment,liquids,targets);
        const auto left=liquidRk4(p,out.motion,time,h/2,environment,liquids,targets);
        if(!whole.valid||!left.valid)return {};
        const auto right=liquidRk4(p,left.motion,time+h/2,h/2,environment,liquids,targets);if(!right.valid)return {};
        const auto& c=liquids.controls;
        const double positionError=std::max(length(whole.motion.position0-right.motion.position0),p.segment?length(whole.motion.position1-right.motion.position1):0);
        const double velocityError=std::max(length(whole.motion.velocity0-right.motion.velocity0),p.segment?length(whole.motion.velocity1-right.motion.velocity1):0);
        const double positionScale=std::max(length(right.motion.position0-out.motion.position0),p.segment?length(right.motion.position1-out.motion.position1):0);
        const double velocityScale=std::max(length(right.motion.velocity0),p.segment?length(right.motion.velocity1):0);
        if(positionError>c.absolutePositionToleranceM+c.relativeTolerance*positionScale||velocityError>c.absoluteVelocityToleranceMs+c.relativeTolerance*velocityScale) {
            if(h/2<c.minStepS)return {};
            h/=2;continue;
        }
        out.motion=right.motion;out.airImpulse+=left.airImpulse+right.airImpulse;
        for(const auto* part:{&left,&right})for(const auto& [key,value]:part->loads){auto& load=out.loads[key];load.impulse+=value.impulse;load.angular+=value.angular;load.energy+=value.energy;}
        time+=h;h=std::min(h*2,c.maxStepS);
    }
    out.valid=finiteScene(out.motion.position0)&&finiteScene(out.motion.position1)&&finiteScene(out.motion.velocity0)&&finiteScene(out.motion.velocity1);return out;
}
double liquidHorizon(const FragmentFieldV1::Projectile& p,double duration,const SceneEnvironmentV1& environment,
    const PreparedLiquids& liquids,const std::vector<Target>& targets,bool& potential) {
    const double original=duration;potential=false;
    const double airDrag=environment.airDensity*p.cd*p.areaM2/(2*p.massKg);
    const auto free0=fragmentFlight(p.at,p.velocity,original,airDrag,environment.airVelocity,environment.gravityEcef);
    const auto free1=p.segment?fragmentFlight(p.end,p.endVelocity,original,airDrag,environment.airVelocity,environment.gravityEcef):free0;
    for(std::size_t vi=0;vi<liquids.recipients.size();++vi)if(liquids.recipients[vi]) {
        const auto& tree=*liquids.recipients[vi];const auto& target=targets[vi];
        const V3 a=toBody(target,p.at-target.at),v=toBody(target,(free0.at-p.at)*(1/original)-target.velocity),
                 b=p.segment?toBody(target,p.end-target.at):a,w=p.segment?toBody(target,(free1.at-p.end)*(1/original)-target.velocity):v;
        std::vector<std::size_t> candidates;tree.query(unite(unite({a,a},{a+v*original,a+v*original}),unite({b,b},{b+w*original,b+w*original})),candidates);
        if(!p.segment) {
            const auto owned=liquidOwnedIntervals(tree,candidates,a,a+v*original,v,v);
            if(!owned)return 0;
            if(owned->empty())continue;
            // Broadphase only: the point integrator brackets its actual
            // material-face events. A separate free-air chord clock can
            // otherwise approach the same moving face forever.
            potential=true;
            continue;
        }
        for(const auto ci:candidates) {
            const auto& cell=tree.source->cells[ci];const auto& tank=*tree.tanks.at(cell.nativeTank);if(tank.actualContentsKg==0)continue;
            auto planes=boxPlanes((cell.bodyBounds.low+cell.bodyBounds.high)*.5,(cell.bodyBounds.high-cell.bodyBounds.low)*.5);
            planes.push_back({tank.freeSurfaceBody.normal,tank.freeSurfaceBody.offset});
            const V3 d=b-a,change=w-v;const double k=dot(d,change)/dot(d,d);double time=0,along=0;V3 normal;
            if(finiteLineContact(a,v,d,k,original,planes,time,along,normal)) {
                potential=true;
                if(time>0)duration=std::min(duration,time);
                else duration=std::min(duration,liquids.controls.maxStepS);
                double exit=0;
                if(finiteLineContact(a,v,d,k,original,planes,exit,along,normal,true)&&exit>0)
                    duration=std::min(duration,exit);
            }
        }
    }
    if(potential&&(p.segment||environment.airDensity>0||length(environment.gravityEcef)>0))duration=std::min(duration,liquids.controls.maxStepS);
    return duration;
}
}

FragmentFieldV1 emitFragmentFieldV1(const Warhead& w,const Burst& burst,Rng& rng,bool connectedRod) {
    FragmentFieldV1 result;
    if(!(w.caseKg>0)||!std::isfinite(w.caseKg)||w.fragments<=0||w.fragments>1000000||
       !(w.speedMs>0)||!std::isfinite(w.speedMs)||!(w.fragmentAreaM2>0)||!std::isfinite(w.fragmentAreaM2)||
       !(w.lengthM>0)||!std::isfinite(w.lengthM)||!(w.diameterM>0)||!std::isfinite(w.diameterM)||
       !finiteScene(burst.at)||!finiteScene(burst.velocity)||!finiteScene(burst.axis)||!finiteScene(burst.angularVelocityEcef)||
       !std::isfinite(length(burst.at))||!std::isfinite(length(burst.velocity))||
       !(length(burst.axis)>0)||!std::isfinite(length(burst.axis))||!(w.fragmentCd>0)||!std::isfinite(w.fragmentCd)||
       !std::isfinite(w.tntKg)||w.tntKg<0||
       (connectedRod&&(w.kind!=Warhead::Kind::Rod||w.fragments<3)))return result;
    const V3 nose=normalized(burst.axis);
    const V3 e1=normalized(cross(nose,std::abs(nose.z)<.9?V3{0,0,1}:V3{1,0,0})),e2=cross(nose,e1);
    auto random=rng;result.caseKg=w.caseKg;result.tntKg=w.tntKg;result.burst=burst;
    result.projectiles.reserve(static_cast<std::size_t>(w.fragments));
    const double mass=w.caseKg/w.fragments,outer=w.diameterM/2;
    if(connectedRod) {
        // Homogenized connected material; total available rod length bounds
        // unfolding. Resolution is the declared equivalent rod count, not an
        // invented hardware weld count. The last endpoint is copied exactly.
        const double opening=2*kPi/w.fragments,phase=2*kPi*random.uniform();
        const double maxRadius=w.fragments*w.lengthM/(2*kPi);
        if(!(maxRadius>outer))return {};
        std::vector<V3> radial;radial.reserve(static_cast<std::size_t>(w.fragments)+1);
        for(int i=0;i<w.fragments;++i)radial.push_back(e1*std::cos(phase+i*opening)+e2*std::sin(phase+i*opening));
        radial.push_back(radial.front());
        for(std::size_t i=0;i<static_cast<std::size_t>(w.fragments);++i) {
            FragmentFieldV1::Projectile p;
            p.at=burst.at+radial[i]*outer;p.end=burst.at+radial[i+1]*outer;
            p.velocity=burst.velocity+radial[i]*w.speedMs;p.endVelocity=burst.velocity+radial[i+1]*w.speedMs;
            // Adjacent elements copy the same material vertex velocities.
            // Within each captured interval the line solver freezes its
            // orientation; that numerical approximation converges with dt.
            p.velocity+=cross(burst.angularVelocityEcef,p.at-burst.at);
            p.endVelocity+=cross(burst.angularVelocityEcef,p.end-burst.at);
            p.massKg=mass;p.areaM2=w.fragmentAreaM2;p.cd=w.fragmentCd;p.segment=true;
            p.expansionPerS=w.speedMs/outer;p.breakupAgeS=(maxRadius-outer)/w.speedMs;
            p.materialLengthM=length(p.end-p.at)*(1+p.expansionPerS*p.breakupAgeS);
            result.projectiles.push_back(p);
        }
    } else {
        double shares=0;
        if(w.zones.empty()&&w.fragmentSource!=Warhead::FragmentSource::CylindricalSideCase)return {};
        for(const auto& zone:w.zones) {
            if(!std::isfinite(zone.fromDeg)||!std::isfinite(zone.toDeg)||!std::isfinite(zone.share)||
                zone.fromDeg<0||zone.toDeg>180||zone.fromDeg>zone.toDeg||zone.share<0)return {};
            shares+=zone.share;
        }
        if(w.fragmentSource==Warhead::FragmentSource::CylindricalSideCase)shares=1;
        if(!(shares>0)||!std::isfinite(shares))return {};
        const double inner=w.caseInnerRadiusM>0?w.caseInnerRadiusM:std::max(0.0,outer-w.caseThicknessM);
        if(!(inner<outer)||!std::isfinite(inner))return {};
        for(int i=0;i<w.fragments;++i) {
            const double axial=w.lengthM*(random.uniform()-.5),phi=2*kPi*random.uniform();
            const double radius=std::sqrt(inner*inner+random.uniform()*(outer*outer-inner*inner));
            const V3 radial=e1*std::cos(phi)+e2*std::sin(phi),offset=nose*axial+radial*radius;
            V3 direction=radial;
            if(w.fragmentSource!=Warhead::FragmentSource::CylindricalSideCase) {
                const double pick=random.uniform()*shares;double sum=0;const Zone* selected=&w.zones.back();
                for(const auto& zone:w.zones){sum+=zone.share;if(pick<sum){selected=&zone;break;}}
                const double low=std::cos(selected->toDeg*kPi/180),high=std::cos(selected->fromDeg*kPi/180);
                const double cosine=low+(high-low)*random.uniform();
                direction=nose*cosine+radial*std::sqrt(std::max(0.0,1-cosine*cosine));
            }
            FragmentFieldV1::Projectile p;p.at=burst.at+offset;
            p.velocity=burst.velocity+cross(burst.angularVelocityEcef,offset)+direction*w.speedMs;
            p.massKg=mass;p.areaM2=w.fragmentAreaM2;p.cd=w.fragmentCd;
            if(w.kind==Warhead::Kind::Rod) {
                // Discrete rods occupy their full finite authored equivalent
                // length. Their material is not replaced by an endpoint ray.
                p.segment=true;p.at=burst.at+radial*radius-nose*(w.lengthM/2);p.end=burst.at+radial*radius+nose*(w.lengthM/2);
                p.velocity=burst.velocity+cross(burst.angularVelocityEcef,p.at-burst.at)+direction*w.speedMs;
                p.endVelocity=burst.velocity+cross(burst.angularVelocityEcef,p.end-burst.at)+direction*w.speedMs;
            }
            result.projectiles.push_back(p);
        }
    }
    for(const auto& particle:result.projectiles)
        if(!finiteScene(particle.at)||!finiteScene(particle.velocity)||!std::isfinite(length(particle.velocity))||
           (particle.segment&&(!finiteScene(particle.end)||!finiteScene(particle.endVelocity)||!std::isfinite(length(particle.endVelocity)))))return {};
    for(std::size_t i=0;i<result.projectiles.size();++i)result.projectiles[i].sourceOrdinalV1=static_cast<std::uint64_t>(i)+1;
    result.emittedCount=result.projectiles.size();result.valid=true;rng=random;return result;
}

static SceneAssessmentV1 assessSceneImpl(const FragmentFieldV1& field,const std::vector<MaterialSceneVictimV2>& supplied,
                              const SceneEnvironmentV1& environment,const SceneLiquidInputsV1* liquidInput,
                              std::vector<SceneLiquidOnlyWorkV2>* onlyWork=nullptr,bool localCoordinates=false) {
    if(liquidInput&&!localCoordinates) {
        // Integrate local displacement, not successive sub-ULP additions to
        // Earth-scale ECEF coordinates. One frozen translation covers every
        // source, recipient and captured motion sample; vectors and moving-
        // origin moments are unchanged. The legacy null-liquid path is intact.
        const V3 origin=field.burst.at;if(!finiteScene(origin))return {};
        auto localField=field;localField.burst.at=localField.burst.at-origin;
        for(auto& p:localField.projectiles){p.at=p.at-origin;if(p.segment)p.end=p.end-origin;}
        auto localVictims=supplied;
        for(auto& victim:localVictims){victim.target.at=victim.target.at-origin;
            for(auto& sample:victim.motion)sample.target.at=sample.target.at-origin;}
        auto result=assessSceneImpl(localField,localVictims,environment,liquidInput,onlyWork,true);
        if(!result.valid)return result;
        result.continuation.burst.at=field.burst.at;
        for(auto& p:result.continuation.projectiles){p.at+=origin;if(p.segment)p.end+=origin;}
        return result;
    }
    SceneAssessmentV1 result;
    if(!field.valid||!std::isfinite(field.caseKg)||field.caseKg<0||!std::isfinite(field.tntKg)||field.tntKg<0||
       !finiteScene(field.burst.at)||!finiteScene(field.burst.velocity)||!finiteScene(field.burst.axis)||
       !finiteScene(field.burst.angularVelocityEcef)||!std::isfinite(field.ageS)||field.ageS<0||
       !std::isfinite(environment.airDensity)||environment.airDensity<0||!finiteScene(environment.airVelocity)||
       !(environment.maxFlightTimeS>0)||!std::isfinite(environment.maxFlightTimeS)||
       !(environment.maxSubstepS>0)||!std::isfinite(environment.maxSubstepS)||
       environment.maxFlightTimeS/environment.maxSubstepS>100000||!finiteScene(environment.gravityEcef)||
       !std::isfinite(field.ageS+environment.maxFlightTimeS)||field.ageS+environment.maxFlightTimeS<=field.ageS||
       field.emittedCount>static_cast<std::size_t>(std::numeric_limits<int>::max()))return result;
    std::vector<const MaterialSceneVictimV2*> victims;
    std::vector<double> victimRadii;
    for(const auto& victim:supplied) {
        if(!victim.identity||!validSceneTarget(victim.target))return {};
        if(!victim.motion.empty()) {
            if(victim.motion.front().offsetS!=0||victim.motion.back().offsetS<environment.maxFlightTimeS)return {};
            double previous=-1;
            for(const auto& sample:victim.motion) {
                if(!std::isfinite(sample.offsetS)||sample.offsetS<=previous||!validSceneTarget(sample.target))return {};
                previous=sample.offsetS;
            }
        }
        victims.push_back(&victim);
    }
    std::sort(victims.begin(),victims.end(),[](auto a,auto b){return a->identity<b->identity;});
    for(std::size_t i=0;i<victims.size();++i) {
        if(i&&victims[i-1]->identity==victims[i]->identity)return {};
        SceneImpactV1 row;row.identity=victims[i]->identity;
        const auto relative=toBody(victims[i]->target,field.burst.at-victims[i]->target.at);
        row.impact.missM=length(relative);row.impact.fromBody=normalized(relative);
        row.impact.fragmentsThrown=static_cast<int>(field.emittedCount?field.emittedCount:field.projectiles.size());result.victims.push_back(row);
        double radius=0;
        if(victims[i]->geometry) {
        const auto& frame=victims[i]->geometry->airframe();
        for(const auto index:victims[i]->geometry->panelIndices()) {
            const auto& panel=frame.panels[index];
            if(panel.surface)for(const auto corner:panel.corners)radius=std::max(radius,length(corner)+std::abs(panel.thicknessM));
            else if(const auto* mesh=frame.contactGeometry?frame.contactGeometry->panel(index):nullptr) {
                for(const auto vertex:mesh->vertices)radius=std::max(radius,length(vertex));
            } else radius=std::max(radius,length(panel.centre)+length(panel.half));
        }
        for(const auto index:victims[i]->geometry->componentIndices()) {
            const auto& component=frame.components[index];radius=std::max(radius,length(component.centre)+length(component.half));
        }
        }
        victimRadii.push_back(radius);
    }
    PreparedLiquids liquids;
    if(liquidInput&&!prepareLiquids(*liquidInput,victims,liquids))return {};
    for(std::size_t vi=0;vi<victims.size();++vi)
        if(!victims[vi]->geometry&&(!onlyWork||!liquidInput||!liquids.recipients[vi]))return {};
    const auto recordOnly=[&](std::size_t vi,int tank,double energy,unsigned contacts) {
        auto found=std::find_if(onlyWork->begin(),onlyWork->end(),[&](const auto& row){return row.identity==victims[vi]->identity&&row.nativeTank==tank;});
        if(found==onlyWork->end()){onlyWork->push_back({victims[vi]->identity,victims[vi]->generation,tank,0,0});found=std::prev(onlyWork->end());}
        found->energyJ+=energy;found->wallContacts+=contacts;
    };
    if(liquidInput)for(std::size_t vi=0;vi<victims.size();++vi)if(liquids.recipients[vi])
        for(const auto& cell:liquids.recipients[vi]->source->cells) {
            const auto& b=cell.bodyBounds;
            victimRadii[vi]=std::max(victimRadii[vi],length({std::max(std::abs(b.low.x),std::abs(b.high.x)),
                std::max(std::abs(b.low.y),std::abs(b.high.y)),std::max(std::abs(b.low.z),std::abs(b.high.z))}));
        }
    std::map<const ContactMesh*,std::vector<SceneSkinTriangle>> skinMeshes;
    for(const auto* victim:victims) {
        if(!victim->geometry)continue;
        const auto& frame=victim->geometry->airframe();
        if(frame.contactGeometry)for(const auto index:victim->geometry->panelIndices())
            if(!frame.panels[index].surface)if(const auto* mesh=frame.contactGeometry->panel(index))
                skinMeshes.try_emplace(mesh,authoredSkinTriangles(*mesh));
    }
    double mass=0;
    for(const auto& p:field.projectiles) {
        if(!finiteScene(p.at)||!finiteScene(p.velocity)||!std::isfinite(length(p.velocity))||!(p.massKg>0)||!std::isfinite(p.massKg)||
           !(p.areaM2>0)||!std::isfinite(p.areaM2)||!(p.cd>0)||!std::isfinite(p.cd)||
           (p.segment&&(!finiteScene(p.end)||!finiteScene(p.endVelocity)||!(length(p.end-p.at)>0)||
             !std::isfinite(p.expansionPerS)||p.expansionPerS<0||!std::isfinite(p.breakupAgeS)||p.breakupAgeS<0)))return {};
        mass+=p.massKg;
    }
    if(!std::isfinite(mass)||std::abs(mass-field.caseKg)>1e-9*std::max(1.0,field.caseKg))return {};
    struct Crossing {double duration=0;std::size_t victim=0,index=0;int side=0;bool skin=false;V3 normal,point;int liquidTank=-1;};
    for(auto p:field.projectiles) {
        double age=0;bool absorbed=false,anyStrike=false;
        const double materialLength=p.materialLengthM>0?p.materialLengthM:p.segment&&p.expansionPerS>0?
            length(p.end-p.at)*(1+p.expansionPerS*p.breakupAgeS):0;
        p.materialLengthM=materialLength;
        std::vector<bool> struck(victims.size(),false);
        std::set<std::array<std::uint64_t,5>> visited(p.crossedFaces.begin(),p.crossedFaces.end());
        for(std::size_t vi=0;vi<victims.size();++vi)
            struck[vi]=std::find(p.struckVictims.begin(),p.struckVictims.end(),
                std::array<std::uint64_t,2>{victims[vi]->identity,victims[vi]->generation})!=p.struckVictims.end();
        unsigned iterations=0;
        while(age<environment.maxFlightTimeS&&!absorbed) {
            if(++iterations>200000||(liquidInput&&iterations>liquids.controls.maxSteps))return {};
            double duration=std::min(environment.maxSubstepS,environment.maxFlightTimeS-age);
            // Vacuum point/affine expansion is exact over the whole captured
            // interval. No repeated empty microsteps are needed in that case.
            if(environment.airDensity==0&&(length(environment.gravityEcef)==0||victims.empty())&&
               (!p.segment||length(cross(p.end-p.at,p.endVelocity-p.velocity))<1e-12))
                duration=environment.maxFlightTimeS-age;
            std::vector<Target> sampledTargets(victims.size());double nextBoundary=environment.maxFlightTimeS;
            for(std::size_t vi=0;vi<victims.size();++vi)
                if(!sceneTargetAt(*victims[vi],age,nextBoundary,sampledTargets[vi]))return {};
            duration=std::min(duration,nextBoundary-age);
            bool liquidFlight=false;
            if(liquidInput) {
                duration=liquidHorizon(p,duration,environment,liquids,sampledTargets,liquidFlight);
                if(!(duration>0))return {};
                // Round an event's absolute local-flight age upward, not only
                // its small duration. A positive sub-ULP duration otherwise
                // leaves age unchanged forever. This is one clock quantum,
                // bounded by the existing captured sample/window endpoint;
                // no material coordinate or integration budget is changed.
                double eventAge=age+duration;
                if(eventAge-age<duration)eventAge=std::nextafter(eventAge,std::numeric_limits<double>::infinity());
                duration=std::min(eventAge-age,std::min(nextBoundary,environment.maxFlightTimeS)-age);
            }
            if(p.segment&&p.expansionPerS>0) {
                if(length(p.end-p.at)>=materialLength*(1-1e-12)) {
                    p.velocity=p.endVelocity=(p.velocity+p.endVelocity)*.5;p.expansionPerS=0;
                    ++result.rodBreakups;
                    if(!result.firstRodBreakAgeS||field.ageS+age<*result.firstRodBreakAgeS)result.firstRodBreakAgeS=field.ageS+age;
                }
            }
            if(!(duration>0))return {};
            const double drag=environment.airDensity*p.cd*p.areaM2/(2*p.massKg);
            auto next=fragmentFlight(p.at,p.velocity,duration,drag,environment.airVelocity,environment.gravityEcef);
            auto nextEnd=p.segment?fragmentFlight(p.end,p.endVelocity,duration,drag,environment.airVelocity,environment.gravityEcef):next;
            if(liquidFlight) {
                const auto predicted=advanceLiquid(p,duration,environment,liquids,sampledTargets);if(!predicted.valid)return {};
                next={predicted.motion.position0,predicted.motion.velocity0};nextEnd=p.segment?
                    FlightInterval{predicted.motion.position1,predicted.motion.velocity1}:next;
            }
            if(!finiteScene(next.at)||!finiteScene(next.velocity)||!finiteScene(nextEnd.at)||!finiteScene(nextEnd.velocity))return {};
            if(p.segment&&p.expansionPerS>0&&length(nextEnd.at-next.at)>materialLength) {
                double lower=0,upper=duration;
                for(unsigned iteration=0;iteration<56;++iteration) {
                    const double middle=(lower+upper)/2;
                    auto a=fragmentFlight(p.at,p.velocity,middle,drag,environment.airVelocity,environment.gravityEcef);
                    auto b=fragmentFlight(p.end,p.endVelocity,middle,drag,environment.airVelocity,environment.gravityEcef);
                    if(liquidFlight) {
                        const auto step=advanceLiquid(p,middle,environment,liquids,sampledTargets);if(!step.valid)return {};
                        a={step.motion.position0,step.motion.velocity0};b={step.motion.position1,step.motion.velocity1};
                    }
                    if(length(b.at-a.at)<materialLength)lower=middle;else upper=middle;
                }
                duration=upper;
                next=fragmentFlight(p.at,p.velocity,duration,drag,environment.airVelocity,environment.gravityEcef);
                nextEnd=fragmentFlight(p.end,p.endVelocity,duration,drag,environment.airVelocity,environment.gravityEcef);
                if(liquidFlight) {
                    const auto step=advanceLiquid(p,duration,environment,liquids,sampledTargets);if(!step.valid)return {};
                    next={step.motion.position0,step.motion.velocity0};nextEnd={step.motion.position1,step.motion.velocity1};
                }
            }
            const V3 chord=(next.at-p.at)*(1/duration),endChord=(nextEnd.at-p.end)*(1/duration);
            std::optional<Crossing> first;
            bool invalidFlight=false;
            const auto consider=[&](std::size_t victim,std::size_t index,bool skin,int side,double time,V3 normal,V3 point,int liquidTank=-1) {
                if(!std::isfinite(time)||time<0||time>duration||visited.count({victims[victim]->identity,victims[victim]->generation,
                    liquidTank>=0?2u:skin?0u:1u,liquidTank>=0?static_cast<std::uint64_t>(liquidTank):index,static_cast<std::uint64_t>(side)}))return;
                if(liquidFlight&&!p.segment&&time>0&&time<duration) {
                    const auto& target=sampledTargets[victim];const V3 relativeChord=toBody(target,chord-target.velocity);
                    if(dot(relativeChord,relativeChord)>0) {
                        double lo=0,hi=duration;
                        for(unsigned iteration=0;iteration<48;++iteration) {
                            const double middle=(lo+hi)/2;const auto trial=advanceLiquid(p,middle,environment,liquids,sampledTargets);
                            if(!trial.valid){invalidFlight=true;return;}
                            const V3 body=toBody(target,trial.motion.position0-target.at-target.velocity*middle);
                            // Compare at the actual crossing plane. Subtracting
                            // two large distances from the old start can erase
                            // a finite remaining outside gap near a boundary.
                            if(dot(relativeChord,body-point)<0)lo=middle;else hi=middle;
                        }
                        time=hi; // The upper bracket has actually reached the face.
                    }
                }
                Crossing crossing{time,victim,index,side,skin,normal,point,liquidTank};
                if(!first||time<first->duration)first=crossing; // Equal ties retain sorted source order.
            };
            for(std::size_t vi=0;vi<victims.size();++vi) {
                const auto& victim=*victims[vi];const auto& target=sampledTargets[vi];
                const V3 origin=target.at;
                const V3 source=toBody(target,p.at-origin),velocity=toBody(target,chord-target.velocity);
                const double velocitySquared=dot(velocity,velocity);
                const double closestTime=velocitySquared>0?std::clamp(-dot(source,velocity)/velocitySquared,0.0,duration):0;
                const double segmentReach=p.segment?std::max(length(p.end-p.at),length(nextEnd.at-next.at)):0;
                if(length(source+velocity*closestTime)>victimRadii[vi]+segmentReach)continue;
                const double speed=length(velocity);if(!(speed>0)&&!p.segment)continue;
                const V3 direction=speed>0?velocity*(1/speed):V3{};
                if(victim.geometry) {
                const auto& frame=victim.geometry->airframe();
                const auto hit=[&](std::size_t index,bool skin) {
                    const auto* mesh=skin&&!frame.panels[index].surface&&frame.contactGeometry?
                        frame.contactGeometry->panel(index):nullptr;
                    if(mesh) {
                        const V3 d=p.segment?toBody(target,p.end-p.at):V3{};
                        const V3 change=p.segment?toBody(target,endChord-chord):V3{};
                        const double k=p.segment?dot(d,change)/dot(d,d):0;
                        const V3 centre=(mesh->low+mesh->high)*.5,half=(mesh->high-mesh->low)*.5;
                        double broad=0,leave=0,along=0;V3 ignored;
                        if(p.segment) {
                            if(!finiteLineContact(source,velocity,d,k,duration,boxPlanes(centre,half),broad,along,ignored))return;
                        } else if(!slab(source,direction,centre,half,broad,leave,ignored,ignored)||
                                  std::max(broad,0.0)>speed*duration)return;
                        for(const auto& triangle:skinMeshes.at(mesh)) {
                            double time=0;along=0;
                            if(p.segment) {
                                if(finiteLineContact(source,velocity,d,k,duration,trianglePlanes(triangle),time,along,ignored))
                                    consider(vi,index,true,triangle.face,time,triangle.normal,source+velocity*time+d*along);
                            } else {
                                const auto& v=triangle.vertices;
                                // A repeated last corner lets the established
                                // convex face query test a triangle's three edges.
                                if(quad(source,direction,{v[0],v[1],v[2],v[2]},time,ignored))
                                    consider(vi,index,true,triangle.face,time/speed,triangle.normal,source+direction*time);
                            }
                        }
                    } else if(p.segment) {
                        const V3 d=toBody(target,p.end-p.at);
                        const V3 change=toBody(target,endChord-chord);
                        const double k=dot(d,change)/dot(d,d);
                        const bool surface=skin&&frame.panels[index].surface;
                        const int faces=surface&&frame.panels[index].thicknessM>0?2:1;
                        for(int side=0;side<faces;++side) {
                            const auto planes=surface?facePlanes(surfaceFace(frame.panels[index],side)):
                                skin?boxPlanes(frame.panels[index].centre,frame.panels[index].half):
                                     boxPlanes(frame.components[index].centre,frame.components[index].half);
                            double time=0,along=0;V3 normal;
                            if(!planes.empty()&&finiteLineContact(source,velocity,d,k,duration,planes,time,along,normal,false,skin&&!surface))
                                consider(vi,index,skin,side,time,normal,source+velocity*time+d*along);
                            if(skin&&!surface&&finiteLineContact(source,velocity,d,k,duration,planes,time,along,normal,true))
                                consider(vi,index,true,1,time,normal,source+velocity*time+d*along);
                        }
                    } else if(skin) {
                        const auto& panel=frame.panels[index];double enter=0,leave=0;V3 normal,exitNormal;
                        if(panel.surface) {
                            for(int side=0;side<(panel.thicknessM>0?2:1);++side)
                                if(quad(source,direction,surfaceFace(panel,side).corners,enter,normal))
                                    consider(vi,index,true,side,enter/speed,normal,source+direction*enter);
                        } else if(slab(source,direction,panel.centre,panel.half,enter,leave,normal,exitNormal)) {
                            if(enter>=0)consider(vi,index,true,0,enter/speed,normal,source+direction*enter);
                            if(leave>=0)consider(vi,index,true,1,leave/speed,exitNormal,source+direction*leave);
                        }
                    } else {
                        double enter=0,leave=0;V3 normal,exitNormal;const auto& component=frame.components[index];
                        if(slab(source,direction,component.centre,component.half,enter,leave,normal,exitNormal)&&leave>=0) {
                            enter=std::max(enter,0.0);consider(vi,index,false,0,enter/speed,normal,source+direction*enter);
                        }
                    }
                };
                for(const auto index:victim.geometry->panelIndices())hit(index,true);
                for(const auto index:victim.geometry->componentIndices()) {
                    if(liquidInput&&liquids.recipients[vi]&&frame.components[index].kind=="fuel")continue;
                    hit(index,false);
                }
                }
                if(liquidInput&&liquids.recipients[vi]) {
                    const auto& tree=*liquids.recipients[vi];
                    const V3 endSource=p.segment?toBody(target,p.end-origin):source;
                    const V3 endVelocity=p.segment?toBody(target,endChord-target.velocity):velocity;
                    std::vector<std::size_t> candidates;
                    tree.query(unite(unite({source,source},{source+velocity*duration,source+velocity*duration}),
                        unite({endSource,endSource},{endSource+endVelocity*duration,endSource+endVelocity*duration})),candidates);
                    std::sort(candidates.begin(),candidates.end(),[&](auto a,auto b){return tree.source->cells[a].stableCellId<tree.source->cells[b].stableCellId;});
                    for(const auto ci:candidates) {
                        const auto& cell=tree.source->cells[ci];
                        if(visited.count({victim.identity,victim.generation,2u,static_cast<std::uint64_t>(cell.nativeTank),0u}))continue;
                        const auto& boundary=tree.boundary(ci);if(!tree.boundaryValid){invalidFlight=true;break;}
                        for(const auto& face:boundary) {
                            double time=0,along=0;V3 ignored;
                            const V3 normal=capacityNormal(face.axis,face.positive?1:-1);
                            if(p.segment) {
                                const V3 d=endSource-source,change=endVelocity-velocity;const double k=dot(d,change)/dot(d,d);
                                if(finiteLineContact(source,velocity,d,k,duration,capacityFacePlanes(face),time,along,ignored))
                                    consider(vi,cell.componentIndex,false,0,time,normal,source+velocity*time+d*along,cell.nativeTank);
                            } else {
                                const double rate=capacityAxis(velocity,face.axis);if(rate==0)continue;
                                time=(face.coordinate-capacityAxis(source,face.axis))/rate;
                                if(time<0||time>duration)continue;
                                V3 point=source+velocity*time;
                                if(face.axis==0)point.x=face.coordinate;else if(face.axis==1)point.y=face.coordinate;else point.z=face.coordinate;
                                const unsigned u=(face.axis+1)%3,v=(face.axis+2)%3;
                                if(capacityAxis(point,u)>=face.u0&&capacityAxis(point,u)<=face.u1&&capacityAxis(point,v)>=face.v0&&capacityAxis(point,v)<=face.v1)
                                    consider(vi,cell.componentIndex,false,0,time,normal,point,cell.nativeTank);
                            }
                        }
                    }
                }
            }
            if(invalidFlight)return {};
            const double elapsed=first?first->duration:duration;
            const V3 oldVelocity=p.segment?(p.velocity+p.endVelocity)*.5:p.velocity;
            auto actual=fragmentFlight(p.at,p.velocity,elapsed,drag,environment.airVelocity,environment.gravityEcef);
            auto actualEnd=p.segment?fragmentFlight(p.end,p.endVelocity,elapsed,drag,environment.airVelocity,environment.gravityEcef):actual;
            LiquidAdvance fluidStep;
            if(liquidFlight) {
                fluidStep=advanceLiquid(p,elapsed,environment,liquids,sampledTargets);if(!fluidStep.valid)return {};
                actual={fluidStep.motion.position0,fluidStep.motion.velocity0};actualEnd=p.segment?
                    FlightInterval{fluidStep.motion.position1,fluidStep.motion.velocity1}:actual;
            }
            // Captured affine geometry and exact bounded drag velocities share
            // the same interval endpoints; reducing maxSubstepS converges the
            // path without changing source or target criteria.
            if(liquidFlight){p.at=actual.at;if(p.segment)p.end=actualEnd.at;}
            else {p.at+=chord*elapsed;if(p.segment)p.end+=endChord*elapsed;}
            p.velocity=actual.velocity;if(p.segment)p.endVelocity=actualEnd.velocity;
            const V3 incoming=p.segment?(p.velocity+p.endVelocity)*.5:p.velocity;
            const V3 gravityImpulse=environment.gravityEcef*(p.massKg*elapsed);
            result.gravityImpulseEcef+=gravityImpulse;
            if(liquidFlight) {
                result.airMomentumEcef+=fluidStep.airImpulse;
                for(const auto& [key,load]:fluidStep.loads) {
                    if(!finiteScene(load.impulse)||!finiteScene(load.angular)||!std::isfinite(load.energy)||load.energy<0)return {};
                    auto& impact=result.victims[key.first].impact;
                    if(victims[key.first]->geometry) {
                        auto found=std::find_if(impact.components.begin(),impact.components.end(),[&](const auto& hit){return hit.component==static_cast<int>(key.second);});
                        if(found==impact.components.end()){Impact::ComponentHits hit;hit.component=static_cast<int>(key.second);impact.components.push_back(hit);found=std::prev(impact.components.end());}
                        found->energyJ+=load.energy;
                    } else recordOnly(key.first,static_cast<int>(key.second),load.energy,0);
                    result.transferredMomentumEcef+=load.impulse;
                    impact.impulseBody+=toBody(victims[key.first]->target,load.impulse);
                    impact.angularImpulseBody+=toBody(victims[key.first]->target,load.angular);
                    if(load.energy>0||length(load.impulse)>0||length(load.angular)>0) {
                        const double firstTime=field.ageS+age+fluidStep.firstInteractionS.value_or(0);
                        if(!result.firstStrikeAgeS||firstTime<*result.firstStrikeAgeS)result.firstStrikeAgeS=firstTime;
                        if(!struck[key.first]){++impact.fragmentsStriking;struck[key.first]=true;p.struckVictims.push_back({victims[key.first]->identity,victims[key.first]->generation});}
                        anyStrike=true;
                    }
                }
            } else result.airMomentumEcef+=(oldVelocity-incoming)*p.massKg+gravityImpulse;
            age+=elapsed;
            if(!first)continue;
            const auto& crossing=*first;const auto& victim=*victims[crossing.victim];const auto& target=sampledTargets[crossing.victim];
            auto& impact=result.victims[crossing.victim].impact;
            visited.insert({victim.identity,victim.generation,crossing.liquidTank>=0?2u:crossing.skin?0u:1u,
                crossing.liquidTank>=0?static_cast<std::uint64_t>(crossing.liquidTank):crossing.index,static_cast<std::uint64_t>(crossing.side)});
            const V3 centre=p.segment?(p.at+p.end)*.5:p.at;
            const V3 targetOrigin=target.at+target.velocity*elapsed;
            const V3 materialVelocity=target.velocity+cross(target.angularVelocityEcef,centre-targetOrigin);
            const V3 relative=incoming-materialVelocity;const double speed=length(relative);
            if(crossing.liquidTank>=0) {
                // Surface intersection concerns actual material, not just its
                // COM. A rotating rod may cross the bladder with zero COM
                // velocity while its endpoints retain finite material speed.
                if(!result.firstStrikeAgeS||field.ageS+age<*result.firstStrikeAgeS)result.firstStrikeAgeS=field.ageS+age;
                if(victim.geometry) {
                auto found=std::find_if(impact.components.begin(),impact.components.end(),[&](const auto& hit){return hit.component==static_cast<int>(crossing.index);});
                if(found==impact.components.end()){Impact::ComponentHits hit;hit.component=static_cast<int>(crossing.index);impact.components.push_back(hit);found=std::prev(impact.components.end());}
                const double materialSpeed=p.segment?std::max(length(p.velocity-target.velocity-cross(target.angularVelocityEcef,p.at-targetOrigin)),
                    length(p.endVelocity-target.velocity-cross(target.angularVelocityEcef,p.end-targetOrigin))):speed;
                ++found->hits;found->areaM2+=p.areaM2;found->fastestMs=std::max(found->fastestMs,materialSpeed);
                } else recordOnly(crossing.victim,crossing.liquidTank,0,1);
                if(!struck[crossing.victim]){++impact.fragmentsStriking;struck[crossing.victim]=true;p.struckVictims.push_back({victim.identity,victim.generation});}
                anyStrike=true;continue; // Negligible bladder wall: no stopping work.
            }
            if(!(speed>0))continue;
            if(!result.firstStrikeAgeS||field.ageS+age<*result.firstStrikeAgeS)result.firstStrikeAgeS=field.ageS+age;
            const V3 direction=relative*(1/speed);double after=0;
            if(crossing.skin) {
                const auto& panel=victim.geometry->airframe().panels[crossing.index];
                const double obliquity=std::acos(std::min(1.0,std::abs(dot(toBody(target,direction),crossing.normal))));
                after=thorResidual(victim.geometry->airframe().skinThicknessM(panel.section),p.areaM2,p.massKg,speed,obliquity);
                auto found=std::find_if(impact.panels.begin(),impact.panels.end(),[&](auto& hit){return hit.panel==static_cast<int>(crossing.index);});
                if(found==impact.panels.end()){Impact::PanelHits hit;hit.panel=static_cast<int>(crossing.index);impact.panels.push_back(hit);found=std::prev(impact.panels.end());}
                ++found->hits;if(after>0){++found->perforations;found->holeAreaM2+=p.areaM2;}
                found->momentumNs+=toBody(victim.target,direction)*(p.massKg*(speed-after));
            } else {
                auto found=std::find_if(impact.components.begin(),impact.components.end(),[&](auto& hit){return hit.component==static_cast<int>(crossing.index);});
                if(found==impact.components.end()){Impact::ComponentHits hit;hit.component=static_cast<int>(crossing.index);impact.components.push_back(hit);found=std::prev(impact.components.end());}
                const V3 endRelative=p.segment?p.endVelocity-p.velocity-cross(target.angularVelocityEcef,p.end-p.at):V3{};
                ++found->hits;found->energyJ+=.5*p.massKg*speed*speed+p.massKg*dot(endRelative,endRelative)/24;
                found->fastestMs=std::max(found->fastestMs,speed);found->areaM2+=p.areaM2;
            }
            const V3 impulse=direction*(p.massKg*(speed-after));
            result.transferredMomentumEcef+=impulse;const V3 bodyImpulse=toBody(victim.target,impulse);
            const V3 materialAxis=p.segment?p.end-p.at:V3{};
            const V3 retainedSpin=after<=0?cross(target.angularVelocityEcef,materialAxis):V3{};
            const V3 intrinsic=p.segment?cross(materialAxis,p.endVelocity-p.velocity-retainedSpin)*(p.massKg/12):V3{};
            // Spin is driven by the material lever at this actual crossing,
            // not by the target's translation since the scene began. Reduce
            // the moving-origin moment into the original reference body axes.
            const V3 angular=cross(centre-targetOrigin,impulse)+intrinsic;
            impact.impulseBody+=bodyImpulse;impact.angularImpulseBody+=toBody(victim.target,angular);
            if(!struck[crossing.victim]) {
                ++impact.fragmentsStriking;struck[crossing.victim]=true;
                p.struckVictims.push_back({victim.identity,victim.generation});
            }
            anyStrike=true;
            if(after<=0){absorbed=true;result.absorbedMassKg+=p.massKg;result.retainedMomentumEcef+=materialVelocity*p.massKg;}
            else {
                p.velocity=materialVelocity+direction*after;
                // A hit locally breaks a connected element; surviving material
                // flies as a constant-length finite rod, without elastic energy
                // or a second copy of the original projectile.
                if(p.segment){p.endVelocity=p.velocity;p.expansionPerS=0;p.breakupAgeS=0;}
            }
        }
        if(anyStrike)++result.projectilesStriking;
        if(!absorbed) {
            result.remainingMassKg+=p.massKg;result.remainingMomentumEcef+=(p.segment?(p.velocity+p.endVelocity)*.5:p.velocity)*p.massKg;
            p.crossedFaces.assign(visited.begin(),visited.end());result.continuation.projectiles.push_back(std::move(p));
        }
    }
    // Preserve the established blast fit without resampling a fragment field.
    for(std::size_t vi=0;field.applyBlast&&vi<victims.size();++vi) {
        const auto& victim=*victims[vi];if(!victim.geometry)continue;
        const auto& frame=victim.geometry->airframe();auto& impact=result.victims[vi].impact;
        const V3 burst=toBody(victim.target,field.burst.at-victim.target.at);
        for(const auto index:victim.geometry->panelIndices()) {
            const auto& panel=frame.panels[index];const V3 to=burst-panel.middle();
            const auto blast=blastAt(field.tntKg,length(to),victim.target.airPressurePa,victim.target.airTemperatureK);
            if(blast.overpressurePa<kBlastSeenPa)continue;
            double facing=1;
            if(panel.surface)facing=std::abs(dot(normalized(cross(panel.corners[2]-panel.corners[0],panel.corners[3]-panel.corners[1])),normalized(to)));
            const double peak=blast.overpressurePa+(reflected(blast.overpressurePa,victim.target.airPressurePa)-blast.overpressurePa)*facing*facing;
            const double impulse=blast.overpressurePa>0?blast.impulsePas*peak/blast.overpressurePa:0;
            auto found=std::find_if(impact.panels.begin(),impact.panels.end(),[&](auto& hit){return hit.panel==static_cast<int>(index);});
            if(found==impact.panels.end()){Impact::PanelHits hit;hit.panel=static_cast<int>(index);impact.panels.push_back(hit);found=std::prev(impact.panels.end());}
            found->overpressurePa=peak;found->impulsePas=impulse;
            const V3 kick=panelBlastImpulse(panel,burst,impulse);impact.impulseBody+=kick;impact.angularImpulseBody+=cross(panel.middle(),kick);
        }
        std::sort(impact.panels.begin(),impact.panels.end(),[](const auto& a,const auto& b){return a.panel<b.panel;});
        std::sort(impact.components.begin(),impact.components.end(),[](const auto& a,const auto& b){return a.component<b.component;});
    }
    result.continuation.valid=true;result.continuation.caseKg=result.remainingMassKg;
    result.continuation.tntKg=field.tntKg;result.continuation.burst=field.burst;
    result.continuation.ageS=field.ageS+environment.maxFlightTimeS;
    result.continuation.emittedCount=field.emittedCount?field.emittedCount:field.projectiles.size();result.continuation.applyBlast=false;
    result.valid=true;return result;
}

SceneAssessmentV1 assessSceneV1(const FragmentFieldV1& field,const std::vector<SceneVictimV1>& victims,const SceneEnvironmentV1& environment) {
    std::vector<MaterialSceneVictimV2> adapted;adapted.reserve(victims.size());
    for(const auto& v:victims)adapted.push_back({v.identity,v.generation,&v.geometry,v.target,v.motion});
    return assessSceneImpl(field,adapted,environment,nullptr);
}
SceneAssessmentV1 assessSceneLiquidsV1(const FragmentFieldV1& field,const std::vector<SceneVictimV1>& victims,
    const SceneEnvironmentV1& environment,const SceneLiquidInputsV1& liquids) {
    std::vector<MaterialSceneVictimV2> adapted;adapted.reserve(victims.size());
    for(const auto& v:victims)adapted.push_back({v.identity,v.generation,&v.geometry,v.target,v.motion});
    return assessSceneImpl(field,adapted,environment,&liquids);
}

SceneLiquidAssessmentV2 assessSceneLiquidsV2(const FragmentFieldV1& field,const std::vector<SceneVictimV1>& victims,
    const std::vector<SceneLiquidOnlyVictimV2>& only,const SceneEnvironmentV1& environment,const SceneLiquidInputsV1& liquids) {
    std::vector<MaterialSceneVictimV2> adapted;adapted.reserve(victims.size()+only.size());
    for(const auto& v:victims)adapted.push_back({v.identity,v.generation,&v.geometry,v.target,v.motion});
    for(const auto& v:only)adapted.push_back({v.identity,v.generation,nullptr,v.target,v.motion});
    SceneLiquidAssessmentV2 result;
    result.scene=assessSceneImpl(field,adapted,environment,&liquids,&result.liquidOnlyWork);
    if(!result.scene.valid)return {};
    for(const auto& row:result.liquidOnlyWork)if(!std::isfinite(row.energyJ)||row.energyJ<0)return {};
    std::sort(result.liquidOnlyWork.begin(),result.liquidOnlyWork.end(),[](const auto& a,const auto& b){
        return a.identity<b.identity||(a.identity==b.identity&&a.nativeTank<b.nativeTank);});
    return result;
}

} // namespace fsim::ms::damage
