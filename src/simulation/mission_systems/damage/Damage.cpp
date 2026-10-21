#include "simulation/mission_systems/damage/Damage.h"

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

} // namespace fsim::ms::damage
