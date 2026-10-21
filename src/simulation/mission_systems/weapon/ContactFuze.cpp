#include "ContactFuze.h"

#include <algorithm>
#include <cmath>

namespace fsim::ms::weapon {
namespace {

// Use extra precision for predicates/intersections, without a physical epsilon
// that would enlarge a finite face or close a genuine gap. Non-finite derived
// arithmetic rejects the sweep on platforms where long double has no extra range.
using Real = long double;
struct Point {
    Real x, y, z;
};
Point point(damage::V3 v) { return {v.x, v.y, v.z}; }
Point subtract(Point a, Point b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Point cross(Point a, Point b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Real dot(Point a, Point b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
bool finite(damage::V3 p) { return finite(point(p)); }

struct Intersection {
    bool valid = true;
    bool hit = false;
    Real fraction = 0;
    Point at{}, normal{};
    bool ownershipCap=false;
};

Intersection bounds(Point p, Point q, damage::V3 lowCorner, damage::V3 highCorner) {
    const Point lowPoint=point(lowCorner), highPoint=point(highCorner), d = subtract(q, p);
    const Real start[] = {p.x, p.y, p.z}, movement[] = {d.x, d.y, d.z};
    const Real lower[] = {lowPoint.x,lowPoint.y,lowPoint.z}, upper[] = {highPoint.x,highPoint.y,highPoint.z};
    if (!finite(d)) return {false};
    Real first = 0, last = 1;
    for (int axis = 0; axis < 3; ++axis) {
        const Real low = lower[axis], high = upper[axis];
        if (!std::isfinite(low) || !std::isfinite(high)) return {false};
        if (movement[axis] == 0) {
            if (start[axis] < low || start[axis] > high) return {};
            continue;
        }
        Real a = (low - start[axis]) / movement[axis];
        Real b = (high - start[axis]) / movement[axis];
        if (!std::isfinite(a) || !std::isfinite(b)) return {false};
        if (a > b) std::swap(a, b);
        first = std::max(first, a);
        last = std::min(last, b);
        if (first > last) return {};
    }
    return {true, true, first};
}

Intersection triangle(Point p, Point q, Point a, Point b, Point c) {
    const Point ab = subtract(b, a), ac = subtract(c, a), n = cross(ab, ac);
    const Real square = dot(n, n);
    if (!finite(ab) || !finite(ac) || !finite(n) || !std::isfinite(square)) return {false};
    if (square == 0) return {}; // A collapsed triangle represents no finite face.
    const Real before = dot(n, subtract(p, a)), after = dot(n, subtract(q, a));
    if (!std::isfinite(before) || !std::isfinite(after)) return {false};
    const Point vertices[] = {a, b, c};
    const auto movement=subtract(q,p);
    if(!finite(movement)) return {false};
    const Real norm=std::hypot(n.x,n.y,n.z);
    if(!(norm>0) || !std::isfinite(norm)) return {false};
    Point normal{n.x/norm,n.y/norm,n.z/norm};
    const Real closing=dot(normal,movement);
    if(!std::isfinite(closing)) return {false};
    if(closing>0) normal={-normal.x,-normal.y,-normal.z};
    const auto contact=[&](Real f) -> Intersection {
        const Point at{p.x+f*movement.x,p.y+f*movement.y,p.z+f*movement.z};
        if(!finite(at)) return {false};
        return {true,true,f,at,normal,false};
    };
    if (before == 0 && after == 0) {
        // A coplanar segment enters the triangle through an edge: clip it by
        // the three oriented half planes. Boundaries remain inclusive.
        Real first = 0, last = 1;
        for (int edge = 0; edge < 3; ++edge) {
            const Point v = vertices[edge], e = subtract(vertices[(edge + 1) % 3], v);
            const Real h0 = dot(n, cross(e, subtract(p, v)));
            const Real h1 = dot(n, cross(e, subtract(q, v)));
            if (!std::isfinite(h0) || !std::isfinite(h1)) return {false};
            if (h0 < 0 && h1 < 0) return {};
            if (h0 < 0 || h1 < 0) {
                const Real change = h1 - h0;
                const Real f = -h0 / change;
                if (!std::isfinite(change) || !std::isfinite(f)) return {false};
                if (h0 < 0) first = std::max(first, f);
                else last = std::min(last, f);
                if (first > last) return {};
            }
        }
        return contact(first);
    }
    if ((before < 0 && after < 0) || (before > 0 && after > 0)) return {};
    const Real denominator = before - after;
    const Real f = before / denominator;
    if (!std::isfinite(denominator) || !std::isfinite(f)) return {false};
    if (f < 0 || f > 1) return {};
    const Point d = subtract(q, p);
    const Point at{p.x + f * d.x, p.y + f * d.y, p.z + f * d.z};
    if (!finite(d) || !finite(at)) return {false};
    for (int edge = 0; edge < 3; ++edge) {
        const Point v = vertices[edge], e = subtract(vertices[(edge + 1) % 3], v);
        const Real side = dot(n, cross(e, subtract(at, v)));
        if (!std::isfinite(side)) return {false};
        if (side < 0) return {};
    }
    return contact(f);
}

Intersection panel(Point p, Point q, const damage::Airframe::Panel& face) {
    const auto a = triangle(p, q, point(face.corners[0]), point(face.corners[1]), point(face.corners[2]));
    const auto b = triangle(p, q, point(face.corners[0]), point(face.corners[2]), point(face.corners[3]));
    if (!a.valid || !b.valid) return {false};
    if (!a.hit) return b;
    if (!b.hit || a.fraction <= b.fraction) return a;
    return b;
}

Intersection meshCrossing(Point p,Point q,const damage::ContactMesh& mesh,const damage::AttachedGeometryView& geometry) {
    const auto broad=bounds(p,q,mesh.low,mesh.high);
    if (!broad.valid || !broad.hit) return broad;
    Intersection first;
    for (std::size_t i=0;i<mesh.triangles.size();++i) {
        const auto& neighbor=mesh.triangleNeighbors[i];
        if(neighbor) {
            const auto& owners=neighbor->kind==damage::ContactMesh::Kind::Panel ? geometry.panelIndices() : geometry.componentIndices();
            if(std::binary_search(owners.begin(),owners.end(),neighbor->index)) continue;
        }
        const auto& indices=mesh.triangles[i];
        auto hit=triangle(p,q,point(mesh.vertices[indices[0]]),point(mesh.vertices[indices[1]]),point(mesh.vertices[indices[2]]));
        if (!hit.valid) return {false};
        hit.ownershipCap=neighbor.has_value();
        if (hit.hit && (!first.hit || hit.fraction<first.fraction ||
            (hit.fraction==first.fraction && first.ownershipCap && !hit.ownershipCap))) first=hit;
    }
    return first; // A broad-phase overlap alone never authorizes contact.
}

Intersection meshContains(Point p,const damage::ContactMesh& mesh) {
    const auto broad=bounds(p,p,mesh.low,mesh.high);
    if (!broad.valid || !broad.hit) return broad;
    Real angle=0;
    for (const auto& indices : mesh.triangles) {
        const Point vertices[]={point(mesh.vertices[indices[0]]),point(mesh.vertices[indices[1]]),point(mesh.vertices[indices[2]])};
        const auto boundary=triangle(p,p,vertices[0],vertices[1],vertices[2]);
        if (!boundary.valid || boundary.hit) return boundary;
        Point directions[3];
        for (int k=0;k<3;++k) {
            const auto d=subtract(vertices[k],p);
            const Real norm=std::hypot(d.x,d.y,d.z);
            if (!(norm>0) || !std::isfinite(norm)) return {false};
            directions[k]={d.x/norm,d.y/norm,d.z/norm};
        }
        const Real numerator=dot(directions[0],cross(directions[1],directions[2]));
        const Real denominator=1+dot(directions[0],directions[1])+dot(directions[1],directions[2])+dot(directions[2],directions[0]);
        angle+=2*std::atan2(numerator,denominator);
        if (!std::isfinite(angle)) return {false};
    }
    // A consistently wound closed surface has winding 0 outside and +/-1
    // inside. This topological threshold adds no distance/radius tolerance.
    constexpr Real twoPi=6.283185307179586476925286766559005768L;
    return {true,std::abs(angle)>twoPi,0};
}

bool validGeometry(const damage::AttachedGeometryView& geometry) {
    const auto& source = geometry.airframe();
    if (!source.contactGeometry) return false;
    for (const auto index : geometry.panelIndices()) {
        if (index >= source.panels.size()) return false;
        const auto& face = source.panels[index];
        if (face.surface) {
            for (const auto corner : face.corners) if (!finite(corner)) return false;
        }
    }
    for (const auto index : geometry.componentIndices()) {
        if (index >= source.components.size()) return false;
    }
    return true;
}

bool validInputs(const std::vector<ContactSample>& path, const ContactWindow& w) {
    if (!std::isfinite(w.ageBeforeS) || w.ageBeforeS < 0 ||
        !std::isfinite(w.durationS) || w.durationS <= 0 ||
        !std::isfinite(w.armingS) || w.armingS < 0 ||
        !std::isfinite(w.lifeS) || w.lifeS <= w.armingS ||
        !std::isfinite(w.cutoffS) || w.cutoffS < 0 || w.cutoffS > w.durationS ||
        !std::isfinite(w.ageBeforeS + w.durationS) ||
        path.size() < 2 || path.front().offsetS != 0) return false;
    double previous = -1;
    for (const auto& sample : path) {
        if (!sample.available || !std::isfinite(sample.offsetS) || sample.offsetS <= previous ||
            sample.offsetS > w.durationS || !finite(sample.pointBody)) return false;
        previous = sample.offsetS;
    }
    return true;
}

struct Candidate {
    bool hit = false;
    Real offset = 0;
    ContactKind kind = ContactKind::Panel;
    std::size_t index = 0;
    Point at{}, normal{};
    bool ownershipCap=false;
};
bool earlier(const Candidate& a, const Candidate& b) {
    if (!b.hit || a.offset < b.offset) return true;
    if (a.offset > b.offset) return false;
    if (a.ownershipCap!=b.ownershipCap) return !a.ownershipCap;
    if (a.kind != b.kind) return a.kind == ContactKind::Panel;
    return a.index < b.index;
}

} // namespace

ContactFuzeResult contactFuze(const std::vector<ContactSample>& path,
                              const damage::AttachedGeometryView& geometry,
                              const ContactWindow& window) {
    ContactFuzeResult result;
    if (!validInputs(path, window) || !validGeometry(geometry)) return result;
    result.valid = true;
    const auto& source = geometry.airframe();
    const Point initial = point(path.front().pointBody);
    for (const auto index : geometry.panelIndices()) {
        const auto* mesh=source.contactGeometry->panel(index);
        const auto overlap = source.panels[index].surface ? panel(initial,initial,source.panels[index]) :
                             mesh ? meshContains(initial,*mesh) : Intersection{};
        if (!overlap.valid) return {};
        result.initialOverlap = result.initialOverlap || overlap.hit;
    }
    for (const auto index : geometry.componentIndices()) {
        const auto* mesh=source.contactGeometry->component(index);
        const auto overlap = mesh ? meshContains(initial,*mesh) : Intersection{};
        if (!overlap.valid) return {};
        result.initialOverlap = result.initialOverlap || overlap.hit;
    }
    if (result.initialOverlap) return result;

    const auto eligible = [&](Real offset) {
        return offset >= 0 && offset <= static_cast<Real>(window.cutoffS) &&
            (!window.exclusiveCutoff || offset < static_cast<Real>(window.cutoffS)) &&
            static_cast<Real>(window.ageBeforeS) + offset < static_cast<Real>(window.lifeS);
    };
    Candidate first;
    for (std::size_t segment = 1; segment < path.size(); ++segment) {
        const auto& a = path[segment - 1];
        const auto& b = path[segment];
        if (a.offsetS > window.cutoffS ||
            static_cast<Real>(window.ageBeforeS) + a.offsetS >= window.lifeS) break;
        const Point p = point(a.pointBody), q = point(b.pointBody);
        const auto consider = [&](const Intersection& hit, ContactKind kind, std::size_t index) {
            if (!hit.hit) return;
            const Real offset = static_cast<Real>(a.offsetS) + hit.fraction *
                (static_cast<Real>(b.offsetS) - a.offsetS);
            const Candidate candidate{true, offset, kind, index,hit.at,hit.normal,hit.ownershipCap};
            if (eligible(offset) && earlier(candidate, first)) first = candidate;
        };
        for (const auto index : geometry.panelIndices()) {
            const auto* mesh=source.contactGeometry->panel(index);
            const auto hit = source.panels[index].surface ? panel(p,q,source.panels[index]) :
                             mesh ? meshCrossing(p,q,*mesh,geometry) : Intersection{};
            if (!hit.valid) return {};
            consider(hit, ContactKind::Panel, index);
        }
        for (const auto index : geometry.componentIndices()) {
            const auto* mesh=source.contactGeometry->component(index);
            const auto hit = mesh ? meshCrossing(p,q,*mesh,geometry) : Intersection{};
            if (!hit.valid) return {};
            consider(hit, ContactKind::Component, index);
        }
        // Include a following segment beginning at this exact event time so
        // authored skin can win a shared-endpoint tie against an exposed cut.
        if (first.hit && first.offset<static_cast<Real>(b.offsetS)) break;
    }
    if (!first.hit) return result;
    const double offset = static_cast<double>(first.offset);
    const double fraction = static_cast<double>(first.offset / window.durationS);
    // Rounded returned event coordinates must still belong to the admitted time
    // interval. If an exclusive boundary cannot be represented, fail closed.
    if (!std::isfinite(offset) || !std::isfinite(fraction) ||
        !eligible(offset) || !eligible(fraction * window.durationS)) return {};
    result.contact = true;
    result.triggered = static_cast<Real>(window.ageBeforeS) + first.offset >= window.armingS;
    result.offsetS = offset;
    result.fraction = fraction;
    result.kind = first.kind;
    result.index = first.index;
    result.pointBody={static_cast<double>(first.at.x),static_cast<double>(first.at.y),static_cast<double>(first.at.z)};
    result.normalBody={static_cast<double>(first.normal.x),static_cast<double>(first.normal.y),static_cast<double>(first.normal.z)};
    if(!finite(result.pointBody) || !finite(result.normalBody)) return {};
    return result;
}

bool ContactFuzeState::commit(const ContactFuzeResult& crossing) noexcept {
    if (consumed_ || !crossing.valid || !crossing.contact) return false;
    consumed_ = true;
    return crossing.triggered;
}

} // namespace fsim::ms::weapon
