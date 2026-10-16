#include "simulation/mission_systems/damage/State.h"

#include <algorithm>

namespace fsim::ms::damage {

namespace {
constexpr double kG = 9.80665;
constexpr double kAluminium = 2780.0;
} // namespace

double tearImpulse(double t) noexcept { return t * std::sqrt(2.0 * kAluminium * kPanelYieldPa * kPanelElongation * kPanelStraining); }

VehicleDamage::VehicleDamage(std::shared_ptr<const Airframe> airframe) : airframe_(std::move(airframe)) {
    const Airframe& a = *airframe_;
    sections_.resize(a.sections.size());
    torn_.assign(a.panels.size(), false);
    span_.assign(a.sections.size(), 1.0);
    for (const auto& p : a.panels) {
        if (p.section < 0) continue;
        const auto& s = a.sections[static_cast<std::size_t>(p.section)];
        if (!s.hasRoot) continue;
        auto& reach = span_[static_cast<std::size_t>(p.section)];
        reach = std::max(reach, length(p.middle() - s.root));
    }
}

bool VehicleDamage::engineOut(int engine) const noexcept {
    return std::find(engines_out_.begin(), engines_out_.end(), engine) != engines_out_.end();
}

void VehicleDamage::weaken(int s, const V3& where, double removedM2) {
    if (s < 0) return;
    const auto& sec = airframe_->sections[static_cast<std::size_t>(s)];
    auto& st = sections_[static_cast<std::size_t>(s)];
    if (!sec.hasRoot || !(sec.skinAreaM2 > 0.0)) return;
    // the nearer its root, the more of its strength it takes: twice the mean at the root, none at its far end - a
    // section's skin removed evenly leaves (1 - f) of its net section
    const double w = 2.0 * std::max(0.0, 1.0 - length(where - sec.root) / span_[static_cast<std::size_t>(s)]);
    st.removed += w * removedM2 / sec.skinAreaM2;
    st.strength = std::pow(std::max(0.0, 1.0 - std::min(st.removed, 1.0)), kCompressionExponent);
}

std::vector<Change> VehicleDamage::apply(const Impact& impact, double airPressurePa, Rng& rng) {
    std::vector<Change> out;
    const Airframe& a = *airframe_;
    const double thin = std::sqrt(std::clamp(airPressurePa / 101325.0, 0.0, 1.0)); // (fires catch less in thin air: an estimate)
    for (const auto& hit : impact.components) {
        const auto& c = a.components[static_cast<std::size_t>(hit.component)];
        const double each = hit.hits > 0 ? hit.energyJ / hit.hits : 0.0;
        if (c.kind == "fuel" && c.tank >= 0) {
            auto& t = tanks_[c.tank];
            if (t.component < 0) t.component = hit.component;
            t.holeM2 += hit.areaM2 * (hit.fastestMs > kRamSpeedMs ? kRamFactor : 1.0);
            if (!t.burning)
                for (int k = 0; k < hit.hits; ++k)
                    if (rng.uniform() < 1.0 - std::exp(-each * thin / kIgnitionJ)) {
                        t.burning = true;
                        Change ch;
                        ch.kind = Change::Kind::Fire, ch.component = hit.component, ch.tank = c.tank;
                        ch.spillKg = std::min(kSpillShare * c.fuelKg, kSpillMostKg);
                        out.push_back(ch);
                        break;
                    }
        } else if (c.kind == "engine" && c.engine >= 0 && !engineOut(c.engine)) {
            for (int k = 0; k < hit.hits; ++k)
                if (rng.uniform() < kEngineKill) {
                    engines_out_.push_back(c.engine);
                    Change ch;
                    ch.kind = Change::Kind::EngineOut, ch.component = hit.component, ch.engine = c.engine;
                    out.push_back(ch);
                    if (rng.uniform() < kEngineFire) {
                        Change fire = ch;
                        fire.kind = Change::Kind::Fire;
                        out.push_back(fire);
                    }
                    break;
                }
        } else if (c.kind == "control" && !c.channel.empty() &&
                   std::find(lostChannels_.begin(), lostChannels_.end(), c.channel) == lostChannels_.end()) {
            for (int k = 0; k < hit.hits; ++k)
                if (rng.uniform() < kControlKill) {
                    lostChannels_.push_back(c.channel);
                    Change ch;
                    ch.kind = Change::Kind::ControlLost, ch.component = hit.component, ch.channel = c.channel;
                    out.push_back(ch);
                    break;
                }
        } else if (c.kind == "crew" && !crewLost_ && hit.fastestMs > kCrewSpeedMs) {
            for (int k = 0; k < hit.hits; ++k)
                if (rng.uniform() < kCrewKill) {
                    crewLost_ = true;
                    Change ch;
                    ch.kind = Change::Kind::CrewLost, ch.component = hit.component;
                    out.push_back(ch);
                    break;
                }
        }
    }
    // the skin: holed where fragments went through, torn where the blast's impulse is more than it can take
    std::vector<double> surface(a.sections.size(), 0.0);
    for (const auto& p : a.panels)
        if (p.surface && p.section >= 0) surface[static_cast<std::size_t>(p.section)] += 0.5 * p.areaM2;
    for (const auto& h : impact.panels) {
        const auto& p = a.panels[static_cast<std::size_t>(h.panel)];
        if (p.section < 0) continue;
        auto& st = sections_[static_cast<std::size_t>(p.section)];
        if (h.holeAreaM2 > 0.0) {
            st.holeM2 += h.holeAreaM2;
            // (its holes and the cracks linking them: what of the panel is lost)
            const double face = 0.5 * p.areaM2;
            weaken(p.section, p.middle(), face * (1.0 - std::exp(-kCrackLink * h.holeAreaM2 / std::max(face, 1e-6))));
        }
        if (!torn_[static_cast<std::size_t>(h.panel)] && h.impulsePas > tearImpulse(a.skinThicknessM(p.section))) {
            torn_[static_cast<std::size_t>(h.panel)] = true;
            const double face = 0.5 * p.areaM2; // (the face toward the burst)
            st.tornM2 += face;
            weaken(p.section, p.middle(), face);
            if (p.surface && surface[static_cast<std::size_t>(p.section)] > 0.0)
                st.lift = std::max(0.0, st.lift - face / surface[static_cast<std::size_t>(p.section)]);
            Change ch;
            ch.kind = Change::Kind::PanelTorn, ch.panel = h.panel, ch.section = p.section;
            out.push_back(ch);
        }
    }
    // the blast's impulse on each section's skin, its root's bending mode struck: the moment it peaks at, against
    // what is left of its strength (now weakened) - the close burst's structural kill
    const V3 burst = impact.fromBody * impact.missM;
    std::vector<V3> kick(a.sections.size());
    for (const auto& h : impact.panels) {
        const auto& p = a.panels[static_cast<std::size_t>(h.panel)];
        if (p.section < 0 || (!(h.impulsePas > 0.0) && h.hits == 0)) continue;
        const auto& sec = a.sections[static_cast<std::size_t>(p.section)];
        if (!sec.hasRoot) continue;
        const V3 m = p.middle();
        const V3 away = normalized(m - burst);
        double face = 0.5 * p.areaM2;
        if (p.surface) {
            const V3 n = normalized(cross(p.corners[2] - p.corners[0], p.corners[3] - p.corners[1]));
            face *= std::abs(dot(n, away)); // (its face as the shock meets it)
        }
        kick[static_cast<std::size_t>(p.section)] += cross(m - sec.root, away * (h.impulsePas * face) + h.momentumNs);
    }
    for (std::size_t s = 0; s < a.sections.size(); ++s) {
        const auto& sec = a.sections[s];
        auto& st = sections_[s];
        if (!sec.hasRoot || st.separated) continue;
        const double omega = std::sqrt(a.nUltimate[1] * 9.80665 / (kTipDeflection * std::max(span_[s], 0.5)));
        st.kick = 0.0;
        for (std::size_t k = 0; k < sec.axes.size() && k < sec.strengthNm.size(); ++k)
            st.kick = std::max(st.kick, std::abs(dot(kick[s], sec.axes[k])) * omega / std::max(sec.strengthNm[k] * st.strength, 1e-9));
        for (std::size_t k = 0; k < sec.axes.size() && k < sec.strengthNm.size(); ++k)
            if (std::abs(dot(kick[s], sec.axes[k])) * omega > sec.strengthNm[k] * st.strength) {
                st.separated = true;
                st.lift = 0.0;
                Change ch;
                ch.kind = Change::Kind::Separated, ch.section = static_cast<int>(s);
                out.push_back(ch);
                break;
            }
    }
    return out;
}

double VehicleDamage::leakKgS(int tank, double contentsKg, double loadFactor, double q) const {
    const auto it = tanks_.find(tank);
    if (it == tanks_.end() || !(it->second.holeM2 > 0.0) || !(contentsKg > 0.0)) return 0.0;
    const double dp = kFuelDensity * kG * std::max(loadFactor, 0.3) * kHeadM + kSuctionCp * std::max(q, 0.0);
    return kHoleCd * it->second.holeM2 * std::sqrt(2.0 * kFuelDensity * dp);
}

void VehicleDamage::aeroLoss(const Flight& f, V3& force, V3& moment) const {
    force = {}, moment = {};
    const Airframe& a = *airframe_;
    double damagedM2 = 0.0;
    for (std::size_t s = 0; s < a.sections.size(); ++s) {
        const auto& sec = a.sections[s];
        const auto& st = sections_[s];
        const double lost = st.separated ? 1.0 : 1.0 - st.lift;
        if (!st.separated) damagedM2 += st.tornM2 + st.holeM2;
        if (!(lost > 0.0)) continue;
        if (sec.normalCp) {
            const V3 dF{0.0, 0.0, -lost * sec.normalShare * f.aeroForceN.z};
            force += dF;
            moment += cross(*sec.normalCp, dF);
        }
        if (sec.sideCp) {
            const V3 dF{0.0, -lost * sec.sideShare * f.aeroForceN.y, 0.0};
            force += dF;
            moment += cross(*sec.sideCp, dF);
        }
    }
    // (the torn and holed skin's drag, along the body's axis: an estimate's direction too)
    force += V3{-kDamagedCd * f.dynamicPressurePa * damagedM2, 0.0, 0.0};
}

std::vector<double> VehicleDamage::bending(int s, const Flight& f) const {
    const Airframe& a = *airframe_;
    const auto& sec = a.sections[static_cast<std::size_t>(s)];
    const auto& st = sections_[static_cast<std::size_t>(s)];
    std::vector<double> out(sec.axes.size(), 0.0);
    if (!sec.hasRoot) return out;
    V3 M;
    const double lift = st.lift;
    if (sec.normalCp) M += cross(*sec.normalCp - sec.root, V3{0.0, 0.0, lift * sec.normalShare * f.aeroForceN.z});
    if (sec.sideCp) M += cross(*sec.sideCp - sec.root, V3{0.0, lift * sec.sideShare * f.aeroForceN.y, 0.0});
    // its own mass and its tanks' fuel: what the structure must hold against the CG's acceleration and the rotation's
    auto inertia = [&](double kg, const V3& r) {
        // (its acceleration less gravity: the specific force at the CG and the rotation's about it)
        const V3 sf = f.specificForceMs2 + cross(f.rateDotRadS2, r) + cross(f.rateRadS, cross(f.rateRadS, r));
        M += cross(r - sec.root, sf * (-kg));
    };
    inertia(sec.massKg, sec.cg);
    for (const auto& t : sec.tanks)
        if (t.index >= 0 && t.index < static_cast<int>(f.tankKg.size())) inertia(f.tankKg[static_cast<std::size_t>(t.index)], t.centre);
    for (std::size_t k = 0; k < sec.axes.size(); ++k) out[k] = dot(M, sec.axes[k]);
    return out;
}

bool VehicleDamage::separate(int s) {
    if (s < 0 || s >= static_cast<int>(sections_.size())) return false;
    auto& st = sections_[static_cast<std::size_t>(s)];
    if (st.separated || !airframe_->sections[static_cast<std::size_t>(s)].hasRoot) return false;
    st.separated = true;
    st.lift = 0.0;
    return true;
}

std::vector<Change> VehicleDamage::loads(const Flight& f) {
    std::vector<Change> out;
    const Airframe& a = *airframe_;
    for (std::size_t s = 0; s < a.sections.size(); ++s) {
        auto& st = sections_[s];
        const auto& sec = a.sections[s];
        if (st.separated || !sec.hasRoot) continue;
        const auto M = bending(static_cast<int>(s), f);
        for (std::size_t k = 0; k < M.size() && k < sec.strengthNm.size(); ++k)
            if (std::abs(M[k]) > sec.strengthNm[k] * st.strength) {
                st.separated = true;
                st.lift = 0.0;
                Change ch;
                ch.kind = Change::Kind::Separated, ch.section = static_cast<int>(s);
                out.push_back(ch);
                break;
            }
    }
    return out;
}

} // namespace fsim::ms::damage
