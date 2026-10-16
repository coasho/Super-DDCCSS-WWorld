#include "simulation/mission_systems/track/Tracker.h"

#include "simulation/mission_systems/track/Assignment.h"

#include <algorithm>
#include <limits>
#include <numeric>

namespace fsim::ms::track {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// d^T S^-1 d (S symmetric positive definite: its inverse is Stone Soup's pinv).
double squaredMahalanobis(const Meas& d, const MeasCovar& s) noexcept {
    MeasCovar inv;
    if (!inverse(s, inv)) return kInf;
    return (transpose(d) * inv * d).a[0];
}

int find(std::vector<int>& parent, int i) {
    while (parent[static_cast<std::size_t>(i)] != i) i = parent[static_cast<std::size_t>(i)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(i)])];
    return i;
}

} // namespace

void Track::append(const TrackPoint& p) {
    if (p.update || points.empty()) lastUpdateT = p.g.t;
    points.push_back(p);
    if (p.update) ++updates;
    if (points.size() > kHistory) points.erase(points.begin());
}

bool Deleter::shouldDelete(const Track& t) const noexcept {
    if (t.points.empty()) return true;
    if (kind == Kind::CovarianceTrace) {
        const Covar& p = t.state().p;
        const double tr = positionsOnly ? p(0, 0) + p(2, 2) + p(4, 4) : trace(p);
        return tr > traceThreshold;
    }
    // UpdateTimeStepsDeleter: back through its points, as long as they span at most `steps` time steps (their
    // distinct times counted in place: no allocation, run a dwell at a time on hundreds of tentative tracks)
    double times[64];
    int count = 0;
    const int limit = std::min(steps, 62);
    for (auto it = t.points.rbegin(); it != t.points.rend(); ++it) {
        if (std::find(times, times + count, it->g.t) == times + count) times[count++] = it->g.t;
        if (count > limit) return true;
        if (it->update) return false;
    }
    return false;
}

double chiSquare4Quantile(double p) noexcept {
    // F(x) = 1 - e^(-x/2) (1 + x/2); Newton from the exponential's quantile
    double x = std::max(1e-6, -2.0 * std::log1p(-p) + 2.0);
    for (int i = 0; i < 100; ++i) {
        const double f = 1.0 - std::exp(-x / 2.0) * (1.0 + x / 2.0) - p;
        const double pdf = x * std::exp(-x / 2.0) / 4.0;
        if (!(pdf > 0.0)) break;
        const double step = f / pdf;
        x = std::max(x - step, x / 2.0);
        if (std::fabs(step) < 1e-15 * x) break;
    }
    return x;
}

std::vector<std::vector<Hypothesis>> distanceHypotheses(const std::vector<const Track*>& tracks, const std::vector<Detection>& detections,
                                                        double t, const TrackerSettings& s, double missedDistance) {
    std::vector<std::vector<Hypothesis>> out(tracks.size());
    for (std::size_t k = 0; k < tracks.size(); ++k) {
        const Gaussian prediction = predict(tracks[k]->state(), t, s.motion);
        Hypothesis missed;
        missed.prediction = prediction;
        missed.distance = missedDistance;
        out[k].push_back(missed);
        for (std::size_t j = 0; j < detections.size(); ++j) {
            Hypothesis h;
            h.detection = static_cast<int>(j);
            h.prediction = prediction;
            h.mp = predictMeasurement(prediction, detections[j].model, s.filter);
            h.distance = std::sqrt(squaredMahalanobis(RadarMeasurement::difference(detections[j].z, h.mp.z), h.mp.s));
            if (h.distance < missedDistance) out[k].push_back(h);
        }
    }
    return out;
}

std::vector<std::vector<Hypothesis>> probabilityHypotheses(const std::vector<const Track*>& tracks, const std::vector<Detection>& detections,
                                                           double t, const TrackerSettings& s) {
    std::vector<std::vector<Hypothesis>> out(tracks.size());
    const double gate = chiSquare4Quantile(s.pg);
    const double cz = kPi * kPi / 2.0; // (the unit 4-ball's volume)
    for (std::size_t k = 0; k < tracks.size(); ++k) {
        const Gaussian prediction = predict(tracks[k]->state(), t, s.motion);
        Hypothesis missed;
        missed.prediction = prediction;
        missed.logProbability = std::log(1.0 - s.pd * s.pg);
        out[k].push_back(missed);
        int validated = 0;
        for (std::size_t j = 0; j < detections.size(); ++j) {
            Hypothesis h;
            h.detection = static_cast<int>(j);
            h.prediction = prediction;
            h.mp = predictMeasurement(prediction, detections[j].model, s.filter);
            const double d2 = squaredMahalanobis(RadarMeasurement::difference(detections[j].z, h.mp.z), h.mp.s);
            if (!(d2 <= gate)) continue; // (gated out: Stone Soup's include_all off)
            ++validated;
            h.logProbability = -0.5 * (d2 + std::log(determinant(h.mp.s)) + 4.0 * std::log(2.0 * kPi)) + std::log(s.pd);
            if (s.clutterDensity >= 0.0) h.logProbability -= std::log(s.clutterDensity);
            out[k].push_back(h);
        }
        if (s.clutterDensity < 0.0) // (no clutter density: the validation region's volume over the measurements in it)
            for (std::size_t i = 1; i < out[k].size(); ++i)
                out[k][i].logProbability += std::log(cz * gate * gate * std::sqrt(determinant(out[k][i].mp.s)) / validated);
    }
    return out;
}

std::vector<std::size_t> associateGnn(const std::vector<std::vector<Hypothesis>>& hyps, std::size_t detections) {
    std::vector<std::size_t> chosen(hyps.size(), 0);
    std::vector<std::size_t> rows;
    for (std::size_t k = 0; k < hyps.size(); ++k)
        if (hyps[k].size() > 1) rows.push_back(k);
    if (rows.empty()) return chosen;
    // tracks x (detections, then each track's missed detection)
    const std::size_t cols = detections + rows.size();
    std::vector<double> cost(rows.size() * cols, kInf);
    for (std::size_t r = 0; r < rows.size(); ++r)
        for (const Hypothesis& h : hyps[rows[r]])
            cost[r * cols + (h.detection < 0 ? detections + r : static_cast<std::size_t>(h.detection))] = h.distance;
    const std::vector<int> col4row = assign(cost, rows.size(), cols);
    for (std::size_t r = 0; r < rows.size() && r < col4row.size(); ++r) {
        const auto c = static_cast<std::size_t>(col4row[r]);
        const auto& list = hyps[rows[r]];
        for (std::size_t i = 0; i < list.size(); ++i)
            if ((list[i].detection < 0 && c == detections + r) || (list[i].detection >= 0 && static_cast<std::size_t>(list[i].detection) == c))
                chosen[rows[r]] = i;
    }
    return chosen;
}

std::vector<std::vector<double>> associateJpda(const std::vector<std::vector<Hypothesis>>& hyps, std::size_t detections) {
    const std::size_t n = hyps.size();
    std::vector<std::vector<double>> out(n);
    // clusters: tracks joined by a detection both gate
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    std::vector<int> owner(detections, -1);
    for (std::size_t k = 0; k < n; ++k)
        for (const Hypothesis& h : hyps[k]) {
            if (h.detection < 0) continue;
            int& o = owner[static_cast<std::size_t>(h.detection)];
            if (o < 0) o = static_cast<int>(k);
            else parent[static_cast<std::size_t>(find(parent, static_cast<int>(k)))] = find(parent, o);
        }
    std::vector<std::vector<std::size_t>> clusters(n);
    for (std::size_t k = 0; k < n; ++k) clusters[static_cast<std::size_t>(find(parent, static_cast<int>(k)))].push_back(k);
    std::vector<char> used(detections, 0);
    for (const auto& cluster : clusters) {
        if (cluster.empty()) continue;
        // every joint event (each track one hypothesis, no detection twice), depth first: the largest log
        // probability first, then each event's share
        std::vector<std::size_t> pick(cluster.size(), 0);
        double best = -kInf;
        auto visit = [&](auto&& self, std::size_t depth, double logp, auto&& onEvent) -> void {
            if (depth == cluster.size()) {
                onEvent(logp);
                return;
            }
            const auto& list = hyps[cluster[depth]];
            for (std::size_t i = 0; i < list.size(); ++i) {
                const int d = list[i].detection;
                if (d >= 0 && used[static_cast<std::size_t>(d)]) continue;
                if (d >= 0) used[static_cast<std::size_t>(d)] = 1;
                pick[depth] = i;
                self(self, depth + 1, logp + list[i].logProbability, onEvent);
                if (d >= 0) used[static_cast<std::size_t>(d)] = 0;
            }
        };
        visit(visit, 0, 0.0, [&](double logp) { best = std::max(best, logp); });
        std::vector<std::vector<double>> marginal(cluster.size());
        for (std::size_t c = 0; c < cluster.size(); ++c) marginal[c].assign(hyps[cluster[c]].size(), 0.0);
        double total = 0.0;
        visit(visit, 0, 0.0, [&](double logp) {
            const double w = std::exp(logp - best);
            total += w;
            for (std::size_t c = 0; c < cluster.size(); ++c) marginal[c][pick[c]] += w;
        });
        for (std::size_t c = 0; c < cluster.size(); ++c) {
            for (double& m : marginal[c]) m /= total;
            out[cluster[c]] = std::move(marginal[c]);
        }
    }
    return out;
}

Track MultiTargetTracker::simpleInitiate(const Detection& d) {
    const RadarMeasurement& m = d.model;
    const State x = m.inverse(d.z);
    const Mat<4, 6> j = m.jacobian(x);
    // the Jacobian's pseudo-inverse (full row rank: J^T (J J^T)^-1)
    MeasCovar jjInv;
    Mat<6, 4> jinv;
    if (inverse(j * transpose(j), jjInv)) jinv = transpose(j) * jjInv;
    Covar c0 = jinv * m.noise * transpose(jinv);
    Covar prior = s_.prior.p;
    State priorX = s_.prior.x;
    for (std::size_t dim : {0u, 2u, 4u}) {
        priorX.a[dim] = 0.0;
        for (std::size_t k = 0; k < 6; ++k) prior(dim, k) = prior(k, dim) = 0.0;
    }
    c0 = c0 + prior;
    for (std::size_t k = 0; k < 6; ++k) c0(k, k) += s_.diagLoad;
    Track t;
    t.id = nextId_++;
    TrackPoint p;
    p.g.x = priorX + x;
    p.g.p = c0;
    p.g.t = d.t;
    p.update = true;
    p.detection = d.id;
    p.truth = d.truth;
    t.append(p);
    return t;
}

std::vector<Track> MultiTargetTracker::initiate(double t, const std::vector<Detection>& detections, const Looked& looked) {
    std::vector<Track> sure;
    std::vector<char> associated(detections.size(), 0);
    if (!holding_.empty()) {
        std::vector<std::size_t> seen;
        std::vector<const Track*> list;
        std::vector<Look> how;
        for (std::size_t k = 0; k < holding_.size(); ++k) {
            const Look l = looked ? looked(holding_[k]) : Expected;
            if (l != Not) seen.push_back(k), list.push_back(&holding_[k]), how.push_back(l);
        }
        const auto hyps = distanceHypotheses(list, detections, t, s_, s_.initiatorMissedDistance);
        const auto chosen = associateGnn(hyps, detections.size());
        std::vector<char> isSure(holding_.size(), 0), appended(holding_.size(), 0);
        for (std::size_t n = 0; n < seen.size(); ++n) {
            const std::size_t k = seen[n];
            const Hypothesis& h = hyps[n][chosen[n]];
            TrackPoint p;
            if (h.detection >= 0) {
                const Detection& d = detections[static_cast<std::size_t>(h.detection)];
                if (update(h.prediction, h.mp, d.z, p.g)) p.update = true, p.detection = d.id, p.truth = d.truth;
                else p.g = h.prediction;
                associated[static_cast<std::size_t>(h.detection)] = 1;
            } else if (how[n] == Expected) {
                p.g = h.prediction;
            } else {
                continue; // (not expected to be seen: no miss)
            }
            holding_[k].append(p);
            appended[k] = 1;
            if (static_cast<int>(holding_[k].updates) >= s_.minPoints) isSure[k] = 1;
        }
        // released or deleted, in place (a track nothing was appended to keeps the verdict it had: its points the same)
        std::size_t w = 0;
        for (std::size_t k = 0; k < holding_.size(); ++k) {
            if (isSure[k]) sure.push_back(std::move(holding_[k]));
            else if (!(appended[k] && s_.initiatorDeleter.shouldDelete(holding_[k])) && !stale(holding_[k], t)) {
                if (w != k) holding_[w] = std::move(holding_[k]);
                ++w;
            }
        }
        holding_.resize(w);
    }
    for (std::size_t j = 0; j < detections.size(); ++j)
        if (!associated[j] && (heldCapacity_ == 0 || holding_.size() < heldCapacity_)) holding_.push_back(simpleInitiate(detections[j]));
    return sure;
}

void MultiTargetTracker::step(double t, const std::vector<Detection>& detections, const Looked& looked) {
    std::vector<char> associated(detections.size(), 0), appended(tracks_.size(), 0);
    std::vector<std::size_t> seen;
    std::vector<const Track*> list;
    std::vector<Look> how;
    for (std::size_t k = 0; k < tracks_.size(); ++k) {
        const Look l = looked ? looked(tracks_[k]) : Expected;
        if (l != Not) seen.push_back(k), list.push_back(&tracks_[k]), how.push_back(l);
    }
    if (s_.association == Association::Gnn) {
        const auto hyps = distanceHypotheses(list, detections, t, s_, s_.missedDistance);
        const auto chosen = associateGnn(hyps, detections.size());
        for (std::size_t n = 0; n < seen.size(); ++n) {
            const std::size_t k = seen[n];
            const Hypothesis& h = hyps[n][chosen[n]];
            TrackPoint p;
            if (h.detection >= 0) {
                const Detection& d = detections[static_cast<std::size_t>(h.detection)];
                if (update(h.prediction, h.mp, d.z, p.g)) p.update = true, p.detection = d.id, p.truth = d.truth;
                else p.g = h.prediction;
                associated[static_cast<std::size_t>(h.detection)] = 1;
            } else if (how[n] == Expected) {
                p.g = h.prediction;
            } else {
                continue; // (not expected to be seen: no miss)
            }
            tracks_[k].append(p);
        }
    } else {
        const auto hyps = probabilityHypotheses(list, detections, t, s_);
        const auto marginals = associateJpda(hyps, detections.size());
        for (std::size_t n = 0; n < seen.size(); ++n) {
            const std::size_t k = seen[n];
            const auto& options = hyps[n];
            const auto& w = marginals[n];
            std::vector<Gaussian> posterior;
            posterior.reserve(options.size());
            for (const Hypothesis& h : options) {
                Gaussian g = h.prediction;
                if (h.detection >= 0) update(h.prediction, h.mp, detections[static_cast<std::size_t>(h.detection)].z, g);
                posterior.push_back(g);
            }
            TrackPoint p;
            p.g = reduce(posterior, w);
            double bestW = w[0];
            for (std::size_t i = 1; i < options.size(); ++i)
                if (w[i] > w[0]) { // (more probable than missed: an update, and the detection not left to the initiator)
                    p.update = true;
                    associated[static_cast<std::size_t>(options[i].detection)] = 1;
                    if (w[i] > bestW)
                        bestW = w[i], p.detection = detections[static_cast<std::size_t>(options[i].detection)].id,
                        p.truth = detections[static_cast<std::size_t>(options[i].detection)].truth;
                }
            if (!p.update && how[n] != Expected) continue; // (not expected to be seen: no miss)
            tracks_[k].append(p);
        }
    }
    for (const std::size_t k : seen) appended[k] = tracks_[k].points.empty() ? 0 : 1;
    {
        std::size_t w = 0;
        for (std::size_t k = 0; k < tracks_.size(); ++k)
            if (!(appended[k] && s_.deleter.shouldDelete(tracks_[k])) && !stale(tracks_[k], t)) {
                if (w != k) tracks_[w] = std::move(tracks_[k]);
                ++w;
            }
        tracks_.resize(w);
    }
    std::vector<Detection> left;
    for (std::size_t j = 0; j < detections.size(); ++j)
        if (!associated[j]) left.push_back(detections[j]);
    for (Track& tr : initiate(t, left, looked))
        if (capacity_ == 0 || tracks_.size() < capacity_) tracks_.push_back(std::move(tr));
}

} // namespace fsim::ms::track
