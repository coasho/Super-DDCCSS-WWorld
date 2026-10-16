#pragma once
// Multi-target trackers (ADR-30, MS-5): Stone Soup's (Dstl, MIT) components,
// ported - their equations, their order of operations - and cross-checked
// against Stone Soup itself on the same measurements (test_track.cpp, the
// fixtures tools/track/crosscheck.py writes):
//  - hypothesisers: by Mahalanobis distance, gated at a missed-detection
//    distance (DistanceHypothesiser); by probability, Pd, the gate's
//    probability and the clutter's density (PDAHypothesiser);
//  - associators: global nearest neighbour by 2D assignment
//    (GNNWith2DAssignment); joint probabilistic data association (JPDA) -
//    every joint event enumerated, factorised over the clusters of tracks
//    that share gated detections (the same marginals, exactly: a joint
//    event's probability is the product of its tracks', and tracks with no
//    detection in common constrain each other in nothing);
//  - initiators: a track from one detection through the measurement's
//    inverse (SimpleMeasurementInitiator), confirmed after it is updated
//    `minPoints` times while held apart (MultiMeasurementInitiator);
//  - deleters: by the time steps since an update (UpdateTimeStepsDeleter), the
//    time since one (UpdateTimeDeleter) or the covariance's trace
//    (CovarianceBasedDeleter);
//  - trackers: MultiTargetTracker (one hypothesis a track) and
//    MultiTargetMixtureTracker (a track's hypotheses' Gaussian mixture,
//    reduced).

#include "simulation/mission_systems/track/Filters.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace fsim::ms::track {

struct Detection {
    double t = 0.0;
    Meas z;
    RadarMeasurement model; ///< the sensor's pose and noise when it was made
    std::uint64_t id = 0;
    std::uint32_t truth = 0; ///< the entity that made it (0: a false alarm) - for scoring only
};

struct TrackPoint {
    Gaussian g;
    bool update = false;  ///< an update by a detection (else a prediction)
    std::uint64_t detection = 0; ///< the detection that updated it (GNN), or the most probable (JPDA)
    std::uint32_t truth = 0;     ///< that detection's truth (0: a false alarm) - for scoring only
};

struct Track {
    std::uint64_t id = 0;
    std::vector<TrackPoint> points; ///< its last kHistory points
    std::uint32_t updates = 0;      ///< every update, ever
    double lastUpdateT = 0.0;       ///< its last update's time (its first point's, before any)
    const Gaussian& state() const noexcept { return points.back().g; }
    void append(const TrackPoint& p);
    static constexpr std::size_t kHistory = 32;
};

enum class Association : unsigned char { Gnn, Jpda };

struct Deleter {
    enum class Kind : unsigned char { UpdateSteps, CovarianceTrace } kind = Kind::UpdateSteps;
    int steps = 3;                 ///< UpdateSteps: deleted when no update in its last `steps` time steps
    double traceThreshold = 1e9;   ///< CovarianceTrace: deleted when the trace exceeds it
    bool positionsOnly = false;    ///< CovarianceTrace: the positions' variances only (mapping 0, 2, 4)
    bool shouldDelete(const Track& t) const noexcept;
};

struct TrackerSettings {
    Filter filter = Filter::Ekf;
    ConstantVelocity motion;
    Association association = Association::Gnn;
    double missedDistance = 3.0;   ///< GNN: the Mahalanobis gate
    double pd = 0.9, pg = 0.95;    ///< JPDA: detection and gate probabilities
    double clutterDensity = -1.0;  ///< JPDA: the clutter's density in measurement space (< 0: Stone Soup's None)
    Gaussian prior;                ///< initiation's prior (its positions replaced by the detection's)
    double diagLoad = 0.0;
    int minPoints = 2;             ///< updates that confirm a held track
    double initiatorMissedDistance = 3.0; ///< the held tracks' GNN gate
    Deleter initiatorDeleter;      ///< deletes held tracks
    Deleter deleter;               ///< deletes confirmed tracks
    /// > 0: any track (held ones too) without an update for longer is deleted, whether the step looked at it or not
    /// (Stone Soup's UpdateTimeDeleter, with the deleters above: CompositeDeleter's any) - a track that left the
    /// sensor's coverage, never missed again, goes in its time
    double coastS = 0.0;
};

/// A single target-detection hypothesis (detection -1: missed).
struct Hypothesis {
    int detection = -1;
    Gaussian prediction;
    MeasurementPrediction mp;
    double distance = 0.0;     ///< Mahalanobis (GNN)
    double logProbability = 0.0; ///< (JPDA), before normalisation
};

/// The chi-square quantile for 4 degrees of freedom (scipy.stats.chi2.ppf(p, 4)).
double chiSquare4Quantile(double p) noexcept;

/// For each track (in order): its hypotheses, the missed one first.
std::vector<std::vector<Hypothesis>> distanceHypotheses(const std::vector<const Track*>& tracks, const std::vector<Detection>& detections,
                                                        double t, const TrackerSettings& s, double missedDistance);
std::vector<std::vector<Hypothesis>> probabilityHypotheses(const std::vector<const Track*>& tracks, const std::vector<Detection>& detections,
                                                           double t, const TrackerSettings& s);
/// GNN: each track's chosen hypothesis (an index into its hypotheses).
std::vector<std::size_t> associateGnn(const std::vector<std::vector<Hypothesis>>& hypotheses, std::size_t detections);
/// JPDA: each track's hypotheses' marginal probabilities (normalised, the missed one first).
std::vector<std::vector<double>> associateJpda(const std::vector<std::vector<Hypothesis>>& hypotheses, std::size_t detections);

class MultiTargetTracker {
public:
    explicit MultiTargetTracker(const TrackerSettings& s) : s_(s) {}
    /// One time step: its detections (all at `t`). `looked`, where given, says how the step's sensor saw each track
    /// (confirmed and held) - a scanning radar's beam: Expected, in its main lobe (a detection expected: a miss
    /// counts); Possible, near enough for a detection of it to come from there (it may associate one; a miss does
    /// not count, the track left as it was); Not, the rest (neither predicted nor missed). Stone Soup's trackers
    /// expect every track every step: the same with `looked` empty.
    enum Look : int { Not = 0, Possible = 1, Expected = 2 };
    using Looked = std::function<Look(const Track&)>;
    void step(double t, const std::vector<Detection>& detections, const Looked& looked = {});
    /// Confirmed tracks at most (a radar's track capacity): one confirmed beyond it is dropped. 0: no limit.
    void setCapacity(std::size_t confirmed, std::size_t held) noexcept { capacity_ = confirmed, heldCapacity_ = held; }
    /// TrackerSettings::coastS from now on (a scan's period sets it).
    void setCoast(double seconds) noexcept { s_.coastS = seconds; }
    const std::vector<Track>& tracks() const noexcept { return tracks_; }
    const std::vector<Track>& holding() const noexcept { return holding_; }
    const TrackerSettings& settings() const noexcept { return s_; }

private:
    std::vector<Track> initiate(double t, const std::vector<Detection>& detections, const Looked& looked);
    bool stale(const Track& tr, double t) const noexcept { return s_.coastS > 0.0 && t - tr.lastUpdateT > s_.coastS; }
    Track simpleInitiate(const Detection& d);

    TrackerSettings s_;
    std::vector<Track> tracks_, holding_;
    std::uint64_t nextId_ = 1;
    std::size_t capacity_ = 0, heldCapacity_ = 0;
};

} // namespace fsim::ms::track
