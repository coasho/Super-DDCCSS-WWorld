#pragma once
// An air-to-air search radar at the detection level (ADR-30, MS-4; A-GRA's
// AMTI): its scan, its dwells, and on each the radar equation against every
// entity in its beam -
//
//   SNR = P_avg G(beam)^2 sigma lambda^2 T_dwell / ((4 pi)^3 R^4 k T0 F L_sys)
//
// with the radio path's basic transmission loss for each way (src/simulation/mission_systems/rf: P.528,
// the terrain, the gases) in place of free space, the antenna's pattern towards
// the target (its aperture, steered), the target's RCS from its table at the
// aspect and polarisation (MS-3), and the clutter a pulse-Doppler radar meets:
// a target whose Doppler falls in the main beam's clutter is notched out; one
// in the side lobes' clutter band (closing slower than the radar flies) is seen
// against that clutter, the card's penalty on its SNR. Each look detects with
// the probability its SNR, the false-alarm rate and the target's fluctuation
// give (Detection.h), drawn from the radar's own seeded stream; what it detects
// it measures, with errors that shrink as the SNR grows.
//
// Its engagement modes (MS-4b; A-GRA's AMTI_CapabilityEnum): a track it is told
// to follow - the scan's volume kept on it (track-while-scan; an electronic
// array looks at it between its search dwells too), or every dwell on it
// (single-target track, with which it can light the target for a semi-active
// missile) - and a cued search that acquires what it finds: the first
// detection looked at again until its track is confirmed, then tracked alone
// (a fighter's air-combat modes are such volumes). Nothing is given to a
// track: each dwell on it is the same radar equation, the same clutter, the
// same draw as a search dwell's - so a target that turns into the notch, drops
// behind a ridge or leaves the antenna's reach stops being detected, and its
// track goes when the radar's memory time is up.

#include "fsim/MissionSystems.h"
#include "simulation/mission_systems/radar/Detection.h"
#include "simulation/mission_systems/radar/Signal.h"
#include "simulation/mission_systems/rf/Antenna.h"
#include "simulation/mission_systems/track/Tracker.h"

#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace fsim::sim {
struct VehicleState;
}

namespace fsim::ms {
struct Card;
}

namespace fsim::ms::radar {

/// How the radar searches (its waveform and what its processing rejects): the SDK's.
using Mode = AmtiMode;

/// A radar's design, from its card (assets/payloads/<card>.json).
struct Spec {
    double frequencyHz = 9.5e9;
    double polarizationRad = 0.0;        ///< from the vertical towards the horizontal
    rf::ApertureSpec aperture;
    double averagePowerW = 1000.0;
    double noiseFigureDb = 3.0;
    double systemLossDb = 8.0;
    double dwellS = 0.02;                ///< a beam position's coherent integration (electronic scan)
    double scanRateDegS = 60.0;          ///< a mechanical antenna's sweep (its dwell: a beamwidth's time on target)
    double rotationRpm = 0.0;            ///< a rotodome's: > 0, a 360 deg sweep
    double azimuthLimitDeg = 60.0;       ///< its field of regard either side of the nose
    double elevationLimitDeg = 60.0;
    /// The mean time between false alarms its threshold is set for (Stimson, 2nd ed., p. 144: "a minute or so" for a
    /// fighter's radar; an AEW radar's alike, assumed): each dwell's false-alarm probability a resolution cell from it
    /// and the dwell's own count of cells.
    double falseAlarmTimeS = 60.0;
    double rangeResolutionM = 150.0;
    double instrumentedRangeM = 200e3;
    double notchMs = 30.0;               ///< the main-beam clutter notch's half width (radial speed)
    double sidelobeClutterDb = 10.0;     ///< the SNR lost to side-lobe clutter by a target in its band
    double monopulseSlope = 1.6;
    /// Its PRF schedule: a dwell split into this many coherent intervals, each at its own PRF (and frequency: their
    /// returns independent), a detection declared on at least M of them - medium PRF's ambiguity resolution
    /// (range-while-search, track-while-scan, surveillance) and high PRF's (velocity search).
    /// Its class's calibration (dB of SNR): the mean of what its class's published ranges ask of the radar equation -
    /// every radar of a class alike, whoever made it (its power, duty and losses where unknown).
    double classCalibrationDb = 0.0;
    int mediumPrfs = 8, mediumM = 3;
    int highPrfs = 3, highM = 1;
    // Its track processing (MS-5): Stone Soup's GNN tracker, a track's dwell at a time
    std::size_t trackCapacity = 20;      ///< confirmed tracks at most (a fighter's TWS; an AEW radar's hundreds)
    double trackProcessNoise = 100.0;    ///< the motion model's acceleration noise density, m^2/s^3 an axis (~10 m/s^2 of manoeuvre)
    double trackGate = 4.0;              ///< the Mahalanobis gate (4 dimensions: 99.7 % of a target's own detections inside)
    int trackConfirm = 3;                ///< updates that confirm a track (M of its first looks)
    int trackDropLooks = 3;              ///< expected looks without an update that drop a track (and, looked at or not, that many frames and one more)
    double trackVelocitySigma = 300.0;   ///< a new track's velocity uncertainty across the line of sight, m/s an axis
    // Its engagement modes (MS-4b)
    /// How long it keeps a track it cannot update, in single-target track and in acquisition (a scan's tracks go by
    /// their looks, above): an assumption - 3 s, of the few seconds public descriptions of fighter radars' memory
    /// (coast) give; no card has its own radar's.
    double trackMemoryS = 3.0;
    /// An electronic array's look at its designated track between its search dwells, track-while-scan: no longer
    /// apart than this (an assumption: 1 s; a mechanical antenna sees it once a frame).
    double trackUpdateS = 1.0;
    /// Acquisition: the dwells it looks again at a detection for, before it takes up its scan where it left it.
    int acquireDwells = 8;
    bool ok = false;                     ///< the card gives a radar (AMTI supported)
};

/// The card's radar (empty `ok` if it lacks one).
Spec specOf(const Card& card);

/// What the radar is told to do: search a volume, acquire a target in one, or track one of its tracks.
using Task = AmtiTask;

/// What the radar is told to search (A-GRA's AMTI command: its volume and mode).
struct Search {
    Mode mode = Mode::RangeWhileSearch;
    Task task = Task::Volume;
    std::uint64_t track = 0;             ///< Track: the track to follow; Cued: one to cue on (0: the volume as given)
    double elevationHalfWidthRad = std::numeric_limits<double>::quiet_NaN(); ///< given: the bars that cover it
    bool illuminate = false;             ///< single-target track: light the target (a semi-active missile's)
    double azimuthCentreRad = 0.0;       ///< from the nose, right positive
    double azimuthHalfWidthRad = 1.047;  ///< (+-60 deg)
    double elevationCentreRad = 0.0;     ///< above the body's x-y plane
    int bars = 2;
    double rangeScaleM = 150e3;
    /// Signal level (MS-6): each dwell through the radar's samples and processing (Signal.h), not the detection
    /// level's closed forms. Medium PRF modes.
    bool signalLevel = false;
};

/// One detection: a look that crossed the threshold, measured.
struct Detection {
    std::uint64_t serial = 0;            ///< the detections before it
    double timeS = 0.0;
    std::uint32_t truth = 0;             ///< the entity that made it (the simulation's knowledge, not the radar's)
    double rangeM = 0.0;
    double azimuthRad = 0.0;             ///< from the nose, right positive (body axes)
    double elevationRad = 0.0;           ///< above the body's x-y plane
    double rangeRateMs = 0.0;            ///< positive opening
    double snrDb = 0.0;
    /// The measurement's standard deviations - elevation, azimuth (rad), range (m), range rate (m/s) - as its SNR
    /// gives them: the tracker's measurement noise.
    double sigma[4] = {0, 0, 0, 0};
    double latitudeRad = 0.0, longitudeRad = 0.0, heightM = 0.0; ///< where the measurement puts it
};

/// What the radar sees: the world around it, asked during its dwells.
class Scene {
public:
    virtual ~Scene() = default;
    /// The other entities: ids and states (the carrier excluded).
    virtual std::size_t targets(std::uint32_t* ids, const sim::VehicleState** states, std::size_t max) const = 0;
    /// A target's RCS (m^2) towards the radar and its fluctuation ratio; NaN when its type has no table.
    virtual double sigma(std::uint32_t target, double frequencyHz, double polarizationRad, double& fluctuation) const = 0;
    /// The one-way basic transmission loss (dB) between the radar's carrier and a target, the terrain's mask included.
    virtual double pathLossDb(std::uint32_t target, double frequencyHz) const = 0;
    /// The terrain's height (m, above the ellipsoid; the sea's floor below 0) at a place; NaN without terrain (the
    /// ground then flat, land, at the carrier's height above it).
    virtual double surfaceHeightM(double latitudeRad, double longitudeRad) const {
        (void)latitudeRad, (void)longitudeRad;
        return std::numeric_limits<double>::quiet_NaN();
    }
    /// Whether there is ground at all (a scene of free space has none: no clutter).
    virtual bool hasGround() const { return true; }
};

class Radar {
public:
    Radar() = default;
    Radar(const Spec& spec, std::uint64_t seed);

    const Spec& spec() const noexcept { return spec_; }
    const rf::Pattern& pattern() const noexcept { return pattern_; }
    /// Start searching (a new or updated AMTI activity); stop.
    void search(const Search& search, double simTime);
    void stop() noexcept { active_ = false, designated_ = 0, acquiring_ = false; }
    /// Whether it holds a confirmed track of that number.
    bool hasTrack(std::uint64_t id) const noexcept;
    /// What it does now: its command's task - a Cued one Track once it has acquired its target.
    Task task() const noexcept { return task_; }
    /// The track it follows (0: none - a Volume task, a Cued one still searching).
    std::uint64_t designated() const noexcept { return designated_; }
    /// Cued: it looks again at a detection, to confirm its track.
    bool acquiring() const noexcept { return acquiring_; }
    /// It lost the target it tracked (and stopped): its activity's end.
    bool lost() const noexcept { return lost_; }
    /// It lights its designated target for a semi-active missile now: told to, and in single-target track - its beam
    /// on the track, through its memory time too (it points where the track says; what comes back from there is the
    /// seeker's to find). It ends with the track.
    bool illuminating() const noexcept { return active_ && search_.illuminate && staring(); }
    /// Its random stream restarts from `seed` (a vehicle reset with a seed).
    void reseed(std::uint64_t seed) { rng_.seed(seed), signalStream_.reseed(seed ^ kSignalStream); }
    bool active() const noexcept { return active_; }
    const Search& searching() const noexcept { return search_; }

    /// Advance to `simTime`: the dwells due, each against the scene. `own` its carrier's state.
    void step(double simTime, const sim::VehicleState& own, const Scene& scene);

    /// Where the beam points now (body axes) and the frame's progress.
    double beamAzimuthRad() const noexcept { return beamAz_; }
    double beamElevationRad() const noexcept { return beamEl_; }
    double framePeriodS() const noexcept { return framePeriod_; }
    /// The elevation bars its scan has (those its command gave, or those that cover the height it gave).
    int bars() const noexcept { return bars_; }
    std::uint64_t dwells() const noexcept { return dwells_; }

    /// Detections, oldest first (the last 2048), and the count ever made (false alarms among them, truth 0).
    const std::deque<Detection>& detections() const noexcept { return detections_; }
    std::uint64_t detectionCount() const noexcept { return detectionCount_; }
    std::uint64_t falseAlarmCount() const noexcept { return falseAlarms_; }
    /// Its confirmed tracks (MS-5), in the ECEF frame; and their files dropped (the radar switched off).
    const std::vector<track::Track>& tracks() const noexcept { return tracker_.tracks(); }
    std::size_t tentativeTracks() const noexcept { return tracker_.holding().size(); }
    /// A track as the SDK reports it, predicted to `now`, relative to the carrier's state `own`.
    AmtiTrack report(const track::Track& t, double now, const sim::VehicleState& own) const;
    void clearTracks();

    /// The SNR (dB) one dwell would give against a target of `sigmaM2` at `rangeM` on the beam's axis, free space:
    /// the radar equation alone (for its range and the tests).
    double snrDb(double sigmaM2, double rangeM) const noexcept;
    /// The range (m) at which it detects a steady-mean target of `sigmaM2` with probability `pd` on one look, free
    /// space, on the beam's axis, the target fluctuating as `f` (a nominal dwell: its instrumented range searched from
    /// a carrier at kNominalSpeedMs).
    double detectionRangeM(double sigmaM2, double pd, int fluctuation, Mode mode = Mode::RangeWhileSearch) const noexcept;
    /// The signal level's waveform for the planned dwell, and what its processing has done (MS-6).
    const signal::Waveform& waveform() const noexcept { return waveform_; }
    const signal::Stats& signalStats() const noexcept { return signalStats_; }
    /// The last signal-level dwell's clutter (per interval, receive gates by filters).
    const signal::ClutterMap& clutter() const noexcept { return clutter_; }
    /// The last signal-level dwell's first interval as a range-Doppler display shows it (MS-6c), and when it was.
    const signal::Snapshot& snapshot() const noexcept { return snapshot_; }
    double snapshotTimeS() const noexcept { return snapshotTimeS_; }
    /// The transmitter's duty (the same for every radar: the PS-05/A's published 1 kW average of its 10 kW peak,
    /// as the cards' class powers take it).
    static constexpr double kDuty = 0.1;
    /// The ground's reflectivity, constant gamma (sigma0 = gamma sin grazing) at 10 GHz, + 5 log10(f / 10 GHz) - an
    /// engineering assumption: the values MATLAB's surfacegamma gives farmland (after Nathanson et al., Radar Design
    /// Principles, 1999) and sea state 3 (after Long, Radar Reflectivity of Land and Sea, 2001), the primaries not read.
    static constexpr double kGammaLandDb = -15.0, kGammaSeaDb = -40.0;
    /// The ground modelled out to here at most (and its horizon): MPRF folds far clutter into near gates.
    static constexpr double kClutterRangeM = 400e3;
    /// The probability a dwell detects a target its whole dwell would see at `snrDb`: its PRF schedule's M of N, each
    /// interval at 1/N of the energy and a false-alarm probability that keeps the dwell's at `pfa`.
    double dwellPd(double snrDb, Fluctuation f, Mode mode, double pfa) const noexcept;
    /// The detection level's interval false-alarm probability for a dwell's: C(n, m) p^m = the dwell's.
    double detectionIntervalPfa(Mode mode, double dwellPfa) const noexcept;
    /// A dwell's resolution cells: its range bins within `rangeScaleM`, by its Doppler filters (each coherent
    /// interval's resolution) over the range rates it searches from a carrier at `ownSpeedMs`.
    double resolutionCells(Mode mode, double dwellS, double rangeScaleM, double ownSpeedMs) const noexcept;
    /// A dwell's false-alarm probability a resolution cell: its false-alarm time's, shared among its cells.
    double dwellPfa(Mode mode, double dwellS, double rangeScaleM, double ownSpeedMs) const noexcept;
    /// The range rates (m/s) a dwell searches: closing and opening (medium PRF), closing alone (velocity search).
    double rateSpanMs(Mode mode, double ownSpeedMs) const noexcept;
    static constexpr double kNominalSpeedMs = 200.0;

    /// A beam position's time on target as the scan plans it (one bar).
    double nominalDwellS() const noexcept;
    /// Its track processing's settings, from its card (MS-5): Stone Soup's GNN tracker with an unscented filter.
    track::TrackerSettings trackerSettings() const;

private:
    void plan();
    void dwell(double time, double az, double el, const sim::VehicleState& own, const Scene& scene);
    // --- its engagement modes (MS-4b)
    bool staring() const noexcept { return task_ == Task::Track && search_.mode == Mode::SingleTargetTrack; }
    const track::Track* held(std::uint64_t id) const noexcept;
    /// Where a track is from the carrier at `time`, body axes; false: the radar does not hold it.
    bool bearing(std::uint64_t id, double time, const sim::VehicleState& own, double& az, double& el) const;
    /// The scan's volume about a track (a frame's start), within the field of regard.
    void recentre(double az, double el);
    /// Cued, after a dwell: a detection to look at again, or the track that makes the target acquired.
    void acquire(double time, const sim::VehicleState& own, std::uint64_t firstNew);
    /// Every dwell on the track from now: single-target track.
    void stare(std::uint64_t id);
    /// Its target is gone: it stops, its activity's end to report.
    void lose() noexcept { lost_ = true, active_ = false, acquiring_ = false, designated_ = 0; }
    Task task_ = Task::Volume;
    std::uint64_t designated_ = 0;
    bool lost_ = false;
    bool acquiring_ = false;
    double acquireAz_ = 0.0, acquireEl_ = 0.0;
    int acquireLeft_ = 0;
    double trackDwellAt_ = 0.0;   ///< the last dwell an electronic array gave its designated track (track-while-scan)
    /// How long before the states it is given a dwell ends (a step's dwells all see the step's states): each place is
    /// taken back along its velocity by it, so that a detection's range is its stamp's - not that of up to a step
    /// later, off by the closing speed times the difference: metres, where a strong return measures to less than one,
    /// and dwell after dwell on one target that is a track its own detections no longer fall inside.
    double lag_ = 0.0;
    int bars_ = 0;
    /// The dwell's detections to its tracker (MS-5): the carrier's pose (r: body to ECEF) and velocity at the dwell.
    void track(double time, double az, double el, const sim::VehicleState& own, const double r[9], const double vOwn[3],
               const std::vector<Detection>& found);

    Spec spec_;
    rf::Pattern pattern_;
    std::mt19937_64 rng_;
    static constexpr std::uint64_t kSignalStream = 0x5167a1ull; ///< (the signal level's stream: apart from the detection level's)
    signal::Stream signalStream_;
    signal::Waveform waveform_;  ///< the signal level's, for the planned dwell
    signal::Stats signalStats_;
    double signalQ_ = 0.0, signalQAt_ = 0.0; ///< the CFAR's peak survival, and the interval threshold it was found at
    double signalLossDb_[3] = {};            ///< the chain's mean processing loss there, by fluctuation (processingLossDb; NaN: not yet)
    void dwellSignal(double time, double az, double el, const sim::VehicleState& own, const Scene& scene);
    /// The ground's clutter for the dwell's beam (MS-6b; Signal.h's ClutterMap): range rings on a sphere of 4/3 the
    /// Earth's radius, constant-gamma reflectivity, each patch's two-way gain and Doppler; `scaleDb` the radar
    /// equation's terms that are the same for every patch. Its range rate in the main beam, and the altitude, out.
    void clutterMap(const sim::VehicleState& own, const double r[9], const double vOwn[3], const double beamBody[3], const Scene& scene,
                    double scaleDb, signal::ClutterMap& out, double& mainBeamRateMs, double& altitudeM) const;
    signal::ClutterMap clutter_;
    signal::Snapshot snapshot_;
    double snapshotTimeS_ = 0.0;
    /// A dwell's detections kept (numbered, located) and given to its tracker: both levels'.
    void finish(double time, double az, double el, const sim::VehicleState& own, const double r[9], const double vOwn[3],
                std::vector<Detection>& found);
    bool active_ = false;
    Search search_;
    std::vector<std::pair<double, double>> positions_; ///< the frame's beam positions (az, el), body axes
    std::size_t next_ = 0;
    double dwellS_ = 0.02;
    double clock_ = 0.0;      ///< the time the next dwell ends
    double framePeriod_ = 0.0;
    double beamAz_ = 0.0, beamEl_ = 0.0;
    std::uint64_t dwells_ = 0, detectionCount_ = 0, falseAlarms_ = 0;
    std::deque<Detection> detections_;
    track::MultiTargetTracker tracker_{track::TrackerSettings{}};
};

} // namespace fsim::ms::radar
