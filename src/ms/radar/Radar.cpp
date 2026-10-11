#include "ms/radar/Radar.h"

#include "fsim/VehicleState.h"
#include "ms/Cards.h"
#include "ms/radar/Detection.h"
#include "ms/rf/Path.h"
#include "sim/Attitude.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>

namespace fsim::ms::radar {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kLight = 299792458.0;
constexpr double kKT0DbW = -203.977; // 10 log10(k 290 K), dBW/Hz
constexpr std::size_t kKeep = 2048;
constexpr double kMaxTargetSpeedMs = 1000.0; ///< the range rates a radar's Doppler filters search, beyond its own speed

double dot(const double a[3], const double b[3]) noexcept { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

/// North, east, down at a geodetic point, in ECEF.
void ned(double lat, double lon, double n[3], double e[3], double d[3]) noexcept {
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    n[0] = -sl * co, n[1] = -sl * so, n[2] = cl;
    e[0] = -so, e[1] = co, e[2] = 0.0;
    d[0] = -cl * co, d[1] = -cl * so, d[2] = -sl;
}

void velocityEcef(const sim::VehicleState& s, double v[3]) noexcept {
    double n[3], e[3], d[3];
    ned(s.latitudeRad, s.longitudeRad, n, e, d);
    for (int i = 0; i < 3; ++i) v[i] = s.velocityNedMs[0] * n[i] + s.velocityNedMs[1] * e[i] + s.velocityNedMs[2] * d[i];
}

/// ECEF to geodetic (WGS-84; Bowring's method, two iterations).
void geodetic(const double p[3], double& lat, double& lon, double& h) noexcept {
    constexpr double a = 6378137.0, e2 = 6.69437999014e-3, b = a * 0.99664718933525;
    const double ep2 = (a * a - b * b) / (b * b), r = std::hypot(p[0], p[1]);
    lon = std::atan2(p[1], p[0]);
    double beta = std::atan2(p[2] * a, r * b);
    for (int i = 0; i < 2; ++i) {
        lat = std::atan2(p[2] + ep2 * b * std::pow(std::sin(beta), 3), r - e2 * a * std::pow(std::cos(beta), 3));
        beta = std::atan2(b * std::sin(lat), a * std::cos(lat));
    }
    const double sl = std::sin(lat), nr = a / std::sqrt(1.0 - e2 * sl * sl);
    h = r * std::cos(lat) + (p[2] + e2 * nr * sl) * sl - nr;
}

} // namespace

Spec specOf(const Card& c) {
    Spec s;
    if (c.family != Family::AirRadar || !(c.value("average_power_w", 0.0) > 0.0) || !(c.value("frequency_hz", 0.0) > 0.0)) return s;
    s.frequencyHz = c.value("frequency_hz", s.frequencyHz);
    s.polarizationRad = c.value("polarization_deg", 0.0) * kDeg;
    const double diameter = c.value("aperture_diameter_m", 0.0);
    if (diameter > 0.0) {
        s.aperture.shape = rf::ApertureShape::Circular;
        s.aperture.widthM = s.aperture.heightM = diameter;
        s.aperture.taper = rf::Taper::Parabolic;
        s.aperture.pedestal = c.value("aperture_pedestal", 0.3);
    } else {
        s.aperture.shape = rf::ApertureShape::Rectangular;
        s.aperture.widthM = c.value("aperture_width_m", 0.7);
        s.aperture.heightM = c.value("aperture_height_m", s.aperture.widthM);
        const double sll = c.value("taylor_sidelobe_db", 0.0);
        s.aperture.taper = sll > 13.27 ? rf::Taper::Taylor : rf::Taper::Uniform;
        s.aperture.taylorSidelobeDb = sll;
    }
    s.aperture.electronic = c.value("electronic", 0.0) != 0.0;
    s.aperture.scanExponent = c.value("scan_exponent", 1.5);
    s.aperture.efficiency = c.value("aperture_efficiency", 0.7);
    s.aperture.peakGainDbi = c.value("peak_gain_dbi", 0.0);
    s.aperture.floorDbi = c.value("sidelobe_floor_dbi", -5.0);
    s.aperture.backDbi = c.value("back_lobe_dbi", -15.0);
    s.averagePowerW = c.value("average_power_w", s.averagePowerW);
    s.noiseFigureDb = c.value("noise_figure_db", s.noiseFigureDb);
    s.systemLossDb = c.value("system_loss_db", s.systemLossDb);
    s.dwellS = c.value("dwell_s", s.dwellS);
    s.scanRateDegS = c.value("scan_rate_deg_s", s.scanRateDegS);
    s.rotationRpm = c.value("rotation_rpm", 0.0);
    s.azimuthLimitDeg = c.value("azimuth_limit_deg", s.rotationRpm > 0.0 ? 180.0 : s.azimuthLimitDeg);
    s.elevationLimitDeg = c.value("elevation_limit_deg", s.elevationLimitDeg);
    s.falseAlarmTimeS = c.value("false_alarm_time_s", s.falseAlarmTimeS);
    s.rangeResolutionM = c.value("range_resolution_m", s.rangeResolutionM);
    s.instrumentedRangeM = c.value("instrumented_range_m", s.instrumentedRangeM);
    s.notchMs = c.value("notch_ms", s.notchMs);
    s.sidelobeClutterDb = c.value("sidelobe_clutter_db", s.sidelobeClutterDb);
    s.monopulseSlope = c.value("monopulse_slope", s.monopulseSlope);
    s.classCalibrationDb = c.value("class_calibration_db", 0.0);
    s.mediumPrfs = static_cast<int>(c.value("medium_prfs", s.mediumPrfs));
    s.mediumM = static_cast<int>(c.value("medium_m_of_n", s.mediumM));
    s.highPrfs = static_cast<int>(c.value("high_prfs", s.highPrfs));
    s.highM = static_cast<int>(c.value("high_m_of_n", s.highM));
    s.trackCapacity = static_cast<std::size_t>(c.value("track_capacity", s.rotationRpm > 0.0 ? 400.0 : static_cast<double>(s.trackCapacity)));
    s.trackProcessNoise = c.value("track_process_noise", s.trackProcessNoise);
    s.trackGate = c.value("track_gate", s.trackGate);
    s.trackConfirm = static_cast<int>(c.value("track_confirm", s.trackConfirm));
    s.trackDropLooks = static_cast<int>(c.value("track_drop_looks", s.trackDropLooks));
    s.trackMemoryS = c.value("track_memory_s", s.trackMemoryS);
    s.trackUpdateS = c.value("track_update_s", s.trackUpdateS);
    s.acquireDwells = static_cast<int>(c.value("acquire_dwells", s.acquireDwells));
    s.ok = true;
    return s;
}

Radar::Radar(const Spec& spec, std::uint64_t seed)
    : spec_(spec), pattern_(spec.aperture, spec.frequencyHz), rng_(seed), signalStream_(seed ^ kSignalStream) {
    rf::preparePath(spec.frequencyHz); // (its paths' tables built with the vehicle, not in the step of its first look)
    clearTracks();
}

track::TrackerSettings Radar::trackerSettings() const {
    track::TrackerSettings t;
    // the unscented filter: a new track's loose velocity makes the range rate too curved for one linearisation (an
    // EKF's update lands outside its own covariance, and the target's next look in the same pass starts a duplicate)
    t.filter = track::Filter::Ukf;
    for (double& q : t.motion.q) q = spec_.trackProcessNoise;
    t.association = track::Association::Gnn;
    t.missedDistance = t.initiatorMissedDistance = spec_.trackGate;
    // a new track: its position and range rate from its first detection, its velocity across the line of sight unknown
    const double v2 = spec_.trackVelocitySigma * spec_.trackVelocitySigma;
    for (std::size_t k = 0; k < 3; ++k) t.prior.p(2 * k + 1, 2 * k + 1) = v2;
    t.minPoints = spec_.trackConfirm;
    t.initiatorDeleter.steps = 2;
    t.deleter.steps = spec_.trackDropLooks;
    return t;
}

AmtiTrack Radar::report(const track::Track& tr, double now, const sim::VehicleState& own) const {
    AmtiTrack out;
    out.id = tr.id;
    out.timeS = now;
    out.updates = tr.updates;
    for (auto it = tr.points.rbegin(); it != tr.points.rend(); ++it)
        if (it->update) {
            out.lastUpdateS = it->g.t;
            out.truth = it->truth;
            break;
        }
    const track::Gaussian g = track::predict(tr.state(), std::max(now, tr.state().t), tracker_.settings().motion);
    for (std::size_t i = 0; i < 6; ++i) out.stateEcef[i] = g.x.a[i];
    for (std::size_t i = 0; i < 36; ++i) out.covarianceEcef[i] = g.p.a[i];
    const double p[3] = {g.x.a[0], g.x.a[2], g.x.a[4]}, v[3] = {g.x.a[1], g.x.a[3], g.x.a[5]};
    geodetic(p, out.latitudeRad, out.longitudeRad, out.heightM);
    double n[3], e[3], d[3];
    ned(out.latitudeRad, out.longitudeRad, n, e, d);
    out.velocityNedMs[0] = dot(v, n), out.velocityNedMs[1] = dot(v, e), out.velocityNedMs[2] = dot(v, d);
    out.positionSigmaM = std::sqrt((g.p(0, 0) + g.p(2, 2) + g.p(4, 4)) / 3.0);
    out.velocitySigmaMs = std::sqrt((g.p(1, 1) + g.p(3, 3) + g.p(5, 5)) / 3.0);
    // from the carrier, in its body axes
    double r[9];
    sim::attitude::bodyToEcefFromQuaternion(own.attitudeEcefToBody, r);
    double rel[3] = {p[0] - own.positionEcef[0], p[1] - own.positionEcef[1], p[2] - own.positionEcef[2]};
    out.rangeM = std::sqrt(dot(rel, rel));
    if (out.rangeM > 0.0) {
        for (double& c : rel) c /= out.rangeM;
        const double db[3] = {r[0] * rel[0] + r[3] * rel[1] + r[6] * rel[2], r[1] * rel[0] + r[4] * rel[1] + r[7] * rel[2],
                              r[2] * rel[0] + r[5] * rel[1] + r[8] * rel[2]};
        out.azimuthRad = std::atan2(db[1], db[0]);
        out.elevationRad = std::atan2(-db[2], std::hypot(db[0], db[1]));
        double vOwn[3];
        velocityEcef(own, vOwn);
        out.rangeRateMs = (v[0] - vOwn[0]) * rel[0] + (v[1] - vOwn[1]) * rel[1] + (v[2] - vOwn[2]) * rel[2];
    }
    return out;
}

void Radar::clearTracks() {
    const double coast = tracker_.settings().coastS;
    tracker_ = track::MultiTargetTracker(trackerSettings());
    tracker_.setCapacity(spec_.trackCapacity, 4096);
    tracker_.setCoast(coast);
}

void Radar::search(const Search& s, double simTime) {
    search_ = s;
    if (spec_.rotationRpm > 0.0) search_.mode = Mode::Surveillance, search_.task = Task::Volume;
    task_ = search_.task;
    designated_ = task_ == Task::Track ? search_.track : 0;
    lost_ = acquiring_ = false;
    acquireLeft_ = 0;
    plan();
    waveform_ = signal::mediumPrf(spec_.frequencyHz, dwellS_, spec_.mediumPrfs, spec_.mediumM, spec_.rangeResolutionM, kDuty);
    // a track not updated for its drop looks' frames, and one more, is dropped - one that left the scan's coverage too.
    // Every dwell on one track, it goes by the radar's memory time instead; an acquisition's small volume (a frame of
    // a few dwells) keeps the tracks it had for that long at least.
    const double scan = (spec_.trackDropLooks + 1) * framePeriod_;
    tracker_.setCoast(staring() ? spec_.trackMemoryS : task_ == Task::Volume ? scan : std::max(scan, spec_.trackMemoryS));
    if (!active_) clock_ = simTime; // (a running scan carries on from where it is: its next dwell on the new pattern)
    next_ = 0;
    trackDwellAt_ = clock_;
    active_ = true;
}

const track::Track* Radar::held(std::uint64_t id) const noexcept {
    if (id)
        for (const auto& t : tracker_.tracks())
            if (t.id == id) return &t;
    return nullptr;
}

bool Radar::hasTrack(std::uint64_t id) const noexcept { return held(id) != nullptr; }

bool Radar::bearing(std::uint64_t id, double time, const sim::VehicleState& own, double& az, double& el) const {
    const track::Track* t = held(id);
    if (!t) return false;
    const track::Gaussian& g = t->state();
    const double dt = time - g.t;
    double d[3];
    for (std::size_t i = 0; i < 3; ++i) d[i] = g.x.a[2 * i] + g.x.a[2 * i + 1] * dt - own.positionEcef[i];
    double r[9];
    sim::attitude::bodyToEcefFromQuaternion(own.attitudeEcefToBody, r);
    const double db[3] = {r[0] * d[0] + r[3] * d[1] + r[6] * d[2], r[1] * d[0] + r[4] * d[1] + r[7] * d[2], r[2] * d[0] + r[5] * d[1] + r[8] * d[2]};
    az = std::atan2(db[1], db[0]);
    el = std::atan2(-db[2], std::hypot(db[0], db[1]));
    return true;
}

void Radar::recentre(double az, double el) {
    const double limAz = spec_.azimuthLimitDeg * kDeg, limEl = spec_.elevationLimitDeg * kDeg;
    const double half = std::min(search_.azimuthHalfWidthRad, limAz);
    search_.azimuthCentreRad = std::clamp(az, -limAz + half, limAz - half); // (the whole volume inside its field of regard)
    search_.elevationCentreRad = std::clamp(el, -limEl, limEl);
    plan();
}

void Radar::stare(std::uint64_t id) {
    task_ = Task::Track;
    search_.mode = Mode::SingleTargetTrack;
    designated_ = id;
    acquiring_ = false;
    tracker_.setCoast(spec_.trackMemoryS);
}

void Radar::acquire(double time, const sim::VehicleState& own, std::uint64_t firstNew) {
    // acquired: a confirmed track this dwell updated - the one it was cued on, else the nearest
    const track::Track* best = nullptr;
    double bestRange = std::numeric_limits<double>::infinity();
    for (const auto& t : tracker_.tracks()) {
        if (t.lastUpdateT != time) continue;
        if (search_.track && t.id == search_.track) {
            best = &t;
            break;
        }
        const track::Gaussian& g = t.state();
        const double d[3] = {g.x.a[0] - own.positionEcef[0], g.x.a[2] - own.positionEcef[1], g.x.a[4] - own.positionEcef[2]};
        const double range = std::sqrt(dot(d, d));
        if (range < bestRange) bestRange = range, best = &t;
    }
    if (best) return stare(best->id);
    // not yet: the dwell's nearest detection is looked at again, where it was measured - for a few dwells, each with a
    // detection moving the beam onto it; then the scan again, from where it left it
    const Detection* nearest = nullptr;
    for (auto it = detections_.rbegin(); it != detections_.rend() && it->serial >= firstNew; ++it)
        if (!nearest || it->rangeM < nearest->rangeM) nearest = &*it;
    const bool was = acquiring_;
    if (nearest) {
        const double limAz = spec_.azimuthLimitDeg * kDeg, limEl = spec_.elevationLimitDeg * kDeg;
        if (!was) acquireLeft_ = std::max(1, spec_.acquireDwells);
        acquiring_ = true;
        acquireAz_ = std::clamp(nearest->azimuthRad, -limAz, limAz), acquireEl_ = std::clamp(nearest->elevationRad, -limEl, limEl);
    }
    if (was && --acquireLeft_ <= 0) acquiring_ = false;
}

double Radar::nominalDwellS() const noexcept {
    // a beam position's time on target: an electronic beam's dwell; a mechanical one's, the beamwidth swept at its rate
    const double bw = pattern_.beamwidthRad(false);
    if (spec_.rotationRpm > 0.0) return bw / (spec_.rotationRpm * 2.0 * kPi / 60.0);
    return spec_.aperture.electronic ? spec_.dwellS : bw / (spec_.scanRateDegS * kDeg);
}

void Radar::plan() {
    positions_.clear();
    const double bwAz = pattern_.beamwidthRad(false), bwEl = pattern_.beamwidthRad(true);
    // an electronic beam steps 0.8 of a beamwidth; a swept antenna's dwells a beamwidth apart (its sweep, a beamwidth at a time)
    const bool electronic = spec_.aperture.electronic && spec_.rotationRpm <= 0.0;
    const double stepAz = (electronic ? 0.8 : 1.0) * bwAz, stepEl = 0.8 * bwEl;
    const double limAz = spec_.azimuthLimitDeg * kDeg, limEl = spec_.elevationLimitDeg * kDeg;
    int bars = std::max(1, search_.bars);
    if (std::isfinite(search_.elevationHalfWidthRad) && spec_.rotationRpm <= 0.0) // (a volume's height given: the bars that cover it)
        bars = std::clamp(static_cast<int>(std::ceil(2.0 * search_.elevationHalfWidthRad / stepEl - 1e-9)), 1, 64);
    bars_ = bars;
    if (spec_.rotationRpm > 0.0) { // a rotodome: 360 deg, its bars stacked electronically at each azimuth
        const auto n = static_cast<int>(std::ceil(2.0 * kPi / stepAz));
        for (int i = 0; i < n; ++i)
            for (int b = 0; b < bars; ++b)
                positions_.emplace_back(-kPi + (i + 0.5) * 2.0 * kPi / n,
                                        std::clamp(search_.elevationCentreRad + (b - 0.5 * (bars - 1)) * stepEl, -limEl, limEl));
        dwellS_ = nominalDwellS() / bars;
    } else {
        const double lo = std::clamp(search_.azimuthCentreRad - search_.azimuthHalfWidthRad, -limAz, limAz);
        const double hi = std::clamp(search_.azimuthCentreRad + search_.azimuthHalfWidthRad, -limAz, limAz);
        const int n = std::max(1, static_cast<int>(std::ceil((hi - lo) / stepAz)));
        for (int b = 0; b < bars; ++b) {
            const double el = std::clamp(search_.elevationCentreRad + (b - 0.5 * (bars - 1)) * stepEl, -limEl, limEl);
            for (int i = 0; i < n; ++i) {
                const int k = (b % 2 == 0) ? i : n - 1 - i; // (a raster: each bar back the other way)
                positions_.emplace_back(lo + (k + 0.5) * (hi - lo) / n, el);
            }
        }
        dwellS_ = nominalDwellS();
    }
    framePeriod_ = dwellS_ * static_cast<double>(positions_.size());
}

double Radar::snrDb(double sigma, double range) const noexcept {
    const double lambda = kLight / spec_.frequencyHz;
    const double lfs = 20.0 * std::log10(4.0 * kPi * range / lambda);
    return 10.0 * std::log10(spec_.averagePowerW) + 2.0 * pattern_.peakGainDbi() + 10.0 * std::log10(4.0 * kPi * sigma / (lambda * lambda)) -
           2.0 * lfs + 10.0 * std::log10(nominalDwellS()) - kKT0DbW - spec_.noiseFigureDb - spec_.systemLossDb + spec_.classCalibrationDb;
}

double Radar::rateSpanMs(Mode mode, double ownSpeedMs) const noexcept {
    return (mode == Mode::VelocitySearch ? 1.0 : 2.0) * (ownSpeedMs + kMaxTargetSpeedMs);
}

double Radar::resolutionCells(Mode mode, double dwellS, double rangeScaleM, double ownSpeedMs) const noexcept {
    const int prfs = std::max(1, mode == Mode::VelocitySearch ? spec_.highPrfs : spec_.mediumPrfs);
    const double dv = kLight / spec_.frequencyHz / (2.0 * dwellS / prfs); // (a coherent interval's Doppler resolution)
    return std::max(1.0, rangeScaleM / spec_.rangeResolutionM) * std::max(1.0, rateSpanMs(mode, ownSpeedMs) / dv);
}

double Radar::dwellPfa(Mode mode, double dwellS, double rangeScaleM, double ownSpeedMs) const noexcept {
    // one false alarm in its false-alarm time: dwellS / T of them a dwell, among its cells
    return std::min(0.5, dwellS / (spec_.falseAlarmTimeS * resolutionCells(mode, dwellS, rangeScaleM, ownSpeedMs)));
}

double Radar::detectionIntervalPfa(Mode mode, double dwellPfa) const noexcept {
    const bool high = mode == Mode::VelocitySearch;
    const int n = std::max(1, high ? spec_.highPrfs : spec_.mediumPrfs), m = std::clamp(high ? spec_.highM : spec_.mediumM, 1, n);
    // each interval's false-alarm rate: C(n, m) pfa^m = the dwell's
    double c = 1.0;
    for (int k = 0; k < m; ++k) c *= static_cast<double>(n - k) / static_cast<double>(k + 1);
    return std::min(std::pow(dwellPfa / c, 1.0 / m), 0.5);
}

double Radar::dwellPd(double snrDb, Fluctuation f, Mode mode, double dwellPfa) const noexcept {
    const bool high = mode == Mode::VelocitySearch;
    const int n = std::max(1, high ? spec_.highPrfs : spec_.mediumPrfs), m = std::clamp(high ? spec_.highM : spec_.mediumM, 1, n);
    const double p = probabilityOfDetection(std::pow(10.0, snrDb / 10.0) / n, detectionIntervalPfa(mode, dwellPfa), f);
    // at least m of n
    double total = 0.0, binom = 1.0;
    for (int k = 0; k <= n; ++k) {
        if (k > 0) binom *= static_cast<double>(n - k + 1) / static_cast<double>(k);
        if (k >= m) total += binom * std::pow(p, k) * std::pow(1.0 - p, n - k);
    }
    return std::clamp(total, 0.0, 1.0);
}

double Radar::detectionRangeM(double sigma, double pd, int fluctuation, Mode mode) const noexcept {
    // the whole dwell's SNR that makes its M of N detect with probability pd
    const double pfa = dwellPfa(mode, nominalDwellS(), spec_.instrumentedRangeM, kNominalSpeedMs);
    double lo = -30.0, hi = 60.0;
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        (dwellPd(mid, static_cast<Fluctuation>(fluctuation), mode, pfa) < pd ? lo : hi) = mid;
    }
    const double need = hi;
    // SNR falls 40 dB a decade of range: from 1 km
    return 1000.0 * std::pow(10.0, (snrDb(sigma, 1000.0) - need) / 40.0);
}

void Radar::step(double simTime, const sim::VehicleState& carrier, const Scene& scene) {
    if (!active_ || positions_.empty()) return;
    // the dwells that end by now, each at its beam position (a mechanical antenna's sweep taken a beamwidth at a time)
    const double limAz = spec_.rotationRpm > 0.0 ? kPi : spec_.azimuthLimitDeg * kDeg, limEl = spec_.elevationLimitDeg * kDeg;
    // the carrier at each dwell's own time: its state is the step's, a dwell ends up to a step before it (lag_)
    const sim::VehicleState& now = carrier;
    sim::VehicleState own = now;
    double vNow[3];
    velocityEcef(now, vNow);
    int guard = 0;
    while (clock_ + dwellS_ <= simTime + 1e-12 && guard++ < 100000) {
        clock_ += dwellS_;
        lag_ = std::max(0.0, simTime - clock_);
        for (int i = 0; i < 3; ++i) own.positionEcef[i] = now.positionEcef[i] - vNow[i] * lag_;
        double az = 0.0, el = 0.0;
        if (staring()) {
            // single-target track: the beam where the track says the target is - as far as the antenna reaches
            if (!bearing(designated_, clock_, own, az, el)) return lose();
            az = std::clamp(az, -limAz, limAz), el = std::clamp(el, -limEl, limEl);
        } else if (acquiring_) {
            az = acquireAz_, el = acquireEl_;
        } else {
            if (designated_ && !held(designated_)) return lose(); // (the track its scan was kept on is gone)
            // a scan about a track - the one it follows, or the one a cued search was given: each frame's volume on it
            const std::uint64_t about = designated_ ? designated_ : task_ == Task::Cued ? search_.track : 0;
            double trackAz = 0.0, trackEl = 0.0;
            const bool have = about && bearing(about, clock_, own, trackAz, trackEl);
            if (have && next_ == 0) recentre(trackAz, trackEl);
            if (have && designated_ && spec_.aperture.electronic && clock_ - trackDwellAt_ >= spec_.trackUpdateS - 1e-9) {
                // an electronic array's look at its designated track, between two of its search dwells
                az = std::clamp(trackAz, -limAz, limAz), el = std::clamp(trackEl, -limEl, limEl);
                trackDwellAt_ = clock_;
            } else {
                az = positions_[next_].first, el = positions_[next_].second;
                next_ = (next_ + 1) % positions_.size();
            }
        }
        beamAz_ = az, beamEl_ = el;
        const std::uint64_t firstNew = detectionCount_;
        dwell(clock_, az, el, own, scene);
        ++dwells_;
        if (staring()) {
            if (!held(designated_)) return lose(); // (not updated for its memory time: the tracker let it go)
        } else if (task_ == Task::Cued) {
            acquire(clock_, own, firstNew);
        }
    }
}

void Radar::dwell(double time, double az, double el, const sim::VehicleState& own, const Scene& scene) {
    if (search_.signalLevel && search_.mode != Mode::VelocitySearch) return dwellSignal(time, az, el, own, scene);
    constexpr std::size_t kMax = 512;
    std::uint32_t ids[kMax];
    const sim::VehicleState* states[kMax];
    const std::size_t n = scene.targets(ids, states, kMax);
    double r[9];
    sim::attitude::bodyToEcefFromQuaternion(own.attitudeEcefToBody, r); // ecef = r body
    const double beamBody[3] = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), -std::sin(el)};
    double beam[3];
    for (int i = 0; i < 3; ++i) beam[i] = r[3 * i] * beamBody[0] + r[3 * i + 1] * beamBody[1] + r[3 * i + 2] * beamBody[2];
    double vOwn[3];
    velocityEcef(own, vOwn);
    const double speed = std::sqrt(dot(vOwn, vOwn));
    double north[3], east[3], down[3];
    ned(own.latitudeRad, own.longitudeRad, north, east, down);
    // the main beam's clutter: the ground it reaches within the range scale, and that ground's range rate
    const double depression = std::asin(std::clamp(dot(beam, down), -1.0, 1.0));
    const bool mainBeamClutter = depression > 0.0 && own.altitudeAglM / std::sin(depression) < search_.rangeScaleM;
    const double clutterRate = -dot(vOwn, beam); // (a ground point's range rate along the beam: positive opening)
    const double lambda = kLight / spec_.frequencyHz;
    const double bwAz = pattern_.beamwidthRad(false), bwEl = pattern_.beamwidthRad(true);
    const double sy = beamBody[1], sz = beamBody[2];
    // the dwell's threshold: its false-alarm time's, among its resolution cells
    const double cells = resolutionCells(search_.mode, dwellS_, search_.rangeScaleM, speed);
    const double pfa = dwellPfa(search_.mode, dwellS_, search_.rangeScaleM, speed);
    std::vector<Detection> found;
    // a measurement's errors, as its SNR (linear) gives them (range resolution, monopulse beam splitting, Doppler resolution)
    auto measured = [&](Detection& det, double s, double dwellTime) {
        const double root = std::sqrt(2.0 * std::max(s, 1.0));
        det.sigma[0] = bwEl / (spec_.monopulseSlope * root);
        det.sigma[1] = bwAz / (spec_.monopulseSlope * root);
        det.sigma[2] = spec_.rangeResolutionM / root;
        det.sigma[3] = (lambda / 2.0 / dwellTime) / root;
    };
    for (std::size_t k = 0; k < n; ++k) {
        const sim::VehicleState& t = *states[k];
        double vt[3];
        velocityEcef(t, vt);
        double d[3] = {t.positionEcef[0] - vt[0] * lag_ - own.positionEcef[0], t.positionEcef[1] - vt[1] * lag_ - own.positionEcef[1],
                       t.positionEcef[2] - vt[2] * lag_ - own.positionEcef[2]}; // (where it was at the dwell's time: lag_)
        const double range = std::sqrt(dot(d, d));
        if (!(range > 1.0) || range > search_.rangeScaleM) continue;
        for (double& c : d) c /= range;
        // the target in the body axes; far off the beam: none of its main lobe
        const double db[3] = {r[0] * d[0] + r[3] * d[1] + r[6] * d[2], r[1] * d[0] + r[4] * d[1] + r[7] * d[2], r[2] * d[0] + r[5] * d[1] + r[8] * d[2]};
        const double off = std::acos(std::clamp(dot(db, beamBody), -1.0, 1.0));
        if (off > 3.0 * std::max(bwAz, bwEl)) continue;
        double gain;
        if (spec_.aperture.electronic) {
            gain = pattern_.gainDbi(db, sy, sz);
        } else { // the antenna pointed along the beam: the target in its own frame
            double y[3] = {-beamBody[1], beamBody[0], 0.0};
            const double yl = std::hypot(y[0], y[1]);
            if (yl < 1e-9) y[0] = 0.0, y[1] = 1.0, y[2] = 0.0;
            else y[0] /= yl, y[1] /= yl;
            const double z[3] = {beamBody[1] * y[2] - beamBody[2] * y[1], beamBody[2] * y[0] - beamBody[0] * y[2], beamBody[0] * y[1] - beamBody[1] * y[0]};
            const double face[3] = {dot(db, beamBody), dot(db, y), dot(db, z)};
            gain = pattern_.gainDbi(face);
        }
        double fluct = 1.0;
        const double sigma = scene.sigma(ids[k], spec_.frequencyHz, spec_.polarizationRad, fluct);
        if (!(sigma > 0.0)) continue;
        const double rate = (vt[0] - vOwn[0]) * d[0] + (vt[1] - vOwn[1]) * d[1] + (vt[2] - vOwn[2]) * d[2]; // positive opening
        // pulse-Doppler clutter
        if (mainBeamClutter && std::fabs(rate - clutterRate) < spec_.notchMs) continue;     // notched with the main beam's
        const bool inSidelobeBand = std::fabs(rate) < speed;                                 // the side lobes' band
        if (search_.mode == Mode::VelocitySearch && (inSidelobeBand || rate > 0.0)) continue; // (high PRF: closing, clear of it)
        const double loss = scene.pathLossDb(ids[k], spec_.frequencyHz);
        const double dwell = dwellS_;
        double snr = 10.0 * std::log10(spec_.averagePowerW) + 2.0 * gain + 10.0 * std::log10(4.0 * kPi * sigma / (lambda * lambda)) - 2.0 * loss +
                     10.0 * std::log10(dwell) - kKT0DbW - spec_.noiseFigureDb - spec_.systemLossDb + spec_.classCalibrationDb;
        if (inSidelobeBand && search_.mode != Mode::VelocitySearch) snr -= spec_.sidelobeClutterDb;
        const double s = std::pow(10.0, snr / 10.0);
        const double pd = dwellPd(snr, fluctuationOf(fluct), search_.mode, pfa);
        std::uniform_real_distribution<double> u(0.0, 1.0);
        if (u(rng_) >= pd) continue;
        // measured: errors that shrink with the SNR
        std::normal_distribution<double> g(0.0, 1.0);
        Detection det;
        det.timeS = time;
        det.truth = ids[k];
        measured(det, s, dwell);
        det.rangeM = range + g(rng_) * det.sigma[2];
        det.azimuthRad = std::atan2(db[1], db[0]) + g(rng_) * det.sigma[1];
        det.elevationRad = std::atan2(-db[2], std::hypot(db[0], db[1])) + g(rng_) * det.sigma[0];
        det.rangeRateMs = rate + g(rng_) * det.sigma[3];
        det.snrDb = snr;
        found.push_back(det);
    }
    // false alarms: each of the dwell's resolution cells - its range bins within the range scale, by its Doppler
    // filters over the range rates it searches - crosses the threshold with the dwell's probability (its M of N's)
    {
        const bool high = search_.mode == Mode::VelocitySearch;
        const double span = rateSpanMs(search_.mode, speed);
        std::poisson_distribution<int> count(cells * pfa);
        std::uniform_real_distribution<double> u(0.0, 1.0);
        const int alarms = count(rng_);
        const double threshold = 10.0 * std::log10(-std::log(pfa)); // (noise over the threshold: an SNR at it)
        for (int a = 0; a < alarms; ++a) {
            Detection det;
            det.timeS = time;
            det.truth = 0;
            det.rangeM = u(rng_) * search_.rangeScaleM;
            det.azimuthRad = az + (u(rng_) - 0.5) * bwAz;
            det.elevationRad = el + (u(rng_) - 0.5) * bwEl;
            det.rangeRateMs = high ? -u(rng_) * span : (u(rng_) - 0.5) * span;
            det.snrDb = threshold;
            measured(det, std::pow(10.0, threshold / 10.0), dwellS_);
            found.push_back(det);
            ++falseAlarms_;
        }
    }
    finish(time, az, el, own, r, vOwn, found);
}

void Radar::finish(double time, double az, double el, const sim::VehicleState& own, const double r[9], const double vOwn[3],
                   std::vector<Detection>& found) {
    for (Detection& det : found) {
        det.serial = detectionCount_++;
        const double mb[3] = {std::cos(det.elevationRad) * std::cos(det.azimuthRad), std::cos(det.elevationRad) * std::sin(det.azimuthRad),
                              -std::sin(det.elevationRad)};
        double p[3];
        for (int i = 0; i < 3; ++i) p[i] = own.positionEcef[i] + det.rangeM * (r[3 * i] * mb[0] + r[3 * i + 1] * mb[1] + r[3 * i + 2] * mb[2]);
        geodetic(p, det.latitudeRad, det.longitudeRad, det.heightM);
        detections_.push_back(det);
        if (detections_.size() > kKeep) detections_.pop_front();
    }
    if (search_.mode == Mode::VelocitySearch) return; // (high PRF: its range ambiguous - no tracks)
    track(time, az, el, own, r, vOwn, found);
}

void Radar::dwellSignal(double time, double az, double el, const sim::VehicleState& own, const Scene& scene) {
    constexpr std::size_t kMax = 512;
    std::uint32_t ids[kMax];
    const sim::VehicleState* states[kMax];
    const std::size_t n = scene.targets(ids, states, kMax);
    double r[9];
    sim::attitude::bodyToEcefFromQuaternion(own.attitudeEcefToBody, r); // ecef = r body
    const double beamBody[3] = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), -std::sin(el)};
    double vOwn[3];
    velocityEcef(own, vOwn);
    const double speed = std::sqrt(dot(vOwn, vOwn));
    const double lambda = kLight / spec_.frequencyHz;
    const double bwAz = pattern_.beamwidthRad(false), bwEl = pattern_.beamwidthRad(true);
    const double sy = beamBody[1], sz = beamBody[2];
    // the targets as the dwell's samples carry them: their SNR the radar equation's (as the detection level's, without
    // its clutter penalties: the signal level's clutter is its own), their monopulse ratios from their offset
    std::vector<signal::Echo> echoes;
    for (std::size_t k = 0; k < n; ++k) {
        const sim::VehicleState& t = *states[k];
        double vt[3];
        velocityEcef(t, vt);
        double d[3] = {t.positionEcef[0] - vt[0] * lag_ - own.positionEcef[0], t.positionEcef[1] - vt[1] * lag_ - own.positionEcef[1],
                       t.positionEcef[2] - vt[2] * lag_ - own.positionEcef[2]}; // (where it was at the dwell's time: lag_)
        const double range = std::sqrt(dot(d, d));
        if (!(range > 1.0)) continue;
        for (double& c : d) c /= range;
        const double db[3] = {r[0] * d[0] + r[3] * d[1] + r[6] * d[2], r[1] * d[0] + r[4] * d[1] + r[7] * d[2], r[2] * d[0] + r[5] * d[1] + r[8] * d[2]};
        const double off = std::acos(std::clamp(dot(db, beamBody), -1.0, 1.0));
        if (off > 3.0 * std::max(bwAz, bwEl)) continue;
        double gain;
        if (spec_.aperture.electronic) {
            gain = pattern_.gainDbi(db, sy, sz);
        } else {
            double y[3] = {-beamBody[1], beamBody[0], 0.0};
            const double yl = std::hypot(y[0], y[1]);
            if (yl < 1e-9) y[0] = 0.0, y[1] = 1.0, y[2] = 0.0;
            else y[0] /= yl, y[1] /= yl;
            const double z[3] = {beamBody[1] * y[2] - beamBody[2] * y[1], beamBody[2] * y[0] - beamBody[0] * y[2], beamBody[0] * y[1] - beamBody[1] * y[0]};
            const double face[3] = {dot(db, beamBody), dot(db, y), dot(db, z)};
            gain = pattern_.gainDbi(face);
        }
        double fluct = 1.0;
        const double sigma = scene.sigma(ids[k], spec_.frequencyHz, spec_.polarizationRad, fluct);
        if (!(sigma > 0.0)) continue;
        const double loss = scene.pathLossDb(ids[k], spec_.frequencyHz);
        const double snr =10.0 * std::log10(spec_.averagePowerW) + 2.0 * gain + 10.0 * std::log10(4.0 * kPi * sigma / (lambda * lambda)) - 2.0 * loss +
                           10.0 * std::log10(dwellS_) - kKT0DbW - spec_.noiseFigureDb - spec_.systemLossDb + spec_.classCalibrationDb;
        signal::Echo e;
        e.rangeM = range;
        e.rangeRateMs = (vt[0] - vOwn[0]) * d[0] + (vt[1] - vOwn[1]) * d[1] + (vt[2] - vOwn[2]) * d[2];
        e.snr = std::pow(10.0, snr / 10.0);
        const Fluctuation f = fluctuationOf(fluct);
        e.fluctuation = f == Fluctuation::Steady ? 0 : f == Fluctuation::Rayleigh ? 1 : 2;
        double dAz = std::atan2(db[1], db[0]) - az;
        if (spec_.rotationRpm > 0.0) dAz = std::remainder(dAz, 2.0 * kPi);
        const double dEl = std::atan2(-db[2], std::hypot(db[0], db[1])) - el;
        e.monopulseAz = spec_.monopulseSlope * dAz / bwAz;
        e.monopulseEl = spec_.monopulseSlope * dEl / bwEl;
        e.truth = ids[k];
        echoes.push_back(e);
    }
    signal::Settings set;
    const int intervals = static_cast<int>(waveform_.intervals.size());
    // the interval threshold for the false-alarm time: its peak survival (a simulation, ~20 ms) kept while the
    // threshold stays within 20 % of where it was found - it changes slowly with it
    const double pfaDwell = dwellPfa(search_.mode, dwellS_, search_.rangeScaleM, speed);
    double p = signalQ_ > 0.0 ? signal::intervalPfa(pfaDwell, waveform_, set.coincidence, signalQ_) : 0.0;
    if (!(signalQ_ > 0.0) || std::fabs(p / signalQAt_ - 1.0) > 0.2) {
        p = signal::intervalPfa(pfaDwell, waveform_, set.coincidence, set.cfarCells);
        signalQ_ = signal::peakSurvival(p, set.cfarCells), signalQAt_ = p;
        for (double& l : signalLossDb_) l = std::numeric_limits<double>::quiet_NaN();
    }
    // the detection level's SNR is the calibrated one, its losses (processing among them) lumped: the chain takes its
    // own processing loss look by look, so it is given back its mean for the target's fluctuation - against the
    // detection level as it detects, at its own interval threshold (it counts the PRFs' false agreements as C(n, m)
    // alone; the chain's, designed for the same false-alarm time, is the higher: every way they agree, the gates lost
    // while transmitting, the peaks' share) - so on average both levels detect alike
    auto lossDb = [&](int fluctuation) {
        double& l = signalLossDb_[static_cast<std::size_t>(std::clamp(fluctuation, 0, 2))];
        if (std::isnan(l)) l = signal::processingLossDb(waveform_, p, set.cfarCells, fluctuation, detectionIntervalPfa(search_.mode, pfaDwell));
        return l;
    };
    for (signal::Echo& e : echoes) e.snr *= std::pow(10.0, lossDb(e.fluctuation) / 10.0);
    // the ground's clutter, through the same radar equation and the same calibration
    const double scaleDb = 10.0 * std::log10(spec_.averagePowerW) + 10.0 * std::log10(dwellS_) - kKT0DbW - spec_.noiseFigureDb - spec_.systemLossDb +
                           spec_.classCalibrationDb + lossDb(1); // (clutter: many scatterers, Rayleigh)
    if (scene.hasGround()) {
        const auto t0 = std::chrono::steady_clock::now();
        clutterMap(own, r, vOwn, beamBody, scene, scaleDb, clutter_, set.mainBeamRateMs, set.altitudeM);
        signalStats_.clutterS += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        set.notchMs = spec_.notchMs;
    } else {
        clutter_.clear();
    }
    set.pfaInterval = p;
    set.rangeScaleM = search_.rangeScaleM;
    set.maxRateMs = rateSpanMs(search_.mode, speed) / 2.0, set.minRateMs = -set.maxRateMs;
    const auto hits = signal::dwell(waveform_, echoes, set, signalStream_, &signalStats_, &clutter_, &snapshot_);
    snapshotTimeS_ = time;
    // the processor's detections, measured as it measured them
    std::vector<Detection> found;
    found.reserve(hits.size());
    for (const auto& h : hits) {
        Detection det;
        det.timeS = time;
        det.truth = h.truth;
        det.rangeM = h.rangeM;
        det.rangeRateMs = h.rangeRateMs;
        const double mAz = std::clamp(h.monopulseAz / spec_.monopulseSlope, -1.0, 1.0), mEl = std::clamp(h.monopulseEl / spec_.monopulseSlope, -1.0, 1.0);
        det.azimuthRad = az + mAz * bwAz;
        det.elevationRad = el + mEl * bwEl;
        det.snrDb = 10.0 * std::log10(std::max(h.snr, 1e-3));
        const double root = std::sqrt(2.0 * std::max(h.snr, 1.0));
        det.sigma[0] = bwEl / (spec_.monopulseSlope * root);
        det.sigma[1] = bwAz / (spec_.monopulseSlope * root);
        det.sigma[2] = spec_.rangeResolutionM / root;
        det.sigma[3] = (lambda / 2.0 / (dwellS_ / std::max(1, intervals))) / root;
        if (h.truth == 0) ++falseAlarms_;
        found.push_back(det);
    }
    finish(time, az, el, own, r, vOwn, found);
}

void Radar::clutterMap(const sim::VehicleState& own, const double r[9], const double vOwn[3], const double beamBody[3], const Scene& scene,
                       double scaleDb, signal::ClutterMap& out, double& mainBeamRateMs, double& altitudeM) const {
    const int n = static_cast<int>(waveform_.intervals.size());
    out.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const signal::Interval& iv = waveform_.intervals[static_cast<std::size_t>(i)];
        out[static_cast<std::size_t>(i)].assign(static_cast<std::size_t>(std::max(0, iv.gates - iv.blankedGates)) * static_cast<std::size_t>(iv.pulses), 0.0);
    }
    const double lambda = kLight / spec_.frequencyHz;
    const double bwAz = pattern_.beamwidthRad(false), bwEl = pattern_.beamwidthRad(true);
    // the local frame, the ground below and the sphere it lies on (4/3 the Earth's radius: refraction)
    double north[3], east[3], down[3];
    ned(own.latitudeRad, own.longitudeRad, north, east, down);
    const double below = scene.surfaceHeightM(own.latitudeRad, own.longitudeRad);
    const double h = std::max(1.0, std::isfinite(below) ? own.altitudeMslM - std::max(below, 0.0) : own.altitudeAglM);
    altitudeM = h;
    const double a = 4.0 / 3.0 * 6371e3;
    const double horizon = std::sqrt((a + h) * (a + h) - a * a);
    const double rMax = std::min({horizon, kClutterRangeM});
    // the beam in the local frame
    double beam[3];
    for (int i = 0; i < 3; ++i) beam[i] = r[3 * i] * beamBody[0] + r[3 * i + 1] * beamBody[1] + r[3 * i + 2] * beamBody[2];
    const double beamAz = std::atan2(dot(beam, east), dot(beam, north));
    const double beamDep = std::asin(std::clamp(dot(beam, down), -1.0, 1.0));
    mainBeamRateMs = beamDep > -bwEl ? -dot(vOwn, beam) : std::numeric_limits<double>::quiet_NaN();
    const double sy = beamBody[1], sz = beamBody[2];
    auto gain2 = [&](const double db[3]) { // the two-way gain (linear) towards a body direction, the beam as it points now
        double g;
        if (spec_.aperture.electronic) {
            g = pattern_.gain(db, sy, sz);
        } else {
            double y[3] = {-beamBody[1], beamBody[0], 0.0};
            const double yl = std::hypot(y[0], y[1]);
            if (yl < 1e-9) y[0] = 0.0, y[1] = 1.0, y[2] = 0.0;
            else y[0] /= yl, y[1] /= yl;
            const double z[3] = {beamBody[1] * y[2] - beamBody[2] * y[1], beamBody[2] * y[0] - beamBody[0] * y[2], beamBody[0] * y[1] - beamBody[1] * y[0]};
            const double face[3] = {dot(db, beamBody), dot(db, y), dot(db, z)};
            g = pattern_.gain(face);
        }
        return g * g;
    };
    const double radarScale = std::pow(10.0, scaleDb / 10.0);
    const double freqDb = 5.0 * std::log10(spec_.frequencyHz / 10e9);
    const double gammaLand = std::pow(10.0, (kGammaLandDb + freqDb) / 10.0), gammaSea = std::pow(10.0, (kGammaSeaDb + freqDb) / 10.0);
    const bool seaBelow = std::isfinite(below) && below <= 0.0;
    const double gate = waveform_.gateM;
    thread_local std::vector<double> profile; // a ring's power over each interval's filters, before its gates
    // the rings: a gate wide out to 40 gates past the altitude, then 1.5 % of their range (the ring's power falls as R^-4)
    for (double r0 = h; r0 < rMax;) {
        const double dr = r0 < h + 40.0 * gate ? gate : std::max(gate, 0.015 * r0);
        const double rc = std::min(r0 + 0.5 * dr, rMax);
        r0 += dr;
        const double sinEps = std::clamp(((a + h) * (a + h) + rc * rc - a * a) / (2.0 * (a + h) * rc), -1.0, 1.0);
        const double sinPsi = ((a + h) * (a + h) - rc * rc - a * a) / (2.0 * a * rc);
        if (!(sinPsi > 0.0)) break; // (past the horizon)
        const double cosPsi = std::sqrt(std::max(1e-9, 1.0 - sinPsi * sinPsi));
        const double cosBeta = std::clamp(((a + h) * (a + h) + a * a - rc * rc) / (2.0 * a * (a + h)), -1.0, 1.0);
        const double radius = a * std::sqrt(std::max(0.0, 1.0 - cosBeta * cosBeta)); // (the ring's radius on the ground)
        const double eps = std::asin(sinEps), cosEps = std::sqrt(std::max(0.0, 1.0 - sinEps * sinEps));
        // its azimuths: every 7.5 deg, and a quarter beamwidth apart in the main beam's footprint
        const bool footprint = std::fabs(eps - beamDep) < 3.0 * bwEl;
        const double fineHalf = footprint ? std::min(kPi, 3.0 * bwAz / std::max(0.05, cosEps)) : 0.0;
        double gamma = seaBelow ? gammaSea : gammaLand;
        if (footprint) { // the ground the main beam meets: land or sea, there
            double u[3], p[3];
            for (int i = 0; i < 3; ++i) u[i] = cosEps * (std::cos(beamAz) * north[i] + std::sin(beamAz) * east[i]) + sinEps * down[i];
            for (int i = 0; i < 3; ++i) p[i] = own.positionEcef[i] + rc * u[i];
            double lat, lon, hh;
            geodetic(p, lat, lon, hh);
            const double s = scene.surfaceHeightM(lat, lon);
            if (std::isfinite(s)) gamma = s <= 0.0 ? gammaSea : gammaLand;
        }
        const double sigma0 = gamma * sinPsi;
        // a patch's power a pulse in each interval (over the noise), but for its two-way gain and its azimuth width:
        // the radar equation's terms, the ring's reflectivity and area per radian, its range's fourth power
        const double ringFactor = radarScale * 4.0 * kPi * sigma0 * radius * dr / cosPsi / (lambda * lambda) / std::pow(4.0 * kPi * rc / lambda, 4) / n;
        // the ring's patches, each its power into each interval's filters (its gates come after): spread evenly over
        // the Doppler from one edge of the patch to the other (a patch is many scatterers, not a line) - its unfolded
        // filters a difference array (the filters it covers whole) and its two ends, folded into the PRF's once a ring
        std::size_t width = 0;
        for (const auto& iv : waveform_.intervals) width += static_cast<std::size_t>(iv.pulses);
        profile.assign(width, 0.0);
        thread_local std::vector<int> lo, hi, reach; // (each interval's filters the ring touched; its unfolded half-width)
        thread_local std::vector<std::vector<double>> diff, ends;
        lo.assign(static_cast<std::size_t>(n), 1 << 30), hi.assign(static_cast<std::size_t>(n), -1);
        reach.resize(static_cast<std::size_t>(n));
        diff.resize(static_cast<std::size_t>(n)), ends.resize(static_cast<std::size_t>(n));
        const double speed = std::sqrt(dot(vOwn, vOwn));
        for (int i = 0; i < n; ++i) {
            const signal::Interval& iv = waveform_.intervals[static_cast<std::size_t>(i)];
            reach[static_cast<std::size_t>(i)] = static_cast<int>(std::ceil(2.0 * speed / lambda / iv.prfHz * iv.pulses)) + 2;
            const std::size_t size = static_cast<std::size_t>(2 * reach[static_cast<std::size_t>(i)] + 3);
            diff[static_cast<std::size_t>(i)].assign(size, 0.0), ends[static_cast<std::size_t>(i)].assign(size, 0.0);
        }
        auto rateAt = [&](double phi) {
            double ue[3];
            for (int i = 0; i < 3; ++i) ue[i] = cosEps * (std::cos(phi) * north[i] + std::sin(phi) * east[i]) + sinEps * down[i];
            return -dot(vOwn, ue);
        };
        bool any = false;
        auto patch = [&](double phi0, double phi1, double rate0, double rate1) {
            const double phi = 0.5 * (phi0 + phi1), dphi = phi1 - phi0;
            double u[3];
            for (int i = 0; i < 3; ++i) u[i] = cosEps * (std::cos(phi) * north[i] + std::sin(phi) * east[i]) + sinEps * down[i];
            const double db[3] = {r[0] * u[0] + r[3] * u[1] + r[6] * u[2], r[1] * u[0] + r[4] * u[1] + r[7] * u[2], r[2] * u[0] + r[5] * u[1] + r[8] * u[2]};
            const double p = ringFactor * dphi * gain2(db); // (each interval 1/n of the dwell's energy)
            if (p < 1e-3) return; // (30 dB below the noise a pulse, and spread: nothing a cell would show)
            any = true;
            for (int i = 0; i < n; ++i) {
                const signal::Interval& iv = waveform_.intervals[static_cast<std::size_t>(i)];
                const int m = iv.pulses, x = reach[static_cast<std::size_t>(i)];
                const double toFilters = -2.0 / lambda / iv.prfHz * m;
                double x0 = rate0 * toFilters, x1 = rate1 * toFilters;
                if (x0 > x1) std::swap(x0, x1);
                x0 = std::clamp(x0, -x + 1.0, x - 1.0), x1 = std::clamp(x1, -x + 1.0, x - 1.0);
                // an unwindowed filter's power for a tone of per-pulse power p/m: m p (m^2 (p / m)), shared by overlap
                const double total = m * p, span = x1 - x0;
                auto& D = diff[static_cast<std::size_t>(i)];
                auto& E = ends[static_cast<std::size_t>(i)];
                const int f0 = static_cast<int>(std::floor(x0)), f1 = static_cast<int>(std::floor(x1));
                if (span < 1e-6) { // (a line: between the two filters about it)
                    const double f = x0 - f0;
                    E[static_cast<std::size_t>(f0 + x)] += total * (1.0 - f);
                    E[static_cast<std::size_t>(f0 + 1 + x)] += total * f;
                } else if (f0 == f1) {
                    E[static_cast<std::size_t>(f0 + x)] += total;
                } else {
                    const double d = total / span;
                    E[static_cast<std::size_t>(f0 + x)] += d * (f0 + 1 - x0); // (its first filter, in part)
                    E[static_cast<std::size_t>(f1 + x)] += d * (x1 - f1);     // (its last, in part)
                    if (f0 + 1 <= f1 - 1) {                                      // (those between, whole)
                        D[static_cast<std::size_t>(f0 + 1 + x)] += d;
                        D[static_cast<std::size_t>(f1 + x)] -= d;
                    }
                }
            }
        };
        if (fineHalf > 0.0) {
            // a quarter beamwidth apart within 1.5 beamwidths of the beam's azimuth (its main lobe), half a beamwidth out
            // to 3 (its near side lobes)
            const double scaleAz = 1.0 / std::max(0.05, cosEps);
            const double inner = std::min(fineHalf, 1.5 * bwAz * scaleAz);
            double phi0 = beamAz - fineHalf, rate0 = rateAt(phi0);
            auto run = [&](double to, double step) {
                const int count = std::max(1, static_cast<int>(std::ceil((to - phi0) / step - 1e-9)));
                const double d = (to - phi0) / count;
                const double from = phi0;
                for (int j = 0; j < count; ++j) {
                    const double phi1 = from + (j + 1) * d, rate1 = rateAt(phi1);
                    patch(phi0, phi1, rate0, rate1);
                    phi0 = phi1, rate0 = rate1;
                }
            };
            if (beamAz - inner > phi0 + 1e-12) run(beamAz - inner, 0.5 * bwAz * scaleAz);
            run(beamAz + inner, 0.25 * bwAz * scaleAz);
            if (beamAz + fineHalf > phi0 + 1e-12) run(beamAz + fineHalf, 0.5 * bwAz * scaleAz);
            // the rest of the ring, coarse
            const double rest = 2.0 * kPi - 2.0 * fineHalf;
            if (rest > 1e-6) {
                const int coarse = std::max(1, static_cast<int>(std::ceil(rest / (7.5 * kDeg))));
                for (int j = 0; j < coarse; ++j) {
                    const double phi1 = beamAz + fineHalf + (j + 1) * rest / coarse, rate1 = rateAt(phi1);
                    patch(phi0, phi1, rate0, rate1);
                    phi0 = phi1, rate0 = rate1;
                }
            }
        } else {
            double phi0 = 0.0, rate0 = rateAt(0.0);
            for (int j = 0; j < 48; ++j) {
                const double phi1 = (j + 1) * 2.0 * kPi / 48.0, rate1 = rateAt(phi1);
                patch(phi0, phi1, rate0, rate1);
                phi0 = phi1, rate0 = rate1;
            }
        }
        if (!any) continue; // (the ring is below the noise everywhere)
        // each interval's unfolded filters summed and folded into its PRF's
        {
            std::size_t base = 0;
            for (int i = 0; i < n; ++i) {
                const signal::Interval& iv = waveform_.intervals[static_cast<std::size_t>(i)];
                const int m = iv.pulses, x = reach[static_cast<std::size_t>(i)];
                auto& D = diff[static_cast<std::size_t>(i)];
                auto& E = ends[static_cast<std::size_t>(i)];
                double run = 0.0;
                for (std::size_t j = 0; j < D.size(); ++j) {
                    run += D[j];
                    const double v = run + E[j];
                    if (v != 0.0) {
                        const int unfolded = static_cast<int>(j) - x;
                        const int k = ((unfolded % m) + m) % m;
                        profile[base + static_cast<std::size_t>(k)] += v;
                        lo[static_cast<std::size_t>(i)] = std::min(lo[static_cast<std::size_t>(i)], k), hi[static_cast<std::size_t>(i)] = std::max(hi[static_cast<std::size_t>(i)], k);
                    }
                }
                base += static_cast<std::size_t>(m);
            }
        }
        // the ring's gates in each interval: its range folded, a gate at a time across its width, eclipsed where it
        // meets a transmission
        const int sub = std::max(1, static_cast<int>(std::lround(dr / gate)));
        std::size_t base = 0;
        for (int i = 0; i < n; ++i) {
            const signal::Interval& iv = waveform_.intervals[static_cast<std::size_t>(i)];
            const int m = iv.pulses, g0 = iv.blankedGates, rows = iv.gates - g0;
            auto& map = out[static_cast<std::size_t>(i)];
            const int k0 = lo[static_cast<std::size_t>(i)], k1 = hi[static_cast<std::size_t>(i)];
            if (k1 < k0) {
                base += static_cast<std::size_t>(m);
                continue; // (the ring left nothing in this interval)
            }
            const double chips = iv.pulseS * kLight / 2.0 / gate, period = iv.ambiguousRangeM / gate;
            for (int s = 0; s < sub; ++s) {
                const double range = rc - 0.5 * dr + (s + 0.5) * dr / sub;
                const double folded = std::fmod(range, iv.ambiguousRangeM);
                const double x = folded / gate;
                const int g = static_cast<int>(std::floor(x));
                const int gs[2] = {g - g0, g + 1 - g0};
                for (int t = 0; t < 2; ++t) {
                    if (gs[t] < 0 || gs[t] >= rows) continue;
                    const double response = signal::gateResponse(x, g + t, chips, period); // (straddle and eclipse)
                    if (!(response > 0.0)) continue;
                    const double scale = response * response / sub; // (power, for clutter)
                    double* row = map.data() + static_cast<std::size_t>(gs[t]) * static_cast<std::size_t>(m);
                    for (int k = k0; k <= k1; ++k) row[k] += scale * profile[base + static_cast<std::size_t>(k)];
                }
            }
            base += static_cast<std::size_t>(m);
        }
    }
}

void Radar::track(double time, double az, double el, const sim::VehicleState& own, const double r[9], const double vOwn[3],
                  const std::vector<Detection>& found) {
    // the dwell's detections as the tracker measures them: elevation, bearing (Stone Soup's: left positive) and range in
    // the sensor's frame - the body's x forward, y left, z up - from its pose at the dwell, in the ECEF frame
    track::RadarMeasurement model;
    const double m[3] = {1.0, -1.0, -1.0};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) model.rotation(i, j) = m[i] * r[3 * j + i]; // (body = R^T ecef)
    for (std::size_t i = 0; i < 3; ++i) model.translation[i] = own.positionEcef[i], model.velocity[i] = vOwn[i];
    std::vector<track::Detection> dets;
    dets.reserve(found.size());
    for (const Detection& d : found) {
        track::Detection t;
        t.t = time;
        t.z.a = {d.elevationRad, -d.azimuthRad, d.rangeM, d.rangeRateMs};
        t.model = model;
        for (std::size_t i = 0; i < 4; ++i) t.model.noise(i, i) = d.sigma[i] * d.sigma[i];
        t.id = d.serial;
        t.truth = d.truth;
        dets.push_back(t);
    }
    // which tracks this dwell looked at: predicted inside its half-power beam (a detection expected), or near enough
    // for a detection of them (the main lobe's skirt, the next beam position's overlap)
    const double bwAz = pattern_.beamwidthRad(false), bwEl = pattern_.beamwidthRad(true);
    const bool rotodome = spec_.rotationRpm > 0.0;
    // (first a cone about the beam's axis: within 1.5 beamwidths in azimuth and in elevation is within 1.5 (bw_az + bw_el)
    // of it - an arc along each, no shorter than the great circle - so a track outside that cone is outside the test)
    const double beamBody[3] = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), -std::sin(el)};
    double beam[3];
    for (std::size_t i = 0; i < 3; ++i) beam[i] = r[3 * i] * beamBody[0] + r[3 * i + 1] * beamBody[1] + r[3 * i + 2] * beamBody[2];
    const double cone = std::cos(std::min(kPi, 1.5 * (bwAz + bwEl) + 1e-9));
    const bool byTime = staring() || acquiring_;
    auto looked = [&](const track::Track& tr) {
        const track::Gaussian& g = tr.state();
        const double dt = time - g.t;
        double d[3];
        for (std::size_t i = 0; i < 3; ++i) d[i] = g.x.a[2 * i] + g.x.a[2 * i + 1] * dt - own.positionEcef[i];
        const double range = std::sqrt(dot(d, d));
        if (!(range > 1.0) || range > search_.rangeScaleM) return track::MultiTargetTracker::Not;
        if (dot(d, beam) < cone * range) return track::MultiTargetTracker::Not;
        const double db[3] = {(r[0] * d[0] + r[3] * d[1] + r[6] * d[2]) / range, (r[1] * d[0] + r[4] * d[1] + r[7] * d[2]) / range,
                              (r[2] * d[0] + r[5] * d[1] + r[8] * d[2]) / range};
        double dAz = std::atan2(db[1], db[0]) - az;
        if (rotodome) dAz = std::remainder(dAz, 2.0 * kPi);
        const double dEl = std::atan2(-db[2], std::hypot(db[0], db[1])) - el;
        const double off = std::hypot(dAz / bwAz, dEl / bwEl); // (in beamwidths)
        // (a dwell after a dwell on one place - single-target track, an acquisition's looks: a confirmed track's miss is
        // not counted, it goes when it has not been updated for the memory time. One not yet confirmed is held to its
        // looks as in a scan: a stray detection's is gone two dwells on, not seconds)
        if (off <= 0.5 && (!byTime || static_cast<int>(tr.updates) < spec_.trackConfirm)) return track::MultiTargetTracker::Expected;
        return off <= 1.5 ? track::MultiTargetTracker::Possible : track::MultiTargetTracker::Not;
    };
    tracker_.step(time, dets, looked);
}

} // namespace fsim::ms::radar
