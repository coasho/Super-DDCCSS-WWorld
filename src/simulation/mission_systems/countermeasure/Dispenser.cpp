#include "simulation/mission_systems/countermeasure/Dispenser.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace fsim::ms::countermeasure {
namespace {

constexpr double kGravityMs2 = 9.80665;
constexpr double kSeaLevelRho = 1.225;
constexpr double kGasConstAir = 287.05287;
constexpr double kLapseRateKM = 0.0065;
constexpr double kTropopauseM = 11000.0;
constexpr double kMaxSubStepS = 0.008;
constexpr double kMinSubStepS = 0.0002;
constexpr double kTargetStiffnessStep = 0.04; // keeps RK4 local truncation error (rate*h)^5/120 < 1e-9 even at Mach 2+

bool finite(Point p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool normalize(Point& p) noexcept {
    if (!finite(p)) return false;
    const double scale = std::max({std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
    if (scale == 0.0) return false;
    p.x /= scale;
    p.y /= scale;
    p.z /= scale;
    const double length = std::hypot(p.x, p.y, p.z);
    if (!(length > 0.0) || !std::isfinite(length)) return false;
    p.x /= length;
    p.y /= length;
    p.z /= length;
    return true;
}

Spec sanitizeSpec(Spec s) noexcept {
    auto pos = [](double v, double def, double lo, double hi) {
        return (std::isfinite(v) && v >= lo && v <= hi) ? v : def;
    };
    Spec d;
    s.minIntervalS = pos(s.minIntervalS, d.minIntervalS, 0.01, 10.0);
    s.flareLifeS = pos(s.flareLifeS, d.flareLifeS, 0.5, 30.0);
    s.flareRiseS = pos(s.flareRiseS, d.flareRiseS, 0.02, s.flareLifeS * 0.5);
    s.flareMassKg = pos(s.flareMassKg, d.flareMassKg, 0.05, 5.0);
    s.flareBurnedMassKg = pos(s.flareBurnedMassKg, d.flareBurnedMassKg, 0.01, s.flareMassKg);
    s.flareDragAreaM2 = pos(s.flareDragAreaM2, d.flareDragAreaM2, 1e-4, 0.5);
    s.flarePeakIntensityWSr = pos(s.flarePeakIntensityWSr, d.flarePeakIntensityWSr, 100.0, 200000.0);
    s.flarePeakTemperatureK = pos(s.flarePeakTemperatureK, d.flarePeakTemperatureK, 800.0, 4000.0);
    s.flareCoreRadiusM = pos(s.flareCoreRadiusM, d.flareCoreRadiusM, 0.05, 2.0);
    s.flareEjectSpeedMs = pos(s.flareEjectSpeedMs, d.flareEjectSpeedMs, 2.0, 100.0);
    s.chaffLifeS = pos(s.chaffLifeS, d.chaffLifeS, 1.0, 180.0);
    s.chaffBloomTimeS = pos(s.chaffBloomTimeS, d.chaffBloomTimeS, 0.1, 15.0);
    s.chaffBloomRadiusM = pos(s.chaffBloomRadiusM, d.chaffBloomRadiusM, 1.0, 60.0);
    s.chaffPeakRcsM2 = pos(s.chaffPeakRcsM2, d.chaffPeakRcsM2, 0.5, 500.0);
    s.chaffCenterFreqHz = pos(s.chaffCenterFreqHz, d.chaffCenterFreqHz, 1e8, 100e9);
    s.chaffDragCoeffM2Kg = pos(s.chaffDragCoeffM2Kg, d.chaffDragCoeffM2Kg, 0.005, 5.0);
    s.chaffTerminalSinkMs = pos(s.chaffTerminalSinkMs, d.chaffTerminalSinkMs, 0.1, 10.0);
    s.chaffEjectSpeedMs = pos(s.chaffEjectSpeedMs, d.chaffEjectSpeedMs, 2.0, 100.0);
    return s;
}

bool valid(Motion& motion) noexcept {
    return finite(motion.airVelocity) &&
           std::isfinite(motion.airDensityKgM3) && motion.airDensityKgM3 > 1e-6 && motion.airDensityKgM3 < 20.0 &&
           normalize(motion.down);
}

Point addScaled(Point a, Point b, double s) noexcept {
    return {a.x + b.x * s, a.y + b.y * s, a.z + b.z * s};
}

/// Instantaneous aerodynamic + gravitational acceleration of a flare cartridge or chaff dipole cloud.
/// Strictly Galilean invariant in uniform wind `motion.airVelocity`.
/// Also outputs the instantaneous local damping stiffness rate (1/s) for RK4 sub-step sizing.
bool acceleration(const Decoy& decoy, const Motion& motion, double ageS, Point vel, Point& outAcc, double* outRate = nullptr) noexcept {
    if (!finite(vel)) return false;
    const double rho = motion.airDensityKgM3;
    const double rx = vel.x - motion.airVelocity.x;
    const double ry = vel.y - motion.airVelocity.y;
    const double rz = vel.z - motion.airVelocity.z;
    if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) return false;

    if (decoy.kind == Kind::Flare) {
        // Tumbling pyrotechnic MTV cartridge: quadratic aerodynamic drag with burning mass depletion.
        const double frac = std::clamp(ageS / std::max(decoy.lifeS, 1e-3), 0.0, 1.0);
        const double massKg = std::max(decoy.spec.flareBurnedMassKg,
                                       decoy.spec.flareMassKg + (decoy.spec.flareBurnedMassKg - decoy.spec.flareMassKg) * frac);
        const double speedRel = std::hypot(rx, ry, rz);
        if (!std::isfinite(speedRel)) return false;
        const double kQuad = 0.5 * rho * decoy.spec.flareDragAreaM2 / massKg;
        const double dragRate = kQuad * speedRel;
        if (!std::isfinite(dragRate)) return false;
        if (outRate) *outRate = 2.0 * dragRate; // Jacobian spectral radius of quadratic drag is 2 * kQuad * |v_rel|
        outAcc = {
            kGravityMs2 * motion.down.x - dragRate * rx,
            kGravityMs2 * motion.down.y - dragRate * ry,
            kGravityMs2 * motion.down.z - dragRate * rz,
        };
    } else {
        // Al-coated glass chaff dipoles: rapid quadratic deceleration to local wind velocity
        // combined with physical altitude-scaled terminal settling speed v_sink * sqrt(rho0/rho).
        const double rhoRatio = rho / kSeaLevelRho;
        const double vSink = decoy.spec.chaffTerminalSinkMs / std::sqrt(std::max(rhoRatio, 1e-3));
        const double eqX = vSink * motion.down.x;
        const double eqY = vSink * motion.down.y;
        const double eqZ = vSink * motion.down.z;
        const double ux = rx - eqX;
        const double uy = ry - eqY;
        const double uz = rz - eqZ;
        const double uMag = std::hypot(ux, uy, uz);
        if (!std::isfinite(uMag)) return false;
        // At u = -v_eq (released from rest in still air): linear term equals exact gravity g*down;
        // at high release speed: quadratic term decelerates dipole burst to wind speed in ~0.2 s!
        const double linRate = kGravityMs2 / std::max(vSink, 0.05);
        const double quadRate = rhoRatio * decoy.spec.chaffDragCoeffM2Kg * uMag;
        const double rate = linRate + quadRate;
        if (!std::isfinite(rate)) return false;
        if (outRate) *outRate = linRate + 2.0 * quadRate;
        outAcc = {-rate * ux, -rate * uy, -rate * uz};
    }
    return finite(outAcc);
}

/// 4th-order Runge-Kutta stiffness-aware sub-stepped propagation over `dt` starting at `startAgeS`.
bool propagate(const Decoy& before, const Motion& motion, double startAgeS, double dt, Decoy& after) noexcept {
    if (!std::isfinite(dt) || dt < 0.0 || !std::isfinite(startAgeS) || startAgeS < 0.0) return false;
    if (dt == 0.0) {
        after = before;
        return finite(before.position) && finite(before.velocity);
    }

    Point p = before.position;
    Point v = before.velocity;
    double elapsed = 0.0;
    constexpr int kMaxIter = 200000;
    int iter = 0;

    while (elapsed < dt) {
        if (++iter > kMaxIter) return false;
        const double t = startAgeS + elapsed;
        Point a1, a2, a3, a4;
        double rate = 0.0;
        if (!acceleration(before, motion, t, v, a1, &rate)) return false;
        const double rem = dt - elapsed;
        const double hIdeal = (rate > 1e-6) ? (kTargetStiffnessStep / rate) : kMaxSubStepS;
        const double hMax = std::clamp(hIdeal, kMinSubStepS, kMaxSubStepS);
        const double h = (rem <= hMax * 1.05) ? rem : hMax;

        const Point v2 = addScaled(v, a1, 0.5 * h);
        if (!acceleration(before, motion, t + 0.5 * h, v2, a2)) return false;
        const Point v3 = addScaled(v, a2, 0.5 * h);
        if (!acceleration(before, motion, t + 0.5 * h, v3, a3)) return false;
        const Point v4 = addScaled(v, a3, h);
        if (!acceleration(before, motion, t + h, v4, a4)) return false;

        const double h6 = h / 6.0;
        p.x += h6 * (v.x + 2.0 * v2.x + 2.0 * v3.x + v4.x);
        p.y += h6 * (v.y + 2.0 * v2.y + 2.0 * v3.y + v4.y);
        p.z += h6 * (v.z + 2.0 * v2.z + 2.0 * v3.z + v4.z);
        v.x += h6 * (a1.x + 2.0 * a2.x + 2.0 * a3.x + a4.x);
        v.y += h6 * (a1.y + 2.0 * a2.y + 2.0 * a3.y + a4.y);
        v.z += h6 * (a1.z + 2.0 * a2.z + 2.0 * a3.z + a4.z);
        if (!finite(p) || !finite(v)) return false;
        elapsed = (h == rem) ? dt : (elapsed + h);
    }

    after = before;
    after.position = p;
    after.velocity = v;
    return true;
}

} // namespace

double isaAirDensityKgM3(double altitudeMslM, double seaLevelTempK, double seaLevelPressurePa) noexcept {
    const double h = std::isfinite(altitudeMslM) ? std::clamp(altitudeMslM, -500.0, 45000.0) : 0.0;
    const double t0 = (std::isfinite(seaLevelTempK) && seaLevelTempK > 180.0 && seaLevelTempK < 340.0) ? seaLevelTempK : 288.15;
    const double p0 = (std::isfinite(seaLevelPressurePa) && seaLevelPressurePa > 50000.0 && seaLevelPressurePa < 120000.0)
                          ? seaLevelPressurePa
                          : 101325.0;
    constexpr double kExp = kGravityMs2 / (kGasConstAir * kLapseRateKM); // ~5.25588
    if (h <= kTropopauseM) {
        const double temp = std::max(180.0, t0 - kLapseRateKM * h);
        const double pressure = p0 * std::pow(temp / t0, kExp);
        return pressure / (kGasConstAir * temp);
    }
    const double tTrop = std::max(180.0, t0 - kLapseRateKM * kTropopauseM);
    const double pTrop = p0 * std::pow(tTrop / t0, kExp);
    const double pressure = pTrop * std::exp(-kGravityMs2 * (h - kTropopauseM) / (kGasConstAir * tTrop));
    return std::max(1e-4, pressure / (kGasConstAir * tTrop));
}

double flareBurnFactor(const Spec& spec, double ageS) noexcept {
    if (!std::isfinite(ageS) || ageS < 0.0 || !(spec.flareLifeS > 0.0) || ageS >= spec.flareLifeS) return 0.0;
    const double riseTau = std::max(0.02, spec.flareRiseS * 0.35);
    const double rise = 1.0 - std::exp(-ageS / riseTau);
    const double plateauEnd = spec.flareLifeS * 0.68;
    double envelope = 1.0;
    if (ageS <= plateauEnd) {
        const double f = std::clamp((ageS - spec.flareRiseS) / std::max(plateauEnd - spec.flareRiseS, 0.05), 0.0, 1.0);
        envelope = 1.0 - 0.14 * f;
    } else {
        const double u = std::clamp((ageS - plateauEnd) / std::max(spec.flareLifeS - plateauEnd, 0.05), 0.0, 1.0);
        envelope = 0.86 * std::pow(1.0 - u, 1.8);
    }
    return std::clamp(rise * envelope, 0.0, 1.0);
}

double flareIntensityWSr(const Decoy& decoy, double ageS) noexcept {
    if (decoy.kind != Kind::Flare) return 0.0;
    return decoy.spec.flarePeakIntensityWSr * flareBurnFactor(decoy.spec, ageS);
}

double flareTemperatureK(const Decoy& decoy, double ageS, double ambientK) noexcept {
    const double amb = (std::isfinite(ambientK) && ambientK > 100.0) ? ambientK : 250.0;
    if (decoy.kind != Kind::Flare) return amb;
    const double burn = flareBurnFactor(decoy.spec, ageS);
    if (burn <= 0.0) return amb;
    // Combustion flame temperature rises quickly to ~2150 K and drops with fourth-root radiometric scaling on tail-off.
    const double tFrac = std::pow(burn, 0.30);
    return amb + (decoy.spec.flarePeakTemperatureK - amb) * tFrac;
}

double flareCoreRadiusM(const Decoy& decoy, double ageS) noexcept {
    if (decoy.kind != Kind::Flare) return 0.05;
    const double burn = flareBurnFactor(decoy.spec, ageS);
    return std::max(0.06, decoy.spec.flareCoreRadiusM * (0.25 + 0.75 * std::sqrt(burn)));
}

double chaffBloomRadiusM(const Decoy& decoy, double ageS) noexcept {
    if (decoy.kind != Kind::Chaff || !std::isfinite(ageS) || ageS < 0.0) return 0.6;
    const double bloom = 1.0 - std::exp(-ageS / std::max(decoy.spec.chaffBloomTimeS, 0.1));
    // Initial cartridge burst (~0.6 m) + wake vortex bloom + slow atmospheric turbulent wind-shear diffusion (~0.35 m/s).
    return 0.6 + decoy.spec.chaffBloomRadiusM * bloom + 0.35 * std::min(ageS, decoy.lifeS);
}

double chaffRcsM2(const Decoy& decoy, double ageS, double frequencyHz) noexcept {
    if (decoy.kind != Kind::Chaff || !std::isfinite(ageS) || ageS < 0.0 || ageS >= decoy.lifeS) return 0.0;
    // Rapid de-clumping bloom (dipoles separate from cartridge pack in ~0.45 s),
    // followed by gradual wind-shear dilution over the cloud's active lifetime.
    const double bloom = 1.0 - std::exp(-ageS / 0.45);
    const double holdS = std::min(3.0, decoy.lifeS * 0.25);
    const double decayTau = std::max(4.0, (decoy.lifeS - holdS) * 0.72);
    const double persist = (ageS <= holdS) ? 1.0 : std::exp(-(ageS - holdS) / decayTau);
    const double tailFade = 1.0 - std::clamp((ageS - decoy.lifeS * 0.85) / std::max(decoy.lifeS * 0.15, 0.1), 0.0, 1.0);

    // Multi-cut half-wave dipole pack resonance curve: broad plateau across X/Ku/C/S bands,
    // tapering smoothly off-band (e.g. VHF/UHF long wavelengths).
    double freqFactor = 1.0;
    if (std::isfinite(frequencyHz) && frequencyHz > 1e6 && decoy.spec.chaffCenterFreqHz > 1e6) {
        const double logRatio = std::log2(frequencyHz / decoy.spec.chaffCenterFreqHz);
        // Flat across ~2 octaves around center band (~2.5-20 GHz for 10 GHz pack), gentle rolloff beyond.
        const double offBand = std::max(0.0, std::abs(logRatio) - 1.2);
        freqFactor = std::clamp(std::exp(-0.55 * offBand * offBand), 0.12, 1.0);
    }
    return decoy.spec.chaffPeakRcsM2 * bloom * persist * tailFade * freqFactor;
}

bool sampleAt(const Trajectory& trajectory, double offsetS, Sample& out) noexcept {
    const auto& before = trajectory.before;
    const auto& end = trajectory.activeEnd;
    Motion motion = trajectory.motion;
    if (!trajectory.valid || !std::isfinite(offsetS) || offsetS < 0.0
        || !std::isfinite(trajectory.stepDurationS) || trajectory.stepDurationS <= 0.0
        || !std::isfinite(trajectory.liveDurationS) || trajectory.liveDurationS < 0.0
        || trajectory.liveDurationS > trajectory.stepDurationS || offsetS > trajectory.liveDurationS
        || !std::isfinite(before.ageS) || before.ageS < 0.0 || !std::isfinite(end.ageS) || end.ageS < before.ageS
        || (before.state.kind != Kind::Chaff && before.state.kind != Kind::Flare)
        || !std::isfinite(before.state.lifeS) || before.state.lifeS <= 0.0
        || before.ageS >= before.state.lifeS || end.ageS > before.state.lifeS
        || !finite(before.state.position) || !finite(before.state.velocity)
        || !finite(end.state.position) || !finite(end.state.velocity) || !valid(motion)) return false;
    if (offsetS == 0.0) { out = before; return true; }
    if (offsetS == trajectory.liveDurationS) { out = end; return true; }
    Sample sample;
    sample.ageS = before.ageS + offsetS;
    if (!std::isfinite(sample.ageS) || sample.ageS > before.state.lifeS
        || !propagate(before.state, motion, before.ageS, offsetS, sample.state)) return false;
    out = sample;
    return true;
}

Dispenser::Dispenser(std::uint32_t owner, unsigned chaff, unsigned flare, Spec spec)
    : owner_(owner), chaff_(chaff), flare_(flare), spec_(sanitizeSpec(spec)) {}

bool Dispenser::dispense(Kind kind, double now, Point position, Point velocity, std::uint32_t id) {
    if ((kind != Kind::Chaff && kind != Kind::Flare) || !std::isfinite(now) || now < 0.0 ||
        !finite(position) || !finite(velocity) || id == 0 || remaining(kind) == 0) return false;
    if (lastReleaseS_ >= 0.0 && (now < lastReleaseS_ || now < lastReleaseS_ + spec_.minIntervalS)) return false;
    if (std::any_of(decoys_.begin(), decoys_.end(), [id](const Decoy& decoy) { return decoy.id == id; })) return false;

    ages_.push_back({});
    try {
        const double lifeS = (kind == Kind::Chaff) ? spec_.chaffLifeS : spec_.flareLifeS;
        decoys_.push_back({id, owner_, kind, now, lifeS, position, velocity, spec_});
    } catch (...) {
        ages_.pop_back();
        throw;
    }
    --(kind == Kind::Chaff ? chaff_ : flare_);
    lastReleaseS_ = now;
    return true;
}

void Dispenser::step(double dt, Point down) {
    (void)advance(dt, [down](const Decoy&) { return Motion{down, {}, kSeaLevelRho}; });
}

std::vector<Trajectory> Dispenser::advance(double dt, std::function<Motion(const Decoy&)> environment) {
    if (!std::isfinite(dt) || dt <= 0.0 || !environment) return {};
    std::vector<Motion> motions;
    motions.reserve(decoys_.size());
    for (const auto& decoy : decoys_) {
        Motion motion = environment(decoy);
        if (!valid(motion)) return {};
        motions.push_back(motion);
    }
    std::vector<Trajectory> trajectories;
    std::vector<Decoy> survivors;
    std::vector<Age> survivorAges;
    trajectories.reserve(decoys_.size());
    survivors.reserve(decoys_.size());
    survivorAges.reserve(decoys_.size());
    for (std::size_t index = 0; index < decoys_.size(); ++index) {
        const auto& decoy = decoys_[index];
        Age age = ages_[index];
        Trajectory path;
        path.before = {decoy, age.elapsedS};
        path.stepDurationS = dt;
        path.motion = motions[index];
        const double remaining = std::max(0.0, decoy.lifeS - age.elapsedS);
        path.expires = dt >= remaining;
        if (!path.expires) {
            const double correctedDt = dt - age.correctionS;
            const double elapsed = age.elapsedS + correctedDt;
            age.correctionS = (elapsed - age.elapsedS) - correctedDt;
            age.elapsedS = elapsed;
            path.expires = elapsed >= decoy.lifeS;
        }
        path.liveDurationS = path.expires ? std::min(dt, remaining) : dt;
        path.activeEnd.ageS = path.expires ? decoy.lifeS : age.elapsedS;
        path.valid = propagate(decoy, path.motion, path.before.ageS, path.liveDurationS, path.activeEnd.state);
        if (path.valid && !path.expires) {
            survivors.push_back(path.activeEnd.state);
            survivorAges.push_back(age);
        } else if (!path.valid) {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            path.activeEnd.ageS = nan;
            path.activeEnd.state = decoy;
            path.activeEnd.state.position = path.activeEnd.state.velocity = {nan, nan, nan};
        }
        trajectories.push_back(std::move(path));
    }
    decoys_.swap(survivors);
    ages_.swap(survivorAges);
    return trajectories;
}

unsigned Dispenser::remaining(Kind kind) const {
    return kind == Kind::Chaff ? chaff_ : kind == Kind::Flare ? flare_ : 0;
}

const std::vector<Decoy>& Dispenser::decoys() const { return decoys_; }

} // namespace fsim::ms::countermeasure
