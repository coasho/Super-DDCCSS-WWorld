#include "simulation/mission_systems/optical/OpticalSensor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::optical {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kMrad = 1e-3;
constexpr double kStefanBoltzmann = 5.670374419e-8; ///< W / (m^2 K^4)
constexpr double kSecondRadiationC2 = 0.01438776877; ///< h*c/k_B (m*K)
constexpr double kEarthRadiusM = 6371000.0;
constexpr std::size_t kMaxContacts = 64;

void ecefToGeodetic(const double p[3], double& latRad, double& lonRad, double& heightM) noexcept {
    constexpr double a = 6378137.0;
    constexpr double e2 = 0.00669437999014;
    const double horiz = std::hypot(p[0], p[1]);
    double lat = std::atan2(p[2], horiz * (1.0 - e2));
    double n = a;
    for (int k = 0; k < 5; ++k) {
        const double sl = std::sin(lat);
        n = a / std::sqrt(1.0 - e2 * sl * sl);
        lat = std::atan2(p[2] + e2 * n * sl, horiz);
    }
    latRad = lat;
    lonRad = std::atan2(p[1], p[0]);
    heightM = horiz / std::max(std::cos(lat), 1e-9) - n;
}

double cloudSlabFraction(double h1, double h2, double cloudBaseM, double cloudTopM) noexcept {
    const double lo = std::min(h1, h2);
    const double hi = std::max(h1, h2);
    if (hi - lo < 1.0) {
        return (lo >= cloudBaseM && lo <= cloudTopM) ? 1.0 : 0.0;
    }
    const double ovLo = std::max(lo, cloudBaseM);
    const double ovHi = std::min(hi, cloudTopM);
    if (!(ovHi > ovLo)) return 0.0;
    return std::clamp((ovHi - ovLo) / (hi - lo), 0.0, 1.0);
}

// Generic MWIR IRST fallback used only when a synthetic card omits parameters;
// all 19 maintained passive_optical payload cards declare their own parameters in assets/payloads/*.json.
struct OpticalClassDefaults {
    double bandMinUm = 3.0;
    double bandMaxUm = 5.0;
    double neiWm2 = 2.0e-11;          ///< Noise-Equivalent Irradiance (W/m^2)
    double detectionSnr = 6.0;        ///< Linear SNR threshold (~7.8 dB)
    double lookdownFactor = 1.8;      ///< Background-limited NEI multiplier looking below horizon
    double azLimitDeg = 60.0;         ///< Horizontal field of regard half-angle
    double elMinDeg = -20.0;          ///< Lower elevation FOR limit
    double elMaxDeg = 60.0;           ///< Upper elevation FOR limit
    double angleSigmaMrad = 0.8;      ///< Nominal angular uncertainty at threshold (mrad)
    double angleFloorMrad = 0.25;     ///< High-SNR stabilization/IFOV floor (mrad)
    double measureS = 0.25;           ///< Measurement cadence (s)
    double memoryS = 3.0;             ///< Track coast memory (s)
    bool allRound = false;            ///< Full 4*pi spherical coverage (DAS / MLD / MAWS)
    bool continuous = false;          ///< Operates immediately on PayloadState::Operate without Search command
    bool uvOnly = false;              ///< Solar-blind UV missile warning sensor
    bool laserCapable = false;        ///< Fitted active optical LRF / target designator
    double laserWavelengthUm = 1.064; ///< 1.064 um (Nd:YAG) or 1.54 um (Er:Glass eye-safe)
    double laserPeakPowerW = 4.0e5;   ///< Pulsed peak optical power (W)
    double laserApertureM = 0.10;     ///< Receiver aperture diameter (m)
    double laserNepW = 2.5e-9;        ///< Laser receiver noise-equivalent power (W)
    double laserMaxRangeM = 18000.0;  ///< Instrumented max laser range (m)
    double laserRangeSigmaM = 3.0;    ///< 1-sigma laser range measurement error (m)
};

class OpticalSensor final : public Model {
public:
    OpticalSensor(const Card& card, const OpticalClassDefaults& def, std::uint16_t count) {
        (void)count;
        bandMinUm_ = card.value("wavelength_min_um", def.bandMinUm);
        bandMaxUm_ = card.value("wavelength_max_um", def.bandMaxUm);
        neiWm2_ = card.value("nei_w_m2", def.neiWm2);
        detectionSnr_ = card.value("detection_snr", def.detectionSnr);
        lookdownFactor_ = card.value("lookdown_factor", def.lookdownFactor);
        azLimitRad_ = card.value("azimuth_limit_deg", def.azLimitDeg) * kDeg;
        elMinRad_ = card.value("elevation_min_deg", def.elMinDeg) * kDeg;
        elMaxRad_ = card.value("elevation_max_deg", def.elMaxDeg) * kDeg;
        angleSigmaRad_ = card.value("angle_sigma_mrad", def.angleSigmaMrad) * kMrad;
        angleFloorRad_ = std::min(angleSigmaRad_, card.value("angle_floor_mrad", def.angleFloorMrad) * kMrad);
        measureS_ = card.value("measure_s", def.measureS);
        memoryS_ = card.value("memory_s", def.memoryS);
        allRound_ = card.value("all_round", def.allRound ? 1.0 : 0.0) > 0.5;
        continuous_ = card.value("continuous", def.continuous ? 1.0 : 0.0) > 0.5;
        uvOnly_ = def.uvOnly || (bandMaxUm_ < 0.4);
        laserCapable_ = card.value("laser_capable", def.laserCapable ? 1.0 : 0.0) > 0.5;
        laserWavelengthUm_ = card.value("laser_wavelength_um", def.laserWavelengthUm);
        laserPeakPowerW_ = card.value("laser_peak_power_w", def.laserPeakPowerW);
        laserApertureM_ = card.value("laser_aperture_m", def.laserApertureM);
        laserNepW_ = card.value("laser_nep_w", def.laserNepW);
        laserMaxRangeM_ = card.value("laser_max_range_m", def.laserMaxRangeM);
        laserRangeSigmaM_ = card.value("laser_range_sigma_m", def.laserRangeSigmaM);
    }

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Search && task.kind != TaskKind::Track) {
            return Reason::InvalidParameter;
        }
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) {
            return Reason::InvalidParameter;
        }
        if (task.kind == TaskKind::Search) {
            if (!std::isfinite(task.azimuthCentreRad) || !std::isfinite(task.elevationCentreRad)) {
                return Reason::InvalidParameter;
            }
            if (std::isfinite(task.azimuthHalfWidthRad) && !(task.azimuthHalfWidthRad > 0.0)) {
                return Reason::InvalidParameter;
            }
            if (std::isfinite(task.elevationHalfWidthRad) && !(task.elevationHalfWidthRad > 0.0)) {
                return Reason::InvalidParameter;
            }
            const double azC = std::remainder(task.azimuthCentreRad, 2.0 * kPi);
            const double elC = std::clamp(task.elevationCentreRad, -0.5 * kPi, 0.5 * kPi);
            if (!allRound_ && (std::fabs(azC) > azLimitRad_ + 1e-6 || elC < elMinRad_ - 1e-6 || elC > elMaxRad_ + 1e-6)) {
                return Reason::OutOfReach;
            }
            mode_ = Mode::Search;
            finished_ = Reason::None;
            startS_ = now;
            lastS_ = now;
            durationS_ = std::isfinite(task.durationS) ? task.durationS : std::numeric_limits<double>::quiet_NaN();
            searchAzCentreRad_ = azC;
            searchElCentreRad_ = elC;
            searchAzHalfRad_ = std::isfinite(task.azimuthHalfWidthRad)
                                   ? std::min(task.azimuthHalfWidthRad, allRound_ ? kPi : azLimitRad_)
                                   : (allRound_ ? kPi : azLimitRad_);
            searchElHalfRad_ = std::isfinite(task.elevationHalfWidthRad)
                                   ? std::min(task.elevationHalfWidthRad, 0.5 * kPi)
                                   : 0.5 * (elMaxRad_ - elMinRad_);
            followContact_ = 0;
            trackedLocalId_ = 0;
            cueActive_ = false;
            laserActive_ = false;
            laserRanged_ = false;
            lastLaserRangeM_ = std::numeric_limits<double>::quiet_NaN();
            return Reason::None;
        }

        // TaskKind::Track
        if (task.contact == 0) return Reason::InvalidParameter;
        mode_ = Mode::Track;
        finished_ = Reason::None;
        startS_ = now;
        lastS_ = now;
        durationS_ = std::isfinite(task.durationS) ? task.durationS : std::numeric_limits<double>::quiet_NaN();
        followContact_ = task.contact;
        trackedLocalId_ = 0;
        // Check whether `task.contact` already corresponds to one of our own held contacts.
        const std::uint64_t lowId = task.contact & 0x0000FFFFFFFFFFFFull;
        for (const Held& h : held_) {
            if (h.contact.id == lowId) {
                trackedLocalId_ = h.contact.id;
                cueAzRad_ = h.contact.azimuthRad;
                cueElRad_ = h.contact.elevationRad;
                cueActive_ = true;
                break;
            }
        }
        lastTrackSeenS_ = now;
        laserActive_ = laserCapable_;
        laserRanged_ = false;
        lastLaserRangeM_ = std::numeric_limits<double>::quiet_NaN();
        return Reason::None;
    }

    void stop(double) override {
        mode_ = Mode::Idle;
        finished_ = Reason::None;
        followContact_ = 0;
        trackedLocalId_ = 0;
        cueActive_ = false;
        laserActive_ = false;
        laserRanged_ = false;
        lastLaserRangeM_ = std::numeric_limits<double>::quiet_NaN();
    }

    Reason finished() const override { return finished_; }

    double percent() const override {
        if (mode_ == Mode::Idle || !std::isfinite(durationS_) || durationS_ <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::clamp(100.0 * (lastS_ - startS_) / durationS_, 0.0, 100.0);
    }

    std::uint64_t follows() const override {
        return (mode_ == Mode::Track && followContact_ != 0) ? followContact_ : 0;
    }

    void followed(const Contact* held) override {
        if (mode_ != Mode::Track) return;
        if (held) {
            cueAzRad_ = held->azimuthRad;
            cueElRad_ = held->elevationRad;
            cueActive_ = true;
            // If the cued contact is one of our own held contacts, lock directly to its local ID.
            const std::uint64_t lowId = held->id & 0x0000FFFFFFFFFFFFull;
            for (const Held& h : held_) {
                if (h.contact.id == lowId) {
                    trackedLocalId_ = h.contact.id;
                    break;
                }
            }
        } else if (trackedLocalId_ == 0) {
            // External cue vanished before we acquired our own IR track.
            mode_ = Mode::Idle;
            followContact_ = 0;
            cueActive_ = false;
            laserActive_ = false;
            laserRanged_ = false;
            finished_ = Reason::ContactLost;
        }
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) override {
        lastS_ = now;
        if (mode_ != Mode::Idle && std::isfinite(durationS_) && now - startS_ + 1e-9 >= durationS_) {
            mode_ = Mode::Idle;
            followContact_ = 0;
            trackedLocalId_ = 0;
            cueActive_ = false;
            laserActive_ = false;
            laserRanged_ = false;
            finished_ = Reason::GoalReached;
        }

        const bool activeLook = continuous_ || mode_ == Mode::Search || mode_ == Mode::Track;
        if (!activeLook) {
            held_.erase(std::remove_if(held_.begin(), held_.end(),
                                       [&](const Held& h) { return now - h.contact.timeS > memoryS_; }),
                        held_.end());
            return;
        }

        const sim::EnvironmentState& env = scene.environment();
        const double ownUp[3] = {
            std::cos(own.latitudeRad) * std::cos(own.longitudeRad),
            std::cos(own.latitudeRad) * std::sin(own.longitudeRad),
            std::sin(own.latitudeRad)};
        double ownVelEcef[3];
        geom::velocityEcef(own, ownVelEcef);

        laserRanged_ = false;
        laserActive_ = (mode_ == Mode::Track && laserCapable_);
        bool trackedSeenThisStep = false;

        auto evaluateCandidate = [&](std::uint32_t id, const sim::VehicleState& st, Role role,
                                     bool burning, bool isFlare, double flareIntensityWSr) {
            double los[3];
            const double rangeM = geom::lineOfSight(own, st.positionEcef, los);
            if (!(rangeM > 1.0)) return;

            double body[3], az = 0.0, el = 0.0;
            geom::toBody(own, los, body);
            geom::angles(body, az, el);

            // Field of Regard gate
            if (!allRound_) {
                if (std::fabs(az) > azLimitRad_ || el < elMinRad_ || el > elMaxRad_) return;
            }

            // Active scan / track window gate
            if (mode_ == Mode::Search) {
                if (!allRound_ || searchAzHalfRad_ < kPi - 1e-6 || searchElHalfRad_ < 0.5 * kPi - 1e-6) {
                    if (!geom::within(az, el, searchAzCentreRad_, searchAzHalfRad_, searchElCentreRad_, searchElHalfRad_)) {
                        return;
                    }
                }
            } else if (mode_ == Mode::Track) {
                if (cueActive_) {
                    constexpr double kTrackGateRad = 8.0 * kDeg;
                    if (!geom::within(az, el, cueAzRad_, kTrackGateRad, cueElRad_, kTrackGateRad)) return;
                }
            }

            // Line-of-sight horizon / terrain masking
            if (!isFlare) {
                if (scene.masked(id, rangeM)) return;
            } else {
                const double hA = std::max(0.0, own.altitudeMslM);
                const double hB = std::max(0.0, st.altitudeMslM);
                const double horizon = std::sqrt(2.0 * kEarthRadiusM * hA) + std::sqrt(2.0 * kEarthRadiusM * hB);
                if (rangeM > horizon) return;
            }

            // Target band radiant intensity (W/sr)
            double intensityWSr = 0.0;
            if (isFlare) {
                if (uvOnly_) {
                    intensityWSr = 0.0;
                } else if (bandMinUm_ >= 7.0) {
                    intensityWSr = 0.15 * flareIntensityWSr; // Pyrotechnic flares peak in MWIR, weaker in LWIR
                } else {
                    intensityWSr = flareIntensityWSr;
                }
            } else {
                const double extent = scene.targetExtentRmsM(id);
                intensityWSr = targetRadiantIntensityWSr(
                    st, role, burning, own.positionEcef, bandMinUm_, bandMaxUm_, extent, env.temperatureSeaLevelK);
            }
            if (!(intensityWSr > 0.0)) return;

            const double tau = atmosphericTransmittance(
                rangeM, own.altitudeMslM, st.altitudeMslM, bandMinUm_, bandMaxUm_, env);
            if (!(tau > 0.0)) return;

            const double irradiance = intensityWSr * tau / (rangeM * rangeM);
            // Look-down against warm Earth background increases background-limited NEI.
            const bool lookDown = geom::dot3(los, ownUp) < -0.02;
            const double neiEff = neiWm2_ * (lookDown ? lookdownFactor_ : 1.0);
            const double snr = irradiance / std::max(neiEff, 1e-24);
            if (!(snr >= 1.0)) return;

            Held* h = findHeld(id);
            const bool newMeas = (!h) || (now - h->measuredS >= measureS_);
            if (newMeas) {
                const double pd = 0.5 * std::erfc((detectionSnr_ - snr) / std::sqrt(2.0));
                if (rng.uniform() >= pd) return;
            }
            if (!h) {
                h = createHeld(id, now);
                if (!h) return;
            }

            const double snrGain = std::sqrt(std::max(1.0, snr / std::max(detectionSnr_, 1.0)));
            const double sigmaEff = std::clamp(angleSigmaRad_ / snrGain, angleFloorRad_, angleSigmaRad_);
            if (newMeas) {
                h->measuredS = now;
                h->sigmaEff = sigmaEff;
                h->azErr = sigmaEff * rng.normal();
                h->elErr = sigmaEff * rng.normal();
            }

            Contact& c = h->contact;
            c.timeS = now;
            c.azimuthRad = std::remainder(az + h->azErr, 2.0 * kPi);
            c.elevationRad = std::clamp(el + h->elErr, -0.5 * kPi, 0.5 * kPi);
            c.bearingSigmaRad = h->sigmaEff;
            c.strengthDb = 10.0 * std::log10(std::max(snr, 1e-6));

            // Missile approach warning flag on closing / burning weapons
            double tVelEcef[3];
            geom::velocityEcef(st, tVelEcef);
            const double rangeRate = (tVelEcef[0] - ownVelEcef[0]) * los[0] +
                                     (tVelEcef[1] - ownVelEcef[1]) * los[1] +
                                     (tVelEcef[2] - ownVelEcef[2]) * los[2];
            c.warning = (!isFlare && role == Role::Weapon && (burning || rangeRate < -10.0));

            // If in Track mode without a locked local ID yet, acquire the strongest contact inside the cue gate.
            if (mode_ == Mode::Track && trackedLocalId_ == 0) {
                trackedLocalId_ = c.id;
            }

            const bool isTracked = (mode_ == Mode::Track && trackedLocalId_ == c.id);
            if (isTracked) {
                trackedSeenThisStep = true;
                lastTrackSeenS_ = now;
                cueAzRad_ = c.azimuthRad;
                cueElRad_ = c.elevationRad;
                cueActive_ = true;
            }

            // Active Optical Laser Rangefinder (LRF): ONLY during Track mode on LRF-capable payloads!
            if (isTracked && laserCapable_) {
                const double lrfSnr = laserReturnSnr(
                    rangeM, own.altitudeMslM, st.altitudeMslM,
                    laserPeakPowerW_, laserApertureM_, laserWavelengthUm_,
                    0.2, laserNepW_, env);
                if (rangeM <= laserMaxRangeM_ && lrfSnr >= 10.0) {
                    if (newMeas || !std::isfinite(c.rangeM)) {
                        const double prevRange = c.rangeM;
                        const double prevTime = h->lastRangeS;
                        const double measuredR = std::max(10.0, rangeM + laserRangeSigmaM_ * rng.normal());
                        c.rangeM = measuredR;
                        if (std::isfinite(prevRange) && std::isfinite(prevTime) && now - prevTime > 1e-3) {
                            c.rangeRateMs = (measuredR - prevRange) / (now - prevTime);
                        } else {
                            c.rangeRateMs = std::numeric_limits<double>::quiet_NaN();
                        }
                        h->lastRangeS = now;
                    }
                    // Project measured bearing + laser range to geodetic lat/lon/height
                    const double mBody[3] = {
                        std::cos(c.elevationRad) * std::cos(c.azimuthRad),
                        std::cos(c.elevationRad) * std::sin(c.azimuthRad),
                        -std::sin(c.elevationRad)};
                    double mRay[3];
                    geom::toEcef(own, mBody, mRay);
                    const double estEcef[3] = {
                        own.positionEcef[0] + c.rangeM * mRay[0],
                        own.positionEcef[1] + c.rangeM * mRay[1],
                        own.positionEcef[2] + c.rangeM * mRay[2]};
                    double lat = 0.0, lon = 0.0, hM = 0.0;
                    ecefToGeodetic(estEcef, lat, lon, hM);
                    c.latitudeRad = lat;
                    c.longitudeRad = lon;
                    c.heightM = hM;
                    laserRanged_ = true;
                    lastLaserRangeM_ = c.rangeM;
                } else {
                    clearRange(c, *h);
                }
            } else {
                clearRange(c, *h);
            }
        };

        std::uint32_t ids[512];
        const sim::VehicleState* targets[512];
        const std::size_t nTargets = scene.targets(ids, targets, 512);
        for (std::size_t k = 0; k < nTargets; ++k) {
            if (!targets[k]) continue;
            const Role role = scene.role(ids[k]);
            if (role == Role::Debris) continue;
            const bool burning = (role == Role::Weapon) ? scene.weaponBurning(ids[k]) : false;
            evaluateCandidate(ids[k], *targets[k], role, burning, false, 0.0);
        }
        for (const Decoy& flare : scene.flares()) {
            if (flare.kind != Decoy::Kind::Flare || !(flare.intensityWSr > 0.0)) continue;
            evaluateCandidate(flare.id, flare.state, Role::Debris, false, true, flare.intensityWSr);
        }

        // Prune aged contacts
        held_.erase(std::remove_if(held_.begin(), held_.end(),
                                   [&](const Held& h) { return now - h.contact.timeS > memoryS_; }),
                    held_.end());

        // Track-loss check
        if (mode_ == Mode::Track && !trackedSeenThisStep && now - lastTrackSeenS_ > memoryS_) {
            mode_ = Mode::Idle;
            followContact_ = 0;
            trackedLocalId_ = 0;
            cueActive_ = false;
            laserActive_ = false;
            laserRanged_ = false;
            lastLaserRangeM_ = std::numeric_limits<double>::quiet_NaN();
            finished_ = Reason::ContactLost;
        }
    }

    void idle(double now) override {
        stop(now);
        held_.clear();
    }

    void contacts(std::vector<Contact>& out) const override {
        for (const Held& h : held_) out.push_back(h.contact);
    }

    void report(std::vector<PayloadValue>& out) const override {
        double warnings = 0.0;
        for (const Held& h : held_) warnings += h.contact.warning ? 1.0 : 0.0;
        out.push_back({"contacts", static_cast<double>(held_.size()), ""});
        out.push_back({"warnings", warnings, ""});
        out.push_back({"searching", (continuous_ || mode_ == Mode::Search) ? 1.0 : 0.0, ""});
        out.push_back({"tracking", (mode_ == Mode::Track) ? 1.0 : 0.0, ""});
        out.push_back({"laser_capable", laserCapable_ ? 1.0 : 0.0, ""});
        out.push_back({"laser_active", laserActive_ ? 1.0 : 0.0, ""});
        out.push_back({"laser_ranged", laserRanged_ ? 1.0 : 0.0, ""});
        out.push_back({"laser_range", laserRanged_ ? lastLaserRangeM_ : std::numeric_limits<double>::quiet_NaN(), "m"});
        out.push_back({"wavelength_min", bandMinUm_, "um"});
        out.push_back({"wavelength_max", bandMaxUm_, "um"});
        out.push_back({"nei", neiWm2_, "W/m^2"});
    }

private:
    enum class Mode : std::uint8_t { Idle, Search, Track };

    struct Held {
        Contact contact;
        std::uint32_t entity = 0;
        double measuredS = -std::numeric_limits<double>::infinity();
        double lastRangeS = std::numeric_limits<double>::quiet_NaN();
        double sigmaEff = 0.0;
        double azErr = 0.0;
        double elErr = 0.0;
    };

    static void clearRange(Contact& c, Held& h) noexcept {
        c.rangeM = std::numeric_limits<double>::quiet_NaN();
        c.rangeRateMs = std::numeric_limits<double>::quiet_NaN();
        c.latitudeRad = std::numeric_limits<double>::quiet_NaN();
        c.longitudeRad = std::numeric_limits<double>::quiet_NaN();
        c.heightM = std::numeric_limits<double>::quiet_NaN();
        h.lastRangeS = std::numeric_limits<double>::quiet_NaN();
    }

    Held* findHeld(std::uint32_t entity) noexcept {
        for (Held& h : held_)
            if (h.entity == entity) return &h;
        return nullptr;
    }

    Held* createHeld(std::uint32_t entity, double now) {
        if (held_.size() >= kMaxContacts) return nullptr;
        Held h;
        h.entity = entity;
        h.sigmaEff = angleSigmaRad_;
        h.contact.id = ++serial_;
        h.contact.kind = ContactKind::Infrared;
        h.contact.mode = EmitterMode::Unknown;
        h.contact.firstTimeS = now;
        h.contact.timeS = now;
        h.contact.truth = entity;
        held_.push_back(h);
        return &held_.back();
    }

    double bandMinUm_ = 3.0;
    double bandMaxUm_ = 5.0;
    double neiWm2_ = 2.0e-11;
    double detectionSnr_ = 6.0;
    double lookdownFactor_ = 1.8;
    double azLimitRad_ = 60.0 * kDeg;
    double elMinRad_ = -20.0 * kDeg;
    double elMaxRad_ = 60.0 * kDeg;
    double angleSigmaRad_ = 0.8 * kMrad;
    double angleFloorRad_ = 0.25 * kMrad;
    double measureS_ = 0.25;
    double memoryS_ = 3.0;
    bool allRound_ = false;
    bool continuous_ = false;
    bool uvOnly_ = false;
    bool laserCapable_ = false;
    double laserWavelengthUm_ = 1.064;
    double laserPeakPowerW_ = 4.0e5;
    double laserApertureM_ = 0.10;
    double laserNepW_ = 2.5e-9;
    double laserMaxRangeM_ = 18000.0;
    double laserRangeSigmaM_ = 3.0;

    Mode mode_ = Mode::Idle;
    Reason finished_ = Reason::None;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double durationS_ = std::numeric_limits<double>::quiet_NaN();
    double searchAzCentreRad_ = 0.0;
    double searchElCentreRad_ = 0.0;
    double searchAzHalfRad_ = 60.0 * kDeg;
    double searchElHalfRad_ = 30.0 * kDeg;
    std::uint64_t followContact_ = 0;
    std::uint64_t trackedLocalId_ = 0;
    bool cueActive_ = false;
    double cueAzRad_ = 0.0;
    double cueElRad_ = 0.0;
    double lastTrackSeenS_ = 0.0;
    bool laserActive_ = false;
    bool laserRanged_ = false;
    double lastLaserRangeM_ = std::numeric_limits<double>::quiet_NaN();

    std::vector<Held> held_;
    std::uint64_t serial_ = 0;
};

} // namespace

double planckBandFraction(double temperatureK, double lowUm, double highUm) noexcept {
    if (!std::isfinite(temperatureK) || !(temperatureK > 0.0) ||
        !std::isfinite(lowUm) || !std::isfinite(highUm) || !(lowUm > 0.0) || !(highUm > lowUm)) {
        return 0.0;
    }
    const double lower = std::min(80.0, kSecondRadiationC2 / (highUm * 1e-6 * temperatureK));
    const double upper = std::min(80.0, kSecondRadiationC2 / (lowUm * 1e-6 * temperatureK));
    if (!(upper > lower)) return 0.0;
    unsigned intervals = std::max(64u, static_cast<unsigned>(std::ceil((upper - lower) * 16.0)));
    if (intervals % 2u != 0u) ++intervals;
    const double step = (upper - lower) / static_cast<double>(intervals);
    double integral = 0.0;
    for (unsigned i = 0; i <= intervals; ++i) {
        const double x = lower + static_cast<double>(i) * step;
        const double integrand = (x > 0.0) ? (x * x * x / std::expm1(x)) : 0.0;
        const double weight = (i == 0u || i == intervals) ? 1.0 : ((i % 2u != 0u) ? 4.0 : 2.0);
        integral += weight * integrand;
    }
    const double fraction = integral * (step / 3.0) * (15.0 / std::pow(kPi, 4));
    return std::isfinite(fraction) ? std::clamp(fraction, 0.0, 1.0) : 0.0;
}

double isaTemperatureK(double altitudeM, double seaLevelK) noexcept {
    const double sl = (std::isfinite(seaLevelK) && seaLevelK > 150.0) ? seaLevelK : 288.15;
    const double h = std::clamp(std::isfinite(altitudeM) ? altitudeM : 0.0, 0.0, 25000.0);
    const double tropopauseK = std::max(180.0, sl - 0.0065 * 11000.0);
    return (h <= 11000.0) ? std::max(tropopauseK, sl - 0.0065 * h) : tropopauseK;
}

double kineticSkinTemperatureK(double ambientK, double mach, double recoveryFactor) noexcept {
    if (!std::isfinite(ambientK) || !(ambientK > 0.0)) return 0.0;
    const double m = std::max(0.0, std::isfinite(mach) ? mach : 0.0);
    const double r = std::clamp(std::isfinite(recoveryFactor) ? recoveryFactor : 0.85, 0.0, 1.0);
    return ambientK * (1.0 + r * 0.2 * m * m);
}

double atmosphericTransmittance(double rangeM, double hSensorM, double hTargetM,
                                double lowUm, double highUm,
                                const sim::EnvironmentState& env) noexcept {
    if (!std::isfinite(rangeM) || !(rangeM >= 0.0)) return 0.0;
    if (rangeM <= 0.0) return 1.0;
    const double rKm = rangeM * 1e-3;
    const double midUm = 0.5 * (lowUm + highUm);
    const double humidity = std::clamp(std::isfinite(env.humidity) ? env.humidity : 0.0, 0.0, 1.0);
    const double visKm = std::max(0.2, (std::isfinite(env.visibilityM) && env.visibilityM > 0.0) ? env.visibilityM * 1e-3 : 50.0);
    const double rhoFactor = 0.5 * (geom::densityRatio(hSensorM) + geom::densityRatio(hTargetM));

    // Kruse (1962) / Koschmieder visibility aerosol extinction at wavelength midUm (km^-1)
    const double q = (visKm > 50.0) ? 1.6 : ((visKm > 6.0) ? 1.3 : 0.585 * std::cbrt(visKm));
    const double alphaVisKm = (3.912 / visKm) * std::pow(0.55 / std::max(midUm, 0.25), q);

    double alphaClearKm = 0.0;
    if (highUm < 0.4) {
        // Solar-blind UV: strong ozone & Rayleigh extinction
        alphaClearKm = (0.45 + 0.25 * alphaVisKm) * rhoFactor;
    } else if (midUm <= 2.2) {
        // Near-IR laser window (1.064 um / 1.54 um): aerosol-dominated Kruse law + mild molecular
        alphaClearKm = (alphaVisKm + 0.02 * humidity) * std::max(0.15, rhoFactor);
    } else if (midUm <= 6.5) {
        // MWIR (3-5 um): LOWTRAN clear-air H2O/CO2 + aerosol scaled by altitude density
        alphaClearKm = (0.055 + 0.085 * humidity + 0.20 * alphaVisKm) * rhoFactor;
    } else {
        // LWIR (8-12 um) or dual-band (3-12 um): H2O continuum + aerosol scaled by altitude density
        alphaClearKm = (0.070 + 0.130 * humidity + 0.12 * alphaVisKm) * rhoFactor;
    }

    // Cloud-layer optical extinction (cloudBaseM to cloudBaseM + 1500 m)
    double alphaCloudKm = 0.0;
    const double cover = std::clamp(std::isfinite(env.cloudCover) ? env.cloudCover : 0.0, 0.0, 1.0);
    if (cover > 0.0) {
        const double cloudBase = std::isfinite(env.cloudBaseM) ? env.cloudBaseM : 2000.0;
        const double frac = cloudSlabFraction(hSensorM, hTargetM, cloudBase, cloudBase + 1500.0);
        alphaCloudKm = cover * frac * 12.0; // Dense liquid-water cloud extinction (~12 km^-1 at overcast)
    }

    // Precipitation extinction below cloud top
    double alphaPrecipKm = 0.0;
    const double precip = std::clamp(std::isfinite(env.precipitation) ? env.precipitation : 0.0, 0.0, 1.0);
    if (precip > 0.0) {
        const double wetTop = (std::isfinite(env.cloudBaseM) ? env.cloudBaseM : 2000.0) + 1500.0;
        const double wetFrac = cloudSlabFraction(hSensorM, hTargetM, -500.0, wetTop);
        alphaPrecipKm = precip * wetFrac * 2.5;
    }

    const double tau = (alphaClearKm + alphaCloudKm + alphaPrecipKm) * rKm;
    return std::exp(-std::min(tau, 80.0));
}

double targetRadiantIntensityWSr(const sim::VehicleState& target, Role role, bool weaponBurning,
                                 const double sensorPosEcef[3], double lowUm, double highUm,
                                 double extentRmsM, double ambientSeaLevelK) noexcept {
    if (!(highUm > lowUm) || !(lowUm > 0.0)) return 0.0;
    const bool uvBand = (highUm < 0.4);
    const double ambK = isaTemperatureK(target.altitudeMslM, ambientSeaLevelK);
    const double skinK = kineticSkinTemperatureK(ambK, target.mach, 0.85);

    if (role == Role::Weapon) {
        double intensity = 0.0;
        // Scale solid-rocket motor plume and body skin area physically with weapon scale (extentRmsM),
        // matching 1.0x reference at extentRmsM = 1.0 m (SRAAM/MRAAM class).
        const double missileScale = (std::isfinite(extentRmsM) && extentRmsM > 0.2)
                                        ? std::pow(std::clamp(extentRmsM, 0.4, 2.5), 1.5)
                                        : 1.0;
        if (weaponBurning) {
            if (uvBand) {
                // Solid-rocket motor plume UV chemiluminescence (solar-blind 0.25-0.28 um)
                intensity += 75.0 * missileScale;
            } else {
                // Hot solid-rocket motor plume (~2200 K, Hudson 1969 / Mahulikar et al. 2007)
                const double fPlume = planckBandFraction(2200.0, lowUm, highUm);
                intensity += 18000.0 * missileScale * fPlume;
            }
        }
        if (!uvBand) {
            // Aerodynamic kinetic-heating missile body contrast scaled by missile size
            const double areaM2 = (std::isfinite(extentRmsM) && extentRmsM > 0.2)
                                      ? std::clamp(0.35 * extentRmsM * extentRmsM, 0.08, 1.5)
                                      : 0.35;
            const double radSkin = std::pow(skinK, 4) * planckBandFraction(skinK, lowUm, highUm);
            const double radAmb = std::pow(ambK, 4) * planckBandFraction(ambK, lowUm, highUm);
            intensity += (0.85 * areaM2 * kStefanBoltzmann / kPi) * std::max(0.0, radSkin - radAmb);
        }
        return std::max(0.0, intensity);
    }

    // Aircraft: negligible solar-blind UV emission
    if (uvBand) return 0.0;

    // 1) Aerodynamic kinetic-heating airframe skin contrast
    const double extent = (std::isfinite(extentRmsM) && extentRmsM > 0.5) ? extentRmsM : 5.0;
    const double projAreaM2 = std::clamp(1.2 * extent * extent, 5.0, 90.0);
    const double radSkin = std::pow(skinK, 4) * planckBandFraction(skinK, lowUm, highUm);
    const double radAmb = std::pow(ambK, 4) * planckBandFraction(ambK, lowUm, highUm);
    const double iSkin = (0.85 * projAreaM2 * kStefanBoltzmann / kPi) * std::max(0.0, radSkin - radAmb);

    // 2) Aspect-dependent hot exhaust cavity + gas plume scaled by actual propulsion regime
    //    (Hudson, Infrared System Engineering 1969; Mahulikar et al., Prog. Aerospace Sci. 2007).
    double tToS[3] = {
        sensorPosEcef[0] - target.positionEcef[0],
        sensorPosEcef[1] - target.positionEcef[1],
        sensorPosEcef[2] - target.positionEcef[2]};
    const double dNorm = geom::norm3(tToS);
    double cosTail = 0.0;
    if (dNorm > 1e-6) {
        // Target forward body x-axis in ECEF is column 0 of rotationBodyToEcef
        const double fwd[3] = {
            target.rotationBodyToEcef[0],
            target.rotationBodyToEcef[3],
            target.rotationBodyToEcef[6]};
        cosTail = std::clamp(-(tToS[0] * fwd[0] + tToS[1] * fwd[1] + tToS[2] * fwd[2]) / dNorm, -1.0, 1.0);
    }
    const double tailFrac = 0.5 * (1.0 + cosTail); // 0 = head-on, 0.5 = beam, 1 = tail-on
    const double visNozzle = 0.04 + 0.96 * tailFrac * tailFrac;
    const double visPlume = 0.25 + 0.75 * std::sqrt(std::max(0.0, 1.0 - 0.5 * (1.0 - cosTail) * (1.0 - cosTail)));

    const int totalEngines = std::max(1, target.engineCount);
    const int sampleEngines = std::clamp(totalEngines, 1, sim::VehicleState::kMaxEngines);
    const double engineScale = static_cast<double>(totalEngines) / static_cast<double>(sampleEngines);

    double iNozzle = 0.0;
    double iPlume = 0.0;
    for (int e = 0; e < sampleEngines; ++e) {
        const double thr = std::max(0.15, std::clamp(target.throttlePosition[e], 0.0, 1.0));
        const double ab = std::clamp(target.afterburner[e], 0.0, 1.0);
        const double rpm = std::isfinite(target.engineRpm[e]) ? target.engineRpm[e] : 0.0;
        const double n2 = std::isfinite(target.engineN2[e]) ? target.engineN2[e] : 0.0;
        const double thrustN = (std::isfinite(target.thrustN[e]) && target.thrustN[e] > 0.0) ? target.thrustN[e] : 0.0;
        const double nozPos = std::clamp(std::isfinite(target.nozzlePosition[e]) ? target.nozzlePosition[e] : ab, 0.0, 1.0);

        double areaNozM2 = 0.0;
        double tNozzleK = 520.0;
        double tPlumeK = 850.0;
        double plumeRadiantWSr = 0.0;

        if (rpm > 1.0 && n2 <= 1.0) {
            // Piston / reciprocating propeller engine: small shielded exhaust stack, minimal hot gas plume
            areaNozM2 = 0.008;
            tNozzleK = 500.0 + 140.0 * thr;
            tPlumeK = 580.0;
            plumeRadiantWSr = 18.0 * thr;
        } else if (rpm > 1.0 && n2 > 1.0) {
            // Turboprop / turboshaft: cooler compact turbine exhaust duct (e.g. C-130J, MQ-9, UH-60)
            areaNozM2 = 0.10;
            tNozzleK = 480.0 + 230.0 * thr;
            tPlumeK = 680.0;
            plumeRadiantWSr = 160.0 * thr;
        } else {
            // Turbofan / turbojet: nozzle exit area scales from live thrust or airframe scale, plus AB nozzle opening
            const double baseArea = (thrustN > 500.0)
                                        ? std::clamp(thrustN / (1.8e5 * (1.0 + 0.55 * ab)), 0.14, 0.55)
                                        : std::clamp(0.018 * extent * extent / std::sqrt(static_cast<double>(totalEngines)), 0.15, 0.55);
            areaNozM2 = baseArea * (1.0 + 0.35 * nozPos);
            tNozzleK = 520.0 + 380.0 * thr + 240.0 * ab;
            tPlumeK = 850.0 + 950.0 * ab;
            // CO2/H2O exhaust plume scales with live thrust thermal power (Mahulikar et al. 2007) when available
            plumeRadiantWSr = (thrustN > 500.0)
                                  ? ((thrustN * 1e-3) * (14.0 + 48.0 * ab))
                                  : ((areaNozM2 / 0.45) * (950.0 * thr + 6200.0 * ab));
        }

        iNozzle += engineScale * visNozzle * (0.90 * areaNozM2 * kStefanBoltzmann / kPi) *
                   std::pow(tNozzleK, 4) * planckBandFraction(tNozzleK, lowUm, highUm);

        const double fPlumeMwir = planckBandFraction(tPlumeK, std::max(lowUm, 2.7), std::min(highUm, 5.2));
        const double fPlumeLwir = planckBandFraction(tPlumeK, std::max(lowUm, 7.5), std::min(highUm, 13.0));
        const double plumeBandScale = fPlumeMwir + 0.25 * fPlumeLwir;
        iPlume += engineScale * visPlume * plumeRadiantWSr * plumeBandScale;
    }

    return std::max(0.0, iSkin + iNozzle + iPlume);
}

double laserReturnSnr(double rangeM, double hSensorM, double hTargetM,
                      double peakPowerW, double apertureDiameterM, double wavelengthUm,
                      double reflectance, double nepW,
                      const sim::EnvironmentState& env) noexcept {
    if (!std::isfinite(rangeM) || !(rangeM > 1.0) || !(peakPowerW > 0.0) ||
        !(apertureDiameterM > 0.0) || !(reflectance > 0.0) || !(nepW > 0.0)) {
        return 0.0;
    }
    const double tau = atmosphericTransmittance(
        rangeM, hSensorM, hTargetM, wavelengthUm, wavelengthUm + 0.02, env);
    const double rxAreaM2 = 0.25 * kPi * apertureDiameterM * apertureDiameterM;
    constexpr double kOpticalEta = 0.50;
    const double prW = (peakPowerW * rxAreaM2 * reflectance * kOpticalEta * tau * tau) /
                       (kPi * rangeM * rangeM);
    const double snr = prW / nepW;
    return (std::isfinite(snr) && snr > 0.0) ? snr : 0.0;
}

std::unique_ptr<Model> makeOpticalSensor(const Card& card, std::uint16_t count) {
    return std::make_unique<OpticalSensor>(card, OpticalClassDefaults{}, count);
}

} // namespace fsim::ms::optical
