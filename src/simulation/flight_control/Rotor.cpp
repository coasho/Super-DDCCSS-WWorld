#include "fsim/RotorControllers.h"

#include "simulation/flight_control/Registry.h"
#include "simulation/flight_control/OwnRotorPlant.h"
#include "simulation/flight_control/RotorAllocationV1.h"
#include "foundation/core/Geodesy.h"
#include "foundation/core/Units.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace fsim::control {

namespace {

constexpr double kG = 9.80665;

/// As the fixed-wing loops' (Builtin.cpp): true when the loop missed a period
/// - another level flew meanwhile - so what it holds no longer describes the flight.
bool resumed(const ControlContext& ctx, double& lastTime) noexcept {
    const double t = ctx.state.simTime;
    const bool stale = lastTime >= 0.0 && (t - lastTime > 1.5 * ctx.dt || t < lastTime);
    lastTime = t;
    return stale;
}

double withAirframeBias(RotorRateAxis& axis,double demand,double rate,double dt,double bias) noexcept {
    if(!(std::abs(axis.power)>1e-9))return axis.trim;
    const double error=demand-rate;
    const double proportional=std::max(axis.bandwidth-axis.damping,0.25*axis.bandwidth);
    const double accel=axis.damping*demand+proportional*error;
    const double next=axis.sum+axis.integral*error*dt;
    const double u=axis.trim+(accel+next-bias)/axis.power;
    if(u>=-1 && u<=1){axis.sum=next;return u;}
    return std::clamp(axis.trim+(accel+axis.sum-bias)/axis.power,-1.0,1.0);
}

} // namespace

double thrustLoadFactor(const sim::VehicleState& s) noexcept {
    // the body's acceleration along z - the change of w seen in the turning
    // body frame, plus (omega x v)_z - less gravity's share of it
    const double p = s.angularRateBodyRadS[0], q = s.angularRateBodyRadS[1];
    const double az = s.accelerationBodyMs2[2] + p * s.velocityBodyMs[1] - q * s.velocityBodyMs[0];
    const double gz = kG * std::cos(s.eulerRad[1]) * std::cos(s.eulerRad[0]);
    return (gz - az) / kG;
}

double RotorRateAxis::update(double demand, double rate, double dt) noexcept {
    if (!(std::abs(power) > 1e-9)) return trim;
    const double error = demand - rate;
    // the plant's own damping gives part of the bandwidth: the rest is proportional
    const double proportional = std::max(bandwidth - damping, 0.25 * bandwidth);
    const double accel = damping * demand + proportional * error;
    const double next = sum + integral * error * dt;
    const double u = trim + (accel + next) / power;
    if (u >= -1.0 && u <= 1.0) {
        sum = next;
        return u;
    }
    return std::clamp(trim + (accel + sum) / power, -1.0, 1.0); // at the stop: the integral waits
}

// --- RotorAllocation -------------------------------------------------------------

RotorAllocation::RotorAllocation() {
#define AXIS(a)                                                                                                      params_.add(#a ".power", &a.power), params_.add(#a ".damping", &a.damping), params_.add(#a ".bandwidth", &a.bandwidth),         params_.add(#a ".ki", &a.integral), params_.add(#a ".trim", &a.trim)
    AXIS(roll);
    AXIS(pitch);
    AXIS(yaw);
#undef AXIS
    params_.add("throttle.trim", &throttleTrim);
    params_.add("heave.power", &heavePower);
    params_.add("load_factor.ki", &loadFactorGain);
}

Command RotorAllocation::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AccelerationCommand>(in);
    const auto& s = ctx.sensed;
    if (resumed(ctx, lastTime_)) reset();
    const double* rate = s.angularRateBodyRadS;
    OwnRotorPlantV1 plant;
    const auto* native=dynamic_cast<const OwnRotorPlantViewV1*>(ctx.world);
    const bool compensate=native && native->ownRotorPlantV1(ctx.vehicleId,plant) && plant.available;
    const auto update=[&](RotorRateAxis& axis,double demand,unsigned k) {
        return compensate ? withAirframeBias(axis,demand,rate[k],ctx.dt,plant.angularAcceleration[k])
                          : axis.update(demand,rate[k],ctx.dt);
    };
    ActuatorCommand out;
    if (ctx.engaged & axisBit(Axis::Roll)) {
        out.aileron = update(roll,orHold(c.rollRateRadS,0.0),0);
    } else {
        roll.reset(); // someone else flies it: start afresh when it comes back
        out.aileron = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Pitch)) {
        out.elevator = update(pitch,orHold(c.pitchRateRadS,0.0),1);
    } else {
        pitch.reset();
        out.elevator = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Yaw)) {
        out.rudder = update(yaw,orHold(c.yawRateRadS,0.0),2);
    } else {
        yaw.reset();
        out.rudder = kHold;
    }
    if (ctx.engaged & axisBit(Axis::Thrust)) {
        if (!isHold(c.loadFactorG) && std::abs(heavePower) > 1e-9) {
            // the throttle per g of thrust, from the heave's response; the integral trims what the feedforward misses
            const double perG = kG / heavePower;
            const double feed = throttleTrim + (c.loadFactorG - 1.0) * perG;
            const double next = loadSum_ + loadFactorGain * (c.loadFactorG - thrustLoadFactor(s)) * perG * ctx.dt;
            const double u = feed + next;
            if (u >= 0.0 && u <= 1.0) {
                loadSum_ = next;
                out.throttle = u;
            } else {
                out.throttle = std::clamp(feed + loadSum_, 0.0, 1.0);
            }
        } else {
            out.throttle = c.throttle; // may be kHold: the stack keeps the last value
            loadSum_ = 0.0;
        }
    } else {
        loadSum_ = 0.0;
        out.throttle = kHold;
    }
    return out;
}

void RotorAllocation::reset() {
    roll.reset();
    pitch.reset();
    yaw.reset();
    loadSum_ = 0.0;
}

// --- RotorAttitude ---------------------------------------------------------------

RotorAttitude::RotorAttitude() {
    params_.add("roll.gain", &rollGain);
    params_.add("pitch.gain", &pitchGain);
    params_.add("yaw.gain", &yawGain);
    params_.add("roll.max_rate", &maxRollRateRadS);
    params_.add("pitch.max_rate", &maxPitchRateRadS);
    params_.add("yaw.max_rate", &maxYawRateRadS);
}

Command RotorAttitude::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<AttitudeCommand>(in);
    const auto& s = ctx.sensed;
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    if (!haveRef_ || resumed(ctx, lastTime_)) {
        headingRef_ = psi;
        pitchRef_ = theta;
        haveRef_ = true;
    }
    AccelerationCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    const bool cyclic = (ctx.engaged & kCyclic) != 0, yaw = (ctx.engaged & axisBit(Axis::Yaw)) != 0;
    // the rates the Euler angles are to change at
    double phiDot = 0.0, thetaDot = 0.0, psiDot = 0.0;
    if (cyclic) {
        if (!isHold(c.pitchRad)) pitchRef_ = c.pitchRad;
        phiDot = std::clamp(rollGain * geo::wrapPi(orHold(c.rollRad, 0.0) - phi), -maxRollRateRadS, maxRollRateRadS);
        thetaDot = std::clamp(pitchGain * (pitchRef_ - theta), -maxPitchRateRadS, maxPitchRateRadS);
    } else {
        pitchRef_ = theta;
    }
    if (yaw) {
        if (!isHold(c.headingRad)) headingRef_ = c.headingRad;
        psiDot = std::clamp(yawGain * geo::wrapPi(headingRef_ - psi), -maxYawRateRadS, maxYawRateRadS);
    } else {
        headingRef_ = psi;
    }
    // ... as body rates
    const double sphi = std::sin(phi), cphi = std::cos(phi), sth = std::sin(theta), cth = std::cos(theta);
    if (cyclic) {
        out.rollRateRadS = phiDot - psiDot * sth;
        out.pitchRateRadS = thetaDot * cphi + psiDot * sphi * cth;
    }
    if (yaw) out.yawRateRadS = -thetaDot * sphi + psiDot * cphi * cth;
    if (ctx.engaged & axisBit(Axis::Thrust)) out.throttle = c.throttle; // may be kHold: the stack keeps the last value
    return out;
}

void RotorAttitude::reset() {
    haveRef_ = false;
    lastTime_ = -1.0;
}

// --- RotorVelocity ---------------------------------------------------------------

RotorVelocity::RotorVelocity() {
    params_.add("horizontal.kp", &horizontalKp);
    params_.add("horizontal.ki", &horizontalKi);
    params_.add("max_tilt", &maxTiltRad);
    params_.add("vertical.kp", &verticalKp);
    params_.add("vertical.ki", &verticalKi);
    params_.add("throttle.trim", &throttleTrim);
    params_.add("heave.power", &heavePower);
    params_.add("heave.damping", &heaveDamping);
    params_.add("roll.trim", &rollTrimRad);
    params_.add("pitch.trim", &pitchTrimRad);
}

Command RotorVelocity::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<VelocityCommand>(in);
    const auto& s = ctx.sensed;
    const double phi = s.eulerRad[0], theta = s.eulerRad[1], psi = s.eulerRad[2];
    if (!started_ || resumed(ctx, lastTime_)) {
        northHold_ = s.velocityNedMs[0];
        eastHold_ = s.velocityNedMs[1];
        sumX_ = sumY_ = sumZ_ = 0.0;
        headingRef_ = psi;
        started_ = true;
    }
    AttitudeCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    const double dt = ctx.dt;

    OwnRotorPlantV1 plant;
    const auto* view=dynamic_cast<const OwnRotorPlantViewV1*>(ctx.world);
    bool forceAware=view && view->ownRotorPlantV1(ctx.vehicleId,plant) && plant.available && plant.passiveForceIncluded && std::abs(heavePower)>1e-9 &&
        (ctx.engaged & kCyclic)==kCyclic && (ctx.engaged & axisBit(Axis::Thrust));
    double airframeX=0,airframeY=0,airframeDown=0,support=kG;
    if(forceAware) {
        const double roll=ctx.state.eulerRad[0],pitch=ctx.state.eulerRad[1];
        const auto* force=plant.specificForceBody;
        airframeX=std::cos(pitch)*force[0]+std::sin(pitch)*(std::sin(roll)*force[1]+std::cos(roll)*force[2]);
        airframeY=std::cos(roll)*force[1]-std::sin(roll)*force[2];
        airframeDown=-std::sin(pitch)*force[0]+std::cos(pitch)*(std::sin(roll)*force[1]+std::cos(roll)*force[2]);
        const double error=orHold(c.verticalSpeedMs,0.)+s.velocityNedMs[2];
        const double tilt=std::max(std::cos(roll)*std::cos(pitch),.5);
        const auto throttle=[&](double integral){return throttleTrim+((kG+airframeDown+verticalKp*error+integral)/tilt-kG)/heavePower;};
        const double nextZ=sumZ_+verticalKi*error*dt,candidate=throttle(nextZ);
        const double acceptedZ=candidate>=0 && candidate<=1 ? nextZ:sumZ_;
        const double acceptedThrottle=std::clamp(throttle(acceptedZ),0.,1.);
        support=std::max(0.,(kG+heavePower*(acceptedThrottle-throttleTrim))*tilt);
        forceAware=std::isfinite(support)&&std::isfinite(airframeX)&&std::isfinite(airframeY)&&std::isfinite(airframeDown)&&std::isfinite(acceptedThrottle);
        if(forceAware){sumZ_=acceptedZ;out.throttle=acceptedThrottle;}
    }

    // over the ground or through the air: the velocity error along the heading and to its right
    if (ctx.engaged & kCyclic) {
        double ex, ey;
        const bool ground = !isHold(c.northMs) && !isHold(c.eastMs);
        if (!ground && !isHold(c.airspeedMs)) {
            const double v = s.airspeedTrueMs;
            ex = c.airspeedMs - v * std::cos(s.alphaRad) * std::cos(s.betaRad);
            ey = -v * std::sin(s.betaRad);
            northHold_ = s.velocityNedMs[0], eastHold_ = s.velocityNedMs[1]; // a hold after it keeps what this flew
        } else {
            if (ground) northHold_ = c.northMs, eastHold_ = c.eastMs;
            const double en = northHold_ - s.velocityNedMs[0], ee = eastHold_ - s.velocityNedMs[1];
            ex = en * std::cos(psi) + ee * std::sin(psi);
            ey = -en * std::sin(psi) + ee * std::cos(psi);
        }
        // the acceleration that closes it, and the tilt that makes it (nose down to go forward, bank to go right);
        // the integral trims near the target and slows far from it - through a change of speed it would wind up and
        // overshoot: its gain falls off as band^2 / (band^2 + error^2), band the error the proportional term answers
        // with a quarter of the tilt. Forward it may trim the whole tilt - the drag of a fast multirotor takes most of
        // it (a Crazyflie's best range, 22 of its 24 degrees: ADR-29 FA-3e); sideways half, as an orbit needs
        const double aMax = (forceAware?support:kG) * std::tan(maxTiltRad);
        const double band = horizontalKp > 0.0 ? 0.25 * aMax / horizontalKp : 1.0, band2 = band * band;
        const auto increment=[&](double error) {
            const double denominator=band2+error*error;
            return denominator>0 ? horizontalKi*(band2/denominator)*error*dt:0.;
        };
        double nextX=std::clamp(sumX_+increment(ex),-aMax,aMax);
        double nextY=std::clamp(sumY_+increment(ey),-.5*aMax,.5*aMax);
        if(forceAware) {
            const auto allocate=[&](double ax,double ay) {
                return allocateRotorThrustV1(ax,ay,support,maxTiltRad,pitchTrimRad,rollTrimRad);
            };
            const double ax=horizontalKp*ex+nextX-airframeX,ay=horizontalKp*ey+nextY-airframeY;
            const auto limited=allocate(ax,ay);
            const double rx=ax-support*std::tan(pitchTrimRad-limited.pitch);
            const double ry=ay-support*std::tan(limited.roll-rollTrimRad)/std::cos(limited.pitch);
            const auto delta=rotorIntegralIncrementV1(nextX-sumX_,nextY-sumY_,rx,ry,limited.saturated);
            sumX_=std::clamp(sumX_+delta[0],-aMax,aMax);sumY_=std::clamp(sumY_+delta[1],-.5*aMax,.5*aMax);
            const auto demand=allocate(horizontalKp*ex+sumX_-airframeX,horizontalKp*ey+sumY_-airframeY);
            out.pitchRad=demand.pitch;out.rollRad=demand.roll;
        } else {
            sumX_=nextX;sumY_=nextY;
            const double ax=std::clamp(horizontalKp*ex+sumX_,-aMax,aMax),ay=std::clamp(horizontalKp*ey+sumY_,-aMax,aMax);
            out.pitchRad=pitchTrimRad-std::atan(ax/kG);
            out.rollRad=rollTrimRad+std::atan(ay*std::cos(out.pitchRad)/kG);
        }
    } else {
        sumX_ = sumY_ = 0.0;
        northHold_ = s.velocityNedMs[0], eastHold_ = s.velocityNedMs[1];
    }

    // the vertical speed on the throttle: the hover's, what the tilt takes from the thrust, the damping's share, the loop
    if ((ctx.engaged & axisBit(Axis::Thrust)) && std::abs(heavePower) > 1e-9) {
        const double target = orHold(c.verticalSpeedMs, 0.0);
        const double error = target - (-s.velocityNedMs[2]);
        const double tilt = std::max(std::cos(phi) * std::cos(theta), 0.5);
        const double feed = throttleTrim + ((1.0 / tilt - 1.0) * kG + heaveDamping * target) / heavePower;
        const double next = sumZ_ + verticalKi * error * dt;
        // Force-aware collective and accepted integral were computed before
        // tilt allocation. Its measured drag already covers the corresponding
        // identified damping; the legacy path below retains that approximation.
        if(!forceAware) {
        const double u = feed + (verticalKp * error + next) / heavePower;
        if (u >= 0.0 && u <= 1.0) {
            sumZ_ = next;
            out.throttle = u;
        } else {
            out.throttle = std::clamp(feed + (verticalKp * error + sumZ_) / heavePower, 0.0, 1.0);
        }
        }
    } else {
        sumZ_ = 0.0;
    }

    // the heading: given, or a turn rate's (kept within a radian of the nose, so it never laps it)
    if (ctx.engaged & axisBit(Axis::Yaw)) {
        if (!isHold(c.headingRad)) headingRef_ = c.headingRad;
        else if (!isHold(c.turnRateRadS)) headingRef_ = psi + std::clamp(geo::wrapPi(headingRef_ + c.turnRateRadS * dt - psi), -1.0, 1.0);
        out.headingRad = headingRef_;
    } else {
        headingRef_ = psi;
    }
    return out;
}

void RotorVelocity::reset() {
    started_ = false;
    lastTime_ = -1.0;
    sumX_ = sumY_ = sumZ_ = 0.0;
}

// --- RotorPosition ---------------------------------------------------------------

RotorPosition::RotorPosition() {
    params_.add("horizontal.gain", &horizontalGain);
    params_.add("max_speed", &maxSpeedMs);
    params_.add("deceleration", &decelerationMs2);
    params_.add("velocity.lag_s", &velocityLagS);
    params_.add("altitude.gain", &altitudeGain);
    params_.add("max_vertical_speed", &maxVerticalSpeedMs);
}

Command RotorPosition::update(const ControlContext& ctx, const Command& in) {
    const auto& c = std::get<PositionCommand>(in);
    const auto& s = ctx.sensed;
    VelocityCommand out{kHold, kHold, kHold, kHold, kHold, kHold};
    double north, east; // the point, from here
    geo::localNorthEastM(s.latitudeRad, s.longitudeRad, c.latitudeRad, c.longitudeRad, north, east);
    const double distance = std::hypot(north, east);
    if (ctx.engaged & kCyclic) {
        // as fast as it may go, and no faster than it can stop from - the velocity loop's lag
        // first, then the deceleration (v tau + v^2 / 2a = distance): arriving, it slows to a halt
        const double a = decelerationMs2, tau = velocityLagS;
        const double stopping = a * (std::sqrt(tau * tau + 2.0 * distance / a) - tau);
        constexpr double beta = 11.0 / 16.0;
        // Damping must not reduce the declared steady transit speed by
        // 1/(1+beta). Normalize its speed reference to unit DC gain, while
        // retaining the exact near-capture position coefficient and poles.
        const double speed = std::min({orHold(c.airspeedMs, maxSpeedMs), horizontalGain * distance / (1.0+beta), stopping});
        // The velocity PI integrates the complete setpoint error too.
        // Ki=Kv^2/4, Kx=27Kv/64 and beta=11/16 give the ideal polynomial
        // (s+3Kv/16)(s+3Kv/4)^2, a fourfold-separated critically damped pair.
        // This ground-velocity damping also opposes capture momentum.
        const double vn = (distance > 1e-6 ? (1.0+beta) * speed * north / distance : 0.0) - beta * s.velocityNedMs[0];
        const double ve = (distance > 1e-6 ? (1.0+beta) * speed * east / distance : 0.0) - beta * s.velocityNedMs[1];
        const double demand = std::hypot(vn, ve), limit = orHold(c.airspeedMs, maxSpeedMs);
        const double scale = demand > limit && demand > 0.0 ? limit / demand : 1.0;
        out.northMs = scale * vn;
        out.eastMs = scale * ve;
    }
    if (ctx.engaged & axisBit(Axis::Thrust))
        out.verticalSpeedMs = std::clamp(altitudeGain * (c.altitudeMslM - s.altitudeMslM), -maxVerticalSpeedMs, maxVerticalSpeedMs);
    if (ctx.engaged & axisBit(Axis::Yaw)) {
        if (!isHold(c.headingRad)) out.headingRad = c.headingRad;
        else if (distance > std::max(20.0, c.captureRadiusM)) out.headingRad = std::atan2(east, north);
        // near it and without a heading: kHold, the heading it has
    }
    return out;
}

// --- HoverBehavior ---------------------------------------------------------------

void HoverBehavior::start(const ControlContext& ctx, const BehaviorCommand& command) {
    const auto& s = ctx.sensed;
    target_ = PositionCommand{};
    target_.latitudeRad = command.param("lat_deg", units::radiansToDegrees(s.latitudeRad)) * units::kDegreesToRadians;
    target_.longitudeRad = command.param("lon_deg", units::radiansToDegrees(s.longitudeRad)) * units::kDegreesToRadians;
    target_.altitudeMslM = command.param("altitude_m", s.altitudeMslM);
    target_.headingRad = command.param("heading_deg", units::radiansToDegrees(s.eulerRad[2])) * units::kDegreesToRadians;
    target_.airspeedMs = kHold;
    target_.captureRadiusM = 1.0;
}

Command HoverBehavior::update(const ControlContext& ctx, const Command&) {
    lat_ = ctx.sensed.latitudeRad, lon_ = ctx.sensed.longitudeRad;
    return target_;
}

bool HoverBehavior::progress(ActivityProgress& out) const noexcept {
    out.headingRad = target_.headingRad;
    out.altitudeMslM = target_.altitudeMslM;
    out.speedMs = 0.0;
    out.speedReference = 2.0; // SpeedReference::GroundSpeed: none, it holds a point
    out.distanceToGoM = geo::distanceM(lat_, lon_, target_.latitudeRad, target_.longitudeRad);
    return true;
}

// --- Registration ----------------------------------------------------------------

void registerRotorControllers(ControllerRegistry& r) {
    r.add("rotor_allocation", Level::Acceleration, [] { return std::make_unique<RotorAllocation>(); });
    r.add("rotor_attitude", Level::Attitude, [] { return std::make_unique<RotorAttitude>(); });
    r.add("rotor_velocity", Level::Velocity, [] { return std::make_unique<RotorVelocity>(); });
    r.add("rotor_position", Level::Position, [] { return std::make_unique<RotorPosition>(); });
    constexpr double now = kHold, inf = std::numeric_limits<double>::infinity();
    auto p = [](const char* name, const char* unit, double def, double lo, double hi) { return ParameterInfo{name, unit, lo, hi, def, true}; };
    BehaviorTraits hover{Persistence::Persistent,
                         {p("lat_deg", "deg", now, -90.0, 90.0), p("lon_deg", "deg", now, -180.0, 180.0), p("altitude_m", "m", now, -inf, inf),
                          p("heading_deg", "deg", now, -inf, inf)},
                         {"fsim.flight.position"},
                         false,
                         kFeatureHover,
                         FlightMode::Loiter}; // A-GRA's LOITER, its hover
    r.addBehavior("hover", [] { return std::make_unique<HoverBehavior>(); }, std::move(hover));
}

} // namespace fsim::control
