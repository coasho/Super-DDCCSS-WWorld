#pragma once
// Ownship inertial-error mechanization. Never a source of target knowledge.
#include "Guidance.h"
#include "fsim/Rng.h"
#include "simulation/mission_systems/track/Linalg.h"
#include <array>
#include <limits>

namespace fsim::ms::weapon {
/// Explicit engineering policy, not measured missile IMU/GNSS performance.
/// Noise densities are two-sided continuous covariance intensities' square
/// roots, in m/s^2 sqrt(s) and m/s^2/sqrt(s), respectively.
struct NavigationConfigV1 {
    bool gnssAiding=false;
    double imuPeriodS=.01,gnssPeriodS=1;
    double accelerometerNoiseDensity=.03,biasSigmaMs2=.01,biasWalkDensity=.0001;
    double initialPositionSigmaM=5,initialVelocitySigmaMs=.5;
    double gnssPositionSigmaM=3,gnssVelocitySigmaMs=.1;
};
/// Native ownship scene inputs. Position/velocity are used only at initialization
/// or a fitted, available GNSS exposure. Acceleration is ECEF total kinematic
/// acceleration after gravity/Earth-frame resolution, before sensor error.
struct NavigationInputV1 {
    double timeS=0;
    V3 position,velocity,acceleration;
    bool gnssAvailable=true;
};
struct NavigationEstimateV1 {
    double timeS=0,lastImuSampleS=0;
    double lastGnssSampleS=std::numeric_limits<double>::quiet_NaN();
    V3 position,velocity;
    std::array<double,36> covariance{}; ///< interleaved x,vx,y,vy,z,vz
    bool valid=false,gnssAided=false;
};
class NavigationV1 {
public:
    NavigationV1(NavigationConfigV1,std::uint64_t seed);
    bool initialize(const NavigationInputV1&);
    bool advance(const NavigationInputV1&);
    NavigationEstimateV1 estimate() const noexcept;
    void reset() noexcept;
private:
    struct Axis {
        std::array<double,4> state{}; ///< position, velocity, bias, current held white error
        track::Mat<4,4> covariance;
        double actualBias=0,heldAcceleration=0;
    };
    bool advanceImpl(const NavigationInputV1&);
    void propagate(double dt);
    void imu(const NavigationInputV1&,bool first);
    void gnss(const NavigationInputV1&);
    static bool valid(const NavigationInputV1&) noexcept;
    NavigationConfigV1 config_;
    std::uint64_t seed_=0,nextImu_=1,nextGnss_=1;
    Rng initialRng_,imuRng_,biasRng_,gnssRng_;
    std::array<Axis,3> axes_{};
    NavigationInputV1 previous_;
    double epoch_=0,time_=0,lastImu_=0;
    double lastGnss_=std::numeric_limits<double>::quiet_NaN();
    bool initialized_=false,gnssAidingValid_=false;
};
}
