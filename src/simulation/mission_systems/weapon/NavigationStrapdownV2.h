#pragma once
// Additive private body-IMU strapdown navigation. NavigationV1 is unchanged.
#include "NavigationV1.h"

namespace fsim::ms::weapon {
struct NavigationStrapdownConfigV2 {
    bool gnssAiding=false;
    double imuPeriodS=.01,gnssPeriodS=1,maximumIntegrationStepS=.005;
    double accelerometerNoiseDensity=.03,gyroNoiseDensity=.0001;
    double accelerometerBiasSigma=.01,gyroBiasSigma=.0001;
    double accelerometerBiasWalkDensity=.0001,gyroBiasWalkDensity=.000001;
    double initialPositionSigmaM=5,initialVelocitySigmaMs=.5,initialAttitudeSigmaRad=.001;
    double gnssPositionSigmaM=3,gnssVelocitySigmaMs=.1;
    double earthRateRadS=7.292115e-5,earthMuM3S2=3.986004418e14;
    double earthEquatorialRadiusM=6378137,earthJ2=1.08262982e-3;
};
/// Captured OWN native motion, never target knowledge or estimator correction.
/// Body axes are right-handed orthonormal body-to-ECEF matrix COLUMNS.
/// Rates are body relative ECEF, resolved in body axes (JSBSim GetPQR(),
/// not GetPQRi()). The sensor adds C' * Earth rate to produce inertial gyro.
/// Acceleration is total ECEF kinematic acceleration, not specific force.
/// Position/velocity/attitude initialize once; later captures only synthesize
/// physical IMU measurements and, when explicitly fitted/available, GNSS.
struct NavigationStrapdownInputV2 {
    double timeS=0;
    V3 position,velocity,acceleration;
    std::array<V3,3> bodyToEcef{{{1,0,0},{0,1,0},{0,0,1}}};
    V3 angularRateBodyRelativeEcefRadS;
    bool gnssAvailable=true;
};
struct NavigationImuSampleV2 {
    double timeS=0;
    V3 gyroRadS,specificForceMs2;
    bool valid=false;
};
class NavigationStrapdownV2 {
public:
    NavigationStrapdownV2(NavigationStrapdownConfigV2,std::uint64_t seed);
    bool initialize(const NavigationStrapdownInputV2&);
    bool advance(const NavigationStrapdownInputV2&);
    /// Same interleaved ECEF six-state output as V1: x,vx,y,vy,z,vz.
    NavigationEstimateV1 estimate() const noexcept;
    /// Hamilton active quaternion [w,x,y,z], mapping body vectors to ECEF.
    std::array<double,4> attitudeBodyToEcef() const noexcept {return quaternion_;}
    /// Row-major error covariance: dp,dv,dtheta_ECEF,dba_body,dbg_body,
    /// held accelerometer white error, held gyro white error (three each).
    /// The last six states preserve within-exposure correlations; they are
    /// not six extra physical navigation states or additional noise sources.
    const std::array<double,441>& errorCovariance() const noexcept {return covariance_.a;}
    NavigationImuSampleV2 imuSample() const noexcept {return sample_;}
    V3 accelerometerBiasBodyMs2() const noexcept {return accelerometerBias_;}
    V3 gyroBiasBodyRadS() const noexcept {return gyroBias_;}
    /// Next real IMU/GNSS exposure; no state, clock or RNG mutation.
    double nextSensorTimeS() const noexcept;
    void reset() noexcept;
private:
    friend class NavigationCaptureV2;
    bool advanceImpl(const NavigationStrapdownInputV2&);
    bool finiteState() const noexcept;
    static bool valid(const NavigationStrapdownInputV2&) noexcept;
    void imu(const NavigationStrapdownInputV2&,bool first);
    void propagate(double dt);
    void integrate(double dt);
    void gnss(const NavigationStrapdownInputV2&);
    V3 gravity(V3) const;
    track::Mat<3,3> gravityGradient(V3) const;
    track::Mat<21,21> dynamics(V3 position,const std::array<double,4>& quaternion) const;
    NavigationStrapdownConfigV2 config_;
    std::uint64_t seed_=0,nextImu_=1,nextGnss_=1;
    Rng initialRng_,accelerometerRng_,gyroRng_,accelerometerBiasRng_,gyroBiasRng_,gnssRng_;
    NavigationStrapdownInputV2 previous_;
    NavigationImuSampleV2 sample_;
    V3 position_,velocity_,accelerometerBias_,gyroBias_,whiteAcceleration_,whiteGyro_;
    V3 actualAccelerometerBias_,actualGyroBias_;
    std::array<double,4> quaternion_{{1,0,0,0}};
    track::Mat<21,21> covariance_;
    double epoch_=0,time_=0,lastImu_=0;
    double lastGnss_=std::numeric_limits<double>::quiet_NaN();
    bool initialized_=false,gnssAidingValid_=false;
};
}
