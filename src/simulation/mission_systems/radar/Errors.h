#pragma once
// Explicit measurement-level residual error mechanisms. Equipment coefficients
// require independent calibration; these equations do not supply those data.
#include "simulation/mission_systems/radar/Signal.h"
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <algorithm>
#include <stdexcept>

namespace fsim::ms::radar {

struct ErrorParameters {
    double azimuthBiasRad=0,elevationBiasRad=0;
    double radomeAzimuthSlope=0,radomeElevationSlope=0;
    double driftStdRad=0,correlationS=60;
    double glintExtentFraction=0;
    double reflectionCoefficient=0,multipathStdRad=0;
    bool valid()const noexcept {
        for(double v:{azimuthBiasRad,elevationBiasRad,radomeAzimuthSlope,radomeElevationSlope,driftStdRad,
                      correlationS,glintExtentFraction,reflectionCoefficient,multipathStdRad})if(!std::isfinite(v))return false;
        return driftStdRad>=0&&correlationS>0&&glintExtentFraction>=0&&std::fabs(reflectionCoefficient)<=1&&multipathStdRad>=0;
    }
    bool enabled()const noexcept {
        return azimuthBiasRad!=0||elevationBiasRad!=0||radomeAzimuthSlope!=0||radomeElevationSlope!=0||
               driftStdRad>0||glintExtentFraction>0||reflectionCoefficient!=0;
    }
};

class MeasurementErrors {
public:
    explicit MeasurementErrors(ErrorParameters p={},std::uint64_t seed=1):p_(p),stream_(seed){if(!p.valid())throw std::invalid_argument("Invalid radar residual-error model");}
    void reseed(std::uint64_t seed){stream_.reseed(seed);time_=std::numeric_limits<double>::quiet_NaN();drift_={};}
    void advance(double time) {
        if(!p_.enabled())return;
        if(p_.driftStdRad>0) {
            const double rho=std::isfinite(time_)?std::exp(-std::max(0.,time-time_)/p_.correlationS):0.;
            const double sigma=p_.driftStdRad*std::sqrt(std::max(0.,1-rho*rho));
            if(sigma>0)for(double& b:drift_)b=rho*b+sigma*stream_.normal();
        }
        time_=time;
    }
    std::array<double,2> bias(double az,double el,double range,double extentRmsM) {
        std::array<double,2> out={p_.azimuthBiasRad+p_.radomeAzimuthSlope*az+drift_[0],
                                  p_.elevationBiasRad+p_.radomeElevationSlope*el+drift_[1]};
        const double sigma=glintSigma(range,extentRmsM);
        if(sigma>0)for(double& b:out) {
            // Student-t4 normalized to unit variance: a finite-variance,
            // heavy-tailed extended-target residual, not white thermal noise.
            double chi=0;for(int i=0;i<4;++i){const double n=stream_.normal();chi+=n*n;}
            if(!(chi>0))b=std::numeric_limits<double>::quiet_NaN();
            else b+=sigma*std::sqrt(2/chi)*stream_.normal();
        }
        return out;
    }
    double glintSigma(double range,double extentRmsM)const noexcept {
        return range>0&&extentRmsM>0?p_.glintExtentFraction*extentRmsM/range:0.;
    }
    double angularVariance(double range,double extentRmsM,bool multipath)const noexcept {
        const double g=glintSigma(range,extentRmsM);
        return g*g+p_.driftStdRad*p_.driftStdRad+(multipath?p_.multipathStdRad*p_.multipathStdRad:0.);
    }
    /// Coherent direct/image arrival centroid. Propagation power remains owned
    /// by the radio-path model; this models the unresolved angular residual.
    static bool reflectedDirection(const std::array<double,3>& direct,const std::array<double,3>& image,
                                   double pathDifference,double wavelength,double reflection,std::array<double,3>& out) {
        if(!(wavelength>0)||!std::isfinite(pathDifference)||!std::isfinite(reflection)||std::fabs(reflection)>1)return false;
        constexpr double tau=6.2831853071795864769;
        const std::complex<double> echo=std::polar(1.,std::remainder(tau*pathDifference/wavelength,tau))*reflection;
        const auto total=1.+echo;
        if(!(std::norm(total)>std::numeric_limits<double>::epsilon()))return false; // null: no valid angle estimate
        const double weight=(echo/total).real();double norm=0;
        for(std::size_t i=0;i<3;++i){out[i]=direct[i]+weight*(image[i]-direct[i]);norm+=out[i]*out[i];}
        if(!(norm>0)||!std::isfinite(norm))return false;
        for(double& v:out)v/=std::sqrt(norm);
        return true;
    }
private:
    ErrorParameters p_;
    signal::Stream stream_;
    std::array<double,2> drift_{};
    double time_=std::numeric_limits<double>::quiet_NaN();
};

} // namespace fsim::ms::radar
