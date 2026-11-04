#include "ThermalReticleReceiverV1.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace fsim::ms::components {
namespace {
using namespace measured;
using V=weapon::V3;
constexpr double pi=3.14159265358979323846;
double inner(const std::vector<double>& a,const std::vector<double>& b) {
    return std::inner_product(a.begin(),a.end(),b.begin(),0.);
}
double omega(double x,double y,double pitch) {
    const auto f=[](double a,double b){return std::atan(a*b/std::sqrt(1+a*a+b*b));};
    const double x0=std::tan(x-pitch/2),x1=std::tan(x+pitch/2),y0=std::tan(y-pitch/2),y1=std::tan(y+pitch/2);
    return f(x1,y1)-f(x0,y1)-f(x1,y0)+f(x0,y0);
}
std::vector<double> filtered(std::vector<double> values,double alpha) {
    double previous=0;
    for(auto& value:values){previous+=alpha*(value-previous);value=previous;}
    return values;
}
}

double ThermalReticleReceiverV1::maskV1(const ThermalReticleConfigV1& c,double x,double y,double phase) noexcept {
    x+=c.scanRadiusRad*std::cos(phase);y+=c.scanRadiusRad*std::sin(phase);
    if(!std::isfinite(x)||!std::isfinite(y)||x*x+y*y>c.stopRadiusRad*c.stopRadiusRad)return 0;
    return std::cos(c.periods*std::atan2(y,x))>=0?1.:0.;
}
double ThermalReticleReceiverV1::angularSpeedV1(double x,double y,double radius,double phase,double frequency) noexcept {
    const double p=x*std::cos(phase)+y*std::sin(phase),denominator=x*x+y*y+2*radius*p+radius*radius;
    return denominator>0?2*pi*frequency*radius*(radius+p)/denominator:std::numeric_limits<double>::quiet_NaN();
}
bool ThermalReticleReceiverV1::validConfig(const ThermalReticleConfigV1& c) noexcept {
    const auto& p=c.detector;
    return c.version==1&&ThermalImageReceiverV1::validConfig(p)&&p.width<=128&&p.height<=128&&
        c.periods>=3&&c.periods<=64&&c.samplesPerScan>=8*c.periods&&c.samplesPerScan<=512&&
        c.spatialSamples>=2&&c.spatialSamples<=8&&std::isfinite(c.scanRadiusRad)&&c.scanRadiusRad>0&&
        std::isfinite(c.stopRadiusRad)&&c.stopRadiusRad>c.scanRadiusRad&&c.stopRadiusRad<=.5*std::min(p.width,p.height)*p.pixelPitchRad&&
        std::isfinite(c.frequencyHz)&&c.frequencyHz>0&&c.frequencyHz<=2000&&
        p.integrationTimeS*c.frequencyHz*c.samplesPerScan<=1&&
        static_cast<std::size_t>(c.samplesPerScan)*p.exposureSamples*p.width*p.height<=4194304;
}
void ThermalReticleReceiverV1::project(std::vector<double>& values) const {
    for(const auto& column:nuisance_) {
        const double coefficient=inner(values,column);
        for(std::size_t i=0;i<values.size();++i)values[i]-=coefficient*column[i];
    }
}
std::vector<double> ThermalReticleReceiverV1::spot(double x,double y,double sigma,double width,double height,bool removeNuisance) const {
    const auto& c=config_;const auto& p=c.detector;
    const auto pixels=static_cast<std::size_t>(p.width)*p.height;
    std::vector<double> result(c.samplesPerScan,0);
    const auto primitive=[&](double edge,double centre){return .5*std::erfc((centre-edge)/(std::sqrt(2.)*sigma));};
    const auto probability=[&](double low,double high,double centre,double extent) {
        return extent==0?primitive(high,centre)-primitive(low,centre):
            ThermalImageReceiverV1::gaussianIntervalV1(centre-.5*extent,centre+.5*extent,low,high,sigma)/extent;
    };
    // Explicit nine-sigma numerical support, matching the retained optical
    // renderer's tail bound; lost FOV energy is never renormalized.
    for(unsigned row=0;row<p.height;++row) {
        const double lowY=(row-.5*p.height)*p.pixelPitchRad,highY=lowY+p.pixelPitchRad;
        if(lowY>y+.5*height+9*sigma||highY<y-.5*height-9*sigma)continue;
        const double wy=probability(lowY,highY,y,height);
        for(unsigned col=0;col<p.width;++col) {
            const double lowX=(col-.5*p.width)*p.pixelPitchRad,highX=lowX+p.pixelPitchRad;
            if(lowX>x+.5*width+9*sigma||highX<x-.5*width-9*sigma)continue;
            const auto index=static_cast<std::size_t>(row)*p.width+col;
            const double weight=wy*probability(lowX,highX,x,width)*solidAngles_[index]/(p.pixelPitchRad*p.pixelPitchRad);
            for(unsigned j=0;j<c.samplesPerScan;++j)result[j]+=weight*weights_[static_cast<std::size_t>(j)*pixels+index];
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    result=filtered(std::move(result),alpha);if(removeNuisance)project(result);return result;
}
bool ThermalReticleReceiverV1::configure(const ThermalReticleConfigV1& c,std::string& error) {
    if(!validConfig(c)){error="invalid finite conical-scan reticle policy";return false;}
    // Transactional configuration: no rejected policy mutates an active receiver.
    ThermalReticleReceiverV1 next;next.config_=c;
    const auto& p=c.detector;const auto pixels=static_cast<std::size_t>(p.width)*p.height;
    next.weights_.assign(static_cast<std::size_t>(c.samplesPerScan)*pixels,0);
    next.timeWeights_.resize(next.weights_.size()*p.exposureSamples);
    next.solidAngles_.resize(pixels);next.flat_.assign(c.samplesPerScan,0);
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto index=static_cast<std::size_t>(row)*p.width+col;
        const double x=(col+.5-.5*p.width)*p.pixelPitchRad,y=(row+.5-.5*p.height)*p.pixelPitchRad;
        next.solidAngles_[index]=omega(x,y,p.pixelPitchRad);
        for(unsigned j=0;j<c.samplesPerScan;++j)for(unsigned k=0;k<p.exposureSamples;++k) {
            const double phase=2*pi*((j+.5)/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
            double transmission=0;
            for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx)
                transmission+=maskV1(c,x+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad,
                    y+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad,phase);
            transmission/=static_cast<double>(c.spatialSamples)*c.spatialSamples;
            next.timeWeights_[(static_cast<std::size_t>(j)*p.exposureSamples+k)*pixels+index]=transmission;
            next.weights_[static_cast<std::size_t>(j)*pixels+index]+=transmission/p.exposureSamples;
        }
        for(unsigned j=0;j<c.samplesPerScan;++j)next.flat_[j]+=next.solidAngles_[index]*next.weights_[static_cast<std::size_t>(j)*pixels+index];
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    const auto addNuisance=[&](std::vector<double> values) {
        next.project(values);const double length=std::sqrt(inner(values,values));
        if(length>128*std::numeric_limits<double>::epsilon()*std::sqrt(static_cast<double>(values.size()))) {
            for(auto& v:values)v/=length;
            next.nuisance_.push_back(std::move(values));
        }
    };
    addNuisance(std::vector<double>(c.samplesPerScan,1));
    std::vector<double> transient(c.samplesPerScan);
    for(unsigned j=0;j<c.samplesPerScan;++j)transient[j]=std::pow(1-alpha,j+1);
    addNuisance(std::move(transient));addNuisance(filtered(next.flat_,alpha));
    for(unsigned harmonic=1;harmonic<=2;++harmonic)for(unsigned quadrature=0;quadrature<2;++quadrature) {
        std::vector<double> basis(c.samplesPerScan);
        for(unsigned j=0;j<c.samplesPerScan;++j) {
            const double phase=2*pi*(j+.5)/c.samplesPerScan*harmonic;
            basis[j]=quadrature?std::sin(phase):std::cos(phase);
        }
        addNuisance(filtered(std::move(basis),alpha));
    }
    next.templates_.reserve(pixels*c.samplesPerScan);
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto values=next.spot((col+.5-.5*p.width)*p.pixelPitchRad,(row+.5-.5*p.height)*p.pixelPitchRad,p.psfSigmaRad);
        next.templates_.insert(next.templates_.end(),values.begin(),values.end());
    }
    *this=std::move(next);return true;
}
void ThermalReticleReceiverV1::reset() noexcept {
    startedS_=lastTimeS_=lastDetectorSampleTimeS_=-1;detectorCharge_=expectedCharge_=0;seeded_=false;cached_={};
}
double ThermalReticleReceiverV1::normal() {
    const auto uniform=[&]{return (static_cast<double>(rng_()>>11)+.5)/9007199254740992.;};
    return std::sqrt(-2*std::log(uniform()))*std::cos(2*pi*uniform());
}
std::optional<std::array<double,3>> ThermalReticleReceiverV1::captureWindowV1(double readTimeS) const noexcept {
    if(startedS_<0||!std::isfinite(readTimeS)||readTimeS<startedS_)return std::nullopt;
    double scans=std::floor((readTimeS-startedS_)*config_.frequencyHz);
    if(!std::isfinite(scans))return std::nullopt;
    if(scans<1)return std::nullopt;
    double end=startedS_+scans/config_.frequencyHz;
    if(end>readTimeS){scans-=1;end=startedS_+scans/config_.frequencyHz;}
    if(scans<1||!std::isfinite(end)||end>readTimeS)return std::nullopt;
    const double start=scans==1?startedS_:startedS_+(scans-1)/config_.frequencyHz;
    if(!(start>=startedS_&&end>start))return std::nullopt;
    return std::array<double,3>{start,.5*(start+end),end};
}

ThermalReticleResultV1 ThermalReticleReceiverV1::expose(const ThermalSceneFrameV1& scene,V pointing,std::uint64_t seed) {
    const auto& c=config_;const auto& p=c.detector;
    if(!validConfig(c)||!ThermalImageReceiverV1::validScene(scene,p)||scene.timeS<lastTimeS_||!finite(pointing)||norm(pointing)<=0)return {};
    if(scene.timeS==lastTimeS_)return cached_;
    const auto beforeRng=rng_;const bool beforeSeeded=seeded_;
    const double beforeStarted=startedS_,beforeDetectorTime=lastDetectorSampleTimeS_,beforeCharge=detectorCharge_,beforeExpected=expectedCharge_;
    const auto refuse=[&]() {
        rng_=beforeRng;seeded_=beforeSeeded;startedS_=beforeStarted;lastDetectorSampleTimeS_=beforeDetectorTime;
        detectorCharge_=beforeCharge;expectedCharge_=beforeExpected;return ThermalReticleResultV1{};
    };
    if(!seeded_){rng_.seed(seed^0x52455449434c4531ull);seeded_=true;startedS_=lastDetectorSampleTimeS_=scene.timeS;}
    ThermalReticleResultV1 result;result.valid=true;result.timeS=scene.timeS;result.pointing=unit(pointing);
    const auto window=captureWindowV1(scene.timeS);
    if(!window){cached_=result;lastTimeS_=scene.timeS;return result;}
    const double start=(*window)[0],end=(*window)[2];
    if(cached_.complete&&start==cached_.exposureStartTimeS){lastTimeS_=scene.timeS;return cached_;}
    result.complete=true;result.exposureStartTimeS=start;result.exposureEndTimeS=end;result.measurementTimeS=.5*(start+end);
    const auto pixels=static_cast<std::size_t>(p.width)*p.height;
    // Equivalent-area engineering electronics. This is not a claim of4096
    // physical read amplifiers or an OEM scalar detector specification.
    const double area=static_cast<double>(pixels),fullWell=area*p.fullWellElectrons;
    const double readNoise=std::sqrt(area)*p.readNoiseElectrons;
    const double dark=area*p.darkCurrentElectronsS*p.integrationTimeS;
    const double adc=fullWell/(std::ldexp(1.,static_cast<int>(p.adcBits))-1);
    result.fullWellElectrons=fullWell;result.readNoiseElectrons=readNoise;result.darkElectrons=dark;result.adcStepElectrons=adc;
    const double gain=p.apertureAreaM2*p.opticalTransmission*p.integrationTimeS*p.quantumEfficiency*p.effectiveWavelengthM/(6.62607015e-34*299792458.);
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    auto optical=p;optical.exposureSamples=1;optical.integrationTimeS=p.integrationTimeS/p.exposureSamples;
    std::vector<double> background;
    double maximumMeasured=0;
    for(unsigned j=0;j<c.samplesPerScan;++j) {
        double photons=0;
        const double middle=start+(j+.5)/(c.frequencyHz*c.samplesPerScan);
        for(unsigned k=0;k<p.exposureSamples;++k) {
            const double sample=middle+p.integrationTimeS*((k+.5)/p.exposureSamples-.5);
            auto exposure=scene;exposure.timeS=sample+.5*optical.integrationTimeS;
            for(auto& patch:exposure.patches)patch.direction=unit(subtract(patch.direction,scale(patch.directionRate,scene.timeS-exposure.timeS)));
            const auto field=ThermalImageReceiverV1::incidentFieldImplV1(exposure,pointing,optical,background.empty()?nullptr:&background);
            if(!field)return refuse();
            if(background.empty())background=field->backgroundRadiance;
            const auto offset=(static_cast<std::size_t>(j)*p.exposureSamples+k)*pixels;
            for(std::size_t i=0;i<pixels;++i) {
                if(!std::isfinite(field->radiance[i]))return refuse();
                photons+=std::max(0.,field->radiance[i])*solidAngles_[i]*timeWeights_[offset+i]*gain/p.exposureSamples;
            }
        }
        photons+=dark;
        // The positive finite-mean standard-library Poisson domain is used
        // without an ADC-derived input cap. The2^52 reference domain leaves
        // a factor-two margin below binary64's consecutive-integer limit.
        // Outside it we explicitly refuse rather than substitute
        // a clipped source or an unvalidated large-mean noise approximation.
        if(!std::isfinite(photons)||photons<0||photons>std::ldexp(1.,52))return refuse();
        result.incidentElectrons.push_back(photons);
        // Exact first-order ZOH update for the actual elapsed detector clock,
        // including the unobserved interval between requested readout windows.
        // Its unknown initial transient is a DSP nuisance, never a bearing.
        const double elapsed=middle-lastDetectorSampleTimeS_;
        if(!(elapsed>0))return refuse();
        const double response=-std::expm1(-elapsed/p.pixelResponseTimeS);
        expectedCharge_+=response*(photons-expectedCharge_);result.expectedElectrons.push_back(expectedCharge_);
        double shot=0;
        // mean==0 is a deterministic zero, not a violation of the library's
        // strict positive-mean constructor precondition. Full well belongs
        // ONLY to the post-response readout below, never to incident charge.
        if(photons>0){std::poisson_distribution<std::uint64_t> draw(photons);shot=static_cast<double>(draw(rng_));}
        detectorCharge_+=response*(shot-detectorCharge_);lastDetectorSampleTimeS_=middle;
        const double raw=detectorCharge_+normal()*readNoise;
        result.saturated=result.saturated||raw<=0||raw>=fullWell;
        const double measured=std::round(std::clamp(raw,0.,fullWell)/adc)*adc;
        result.measuredElectrons.push_back(measured);result.phases.push_back(2*pi*(j+.5)/c.samplesPerScan);
        maximumMeasured=std::max(maximumMeasured,measured);
    }
    // An explicit conservative Gaussian high-count screening covariance:
    // stable first-order convolution has l2 gain<=1. Estimate the input upper
    // scale from MEASURED ADC charge and finite response/noise, never photons.
    const double inputUpper=(maximumMeasured+p.detectionSigma*(std::sqrt(maximumMeasured)+readNoise)+adc)/alpha;
    result.varianceBoundElectrons2=inputUpper+readNoise*readNoise+adc*adc/12;
    if(!result.saturated) {
        fit(result,pointing,scene.up);
        if(result.features.empty())fitHorizon(result,pointing,scene.up);
    }
    cached_=result;lastTimeS_=scene.timeS;return result;
}

void ThermalReticleReceiverV1::fit(ThermalReticleResultV1& result,V pointing,V up) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    auto data=result.measuredElectrons;project(data);
    const double dataNorm=inner(data,data),variance=result.varianceBoundElectrons2;
    if(!std::isfinite(dataNorm)||!std::isfinite(variance)||variance<=0)return;
    double best=0,bestX=0,bestY=0,bestWidth=0,bestHeight=0,bestAmplitude=0,bestNorm=0;
    const auto consider=[&](const double* values,double x,double y,double width,double height) {
        double correlation=0,norm2=0;
        for(unsigned j=0;j<count;++j){correlation+=data[j]*values[j];norm2+=values[j]*values[j];}
        if(norm2<=128*std::numeric_limits<double>::epsilon())return;
        const double explained=correlation*correlation/norm2;
        if(explained>best){best=explained;bestX=x;bestY=y;bestWidth=width;bestHeight=height;bestAmplitude=correlation/norm2;bestNorm=norm2;}
    };
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto index=static_cast<std::size_t>(row)*p.width+col;
        consider(templates_.data()+index*count,(col+.5-.5*p.width)*p.pixelPitchRad,(row+.5-.5*p.height)*p.pixelPitchRad,0,0);
    }
    // A real unresolved compact radiator has finite unknown support. Fit its
    // rectangle-convolved PSF from the scalar waveform rather than copying
    // the scene's extent or rejecting every high-SNR non-point radiator.
    for(const double w:{0.,1.,2.,4.,8.,16.,32.})for(const double h:{0.,1.,2.,4.,8.,16.,32.}) {
        const double width=w*p.pixelPitchRad,height=h*p.pixelPitchRad;
        const auto values=spot(bestX,bestY,p.psfSigmaRad,width,height);
        consider(values.data(),bestX,bestY,width,height);
    }
    double step=p.pixelPitchRad;
    for(unsigned level=0;level<18;++level) {
        step*=.5;
        for(unsigned sweep=0;sweep<3;++sweep)for(unsigned coordinate=0;coordinate<4;++coordinate) {
            const std::array<double,4> centre{bestX,bestY,bestWidth,bestHeight};
            for(const double sign:{-1.,1.}) {
                auto q=centre;q[coordinate]+=sign*step;
                if(std::abs(q[0])>.5*p.width*p.pixelPitchRad||std::abs(q[1])>.5*p.height*p.pixelPitchRad||
                   q[2]<0||q[3]<0||q[2]>p.width*p.pixelPitchRad||q[3]>p.height*p.pixelPitchRad)continue;
                const auto values=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);consider(values.data(),q[0],q[1],q[2],q[3]);
            }
        }
    }
    result.fitSnr=std::sqrt(best/variance);result.fitResidual=std::max(0.,dataNorm-best)/variance;
    const auto forward=unit(pointing);auto right=unit(cross(forward,unit(up)));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    const auto emit=[&](double angularVariance) {
        ThermalImageFeatureV1 feature;feature.direction=unit(add(forward,add(scale(right,std::tan(bestX)),scale(vertical,std::tan(bestY)))));
        feature.signalToNoise=result.fitSnr;feature.angularVarianceRad2=angularVariance;
        feature.signedContrastElectrons=bestAmplitude;
        feature.extent.widthRad=bestWidth;feature.extent.heightRad=bestHeight;
        feature.centroidColumn=(bestX/p.pixelPitchRad)+.5*p.width;
        feature.centroidRow=(bestY/p.pixelPitchRad)+.5*p.height;
        result.features.push_back(feature);result.bearingResolved=true;
    };
    const double coarseVariance=std::max({p.pixelPitchRad*p.pixelPitchRad/12,
        (bestWidth*bestWidth+bestHeight*bestHeight)/12,step*step/12});
    // A source that fills the detector support has no unique bearing. The
    // coarse fallback is only for a compact source whose local fit is
    // ill-conditioned, never for extended DC/horizon clutter.
    const bool compactSource=bestWidth<.5*p.width*p.pixelPitchRad&&bestHeight<.5*p.height*p.pixelPitchRad;
    // A compact source truncated by the optical stop can fit a wide support.
    // Its best centre remains near the stop; keep the coarse measurement there
    // rather than losing an already detected edge source.
    const bool nearStop=std::hypot(bestX,bestY)>=.75*c.stopRadiusRad;
    // Bound the ENTIRE fixed final angle/support lattice, not merely the
    // number of data-adaptively selected refinement queries. This keeps the
    // nominal Gaussian trial screen honest without changing detectionSigma.
    const double subdivisions=std::ldexp(1.,18);
    const double logTrials=2*std::log(p.width*subdivisions+1)+2*std::log(p.height*subdivisions+1);
    const double threshold=std::sqrt(p.detectionSigma*p.detectionSigma+2*(std::log(2.)+logTrials));
    const double degrees=static_cast<double>(count)-static_cast<double>(nuisance_.size())-5,tail=.5*p.detectionSigma*p.detectionSigma;
    // Laurent-Massart upper tail for a Gaussian residual, disclosed as a
    // high-count engineering screen, not a calibrated equipment ROC.
    if(result.fitSnr<threshold||degrees<=0||bestNorm<=0)return;
    const double residualBound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    if(result.fitResidual>residualBound) {
        // The source cleared the detection gate but the compact-template model
        // does not explain an extended or truncated image. A detected source
        // still supplies a coarse measured bearing; its uncertainty is the
        // sampled support/quantization bound, not a precise local fit.
        if((compactSource||nearStop)&&std::isfinite(coarseVariance)&&coarseVariance>0)emit(coarseVariance);
        return;
    }
    const double h=p.pixelPitchRad/1024;
    auto xp=spot(bestX+h,bestY,p.psfSigmaRad,bestWidth,bestHeight),xm=spot(bestX-h,bestY,p.psfSigmaRad,bestWidth,bestHeight);
    auto yp=spot(bestX,bestY+h,p.psfSigmaRad,bestWidth,bestHeight),ym=spot(bestX,bestY-h,p.psfSigmaRad,bestWidth,bestHeight);
    const auto centre=spot(bestX,bestY,p.psfSigmaRad,bestWidth,bestHeight);const double norm2=inner(centre,centre);
    for(unsigned j=0;j<count;++j){xp[j]=(xp[j]-xm[j])/(2*h);yp[j]=(yp[j]-ym[j])/(2*h);}
    // Eliminate fitted support widths as nuisance parameters as well as the
    // unknown amplitude. An extent is a waveform fit, never a source oracle.
    std::vector<std::vector<double>> shape;
    for(unsigned coordinate=0;coordinate<2;++coordinate) {
        if((coordinate?bestHeight:bestWidth)<=h)continue;
        auto a=spot(bestX,bestY,p.psfSigmaRad,bestWidth+(coordinate?0:h),bestHeight+(coordinate?h:0));
        const auto b=spot(bestX,bestY,p.psfSigmaRad,bestWidth-(coordinate?0:h),bestHeight-(coordinate?h:0));
        for(unsigned j=0;j<count;++j)a[j]=(a[j]-b[j])/(2*h);
        const double amplitudeProjection=inner(a,centre)/norm2;
        for(unsigned j=0;j<count;++j)a[j]-=amplitudeProjection*centre[j];
        for(const auto& previous:shape){const double coefficient=inner(a,previous);for(unsigned j=0;j<count;++j)a[j]-=coefficient*previous[j];}
        const double length=std::sqrt(inner(a,a));
        if(length>128*std::numeric_limits<double>::epsilon()) {for(auto& value:a)value/=length;shape.push_back(std::move(a));}
    }
    for(const auto& column:shape) {
        const double a=inner(xp,column),b=inner(yp,column);
        for(unsigned j=0;j<count;++j){xp[j]-=a*column[j];yp[j]-=b*column[j];}
    }
    const double xx=inner(xp,xp)-std::pow(inner(xp,centre),2)/norm2;
    const double yy=inner(yp,yp)-std::pow(inner(yp,centre),2)/norm2;
    const double xy=inner(xp,yp)-inner(xp,centre)*inner(yp,centre)/norm2;
    const double determinant=xx*yy-xy*xy,trace=xx+yy;
    if(std::isfinite(determinant)&&determinant>128*std::numeric_limits<double>::epsilon()*trace*trace&&xx>0&&yy>0) {
        const double angularVariance=variance*(trace+std::hypot(xx-yy,2*xy))/(2*determinant*bestAmplitude*bestAmplitude)+step*step/12;
        if(std::isfinite(angularVariance)&&angularVariance>0) {emit(angularVariance);return;}
    }
    // The source cleared the detection and residual gates, but its local
    // angle/support Hessian is singular (edge truncation or unresolved extent).
    // A detected source must still produce a coarse measured bearing; its
    // uncertainty is the sampled template/support quantization, not a precise
    // local fit. This preserves an honest measurement without inventing angle
    // resolution the receiver did not achieve.
    if((compactSource||nearStop)&&std::isfinite(coarseVariance)&&coarseVariance>0)emit(coarseVariance);
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldV1(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,bool* accelerated) {
    if(accelerated)*accelerated=false;
    if(!ThermalImageReceiverV1::validConfig(p)||!finite(pointing)||norm(pointing)<=0||
       !finite(up)||std::abs(norm(up)-1)>1e-6||!std::isfinite(cutoff)||std::abs(cutoff)>1)return std::nullopt;
    if(region&&(!std::all_of(region->begin(),region->end(),[](double v){return std::isfinite(v)&&std::abs(v)<=1.55;})||
        !((*region)[1]>(*region)[0])||!((*region)[3]>(*region)[2])))return std::nullopt;
    const auto forward=unit(pointing),nativeUp=unit(up);
    auto right=unit(cross(forward,nativeUp));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    auto optical=p;optical.exposureSamples=1;
    const auto original=[&]()->std::optional<std::vector<double>> {
        ThermalSceneFrameV1 hypothesis;hypothesis.up=nativeUp;hypothesis.groundHorizonCosine=cutoff;hypothesis.groundRadianceWm2Sr=1;
        const auto ground=ThermalImageReceiverV1::incidentFieldImplV1(hypothesis,forward,optical,nullptr);
        if(!ground)return std::nullopt;
        auto answer=ground->radiance;
        if(region) {
            ThermalRadiancePatchV1 black;black.direction=unit(add(forward,add(scale(right,std::tan(.5*((*region)[0]+(*region)[1]))),
                scale(vertical,std::tan(.5*((*region)[2]+(*region)[3]))))));
            black.widthRad=(*region)[1]-(*region)[0];black.heightRad=(*region)[3]-(*region)[2];black.rangeM=1;black.transmission=1;
            hypothesis.patches.push_back(black);hypothesis.refereeAttributionIndices.emplace_back(std::nullopt);
            const auto outside=ThermalImageReceiverV1::incidentFieldImplV1(hypothesis,forward,optical,&ground->radiance);
            if(!outside)return std::nullopt;
            for(std::size_t i=0;i<answer.size();++i)answer[i]-=outside->radiance[i];
        }
        return answer;
    };
    const double pitch=p.pixelPitchRad,sigma=p.psfSigmaRad,margin=9*sigma,pixelArea=pitch*pitch;
    const double halfX=.5*p.width*pitch,halfY=.5*p.height*pitch;
    const double fieldLeft=std::max(-1.55,-halfX-margin),fieldRight=std::min(1.55,halfX+margin);
    const double fieldBottom=std::max(-1.55,-halfY-margin),fieldTop=std::min(1.55,halfY+margin);
    const double left=region?std::max(fieldLeft,(*region)[0]):fieldLeft;
    const double rightEdge=region?std::min(fieldRight,(*region)[1]):fieldRight;
    const double bottom=region?std::max(fieldBottom,(*region)[2]):fieldBottom;
    const double top=region?std::min(fieldTop,(*region)[3]):fieldTop;
    std::vector<double> answer(static_cast<std::size_t>(p.width)*p.height,0);
    if(!(rightEdge>left)||!(top>bottom)){if(accelerated)*accelerated=true;return answer;}
    const double a=dot(forward,nativeUp),b=dot(right,nativeUp),c=dot(vertical,nativeUp);
    // This sufficient monotonicity bound avoids a folded vertical cone. It
    // changes no optical domain: unsupported geometry uses the old renderer.
    const double maxTx=std::max(std::abs(std::tan(left)),std::abs(std::tan(rightEdge)));
    const double maxTy=std::max(std::abs(std::tan(bottom)),std::abs(std::tan(top)));
    const double maxN=std::nextafter(std::abs(a)+std::nextafter(std::abs(b)*maxTx,std::numeric_limits<double>::infinity()),std::numeric_limits<double>::infinity());
    if(!(std::nextafter(c,-std::numeric_limits<double>::infinity())>
        std::nextafter(maxN*maxTy,std::numeric_limits<double>::infinity())))return original();
    if(region) {
        // A finite support whose edge lands on the horizon cone is a mixed
        // endpoint quadrature. The accelerated interior-node rule cannot
        // represent the disappearing cap, so retain the exact original
        // renderer for this sensitive regional hypothesis.
        const auto horizon=[&](double x){return std::atan((cutoff-a-b*std::tan(x))/c);};
        const double h0=horizon(left),h1=horizon(rightEdge);
        const double low=std::min(h0,h1),high=std::max(h0,h1);
        const bool touchesField=(*region)[0]<=-halfX+1e-12||(*region)[1]>=halfX-1e-12||
            (*region)[2]<=-halfY+1e-12||(*region)[3]>=halfY-1e-12;
        if(touchesField&&(*region)[2]<=low+pitch&&(*region)[3]>=high-pitch)return original();
    }
    // For an uncut pixel, Gaussian convolution equals E[g(U+sigma Z)],
    // U uniform in the detector interval and Z standard normal. The omitted
    // source beyond the existing9-sigma support is <=2Q(9)<2.3e-19.
    // The rule changes numerical quadrature, never source-grid resolution.
    // Gauss rules: NIST DLMF3.5 (Legendre/Hermite). All weights are positive.
    struct Node {double x,w;};
    const double root3=std::sqrt(3.),root10=std::sqrt(10.),root30=std::sqrt(30.);
    const std::array<Node,2> l2{{{-1/root3,.5},{1/root3,.5}}};
    const std::array<Node,3> l3{{{-std::sqrt(.6),5./18},{0,4./9},{std::sqrt(.6),5./18}}};
    const double l4a=std::sqrt((3-2*std::sqrt(1.2))/7),l4b=std::sqrt((3+2*std::sqrt(1.2))/7);
    const std::array<Node,4> l4{{{-l4b,(18-root30)/72},{-l4a,(18+root30)/72},{l4a,(18+root30)/72},{l4b,(18-root30)/72}}};
    const std::array<Node,3> h3{{{-root3,1./6},{0,2./3},{root3,1./6}}};
    const std::array<Node,5> h5{{{-std::sqrt(5+root10),(7-2*root10)/60},{-std::sqrt(5-root10),(7+2*root10)/60},
        {0,8./15},{std::sqrt(5-root10),(7+2*root10)/60},{std::sqrt(5+root10),(7-2*root10)/60}}};
    const double sq2=std::sqrt(2.),sqPi=std::sqrt(pi);
    const std::array<Node,10> h10{{
        {-3.436159118837737603*sq2,.00000764043285523262063/sqPi},
        {-2.532731674232789797*sq2,.00134364574678123269/sqPi},
        {-1.756683649299881774*sq2,.0338743944554810631/sqPi},
        {-1.036610829789513654*sq2,.240138611082314687/sqPi},
        {-.342901327223704609*sq2,.610862633735325799/sqPi},
        {.342901327223704609*sq2,.610862633735325799/sqPi},
        {1.036610829789513654*sq2,.240138611082314687/sqPi},
        {1.756683649299881774*sq2,.0338743944554810631/sqPi},
        {2.532731674232789797*sq2,.00134364574678123269/sqPi},
        {3.436159118837737603*sq2,.00000764043285523262063/sqPi}}};
    const std::array<Node,8> l8{{{-.960289856497536232,.050614268145188130},{-.796666477413626740,.111190517226687235},
        {-.525532409916328986,.156853322938943644},{-.183434642495649805,.181341891689180991},
        {.183434642495649805,.181341891689180991},{.525532409916328986,.156853322938943644},
        {.796666477413626740,.111190517226687235},{.960289856497536232,.050614268145188130}}};
    const auto primitive=[](double z){return z*.5*std::erfc(-z/std::sqrt(2.))+std::exp(-.5*z*z)/std::sqrt(2*pi);};
    struct Row {double low=0,high=0,sourceLow=0,sourceHigh=0,full=0,constant=0,minCy=0,maxCy=0,minY2=0,maxY2=0;bool active=false;};
    std::array<Row,128> rows{};
    const auto minSquare=[](double low,double high){return low<=0&&high>=0?0.:std::min(low*low,high*high);};
    const auto bounds=[&](const std::array<double,4>& x,const Row& row) {
        const double n0=a+x[0]+row.minCy,n1=a+x[1]+row.maxCy;
        const double d0=std::sqrt(1+x[2]+row.minY2),d1=std::sqrt(1+x[3]+row.maxY2);
        return std::array<double,2>{std::min({n0/d0,n0/d1,n1/d0,n1/d1}),std::max({n0/d0,n0/d1,n1/d0,n1/d1})};
    };
    using Values=std::array<double,128>;
    // Y support, Gaussian primitives and full interval mass are independent
    // of detector column. Retain their exact values once per calibration.
    for(unsigned row=0;row<p.height;++row) {
        auto& info=rows[row];info.low=(row-.5*p.height)*pitch;info.high=info.low+pitch;
        info.sourceLow=std::max(bottom,info.low-margin);info.sourceHigh=std::min(top,info.high+margin);
        if(!(info.sourceHigh>info.sourceLow))continue;
        info.full=ThermalImageReceiverV1::gaussianIntervalV1(info.sourceLow,info.sourceHigh,info.low,info.high,sigma);
        info.constant=sigma*(primitive((info.high-info.sourceLow)/sigma)-primitive((info.low-info.sourceLow)/sigma));
        const double ty0=std::tan(info.sourceLow),ty1=std::tan(info.sourceHigh);
        info.minCy=std::min(c*ty0,c*ty1);info.maxCy=std::max(c*ty0,c*ty1);
        info.minY2=minSquare(ty0,ty1);info.maxY2=std::max(ty0*ty0,ty1*ty1);
    }
    for(unsigned col=0;col<p.width;++col) {
        const double px0=(col-.5*p.width)*pitch,px1=px0+pitch;
        const double sx0=std::max(left,px0-margin),sx1=std::min(rightEdge,px1+margin);
        if(!(sx1>sx0))continue;
        if(!region&&(px0-margin<=-1.55||px1+margin>=1.55))return original();
        const double ix=ThermalImageReceiverV1::gaussianIntervalV1(sx0,sx1,px0,px1,sigma);
        const double tx0=std::tan(sx0),tx1=std::tan(sx1);
        const std::array<double,4> xBounds{std::min(b*tx0,b*tx1),std::max(b*tx0,b*tx1),minSquare(tx0,tx1),std::max(tx0*tx0,tx1*tx1)};
        unsigned active=0;
        for(unsigned row=0;row<p.height;++row) {
            auto& info=rows[row];info.active=false;
            if(!(info.sourceHigh>info.sourceLow))continue;
            const auto range=bounds(xBounds,info);
            if(range[0]>=cutoff)continue;
            if(range[1]<cutoff){answer[static_cast<std::size_t>(row)*p.width+col]=ix*info.full/pixelArea;continue;}
            info.active=true;++active;
        }
        if(!active)continue;
        const auto at=[&](double x) {
            Values values{};const double tx=std::tan(x),n=a+b*tx;
            std::array<double,4> edges{bottom,top,0,0};unsigned size=2;
            const auto addRoot=[&](double t){const double y=std::atan(t);if(y>bottom&&y<top&&size<4)edges[size++]=y;};
            const double aa=c*c-cutoff*cutoff,bb=2*n*c,cc=n*n-cutoff*cutoff*(1+tx*tx);
            if(std::abs(aa)<1e-14){if(std::abs(bb)>1e-14)addRoot(-cc/bb);}
            else {const double disc=bb*bb-4*aa*cc;if(disc>=0){addRoot((-bb-std::sqrt(disc))/(2*aa));addRoot((-bb+std::sqrt(disc))/(2*aa));}}
            // At most two quadratic roots plus the two interval endpoints.
            // Explicit bounded insertion avoids library-sort speculative
            // accesses beyond this four-element array under GCC16.
            if(edges[1]<edges[0])std::swap(edges[1],edges[0]);
            if(size>2) {
                if(edges[2]<edges[1])std::swap(edges[2],edges[1]);
                if(edges[1]<edges[0])std::swap(edges[1],edges[0]);
            }
            if(size>3) {
                if(edges[3]<edges[2])std::swap(edges[3],edges[2]);
                if(edges[2]<edges[1])std::swap(edges[2],edges[1]);
                if(edges[1]<edges[0])std::swap(edges[1],edges[0]);
            }
            std::array<std::array<double,2>,3> ground{};unsigned pieces=0;
            for(unsigned part=1;part<size;++part) {
                const double y=.5*(edges[part-1]+edges[part]);
                const auto ray=unit(add(forward,add(scale(right,tx),scale(vertical,std::tan(y)))));
                if(dot(ray,nativeUp)<cutoff)ground[pieces++]={edges[part-1],edges[part]};
            }
            for(unsigned row=0;row<p.height;++row)if(rows[row].active) {
                const auto& info=rows[row];
                const auto prefix=[&](double t) {
                    if(t<=info.sourceLow)return 0.;
                    if(t>=info.sourceHigh)return info.full;
                    return std::clamp(info.constant-sigma*(primitive((info.high-t)/sigma)-primitive((info.low-t)/sigma)),0.,std::min(t-info.sourceLow,pitch));
                };
                for(unsigned part=0;part<pieces;++part) {
                    const double lo=std::max(info.sourceLow,ground[part][0]),hi=std::min(info.sourceHigh,ground[part][1]);
                    if(hi>lo)values[row]+=std::max(0.,prefix(hi)-prefix(lo));
                }
            }
            return values;
        };
        Values coarse{},fine{};
        if(!region) {
            const auto rule=[&](const auto& uniform,const auto& normal) {
                Values value{};
                for(const auto& u:uniform)for(const auto& z:normal) {
                    const auto sample=at(.5*(px0+px1)+.5*pitch*u.x+sigma*z.x);
                    for(unsigned row=0;row<p.height;++row)if(rows[row].active)value[row]+=u.w*z.w*sample[row]/pitch;
                }
                return value;
            };
            coarse=rule(l2,h3);fine=rule(l3,h5);
            bool converged=true;for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(fine[row]-coarse[row])>1e-9)converged=false;
            if(!converged) {
                coarse=fine;fine=rule(l4,h10);
                for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(fine[row]-coarse[row])>1e-9)return original();
            }
        } else {
            const auto rule=[&](double lo,double hi,const auto& nodes) {
                Values value{};const double middle=.5*(lo+hi),half=.5*(hi-lo);
                for(const auto& node:nodes) {
                    const double x=middle+half*node.x;
                    const double probability=.5*(std::erf((px1-x)/(std::sqrt(2.)*sigma))-std::erf((px0-x)/(std::sqrt(2.)*sigma)));
                    const auto sample=at(x);
                    for(unsigned row=0;row<p.height;++row)if(rows[row].active)value[row]+=2*half*node.w*probability*sample[row]/pixelArea;
                }
                return value;
            };
            bool converged=true;
            const auto integrate=[&](auto&& self,double lo,double hi,double tolerance,unsigned depth)->Values {
                const auto small=rule(lo,hi,l4),large=rule(lo,hi,l8);
                bool accepted=true;for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(large[row]-small[row])>tolerance)accepted=false;
                if(accepted)return large;
                if(depth==0){converged=false;return large;}
                const double middle=.5*(lo+hi);auto first=self(self,lo,middle,.5*tolerance,depth-1);
                const auto last=self(self,middle,hi,.5*tolerance,depth-1);
                for(unsigned row=0;row<p.height;++row)first[row]+=last[row];
                return first;
            };
            fine=integrate(integrate,sx0,sx1,1e-9,10);
            if(!converged)return original();
        }
        for(unsigned row=0;row<p.height;++row)if(rows[row].active) {
            if(!std::isfinite(fine[row])||fine[row]<0||fine[row]>1+1e-9)return original();
            answer[static_cast<std::size_t>(row)*p.width+col]=std::clamp(fine[row],0.,1.);
        }
    }
    if(accelerated)*accelerated=true;
    return answer;
}

void ThermalReticleReceiverV1::fitHorizon(ThermalReticleResultV1& result,V pointing,V up) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    const double variance=result.varianceBoundElectrons2;
    if(!std::isfinite(variance)||variance<=0)return;
    auto data=result.measuredElectrons;project(data);const double dataNorm=inner(data,data);
    const double subdivisions=std::ldexp(1.,18);
    const double logTrials=2*std::log(p.width*subdivisions+1)+3*std::log(p.height*subdivisions+1);
    const double threshold=std::sqrt(p.detectionSigma*p.detectionSigma+2*(std::log(2.)+logTrials));
    // Neither a nuisance fit nor a compact alternative can explain more
    // energy than the measured vector. This is exact rejection pruning,
    // not a lower source threshold or an approximate optical shortcut.
    if(dataNorm/variance<threshold*threshold)return;
    const double alpha=-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS));
    const double pixelArea=p.pixelPitchRad*p.pixelPitchRad;
    const auto forward=unit(pointing),verticalUp=unit(up);
    auto right=unit(cross(forward,verticalUp));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    const double margin=9*p.psfSigmaRad;
    const double halfX=.5*p.width*p.pixelPitchRad,halfY=.5*p.height*p.pixelPitchRad;
    const double radius=std::atan(std::hypot(std::tan(std::min(1.55,halfX+margin)),std::tan(std::min(1.55,halfY+margin))));
    const double tilt=std::acos(std::clamp(dot(forward,verticalUp),-1.,1.));
    const double low=std::cos(std::min(pi,tilt+radius)),high=std::cos(std::max(0.,tilt-radius));
    if(!(high>low))return;
    const double initialCutStep=(high-low)/p.height;
    struct Calibration {double cutoff=0;std::vector<double> field,wave;};
    // All scenes below are INSTRUMENT HYPOTHESES: unit ground, zero sky,
    // unknown cutoff and optional black support. No actual scene radiance,
    // source rectangle, identity or horizon threshold reaches this inverse.
    const auto waveform=[&](const std::vector<double>& field,bool removeNuisance) {
        std::vector<double> wave(count,0);
        for(unsigned j=0;j<count;++j)for(std::size_t i=0;i<field.size();++i)
            wave[j]+=field[i]*solidAngles_[i]*weights_[static_cast<std::size_t>(j)*field.size()+i]/pixelArea;
        wave=filtered(std::move(wave),alpha);if(removeNuisance)project(wave);return wave;
    };
    double nullExplained=0,nullCutoff=low,nullAmplitude=0,nullCost=std::numeric_limits<double>::infinity();
    std::vector<Calibration> calibrations;calibrations.reserve(512);
    const auto calibration=[&](double cutoff)->const Calibration* {
        for(const auto& item:calibrations)if(item.cutoff==cutoff)return &item;
        if(calibrations.size()>=512)return nullptr;
        const auto field=unitGroundFieldV1(p,forward,verticalUp,cutoff);
        if(!field)return nullptr;
        Calibration item;item.cutoff=cutoff;item.field=*field;item.wave=waveform(item.field,true);
        calibrations.push_back(std::move(item));const auto& added=calibrations.back();
        // The null sees EVERY cutoff visited by any alternative, including
        // derivative and alias hypotheses. Improved background fitting can
        // therefore never be credited exclusively to the compact source.
        const double norm2=inner(added.wave,added.wave);
        if(norm2>128*std::numeric_limits<double>::epsilon()) {
            const double correlation=inner(data,added.wave),amplitude=correlation/norm2;
            double cost=0;for(unsigned j=0;j<count;++j){const double residual=data[j]-amplitude*added.wave[j];cost+=residual*residual;}
            if(cost<nullCost){nullCost=cost;nullExplained=correlation*amplitude;nullCutoff=cutoff;nullAmplitude=amplitude;}
        }
        return &added;
    };
    for(unsigned row=0;row<=p.height;++row)calibration(low+(high-low)*row/p.height);
    double cutStep=initialCutStep;
    for(unsigned level=0;level<18;++level) {
        cutStep*=.5;const double centre=nullCutoff;
        for(const double sign:{-1.,1.})if(centre+sign*cutStep>=low&&centre+sign*cutStep<=high)calibration(centre+sign*cutStep);
    }
    if(nullExplained/variance<threshold*threshold)return;
    if(nullCost/variance<threshold*threshold)return;
    const auto* nullBasis=calibration(nullCutoff);if(!nullBasis)return;
    auto compactData=data;
    for(unsigned j=0;j<count;++j)compactData[j]-=nullAmplitude*nullBasis->wave[j];
    double startExplained=0,startX=0,startY=0,startWidth=0,startHeight=0;
    const auto considerStart=[&](const double* values,double x,double y,double width,double height) {
        double correlation=0,norm2=0;
        for(unsigned j=0;j<count;++j){correlation+=compactData[j]*values[j];norm2+=values[j]*values[j];}
        if(norm2<=128*std::numeric_limits<double>::epsilon())return;
        const double explained=correlation*correlation/norm2;
        if(explained>startExplained){startExplained=explained;startX=x;startY=y;startWidth=width;startHeight=height;}
    };
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto index=static_cast<std::size_t>(row)*p.width+col;
        considerStart(templates_.data()+index*count,(col+.5-.5*p.width)*p.pixelPitchRad,(row+.5-.5*p.height)*p.pixelPitchRad,0,0);
    }
    for(const double w:{0.,1.,2.,4.,8.,16.,32.})for(const double h:{0.,1.,2.,4.,8.,16.,32.}) {
        const auto values=spot(startX,startY,p.psfSigmaRad,w*p.pixelPitchRad,h*p.pixelPitchRad);
        considerStart(values.data(),startX,startY,w*p.pixelPitchRad,h*p.pixelPitchRad);
    }
    using Parameters=std::array<double,5>;
    const auto basis=[&](const Parameters& q,bool removeNuisance)->std::optional<std::array<std::vector<double>,2>> {
        const auto* ground=calibration(q[4]);if(!ground)return std::nullopt;
        std::vector<double> outside;
        if(q[2]>0&&q[3]>0) {
            const std::array<double,4> support{q[0]-.5*q[2],q[0]+.5*q[2],q[1]-.5*q[3],q[1]+.5*q[3]};
            const auto occulted=unitGroundFieldV1(p,forward,verticalUp,q[4],support);
            if(!occulted)return std::nullopt;
            auto field=ground->field;for(std::size_t i=0;i<field.size();++i)field[i]-=(*occulted)[i];
            outside=waveform(field,removeNuisance);
        } else outside=removeNuisance?ground->wave:waveform(ground->field,false);
        return std::array<std::vector<double>,2>{std::move(outside),spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],removeNuisance)};
    };
    double bestCost=std::numeric_limits<double>::infinity(),beta=0,contrast=0;
    Parameters bestQ{startX,startY,startWidth,startHeight,nullCutoff};
    const auto solve=[&](const Parameters& q)->std::optional<std::array<double,3>> {
        if(std::abs(q[0])>halfX||std::abs(q[1])>halfY||q[2]<0||q[3]<0||q[2]>2*halfX||q[3]>2*halfY||q[4]<low||q[4]>high)return std::nullopt;
        const auto values=basis(q,true);if(!values)return std::nullopt;
        const auto& u=(*values)[0];const auto& v=(*values)[1];
        const double uu=inner(u,u),uv=inner(u,v),vv=inner(v,v);
        const double determinant=uu*vv-uv*uv;
        if(uu<=0||vv<=0||determinant<=128*std::numeric_limits<double>::epsilon()*uu*vv)return std::nullopt;
        const double yu=inner(data,u),yv=inner(data,v);
        const double b=(yu*vv-yv*uv)/determinant,d=(yv*uu-yu*uv)/determinant;
        double cost=0;for(unsigned j=0;j<count;++j){const double residual=data[j]-b*u[j]-d*v[j];cost+=residual*residual;}
        if(!std::isfinite(cost))return std::nullopt;
        return std::array<double,3>{cost,b,d};
    };
    const auto consider=[&](const Parameters& q) {
        const auto fit=solve(q);if(!fit)return;
        if((*fit)[0]<bestCost){bestCost=(*fit)[0];bestQ=q;beta=(*fit)[1];contrast=(*fit)[2];}
    };
    consider(bestQ);
    double step=p.pixelPitchRad;cutStep=initialCutStep;
    for(unsigned level=0;level<18;++level) {
        step*=.5;cutStep*=.5;
        for(unsigned sweep=0;sweep<3;++sweep)for(unsigned coordinate=0;coordinate<5;++coordinate) {
            const auto centre=bestQ;
            for(const double sign:{-1.,1.}){auto q=centre;q[coordinate]+=sign*(coordinate==4?cutStep:step);consider(q);}
        }
    }
    const double residual=bestCost/variance;
    double evidence=std::sqrt(std::max(0.,nullCost-bestCost)/variance);
    const double degrees=static_cast<double>(count)-static_cast<double>(nuisance_.size())-7,tail=.5*p.detectionSigma*p.detectionSigma;
    result.fitSnr=evidence;result.fitResidual=residual;
    if(evidence<threshold||degrees<=0||residual>degrees+2*std::sqrt(degrees*tail)+2*tail)return;
    const auto projected=basis(bestQ,true);if(!projected)return;
    // Recover a necessary physical radiance budget from the unprojected ADC
    // waveform. Uniform radiance and the unknown initial pole state are fit
    // jointly; no radiance is copied from the actual scene.
    auto uniform=filtered(flat_,alpha);for(auto& value:uniform)value/=pixelArea;
    std::vector<double> transient(count),dark(count,0);
    double darkState=0;
    const double darkInput=static_cast<double>(p.width)*p.height*p.darkCurrentElectronsS*p.integrationTimeS;
    for(unsigned j=0;j<count;++j) {
        transient[j]=std::pow(1-alpha,j+1);darkState+=alpha*(darkInput-darkState);dark[j]=darkState;
    }
    const double ut=inner(uniform,transient),tt=inner(transient,transient),uu=inner(uniform,uniform),det=uu*tt-ut*ut;
    if(!(uu>0)||!(tt>=0))return;
    const bool hasTransient=tt>0;
    if(hasTransient&&!(det>128*std::numeric_limits<double>::epsilon()*uu*tt))return;
    const auto physicalBudget=[&](const Parameters& q,double b,double d) {
        const auto raw=basis(q,false);if(!raw)return false;
        auto remaining=result.measuredElectrons;
        for(unsigned j=0;j<count;++j)remaining[j]-=b*(*raw)[0][j]+d*(*raw)[1][j]+dark[j];
        const double sky=hasTransient?(inner(remaining,uniform)*tt-inner(remaining,transient)*ut)/det:inner(remaining,uniform)/uu;
        const double areaCells=q[2]*q[3]/pixelArea;
        return std::isfinite(sky)&&sky>=0&&sky+b>=0&&(areaCells>0?sky+d/areaCells>=0:d>=0);
    };
    if(!physicalBudget(bestQ,beta,contrast))return;
    const auto prediction=[&](const Parameters& q)->std::optional<std::vector<double>> {
        const auto values=basis(q,true);if(!values)return std::nullopt;
        auto answer=(*values)[0];for(unsigned j=0;j<count;++j)answer[j]=beta*answer[j]+contrast*(*values)[1][j];return answer;
    };
    std::vector<std::vector<double>> nuisance;
    const auto addColumn=[&](std::vector<double> column) {
        for(const auto& previous:nuisance){const double a=inner(column,previous);for(unsigned j=0;j<count;++j)column[j]-=a*previous[j];}
        const double length=std::sqrt(inner(column,column));
        if(length<=128*std::numeric_limits<double>::epsilon())return false;
        for(auto& value:column)value/=length;
        nuisance.push_back(std::move(column));return true;
    };
    if(!addColumn((*projected)[0])||!addColumn((*projected)[1]))return;
    const double derivativeStep=p.pixelPitchRad/1024,cutDerivative=initialCutStep/1024;
    const auto derivative=[&](unsigned coordinate)->std::optional<std::vector<double>> {
        auto a=bestQ,b=bestQ;const double h=coordinate==4?cutDerivative:derivativeStep;
        a[coordinate]+=h;b[coordinate]-=h;
        if(coordinate==4){a[4]=std::min(high,a[4]);b[4]=std::max(low,b[4]);}
        const auto plus=prediction(a),minus=prediction(b);if(!plus||!minus||!(a[coordinate]>b[coordinate]))return std::nullopt;
        auto answer=*plus;for(unsigned j=0;j<count;++j)answer[j]=(answer[j]-(*minus)[j])/(a[coordinate]-b[coordinate]);return answer;
    };
    for(const unsigned coordinate:{2u,3u,4u}) {
        if(coordinate!=4&&bestQ[coordinate]<=derivativeStep)continue;
        const auto values=derivative(coordinate);if(!values||!addColumn(*values))return;
    }
    auto x=derivative(0),y=derivative(1);if(!x||!y)return;
    for(const auto& column:nuisance) {
        const double a=inner(*x,column),b=inner(*y,column);
        for(unsigned j=0;j<count;++j){(*x)[j]-=a*column[j];(*y)[j]-=b*column[j];}
    }
    const double xx=inner(*x,*x),yy=inner(*y,*y),xy=inner(*x,*y),determinant=xx*yy-xy*xy,trace=xx+yy;
    if(!std::isfinite(determinant)||xx<=0||yy<=0||determinant<=128*std::numeric_limits<double>::epsilon()*trace*trace)return;
    const double angularVariance=variance*(trace+std::hypot(xx-yy,2*xy))/(2*determinant)+step*step/12;
    if(!std::isfinite(angularVariance)||angularVariance<=0)return;
    const auto ray=[&](const Parameters& q){return unit(add(forward,add(scale(right,std::tan(q[0])),scale(vertical,std::tan(q[1])))));};
    const auto direction=ray(bestQ);
    // A local Fisher matrix cannot exclude remote reflected/sign aliases.
    // Explicitly test the reflected compact supports against EVERY visited
    // cutoff. An equally plausible separated bearing is unresolved, even
    // when the covariance around the selected local mode is very small.
    for(const auto& ground:calibrations)for(const double sx:{-1.,1.})for(const double sy:{-1.,1.}) {
        if(sx==1&&sy==1)continue;
        auto q=bestQ;q[0]*=sx;q[1]*=sy;q[4]=ground.cutoff;
        const auto other=ray(q);
        const double separation=std::atan2(norm(cross(direction,other)),dot(direction,other));
        if(separation<=p.detectionSigma*std::sqrt(angularVariance))continue;
        const auto alternative=solve(q);
        if(alternative&&(*alternative)[0]<=bestCost+threshold*threshold*variance&&physicalBudget(q,(*alternative)[1],(*alternative)[2]))return;
    }
    evidence=std::sqrt(std::max(0.,nullCost-bestCost)/variance);
    result.fitSnr=evidence;
    if(evidence<threshold)return;
    ThermalImageFeatureV1 feature;
    feature.direction=direction;
    feature.signalToNoise=evidence;feature.angularVarianceRad2=angularVariance;feature.signedContrastElectrons=contrast;
    result.features.push_back(feature);result.bearingResolved=true;
}

}
