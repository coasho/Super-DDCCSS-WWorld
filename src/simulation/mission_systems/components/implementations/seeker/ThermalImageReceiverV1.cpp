#include "ThermalImageReceiverV1.h"
#include "EngineeringSeekerV3.h"
#include "ThermalDualBandReceiverV1.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>
#include <cfenv>
#include <cfloat>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <bitset>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif

namespace fsim::ms::components {
using namespace measured;
namespace {
constexpr double pi=3.14159265358979323846;
constexpr double planck=6.62607015e-34,light=299792458.;
bool positive(double v) {return std::isfinite(v)&&v>0;}
bool nonnegative(double v) {return std::isfinite(v)&&v>=0;}
struct Plane {
    V forward,right,up;
    V ray(double x,double y) const {return unit(add(forward,add(scale(right,std::tan(x)),scale(up,std::tan(y)))));}
};
Plane planeFor(V direction,V up) {
    Plane p;p.forward=unit(direction);p.right=unit(cross(p.forward,up));
    if(norm(p.right)<.5)p.right=unit(cross(p.forward,std::abs(p.forward.z)<.9?V{0,0,1}:V{0,1,0}));
    p.up=unit(cross(p.right,p.forward));return p;
}
double solidAngle(double x0,double x1,double y0,double y1) {
    const auto primitive=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    x0=std::tan(x0);x1=std::tan(x1);y0=std::tan(y0);y1=std::tan(y1);
    return primitive(x1,y1)-primitive(x0,y1)-primitive(x1,y0)+primitive(x0,y0);
}
// Convolve the continuous source rectangle with the normalized optical
// Gaussian, THEN integrate the detector interval. No source position is
// quantized to a detector centre, and FOV losses are never renormalized away.
struct Rectangle {double left,right,bottom,top;};
// One visible-region lifetime, never a receiver/global cache. Every stored
// value is the original native arithmetic at an exact binary64 x key.
class HorizonQuadratureMemoV2 {
public:
    struct Pattern {
        std::array<double,4> edges{};std::array<double,3> lights{};
        unsigned size=0;bool rootsReady=false;
    };
    struct Node:Pattern {
        std::array<double,128> yValues{},probabilities{};
        std::bitset<128> yKnown,probabilityKnown;
    };
private:
    Rectangle region_;ThermalOpaqueMemoWorkV2* work_;
    unsigned width_,height_;std::size_t capacity_;
    bool enabled_=false,initialized_=false;
    std::unordered_map<std::uint64_t,Node> nodes_;
    static std::uint64_t key(double x) noexcept {
        static_assert(sizeof(x)==sizeof(std::uint64_t),"binary64 key required");
        std::uint64_t value;std::memcpy(&value,&x,sizeof(value));return value;
    }
public:
    HorizonQuadratureMemoV2(unsigned width,unsigned height,Rectangle region,ThermalOpaqueMemoWorkV2* work):
        region_(region),work_(work),width_(width),height_(height),capacity_(std::max(24u,width+height)) {
        enabled_=std::numeric_limits<double>::is_iec559&&FLT_EVAL_METHOD==0&&std::fegetround()==FE_TONEAREST;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        enabled_=enabled_&&(_mm_getcsr()&0x6000u)==0;
#endif
    }
    ThermalOpaqueMemoWorkV2* diagnostic() const noexcept {return work_;}
    Node* find(double x,unsigned column,unsigned row) {
        if(work_)++work_->queries;
        if(!enabled_||column>=width_||row>=height_)return nullptr;
        if(!initialized_) {
            initialized_=true;nodes_.reserve(capacity_);
            // Reserve the exact original 8-segment initial nodes before any
            // single pixel's adaptive descent can fill the performance cache.
            for(unsigned i=0;i<8;++i) {
                const double low=region_.left+(region_.right-region_.left)*i/8;
                const double high=region_.left+(region_.right-region_.left)*(i+1)/8,middle=.5*(low+high);
                for(const double value:{low,middle,high})nodes_.try_emplace(key(value));
            }
        }
        const auto value=key(x);const auto found=nodes_.find(value);
        if(found!=nodes_.end()){if(work_){++work_->hits;work_->maxNodes=std::max(work_->maxNodes,nodes_.size());}return &found->second;}
        // Capacity only limits memoization. An uncached node executes the
        // complete old calculation with the same integral/accuracy budget.
        if(nodes_.size()>=capacity_){if(work_)++work_->capacityMisses;return nullptr;}
        auto* node=&nodes_.try_emplace(value).first->second;
        if(work_)work_->maxNodes=std::max(work_->maxNodes,nodes_.size());
        return node;
    }
};
double gaussianRectangle(double low,double high,double pixelLow,double pixelHigh,double sigma) {
    const auto primitive=[](double z) {
        return z*.5*std::erfc(-z/std::sqrt(2.))+std::exp(-.5*z*z)/std::sqrt(2*pi);
    };
    const auto integratedCdf=[&](double edge) {
        return sigma*(primitive((edge-low)/sigma)-primitive((edge-high)/sigma));
    };
    return std::clamp(integratedCdf(pixelHigh)-integratedCdf(pixelLow),0.,std::min(high-low,pixelHigh-pixelLow));
}
// Integrate the sky/ground step in continuous source space as well. At fixed
// x the exact native conical horizon has at most two roots in tan(y). The y
// integral is analytic; adaptive Simpson integrates its smooth x dependence.
// Uniform-sky/ground rectangles use an exact interval bound and no quadrature.
double backgroundIntegral(const ThermalSceneFrameV1& scene,const Plane& plane,Rectangle r,
    double px0,double px1,double py0,double py1,double sigma,
    HorizonQuadratureMemoV2* memo=nullptr,unsigned memoColumn=0,unsigned memoRow=0) {
    const double ix=gaussianRectangle(r.left,r.right,px0,px1,sigma);
    const double iy=gaussianRectangle(r.bottom,r.top,py0,py1,sigma);
    if(ix==0||iy==0)return 0;
    if(scene.skyRadianceWm2Sr==scene.groundRadianceWm2Sr)return scene.skyRadianceWm2Sr*ix*iy;
    const double a=dot(plane.forward,scene.up),b=dot(plane.right,scene.up),c=dot(plane.up,scene.up);
    const double tx0=std::tan(r.left),tx1=std::tan(r.right),ty0=std::tan(r.bottom),ty1=std::tan(r.top);
    const double n0=a+std::min(b*tx0,b*tx1)+std::min(c*ty0,c*ty1);
    const double n1=a+std::max(b*tx0,b*tx1)+std::max(c*ty0,c*ty1);
    const auto minSquare=[](double low,double high){return low<=0&&high>=0?0.:std::min(low*low,high*high);};
    const double d0=std::sqrt(1+minSquare(tx0,tx1)+minSquare(ty0,ty1));
    const double d1=std::sqrt(1+std::max(tx0*tx0,tx1*tx1)+std::max(ty0*ty0,ty1*ty1));
    const double minimum=std::min({n0/d0,n0/d1,n1/d0,n1/d1}),maximum=std::max({n0/d0,n0/d1,n1/d0,n1/d1});
    if(minimum>=scene.groundHorizonCosine)return scene.skyRadianceWm2Sr*ix*iy;
    if(maximum<scene.groundHorizonCosine)return scene.groundRadianceWm2Sr*ix*iy;
    bool nearest=std::fegetround()==FE_TONEAREST;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    nearest=nearest&&(_mm_getcsr()&0x6000u)==0;
#endif
    const bool boundedUnit=nearest&&scene.skyRadianceWm2Sr>=0&&scene.skyRadianceWm2Sr<=1&&
        scene.groundRadianceWm2Sr>=0&&scene.groundRadianceWm2Sr<=1;
    const auto function=[&](double x) {
        auto* cached=memo?memo->find(x,memoColumn,memoRow):nullptr;
        auto* counted=memo?memo->diagnostic():nullptr;
        const auto probabilityAt=[&]() {
            if(cached&&cached->probabilityKnown[memoColumn]){if(counted)++counted->probabilityHits;return cached->probabilities[memoColumn];}
            if(counted)++counted->probabilityEvaluations;
            const double value=.5*(std::erf((px1-x)/(std::sqrt(2.)*sigma))-std::erf((px0-x)/(std::sqrt(2.)*sigma)));
            if(cached){cached->probabilityKnown.set(memoColumn);cached->probabilities[memoColumn]=value;}
            return value;
        };
        const double knownProbability=boundedUnit?probabilityAt():0.;
        if(boundedUnit&&knownProbability==0)return knownProbability;
        double yIntegral=0;
        if(cached&&cached->yKnown[memoRow]){if(counted)++counted->yHits;yIntegral=cached->yValues[memoRow];}
        else {
            if(counted)++counted->yEvaluations;
            // The vector/root/sort and midpoint classifications are retained
            // exactly, including duplicate roots and zero-length intervals.
            // They depend on this x/scene/region, never the detector pixel.
            HorizonQuadratureMemoV2::Pattern local;
            auto* pattern=cached?static_cast<HorizonQuadratureMemoV2::Pattern*>(cached):&local;
            if(!pattern->rootsReady) {
                if(counted)++counted->rootsBuilt;
                const double tx=std::tan(x),n=a+b*tx,h=scene.groundHorizonCosine;
                std::vector<double> edges{r.bottom,r.top};
                const auto root=[&](double t){const double y=std::atan(t);if(y>r.bottom&&y<r.top)edges.push_back(y);};
                const double aa=c*c-h*h,bb=2*n*c,cc=n*n-h*h*(1+tx*tx);
                if(std::abs(aa)<1e-14) {if(std::abs(bb)>1e-14)root(-cc/bb);}
                else {
                    const double discriminant=bb*bb-4*aa*cc;
                    if(discriminant>=0) {root((-bb-std::sqrt(discriminant))/(2*aa));root((-bb+std::sqrt(discriminant))/(2*aa));}
                }
                std::sort(edges.begin(),edges.end());pattern->size=static_cast<unsigned>(edges.size());
                std::copy(edges.begin(),edges.end(),pattern->edges.begin());
                for(unsigned i=1;i<pattern->size;++i) {
                    const double y=.5*(pattern->edges[i-1]+pattern->edges[i]);
                    pattern->lights[i-1]=dot(plane.ray(x,y),scene.up)>=h?scene.skyRadianceWm2Sr:scene.groundRadianceWm2Sr;
                }
                pattern->rootsReady=true;
            }
            // Gaussian y intervals depend on the row, not the column. Keep
            // every original multiplication/addition in its original order.
            for(unsigned i=1;i<pattern->size;++i)
                yIntegral+=pattern->lights[i-1]*gaussianRectangle(pattern->edges[i-1],pattern->edges[i],py0,py1,sigma);
            if(cached){cached->yKnown.set(memoRow);cached->yValues[memoRow]=yIntegral;}
        }
        const double probability=boundedUnit?knownProbability:probabilityAt();
        return probability*yIntegral;
    };
    const auto adaptive=[&](auto&& self,double low,double high,double fl,double fm,double fh,double estimate,double tolerance,unsigned depth)->double {
        const double middle=.5*(low+high),lmid=.5*(low+middle),rmid=.5*(middle+high);
        const double fleft=function(lmid),fright=function(rmid);
        const double left=(middle-low)*(fl+4*fleft+fm)/6,right=(high-middle)*(fm+4*fright+fh)/6;
        if(depth==0||std::abs(left+right-estimate)<=15*tolerance)return left+right+(left+right-estimate)/15;
        return self(self,low,middle,fl,fleft,fm,left,tolerance*.5,depth-1)+
            self(self,middle,high,fm,fright,fh,right,tolerance*.5,depth-1);
    };
    double result=0;
    const double tolerance=1e-9*std::max(scene.skyRadianceWm2Sr,scene.groundRadianceWm2Sr)*(px1-px0)*(py1-py0);
    for(unsigned i=0;i<8;++i) {
        const double low=r.left+(r.right-r.left)*i/8,high=r.left+(r.right-r.left)*(i+1)/8,middle=.5*(low+high);
        const double fl=function(low),fm=function(middle),fh=function(high),estimate=(high-low)*(fl+4*fm+fh)/6;
        result+=adaptive(adaptive,low,high,fl,fm,fh,estimate,tolerance/8,10);
    }
    return std::clamp(result,0.,std::max(scene.skyRadianceWm2Sr,scene.groundRadianceWm2Sr)*ix*iy);
}
// Multi-band association must compare the same measured spatial support.
// Threshold-connected islands can contain two resolved maxima with very
// different colours, so their whole-island centroids are not cross-band
// counterparts. Deblend local detector extrema, then use a common angular
// aperture for photometry; never use source rectangles or their identities.
std::optional<ThermalImageFeatureV1> apertureFeature(const ThermalImageResultV1& image,const Plane& plane,
    double centreX,double centreY,double radius,double polarity) {
    const auto& c=image.config;
    double total=0,noise=0,xSum=0,ySum=0;unsigned lowX=c.width,highX=0,lowY=c.height,highY=0,supportPixels=0;
    for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
        const double x0=(x-.5*c.width)*c.pixelPitchRad,y0=(y-.5*c.height)*c.pixelPitchRad;
        if(x0>centreX+radius||x0+c.pixelPitchRad<centreX-radius||y0>centreY+radius||y0+c.pixelPitchRad<centreY-radius)continue;
        unsigned inside=0;
        for(unsigned sy=0;sy<4;++sy)for(unsigned sx=0;sx<4;++sx) {
            const double dx=x0+(sx+.5)*c.pixelPitchRad/4-centreX,dy=y0+(sy+.5)*c.pixelPitchRad/4-centreY;
            if(dx*dx+dy*dy<=radius*radius)++inside;
        }
        if(!inside)continue;
        const double fraction=inside/16.;const auto index=static_cast<std::size_t>(y)*c.width+x;
        const double charge=fraction*polarity*image.contrastElectrons[index];
        total+=charge;xSum+=charge*(x0+.5*c.pixelPitchRad);ySum+=charge*(y0+.5*c.pixelPitchRad);
        noise+=fraction*fraction*image.noiseSigmaElectrons[index]*image.noiseSigmaElectrons[index];
        if(polarity*image.contrastElectrons[index]>=.5*image.detectionThresholdElectrons[index]) {
            ++supportPixels;
            lowX=std::min(lowX,x);highX=std::max(highX,x);lowY=std::min(lowY,y);highY=std::max(highY,y);
        }
    }
    if(!positive(total)||!positive(noise)||total/std::sqrt(noise)<c.detectionSigma||
        static_cast<double>(supportPixels)<c.minimumFeaturePixels)return std::nullopt;
    ThermalImageFeatureV1 feature;feature.direction=plane.ray(xSum/total,ySum/total);
    feature.centroidColumn=xSum/total/c.pixelPitchRad+.5*c.width-.5;
    feature.centroidRow=ySum/total/c.pixelPitchRad+.5*c.height-.5;
    feature.signedContrastElectrons=polarity*total;feature.signalToNoise=total/std::sqrt(noise);
    feature.extent={(highX-lowX+1)*c.pixelPitchRad,(highY-lowY+1)*c.pixelPitchRad};
    feature.angularVarianceRad2=std::max(c.pixelPitchRad*c.pixelPitchRad/12,c.psfSigmaRad*c.psfSigmaRad/(feature.signalToNoise*feature.signalToNoise));
    return feature;
}
std::vector<ThermalImageFeatureV1> localApertureFeatures(const ThermalImageResultV1& image,const Plane& plane,double radius) {
    const auto& c=image.config;struct Peak {unsigned x,y;double significance,polarity;};std::vector<Peak> peaks;
    const int neighbors=std::max(1,static_cast<int>(std::ceil(c.psfSigmaRad/c.pixelPitchRad)));
    for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
        const auto index=static_cast<std::size_t>(y)*c.width+x;
        if(std::abs(image.contrastElectrons[index])<image.detectionThresholdElectrons[index])continue;
        const double polarity=image.contrastElectrons[index]>=0?1.:-1.;bool maximum=true;
        for(int dy=-neighbors;dy<=neighbors&&maximum;++dy)for(int dx=-neighbors;dx<=neighbors;++dx) {
            const int px=static_cast<int>(x)+dx,py=static_cast<int>(y)+dy;
            if(px<0||py<0||px>=static_cast<int>(c.width)||py>=static_cast<int>(c.height))continue;
            const auto other=static_cast<std::size_t>(py)*c.width+static_cast<unsigned>(px);
            if(polarity*image.contrastElectrons[other]>polarity*image.contrastElectrons[index]||
                (image.contrastElectrons[other]==image.contrastElectrons[index]&&other<index)){maximum=false;break;}
        }
        if(maximum)peaks.push_back({x,y,std::abs(image.contrastElectrons[index])/image.noiseSigmaElectrons[index],polarity});
    }
    std::sort(peaks.begin(),peaks.end(),[](const auto& a,const auto& b){return std::tuple{-a.significance,a.x,a.y}<std::tuple{-b.significance,b.x,b.y};});
    std::vector<ThermalImageFeatureV1> result;std::vector<Peak> retained;
    for(const auto peak:peaks) {
        if(std::any_of(retained.begin(),retained.end(),[&](const auto& previous){
            return previous.polarity==peak.polarity&&std::hypot(static_cast<double>(peak.x)-previous.x,static_cast<double>(peak.y)-previous.y)*c.pixelPitchRad<radius;
        }))continue;
        const double x=(peak.x+.5-.5*c.width)*c.pixelPitchRad,y=(peak.y+.5-.5*c.height)*c.pixelPitchRad;
        auto feature=apertureFeature(image,plane,x,y,radius,peak.polarity);
        if(feature){result.push_back(*feature);retained.push_back(peak);if(result.size()==32)break;}
    }
    return result;
}
double inverseErf(double value) {
    double low=-6,high=6;value=std::clamp(value,-1.+1e-12,1.-1e-12);
    for(unsigned i=0;i<48;++i) {const auto middle=.5*(low+high);if(std::erf(middle)<value)low=middle;else high=middle;}
    return .5*(low+high);
}
double median(std::vector<double>& values) {
    const auto middle=values.begin()+static_cast<std::ptrdiff_t>(values.size()/2);
    std::nth_element(values.begin(),middle,values.end());
    return values.size()%2?*middle:.5*(*middle+*std::max_element(values.begin(),middle));
}
}

bool ThermalImageReceiverV1::validConfig(const ThermalImageConfigV1& c) noexcept {
    if(c.version!=1||c.width<8||c.width>128||c.height<8||c.height>128||
       c.adcBits<4||c.adcBits>24||c.exposureSamples<1||c.exposureSamples>16)return false;
    for(double v:{c.pixelPitchRad,c.psfSigmaRad,c.apertureAreaM2,c.opticalTransmission,c.quantumEfficiency,
        c.effectiveWavelengthM,c.integrationTimeS,c.pixelResponseTimeS,c.readNoiseElectrons,
        c.darkCurrentElectronsS,c.fullWellElectrons,c.detectionSigma,c.backgroundRadiusPixels,
        c.minimumFeaturePixels,c.associationGateRad})if(!positive(v))return false;
    return c.pixelPitchRad>=1e-7&&c.pixelPitchRad*c.width<1.4&&c.pixelPitchRad*c.height<1.4&&
        c.psfSigmaRad/c.pixelPitchRad>=.125&&c.psfSigmaRad/c.pixelPitchRad<=8&&
        (.5*std::max(c.width,c.height)+std::ceil(4*c.psfSigmaRad/c.pixelPitchRad))*c.pixelPitchRad<1.4&&
        c.apertureAreaM2<=1&&c.opticalTransmission<=1&&c.quantumEfficiency<=1&&
        c.effectiveWavelengthM>=1e-7&&c.effectiveWavelengthM<=1e-3&&
        c.integrationTimeS<=1&&c.pixelResponseTimeS<=5&&c.fullWellElectrons>=100&&c.fullWellElectrons<=1e12&&
        c.readNoiseElectrons<=1e6&&c.darkCurrentElectronsS<=1e12&&c.detectionSigma<=30&&
        c.backgroundRadiusPixels>=2&&c.backgroundRadiusPixels<=32&&
        c.backgroundRadiusPixels==std::floor(c.backgroundRadiusPixels)&&
        c.minimumFeaturePixels<=c.width*c.height&&c.minimumFeaturePixels==std::floor(c.minimumFeaturePixels)&&
        c.associationGateRad<1.4;
}
bool ThermalImageReceiverV1::configure(const ThermalImageConfigV1& c,std::string& error) {
    if(!validConfig(c)){error="invalid explicit thermal image receiver policy";return false;}
    config_=c;reset();return true;
}
bool ThermalImageReceiverV1::validScene(const ThermalSceneFrameV1& scene,const ThermalImageConfigV1& c) noexcept {
    if(!nonnegative(scene.timeS)||!finite(scene.up)||std::abs(norm(scene.up)-1)>1e-6||
       !std::isfinite(scene.groundHorizonCosine)||std::abs(scene.groundHorizonCosine)>1||
       !nonnegative(scene.skyRadianceWm2Sr)||!nonnegative(scene.groundRadianceWm2Sr)||
       scene.skyRadianceWm2Sr>1e12||scene.groundRadianceWm2Sr>1e12||
       scene.patches.size()>128||scene.refereeAttributionIndices.size()!=scene.patches.size())return false;
    for(const auto& patch:scene.patches)if(!finite(patch.direction)||std::abs(norm(patch.direction)-1)>1e-6||
        !finite(patch.directionRate)||!std::isfinite(norm(patch.directionRate))||norm(patch.directionRate)*c.integrationTimeS>.5||
        !positive(patch.widthRad)||!positive(patch.heightRad)||patch.widthRad>=pi||patch.heightRad>=pi||!positive(patch.rangeM)||
        !nonnegative(patch.radianceWm2Sr)||patch.radianceWm2Sr>1e12||
        !nonnegative(patch.pathRadianceWm2Sr)||patch.pathRadianceWm2Sr>1e12||
        !nonnegative(patch.transmission)||patch.transmission>1)return false;
    return true;
}
void ThermalImageReceiverV1::reset() noexcept {
    pixelResponse_.clear();attributionResponse_.clear();unattributedResponse_.clear();lastTimeS_=-1;seeded_=false;cached_={};
}
double ThermalImageReceiverV1::normal() {
    const auto uniform=[&]{return (static_cast<double>(rng_()>>11)+.5)/9007199254740992.;};
    return std::sqrt(-2*std::log(uniform()))*std::cos(2*pi*uniform());
}
double ThermalImageReceiverV1::bandRadiance(double temperature,double emissivity,double low,double high) noexcept {
    if(!nonnegative(temperature)||!nonnegative(emissivity)||emissivity>1||!positive(low)||!positive(high)||high<=low)return 0;
    const double result=emissivity*5.670374419e-8*std::pow(temperature,4)*
        EngineeringSeekerV3::bandFraction(temperature,low,high)/pi;
    return nonnegative(result)?result:0;
}
double ThermalImageReceiverV1::gaussianIntervalV1(double low,double high,double detectorLow,double detectorHigh,double sigma) {
    return gaussianRectangle(low,high,detectorLow,detectorHigh,sigma);
}
std::optional<ThermalIncidentFieldV1> ThermalImageReceiverV1::incidentFieldV1(
    const ThermalSceneFrameV1& scene,V pointing,const ThermalImageConfigV1& c) {
    return incidentFieldImplV1(scene,pointing,c,nullptr);
}
std::optional<ThermalIncidentFieldV1> ThermalImageReceiverV1::incidentFieldImplV1(
    const ThermalSceneFrameV1& scene,V pointing,const ThermalImageConfigV1& c,const std::vector<double>* background) {
    return incidentFieldProfileV2(scene,pointing,c,background,nullptr);
}
std::optional<ThermalIncidentFieldV1> ThermalImageReceiverV1::incidentFieldProfileV2(
    const ThermalSceneFrameV1& scene,V pointing,const ThermalImageConfigV1& c,const std::vector<double>* background,ThermalOpaqueMemoWorkV2* work) {
    if(!validConfig(c)||!validScene(scene,c)||!finite(pointing)||!positive(norm(pointing)))return std::nullopt;
    const auto plane=planeFor(pointing,unit(scene.up));
    const double halfWidth=.5*c.width,halfHeight=.5*c.height;
    const auto pixels=static_cast<std::size_t>(c.width)*c.height;
    std::vector<double> radiance(pixels,0);
    std::vector<std::vector<double>> sourceContrast(scene.patches.size(),std::vector<double>(pixels,0));
    std::vector<std::size_t> order(scene.patches.size());std::iota(order.begin(),order.end(),0);
    // Near-to-far exact opaque geometry; disjoint subpixel supports cannot
    // attenuate one another. The canonical tie policy is identity-independent.
    std::sort(order.begin(),order.end(),[&](auto a,auto b){
        const auto key=[](const auto& p){return std::tuple{p.rangeM,p.direction.x,p.direction.y,p.direction.z,
            p.directionRate.x,p.directionRate.y,p.directionRate.z,
            p.widthRad,p.heightRad,p.radianceWm2Sr,p.transmission,p.pathRadianceWm2Sr};};
        return key(scene.patches[a])<key(scene.patches[b]);
    });
    // Gaussian tail beyond nine sigma has < 3e-19 probability per axis.
    // No finite-detector energy is normalized back into the field of view.
    const double margin=9*c.psfSigmaRad,pitch=c.pixelPitchRad;
    if(background) {
        if(background->size()!=pixels)return std::nullopt;
        radiance=*background;
    } else for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
        const double x0=(x-halfWidth)*pitch,y0=(y-halfHeight)*pitch;
        const Rectangle support{std::max(-1.55,x0-margin),std::min(1.55,x0+pitch+margin),
            std::max(-1.55,y0-margin),std::min(1.55,y0+pitch+margin)};
        radiance[static_cast<std::size_t>(y)*c.width+x]=
            backgroundIntegral(scene,plane,support,x0,x0+pitch,y0,y0+pitch,c.psfSigmaRad)/(pitch*pitch);
    }
    auto backgroundRadiance=radiance;
    for(unsigned sample=0;sample<c.exposureSamples;++sample) {
        const double before=c.integrationTimeS*(1-(sample+.5)/c.exposureSamples);
        std::vector<std::optional<Rectangle>> projected(scene.patches.size());
        std::vector<double> yEdges;
        for(const auto index:order) {
            const auto& patch=scene.patches[index];
            const auto direction=unit(subtract(patch.direction,scale(patch.directionRate,before)));
            const double forward=dot(direction,plane.forward);if(forward<=0)continue;
            const double ax=std::atan2(dot(direction,plane.right),forward),ay=std::atan2(dot(direction,plane.up),forward);
            const Rectangle rectangle{std::max({-1.55,-halfWidth*pitch-margin,ax-.5*patch.widthRad}),
                std::min({1.55,halfWidth*pitch+margin,ax+.5*patch.widthRad}),
                std::max({-1.55,-halfHeight*pitch-margin,ay-.5*patch.heightRad}),
                std::min({1.55,halfHeight*pitch+margin,ay+.5*patch.heightRad})};
            if(rectangle.left>=rectangle.right||rectangle.bottom>=rectangle.top)continue;
            projected[index]=rectangle;yEdges.push_back(rectangle.bottom);yEdges.push_back(rectangle.top);
        }
        std::sort(yEdges.begin(),yEdges.end());yEdges.erase(std::unique(yEdges.begin(),yEdges.end()),yEdges.end());
        struct Visible {std::size_t source;Rectangle region;};std::vector<Visible> visible;
        std::map<std::tuple<std::size_t,double,double>,std::size_t> verticalRuns;
        // An edge arrangement has at most (2N-1)^2 cells. Sweep the finite
        // arrangement once; do not recursively split fragments per occluder.
        // Coalesce same-owner horizontal and vertical runs before optics.
        for(std::size_t row=1;row<yEdges.size();++row) {
            const double bottom=yEdges[row-1],top=yEdges[row],middle=.5*(bottom+top);
            std::vector<std::size_t> active;std::vector<double> xEdges;
            for(const auto index:order)if(projected[index]&&projected[index]->bottom<middle&&projected[index]->top>middle) {
                active.push_back(index);xEdges.push_back(projected[index]->left);xEdges.push_back(projected[index]->right);
            }
            std::sort(xEdges.begin(),xEdges.end());xEdges.erase(std::unique(xEdges.begin(),xEdges.end()),xEdges.end());
            std::vector<Visible> horizontal;
            for(std::size_t column=1;column<xEdges.size();++column) {
                const double left=xEdges[column-1],right=xEdges[column],centre=.5*(left+right);
                std::optional<std::size_t> owner;
                for(const auto index:active)if(projected[index]->left<centre&&projected[index]->right>centre){owner=index;break;}
                if(!owner)continue;
                if(!horizontal.empty()&&horizontal.back().source==*owner&&horizontal.back().region.right==left)
                    horizontal.back().region.right=right;
                else horizontal.push_back({*owner,{left,right,bottom,top}});
            }
            for(const auto& run:horizontal) {
                const auto key=std::tuple{run.source,run.region.left,run.region.right};const auto previous=verticalRuns.find(key);
                if(previous!=verticalRuns.end()&&visible[previous->second].region.top==bottom)visible[previous->second].region.top=top;
                else {verticalRuns[key]=visible.size();visible.push_back(run);}
            }
        }
        for(const auto& run:visible) {
            const auto& part=run.region;const auto index=run.source;const auto& patch=scene.patches[index];
            HorizonQuadratureMemoV2 integralMemo(c.width,c.height,part,work);
            const double lightAtSensor=patch.transmission*patch.radianceWm2Sr+(1-patch.transmission)*patch.pathRadianceWm2Sr;
            const int firstX=std::max(0,static_cast<int>(std::floor((part.left-margin)/pitch+halfWidth)));
            const int lastX=std::min(static_cast<int>(c.width)-1,static_cast<int>(std::floor((part.right+margin)/pitch+halfWidth)));
            const int firstY=std::max(0,static_cast<int>(std::floor((part.bottom-margin)/pitch+halfHeight)));
            const int lastY=std::min(static_cast<int>(c.height)-1,static_cast<int>(std::floor((part.top+margin)/pitch+halfHeight)));
            std::vector<double> wx(c.width),wy(c.height);
            for(int x=firstX;x<=lastX;++x)wx[static_cast<unsigned>(x)]=gaussianRectangle(part.left,part.right,(x-halfWidth)*pitch,(x+1-halfWidth)*pitch,c.psfSigmaRad);
            for(int y=firstY;y<=lastY;++y)wy[static_cast<unsigned>(y)]=gaussianRectangle(part.bottom,part.top,(y-halfHeight)*pitch,(y+1-halfHeight)*pitch,c.psfSigmaRad);
            for(int y=firstY;y<=lastY;++y)for(int x=firstX;x<=lastX;++x) {
                const double x0=(x-halfWidth)*pitch,y0=(y-halfHeight)*pitch;
                const double contrast=(lightAtSensor*wx[static_cast<unsigned>(x)]*wy[static_cast<unsigned>(y)]-
                    backgroundIntegral(scene,plane,part,x0,x0+pitch,y0,y0+pitch,c.psfSigmaRad,&integralMemo,static_cast<unsigned>(x),static_cast<unsigned>(y)))/(pitch*pitch*c.exposureSamples);
                sourceContrast[index][static_cast<std::size_t>(y)*c.width+static_cast<unsigned>(x)]+=contrast;
            }
        }
    }
    // Sum in canonical geometric order so reordering a scene does not alter
    // floating-point summation, photon draws or measured image features.
    for(const auto index:order)for(std::size_t i=0;i<pixels;++i)radiance[i]+=sourceContrast[index][i];
    return ThermalIncidentFieldV1{std::move(radiance),std::move(sourceContrast),std::move(order),std::move(backgroundRadiance)};
}
ThermalImageResultV1 ThermalImageReceiverV1::expose(const ThermalSceneFrameV1& scene,V pointing,
    std::uint64_t seed,bool imaging) {
    const auto& c=config_;
    if(!validConfig(c)||!validScene(scene,c)||scene.timeS<lastTimeS_||!finite(pointing)||!positive(norm(pointing)))return {};
    if(scene.timeS==lastTimeS_)return cached_;
    const auto plane=planeFor(pointing,unit(scene.up));
    const double halfWidth=.5*c.width,halfHeight=.5*c.height,pitch=c.pixelPitchRad;
    const auto pixels=static_cast<std::size_t>(c.width)*c.height;
    auto field=incidentFieldV1(scene,pointing,c);if(!field)return {};
    auto& radiance=field->radiance;auto& sourceContrast=field->sourceContrast;
    const auto& order=field->sourceOrder;
    ThermalImageResultV1 result;result.valid=true;result.timeS=scene.timeS;result.width=c.width;result.height=c.height;
    // Keep the exact optical input. Re-normalizing a diagnostic direction and
    // then normalizing it again in planeFor changes subpixel irradiance bits.
    // Optics already normalizes its own basis; replay needs the original input.
    result.pointing=pointing;result.config=c;result.exposureStartTimeS=scene.timeS-c.integrationTimeS;
    result.expectedElectrons.resize(pixels);result.measuredElectrons.resize(pixels);
    std::vector<double> pixelWeights;
    if(!imaging)pixelWeights.resize(pixels);
    if(!seeded_){rng_.seed(seed^0x544845524d414c31ull);seeded_=true;}
    if(pixelResponse_.size()!=pixels)pixelResponse_.assign(pixels,0);
    const double elapsed=lastTimeS_<0?c.integrationTimeS:scene.timeS-lastTimeS_;
    const double response=-std::expm1(-elapsed/c.pixelResponseTimeS);
    const double electronsPerJoule=c.quantumEfficiency*c.effectiveWavelengthM/(planck*light);
    const double adcStep=c.fullWellElectrons/(std::ldexp(1.,static_cast<int>(c.adcBits))-1);
    for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
        const double value=std::max(0.,radiance[static_cast<std::size_t>(y)*c.width+x]);
        const double x0=(x-halfWidth)*c.pixelPitchRad,y0=(y-halfHeight)*c.pixelPitchRad;
        const double omega=solidAngle(x0,x0+c.pixelPitchRad,y0,y0+c.pixelPitchRad);
        const auto index=static_cast<std::size_t>(y)*c.width+x;
        if(!imaging)pixelWeights[index]=omega/(pitch*pitch);
        const double expected=value*omega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*electronsPerJoule;
        if(!nonnegative(expected)){reset();return {};}
        pixelResponse_[index]+=response*(expected-pixelResponse_[index]);
        const double gain=omega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*electronsPerJoule;
        for(auto& contribution:sourceContrast)contribution[index]*=gain;
        const double total=pixelResponse_[index]+c.darkCurrentElectronsS*c.integrationTimeS;
        result.expectedElectrons[index]=total;
        // At >=20 shot-noise standard deviations above full well, saturation
        // is certain to far below binary64 tail precision; avoid huge draws.
        double measured=c.fullWellElectrons;
        if(total<c.fullWellElectrons+20*(std::sqrt(c.fullWellElectrons)+c.readNoiseElectrons)) {
            std::poisson_distribution<std::uint64_t> photons(total);
            measured=std::min(c.fullWellElectrons,static_cast<double>(photons(rng_)));
        }
        measured+=normal()*c.readNoiseElectrons;
        result.measuredElectrons[index]=std::round(std::clamp(measured,0.,c.fullWellElectrons)/adcStep)*adcStep;
    }
    std::vector<double> contrast(pixels),variance(pixels),significance(pixels);
    const auto distance=static_cast<int>(c.backgroundRadiusPixels);
    std::vector<std::pair<int,int>> annulus;
    for(unsigned i=0;i<32;++i) {
        const double phase=2*pi*i/32;
        const auto offset=std::pair<int,int>{static_cast<int>(std::lround(distance*std::cos(phase))),
            static_cast<int>(std::lround(distance*std::sin(phase)))};
        if(std::find(annulus.begin(),annulus.end(),offset)==annulus.end())annulus.push_back(offset);
    }
    std::vector<double> neighbors;neighbors.reserve(annulus.size());
    std::vector<std::size_t> neighborIndices;neighborIndices.reserve(annulus.size());
    // Scene radiance is never subtracted as detector knowledge. A dense unique
    // circular annulus avoids the four-of-eight square-stencil optical-tail
    // contamination that otherwise generates artificial cold companion lobes.
    result.imageProcessingAvailable=imaging;
    if(imaging){result.backgroundElectrons.resize(pixels);result.contrastElectrons.resize(pixels);
        result.noiseSigmaElectrons.resize(pixels);result.detectionThresholdElectrons.resize(pixels);}
    for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
        neighbors.clear();neighborIndices.clear();
        for(const auto& [dx,dy]:annulus) {
            const auto px=std::clamp(static_cast<int>(x)+dx,0,static_cast<int>(c.width)-1);
            const auto py=std::clamp(static_cast<int>(y)+dy,0,static_cast<int>(c.height)-1);
            const auto neighbor=static_cast<std::size_t>(py)*c.width+static_cast<unsigned>(px);
            if(std::find(neighborIndices.begin(),neighborIndices.end(),neighbor)==neighborIndices.end()) {
                neighborIndices.push_back(neighbor);neighbors.push_back(result.measuredElectrons[neighbor]);
            }
        }
        const auto index=static_cast<std::size_t>(y)*c.width+x;
        const double background=median(neighbors);
        contrast[index]=result.measuredElectrons[index]-background;
        variance[index]=std::max(1.,result.measuredElectrons[index]+c.readNoiseElectrons*c.readNoiseElectrons+adcStep*adcStep/12)+
            (background+c.readNoiseElectrons*c.readNoiseElectrons+adcStep*adcStep/12)*pi/(2*static_cast<double>(neighbors.size()));
        significance[index]=contrast[index]/std::sqrt(variance[index]);
        if(imaging){result.backgroundElectrons[index]=background;result.contrastElectrons[index]=contrast[index];
            result.noiseSigmaElectrons[index]=std::sqrt(variance[index]);result.detectionThresholdElectrons[index]=c.detectionSigma*std::sqrt(variance[index]);}
    }
    std::vector<std::vector<std::size_t>> componentPixels;
    if(imaging) {
        result.segmentationAvailable=true;result.segmentation.assign(pixels,0);
        std::vector<unsigned char> visited(pixels,0);std::vector<std::size_t> queue;queue.reserve(pixels);
        for(std::size_t seedPixel=0;seedPixel<pixels;++seedPixel) {
            if(visited[seedPixel]||std::abs(significance[seedPixel])<c.detectionSigma)continue;
            const double polarity=contrast[seedPixel]>=0?1.:-1.;queue.clear();queue.push_back(seedPixel);visited[seedPixel]=1;
            double total=0,xSum=0,ySum=0,noise=0;unsigned lowX=c.width,highX=0,lowY=c.height,highY=0;
            for(std::size_t at=0;at<queue.size();++at) {
                const auto index=queue[at];const auto x=static_cast<unsigned>(index%c.width),y=static_cast<unsigned>(index/c.width);
                const double weight=polarity*contrast[index];total+=weight;xSum+=weight*(x+.5-halfWidth)*c.pixelPitchRad;
                ySum+=weight*(y+.5-halfHeight)*c.pixelPitchRad;noise+=variance[index];
                lowX=std::min(lowX,x);highX=std::max(highX,x);lowY=std::min(lowY,y);highY=std::max(highY,y);
                for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
                    const int nx=static_cast<int>(x)+dx,ny=static_cast<int>(y)+dy;
                    if(nx<0||ny<0||nx>=static_cast<int>(c.width)||ny>=static_cast<int>(c.height))continue;
                    const auto next=static_cast<std::size_t>(ny)*c.width+static_cast<unsigned>(nx);
                    if(!visited[next]&&polarity*significance[next]>=.5*c.detectionSigma) {visited[next]=1;queue.push_back(next);}
                }
            }
            if(static_cast<double>(queue.size())<c.minimumFeaturePixels||total/std::sqrt(noise)<c.detectionSigma)continue;
            ThermalImageFeatureV1 feature;feature.direction=plane.ray(xSum/total,ySum/total);
            feature.centroidColumn=xSum/total/c.pixelPitchRad+halfWidth-.5;
            feature.centroidRow=ySum/total/c.pixelPitchRad+halfHeight-.5;
            feature.signalToNoise=total/std::sqrt(noise);feature.signedContrastElectrons=polarity*total;
            feature.extent={(highX-lowX+1)*c.pixelPitchRad,(highY-lowY+1)*c.pixelPitchRad};
            feature.angularVarianceRad2=std::max(c.pixelPitchRad*c.pixelPitchRad/12,
                c.psfSigmaRad*c.psfSigmaRad/(feature.signalToNoise*feature.signalToNoise));
            if(result.features.size()<32){result.features.push_back(feature);componentPixels.push_back(queue);}
            else {
                const auto least=std::min_element(result.features.begin(),result.features.end(),[](const auto& a,const auto& b){
                    return a.signalToNoise<b.signalToNoise;
                });
                if(feature.signalToNoise>least->signalToNoise){componentPixels[static_cast<std::size_t>(least-result.features.begin())]=queue;*least=feature;}
            }
        }
    } else {
        // Four physical quadrant channels integrate signed detector contrast.
        // Opposite-side sources can cancel. No selected source bearing, extent
        // or source class supplies the error or rescues an absent signal.
        // A guard detector around the active field supplies the measured DC
        // reference; no pixel-shape/connected-component processor enters this
        // nonimaging error. Its common reference noise is included below.
        std::vector<double> guard,guardNoise;
        const double dark=c.darkCurrentElectronsS*c.integrationTimeS;
        const auto varianceOf=[&](std::size_t i) {
            return std::max(1.,result.measuredElectrons[i]+c.readNoiseElectrons*c.readNoiseElectrons+adcStep*adcStep/12);
        };
        for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x)
            if(x==0||y==0||x+1==c.width||y+1==c.height) {
                const auto i=static_cast<std::size_t>(y)*c.width+x;const double weight=pixelWeights[i];
                // A uniform radiance has unequal corner/centre pixel flux.
                // Calibrate only the known optical etendue and dark offset;
                // the background level still comes from measured guards.
                guard.push_back((result.measuredElectrons[i]-dark)/weight);
                guardNoise.push_back(varianceOf(i)/(weight*weight));
            }
        auto orderedGuard=guard;const double background=median(orderedGuard);
        const double referenceNoise=pi*(*std::max_element(guardNoise.begin(),guardNoise.end()))/(2*static_cast<double>(guard.size()));
        const double threshold2=c.detectionSigma*c.detectionSigma+2*std::log(static_cast<double>(guard.size()));
        bool uniformBackground=true;
        for(std::size_t i=0;i<guard.size();++i) {
            const double residual=guard[i]-background;
            if(residual*residual>threshold2*(guardNoise[i]+referenceNoise))uniformBackground=false;
        }
        double left=0,right=0,top=0,bottom=0,total=0,noise=0,totalWeight=0;
        for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
            const auto i=static_cast<std::size_t>(y)*c.width+x;const auto v=result.measuredElectrons[i]-dark-background*pixelWeights[i];
            const double splitX=std::clamp(halfWidth-x,0.,1.),splitY=std::clamp(halfHeight-y,0.,1.);
            left+=splitX*v;right+=(1-splitX)*v;bottom+=splitY*v;top+=(1-splitY)*v;total+=v;
            result.quadrantElectrons[0]+=splitX*(1-splitY)*v;result.quadrantElectrons[1]+=(1-splitX)*(1-splitY)*v;
            result.quadrantElectrons[2]+=splitX*splitY*v;result.quadrantElectrons[3]+=(1-splitX)*splitY*v;
            noise+=varianceOf(i);totalWeight+=pixelWeights[i];
        }
        noise+=totalWeight*totalWeight*referenceNoise;
        result.quadrantBackgroundElectrons=background+dark;result.quadrantVarianceElectrons2=noise;
        // Gaussian inversion is inadmissible in a measured structured guard
        // field. This confidence test reads no sky/ground labels or source
        // attribution; physical charge and raw quadrant outputs are retained.
        if(uniformBackground&&std::abs(total)/std::sqrt(noise)>=c.detectionSigma) {
            const double dx=(right-left)/total,dy=(top-bottom)/total;
            // The physical difference is bounded by |total|. Read/shot/guard
            // noise perturbs both terms; the configured confidence envelope
            // is 2*sigma*sqrt(noise), not a fixed normalized-ratio cutoff.
            const double ratioUncertainty=2*c.detectionSigma*std::sqrt(noise)/std::abs(total);
            if(std::isfinite(dx)&&std::isfinite(dy)&&std::abs(dx)<=1+ratioUncertainty&&std::abs(dy)<=1+ratioUncertainty) {
                ThermalImageFeatureV1 feature;
                const double ax=std::sqrt(2.)*c.psfSigmaRad*inverseErf(dx),ay=std::sqrt(2.)*c.psfSigmaRad*inverseErf(dy);
                feature.direction=plane.ray(ax,ay);feature.signalToNoise=std::abs(total)/std::sqrt(noise);
                feature.centroidColumn=ax/c.pixelPitchRad+halfWidth-.5;feature.centroidRow=ay/c.pixelPitchRad+halfHeight-.5;
                feature.signedContrastElectrons=total;
                // Saturated quadrant ratios provide direction but little angle
                // information. Account for inverse-response conditioning.
                const double slope=std::sqrt(2/pi)/c.psfSigmaRad*
                    std::exp(-std::max(ax*ax,ay*ay)/(2*c.psfSigmaRad*c.psfSigmaRad));
                feature.angularVarianceRad2=std::min(c.pixelPitchRad*c.pixelPitchRad*c.width*c.width,
                    4/(feature.signalToNoise*feature.signalToNoise*slope*slope));
                const double uncertainty=ratioUncertainty;
                const auto unresolved=[&](double ratio) {
                    return std::abs(ratio)>uncertainty&&std::abs(ratio)+uncertainty>=1;
                };
                if(unresolved(dx)||unresolved(dy)) {
                    // The noisy ratio reaches the noninvertible endpoint.
                    // It supplies a centering sign, not an accurate angular
                    // magnitude. Bound it by the full physical detector
                    // diagonal instead of a local Gaussian derivative.
                    const double diameter=std::hypot(static_cast<double>(c.width),static_cast<double>(c.height))*c.pixelPitchRad;
                    feature.angularVarianceRad2=std::max(feature.angularVarianceRad2,diameter*diameter);
                }
                result.features.push_back(feature);
            }
        }
    }
    const auto featureLess=[](const auto& a,const auto& b){
        return std::tuple{-a.signalToNoise,a.direction.x,a.direction.y,a.direction.z}<
            std::tuple{-b.signalToNoise,b.direction.x,b.direction.y,b.direction.z};
    };
    if(imaging) {
        std::vector<std::size_t> permutation(result.features.size());std::iota(permutation.begin(),permutation.end(),0);
        std::sort(permutation.begin(),permutation.end(),[&](auto a,auto b){return featureLess(result.features[a],result.features[b]);});
        std::vector<ThermalImageFeatureV1> sorted;sorted.reserve(permutation.size());
        for(std::size_t i=0;i<permutation.size();++i) {
            sorted.push_back(result.features[permutation[i]]);
            for(const auto pixel:componentPixels[permutation[i]])result.segmentation[pixel]=static_cast<std::uint16_t>(i+1);
        }
        result.features=std::move(sorted);
    } else std::sort(result.features.begin(),result.features.end(),featureLess);
    // Referee-only attribution is computed AFTER measurements and follows the
    // actual visible signed detector charge, including its retained response.
    // Fully occluded sources contribute zero regardless of their temperature.
    if(unattributedResponse_.size()!=pixels)unattributedResponse_.assign(pixels,0);
    for(auto& value:unattributedResponse_)value*=1-response;
    // Scene indices name only the current frame. Detector lag is physical,
    // but its old frame-local slot is not a stable object identity. Retain
    // that charge as unknown rather than crediting a newly reused slot.
    for(const auto& [id,charge]:attributionResponse_)
        for(std::size_t i=0;i<pixels;++i)unattributedResponse_[i]+=(1-response)*charge[i];
    attributionResponse_.clear();
    for(const auto source:order) {
        auto& charge=scene.refereeAttributionIndices[source]?
            attributionResponse_[*scene.refereeAttributionIndices[source]]:unattributedResponse_;
        if(charge.size()!=pixels)charge.assign(pixels,0);
        for(std::size_t i=0;i<pixels;++i)charge[i]+=response*sourceContrast[source][i];
    }
    for(auto at=attributionResponse_.begin();at!=attributionResponse_.end();) {
        const double magnitude=std::accumulate(at->second.begin(),at->second.end(),0.,[](double sum,double value){return sum+std::abs(value);});
        if(magnitude<1e-9)at=attributionResponse_.erase(at);else ++at;
    }
    for(std::size_t featureIndex=0;featureIndex<result.features.size();++featureIndex) {
        auto& feature=result.features[featureIndex];double best=0;
        const double polarity=feature.signedContrastElectrons>=0?1.:-1.;
        const auto detectorScore=[&](const auto& charge) {
            double score=0;
            for(std::size_t i=0;i<pixels;++i)if(!imaging||result.segmentation[i]==featureIndex+1)
                score+=polarity*charge[i]*std::min(1.,c.fullWellElectrons/std::max(1e-300,result.expectedElectrons[i]));
            return score;
        };
        best=std::max(0.,detectorScore(unattributedResponse_));
        for(const auto& [id,charge]:attributionResponse_) {
            const double score=detectorScore(charge);
            if(score>best){best=score;feature.refereeAttributionIndex=id;}
        }
    }
    lastTimeS_=scene.timeS;cached_=result;return result;
}

std::optional<std::size_t> ThermalImageReceiverV1::apertureAttribution(double centreX,double centreY,double radius,double polarity) const {
    if(!cached_.valid)return std::nullopt;
    const auto& c=config_;std::vector<std::pair<std::size_t,double>> weights;
    for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
        const double x0=(x-.5*c.width)*c.pixelPitchRad,y0=(y-.5*c.height)*c.pixelPitchRad;
        if(x0>centreX+radius||x0+c.pixelPitchRad<centreX-radius||y0>centreY+radius||y0+c.pixelPitchRad<centreY-radius)continue;
        unsigned inside=0;
        for(unsigned sy=0;sy<4;++sy)for(unsigned sx=0;sx<4;++sx) {
            const double dx=x0+(sx+.5)*c.pixelPitchRad/4-centreX,dy=y0+(sy+.5)*c.pixelPitchRad/4-centreY;
            if(dx*dx+dy*dy<=radius*radius)++inside;
        }
        if(inside) {
            const auto i=static_cast<std::size_t>(y)*c.width+x;
            weights.push_back({i,inside/16.*std::min(1.,c.fullWellElectrons/std::max(1e-300,cached_.expectedElectrons[i]))});
        }
    }
    const auto score=[&](const auto& charge){double sum=0;for(const auto& [i,weight]:weights)sum+=weight*charge[i];return polarity*sum;};
    double best=std::max(0.,score(unattributedResponse_));std::optional<std::size_t> selected;
    for(const auto& [id,charge]:attributionResponse_)if(score(charge)>best){best=score(charge);selected=id;}
    return selected;
}

bool ThermalDualBandReceiverV1::validConfig(const ThermalDualBandConfigV1& c) noexcept {
    return c.version==1&&ThermalImageReceiverV1::validConfig(c.channels[0])&&ThermalImageReceiverV1::validConfig(c.channels[1])&&
        c.channels[0].integrationTimeS==c.channels[1].integrationTimeS&&
        positive(c.angularPairGateRad)&&c.angularPairGateRad<1.4&&
        positive(c.logRatioGate)&&c.logRatioGate<=20&&positive(c.spectralMemoryS)&&c.spectralMemoryS<=60;
}
bool ThermalDualBandReceiverV1::validScene(const ThermalDualBandSceneFrameV1& f,const ThermalDualBandConfigV1& c) noexcept {
    // Validate both before either detector mutates or draws noise. Two physical
    // bands share an exposure clock and pose, not radiance or detector noise.
    if(!validConfig(c)||f.channels[0].timeS!=f.channels[1].timeS||
        !ThermalImageReceiverV1::validScene(f.channels[0],c.channels[0])||
        !ThermalImageReceiverV1::validScene(f.channels[1],c.channels[1]))return false;
    const auto& a=f.channels[0];const auto& b=f.channels[1];
    return a.up.x==b.up.x&&a.up.y==b.up.y&&a.up.z==b.up.z&&a.groundHorizonCosine==b.groundHorizonCosine;
}
bool ThermalDualBandReceiverV1::configure(const ThermalDualBandConfigV1& c,std::string& error) {
    if(!validConfig(c)){error="invalid explicit dual-waveband image receiver policy";return false;}
    std::array<ThermalImageReceiverV1,2> receivers;
    if(!receivers[0].configure(c.channels[0],error)||!receivers[1].configure(c.channels[1],error))return false;
    config_=c;channels_=std::move(receivers);reset();return true;
}
void ThermalDualBandReceiverV1::reset() noexcept {
    for(auto& channel:channels_)channel.reset();
    retained_.reset();retainedTimeS_=-1;committedMeasurement_.reset();cached_={};
}
bool ThermalDualBandReceiverV1::commitSelectedMeasurementV1(double epoch,std::size_t featureIndex) {
    if(!cached_.valid||!std::isfinite(epoch)||epoch<0||epoch!=cached_.timeS||featureIndex>=cached_.features.size())return false;
    if(committedMeasurement_&&committedMeasurement_->first==epoch)return committedMeasurement_->second==featureIndex;
    retained_=cached_.features[featureIndex];retainedTimeS_=epoch;
    committedMeasurement_=std::pair{epoch,featureIndex};return true;
}
ThermalDualBandResultV1 ThermalDualBandReceiverV1::expose(const ThermalDualBandSceneFrameV1& frame,V pointing,std::uint64_t seed) {
    if(!validScene(frame,config_)||!finite(pointing)||!positive(norm(pointing))||
        (cached_.valid&&frame.channels[0].timeS<cached_.timeS))return {};
    if(cached_.valid&&frame.channels[0].timeS==cached_.timeS)return cached_;
    ThermalDualBandResultV1 result;result.timeS=frame.channels[0].timeS;
    result.channels[0]=channels_[0].expose(frame.channels[0],pointing,seed^0x42414e4430303031ull,true);
    result.channels[1]=channels_[1].expose(frame.channels[1],pointing,seed^0x42414e4430303032ull,true);
    if(!result.channels[0].valid||!result.channels[1].valid)return {};
    result.valid=true;
    if(retained_&&result.timeS-retainedTimeS_>config_.spectralMemoryS)retained_.reset();
    const auto gain=[](const auto& c){return c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*
        c.quantumEfficiency*c.effectiveWavelengthM/(planck*light);};
    struct Pair {std::size_t a,b;double separation;};std::vector<Pair> pairs;
    const auto plane=planeFor(pointing,unit(frame.channels[0].up));
    const double apertureRadius=std::max({2*config_.channels[0].psfSigmaRad,2*config_.channels[1].psfSigmaRad,
        1.5*config_.channels[0].pixelPitchRad,1.5*config_.channels[1].pixelPitchRad});
    const auto first=localApertureFeatures(result.channels[0],plane,apertureRadius);
    const auto second=localApertureFeatures(result.channels[1],plane,apertureRadius);
    for(std::size_t a=0;a<first.size();++a)for(std::size_t b=0;b<second.size();++b) {
        if(first[a].signedContrastElectrons*second[b].signedContrastElectrons<=0)continue;
        const double angle=std::acos(std::clamp(dot(first[a].direction,second[b].direction),-1.,1.));
        if(angle<=config_.angularPairGateRad)pairs.push_back({a,b,angle});
    }
    std::sort(pairs.begin(),pairs.end(),[](const auto& a,const auto& b){return std::tie(a.separation,a.a,a.b)<std::tie(b.separation,b.a,b.b);});
    std::vector<bool> usedFirst(first.size()),usedSecond(second.size());
    for(const auto pair:pairs) {
        if(usedFirst[pair.a]||usedSecond[pair.b])continue;
        usedFirst[pair.a]=usedSecond[pair.b]=true;
        const auto direction=unit(add(first[pair.a].direction,second[pair.b].direction));
        const double forward=dot(direction,plane.forward);
        const double ax=std::atan2(dot(direction,plane.right),forward),ay=std::atan2(dot(direction,plane.up),forward);
        const double polarity=first[pair.a].signedContrastElectrons>=0?1.:-1.;
        const auto aMeasurement=apertureFeature(result.channels[0],plane,ax,ay,apertureRadius,polarity);
        const auto bMeasurement=apertureFeature(result.channels[1],plane,ax,ay,apertureRadius,polarity);
        if(!aMeasurement||!bMeasurement)continue;
        const auto& a=*aMeasurement;const auto& b=*bMeasurement;
        const double colour=std::log(std::abs(a.signedContrastElectrons)/gain(config_.channels[0]))-
            std::log(std::abs(b.signedContrastElectrons)/gain(config_.channels[1]));
        const double variance=1/(a.signalToNoise*a.signalToNoise)+1/(b.signalToNoise*b.signalToNoise);
        if(!std::isfinite(colour)||!positive(variance))continue;
        if(retained_&&std::abs(colour-*retained_->logSpectralRatio)>
            config_.logRatioGate+3*std::sqrt(variance+retained_->logSpectralVariance))continue;
        ThermalDualBandFeatureV1 feature;static_cast<ThermalImageFeatureV1&>(feature)=a;
        const double wa=1/a.angularVarianceRad2,wb=1/b.angularVarianceRad2;
        feature.direction=unit(add(scale(a.direction,wa),scale(b.direction,wb)));
        feature.angularVarianceRad2=1/(wa+wb);feature.signalToNoise=std::hypot(a.signalToNoise,b.signalToNoise);
        feature.logSpectralRatio=colour;feature.logSpectralVariance=variance;
        // Both physical channels must attribute to the same visible object.
        // Attribution disagreement cannot alter measured spectral processing.
        if(a.refereeAttributionIndex!=b.refereeAttributionIndex)feature.refereeAttributionIndex.reset();
        result.features.push_back(feature);
    }
    // Ordering is presentation only. Native family association selects the
    // actual measured feature and explicitly commits its spectral history.
    std::sort(result.features.begin(),result.features.end(),[&](const auto& a,const auto& b){
        const auto score=[&](const auto& feature) {
            return retained_?std::abs(*feature.logSpectralRatio-*retained_->logSpectralRatio)/config_.logRatioGate+
                std::acos(std::clamp(dot(feature.direction,retained_->direction),-1.,1.))/config_.channels[0].associationGateRad:
                -feature.signalToNoise;
        };
        return std::tuple{score(a),a.direction.x,a.direction.y,a.direction.z}<std::tuple{score(b),b.direction.x,b.direction.y,b.direction.z};
    });
    result.spectralAssociationAvailable=!result.features.empty();
    // Referee association is a final pass over exactly the common measured
    // aperture; a merged island's brighter competitor cannot own this peak.
    for(auto& feature:result.features) {
        const double forward=dot(feature.direction,plane.forward);
        const double ax=std::atan2(dot(feature.direction,plane.right),forward),ay=std::atan2(dot(feature.direction,plane.up),forward);
        const double polarity=feature.signedContrastElectrons>=0?1.:-1.;
        const auto a=channels_[0].apertureAttribution(ax,ay,apertureRadius,polarity),b=channels_[1].apertureAttribution(ax,ay,apertureRadius,polarity);
        if(a==b)feature.refereeAttributionIndex=a;
    }
    cached_=result;return result;
}
} // namespace fsim::ms::components
