#include "ThermalReticleReceiverV1.h"
#include "MovingRadiatorInverseV2.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <chrono>
#include <unordered_set>
#include <cstring>

namespace fsim::ms::components {
namespace {
using namespace measured;
using V=weapon::V3;
struct UnitGroundClockV2 {
    using Clock=std::chrono::steady_clock;
    ThermalGroundWorkV2* work;bool original;Clock::time_point began{};
    UnitGroundClockV2(ThermalGroundWorkV2* report,bool native):work(report),original(native){if(work)began=Clock::now();}
    ~UnitGroundClockV2(){if(work){const double us=std::chrono::duration<double,std::micro>(Clock::now()-began).count();if(original)work->originalMicros+=us;else work->totalMicros+=us;}}
};

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
using P4=std::array<double,4>;
// Box-constrained, column-scaled augmented least squares. Pivoted Householder
// QR avoids squaring the conditioning of nearly dependent optical columns.
// All3^N active faces are finite numerical proposals, not accepted detections.
// The4D specialization retains the original operation and face order.
template<std::size_t Dimension>
bool dampedBoxStep(const std::array<std::vector<double>,Dimension>& jacobian,const std::vector<double>& residual,
    const std::array<double,Dimension>& low,const std::array<double,Dimension>& high,double damping,std::array<double,Dimension>& result) {
    static_assert(Dimension==4||Dimension==5);
    constexpr unsigned faceCount=Dimension==4?81:243;
    const auto count=residual.size(),rows=count+Dimension;
    if(!(damping>0)||!std::isfinite(damping))return false;
    std::array<double,Dimension> norms{};for(unsigned column=0;column<Dimension;++column) {
        if(jacobian[column].size()!=count)return false;
        for(const auto value:jacobian[column])norms[column]=std::hypot(norms[column],value);
        if(!std::isfinite(norms[column]))return false;
        if(norms[column]==0)norms[column]=1;
    }
    std::vector<std::array<double,Dimension>> design(rows);std::vector<double> target(rows);
    const double regularizer=std::sqrt(damping);
    for(std::size_t row=0;row<count;++row) {
        target[row]=-residual[row];if(!std::isfinite(target[row]))return false;
        for(unsigned column=0;column<Dimension;++column)design[row][column]=jacobian[column][row]/norms[column];
    }
    for(unsigned column=0;column<Dimension;++column)design[count+column][column]=regularizer;
    std::array<double,Dimension> scaledLow{},scaledHigh{};
    for(unsigned axis=0;axis<Dimension;++axis) {
        scaledLow[axis]=low[axis]*norms[axis];scaledHigh[axis]=high[axis]*norms[axis];
        if(!std::isfinite(scaledLow[axis])||!std::isfinite(scaledHigh[axis])||scaledLow[axis]>scaledHigh[axis])return false;
    }
    double bestCost=std::numeric_limits<double>::infinity();std::array<double,Dimension> selected{};
    for(unsigned face=0;face<faceCount;++face) {
        unsigned encoded=face,freeCount=0;std::array<unsigned,Dimension> free{};std::array<double,Dimension> step{};
        auto rhs=target;
        for(unsigned axis=0;axis<Dimension;++axis) {
            const auto state=encoded%3;encoded/=3;
            if(state==0)free[freeCount++]=axis;
            else {
                step[axis]=state==1?scaledLow[axis]:scaledHigh[axis];
                for(std::size_t row=0;row<rows;++row)rhs[row]-=design[row][axis]*step[axis];
            }
        }
        std::vector<std::array<double,Dimension>> matrix(rows);
        for(std::size_t row=0;row<rows;++row)for(unsigned column=0;column<freeCount;++column)
            matrix[row][column]=design[row][free[column]];
        bool valid=true;
        for(unsigned column=0;column<freeCount&&valid;++column) {
            unsigned pivot=column;double largest=-1;
            for(unsigned candidate=column;candidate<freeCount;++candidate) {
                double length=0;for(std::size_t row=column;row<rows;++row)length=std::hypot(length,matrix[row][candidate]);
                if(length>largest){largest=length;pivot=candidate;}
            }
            if(!(largest>0)||!std::isfinite(largest)){valid=false;break;}
            if(pivot!=column) {
                std::swap(free[pivot],free[column]);
                for(std::size_t row=0;row<rows;++row)std::swap(matrix[row][pivot],matrix[row][column]);
            }
            const double diagonal=-std::copysign(largest,matrix[column][column]);
            std::vector<double> reflector(rows,0);
            for(std::size_t row=column;row<rows;++row)reflector[row]=matrix[row][column];
            reflector[column]-=diagonal;
            double norm2=0;for(std::size_t row=column;row<rows;++row)norm2+=reflector[row]*reflector[row];
            if(!(norm2>0)||!std::isfinite(norm2)){valid=false;break;}
            const double beta=2/norm2;
            for(unsigned destination=column;destination<freeCount;++destination) {
                double dot=0;for(std::size_t row=column;row<rows;++row)dot+=reflector[row]*matrix[row][destination];
                for(std::size_t row=column;row<rows;++row)matrix[row][destination]-=beta*reflector[row]*dot;
            }
            double dot=0;for(std::size_t row=column;row<rows;++row)dot+=reflector[row]*rhs[row];
            for(std::size_t row=column;row<rows;++row)rhs[row]-=beta*reflector[row]*dot;
            matrix[column][column]=diagonal;
        }
        if(!valid)continue;
        std::array<double,Dimension> solved{};
        for(unsigned reverse=freeCount;reverse>0;--reverse) {
            const auto row=reverse-1;double value=rhs[row];
            for(unsigned column=row+1;column<freeCount;++column)value-=matrix[row][column]*solved[column];
            solved[row]=value/matrix[row][row];if(!std::isfinite(solved[row])){valid=false;break;}
        }
        if(!valid)continue;
        for(unsigned column=0;column<freeCount;++column)step[free[column]]=solved[column];
        for(unsigned axis=0;axis<Dimension;++axis) {
            const double tolerance=256*std::numeric_limits<double>::epsilon()*
                std::max({1.,std::abs(scaledLow[axis]),std::abs(scaledHigh[axis])});
            if(step[axis]<scaledLow[axis]-tolerance||step[axis]>scaledHigh[axis]+tolerance){valid=false;break;}
            step[axis]=std::clamp(step[axis],scaledLow[axis],scaledHigh[axis]);
        }
        if(!valid)continue;
        double cost=0;
        for(std::size_t row=0;row<rows;++row) {
            double error=-target[row];for(unsigned column=0;column<Dimension;++column)error+=design[row][column]*step[column];
            cost+=error*error;
        }
        if(std::isfinite(cost)&&cost<bestCost){bestCost=cost;selected=step;}
    }
    if(!std::isfinite(bestCost))return false;
    for(unsigned axis=0;axis<Dimension;++axis)result[axis]=selected[axis]/norms[axis];
    return true;
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
std::optional<double> ThermalReticleReceiverV1::physicalBackgroundAmplitudeV1(double skyAtZero,double skyProjection,double unconstrained) noexcept {
    if(!std::isfinite(skyAtZero)||!std::isfinite(skyProjection)||!std::isfinite(unconstrained))return std::nullopt;
    double low=-std::numeric_limits<double>::infinity(),high=std::numeric_limits<double>::infinity();
    for(const double coefficient:{-skyProjection,1-skyProjection}) {
        if(coefficient==0){if(skyAtZero<0)return std::nullopt;continue;}
        const double boundary=-skyAtZero/coefficient;
        if(coefficient>0)low=std::max(low,boundary);else high=std::min(high,boundary);
    }
    if(low>high)return std::nullopt;
    double admitted=std::clamp(unconstrained,low,high);
    for(unsigned iteration=0;iteration<8;++iteration) {
        if(!std::isfinite(admitted))return std::nullopt;
        const double sky=skyAtZero-skyProjection*admitted,ground=sky+admitted;
        if(std::isfinite(sky)&&std::isfinite(ground)&&sky>=0&&ground>=0)return admitted;
        const double inside=std::isfinite(low)&&std::isfinite(high)?low/2+high/2:
            std::isfinite(low)?std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity();
        const double next=std::nextafter(admitted,inside);
        if(next==admitted||next<low||next>high)return std::nullopt;
        admitted=next;
    }
    return std::nullopt;
}
std::optional<double> ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(const ThermalReticleConfigV1& config,
    const std::vector<std::array<double,5>>& modes) noexcept {
    if(!validConfig(config)||modes.empty())return std::nullopt;
    const auto& p=config.detector;
    const double halfX=.5*p.width*p.pixelPitchRad,halfY=.5*p.height*p.pixelPitchRad;
    const auto ray=[](const std::array<double,5>& q){return unit(V{1,std::tan(q[0]),std::tan(q[1])});};
    const auto selected=ray(modes.front());
    double lowX=-halfX,highX=halfX,lowY=-halfY,highY=halfY,envelope=0;
    const auto clearance=[&](V direction) {
        double distance=std::atan(config.stopRadiusRad+config.scanRadiusRad)-
            std::atan2(std::hypot(direction.y,direction.z),direction.x);
        for(const double sign:{-1.,1.}) {
            distance=std::min(distance,std::asin(std::clamp(std::sin(halfX)*direction.x+sign*std::cos(halfX)*direction.y,-1.,1.)));
            distance=std::min(distance,std::asin(std::clamp(std::sin(halfY)*direction.x+sign*std::cos(halfY)*direction.z,-1.,1.)));
        }
        return distance;
    };
    for(const auto& q:modes) {
        if(!std::all_of(q.begin(),q.end(),[](double value){return std::isfinite(value);})||q[4]<=0||
            q[2]<0||q[3]<0||q[2]>=halfX||q[3]>=halfY||
            q[0]-.5*q[2]<=-halfX||q[0]+.5*q[2]>=halfX||q[1]-.5*q[3]<=-halfY||q[1]+.5*q[3]>=halfY)return std::nullopt;
        const auto direction=ray(q);
        if(!(p.detectionSigma*std::sqrt(q[4])<clearance(direction)))return std::nullopt;
        lowX=std::max(lowX,q[0]-.5*q[2]);highX=std::min(highX,q[0]+.5*q[2]);
        lowY=std::max(lowY,q[1]-.5*q[3]);highY=std::min(highY,q[1]+.5*q[3]);
        if(lowX>highX||lowY>highY)return std::nullopt;
        // Outward rounding preserves the selected mode and local error even
        // when a subpixel reflected centre is only a few ulps away.
        const double displacement=std::nextafter(std::atan2(norm(cross(selected,direction)),dot(selected,direction)),
            std::numeric_limits<double>::infinity());
        const double spread=std::nextafter(std::sqrt(q[4]),std::numeric_limits<double>::infinity());
        const double total=std::nextafter(spread+displacement,std::numeric_limits<double>::infinity());
        envelope=std::max(envelope,std::nextafter(total*total,std::numeric_limits<double>::infinity()));
    }
    if(!std::isfinite(envelope)||!(p.detectionSigma*std::sqrt(envelope)<clearance(selected)))return std::nullopt;
    return envelope;
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
    std::vector<double> result(c.samplesPerScan,0);
    const auto primitive=[&](double edge,double centre){return .5*std::erfc((centre-edge)/(std::sqrt(2.)*sigma));};
    const auto probability=[&](double low,double high,double centre,double extent) {
        return extent==0?primitive(high,centre)-primitive(low,centre):
            ThermalImageReceiverV1::gaussianIntervalV1(centre-.5*extent,centre+.5*extent,low,high,sigma)/extent;
    };
    // Explicit nine-sigma numerical support, matching the retained optical
    // renderer's tail bound; lost FOV energy is never renormalized.
    std::array<double,128> xProbabilities{};
    std::array<bool,128> includedColumns{};
    bool columnsPrepared=false;
    for(unsigned row=0;row<p.height;++row) {
        const double lowY=(row-.5*p.height)*p.pixelPitchRad,highY=lowY+p.pixelPitchRad;
        if(lowY>y+.5*height+9*sigma||highY<y-.5*height-9*sigma)continue;
        const double wy=probability(lowY,highY,y,height);
        // X probability and inclusion are independent of row. Prepare in the
        // original column order at the first active row, avoiding any extra
        // probability evaluation when the entire Y support is off field.
        if(!columnsPrepared) {
            for(unsigned col=0;col<p.width;++col) {
                const double lowX=(col-.5*p.width)*p.pixelPitchRad,highX=lowX+p.pixelPitchRad;
                if(lowX>x+.5*width+9*sigma||highX<x-.5*width-9*sigma)continue;
                includedColumns[col]=true;xProbabilities[col]=probability(lowX,highX,x,width);
            }
            columnsPrepared=true;
        }
        for(unsigned col=0;col<p.width;++col) {
            if(!includedColumns[col])continue;
            const auto index=static_cast<std::size_t>(row)*p.width+col;
            const double weight=wy*xProbabilities[col]*solidAngles_[index]/(p.pixelPitchRad*p.pixelPitchRad);
            for(unsigned j=0;j<c.samplesPerScan;++j)result[j]+=weight*weights_[index*c.samplesPerScan+j];
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    result=filtered(std::move(result),alpha);if(removeNuisance)project(result);return result;
}
std::array<std::vector<double>,4> ThermalReticleReceiverV1::spotJacobianV1(double x,double y,double width,double height) const {
    const auto& c=config_;const auto& p=c.detector;
    // Derivatives are with respect to centreX, centreY, width^2, height^2.
    // Squared extent has a finite nonzero Gaussian second-derivative limit
    // at a point source; a width derivative would falsely stick at zero.
    struct Axis {double probability,centre,squaredExtent;};
    const auto axis=[&](double low,double high,double centre,double extent) {
        const double sigma=p.psfSigmaRad;
        const auto density=[](double z){return std::exp(-.5*z*z)/std::sqrt(2*pi);};
        const auto at=[&](double location){return .5*std::erfc((location-high)/(std::sqrt(2.)*sigma))-
            .5*std::erfc((location-low)/(std::sqrt(2.)*sigma));};
        const double a=(low-centre)/sigma,b=(high-centre)/sigma;
        const double pointCentre=(density(a)-density(b))/sigma;
        const double pointSquared=(a*density(a)-b*density(b))/(24*sigma*sigma);
        if(extent==0)return Axis{at(centre),pointCentre,pointSquared};
        const double left=centre-.5*extent,right=centre+.5*extent;
        const double integral=ThermalImageReceiverV1::gaussianIntervalV1(left,right,low,high,sigma);
        const double probability=integral/extent;
        if(extent<sigma*std::pow(std::numeric_limits<double>::epsilon(),.25))
            return Axis{probability,pointCentre,pointSquared};
        const double pl=at(left),pr=at(right);
        return Axis{probability,(pr-pl)/extent,(.5*(pr+pl)*extent-integral)/(2*extent*extent*extent)};
    };
    std::vector<Axis> xs(p.width),ys(p.height);
    for(unsigned col=0;col<p.width;++col)xs[col]=axis((col-.5*p.width)*p.pixelPitchRad,(col+1-.5*p.width)*p.pixelPitchRad,x,width);
    for(unsigned row=0;row<p.height;++row)ys[row]=axis((row-.5*p.height)*p.pixelPitchRad,(row+1-.5*p.height)*p.pixelPitchRad,y,height);
    std::array<std::vector<double>,4> derivative;
    for(auto& values:derivative)values.assign(c.samplesPerScan,0);
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const auto index=static_cast<std::size_t>(row)*p.width+col;
        const auto& a=xs[col];const auto& b=ys[row];
        const std::array<double,4> coefficient{{b.probability*a.centre,b.centre*a.probability,
                                               b.probability*a.squaredExtent,b.squaredExtent*a.probability}};
        const double omega=solidAngles_[index]/(p.pixelPitchRad*p.pixelPitchRad);
        for(unsigned j=0;j<c.samplesPerScan;++j) {
            const double value=weights_[index*c.samplesPerScan+j]*omega;
            for(unsigned coordinate=0;coordinate<4;++coordinate)derivative[coordinate][j]+=value*coefficient[coordinate];
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    for(auto& values:derivative){values=filtered(std::move(values),alpha);project(values);}
    return derivative;
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
            next.weights_[index*c.samplesPerScan+j]+=transmission/p.exposureSamples;
        }
        for(unsigned j=0;j<c.samplesPerScan;++j)next.flat_[j]+=next.solidAngles_[index]*next.weights_[index*c.samplesPerScan+j];
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
    // Linearity of optics, electronics and nuisance projection permits exact
    // unions of disjoint source cells. This bank only proposes inverse-fit
    // starts; every selected rectangle is re-evaluated by spot() below.
    // Include the same nine-sigma off-field support as the native renderer.
    const auto spacing=std::max(1u,static_cast<unsigned>(std::ceil(2*p.psfSigmaRad/p.pixelPitchRad)));
    next.supportPitch_=spacing*p.pixelPitchRad;
    next.supportMargin_=static_cast<unsigned>(std::ceil(9*p.psfSigmaRad/next.supportPitch_));
    next.supportColumns_=static_cast<unsigned>(std::ceil(static_cast<double>(p.width)/spacing))+2*next.supportMargin_;
    next.supportRows_=static_cast<unsigned>(std::ceil(static_cast<double>(p.height)/spacing))+2*next.supportMargin_;
    const auto stride=static_cast<std::size_t>(next.supportColumns_+1)*c.samplesPerScan;
    next.supportPrefix_.assign(static_cast<std::size_t>(next.supportRows_+1)*stride,0.);
    for(unsigned row=0;row<next.supportRows_;++row)for(unsigned col=0;col<next.supportColumns_;++col) {
        const auto values=next.spot((col+.5-next.supportMargin_)*next.supportPitch_-.5*p.width*p.pixelPitchRad,
            (row+.5-next.supportMargin_)*next.supportPitch_-.5*p.height*p.pixelPitchRad,
            p.psfSigmaRad,next.supportPitch_,next.supportPitch_);
        const auto here=static_cast<std::size_t>(row+1)*stride+static_cast<std::size_t>(col+1)*c.samplesPerScan;
        for(unsigned j=0;j<c.samplesPerScan;++j)
            next.supportPrefix_[here+j]=values[j]+next.supportPrefix_[here-stride+j]+
                next.supportPrefix_[here-c.samplesPerScan+j]-next.supportPrefix_[here-stride-c.samplesPerScan+j];
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
    return exposeWithMeasuredPriorV2(scene,pointing,seed,nullptr);
}

ThermalReticleResultV1 ThermalReticleReceiverV1::exposeWithMeasuredPriorV2(
    const ThermalSceneFrameV1& scene,V pointing,std::uint64_t seed,const ThermalReticleMeasuredPriorV2* prior) {
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
    if(!result.saturated)inferMeasuredWithPriorV2(result,pointing,scene.up,prior);
    cached_=result;lastTimeS_=scene.timeS;return result;
}

void ThermalReticleReceiverV1::inferStaticHorizonForCalibrationV2(ThermalReticleResultV1& result,V pointing,V up,
    const ThermalReticleMeasuredPriorV2* prior) const {
    result.horizonDiagnostic={};
    result.features.clear();result.bearingResolved=false;result.preciseBearing=false;
    result.fitSnr=0;result.fitResidual=0;result.compactFitSnr=0;result.compactFitResidual=0;
    result.fittedCentreXRad=0;result.fittedCentreYRad=0;result.fittedWidthRad=0;result.fittedHeightRad=0;
    result.ambiguousAlternativeKind=0;result.alternativeRadianceAvailable=false;result.alternativeShape={};
    result.alternativeAmplitude=0;result.alternativeCostDifference=0;result.alternativeSky=0;result.alternativeSource=0;
    if(!result.valid||!result.complete||result.saturated||weights_.empty()||
       result.measuredElectrons.size()!=config_.samplesPerScan||!std::isfinite(result.varianceBoundElectrons2)||
       result.varianceBoundElectrons2<=0||!finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6||
       !std::all_of(result.measuredElectrons.begin(),result.measuredElectrons.end(),[](double v){return std::isfinite(v)&&v>=0;}))return;
    fitHorizonWithMeasuredPriorV2(result,pointing,up,nullptr,false,prior);
}

void ThermalReticleReceiverV1::inferMeasuredV1(ThermalReticleResultV1& result,V pointing,V up) const {
    inferMeasuredWithPriorV2(result,pointing,up,nullptr);
}

void ThermalReticleReceiverV1::inferMeasuredWithPriorV2(ThermalReticleResultV1& result,V pointing,V up,
    const ThermalReticleMeasuredPriorV2* prior) const {
    result.horizonDiagnostic={};
    result.features.clear();result.bearingResolved=false;result.preciseBearing=false;
    result.fitSnr=0;result.fitResidual=0;result.compactFitSnr=0;result.compactFitResidual=0;
    result.fittedCentreXRad=0;result.fittedCentreYRad=0;result.fittedWidthRad=0;result.fittedHeightRad=0;
    result.ambiguousAlternativeKind=0;result.alternativeRadianceAvailable=false;result.alternativeShape={};
    result.alternativeAmplitude=0;result.alternativeCostDifference=0;result.alternativeSky=0;result.alternativeSource=0;
    if(!result.valid||!result.complete||result.saturated||weights_.empty()||
       result.measuredElectrons.size()!=config_.samplesPerScan||!std::isfinite(result.varianceBoundElectrons2)||
       result.varianceBoundElectrons2<=0||!finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6||
       !std::all_of(result.measuredElectrons.begin(),result.measuredElectrons.end(),[](double v){return std::isfinite(v)&&v>=0;}))return;
    fit(result,pointing,up);
    if(!result.features.empty()) {
        auto hypothesis=result;hypothesis.features.clear();hypothesis.bearingResolved=false;hypothesis.preciseBearing=false;
        bool backgroundExplained=false;
        fitHorizonWithMeasuredPriorV2(hypothesis,pointing,up,&backgroundExplained,true,prior);
        result.horizonDiagnostic=hypothesis.horizonDiagnostic;
        if(backgroundExplained||hypothesis.horizonDiagnostic.refusal==21)result=std::move(hypothesis);
    } else if(result.features.empty())fitHorizonWithMeasuredPriorV2(result,pointing,up,nullptr,false,prior);
    // The stationary family and ALL its thresholds/diagnostics above remain
    // unchanged. A separately versioned measured-only family handles source
    // motion during an otherwise complete scalar exposure.
    if(result.features.empty()&&result.horizonDiagnostic.refusal!=2&&result.horizonDiagnostic.refusal!=5&&
       result.horizonDiagnostic.refusal!=21)inferMovingMeasuredV2(result,pointing,up);
}

void ThermalReticleReceiverV1::inferMovingMeasuredV2(ThermalReticleResultV1& result,V pointing,V up,
    ThermalReticleMovingDiagnosticV2* diagnostic) const {
    const moving_radiator_v2::Optics optics{config_,weights_,timeWeights_,solidAngles_,flat_,nuisance_,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    moving_radiator_v2::infer(optics,result,pointing,up,diagnostic);
}

std::optional<std::array<std::vector<double>,2>> ThermalReticleReceiverV1::movingBasisForCalibrationV2(
    const ThermalReticleResultV1& clock,V pointing,V up,const std::array<double,7>& shape,unsigned branch) const {
    const moving_radiator_v2::Optics optics{config_,weights_,timeWeights_,solidAngles_,flat_,nuisance_,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    return moving_radiator_v2::calibrateBasis(optics,clock,pointing,up,shape,branch);
}

std::optional<std::array<std::vector<double>,2>> ThermalReticleReceiverV1::movingBasisForCalibrationV4(
    const ThermalReticleResultV1& clock,V pointing,V up,const std::array<double,7>& shape,unsigned branch) const {
    const moving_radiator_v2::Optics optics{config_,weights_,timeWeights_,solidAngles_,flat_,nuisance_,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    return moving_radiator_v2::calibrateBasisV4(optics,clock,pointing,up,shape,branch);
}

std::optional<std::array<std::vector<double>,2>> ThermalReticleReceiverV1::movingBasisForCalibrationV5(
    const ThermalReticleResultV1& clock,V pointing,V up,const std::array<double,7>& shape,unsigned branch) const {
    const moving_radiator_v2::Optics optics{config_,weights_,timeWeights_,solidAngles_,flat_,nuisance_,
        [this](const std::array<double,4>& q){return spot(q[0],q[1],config_.detector.psfSigmaRad,q[2],q[3],false);}};
    return moving_radiator_v2::calibrateBasisV5(optics,clock,pointing,up,shape,branch);
}

std::optional<std::array<double,2>> ThermalReticleReceiverV1::rawSkyFitV1(
    const ThermalReticleResultV1& result,const std::vector<double>& rawTemplate) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    if(rawTemplate.size()!=count||result.measuredElectrons.size()!=count)return std::nullopt;
    const double alpha=-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS));
    auto uniform=filtered(flat_,alpha);for(auto& value:uniform)value/=p.pixelPitchRad*p.pixelPitchRad;
    std::vector<double> transient(count),remaining=result.measuredElectrons;
    double dark=0;const double input=static_cast<double>(p.width)*p.height*p.darkCurrentElectronsS*p.integrationTimeS;
    for(unsigned j=0;j<count;++j){transient[j]=std::pow(1-alpha,j+1);dark+=alpha*(input-dark);remaining[j]-=dark;}
    const double uu=inner(uniform,uniform),tt=inner(transient,transient),ut=inner(uniform,transient),det=uu*tt-ut*ut;
    if(!(uu>0)||!std::isfinite(uu)||!std::isfinite(tt))return std::nullopt;
    if(tt>0&&!(det>128*std::numeric_limits<double>::epsilon()*uu*tt))return std::nullopt;
    const double sky=tt>0?(inner(remaining,uniform)*tt-inner(remaining,transient)*ut)/det:inner(remaining,uniform)/uu;
    const double projection=tt>0?(inner(rawTemplate,uniform)*tt-inner(rawTemplate,transient)*ut)/det:inner(rawTemplate,uniform)/uu;
    if(!std::isfinite(sky)||!std::isfinite(projection))return std::nullopt;
    return std::array<double,2>{{sky,projection}};
}

std::optional<double> ThermalReticleReceiverV1::physicalSourceAmplitudeV1(const ThermalReticleResultV1& result,const P4& q,double amplitude) const {
    const auto& p=config_.detector;
    const auto budget=rawSkyFitV1(result,spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],false));
    if(!budget)return std::nullopt;
    const double cells=q[2]*q[3]/(p.pixelPitchRad*p.pixelPitchRad);
    if(cells>0) {
        const double divided=amplitude/cells;
        const auto physical=physicalBackgroundAmplitudeV1((*budget)[0],(*budget)[1]*cells,divided);
        if(!physical)return std::nullopt;
        if(*physical==divided)return amplitude;
        const double value=*physical*cells;
        if(!std::isfinite(value))return std::nullopt;
        return value;
    }
    double low=0,high=std::numeric_limits<double>::infinity();
    if((*budget)[1]>0)high=(*budget)[0]/(*budget)[1];
    else if((*budget)[1]<0)low=std::max(0.,(*budget)[0]/(*budget)[1]);
    else if((*budget)[0]<0)return std::nullopt;
    if(low>high)return std::nullopt;
    const double value=std::clamp(amplitude,low,high);
    if(!std::isfinite(value)||(*budget)[0]-(*budget)[1]*value<0)return std::nullopt;
    return value;
}

std::optional<double> ThermalReticleReceiverV1::localAngularVarianceV1(const P4& q,double amplitude,double variance,double lattice) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    const double h=p.pixelPitchRad/1024;
    auto xp=spot(q[0]+h,q[1],p.psfSigmaRad,q[2],q[3]),xm=spot(q[0]-h,q[1],p.psfSigmaRad,q[2],q[3]);
    auto yp=spot(q[0],q[1]+h,p.psfSigmaRad,q[2],q[3]),ym=spot(q[0],q[1]-h,p.psfSigmaRad,q[2],q[3]);
    const auto centre=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);const double norm2=inner(centre,centre);
    for(unsigned j=0;j<count;++j){xp[j]=(xp[j]-xm[j])/(2*h);yp[j]=(yp[j]-ym[j])/(2*h);}
    // Eliminate fitted support widths as nuisance parameters as well as the
    // unknown amplitude. An extent is a waveform fit, never a source oracle.
    std::vector<std::vector<double>> shape;
    for(unsigned coordinate=0;coordinate<2;++coordinate) {
        if((coordinate?q[3]:q[2])<=h)continue;
        auto a=spot(q[0],q[1],p.psfSigmaRad,q[2]+(coordinate?0:h),q[3]+(coordinate?h:0));
        const auto b=spot(q[0],q[1],p.psfSigmaRad,q[2]-(coordinate?0:h),q[3]-(coordinate?h:0));
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
    if(!std::isfinite(determinant)||determinant<=128*std::numeric_limits<double>::epsilon()*trace*trace||xx<=0||yy<=0)return std::nullopt;
    const double tx=std::tan(q[0]),ty=std::tan(q[1]),n2=1+tx*tx+ty*ty;
    const double gx=(1+tx*tx)*(1+tx*tx)*(1+ty*ty)/(n2*n2);
    const double gy=(1+ty*ty)*(1+ty*ty)*(1+tx*tx)/(n2*n2);
    const double gxy=-tx*ty*(1+tx*tx)*(1+ty*ty)/(n2*n2);
    const double angularMetric=.5*(gx+gy+std::hypot(gx-gy,2*gxy));
    const double value=(variance*(trace+std::hypot(xx-yy,2*xy))/(2*determinant*amplitude*amplitude)+lattice*lattice/12)*angularMetric;
    if(!std::isfinite(value)||value<=0)return std::nullopt;
    return value;
}

void ThermalReticleReceiverV1::fit(ThermalReticleResultV1& result,V pointing,V up) const {
    const auto& c=config_;const auto& p=c.detector;const auto count=c.samplesPerScan;
    auto data=result.measuredElectrons;project(data);
    const double dataNorm=inner(data,data),variance=result.varianceBoundElectrons2;
    if(!std::isfinite(dataNorm)||!std::isfinite(variance)||variance<=0)return;
    double best=0,bestX=0,bestY=0,bestWidth=0,bestHeight=0,bestAmplitude=0,bestNorm=0;
    struct Seed {double score;P4 shape;};std::vector<Seed> supportSeeds;
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
    const auto refine=[&](double initialStep) {
        step=initialStep;
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
    };
    refine(p.pixelPitchRad);
    const double degrees=static_cast<double>(count)-static_cast<double>(nuisance_.size())-5,tail=.5*p.detectionSigma*p.detectionSigma;
    const double residualBound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    const double subdivisions=std::ldexp(1.,18);
    const double logTrials=2*std::log(p.width*subdivisions+1)+2*std::log(p.height*subdivisions+1);
    const double threshold=std::sqrt(p.detectionSigma*p.detectionSigma+2*(std::log(2.)+logTrials));
    const P4 initialShape{{bestX,bestY,bestWidth,bestHeight}};
    const auto initialPhysical=physicalSourceAmplitudeV1(result,initialShape,bestAmplitude);
    const double initialPhysicalScore=initialPhysical?
        2*(*initialPhysical)*bestAmplitude*bestNorm-(*initialPhysical)*(*initialPhysical)*bestNorm:0;
    const bool physicalRecovery=std::sqrt(std::max(0.,best)/variance)>=threshold&&
        (!(initialPhysicalScore>0)||!std::isfinite(initialPhysicalScore)||
         std::max(0.,dataNorm-initialPhysicalScore)/variance>residualBound);
    if(std::max(0.,dataNorm-best)/variance>residualBound||physicalRecovery) {
        // A wide source may have its largest point-template correlation at a
        // remote reticle alias. Search finite support AND centre jointly before
        // repeating the unchanged exact local refinement. A large correlation
        // can never substitute for the final waveform goodness-of-fit gate.
        double proposed=best;
        std::array<double,4> proposal{bestX,bestY,bestWidth,bestHeight};
        const auto stride=static_cast<std::size_t>(supportColumns_+1)*count;
        const auto spacing=static_cast<unsigned>(std::llround(supportPitch_/p.pixelPitchRad));
        const auto sizes=[&](unsigned dimension) {
            std::vector<unsigned> supportedSizes{1u,2u,3u,4u,6u,8u,12u,16u,24u,32u,48u,64u,96u,128u};
            const auto maximum=dimension/spacing;
            if(maximum&&std::find(supportedSizes.begin(),supportedSizes.end(),maximum)==supportedSizes.end())supportedSizes.push_back(maximum);
            std::sort(supportedSizes.begin(),supportedSizes.end());return supportedSizes;
        };
        const auto widths=sizes(p.width),heights=sizes(p.height);
        for(const unsigned width:widths)for(const unsigned height:heights) {
            if(width*supportPitch_>p.width*p.pixelPitchRad||height*supportPitch_>p.height*p.pixelPitchRad)continue;
            double scaleBest=-1;P4 scaleShape{};
            for(int row=-static_cast<int>(height);row<=static_cast<int>(supportRows_);++row)
                for(int col=-static_cast<int>(width);col<=static_cast<int>(supportColumns_);++col) {
                const double x=(col+.5*width-supportMargin_)*supportPitch_-.5*p.width*p.pixelPitchRad;
                const double y=(row+.5*height-supportMargin_)*supportPitch_-.5*p.height*p.pixelPitchRad;
                if(std::abs(x)>.5*p.width*p.pixelPitchRad||std::abs(y)>.5*p.height*p.pixelPitchRad)continue;
                // An unobserved source edge may lie outside the nine-sigma
                // optical support. Clip only this proposal integral, retaining
                // the FULL rectangle normalization and centre/extent. The
                // original spot() re-evaluation remains the final authority.
                const auto row0=static_cast<unsigned>(std::max(0,row));
                const auto row1=static_cast<unsigned>(std::min(static_cast<int>(supportRows_),row+static_cast<int>(height)));
                const auto col0=static_cast<unsigned>(std::max(0,col));
                const auto col1=static_cast<unsigned>(std::min(static_cast<int>(supportColumns_),col+static_cast<int>(width)));
                if(row1<=row0||col1<=col0)continue;
                const auto a=static_cast<std::size_t>(row0)*stride+static_cast<std::size_t>(col0)*count;
                const auto b=static_cast<std::size_t>(row0)*stride+static_cast<std::size_t>(col1)*count;
                const auto c0=static_cast<std::size_t>(row1)*stride+static_cast<std::size_t>(col0)*count;
                const auto d=static_cast<std::size_t>(row1)*stride+static_cast<std::size_t>(col1)*count;
                const double areaCells=static_cast<double>(width)*height;
                double correlation=0,norm2=0;
                for(unsigned j=0;j<count;++j) {
                    const double value=(supportPrefix_[d+j]-supportPrefix_[b+j]-supportPrefix_[c0+j]+supportPrefix_[a+j])/areaCells;
                    correlation+=data[j]*value;norm2+=value*value;
                }
                if(norm2<=128*std::numeric_limits<double>::epsilon())continue;
                const double explained=correlation*correlation/norm2;
                if(explained>proposed){proposed=explained;proposal={x,y,width*supportPitch_,height*supportPitch_};}
                if(explained>scaleBest){scaleBest=explained;scaleShape={{x,y,width*supportPitch_,height*supportPitch_}};}
            }
            if(scaleBest>0)supportSeeds.push_back({scaleBest,scaleShape});
        }
        if(proposed>best) {
            const auto exact=spot(proposal[0],proposal[1],p.psfSigmaRad,proposal[2],proposal[3]);
            consider(exact.data(),proposal[0],proposal[1],proposal[2],proposal[3]);refine(supportPitch_);
        }
    }
    const auto trustRefine=[&] {
        if(!(bestNorm>0))return;
        // Bounded, scaled damped Gauss-Newton with signed amplitude removed
        // analytically. Independent SciPy TRF fits of the actual frozen ADC
        // waveforms verify this forward model and its analytic Jacobian.
        // Every accepted candidate is projected BACK to the pre-existing
        // pitch/2^18 lattice, then judged by the original exact spot objective.
        const double lattice=p.pixelPitchRad/std::ldexp(1.,18),sigma=std::sqrt(variance);
        const double fullX=p.width*p.pixelPitchRad,fullY=p.height*p.pixelPitchRad;
        const P4 lower{{-.5*fullX,-.5*fullY,0,0}},upper{{.5*fullX,.5*fullY,fullX*fullX,fullY*fullY}};
        const double minimumDamping=64*std::numeric_limits<double>::epsilon();
        double damping=1e-3;
        for(unsigned iteration=0;iteration<64;++iteration) {
            const P4 current{{bestX,bestY,bestWidth*bestWidth,bestHeight*bestHeight}};
            const auto values=spot(bestX,bestY,p.psfSigmaRad,bestWidth,bestHeight);
            const double norm2=inner(values,values);if(!(norm2>0))break;
            const double amplitude=inner(values,data)/norm2;
            const auto derivative=spotJacobianV1(bestX,bestY,bestWidth,bestHeight);
            std::vector<double> residual(count);std::array<std::vector<double>,4> jacobian;
            for(unsigned j=0;j<count;++j)residual[j]=(data[j]-amplitude*values[j])/sigma;
            for(unsigned coordinate=0;coordinate<4;++coordinate) {
                const double amplitudeDerivative=(inner(data,derivative[coordinate])-2*amplitude*inner(values,derivative[coordinate]))/norm2;
                jacobian[coordinate].resize(count);
                for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=
                    (-amplitude*derivative[coordinate][j]-amplitudeDerivative*values[j])/sigma;
            }
            P4 low{},high{};for(unsigned coordinate=0;coordinate<4;++coordinate) {
                low[coordinate]=lower[coordinate]-current[coordinate];high[coordinate]=upper[coordinate]-current[coordinate];
            }
            bool accepted=false;
            for(unsigned trial=0;trial<12;++trial) {
                P4 delta{};if(!dampedBoxStep(jacobian,residual,low,high,damping,delta))break;
                P4 candidate{};for(unsigned coordinate=0;coordinate<4;++coordinate)
                    candidate[coordinate]=std::clamp(current[coordinate]+delta[coordinate],lower[coordinate],upper[coordinate]);
                P4 geometry{{candidate[0],candidate[1],std::sqrt(candidate[2]),std::sqrt(candidate[3])}};
                for(auto& value:geometry)value=std::round(value/lattice)*lattice;
                geometry[0]=std::clamp(geometry[0],lower[0],upper[0]);geometry[1]=std::clamp(geometry[1],lower[1],upper[1]);
                geometry[2]=std::clamp(geometry[2],0.,fullX);geometry[3]=std::clamp(geometry[3],0.,fullY);
                candidate={{geometry[0],geometry[1],geometry[2]*geometry[2],geometry[3]*geometry[3]}};
                if(candidate==current)break;
                const double old=best;
                const auto proposed=spot(geometry[0],geometry[1],p.psfSigmaRad,geometry[2],geometry[3]);
                consider(proposed.data(),geometry[0],geometry[1],geometry[2],geometry[3]);
                if(best>old) {
                    double predictedCost=0;
                    for(unsigned j=0;j<count;++j) {
                        double predicted=residual[j];for(unsigned coordinate=0;coordinate<4;++coordinate)
                            predicted+=jacobian[coordinate][j]*(candidate[coordinate]-current[coordinate]);
                        predictedCost+=predicted*predicted;
                    }
                    const double reduction=inner(residual,residual)-predictedCost;
                    const double ratio=reduction>0?(best-old)/(variance*reduction):0;
                    if(ratio>.75)damping=std::max(minimumDamping,damping/3);
                    else if(ratio<.25)damping=std::min(1/minimumDamping,damping*2);
                    step=lattice;accepted=true;break;
                }
                damping=std::min(1/minimumDamping,damping*10);
            }
            if(!accepted)break;
        }
    };
    if(std::max(0.,dataNorm-best)/variance>residualBound)trustRefine();
    const auto enforcePhysical=[&]() {
        if(!(bestNorm>0)){best=0;return false;}
        const P4 shape{{bestX,bestY,bestWidth,bestHeight}};
        const auto physical=physicalSourceAmplitudeV1(result,shape,bestAmplitude);
        if(!physical){best=0;return false;}
        const bool interior=*physical==bestAmplitude;
        if(!interior) {
            const auto wave=spot(bestX,bestY,p.psfSigmaRad,bestWidth,bestHeight);
            bestAmplitude=*physical;best=2*bestAmplitude*inner(data,wave)-bestAmplitude*bestAmplitude*inner(wave,wave);
        }
        return interior;
    };
    bool sourceInterior=enforcePhysical();
    if((!(best>0)||std::max(0.,dataNorm-best)/variance>residualBound)&&!supportSeeds.empty()) {
        // Each distinct support-scale minimum descends in LOCAL state. A
        // poorer coarse seed must not be prevented by another basin's score
        // from reaching a better physical solution. Numerical effort is
        // bounded; source geometry/labels never supply a seed.
        using State=std::array<double,8>;
        const auto save=[&](){return State{{best,bestX,bestY,bestWidth,bestHeight,bestAmplitude,bestNorm,step}};};
        const auto load=[&](const State& q){best=q[0];bestX=q[1];bestY=q[2];bestWidth=q[3];bestHeight=q[4];bestAmplitude=q[5];bestNorm=q[6];step=q[7];};
        auto selected=save();bool selectedInterior=sourceInterior;
        std::stable_sort(supportSeeds.begin(),supportSeeds.end(),[](const Seed& a,const Seed& b){return a.score>b.score;});
        const auto budget=std::min<std::size_t>(16,supportSeeds.size());
        for(std::size_t index=0;index<budget;++index) {
            const auto& q=supportSeeds[index].shape;
            best=0;bestNorm=0;
            const auto wave=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);consider(wave.data(),q[0],q[1],q[2],q[3]);
            if(!(bestNorm>0))continue;
            refine(supportPitch_);trustRefine();
            const bool interior=enforcePhysical();
            if(best>selected[0]){selected=save();selectedInterior=interior;}
            if(selected[0]>0&&std::max(0.,dataNorm-selected[0])/variance<=residualBound)break;
        }
        load(selected);sourceInterior=selectedInterior;
    }
    if(!(best>0)||!std::isfinite(best))return;
    result.fittedCentreXRad=bestX;result.fittedCentreYRad=bestY;
    result.fittedWidthRad=bestWidth;result.fittedHeightRad=bestHeight;
    result.fitSnr=std::sqrt(best/variance);result.fitResidual=std::max(0.,dataNorm-best)/variance;
    result.compactFitSnr=result.fitSnr;result.compactFitResidual=result.fitResidual;
    const auto forward=unit(pointing);auto right=unit(cross(forward,unit(up)));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    const auto emit=[&](double angularVariance,bool precise) {
        ThermalImageFeatureV1 feature;feature.direction=unit(add(forward,add(scale(right,std::tan(bestX)),scale(vertical,std::tan(bestY)))));
        feature.signalToNoise=result.fitSnr;feature.angularVarianceRad2=angularVariance;
        feature.signedContrastElectrons=bestAmplitude;
        feature.extent.widthRad=bestWidth;feature.extent.heightRad=bestHeight;
        feature.centroidColumn=(bestX/p.pixelPitchRad)+.5*p.width;
        feature.centroidRow=(bestY/p.pixelPitchRad)+.5*p.height;
        result.features.push_back(feature);result.bearingResolved=true;result.preciseBearing=precise;
    };
    // A singular local Fisher matrix supplies no local precision bound. A
    // provisional coarse direction uses the squared diameter of the ENTIRE
    // admitted optical field, not an invented width/quantization covariance.
    const double fieldRadius=std::atan(std::hypot(std::tan(.5*p.width*p.pixelPitchRad),std::tan(.5*p.height*p.pixelPitchRad)));
    const double coarseVariance=4*fieldRadius*fieldRadius;
    // A finite strip still provides centering in its compact axis even if
    // its long axis is ambiguous. Field-filling DC remains subject to the
    // same measured modulation/residual and physical background-null gates.
    const bool compactSource=bestWidth<.5*p.width*p.pixelPitchRad||bestHeight<.5*p.height*p.pixelPitchRad;
    // A compact source truncated by the optical stop can fit a wide support.
    // Its best centre remains near the stop; keep the coarse measurement there
    // rather than losing an already detected edge source.
    const bool nearStop=std::hypot(bestX,bestY)>=.75*c.stopRadiusRad;
    // Bound the ENTIRE fixed final angle/support lattice, not merely the
    // number of data-adaptively selected refinement queries. This keeps the
    // nominal Gaussian trial screen honest without changing detectionSigma.
    // Laurent-Massart upper tail for a Gaussian residual, disclosed as a
    // high-count engineering screen, not a calibrated equipment ROC.
    if(result.fitSnr<threshold||degrees<=0||bestNorm<=0||result.fitResidual>residualBound)return;
    const P4 fitted{{bestX,bestY,bestWidth,bestHeight}};
    const auto local=sourceInterior?localAngularVarianceV1(fitted,bestAmplitude,variance,step):std::nullopt;
    if(local) {
        const double lattice=p.pixelPitchRad/std::ldexp(1.,18);
        const auto ray=[&](const P4& q){return unit(add(forward,add(scale(right,std::tan(q[0])),scale(vertical,std::tan(q[1])))));};
        const auto direction=ray(fitted);
        std::vector<std::array<double,5>> modes{{{bestX,bestY,bestWidth,bestHeight,*local}}};
        bool invalidMode=false,profileUnresolved=false;
        const auto alternative=[&](P4 q,unsigned kind) {
            for(auto& value:q)value=std::round(value/lattice)*lattice;
            if(std::abs(q[0])>.5*p.width*p.pixelPitchRad||std::abs(q[1])>.5*p.height*p.pixelPitchRad||
               q[2]<0||q[3]<0||q[2]>p.width*p.pixelPitchRad||q[3]>p.height*p.pixelPitchRad)return;
            const auto other=ray(q);
            const double displacement=std::atan2(norm(cross(direction,other)),dot(direction,other));
            if(displacement<=lattice)return;
            const auto wave=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3]);
            const double squared=inner(wave,wave);if(squared<=128*std::numeric_limits<double>::epsilon())return;
            const double correlation=inner(data,wave),unconstrained=correlation/squared;
            const auto legal=physicalSourceAmplitudeV1(result,q,unconstrained);if(!legal)return;
            const double explained=2*(*legal)*correlation-(*legal)*(*legal)*squared;
            if(!std::isfinite(explained)||best-explained>threshold*threshold*variance)return;
            result.ambiguousAlternativeKind=kind;result.alternativeShape=q;result.alternativeAmplitude=*legal;
            result.alternativeCostDifference=(best-explained)/variance;
            if(const auto budget=rawSkyFitV1(result,spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],false))) {
                result.alternativeRadianceAvailable=true;
                result.alternativeSky=(*budget)[0]-(*budget)[1]*(*legal);
                const double cells=q[2]*q[3]/(p.pixelPitchRad*p.pixelPitchRad);
                result.alternativeSource=cells>0?result.alternativeSky+(*legal)/cells:*legal;
            }
            if(kind==2){profileUnresolved=true;return;}
            const auto modeVariance=*legal==unconstrained?localAngularVarianceV1(q,*legal,variance,lattice):std::nullopt;
            if(!modeVariance){invalidMode=true;return;}
            modes.push_back({{q[0],q[1],q[2],q[3],*modeVariance}});
        };
        // A small finite mirror cluster carries its actual mode displacement,
        // not the entire field. Every reflection is evaluated independently.
        for(const double sx:{-1.,1.})for(const double sy:{-1.,1.}) {
            if(sx==1&&sy==1)continue;
            auto q=fitted;q[0]*=sx;q[1]*=sy;alternative(q,1);
        }
        // Never certify optical-edge observability from detector rectangle
        // containment: an admitted physical stop can be much smaller.
        for(unsigned axis=0;axis<2;++axis) {
            const double full=(axis?p.height:p.width)*p.pixelPitchRad;
            for(const double sign:{-1.,1.}) {
                const double low=std::max(-.5*full-fitted[axis],sign>0?-.5*fitted[axis+2]:.5*(fitted[axis+2]-full));
                const double high=std::min(.5*full-fitted[axis],sign>0?.5*(full-fitted[axis+2]):.5*fitted[axis+2]);
                if(low>high)continue;
                for(const double delta:{low,high}) {
                    auto q=fitted;q[axis]+=delta;q[axis+2]+=2*sign*delta;alternative(q,2);
                }
            }
        }
        if(!invalidMode&&!profileUnresolved) {
            if(const auto envelope=finiteMirrorEnvelopeV1(c,modes)){emit(*envelope,true);return;}
        }
    }
    // The source cleared the detection and residual gates, but its local
    // angle/support Hessian is singular (edge truncation or unresolved extent).
    // A detected source must still produce a coarse measured bearing; its
    // uncertainty covers the entire admitted field, rather than claiming
    // precision from a singular or globally ambiguous local fit.
    if((compactSource||nearStop)&&std::isfinite(coarseVariance)&&coarseVariance>0){emit(coarseVariance,false);return;}
    return;
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldV1(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,bool* accelerated) {
    return unitGroundFieldMemoV2(p,pointing,up,cutoff,region,{},accelerated);
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldMemoV2(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,
    const std::function<const std::vector<double>*()>& nativeBackground,bool* accelerated) {
    return unitGroundFieldProfileV2(p,pointing,up,cutoff,region,nativeBackground,nullptr,accelerated);
}
std::optional<std::vector<double>> ThermalReticleReceiverV1::unitGroundFieldProfileV2(
    const ThermalImageConfigV1& p,V pointing,V up,double cutoff,
    const std::optional<std::array<double,4>>& region,
    const std::function<const std::vector<double>*()>& nativeBackground,ThermalGroundWorkV2* work,bool* accelerated) {
    if(work){*work={};work->fallbackReason=7;}
    UnitGroundClockV2 totalClock(work,false);
    std::unordered_set<std::uint64_t> diagnosticSamples;

    if(accelerated)*accelerated=false;
    if(!ThermalImageReceiverV1::validConfig(p)||!finite(pointing)||norm(pointing)<=0||
       !finite(up)||std::abs(norm(up)-1)>1e-6||!std::isfinite(cutoff)||std::abs(cutoff)>1)return std::nullopt;
    if(region&&(!std::all_of(region->begin(),region->end(),[](double v){return std::isfinite(v)&&std::abs(v)<=1.55;})||
        !((*region)[1]>(*region)[0])||!((*region)[3]>(*region)[2])))return std::nullopt;
    if(work)work->fallbackReason=0;
    const auto forward=unit(pointing),nativeUp=unit(up);
    auto right=unit(cross(forward,nativeUp));
    if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=unit(cross(right,forward));
    auto optical=p;optical.exposureSamples=1;
    const auto original=[&]()->std::optional<std::vector<double>> {
        UnitGroundClockV2 nativeClock(work,true);if(work)++work->originalCalls;
        ThermalSceneFrameV1 hypothesis;hypothesis.up=nativeUp;hypothesis.groundHorizonCosine=cutoff;hypothesis.groundRadianceWm2Sr=1;
        std::optional<ThermalIncidentFieldV1> ground;
        const std::vector<double>* native=nullptr;
        if(nativeBackground)native=nativeBackground();
        else {ground=ThermalImageReceiverV1::incidentFieldProfileV2(hypothesis,forward,optical,nullptr,work?&work->memo:nullptr);if(ground)native=&ground->radiance;}
        if(!native||native->size()!=static_cast<std::size_t>(p.width)*p.height)return std::nullopt;
        auto answer=*native;
        if(region) {
            ThermalRadiancePatchV1 black;black.direction=unit(add(forward,add(scale(right,std::tan(.5*((*region)[0]+(*region)[1]))),
                scale(vertical,std::tan(.5*((*region)[2]+(*region)[3]))))));
            black.widthRad=(*region)[1]-(*region)[0];black.heightRad=(*region)[3]-(*region)[2];black.rangeM=1;black.transmission=1;
            hypothesis.patches.push_back(black);hypothesis.refereeAttributionIndices.emplace_back(std::nullopt);
            const auto outside=ThermalImageReceiverV1::incidentFieldProfileV2(hypothesis,forward,optical,native,work?&work->memo:nullptr);
            if(!outside)return std::nullopt;
            for(std::size_t i=0;i<answer.size();++i)answer[i]-=outside->radiance[i];
        }
        return answer;
    };
    const auto fallback=[&](unsigned reason){if(work)work->fallbackReason=reason;return original();};
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
        std::nextafter(maxN*maxTy,std::numeric_limits<double>::infinity())))return fallback(1);
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
        if(touchesField&&(*region)[2]<=low+pitch&&(*region)[3]>=high-pitch)return fallback(2);
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
        if(!region&&(px0-margin<=-1.55||px1+margin>=1.55))return fallback(3);
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
        if(work){++work->activeColumns;work->activeRowVisits+=active;}
        const auto at=[&](double x) {
            if(work){++work->atEvaluations;std::uint64_t bits;std::memcpy(&bits,&x,sizeof(bits));if(!diagnosticSamples.insert(bits).second)++work->repeatedAtSamples;}
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
                if(work)++work->ruleEvaluations;
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
                for(unsigned row=0;row<p.height;++row)if(rows[row].active&&std::abs(fine[row]-coarse[row])>1e-9)return fallback(4);
            }
        } else {
            const auto rule=[&](double lo,double hi,const auto& nodes) {
                if(work)++work->ruleEvaluations;
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
                if(work){++work->adaptiveCalls;work->maxRefinementDepth=std::max(work->maxRefinementDepth,10-depth);}
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
            if(!converged)return fallback(5);
        }
        for(unsigned row=0;row<p.height;++row)if(rows[row].active) {
            if(!std::isfinite(fine[row])||fine[row]<0||fine[row]>1+1e-9)return fallback(6);
            answer[static_cast<std::size_t>(row)*p.width+col]=std::clamp(fine[row],0.,1.);
        }
    }
    if(accelerated)*accelerated=true;
    return answer;
}

void ThermalReticleReceiverV1::fitHorizon(ThermalReticleResultV1& result,V pointing,V up,bool* backgroundExplained,bool nullOnly) const {
    fitHorizonWithMeasuredPriorV2(result,pointing,up,backgroundExplained,nullOnly,nullptr);
}

void ThermalReticleReceiverV1::fitHorizonWithMeasuredPriorV2(ThermalReticleResultV1& result,V pointing,V up,
    bool* backgroundExplained,bool nullOnly,const ThermalReticleMeasuredPriorV2* prior) const {
    auto& observed=result.horizonDiagnostic;observed={};observed.ownUp=up;observed.refusal=1;
    if(backgroundExplained)*backgroundExplained=false;
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
    observed.refusal=2;
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
    observed.refusal=3;
    if(!(high>low))return;
    const double initialCutStep=(high-low)/p.height;
    std::optional<std::array<double,5>> priorStart;
    // A prior must already be available before any photon in this capture.
    // Domain checks use only existing instrument bounds and exact chronology.
    if(!nullOnly&&prior&&std::isfinite(prior->captureTimeS)&&prior->captureTimeS>=0&&
       std::isfinite(prior->availableTimeS)&&prior->availableTimeS>=prior->captureTimeS&&
       prior->captureTimeS<result.measurementTimeS&&prior->availableTimeS<=result.exposureStartTimeS&&
       finite(prior->direction)&&norm(prior->direction)>0&&finite(prior->directionRate)&&
       std::isfinite(prior->widthRad)&&prior->widthRad>0&&prior->widthRad<=2*halfX&&
       std::isfinite(prior->heightRad)&&prior->heightRad>0&&prior->heightRad<=2*halfY&&
       std::isfinite(prior->cutoff)&&prior->cutoff>=low&&prior->cutoff<=high) {
        const auto predicted=add(prior->direction,scale(prior->directionRate,result.measurementTimeS-prior->captureTimeS));
        if(finite(predicted)&&norm(predicted)>0) {
            const auto direction=unit(predicted);const double front=dot(forward,direction);
            const double x=std::atan2(dot(right,direction),front),y=std::atan2(dot(vertical,direction),front);
            if(front>0&&std::abs(x)<=halfX&&std::abs(y)<=halfY)
                priorStart=std::array<double,5>{{x,y,prior->widthRad,prior->heightRad,prior->cutoff}};
        }
    }
    // Count the EXISTING search, not a scene-tuned allowance. Adjacent equal
    // low samples can both qualify, but three consecutive samples cannot:
    // their middle would have no strict neighbour. Thus at most n-floor(n/3)
    // coarse entries qualify. Each receives36cutoff evaluations. Joint descent
    // adds at most108cutoffs per start (initial plus up to4 existing starts).
    // Selected curvature adds2; each snapshot cutoff adds at most its two
    // neighbouring derivative cutoffs, shared by all reflected source modes.
    constexpr std::size_t refinementLevels=18,coordinateSweeps=3,additionalJointStarts=4;
    constexpr std::size_t maximumJointStarts=1+additionalJointStarts;
    const std::size_t coarseCount=static_cast<std::size_t>(p.height)+1;
    const std::size_t maximumNullMinima=coarseCount-coarseCount/3;
    constexpr std::size_t trustIterations=64,trustTrials=12;
    const std::size_t oldCutoffBudget=coarseCount+2*refinementLevels*maximumNullMinima+
        2*refinementLevels*coordinateSweeps*maximumJointStarts;
    // Every trust iteration visits at most two new cutoff derivative points
    // and trustTrials proposed cutoffs. Other derivative axes reuse a cutoff.
    // A later finite snapshot tests every visited cutoff, including rejected
    // trust proposals. Each snapshot cutoff may add two derivative neighbours;
    // they are shared by its reflected modes and are not recursively profiled.
    // A valid measured start adds at most one previous cutoff plus the existing
    // null-cutoff bank. This is a derived work bound, never a new likelihood,
    // iteration, geometric or false-alarm allowance. No-prior count is exact old.
    const std::size_t priorTrustStarts=priorStart?maximumJointStarts+1:0;
    const std::size_t trustCutoffBudget=(maximumJointStarts+priorTrustStarts)*(1+trustIterations*(2+trustTrials));
    const std::size_t maximumCalibrations=3*(oldCutoffBudget+trustCutoffBudget+2);
    struct Calibration {
        double cutoff=0,cost=std::numeric_limits<double>::infinity(),amplitude=0;
        std::vector<double> wave,rawWave;
    };
    // All scenes below are INSTRUMENT HYPOTHESES: unit ground, zero sky,
    // unknown cutoff and optional black support. No actual scene radiance,
    // source rectangle, identity or horizon threshold reaches this inverse.
    std::optional<double> lastGroundCutoff;
    std::vector<double> lastGroundField;
    const auto groundField=[&](double cutoff)->const std::vector<double>* {
        if(lastGroundCutoff&&cutoff==*lastGroundCutoff&&std::signbit(cutoff)==std::signbit(*lastGroundCutoff))
            return &lastGroundField;
        auto field=unitGroundFieldV1(p,forward,verticalUp,cutoff);if(!field)return nullptr;
        lastGroundField=std::move(*field);lastGroundCutoff=cutoff;return &lastGroundField;
    };
    const auto waveform=[&](const std::vector<double>& field,bool removeNuisance) {
        std::vector<double> wave(count,0);
        for(std::size_t i=0;i<field.size();++i)for(unsigned j=0;j<count;++j)
            wave[j]+=field[i]*solidAngles_[i]*weights_[i*count+j]/pixelArea;
        wave=filtered(std::move(wave),alpha);if(removeNuisance)project(wave);return wave;
    };
    double nullExplained=0,nullCutoff=low,nullAmplitude=0,nullCost=std::numeric_limits<double>::infinity();
    std::vector<Calibration> calibrations;calibrations.reserve(maximumCalibrations);
    const auto calibration=[&](double cutoff)->const Calibration* {
        for(const auto& item:calibrations)if(item.cutoff==cutoff)return &item;
        if(calibrations.size()>=maximumCalibrations)return nullptr;
        const auto* field=groundField(cutoff);
        if(!field)return nullptr;
        Calibration item;item.cutoff=cutoff;item.rawWave=waveform(*field,false);
        item.wave=item.rawWave;project(item.wave);
        calibrations.push_back(std::move(item));auto& added=calibrations.back();
        // The null sees EVERY cutoff visited by any alternative, including
        // derivative and alias hypotheses. Improved background fitting can
        // therefore never be credited exclusively to the compact source.
        const double norm2=inner(added.wave,added.wave);
        if(norm2>128*std::numeric_limits<double>::epsilon()) {
            const double correlation=inner(data,added.wave);
            const auto budget=rawSkyFitV1(result,added.rawWave);
            if(!budget)return &added;
            const auto physical=physicalBackgroundAmplitudeV1((*budget)[0],(*budget)[1],correlation/norm2);
            if(!physical)return &added;
            const double amplitude=*physical;
            double cost=0;for(unsigned j=0;j<count;++j){const double residual=data[j]-amplitude*added.wave[j];cost+=residual*residual;}
            added.cost=cost;added.amplitude=amplitude;
            if(cost<nullCost){nullCost=cost;nullExplained=std::max(0.,dataNorm-cost);nullCutoff=cutoff;nullAmplitude=amplitude;}
        }
        return &added;
    };
    struct NullFit {double cutoff,cost,amplitude;};
    std::vector<NullFit> coarse,minima;
    for(unsigned row=0;row<=p.height;++row) {
        const auto* item=calibration(low+(high-low)*row/p.height);
        if(item)coarse.push_back({item->cutoff,item->cost,item->amplitude});
    }
    // A lower coarse sample does not identify the basin containing the best
    // continuous null. Refine EVERY strict measured coarse local minimum in
    // its own state. Flat, unmodulated plateaus cannot improve the null.
    bool nullSearchComplete=coarse.size()==static_cast<std::size_t>(p.height)+1;
    for(std::size_t index=0;index<coarse.size();++index) {
        const auto seed=coarse[index];
        const double left=index?coarse[index-1].cost:std::numeric_limits<double>::infinity();
        const double rightCost=index+1<coarse.size()?coarse[index+1].cost:std::numeric_limits<double>::infinity();
        if(!std::isfinite(seed.cost)||seed.cost>left||seed.cost>rightCost||
           !(seed.cost<left||seed.cost<rightCost))continue;
        auto local=seed;double cutStep=initialCutStep;
        for(std::size_t level=0;level<refinementLevels&&nullSearchComplete;++level) {
            cutStep*=.5;const double centre=local.cutoff;
            for(const double sign:{-1.,1.}) {
                const double cutoff=centre+sign*cutStep;
                if(cutoff<low||cutoff>high)continue;
                const auto* item=calibration(cutoff);
                if(!item){nullSearchComplete=false;break;}
                if(item->cost<local.cost)local={item->cutoff,item->cost,item->amplitude};
            }
        }
        minima.push_back(local);
        if(!nullSearchComplete)break;
    }
    std::stable_sort(minima.begin(),minima.end(),[](const NullFit& a,const NullFit& b){return a.cost<b.cost;});
    observed.nullCutoff=nullCutoff;observed.nullResidual=nullCost/variance;observed.refusal=4;
    if(!nullSearchComplete&&nullCost/variance>=threshold*threshold){observed.refusal=21;return;}
    if(nullExplained/variance<threshold*threshold)return;
    observed.refusal=5;
    if(nullCost/variance<threshold*threshold){if(backgroundExplained)*backgroundExplained=true;return;}
    // Exceptional exhaustion remains numerical unavailability, never proof
    // that a source-free null failed. Normal work is bounded above by the
    // admitted grid/refinement/start counts rather than an arbitrary512cap.
    observed.refusal=6;
    if(nullOnly)return;
    observed.refusal=7;
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
    struct ShapeWave {std::array<double,4> shape;bool projected;std::vector<double> wave;};
    bool cacheShapes=false;std::vector<ShapeWave> shapeWaves;shapeWaves.reserve(128);
    const auto sourceWave=[&](const Parameters& q,bool removeNuisance) {
        const std::array<double,4> shape{{q[0],q[1],q[2],q[3]}};
        if(cacheShapes)for(const auto& item:shapeWaves)
            if(item.shape==shape&&item.projected==removeNuisance)return item.wave;
        auto wave=spot(q[0],q[1],p.psfSigmaRad,q[2],q[3],removeNuisance);
        if(cacheShapes&&shapeWaves.size()<128)shapeWaves.push_back({shape,removeNuisance,wave});
        return wave;
    };
    std::optional<Parameters> lastOutsideShape;
    std::vector<double> lastOutsideRaw;
    const auto basis=[&](const Parameters& q,bool removeNuisance)->std::optional<std::array<std::vector<double>,2>> {
        const auto* ground=calibration(q[4]);if(!ground)return std::nullopt;
        bool sameOutside=lastOutsideShape.has_value();
        if(sameOutside)for(unsigned coordinate=0;coordinate<5;++coordinate) {
            if(q[coordinate]!=(*lastOutsideShape)[coordinate]||
               std::signbit(q[coordinate])!=std::signbit((*lastOutsideShape)[coordinate])){sameOutside=false;break;}
        }
        std::vector<double> outside;
        if(sameOutside)outside=lastOutsideRaw;
        else {
            if(q[2]>0&&q[3]>0) {
                const std::array<double,4> support{q[0]-.5*q[2],q[0]+.5*q[2],q[1]-.5*q[3],q[1]+.5*q[3]};
                const auto occulted=unitGroundFieldV1(p,forward,verticalUp,q[4],support);
                if(!occulted)return std::nullopt;
                if(std::all_of(occulted->begin(),occulted->end(),[](double value){return value==0;}))outside=ground->rawWave;
                else {
                    // Keep the original spatial subtraction and summation
                    // order. Memoize only its unprojected phase vector.
                    const auto* full=groundField(q[4]);if(!full)return std::nullopt;
                    auto field=*full;
                    for(std::size_t i=0;i<field.size();++i)field[i]-=(*occulted)[i];
                    outside=waveform(field,false);
                }
            } else outside=ground->rawWave;
            lastOutsideShape=q;lastOutsideRaw=outside;
        }
        if(removeNuisance)project(outside);
        return std::array<std::vector<double>,2>{std::move(outside),sourceWave(q,removeNuisance)};
    };
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
    observed.refusal=10;
    if(!(uu>0)||!(tt>=0))return;
    const bool hasTransient=tt>0;
    observed.refusal=11;
    if(hasTransient&&!(det>128*std::numeric_limits<double>::epsilon()*uu*tt))return;
    const auto physicalBudget=[&](const Parameters& q,double b,double d,std::array<double,3>* budget=nullptr) {
        const auto raw=basis(q,false);if(!raw)return false;
        auto remaining=result.measuredElectrons;
        for(unsigned j=0;j<count;++j)remaining[j]-=b*(*raw)[0][j]+d*(*raw)[1][j]+dark[j];
        const double sky=hasTransient?(inner(remaining,uniform)*tt-inner(remaining,transient)*ut)/det:inner(remaining,uniform)/uu;
        const double areaCells=q[2]*q[3]/pixelArea;
        const double ground=sky+b,radiator=areaCells>0?sky+d/areaCells:d;
        if(budget){*budget={sky,ground,radiator};observed.budgetAvailable=true;}
        return std::isfinite(sky)&&std::isfinite(ground)&&std::isfinite(radiator)&&sky>=0&&ground>=0&&radiator>=0;
    };
    double bestCost=std::numeric_limits<double>::infinity(),beta=0,contrast=0;
    Parameters bestQ{startX,startY,startWidth,startHeight,nullCutoff};
    const auto solve=[&](const Parameters& q,std::vector<double>* reduced=nullptr)->std::optional<std::array<double,3>> {
        if(std::abs(q[0])>halfX||std::abs(q[1])>halfY||q[2]<0||q[3]<0||q[2]>2*halfX||q[3]>2*halfY||q[4]<low||q[4]>high)return std::nullopt;
        const auto values=basis(q,true);if(!values)return std::nullopt;
        const auto& u=(*values)[0];const auto& v=(*values)[1];
        const double backgroundNorm=inner(u,u),uv=inner(u,v),vv=inner(v,v);
        const double determinant=backgroundNorm*vv-uv*uv;
        if(backgroundNorm<=0||vv<=0||determinant<=128*std::numeric_limits<double>::epsilon()*backgroundNorm*vv)return std::nullopt;
        const double yu=inner(data,u),yv=inner(data,v);
        const double b=(yu*vv-yv*uv)/determinant,d=(yv*backgroundNorm-yu*uv)/determinant;
        if(reduced)reduced->resize(count);
        double cost=0;for(unsigned j=0;j<count;++j) {
            const double residual=data[j]-b*u[j]-d*v[j];cost+=residual*residual;
            if(reduced)(*reduced)[j]=residual/std::sqrt(variance);
        }
        if(!std::isfinite(cost))return std::nullopt;
        return std::array<double,3>{cost,b,d};
    };
    const auto consider=[&](const Parameters& q) {
        const auto fit=solve(q);if(!fit)return;
        if((*fit)[0]<bestCost&&physicalBudget(q,(*fit)[1],(*fit)[2])) {
            bestCost=(*fit)[0];bestQ=q;beta=(*fit)[1];contrast=(*fit)[2];
        }
    };
    const double degrees=static_cast<double>(count)-static_cast<double>(nuisance_.size())-7,tail=.5*p.detectionSigma*p.detectionSigma;
    const double residualBound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    const auto descend=[&](const Parameters& initial) {
        bestQ=initial;bestCost=std::numeric_limits<double>::infinity();beta=0;contrast=0;
        consider(initial);
        double step=p.pixelPitchRad,cutStep=initialCutStep;
        for(std::size_t level=0;level<refinementLevels;++level) {
            step*=.5;cutStep*=.5;
            for(std::size_t sweep=0;sweep<coordinateSweeps;++sweep)for(unsigned coordinate=0;coordinate<5;++coordinate) {
                const auto centre=bestQ;
                for(const double sign:{-1.,1.}){auto q=centre;q[coordinate]+=sign*(coordinate==4?cutStep:step);consider(q);}
            }
        }
    };
    const auto start=bestQ;descend(start);
    struct JointFit {Parameters shape;double cost,background,source;};
    JointFit selected{bestQ,bestCost,beta,contrast};
    // Different measured null basins receive independent local descent.
    // Their source centre/support is still selected from measured residuals;
    // neither a target label nor its actual geometry supplies a start.
    const auto jointBudget=std::min(additionalJointStarts,minima.size());
    for(std::size_t index=0;index<jointBudget&&selected.cost/variance>residualBound;++index) {
        const auto& minimum=minima[index];if(minimum.cutoff==nullCutoff)continue;
        const auto* ground=calibration(minimum.cutoff);if(!ground)break;
        compactData=data;for(unsigned j=0;j<count;++j)compactData[j]-=minimum.amplitude*ground->wave[j];
        startExplained=0;startX=0;startY=0;startWidth=0;startHeight=0;
        for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
            const auto at=static_cast<std::size_t>(row)*p.width+col;
            considerStart(templates_.data()+at*count,(col+.5-.5*p.width)*p.pixelPitchRad,(row+.5-.5*p.height)*p.pixelPitchRad,0,0);
        }
        for(const double w:{0.,1.,2.,4.,8.,16.,32.})for(const double h:{0.,1.,2.,4.,8.,16.,32.}) {
            const auto wave=spot(startX,startY,p.psfSigmaRad,w*p.pixelPitchRad,h*p.pixelPitchRad);
            considerStart(wave.data(),startX,startY,w*p.pixelPitchRad,h*p.pixelPitchRad);
        }
        descend({startX,startY,startWidth,startHeight,minimum.cutoff});
        if(bestCost<selected.cost)selected={bestQ,bestCost,beta,contrast};
    }
    if(selected.cost/variance>residualBound) {
        // A single winning point correlation can select a horizon-shaped
        // basin whose unconstrained amplitudes are not physically feasible.
        // Refine the complete two-amplitude reduced residual in independent
        // LOCAL state; only physical candidates compete for selected output.
        const double angularLattice=p.pixelPitchRad/std::ldexp(1.,18);
        const double cutoffLattice=initialCutStep/std::ldexp(1.,18);
        const Parameters lower{{-halfX,-halfY,0,0,low}};
        const Parameters upper{{halfX,halfY,4*halfX*halfX,4*halfY*halfY,high}};
        const auto squared=[](Parameters q){q[2]*=q[2];q[3]*=q[3];return q;};
        const auto geometry=[](Parameters q){q[2]=std::sqrt(std::max(0.,q[2]));q[3]=std::sqrt(std::max(0.,q[3]));return q;};
        const auto quantize=[&](Parameters q) {
            q=geometry(q);
            for(unsigned coordinate=0;coordinate<4;++coordinate)q[coordinate]=std::round(q[coordinate]/angularLattice)*angularLattice;
            q[4]=low+std::round((q[4]-low)/cutoffLattice)*cutoffLattice;
            q[0]=std::clamp(q[0],-halfX,halfX);q[1]=std::clamp(q[1],-halfY,halfY);
            q[2]=std::clamp(q[2],0.,2*halfX);q[3]=std::clamp(q[3],0.,2*halfY);q[4]=std::clamp(q[4],low,high);
            return squared(q);
        };
        const auto record=[&](const Parameters& q,const std::array<double,3>& fit) {
            if(fit[0]<selected.cost&&physicalBudget(q,fit[1],fit[2]))selected={q,fit[0],fit[1],fit[2]};
        };
        const auto trust=[&](const Parameters& initial) {
            Parameters current=quantize(squared(initial));
            std::vector<double> residual;
            auto local=solve(geometry(current),&residual);if(!local)return;
            record(geometry(current),*local);
            const double minimumDamping=64*std::numeric_limits<double>::epsilon();
            double damping=1e-3;
            for(std::size_t iteration=0;iteration<trustIterations;++iteration) {
                std::array<std::vector<double>,5> jacobian;
                bool derivatives=true;
                for(unsigned coordinate=0;coordinate<5;++coordinate) {
                    const double difference=coordinate<2?p.pixelPitchRad/1024:
                        coordinate<4?p.pixelPitchRad*p.pixelPitchRad/1024:initialCutStep/1024;
                    auto plus=current,minus=current;
                    plus[coordinate]=std::min(upper[coordinate],current[coordinate]+difference);
                    minus[coordinate]=std::max(lower[coordinate],current[coordinate]-difference);
                    std::vector<double> a,b;
                    if(!(plus[coordinate]>minus[coordinate])||!solve(geometry(plus),&a)||!solve(geometry(minus),&b)) {
                        derivatives=false;break;
                    }
                    jacobian[coordinate].resize(count);
                    for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=(a[j]-b[j])/(plus[coordinate]-minus[coordinate]);
                }
                if(!derivatives)break;
                Parameters lowerStep{},upperStep{};
                for(unsigned coordinate=0;coordinate<5;++coordinate) {
                    lowerStep[coordinate]=lower[coordinate]-current[coordinate];upperStep[coordinate]=upper[coordinate]-current[coordinate];
                }
                bool accepted=false;
                for(std::size_t trial=0;trial<trustTrials;++trial) {
                    Parameters delta{};
                    if(!dampedBoxStep(jacobian,residual,lowerStep,upperStep,damping,delta))break;
                    auto proposed=current;
                    for(unsigned coordinate=0;coordinate<5;++coordinate)
                        proposed[coordinate]=std::clamp(current[coordinate]+delta[coordinate],lower[coordinate],upper[coordinate]);
                    proposed=quantize(proposed);if(proposed==current)break;
                    std::vector<double> candidateResidual;
                    const auto fit=solve(geometry(proposed),&candidateResidual);
                    if(fit)record(geometry(proposed),*fit);
                    if(fit&&(*fit)[0]<(*local)[0]) {
                        double predictedCost=0;
                        for(unsigned j=0;j<count;++j) {
                            double value=residual[j];
                            for(unsigned coordinate=0;coordinate<5;++coordinate)value+=jacobian[coordinate][j]*(proposed[coordinate]-current[coordinate]);
                            predictedCost+=value*value;
                        }
                        const double predictedReduction=inner(residual,residual)-predictedCost;
                        const double ratio=predictedReduction>0?((*local)[0]-(*fit)[0])/(variance*predictedReduction):0;
                        if(ratio>.75)damping=std::max(minimumDamping,damping/3);
                        else if(ratio<.25)damping=std::min(1/minimumDamping,damping*2);
                        current=proposed;local=fit;residual=std::move(candidateResidual);accepted=true;break;
                    }
                    damping=std::min(1/minimumDamping,damping*10);
                }
                if(!accepted)break;
            }
        };
        // Fixed instrument-centred compact proposals at measured null minima;
        // no source location, source size, label, geometry or RNG seed enters.
        std::vector<double> starts{nullCutoff};
        for(const auto& minimum:minima) {
            if(starts.size()>=maximumJointStarts)break;
            if(std::find(starts.begin(),starts.end(),minimum.cutoff)==starts.end())starts.push_back(minimum.cutoff);
        }
        for(const double cutoff:starts) {
            const double compact=2*p.pixelPitchRad;
            trust({{0,0,compact,compact,cutoff}});
            if(selected.cost/variance<=residualBound)break;
        }
        // Run every original cold-start decision first. Only a remaining failed
        // residual can request independent measured-history continuations.
        // These propose shapes; physical budgets, null updates, curvature,
        // finite mirrors, domain/profile screens and admission remain below.
        if(priorStart&&selected.cost/variance>residualBound) {
            auto priorCutoffs=starts;
            if(std::find(priorCutoffs.begin(),priorCutoffs.end(),(*priorStart)[4])==priorCutoffs.end())
                priorCutoffs.push_back((*priorStart)[4]);
            for(const double cutoff:priorCutoffs) {
                auto initial=*priorStart;initial[4]=cutoff;trust(initial);
                if(selected.cost/variance<=residualBound)break;
            }
        }
    }
    bestQ=selected.shape;bestCost=selected.cost;beta=selected.background;contrast=selected.source;
    const double step=p.pixelPitchRad/std::ldexp(1.,18);
    const double residual=bestCost/variance;
    double evidence=std::sqrt(std::max(0.,nullCost-bestCost)/variance);
    observed.shape=bestQ;observed.fitAvailable=std::isfinite(bestCost);observed.sourceResidual=residual;observed.nullResidual=nullCost/variance;observed.refusal=8;
    result.fitSnr=evidence;result.fitResidual=residual;
    if(evidence<threshold||degrees<=0||residual>residualBound)return;
    observed.refusal=9;
    const auto projected=basis(bestQ,true);if(!projected)return;
    observed.refusal=12;
    if(!physicalBudget(bestQ,beta,contrast,&observed.rawBudget))return;
    // Shape is fixed during background-mode/curvature checks. Reuse exactly
    // identical source waveforms; this changes no floating-point operation
    // within a waveform and bounds transient storage to128phase vectors.
    cacheShapes=true;
    const auto localVariance=[&](const Parameters& q,double background,double source,bool selectedMode)->std::optional<double> {
        const auto values=basis(q,true);if(!values)return std::nullopt;
        const auto prediction=[&](const Parameters& shape)->std::optional<std::vector<double>> {
            const auto wave=basis(shape,true);if(!wave)return std::nullopt;
            auto answer=(*wave)[0];
            for(unsigned j=0;j<count;++j)answer[j]=background*answer[j]+source*(*wave)[1][j];
            return answer;
        };
        std::vector<std::vector<double>> nuisance;
        const auto addColumn=[&](std::vector<double> column) {
            for(const auto& previous:nuisance) {
                const double coefficient=inner(column,previous);
                for(unsigned j=0;j<count;++j)column[j]-=coefficient*previous[j];
            }
            const double length=std::sqrt(inner(column,column));
            if(length<=128*std::numeric_limits<double>::epsilon())return false;
            for(auto& value:column)value/=length;
            nuisance.push_back(std::move(column));return true;
        };
        if(selectedMode)observed.refusal=13;
        if(!addColumn((*values)[0])||!addColumn((*values)[1]))return std::nullopt;
        const double derivativeStep=p.pixelPitchRad/1024,cutDerivative=initialCutStep/1024;
        const auto derivative=[&](unsigned coordinate)->std::optional<std::vector<double>> {
            auto plus=q,minus=q;const double h=coordinate==4?cutDerivative:derivativeStep;
            plus[coordinate]+=h;minus[coordinate]-=h;
            if(coordinate==4){plus[4]=std::min(high,plus[4]);minus[4]=std::max(low,minus[4]);}
            const auto a=prediction(plus),b=prediction(minus);
            if(!a||!b||!(plus[coordinate]>minus[coordinate]))return std::nullopt;
            auto answer=*a;
            for(unsigned j=0;j<count;++j)answer[j]=(answer[j]-(*b)[j])/(plus[coordinate]-minus[coordinate]);
            return answer;
        };
        if(selectedMode)observed.refusal=14;
        for(const unsigned coordinate:{2u,3u,4u}) {
            if(coordinate!=4&&q[coordinate]<=derivativeStep)continue;
            const auto column=derivative(coordinate);
            if(!column||!addColumn(*column))return std::nullopt;
        }
        if(selectedMode)observed.refusal=15;
        auto x=derivative(0),y=derivative(1);if(!x||!y)return std::nullopt;
        for(const auto& column:nuisance) {
            const double a=inner(*x,column),b=inner(*y,column);
            for(unsigned j=0;j<count;++j){(*x)[j]-=a*column[j];(*y)[j]-=b*column[j];}
        }
        const double xx=inner(*x,*x),yy=inner(*y,*y),xy=inner(*x,*y);
        const double determinant=xx*yy-xy*xy,trace=xx+yy;
        if(selectedMode) {
            observed.informationAvailable=true;observed.informationDeterminant=determinant;
            observed.informationTrace=trace;observed.refusal=16;
        }
        if(!std::isfinite(determinant)||xx<=0||yy<=0||
           determinant<=128*std::numeric_limits<double>::epsilon()*trace*trace)return std::nullopt;
        const double tx=std::tan(q[0]),ty=std::tan(q[1]),n2=1+tx*tx+ty*ty;
        const double gx=(1+tx*tx)*(1+tx*tx)*(1+ty*ty)/(n2*n2);
        const double gy=(1+ty*ty)*(1+ty*ty)*(1+tx*tx)/(n2*n2);
        const double gxy=-tx*ty*(1+tx*tx)*(1+ty*ty)/(n2*n2);
        const double metric=.5*(gx+gy+std::hypot(gx-gy,2*gxy));
        const double value=metric*variance*(trace+std::hypot(xx-yy,2*xy))/(2*determinant)+step*step/12;
        if(selectedMode){observed.localVariance=value;observed.refusal=17;}
        if(!std::isfinite(value)||value<=0)return std::nullopt;
        return value;
    };
    const auto ray=[&](const Parameters& q){return unit(add(forward,add(scale(right,std::tan(q[0])),scale(vertical,std::tan(q[1])))));};
    const auto direction=ray(bestQ);
    const auto selectedVariance=localVariance(bestQ,beta,contrast,true);
    std::vector<std::array<double,5>> modes;
    bool invalidMode=!selectedVariance||std::any_of(observed.rawBudget.begin(),observed.rawBudget.end(),[](double value){return value<=0;}),profileUnresolved=false;
    if(selectedVariance)modes.push_back({{bestQ[0],bestQ[1],bestQ[2],bestQ[3],*selectedVariance}});
    // Hold cutoff values, never iterators into the bounded calibration cache:
    // evaluating a mode's own derivatives can add calibration entries.
    std::vector<double> cutoffs;cutoffs.reserve(calibrations.size());
    for(const auto& entry:calibrations)cutoffs.push_back(entry.cutoff);
    observed.refusal=18;
    for(const double cutoff:cutoffs)for(const double sx:{-1.,1.})for(const double sy:{-1.,1.}) {
        if(sx==1&&sy==1&&cutoff==bestQ[4])continue;
        auto q=bestQ;q[0]*=sx;q[1]*=sy;q[4]=cutoff;
        const auto alternative=solve(q);
        if(!alternative||(*alternative)[0]>bestCost+threshold*threshold*variance)continue;
        std::array<double,3> budget{};
        if(!physicalBudget(q,(*alternative)[1],(*alternative)[2],&budget))continue;
        if(std::any_of(budget.begin(),budget.end(),[](double value){return value<=0;}))invalidMode=true;
        const auto other=ray(q);
        const double separation=std::atan2(norm(cross(direction,other)),dot(direction,other));
        if(!observed.alternativeAvailable) {
            observed.alternativeAvailable=true;observed.alternativeShape=q;
            observed.alternativeDelta=((*alternative)[0]-bestCost)/variance;observed.alternativeSeparation=separation;
        }
        const auto modeVariance=localVariance(q,(*alternative)[1],(*alternative)[2],false);
        if(!modeVariance){invalidMode=true;continue;}
        modes.push_back({{q[0],q[1],q[2],q[3],*modeVariance}});
    }
    // A finite mirror cluster is different from a continuation that moves an
    // unseen support edge. Such endpoint profiles retain only coarse authority.
    for(unsigned axis=0;axis<2;++axis) {
        const double full=(axis?p.height:p.width)*p.pixelPitchRad;
        for(const double sign:{-1.,1.}) {
            const double first=std::max(-.5*full-bestQ[axis],sign>0?-.5*bestQ[axis+2]:.5*(bestQ[axis+2]-full));
            const double last=std::min(.5*full-bestQ[axis],sign>0?.5*(full-bestQ[axis+2]):.5*bestQ[axis+2]);
            if(first>last)continue;
            for(const double displacement:{first,last}) {
                if(std::abs(displacement)<=step)continue;
                auto q=bestQ;q[axis]+=displacement;q[axis+2]+=2*sign*displacement;
                for(unsigned coordinate=0;coordinate<4;++coordinate)q[coordinate]=std::round(q[coordinate]/step)*step;
                q[0]=std::clamp(q[0],-halfX,halfX);q[1]=std::clamp(q[1],-halfY,halfY);
                q[2]=std::clamp(q[2],0.,2*halfX);q[3]=std::clamp(q[3],0.,2*halfY);
                const auto alternative=solve(q);
                if(alternative&&(*alternative)[0]<=bestCost+threshold*threshold*variance&&
                   physicalBudget(q,(*alternative)[1],(*alternative)[2]))profileUnresolved=true;
            }
        }
    }
    // Every additional calibration still competes as a physical null. Source
    // evidence cannot receive exclusive credit for a better background fit.
    observed.nullCutoff=nullCutoff;observed.nullResidual=nullCost/variance;
    if(nullCost/variance<threshold*threshold) {
        if(backgroundExplained)*backgroundExplained=true;
        observed.refusal=5;return;
    }
    evidence=std::sqrt(std::max(0.,nullCost-bestCost)/variance);result.fitSnr=evidence;
    observed.refusal=19;
    if(evidence<threshold)return;
    const auto envelope=!invalidMode&&!profileUnresolved?finiteMirrorEnvelopeV1(c,modes):std::nullopt;
    const bool compactAxis=bestQ[2]<halfX||bestQ[3]<halfY;
    const bool nearStop=std::hypot(bestQ[0],bestQ[1])>=.75*c.stopRadiusRad;
    if(!envelope&&!compactAxis&&!nearStop)return;
    ThermalImageFeatureV1 feature;feature.direction=direction;
    feature.extent.widthRad=bestQ[2];feature.extent.heightRad=bestQ[3];
    const double fieldRadius=std::atan(std::hypot(std::tan(halfX),std::tan(halfY)));
    feature.signalToNoise=evidence;feature.angularVarianceRad2=envelope?*envelope:4*fieldRadius*fieldRadius;
    feature.signedContrastElectrons=contrast;
    result.features.push_back(feature);result.bearingResolved=true;result.preciseBearing=envelope.has_value();observed.refusal=20;
}

}
