#include "MovingRadiatorInverseV2.h"
#include "simulation/mission_systems/components/support/SourceWaveSumV5.h"
#include "simulation/mission_systems/components/support/MovingConicMomentsV4.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cfenv>
#include <cstring>
#include <sstream>
#include <iomanip>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif

namespace fsim::ms::components::moving_radiator_v2 {
namespace {
using namespace measured;

struct LocalProfileV2 {
    using Clock=std::chrono::steady_clock;
    enum Part {FullGround,GroundWave,Lookup,Prepare,BasisWork,Regional,Qp,Trust,Information,ModeWork,SubsetWork,PartCount};
    bool enabled=false;Clock::time_point began{};
    std::array<std::size_t,PartCount> calls{};std::array<double,PartCount> micros{};
    std::size_t probes=0,hits=0,prepareHits=0,groundHits=0,faces=0,iterations=0,trials=0,modes=0,admittedModes=0;
    std::size_t subsetChecks=0,subsetRefusals=0,subsetFallbacks=0,phaseHits=0;
    std::size_t regionalAccelerated=0,regionalNative=0;double groundAttemptUs=0,groundOriginalUs=0;
    std::size_t conicRequests=0,conicBanks=0,conicFields=0,conicFallbacks=0;
    double conicSetupUs=0,conicRenderUs=0,conicMaximumError=0;
    unsigned conicVersion=4;
    std::array<std::size_t,8> fallbackReasons{};
    ThermalGroundWorkV2 groundWork;
    unsigned hotFields=0;std::array<unsigned,3> hotBucketCounts{};
    double readTime=0,captureTime=0;
    LocalProfileV2(){const auto* value=std::getenv("FSIM_MOVING_RETICLE_PROFILE");enabled=value&&value[0]=='1';if(enabled)began=Clock::now();}
    void checkpoint(const char* stage,double cost=std::numeric_limits<double>::quiet_NaN(),bool coarse=false) const {
        if(!enabled)return;
        const double us=std::chrono::duration<double,std::micro>(Clock::now()-began).count();
        char costJson[64];if(std::isfinite(cost))std::snprintf(costJson,sizeof(costJson),"%.17g",cost);else std::snprintf(costJson,sizeof(costJson),"null");
        std::fprintf(stderr,"MOVING_RETICLE_PROFILE {\"stage\":\"%s\",\"read\":%.17g,\"capture\":%.17g,\"elapsed_us\":%.3f,\"cost\":%s,\"coarse\":%d,\"basis_calls\":%zu,\"regions\":%zu,\"lookup_probes\":%zu,\"qp_calls\":%zu,\"qp_faces\":%zu,\"iterations\":%zu,\"trials\":%zu,\"modes\":%zu,\"admitted_modes\":%zu,\"ground_us\":%.3f,\"regional_us\":%.3f,\"qp_us\":%.3f,\"information_us\":%.3f,\"lookup_us\":%.3f,\"prepare_us\":%.3f,\"ground_hits\":%zu,\"prepare_hits\":%zu,\"subset_checks\":%zu,\"subset_refusals\":%zu,\"subset_fallbacks\":%zu,\"phase_hits\":%zu}\n",
            stage,readTime,captureTime,us,costJson,coarse?1:0,calls[BasisWork],calls[Regional],probes,calls[Qp],faces,iterations,trials,modes,admittedModes,
            micros[FullGround],micros[Regional],micros[Qp],micros[Information],micros[Lookup],micros[Prepare],groundHits,prepareHits,subsetChecks,subsetRefusals,subsetFallbacks,phaseHits);
        std::fprintf(stderr,"MOVING_GROUND_PROFILE {\"stage\":\"%s\",\"read\":%.17g,\"accelerated\":%zu,\"native\":%zu,\"attempt_us\":%.3f,\"original_us\":%.3f,\"at\":%zu,\"at_repeats\":%zu,\"rules\":%zu,\"recursions\":%zu,\"max_depth\":%u,\"active_rows\":%zu,\"memo_queries\":%zu,\"memo_hits\":%zu,\"memo_capacity_misses\":%zu,\"memo_max_nodes\":%zu,\"roots_built\":%zu,\"y_hits\":%zu,\"y_evaluations\":%zu,\"probability_hits\":%zu,\"probability_evaluations\":%zu,\"fallback_reasons\":[%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu]}\n",
            stage,readTime,regionalAccelerated,regionalNative,groundAttemptUs,groundOriginalUs,groundWork.atEvaluations,groundWork.repeatedAtSamples,
            groundWork.ruleEvaluations,groundWork.adaptiveCalls,groundWork.maxRefinementDepth,groundWork.activeRowVisits,
            groundWork.memo.queries,groundWork.memo.hits,groundWork.memo.capacityMisses,groundWork.memo.maxNodes,groundWork.memo.rootsBuilt,
            groundWork.memo.yHits,groundWork.memo.yEvaluations,groundWork.memo.probabilityHits,groundWork.memo.probabilityEvaluations,
            fallbackReasons[0],fallbackReasons[1],fallbackReasons[2],fallbackReasons[3],fallbackReasons[4],fallbackReasons[5],fallbackReasons[6],fallbackReasons[7]);
        std::fprintf(stderr,"MOVING_CONIC_V%u_PROFILE {\"stage\":\"%s\",\"read\":%.17g,\"requests\":%zu,\"banks\":%zu,\"fields\":%zu,\"fallbacks\":%zu,\"setup_us\":%.3f,\"render_us\":%.3f,\"maximum_error\":%.17g}\n",
            conicVersion,stage,readTime,conicRequests,conicBanks,conicFields,conicFallbacks,conicSetupUs,conicRenderUs,conicMaximumError);
        std::fflush(stderr);
    }
    void captureGround(const ThermalGroundWorkV2& work,bool accelerated,const ThermalImageConfigV1& policy,
        V pointing,V up,double cutoff,const std::array<double,4>& region,const std::vector<double>& field,
        const std::array<double,7>& q,unsigned branch,unsigned phase,unsigned innerSample) {
        // Observe at most three fields in each independent bucket: costly
        // accelerated calls, sensitive-edge native fallback (reason 2), and
        // regional-nonconvergence native fallback (reason 5). Only diagnostic
        // counters are changed; selection never controls an inverse branch.
        if(!enabled)return;
        unsigned bucket=0;
        if(accelerated) {
            if(work.atEvaluations<static_cast<std::size_t>(policy.width)*policy.height)return;
        } else if(work.originalCalls>0&&work.fallbackReason==2)bucket=1;
        else if(work.originalCalls>0&&work.fallbackReason==5)bucket=2;
        else return;
        if(hotBucketCounts[bucket]>=3)return;
        // Process-wide diagnostic budget, independent for all three roles.
        // These counters never publish simulation data or synchronize an
        // inverse/provider. Relaxed atomic tickets only bound stderr records.
        static std::atomic<unsigned> acceleratedCaptures{0};
        static std::atomic<unsigned> edgeCaptures{0};
        static std::atomic<unsigned> nonconvergenceCaptures{0};
        auto& processCounter=bucket==0?acceleratedCaptures:bucket==1?edgeCaptures:nonconvergenceCaptures;
        unsigned ticket=processCounter.load(std::memory_order_relaxed);
        while(ticket<3) {
            if(processCounter.compare_exchange_strong(ticket,ticket+1,std::memory_order_relaxed,std::memory_order_relaxed))break;
        }
        if(ticket>=3)return;
        // Strong CAS has no spurious failure. Each failed attempt observes a
        // strictly larger counter, so at most three attempts are possible.
        const unsigned processIndex=3*bucket+ticket+1;
        ++hotBucketCounts[bucket];++hotFields;
        std::ostringstream out;out<<std::setprecision(17);
        const auto bits=[&](double value){std::uint64_t raw;std::memcpy(&raw,&value,sizeof(raw));char text[17];std::snprintf(text,sizeof(text),"%016llx",static_cast<unsigned long long>(raw));out<<'"'<<text<<'"';};
        const auto bitsArray=[&](const auto& values){out<<'[';bool comma=false;for(const auto value:values){if(comma)out<<',';comma=true;bits(value);}out<<']';};
        out<<"MOVING_GROUND_CAPTURE {\"read\":"<<readTime<<",\"capture\":"<<captureTime<<",\"index\":"<<hotFields
           <<",\"accelerated\":"<<(accelerated?"true":"false")<<",\"fallback_reason\":"<<work.fallbackReason
           <<",\"bucket\":"<<bucket<<",\"bucket_index\":"<<hotBucketCounts[bucket]
           <<",\"process_index\":"<<processIndex<<",\"process_bucket_index\":"<<ticket+1
           <<",\"branch\":"<<branch<<",\"phase\":"<<phase<<",\"inner_sample\":"<<innerSample
           <<",\"total_us\":"<<work.totalMicros<<",\"original_us\":"<<work.originalMicros
           <<",\"at\":"<<work.atEvaluations<<",\"recursions\":"<<work.adaptiveCalls<<",\"max_depth\":"<<work.maxRefinementDepth;
        out<<",\"pointing_bits\":";bitsArray(std::array<double,3>{{pointing.x,pointing.y,pointing.z}});
        out<<",\"up_bits\":";bitsArray(std::array<double,3>{{up.x,up.y,up.z}});
        out<<",\"cutoff_bits\":";bits(cutoff);out<<",\"region_bits\":";bitsArray(region);
        out<<",\"hypothesis_bits\":";bitsArray(q);
        out<<",\"policy\":{\"version\":"<<policy.version<<",\"width\":"<<policy.width<<",\"height\":"<<policy.height
           <<",\"adcBits\":"<<policy.adcBits<<",\"exposureSamples\":"<<policy.exposureSamples
           <<",\"pixelPitchRad\":"<<policy.pixelPitchRad<<",\"psfSigmaRad\":"<<policy.psfSigmaRad
           <<",\"apertureAreaM2\":"<<policy.apertureAreaM2<<",\"opticalTransmission\":"<<policy.opticalTransmission
           <<",\"quantumEfficiency\":"<<policy.quantumEfficiency<<",\"effectiveWavelengthM\":"<<policy.effectiveWavelengthM
           <<",\"integrationTimeS\":"<<policy.integrationTimeS<<",\"pixelResponseTimeS\":"<<policy.pixelResponseTimeS
           <<",\"readNoiseElectrons\":"<<policy.readNoiseElectrons<<",\"darkCurrentElectronsS\":"<<policy.darkCurrentElectronsS
           <<",\"fullWellElectrons\":"<<policy.fullWellElectrons<<",\"detectionSigma\":"<<policy.detectionSigma
           <<",\"backgroundRadiusPixels\":"<<policy.backgroundRadiusPixels<<",\"minimumFeaturePixels\":"<<policy.minimumFeaturePixels
           <<",\"associationGateRad\":"<<policy.associationGateRad<<"},\"field_bits\":";
        bitsArray(field);out<<"}\n";const auto text=out.str();std::fwrite(text.data(),1,text.size(),stderr);std::fflush(stderr);
    }
    void addGround(const ThermalGroundWorkV2& work,bool accelerated) {
        if(!enabled)return;
        if(accelerated)++regionalAccelerated;else ++regionalNative;
        groundAttemptUs+=std::max(0.,work.totalMicros-work.originalMicros);groundOriginalUs+=work.originalMicros;
        ++fallbackReasons[std::min<std::size_t>(7,work.fallbackReason)];
        groundWork.activeColumns+=work.activeColumns;groundWork.activeRowVisits+=work.activeRowVisits;
        groundWork.atEvaluations+=work.atEvaluations;groundWork.ruleEvaluations+=work.ruleEvaluations;
        groundWork.adaptiveCalls+=work.adaptiveCalls;groundWork.repeatedAtSamples+=work.repeatedAtSamples;
        groundWork.maxRefinementDepth=std::max(groundWork.maxRefinementDepth,work.maxRefinementDepth);
        auto& a=groundWork.memo;const auto& b=work.memo;
        a.queries+=b.queries;a.hits+=b.hits;a.capacityMisses+=b.capacityMisses;a.maxNodes=std::max(a.maxNodes,b.maxNodes);
        a.rootsBuilt+=b.rootsBuilt;a.yHits+=b.yHits;a.yEvaluations+=b.yEvaluations;
        a.probabilityHits+=b.probabilityHits;a.probabilityEvaluations+=b.probabilityEvaluations;
    }
    ~LocalProfileV2(){if(!enabled)return;checkpoint("exit");for(unsigned i=0;i<PartCount;++i)std::fprintf(stderr,"MOVING_RETICLE_PART {\"part\":%u,\"read\":%.17g,\"calls\":%zu,\"us\":%.3f}\n",i,readTime,calls[i],micros[i]);std::fflush(stderr);}
};
struct LocalTimerV2 {
    LocalProfileV2* profile;LocalProfileV2::Part part;LocalProfileV2::Clock::time_point began{};
    LocalTimerV2(LocalProfileV2* target,LocalProfileV2::Part category):profile(target),part(category){if(profile&&profile->enabled){++profile->calls[part];began=LocalProfileV2::Clock::now();}}
    ~LocalTimerV2(){if(profile&&profile->enabled)profile->micros[part]+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-began).count();}
};

constexpr double infinity=std::numeric_limits<double>::infinity();
constexpr double epsilon=std::numeric_limits<double>::epsilon();
bool exactScalar(double a,double b) noexcept {return a==b&&std::signbit(a)==std::signbit(b);}
template<std::size_t N> bool exactCoordinates(const std::array<double,N>& a,const std::array<double,N>& b) noexcept {
    return std::equal(a.begin(),a.end(),b.begin(),exactScalar);
}
double inner(const std::vector<double>& a,const std::vector<double>& b) {
    return std::inner_product(a.begin(),a.end(),b.begin(),0.);
}
std::vector<double> filter(std::vector<double> values,double alpha) {
    double previous=0;for(auto& value:values){previous+=alpha*(value-previous);value=previous;}return values;
}
void project(std::vector<double>& values,const std::vector<std::vector<double>>& nuisance) {
    for(const auto& column:nuisance) {
        const double coefficient=inner(values,column);
        for(std::size_t j=0;j<values.size();++j)values[j]-=coefficient*column[j];
    }
}
V opticalRay(double x,double y) {return unit(V{1,std::tan(x),std::tan(y)});}
}

std::optional<RayPath> endpointPath(V first,V last,double firstRelativeReadS,
    double lastRelativeReadS,double integrationTimeS,unsigned branch) noexcept {
    if(!finite(first)||!finite(last)||!(norm(first)>0)||!(norm(last)>0)||
       !std::isfinite(firstRelativeReadS)||!std::isfinite(lastRelativeReadS)||
       !(lastRelativeReadS>firstRelativeReadS)||!std::isfinite(integrationTimeS)||integrationTimeS<=0||branch>1)return std::nullopt;
    first=unit(first);last=unit(last);
    if(first.x==last.x&&first.y==last.y&&first.z==last.z) {
        if(branch)return std::nullopt;
        RayPath stationary;stationary.readDirection=first;stationary.stationary=true;return stationary;
    }
    const double timeScale=std::max(std::abs(firstRelativeReadS),std::abs(lastRelativeReadS));
    if(!(timeScale>0))return std::nullopt;
    const double a=firstRelativeReadS/timeScale,c=lastRelativeReadS/timeScale;
    const double cosine=dot(first,last),b=-(a+c)*cosine;
    const double discriminant=b*b-4*a*c;
    // Negative/indeterminate discriminants are numerical unavailability, never
    // clamped into an invented finite-precision ray branch.
    if(!std::isfinite(discriminant)||discriminant<0)return std::nullopt;
    const double q=-.5*(b+std::copysign(std::sqrt(discriminant),b));
    std::array<double,2> ratios{{infinity,infinity}};
    if(a!=0&&q!=0)ratios={{q/a,c/q}};
    else if(a!=0)ratios[0]=-b/(2*a);
    else if(b!=0)ratios[0]=-c/b;
    std::array<RayPath,2> admitted{};unsigned count=0;
    for(unsigned index=0;index<2;++index) {
        const double ratio=ratios[index];if(!std::isfinite(ratio)||ratio<=0)continue;
        if(index&&ratio==ratios[0])continue;
        // Compute in dimensionless time: the tangent roundoff certificate
        // must not lose the inverse-duration factor for short exposures.
        const double interval=c-a;
        const auto read=scale(subtract(scale(first,c),scale(last,a*ratio)),1/interval);
        const auto rate=scale(subtract(scale(last,ratio),first),1/interval);
        const double length=norm(read);
        const double cancellation=(std::abs(c)+std::abs(a*ratio))/interval;
        if(!std::isfinite(length)||length<=128*epsilon*cancellation)continue;
        RayPath path;path.readDirection=scale(read,1/length);
        auto scaledRate=scale(rate,1/length);
        const double readScale=cancellation/length;
        const double rateScale=(std::abs(ratio)+1)/(interval*length);
        const double polynomial=(std::abs(a)*ratio*ratio+std::abs(a+c)*std::abs(ratio)+std::abs(c))/
            (interval*interval*length*length);
        constexpr double gamma=256*epsilon/(1-256*epsilon);
        const double radial=dot(path.readDirection,scaledRate);
        const double radialBound=gamma*(polynomial+readScale*rateScale);
        if(!std::isfinite(radial)||!std::isfinite(radialBound)||std::abs(radial)>radialBound)continue;
        // Orthogonal projection removes only the certified arithmetic radial
        // component; it cannot repair an inconsistent endpoint polynomial.
        scaledRate=subtract(scaledRate,scale(path.readDirection,radial));
        path.readDerivative=scale(scaledRate,1/timeScale);
        if(!finite(path.readDirection)||!finite(path.readDerivative))continue;
        const double rateError=(gamma*(rateScale+norm(scaledRate)*readScale)+std::abs(radial))/timeScale;
        const double dimensionless=norm(path.readDerivative)*integrationTimeS;
        const double rateRadius=rateError*integrationTimeS;
        const double originalDomainRoundoff=128*epsilon*std::max(.5,dimensionless);
        if(!std::isfinite(dimensionless)||!std::isfinite(rateRadius)||dimensionless>.5+originalDomainRoundoff)continue;
        const double minimumNorm=std::min(1.,ratio)/length*std::sqrt(std::max(0.,.5*(1+cosine)));
        const double endpointError=gamma*(readScale+std::max(std::abs(a),std::abs(c))*rateScale)+
            std::abs(radial)*std::max(std::abs(a),std::abs(c));
        const double relativeError=endpointError/minimumNorm;
        if(!(relativeError>=0&&relativeError<1))continue;
        path.endpointRoundoffRad=std::asin(relativeError);
        path.domainBoundary=dimensionless+rateRadius>=.5||
            discriminant<=128*epsilon*(b*b+std::abs(4*a*c));
        path.stationary=norm(path.readDerivative)==0;
        admitted[count++]=path;
    }
    if(count==2&&norm(admitted[1].readDerivative)<norm(admitted[0].readDerivative))std::swap(admitted[0],admitted[1]);
    if(branch>=count)return std::nullopt;
    admitted[branch].branch=branch;return admitted[branch];
}
V directionAt(const RayPath& path,double relativeReadS) noexcept {
    return unit(add(path.readDirection,scale(path.readDerivative,relativeReadS)));
}
V derivativeAt(const RayPath& path,double relativeReadS) noexcept {
    const auto raw=add(path.readDirection,scale(path.readDerivative,relativeReadS));const double length=norm(raw);
    if(!(length>0))return {};
    const auto direction=scale(raw,1/length);
    return scale(subtract(path.readDerivative,scale(direction,dot(direction,path.readDerivative))),1/length);
}

namespace {
bool boxStep(const std::array<std::vector<double>,7>& jacobian,const std::vector<double>& residual,
    const Shape& low,const Shape& high,double damping,Shape& result,LocalProfileV2* profile=nullptr) {
    LocalTimerV2 timer(profile,LocalProfileV2::Qp);
    constexpr unsigned faceCount=2187;
    const auto count=residual.size(),rows=count+7;
    if(!(damping>0)||!std::isfinite(damping))return false;
    Shape norms{};for(unsigned column=0;column<7;++column) {
        if(jacobian[column].size()!=count)return false;
        for(const auto value:jacobian[column])norms[column]=std::hypot(norms[column],value);
        if(!std::isfinite(norms[column]))return false;
        if(norms[column]==0)norms[column]=1;
    }
    std::vector<Shape> design(rows);std::vector<double> target(rows);
    const double regularizer=std::sqrt(damping);
    for(std::size_t row=0;row<count;++row) {
        target[row]=-residual[row];if(!std::isfinite(target[row]))return false;
        for(unsigned column=0;column<7;++column)design[row][column]=jacobian[column][row]/norms[column];
    }
    for(unsigned column=0;column<7;++column)design[count+column][column]=regularizer;
    Shape scaledLow{},scaledHigh{};
    for(unsigned axis=0;axis<7;++axis) {
        scaledLow[axis]=low[axis]*norms[axis];scaledHigh[axis]=high[axis]*norms[axis];
        if(!std::isfinite(scaledLow[axis])||!std::isfinite(scaledHigh[axis])||scaledLow[axis]>scaledHigh[axis])return false;
    }
    double bestCost=std::numeric_limits<double>::infinity();Shape selected{};
    for(unsigned face=0;face<faceCount;++face) {
        if(profile&&profile->enabled)++profile->faces;
        unsigned encoded=face,freeCount=0;std::array<unsigned,7> free{};Shape step{};
        auto rhs=target;
        for(unsigned axis=0;axis<7;++axis) {
            const auto state=encoded%3;encoded/=3;
            if(state==0)free[freeCount++]=axis;
            else {
                step[axis]=state==1?scaledLow[axis]:scaledHigh[axis];
                for(std::size_t row=0;row<rows;++row)rhs[row]-=design[row][axis]*step[axis];
            }
        }
        std::vector<Shape> matrix(rows);
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
        Shape solved{};
        for(unsigned reverse=freeCount;reverse>0;--reverse) {
            const auto row=reverse-1;double value=rhs[row];
            for(unsigned column=row+1;column<freeCount;++column)value-=matrix[row][column]*solved[column];
            solved[row]=value/matrix[row][row];if(!std::isfinite(solved[row])){valid=false;break;}
        }
        if(!valid)continue;
        for(unsigned column=0;column<freeCount;++column)step[free[column]]=solved[column];
        for(unsigned axis=0;axis<7;++axis) {
            const double tolerance=256*std::numeric_limits<double>::epsilon()*
                std::max({1.,std::abs(scaledLow[axis]),std::abs(scaledHigh[axis])});
            if(step[axis]<scaledLow[axis]-tolerance||step[axis]>scaledHigh[axis]+tolerance){valid=false;break;}
            step[axis]=std::clamp(step[axis],scaledLow[axis],scaledHigh[axis]);
        }
        if(!valid)continue;
        // The feasible unconstrained minimizer is also the box minimizer.
        // This exact quadratic fact avoids enumerating inactive faces.
        if(face==0){for(unsigned axis=0;axis<7;++axis)result[axis]=step[axis]/norms[axis];return true;}
        double cost=0;
        for(std::size_t row=0;row<rows;++row) {
            double error=-target[row];for(unsigned column=0;column<7;++column)error+=design[row][column]*step[column];
            cost+=error*error;
        }
        if(std::isfinite(cost)&&cost<bestCost){bestCost=cost;selected=step;}
    }
    if(!std::isfinite(bestCost))return false;
    for(unsigned axis=0;axis<7;++axis)result[axis]=selected[axis]/norms[axis];
    return true;
}
// Finite-arithmetic certificate only. A false/uncertain certificate always
// falls back to the full native likelihood; it never changes that likelihood.
bool certifiedArithmeticV2() noexcept {
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__>0)
    return false;
#endif
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    // Actual runtime state, not a constant-foldable subnormal probe:
    // FTZ, DAZ, and non-nearest SSE rounding invalidate the tiny/gamma model.
    if((_mm_getcsr()&0xe040u)!=0)return false;
#else
    return false;
#endif
    if(std::fegetround()!=FE_TONEAREST)return false;
    return std::numeric_limits<double>::is_iec559&&std::numeric_limits<long double>::digits>=std::numeric_limits<double>::digits&&
        std::numeric_limits<long double>::min_exponent<2*(std::numeric_limits<double>::min_exponent-std::numeric_limits<double>::digits)&&
        std::numeric_limits<long double>::max_exponent>2*std::numeric_limits<double>::max_exponent;
}
double roundGamma(std::size_t operations) {
    if(operations>9007199254740992ull)return infinity;
    const double value=static_cast<double>(operations)*epsilon;
    return value<.5?value/(1-value):infinity;
}
double upperPositive(long double value,std::size_t operations=8) {
    if(!(value>=0)||!std::isfinite(value))return infinity;
    const auto gamma=roundGamma(operations);if(!std::isfinite(gamma)||!(gamma<1))return infinity;
    const long double bounded=value/(1-gamma);
    return std::nextafter(static_cast<double>(bounded),infinity);
}
double vectorUpper(const std::vector<double>& values) {
    if(!certifiedArithmeticV2())return infinity;
    long double square=0;for(const auto x:values){if(!std::isfinite(x))return infinity;square+=static_cast<long double>(x)*x;}
    return upperPositive(std::sqrt(square),2*values.size()+8);
}
double absoluteDotUpper(const std::vector<double>& a,const std::vector<double>& b) {
    if(!certifiedArithmeticV2())return infinity;
    long double sum=0,absolute=0;
    for(std::size_t row=0;row<a.size();++row){const auto term=static_cast<long double>(a[row])*b[row];sum+=term;absolute+=std::abs(term);}
    return upperPositive(std::abs(sum)+roundGamma(2*a.size()+4)*absolute);
}
double absoluteDotLower(const std::vector<double>& a,const std::vector<double>& b) {
    if(!certifiedArithmeticV2())return 0;
    long double sum=0,absolute=0;
    for(std::size_t row=0;row<a.size();++row){const auto term=static_cast<long double>(a[row])*b[row];sum+=term;absolute+=std::abs(term);}
    const auto lower=std::abs(sum)-roundGamma(2*a.size()+4)*absolute;
    return lower>0?std::max(0.,std::nextafter(static_cast<double>(lower),-infinity)):0;
}
struct SubsetMapV2 {
    std::vector<unsigned> phases;
    std::vector<std::vector<double>> matrix;
    std::vector<double> target;
    double operatorBound=infinity,coefficientLeak=infinity;
    double matrixNorm=infinity,targetError=infinity;
    bool valid=false;
};
// All nuisance columns remain in this bound, including columns with zero or
// nearly dependent transformed residual. No numerical-rank truncation occurs.
SubsetMapV2 makeSubsetMap(unsigned stride,double beta,const std::vector<double>& data,
    const std::vector<std::vector<double>>& nuisance,double rawBackgroundNormBound) {
    SubsetMapV2 answer;
    const auto count=data.size();
    if(!certifiedArithmeticV2()||!(beta>=0&&beta<=1)||stride<2)return answer;
    for(unsigned phase=stride-1;phase<count;phase+=stride)answer.phases.push_back(phase);
    const auto rows=answer.phases.size();if(rows<3)return answer;
    const auto transformed=[&](const std::vector<double>& values) {
        std::vector<double> result(rows);
        for(std::size_t row=0;row<rows;++row){const auto j=answer.phases[row];result[row]=std::fma(-beta,values[j-1],values[j]);}
        return result;
    };
    std::vector<std::vector<double>> allNuisance,orthogonal;
    double intermediateBound=rawBackgroundNormBound;
    const double nativeTiny=std::numeric_limits<double>::denorm_min();
    std::vector<double> coefficientBounds;
    for(const auto& column:nuisance) {
        const double lengthBound=vectorUpper(column);
        if(!(lengthBound*lengthBound<=2))return answer;
        coefficientBounds.push_back(upperPositive((1+roundGamma(count+2))*lengthBound*intermediateBound+static_cast<double>(count)*nativeTiny));
        intermediateBound=upperPositive(intermediateBound*(1+4*roundGamma(count+6))+4*static_cast<double>(count)*nativeTiny);
        auto value=transformed(column);allNuisance.push_back(value);
        for(unsigned pass=0;pass<2;++pass)for(const auto& previous:orthogonal) {
            const auto a=inner(value,previous);for(std::size_t row=0;row<rows;++row)value[row]-=a*previous[row];
        }
        double length=0;for(const auto x:value)length=std::hypot(length,x);
        // This basis is only a chosen contraction, not an asserted nuisance
        // span. Even an exactly zero remainder stays in allNuisance/leak.
        if(length>0&&std::isfinite(length)) {
            for(auto& x:value)x/=length;
            if(std::all_of(value.begin(),value.end(),[](double x){return std::isfinite(x);}))orthogonal.push_back(std::move(value));
        }
    }
    std::vector<double> gramErrors;
    for(std::size_t i=0;i<orthogonal.size();++i)for(std::size_t j=0;j<orthogonal.size();++j) {
        long double sum=0,absolute=0;
        for(std::size_t row=0;row<rows;++row){const auto term=static_cast<long double>(orthogonal[i][row])*orthogonal[j][row];sum+=term;absolute+=std::abs(term);}
        gramErrors.push_back(upperPositive(std::abs(sum-(i==j?1:0))+roundGamma(2*rows+4)*absolute));
    }
    double theta=vectorUpper(gramErrors);
    // A poorly conditioned chosen basis is harmless: choose identity and
    // retain the resulting (usually unhelpful) full nuisance leakage bound.
    if(!(theta<1)){orthogonal.clear();theta=0;}
    answer.matrix.assign(rows,std::vector<double>(rows));std::vector<double> formationErrors;
    for(std::size_t row=0;row<rows;++row)for(std::size_t col=0;col<rows;++col) {
        double value=row==col?1:0;long double absolute=std::abs(value);
        for(const auto& q:orthogonal){value-=q[row]*q[col];absolute+=std::abs(static_cast<long double>(q[row])*q[col]);}
        answer.matrix[row][col]=value;
        formationErrors.push_back(upperPositive(roundGamma(2*orthogonal.size()+4)*absolute));
    }
    answer.operatorBound=upperPositive(std::max(1.,theta)+vectorUpper(formationErrors));
    std::vector<double> entries;for(const auto& row:answer.matrix)entries.insert(entries.end(),row.begin(),row.end());
    answer.matrixNorm=vectorUpper(entries);
    const auto apply=[&](const std::vector<double>& values) {
        std::vector<double> result(rows,0);for(std::size_t row=0;row<rows;++row)for(std::size_t col=0;col<rows;++col)result[row]+=answer.matrix[row][col]*values[col];
        return result;
    };
    long double leak=0;
    const double tiny=std::numeric_limits<double>::denorm_min();
    for(std::size_t i=0;i<allNuisance.size();++i) {
        const auto mapped=apply(allNuisance[i]);
        const double inputError=upperPositive(roundGamma(2)*vectorUpper(allNuisance[i])+static_cast<double>(rows)*tiny);
        const double applyError=upperPositive(roundGamma(2*rows+4)*answer.matrixNorm*vectorUpper(allNuisance[i])+static_cast<double>(rows*rows)*tiny);
        const double coupling=upperPositive(vectorUpper(mapped)+answer.operatorBound*inputError+applyError);
        leak+=static_cast<long double>(coupling)*coefficientBounds[i];
    }
    answer.coefficientLeak=upperPositive(leak);
    const auto changed=transformed(data);answer.target=apply(changed);
    const double lop=upperPositive(std::sqrt(1+static_cast<long double>(beta)*beta));
    const double targetInputError=upperPositive(roundGamma(2)*vectorUpper(changed)+static_cast<double>(rows)*tiny);
    answer.targetError=upperPositive(answer.operatorBound*(targetInputError+lop*roundGamma(5)*vectorUpper(data))+
        roundGamma(2*rows+4)*answer.matrixNorm*vectorUpper(changed)+static_cast<double>(rows*rows)*tiny+answer.operatorBound*lop*4*static_cast<double>(count)*tiny);
    answer.valid=std::isfinite(answer.operatorBound)&&std::isfinite(answer.coefficientLeak)&&
        std::isfinite(answer.targetError);
    return answer;
}

// A posteriori QR/inverse-residual certificate for TWO uncertain radiator
// columns. No normal-equation solve and no approximate-rank deletion.
bool certifiedTwoColumnRefusal(const std::vector<double>& y,std::array<std::vector<double>,2> columns,
    double targetError,std::array<double,2> columnErrors,double allowedNorm) {
    const auto rows=y.size();if(rows<3||columns[0].size()!=rows||columns[1].size()!=rows)return false;
    if(!(targetError>=0)||!std::isfinite(targetError)||std::any_of(columnErrors.begin(),columnErrors.end(),[](double x){return !(x>=0)||!std::isfinite(x);}))return false;
    if(!certifiedArithmeticV2()||!(allowedNorm>=0)||!std::isfinite(allowedNorm))return false;
    std::array<std::vector<double>,2> a=columns,q;
    std::array<std::array<double,2>,2> r{};
    std::vector<double> errors;
    for(unsigned i=0;i<2;++i) {
        const double scale=1/vectorUpper(a[i]);if(!(scale>0)||!std::isfinite(scale))return false;
        for(auto& x:a[i])x*=scale;
        errors.push_back(upperPositive(columnErrors[i]*scale+roundGamma(2)*vectorUpper(a[i])));
        q[i]=a[i];
        for(unsigned pass=0;pass<2;++pass)for(unsigned previous=0;previous<i;++previous) {
            const double coefficient=inner(q[i],q[previous]);r[previous][i]+=coefficient;
            for(std::size_t row=0;row<rows;++row)q[i][row]-=coefficient*q[previous][row];
        }
        double length=0;for(const auto x:q[i])length=std::hypot(length,x);
        if(!(length>0)||!std::isfinite(length))return false;
        r[i][i]=length;for(auto& x:q[i])x/=length;
    }
    std::vector<double> gramErrors,reconstruction;
    for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j) {
        long double sum=0,absolute=0;for(std::size_t row=0;row<rows;++row){const auto term=static_cast<long double>(q[i][row])*q[j][row];sum+=term;absolute+=std::abs(term);}
        gramErrors.push_back(upperPositive(std::abs(sum-(i==j?1:0))+roundGamma(2*rows+4)*absolute));
    }
    for(unsigned i=0;i<2;++i)for(std::size_t row=0;row<rows;++row) {
        long double value=a[i][row],absolute=std::abs(value);
        for(unsigned j=0;j<2;++j){const auto term=static_cast<long double>(q[j][row])*r[j][i];value-=term;absolute+=std::abs(term);}
        reconstruction.push_back(upperPositive(std::abs(value)+roundGamma(8)*absolute));
    }
    const double theta=vectorUpper(gramErrors),chi=vectorUpper(reconstruction);
    if(!(theta<1))return false;
    const std::array<std::array<double,2>,2> inverse{{{{1/r[0][0],-(r[0][1]/r[0][0])/r[1][1]}},{{0,1/r[1][1]}}}};
    std::vector<double> inverseEntries,inverseResidual;
    for(unsigned i=0;i<2;++i)for(unsigned j=0;j<2;++j) {
        inverseEntries.push_back(inverse[i][j]);
        long double sum=0,absolute=0;for(unsigned k=0;k<2;++k){const auto term=static_cast<long double>(r[i][k])*inverse[k][j];sum+=term;absolute+=std::abs(term);}
        inverseResidual.push_back(upperPositive(std::abs(sum-(i==j?1:0))+roundGamma(8)*absolute));
    }
    const double delta=vectorUpper(inverseResidual),inverseNorm=vectorUpper(inverseEntries);
    if(!(delta<1)||!(inverseNorm>0)||!std::isfinite(inverseNorm))return false;
    const long double approximate=std::sqrt(1-static_cast<long double>(theta))*(1-delta)/inverseNorm;
    const double singularLower=std::nextafter(static_cast<double>(approximate-chi-roundGamma(16)*std::abs(approximate)),-infinity);
    const double eta=vectorUpper(errors);
    if(!(singularLower>eta))return false;
    auto witness=y;
    for(const auto& direction:q) {
        const auto coefficient=inner(y,direction);
        for(std::size_t row=0;row<rows;++row)witness[row]-=coefficient*direction[row];
    }
    const double initialNorm=vectorUpper(witness);if(!(initialNorm>0)||!std::isfinite(initialNorm))return false;
    for(auto& x:witness)x/=initialNorm;
    const double witnessNorm=vectorUpper(witness),left=absoluteDotLower(witness,y);
    const std::vector<double> leakage{absoluteDotUpper(witness,a[0]),absoluteDotUpper(witness,a[1])};
    const double yNorm=vectorUpper(y),coefficientNorm=upperPositive((static_cast<long double>(yNorm)+targetError+allowedNorm)/(singularLower-eta));
    const long double right=static_cast<long double>(witnessNorm)*(allowedNorm+targetError)+
        (vectorUpper(leakage)+witnessNorm*eta)*coefficientNorm;
    return std::isfinite(left)&&left>upperPositive(right,64);
}

struct Basis {std::array<std::vector<double>,2> raw,projected;bool sourceRelativeGround=false;};
struct Fit {double cost=infinity,background=0,source=0;};
struct SourceWave {std::array<double,6> shape{};unsigned branch=0;std::vector<double> raw;};
enum class RegionalOperator {LegacyV2,ConicV4,ConicFastV5};

class Inverse {
public:
    const Optics& optics;const ThermalReticleConfigV1& c;const ThermalImageConfigV1& p;
    const ThermalReticleResultV1& measured;ThermalReticleMovingDiagnosticV2 diagnostic;
    const RegionalOperator regionalOperator;
    mutable LocalProfileV2 profile;
    V forward,right,vertical,up;
    unsigned count;std::size_t pixels;
    double alpha,pixelArea,variance,halfX,halfY,low,high,cutStep,angularLattice,cutLattice,first,last;
    std::vector<double> data,uniform,transient,dark;
    double dataNorm,ut,tt,uu,det;
    struct Ground {double cutoff=0,cost=infinity,amplitude=0;std::vector<double> raw,wave;};
    std::vector<Ground> grounds;
    std::optional<double> lastGroundCutoff;std::vector<double> lastGroundField;
    std::optional<double> nativeGroundCutoff;std::vector<double> nativeGroundField;
    std::optional<Shape> lastShape;unsigned lastBranch=0;std::optional<Basis> lastBasis;
    std::optional<Shape> regionalShape;unsigned regionalBranch=0;
    std::vector<double> regionalPhases;std::vector<bool> regionalKnown;
    // At most one fixed-optics bank; signed cutoff identity controls reset.
    std::optional<double> conicCutoff;bool conicBankAttempted=false;
    std::unique_ptr<conic_moments_v4::Bank> conicBank;
    bool subsetPrepared=false;std::vector<SubsetMapV2> subsetMaps;
    double backgroundPhaseBound=infinity,polePhaseBound=infinity,backgroundNormBound=infinity,projectionRoundFactor=infinity;
    bool modeCache=false;std::vector<SourceWave> sources;
    std::optional<source_wave_sum_v5::OpticsV5> sourceWaveOpticsV5;
    struct Trajectory {
        std::array<double,6> shape{};unsigned branch=0;RayPath path;
        std::vector<double> x,y,xProbability,yProbability;
        double minimumLevel=infinity,maximumLevel=-infinity;
        std::vector<double> rawSource;
    };
    std::optional<Trajectory> trajectory;
    double nullCost=infinity,nullCutoff=0,nullAmplitude=0;
    std::size_t maximumGrounds;
    struct Null {double cutoff,cost,amplitude;};std::vector<Null> nullMinima;
    Inverse(const Optics& o,const ThermalReticleResultV1& input,V pointing,V ownUp,RegionalOperator operation):
        optics(o),c(o.policy),p(c.detector),measured(input),regionalOperator(operation),forward(unit(pointing)),up(unit(ownUp)),count(c.samplesPerScan),
        pixels(static_cast<std::size_t>(p.width)*p.height),alpha(-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS))),
        pixelArea(p.pixelPitchRad*p.pixelPitchRad),variance(input.varianceBoundElectrons2),
        halfX(.5*p.width*p.pixelPitchRad),halfY(.5*p.height*p.pixelPitchRad),
        low(0),high(0),cutStep(0),angularLattice(p.pixelPitchRad/std::ldexp(1.,18)),cutLattice(0),first(0),last(0),
        data(input.measuredElectrons),dataNorm(0),ut(0),tt(0),uu(0),det(0),maximumGrounds(0) {
        profile.conicVersion=regionalOperator==RegionalOperator::ConicFastV5?5u:
            regionalOperator==RegionalOperator::ConicV4?4u:2u;
        // Explicit V5 only. LegacyV2 and ConicV4 keep the old dense source operator.
        if(regionalOperator==RegionalOperator::ConicFastV5)
            sourceWaveOpticsV5.emplace(optics.solidAngles,optics.timeWeights,p.width,p.height,count,p.exposureSamples,pixelArea);
        right=unit(cross(forward,up));if(norm(right)<.5)right=unit(cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
        vertical=unit(cross(right,forward));
        const double margin=9*p.psfSigmaRad;
        const double radius=std::atan(std::hypot(std::tan(std::min(1.55,halfX+margin)),std::tan(std::min(1.55,halfY+margin))));
        const double tilt=std::acos(std::clamp(dot(forward,up),-1.,1.));
        low=std::cos(std::min(3.14159265358979323846,tilt+radius));high=std::cos(std::max(0.,tilt-radius));
        cutStep=(high-low)/p.height;cutLattice=cutStep/std::ldexp(1.,18);
        first=input.exposureStartTimeS+.5/(c.frequencyHz*count)+p.integrationTimeS*(1./p.exposureSamples-.5);
        last=input.exposureStartTimeS+(count-.5)/(c.frequencyHz*count)+.5*p.integrationTimeS;
        project(data,optics.nuisance);dataNorm=inner(data,data);
        uniform=filter(optics.flat,alpha);for(auto& value:uniform)value/=pixelArea;
        transient.resize(count);dark.resize(count);double state=0;
        const double darkInput=static_cast<double>(pixels)*p.darkCurrentElectronsS*p.integrationTimeS;
        for(unsigned j=0;j<count;++j){transient[j]=std::pow(1-alpha,j+1);state+=alpha*(darkInput-state);dark[j]=state;}
        ut=inner(uniform,transient);tt=inner(transient,transient);uu=inner(uniform,uniform);det=uu*tt-ut*ut;
        const auto coarse=static_cast<std::size_t>(p.height)+1,minima=coarse-coarse/3;
        // One retained measured static proposal plus2branches*5null basins;
        //64iterations*2cut derivatives+12trials per independent start.
        // Selected plus at most11local-basin covariances add2each before a finite snapshot. Every snapshot
        // cutoff can add its two neighbours, shared by all branch/reflections.
        // Up to5explicit initial cutoffs may need their canonical lattice
        // representation in addition to the source static-proposal value.
        maximumGrounds=3*(coarse+36*minima+5+11*(1+64*(2+12))+2*(1+11));
        profile.readTime=input.timeS;profile.captureTime=input.measurementTimeS;profile.checkpoint("start");
        diagnostic.attempted=true;diagnostic.readTimeS=input.timeS;diagnostic.captureTimeS=input.measurementTimeS;
        diagnostic.firstStateTimeS=first;diagnostic.lastStateTimeS=last;
        diagnostic.firstPhotonTimeS=first-.5*p.integrationTimeS/p.exposureSamples;
        diagnostic.lastPhotonTimeS=last-.5*p.integrationTimeS/p.exposureSamples;
        diagnostic.degrees=static_cast<double>(count)-static_cast<double>(optics.nuisance.size())-9;
        const double nx=p.width*std::ldexp(1.,18)+1,ny=p.height*std::ldexp(1.,18)+1;
        diagnostic.logHypotheses=std::log(8.)+3*std::log(nx)+4*std::log(ny);
        diagnostic.evidenceThreshold=std::sqrt(p.detectionSigma*p.detectionSigma+2*diagnostic.logHypotheses);
        const double tail=.5*p.detectionSigma*p.detectionSigma;
        diagnostic.residualBound=diagnostic.degrees+2*std::sqrt(std::max(0.,diagnostic.degrees*tail))+2*tail;
    }
    const std::vector<double>* groundField(double cutoff) {
        LocalTimerV2 timer(&profile,LocalProfileV2::FullGround);
        if(lastGroundCutoff&&*lastGroundCutoff==cutoff&&std::signbit(*lastGroundCutoff)==std::signbit(cutoff)){if(profile.enabled)++profile.groundHits;return &lastGroundField;}
        auto field=ThermalReticleReceiverV1::unitGroundFieldV1(p,forward,up,cutoff);if(!field)return nullptr;
        lastGroundCutoff=cutoff;lastGroundField=std::move(*field);return &lastGroundField;
    }
    const std::vector<double>* originalGround(double cutoff) {
        if(nativeGroundCutoff&&*nativeGroundCutoff==cutoff&&std::signbit(*nativeGroundCutoff)==std::signbit(cutoff))return &nativeGroundField;
        ThermalSceneFrameV1 hypothesis;hypothesis.up=unit(up);hypothesis.groundHorizonCosine=cutoff;hypothesis.groundRadianceWm2Sr=1;
        auto optical=p;optical.exposureSamples=1;
        // Match the normalization performed inside unitGroundFieldV1 exactly.
        auto ground=ThermalImageReceiverV1::incidentFieldV1(hypothesis,unit(forward),optical);
        if(!ground)return nullptr;
        nativeGroundCutoff=cutoff;nativeGroundField=std::move(ground->radiance);return &nativeGroundField;
    }
    std::vector<double> wave(const std::vector<double>& field) const {
        LocalTimerV2 timer(&profile,LocalProfileV2::GroundWave);
        std::vector<double> result(count,0);
        for(std::size_t i=0;i<field.size();++i)for(unsigned j=0;j<count;++j)
            result[j]+=field[i]*optics.solidAngles[i]*optics.weights[i*count+j]/pixelArea;
        return filter(std::move(result),alpha);
    }
    std::optional<std::array<double,2>> rawSky(const std::vector<double>& component) const {
        if(!(uu>0)||!(tt>=0)||(tt>0&&!(det>128*epsilon*uu*tt)))return std::nullopt;
        auto remaining=measured.measuredElectrons;
        for(unsigned j=0;j<count;++j)remaining[j]-=dark[j];
        const auto coefficient=[&](const std::vector<double>& y){return tt>0?(inner(y,uniform)*tt-inner(y,transient)*ut)/det:inner(y,uniform)/uu;};
        return std::array<double,2>{{coefficient(remaining),coefficient(component)}};
    }
    const Ground* calibration(double cutoff) {
        {LocalTimerV2 timer(&profile,LocalProfileV2::Lookup);
        for(const auto& item:grounds){if(profile.enabled)++profile.probes;if(exactScalar(item.cutoff,cutoff)){if(profile.enabled)++profile.hits;return &item;}}}
        if(grounds.size()>=maximumGrounds){diagnostic.refusal=21;return nullptr;}
        const auto* field=groundField(cutoff);if(!field){diagnostic.refusal=22;return nullptr;}
        Ground item;item.cutoff=cutoff;item.raw=wave(*field);item.wave=item.raw;project(item.wave,optics.nuisance);
        grounds.push_back(std::move(item));auto& added=grounds.back();diagnostic.calibrations=grounds.size();
        const double norm2=inner(added.wave,added.wave);
        if(norm2>128*epsilon)if(const auto sky=rawSky(added.raw)) {
            const auto physical=ThermalReticleReceiverV1::physicalBackgroundAmplitudeV1((*sky)[0],(*sky)[1],inner(data,added.wave)/norm2);
            if(physical) {
                added.amplitude=*physical;added.cost=0;
                for(unsigned j=0;j<count;++j){const double r=data[j]-added.amplitude*added.wave[j];added.cost+=r*r;}
                if(added.cost<nullCost){nullCost=added.cost;nullCutoff=cutoff;nullAmplitude=added.amplitude;}
            }
        }
        return &added;
    }
    bool nullSearch() {
        profile.checkpoint("null.begin");
        grounds.reserve(maximumGrounds);
        std::vector<Null> coarse;
        for(unsigned row=0;row<=p.height;++row) {
            const auto* item=calibration(low+(high-low)*row/p.height);if(!item)return false;
            coarse.push_back({item->cutoff,item->cost,item->amplitude});
        }
        for(std::size_t i=0;i<coarse.size();++i) {
            const auto seed=coarse[i];const double left=i?coarse[i-1].cost:infinity,rightCost=i+1<coarse.size()?coarse[i+1].cost:infinity;
            if(!std::isfinite(seed.cost)||seed.cost>left||seed.cost>rightCost||!(seed.cost<left||seed.cost<rightCost))continue;
            auto local=seed;double step=cutStep;
            for(unsigned level=0;level<18;++level) {
                step*=.5;const double centre=local.cutoff;
                for(const double sign:{-1.,1.}) {
                    const double cutoff=centre+sign*step;if(cutoff<low||cutoff>high)continue;
                    const auto* item=calibration(cutoff);if(!item)return false;
                    if(item->cost<local.cost)local={item->cutoff,item->cost,item->amplitude};
                }
            }
            nullMinima.push_back(local);
        }
        std::stable_sort(nullMinima.begin(),nullMinima.end(),[](const Null& a,const Null& b){return a.cost<b.cost;});
        profile.checkpoint("null.end",nullCost/variance);
        return std::isfinite(nullCost);
    }
    std::optional<RayPath> path(const Shape& q,unsigned branch) const {
        return endpointPath(opticalRay(q[0],q[1]),opticalRay(q[2],q[3]),first-measured.timeS,last-measured.timeS,p.integrationTimeS,branch);
    }
    V photonAtState(const RayPath& route,double relativeStateS) const {
        // Mirror BOTH normalizations in expose()->incidentFieldImplV1().
        // The outer scene is at subexposure end; the inner optical integration
        // samples back half its duration with the unchanged read derivative.
        return unit(subtract(directionAt(route,relativeStateS),scale(route.readDerivative,.5*p.integrationTimeS/p.exposureSamples)));
    }
    std::array<double,2> levelBounds(double x,double y,double width,double height) const {
        const double xl=std::tan(x-.5*width),xr=std::tan(x+.5*width),yl=std::tan(y-.5*height),yr=std::tan(y+.5*height);
        const double a=dot(forward,up),b=dot(right,up),d=dot(vertical,up);
        const double nlo=a+std::min(b*xl,b*xr)+std::min(d*yl,d*yr),nhi=a+std::max(b*xl,b*xr)+std::max(d*yl,d*yr);
        const auto nearest=[](double l,double r){return l<=0&&r>=0?0.:std::min(std::abs(l),std::abs(r));};
        const double dx=nearest(xl,xr),dy=nearest(yl,yr);
        const double dlo=std::sqrt(1+dx*dx+dy*dy),dhi=std::sqrt(1+std::max(xl*xl,xr*xr)+std::max(yl*yl,yr*yr));
        const double error=128*epsilon*(1+std::abs(a)+std::abs(b)+std::abs(d));
        return {{std::nextafter(nlo/(nlo>=0?dhi:dlo)-error,-infinity),std::nextafter(nhi/(nhi>=0?dlo:dhi)+error,infinity)}};
    }
    Trajectory* prepare(const Shape& q,unsigned branch) {
        LocalTimerV2 timer(&profile,LocalProfileV2::Prepare);
        const std::array<double,6> key{{q[0],q[1],q[2],q[3],q[4],q[5]}};
        if(trajectory&&exactCoordinates(trajectory->shape,key)&&trajectory->branch==branch){if(profile.enabled)++profile.prepareHits;return &*trajectory;}
        const auto ray=path(q,branch);if(!ray)return nullptr;
        const SourceWave* cachedSource=nullptr;
        if(modeCache)for(const auto& entry:sources)if(exactCoordinates(entry.shape,key)&&entry.branch==branch){cachedSource=&entry;break;}
        Trajectory item;item.shape=key;item.branch=branch;item.path=*ray;
        const auto samples=static_cast<std::size_t>(count)*p.exposureSamples;
        item.x.resize(samples);item.y.resize(samples);
        if(!cachedSource){item.xProbability.assign(samples*p.width,0);item.yProbability.assign(samples*p.height,0);item.rawSource.assign(count,0);}
        else item.rawSource=cachedSource->raw;
        const double sigma=p.psfSigmaRad;
        const auto probability=[&](double lo,double hi,double centre,double extent) {
            return extent==0?.5*(std::erfc((centre-hi)/(std::sqrt(2.)*sigma))-std::erfc((centre-lo)/(std::sqrt(2.)*sigma))):
                ThermalImageReceiverV1::gaussianIntervalV1(centre-.5*extent,centre+.5*extent,lo,hi,sigma)/extent;
        };
        for(unsigned j=0;j<count;++j)for(unsigned k=0;k<p.exposureSamples;++k) {
            const auto timeIndex=static_cast<std::size_t>(j)*p.exposureSamples+k;
            const double time=measured.exposureStartTimeS+(j+.5)/(c.frequencyHz*count)+
                p.integrationTimeS*((k+.5)/p.exposureSamples-.5)+.5*p.integrationTimeS/p.exposureSamples;
            const auto direction=photonAtState(*ray,time-measured.timeS);if(!(direction.x>0))return nullptr;
            const double x=std::atan2(direction.y,direction.x),y=std::atan2(direction.z,direction.x);
            item.x[timeIndex]=x;item.y[timeIndex]=y;
            const auto levels=levelBounds(x,y,q[4],q[5]);
            item.minimumLevel=std::min(item.minimumLevel,levels[0]);item.maximumLevel=std::max(item.maximumLevel,levels[1]);
            if(cachedSource)continue;
            for(unsigned col=0;col<p.width;++col) {
                const double lo=(col-.5*p.width)*p.pixelPitchRad,hi=lo+p.pixelPitchRad;
                if(lo>x+.5*q[4]+9*sigma||hi<x-.5*q[4]-9*sigma)continue;
                item.xProbability[timeIndex*p.width+col]=probability(lo,hi,x,q[4]);
            }
            for(unsigned row=0;row<p.height;++row) {
                const double lo=(row-.5*p.height)*p.pixelPitchRad,hi=lo+p.pixelPitchRad;
                if(lo>y+.5*q[5]+9*sigma||hi<y-.5*q[5]-9*sigma)continue;
                item.yProbability[timeIndex*p.height+row]=probability(lo,hi,y,q[5]);
            }
            const bool sparseV5=sourceWaveOpticsV5&&sourceWaveOpticsV5->addSample(
                item.rawSource[j],item.xProbability,item.yProbability,timeIndex);
            if(!sparseV5) {
                for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
                    const auto index=static_cast<std::size_t>(row)*p.width+col;
                    const double value=item.yProbability[timeIndex*p.height+row]*item.xProbability[timeIndex*p.width+col]*optics.solidAngles[index]/pixelArea;
                    item.rawSource[j]+=value*optics.timeWeights[timeIndex*pixels+index]/p.exposureSamples;
                }
            }
        }
        if(!cachedSource)item.rawSource=filter(std::move(item.rawSource),alpha);
        if(ray->stationary)item.rawSource=optics.staticSourceRaw({{q[0],q[1],q[4],q[5]}});
        // The128-entry source memo bounds payload only; further exact
        // geometries are recomputed and remain in the hypothesis bank.
        if(modeCache&&!cachedSource&&sources.size()<128)sources.push_back({key,branch,item.rawSource});
        trajectory=std::move(item);return &*trajectory;
    }
    std::optional<double> regionalPhase(const Shape& q,unsigned branch,unsigned j,
        const Trajectory& source,const std::vector<double>& full) {
        if(!regionalShape||!exactCoordinates(*regionalShape,q)||regionalBranch!=branch) {
            regionalShape=q;regionalBranch=branch;regionalPhases.assign(count,0);regionalKnown.assign(count,false);
        }
        if(regionalKnown[j]){if(profile.enabled)++profile.phaseHits;return regionalPhases[j];}
        double value=0;
        // This is the original native phase loop, shared by full likelihood
        // and certified partial-phase tests. A fallback reuses exact scalars.
        for(unsigned k=0;k<p.exposureSamples;++k) {
            const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
            const auto levels=levelBounds(source.x[ti],source.y[ti],q[4],q[5]);
            std::optional<std::vector<double>> occulted;
            if(levels[0]<q[6]) {
                const std::array<double,4> region{{source.x[ti]-.5*q[4],source.x[ti]+.5*q[4],source.y[ti]-.5*q[5],source.y[ti]+.5*q[5]}};
                LocalTimerV2 regionalTimer(&profile,LocalProfileV2::Regional);
                if((regionalOperator==RegionalOperator::ConicV4||regionalOperator==RegionalOperator::ConicFastV5)) {
                    // Private V4 is a separately versioned continuous stored-basis
                    // numerical operator. Its measured native differences are not
                    // a universal certificate for an arbitrary UCRT classifier.
                    if(profile.enabled)++profile.conicRequests;
                    if(!conicCutoff||!exactScalar(*conicCutoff,q[6])) {
                        conicBank.reset();conicCutoff=q[6];conicBankAttempted=false;
                    }
                    // Keep the old V2 monotonic/sensitive-edge preflight. Failed
                    // V4 geometry/arithmetic/error still runs the complete old
                    // V2/native implementation below, with its original callback.
                    const auto eligible=[&]() {
                        const auto nativeForward=unit(forward),nativeUp=unit(up);
                        auto nativeRight=unit(cross(nativeForward,nativeUp));
                        if(norm(nativeRight)<.5)nativeRight=unit(cross(nativeForward,std::abs(nativeForward.z)<.9?V{0,0,1}:V{0,1,0}));
                        const auto nativeVertical=unit(cross(nativeRight,nativeForward));
                        const double margin=9*p.psfSigmaRad,hx=.5*p.width*p.pixelPitchRad,hy=.5*p.height*p.pixelPitchRad;
                        const double left=std::max(std::max(-1.55,-hx-margin),region[0]);
                        const double rightEdge=std::min(std::min(1.55,hx+margin),region[1]);
                        const double bottom=std::max(std::max(-1.55,-hy-margin),region[2]);
                        const double top=std::min(std::min(1.55,hy+margin),region[3]);
                        if(!(rightEdge>left)||!(top>bottom))return true;
                        const double a=dot(nativeForward,nativeUp),b=dot(nativeRight,nativeUp),cc=dot(nativeVertical,nativeUp);
                        const double maxTx=std::max(std::abs(std::tan(left)),std::abs(std::tan(rightEdge)));
                        const double maxTy=std::max(std::abs(std::tan(bottom)),std::abs(std::tan(top)));
                        const double maxN=std::nextafter(std::abs(a)+std::nextafter(std::abs(b)*maxTx,infinity),infinity);
                        if(!(std::nextafter(cc,-infinity)>std::nextafter(maxN*maxTy,infinity)))return false;
                        const auto horizon=[&](double x){return std::atan((q[6]-a-b*std::tan(x))/cc);};
                        const double h0=horizon(left),h1=horizon(rightEdge),lowEdge=std::min(h0,h1),highEdge=std::max(h0,h1);
                        const bool touches=region[0]<=-hx+1e-12||region[1]>=hx-1e-12||region[2]<=-hy+1e-12||region[3]>=hy-1e-12;
                        // V5 uses a certified conic moment operator, not the old
                        // interior-node quadrature. Keep its finite/monotonic
                        // preflight, then let Bank certify this edge domain.
                        // V4 retains the exact sensitive-edge restriction.
                        return regionalOperator==RegionalOperator::ConicFastV5||
                            !(touches&&region[2]<=lowEdge+p.pixelPitchRad&&region[3]>=highEdge-p.pixelPitchRad);
                    };
                    if(eligible()) {
                        if(!conicBankAttempted) {
                            conicBankAttempted=true;
                            conic_moments_v4::Request request;request.width=p.width;request.height=p.height;
                            request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;
                            request.pointing={{forward.x,forward.y,forward.z}};request.up={{up.x,up.y,up.z}};request.cutoff=q[6];
                            LocalProfileV2::Clock::time_point started{};
                            if(profile.enabled)started=LocalProfileV2::Clock::now();
                            conicBank=conic_moments_v4::Bank::prepare(request);
                            if(profile.enabled){++profile.conicBanks;profile.conicSetupUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-started).count();}
                        }
                        if(conicBank) {
                            LocalProfileV2::Clock::time_point started{};
                            if(profile.enabled)started=LocalProfileV2::Clock::now();
                            auto rendered=regionalOperator==RegionalOperator::ConicFastV5?
                                conicBank->evaluateFastMoments(region):conicBank->evaluate(region);
                            if(profile.enabled)profile.conicRenderUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-started).count();
                            if(rendered&&rendered->field.size()==pixels&&rendered->absoluteError.size()==pixels) {
                                if(profile.enabled){++profile.conicFields;profile.conicMaximumError=std::max(profile.conicMaximumError,*std::max_element(rendered->absoluteError.begin(),rendered->absoluteError.end()));}
                                occulted=std::move(rendered->field);
                            }
                        }
                    }
                }
                if(!occulted) {
                    if(profile.enabled&&(regionalOperator==RegionalOperator::ConicV4||regionalOperator==RegionalOperator::ConicFastV5))++profile.conicFallbacks;
                    if(profile.enabled) {
                        ThermalGroundWorkV2 work;bool accelerated=false;
                        occulted=ThermalReticleReceiverV1::unitGroundFieldProfileV2(p,forward,up,q[6],region,[&](){return originalGround(q[6]);},&work,&accelerated);
                        profile.addGround(work,accelerated);
                        if(occulted)profile.captureGround(work,accelerated,p,forward,up,q[6],region,*occulted,q,branch,j,k);
                    } else occulted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,q[6],region,[&](){return originalGround(q[6]);});
                }
                if(!occulted){diagnostic.refusal=22;return std::nullopt;}
                ++diagnostic.regionalFields;
                if(profile.enabled&&diagnostic.regionalFields%1024==0)profile.checkpoint("regional.progress");
            }
            for(std::size_t i=0;i<pixels;++i) {
                const double field=full[i]-(occulted?(*occulted)[i]:0.);
                value+=field*optics.solidAngles[i]*optics.timeWeights[ti*pixels+i]/pixelArea/p.exposureSamples;
            }
        }
        regionalKnown[j]=true;regionalPhases[j]=value;return value;
    }
    void prepareSubsetMaps() {
        if(subsetPrepared)return;
        subsetPrepared=true;
        if(!certifiedArithmeticV2()||!(alpha>0&&alpha<=1)||std::fegetround()!=FE_TONEAREST)return;
        if(!std::all_of(optics.timeWeights.begin(),optics.timeWeights.end(),[](double value){return std::isfinite(value)&&value>=0&&value<=1;}))return;
        long double total=0;for(const auto solid:optics.solidAngles)total+=std::abs(static_cast<long double>(solid))/pixelArea;
        // A native unit-field integral is clipped before normal division;
        // the original subtraction costs only a few normal operations.
        // Each unit field is therefore <2, so |full-occulted|<4.
        // Every reticle transmission lies in[0,1]. All actual native weighted
        // products/divisions/summations are covered by this gamma factor.
        backgroundPhaseBound=upperPositive(4*total*(1+roundGamma(8*pixels*p.exposureSamples+16)));
        if(!std::isfinite(backgroundPhaseBound)||backgroundPhaseBound<std::numeric_limits<double>::min()*count)return;
        polePhaseBound=upperPositive((backgroundPhaseBound+16*count*std::numeric_limits<double>::denorm_min())/(1-roundGamma(16*count+16)));
        backgroundNormBound=upperPositive(std::sqrt(static_cast<long double>(count))*polePhaseBound);
        double growth=1;for(std::size_t i=0;i<optics.nuisance.size();++i)growth=upperPositive(static_cast<long double>(growth)*(1+4*roundGamma(count+6)));
        projectionRoundFactor=upperPositive(static_cast<long double>(growth)-1);
        const double beta=1-alpha;
        for(const unsigned stride:{8u,4u,2u}) {
            auto map=makeSubsetMap(stride,beta,data,optics.nuisance,backgroundNormBound);
            if(map.valid)subsetMaps.push_back(std::move(map));
        }
    }
    bool certifiedModeRefusal(const Shape& q,unsigned branch,double maximumCost) {
        LocalTimerV2 timer(&profile,LocalProfileV2::SubsetWork);
        if(!valid(q)||!(maximumCost>=0)||!std::isfinite(maximumCost))return false;
        if(lastBasis&&lastShape&&exactCoordinates(*lastShape,q)&&lastBranch==branch)return false;
        // Calibration is visited even when the fixed hypothesis is rejected;
        // no background-null search or planned cutoff is removed.
        if(!calibration(q[6]))return false;
        auto* source=prepare(q,branch);if(!source)return false;
        if(source->path.stationary||q[4]==0||q[5]==0||source->minimumLevel>=q[6]||source->maximumLevel<q[6])return false;
        // A partial-cost refusal must not hide a native failure at an
        // unrendered phase. Validate every required region before pruning.
        for(std::size_t ti=0;ti<source->x.size();++ti) {
            if(levelBounds(source->x[ti],source->y[ti],q[4],q[5])[0]>=q[6])continue;
            const std::array<double,4> region{{source->x[ti]-.5*q[4],source->x[ti]+.5*q[4],source->y[ti]-.5*q[5],source->y[ti]+.5*q[5]}};
            if(!std::all_of(region.begin(),region.end(),[](double value){return std::isfinite(value)&&std::abs(value)<=1.55;})||
                !(region[1]>region[0])||!(region[3]>region[2]))return false;
        }
        prepareSubsetMaps();if(subsetMaps.empty())return false;
        auto sourceProjected=source->rawSource;project(sourceProjected,optics.nuisance);
        const auto* full=groundField(q[6]);if(!full)return false;
        const double beta=1-alpha,tiny=std::numeric_limits<double>::denorm_min();
        const double lop=upperPositive(std::sqrt(1+static_cast<long double>(beta)*beta));
        const double sourceNorm=vectorUpper(sourceProjected);
        for(const auto& map:subsetMaps) {
            if(profile.enabled)++profile.subsetChecks;
            const auto rows=map.phases.size();std::vector<double> input(rows),sourceInput(rows);
            for(std::size_t row=0;row<rows;++row) {
                const auto j=map.phases[row];const auto value=regionalPhase(q,branch,j,*source,*full);if(!value)return false;
                input[row]=alpha*(*value);
                sourceInput[row]=std::fma(-beta,sourceProjected[j-1],sourceProjected[j]);
            }
            const auto apply=[&](const std::vector<double>& values) {
                std::vector<double> result(rows,0);
                for(std::size_t row=0;row<rows;++row)for(std::size_t col=0;col<rows;++col)result[row]+=map.matrix[row][col]*values[col];
                return result;
            };
            std::array<std::vector<double>,2> columns{{apply(input),apply(sourceInput)}};
            const double inputNorm=vectorUpper(input),sourceInputNorm=vectorUpper(sourceInput);
            const double poleError=upperPositive(std::sqrt(static_cast<long double>(rows))*roundGamma(20)*polePhaseBound+16*static_cast<double>(rows)*tiny);
            const double projectionError=upperPositive(projectionRoundFactor*backgroundNormBound+4*static_cast<double>(count*optics.nuisance.size())*tiny*(1+projectionRoundFactor));
            const double projectedNorm=upperPositive((1+projectionRoundFactor)*backgroundNormBound);
            const double backgroundError=upperPositive(map.operatorBound*(poleError+lop*(projectionError+roundGamma(5)*projectedNorm))+
                map.coefficientLeak+roundGamma(2*rows+4)*map.matrixNorm*inputNorm+static_cast<double>(rows*rows)*tiny);
            const double sourceError=upperPositive(map.operatorBound*(roundGamma(2)*sourceInputNorm+lop*roundGamma(5)*sourceNorm+static_cast<double>(rows)*tiny)+
                roundGamma(2*rows+4)*map.matrixNorm*sourceInputNorm+static_cast<double>(rows*rows)*tiny);
            // The original full score squares and sums rounded residuals.
            // Include its positive-sum relative error and absolute underflow.
            const auto fullNorm=std::sqrt((static_cast<long double>(maximumCost)+count*tiny)/(1-roundGamma(2*count+8)));
            const double allowed=upperPositive(map.operatorBound*lop*fullNorm);
            if(certifiedSubsetRefusalV2(map.target,std::move(columns),map.targetError,{{backgroundError,sourceError}},allowed)) {
                if(profile.enabled)++profile.subsetRefusals;
                return true;
            }
        }
        if(profile.enabled)++profile.subsetFallbacks;
        return false;
    }

    bool valid(const Shape& q) const {
        return std::all_of(q.begin(),q.end(),[](double x){return std::isfinite(x);})&&
            std::abs(q[0])<=halfX&&std::abs(q[2])<=halfX&&std::abs(q[1])<=halfY&&std::abs(q[3])<=halfY&&
            q[4]>=0&&q[4]<=2*halfX&&q[5]>=0&&q[5]<=2*halfY&&q[6]>=low&&q[6]<=high;
    }
    std::optional<Basis> basis(const Shape& q,unsigned branch) {
        LocalTimerV2 timer(&profile,LocalProfileV2::BasisWork);
        if(profile.enabled&&profile.calls[LocalProfileV2::BasisWork]%256==0)profile.checkpoint("basis.progress");
        if(!valid(q))return std::nullopt;
        if(lastBasis&&lastShape&&exactCoordinates(*lastShape,q)&&lastBranch==branch)return lastBasis;
        const auto* ground=calibration(q[6]);if(!ground)return std::nullopt;
        auto* source=prepare(q,branch);if(!source)return std::nullopt;
        ++diagnostic.basisEvaluations;Basis answer;answer.raw[1]=source->rawSource;
        if(source->path.stationary) {
            if(q[4]>0&&q[5]>0) {
                const std::array<double,4> region{{q[0]-.5*q[4],q[0]+.5*q[4],q[1]-.5*q[5],q[1]+.5*q[5]}};
                const auto occulted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,q[6],region,[&](){return originalGround(q[6]);});if(!occulted)return std::nullopt;
                const auto* full=groundField(q[6]);if(!full)return std::nullopt;
                auto field=*full;for(std::size_t i=0;i<pixels;++i)field[i]-=(*occulted)[i];answer.raw[0]=wave(field);
            } else answer.raw[0]=ground->raw;
        } else if(q[4]==0||q[5]==0||source->minimumLevel>=q[6])answer.raw[0]=ground->raw;
        else if(source->maximumLevel<q[6]) {
            // Exact change of linear coordinates: a wholly ground-backed
            // radiator has contrast relative to ground. No subtraction of
            // separately rounded phase vectors or missing occulted geometry.
            answer.raw[0]=ground->raw;answer.sourceRelativeGround=true;
        } else {
            const auto* full=groundField(q[6]);if(!full)return std::nullopt;
            answer.raw[0].assign(count,0);
            for(unsigned j=0;j<count;++j) {
                const auto value=regionalPhase(q,branch,j,*source,*full);if(!value)return std::nullopt;
                answer.raw[0][j]=*value;
            }
            answer.raw[0]=filter(std::move(answer.raw[0]),alpha);
        }
        answer.projected=answer.raw;for(auto& column:answer.projected)project(column,optics.nuisance);
        lastShape=q;lastBranch=branch;lastBasis=answer;return answer;
    }
    double effectiveSource(const Shape& q,const Basis& values,const Fit& fit) const {
        return fit.source-(values.sourceRelativeGround?q[4]*q[5]/pixelArea*fit.background:0.);
    }
    std::optional<Fit> solve(const Shape& q,unsigned branch,std::vector<double>* reduced=nullptr) {
        const auto values=basis(q,branch);if(!values)return std::nullopt;
        const auto& u=values->projected[0];const auto& v=values->projected[1];
        const double uu2=inner(u,u),uv=inner(u,v),vv=inner(v,v),determinant=uu2*vv-uv*uv;
        if(!(uu2>0)||!(vv>0)||!(determinant>128*epsilon*uu2*vv))return std::nullopt;
        const double yu=inner(data,u),yv=inner(data,v),b=(yu*vv-yv*uv)/determinant,d=(yv*uu2-yu*uv)/determinant;
        Fit fit;fit.cost=0;fit.background=b;fit.source=d+(values->sourceRelativeGround?q[4]*q[5]/pixelArea*b:0.);
        if(reduced)reduced->resize(count);
        for(unsigned j=0;j<count;++j) {
            const double r=data[j]-b*u[j]-d*v[j];fit.cost+=r*r;
            if(reduced)(*reduced)[j]=r/std::sqrt(variance);
        }
        if(!std::isfinite(fit.cost)||!std::isfinite(b)||!std::isfinite(d))return std::nullopt;
        return fit;
    }
    bool physical(const Shape& q,unsigned branch,const Fit& fit,std::array<double,3>* output=nullptr) {
        const auto values=basis(q,branch);if(!values)return false;
        auto remaining=measured.measuredElectrons;const double d=effectiveSource(q,*values,fit);
        for(unsigned j=0;j<count;++j)remaining[j]-=fit.background*values->raw[0][j]+d*values->raw[1][j]+dark[j];
        if(!(uu>0)||!(tt>=0)||(tt>0&&!(det>128*epsilon*uu*tt)))return false;
        const double sky=tt>0?(inner(remaining,uniform)*tt-inner(remaining,transient)*ut)/det:inner(remaining,uniform)/uu;
        const double area=q[4]*q[5]/pixelArea;
        const std::array<double,3> budget{{sky,sky+fit.background,area>0?sky+fit.source/area:fit.source}};
        if(output)*output=budget;
        return std::all_of(budget.begin(),budget.end(),[](double x){return std::isfinite(x)&&x>=0;});
    }
    struct Selected {Shape shape{};unsigned branch=0;Fit fit;};
    std::vector<Selected> localModes;
    std::array<Selected,2> branchBest{};
    bool searchUnresolved=false;
    bool alternateBranchPossible() const {
        const double a=first-measured.timeS,b=last-measured.timeS;
        if(!(a*b>0))return false;
        const double maximumRate=.5/p.integrationTimeS,peakRate=1/std::sqrt(std::abs(a*b));
        const double rounding=256*epsilon*std::max(maximumRate,peakRate);
        if(maximumRate+rounding<peakRate)return false;
        const double farther=std::max(std::abs(a),std::abs(b)),nearer=std::min(std::abs(a),std::abs(b));
        const double minimumAngle=std::atan2(maximumRate*(farther-nearer),1+maximumRate*maximumRate*farther*nearer);
        const double diameter=2*std::atan(std::hypot(std::tan(halfX),std::tan(halfY)));
        return minimumAngle<=diameter+256*epsilon*std::max(1.,diameter);
    }
    std::vector<Shape> fastProposals(double cutoff,double width,double height) const {
        std::vector<Shape> result;
        if(!alternateBranchPossible())return result;
        const double a=std::abs(first-measured.timeS),b=std::abs(last-measured.timeS),rate=.5/p.integrationTimeS;
        const double minimum=std::atan2(rate*std::abs(a-b),1+rate*rate*a*b);
        const double peak=std::atan2(std::abs(a-b),2*std::sqrt(a*b));
        const double diagonal=std::hypot(std::tan(halfX),std::tan(halfY));
        const double maximum=std::min(peak,2*std::atan(diagonal));
        if(!(maximum>minimum))return result;
        const double scale=std::tan(.25*(minimum+maximum))/diagonal;
        const double x=std::atan(scale*std::tan(halfX)),y=std::atan(scale*std::tan(halfY));
        for(const double sx:{-1.,1.})for(const double sy:{-1.,1.})
            result.push_back(quantize({{-sx*x,-sy*y,sx*x,sy*y,width,height,cutoff}}));
        return result;
    }
    Shape quantize(Shape q) const {
        const double subdivisions=std::ldexp(1.,18);
        for(unsigned i=0;i<6;++i) {
            const double maximum=(i==0||i==2||i==4?p.width:p.height)*subdivisions;
            const bool endpoint=i<4;const double lower=endpoint?-.5*maximum:0,upper=endpoint?.5*maximum:maximum;
            const double index=std::clamp(std::round(q[i]/angularLattice),lower,upper);
            if(index==lower)q[i]=endpoint?-(i%2?halfY:halfX):0;
            else if(index==upper)q[i]=endpoint?(i%2?halfY:halfX):(i==4?2*halfX:2*halfY);
            else q[i]=index*angularLattice;
        }
        const double maximum=p.height*subdivisions;
        const double index=std::clamp(std::round((q[6]-low)/cutLattice),0.,maximum);
        q[6]=index==0?low:index==maximum?high:low+index*cutLattice;return q;
    }
    static Shape squared(Shape q){q[4]*=q[4];q[5]*=q[5];return q;}
    static Shape geometry(Shape q){q[4]=std::sqrt(std::max(0.,q[4]));q[5]=std::sqrt(std::max(0.,q[5]));return q;}
    void record(Selected& selected,const Shape& q,unsigned branch,const Fit& fit) {
        if(!(fit.cost<selected.fit.cost))return;
        const auto canonical=quantize(q);
        const auto exact=exactCoordinates(canonical,q)?std::optional<Fit>(fit):solve(canonical,branch);
        if(exact&&exact->cost<selected.fit.cost&&physical(canonical,branch,*exact))selected={canonical,branch,*exact};
    }
    void trust(Selected& selected,Shape initial,unsigned branch) {
        LocalTimerV2 timer(&profile,LocalProfileV2::Trust);profile.checkpoint("trust.begin",selected.fit.cost/variance);
        Shape current=squared(quantize(initial));std::vector<double> residual;
        auto local=solve(geometry(current),branch,&residual);if(!local)return;
        record(selected,geometry(current),branch,*local);
        const Shape lower{{-halfX,-halfY,-halfX,-halfY,0,0,low}};
        const Shape upper{{halfX,halfY,halfX,halfY,4*halfX*halfX,4*halfY*halfY,high}};
        const double minimumDamping=64*epsilon;double damping=1e-3;
        for(unsigned iteration=0;iteration<64;++iteration) {
            if(profile.enabled){++profile.iterations;if(iteration%8==0)profile.checkpoint("trust.progress",local->cost/variance);}
            std::array<std::vector<double>,7> jacobian;bool complete=true;
            for(unsigned coordinate=0;coordinate<7;++coordinate) {
                const double step=coordinate<4?p.pixelPitchRad/1024:coordinate<6?pixelArea/1024:cutStep/1024;
                auto plus=current,minus=current;plus[coordinate]=std::min(upper[coordinate],current[coordinate]+step);
                minus[coordinate]=std::max(lower[coordinate],current[coordinate]-step);
                std::vector<double> a,b;
                if(!(plus[coordinate]>minus[coordinate])||!solve(geometry(plus),branch,&a)||!solve(geometry(minus),branch,&b)) {complete=false;break;}
                jacobian[coordinate].resize(count);
                for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=(a[j]-b[j])/(plus[coordinate]-minus[coordinate]);
            }
            if(!complete)break;
            Shape lowerStep{},upperStep{};
            for(unsigned i=0;i<7;++i){lowerStep[i]=lower[i]-current[i];upperStep[i]=upper[i]-current[i];}
            bool accepted=false;
            for(unsigned trial=0;trial<12;++trial) {
                if(profile.enabled)++profile.trials;
                Shape delta{};if(!boxStep(jacobian,residual,lowerStep,upperStep,damping,delta,&profile))break;
                auto candidate=current;for(unsigned i=0;i<7;++i)candidate[i]=std::clamp(current[i]+delta[i],lower[i],upper[i]);
                candidate=squared(quantize(geometry(candidate)));if(exactCoordinates(candidate,current))break;
                std::vector<double> candidateResidual;const auto fit=solve(geometry(candidate),branch,&candidateResidual);
                if(fit)record(selected,geometry(candidate),branch,*fit);
                if(fit&&fit->cost<local->cost) {
                    double predicted=0;
                    for(unsigned j=0;j<count;++j) {
                        double value=residual[j];for(unsigned i=0;i<7;++i)value+=jacobian[i][j]*(candidate[i]-current[i]);predicted+=value*value;
                    }
                    const double reduction=inner(residual,residual)-predicted;
                    const double ratio=reduction>0?(local->cost-fit->cost)/(variance*reduction):0;
                    if(ratio>.75)damping=std::max(minimumDamping,damping/3);
                    else if(ratio<.25)damping=std::min(1/minimumDamping,damping*2);
                    current=candidate;local=fit;residual=std::move(candidateResidual);accepted=true;break;
                }
                damping=std::min(1/minimumDamping,damping*10);
            }
            if(!accepted||diagnostic.refusal==21)break;
        }
    }
    Selected optimize() {
        profile.checkpoint("optimizer.begin");
        Selected selected;
        const auto continuation=[&](const Shape& initial,unsigned branch) {
            Selected local;trust(local,initial,branch);
            if(!std::isfinite(local.fit.cost))return;
            localModes.push_back(local);
            if(local.fit.cost<branchBest[branch].fit.cost)branchBest[branch]=local;
            record(selected,local.shape,branch,local.fit);
        };
        double x=0,y=0,width=2*p.pixelPitchRad,height=2*p.pixelPitchRad,proposalCut=nullCutoff;
        const auto& previous=measured.horizonDiagnostic;
        if(previous.fitAvailable&&std::all_of(previous.shape.begin(),previous.shape.end(),[](double v){return std::isfinite(v);})) {
            x=previous.shape[0];y=previous.shape[1];width=previous.shape[2];height=previous.shape[3];proposalCut=previous.shape[4];
        } else if(std::isfinite(measured.compactFitResidual)&&measured.compactFitSnr>0) {
            x=measured.fittedCentreXRad;y=measured.fittedCentreYRad;width=measured.fittedWidthRad;height=measured.fittedHeightRad;
        }
        // Retain the measured static solution as an INDEPENDENT continuation;
        // a different proposal's larger initial correlation must not suppress
        // its own convergent moving-source basin.
        if(proposalCut>=low&&proposalCut<=high) {
            continuation({{x,y,x,y,width,height,proposalCut}},0);
            if(diagnostic.refusal==21)return selected;
        }
        std::vector<double> starts;
        if(proposalCut>=low&&proposalCut<=high)starts.push_back(proposalCut);
        if(std::find(starts.begin(),starts.end(),nullCutoff)==starts.end())starts.push_back(nullCutoff);
        for(const auto& minimum:nullMinima) {
            if(starts.size()>=5)break;
            if(std::find(starts.begin(),starts.end(),minimum.cutoff)==starts.end())starts.push_back(minimum.cutoff);
        }
        // Source-only proposal vectors depend on geometry/branch, never on
        // the background cutoff. Retain each exact phase vector once across
        // the at most 5 cutoff starts, preserving the original ranking order.
        struct RankSource {Shape shape;std::vector<double> projected;double norm2;};
        std::array<std::optional<std::vector<RankSource>>,2> rankBanks;
        for(const double cutoff:starts) {
            const auto* ground=calibration(cutoff);if(!ground)break;
            auto residual=data;for(unsigned j=0;j<count;++j)residual[j]-=ground->amplitude*ground->wave[j];
            for(unsigned branch=0;branch<2;++branch) {
                if(branch==1&&!alternateBranchPossible())continue;
                double best=-infinity;std::optional<Shape> initial;
                if(!rankBanks[branch]) {
                    rankBanks[branch].emplace();auto& bank=*rankBanks[branch];
                    bank.reserve(branch==1?104:100);
                    const auto retain=[&](const Shape& q) {
                        const auto* source=prepare(q,branch);if(!source)return;
                        auto values=source->rawSource;project(values,optics.nuisance);
                        const double norm2=inner(values,values);if(!(norm2>128*epsilon))return;
                        bank.push_back({q,std::move(values),norm2});
                    };
                    // Exact original source-only proposal order and geometry.
                    for(const bool centred:{false,true})for(const bool compact:{false,true})
                    for(const double span:{0.,1.,2.,4.,8.,16.,32.})for(unsigned axis=0;axis<(span==0?1u:4u);++axis) {
                        const double cx=centred?0:x,cy=centred?0:y;
                        const double dx=axis<2?(axis==0?1:-1)*.5*span*p.pixelPitchRad:0;
                        const double dy=axis>=2?(axis==2?1:-1)*.5*span*p.pixelPitchRad:0;
                        retain(quantize({{cx-dx,cy-dy,cx+dx,cy+dy,compact?2*p.pixelPitchRad:width,compact?2*p.pixelPitchRad:height,cutoff}}));
                    }
                    if(branch==1)for(const auto& q:fastProposals(cutoff,width,height))retain(q);
                }
                for(const auto& proposal:*rankBanks[branch]) {
                    auto q=proposal.shape;q[6]=cutoff;q=quantize(q);
                    const double correlation=inner(residual,proposal.projected),score=correlation*correlation/proposal.norm2;
                    if(score>best){best=score;initial=q;}
                }
                if(initial)continuation(*initial,branch);else searchUnresolved=true;
                if(diagnostic.refusal==21)return selected;
            }
        }
        return selected;
    }
    struct Information {double capture=0,first=0,last=0,readRate=0,firstState=0,lastState=0;};
    std::optional<Information> information(const Shape& q,unsigned branch,const Fit& fit) {
        LocalTimerV2 timer(&profile,LocalProfileV2::Information);
        const auto base=basis(q,branch);if(!base)return std::nullopt;
        std::vector<std::vector<double>> amplitude;
        for(auto column:base->projected) {
            for(unsigned pass=0;pass<2;++pass)for(const auto& previous:amplitude) {
                const double a=inner(column,previous);for(unsigned j=0;j<count;++j)column[j]-=a*previous[j];
            }
            const double length=std::sqrt(inner(column,column));if(!(length>128*epsilon))return std::nullopt;
            for(auto& value:column)value/=length;
            amplitude.push_back(std::move(column));
        }
        const Shape current=squared(q);
        std::array<std::vector<double>,7> jacobian;Shape norms{};
        std::array<std::array<V,7>,6> target{};
        const auto prediction=[&](const Shape& shape)->std::optional<std::vector<double>> {
            const auto values=basis(shape,branch);if(!values)return std::nullopt;
            const double d=effectiveSource(shape,*values,fit);auto result=values->projected[0];
            for(unsigned j=0;j<count;++j)result[j]=fit.background*result[j]+d*values->projected[1][j];
            return result;
        };
        for(unsigned coordinate=0;coordinate<7;++coordinate) {
            const double step=coordinate<4?p.pixelPitchRad/1024:coordinate<6?pixelArea/1024:cutStep/1024;
            auto plus=current,minus=current;plus[coordinate]+=step;minus[coordinate]-=step;
            if(coordinate==4||coordinate==5)minus[coordinate]=std::max(0.,minus[coordinate]);
            if(coordinate==6){plus[6]=std::min(high,plus[6]);minus[6]=std::max(low,minus[6]);}
            const auto gp=geometry(plus),gm=geometry(minus);
            const auto a=prediction(gp),b=prediction(gm);if(!a||!b||!(plus[coordinate]>minus[coordinate]))return std::nullopt;
            const double divisor=plus[coordinate]-minus[coordinate];jacobian[coordinate].resize(count);
            for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=((*a)[j]-(*b)[j])/divisor;
            for(unsigned pass=0;pass<2;++pass)for(const auto& column:amplitude) {
                const double coefficient=inner(jacobian[coordinate],column);
                for(unsigned j=0;j<count;++j)jacobian[coordinate][j]-=coefficient*column[j];
            }
            for(const auto value:jacobian[coordinate])norms[coordinate]=std::hypot(norms[coordinate],value);
            if(!std::isfinite(norms[coordinate]))return std::nullopt;
            if(norms[coordinate]==0)norms[coordinate]=1;
            if(coordinate<4) {
                const auto rp=path(gp,branch),rm=path(gm,branch);if(!rp||!rm)return std::nullopt;
                target[0][coordinate]=scale(subtract(directionAt(*rp,measured.measurementTimeS-measured.timeS),directionAt(*rm,measured.measurementTimeS-measured.timeS)),1/divisor);
                target[1][coordinate]=scale(subtract(photonAtState(*rp,first-measured.timeS),photonAtState(*rm,first-measured.timeS)),1/divisor);
                target[2][coordinate]=scale(subtract(photonAtState(*rp,last-measured.timeS),photonAtState(*rm,last-measured.timeS)),1/divisor);
                target[3][coordinate]=scale(subtract(rp->readDerivative,rm->readDerivative),1/divisor);
                target[4][coordinate]=scale(subtract(directionAt(*rp,first-measured.timeS),directionAt(*rm,first-measured.timeS)),1/divisor);
                target[5][coordinate]=scale(subtract(directionAt(*rp,last-measured.timeS),directionAt(*rm,last-measured.timeS)),1/divisor);
            }
        }
        std::array<Shape,7> gram{},vectors{};
        double trace=0;
        for(unsigned i=0;i<7;++i) {
            vectors[i][i]=1;
            for(unsigned j=0;j<7;++j)gram[i][j]=inner(jacobian[i],jacobian[j])/(norms[i]*norms[j]);
            trace+=gram[i][i];
        }
        // Symmetric Jacobi diagonalization of the scaled7D information.
        // Null nuisance directions are allowed ONLY if the requested target
        // differential annihilates them; an endpoint/rate null stays coarse.
        for(unsigned sweep=0;sweep<64;++sweep) {
            bool changed=false;
            for(unsigned a=0;a<7;++a)for(unsigned b=a+1;b<7;++b) {
                const double off=gram[a][b];if(std::abs(off)<=32*epsilon*std::sqrt(std::abs(gram[a][a]*gram[b][b])))continue;
                const double tau=(gram[b][b]-gram[a][a])/(2*off);
                const double tangent=std::copysign(1.,tau)/(std::abs(tau)+std::hypot(1.,tau));
                const double cosine=1/std::sqrt(1+tangent*tangent),sine=tangent*cosine;
                const double aa=gram[a][a],bb=gram[b][b];gram[a][a]=aa-tangent*off;gram[b][b]=bb+tangent*off;gram[a][b]=gram[b][a]=0;
                for(unsigned k=0;k<7;++k)if(k!=a&&k!=b) {
                    const double ka=gram[k][a],kb=gram[k][b];gram[k][a]=gram[a][k]=cosine*ka-sine*kb;gram[k][b]=gram[b][k]=sine*ka+cosine*kb;
                }
                for(unsigned k=0;k<7;++k) {
                    const double ka=vectors[k][a],kb=vectors[k][b];vectors[k][a]=cosine*ka-sine*kb;vectors[k][b]=sine*ka+cosine*kb;
                }
                changed=true;
            }
            if(!changed)break;
        }
        const double rankTolerance=128*epsilon*trace;
        std::array<double,6> values{};
        for(unsigned mode=0;mode<7;++mode) {
            const double eigenvalue=gram[mode][mode];if(!std::isfinite(eigenvalue)||eigenvalue<-rankTolerance)return std::nullopt;
            for(unsigned epoch=0;epoch<6;++epoch) {
                V projection{};
                for(unsigned i=0;i<7;++i)projection=add(projection,scale(target[epoch][i],vectors[i][mode]/norms[i]));
                const double magnitude=dot(projection,projection);
                if(eigenvalue<=rankTolerance) {if(magnitude!=0)return std::nullopt;}
                else values[epoch]+=variance*magnitude/eigenvalue;
            }
        }
        // Endpoint quantization is propagated to each reported angular epoch;
        // rate uncertainty is not used to create guidance authority.
        for(unsigned epoch=0;epoch<6;++epoch)if(epoch!=3)for(unsigned i=0;i<4;++i)
            values[epoch]+=dot(target[epoch][i],target[epoch][i])*angularLattice*angularLattice/12;
        if(!std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x)&&x>=0;})||values[0]<=0)return std::nullopt;
        return Information{values[0],values[1],values[2],values[3],values[4],values[5]};
    }
    V world(V optical) const {return unit(add(scale(forward,optical.x),add(scale(right,optical.y),scale(vertical,optical.z))));}
    std::optional<std::array<double,5>> mode(const Shape& q,unsigned branch,const Fit& fit) {
        LocalTimerV2 timer(&profile,LocalProfileV2::ModeWork);
        const auto route=path(q,branch);if(!route||route->domainBoundary)return std::nullopt;
        const auto info=information(q,branch,fit);if(!info)return std::nullopt;
        const auto firstPhoton=photonAtState(*route,first-measured.timeS),lastPhoton=photonAtState(*route,last-measured.timeS);
        const std::vector<std::array<double,5>> firstMode{{{std::atan2(firstPhoton.y,firstPhoton.x),std::atan2(firstPhoton.z,firstPhoton.x),q[4],q[5],info->first}}};
        const std::vector<std::array<double,5>> lastMode{{{std::atan2(lastPhoton.y,lastPhoton.x),std::atan2(lastPhoton.z,lastPhoton.x),q[4],q[5],info->last}}};
        if(!ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(c,firstMode)||!ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(c,lastMode))return std::nullopt;
        if(!ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(c,{{{q[0],q[1],0,0,info->firstState}}})||
           !ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(c,{{{q[2],q[3],0,0,info->lastState}}}))return std::nullopt;
        const double speed=norm(route->readDerivative)*p.integrationTimeS;
        if(!(speed+p.detectionSigma*std::sqrt(info->readRate)*p.integrationTimeS<.5))return std::nullopt;
        const auto capture=directionAt(*route,measured.measurementTimeS-measured.timeS);
        if(!(capture.x>0))return std::nullopt;
        return std::array<double,5>{{std::atan2(capture.y,capture.x),std::atan2(capture.z,capture.x),q[4],q[5],std::pow(std::sqrt(info->capture)+route->endpointRoundoffRad,2)}};
    }
    void run(ThermalReticleResultV1& output) {
        diagnostic.refusal=1;
        if(!(high>low)||!(last>first)||!(diagnostic.degrees>0)||!std::isfinite(first)||!std::isfinite(last)||
           measured.measurementTimeS<first||measured.measurementTimeS>last)return;
        diagnostic.refusal=2;
        const double threshold2=diagnostic.evidenceThreshold*diagnostic.evidenceThreshold;
        if(dataNorm/variance<threshold2)return;
        diagnostic.refusal=3;if(!nullSearch())return;
        diagnostic.nullResidual=nullCost/variance;
        // Physical null remains authoritative. This is also an exact bound
        // on possible source gain; no shape can improve more than null energy.
        diagnostic.refusal=4;
        if(std::min(nullCost,dataNorm)/variance<threshold2)return;
        const auto selected=optimize();profile.checkpoint("optimizer.end",selected.fit.cost/variance);
        if(diagnostic.refusal==21||diagnostic.refusal==22)return;
        diagnostic.fitAvailable=std::isfinite(selected.fit.cost);diagnostic.residual=selected.fit.cost/variance;
        diagnostic.endpointsWidthHeightCutoff=selected.shape;diagnostic.branch=selected.branch;
        diagnostic.refusal=5;
        if(!diagnostic.fitAvailable||diagnostic.residual>diagnostic.residualBound)return;
        diagnostic.refusal=6;
        if(!physical(selected.shape,selected.branch,selected.fit,&diagnostic.rawBudget))return;
        const auto route=path(selected.shape,selected.branch);if(!route)return;
        const auto capture=directionAt(*route,measured.measurementTimeS-measured.timeS);
        diagnostic.captureDirection=world(capture);
        const auto rate=derivativeAt(*route,measured.measurementTimeS-measured.timeS);
        diagnostic.captureDirectionRate=add(scale(forward,rate.x),add(scale(right,rate.y),scale(vertical,rate.z)));
        diagnostic.domainBoundary=route->domainBoundary;
        // Fixed trajectory source waveforms are independent of cutoff. Cache
        // at most104mode/derivative geometries, not spatial fields.
        modeCache=true;sources.clear();sources.reserve(128);lastShape.reset();lastBasis.reset();
        std::vector<std::array<double,5>> modes;
        bool unresolved=searchUnresolved||std::any_of(diagnostic.rawBudget.begin(),diagnostic.rawBudget.end(),[](double x){return x<=0;});
        if(const auto own=mode(selected.shape,selected.branch,selected.fit))modes.push_back(*own);else unresolved=true;
        // Each independently optimized physical basin participates, including
        // basins that were initially worse than the global selected fit.
        for(const auto& local:localModes) {
            if(local.branch==selected.branch&&exactCoordinates(local.shape,selected.shape))continue;
            if(local.fit.cost>selected.fit.cost+threshold2*variance)continue;
            if(const auto candidate=mode(local.shape,local.branch,local.fit))modes.push_back(*candidate);else unresolved=true;
        }
        // Canonical identity, including branch and all six geometry values,
        // is the only deduplication rule. Different cutoffs share a geometry
        // bank because every optimizer cutoff is explicitly visited below.
        struct ModeGeometry {Shape shape;unsigned branch;};
        std::vector<ModeGeometry> geometries;
        std::vector<ModeGeometry> profileSeeds;
        const auto addGeometry=[&](const Selected& local) {
            if(local.fit.cost>selected.fit.cost+threshold2*variance)return;
            for(const double sx:{1.,-1.})for(const double sy:{1.,-1.}) {
                auto q=local.shape;q[0]*=sx;q[2]*=sx;q[1]*=sy;q[3]*=sy;
                if(!path(q,local.branch))continue;
                const auto sameGeometry=[&](const ModeGeometry& item) {
                    return item.branch==local.branch&&std::equal(q.begin(),q.begin()+6,item.shape.begin(),exactScalar);
                };
                if(std::none_of(geometries.begin(),geometries.end(),sameGeometry))geometries.push_back({q,local.branch});
                if(std::none_of(profileSeeds.begin(),profileSeeds.end(),[&](const ModeGeometry& item){return item.branch==local.branch&&exactCoordinates(item.shape,q);}))
                    profileSeeds.push_back({q,local.branch});
            }
        };
        addGeometry(selected);for(const auto& local:localModes)addGeometry(local);
        // Snapshot values are immutable: curvature adds cutoff neighbours but
        // never recursively grows the mode bank. All independent local basins
        // and all their reflected geometries visit the SAME complete cutoff
        // set, including rejected optimizer trials. There is no coarse-only
        // placeholder for an unvisited admitted local-basin continuation.
        std::vector<double> cutoffs;cutoffs.reserve(grounds.size());for(const auto& item:grounds)cutoffs.push_back(item.cutoff);
        profile.checkpoint("modes.begin",selected.fit.cost/variance,unresolved);
        for(const auto& candidateGeometry:geometries) {
            auto q=candidateGeometry.shape;
            for(const double cutoff:cutoffs) {
                if(profile.enabled){++profile.modes;if(profile.modes%32==0)profile.checkpoint("modes.progress",selected.fit.cost/variance,unresolved);}
                q[6]=cutoff;
                if(candidateGeometry.branch==selected.branch&&exactCoordinates(q,selected.shape))continue;
                if(certifiedModeRefusal(q,candidateGeometry.branch,selected.fit.cost+threshold2*variance))continue;
                const auto alternative=solve(q,candidateGeometry.branch);
                if(!alternative||alternative->cost>selected.fit.cost+threshold2*variance)continue;
                if(profile.enabled)++profile.admittedModes;
                std::array<double,3> budget{};if(!physical(q,candidateGeometry.branch,*alternative,&budget))continue;
                if(std::any_of(budget.begin(),budget.end(),[](double x){return x<=0;}))unresolved=true;
                if(const auto candidate=mode(q,candidateGeometry.branch,*alternative))modes.push_back(*candidate);else unresolved=true;
            }
        }
        profile.checkpoint("modes.end",selected.fit.cost/variance,unresolved);
        // A moving endpoint/domain or an unseen support-edge continuation is
        // an unresolved family, even if one fitted local covariance is finite.
        const auto profileContinuation=[&](Shape q) {
            q=quantize(q);
            for(unsigned branch=0;branch<2;++branch) {
                if(certifiedModeRefusal(q,branch,selected.fit.cost+threshold2*variance))continue;
                const auto alternative=solve(q,branch);
                if(alternative&&alternative->cost<=selected.fit.cost+threshold2*variance&&physical(q,branch,*alternative))unresolved=true;
            }
        };
        for(const auto& seed:profileSeeds) {
            for(unsigned coordinate=0;coordinate<4;++coordinate)for(const double sign:{-1.,1.}) {
                auto q=seed.shape;q[coordinate]=sign*(coordinate%2?halfY:halfX);
                if(q[coordinate]!=seed.shape[coordinate])profileContinuation(q);
            }
            for(unsigned axis=0;axis<2;++axis)for(const double sign:{-1.,1.}) {
                const double half=axis?halfY:halfX,extent=seed.shape[axis+4];
                const double firstShift=std::max({-half-seed.shape[axis],-half-seed.shape[axis+2],sign>0?-.5*extent:.5*(extent-2*half)});
                const double lastShift=std::min({half-seed.shape[axis],half-seed.shape[axis+2],sign>0?.5*(2*half-extent):.5*extent});
                if(firstShift>lastShift)continue;
                for(const double shift:{firstShift,lastShift})if(std::abs(shift)>angularLattice) {
                    auto q=seed.shape;q[axis]+=shift;q[axis+2]+=shift;q[axis+4]+=2*sign*shift;profileContinuation(q);
                }
            }
        }
        diagnostic.nullResidual=nullCost/variance;
        if(diagnostic.refusal==21||diagnostic.refusal==22)return;
        // Score cap makes the2D Gaussian-projection bound explicit even when
        // a constrained raw null cannot choose zero contrast. It never treats
        // an impossible raw null as a physical background explanation.
        const double gain=std::max(0.,std::min(nullCost-selected.fit.cost,dataNorm-selected.fit.cost))/variance;
        diagnostic.refusal=7;if(gain<threshold2)return;
        const auto envelope=!unresolved?ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(c,modes):std::nullopt;
        const bool compact=selected.shape[4]<halfX||selected.shape[5]<halfY;
        const double x=std::atan2(capture.y,capture.x),y=std::atan2(capture.z,capture.x);
        const bool nearStop=std::hypot(x,y)>=.75*c.stopRadiusRad;
        diagnostic.refusal=8;if(!envelope&&!compact&&!nearStop)return;
        const double radius=std::atan(std::hypot(std::tan(halfX),std::tan(halfY)));
        ThermalImageFeatureV1 feature;feature.direction=diagnostic.captureDirection;feature.signalToNoise=std::sqrt(gain);
        feature.angularVarianceRad2=envelope?*envelope:4*radius*radius;
        feature.signedContrastElectrons=selected.fit.source;feature.extent.widthRad=selected.shape[4];feature.extent.heightRad=selected.shape[5];
        feature.centroidColumn=x/p.pixelPitchRad+.5*p.width;feature.centroidRow=y/p.pixelPitchRad+.5*p.height;
        output.features.push_back(feature);output.bearingResolved=true;output.preciseBearing=envelope.has_value();
        output.fitSnr=feature.signalToNoise;output.fitResidual=diagnostic.residual;
        diagnostic.accepted=true;diagnostic.precise=envelope.has_value();diagnostic.angularVarianceRad2=feature.angularVarianceRad2;diagnostic.refusal=9;
    }
};
bool validOptics(const Optics& optics) {
    const auto& c=optics.policy;const auto pixels=static_cast<std::size_t>(c.detector.width)*c.detector.height;
    return ThermalReticleReceiverV1::validConfig(c)&&optics.weights.size()==pixels*c.samplesPerScan&&
        optics.timeWeights.size()==pixels*c.samplesPerScan*c.detector.exposureSamples&&optics.solidAngles.size()==pixels&&
        optics.flat.size()==c.samplesPerScan&&static_cast<bool>(optics.staticSourceRaw)&&
        std::all_of(optics.nuisance.begin(),optics.nuisance.end(),[&](const auto& column){return column.size()==c.samplesPerScan;});
}
}
bool subsetCertificateArithmeticV2() noexcept {return certifiedArithmeticV2();}
bool certifiedSubsetRefusalV2(const std::vector<double>& target,
    const std::array<std::vector<double>,2>& columns,double targetError,
    const std::array<double,2>& columnErrors,double allowedResidualNorm) {
    return certifiedTwoColumnRefusal(target,columns,targetError,columnErrors,allowedResidualNorm);
}
void infer(const Optics& optics,ThermalReticleResultV1& result,V pointing,V up,ThermalReticleMovingDiagnosticV2* output) {
    ThermalReticleMovingDiagnosticV2 diagnostic;diagnostic.refusal=1;
    const auto& c=optics.policy;
    const auto validTime=[](double x){return std::isfinite(x)&&x>=0;};
    if(!validOptics(optics)||!result.valid||!result.complete||result.saturated||!result.features.empty()||
       result.measuredElectrons.size()!=c.samplesPerScan||!std::isfinite(result.varianceBoundElectrons2)||result.varianceBoundElectrons2<=0||
       !finite(pointing)||!(norm(pointing)>0)||!finite(up)||std::abs(norm(up)-1)>1e-6||
       !validTime(result.timeS)||!validTime(result.exposureStartTimeS)||!validTime(result.exposureEndTimeS)||!validTime(result.measurementTimeS)||
       !(result.exposureEndTimeS>result.exposureStartTimeS)||result.timeS<result.exposureEndTimeS||
       !std::all_of(result.measuredElectrons.begin(),result.measuredElectrons.end(),[](double x){return std::isfinite(x)&&x>=0;})) {
        if(output)*output=diagnostic;
        return;
    }
    Inverse inverse(optics,result,pointing,up,RegionalOperator::ConicFastV5);inverse.run(result);if(output)*output=inverse.diagnostic;
}
std::optional<std::array<std::vector<double>,2>> calibrateBasis(const Optics& optics,
    const ThermalReticleResultV1& clock,V pointing,V up,const Shape& q,unsigned branch) {
    if(!validOptics(optics)||clock.measuredElectrons.size()!=optics.policy.samplesPerScan||
       !std::isfinite(clock.varianceBoundElectrons2)||clock.varianceBoundElectrons2<=0||
       !finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6)return std::nullopt;
    Inverse inverse(optics,clock,pointing,up,RegionalOperator::LegacyV2);const auto values=inverse.basis(q,branch);
    if(!values)return std::nullopt;
    // Return canonical sky-relative columns even if a ground-relative contrast
    // was used as an exact internal change of linear coordinates.
    auto raw=values->raw;
    if(values->sourceRelativeGround)for(unsigned j=0;j<optics.policy.samplesPerScan;++j)
        raw[0][j]-=q[4]*q[5]/(optics.policy.detector.pixelPitchRad*optics.policy.detector.pixelPitchRad)*raw[1][j];
    return raw;
}
// Explicit versioned calibration of the production V4 regional operator.
std::optional<std::array<std::vector<double>,2>> calibrateBasisV4(const Optics& optics,
    const ThermalReticleResultV1& clock,V pointing,V up,const Shape& q,unsigned branch) {
    if(!validOptics(optics)||clock.measuredElectrons.size()!=optics.policy.samplesPerScan||
       !std::isfinite(clock.varianceBoundElectrons2)||clock.varianceBoundElectrons2<=0||
       !finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6)return std::nullopt;
    Inverse inverse(optics,clock,pointing,up,RegionalOperator::ConicV4);const auto values=inverse.basis(q,branch);
    if(!values)return std::nullopt;
    // Return canonical sky-relative columns even if a ground-relative contrast
    // was used as an exact internal change of linear coordinates.
    auto raw=values->raw;
    if(values->sourceRelativeGround)for(unsigned j=0;j<optics.policy.samplesPerScan;++j)
        raw[0][j]-=q[4]*q[5]/(optics.policy.detector.pixelPitchRad*optics.policy.detector.pixelPitchRad)*raw[1][j];
    return raw;
}

// Explicit version5 uses bounded fast moments and exact-order sparse source sums.
std::optional<std::array<std::vector<double>,2>> calibrateBasisV5(const Optics& optics,
    const ThermalReticleResultV1& clock,V pointing,V up,const Shape& q,unsigned branch) {
    if(!validOptics(optics)||clock.measuredElectrons.size()!=optics.policy.samplesPerScan||
       !std::isfinite(clock.varianceBoundElectrons2)||clock.varianceBoundElectrons2<=0||
       !finite(pointing)||norm(pointing)<=0||!finite(up)||std::abs(norm(up)-1)>1e-6)return std::nullopt;
    Inverse inverse(optics,clock,pointing,up,RegionalOperator::ConicFastV5);const auto values=inverse.basis(q,branch);
    if(!values)return std::nullopt;
    // Return canonical sky-relative columns even if a ground-relative contrast
    // was used as an exact internal change of linear coordinates.
    auto raw=values->raw;
    if(values->sourceRelativeGround)for(unsigned j=0;j<optics.policy.samplesPerScan;++j)
        raw[0][j]-=q[4]*q[5]/(optics.policy.detector.pixelPitchRad*optics.policy.detector.pixelPitchRad)*raw[1][j];
    return raw;
}
}
