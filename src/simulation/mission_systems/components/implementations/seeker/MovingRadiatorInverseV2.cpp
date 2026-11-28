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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <system_error>
#include <exception>
#include <unordered_map>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif
#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
#include <immintrin.h>
#define FSIM_MOVING_INV_HAS_AVX2 1
#else
#define FSIM_MOVING_INV_HAS_AVX2 0
#endif

namespace fsim::ms::components::moving_radiator_v2 {
namespace {
using namespace measured;

inline bool movingInvHasAvx2() noexcept {
#if FSIM_MOVING_INV_HAS_AVX2
    static const bool supported=__builtin_cpu_supports("avx2");
    return supported;
#else
    return false;
#endif
}

#if FSIM_MOVING_INV_HAS_AVX2
__attribute__((target("avx2")))
void accumulateSparseDiffAvx2(double& value,const double* __restrict fullPtr,const double* __restrict occPtr,
    const std::uint16_t* __restrict pixPtr,const double* __restrict solPtr,const double* __restrict wgtPtr,
    std::size_t beginIdx,std::size_t endIdx,double pixelArea,double invExposure) noexcept {
    const __m256d vZero=_mm256_setzero_pd();
    const __m256d vArea=_mm256_set1_pd(pixelArea);
    const __m256d vInv=_mm256_set1_pd(invExposure);
    std::size_t idx=beginIdx;
    for(;idx+8u<=endIdx;idx+=8u) {
        const unsigned p0=pixPtr[idx],p1=pixPtr[idx+1],p2=pixPtr[idx+2],p3=pixPtr[idx+3];
        const unsigned p4=pixPtr[idx+4],p5=pixPtr[idx+5],p6=pixPtr[idx+6],p7=pixPtr[idx+7];
        const __m256d fA=_mm256_sub_pd(
            _mm256_set_pd(fullPtr[p3],fullPtr[p2],fullPtr[p1],fullPtr[p0]),
            _mm256_set_pd(occPtr[p3],occPtr[p2],occPtr[p1],occPtr[p0]));
        const __m256d fB=_mm256_sub_pd(
            _mm256_set_pd(fullPtr[p7],fullPtr[p6],fullPtr[p5],fullPtr[p4]),
            _mm256_set_pd(occPtr[p7],occPtr[p6],occPtr[p5],occPtr[p4]));
        const int mA=_mm256_movemask_pd(_mm256_cmp_pd(fA,vZero,_CMP_NEQ_OQ));
        const int mB=_mm256_movemask_pd(_mm256_cmp_pd(fB,vZero,_CMP_NEQ_OQ));
        if((mA|mB)==0)continue;
        if(mA!=0) {
            const __m256d tA=_mm256_mul_pd(_mm256_div_pd(_mm256_mul_pd(_mm256_mul_pd(fA,_mm256_loadu_pd(solPtr+idx)),_mm256_loadu_pd(wgtPtr+idx)),vArea),vInv);
            alignas(32) double outA[4];
            _mm256_store_pd(outA,tA);
            if(mA&1)value+=outA[0];
            if(mA&2)value+=outA[1];
            if(mA&4)value+=outA[2];
            if(mA&8)value+=outA[3];
        }
        if(mB!=0) {
            const __m256d tB=_mm256_mul_pd(_mm256_div_pd(_mm256_mul_pd(_mm256_mul_pd(fB,_mm256_loadu_pd(solPtr+idx+4)),_mm256_loadu_pd(wgtPtr+idx+4)),vArea),vInv);
            alignas(32) double outB[4];
            _mm256_store_pd(outB,tB);
            if(mB&1)value+=outB[0];
            if(mB&2)value+=outB[1];
            if(mB&4)value+=outB[2];
            if(mB&8)value+=outB[3];
        }
    }
    for(;idx<endIdx;++idx) {
        const auto pix=pixPtr[idx];
        const double field=fullPtr[pix]-occPtr[pix];
        if(field==0.0)continue;
        value+=field*solPtr[idx]*wgtPtr[idx]/pixelArea*invExposure;
    }
}
#endif

inline void accumulateSparseDiffV5(double& value,const double* fullPtr,const double* occPtr,
    const std::uint16_t* pixPtr,const double* solPtr,const double* wgtPtr,
    std::size_t beginIdx,std::size_t endIdx,double pixelArea,double invExposure) noexcept {
#if FSIM_MOVING_INV_HAS_AVX2
    if(movingInvHasAvx2()) {
        accumulateSparseDiffAvx2(value,fullPtr,occPtr,pixPtr,solPtr,wgtPtr,beginIdx,endIdx,pixelArea,invExposure);
        return;
    }
#endif
    for(std::size_t idx=beginIdx;idx<endIdx;++idx) {
        const auto pix=pixPtr[idx];
        const double field=fullPtr[pix]-occPtr[pix];
        if(field==0.0)continue;
        value+=field*solPtr[idx]*wgtPtr[idx]/pixelArea*invExposure;
    }
}

struct LocalProfileV2 {
    using Clock=std::chrono::steady_clock;
    enum Part {FullGround,GroundWave,Lookup,Prepare,BasisWork,Regional,Qp,Trust,Information,ModeWork,SubsetWork,PartCount};
    bool enabled=false;Clock::time_point began{};
    std::array<std::size_t,PartCount> calls{};std::array<double,PartCount> micros{};
    std::size_t probes=0,hits=0,prepareHits=0,groundHits=0,faces=0,iterations=0,trials=0,modes=0,admittedModes=0;
    std::size_t subsetChecks=0,subsetRefusals=0,subsetFallbacks=0,phaseHits=0;
    std::size_t regionalAccelerated=0,regionalNative=0;double groundAttemptUs=0,groundOriginalUs=0;
    std::size_t conicRequests=0,conicBanks=0,conicFields=0,conicFallbacks=0;
    std::size_t parallelBatches=0,parallelFields=0,parallelWorkersPeak=0;
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
            stage,readTime,captureTime,us,costJson,coarse?1:0,calls[BasisWork],calls[Regional]-parallelBatches+parallelFields,probes,calls[Qp],faces,iterations,trials,modes,admittedModes,
            micros[FullGround],micros[Regional],micros[Qp],micros[Information],micros[Lookup],micros[Prepare],groundHits,prepareHits,subsetChecks,subsetRefusals,subsetFallbacks,phaseHits);
        std::fprintf(stderr,"MOVING_GROUND_PROFILE {\"stage\":\"%s\",\"read\":%.17g,\"accelerated\":%zu,\"native\":%zu,\"attempt_us\":%.3f,\"original_us\":%.3f,\"at\":%zu,\"at_repeats\":%zu,\"rules\":%zu,\"recursions\":%zu,\"max_depth\":%u,\"active_rows\":%zu,\"memo_queries\":%zu,\"memo_hits\":%zu,\"memo_capacity_misses\":%zu,\"memo_max_nodes\":%zu,\"roots_built\":%zu,\"y_hits\":%zu,\"y_evaluations\":%zu,\"probability_hits\":%zu,\"probability_evaluations\":%zu,\"fallback_reasons\":[%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu]}\n",
            stage,readTime,regionalAccelerated,regionalNative,groundAttemptUs,groundOriginalUs,groundWork.atEvaluations,groundWork.repeatedAtSamples,
            groundWork.ruleEvaluations,groundWork.adaptiveCalls,groundWork.maxRefinementDepth,groundWork.activeRowVisits,
            groundWork.memo.queries,groundWork.memo.hits,groundWork.memo.capacityMisses,groundWork.memo.maxNodes,groundWork.memo.rootsBuilt,
            groundWork.memo.yHits,groundWork.memo.yEvaluations,groundWork.memo.probabilityHits,groundWork.memo.probabilityEvaluations,
            fallbackReasons[0],fallbackReasons[1],fallbackReasons[2],fallbackReasons[3],fallbackReasons[4],fallbackReasons[5],fallbackReasons[6],fallbackReasons[7]);
        std::fprintf(stderr,"MOVING_CONIC_V%u_PROFILE {\"stage\":\"%s\",\"read\":%.17g,\"requests\":%zu,\"banks\":%zu,\"fields\":%zu,\"fallbacks\":%zu,\"setup_us\":%.3f,\"render_us\":%.3f,\"maximum_error\":%.17g}\n",
            conicVersion,stage,readTime,conicRequests,conicBanks,conicFields,conicFallbacks,conicSetupUs,conicRenderUs,conicMaximumError);
        if(conicVersion==5)std::fprintf(stderr,
            "MOVING_PARALLEL_PHASE_PROFILE {\"stage\":\"%s\",\"read\":%.17g,\"batches\":%zu,\"fields\":%zu,\"peak_workers\":%zu,\"render_us_kind\":\"sum_of_worker_elapsed_us\",\"detailed_ground_work\":\"serial_requests_only\"}\n",
            stage,readTime,parallelBatches,parallelFields,parallelWorkersPeak);
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
    // Single initial Householder QR reduces the (count + 7) x 7 least-squares
    // system to an exact orthogonal 7 x 7 upper-triangular system R * x = cTop
    // without squaring the condition number: ||A*x - b||^2 = ||R*x - cTop||^2 + const.
    std::array<Shape,7> R{};
    Shape cTop{};
    {
        auto workA=design;
        auto workB=target;
        std::vector<double> reflector(rows,0.);
        for(unsigned col=0;col<7;++col) {
            double colNorm=0;
            for(std::size_t row=col;row<rows;++row)colNorm=std::hypot(colNorm,workA[row][col]);
            if(!(colNorm>0)||!std::isfinite(colNorm))return false;
            const double diagonal=-std::copysign(colNorm,workA[col][col]);
            for(std::size_t row=col;row<rows;++row)reflector[row]=workA[row][col];
            reflector[col]-=diagonal;
            double norm2=0;for(std::size_t row=col;row<rows;++row)norm2+=reflector[row]*reflector[row];
            if(!(norm2>0)||!std::isfinite(norm2))return false;
            const double beta=2/norm2;
            for(unsigned dst=col+1;dst<7;++dst) {
                double d=0;for(std::size_t row=col;row<rows;++row)d+=reflector[row]*workA[row][dst];
                for(std::size_t row=col;row<rows;++row)workA[row][dst]-=beta*reflector[row]*d;
            }
            double db=0;for(std::size_t row=col;row<rows;++row)db+=reflector[row]*workB[row];
            for(std::size_t row=col;row<rows;++row)workB[row]-=beta*reflector[row]*db;
            workA[col][col]=diagonal;
            for(unsigned dst=col;dst<7;++dst)R[col][dst]=workA[col][dst];
            cTop[col]=workB[col];
        }
    }
    double bestCost=std::numeric_limits<double>::infinity();Shape selected{};
    for(unsigned face=0;face<faceCount;++face) {
        if(profile&&profile->enabled)++profile->faces;
        unsigned encoded=face,freeCount=0;std::array<unsigned,7> free{};Shape step{};
        Shape rhs=cTop;
        for(unsigned axis=0;axis<7;++axis) {
            const auto state=encoded%3;encoded/=3;
            if(state==0)free[freeCount++]=axis;
            else {
                step[axis]=state==1?scaledLow[axis]:scaledHigh[axis];
                for(unsigned row=0;row<=axis;++row)rhs[row]-=R[row][axis]*step[axis];
            }
        }
        std::array<Shape,7> matrix{};
        for(unsigned row=0;row<7;++row)for(unsigned column=0;column<freeCount;++column)
            matrix[row][column]=R[row][free[column]];
        bool valid=true;
        if(face>0) {
            for(unsigned column=0;column<freeCount&&valid;++column) {
                unsigned pivot=column;double largest=-1;
                for(unsigned candidate=column;candidate<freeCount;++candidate) {
                    double sum2=0.,maxAbs=0.;
                    for(unsigned row=column;row<7;++row) {
                        const double v=std::abs(matrix[row][candidate]);
                        if(v>maxAbs)maxAbs=v;
                        sum2+=v*v;
                    }
                    double length=0.;
                    if(maxAbs>0.) {
                        if(maxAbs>1e-150&&maxAbs<1e150)length=std::sqrt(sum2);
                        else {
                            double scaled=0.;
                            for(unsigned row=column;row<7;++row){const double t=matrix[row][candidate]/maxAbs;scaled+=t*t;}
                            length=maxAbs*std::sqrt(scaled);
                        }
                    }
                    if(length>largest){largest=length;pivot=candidate;}
                }
                if(!(largest>0)||!std::isfinite(largest)){valid=false;break;}
                if(pivot!=column) {
                    std::swap(free[pivot],free[column]);
                    for(unsigned row=0;row<7;++row)std::swap(matrix[row][pivot],matrix[row][column]);
                }
                const double diagonal=-std::copysign(largest,matrix[column][column]);
                Shape reflector{};
                for(unsigned row=column;row<7;++row)reflector[row]=matrix[row][column];
                reflector[column]-=diagonal;
                double norm2=0;for(unsigned row=column;row<7;++row)norm2+=reflector[row]*reflector[row];
                if(!(norm2>0)||!std::isfinite(norm2)){valid=false;break;}
                const double beta=2/norm2;
                for(unsigned destination=column;destination<freeCount;++destination) {
                    double d=0;for(unsigned row=column;row<7;++row)d+=reflector[row]*matrix[row][destination];
                    for(unsigned row=column;row<7;++row)matrix[row][destination]-=beta*reflector[row]*d;
                }
                double d=0;for(unsigned row=column;row<7;++row)d+=reflector[row]*rhs[row];
                for(unsigned row=column;row<7;++row)rhs[row]-=beta*reflector[row]*d;
                matrix[column][column]=diagonal;
            }
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
        for(unsigned row=0;row<7;++row) {
            double error=-cTop[row];for(unsigned column=row;column<7;++column)error+=R[row][column]*step[column];
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

// Private per-Inverse worker pool for pure-optics V5 phase batches.
// Per-worker cache-line-padded slots with hybrid short spin-then-park avoid
// OS condition-variable wakeups across thousands of tight sub-millisecond
// prepare()/regional() calls, never wake idle helpers, and strictly join all
// worker threads before infer() returns.
class PhasePoolV5 {
public:
    static constexpr unsigned maximumHelpers=15u;
    ~PhasePoolV5() {
        for(std::size_t i=0;i<threads_.size();++i) {
            slots_[i].stop.store(true,std::memory_order_seq_cst);
            {
                std::lock_guard<std::mutex> lock(slots_[i].mutex);
            }
            slots_[i].wake.notify_one();
        }
        for(auto& thread:threads_)if(thread.joinable())thread.join();
    }
    bool syncEnvironment() noexcept {
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        const unsigned curCsr=_mm_getcsr();
#else
        const unsigned curCsr=0u;
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
        unsigned short curCw=0u;__asm__ __volatile__("fnstcw %0":"=m"(curCw));
#else
        const unsigned short curCw=0u;
#endif
        if(envValid_&&curCsr==csr_&&curCw==cw_)return true;
        if(std::fegetenv(&env_)!=0)return false;
        csr_=curCsr;
        cw_=curCw;
        envValid_=true;
        ++envEpoch_;
        return true;
    }
    template<typename Fn>
    void run(unsigned workers,Fn& fn) {
        if(workers<=1){fn(0);return;}
        const unsigned helpers=std::min<unsigned>(workers-1u,maximumHelpers);
        while(threads_.size()<helpers) {
            const unsigned idx=static_cast<unsigned>(threads_.size());
            try {threads_.emplace_back(&PhasePoolV5::loop,this,idx);}
            catch(const std::system_error&){break;}
        }
        if(threads_.empty()){fn(0);return;}
        const unsigned useHelpers=std::min<unsigned>(helpers,static_cast<unsigned>(threads_.size()));
        const auto invoke=[](const void* context,unsigned id) noexcept {(*static_cast<const Fn*>(context))(id);};
        fn_=invoke;
        context_=&fn;
        std::array<std::uint64_t,maximumHelpers> targets{};
        for(unsigned i=0;i<useHelpers;++i) {
            auto& slot=slots_[i];
            const std::uint64_t nextGen=slot.workGen.load(std::memory_order_relaxed)+1u;
            targets[i]=nextGen;
            slot.workGen.store(nextGen,std::memory_order_seq_cst);
            if(slot.parked.load(std::memory_order_seq_cst)) {
                {
                    std::lock_guard<std::mutex> lock(slot.mutex);
                }
                slot.wake.notify_one();
            }
        }
        fn(0);
        for(unsigned i=0;i<useHelpers;++i) {
            auto& slot=slots_[i];
            const std::uint64_t target=targets[i];
            unsigned spin=0;
            while(slot.doneGen.load(std::memory_order_acquire)!=target) {
                if(spin<4096u) {
                    ++spin;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                    _mm_pause();
#endif
                } else {
                    std::this_thread::yield();
                }
            }
        }
    }
private:
    struct alignas(64) WorkerSlot {
        std::mutex mutex;
        std::condition_variable wake;
        std::atomic<std::uint64_t> workGen{0};
        std::atomic<std::uint64_t> doneGen{0};
        std::atomic<bool> parked{false};
        std::atomic<bool> stop{false};
    };
    void loop(unsigned idx) noexcept {
        auto& slot=slots_[idx];
        const unsigned id=idx+1u;
        std::uint64_t seen=0;
        std::uint64_t appliedEnvEpoch=0;
        for(;;) {
            std::uint64_t gen=slot.workGen.load(std::memory_order_acquire);
            if(gen==seen&&!slot.stop.load(std::memory_order_acquire)) {
                for(unsigned spin=0;spin<8192u;++spin) {
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                    _mm_pause();
#endif
                    gen=slot.workGen.load(std::memory_order_acquire);
                    if(gen!=seen||slot.stop.load(std::memory_order_acquire))break;
                }
                if(gen==seen&&!slot.stop.load(std::memory_order_acquire)) {
                    std::unique_lock<std::mutex> lock(slot.mutex);
                    slot.parked.store(true,std::memory_order_seq_cst);
                    slot.wake.wait(lock,[&]{
                        return slot.stop.load(std::memory_order_seq_cst)||
                            slot.workGen.load(std::memory_order_seq_cst)!=seen;
                    });
                    slot.parked.store(false,std::memory_order_relaxed);
                    gen=slot.workGen.load(std::memory_order_acquire);
                }
            }
            if(slot.stop.load(std::memory_order_acquire))return;
            if(gen==seen)continue;
            seen=gen;
            if(envValid_&&appliedEnvEpoch!=envEpoch_) {
                if(std::fesetenv(&env_)==0) {
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
                    _mm_setcsr(csr_);
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
                    __asm__ __volatile__("fldcw %0"::"m"(cw_));
#endif
                    appliedEnvEpoch=envEpoch_;
                }
            }
            if(fn_)fn_(context_,id);
            slot.doneGen.store(seen,std::memory_order_release);
        }
    }
    std::array<WorkerSlot,maximumHelpers> slots_{};
    std::vector<std::thread> threads_;
    void (*fn_)(const void*,unsigned)=nullptr;
    const void* context_=nullptr;
    bool envValid_=false;
    std::fenv_t env_{};
    unsigned csr_=0u;
    unsigned short cw_=0u;
    std::uint64_t envEpoch_=0u;
};

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
    std::unordered_map<std::uint64_t,std::size_t> groundIndex;
    struct GroundFieldEntry {double cutoff=0;std::vector<double> field;};
    std::vector<GroundFieldEntry> groundFieldCache;
    std::optional<double> nativeGroundCutoff;std::vector<double> nativeGroundField;
    std::optional<Shape> lastShape;unsigned lastBranch=0;std::optional<Basis> lastBasis;
    std::optional<Shape> regionalShape;unsigned regionalBranch=0;
    std::vector<double> regionalPhases;std::vector<bool> regionalKnown;
    // At most one fixed-optics bank; signed cutoff identity controls reset.
    std::optional<double> conicCutoff;bool conicBankAttempted=false;
    std::unique_ptr<conic_moments_v4::Bank> conicBank;
    // Private pure-optics concurrency, independent of World stepping/RNG.
    static constexpr unsigned maximumPhaseWorkersV5=16;
    static constexpr std::size_t maximumPhaseBanksV5=16;
    static constexpr std::size_t maximumModePhaseBanksV5=128;
    std::size_t activeMaxPhaseBanksV5() const noexcept {return modeCache?maximumModePhaseBanksV5:maximumPhaseBanksV5;}
    struct CachedPhaseBankV5 {
        double cutoff=0;
        std::shared_ptr<conic_moments_v4::Bank> bank;
        std::uint64_t warmedLo=0,warmedHi=0;
        bool columnWarmed(unsigned col) const noexcept {
            if(col<64u)return (warmedLo&(1ull<<col))!=0ull;
            if(col<128u)return (warmedHi&(1ull<<(col-64u)))!=0ull;
            return false;
        }
        void markWarmedRange(unsigned colBegin,unsigned colEnd) noexcept {
            for(unsigned col=colBegin;col<colEnd;++col) {
                if(col<64u)warmedLo|=(1ull<<col);
                else if(col<128u)warmedHi|=(1ull<<(col-64u));
            }
        }
    };
    std::vector<CachedPhaseBankV5> phaseBankCacheV5;
    PhasePoolV5 phasePoolV5;
    std::array<conic_moments_v4::FieldWorkspaceV5,maximumPhaseWorkersV5> phaseWorkspacesV5;
    conic_moments_v4::FieldWorkspaceV5 serialWorkspaceV5;
    bool sparseMaskReadyV5=false,powerOfTwoExposureV5=false;
    double invExposureV5=0;
    std::vector<std::size_t> sparseOffsetV5;
    std::vector<std::uint16_t> sparsePixelV5;
    std::vector<double> sparseSolidV5,sparseWeightV5;
    std::unique_ptr<conic_moments_v4::GaussianMemoV5> gaussianMemoV5;
    bool subsetPrepared=false;std::vector<SubsetMapV2> subsetMaps;
    double backgroundPhaseBound=infinity,polePhaseBound=infinity,backgroundNormBound=infinity,projectionRoundFactor=infinity;
    bool modeCache=false;std::vector<SourceWave> sources;
    std::optional<source_wave_sum_v5::OpticsV5> sourceWaveOpticsV5;
    struct Trajectory {
        std::array<double,6> shape{};unsigned branch=0;RayPath path;
        std::vector<double> x,y;
        std::vector<double> levelMin,levelMax;
        std::vector<double> subexposureSource;
        double minimumLevel=infinity,maximumLevel=-infinity;
        std::vector<double> rawSource;
        std::vector<double> projectedSource;
        std::vector<conic_moments_v4::RegionMomentsV5> precomputedRegions;
    };
    struct TrajKeyV5 {
        std::array<std::uint64_t,6> bits{};
        unsigned branch=0;
        bool operator==(const TrajKeyV5& o) const noexcept {return branch==o.branch&&bits==o.bits;}
    };
    struct TrajKeyHashV5 {
        std::size_t operator()(const TrajKeyV5& k) const noexcept {
            std::uint64_t h=k.branch*0x9e3779b97f4a7c15ull;
            for(const auto b:k.bits) {
                std::uint64_t x=b+0x9e3779b97f4a7c15ull;
                x=(x^(x>>30))*0xbf58476d1ce4e5b9ull;
                x=(x^(x>>27))*0x94d049bb133111ebull;
                h^=x+(h<<6)+(h>>2);
            }
            return static_cast<std::size_t>(h);
        }
    };
    static TrajKeyV5 makeTrajKeyV5(const std::array<double,6>& key,unsigned branch) noexcept {
        TrajKeyV5 out;out.branch=branch;
        for(unsigned i=0;i<6;++i)std::memcpy(&out.bits[i],&key[i],sizeof(std::uint64_t));
        return out;
    }
    struct GroundSubCacheV5 {
        double cutoff=0;
        std::vector<double> subexposure;
    };
    std::vector<GroundSubCacheV5> groundSubCacheV5;
    std::vector<double> rowFullWaveV5;
    struct JacWorkerScratchV5 {
        Trajectory traj;
        std::vector<double> raw0,u,v;
    };
    std::array<JacWorkerScratchV5,maximumPhaseWorkersV5> jacScratchV5;
    static constexpr std::size_t maximumTrajectoryCacheV5=16;
    std::vector<std::unique_ptr<Trajectory>> trajectoryCache;
    static constexpr std::size_t maximumModeTrajectoriesV5=1024;
    std::vector<std::unique_ptr<Trajectory>> modeTrajectories;
    std::unordered_map<TrajKeyV5,Trajectory*,TrajKeyHashV5> modeTrajMap;
    std::optional<Shape> priorStart;
    double nullCost=infinity,nullCutoff=0,nullAmplitude=0;
    std::size_t maximumGrounds;
    struct Null {double cutoff,cost,amplitude;};std::vector<Null> nullMinima;
    Inverse(const Optics& o,const ThermalReticleResultV1& input,V pointing,V ownUp,RegionalOperator operation,
            const ThermalReticleMeasuredPriorV2* prior=nullptr):
        optics(o),c(o.policy),p(c.detector),measured(input),regionalOperator(operation),forward(unit(pointing)),up(unit(ownUp)),count(c.samplesPerScan),
        pixels(static_cast<std::size_t>(p.width)*p.height),alpha(-std::expm1(-1/(c.frequencyHz*count*p.pixelResponseTimeS))),
        pixelArea(p.pixelPitchRad*p.pixelPitchRad),variance(input.varianceBoundElectrons2),
        halfX(.5*p.width*p.pixelPitchRad),halfY(.5*p.height*p.pixelPitchRad),
        low(0),high(0),cutStep(0),angularLattice(p.pixelPitchRad/std::ldexp(1.,18)),cutLattice(0),first(0),last(0),
        data(input.measuredElectrons),dataNorm(0),ut(0),tt(0),uu(0),det(0),maximumGrounds(0) {
        profile.conicVersion=regionalOperator==RegionalOperator::ConicFastV5?5u:
            regionalOperator==RegionalOperator::ConicV4?4u:2u;
        // Explicit V5 only. LegacyV2 and ConicV4 keep the old dense source operator.
        if(regionalOperator==RegionalOperator::ConicFastV5) {
            sourceWaveOpticsV5.emplace(optics.solidAngles,optics.timeWeights,p.width,p.height,count,p.exposureSamples,pixelArea);
            powerOfTwoExposureV5=p.exposureSamples>0&&(p.exposureSamples&(p.exposureSamples-1u))==0;
            invExposureV5=powerOfTwoExposureV5?(1.0/static_cast<double>(p.exposureSamples)):0.0;
            rowFullWaveV5.assign(static_cast<std::size_t>(p.height)*count,0.0);
            for(unsigned row=0;row<p.height;++row) {
                double* dst=rowFullWaveV5.data()+static_cast<std::size_t>(row)*count;
                const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
                for(unsigned col=0;col<p.width;++col) {
                    const std::size_t i=rowBase+col;
                    const double s=optics.solidAngles[i]/pixelArea;
                    const double* w=optics.weights.data()+i*count;
                    for(unsigned j=0;j<count;++j)dst[j]+=s*w[j];
                }
            }
            if(pixels<=65535u&&
               std::all_of(optics.solidAngles.begin(),optics.solidAngles.end(),[](double v){return std::isfinite(v)&&v>=0;})&&
               std::all_of(optics.timeWeights.begin(),optics.timeWeights.end(),[](double v){return std::isfinite(v)&&v>=0;})) {
                const std::size_t subexposures=static_cast<std::size_t>(count)*p.exposureSamples;
                sparseOffsetV5.resize(subexposures+1,0);
                std::size_t totalNonzero=0;
                for(std::size_t ti=0;ti<subexposures;++ti) {
                    const double* w=optics.timeWeights.data()+ti*pixels;
                    for(std::size_t i=0;i<pixels;++i)if(w[i]!=0.0)++totalNonzero;
                }
                sparsePixelV5.reserve(totalNonzero);
                sparseSolidV5.reserve(totalNonzero);
                sparseWeightV5.reserve(totalNonzero);
                for(std::size_t ti=0;ti<subexposures;++ti) {
                    sparseOffsetV5[ti]=sparsePixelV5.size();
                    const double* w=optics.timeWeights.data()+ti*pixels;
                    for(std::size_t i=0;i<pixels;++i)if(w[i]!=0.0) {
                        sparsePixelV5.push_back(static_cast<std::uint16_t>(i));
                        sparseSolidV5.push_back(optics.solidAngles[i]);
                        sparseWeightV5.push_back(w[i]);
                    }
                }
                sparseOffsetV5[subexposures]=sparsePixelV5.size();
                sparseMaskReadyV5=true;
            }
        }
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
        if(prior&&std::isfinite(prior->captureTimeS)&&prior->captureTimeS>=0&&
           std::isfinite(prior->availableTimeS)&&prior->availableTimeS>=prior->captureTimeS&&
           prior->captureTimeS<input.measurementTimeS&&prior->availableTimeS<=input.exposureStartTimeS&&
           finite(prior->direction)&&norm(prior->direction)>0&&finite(prior->directionRate)&&
           std::isfinite(prior->widthRad)&&prior->widthRad>0&&prior->widthRad<=2*halfX&&
           std::isfinite(prior->heightRad)&&prior->heightRad>0&&prior->heightRad<=2*halfY&&
           std::isfinite(prior->cutoff)&&prior->cutoff>=low&&prior->cutoff<=high) {
            const auto projectRay=[&](double epochS)->std::optional<std::array<double,2>> {
                const auto predicted=add(prior->direction,scale(prior->directionRate,epochS-prior->captureTimeS));
                if(!finite(predicted)||!(norm(predicted)>0))return std::nullopt;
                const auto dir=unit(predicted);const double front=dot(forward,dir);
                const double px=std::atan2(dot(right,dir),front),py=std::atan2(dot(vertical,dir),front);
                if(!(front>0)||std::abs(px)>halfX||std::abs(py)>halfY)return std::nullopt;
                return std::array<double,2>{{px,py}};
            };
            const auto pFirst=projectRay(first),pLast=projectRay(last);
            if(pFirst&&pLast) {
                const Shape candidate=quantize({{(*pFirst)[0],(*pFirst)[1],(*pLast)[0],(*pLast)[1],
                    prior->widthRad,prior->heightRad,prior->cutoff}});
                if(path(candidate,0))priorStart=candidate;
            }
        }
        // One retained measured static proposal plus2branches*5null basins;
        //64iterations*2cut derivatives+12trials per independent start.
        // Selected plus at most11local-basin covariances add2each before a finite snapshot. Every snapshot
        // cutoff can add its two neighbours, shared by all branch/reflections.
        // Up to5explicit initial cutoffs may need their canonical lattice
        // representation in addition to the source static-proposal value.
        // A valid causal measured prior adds at most 3 roll-rotated extent
        // starts across 2 cutoffs (priorCutoff and nullCutoff: 6 independent
        // starts) and 1 explicit initial cutoff; no-prior count is exact old.
        const std::size_t extraStarts=priorStart?6u:0u;
        const std::size_t extraCutoffs=priorStart?1u:0u;
        maximumGrounds=3*(coarse+36*minima+(5+extraCutoffs)+(11+extraStarts)*(1+64*(2+12))+2*(1+11+extraStarts));
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
        for(std::size_t idx=0;idx<groundFieldCache.size();++idx) {
            if(exactScalar(groundFieldCache[idx].cutoff,cutoff)) {
                if(idx!=0)std::rotate(groundFieldCache.begin(),groundFieldCache.begin()+static_cast<std::ptrdiff_t>(idx),
                    groundFieldCache.begin()+static_cast<std::ptrdiff_t>(idx+1));
                if(profile.enabled)++profile.groundHits;
                return &groundFieldCache[0].field;
            }
        }
        auto field=ThermalReticleReceiverV1::unitGroundFieldV1(p,forward,up,cutoff);if(!field)return nullptr;
        const std::size_t maxFields=modeCache?maximumModePhaseBanksV5:4u;
        if(groundFieldCache.size()>=maxFields)groundFieldCache.pop_back();
        groundFieldCache.insert(groundFieldCache.begin(),GroundFieldEntry{cutoff,std::move(*field)});
        return &groundFieldCache[0].field;
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
        if(!rowFullWaveV5.empty()&&field.size()==pixels) {
            const double* fPtr=field.data();
            const double* rwPtr=rowFullWaveV5.data();
            for(unsigned row=0;row<p.height;++row) {
                const std::size_t rowBase=static_cast<std::size_t>(row)*p.width;
                const double* fRow=fPtr+rowBase;
                const double f0=fRow[0];
                bool allEqual=true;
                for(unsigned col=1;col<p.width;++col)if(fRow[col]!=f0){allEqual=false;break;}
                if(allEqual) {
                    if(f0==0.0)continue;
                    const double* rw=rwPtr+static_cast<std::size_t>(row)*count;
                    if(f0==1.0) {
                        for(unsigned j=0;j<count;++j)result[j]+=rw[j];
                    } else {
                        for(unsigned j=0;j<count;++j)result[j]+=f0*rw[j];
                    }
                    continue;
                }
                for(unsigned col=0;col<p.width;++col) {
                    const double f=fRow[col];
                    if(f==0.0)continue;
                    const std::size_t i=rowBase+col;
                    const double s=f*optics.solidAngles[i]/pixelArea;
                    const double* w=optics.weights.data()+i*count;
                    for(unsigned j=0;j<count;++j)result[j]+=s*w[j];
                }
            }
            return filter(std::move(result),alpha);
        }
        for(std::size_t i=0;i<field.size();++i) {
            const double f=field[i];
            if(f==0)continue;
            const double s=f*optics.solidAngles[i]/pixelArea;
            const double* w=optics.weights.data()+i*count;
            for(unsigned j=0;j<count;++j)result[j]+=s*w[j];
        }
        return filter(std::move(result),alpha);
    }
    std::optional<std::array<double,2>> rawSky(const std::vector<double>& component) const {
        if(!(uu>0)||!(tt>=0)||(tt>0&&!(det>128*epsilon*uu*tt)))return std::nullopt;
        auto remaining=measured.measuredElectrons;
        for(unsigned j=0;j<count;++j)remaining[j]-=dark[j];
        const auto coefficient=[&](const std::vector<double>& y){return tt>0?(inner(y,uniform)*tt-inner(y,transient)*ut)/det:inner(y,uniform)/uu;};
        return std::array<double,2>{{coefficient(remaining),coefficient(component)}};
    }
    static std::uint64_t cutoffBits(double cutoff) noexcept {
        std::uint64_t bits=0;std::memcpy(&bits,&cutoff,sizeof(bits));return bits;
    }
    const Ground* calibration(double cutoff) {
        const auto key=cutoffBits(cutoff);
        {LocalTimerV2 timer(&profile,LocalProfileV2::Lookup);
        if(profile.enabled)++profile.probes;
        if(const auto found=groundIndex.find(key);found!=groundIndex.end()){if(profile.enabled)++profile.hits;return &grounds[found->second];}}
        if(grounds.size()>=maximumGrounds){diagnostic.refusal=21;return nullptr;}
        const auto* field=groundField(cutoff);if(!field){diagnostic.refusal=22;return nullptr;}
        Ground item;item.cutoff=cutoff;item.raw=wave(*field);item.wave=item.raw;project(item.wave,optics.nuisance);
        groundIndex.emplace(key,grounds.size());
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
        if(modeCache) {
            const auto tk=makeTrajKeyV5(key,branch);
            if(const auto it=modeTrajMap.find(tk);it!=modeTrajMap.end()) {
                if(profile.enabled)++profile.prepareHits;
                return it->second;
            }
        } else {
            for(std::size_t idx=0;idx<trajectoryCache.size();++idx) {
                if(exactCoordinates(trajectoryCache[idx]->shape,key)&&trajectoryCache[idx]->branch==branch) {
                    if(idx>0) {
                        const auto offset=static_cast<std::ptrdiff_t>(idx);
                        std::rotate(trajectoryCache.begin(),trajectoryCache.begin()+offset,trajectoryCache.begin()+offset+1);
                    }
                    if(profile.enabled)++profile.prepareHits;
                    return trajectoryCache.front().get();
                }
            }
        }
        auto& activeTrajCache=modeCache?modeTrajectories:trajectoryCache;
        const auto ray=path(q,branch);if(!ray)return nullptr;
        const SourceWave* cachedSource=nullptr;
        if(modeCache)for(const auto& entry:sources)if(exactCoordinates(entry.shape,key)&&entry.branch==branch){cachedSource=&entry;break;}
        const std::size_t limit=modeCache?maximumModeTrajectoriesV5:maximumTrajectoryCacheV5;
        std::unique_ptr<Trajectory> recycled;
        if(activeTrajCache.size()>=limit) {
            if(modeCache)modeTrajMap.erase(makeTrajKeyV5(activeTrajCache.back()->shape,activeTrajCache.back()->branch));
            recycled=std::move(activeTrajCache.back());
            activeTrajCache.pop_back();
        } else {
            recycled=std::make_unique<Trajectory>();
        }
        Trajectory& item=*recycled;
        item.precomputedRegions.clear();
        item.shape=key;item.branch=branch;item.path=*ray;
        item.minimumLevel=infinity;item.maximumLevel=-infinity;
        const auto samples=static_cast<std::size_t>(count)*p.exposureSamples;
        item.x.resize(samples);item.y.resize(samples);
        item.levelMin.resize(samples);item.levelMax.resize(samples);
        item.subexposureSource.assign(samples,0.0);
        if(!cachedSource)item.rawSource.assign(count,0);
        else item.rawSource=cachedSource->raw;
        const double sigma=p.psfSigmaRad;
        const bool nearestArith=certifiedArithmeticV2();
        const bool useFastSourceSum=sourceWaveOpticsV5&&sourceWaveOpticsV5->eligibleOptics()&&nearestArith;
        const auto gaussianPrimBitExact=[&](double z) noexcept -> double {
            if(nearestArith) {
                if(z>=9.5)return z;
                if(z<=-38.5)return 0.0;
            }
            return z*.5*std::erfc(-z/std::sqrt(2.))+std::exp(-.5*z*z)/std::sqrt(2.*3.14159265358979323846);
        };
        const auto fillAxisProb=[&](double* prob,int i0,int i1,unsigned dim,double centre,double extent,double rMin,double rMax) {
            if(i0>i1)return;
            if(extent==0.0) {
                const double invSqrt2Sigma=1.0/(std::sqrt(2.)*sigma);
                bool hasPrev=false;double prevHi=0.0,prevErfc=0.0;
                for(int idx=i0;idx<=i1;++idx) {
                    const unsigned u=static_cast<unsigned>(idx);
                    const double lo=(u-.5*dim)*p.pixelPitchRad,hi=lo+p.pixelPitchRad;
                    if(lo>rMax||hi<rMin){hasPrev=false;continue;}
                    const double eLo=(hasPrev&&lo==prevHi)?prevErfc:std::erfc((centre-lo)*invSqrt2Sigma);
                    const double eHi=std::erfc((centre-hi)*invSqrt2Sigma);
                    hasPrev=true;prevHi=hi;prevErfc=eHi;
                    prob[u]=.5*(eHi-eLo);
                }
            } else {
                const double a=centre-.5*extent,b=centre+.5*extent;
                const auto integratedCdf=[&](double edge) noexcept -> double {
                    const double za=(edge-a)/sigma,zb=(edge-b)/sigma;
                    if(nearestArith&&za>=9.5&&zb<=-9.5)return sigma*za;
                    return sigma*(gaussianPrimBitExact(za)-gaussianPrimBitExact(zb));
                };
                bool hasPrev=false;double prevHi=0.0,prevInt=0.0;
                for(int idx=i0;idx<=i1;++idx) {
                    const unsigned u=static_cast<unsigned>(idx);
                    const double lo=(u-.5*dim)*p.pixelPitchRad,hi=lo+p.pixelPitchRad;
                    if(lo>rMax||hi<rMin){hasPrev=false;continue;}
                    const double intLo=(hasPrev&&lo==prevHi)?prevInt:integratedCdf(lo);
                    const double intHi=integratedCdf(hi);
                    hasPrev=true;prevHi=hi;prevInt=intHi;
                    prob[u]=std::clamp(intHi-intLo,0.,std::min(b-a,hi-lo))/extent;
                }
            }
        };
        const bool needSubSource=!cachedSource||!ray->stationary;
        const auto processPhase=[&](unsigned j,double* xProb,double* yProb,double& localMin,double& localMax)->bool {
            double rawAcc=0.0;
            for(unsigned k=0;k<p.exposureSamples;++k) {
                const auto timeIndex=static_cast<std::size_t>(j)*p.exposureSamples+k;
                const double time=measured.exposureStartTimeS+(j+.5)/(c.frequencyHz*count)+
                    p.integrationTimeS*((k+.5)/p.exposureSamples-.5)+.5*p.integrationTimeS/p.exposureSamples;
                const auto direction=photonAtState(*ray,time-measured.timeS);if(!(direction.x>0))return false;
                const double x=std::atan2(direction.y,direction.x),y=std::atan2(direction.z,direction.x);
                item.x[timeIndex]=x;item.y[timeIndex]=y;
                const auto levels=levelBounds(x,y,q[4],q[5]);
                item.levelMin[timeIndex]=levels[0];item.levelMax[timeIndex]=levels[1];
                localMin=std::min(localMin,levels[0]);localMax=std::max(localMax,levels[1]);
                if(!needSubSource)continue;
                const double xMin=x-.5*q[4]-9*sigma,xMax=x+.5*q[4]+9*sigma;
                const int col0=std::max(0,static_cast<int>(std::floor(xMin/p.pixelPitchRad+.5*p.width))-1);
                const int col1=std::min(static_cast<int>(p.width)-1,static_cast<int>(std::ceil(xMax/p.pixelPitchRad+.5*p.width)));
                fillAxisProb(xProb,col0,col1,p.width,x,q[4],xMin,xMax);
                const double yMin=y-.5*q[5]-9*sigma,yMax=y+.5*q[5]+9*sigma;
                const int row0=std::max(0,static_cast<int>(std::floor(yMin/p.pixelPitchRad+.5*p.height))-1);
                const int row1=std::min(static_cast<int>(p.height)-1,static_cast<int>(std::ceil(yMax/p.pixelPitchRad+.5*p.height)));
                fillAxisProb(yProb,row0,row1,p.height,y,q[5],yMin,yMax);
                double subVal=0.0;
                const bool sparseV5=col0<=col1&&row0<=row1&&sourceWaveOpticsV5&&
                    sourceWaveOpticsV5->addSampleRange(
                        rawAcc,xProb,yProb,
                        static_cast<unsigned>(col0),static_cast<unsigned>(col1+1),
                        static_cast<unsigned>(row0),static_cast<unsigned>(row1+1),
                        timeIndex,&subVal,useFastSourceSum);
                if(!sparseV5) {
                    for(int rowI=row0;rowI<=row1;++rowI) {
                        const unsigned row=static_cast<unsigned>(rowI);
                        const double yv=yProb[row];
                        if(yv==0.0)continue;
                        for(int colI=col0;colI<=col1;++colI) {
                            const unsigned col=static_cast<unsigned>(colI);
                            const double xv=xProb[col];
                            if(xv==0.0)continue;
                            const auto index=static_cast<std::size_t>(row)*p.width+col;
                            const double tw=optics.timeWeights[timeIndex*pixels+index];
                            if(tw==0.0)continue;
                            const double value=yv*xv*optics.solidAngles[index]/pixelArea;
                            const double term=value*tw/p.exposureSamples;
                            rawAcc+=term;
                            subVal+=term;
                        }
                    }
                }
                item.subexposureSource[timeIndex]=subVal;
                for(int colI=col0;colI<=col1;++colI)xProb[static_cast<unsigned>(colI)]=0.0;
                for(int rowI=row0;rowI<=row1;++rowI)yProb[static_cast<unsigned>(rowI)]=0.0;
            }
            if(!cachedSource)item.rawSource[j]=rawAcc;
            return true;
        };
        const auto runWorkerPhases=[&](auto&& nextPhase,double& localMin,double& localMax,auto&& onFail) {
            if(p.width<=128u&&p.height<=128u) {
                std::array<double,128> xProb{},yProb{};
                while(true) {
                    const unsigned j=nextPhase();
                    if(j>=count)break;
                    if(!processPhase(j,xProb.data(),yProb.data(),localMin,localMax)){onFail();break;}
                }
            } else {
                std::vector<double> xProb(p.width,0.0),yProb(p.height,0.0);
                while(true) {
                    const unsigned j=nextPhase();
                    if(j>=count)break;
                    if(!processPhase(j,xProb.data(),yProb.data(),localMin,localMax)){onFail();break;}
                }
            }
        };
        const unsigned prepWorkers=regionalOperator==RegionalOperator::ConicFastV5&&needSubSource&&!ray->stationary&&nearestArith
            ?std::min(ReticleNestedPoolGuardV1::maxWorkers(maximumPhaseWorkersV5),count):1u;
        if(prepWorkers>=2&&phasePoolV5.syncEnvironment()) {
            std::array<double,maximumPhaseWorkersV5> wMin,wMax;
            wMin.fill(infinity);wMax.fill(-infinity);
            std::atomic<unsigned> nextJ{0};std::atomic<bool> failed{false};
            const auto prepWork=[&](unsigned worker) noexcept {
                double lMin=infinity,lMax=-infinity;
                runWorkerPhases(
                    [&]() -> unsigned {
                        if(failed.load(std::memory_order_relaxed))return count;
                        return nextJ.fetch_add(1,std::memory_order_relaxed);
                    },
                    lMin,lMax,
                    [&]() {failed.store(true,std::memory_order_relaxed);});
                wMin[worker]=lMin;wMax[worker]=lMax;
            };
            phasePoolV5.run(prepWorkers,prepWork);
            if(failed.load(std::memory_order_relaxed))return nullptr;
            for(unsigned w=0;w<prepWorkers;++w){item.minimumLevel=std::min(item.minimumLevel,wMin[w]);item.maximumLevel=std::max(item.maximumLevel,wMax[w]);}
        } else {
            unsigned nextSerial=0;bool serialFailed=false;
            runWorkerPhases([&](){return nextSerial++;},item.minimumLevel,item.maximumLevel,[&](){serialFailed=true;});
            if(serialFailed)return nullptr;
        }
        if(!cachedSource)item.rawSource=filter(std::move(item.rawSource),alpha);
        if(ray->stationary)item.rawSource=optics.staticSourceRaw({{q[0],q[1],q[4],q[5]}});
        item.projectedSource=item.rawSource;
        project(item.projectedSource,optics.nuisance);
        // Bounded transient source/trajectory memo for cutoff scans in modeCache.
        if(modeCache&&!cachedSource&&sources.size()<1024)sources.push_back({key,branch,item.rawSource});
        Trajectory* ptr=recycled.get();
        if(modeCache) {
            modeTrajectories.push_back(std::move(recycled));
            modeTrajMap.emplace(makeTrajKeyV5(key,branch),ptr);
        } else {
            trajectoryCache.insert(trajectoryCache.begin(),std::move(recycled));
        }
        return ptr;
    }
    std::optional<double> regionalPhase(const Shape& q,unsigned branch,unsigned j,
        const Trajectory& source,const std::vector<double>& full) {
        if(!regionalShape||!exactCoordinates(*regionalShape,q)||regionalBranch!=branch) {
            regionalShape=q;regionalBranch=branch;regionalPhases.assign(count,0);regionalKnown.assign(count,false);
        }
        if(regionalKnown[j]){if(profile.enabled)++profile.phaseHits;return regionalPhases[j];}
        double value=0;
        const bool fastSubV5=regionalOperator==RegionalOperator::ConicFastV5&&certifiedArithmeticV2();
        const double* groundSubPtr=fastSubV5?groundSubexposuresV5(q[6],full).data():nullptr;
        const double marginV5=9*p.psfSigmaRad,hxV5=.5*p.width*p.pixelPitchRad,hyV5=.5*p.height*p.pixelPitchRad;
        const double fLeftV5=std::max(-1.55,-hxV5-marginV5),fRightV5=std::min(1.55,hxV5+marginV5);
        const double fBottomV5=std::max(-1.55,-hyV5-marginV5),fTopV5=std::min(1.55,hyV5+marginV5);
        const double areaScaleV5=(q[4]*q[5])/pixelArea;
        const double* subSourcePtr=(fastSubV5&&source.subexposureSource.size()==static_cast<std::size_t>(count)*p.exposureSamples)
            ?source.subexposureSource.data():nullptr;
        // This is the original native phase loop, shared by full likelihood
        // and certified partial-phase tests. A fallback reuses exact scalars.
        for(unsigned k=0;k<p.exposureSamples;++k) {
            const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
            const double lv0=source.levelMin[ti],lv1=source.levelMax[ti];
            if(groundSubPtr&&lv0>=q[6]) {
                value+=groundSubPtr[ti];
                continue;
            }
            if(groundSubPtr&&subSourcePtr&&lv1<q[6]&&
               source.x[ti]-.5*q[4]>=fLeftV5&&source.x[ti]+.5*q[4]<=fRightV5&&
               source.y[ti]-.5*q[5]>=fBottomV5&&source.y[ti]+.5*q[5]<=fTopV5) {
                value+=groundSubPtr[ti]-areaScaleV5*subSourcePtr[ti];
                continue;
            }
            const double* occultedPtr=nullptr;
            std::optional<std::vector<double>> occulted;
            if(lv0<q[6]) {
                const std::array<double,4> region{{source.x[ti]-.5*q[4],source.x[ti]+.5*q[4],source.y[ti]-.5*q[5],source.y[ti]+.5*q[5]}};
                LocalTimerV2 regionalTimer(&profile,LocalProfileV2::Regional);
                if((regionalOperator==RegionalOperator::ConicV4||regionalOperator==RegionalOperator::ConicFastV5)) {
                    // Private V4 is a separately versioned continuous stored-basis
                    // numerical operator. Its measured native differences are not
                    // a universal certificate for an arbitrary UCRT classifier.
                    if(profile.enabled)++profile.conicRequests;
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
                        if(regionalOperator==RegionalOperator::ConicFastV5) {
                            std::shared_ptr<conic_moments_v4::Bank> activeBankV5;
                            bool bankFound=false;
                            for(std::size_t idx=0;idx<phaseBankCacheV5.size();++idx) {
                                if(exactScalar(phaseBankCacheV5[idx].cutoff,q[6])) {
                                    if(idx>0) {
                                        const auto offset=static_cast<std::ptrdiff_t>(idx);
                                        std::rotate(phaseBankCacheV5.begin(),phaseBankCacheV5.begin()+offset,phaseBankCacheV5.begin()+offset+1);
                                    }
                                    activeBankV5=phaseBankCacheV5.front().bank;
                                    bankFound=true;
                                    break;
                                }
                            }
                            if(!bankFound) {
                                conic_moments_v4::Request request;request.width=p.width;request.height=p.height;
                                request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;
                                request.pointing={{forward.x,forward.y,forward.z}};request.up={{up.x,up.y,up.z}};request.cutoff=q[6];
                                LocalProfileV2::Clock::time_point started{};
                                if(profile.enabled)started=LocalProfileV2::Clock::now();
                                activeBankV5=std::shared_ptr<conic_moments_v4::Bank>(conic_moments_v4::Bank::prepareSharedV5(request));
                                if(phaseBankCacheV5.size()>=activeMaxPhaseBanksV5())phaseBankCacheV5.pop_back();
                                phaseBankCacheV5.insert(phaseBankCacheV5.begin(),CachedPhaseBankV5{q[6],activeBankV5});
                                if(profile.enabled){++profile.conicBanks;profile.conicSetupUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-started).count();}
                            }
                            if(activeBankV5) {
                                LocalProfileV2::Clock::time_point started{};
                                if(profile.enabled)started=LocalProfileV2::Clock::now();
                                const bool hasPre=ti<source.precomputedRegions.size()&&source.precomputedRegions[ti].valid;
                                const bool ok=hasPre
                                    ?activeBankV5->evaluateFastPreparedIntoV5(source.precomputedRegions[ti],serialWorkspaceV5)
                                    :activeBankV5->evaluateFastIntoV5(region,serialWorkspaceV5);
                                if(profile.enabled)profile.conicRenderUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-started).count();
                                if(ok&&serialWorkspaceV5.field.size()==pixels) {
                                    if(profile.enabled){++profile.conicFields;profile.conicMaximumError=std::max(profile.conicMaximumError,serialWorkspaceV5.maxError);}
                                    occultedPtr=serialWorkspaceV5.field.data();
                                }
                            }
                        } else {
                            if(!conicCutoff||!exactScalar(*conicCutoff,q[6])) {
                                conicBank.reset();conicCutoff=q[6];conicBankAttempted=false;
                            }
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
                                auto rendered=conicBank->evaluate(region);
                                if(profile.enabled)profile.conicRenderUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-started).count();
                                if(rendered&&rendered->field.size()==pixels&&rendered->absoluteError.size()==pixels) {
                                    if(profile.enabled){++profile.conicFields;profile.conicMaximumError=std::max(profile.conicMaximumError,*std::max_element(rendered->absoluteError.begin(),rendered->absoluteError.end()));}
                                    occulted=std::move(rendered->field);
                                    occultedPtr=occulted->data();
                                }
                            }
                        }
                    }
                }
                if(!occultedPtr) {
                    if(profile.enabled&&(regionalOperator==RegionalOperator::ConicV4||regionalOperator==RegionalOperator::ConicFastV5))++profile.conicFallbacks;
                    if(profile.enabled) {
                        ThermalGroundWorkV2 work;bool accelerated=false;
                        occulted=ThermalReticleReceiverV1::unitGroundFieldProfileV2(p,forward,up,q[6],region,[&](){return originalGround(q[6]);},&work,&accelerated);
                        profile.addGround(work,accelerated);
                        if(occulted)profile.captureGround(work,accelerated,p,forward,up,q[6],region,*occulted,q,branch,j,k);
                    } else occulted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,q[6],region,[&](){return originalGround(q[6]);});
                    if(occulted)occultedPtr=occulted->data();
                }
                if(!occultedPtr){diagnostic.refusal=22;return std::nullopt;}
                ++diagnostic.regionalFields;
                if(profile.enabled&&diagnostic.regionalFields%1024==0)profile.checkpoint("regional.progress");
            }
            const double* fullPtr=full.data();
            if(regionalOperator==RegionalOperator::ConicFastV5&&sparseMaskReadyV5&&powerOfTwoExposureV5) {
                const std::size_t beginIdx=sparseOffsetV5[ti],endIdx=sparseOffsetV5[ti+1];
                const std::uint16_t* pixPtr=sparsePixelV5.data();
                const double* solPtr=sparseSolidV5.data();
                const double* wgtPtr=sparseWeightV5.data();
                if(occultedPtr) {
                    accumulateSparseDiffV5(value,fullPtr,occultedPtr,pixPtr,solPtr,wgtPtr,beginIdx,endIdx,pixelArea,invExposureV5);
                } else {
                    for(std::size_t idx=beginIdx;idx<endIdx;++idx) {
                        const double field=fullPtr[pixPtr[idx]];
                        if(field==0.0)continue;
                        value+=field*solPtr[idx]*wgtPtr[idx]/pixelArea*invExposureV5;
                    }
                }
            } else {
                const double* weightsPtr=optics.timeWeights.data()+ti*pixels;
                const double* solidPtr=optics.solidAngles.data();
                if(occultedPtr) {
                    for(std::size_t i=0;i<pixels;++i) {
                        const double field=fullPtr[i]-occultedPtr[i];
                        value+=field*solidPtr[i]*weightsPtr[i]/pixelArea/p.exposureSamples;
                    }
                } else {
                    for(std::size_t i=0;i<pixels;++i) {
                        value+=fullPtr[i]*solidPtr[i]*weightsPtr[i]/pixelArea/p.exposureSamples;
                    }
                }
            }
        }
        regionalKnown[j]=true;regionalPhases[j]=value;return value;
    }
    const std::vector<double>& groundSubexposuresV5(double cutoff,const std::vector<double>& full) {
        for(std::size_t idx=0;idx<groundSubCacheV5.size();++idx) {
            if(exactScalar(groundSubCacheV5[idx].cutoff,cutoff)) {
                if(idx>0) {
                    const auto offset=static_cast<std::ptrdiff_t>(idx);
                    std::rotate(groundSubCacheV5.begin(),groundSubCacheV5.begin()+offset,groundSubCacheV5.begin()+offset+1);
                }
                return groundSubCacheV5.front().subexposure;
            }
        }
        const std::size_t subexposures=static_cast<std::size_t>(count)*p.exposureSamples;
        std::vector<double> sub(subexposures,0.0);
        const double* fullPtr=full.data();
        if(sparseMaskReadyV5&&powerOfTwoExposureV5) {
            const std::uint16_t* pixPtr=sparsePixelV5.data();
            const double* solPtr=sparseSolidV5.data();
            const double* wgtPtr=sparseWeightV5.data();
            for(std::size_t ti=0;ti<subexposures;++ti) {
                double acc=0.0;
                const std::size_t beginIdx=sparseOffsetV5[ti],endIdx=sparseOffsetV5[ti+1];
                for(std::size_t idx=beginIdx;idx<endIdx;++idx) {
                    acc+=fullPtr[pixPtr[idx]]*solPtr[idx]*wgtPtr[idx]/pixelArea*invExposureV5;
                }
                sub[ti]=acc;
            }
        } else {
            const double* solidPtr=optics.solidAngles.data();
            for(std::size_t ti=0;ti<subexposures;++ti) {
                double acc=0.0;
                const double* weightsPtr=optics.timeWeights.data()+ti*pixels;
                for(std::size_t i=0;i<pixels;++i) {
                    acc+=fullPtr[i]*solidPtr[i]*weightsPtr[i]/pixelArea/p.exposureSamples;
                }
                sub[ti]=acc;
            }
        }
        if(groundSubCacheV5.size()>=activeMaxPhaseBanksV5())groundSubCacheV5.pop_back();
        groundSubCacheV5.insert(groundSubCacheV5.begin(),GroundSubCacheV5{cutoff,std::move(sub)});
        return groundSubCacheV5.front().subexposure;
    }

    std::optional<bool> parallelRegionalPhasesV5(const Shape& q,unsigned branch,
        const Trajectory& source,const std::vector<double>& full,
        const std::vector<unsigned>* subset=nullptr) {
        if(regionalOperator!=RegionalOperator::ConicFastV5||!certifiedArithmeticV2())return false;
        if(!regionalShape||!exactCoordinates(*regionalShape,q)||regionalBranch!=branch) {
            regionalShape=q;regionalBranch=branch;regionalPhases.assign(count,0);regionalKnown.assign(count,false);
        }
        const auto& groundSub=groundSubexposuresV5(q[6],full);
        const double* groundSubPtr=groundSub.data();
        const double margin=9*p.psfSigmaRad,hx=.5*p.width*p.pixelPitchRad,hy=.5*p.height*p.pixelPitchRad;
        const double fLeft=std::max(-1.55,-hx-margin),fRight=std::min(1.55,hx+margin);
        const double fBottom=std::max(-1.55,-hy-margin),fTop=std::min(1.55,hy+margin);
        const double areaScale=(q[4]*q[5])/pixelArea;
        const bool haveSubSource=source.subexposureSource.size()==static_cast<std::size_t>(count)*p.exposureSamples;
        const double* subSourcePtr=haveSubSource?source.subexposureSource.data():nullptr;
        std::vector<unsigned> jobs;
        const auto classifyPhase=[&](unsigned j) {
            if(j>=count||regionalKnown[j])return;
            bool needsConic=false;
            double trivialSum=0.0;
            for(unsigned k=0;k<p.exposureSamples;++k) {
                const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
                if(source.levelMin[ti]>=q[6]) {
                    trivialSum+=groundSubPtr[ti];
                } else if(subSourcePtr&&source.levelMax[ti]<q[6]&&
                          source.x[ti]-.5*q[4]>=fLeft&&source.x[ti]+.5*q[4]<=fRight&&
                          source.y[ti]-.5*q[5]>=fBottom&&source.y[ti]+.5*q[5]<=fTop) {
                    trivialSum+=groundSubPtr[ti]-areaScale*subSourcePtr[ti];
                } else {
                    needsConic=true;
                    break;
                }
            }
            if(!needsConic) {
                regionalPhases[j]=trivialSum;
                regionalKnown[j]=true;
            } else {
                jobs.push_back(j);
            }
        };
        if(subset) {
            jobs.reserve(subset->size());
            for(const unsigned j:*subset)classifyPhase(j);
        } else {
            jobs.reserve(count);
            for(unsigned j=0;j<count;++j)classifyPhase(j);
        }
        if(jobs.empty())return true;
        const unsigned maxW=ReticleNestedPoolGuardV1::maxWorkers(maximumPhaseWorkersV5);
        if(maxW<2)return false;
        if(!phasePoolV5.syncEnvironment())return false;
        struct Statistics {
            std::size_t phases=0,regions=0,requests=0,fields=0,fallbacks=0,accelerated=0,native=0;
            double maximumError=0,renderUs=0;
        };
        std::array<Statistics,maximumPhaseWorkersV5> statistics{};
        std::array<std::exception_ptr,maximumPhaseWorkersV5> errors{};
        std::atomic<std::size_t> next{0};std::atomic<bool> refused{false};
        const auto nativeForward=unit(forward),nativeUp=unit(up);
        auto nativeRight=unit(cross(nativeForward,nativeUp));
        if(norm(nativeRight)<.5)nativeRight=unit(cross(nativeForward,std::abs(nativeForward.z)<.9?V{0,0,1}:V{0,1,0}));
        const auto nativeVertical=unit(cross(nativeRight,nativeForward));
        const double a=dot(nativeForward,nativeUp),b=dot(nativeRight,nativeUp),cc=dot(nativeVertical,nativeUp);
        const auto eligible=[&](const std::array<double,4>& region) {
            const double left=std::max(fLeft,region[0]);
            const double rightEdge=std::min(fRight,region[1]);
            const double bottom=std::max(fBottom,region[2]);
            const double top=std::min(fTop,region[3]);
            if(!(rightEdge>left)||!(top>bottom))return true;
            const double maxTx=std::max(std::abs(std::tan(left)),std::abs(std::tan(rightEdge)));
            const double maxTy=std::max(std::abs(std::tan(bottom)),std::abs(std::tan(top)));
            const double maxN=std::nextafter(std::abs(a)+std::nextafter(std::abs(b)*maxTx,infinity),infinity);
            return std::nextafter(cc,-infinity)>std::nextafter(maxN*maxTy,infinity);
        };
        CachedPhaseBankV5* bankEntry=nullptr;
        for(std::size_t idx=0;idx<phaseBankCacheV5.size();++idx) {
            if(exactScalar(phaseBankCacheV5[idx].cutoff,q[6])) {
                if(idx>0) {
                    const auto offset=static_cast<std::ptrdiff_t>(idx);
                    std::rotate(phaseBankCacheV5.begin(),phaseBankCacheV5.begin()+offset,phaseBankCacheV5.begin()+offset+1);
                }
                bankEntry=&phaseBankCacheV5.front();
                break;
            }
        }
        if(!bankEntry) {
            conic_moments_v4::Request request;request.width=p.width;request.height=p.height;
            request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;request.cutoff=q[6];
            request.pointing={{forward.x,forward.y,forward.z}};request.up={{up.x,up.y,up.z}};
            const auto began=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
            auto newBank=std::shared_ptr<conic_moments_v4::Bank>(conic_moments_v4::Bank::prepareSharedV5(request));
            if(phaseBankCacheV5.size()>=activeMaxPhaseBanksV5())phaseBankCacheV5.pop_back();
            phaseBankCacheV5.insert(phaseBankCacheV5.begin(),CachedPhaseBankV5{q[6],std::move(newBank),0ull,0ull});
            bankEntry=&phaseBankCacheV5.front();
            if(profile.enabled) {
                ++profile.conicBanks;
                profile.conicSetupUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-began).count();
            }
        }
        const conic_moments_v4::Bank* phaseBankPtr=bankEntry->bank.get();
        // Pre-warm cold degree-3 bank columns concurrently across workers even
        // when jobs.size() == 1 so a new cutoff never warms 64-128 columns serially.
        unsigned warmColBegin=p.width,warmColEnd=0,coldCols=0;
        if(phaseBankPtr) {
            double minX=infinity,maxX=-infinity;
            for(const unsigned j:jobs) {
                for(unsigned k=0;k<p.exposureSamples;++k) {
                    const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
                    if(source.x[ti]<minX)minX=source.x[ti];
                    if(source.x[ti]>maxX)maxX=source.x[ti];
                }
            }
            if(minX<=maxX) {
                const double rLeft=std::max(fLeft,minX-.5*q[4]);
                const double rRight=std::min(fRight,maxX+.5*q[4]);
                for(unsigned col=0;col<p.width;++col) {
                    const double px0=(col-.5*p.width)*p.pixelPitchRad,px1=px0+p.pixelPitchRad;
                    const double cLeft=std::max(rLeft,std::max(-1.55,px0-margin));
                    const double cRight=std::min(rRight,std::min(1.55,px1+margin));
                    if(cRight>cLeft) {
                        if(col<warmColBegin)warmColBegin=col;
                        warmColEnd=col+1;
                        if(!bankEntry->columnWarmed(col))++coldCols;
                    }
                }
            }
        }
        const unsigned workers=std::min(maxW,std::max<unsigned>(static_cast<unsigned>(jobs.size()),coldCols));
        if(workers<2&&jobs.size()<2)return false;
        std::atomic<unsigned> nextWarmCol{warmColBegin};
        const bool needWarm=phaseBankPtr&&coldCols>0&&warmColBegin<warmColEnd;
        const auto work=[&](unsigned worker) noexcept {
            try {
                if(needWarm) {
                    while(true) {
                        const unsigned col=nextWarmCol.fetch_add(1,std::memory_order_relaxed);
                        if(col>=warmColEnd)break;
                        if(!bankEntry->columnWarmed(col))phaseBankPtr->ensureColumnV5(col);
                    }
                }
                auto& stats=statistics[worker];
                auto& workspace=phaseWorkspacesV5[worker];
                while(!refused.load(std::memory_order_relaxed)) {
                    const auto index=next.fetch_add(1,std::memory_order_relaxed);if(index>=jobs.size())break;
                    ++stats.phases;
                    const unsigned j=jobs[index];double value=0;
                    for(unsigned k=0;k<p.exposureSamples;++k) {
                        const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
                        if(source.levelMin[ti]>=q[6]) {
                            value+=groundSubPtr[ti];
                            continue;
                        }
                        if(subSourcePtr&&source.levelMax[ti]<q[6]&&
                           source.x[ti]-.5*q[4]>=fLeft&&source.x[ti]+.5*q[4]<=fRight&&
                           source.y[ti]-.5*q[5]>=fBottom&&source.y[ti]+.5*q[5]<=fTop) {
                            value+=groundSubPtr[ti]-areaScale*subSourcePtr[ti];
                            continue;
                        }
                        const double* occultedPtr=nullptr;
                        std::optional<std::vector<double>> fallbackOcculted;
                        {
                            ++stats.regions;++stats.requests;
                            const std::array<double,4> region{{source.x[ti]-.5*q[4],source.x[ti]+.5*q[4],source.y[ti]-.5*q[5],source.y[ti]+.5*q[5]}};
                            if(phaseBankPtr&&eligible(region)) {
                                const auto began=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
                                const bool hasPre=ti<source.precomputedRegions.size()&&source.precomputedRegions[ti].valid;
                                const bool ok=hasPre
                                    ?phaseBankPtr->evaluateFastPreparedIntoV5(source.precomputedRegions[ti],workspace)
                                    :phaseBankPtr->evaluateFastIntoV5(region,workspace);
                                if(profile.enabled)stats.renderUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-began).count();
                                if(ok&&workspace.field.size()==pixels) {
                                    ++stats.fields;stats.maximumError=std::max(stats.maximumError,workspace.maxError);
                                    occultedPtr=workspace.field.data();
                                }
                            }
                            if(!occultedPtr) {
                                ++stats.fallbacks;
                                bool accelerated=false;
                                fallbackOcculted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,q[6],region,[&](){return &full;},&accelerated);
                                if(accelerated)++stats.accelerated;else ++stats.native;
                                if(fallbackOcculted)occultedPtr=fallbackOcculted->data();
                            }
                            if(!occultedPtr){refused.store(true,std::memory_order_relaxed);break;}
                        }
                        // Identical chronological subexposure and row-major
                        // accumulation inside this phase. Skipping exact-zero
                        // field/mask products and multiplying by exact binary
                        // powers of two preserves FE_TONEAREST bits.
                        const double* fullPtr=full.data();
                        if(sparseMaskReadyV5&&powerOfTwoExposureV5) {
                            const std::size_t beginIdx=sparseOffsetV5[ti],endIdx=sparseOffsetV5[ti+1];
                            accumulateSparseDiffV5(value,fullPtr,occultedPtr,
                                sparsePixelV5.data(),sparseSolidV5.data(),sparseWeightV5.data(),
                                beginIdx,endIdx,pixelArea,invExposureV5);
                        } else {
                            const double* weightsPtr=optics.timeWeights.data()+ti*pixels;
                            const double* solidPtr=optics.solidAngles.data();
                            for(std::size_t i=0;i<pixels;++i) {
                                const double field=fullPtr[i]-occultedPtr[i];
                                if(field==0.0)continue;
                                value+=field*solidPtr[i]*weightsPtr[i]/pixelArea/p.exposureSamples;
                            }
                        }
                    }
                    regionalPhases[j]=value;
                }
            } catch(...) {errors[worker]=std::current_exception();refused.store(true,std::memory_order_relaxed);}
        };
        LocalTimerV2 timer(&profile,LocalProfileV2::Regional);
        phasePoolV5.run(workers,work);
        if(needWarm)bankEntry->markWarmedRange(warmColBegin,warmColEnd);
        for(const auto& error:errors)if(error)std::rethrow_exception(error);
        std::size_t activeWorkers=0;
        for(const auto& stats:statistics) {
            if(stats.phases)++activeWorkers;
            diagnostic.regionalFields+=stats.regions;
            if(profile.enabled) {
                profile.conicRequests+=stats.requests;
                profile.conicFields+=stats.fields;profile.conicFallbacks+=stats.fallbacks;
                profile.parallelFields+=stats.regions;profile.conicRenderUs+=stats.renderUs;
                profile.regionalAccelerated+=stats.accelerated;profile.regionalNative+=stats.native;
                profile.conicMaximumError=std::max(profile.conicMaximumError,stats.maximumError);
            }
        }
        if(profile.enabled){++profile.parallelBatches;profile.parallelWorkersPeak=std::max(profile.parallelWorkersPeak,activeWorkers);}
        if(refused.load(std::memory_order_relaxed)){diagnostic.refusal=22;return std::nullopt;}
        for(const auto j:jobs)regionalKnown[j]=true;
        return true;
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
            if(source->levelMin[ti]>=q[6])continue;
            const std::array<double,4> region{{source->x[ti]-.5*q[4],source->x[ti]+.5*q[4],source->y[ti]-.5*q[5],source->y[ti]+.5*q[5]}};
            if(!std::all_of(region.begin(),region.end(),[](double value){return std::isfinite(value)&&std::abs(value)<=1.55;})||
                !(region[1]>region[0])||!(region[3]>region[2]))return false;
        }
        prepareSubsetMaps();if(subsetMaps.empty())return false;
        const auto& sourceProjected=source->projectedSource;
        const auto* full=groundField(q[6]);if(!full)return false;
        const double beta=1-alpha,tiny=std::numeric_limits<double>::denorm_min();
        const double lop=upperPositive(std::sqrt(1+static_cast<long double>(beta)*beta));
        const double sourceNorm=vectorUpper(sourceProjected);
        for(const auto& map:subsetMaps) {
            if(profile.enabled)++profile.subsetChecks;
            if(regionalOperator==RegionalOperator::ConicFastV5) {
                const auto parallel=parallelRegionalPhasesV5(q,branch,*source,*full,&map.phases);
                if(!parallel)return false;
            }
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
    struct BasisView {
        const std::vector<double>* raw0=nullptr;
        const std::vector<double>* raw1=nullptr;
        const std::vector<double>* proj0=nullptr;
        const std::vector<double>* proj1=nullptr;
        bool sourceRelativeGround=false;
    };
    bool basisView(const Shape& q,unsigned branch,BasisView& out) {
        if(lastBasis&&lastShape&&exactCoordinates(*lastShape,q)&&lastBranch==branch) {
            out={&lastBasis->raw[0],&lastBasis->raw[1],&lastBasis->projected[0],&lastBasis->projected[1],lastBasis->sourceRelativeGround};
            return true;
        }
        if(!valid(q))return false;
        const auto* ground=calibration(q[6]);if(!ground)return false;
        auto* source=prepare(q,branch);if(!source)return false;
        if(!source->path.stationary&&(q[4]==0||q[5]==0||source->minimumLevel>=q[6]||source->maximumLevel<q[6])) {
            ++diagnostic.basisEvaluations;
            if(profile.enabled)++profile.calls[LocalProfileV2::BasisWork];
            out={&ground->raw,&source->rawSource,&ground->wave,&source->projectedSource,
                q[4]>0&&q[5]>0&&source->maximumLevel<q[6]};
            return true;
        }
        if(!basis(q,branch))return false;
        out={&lastBasis->raw[0],&lastBasis->raw[1],&lastBasis->projected[0],&lastBasis->projected[1],lastBasis->sourceRelativeGround};
        return true;
    }
    std::optional<Basis> basis(const Shape& q,unsigned branch) {
        LocalTimerV2 timer(&profile,LocalProfileV2::BasisWork);
        if(profile.enabled&&profile.calls[LocalProfileV2::BasisWork]%256==0) {
            profile.checkpoint("basis.progress");
            if(gaussianMemoV5) {
                const auto work=gaussianMemoV5->work();
                std::fprintf(stderr,"MOVING_GAUSSIAN_MEMO {\"basis_calls\":%zu,\"hits\":%zu,\"misses\":%zu,\"entries\":%zu,\"charged_bytes\":%zu}\n",
                    profile.calls[LocalProfileV2::BasisWork],work.hits,work.misses,work.entries,work.chargedBytes);
            }
        }
        if(!valid(q))return std::nullopt;
        if(lastBasis&&lastShape&&exactCoordinates(*lastShape,q)&&lastBranch==branch)return lastBasis;
        const auto* ground=calibration(q[6]);if(!ground)return std::nullopt;
        auto* source=prepare(q,branch);if(!source)return std::nullopt;
        ++diagnostic.basisEvaluations;Basis answer;answer.raw[1]=source->rawSource;
        bool usedGroundRaw=false;
        if(source->path.stationary) {
            if(q[4]>0&&q[5]>0) {
                const std::array<double,4> region{{q[0]-.5*q[4],q[0]+.5*q[4],q[1]-.5*q[5],q[1]+.5*q[5]}};
                const auto occulted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,q[6],region,[&](){return originalGround(q[6]);});if(!occulted)return std::nullopt;
                const auto* full=groundField(q[6]);if(!full)return std::nullopt;
                auto field=*full;for(std::size_t i=0;i<pixels;++i)field[i]-=(*occulted)[i];answer.raw[0]=wave(field);
            } else {answer.raw[0]=ground->raw;usedGroundRaw=true;}
        } else if(q[4]==0||q[5]==0||source->minimumLevel>=q[6]){answer.raw[0]=ground->raw;usedGroundRaw=true;}
        else if(source->maximumLevel<q[6]) {
            // Exact change of linear coordinates: a wholly ground-backed
            // radiator has contrast relative to ground. No subtraction of
            // separately rounded phase vectors or missing occulted geometry.
            answer.raw[0]=ground->raw;answer.sourceRelativeGround=true;usedGroundRaw=true;
        } else {
            const auto* full=groundField(q[6]);if(!full)return std::nullopt;
            answer.raw[0].assign(count,0);
            const auto parallel=parallelRegionalPhasesV5(q,branch,*source,*full);if(!parallel)return std::nullopt;
            if(*parallel)answer.raw[0]=regionalPhases;
            else for(unsigned j=0;j<count;++j) {
                    const auto value=regionalPhase(q,branch,j,*source,*full);if(!value)return std::nullopt;
                    answer.raw[0][j]=*value;
                }
            answer.raw[0]=filter(std::move(answer.raw[0]),alpha);
        }
        if(usedGroundRaw)answer.projected[0]=ground->wave;
        else {answer.projected[0]=answer.raw[0];project(answer.projected[0],optics.nuisance);}
        answer.projected[1]=source->projectedSource;
        lastShape=q;lastBranch=branch;lastBasis=answer;return answer;
    }
    double effectiveSource(const Shape& q,bool sourceRelativeGround,const Fit& fit) const {
        return fit.source-(sourceRelativeGround?q[4]*q[5]/pixelArea*fit.background:0.);
    }
    double effectiveSource(const Shape& q,const Basis& values,const Fit& fit) const {
        return effectiveSource(q,values.sourceRelativeGround,fit);
    }
    std::optional<Fit> solve(const Shape& q,unsigned branch,std::vector<double>* reduced=nullptr) {
        BasisView view;if(!basisView(q,branch,view))return std::nullopt;
        const auto& u=*view.proj0;const auto& v=*view.proj1;
        const double uu2=inner(u,u),uv=inner(u,v),vv=inner(v,v),determinant=uu2*vv-uv*uv;
        if(!(uu2>0)||!(vv>0)||!(determinant>128*epsilon*uu2*vv))return std::nullopt;
        const double yu=inner(data,u),yv=inner(data,v),b=(yu*vv-yv*uv)/determinant,d=(yv*uu2-yu*uv)/determinant;
        Fit fit;fit.cost=0;fit.background=b;fit.source=d+(view.sourceRelativeGround?q[4]*q[5]/pixelArea*b:0.);
        if(reduced)reduced->resize(count);
        for(unsigned j=0;j<count;++j) {
            const double r=data[j]-b*u[j]-d*v[j];fit.cost+=r*r;
            if(reduced)(*reduced)[j]=r/std::sqrt(variance);
        }
        if(!std::isfinite(fit.cost)||!std::isfinite(b)||!std::isfinite(d))return std::nullopt;
        return fit;
    }
    bool physical(const Shape& q,unsigned branch,const Fit& fit,std::array<double,3>* output=nullptr) {
        BasisView view;if(!basisView(q,branch,view))return false;
        const double d=effectiveSource(q,view.sourceRelativeGround,fit);
        if(!(uu>0)||!(tt>=0)||(tt>0&&!(det>128*epsilon*uu*tt)))return false;
        const double* measPtr=measured.measuredElectrons.data();
        const double* r0Ptr=view.raw0->data();
        const double* r1Ptr=view.raw1->data();
        const double* darkPtr=dark.data();
        const double* unifPtr=uniform.data();
        const double* tranPtr=transient.data();
        double dotUnif=0.0,dotTran=0.0;
        for(unsigned j=0;j<count;++j) {
            const double rem=measPtr[j]-(fit.background*r0Ptr[j]+d*r1Ptr[j]+darkPtr[j]);
            dotUnif+=rem*unifPtr[j];
            if(tt>0)dotTran+=rem*tranPtr[j];
        }
        const double sky=tt>0?(dotUnif*tt-dotTran*ut)/det:dotUnif/uu;
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
    std::shared_ptr<conic_moments_v4::Bank> getOrCreatePhaseBankV5(double cutoff) {
        for(std::size_t idx=0;idx<phaseBankCacheV5.size();++idx) {
            if(exactScalar(phaseBankCacheV5[idx].cutoff,cutoff)) {
                if(idx>0) {
                    const auto offset=static_cast<std::ptrdiff_t>(idx);
                    std::rotate(phaseBankCacheV5.begin(),phaseBankCacheV5.begin()+offset,phaseBankCacheV5.begin()+offset+1);
                }
                return phaseBankCacheV5.front().bank;
            }
        }
        conic_moments_v4::Request request;request.width=p.width;request.height=p.height;
        request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;request.cutoff=cutoff;
        request.pointing={{forward.x,forward.y,forward.z}};request.up={{up.x,up.y,up.z}};
        const auto began=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
        auto newBank=std::shared_ptr<conic_moments_v4::Bank>(conic_moments_v4::Bank::prepareSharedV5(request));
        if(phaseBankCacheV5.size()>=activeMaxPhaseBanksV5())phaseBankCacheV5.pop_back();
        phaseBankCacheV5.insert(phaseBankCacheV5.begin(),CachedPhaseBankV5{cutoff,newBank,0ull,0ull});
        if(profile.enabled) {
            ++profile.conicBanks;
            profile.conicSetupUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-began).count();
        }
        return newBank;
    }

    struct JacSlotStatsV5 {
        std::size_t regions=0,requests=0,fields=0,fallbacks=0,accelerated=0,native=0;
        double maximumError=0,renderUs=0,prepareUs=0;
    };

    bool evaluateJacobianSlotV5(const Shape& q,unsigned branch,const Trajectory* baseTraj,bool reuseFullTraj,
        const Ground& ground,const std::vector<double>* fullPtrVec,const std::vector<double>* groundSubVec,
        const conic_moments_v4::Bank* bankPtr,conic_moments_v4::FieldWorkspaceV5& workspace,
        JacWorkerScratchV5& scratch,std::vector<double>& outReduced,bool& outRefused,bool& outNeedFallback,JacSlotStatsV5& stats) const {
        const double sigma=p.psfSigmaRad;
        const double margin=9*sigma,hx=.5*p.width*p.pixelPitchRad,hy=.5*p.height*p.pixelPitchRad;
        const double fLeft=std::max(-1.55,-hx-margin),fRight=std::min(1.55,hx+margin);
        const double fBottom=std::max(-1.55,-hy-margin),fTop=std::min(1.55,hy+margin);
        const std::size_t samples=static_cast<std::size_t>(count)*p.exposureSamples;
        const Trajectory* traj=reuseFullTraj?baseTraj:nullptr;
        if(!traj) {
            const auto tPrep0=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
            auto& localTraj=scratch.traj;
            const bool sameRay=baseTraj&&
                exactScalar(q[0],baseTraj->shape[0])&&exactScalar(q[1],baseTraj->shape[1])&&
                exactScalar(q[2],baseTraj->shape[2])&&exactScalar(q[3],baseTraj->shape[3]);
            if(sameRay) {
                if(baseTraj->path.stationary){outNeedFallback=true;return false;}
                localTraj.path=baseTraj->path;
            } else {
                const auto ray=path(q,branch);if(!ray)return false;
                if(ray->stationary){outNeedFallback=true;return false;}
                localTraj.path=*ray;
            }
            localTraj.minimumLevel=infinity;localTraj.maximumLevel=-infinity;
            localTraj.x.resize(samples);localTraj.y.resize(samples);
            localTraj.levelMin.resize(samples);localTraj.levelMax.resize(samples);
            localTraj.subexposureSource.assign(samples,0.0);
            localTraj.rawSource.assign(count,0.0);
            const bool useFastSum=sourceWaveOpticsV5&&sourceWaveOpticsV5->eligibleOptics();
            const auto gaussianPrimBitExact=[](double z) noexcept -> double {
                if(z>=9.5)return z;
                if(z<=-38.5)return 0.0;
                return z*.5*std::erfc(-z/std::sqrt(2.))+std::exp(-.5*z*z)/std::sqrt(2.*3.14159265358979323846);
            };
            const auto fillAxisProb=[&](double* prob,int i0,int i1,unsigned dim,double centre,double extent,double rMin,double rMax) {
                if(i0>i1)return;
                if(extent==0.0) {
                    const double invSqrt2Sigma=1.0/(std::sqrt(2.)*sigma);
                    bool hasPrev=false;double prevHi=0.0,prevErfc=0.0;
                    for(int idx=i0;idx<=i1;++idx) {
                        const unsigned u=static_cast<unsigned>(idx);
                        const double lo=(u-.5*dim)*p.pixelPitchRad,hi=lo+p.pixelPitchRad;
                        if(lo>rMax||hi<rMin){hasPrev=false;continue;}
                        const double eLo=(hasPrev&&lo==prevHi)?prevErfc:std::erfc((centre-lo)*invSqrt2Sigma);
                        const double eHi=std::erfc((centre-hi)*invSqrt2Sigma);
                        hasPrev=true;prevHi=hi;prevErfc=eHi;
                        prob[u]=.5*(eHi-eLo);
                    }
                } else {
                    const double a=centre-.5*extent,b=centre+.5*extent;
                    const auto integratedCdf=[&](double edge) noexcept -> double {
                        const double za=(edge-a)/sigma,zb=(edge-b)/sigma;
                        if(za>=9.5&&zb<=-9.5)return sigma*za;
                        return sigma*(gaussianPrimBitExact(za)-gaussianPrimBitExact(zb));
                    };
                    bool hasPrev=false;double prevHi=0.0,prevInt=0.0;
                    for(int idx=i0;idx<=i1;++idx) {
                        const unsigned u=static_cast<unsigned>(idx);
                        const double lo=(u-.5*dim)*p.pixelPitchRad,hi=lo+p.pixelPitchRad;
                        if(lo>rMax||hi<rMin){hasPrev=false;continue;}
                        const double intLo=(hasPrev&&lo==prevHi)?prevInt:integratedCdf(lo);
                        const double intHi=integratedCdf(hi);
                        hasPrev=true;prevHi=hi;prevInt=intHi;
                        prob[u]=std::clamp(intHi-intLo,0.,std::min(b-a,hi-lo))/extent;
                    }
                }
            };
            std::array<double,128> xProbArr{},yProbArr{};
            std::vector<double> xProbVec,yProbVec;
            double* xProb=xProbArr.data();double* yProb=yProbArr.data();
            if(p.width>128u||p.height>128u){xProbVec.assign(p.width,0.0);yProbVec.assign(p.height,0.0);xProb=xProbVec.data();yProb=yProbVec.data();}
            for(unsigned j=0;j<count;++j) {
                double rawAcc=0.0;
                for(unsigned k=0;k<p.exposureSamples;++k) {
                    const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
                    double x=0.0,y=0.0;
                    if(sameRay) {
                        x=baseTraj->x[ti];y=baseTraj->y[ti];
                    } else {
                        const double time=measured.exposureStartTimeS+(j+.5)/(c.frequencyHz*count)+
                            p.integrationTimeS*((k+.5)/p.exposureSamples-.5)+.5*p.integrationTimeS/p.exposureSamples;
                        const auto direction=photonAtState(localTraj.path,time-measured.timeS);if(!(direction.x>0))return false;
                        x=std::atan2(direction.y,direction.x);y=std::atan2(direction.z,direction.x);
                    }
                    localTraj.x[ti]=x;localTraj.y[ti]=y;
                    const auto levels=levelBounds(x,y,q[4],q[5]);
                    localTraj.levelMin[ti]=levels[0];localTraj.levelMax[ti]=levels[1];
                    localTraj.minimumLevel=std::min(localTraj.minimumLevel,levels[0]);
                    localTraj.maximumLevel=std::max(localTraj.maximumLevel,levels[1]);
                    const double xMin=x-.5*q[4]-9*sigma,xMax=x+.5*q[4]+9*sigma;
                    const int col0=std::max(0,static_cast<int>(std::floor(xMin/p.pixelPitchRad+.5*p.width))-1);
                    const int col1=std::min(static_cast<int>(p.width)-1,static_cast<int>(std::ceil(xMax/p.pixelPitchRad+.5*p.width)));
                    fillAxisProb(xProb,col0,col1,p.width,x,q[4],xMin,xMax);
                    const double yMin=y-.5*q[5]-9*sigma,yMax=y+.5*q[5]+9*sigma;
                    const int row0=std::max(0,static_cast<int>(std::floor(yMin/p.pixelPitchRad+.5*p.height))-1);
                    const int row1=std::min(static_cast<int>(p.height)-1,static_cast<int>(std::ceil(yMax/p.pixelPitchRad+.5*p.height)));
                    fillAxisProb(yProb,row0,row1,p.height,y,q[5],yMin,yMax);
                    double subVal=0.0;
                    const bool sparseV5=col0<=col1&&row0<=row1&&sourceWaveOpticsV5&&
                        sourceWaveOpticsV5->addSampleRange(
                            rawAcc,xProb,yProb,
                            static_cast<unsigned>(col0),static_cast<unsigned>(col1+1),
                            static_cast<unsigned>(row0),static_cast<unsigned>(row1+1),
                            ti,&subVal,useFastSum);
                    if(!sparseV5) {
                        for(int rowI=row0;rowI<=row1;++rowI) {
                            const unsigned row=static_cast<unsigned>(rowI);
                            const double yv=yProb[row];
                            if(yv==0.0)continue;
                            for(int colI=col0;colI<=col1;++colI) {
                                const unsigned col=static_cast<unsigned>(colI);
                                const double xv=xProb[col];
                                if(xv==0.0)continue;
                                const auto index=static_cast<std::size_t>(row)*p.width+col;
                                const double tw=optics.timeWeights[ti*pixels+index];
                                if(tw==0.0)continue;
                                const double value=yv*xv*optics.solidAngles[index]/pixelArea;
                                const double term=value*tw/p.exposureSamples;
                                rawAcc+=term;subVal+=term;
                            }
                        }
                    }
                    localTraj.subexposureSource[ti]=subVal;
                    for(int colI=col0;colI<=col1;++colI)xProb[static_cast<unsigned>(colI)]=0.0;
                    for(int rowI=row0;rowI<=row1;++rowI)yProb[static_cast<unsigned>(rowI)]=0.0;
                }
                localTraj.rawSource[j]=rawAcc;
            }
            {
                double previous=0.0;
                for(auto& val:localTraj.rawSource){previous+=alpha*(val-previous);val=previous;}
            }
            if(profile.enabled)stats.prepareUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-tPrep0).count();
            traj=&localTraj;
        }
        auto& u=scratch.u;
        bool sourceRelativeGround=false;
        if(q[4]==0||q[5]==0||traj->minimumLevel>=q[6]) {
            u=ground.raw;
        } else if(traj->maximumLevel<q[6]) {
            u=ground.raw;
            sourceRelativeGround=true;
        } else {
            if(!bankPtr||!fullPtrVec||!groundSubVec){outNeedFallback=true;return false;}
            const auto& full=*fullPtrVec;
            const auto& groundSub=*groundSubVec;
            const auto nativeForward=unit(forward),nativeUp=unit(up);
            auto nativeRight=unit(cross(nativeForward,nativeUp));
            if(norm(nativeRight)<.5)nativeRight=unit(cross(nativeForward,std::abs(nativeForward.z)<.9?V{0,0,1}:V{0,1,0}));
            const auto nativeVertical=unit(cross(nativeRight,nativeForward));
            const double aa=dot(nativeForward,nativeUp),bb=dot(nativeRight,nativeUp),cc=dot(nativeVertical,nativeUp);
            const auto eligible=[&](const std::array<double,4>& region) {
                const double left=std::max(fLeft,region[0]),rightEdge=std::min(fRight,region[1]);
                const double bottom=std::max(fBottom,region[2]),top=std::min(fTop,region[3]);
                if(!(rightEdge>left)||!(top>bottom))return true;
                const double maxTx=std::max(std::abs(std::tan(left)),std::abs(std::tan(rightEdge)));
                const double maxTy=std::max(std::abs(std::tan(bottom)),std::abs(std::tan(top)));
                const double maxN=std::nextafter(std::abs(aa)+std::nextafter(std::abs(bb)*maxTx,infinity),infinity);
                return std::nextafter(cc,-infinity)>std::nextafter(maxN*maxTy,infinity);
            };
            const double areaScale=(q[4]*q[5])/pixelArea;
            const double* groundSubPtr=groundSub.data();
            const double* subSourcePtr=traj->subexposureSource.size()==samples?traj->subexposureSource.data():nullptr;
            const double* fullPtr=full.data();
            u.assign(count,0.0);
            for(unsigned j=0;j<count;++j) {
                double value=0.0;
                for(unsigned k=0;k<p.exposureSamples;++k) {
                    const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
                    if(traj->levelMin[ti]>=q[6]){value+=groundSubPtr[ti];continue;}
                    if(subSourcePtr&&traj->levelMax[ti]<q[6]&&
                       traj->x[ti]-.5*q[4]>=fLeft&&traj->x[ti]+.5*q[4]<=fRight&&
                       traj->y[ti]-.5*q[5]>=fBottom&&traj->y[ti]+.5*q[5]<=fTop) {
                        value+=groundSubPtr[ti]-areaScale*subSourcePtr[ti];
                        continue;
                    }
                    ++stats.regions;++stats.requests;
                    const double* occultedPtr=nullptr;
                    std::optional<std::vector<double>> fallbackOcculted;
                    const std::array<double,4> region{{traj->x[ti]-.5*q[4],traj->x[ti]+.5*q[4],traj->y[ti]-.5*q[5],traj->y[ti]+.5*q[5]}};
                    if(eligible(region)) {
                        const auto began=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
                        const bool ok=bankPtr->evaluateFastIntoV5(region,workspace);
                        if(profile.enabled)stats.renderUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-began).count();
                        if(ok&&workspace.field.size()==pixels) {
                            ++stats.fields;stats.maximumError=std::max(stats.maximumError,workspace.maxError);
                            occultedPtr=workspace.field.data();
                        }
                    }
                    if(!occultedPtr) {
                        ++stats.fallbacks;
                        bool accelerated=false;
                        fallbackOcculted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,q[6],region,[&](){return &full;},&accelerated);
                        if(accelerated)++stats.accelerated;else ++stats.native;
                        if(fallbackOcculted)occultedPtr=fallbackOcculted->data();
                    }
                    if(!occultedPtr){outRefused=true;return false;}
                    if(sparseMaskReadyV5&&powerOfTwoExposureV5) {
                        const std::size_t beginIdx=sparseOffsetV5[ti],endIdx=sparseOffsetV5[ti+1];
                        accumulateSparseDiffV5(value,fullPtr,occultedPtr,
                            sparsePixelV5.data(),sparseSolidV5.data(),sparseWeightV5.data(),
                            beginIdx,endIdx,pixelArea,invExposureV5);
                    } else {
                        const double* weightsPtr=optics.timeWeights.data()+ti*pixels;
                        const double* solidPtr=optics.solidAngles.data();
                        for(std::size_t i=0;i<pixels;++i) {
                            const double field=fullPtr[i]-occultedPtr[i];
                            if(field==0.0)continue;
                            value+=field*solidPtr[i]*weightsPtr[i]/pixelArea/p.exposureSamples;
                        }
                    }
                }
                u[j]=value;
            }
            {
                double previous=0.0;
                for(auto& val:u){previous+=alpha*(val-previous);val=previous;}
            }
        }
        auto& v=scratch.v;
        v=traj->rawSource;
        project(u,optics.nuisance);project(v,optics.nuisance);
        const double uu2=inner(u,u),uv=inner(u,v),vv=inner(v,v),determinant=uu2*vv-uv*uv;
        if(!(uu2>0)||!(vv>0)||!(determinant>128*epsilon*uu2*vv))return false;
        const double yu=inner(data,u),yv=inner(data,v),b=(yu*vv-yv*uv)/determinant,d=(yv*uu2-yu*uv)/determinant;
        const double sourceAmp=d+(sourceRelativeGround?q[4]*q[5]/pixelArea*b:0.);
        double cost=0.0;
        const double stdDev=std::sqrt(variance);
        outReduced.resize(count);
        for(unsigned j=0;j<count;++j) {
            const double r=data[j]-b*u[j]-d*v[j];cost+=r*r;
            outReduced[j]=r/stdDev;
        }
        return std::isfinite(cost)&&std::isfinite(b)&&std::isfinite(sourceAmp);
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
            bool ranParallelJac=false;
            const unsigned jacMaxW=regionalOperator==RegionalOperator::ConicFastV5&&certifiedArithmeticV2()
                ?ReticleNestedPoolGuardV1::maxWorkers(maximumPhaseWorkersV5):1u;
            if(jacMaxW>=2u&&phasePoolV5.syncEnvironment()) {
                std::array<Shape,14> slotShapes{};
                std::array<double,7> coordSpan{};
                bool canParallel=true;
                for(unsigned coordinate=0;coordinate<7;++coordinate) {
                    const double step=coordinate<4?p.pixelPitchRad/1024:coordinate<6?pixelArea/1024:cutStep/1024;
                    auto plus=current,minus=current;
                    plus[coordinate]=std::min(upper[coordinate],current[coordinate]+step);
                    minus[coordinate]=std::max(lower[coordinate],current[coordinate]-step);
                    coordSpan[coordinate]=plus[coordinate]-minus[coordinate];
                    if(!(coordSpan[coordinate]>0)){canParallel=false;break;}
                    const Shape gp=geometry(plus),gm=geometry(minus);
                    if(!valid(gp)||!valid(gm)||(gp[0]==gp[2]&&gp[1]==gp[3])||(gm[0]==gm[2]&&gm[1]==gm[3])){canParallel=false;break;}
                    if(coordinate<6&&(!path(gp,branch)||!path(gm,branch))){canParallel=false;break;}
                    slotShapes[2*coordinate]=gp;
                    slotShapes[2*coordinate+1]=gm;
                }
                if(canParallel) {
                    const double cutCur=current[6],cutP6=slotShapes[12][6],cutM6=slotShapes[13][6];
                    const Trajectory* curTraj=prepare(geometry(current),branch);
                    const Ground* gCur=curTraj?calibration(cutCur):nullptr;
                    const Ground* gP6=gCur?calibration(cutP6):nullptr;
                    const Ground* gM6=gP6?calibration(cutM6):nullptr;
                    if(!curTraj||!gCur||!gP6||!gM6) {
                        complete=false;
                        ranParallelJac=true;
                    } else {
                        const bool curNearHorizon=curTraj->minimumLevel<=cutCur+0.002&&curTraj->maximumLevel>=cutCur-0.002;
                        const bool p6Crosses=curTraj->minimumLevel<cutP6&&curTraj->maximumLevel>=cutP6;
                        const bool m6Crosses=curTraj->minimumLevel<cutM6&&curTraj->maximumLevel>=cutM6;
                        bool fieldsOk=true;
                        if(curNearHorizon)fieldsOk=fieldsOk&&(groundField(cutCur)!=nullptr);
                        if(p6Crosses)fieldsOk=fieldsOk&&(groundField(cutP6)!=nullptr);
                        if(m6Crosses)fieldsOk=fieldsOk&&(groundField(cutM6)!=nullptr);
                        if(!fieldsOk) {
                            complete=false;
                            ranParallelJac=true;
                        } else {
                            const auto findFull=[&](double cut) -> const std::vector<double>* {
                                for(const auto& e:groundFieldCache)if(exactScalar(e.cutoff,cut))return &e.field;
                                return nullptr;
                            };
                            const std::vector<double>* fullCur=curNearHorizon?findFull(cutCur):nullptr;
                            const std::vector<double>* fullP6=p6Crosses?findFull(cutP6):nullptr;
                            const std::vector<double>* fullM6=m6Crosses?findFull(cutM6):nullptr;
                            if(fullCur)(void)groundSubexposuresV5(cutCur,*fullCur);
                            if(fullP6)(void)groundSubexposuresV5(cutP6,*fullP6);
                            if(fullM6)(void)groundSubexposuresV5(cutM6,*fullM6);
                            const auto findSub=[&](double cut) -> const std::vector<double>* {
                                for(const auto& e:groundSubCacheV5)if(exactScalar(e.cutoff,cut))return &e.subexposure;
                                return nullptr;
                            };
                            const std::vector<double>* subCur=fullCur?findSub(cutCur):nullptr;
                            const std::vector<double>* subP6=fullP6?findSub(cutP6):nullptr;
                            const std::vector<double>* subM6=fullM6?findSub(cutM6):nullptr;
                            std::shared_ptr<conic_moments_v4::Bank> bCur=fullCur?getOrCreatePhaseBankV5(cutCur):nullptr;
                            std::shared_ptr<conic_moments_v4::Bank> bP6=fullP6?getOrCreatePhaseBankV5(cutP6):nullptr;
                            std::shared_ptr<conic_moments_v4::Bank> bM6=fullM6?getOrCreatePhaseBankV5(cutM6):nullptr;
                            // Collect only the cold degree-3 columns touched by subexposures that
                            // actually straddle each cutoff so we never warm unused sky/ground columns.
                            struct WarmTask {const conic_moments_v4::Bank* bank;unsigned col;};
                            std::vector<WarmTask> warmTasks;
                            std::array<CachedPhaseBankV5*,3> warmEntries{};
                            std::array<std::pair<unsigned,unsigned>,3> warmRanges{};
                            unsigned warmEntryCount=0u;
                            const auto collectWarm=[&](double cut,const std::shared_ptr<conic_moments_v4::Bank>& bk,double padX,double wBox) {
                                if(!bk)return;
                                double minX=infinity,maxX=-infinity;
                                const double marginW=9*p.psfSigmaRad,hxW=.5*p.width*p.pixelPitchRad,hyW=.5*p.height*p.pixelPitchRad;
                                const double fLW=std::max(-1.55,-hxW-marginW),fRW=std::min(1.55,hxW+marginW);
                                const double fBW=std::max(-1.55,-hyW-marginW),fTW=std::min(1.55,hyW+marginW);
                                const bool hasSub=!curTraj->subexposureSource.empty();
                                for(std::size_t ti=0;ti<curTraj->x.size();++ti) {
                                    if(curTraj->levelMin[ti]>=cut+padX)continue;
                                    if(hasSub&&curTraj->levelMax[ti]<cut-padX&&
                                       curTraj->x[ti]-.5*wBox>=fLW&&curTraj->x[ti]+.5*wBox<=fRW&&
                                       curTraj->y[ti]-.5*curTraj->shape[5]>=fBW&&curTraj->y[ti]+.5*curTraj->shape[5]<=fTW)continue;
                                    if(curTraj->x[ti]<minX)minX=curTraj->x[ti];
                                    if(curTraj->x[ti]>maxX)maxX=curTraj->x[ti];
                                }
                                if(!(minX<=maxX))return;
                                CachedPhaseBankV5* entry=nullptr;
                                for(auto& item:phaseBankCacheV5)if(exactScalar(item.cutoff,cut)){entry=&item;break;}
                                const double rLeft=std::max(fLW,minX-padX-.5*wBox),rRight=std::min(fRW,maxX+padX+.5*wBox);
                                unsigned cBegin=p.width,cEnd=0;
                                for(unsigned col=0;col<p.width;++col) {
                                    const double px0=(col-.5*p.width)*p.pixelPitchRad,px1=px0+p.pixelPitchRad;
                                    const double cL=std::max(rLeft,std::max(-1.55,px0-marginW));
                                    const double cR=std::min(rRight,std::min(1.55,px1+marginW));
                                    if(cR>cL) {
                                        if(col<cBegin)cBegin=col;
                                        cEnd=col+1;
                                        if(!entry||!entry->columnWarmed(col))warmTasks.push_back({bk.get(),col});
                                    }
                                }
                                if(entry&&cBegin<cEnd&&warmEntryCount<3u) {
                                    warmEntries[warmEntryCount]=entry;
                                    warmRanges[warmEntryCount]={cBegin,cEnd};
                                    ++warmEntryCount;
                                }
                            };
                            if(bCur)collectWarm(cutCur,bCur,2.0*p.pixelPitchRad/1024.0,curTraj->shape[4]+2.0*p.pixelPitchRad/1024.0);
                            if(bP6)collectWarm(cutP6,bP6,0.0,curTraj->shape[4]);
                            if(bM6)collectWarm(cutM6,bM6,0.0,curTraj->shape[4]);
                            static constexpr std::array<unsigned,14> slotOrder{{12,13,0,1,2,3,4,5,6,7,8,9,10,11}};
                            std::array<std::vector<double>,14> slotReduced{};
                            std::array<bool,14> slotOk{};
                            std::array<bool,14> slotRefused{};
                            std::array<bool,14> slotFallback{};
                            std::array<JacSlotStatsV5,14> slotStats{};
                            std::array<std::exception_ptr,maximumPhaseWorkersV5> slotErrors{};
                            std::atomic<std::size_t> nextWarm{0u};
                            std::atomic<unsigned> nextSlot{0u};
                            const unsigned jacWorkers=std::min(jacMaxW,14u);
                            const auto runJac=[&](unsigned worker) noexcept {
                                try {
                                    while(true) {
                                        const std::size_t wi=nextWarm.fetch_add(1u,std::memory_order_relaxed);
                                        if(wi>=warmTasks.size())break;
                                        warmTasks[wi].bank->ensureColumnV5(warmTasks[wi].col);
                                    }
                                    auto& ws=phaseWorkspacesV5[worker];
                                    auto& scratch=jacScratchV5[worker];
                                    while(true) {
                                        const unsigned orderIdx=nextSlot.fetch_add(1u,std::memory_order_relaxed);
                                        if(orderIdx>=14u)break;
                                        const unsigned s=slotOrder[orderIdx];
                                        const bool isP6=(s==12u),isM6=(s==13u);
                                        const Ground& g=isP6?*gP6:(isM6?*gM6:*gCur);
                                        const std::vector<double>* fl=isP6?fullP6:(isM6?fullM6:fullCur);
                                        const std::vector<double>* gs=isP6?subP6:(isM6?subM6:subCur);
                                        const conic_moments_v4::Bank* bk=isP6?bP6.get():(isM6?bM6.get():bCur.get());
                                        slotOk[s]=evaluateJacobianSlotV5(slotShapes[s],branch,curTraj,isP6||isM6,g,fl,gs,bk,ws,scratch,
                                            slotReduced[s],slotRefused[s],slotFallback[s],slotStats[s]);
                                    }
                                } catch(...) {
                                    slotErrors[worker]=std::current_exception();
                                }
                            };
                            const auto tJac0=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
                            phasePoolV5.run(jacWorkers,runJac);
                            const double jacWallUs=profile.enabled?std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-tJac0).count():0.0;
                            for(unsigned i=0;i<warmEntryCount;++i)warmEntries[i]->markWarmedRange(warmRanges[i].first,warmRanges[i].second);
                            for(const auto& err:slotErrors)if(err)std::rethrow_exception(err);
                            bool anyFallback=false,anyRefused=false,allOk=true;
                            for(unsigned s=0;s<14u;++s) {
                                if(slotFallback[s])anyFallback=true;
                                if(slotRefused[s])anyRefused=true;
                                if(!slotOk[s])allOk=false;
                            }
                            if(!anyFallback) {
                                ranParallelJac=true;
                                for(unsigned s=0;s<14u;++s) {
                                    ++diagnostic.basisEvaluations;
                                    diagnostic.regionalFields+=slotStats[s].regions;
                                    if(profile.enabled) {
                                        profile.conicRequests+=slotStats[s].requests;
                                        profile.conicFields+=slotStats[s].fields;
                                        profile.conicFallbacks+=slotStats[s].fallbacks;
                                        profile.parallelFields+=slotStats[s].regions;
                                        profile.conicRenderUs+=slotStats[s].renderUs;
                                        profile.regionalAccelerated+=slotStats[s].accelerated;
                                        profile.regionalNative+=slotStats[s].native;
                                        profile.conicMaximumError=std::max(profile.conicMaximumError,slotStats[s].maximumError);
                                    }
                                }
                                if(profile.enabled) {
                                    profile.calls[LocalProfileV2::BasisWork]+=14u;
                                    profile.micros[LocalProfileV2::BasisWork]+=jacWallUs;
                                    profile.micros[LocalProfileV2::Regional]+=jacWallUs;
                                    ++profile.parallelBatches;
                                    profile.parallelWorkersPeak=std::max<std::size_t>(profile.parallelWorkersPeak,jacWorkers);
                                }
                                if(anyRefused){diagnostic.refusal=22;complete=false;}
                                else if(!allOk){complete=false;}
                                else {
                                    for(unsigned coordinate=0;coordinate<7;++coordinate) {
                                        jacobian[coordinate].resize(count);
                                        const auto& a=slotReduced[2*coordinate];
                                        const auto& b=slotReduced[2*coordinate+1];
                                        const double span=coordSpan[coordinate];
                                        for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=(a[j]-b[j])/span;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(!ranParallelJac) {
                for(unsigned coordinate=0;coordinate<7;++coordinate) {
                    const double step=coordinate<4?p.pixelPitchRad/1024:coordinate<6?pixelArea/1024:cutStep/1024;
                    auto plus=current,minus=current;plus[coordinate]=std::min(upper[coordinate],current[coordinate]+step);
                    minus[coordinate]=std::max(lower[coordinate],current[coordinate]-step);
                    std::vector<double> a,b;
                    if(!(plus[coordinate]>minus[coordinate])||!solve(geometry(plus),branch,&a)||!solve(geometry(minus),branch,&b)) {complete=false;break;}
                    jacobian[coordinate].resize(count);
                    for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=(a[j]-b[j])/(plus[coordinate]-minus[coordinate]);
                }
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
        // Run every original cold-start continuation first. Only a remaining
        // failed residual can request independent causal measured-history
        // continuations; physical budgets, null updates, 7D curvature, finite
        // mirrors, domain/profile screens and admission remain unchanged below.
        if(priorStart&&(!std::isfinite(selected.fit.cost)||selected.fit.cost/variance>diagnostic.residualBound)) {
            std::vector<double> priorCutoffs{(*priorStart)[6]};
            if(!exactScalar(nullCutoff,(*priorStart)[6]))priorCutoffs.push_back(nullCutoff);
            std::vector<std::array<double,2>> priorExtents{{(*priorStart)[4],(*priorStart)[5]}};
            if((*priorStart)[4]!=(*priorStart)[5]) {
                if((*priorStart)[5]<=2*halfX&&(*priorStart)[4]<=2*halfY)
                    priorExtents.push_back({(*priorStart)[5],(*priorStart)[4]});
                const double side=std::max((*priorStart)[4],(*priorStart)[5]);
                if(side<=2*halfX&&side<=2*halfY)priorExtents.push_back({side,side});
            }
            for(const double cutoff:priorCutoffs) {
                for(const auto& extent:priorExtents) {
                    auto initial=*priorStart;
                    initial[4]=extent[0];initial[5]=extent[1];initial[6]=cutoff;
                    continuation(initial,0);
                    if(diagnostic.refusal==21)return selected;
                    if(std::isfinite(selected.fit.cost)&&selected.fit.cost/variance<=diagnostic.residualBound)break;
                }
                if(std::isfinite(selected.fit.cost)&&selected.fit.cost/variance<=diagnostic.residualBound)break;
            }
        }
        return selected;
    }
    struct Information {double capture=0,first=0,last=0,readRate=0,firstState=0,lastState=0;};
    bool evaluateInfoSlotV5(const Shape& shape,const Trajectory& traj,const Ground& ground,const Fit& fit,
        const std::vector<double>* fullPtrVec,const std::vector<double>* groundSubVec,
        const conic_moments_v4::Bank* bankPtr,conic_moments_v4::FieldWorkspaceV5& workspace,
        JacWorkerScratchV5& scratch,std::vector<double>& outPred,bool& outRefused,bool& outNeedFallback,JacSlotStatsV5& stats) const {
        if(traj.path.stationary){outNeedFallback=true;return false;}
        const std::size_t samples=static_cast<std::size_t>(count)*p.exposureSamples;
        const double sigma=p.psfSigmaRad;
        const double margin=9*sigma,hx=.5*p.width*p.pixelPitchRad,hy=.5*p.height*p.pixelPitchRad;
        const double fLeft=std::max(-1.55,-hx-margin),fRight=std::min(1.55,hx+margin);
        const double fBottom=std::max(-1.55,-hy-margin),fTop=std::min(1.55,hy+margin);
        const std::vector<double>* p0Vec=nullptr;
        bool sourceRelativeGround=false;
        if(shape[4]==0||shape[5]==0||traj.minimumLevel>=shape[6]) {
            p0Vec=&ground.wave;
        } else if(traj.maximumLevel<shape[6]) {
            p0Vec=&ground.wave;
            sourceRelativeGround=true;
        } else {
            if(!bankPtr||!fullPtrVec||!groundSubVec){outNeedFallback=true;return false;}
            const auto& full=*fullPtrVec;
            const auto& groundSub=*groundSubVec;
            const auto nativeForward=unit(forward),nativeUp=unit(up);
            auto nativeRight=unit(cross(nativeForward,nativeUp));
            if(norm(nativeRight)<.5)nativeRight=unit(cross(nativeForward,std::abs(nativeForward.z)<.9?V{0,0,1}:V{0,1,0}));
            const auto nativeVertical=unit(cross(nativeRight,nativeForward));
            const double aa=dot(nativeForward,nativeUp),bb=dot(nativeRight,nativeUp),cc=dot(nativeVertical,nativeUp);
            const auto eligible=[&](const std::array<double,4>& region) {
                const double leftE=std::max(fLeft,region[0]),rightE=std::min(fRight,region[1]);
                const double bottomE=std::max(fBottom,region[2]),topE=std::min(fTop,region[3]);
                if(!(rightE>leftE)||!(topE>bottomE))return true;
                const double maxTx=std::max(std::abs(std::tan(leftE)),std::abs(std::tan(rightE)));
                const double maxTy=std::max(std::abs(std::tan(bottomE)),std::abs(std::tan(topE)));
                const double maxN=std::nextafter(std::abs(aa)+std::nextafter(std::abs(bb)*maxTx,infinity),infinity);
                return std::nextafter(cc,-infinity)>std::nextafter(maxN*maxTy,infinity);
            };
            const double areaScale=(shape[4]*shape[5])/pixelArea;
            const double* groundSubPtr=groundSub.data();
            const double* subSourcePtr=traj.subexposureSource.size()==samples?traj.subexposureSource.data():nullptr;
            const double* fullPtr=full.data();
            auto& u=scratch.u;
            u.assign(count,0.0);
            for(unsigned j=0;j<count;++j) {
                double value=0.0;
                for(unsigned k=0;k<p.exposureSamples;++k) {
                    const auto ti=static_cast<std::size_t>(j)*p.exposureSamples+k;
                    if(traj.levelMin[ti]>=shape[6]){value+=groundSubPtr[ti];continue;}
                    if(subSourcePtr&&traj.levelMax[ti]<shape[6]&&
                       traj.x[ti]-.5*shape[4]>=fLeft&&traj.x[ti]+.5*shape[4]<=fRight&&
                       traj.y[ti]-.5*shape[5]>=fBottom&&traj.y[ti]+.5*shape[5]<=fTop) {
                        value+=groundSubPtr[ti]-areaScale*subSourcePtr[ti];
                        continue;
                    }
                    ++stats.regions;++stats.requests;
                    const double* occultedPtr=nullptr;
                    std::optional<std::vector<double>> fallbackOcculted;
                    const std::array<double,4> region{{traj.x[ti]-.5*shape[4],traj.x[ti]+.5*shape[4],traj.y[ti]-.5*shape[5],traj.y[ti]+.5*shape[5]}};
                    if(eligible(region)) {
                        const auto began=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
                        const bool hasPre=ti<traj.precomputedRegions.size()&&traj.precomputedRegions[ti].valid;
                        const bool ok=hasPre
                            ?bankPtr->evaluateFastPreparedIntoV5(traj.precomputedRegions[ti],workspace)
                            :bankPtr->evaluateFastIntoV5(region,workspace);
                        if(profile.enabled)stats.renderUs+=std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-began).count();
                        if(ok&&workspace.field.size()==pixels) {
                            ++stats.fields;stats.maximumError=std::max(stats.maximumError,workspace.maxError);
                            occultedPtr=workspace.field.data();
                        }
                    }
                    if(!occultedPtr) {
                        ++stats.fallbacks;
                        bool accelerated=false;
                        fallbackOcculted=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,forward,up,shape[6],region,[&](){return &full;},&accelerated);
                        if(accelerated)++stats.accelerated;else ++stats.native;
                        if(fallbackOcculted)occultedPtr=fallbackOcculted->data();
                    }
                    if(!occultedPtr){outRefused=true;return false;}
                    if(sparseMaskReadyV5&&powerOfTwoExposureV5) {
                        const std::size_t beginIdx=sparseOffsetV5[ti],endIdx=sparseOffsetV5[ti+1];
                        accumulateSparseDiffV5(value,fullPtr,occultedPtr,
                            sparsePixelV5.data(),sparseSolidV5.data(),sparseWeightV5.data(),
                            beginIdx,endIdx,pixelArea,invExposureV5);
                    } else {
                        const double* weightsPtr=optics.timeWeights.data()+ti*pixels;
                        const double* solidPtr=optics.solidAngles.data();
                        for(std::size_t i=0;i<pixels;++i) {
                            const double field=fullPtr[i]-occultedPtr[i];
                            if(field==0.0)continue;
                            value+=field*solidPtr[i]*weightsPtr[i]/pixelArea/p.exposureSamples;
                        }
                    }
                }
                u[j]=value;
            }
            {
                double previous=0.0;
                for(auto& val:u){previous+=alpha*(val-previous);val=previous;}
            }
            project(u,optics.nuisance);
            p0Vec=&u;
        }
        const double d=effectiveSource(shape,sourceRelativeGround,fit);
        outPred.resize(count);
        const double* p0=p0Vec->data();
        const double* p1=traj.projectedSource.data();
        double* dst=outPred.data();
        for(unsigned j=0;j<count;++j)dst[j]=fit.background*p0[j]+d*p1[j];
        return true;
    }

    std::optional<Information> information(const Shape& q,unsigned branch,const Fit& fit) {
        LocalTimerV2 timer(&profile,LocalProfileV2::Information);
        BasisView baseView;if(!basisView(q,branch,baseView))return std::nullopt;
        std::array<std::vector<double>,2> amplitude{{*baseView.proj0,*baseView.proj1}};
        for(unsigned colIdx=0;colIdx<2;++colIdx) {
            auto& column=amplitude[colIdx];
            if(colIdx==1) {
                for(unsigned pass=0;pass<2;++pass) {
                    const double a=inner(column,amplitude[0]);
                    for(unsigned j=0;j<count;++j)column[j]-=a*amplitude[0][j];
                }
            }
            const double length=std::sqrt(inner(column,column));if(!(length>128*epsilon))return std::nullopt;
            for(auto& value:column)value/=length;
        }
        const Shape current=squared(q);
        std::array<std::vector<double>,7> jacobian;Shape norms{};
        std::array<std::array<V,7>,6> target{};
        std::array<Shape,14> slotShapes{};
        std::array<double,7> coordDivisor{};
        for(unsigned coordinate=0;coordinate<7;++coordinate) {
            const double step=coordinate<4?p.pixelPitchRad/1024:coordinate<6?pixelArea/1024:cutStep/1024;
            auto plus=current,minus=current;plus[coordinate]+=step;minus[coordinate]-=step;
            if(coordinate==4||coordinate==5)minus[coordinate]=std::max(0.,minus[coordinate]);
            if(coordinate==6){plus[6]=std::min(high,plus[6]);minus[6]=std::max(low,minus[6]);}
            coordDivisor[coordinate]=plus[coordinate]-minus[coordinate];
            slotShapes[2*coordinate]=geometry(plus);
            slotShapes[2*coordinate+1]=geometry(minus);
        }
        bool ranParallelInfo=false;
        std::array<std::vector<double>,14> slotPreds{};
        std::array<const Trajectory*,14> slotTrajs{};
        const unsigned infoMaxW=regionalOperator==RegionalOperator::ConicFastV5&&certifiedArithmeticV2()
            ?ReticleNestedPoolGuardV1::maxWorkers(maximumPhaseWorkersV5):1u;
        if(infoMaxW>=2u&&phasePoolV5.syncEnvironment()) {
            bool allInputsOk=true;
            for(unsigned coord=0;coord<7u;++coord) {
                if(!(coordDivisor[coord]>0)||!valid(slotShapes[2*coord])||!valid(slotShapes[2*coord+1])) {
                    allInputsOk=false;break;
                }
            }
            if(allInputsOk) {
                const double cutCur=q[6],cutP6=slotShapes[12][6],cutM6=slotShapes[13][6];
                const Trajectory* baseTraj=prepare(q,branch);
                const Ground* gCur=baseTraj?calibration(cutCur):nullptr;
                const Ground* gP6=gCur?calibration(cutP6):nullptr;
                const Ground* gM6=gP6?calibration(cutM6):nullptr;
                if(baseTraj&&gCur&&gP6&&gM6) {
                    bool trajsOk=true,anyRegional=false;
                    for(unsigned s=0;s<12u;++s) {
                        slotTrajs[s]=prepare(slotShapes[s],branch);
                        if(!slotTrajs[s]||slotTrajs[s]->path.stationary){trajsOk=false;break;}
                        if(slotShapes[s][4]>0&&slotShapes[s][5]>0&&
                           slotTrajs[s]->minimumLevel<cutCur&&slotTrajs[s]->maximumLevel>=cutCur)anyRegional=true;
                    }
                    slotTrajs[12]=baseTraj;slotTrajs[13]=baseTraj;
                    if(trajsOk&&baseTraj->path.stationary)trajsOk=false;
                    const bool p6Crosses=trajsOk&&q[4]>0&&q[5]>0&&baseTraj->minimumLevel<cutP6&&baseTraj->maximumLevel>=cutP6;
                    const bool m6Crosses=trajsOk&&q[4]>0&&q[5]>0&&baseTraj->minimumLevel<cutM6&&baseTraj->maximumLevel>=cutM6;
                    if(p6Crosses||m6Crosses)anyRegional=true;
                    if(trajsOk&&anyRegional) {
                        const bool curCrosses=[&](){
                            for(unsigned s=0;s<12u;++s)
                                if(slotShapes[s][4]>0&&slotShapes[s][5]>0&&slotTrajs[s]->minimumLevel<cutCur&&slotTrajs[s]->maximumLevel>=cutCur)
                                    return true;
                            return false;
                        }();
                        bool fieldsOk=true;
                        if(curCrosses)fieldsOk=fieldsOk&&(groundField(cutCur)!=nullptr);
                        if(p6Crosses)fieldsOk=fieldsOk&&(groundField(cutP6)!=nullptr);
                        if(m6Crosses)fieldsOk=fieldsOk&&(groundField(cutM6)!=nullptr);
                        if(fieldsOk) {
                            const auto findFull=[&](double cut) -> const std::vector<double>* {
                                for(const auto& e:groundFieldCache)if(exactScalar(e.cutoff,cut))return &e.field;
                                return nullptr;
                            };
                            const std::vector<double>* fullCur=curCrosses?findFull(cutCur):nullptr;
                            const std::vector<double>* fullP6=p6Crosses?findFull(cutP6):nullptr;
                            const std::vector<double>* fullM6=m6Crosses?findFull(cutM6):nullptr;
                            if(fullCur)(void)groundSubexposuresV5(cutCur,*fullCur);
                            if(fullP6)(void)groundSubexposuresV5(cutP6,*fullP6);
                            if(fullM6)(void)groundSubexposuresV5(cutM6,*fullM6);
                            const auto findSub=[&](double cut) -> const std::vector<double>* {
                                for(const auto& e:groundSubCacheV5)if(exactScalar(e.cutoff,cut))return &e.subexposure;
                                return nullptr;
                            };
                            const std::vector<double>* subCur=fullCur?findSub(cutCur):nullptr;
                            const std::vector<double>* subP6=fullP6?findSub(cutP6):nullptr;
                            const std::vector<double>* subM6=fullM6?findSub(cutM6):nullptr;
                            std::shared_ptr<conic_moments_v4::Bank> bCur=fullCur?getOrCreatePhaseBankV5(cutCur):nullptr;
                            std::shared_ptr<conic_moments_v4::Bank> bP6=fullP6?getOrCreatePhaseBankV5(cutP6):nullptr;
                            std::shared_ptr<conic_moments_v4::Bank> bM6=fullM6?getOrCreatePhaseBankV5(cutM6):nullptr;
                            struct WarmTask {const conic_moments_v4::Bank* bank;unsigned col;};
                            std::vector<WarmTask> warmTasks;
                            std::array<CachedPhaseBankV5*,3> warmEntries{};
                            std::array<std::pair<unsigned,unsigned>,3> warmRanges{};
                            unsigned warmEntryCount=0u;
                            const auto collectWarmInfo=[&](double cut,const std::shared_ptr<conic_moments_v4::Bank>& bk,const Trajectory* tr,double padX,double wBox) {
                                if(!bk||!tr)return;
                                double minX=infinity,maxX=-infinity;
                                const double marginW=9*p.psfSigmaRad,hxW=.5*p.width*p.pixelPitchRad,hyW=.5*p.height*p.pixelPitchRad;
                                const double fLW=std::max(-1.55,-hxW-marginW),fRW=std::min(1.55,hxW+marginW);
                                const double fBW=std::max(-1.55,-hyW-marginW),fTW=std::min(1.55,hyW+marginW);
                                const bool hasSub=!tr->subexposureSource.empty();
                                for(std::size_t ti=0;ti<tr->x.size();++ti) {
                                    if(tr->levelMin[ti]>=cut+padX)continue;
                                    if(hasSub&&tr->levelMax[ti]<cut-padX&&
                                       tr->x[ti]-.5*wBox>=fLW&&tr->x[ti]+.5*wBox<=fRW&&
                                       tr->y[ti]-.5*tr->shape[5]>=fBW&&tr->y[ti]+.5*tr->shape[5]<=fTW)continue;
                                    if(tr->x[ti]<minX)minX=tr->x[ti];
                                    if(tr->x[ti]>maxX)maxX=tr->x[ti];
                                }
                                if(!(minX<=maxX))return;
                                CachedPhaseBankV5* entry=nullptr;
                                for(auto& item:phaseBankCacheV5)if(exactScalar(item.cutoff,cut)){entry=&item;break;}
                                const double rLeft=std::max(fLW,minX-padX-.5*wBox),rRight=std::min(fRW,maxX+padX+.5*wBox);
                                unsigned cBegin=p.width,cEnd=0;
                                for(unsigned col=0;col<p.width;++col) {
                                    const double px0=(col-.5*p.width)*p.pixelPitchRad,px1=px0+p.pixelPitchRad;
                                    const double cL=std::max(rLeft,std::max(-1.55,px0-marginW));
                                    const double cR=std::min(rRight,std::min(1.55,px1+marginW));
                                    if(cR>cL) {
                                        if(col<cBegin)cBegin=col;
                                        cEnd=col+1;
                                        if(!entry||!entry->columnWarmed(col))warmTasks.push_back({bk.get(),col});
                                    }
                                }
                                if(entry&&cBegin<cEnd&&warmEntryCount<3u) {
                                    warmEntries[warmEntryCount]=entry;
                                    warmRanges[warmEntryCount]={cBegin,cEnd};
                                    ++warmEntryCount;
                                }
                            };
                            if(bCur)collectWarmInfo(cutCur,bCur,baseTraj,2.0*p.pixelPitchRad/1024.0,q[4]+2.0*p.pixelPitchRad/1024.0);
                            if(bP6)collectWarmInfo(cutP6,bP6,baseTraj,0.0,q[4]);
                            if(bM6)collectWarmInfo(cutM6,bM6,baseTraj,0.0,q[4]);
                            std::array<bool,14> slotOk{},slotRefused{},slotFallback{};
                            std::array<JacSlotStatsV5,14> slotStats{};
                            std::array<std::exception_ptr,maximumPhaseWorkersV5> slotErrors{};
                            std::atomic<std::size_t> nextWarm{0u};
                            std::atomic<unsigned> nextSlot{0u};
                            const unsigned infoWorkers=std::min(infoMaxW,14u);
                            const auto runInfo=[&](unsigned worker) noexcept {
                                try {
                                    while(true) {
                                        const std::size_t wi=nextWarm.fetch_add(1u,std::memory_order_relaxed);
                                        if(wi>=warmTasks.size())break;
                                        warmTasks[wi].bank->ensureColumnV5(warmTasks[wi].col);
                                    }
                                    auto& ws=phaseWorkspacesV5[worker];
                                    auto& scratch=jacScratchV5[worker];
                                    while(true) {
                                        const unsigned s=nextSlot.fetch_add(1u,std::memory_order_relaxed);
                                        if(s>=14u)break;
                                        const bool isP6=(s==12u),isM6=(s==13u);
                                        const Ground& g=isP6?*gP6:(isM6?*gM6:*gCur);
                                        const std::vector<double>* fl=isP6?fullP6:(isM6?fullM6:fullCur);
                                        const std::vector<double>* gs=isP6?subP6:(isM6?subM6:subCur);
                                        const conic_moments_v4::Bank* bk=isP6?bP6.get():(isM6?bM6.get():bCur.get());
                                        slotOk[s]=evaluateInfoSlotV5(slotShapes[s],*slotTrajs[s],g,fit,fl,gs,bk,ws,scratch,
                                            slotPreds[s],slotRefused[s],slotFallback[s],slotStats[s]);
                                    }
                                } catch(...) {
                                    slotErrors[worker]=std::current_exception();
                                }
                            };
                            const auto tInfo0=profile.enabled?LocalProfileV2::Clock::now():LocalProfileV2::Clock::time_point{};
                            phasePoolV5.run(infoWorkers,runInfo);
                            const double infoWallUs=profile.enabled?std::chrono::duration<double,std::micro>(LocalProfileV2::Clock::now()-tInfo0).count():0.0;
                            for(unsigned i=0;i<warmEntryCount;++i)warmEntries[i]->markWarmedRange(warmRanges[i].first,warmRanges[i].second);
                            for(const auto& err:slotErrors)if(err)std::rethrow_exception(err);
                            bool anyFallback=false,anyRefused=false,allOk=true;
                            for(unsigned s=0;s<14u;++s) {
                                if(slotFallback[s])anyFallback=true;
                                if(slotRefused[s])anyRefused=true;
                                if(!slotOk[s])allOk=false;
                            }
                            if(!anyFallback) {
                                ranParallelInfo=true;
                                for(unsigned s=0;s<14u;++s) {
                                    ++diagnostic.basisEvaluations;
                                    diagnostic.regionalFields+=slotStats[s].regions;
                                    if(profile.enabled) {
                                        profile.conicRequests+=slotStats[s].requests;
                                        profile.conicFields+=slotStats[s].fields;
                                        profile.conicFallbacks+=slotStats[s].fallbacks;
                                        profile.parallelFields+=slotStats[s].regions;
                                        profile.conicRenderUs+=slotStats[s].renderUs;
                                        profile.regionalAccelerated+=slotStats[s].accelerated;
                                        profile.regionalNative+=slotStats[s].native;
                                        profile.conicMaximumError=std::max(profile.conicMaximumError,slotStats[s].maximumError);
                                    }
                                }
                                if(profile.enabled) {
                                    profile.calls[LocalProfileV2::BasisWork]+=14u;
                                    profile.micros[LocalProfileV2::BasisWork]+=infoWallUs;
                                    profile.micros[LocalProfileV2::Regional]+=infoWallUs;
                                    ++profile.parallelBatches;
                                    profile.parallelWorkersPeak=std::max<std::size_t>(profile.parallelWorkersPeak,infoWorkers);
                                }
                                if(anyRefused){diagnostic.refusal=22;return std::nullopt;}
                                if(!allOk)return std::nullopt;
                            }
                        }
                    }
                }
            }
        }
        std::vector<double> predA(count),predB(count);
        const auto fillPrediction=[&](const Shape& shape,std::vector<double>& out)->bool {
            BasisView view;if(!basisView(shape,branch,view))return false;
            const double d=effectiveSource(shape,view.sourceRelativeGround,fit);
            const double* p0=view.proj0->data();
            const double* p1=view.proj1->data();
            double* dst=out.data();
            for(unsigned j=0;j<count;++j)dst[j]=fit.background*p0[j]+d*p1[j];
            return true;
        };
        for(unsigned coordinate=0;coordinate<7;++coordinate) {
            const auto& gp=slotShapes[2*coordinate];
            const auto& gm=slotShapes[2*coordinate+1];
            const double divisor=coordDivisor[coordinate];
            if(!(divisor>0))return std::nullopt;
            const double* aPtr=nullptr;
            const double* bPtr=nullptr;
            if(ranParallelInfo) {
                aPtr=slotPreds[2*coordinate].data();
                bPtr=slotPreds[2*coordinate+1].data();
            } else {
                if(!fillPrediction(gp,predA)||!fillPrediction(gm,predB))return std::nullopt;
                aPtr=predA.data();
                bPtr=predB.data();
            }
            jacobian[coordinate].resize(count);
            for(unsigned j=0;j<count;++j)jacobian[coordinate][j]=(aPtr[j]-bPtr[j])/divisor;
            for(unsigned pass=0;pass<2;++pass)for(const auto& column:amplitude) {
                const double coefficient=inner(jacobian[coordinate],column);
                for(unsigned j=0;j<count;++j)jacobian[coordinate][j]-=coefficient*column[j];
            }
            double sum2=0.,maxAbs=0.;
            for(const auto value:jacobian[coordinate]) {
                const double av=std::abs(value);
                if(av>maxAbs)maxAbs=av;
                sum2+=av*av;
            }
            if(maxAbs>0.) {
                if(maxAbs>1e-150&&maxAbs<1e150)norms[coordinate]=std::sqrt(sum2);
                else {
                    double scaled=0.;
                    for(const auto value:jacobian[coordinate]){const double t=value/maxAbs;scaled+=t*t;}
                    norms[coordinate]=maxAbs*std::sqrt(scaled);
                }
            } else norms[coordinate]=0.;
            if(!std::isfinite(norms[coordinate]))return std::nullopt;
            if(norms[coordinate]==0)norms[coordinate]=1;
            if(coordinate<4) {
                const Trajectory* tp=ranParallelInfo?slotTrajs[2*coordinate]:(modeCache?prepare(gp,branch):nullptr);
                const Trajectory* tm=ranParallelInfo?slotTrajs[2*coordinate+1]:(modeCache?prepare(gm,branch):nullptr);
                std::optional<RayPath> rpOpt,rmOpt;
                const RayPath* rp=tp?&tp->path:((rpOpt=path(gp,branch))?&*rpOpt:nullptr);
                const RayPath* rm=tm?&tm->path:((rmOpt=path(gm,branch))?&*rmOpt:nullptr);
                if(!rp||!rm)return std::nullopt;
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
        // bounded mode/derivative trajectories in O(1), never spatial fields.
        modeCache=true;sources.clear();sources.reserve(1024);modeTrajectories.clear();modeTrajMap.clear();lastShape.reset();lastBasis.reset();
        // Exact O(1) incremental evaluator of ThermalReticleReceiverV1::finiteMirrorEnvelopeV1
        // so large mode banks avoid O(M^2) full-vector rescans while preserving 0-ULP parity.
        struct IncrementalMirrorEnvelopeV5 {
            const ThermalReticleConfigV1& cfg;
            double hx=0,hy=0,lowX=0,highX=0,lowY=0,highY=0,envelope=0,selectedClearance=0;
            V selectedRay{};
            bool initialized=false,ok=false;
            explicit IncrementalMirrorEnvelopeV5(const ThermalReticleConfigV1& config) noexcept
                : cfg(config), hx(.5*config.detector.width*config.detector.pixelPitchRad),
                  hy(.5*config.detector.height*config.detector.pixelPitchRad),
                  lowX(-hx), highX(hx), lowY(-hy), highY(hy),
                  ok(ThermalReticleReceiverV1::validConfig(config)) {}
            double clearance(V direction) const noexcept {
                double distance=std::atan(cfg.stopRadiusRad+cfg.scanRadiusRad)-
                    std::atan2(std::hypot(direction.y,direction.z),direction.x);
                for(const double sign:{-1.,1.}) {
                    distance=std::min(distance,std::asin(std::clamp(std::sin(hx)*direction.x+sign*std::cos(hx)*direction.y,-1.,1.)));
                    distance=std::min(distance,std::asin(std::clamp(std::sin(hy)*direction.x+sign*std::cos(hy)*direction.z,-1.,1.)));
                }
                return distance;
            }
            bool add(const std::array<double,5>& q) noexcept {
                if(!ok)return false;
                const auto& det=cfg.detector;
                if(!std::all_of(q.begin(),q.end(),[](double value){return std::isfinite(value);})||q[4]<=0||
                    q[2]<0||q[3]<0||q[2]>=hx||q[3]>=hy||
                    q[0]-.5*q[2]<=-hx||q[0]+.5*q[2]>=hx||q[1]-.5*q[3]<=-hy||q[1]+.5*q[3]>=hy){ok=false;return false;}
                const auto direction=unit(V{1,std::tan(q[0]),std::tan(q[1])});
                const double clr=clearance(direction);
                if(!initialized){selectedRay=direction;selectedClearance=clr;initialized=true;}
                if(!(det.detectionSigma*std::sqrt(q[4])<clr)){ok=false;return false;}
                lowX=std::max(lowX,q[0]-.5*q[2]);highX=std::min(highX,q[0]+.5*q[2]);
                lowY=std::max(lowY,q[1]-.5*q[3]);highY=std::min(highY,q[1]+.5*q[3]);
                if(lowX>highX||lowY>highY){ok=false;return false;}
                const double displacement=std::nextafter(std::atan2(norm(cross(selectedRay,direction)),dot(selectedRay,direction)),infinity);
                const double spread=std::nextafter(std::sqrt(q[4]),infinity);
                const double total=std::nextafter(spread+displacement,infinity);
                envelope=std::max(envelope,std::nextafter(total*total,infinity));
                if(!std::isfinite(envelope)||!(det.detectionSigma*std::sqrt(envelope)<selectedClearance)){ok=false;return false;}
                return true;
            }
        } mirrorEnv(c);
        std::vector<std::array<double,5>> modes;
        bool unresolved=searchUnresolved||std::any_of(diagnostic.rawBudget.begin(),diagnostic.rawBudget.end(),[](double x){return x<=0;});
        if(!unresolved) {
            if(const auto own=mode(selected.shape,selected.branch,selected.fit)) {
                modes.push_back(*own);
                if(!mirrorEnv.add(*own))unresolved=true;
            } else unresolved=true;
        }
        // Each independently optimized physical basin participates, including
        // basins that were initially worse than the global selected fit.
        for(const auto& local:localModes) {
            if(unresolved)break;
            if(local.branch==selected.branch&&exactCoordinates(local.shape,selected.shape))continue;
            if(local.fit.cost>selected.fit.cost+threshold2*variance)continue;
            if(const auto candidate=mode(local.shape,local.branch,local.fit)) {
                modes.push_back(*candidate);
                if(!mirrorEnv.add(*candidate))unresolved=true;
            } else unresolved=true;
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
        if(!unresolved){addGeometry(selected);for(const auto& local:localModes)addGeometry(local);}
        // A moving endpoint/domain or an unseen support-edge continuation is
        // an unresolved family, even if one fitted local covariance is finite.
        // Check these finite endpoint continuations before scanning the full
        // cutoff bank: once unresolved, no mirror envelope can be admitted.
        const auto profileContinuation=[&](Shape q) {
            if(unresolved)return;
            q=quantize(q);
            for(unsigned branch=0;branch<2&&!unresolved;++branch) {
                if(certifiedModeRefusal(q,branch,selected.fit.cost+threshold2*variance))continue;
                const auto alternative=solve(q,branch);
                if(alternative&&alternative->cost<=selected.fit.cost+threshold2*variance&&physical(q,branch,*alternative))unresolved=true;
            }
        };
        for(const auto& seed:profileSeeds) {
            if(unresolved)break;
            for(unsigned coordinate=0;coordinate<4&&!unresolved;++coordinate)for(const double sign:{-1.,1.}) {
                if(unresolved)break;
                auto q=seed.shape;q[coordinate]=sign*(coordinate%2?halfY:halfX);
                if(q[coordinate]!=seed.shape[coordinate])profileContinuation(q);
            }
            for(unsigned axis=0;axis<2&&!unresolved;++axis)for(const double sign:{-1.,1.}) {
                if(unresolved)break;
                const double half=axis?halfY:halfX,extent=seed.shape[axis+4];
                const double firstShift=std::max({-half-seed.shape[axis],-half-seed.shape[axis+2],sign>0?-.5*extent:.5*(extent-2*half)});
                const double lastShift=std::min({half-seed.shape[axis],half-seed.shape[axis+2],sign>0?.5*(2*half-extent):.5*extent});
                if(firstShift>lastShift)continue;
                for(const double shift:{firstShift,lastShift})if(!unresolved&&std::abs(shift)>angularLattice) {
                    auto q=seed.shape;q[axis]+=shift;q[axis+2]+=shift;q[axis+4]+=2*sign*shift;profileContinuation(q);
                }
            }
        }
        // Snapshot values are immutable: curvature adds cutoff neighbours but
        // never recursively grows the mode bank. All independent local basins
        // and all their reflected geometries visit the SAME complete cutoff
        // set, including rejected optimizer trials. Because finiteMirrorEnvelopeV1
        // is monotone-decreasing under added modes, scanning stops once
        // unresolved or once the mirror envelope is already refuted.
        std::vector<double> cutoffs;
        if(!unresolved){cutoffs.reserve(grounds.size());for(const auto& item:grounds)cutoffs.push_back(item.cutoff);}
        if(profile.enabled)std::fprintf(stderr,
            "MOVING_MODE_BANK {\"read\":%.17g,\"geometries\":%zu,\"cutoffs\":%zu,\"initial_modes\":%zu,\"unresolved\":%d,\"initial_envelope\":%d}\n",
            measured.timeS,geometries.size(),cutoffs.size(),modes.size(),unresolved,
            ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(c,modes).has_value());
        profile.checkpoint("modes.begin",selected.fit.cost/variance,unresolved);
        const auto visitModeCandidate=[&](const ModeGeometry& candidateGeometry,double cutoff) {
            auto q=candidateGeometry.shape;
            if(profile.enabled){++profile.modes;if(profile.modes%32==0)profile.checkpoint("modes.progress",selected.fit.cost/variance,unresolved);}
            q[6]=cutoff;
            if(candidateGeometry.branch==selected.branch&&exactCoordinates(q,selected.shape))return;
            if(certifiedModeRefusal(q,candidateGeometry.branch,selected.fit.cost+threshold2*variance))return;
            const auto alternative=solve(q,candidateGeometry.branch);
            if(!alternative||alternative->cost>selected.fit.cost+threshold2*variance)return;
            if(profile.enabled)++profile.admittedModes;
            std::array<double,3> budget{};if(!physical(q,candidateGeometry.branch,*alternative,&budget))return;
            if(std::any_of(budget.begin(),budget.end(),[](double x){return x<=0;})){unresolved=true;return;}
            if(const auto candidate=mode(q,candidateGeometry.branch,*alternative)) {
                modes.push_back(*candidate);
                if(!mirrorEnv.add(*candidate))unresolved=true;
            } else unresolved=true;
            if(profile.enabled)std::fprintf(stderr,
                "MOVING_MODE_ENVELOPE {\"read\":%.17g,\"visited\":%zu,\"admitted\":%zu,\"modes\":%zu,\"unresolved\":%d,\"finite\":%d}\n",
                measured.timeS,profile.modes,profile.admittedModes,modes.size(),unresolved,
                mirrorEnv.ok?1:0);
        };
        if(!unresolved&&!cutoffs.empty()&&!geometries.empty()) {
            // Partition pure sky/ground cutoffs (zero conic work) from horizon-band
            // cutoffs. Tile horizon-band cutoffs in chunks of <=32 so every bank
            // stays cached across all geometries while precomputing each geometry's
            // cutoff-independent Gaussian rectangle moments once per tile.
            double gMinLevel=infinity,gMaxLevel=-infinity;
            for(const auto& g:geometries) {
                if(const auto* tr=prepare(g.shape,g.branch)) {
                    gMinLevel=std::min(gMinLevel,tr->minimumLevel);
                    gMaxLevel=std::max(gMaxLevel,tr->maximumLevel);
                }
            }
            const double bandPad=16.0*std::max(cutStep,p.pixelPitchRad)/1024.0;
            std::vector<double> plainCutoffs,horizonCutoffs;
            plainCutoffs.reserve(cutoffs.size());
            horizonCutoffs.reserve(cutoffs.size());
            for(const double cutoff:cutoffs) {
                if(gMinLevel<=gMaxLevel&&cutoff>=gMinLevel-bandPad&&cutoff<=gMaxLevel+bandPad)
                    horizonCutoffs.push_back(cutoff);
                else plainCutoffs.push_back(cutoff);
            }
            std::sort(horizonCutoffs.begin(),horizonCutoffs.end());
            for(const double cutoff:plainCutoffs) {
                if(unresolved)break;
                for(const auto& candidateGeometry:geometries) {
                    if(unresolved)break;
                    visitModeCandidate(candidateGeometry,cutoff);
                }
            }
            const bool canPrecomputeRegions=regionalOperator==RegionalOperator::ConicFastV5&&certifiedArithmeticV2();
            constexpr std::size_t kCutoffTileSize=40u;
            for(std::size_t tileBegin=0;tileBegin<horizonCutoffs.size()&&!unresolved;tileBegin+=kCutoffTileSize) {
                const std::size_t tileEnd=std::min(horizonCutoffs.size(),tileBegin+kCutoffTileSize);
                double tileMin=infinity,tileMax=-infinity;
                for(std::size_t ci=tileBegin;ci<tileEnd;++ci) {
                    tileMin=std::min(tileMin,horizonCutoffs[ci]);
                    tileMax=std::max(tileMax,horizonCutoffs[ci]);
                }
                const double cutPad=2.0*(cutStep/1024.0);
                const std::shared_ptr<conic_moments_v4::Bank> prepBank=canPrecomputeRegions
                    ?getOrCreatePhaseBankV5(horizonCutoffs[tileBegin]):nullptr;
                for(const auto& candidateGeometry:geometries) {
                    if(unresolved)break;
                    std::vector<Trajectory*> geomTrajs;
                    const auto addUniqueTraj=[&](Trajectory* tr) {
                        if(!tr||tr->path.stationary)return;
                        if(std::find(geomTrajs.begin(),geomTrajs.end(),tr)==geomTrajs.end())geomTrajs.push_back(tr);
                    };
                    if(prepBank&&candidateGeometry.shape[4]>0&&candidateGeometry.shape[5]>0) {
                        geomTrajs.reserve(13u);
                        addUniqueTraj(prepare(candidateGeometry.shape,candidateGeometry.branch));
                        const Shape curSq=squared(candidateGeometry.shape);
                        for(unsigned coord=0;coord<6u;++coord) {
                            const double step=coord<4?p.pixelPitchRad/1024:pixelArea/1024;
                            auto plus=curSq,minus=curSq;plus[coord]+=step;minus[coord]-=step;
                            if(coord==4||coord==5)minus[coord]=std::max(0.,minus[coord]);
                            const auto gp=geometry(plus),gm=geometry(minus);
                            if(valid(gp))addUniqueTraj(prepare(gp,candidateGeometry.branch));
                            if(valid(gm))addUniqueTraj(prepare(gm,candidateGeometry.branch));
                        }
                        struct PrepTask {Trajectory* traj;std::size_t ti;};
                        std::vector<PrepTask> tasks;
                        const double marginP=9*p.psfSigmaRad,hxP=.5*p.width*p.pixelPitchRad,hyP=.5*p.height*p.pixelPitchRad;
                        const double fLP=std::max(-1.55,-hxP-marginP),fRP=std::min(1.55,hxP+marginP);
                        const double fBP=std::max(-1.55,-hyP-marginP),fTP=std::min(1.55,hyP+marginP);
                        // Precompute ONLY if at least 2 cutoffs in this tile actually cross
                        // this geometry; otherwise single-use cutoffs avoid eager prep overhead.
                        unsigned crossingCutsInTile=0u;
                        for(std::size_t ci=tileBegin;ci<tileEnd;++ci) {
                            const double cut=horizonCutoffs[ci];
                            if(!geomTrajs.empty()&&geomTrajs[0]->minimumLevel<cut+cutPad&&geomTrajs[0]->maximumLevel>=cut-cutPad)
                                ++crossingCutsInTile;
                        }
                        if(crossingCutsInTile>=2u) {
                            for(Trajectory* tr:geomTrajs) {
                                if(tr->minimumLevel>=tileMax+cutPad||tr->maximumLevel<tileMin-cutPad)continue;
                                if(tr->precomputedRegions.size()!=tr->x.size())tr->precomputedRegions.resize(tr->x.size());
                                const double halfW=.5*tr->shape[4],halfH=.5*tr->shape[5];
                                for(std::size_t ti=0;ti<tr->x.size();++ti) {
                                    if(tr->precomputedRegions[ti].valid)continue;
                                    if(tr->levelMin[ti]>=tileMax+cutPad)continue;
                                    if(tr->levelMax[ti]<tileMin-cutPad&&
                                       tr->x[ti]-halfW>=fLP&&tr->x[ti]+halfW<=fRP&&
                                       tr->y[ti]-halfH>=fBP&&tr->y[ti]+halfH<=fTP)continue;
                                    tasks.push_back({tr,ti});
                                }
                            }
                        }
                        if(!tasks.empty()) {
                            const unsigned maxW=ReticleNestedPoolGuardV1::maxWorkers(maximumPhaseWorkersV5);
                            const unsigned workers=std::min<unsigned>(maxW,static_cast<unsigned>(tasks.size()));
                            if(workers>=2u&&phasePoolV5.syncEnvironment()) {
                                std::atomic<std::size_t> nextTask{0u};
                                const auto runPrep=[&](unsigned) noexcept {
                                    while(true) {
                                        const std::size_t idx=nextTask.fetch_add(1u,std::memory_order_relaxed);
                                        if(idx>=tasks.size())break;
                                        auto& task=tasks[idx];
                                        const auto* tr=task.traj;
                                        const std::size_t ti=task.ti;
                                        const std::array<double,4> reg{{tr->x[ti]-.5*tr->shape[4],tr->x[ti]+.5*tr->shape[4],tr->y[ti]-.5*tr->shape[5],tr->y[ti]+.5*tr->shape[5]}};
                                        (void)prepBank->prepareRegionV5(reg,task.traj->precomputedRegions[ti]);
                                    }
                                };
                                phasePoolV5.run(workers,runPrep);
                            } else {
                                for(auto& task:tasks) {
                                    const auto* tr=task.traj;
                                    const std::size_t ti=task.ti;
                                    const std::array<double,4> reg{{tr->x[ti]-.5*tr->shape[4],tr->x[ti]+.5*tr->shape[4],tr->y[ti]-.5*tr->shape[5],tr->y[ti]+.5*tr->shape[5]}};
                                    (void)prepBank->prepareRegionV5(reg,task.traj->precomputedRegions[ti]);
                                }
                            }
                        }
                    }
                    for(std::size_t ci=tileBegin;ci<tileEnd&&!unresolved;++ci) {
                        visitModeCandidate(candidateGeometry,horizonCutoffs[ci]);
                    }
                    for(Trajectory* tr:geomTrajs) {
                        std::vector<conic_moments_v4::RegionMomentsV5>().swap(tr->precomputedRegions);
                    }
                }
            }
        }
        profile.checkpoint("modes.end",selected.fit.cost/variance,unresolved);
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
    infer(optics,result,pointing,up,output,nullptr);
}
void infer(const Optics& optics,ThermalReticleResultV1& result,V pointing,V up,ThermalReticleMovingDiagnosticV2* output,
    const ThermalReticleMeasuredPriorV2* prior) {
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
    Inverse inverse(optics,result,pointing,up,RegionalOperator::ConicFastV5,prior);inverse.run(result);if(output)*output=inverse.diagnostic;
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
