#include "ThermalImageReceiverV1.h"
#include "EngineeringSeekerV3.h"
#include "ThermalDualBandReceiverV1.h"
#include "ThermalReticleReceiverV1.h"
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
#include <exception>
#include <mutex>
#include <thread>
#if defined(__x86_64__) || defined(_M_X64) || defined(__AVX2__)
#include <immintrin.h>
#endif
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
struct BorderAnnulusEntryV1 {
    std::uint8_t count = 0;
    std::array<std::uint16_t, 32> indices{};
};

#if (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
#define FSIM_AVX2_TARGET __attribute__((target("avx2")))
#define FSIM_HAS_RUNTIME_AVX2 1
#elif defined(__AVX2__)
#define FSIM_AVX2_TARGET
#define FSIM_HAS_RUNTIME_AVX2 1
#else
#define FSIM_HAS_RUNTIME_AVX2 0
#endif

#if FSIM_HAS_RUNTIME_AVX2
FSIM_AVX2_TARGET inline void cmpSwapAvx2(__m256d& a,__m256d& b) noexcept {
    const __m256d lo=_mm256_min_pd(a,b);
    const __m256d hi=_mm256_max_pd(a,b);
    a=lo;b=hi;
}
template<std::size_t I,std::size_t End,std::size_t Step,std::size_t R>
FSIM_AVX2_TARGET inline void oddEvenPassAvx2(__m256d* v) noexcept {
    if constexpr(I+R<End) {
        cmpSwapAvx2(v[I],v[I+R]);
        oddEvenPassAvx2<I+Step,End,Step,R>(v);
    }
}
template<std::size_t Lo,std::size_t N,std::size_t R>
FSIM_AVX2_TARGET inline void oddEvenMergeAvx2(__m256d* v) noexcept {
    constexpr std::size_t M=R*2;
    if constexpr(M<N) {
        oddEvenMergeAvx2<Lo,N,M>(v);
        oddEvenMergeAvx2<Lo+R,N,M>(v);
        oddEvenPassAvx2<Lo+R,Lo+N,M,R>(v);
    } else {
        cmpSwapAvx2(v[Lo],v[Lo+R]);
    }
}
template<std::size_t Lo,std::size_t N>
FSIM_AVX2_TARGET inline void oddEvenSortAvx2(__m256d* v) noexcept {
    if constexpr(N>1) {
        constexpr std::size_t M=N/2;
        oddEvenSortAvx2<Lo,M>(v);
        oddEvenSortAvx2<Lo+M,M>(v);
        oddEvenMergeAvx2<Lo,N,1>(v);
    }
}
FSIM_AVX2_TARGET inline void bitonicStepAvx2(__m256d& maxLo,__m256d& minHi,__m256d x,__m256d y) noexcept {
    maxLo=_mm256_max_pd(maxLo,_mm256_min_pd(x,y));
    minHi=_mm256_min_pd(minHi,_mm256_max_pd(x,y));
}
FSIM_AVX2_TARGET inline void bitonicSplit16Avx2(__m256d* a,__m256d* b,bool odd,double out[4]) noexcept {
    __m256d maxLo=_mm256_min_pd(a[0],b[15]);
    __m256d minHi=_mm256_max_pd(a[0],b[15]);
    bitonicStepAvx2(maxLo,minHi,a[1],b[14]);bitonicStepAvx2(maxLo,minHi,a[2],b[13]);
    bitonicStepAvx2(maxLo,minHi,a[3],b[12]);bitonicStepAvx2(maxLo,minHi,a[4],b[11]);
    bitonicStepAvx2(maxLo,minHi,a[5],b[10]);bitonicStepAvx2(maxLo,minHi,a[6],b[9]);
    bitonicStepAvx2(maxLo,minHi,a[7],b[8]);bitonicStepAvx2(maxLo,minHi,a[8],b[7]);
    bitonicStepAvx2(maxLo,minHi,a[9],b[6]);bitonicStepAvx2(maxLo,minHi,a[10],b[5]);
    bitonicStepAvx2(maxLo,minHi,a[11],b[4]);bitonicStepAvx2(maxLo,minHi,a[12],b[3]);
    bitonicStepAvx2(maxLo,minHi,a[13],b[2]);bitonicStepAvx2(maxLo,minHi,a[14],b[1]);
    bitonicStepAvx2(maxLo,minHi,a[15],b[0]);
    const __m256d m=odd?maxLo:_mm256_mul_pd(_mm256_set1_pd(0.5),_mm256_add_pd(maxLo,minHi));
    _mm256_storeu_pd(out,m);
}
FSIM_AVX2_TARGET void annulusInteriorRowAvx2(const double* meas,std::size_t rowBase,unsigned xStart,unsigned xVecEnd,
    const std::ptrdiff_t* deltas,std::size_t n,double* outBg) noexcept {
    const __m256d posInf=_mm256_set1_pd(std::numeric_limits<double>::infinity());
    const __m256d negInf=_mm256_set1_pd(-std::numeric_limits<double>::infinity());
    if(n==32u) {
        for(unsigned x=xStart;x<xVecEnd;x+=4u) {
            const std::ptrdiff_t baseIdx=static_cast<std::ptrdiff_t>(rowBase+x);
            __m256d a[16]={
                _mm256_loadu_pd(meas+baseIdx+deltas[0]),_mm256_loadu_pd(meas+baseIdx+deltas[1]),
                _mm256_loadu_pd(meas+baseIdx+deltas[2]),_mm256_loadu_pd(meas+baseIdx+deltas[3]),
                _mm256_loadu_pd(meas+baseIdx+deltas[4]),_mm256_loadu_pd(meas+baseIdx+deltas[5]),
                _mm256_loadu_pd(meas+baseIdx+deltas[6]),_mm256_loadu_pd(meas+baseIdx+deltas[7]),
                _mm256_loadu_pd(meas+baseIdx+deltas[8]),_mm256_loadu_pd(meas+baseIdx+deltas[9]),
                _mm256_loadu_pd(meas+baseIdx+deltas[10]),_mm256_loadu_pd(meas+baseIdx+deltas[11]),
                _mm256_loadu_pd(meas+baseIdx+deltas[12]),_mm256_loadu_pd(meas+baseIdx+deltas[13]),
                _mm256_loadu_pd(meas+baseIdx+deltas[14]),_mm256_loadu_pd(meas+baseIdx+deltas[15])
            };
            oddEvenSortAvx2<0,16>(a);
            __m256d b[16]={
                _mm256_loadu_pd(meas+baseIdx+deltas[16]),_mm256_loadu_pd(meas+baseIdx+deltas[17]),
                _mm256_loadu_pd(meas+baseIdx+deltas[18]),_mm256_loadu_pd(meas+baseIdx+deltas[19]),
                _mm256_loadu_pd(meas+baseIdx+deltas[20]),_mm256_loadu_pd(meas+baseIdx+deltas[21]),
                _mm256_loadu_pd(meas+baseIdx+deltas[22]),_mm256_loadu_pd(meas+baseIdx+deltas[23]),
                _mm256_loadu_pd(meas+baseIdx+deltas[24]),_mm256_loadu_pd(meas+baseIdx+deltas[25]),
                _mm256_loadu_pd(meas+baseIdx+deltas[26]),_mm256_loadu_pd(meas+baseIdx+deltas[27]),
                _mm256_loadu_pd(meas+baseIdx+deltas[28]),_mm256_loadu_pd(meas+baseIdx+deltas[29]),
                _mm256_loadu_pd(meas+baseIdx+deltas[30]),_mm256_loadu_pd(meas+baseIdx+deltas[31])
            };
            oddEvenSortAvx2<0,16>(b);
            bitonicSplit16Avx2(a,b,false,outBg+x);
        }
        return;
    }
    const std::size_t mid=n/2;
    if(n<=16u) {
        for(unsigned x=xStart;x<xVecEnd;x+=4u) {
            const std::ptrdiff_t baseIdx=static_cast<std::ptrdiff_t>(rowBase+x);
            __m256d v[16];
            #pragma GCC unroll 16
            for(std::size_t k=0;k<16;++k)v[k]=(k<n)?_mm256_loadu_pd(meas+baseIdx+deltas[k]):posInf;
            oddEvenSortAvx2<0,16>(v);
            const __m256d m=(n&1u)?v[mid]:_mm256_mul_pd(_mm256_set1_pd(0.5),_mm256_add_pd(v[mid],v[mid-1]));
            _mm256_storeu_pd(outBg+x,m);
        }
        return;
    }
    const std::size_t padNeg=(n&1u)?(15u-mid):(16u-mid);
    const std::size_t firstData=16u-padNeg;
    const std::size_t secondData=n-firstData;
    for(unsigned x=xStart;x<xVecEnd;x+=4u) {
        const std::ptrdiff_t baseIdx=static_cast<std::ptrdiff_t>(rowBase+x);
        __m256d a[16];
        #pragma GCC unroll 16
        for(std::size_t k=0;k<16;++k)a[k]=(k<firstData)?_mm256_loadu_pd(meas+baseIdx+deltas[k]):negInf;
        oddEvenSortAvx2<0,16>(a);
        __m256d b[16];
        #pragma GCC unroll 16
        for(std::size_t k=0;k<16;++k)b[k]=(k<secondData)?_mm256_loadu_pd(meas+baseIdx+deltas[firstData+k]):posInf;
        oddEvenSortAvx2<0,16>(b);
        bitonicSplit16Avx2(a,b,(n&1u)!=0u,outBg+x);
    }
}
FSIM_AVX2_TARGET inline __m256d loadBorder4Avx2(const double* meas,const BorderAnnulusEntryV1* b0,
    const BorderAnnulusEntryV1* b1,const BorderAnnulusEntryV1* b2,const BorderAnnulusEntryV1* b3,std::size_t k) noexcept {
    return _mm256_set_pd(meas[b3->indices[k]],meas[b2->indices[k]],meas[b1->indices[k]],meas[b0->indices[k]]);
}
FSIM_AVX2_TARGET void annulusBorder4Avx2(const double* meas,const BorderAnnulusEntryV1* b0,
    const BorderAnnulusEntryV1* b1,const BorderAnnulusEntryV1* b2,const BorderAnnulusEntryV1* b3,
    std::size_t n,double out[4]) noexcept {
    const __m256d posInf=_mm256_set1_pd(std::numeric_limits<double>::infinity());
    const std::size_t mid=n/2;
    if(n<=16u) {
        __m256d v[16];
        #pragma GCC unroll 16
        for(std::size_t k=0;k<16;++k)v[k]=(k<n)?loadBorder4Avx2(meas,b0,b1,b2,b3,k):posInf;
        oddEvenSortAvx2<0,16>(v);
        const __m256d m=(n&1u)?v[mid]:_mm256_mul_pd(_mm256_set1_pd(0.5),_mm256_add_pd(v[mid],v[mid-1]));
        _mm256_storeu_pd(out,m);
        return;
    }
    const __m256d negInf=_mm256_set1_pd(-std::numeric_limits<double>::infinity());
    const std::size_t padNeg=(n&1u)?(15u-mid):(16u-mid);
    const std::size_t firstData=16u-padNeg;
    const std::size_t secondData=n-firstData;
    __m256d a[16];
    #pragma GCC unroll 16
    for(std::size_t k=0;k<16;++k)a[k]=(k<firstData)?loadBorder4Avx2(meas,b0,b1,b2,b3,k):negInf;
    oddEvenSortAvx2<0,16>(a);
    __m256d b[16];
    #pragma GCC unroll 16
    for(std::size_t k=0;k<16;++k)b[k]=(k<secondData)?loadBorder4Avx2(meas,b0,b1,b2,b3,firstData+k):posInf;
    oddEvenSortAvx2<0,16>(b);
    bitonicSplit16Avx2(a,b,(n&1u)!=0u,out);
}
bool runtimeHasAvx2V1() noexcept {
#if defined(__AVX2__)
    return true;
#elif (defined(__GNUC__) || defined(__clang__)) && defined(__x86_64__)
    static const bool supported = __builtin_cpu_supports("avx2");
    return supported;
#else
    return false;
#endif
}
#endif

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
double gaussianPrimitiveBitExact(double z) {
    return z*.5*std::erfc(-z/std::sqrt(2.))+std::exp(-.5*z*z)/std::sqrt(2*pi);
}
double gaussianPrimitiveBitExactNearest(double z,bool nearest) noexcept {
    if(nearest) {
        // Under IEEE-754 binary64 FE_TONEAREST: for z >= 9.5, erfc(-z/sqrt(2)) == 2.0
        // and exp(-0.5*z^2)/sqrt(2*pi) <= 1.0e-20 << 0.5*ulp(z), so result == z bit-exact.
        // For z <= -38.5, both erfc(-z/sqrt(2)) and exp(-0.5*z^2) underflow to +0.0.
        if(z>=9.5)return z;
        if(z<=-38.5)return 0.;
    }
    return gaussianPrimitiveBitExact(z);
}
double gaussianRectangle(double low,double high,double pixelLow,double pixelHigh,double sigma) {
    const auto integratedCdf=[&](double edge) {
        return sigma*(gaussianPrimitiveBitExact((edge-low)/sigma)-gaussianPrimitiveBitExact((edge-high)/sigma));
    };
    return std::clamp(integratedCdf(pixelHigh)-integratedCdf(pixelLow),0.,std::min(high-low,pixelHigh-pixelLow));
}
// One background-column exact memo: all rows in the same column share the
// exact same horizontal support [left, right] and pixel x-interval [px0, px1],
// so x-probability, tan(x), quadratic horizon roots, open-interval sky/ground
// classifications, and shared row-edge Gaussian primitives are bit-identical.
struct BackgroundColumnMemoV2 {
    struct Entry {
        std::uint64_t keyBits=0;
        double probability=0,tx=0;
        std::array<double,2> roots{};
        std::array<double,3> intervalRad{};
        std::size_t primOffset=static_cast<std::size_t>(-1);
        unsigned rootCount=0;
        bool occupied=false;
        bool rootsReady=false;
        bool safeIntervals=false;
    };
    bool enabled=false;
    unsigned activeYLo=0,activeEdgeCount=0;
    std::array<std::uint8_t,129> edgeShared{};
    std::vector<Entry> table;
    std::vector<double> primPool;
    std::vector<std::uint8_t> primReady;
    std::size_t count=0;
    BackgroundColumnMemoV2(unsigned yLo=0,unsigned yHi=0,const std::array<std::uint8_t,129>* shared=nullptr) {
        enabled=std::numeric_limits<double>::is_iec559&&FLT_EVAL_METHOD==0&&std::fegetround()==FE_TONEAREST;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        enabled=enabled&&(_mm_getcsr()&0x6000u)==0;
#endif
        if(enabled) {
            table.resize(1024);
            if(shared)edgeShared=*shared;
            if(yHi>=yLo) {
                activeYLo=yLo;
                activeEdgeCount=yHi-yLo+2u;
                primPool.reserve(static_cast<std::size_t>(activeEdgeCount)*512u);
                primReady.reserve(static_cast<std::size_t>(activeEdgeCount)*512u);
            }
        }
    }
    static std::uint64_t key(double x) noexcept {
        std::uint64_t value;std::memcpy(&value,&x,sizeof(value));return value;
    }
    static std::size_t hashKey(std::uint64_t k) noexcept {
        k^=k>>33;k*=0xff51afd7ed558ccdull;k^=k>>33;k*=0xc4ceb9fe1a85ec53ull;k^=k>>33;
        return static_cast<std::size_t>(k);
    }
    void grow() {
        std::vector<Entry> next(table.size()*2);
        const std::size_t mask=next.size()-1;
        for(const auto& e:table) {
            if(!e.occupied)continue;
            std::size_t slot=hashKey(e.keyBits)&mask;
            while(next[slot].occupied)slot=(slot+1)&mask;
            next[slot]=e;
        }
        table=std::move(next);
    }
    std::pair<Entry*,bool> findOrInsert(std::uint64_t k) {
        if((count+1)*4>=table.size()*3)grow();
        const std::size_t mask=table.size()-1;
        std::size_t slot=hashKey(k)&mask;
        for(;;) {
            auto& e=table[slot];
            if(!e.occupied) {
                e.occupied=true;e.keyBits=k;e.rootsReady=false;e.safeIntervals=false;
                e.primOffset=static_cast<std::size_t>(-1);++count;
                return {&e,true};
            }
            if(e.keyBits==k)return {&e,false};
            slot=(slot+1)&mask;
        }
    }
    double cachedPrimitive(Entry& entry,unsigned rootIdx,unsigned edgeIdx,double z,bool nearest) {
        if(activeEdgeCount==0||edgeIdx>=activeEdgeCount||rootIdx>=entry.rootCount)
            return gaussianPrimitiveBitExactNearest(z,nearest);
        if(entry.primOffset==static_cast<std::size_t>(-1)) {
            entry.primOffset=primPool.size();
            const std::size_t needed=static_cast<std::size_t>(entry.rootCount)*activeEdgeCount;
            primPool.resize(entry.primOffset+needed,0.);
            primReady.resize(entry.primOffset+needed,0u);
        }
        const std::size_t pos=entry.primOffset+static_cast<std::size_t>(rootIdx)*activeEdgeCount+edgeIdx;
        if(!primReady[pos]) {
            primPool[pos]=gaussianPrimitiveBitExactNearest(z,nearest);
            primReady[pos]=1u;
        }
        return primPool[pos];
    }
};
// Integrate the sky/ground step in continuous source space as well. At fixed
// x the exact native conical horizon has at most two roots in tan(y). The y
// integral is analytic; adaptive Simpson integrates its smooth x dependence.
// Uniform-sky/ground rectangles use an exact interval bound and no quadrature.
double backgroundIntegral(const ThermalSceneFrameV1& scene,const Plane& plane,Rectangle r,
    double px0,double px1,double py0,double py1,double sigma,
    HorizonQuadratureMemoV2* memo=nullptr,unsigned memoColumn=0,unsigned memoRow=0,
    BackgroundColumnMemoV2* colMemo=nullptr) {
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
    const bool skipZeroProb=nearest&&std::isfinite(scene.skyRadianceWm2Sr)&&std::isfinite(scene.groundRadianceWm2Sr)&&
        scene.skyRadianceWm2Sr>=0&&scene.groundRadianceWm2Sr>=0;
    const double invRoot2Sigma=1./(std::sqrt(2.)*sigma);
    const auto sortEdges=[](std::array<double,4>& edges,unsigned size) noexcept {
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
    };
    // Pixel-constant endpoints for bit-exact Gaussian y-interval evaluation:
    // r.bottom and r.top are fixed across all adaptive x samples of this pixel,
    // and interior roots share primitives between adjacent sub-intervals.
    const double pHiBot=gaussianPrimitiveBitExact((py1-r.bottom)/sigma);
    const double pLoBot=gaussianPrimitiveBitExact((py0-r.bottom)/sigma);
    const double pHiTop=gaussianPrimitiveBitExact((py1-r.top)/sigma);
    const double pLoTop=gaussianPrimitiveBitExact((py0-r.top)/sigma);
    const double pixelDy=py1-py0;
    bool hasLastY=false;
    unsigned lastEdgeCount=0;
    std::array<double,4> lastEdges{};
    std::array<double,3> lastRadiances{};
    double lastYIntegral=0.;
    const auto function=[&](double x) {
        if(colMemo&&colMemo->enabled&&!memo) {
            auto [entryPtr,inserted]=colMemo->findOrInsert(BackgroundColumnMemoV2::key(x));
            auto& entry=*entryPtr;
            if(inserted) {
                const double denom=std::sqrt(2.)*sigma;
                const double uHi=(px1-x)/denom,uLo=(px0-x)/denom;
                if(skipZeroProb&&(uHi<=-6.0||uLo>=6.0)) {
                    entry.probability=0.;
                } else {
                    entry.probability=.5*(std::erf(uHi)-std::erf(uLo));
                }
            }
            if(skipZeroProb&&entry.probability==0)return entry.probability;
            const double h=scene.groundHorizonCosine;
            if(!entry.rootsReady) {
                const double tx=std::tan(x),n=a+b*tx;
                entry.tx=tx;entry.rootCount=0;entry.safeIntervals=false;
                const double aa=c*c-h*h,bb=2*n*c,cc=n*n-h*h*(1+tx*tx);
                double disc=-1.;
                bool linearRoot=false;
                if(std::abs(aa)<1e-14) {
                    linearRoot=true;
                    if(std::abs(bb)>1e-14)entry.roots[entry.rootCount++]=std::atan(-cc/bb);
                } else {
                    disc=bb*bb-4*aa*cc;
                    if(disc>=0) {
                        const double sq=std::sqrt(disc);
                        entry.roots[entry.rootCount++]=std::atan((-bb-sq)/(2*aa));
                        entry.roots[entry.rootCount++]=std::atan((-bb+sq)/(2*aa));
                        if(entry.roots[1]<entry.roots[0])std::swap(entry.roots[0],entry.roots[1]);
                    }
                }
                const auto classifyY=[&](double yTest) noexcept {
                    const auto ray=unit(add(plane.forward,add(scale(plane.right,tx),scale(plane.up,std::tan(yTest)))));
                    return dot(ray,scene.up)>=h?scene.skyRadianceWm2Sr:scene.groundRadianceWm2Sr;
                };
                if(entry.rootCount==0) {
                    if(!linearRoot&&disc<-1e-10) {
                        entry.intervalRad[0]=classifyY(0.);
                        entry.safeIntervals=true;
                    }
                } else if(entry.rootCount==1) {
                    const double r0=entry.roots[0];
                    if(std::abs(bb)>1e-10&&r0>-1.55+1e-5&&r0<1.55-1e-5) {
                        entry.intervalRad[0]=classifyY(.5*(-1.55+r0));
                        entry.intervalRad[1]=classifyY(.5*(r0+1.55));
                        entry.safeIntervals=true;
                    }
                } else if(entry.rootCount==2) {
                    const double r0=entry.roots[0],r1=entry.roots[1];
                    if(disc>1e-10&&(r1-r0)>1e-5&&r0>-1.55+1e-5&&r1<1.55-1e-5) {
                        entry.intervalRad[0]=classifyY(.5*(-1.55+r0));
                        entry.intervalRad[1]=classifyY(.5*(r0+r1));
                        entry.intervalRad[2]=classifyY(.5*(r1+1.55));
                        entry.safeIntervals=true;
                    }
                }
                entry.rootsReady=true;
            }
            std::array<double,4> edges{r.bottom,r.top,0.,0.};
            std::array<unsigned,4> edgeRootIdx{2u,2u,2u,2u};
            unsigned size=2;
            for(unsigned k=0;k<entry.rootCount;++k) {
                if(entry.roots[k]>r.bottom&&entry.roots[k]<r.top) {
                    edges[size]=entry.roots[k];
                    edgeRootIdx[size]=k;
                    ++size;
                }
            }
            if(size==3) {
                // edges[0]=r.bottom < edges[2]=root < edges[1]=r.top
                std::swap(edges[1],edges[2]);
                std::swap(edgeRootIdx[1],edgeRootIdx[2]);
            } else if(size==4) {
                // entry.roots[0] <= entry.roots[1], both strictly in (r.bottom, r.top)
                const double topVal=edges[1];
                edges[1]=edges[2];edgeRootIdx[1]=edgeRootIdx[2];
                edges[2]=edges[3];edgeRootIdx[2]=edgeRootIdx[3];
                edges[3]=topVal;edgeRootIdx[3]=2u;
            }
            std::array<double,3> radiances{};
            for(unsigned i=1;i<size;++i) {
                if(skipZeroProb&&edges[i]==edges[i-1]){radiances[i-1]=0.;continue;}
                const double y=.5*(edges[i-1]+edges[i]);
                bool classified=false;
                if(entry.safeIntervals&&(edges[i]-edges[i-1])>1e-6) {
                    unsigned region=0;
                    bool farFromRoots=true;
                    for(unsigned k=0;k<entry.rootCount;++k) {
                        if(std::abs(y-entry.roots[k])<=1e-6){farFromRoots=false;break;}
                        if(y>entry.roots[k])++region;
                    }
                    if(farFromRoots) {
                        radiances[i-1]=entry.intervalRad[region];
                        classified=true;
                    }
                }
                if(!classified) {
                    const auto ray=unit(add(plane.forward,add(scale(plane.right,entry.tx),scale(plane.up,std::tan(y)))));
                    radiances[i-1]=dot(ray,scene.up)>=h?scene.skyRadianceWm2Sr:scene.groundRadianceWm2Sr;
                }
            }
            if(hasLastY&&size==lastEdgeCount) {
                bool same=true;
                for(unsigned i=0;i<size;++i)if(edges[i]!=lastEdges[i]){same=false;break;}
                if(same)for(unsigned i=0;i+1<size;++i)if(radiances[i]!=lastRadiances[i]){same=false;break;}
                if(same)return entry.probability*lastYIntegral;
            }
            std::array<double,4> pHi{},pLo{};
            pHi[0]=pHiBot;pLo[0]=pLoBot;
            pHi[size-1]=pHiTop;pLo[size-1]=pLoTop;
            const bool canCacheEdge=colMemo->activeEdgeCount>0&&memoRow>=colMemo->activeYLo&&
                (memoRow-colMemo->activeYLo+1u)<colMemo->activeEdgeCount;
            const unsigned loIdx=canCacheEdge?(memoRow-colMemo->activeYLo):0u;
            const bool shareLo=canCacheEdge&&memoRow<129u&&colMemo->edgeShared[memoRow]!=0u;
            const bool shareHi=canCacheEdge&&(memoRow+1u)<129u&&colMemo->edgeShared[memoRow+1u]!=0u;
            for(unsigned i=1;i+1<size;++i) {
                if(edges[i]==edges[i-1]){pHi[i]=pHi[i-1];pLo[i]=pLo[i-1];}
                else {
                    const double zHi=(py1-edges[i])/sigma,zLo=(py0-edges[i])/sigma;
                    pHi[i]=(shareHi&&edgeRootIdx[i]<entry.rootCount)
                        ?colMemo->cachedPrimitive(entry,edgeRootIdx[i],loIdx+1u,zHi,nearest)
                        :gaussianPrimitiveBitExactNearest(zHi,nearest);
                    pLo[i]=(shareLo&&edgeRootIdx[i]<entry.rootCount)
                        ?colMemo->cachedPrimitive(entry,edgeRootIdx[i],loIdx,zLo,nearest)
                        :gaussianPrimitiveBitExactNearest(zLo,nearest);
                }
            }
            double yIntegral=0;
            for(unsigned i=1;i<size;++i) {
                if(skipZeroProb&&edges[i]==edges[i-1])continue;
                const double cdfHi=sigma*(pHi[i-1]-pHi[i]);
                const double cdfLo=sigma*(pLo[i-1]-pLo[i]);
                const double rect=std::clamp(cdfHi-cdfLo,0.,std::min(edges[i]-edges[i-1],pixelDy));
                yIntegral+=radiances[i-1]*rect;
            }
            hasLastY=true;lastEdgeCount=size;lastEdges=edges;lastRadiances=radiances;lastYIntegral=yIntegral;
            return entry.probability*yIntegral;
        }
        auto* cached=memo?memo->find(x,memoColumn,memoRow):nullptr;
        auto* counted=memo?memo->diagnostic():nullptr;
        const auto probabilityAt=[&]() {
            if(cached&&cached->probabilityKnown[memoColumn]){if(counted)++counted->probabilityHits;return cached->probabilities[memoColumn];}
            if(counted)++counted->probabilityEvaluations;
            const double value=.5*(std::erf((px1-x)/(std::sqrt(2.)*sigma))-std::erf((px0-x)/(std::sqrt(2.)*sigma)));
            if(cached){cached->probabilityKnown.set(memoColumn);cached->probabilities[memoColumn]=value;}
            return value;
        };
        (void)invRoot2Sigma;
        const double knownProbability=skipZeroProb?probabilityAt():0.;
        if(skipZeroProb&&knownProbability==0)return knownProbability;
        double yIntegral=0;
        if(cached&&cached->yKnown[memoRow]){if(counted)++counted->yHits;yIntegral=cached->yValues[memoRow];}
        else {
            if(counted)++counted->yEvaluations;
            // The root/sort and midpoint classifications are retained
            // exactly, including duplicate roots and zero-length intervals.
            // They depend on this x/scene/region, never the detector pixel.
            HorizonQuadratureMemoV2::Pattern local;
            auto* pattern=cached?static_cast<HorizonQuadratureMemoV2::Pattern*>(cached):&local;
            if(!pattern->rootsReady) {
                if(counted)++counted->rootsBuilt;
                const double tx=std::tan(x),n=a+b*tx,h=scene.groundHorizonCosine;
                std::array<double,4> edges{r.bottom,r.top,0.,0.};unsigned edgeCount=2;
                const auto root=[&](double t){const double y=std::atan(t);if(y>r.bottom&&y<r.top)edges[edgeCount++]=y;};
                const double aa=c*c-h*h,bb=2*n*c,cc=n*n-h*h*(1+tx*tx);
                if(std::abs(aa)<1e-14) {if(std::abs(bb)>1e-14)root(-cc/bb);}
                else {
                    const double discriminant=bb*bb-4*aa*cc;
                    if(discriminant>=0) {root((-bb-std::sqrt(discriminant))/(2*aa));root((-bb+std::sqrt(discriminant))/(2*aa));}
                }
                sortEdges(edges,edgeCount);pattern->size=edgeCount;
                pattern->edges=edges;
                for(unsigned i=1;i<pattern->size;++i) {
                    const double y=.5*(pattern->edges[i-1]+pattern->edges[i]);
                    const auto ray=unit(add(plane.forward,add(scale(plane.right,tx),scale(plane.up,std::tan(y)))));
                    pattern->lights[i-1]=dot(ray,scene.up)>=h?scene.skyRadianceWm2Sr:scene.groundRadianceWm2Sr;
                }
                pattern->rootsReady=true;
            }
            // Gaussian y intervals depend on the row, not the column. Keep
            // every original multiplication/addition in its original order.
            for(unsigned i=1;i<pattern->size;++i)
                yIntegral+=pattern->lights[i-1]*gaussianRectangle(pattern->edges[i-1],pattern->edges[i],py0,py1,sigma);
            if(cached){cached->yKnown.set(memoRow);cached->yValues[memoRow]=yIntegral;}
        }
        const double probability=skipZeroProb?knownProbability:probabilityAt();
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
    } else {
        const double aPlane=dot(plane.forward,scene.up),bPlane=dot(plane.right,scene.up),cPlane=dot(plane.up,scene.up);
        const auto minSq=[](double lo,double hi) noexcept {return lo<=0&&hi>=0?0.:std::min(lo*lo,hi*hi);};
        std::array<double,128> rowY0{},rowSy0{},rowSy1{},rowMinCy{},rowMaxCy{},rowMinTy2{},rowMaxTy2{},rowIy{};
        std::array<std::uint8_t,129> edgeShared{};
        for(unsigned y=0;y<c.height;++y) {
            const double y0=(y-halfHeight)*pitch;
            rowY0[y]=y0;
            rowSy0[y]=std::max(-1.55,y0-margin);rowSy1[y]=std::min(1.55,y0+pitch+margin);
            rowIy[y]=gaussianRectangle(rowSy0[y],rowSy1[y],y0,y0+pitch,c.psfSigmaRad);
            const double ty0=std::tan(rowSy0[y]),ty1=std::tan(rowSy1[y]);
            rowMinCy[y]=std::min(cPlane*ty0,cPlane*ty1);rowMaxCy[y]=std::max(cPlane*ty0,cPlane*ty1);
            rowMinTy2[y]=minSq(ty0,ty1);rowMaxTy2[y]=std::max(ty0*ty0,ty1*ty1);
        }
        for(unsigned y=1;y<c.height;++y) {
            // Exact binary64 equality check so cached edge y between row y-1's py1
            // and row y's py0 produces bit-identical (edge - root)/sigma arguments.
            if(rowY0[y]==rowY0[y-1]+pitch)edgeShared[y]=1u;
        }
        const double invPitch2=1./(pitch*pitch);
        const bool uniformRad=(scene.skyRadianceWm2Sr==scene.groundRadianceWm2Sr);
        for(unsigned x=0;x<c.width;++x) {
            const double x0=(x-halfWidth)*pitch;
            const double sx0=std::max(-1.55,x0-margin),sx1=std::min(1.55,x0+pitch+margin);
            const double ix=gaussianRectangle(sx0,sx1,x0,x0+pitch,c.psfSigmaRad);
            if(ix==0||uniformRad) {
                for(unsigned y=0;y<c.height;++y)
                    radiance[static_cast<std::size_t>(y)*c.width+x]=(ix==0||rowIy[y]==0)?0.:(scene.skyRadianceWm2Sr*ix*rowIy[y])/(pitch*pitch);
                continue;
            }
            const double tx0=std::tan(sx0),tx1=std::tan(sx1);
            const double minBx=std::min(bPlane*tx0,bPlane*tx1),maxBx=std::max(bPlane*tx0,bPlane*tx1);
            const double minTx2=minSq(tx0,tx1),maxTx2=std::max(tx0*tx0,tx1*tx1);
            unsigned activeLo=c.height,activeHi=0;
            std::array<std::uint8_t,128> rowState{}; // 0=active, 1=sky, 2=ground, 3=zero
            for(unsigned y=0;y<c.height;++y) {
                if(rowIy[y]==0){rowState[y]=3;continue;}
                const double n0=aPlane+minBx+rowMinCy[y],n1=aPlane+maxBx+rowMaxCy[y];
                const double d0=std::sqrt(1+minTx2+rowMinTy2[y]),d1=std::sqrt(1+maxTx2+rowMaxTy2[y]);
                const double minimum=std::min({n0/d0,n0/d1,n1/d0,n1/d1}),maximum=std::max({n0/d0,n0/d1,n1/d0,n1/d1});
                if(minimum>=scene.groundHorizonCosine){rowState[y]=1;continue;}
                if(maximum<scene.groundHorizonCosine){rowState[y]=2;continue;}
                if(y<activeLo)activeLo=y;
                activeHi=y;
            }
            (void)invPitch2;
            BackgroundColumnMemoV2 colMemo(activeLo<=activeHi?activeLo:0u,activeLo<=activeHi?activeHi:0u,&edgeShared);
            for(unsigned y=0;y<c.height;++y) {
                if(rowState[y]==3){radiance[static_cast<std::size_t>(y)*c.width+x]=0.;continue;}
                if(rowState[y]==1){radiance[static_cast<std::size_t>(y)*c.width+x]=(scene.skyRadianceWm2Sr*ix*rowIy[y])/(pitch*pitch);continue;}
                if(rowState[y]==2){radiance[static_cast<std::size_t>(y)*c.width+x]=(scene.groundRadianceWm2Sr*ix*rowIy[y])/(pitch*pitch);continue;}
                const double y0=(y-halfHeight)*pitch;
                const Rectangle support{sx0,sx1,rowSy0[y],rowSy1[y]};
                radiance[static_cast<std::size_t>(y)*c.width+x]=
                    backgroundIntegral(scene,plane,support,x0,x0+pitch,y0,y0+pitch,c.psfSigmaRad,nullptr,x,y,&colMemo)/(pitch*pitch);
            }
        }
    }
    auto backgroundRadiance=radiance;
    const bool stationaryPatches=c.exposureSamples>1&&!scene.patches.empty()&&
        std::all_of(scene.patches.begin(),scene.patches.end(),[](const auto& patch){
            return patch.directionRate.x==0.&&patch.directionRate.y==0.&&patch.directionRate.z==0.;
        });
    const unsigned sampleCount=stationaryPatches?1u:c.exposureSamples;
    for(unsigned sample=0;sample<sampleCount;++sample) {
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
        std::vector<std::uint8_t> patchTouched(scene.patches.size(),0u);
        for(const auto& run:visible) {
            const auto& part=run.region;const auto index=run.source;const auto& patch=scene.patches[index];
            patchTouched[index]=1u;
            const double lightAtSensor=patch.transmission*patch.radianceWm2Sr+(1-patch.transmission)*patch.pathRadianceWm2Sr;
            const int firstX=std::max(0,static_cast<int>(std::floor((part.left-margin)/pitch+halfWidth)));
            const int lastX=std::min(static_cast<int>(c.width)-1,static_cast<int>(std::floor((part.right+margin)/pitch+halfWidth)));
            const int firstY=std::max(0,static_cast<int>(std::floor((part.bottom-margin)/pitch+halfHeight)));
            const int lastY=std::min(static_cast<int>(c.height)-1,static_cast<int>(std::floor((part.top+margin)/pitch+halfHeight)));
            if(firstX>lastX||firstY>lastY)continue;
            // Hoist uniform-sky/ground check once per visible rectangle (bit-exact
            // match to backgroundIntegral's interval check, eliminating O(W*H)
            // redundant gaussianRectangle + tan + sqrt evaluations per patch).
            bool uniformBg=false;
            double uniformBgRad=0.;
            if(scene.skyRadianceWm2Sr==scene.groundRadianceWm2Sr) {
                uniformBg=true;uniformBgRad=scene.skyRadianceWm2Sr;
            } else {
                const double aP=dot(plane.forward,scene.up),bP=dot(plane.right,scene.up),cP=dot(plane.up,scene.up);
                const double tx0=std::tan(part.left),tx1=std::tan(part.right),ty0=std::tan(part.bottom),ty1=std::tan(part.top);
                const double n0=aP+std::min(bP*tx0,bP*tx1)+std::min(cP*ty0,cP*ty1);
                const double n1=aP+std::max(bP*tx0,bP*tx1)+std::max(cP*ty0,cP*ty1);
                const auto minSq=[](double lo,double hi) noexcept {return lo<=0&&hi>=0?0.:std::min(lo*lo,hi*hi);};
                const double d0=std::sqrt(1+minSq(tx0,tx1)+minSq(ty0,ty1));
                const double d1=std::sqrt(1+std::max(tx0*tx0,tx1*tx1)+std::max(ty0*ty0,ty1*ty1));
                const double minimum=std::min({n0/d0,n0/d1,n1/d0,n1/d1}),maximum=std::max({n0/d0,n0/d1,n1/d0,n1/d1});
                if(minimum>=scene.groundHorizonCosine){uniformBg=true;uniformBgRad=scene.skyRadianceWm2Sr;}
                else if(maximum<scene.groundHorizonCosine){uniformBg=true;uniformBgRad=scene.groundRadianceWm2Sr;}
            }
            std::array<double,128> wx{},wy{};
            for(int x=firstX;x<=lastX;++x)wx[static_cast<unsigned>(x)]=gaussianRectangle(part.left,part.right,(x-halfWidth)*pitch,(x+1-halfWidth)*pitch,c.psfSigmaRad);
            for(int y=firstY;y<=lastY;++y)wy[static_cast<unsigned>(y)]=gaussianRectangle(part.bottom,part.top,(y-halfHeight)*pitch,(y+1-halfHeight)*pitch,c.psfSigmaRad);
            const double denom=pitch*pitch*c.exposureSamples;
            double* scRow=sourceContrast[index].data();
            if(uniformBg) {
                std::array<double,128> wxBg{},wyBg{};
                for(int x=firstX;x<=lastX;++x) {
                    const double x0=(x-halfWidth)*pitch,x1a=(x+1-halfWidth)*pitch,x1b=x0+pitch;
                    wxBg[static_cast<unsigned>(x)]=(x1b==x1a)?wx[static_cast<unsigned>(x)]:gaussianRectangle(part.left,part.right,x0,x1b,c.psfSigmaRad);
                }
                for(int y=firstY;y<=lastY;++y) {
                    const double y0=(y-halfHeight)*pitch,y1a=(y+1-halfHeight)*pitch,y1b=y0+pitch;
                    wyBg[static_cast<unsigned>(y)]=(y1b==y1a)?wy[static_cast<unsigned>(y)]:gaussianRectangle(part.bottom,part.top,y0,y1b,c.psfSigmaRad);
                }
                for(int y=firstY;y<=lastY;++y) {
                    const double wyv=wy[static_cast<unsigned>(y)],wyBgV=wyBg[static_cast<unsigned>(y)];
                    const std::size_t rowOff=static_cast<std::size_t>(y)*c.width;
                    for(int x=firstX;x<=lastX;++x) {
                        const double wxv=wx[static_cast<unsigned>(x)],wxBgV=wxBg[static_cast<unsigned>(x)];
                        const double bgVal=(wxBgV==0.||wyBgV==0.)?0.:(uniformBgRad*wxBgV*wyBgV);
                        const double contrast=(lightAtSensor*wxv*wyv-bgVal)/denom;
                        auto& dst=scRow[rowOff+static_cast<unsigned>(x)];
                        if(stationaryPatches)for(unsigned rep=0;rep<c.exposureSamples;++rep)dst+=contrast;
                        else dst+=contrast;
                    }
                }
            } else {
                HorizonQuadratureMemoV2 integralMemo(c.width,c.height,part,work);
                for(int y=firstY;y<=lastY;++y)for(int x=firstX;x<=lastX;++x) {
                    const double x0=(x-halfWidth)*pitch,y0=(y-halfHeight)*pitch;
                    const double contrast=(lightAtSensor*wx[static_cast<unsigned>(x)]*wy[static_cast<unsigned>(y)]-
                        backgroundIntegral(scene,plane,part,x0,x0+pitch,y0,y0+pitch,c.psfSigmaRad,&integralMemo,static_cast<unsigned>(x),static_cast<unsigned>(y)))/denom;
                    auto& dst=scRow[static_cast<std::size_t>(y)*c.width+static_cast<unsigned>(x)];
                    if(stationaryPatches)for(unsigned rep=0;rep<c.exposureSamples;++rep)dst+=contrast;
                    else dst+=contrast;
                }
            }
        }
        if(sample==0&&sampleCount==1u) {
            bool nearestAdd=std::fegetround()==FE_TONEAREST;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
            nearestAdd=nearestAdd&&(_mm_getcsr()&0x6000u)==0;
#endif
            if(nearestAdd&&!std::signbit(scene.skyRadianceWm2Sr)&&!std::signbit(scene.groundRadianceWm2Sr)) {
                for(const auto index:order) {
                    if(!patchTouched[index])continue;
                    const double* src=sourceContrast[index].data();
                    for(std::size_t i=0;i<pixels;++i)radiance[i]+=src[i];
                }
                return ThermalIncidentFieldV1{std::move(radiance),std::move(sourceContrast),std::move(order),std::move(backgroundRadiance)};
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
    return exposeImplV1(scene,pointing,seed,imaging,nullptr);
}
ThermalImageResultV1 ThermalImageReceiverV1::exposeFastBackgroundV1(const ThermalSceneFrameV1& scene,V pointing,
    std::uint64_t seed,bool imaging) {
    const auto& c=config_;
    if(!validConfig(c)||!validScene(scene,c)||scene.timeS<lastTimeS_||!finite(pointing)||!positive(norm(pointing)))return {};
    if(scene.timeS==lastTimeS_)return cached_;
    std::optional<std::vector<double>> fastBg;
    if(scene.skyRadianceWm2Sr!=scene.groundRadianceWm2Sr) {
        fastBg=ThermalReticleReceiverV1::fastBackgroundRadianceV1(c,pointing,scene.up,
            scene.skyRadianceWm2Sr,scene.groundRadianceWm2Sr,scene.groundHorizonCosine);
    }
    return exposeImplV1(scene,pointing,seed,imaging,fastBg?&*fastBg:nullptr);
}
ThermalImageResultV1 ThermalImageReceiverV1::exposeImplV1(const ThermalSceneFrameV1& scene,V pointing,
    std::uint64_t seed,bool imaging,const std::vector<double>* precomputedBackground) {
    const auto& c=config_;
    if(!validConfig(c)||!validScene(scene,c)||scene.timeS<lastTimeS_||!finite(pointing)||!positive(norm(pointing)))return {};
    if(scene.timeS==lastTimeS_)return cached_;
    const auto plane=planeFor(pointing,unit(scene.up));
    const double halfWidth=.5*c.width,halfHeight=.5*c.height,pitch=c.pixelPitchRad;
    const auto pixels=static_cast<std::size_t>(c.width)*c.height;
    auto field=precomputedBackground?incidentFieldImplV1(scene,pointing,c,precomputedBackground):incidentFieldV1(scene,pointing,c);
    if(!field)return {};
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
    // Precompute exact pixel solid angles and clamped/deduped border annulus
    // neighbor lists once per (width, height, pixelPitchRad, backgroundRadiusPixels)
    // (100% bit-for-bit identical to the original formulas).
    struct SolidAngleCacheEntryV1 {
        unsigned width = 0, height = 0;
        std::uint64_t pitchBits = 0;
        int distance = 0;
        std::shared_ptr<const std::vector<double>> omegas;
        std::size_t annulusCount = 0;
        std::array<std::pair<int, int>, 32> annulus{};
        std::array<std::ptrdiff_t, 32> annulusFlatDelta{};
        std::shared_ptr<const std::vector<BorderAnnulusEntryV1>> borderNeighbors;
    };
    const auto distance=static_cast<int>(c.backgroundRadiusPixels);
    std::uint64_t pitchBits = 0;
    std::memcpy(&pitchBits, &c.pixelPitchRad, sizeof(pitchBits));
    SolidAngleCacheEntryV1 cachedGeom;
    {
        static std::mutex omegaMutex;
        static std::vector<SolidAngleCacheEntryV1> omegaCache;
        std::lock_guard<std::mutex> lock(omegaMutex);
        for (const auto& entry : omegaCache) {
            if (entry.width == c.width && entry.height == c.height &&
                entry.pitchBits == pitchBits && entry.distance == distance) {
                cachedGeom = entry;
                break;
            }
        }
        if (!cachedGeom.omegas) {
            cachedGeom.width = c.width;
            cachedGeom.height = c.height;
            cachedGeom.pitchBits = pitchBits;
            cachedGeom.distance = distance;
            auto tbl = std::make_shared<std::vector<double>>(pixels);
            std::vector<double> tanX0(c.width), tanX1(c.width), tanY0(c.height), tanY1(c.height);
            for (unsigned x = 0; x < c.width; ++x) {
                const double x0 = (x - halfWidth) * c.pixelPitchRad;
                tanX0[x] = std::tan(x0);
                tanX1[x] = std::tan(x0 + c.pixelPitchRad);
            }
            for (unsigned y = 0; y < c.height; ++y) {
                const double y0 = (y - halfHeight) * c.pixelPitchRad;
                tanY0[y] = std::tan(y0);
                tanY1[y] = std::tan(y0 + c.pixelPitchRad);
            }
            const auto saPrim = [](double tx, double ty) noexcept {
                return std::atan(tx * ty / std::sqrt(1 + tx * tx + ty * ty));
            };
            for (unsigned y = 0; y < c.height; ++y) {
                const double ty0 = tanY0[y], ty1 = tanY1[y];
                for (unsigned x = 0; x < c.width; ++x) {
                    const double tx0 = tanX0[x], tx1 = tanX1[x];
                    (*tbl)[static_cast<std::size_t>(y) * c.width + x] =
                        saPrim(tx1, ty1) - saPrim(tx0, ty1) - saPrim(tx1, ty0) + saPrim(tx0, ty0);
                }
            }
            cachedGeom.omegas = std::move(tbl);
            for (unsigned i = 0; i < 32; ++i) {
                const double phase = 2 * pi * i / 32;
                const auto offset = std::pair<int, int>{static_cast<int>(std::lround(distance * std::cos(phase))),
                    static_cast<int>(std::lround(distance * std::sin(phase)))};
                if (std::find(cachedGeom.annulus.begin(),
                              cachedGeom.annulus.begin() + static_cast<std::ptrdiff_t>(cachedGeom.annulusCount),
                              offset) == cachedGeom.annulus.begin() + static_cast<std::ptrdiff_t>(cachedGeom.annulusCount)) {
                    cachedGeom.annulus[cachedGeom.annulusCount++] = offset;
                }
            }
            const int wI = static_cast<int>(c.width), hI = static_cast<int>(c.height);
            for (std::size_t i = 0; i < cachedGeom.annulusCount; ++i) {
                cachedGeom.annulusFlatDelta[i] = static_cast<std::ptrdiff_t>(cachedGeom.annulus[i].second) * wI + cachedGeom.annulus[i].first;
            }
            if (pixels <= 65535u) {
                auto bTbl = std::make_shared<std::vector<BorderAnnulusEntryV1>>(pixels);
                for (unsigned y = 0; y < c.height; ++y) {
                    const int yI = static_cast<int>(y);
                    const bool yIn = yI >= distance && yI + distance < hI;
                    for (unsigned x = 0; x < c.width; ++x) {
                        const int xI = static_cast<int>(x);
                        if (yIn && xI >= distance && xI + distance < wI) continue;
                        auto& dst = (*bTbl)[static_cast<std::size_t>(y) * c.width + x];
                        for (std::size_t k = 0; k < cachedGeom.annulusCount; ++k) {
                            const auto px = std::clamp(xI + cachedGeom.annulus[k].first, 0, wI - 1);
                            const auto py = std::clamp(yI + cachedGeom.annulus[k].second, 0, hI - 1);
                            const auto neighbor = static_cast<std::uint16_t>(static_cast<std::size_t>(py) * c.width + static_cast<unsigned>(px));
                            bool dup = false;
                            for (std::uint8_t j = 0; j < dst.count; ++j) {
                                if (dst.indices[j] == neighbor) { dup = true; break; }
                            }
                            if (!dup) dst.indices[dst.count++] = neighbor;
                        }
                    }
                }
                cachedGeom.borderNeighbors = std::move(bTbl);
            }
            if (omegaCache.size() < 64) omegaCache.push_back(cachedGeom);
        }
    }
    bool nearestRounding=std::fegetround()==FE_TONEAREST;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    nearestRounding=nearestRounding&&(_mm_getcsr()&0x6000u)==0;
#endif
    std::vector<double> nonNearestOmega;
    const double* omegaPtr = cachedGeom.omegas->data();
    if (!nearestRounding) {
        nonNearestOmega.resize(pixels);
        const auto saPrim = [](double tx, double ty) noexcept {
            return std::atan(tx * ty / std::sqrt(1 + tx * tx + ty * ty));
        };
        for (unsigned y = 0; y < c.height; ++y) {
            const double y0 = (y - halfHeight) * c.pixelPitchRad;
            const double ty0 = std::tan(y0), ty1 = std::tan(y0 + c.pixelPitchRad);
            for (unsigned x = 0; x < c.width; ++x) {
                const double x0 = (x - halfWidth) * c.pixelPitchRad;
                const double tx0 = std::tan(x0), tx1 = std::tan(x0 + c.pixelPitchRad);
                nonNearestOmega[static_cast<std::size_t>(y) * c.width + x] =
                    saPrim(tx1, ty1) - saPrim(tx0, ty1) - saPrim(tx1, ty0) + saPrim(tx0, ty0);
            }
        }
        omegaPtr = nonNearestOmega.data();
    }
    const double satThreshold=c.fullWellElectrons+20*(std::sqrt(c.fullWellElectrons)+c.readNoiseElectrons);
    const double darkPerFrame=c.darkCurrentElectronsS*c.integrationTimeS;
    std::vector<std::size_t> activeContrastSources;
    if(!sourceContrast.empty()) {
        activeContrastSources.reserve(sourceContrast.size());
        for(std::size_t sIdx=0;sIdx<sourceContrast.size();++sIdx) {
            const double* sp=sourceContrast[sIdx].data();
            bool anyNz=false;
            for(std::size_t i=0;i<pixels;++i){if(sp[i]!=0.){anyNz=true;break;}}
            if(anyNz)activeContrastSources.push_back(sIdx);
        }
    }
    const bool hasContrast=!activeContrastSources.empty();
    for(unsigned y=0;y<c.height;++y) {
        const std::size_t rowBase=static_cast<std::size_t>(y)*c.width;
        for(unsigned x=0;x<c.width;++x) {
        const auto index=rowBase+x;
        const double value=std::max(0.,radiance[index]);
        const double omega=omegaPtr[index];
        if(!imaging)pixelWeights[index]=omega/(pitch*pitch);
        const double expected=value*omega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*electronsPerJoule;
        if(!nonnegative(expected)){reset();return {};}
        pixelResponse_[index]+=response*(expected-pixelResponse_[index]);
        if(hasContrast) {
            const double gain=omega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*electronsPerJoule;
            for(const std::size_t sIdx:activeContrastSources)sourceContrast[sIdx][index]*=gain;
        }
        const double total=pixelResponse_[index]+darkPerFrame;
        result.expectedElectrons[index]=total;
        // At >=20 shot-noise standard deviations above full well, saturation
        // is certain to far below binary64 tail precision; avoid huge draws.
        double measured=c.fullWellElectrons;
        if(total<satThreshold) {
            std::poisson_distribution<std::uint64_t> photons(total);
            measured=std::min(c.fullWellElectrons,static_cast<double>(photons(rng_)));
        }
        measured+=normal()*c.readNoiseElectrons;
        result.measuredElectrons[index]=std::round(std::clamp(measured,0.,c.fullWellElectrons)/adcStep)*adcStep;
        }
    }
    std::vector<double> contrast(pixels),variance(pixels),significance(pixels);
    const std::size_t annulusCount = cachedGeom.annulusCount;
    const auto& annulus = cachedGeom.annulus;
    const auto& annulusFlatDelta = cachedGeom.annulusFlatDelta;
    const auto* borderTable = cachedGeom.borderNeighbors ? cachedGeom.borderNeighbors->data() : nullptr;
    const int wInt=static_cast<int>(c.width),hInt=static_cast<int>(c.height);
    std::array<double,32> neighborBuf{};
    std::array<std::size_t,32> neighborIdxBuf{};
    const auto medianSlice=[](double* data,std::size_t n) noexcept {
        if(n==0)return 0.;
        const std::size_t mid=n/2;
        std::nth_element(data,data+mid,data+n);
        return (n&1u)?data[mid]:.5*(data[mid]+*std::max_element(data,data+mid));
    };
    const double readAdcVar=c.readNoiseElectrons*c.readNoiseElectrons+adcStep*adcStep/12;
    const double* measPtr=result.measuredElectrons.data();
    // Scene radiance is never subtracted as detector knowledge. A dense unique
    // circular annulus avoids the four-of-eight square-stencil optical-tail
    // contamination that otherwise generates artificial cold companion lobes.
    result.imageProcessingAvailable=imaging;
    if(imaging){result.backgroundElectrons.resize(pixels);result.contrastElectrons.resize(pixels);
        result.noiseSigmaElectrons.resize(pixels);result.detectionThresholdElectrons.resize(pixels);}
    const auto storePixelStats=[&](std::size_t index,double background,std::size_t nCount) noexcept {
        const double ctr=measPtr[index]-background;
        const double var=std::max(1.,measPtr[index]+readAdcVar)+
            (background+readAdcVar)*pi/(2*static_cast<double>(nCount));
        const double sig=std::sqrt(var);
        contrast[index]=ctr;
        variance[index]=var;
        significance[index]=ctr/sig;
        if(imaging){result.backgroundElectrons[index]=background;result.contrastElectrons[index]=ctr;
            result.noiseSigmaElectrons[index]=sig;result.detectionThresholdElectrons[index]=c.detectionSigma*sig;}
    };
    const auto runBorderPixel=[&](std::size_t index,int xInt,int yInt) noexcept {
        std::size_t nCount=0;
        if(borderTable) {
            const auto& b=borderTable[index];
            nCount=b.count;
            for(std::size_t k=0;k<nCount;++k)neighborBuf[k]=measPtr[b.indices[k]];
        } else {
            for(std::size_t k=0;k<annulusCount;++k) {
                const auto px=std::clamp(xInt+annulus[k].first,0,wInt-1);
                const auto py=std::clamp(yInt+annulus[k].second,0,hInt-1);
                const auto neighbor=static_cast<std::size_t>(py)*c.width+static_cast<unsigned>(px);
                bool dup=false;
                for(std::size_t j=0;j<nCount;++j)if(neighborIdxBuf[j]==neighbor){dup=true;break;}
                if(!dup){neighborIdxBuf[nCount]=neighbor;neighborBuf[nCount]=measPtr[neighbor];++nCount;}
            }
        }
        storePixelStats(index,medianSlice(neighborBuf.data(),nCount),nCount);
    };
#if FSIM_HAS_RUNTIME_AVX2
    const bool useAvx2=runtimeHasAvx2V1();
    std::array<double,256> rowBgBuf{};
    std::vector<double> rowBgHeap;
    double* rowBgPtr=(useAvx2&&c.width<=rowBgBuf.size())?rowBgBuf.data():(useAvx2?(rowBgHeap.resize(c.width),rowBgHeap.data()):nullptr);
#endif
    for(unsigned y=0;y<c.height;++y) {
        const int yInt=static_cast<int>(y);
        const bool yInterior=yInt>=distance&&yInt+distance<hInt;
        const std::size_t rowBase=static_cast<std::size_t>(y)*c.width;
        unsigned x=0;
#if FSIM_HAS_RUNTIME_AVX2
        if(useAvx2&&yInterior&&annulusCount>0&&annulusCount<=32&&wInt>=2*distance+4) {
            const unsigned xStart=static_cast<unsigned>(distance);
            const unsigned xVecEnd=xStart+((static_cast<unsigned>(wInt-2*distance))/4u)*4u;
            for(;x<xStart;++x)runBorderPixel(rowBase+x,static_cast<int>(x),yInt);
            annulusInteriorRowAvx2(measPtr,rowBase,xStart,xVecEnd,annulusFlatDelta.data(),annulusCount,rowBgPtr);
            for(;x<xVecEnd;++x)storePixelStats(rowBase+x,rowBgPtr[x],annulusCount);
        } else if(useAvx2&&!yInterior&&borderTable&&c.width>=4u) {
            for(;x+3u<c.width;) {
                const auto& b0=borderTable[rowBase+x];
                const auto& b1=borderTable[rowBase+x+1u];
                const auto& b2=borderTable[rowBase+x+2u];
                const auto& b3=borderTable[rowBase+x+3u];
                if(b0.count>0&&b0.count<=32&&b0.count==b1.count&&b0.count==b2.count&&b0.count==b3.count) {
                    double bg4[4];
                    annulusBorder4Avx2(measPtr,&b0,&b1,&b2,&b3,b0.count,bg4);
                    for(unsigned lane=0;lane<4;++lane)storePixelStats(rowBase+x+lane,bg4[lane],b0.count);
                    x+=4u;
                } else {
                    runBorderPixel(rowBase+x,static_cast<int>(x),yInt);
                    ++x;
                }
            }
        }
#endif
        for(;x<c.width;++x) {
            const int xInt=static_cast<int>(x);
            const std::size_t index=rowBase+x;
            if(yInterior&&xInt>=distance&&xInt+distance<wInt) {
                const std::ptrdiff_t baseIdx=static_cast<std::ptrdiff_t>(index);
                for(std::size_t k=0;k<annulusCount;++k)
                    neighborBuf[k]=measPtr[static_cast<std::size_t>(baseIdx+annulusFlatDelta[k])];
                storePixelStats(index,medianSlice(neighborBuf.data(),annulusCount),annulusCount);
            } else {
                runBorderPixel(index,xInt,yInt);
            }
        }
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
    std::vector<std::vector<std::size_t>> sortedComponents;
    if(imaging) {
        std::vector<std::size_t> permutation(result.features.size());std::iota(permutation.begin(),permutation.end(),0);
        std::sort(permutation.begin(),permutation.end(),[&](auto a,auto b){return featureLess(result.features[a],result.features[b]);});
        std::vector<ThermalImageFeatureV1> sorted;sorted.reserve(permutation.size());
        sortedComponents.resize(permutation.size());
        for(std::size_t i=0;i<permutation.size();++i) {
            sorted.push_back(result.features[permutation[i]]);
            auto& comp=sortedComponents[i];
            comp=std::move(componentPixels[permutation[i]]);
            std::sort(comp.begin(),comp.end());
            for(const auto pixel:comp)result.segmentation[pixel]=static_cast<std::uint16_t>(i+1);
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
    std::vector<std::uint8_t> sourceActiveFlag(sourceContrast.size(),0u);
    for(const std::size_t sIdx:activeContrastSources)sourceActiveFlag[sIdx]=1u;
    for(const auto source:order) {
        if(!sourceActiveFlag[source])continue;
        auto& charge=scene.refereeAttributionIndices[source]?
            attributionResponse_[*scene.refereeAttributionIndices[source]]:unattributedResponse_;
        if(charge.size()!=pixels)charge.assign(pixels,0);
        const double* src=sourceContrast[source].data();
        double* dst=charge.data();
        for(std::size_t i=0;i<pixels;++i)dst[i]+=response*src[i];
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
            if(imaging) {
                for(const std::size_t i:sortedComponents[featureIndex])
                    score+=polarity*charge[i]*std::min(1.,c.fullWellElectrons/std::max(1e-300,result.expectedElectrons[i]));
            } else {
                for(std::size_t i=0;i<pixels;++i)
                    score+=polarity*charge[i]*std::min(1.,c.fullWellElectrons/std::max(1e-300,result.expectedElectrons[i]));
            }
            return score;
        };
        best=std::max(0.,detectorScore(unattributedResponse_));
        for(const auto& [id,charge]:attributionResponse_) {
            const double score=detectorScore(charge);
            if(score>best){best=score;feature.refereeAttributionIndex=id;}
        }
    }
    lastTimeS_=scene.timeS;cached_=result;
    return result;
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
    return exposeImplV1(frame,pointing,seed,false);
}
ThermalDualBandResultV1 ThermalDualBandReceiverV1::exposeFastBackgroundV1(const ThermalDualBandSceneFrameV1& frame,V pointing,std::uint64_t seed) {
    return exposeImplV1(frame,pointing,seed,true);
}
namespace {
thread_local bool gDualBandParallelAllowed = false;
}
DualBandParallelGuardV1::DualBandParallelGuardV1(bool allow) noexcept : previous(gDualBandParallelAllowed) {
    gDualBandParallelAllowed = allow;
}
DualBandParallelGuardV1::~DualBandParallelGuardV1() noexcept {
    gDualBandParallelAllowed = previous;
}
bool DualBandParallelGuardV1::allowed() noexcept {
    return gDualBandParallelAllowed;
}
ThermalDualBandResultV1 ThermalDualBandReceiverV1::exposeImplV1(const ThermalDualBandSceneFrameV1& frame,V pointing,std::uint64_t seed,bool fastBackground) {
    if(!validScene(frame,config_)||!finite(pointing)||!positive(norm(pointing))||
        (cached_.valid&&frame.channels[0].timeS<cached_.timeS))return {};
    if(cached_.valid&&frame.channels[0].timeS==cached_.timeS)return cached_;
    ThermalDualBandResultV1 result;result.timeS=frame.channels[0].timeS;
    if(fastBackground&&DualBandParallelGuardV1::allowed()) {
        std::exception_ptr ch1Err;
        std::thread ch1Thread([&]() noexcept {
            try {
                result.channels[1]=channels_[1].exposeFastBackgroundV1(frame.channels[1],pointing,seed^0x42414e4430303032ull,true);
            } catch(...) { ch1Err=std::current_exception(); }
        });
        result.channels[0]=channels_[0].exposeFastBackgroundV1(frame.channels[0],pointing,seed^0x42414e4430303031ull,true);
        ch1Thread.join();
        if(ch1Err)std::rethrow_exception(ch1Err);
    } else {
        result.channels[0]=fastBackground?channels_[0].exposeFastBackgroundV1(frame.channels[0],pointing,seed^0x42414e4430303031ull,true)
                                         :channels_[0].expose(frame.channels[0],pointing,seed^0x42414e4430303031ull,true);
        result.channels[1]=fastBackground?channels_[1].exposeFastBackgroundV1(frame.channels[1],pointing,seed^0x42414e4430303032ull,true)
                                         :channels_[1].expose(frame.channels[1],pointing,seed^0x42414e4430303032ull,true);
    }
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
