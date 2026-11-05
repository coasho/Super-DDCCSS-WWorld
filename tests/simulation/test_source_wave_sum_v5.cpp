// Standalone native method executable; no World, SDK fixture, or target truth.
#include "simulation/mission_systems/components/support/SourceWaveSumV5.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace {
using fsim::ms::components::source_wave_sum_v5::OpticsV5;
using fsim::ms::components::source_wave_sum_v5::arithmeticV5;
unsigned checks=0,failures=0,cases=0;
void check(bool predicate,const char* description) {
    ++checks;
    if(!predicate){++failures;std::fprintf(stderr,"FAIL %s\n",description);}
}
std::uint64_t bits(double value) {std::uint64_t result=0;std::memcpy(&result,&value,sizeof(result));return result;}
struct Environment {
    std::fenv_t saved{};
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    unsigned csr=_mm_getcsr();
#endif
    Environment(){std::fegetenv(&saved);}
    ~Environment(){std::fesetenv(&saved);
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        _mm_setcsr(csr);
#endif
    }
    void nearest() {
        std::fesetround(FE_TONEAREST);
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        _mm_setcsr(_mm_getcsr()&~0xe040u);
#endif
    }
};
struct Inputs {
    unsigned width=8,height=8,phases=8,exposures=4;
    double pixelArea=1e-6;
    std::vector<double> solid,transmission,x,y;
    explicit Inputs(unsigned count=8,unsigned inner=4,unsigned columns=8,unsigned rows=8):
        width(columns),height(rows),phases(count),exposures(inner),
        solid(static_cast<std::size_t>(columns)*rows,pixelArea),
        transmission(static_cast<std::size_t>(count)*inner*columns*rows,0),
        x(static_cast<std::size_t>(count)*inner*columns,0),y(static_cast<std::size_t>(count)*inner*rows,0) {}
    OpticsV5 optics() const {return {solid,transmission,width,height,phases,exposures,pixelArea};}
};
// Immutable original prepare() per-term arithmetic and loop order. This oracle
// is intentionally dense and separate from the candidate implementation.
void originalDense(double& accumulator,const Inputs& a,std::size_t timeIndex) {
    const auto pixels=static_cast<std::size_t>(a.width)*a.height;
    for(unsigned row=0;row<a.height;++row)for(unsigned col=0;col<a.width;++col) {
        const auto index=static_cast<std::size_t>(row)*a.width+col;
        const double value=a.y[timeIndex*a.height+row]*a.x[timeIndex*a.width+col]*a.solid[index]/a.pixelArea;
        accumulator+=value*a.transmission[timeIndex*pixels+index]/a.exposures;
    }
}
void denseEquivalence() {
    ++cases;
    // Full, sparse holes, first/last detector cells, off-field/all zero, and
    // signed-zero payloads. Every phase and subexposure is separately joined.
    for(unsigned inner=1;inner<=16;++inner)for(unsigned pattern=0;pattern<8;++pattern) {
        Inputs a(8,inner);
        for(std::size_t i=0;i<a.x.size();++i) {
            const auto col=i%8;
            a.x[i]=pattern==0?0:pattern==1?-0.0:pattern==2?.125:
                pattern==3?(col==0?1:0):pattern==4?(col==7?1:0):
                (col%3==0?static_cast<double>((i%7)+1)/8:0);
            a.y[i]=pattern<=1?-0.0:pattern==2?.375:
                pattern==3?(col==7?1:0):pattern==4?(col==0?1:0):
                (col%2==1?static_cast<double>((i%5)+1)/8:0);
        }
        for(std::size_t i=0;i<a.transmission.size();++i)
            a.transmission[i]=i%11==0?-0.0:static_cast<double>(i%17)/16;
        for(std::size_t i=0;i<a.solid.size();++i)a.solid[i]=i%7==0?-0.0:a.pixelArea/(1+static_cast<double>(i%5));
        const auto optics=a.optics();check(optics.eligibleOptics(),"normal optics admitted");
        for(unsigned j=0;j<a.phases;++j) {
            double actual=0,reference=0;
            for(unsigned k=0;k<a.exposures;++k) {
                const auto ti=static_cast<std::size_t>(j)*a.exposures+k;
                originalDense(reference,a,ti);
                check(optics.addSample(actual,a.x,a.y,ti),"normal sparse sample used");
                check(bits(actual)==bits(reference),"byte-equal after EACH subexposure");
            }
        }
    }
}
void detectorBounds() {
    ++cases;
    for(const auto dimensions:{std::array<unsigned,2>{{8,128}},std::array<unsigned,2>{{128,8}},std::array<unsigned,2>{{128,128}}}) {
        Inputs a(1,1,dimensions[0],dimensions[1]);
        a.x.at(0)=.25;a.x.at(a.x.size()-1)=.75;a.y.at(0)=.375;a.y.at(a.y.size()-1)=.625;
        a.transmission.front()=1;a.transmission.back()=1;
        const auto optics=a.optics();double actual=0,reference=0;originalDense(reference,a,0);
        check(optics.addSample(actual,a.x,a.y,0),"rectangular and128-edge buffers admitted");
        check(bits(actual)==bits(reference),"first/last row/column exact");
    }
}
void additionOrder() {
    ++cases;Inputs a(1,2);
    for(std::size_t ti=0;ti<2;++ti){a.x[ti*8]=a.y[ti*8]=1;a.transmission[ti*64]=1;}
    const auto optics=a.optics();double actual=std::ldexp(1.,52),reference=actual;
    for(std::size_t ti=0;ti<2;++ti) {
        originalDense(reference,a,ti);check(optics.addSample(actual,a.x,a.y,ti),"ordered sample admitted");
    }
    check(bits(actual)==bits(reference),"cross-subexposure accumulator exact");
    check(actual==std::ldexp(1.,52),"two half-ULP increments each round away");
    check(actual!=std::ldexp(1.,52)+1,"not a pre-summed phase contribution");
}
void subnormalsAndAccumulator() {
    ++cases;Inputs a(1,1);a.x[0]=std::numeric_limits<double>::min()/2;a.y[0]=1;a.transmission[0]=1;
    const auto optics=a.optics();double actual=0,reference=0;originalDense(reference,a,0);
    check(optics.addSample(actual,a.x,a.y,0),"nonzero subnormal probability retained");
    check(actual>0&&actual<std::numeric_limits<double>::min(),"nonzero subnormal output survives");
    check(bits(actual)==bits(reference),"subnormal bit equality");
    for(const double initial:{-0.0,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        double value=initial;const auto before=bits(value);
        check(!optics.addSample(value,a.x,a.y,0),"uncertified accumulator falls back");
        check(bits(value)==before,"fallback never mutates accumulator");
    }
    actual=3;check(!optics.addSample(actual,a.x,a.y,1),"bad sample index falls back");check(actual==3,"bad index unchanged");
    auto shortX=a.x;shortX.pop_back();check(!optics.addSample(actual,shortX,a.y,0),"short buffer falls back");check(actual==3,"short buffer unchanged");
}
void invalidOptics() {
    ++cases;
    for(unsigned kind=0;kind<8;++kind) {
        Inputs a(1,1);const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
        if(kind<4)a.solid[0]=kind==0?nan:kind==1?inf:kind==2?-1:a.pixelArea*2;
        else a.transmission[0]=kind==4?nan:kind==5?inf:kind==6?-1:2;
        const auto optics=a.optics();check(!optics.eligibleOptics(),"bad immutable optics rejected for sparse use");
        double actual=7;check(!optics.addSample(actual,a.x,a.y,0),"bad optics requests old dense");
        check(actual==7,"bad optics untouched before old dense");
        if(kind==0||kind==1||kind==4||kind==5) {
            double reference=0;originalDense(reference,a,0);
            check(std::isnan(reference),"original zero times nonfinite MUST remain nonfinite");
        }
    }
}
void invalidProbabilities() {
    ++cases;Inputs a(1,1);const auto optics=a.optics();
    for(const double bad:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-1.,2.}) {
        for(const bool axis:{false,true}) {
            auto x=a.x,y=a.y;(axis?y:x)[7]=bad;
            double actual=5;check(!optics.addSample(actual,x,y,0),"bad axis cannot hide behind zero other axis");
            check(actual==5,"axis rejection is atomic");
        }
    }
}
void environmentFallbacks() {
    ++cases;Inputs a(1,1);const auto optics=a.optics();
    for(const int rounding:{FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO}) {
        Environment restore;check(std::fesetround(rounding)==0,"set native rounding mode");
        check(!arithmeticV5(),"non-nearest arithmetic refuses sparse use");
        double actual=3;check(!optics.addSample(actual,a.x,a.y,0),"non-nearest dense fallback");check(actual==3,"rounding fallback unchanged");
        check(!a.optics().eligibleOptics(),"preflight itself must use admitted arithmetic");
    }
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    for(const unsigned mode:{0x8000u,0x0040u,0x2000u,0x4000u}) {
        Environment restore;_mm_setcsr((_mm_getcsr()&~0xe040u)|mode);
        check(!arithmeticV5(),"actual MXCSR flag rejects sparse use");
        double actual=3;check(!optics.addSample(actual,a.x,a.y,0),"MXCSR dense fallback");check(actual==3,"MXCSR fallback unchanged");
        check(!a.optics().eligibleOptics(),"MXCSR preflight rejects");
    }
#endif
}
}
int main() {
    Environment restore;restore.nearest();
    check(arithmeticV5(),"supported native nearest gradual-underflow arithmetic required");
    if(failures==0) {
        denseEquivalence();detectorBounds();additionOrder();subnormalsAndAccumulator();invalidOptics();invalidProbabilities();environmentFallbacks();
    }
    std::printf("{\"cases\":%u,\"assertions\":%u,\"failures\":%u,\"scope\":\"native_source_wave_exact_order_method_only\"}\n",cases,checks,failures);
    return failures==0&&cases==7?0:1;
}
