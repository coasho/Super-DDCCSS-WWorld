#include "MovingConicMomentsV4.h"
#include "CertifiedNormalTableV4.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <atomic>
#include <cfenv>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <unordered_map>
#include <list>
#include <mutex>
#include <stdexcept>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif

namespace fsim::ms::components::conic_moments_v4 {
namespace {
using L=long double;
constexpr L infinity=std::numeric_limits<L>::infinity();
constexpr L lepsilon=std::numeric_limits<L>::epsilon();
constexpr L ltiny=std::numeric_limits<L>::denorm_min();
constexpr double depsilon=std::numeric_limits<double>::epsilon();
constexpr double dtiny=std::numeric_limits<double>::denorm_min();
constexpr unsigned maximumDegree=8;
constexpr L extraTail=5e-19L;
constexpr double extraTailD=5.000000000000001e-19;
constexpr double dminNormal=std::numeric_limits<double>::min();

double fromBits(std::uint64_t raw) {double value;std::memcpy(&value,&raw,sizeof(value));return value;}
double nextUpPositiveD(double x) noexcept {
    if(!std::isfinite(x))return std::numeric_limits<double>::infinity();
    if(x==0)return dtiny;
    std::uint64_t bits=0;std::memcpy(&bits,&x,sizeof(bits));
    return fromBits(bits+1);
}
template<bool subtract=false>
double padSubnormalD(double value,unsigned units) noexcept {
    if(value>=8.0*static_cast<double>(units)*dminNormal)return value;
    const double padding=static_cast<double>(units)*dtiny;
    if constexpr(subtract)return value-padding;
    return value+padding;
}
template<unsigned Ops>
double upperPositiveD(double value) noexcept {
    if(!(value>=0)||!std::isfinite(value))return std::numeric_limits<double>::infinity();
    constexpr double n=static_cast<double>(Ops);
    constexpr double gamma=n*depsilon/(1.0-n*depsilon);
    constexpr double scale=(1.0/(1.0-gamma))*(1.0+4.0*depsilon);
    return nextUpPositiveD(padSubnormalD(value,Ops)*scale);
}
// The installed MinGW nextafterl retains x's sign when x is -0.
// At zero the adjacent representable value is known exactly, regardless of
// the zero sign. Nonzero arithmetic keeps the original native operation.
L down(L x) {if(x==0)return -ltiny;return std::isfinite(x)?std::nextafter(x,-infinity):x;}
L up(L x) {if(x==0)return ltiny;return std::isfinite(x)?std::nextafter(x,infinity):x;}
struct I {
    L lo=0,hi=0;
    I()=default;explicit I(L x):lo(x),hi(x){}
    I(L low,L high):lo(low),hi(high){}
    bool valid() const {return std::isfinite(lo)&&std::isfinite(hi)&&lo<=hi;}
};
I invalid(){return {infinity,-infinity};}
bool exactZero(I a){return a.lo==0&&a.hi==0;}
I operator+(I a,I b){if(!a.valid()||!b.valid())return invalid();if(exactZero(a))return b;if(exactZero(b))return a;return {down(a.lo+b.lo),up(a.hi+b.hi)};}
I operator-(I a){return {-a.hi,-a.lo};}
I operator-(I a,I b){return a+-b;}
I operator*(I a,I b){
    if(!a.valid()||!b.valid())return invalid();
    if(exactZero(a)||exactZero(b))return I(0);
    const std::array<L,4> p{{a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi}};
    return {down(*std::min_element(p.begin(),p.end())),up(*std::max_element(p.begin(),p.end()))};
}
I operator/(I a,I b){
    if(!a.valid()||!b.valid()||(b.lo<=0&&b.hi>=0))return invalid();
    if(exactZero(a))return I(0);
    const std::array<L,4> p{{a.lo/b.lo,a.lo/b.hi,a.hi/b.lo,a.hi/b.hi}};
    return {down(*std::min_element(p.begin(),p.end())),up(*std::max_element(p.begin(),p.end()))};
}
I intersect(I a,L low,L high){return {std::max(a.lo,low),std::min(a.hi,high)};}
L magnitude(I a){return std::max(std::abs(a.lo),std::abs(a.hi));}
I square(I a){
    const L low=a.lo<=0&&a.hi>=0?0:std::min(a.lo*a.lo,a.hi*a.hi);
    return {std::max(0.L,down(low)),up(std::max(a.lo*a.lo,a.hi*a.hi))};
}
I power(I a,unsigned n){I value(1);while(n){if(n&1u)value=value*a;a=a*a;n>>=1u;}return value;}
I encoded(table::Bounds a){return {fromBits(a.low),fromBits(a.high)};}
template<bool subtract=false>
L padSubnormalV5(L value,unsigned units) {
    // V5 admits full precision and round-to-nearest only. At 8*units*min
    // even the gap to the preceding number is strictly wider than twice
    // units*denorm_min. The rounded value is therefore unchanged. Keep
    // the original operation throughout the small-operand domain.
    constexpr L minimumNormal=std::numeric_limits<L>::min();
    if(value>=8*static_cast<L>(units)*minimumNormal)return value;
    const L padding=static_cast<L>(units)*ltiny;
    if constexpr(subtract)return value-padding;
    return value+padding;
}

template<bool fastPadding=false>
double upperPositive(L value,unsigned operations=32){
    if(!(value>=0)||!std::isfinite(value))return std::numeric_limits<double>::infinity();
    const L n=operations,gamma=n*lepsilon/(1-n*lepsilon);
    if(!(gamma<1))return std::numeric_limits<double>::infinity();
    const L padded=fastPadding?padSubnormalV5(value,operations):value+n*ltiny;
    const L bound=padded/(1-gamma);
    const double stored=static_cast<double>(bound);
    if constexpr(fastPadding) {
        if(!std::isfinite(stored))return stored;
        // The caller admits binary64. Positive finite encodings advance by
        // one integer bit pattern, with either zero mapped to the first subnormal.
        if(stored==0)return dtiny;
        std::uint64_t bits=0;
        std::memcpy(&bits,&stored,sizeof(bits));
        return fromBits(bits+1);
    }
    return std::nextafter(stored,std::numeric_limits<double>::infinity());
}
struct Mid {double value=0,error=0;};
struct GaussianRowsV5 {
    std::vector<Mid> lower,height;
    std::vector<unsigned> active;
    std::vector<double> omitted;
};
struct GaussianKeyV5 {
    std::array<std::uint64_t,4> bits{};unsigned kind=0;
    bool operator==(const GaussianKeyV5& other) const {return bits==other.bits&&kind==other.kind;}
};
struct GaussianHashV5 {
    std::size_t operator()(const GaussianKeyV5& key) const noexcept {
        std::uint64_t value=key.kind;
        for(const auto bits:key.bits)value^=bits+0x9e3779b97f4a7c15ull+(value<<6)+(value>>2);
        return static_cast<std::size_t>(value);
    }
};
bool samePointV5(I value,double& stored) {
    if(!value.valid()||value.lo!=value.hi)return false;
    if(value.lo==0&&std::signbit(value.lo)!=std::signbit(value.hi))return false;
    stored=static_cast<double>(value.lo);
    return std::isfinite(stored)&&static_cast<L>(stored)==value.lo&&
        (stored!=0||std::signbit(stored)==std::signbit(value.lo));
}
std::uint64_t gaussianBitsV5(double value) {std::uint64_t bits=0;std::memcpy(&bits,&value,sizeof(bits));return bits;}
struct GaussianCacheV5 {
    const Request policy;const std::size_t capacity;
    std::size_t bytes=0,hits=0,misses=0;
    struct Entry {
        std::shared_ptr<const GaussianRowsV5> rows;
        std::shared_ptr<const std::array<I,8>> moments;
        std::shared_ptr<const std::array<Mid,8>> fastMid;
        std::list<GaussianKeyV5>::iterator position;
        std::size_t charge=0;
    };
    mutable std::mutex mutex;
    std::list<GaussianKeyV5> order;
    std::unordered_map<GaussianKeyV5,Entry,GaussianHashV5> entries;
    GaussianCacheV5(const Request& request,std::size_t limit):policy(request),capacity(limit){}
    Entry get(const GaussianKeyV5& key) {
        std::lock_guard<std::mutex> lock(mutex);
        const auto found=entries.find(key);
        if(found==entries.end()){++misses;return {};}
        ++hits;order.splice(order.begin(),order,found->second.position);return found->second;
    }
    void put(const GaussianKeyV5& key,Entry value) {
        if(value.charge>capacity)return;
        std::lock_guard<std::mutex> lock(mutex);
        const auto found=entries.find(key);
        if(found!=entries.end()){order.splice(order.begin(),order,found->second.position);return;}
        order.push_front(key);value.position=order.begin();
        try {entries.emplace(key,value);}catch(...){order.pop_front();throw;}
        bytes+=value.charge;
        while(bytes>capacity&&!order.empty()) {
            const auto last=entries.find(order.back());
            if(last==entries.end())throw std::logic_error("Gaussian memo order invariant violated");
            bytes-=last->second.charge;
            entries.erase(last);order.pop_back();
        }
    }
    GaussianMemoWorkV5 work() const {
        std::lock_guard<std::mutex> lock(mutex);return {hits,misses,entries.size(),bytes};
    }
};
thread_local GaussianCacheV5* activeGaussianCacheV5=nullptr;
struct GaussianScopeV5 {
    GaussianCacheV5* previous;
    explicit GaussianScopeV5(GaussianCacheV5* cache):previous(activeGaussianCacheV5){activeGaussianCacheV5=cache;}
    ~GaussianScopeV5(){activeGaussianCacheV5=previous;}
};
template<bool fastPadding=false>
Mid midpoint(I a){
    if(!a.valid())return {0,std::numeric_limits<double>::infinity()};
    const double centre=static_cast<double>(.5L*a.lo+.5L*a.hi);
    return {centre,upperPositive<fastPadding>(std::max(std::abs(a.lo-centre),std::abs(a.hi-centre)),8)};
}
I sqrtI(I a){
    if(!a.valid()||a.lo<0)return invalid();
    L low=a.lo==0?0:std::sqrt(a.lo),high=a.hi==0?0:std::sqrt(a.hi);
    // sqrt is ONLY a seed. The squared endpoint inequalities certify the
    // result, so no unsupported libm ulp assertion enters the enclosure.
    for(unsigned attempt=0;attempt<16;++attempt){
        const bool lower=low==0||up(low*low)<=a.lo;
        const bool higher=high==0?a.hi==0:down(high*high)>=a.hi;
        if(lower&&higher)return {low,high};
        if(!lower)low=std::max(0.L,down(low));
        if(!higher)high=up(high);
    }
    return invalid();
}
I atanSmall(I x){
    if(!x.valid()||magnitude(x)>.5L)return invalid();
    I value,p=x;
    for(unsigned n=0;n<64;++n){const I term=p/I(2*n+1);value=n%2?value-term:value+term;p=p*x*x;}
    const L remainder=up(magnitude(p)/129);
    return {down(value.lo-remainder),up(value.hi+remainder)};
}
I atanPoint(L x);
I atanI(I x){if(!x.valid())return invalid();return {atanPoint(x.lo).lo,atanPoint(x.hi).hi};}
I atanPoint(L x){
    if(!std::isfinite(x))return invalid();
    if(x<0)return -atanPoint(-x);
    if(x>1)return encoded(table::pi)/I(2)-atanI(I(1)/I(x));
    if(x>.5L)return encoded(table::pi)/I(4)+atanSmall((I(x)-I(1))/(I(x)+I(1)));
    return atanSmall(I(x));
}
std::pair<I,I> sinCosPoint(L x){
    if(!std::isfinite(x)||std::abs(x)>1.55L)return {invalid(),invalid()};
    const I xx=I(x)*I(x);I s(x),c(1),st(x),ct(1);
    for(unsigned n=1;n<24;++n){
        st=-st*xx/I((2*n)*(2*n+1));ct=-ct*xx/I((2*n-1)*(2*n));s=s+st;c=c+ct;
    }
    const L se=up(magnitude(st*xx/I(48*49))),ce=up(magnitude(ct*xx/I(47*48)));
    return {{down(s.lo-se),up(s.hi+se)},{down(c.lo-ce),up(c.hi+ce)}};
}
I tanPoint(L x){const auto sc=sinCosPoint(x);return sc.first/sc.second;}
I tanI(I x){if(!x.valid())return invalid();return {tanPoint(x.lo).lo,tanPoint(x.hi).hi};}

// V5-only certified early-stop trig helpers. Retain exact alternating-series
// remainder envelopes while avoiding redundant high-order terms for small angles.
I divPosV5(I a,L k){
    if(!a.valid()||!(k>0)||!std::isfinite(k))return invalid();
    return {down(a.lo/k),up(a.hi/k)};
}
I atanSmallV5(I x){
    if(!x.valid()||magnitude(x)>.5L)return invalid();
    const I xx=x*x;
    I value,p=x;
    for(unsigned n=0;n<64;++n){
        const I term=divPosV5(p,static_cast<L>(2*n+1));
        value=(n&1u)?value-term:value+term;
        p=p*xx;
        const L rem=up(magnitude(p)/static_cast<L>(2*n+3));
        if(rem<=.25L*lepsilon*std::max(1.L,magnitude(value))||n==63)
            return {down(value.lo-rem),up(value.hi+rem)};
    }
    return invalid();
}
I atanPointV5(L x);
I atanIV5(I x){if(!x.valid())return invalid();return {atanPointV5(x.lo).lo,atanPointV5(x.hi).hi};}
I atanPointV5(L x){
    if(!std::isfinite(x))return invalid();
    if(x<0)return -atanPointV5(-x);
    if(x>1)return encoded(table::pi)/I(2)-atanIV5(I(1)/I(x));
    if(x>.5L)return encoded(table::pi)/I(4)+atanSmallV5((I(x)-I(1))/(I(x)+I(1)));
    return atanSmallV5(I(x));
}
std::pair<I,I> sinCosPointV5(L x){
    if(!std::isfinite(x)||std::abs(x)>1.55L)return {invalid(),invalid()};
    const I xx=I(x)*I(x);I s(x),c(1),st(x),ct(1);
    for(unsigned n=1;n<24;++n){
        st=-divPosV5(st*xx,static_cast<L>((2*n)*(2*n+1)));
        ct=-divPosV5(ct*xx,static_cast<L>((2*n-1)*(2*n)));
        s=s+st;c=c+ct;
        const L se=up(magnitude(st*xx)/static_cast<L>((2*n+2)*(2*n+3)));
        const L ce=up(magnitude(ct*xx)/static_cast<L>((2*n+1)*(2*n+2)));
        if((se<=.25L*lepsilon&&ce<=.25L*lepsilon)||n==23)
            return {{down(s.lo-se),up(s.hi+se)},{down(c.lo-ce),up(c.hi+ce)}};
    }
    return {invalid(),invalid()};
}
I tanPointV5(L x){const auto sc=sinCosPointV5(x);return sc.first/sc.second;}
I tanIV5(I x){if(!x.valid())return invalid();return {tanPointV5(x.lo).lo,tanPointV5(x.hi).hi};}

template<std::size_t N>
I normalPolynomial(const std::array<table::CentreRadius,N>& data,double z,bool tail,L argumentError,Work* work,unsigned cell){
    const double centre=(static_cast<double>(cell)+.5)/4,delta=z-centre,absolute=std::abs(delta);
    double value=fromBits(data.back().centre),absoluteSum=std::abs(value),radius=fromBits(data.back().radius);
    for(std::size_t n=N-1;n>0;--n){
        const double coefficient=fromBits(data[n-1].centre);
        value=value*delta+coefficient;absoluteSum=absoluteSum*absolute+std::abs(coefficient);
        radius=radius*absolute+fromBits(data[n-1].radius);
    }
    const L operations=2*static_cast<L>(N)+4;
    const L gamma=operations*depsilon/(1-operations*depsilon);
    const L absoluteUpper=(static_cast<L>(absoluteSum)+operations*dtiny)/(1-gamma);
    const L radiusUpper=(static_cast<L>(radius)+operations*dtiny)/(1-gamma);
    const L deltaError=depsilon*(std::abs(static_cast<L>(z))+centre)+dtiny;
    const L lipschitz=tail?.4L:.25L;
    const I remainder=encoded(table::cells[cell].remainder);
    L error=radiusUpper+gamma*absoluteUpper+operations*dtiny+
        lipschitz*(argumentError+deltaError)+remainder.hi*(tail?absolute:1.);
    error=upperPositive(error,8*static_cast<unsigned>(N)+32);
    if(work)++work->normalPolynomials;
    return {down(static_cast<L>(value)-error),up(static_cast<L>(value)+error)};
}
I qPoint(L x,Work* work){
    if(!std::isfinite(x)||x<0)return invalid();
    if(x>=12){if(work)++work->normalTails;return {0,encoded(table::q12).hi};}
    const double rounded=static_cast<double>(x);
    if(rounded>=12){
        const L error=.4L*std::abs(x-rounded);
        const I q=encoded(table::q12);return {std::max(0.L,down(q.lo-error)),up(q.hi+error)};
    }
    const unsigned cell=std::min(47u,static_cast<unsigned>(rounded*4));
    return intersect(normalPolynomial(table::cells[cell].qFast,rounded,true,std::abs(x-rounded),work,cell),0,.5L);
}
I pdfPoint(L x,Work* work){
    x=std::abs(x);if(!std::isfinite(x))return invalid();
    if(x>=12){if(work)++work->normalTails;return {0,encoded(table::density12).hi};}
    const double rounded=static_cast<double>(x);
    if(rounded>=12){const L error=.25L*std::abs(x-rounded);return {0,up(encoded(table::density12).hi+error)};}
    const unsigned cell=std::min(47u,static_cast<unsigned>(rounded*4));
    return intersect(normalPolynomial(table::cells[cell].pdfFast,rounded,false,std::abs(x-rounded),work,cell),0,.4L);
}
I qI(I x,Work* work){if(!x.valid()||x.lo<0)return invalid();return {qPoint(x.hi,work).lo,qPoint(x.lo,work).hi};}
I cdfPoint(L x,Work* work){return x<0?qPoint(-x,work):I(1)-qPoint(x,work);}
I cdfI(I x,Work* work){if(!x.valid())return invalid();return {cdfPoint(x.lo,work).lo,cdfPoint(x.hi,work).hi};}
I pdfI(I x,Work* work){
    if(!x.valid())return invalid();
    const L minimum=x.lo<=0&&x.hi>=0?0:std::min(std::abs(x.lo),std::abs(x.hi));
    return {pdfPoint(magnitude(x),work).lo,pdfPoint(minimum,work).hi};
}
I cdfDifference(I high,I low,Work* work){
    I value;
    if(low.lo>=0)value=qI(low,work)-qI(high,work);
    else if(high.hi<=0)value=qI(-high,work)-qI(-low,work);
    else value=cdfI(high,work)-cdfI(low,work);
    return intersect(value,0,1);
}
I gaussianPrimitive(I z,Work* work){
    if(!z.valid())return invalid();
    // J0 is monotone; evaluating endpoints avoids interval cancellation.
    const auto point=[&](L x){
        if(x>0)return I(x)+intersect(I(-x)*qPoint(x,work)+pdfPoint(x,work),0,infinity);
        return intersect(I(x)*qPoint(-x,work)+pdfPoint(x,work),0,infinity);
    };
    return {point(z.lo).lo,point(z.hi).hi};
}
I gaussianCumulative(I y,I low,I high,I sigma,Work* work){
    const I value=sigma*(gaussianPrimitive((y-low)/sigma,work)-gaussianPrimitive((y-high)/sigma,work));
    return intersect(value,0,(high-low).hi);
}

using Jet=std::array<I,maximumDegree+1>;
Jet constant(I value){Jet result{};result[0]=value;return result;}
Jet plus(const Jet& a,const Jet& b,unsigned degree){Jet out{};for(unsigned n=0;n<=degree;++n)out[n]=a[n]+b[n];return out;}
Jet negative(const Jet& a,unsigned degree){Jet out{};for(unsigned n=0;n<=degree;++n)out[n]=-a[n];return out;}
Jet times(const Jet& a,const Jet& b,unsigned degree){Jet out{};for(unsigned n=0;n<=degree;++n)for(unsigned k=0;k<=n;++k)out[n]=out[n]+a[k]*b[n-k];return out;}
Jet scaled(const Jet& a,I b,unsigned degree){Jet out{};for(unsigned n=0;n<=degree;++n)out[n]=a[n]*b;return out;}
Jet reciprocal(const Jet& a,unsigned degree){Jet out{};out[0]=I(1)/a[0];for(unsigned n=1;n<=degree;++n){I sum;for(unsigned k=1;k<=n;++k)sum=sum+a[k]*out[n-k];out[n]=-sum/a[0];}return out;}
Jet squareRoot(const Jet& a,unsigned degree){Jet out{};out[0]=sqrtI(a[0]);for(unsigned n=1;n<=degree;++n){I sum;for(unsigned k=1;k<n;++k)sum=sum+out[k]*out[n-k];out[n]=(a[n]-sum)/(I(2)*out[0]);}return out;}
Jet tangentVariable(I x,unsigned degree){Jet out{};out[0]=tanI(x);for(unsigned n=0;n<degree;++n){I sum(n==0?1:0);for(unsigned k=0;k<=n;++k)sum=sum+out[k]*out[n-k];out[n+1]=sum/I(n+1);}return out;}
Jet arctangent(const Jet& y,unsigned degree){Jet out{};out[0]=atanI(y[0]);
    auto denominator=times(y,y,degree);denominator[0]=denominator[0]+I(1);const auto inverse=reciprocal(denominator,degree);
    for(unsigned n=1;n<=degree;++n){I sum;for(unsigned k=0;k<n;++k)sum=sum+I(k+1)*y[k+1]*inverse[n-1-k];out[n]=sum/I(n);}return out;}
bool valid(const Jet& a,unsigned degree){for(unsigned n=0;n<=degree;++n)if(!a[n].valid())return false;return true;}

struct Geometry {
    std::array<I,3> f{},r{},v{},u{};
    I a,b,c,h,g00,g01,g02,g11,g12,g22;
    I denominator(I x,I y) const {I answer;for(unsigned k=0;k<3;++k)answer=answer+square(f[k]+r[k]*x+v[k]*y);return answer;}
    I level(I x,I y) const {return (a+b*x+c*y)/sqrtI(denominator(x,y));}
    bool monotone(I x,I y) const {
        const I d=denominator(x,y),n=a+b*x+c*y,by=g02+g12*x+g22*y;
        return d.valid()&&d.lo>0&&(c*d-n*by).lo>0;
    }
    Jet horizon(I angle,unsigned degree) const {
        const Jet x=tangentVariable(angle,degree);const auto n=plus(constant(a),scaled(x,b,degree),degree);
        Jet y{};
        if(h.lo==0&&h.hi==0)y=scaled(n,-I(1)/c,degree);
        else {
            const I aa=c*c-h*h*g22;
            if(!aa.valid()||(aa.lo<=0&&aa.hi>=0))return constant(invalid());
            const auto bb=scaled(plus(scaled(n,c,degree),negative(scaled(plus(constant(g02),scaled(x,g12,degree),degree),h*h,degree),degree),degree),I(2),degree);
            auto dd=plus(constant(g00),scaled(x,I(2)*g01,degree),degree);dd=plus(dd,scaled(times(x,x,degree),g11,degree),degree);
            const auto cc=plus(times(n,n,degree),negative(scaled(dd,h*h,degree),degree),degree);
            const auto discriminant=plus(times(bb,bb,degree),negative(scaled(cc,I(4)*aa,degree),degree),degree);
            if(!discriminant[0].valid()||discriminant[0].lo<=0)return constant(invalid());
            y=scaled(plus(negative(bb,degree),scaled(squareRoot(discriminant,degree),I(h.lo>0?1:-1),degree),degree),I(1)/(I(2)*aa),degree);
            const I numerator=n[0]+c*y[0];
            if((h.lo>0&&numerator.lo<=0)||(h.hi<0&&numerator.hi>=0))return constant(invalid());
        }
        return arctangent(y,degree);
    }
    I horizon0V5(I angle) const {
        const I x=tanIV5(angle);
        const I n=a+b*x;
        I y;
        if(h.lo==0&&h.hi==0)y=n*(-I(1)/c);
        else {
            const I hh=h*h,aa=c*c-hh*g22;
            if(!aa.valid()||(aa.lo<=0&&aa.hi>=0))return invalid();
            const I bb=I(2)*(n*c-(g02+g12*x)*hh);
            const I dd=g00+(I(2)*g01)*x+(x*x)*g11;
            const I cc=n*n-dd*hh;
            const I discriminant=bb*bb-(I(4)*aa)*cc;
            if(!discriminant.valid()||discriminant.lo<=0)return invalid();
            const I radical=sqrtI(discriminant);
            if(!radical.valid())return invalid();
            y=(-bb+(h.lo>0?radical:-radical))/(I(2)*aa);
            const I numerator=n+c*y;
            if((h.lo>0&&numerator.lo<=0)||(h.hi<0&&numerator.hi>=0))return invalid();
        }
        return atanIV5(y);
    }
};

unsigned binomial(unsigned n,unsigned k) noexcept {
    static constexpr unsigned table[9][9]={
        {1,0,0,0,0,0,0,0,0},
        {1,1,0,0,0,0,0,0,0},
        {1,2,1,0,0,0,0,0,0},
        {1,3,3,1,0,0,0,0,0},
        {1,4,6,4,1,0,0,0,0},
        {1,5,10,10,5,1,0,0,0},
        {1,6,15,20,15,6,1,0,0},
        {1,7,21,35,35,21,7,1,0},
        {1,8,28,56,70,56,28,8,1}
    };
    if(n<=8&&k<=n)return table[n][k];
    unsigned result=1;for(unsigned i=1;i<=k;++i)result=result*(n+1-i)/i;return result;
}
std::array<I,8> moments(I low,I high,I detectorLow,I detectorHigh,I sigma,I anchor,unsigned degree,Work* work){
    std::array<I,8> answer{};
    for(unsigned edge=0;edge<2;++edge){
        const I pixel=edge?detectorHigh:detectorLow;
        std::array<std::array<I,8>,2> primitives{};
        for(unsigned side=0;side<2;++side){
            const I z=(pixel-(side?high:low))/sigma;
            const I cdf=cdfI(z,work),density=pdfI(z,work);
            std::array<I,10> zpower{};zpower[0]=I(1);for(unsigned n=1;n<=degree+1;++n)zpower[n]=zpower[n-1]*z;
            std::array<I,10> normal{};normal[0]=cdf;normal[1]=-density;
            for(unsigned n=2;n<=degree+1;++n)normal[n]=-zpower[n-1]*density+I(n-1)*normal[n-2];
            for(unsigned n=0;n<=degree;++n)primitives[side][n]=(zpower[n+1]*cdf-normal[n+1])/I(n+1);
        }
        for(unsigned n=0;n<=degree;++n){
            I value;
            for(unsigned k=0;k<=n;++k)value=value+I(binomial(n,k))*power(pixel-anchor,n-k)*power(-sigma,k)*(primitives[0][k]-primitives[1][k]);
            value=sigma*value;answer[n]=edge?answer[n]+value:answer[n]-value;
        }
    }
    answer[0]=intersect(answer[0],0,(detectorHigh-detectorLow).hi);
    for(unsigned n=2;n<=degree;n+=2)answer[n]=intersect(answer[n],0,infinity);
    return answer;
}
// Separate explicitly selected fast-moment arithmetic strategy. The materialized V4 operator
// retains its original outward interval recurrence.
struct Ball {
    double value=0;
    double radius=0;
    Ball()=default;
    explicit Ball(double x):value(x){}
    Ball(double x,double r):value(x),radius(r){}
    bool valid() const noexcept {return std::isfinite(value)&&std::isfinite(radius)&&radius>=0;}
};
Ball badBall() noexcept {return {0,std::numeric_limits<double>::infinity()};}
double radiusUpper(double x) noexcept {
    // Each caller expression has at most 16 positive basic binary64 operations.
    // The factor covers this inflation's own rounded operations.
    if(!(x>=0)||!std::isfinite(x))return std::numeric_limits<double>::infinity();
    constexpr double factor=1.0+64.0*depsilon;
    const double result=padSubnormalD(x,64)*factor;
    return std::isfinite(result)?result:std::numeric_limits<double>::infinity();
}
double centreError(double x) noexcept {return depsilon*std::abs(x)+dtiny;}
Ball ball(I x) noexcept {
    if(!x.valid())return badBall();
    const double centre=static_cast<double>(.5L*x.lo+.5L*x.hi);
    const L gap=std::max(std::abs(x.lo-centre),std::abs(x.hi-centre));
    return {centre,nextUpPositiveD(static_cast<double>(padSubnormalV5(gap,8)*(1.L+16.L*lepsilon)))};
}
I interval(Ball x) noexcept {
    if(!x.valid())return invalid();
    return {down(static_cast<L>(x.value)-static_cast<L>(x.radius)),
            up(static_cast<L>(x.value)+static_cast<L>(x.radius))};
}
Ball operator-(Ball a) noexcept {return {-a.value,a.radius};}
Ball operator+(Ball a,Ball b) noexcept {
    if(!a.valid()||!b.valid())return badBall();
    const double centre=a.value+b.value;
    return {centre,radiusUpper(a.radius+b.radius+centreError(centre))};
}
Ball operator-(Ball a,Ball b) noexcept {return a+-b;}
Ball operator*(Ball a,Ball b) noexcept {
    if(!a.valid()||!b.valid())return badBall();
    const double centre=a.value*b.value;
    return {centre,radiusUpper(std::abs(a.value)*b.radius+
        std::abs(b.value)*a.radius+a.radius*b.radius+centreError(centre))};
}
Ball operator/(Ball a,Ball b) noexcept {
    if(!a.valid()||!b.valid())return badBall();
    const double centre=a.value/b.value;
    const double difference=std::abs(b.value)-b.radius;
    if(!(difference>0)||!std::isfinite(centre))return badBall();
    // A strict positive lower bound is required. Absolute underflow padding
    // enters the numerator BEFORE division, so small divisors amplify it.
    const double denominator=padSubnormalD<true>(difference,64)*(1.0-64.0*depsilon);
    if(!(denominator>0))return badBall();
    const double numerator=radiusUpper(a.radius+std::abs(centre)*b.radius+
        centreError(centre)*std::abs(b.value));
    return {centre,radiusUpper(numerator/denominator)};
}
template<std::size_t N>
Ball normalBallPolynomial(const std::array<table::CentreRadius,N>& data,double z,bool tail,
    double argumentError,Work* work,unsigned cell) noexcept {
    const double centre=(static_cast<double>(cell)+.5)/4,delta=z-centre,absolute=std::abs(delta);
    double value=fromBits(data.back().centre),absoluteSum=std::abs(value),radius=fromBits(data.back().radius);
    for(std::size_t n=N-1;n>0;--n) {
        const double coefficient=fromBits(data[n-1].centre);
        value=value*delta+coefficient;absoluteSum=absoluteSum*absolute+std::abs(coefficient);
        radius=radius*absolute+fromBits(data[n-1].radius);
    }
    // Same stored coefficients, remainder and operation bound as the retained
    // interval polynomial. One central evaluation plus the global derivative
    // bound encloses the complete argument ball; no endpoint reevaluation.
    constexpr double operations=2.0*static_cast<double>(N)+4.0;
    constexpr double gamma=operations*depsilon/(1.0-operations*depsilon);
    constexpr double invOneMinusGamma=(1.0/(1.0-gamma))*(1.0+4.0*depsilon);
    const double absoluteUpper=(absoluteSum+operations*dtiny)*invOneMinusGamma;
    const double radiusUpperBound=(radius+operations*dtiny)*invOneMinusGamma;
    const double deltaError=depsilon*(std::abs(z)+centre)+dtiny;
    const double lipschitz=tail?0.4:0.25;
    const double remainderHi=fromBits(table::cells[cell].remainder.high);
    const double error=radiusUpperBound+gamma*absoluteUpper+operations*dtiny+
        lipschitz*(argumentError+deltaError)+remainderHi*(tail?absolute:1.0);
    if(work)++work->normalPolynomials;
    return {value,upperPositiveD<8*N+32>(error)};
}
Ball fastNormalAtMagnitude(Ball argument,bool tail,Work* work) noexcept {
    if(!argument.valid())return badBall();
    const double x=std::abs(argument.value);
    if(x>=12) {
        if(work)++work->normalTails;
        const double limitHi=fromBits(tail?table::q12.high:table::density12.high);
        Ball result(0.5*limitHi,nextUpPositiveD(0.5*limitHi));
        // Entire supported tail: monotonicity supplies the existing table
        // bound. A ball straddling 12 keeps the global derivative padding.
        if(x-argument.radius<=12.0+4.0*depsilon*x)
            result.radius=radiusUpper(result.radius+(tail?0.4:0.25)*argument.radius);
        return result;
    }
    const unsigned cell=std::min(47u,static_cast<unsigned>(x*4));
    return tail?normalBallPolynomial(table::cells[cell].qFast,x,true,argument.radius,work,cell):
        normalBallPolynomial(table::cells[cell].pdfFast,x,false,argument.radius,work,cell);
}
void fastNormalPair(Ball argument,Ball& tailOut,Ball& pdfOut,Work* work) noexcept {
    if(!argument.valid()){tailOut=badBall();pdfOut=badBall();return;}
    const double x=std::abs(argument.value);
    if(x>=12) {
        if(work)work->normalTails+=2;
        const double qHi=fromBits(table::q12.high),pHi=fromBits(table::density12.high);
        tailOut={0.5*qHi,nextUpPositiveD(0.5*qHi)};
        pdfOut={0.5*pHi,nextUpPositiveD(0.5*pHi)};
        if(x-argument.radius<=12.0+4.0*depsilon*x) {
            tailOut.radius=radiusUpper(tailOut.radius+0.4*argument.radius);
            pdfOut.radius=radiusUpper(pdfOut.radius+0.25*argument.radius);
        }
        return;
    }
    const unsigned cell=std::min(47u,static_cast<unsigned>(x*4));
    const double centre=(static_cast<double>(cell)+.5)/4,delta=x-centre,absolute=std::abs(delta);
    const auto& c=table::cells[cell];
    double qVal=fromBits(c.qFast.back().centre),qAbs=std::abs(qVal),qRad=fromBits(c.qFast.back().radius);
    for(std::size_t n=c.qFast.size()-1;n>0;--n) {
        const double coeff=fromBits(c.qFast[n-1].centre);
        qVal=qVal*delta+coeff;qAbs=qAbs*absolute+std::abs(coeff);
        qRad=qRad*absolute+fromBits(c.qFast[n-1].radius);
    }
    double pVal=fromBits(c.pdfFast.back().centre),pAbs=std::abs(pVal),pRad=fromBits(c.pdfFast.back().radius);
    for(std::size_t n=c.pdfFast.size()-1;n>0;--n) {
        const double coeff=fromBits(c.pdfFast[n-1].centre);
        pVal=pVal*delta+coeff;pAbs=pAbs*absolute+std::abs(coeff);
        pRad=pRad*absolute+fromBits(c.pdfFast[n-1].radius);
    }
    constexpr double qOps=2.0*14.0+4.0,qGamma=qOps*depsilon/(1.0-qOps*depsilon),qInv=(1.0/(1.0-qGamma))*(1.0+4.0*depsilon);
    constexpr double pOps=2.0*13.0+4.0,pGamma=pOps*depsilon/(1.0-pOps*depsilon),pInv=(1.0/(1.0-pGamma))*(1.0+4.0*depsilon);
    const double deltaError=depsilon*(std::abs(x)+centre)+dtiny;
    const double argPlusDelta=argument.radius+deltaError;
    const double remHi=fromBits(c.remainder.high);
    const double qErr=(qRad+qOps*dtiny)*qInv+qGamma*((qAbs+qOps*dtiny)*qInv)+qOps*dtiny+0.4*argPlusDelta+remHi*absolute;
    const double pErr=(pRad+pOps*dtiny)*pInv+pGamma*((pAbs+pOps*dtiny)*pInv)+pOps*dtiny+0.25*argPlusDelta+remHi;
    if(work)work->normalPolynomials+=2;
    tailOut={qVal,upperPositiveD<8*14+32>(qErr)};
    pdfOut={pVal,upperPositiveD<8*13+32>(pErr)};
}
Ball mulInt(Ball a,unsigned k) noexcept {
    if(k==1)return a;
    if(!a.valid())return badBall();
    const double kd=static_cast<double>(k);
    const double c=a.value*kd;
    return {c,radiusUpper(a.radius*kd+centreError(c))};
}
Ball divInt(Ball a,unsigned k) noexcept {
    if(k==1)return a;
    if(!a.valid()||k==0)return badBall();
    const double kd=static_cast<double>(k);
    const double c=a.value/kd;
    return {c,radiusUpper(a.radius/kd+centreError(c))};
}
Ball divPosExact(Ball a,double s) noexcept {
    if(!a.valid()||!(s>0)||!std::isfinite(s))return badBall();
    const double c=a.value/s;
    return {c,radiusUpper(a.radius/s+centreError(c))};
}
Ball mulPosExact(Ball a,double s) noexcept {
    if(!a.valid()||!(s>=0)||!std::isfinite(s))return badBall();
    const double c=a.value*s;
    return {c,radiusUpper(a.radius*s+centreError(c))};
}
Mid clampBallToMid(Ball b,double loLim,double hiLim) noexcept {
    if(!b.valid())return {0.0,std::numeric_limits<double>::infinity()};
    const double rawLo=b.value-b.radius,rawHi=b.value+b.radius;
    if(rawLo>=loLim&&rawHi<=hiLim)return {b.value,nextUpPositiveD(b.radius)};
    double lo=rawLo-(depsilon*std::abs(rawLo)+dtiny);
    double hi=rawHi+(depsilon*std::abs(rawHi)+dtiny);
    if(lo<loLim)lo=loLim;
    if(hi>hiLim)hi=hiLim;
    if(!(lo<=hi)||!std::isfinite(lo)||!std::isfinite(hi))
        return {0.0,std::numeric_limits<double>::infinity()};
    const double c=0.5*lo+0.5*hi;
    const double r=std::max(std::abs(lo-c),std::abs(hi-c));
    return {c,upperPositiveD<16>(r)};
}
Ball fastGaussianPrimitive(Ball z,Work* work) noexcept {
    if(!z.valid())return badBall();
    Ball tail,density;
    fastNormalPair(z,tail,density,work);
    return z.value>=0?z+(-z*tail+density):z*tail+density;
}
struct ColumnEdgeCacheV5 {
    bool valid=false;
    double x0=0,x1=0;
    std::uint64_t pixelBits=0;
    unsigned degree=0;
    std::array<std::array<Ball,8>,2> primitives{};
};
bool fastMomentsMidUncachedV5(L lowL,L highL,double detectorLow,double detectorHigh,
    double sigmaD,L anchorL,unsigned degree,ColumnEdgeCacheV5* edgeCache,
    std::array<Mid,8>& outMid,Work* work) noexcept {
#if FLT_EVAL_METHOD != 0
    (void)lowL;(void)highL;(void)detectorLow;(void)detectorHigh;(void)sigmaD;(void)anchorL;(void)degree;(void)edgeCache;(void)outMid;(void)work;
    return false;
#else
    const Ball bottom=ball(I(lowL)),top=ball(I(highL));
    const double spanLim=static_cast<double>(up(static_cast<L>(detectorHigh)-static_cast<L>(detectorLow)));
    const bool exactBounds=bottom.radius==0.0&&top.radius==0.0;
    if(degree==0) {
        Ball e0,e1;
        const std::uint64_t lowBits=gaussianBitsV5(detectorLow);
        if(edgeCache&&edgeCache->valid&&exactBounds&&edgeCache->x0==bottom.value&&edgeCache->x1==top.value&&edgeCache->pixelBits==lowBits) {
            e0=edgeCache->primitives[0][0]-edgeCache->primitives[1][0];
        } else {
            const Ball p0(detectorLow);
            const Ball a0=fastGaussianPrimitive(divPosExact(p0-bottom,sigmaD),work);
            const Ball b0=fastGaussianPrimitive(divPosExact(p0-top,sigmaD),work);
            e0=a0-b0;
        }
        const Ball p1(detectorHigh);
        const Ball a1=fastGaussianPrimitive(divPosExact(p1-bottom,sigmaD),work);
        const Ball b1=fastGaussianPrimitive(divPosExact(p1-top,sigmaD),work);
        e1=a1-b1;
        if(edgeCache&&exactBounds) {
            edgeCache->valid=true;edgeCache->x0=bottom.value;edgeCache->x1=top.value;
            edgeCache->pixelBits=gaussianBitsV5(detectorHigh);edgeCache->degree=0;
            edgeCache->primitives[0][0]=a1;edgeCache->primitives[1][0]=b1;
        }
        outMid[0]=clampBallToMid(mulPosExact(e1-e0,sigmaD),0.0,spanLim);
        return std::isfinite(outMid[0].error);
    }
    std::array<Ball,8> answer{};
    const Ball reference=ball(I(anchorL));
    const double negSigma=-sigmaD;
    for(unsigned edge=0;edge<2;++edge){
        const double pixelVal=edge?detectorHigh:detectorLow;
        const Ball pixel(pixelVal);
        std::array<std::array<Ball,8>,2> primitives{};
        bool hit=false;
        if(edge==0&&edgeCache&&edgeCache->valid&&exactBounds&&edgeCache->degree>=degree&&
           edgeCache->x0==bottom.value&&edgeCache->x1==top.value&&edgeCache->pixelBits==gaussianBitsV5(pixelVal)) {
            primitives=edgeCache->primitives;
            hit=true;
        }
        if(!hit) {
            for(unsigned side=0;side<2;++side){
                const Ball z=divPosExact(pixel-(side?top:bottom),sigmaD);
                Ball tail,density;
                fastNormalPair(z,tail,density,work);
                const Ball cdf=z.value<0?tail:Ball(1.0)-tail;
                std::array<Ball,10> zpower{};zpower[0]=Ball(1.0);
                for(unsigned n=1;n<=degree+1;++n)zpower[n]=zpower[n-1]*z;
                std::array<Ball,10> normal{};normal[0]=cdf;normal[1]=-density;
                for(unsigned n=2;n<=degree+1;++n)normal[n]=-zpower[n-1]*density+mulInt(normal[n-2],n-1);
                for(unsigned n=0;n<=degree;++n)primitives[side][n]=divInt(zpower[n+1]*cdf-normal[n+1],n+1);
            }
            if(edge==1&&edgeCache&&exactBounds) {
                edgeCache->valid=true;edgeCache->x0=bottom.value;edgeCache->x1=top.value;
                edgeCache->pixelBits=gaussianBitsV5(pixelVal);edgeCache->degree=degree;
                edgeCache->primitives=primitives;
            }
        }
        std::array<Ball,8> offset{},spread{};offset[0]=Ball(1.0);spread[0]=Ball(1.0);
        const Ball dx=pixel-reference;
        for(unsigned n=1;n<=degree;++n){
            offset[n]=offset[n-1]*dx;
            const double sv=spread[n-1].value*negSigma;
            spread[n]={sv,radiusUpper(spread[n-1].radius*sigmaD+centreError(sv))};
        }
        for(unsigned n=0;n<=degree;++n){
            Ball value;
            for(unsigned k=0;k<=n;++k)value=value+mulInt(offset[n-k]*spread[k]*(primitives[0][k]-primitives[1][k]),binomial(n,k));
            value=mulPosExact(value,sigmaD);
            answer[n]=edge?answer[n]+value:answer[n]-value;
        }
    }
    outMid[0]=clampBallToMid(answer[0],0.0,spanLim);
    if(!std::isfinite(outMid[0].error))return false;
    for(unsigned n=1;n<=degree;++n) {
        if((n&1u)==0)outMid[n]=clampBallToMid(answer[n],0.0,std::numeric_limits<double>::infinity());
        else outMid[n]={answer[n].value,nextUpPositiveD(answer[n].radius)};
        if(!answer[n].valid()||!std::isfinite(outMid[n].error))return false;
    }
    return true;
#endif
}
bool fastMomentsMidV5(L lowL,L highL,double detectorLow,double detectorHigh,
    double sigmaD,L anchorL,unsigned degree,ColumnEdgeCacheV5* edgeCache,
    std::array<Mid,8>& outMid,Work* work) {
    auto* cache=activeGaussianCacheV5;
    if(!cache)return fastMomentsMidUncachedV5(lowL,highL,detectorLow,detectorHigh,sigmaD,anchorL,degree,edgeCache,outMid,work);
    std::array<double,4> values{};
    if(!samePointV5(I(lowL),values[0])||!samePointV5(I(highL),values[1])||
       gaussianBitsV5(sigmaD)!=gaussianBitsV5(cache->policy.psfSigmaRad)||
       anchorL!=.5L*detectorLow+.5L*detectorHigh||
       (anchorL==0&&std::signbit(anchorL)!=std::signbit(.5L*detectorLow+.5L*detectorHigh)))
        return fastMomentsMidUncachedV5(lowL,highL,detectorLow,detectorHigh,sigmaD,anchorL,degree,nullptr,outMid,work);
    values[2]=detectorLow;values[3]=detectorHigh;
    GaussianKeyV5 key;key.kind=degree;
    for(unsigned i=0;i<4;++i)key.bits[i]=gaussianBitsV5(values[i]);
    if(const auto saved=cache->get(key);saved.fastMid){outMid=*saved.fastMid;return true;}
    if(!fastMomentsMidUncachedV5(lowL,highL,detectorLow,detectorHigh,sigmaD,anchorL,degree,nullptr,outMid,work))return false;
    GaussianCacheV5::Entry entry;
    entry.fastMid=std::make_shared<const std::array<Mid,8>>(outMid);
    entry.charge=sizeof(entry)+sizeof(key)+sizeof(outMid)+256;
    cache->put(key,std::move(entry));
    return true;
}
std::array<I,8> fastMomentsUncached(I low,I high,I detectorLow,I detectorHigh,I sigma,I anchor,unsigned degree,Work* work){
#if FLT_EVAL_METHOD != 0
    (void)low;(void)high;(void)detectorLow;(void)detectorHigh;(void)sigma;(void)anchor;(void)degree;(void)work;
    std::array<I,8> refused{};refused.fill(invalid());return refused;
#else
    const Ball scale=ball(sigma),bottom=ball(low),top=ball(high);
    if(degree==0) {
        const Ball p0=ball(detectorLow),p1=ball(detectorHigh);
        const Ball e0=fastGaussianPrimitive((p0-bottom)/scale,work)-fastGaussianPrimitive((p0-top)/scale,work);
        const Ball e1=fastGaussianPrimitive((p1-bottom)/scale,work)-fastGaussianPrimitive((p1-top)/scale,work);
        std::array<I,8> result{};
        result[0]=intersect(interval(scale*(e1-e0)),0,(detectorHigh-detectorLow).hi);
        return result;
    }
    std::array<Ball,8> answer{};
    const Ball reference=ball(anchor);
    for(unsigned edge=0;edge<2;++edge){
        const Ball pixel=ball(edge?detectorHigh:detectorLow);
        std::array<std::array<Ball,8>,2> primitives{};
        for(unsigned side=0;side<2;++side){
            const Ball z=(pixel-(side?top:bottom))/scale;
            Ball tail,density;
            fastNormalPair(z,tail,density,work);
            const Ball cdf=z.value<0?tail:Ball(1.0)-tail;
            std::array<Ball,10> zpower{};zpower[0]=Ball(1);
            for(unsigned n=1;n<=degree+1;++n)zpower[n]=zpower[n-1]*z;
            std::array<Ball,10> normal{};normal[0]=cdf;normal[1]=-density;
            for(unsigned n=2;n<=degree+1;++n)normal[n]=-zpower[n-1]*density+mulInt(normal[n-2],n-1);
            for(unsigned n=0;n<=degree;++n)primitives[side][n]=divInt(zpower[n+1]*cdf-normal[n+1],n+1);
        }
        std::array<Ball,8> offset{},spread{};offset[0]=Ball(1);spread[0]=Ball(1);
        for(unsigned n=1;n<=degree;++n){offset[n]=offset[n-1]*(pixel-reference);spread[n]=spread[n-1]*(-scale);}
        for(unsigned n=0;n<=degree;++n){
            Ball value;
            for(unsigned k=0;k<=n;++k)value=value+mulInt(offset[n-k]*spread[k]*(primitives[0][k]-primitives[1][k]),binomial(n,k));
            value=scale*value;answer[n]=edge?answer[n]+value:answer[n]-value;
        }
    }
    std::array<I,8> result{};for(unsigned n=0;n<=degree;++n)result[n]=interval(answer[n]);
    result[0]=intersect(result[0],0,(detectorHigh-detectorLow).hi);
    for(unsigned n=2;n<=degree;n+=2)result[n]=intersect(result[n],0,infinity);
    return result;
#endif
}

std::array<I,8> fastMoments(I low,I high,I detectorLow,I detectorHigh,I sigma,I anchor,unsigned degree,Work* work) {
    auto* cache=activeGaussianCacheV5;
    std::array<double,4> values{};double scale=0;
    if(!cache||!samePointV5(low,values[0])||!samePointV5(high,values[1])||
       !samePointV5(detectorLow,values[2])||!samePointV5(detectorHigh,values[3])||
       !samePointV5(sigma,scale)||gaussianBitsV5(scale)!=gaussianBitsV5(cache->policy.psfSigmaRad)||
       anchor.lo!=anchor.hi||anchor.lo!=.5L*values[2]+.5L*values[3]||
       (anchor.lo==0&&std::signbit(anchor.lo)!=std::signbit(.5L*values[2]+.5L*values[3])))
        return fastMomentsUncached(low,high,detectorLow,detectorHigh,sigma,anchor,degree,work);
    GaussianKeyV5 key;key.kind=degree;
    for(unsigned i=0;i<4;++i)key.bits[i]=gaussianBitsV5(values[i]);
    if(const auto saved=cache->get(key);saved.moments)return *saved.moments;
    const auto result=fastMomentsUncached(low,high,detectorLow,detectorHigh,sigma,anchor,degree,work);
    bool validResult=true;for(unsigned n=0;n<=degree;++n)validResult=validResult&&result[n].valid();
    if(validResult) {
        GaussianCacheV5::Entry value;value.moments=std::make_shared<const std::array<I,8>>(result);
        value.charge=sizeof(value)+sizeof(key)+sizeof(result)+256;cache->put(key,std::move(value));
    }
    return result;
}


I weightedEvenMoment(unsigned n,I width,I sigma,I offset){
    I sum;
    for(unsigned j=0;j<=n;j+=2){
        I central;
        for(unsigned k=0;k<=j;k+=2){
            unsigned gaussian=1;for(unsigned z=1;z<j-k;z+=2)gaussian*=z;
            central=central+I(binomial(j,k)*gaussian)*power(width,k)*power(sigma,j-k)/I((1u<<k)*(k+1));
        }
        sum=sum+I(binomial(n,j))*power(offset,n-j)*central;
    }
    return sum;
}

std::optional<Bounds> exported(I value){
    if(!value.valid())return std::nullopt;
    double low=static_cast<double>(value.lo),high=static_cast<double>(value.hi);
    low=std::nextafter(low,-std::numeric_limits<double>::infinity());high=std::nextafter(high,std::numeric_limits<double>::infinity());
    return Bounds{low,high};
}
} // namespace

struct GaussianMemoV5::Impl : GaussianCacheV5 {
    Impl(const Request& request,std::size_t limit):GaussianCacheV5(request,limit){}
};
GaussianMemoV5::GaussianMemoV5(const Request& request,std::size_t capacity)
    :implementation_(std::make_unique<Impl>(request,capacity)){}
GaussianMemoV5::~GaussianMemoV5()=default;
GaussianMemoWorkV5 GaussianMemoV5::work() const {return implementation_->work();}

std::array<long double,2> outwardForCalibration(long double x){return {{down(x),up(x)}};}

bool arithmeticAvailable() noexcept {
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__>0)
    return false;
#endif
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    if((_mm_getcsr()&0xe040u)!=0||(_mm_getcsr()&0x1f80u)!=0x1f80u)return false;
#else
    return false;
#endif
#if defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
    unsigned short control=0;__asm__ __volatile__("fnstcw %0":"=m"(control));
    if((control&0x0f00u)!=0x0300u||(control&0x003fu)!=0x003fu)return false;
#else
    return false;
#endif
    return std::fegetround()==FE_TONEAREST&&std::numeric_limits<double>::is_iec559&&
        std::numeric_limits<L>::digits>=64&&std::numeric_limits<L>::max_exponent>2*std::numeric_limits<double>::max_exponent&&
        std::numeric_limits<L>::min_exponent<2*(std::numeric_limits<double>::min_exponent-std::numeric_limits<double>::digits);
}
std::optional<Bounds> normalPdfForCalibration(double x){if(!arithmeticAvailable())return std::nullopt;return exported(pdfI(I(x),nullptr));}
std::optional<Bounds> normalCdfForCalibration(double x){if(!arithmeticAvailable())return std::nullopt;return exported(cdfI(I(x),nullptr));}
std::optional<std::array<Bounds,8>> momentsForCalibration(double low,double high,double detectorLow,double detectorHigh,double sigma,double anchor){
    if(!arithmeticAvailable()||!(high>low)||!(detectorHigh>detectorLow)||!(sigma>0))return std::nullopt;
    const auto values=moments(I(low),I(high),I(detectorLow),I(detectorHigh),I(sigma),I(anchor),7,nullptr);
    std::array<Bounds,8> result{};for(unsigned n=0;n<8;++n){const auto value=exported(values[n]);if(!value)return std::nullopt;result[n]=*value;}return result;
}

std::optional<std::array<Bounds,8>> fastMomentsForCalibration(
    double low,double high,double detectorLow,double detectorHigh,double sigma,double anchor){
    if(!arithmeticAvailable()||!(sigma>0)||!(high>=low)||!(detectorHigh>detectorLow))return std::nullopt;
    const auto values=fastMoments(I(low),I(high),I(detectorLow),I(detectorHigh),I(sigma),I(anchor),7,nullptr);
    std::array<Bounds,8> result{};
    for(unsigned n=0;n<8;++n){const auto value=exported(values[n]);if(!value)return std::nullopt;result[n]=*value;}
    return result;
}

struct Bank::Impl {
    Request signature;
    Geometry geometry;
    L left=0,right=0,bottom=0,top=0;
    I pitch,sigma,area;
    double invAreaHiD=0;
    struct Column {
        enum class Mask {Unknown,Sky,Ground,Horizon};
        Mask mask=Mask::Unknown;
        unsigned degree=0;
        double pixelLow=0,pixelHigh=0;
        L anchor=0;
        I xDomain,horizonRange;
        std::vector<std::array<Mid,8>> coefficients;
        std::vector<double> remainder4,remainder8;
        bool uncut0Valid=false,uncut3Valid=false,uncut7Valid=false;
        Mid uncutMass0{};
        std::array<Mid,8> uncutMass3{},uncutMass7{};
        struct PreHoriz {double value=0,absSum=0,errSum=0;};
        std::vector<PreHoriz> uncutHoriz3,uncutHoriz7;
    };
    mutable std::vector<Column> columns;

    enum class Publication { Pending, Ready, Refused };
    struct PublishedDegree {
        std::once_flag once;
        std::atomic<Publication> state{Publication::Pending};
    };
    struct SharedColumn {
        PublishedDegree degree3,degree7;
    };
    std::unique_ptr<SharedColumn[]> sharedColumns;

    bool ensure(unsigned index,unsigned degree,Work* work) const {
        if(!sharedColumns)return buildColumn(index,degree,work);
        auto& shared=sharedColumns[index];
        if(degree==7&&!ensure(index,3,work))return false;
        auto& publication=degree==3?shared.degree3:shared.degree7;
        auto state=publication.state.load(std::memory_order_acquire);
        if(state==Publication::Pending) {
            std::call_once(publication.once,[&] {
                const bool accepted=buildColumn(index,degree,work);
                publication.state.store(accepted?Publication::Ready:Publication::Refused,std::memory_order_release);
            });
            state=publication.state.load(std::memory_order_acquire);
        }
        return state==Publication::Ready;
    }

    bool buildColumn(unsigned index,unsigned degree,Work* work) const {
        auto& column=columns[index];
        if(column.mask==Column::Mask::Sky||column.mask==Column::Mask::Ground)return true;
        if(column.degree>=degree)return true;
        const unsigned jetDegree=degree==3?4u:8u;
        if(column.mask==Column::Mask::Unknown){
            const I xx=tanI(column.xDomain);
            const I low=geometry.level(xx,tanI(I(bottom))),high=geometry.level(xx,tanI(I(top)));
            if(!low.valid()||!high.valid())return false;
            if(low.lo>=geometry.h.hi){column.mask=Column::Mask::Sky;return true;}
            if(high.hi<geometry.h.lo){
                column.mask=Column::Mask::Ground;
                std::array<Mid,8> m0{};
                if(fastMomentsMidUncachedV5(column.xDomain.lo,column.xDomain.hi,column.pixelLow,column.pixelHigh,
                    signature.psfSigmaRad,column.anchor,0,nullptr,m0,nullptr)) {
                    column.uncutMass0=m0[0];column.uncut0Valid=true;
                }
                return true;
            }
            if(!(low.hi<geometry.h.lo&&high.lo>geometry.h.hi))return false;
            column.mask=Column::Mask::Horizon;
        }
        const auto point=geometry.horizon(I(column.anchor),jetDegree);
        const auto range=geometry.horizon(column.xDomain,jetDegree);
        if(!valid(point,jetDegree)||!valid(range,jetDegree)||range[0].lo<bottom||range[0].hi>top)return false;
        // The constant jet term is independent of degree. In the shared bank
        // degree seven adds coefficients 4..7 and remainder8. Readers of the
        // published degree-three prefix may still be using the horizon range.
        if(!sharedColumns||degree==3)column.horizonRange=range[0];
        if(column.coefficients.empty())column.coefficients.resize(signature.height);
        const I width=I(column.pixelHigh)-I(column.pixelLow);
        const I mean=(I(column.pixelLow)+I(column.pixelHigh))/I(2);
        const I offset=mean-I(column.anchor);
        const I mu=weightedEvenMoment(jetDegree,width,sigma,offset);
        auto& errors=degree==3?column.remainder4:column.remainder8;errors.resize(signature.height);
        // Hoist row-independent jet power towers (delta^n / n!) once per column
        // and share adjacent row edges (high of row r == low of row r+1).
        std::array<Jet,maximumDegree+1> pointTerms{},rangeTerms{};
        std::array<I,maximumDegree+1> factorials{};
        {
            Jet pointDelta=point;pointDelta[0]=I(0);
            Jet rangeDelta=range;rangeDelta[0]=I(0);
            Jet pTerm=constant(I(1)),rTerm=constant(I(1));
            I fact(1);
            for(unsigned n=1;n<=jetDegree;++n) {
                pTerm=times(pTerm,pointDelta,jetDegree);
                rTerm=times(rTerm,rangeDelta,jetDegree);
                fact=fact*I(n);
                pointTerms[n]=pTerm;
                rangeTerms[n]=rTerm;
                factorials[n]=fact;
                if(!valid(pTerm,jetDegree)||!valid(rTerm,jetDegree)||!fact.valid())return false;
            }
        }
        std::array<I,maximumDegree+1> sigmaPowers{};
        sigmaPowers[0]=sigma;
        for(unsigned n=1;n<jetDegree;++n)sigmaPowers[n]=sigmaPowers[n-1]*sigma;
        struct EdgeCache {
            I z;
            I prim;
            std::array<I,maximumDegree> dDens{};
        };
        const auto computeEdge=[&](I h0,double yEdge,bool needPrim,EdgeCache& out)->bool {
            out.z=(h0-I(yEdge))/sigma;
            if(!out.z.valid())return false;
            if(needPrim) {
                out.prim=gaussianPrimitive(out.z,work);
                if(!out.prim.valid())return false;
            }
            const I pdf=pdfI(out.z,work);
            if(!pdf.valid())return false;
            I hPrev(1),hCurr=out.z;
            for(unsigned k=0;k<=jetDegree-2;++k) {
                const I hk=(k==0)?I(1):(k==1?out.z:hCurr);
                const I val=hk*pdf;
                out.dDens[k]=(k&1u)?-val:val;
                if(!out.dDens[k].valid())return false;
                if(k>=1&&k<jetDegree-2) {
                    const I hNext=out.z*hCurr-I(k)*hPrev;
                    hPrev=hCurr;hCurr=hNext;
                }
            }
            return true;
        };
        bool hasPrevEdge=false;double prevEdgeY=0;
        EdgeCache pPrev{},pCurr{},rPrev{},rCurr{};
        const unsigned coeffStart=column.degree==0?0u:column.degree+1u;
        for(unsigned row=0;row<signature.height;++row){
            const double low=(row-.5*signature.height)*signature.pixelPitchRad,high=low+signature.pixelPitchRad;
            if(!hasPrevEdge||low!=prevEdgeY) {
                if(!computeEdge(point[0],low,true,pPrev)||!computeEdge(range[0],low,false,rPrev))return false;
            }
            if(!computeEdge(point[0],high,true,pCurr)||!computeEdge(range[0],high,false,rCurr))return false;
            std::array<I,maximumDegree+1> pDeriv{},rDeriv{};
            pDeriv[0]=intersect(sigma*(pPrev.prim-pCurr.prim),0,(I(high)-I(low)).hi);
            pDeriv[1]=cdfDifference(pPrev.z,pCurr.z,work);
            rDeriv[1]=cdfDifference(rPrev.z,rCurr.z,work);
            if(!pDeriv[0].valid()||!pDeriv[1].valid()||!rDeriv[1].valid())return false;
            for(unsigned n=2;n<=jetDegree;++n) {
                pDeriv[n]=(pPrev.dDens[n-2]-pCurr.dDens[n-2])/sigmaPowers[n-2];
                rDeriv[n]=(rPrev.dDens[n-2]-rCurr.dDens[n-2])/sigmaPowers[n-2];
                if(!pDeriv[n].valid()||!rDeriv[n].valid())return false;
            }
            std::array<I,maximumDegree+1> pScaled{},rScaled{};
            for(unsigned n=1;n<=jetDegree;++n) {
                pScaled[n]=pDeriv[n]/factorials[n];
                rScaled[n]=rDeriv[n]/factorials[n];
                if(!pScaled[n].valid()||!rScaled[n].valid())return false;
            }
            // Compute only the newly needed coefficients [coeffStart..degree]
            // (identical operation prefix: term[n][k] == 0 for k < n), plus
            // verify jetDegree validity, and compute bound[jetDegree] directly.
            for(unsigned k=coeffStart;k<=degree;++k) {
                I coeff=(k==0)?pDeriv[0]:I(0);
                for(unsigned n=1;n<=k;++n)coeff=coeff+pointTerms[n][k]*pScaled[n];
                if(!coeff.valid())return false;
                column.coefficients[row][k]=midpoint(coeff);
                if(!std::isfinite(column.coefficients[row][k].error))return false;
            }
            I pLast(0),rLast(0);
            for(unsigned n=1;n<=jetDegree;++n) {
                pLast=pLast+pointTerms[n][jetDegree]*pScaled[n];
                rLast=rLast+rangeTerms[n][jetDegree]*rScaled[n];
            }
            if(!pLast.valid()||!rLast.valid())return false;
            errors[row]=upperPositive((I(magnitude(rLast))*width*mu/area).hi);
            pPrev=pCurr;
            rPrev=rCurr;
            prevEdgeY=high;
            hasPrevEdge=true;
        }
        // Precompute exact uncut-column V5 moments and horizon contractions
        // once per column (safely published under sharedColumns::once).
        if(degree==3) {
            std::array<Mid,8> m0{};
            if(fastMomentsMidUncachedV5(column.xDomain.lo,column.xDomain.hi,column.pixelLow,column.pixelHigh,
                signature.psfSigmaRad,column.anchor,0,nullptr,m0,nullptr)) {
                column.uncutMass0=m0[0];column.uncut0Valid=true;
            }
            if(fastMomentsMidUncachedV5(column.xDomain.lo,column.xDomain.hi,column.pixelLow,column.pixelHigh,
                signature.psfSigmaRad,column.anchor,3,nullptr,column.uncutMass3,nullptr)) {
                column.uncutHoriz3.resize(signature.height);
                for(unsigned row=0;row<signature.height;++row) {
                    const auto& coeffs=column.coefficients[row];
                    double val=0,absS=0,errS=0;
                    for(unsigned n=0;n<=3;++n) {
                        const double cv=coeffs[n].value,ce=coeffs[n].error;
                        const double mv=column.uncutMass3[n].value,me=column.uncutMass3[n].error;
                        const double prod=cv*mv;
                        val+=prod;absS+=std::abs(prod);
                        errS+=std::abs(cv)*me+std::abs(mv)*ce+ce*me;
                    }
                    column.uncutHoriz3[row]={val,absS,errS};
                }
                column.uncut3Valid=true;
            }
        } else if(degree==7) {
            if(fastMomentsMidUncachedV5(column.xDomain.lo,column.xDomain.hi,column.pixelLow,column.pixelHigh,
                signature.psfSigmaRad,column.anchor,7,nullptr,column.uncutMass7,nullptr)) {
                column.uncutHoriz7.resize(signature.height);
                for(unsigned row=0;row<signature.height;++row) {
                    const auto& coeffs=column.coefficients[row];
                    double val=0,absS=0,errS=0;
                    for(unsigned n=0;n<=7;++n) {
                        const double cv=coeffs[n].value,ce=coeffs[n].error;
                        const double mv=column.uncutMass7[n].value,me=column.uncutMass7[n].error;
                        const double prod=cv*mv;
                        val+=prod;absS+=std::abs(prod);
                        errS+=std::abs(cv)*me+std::abs(mv)*ce+ce*me;
                    }
                    column.uncutHoriz7[row]={val,absS,errS};
                }
                column.uncut7Valid=true;
            }
        }
        column.degree=degree;if(work)++work->preparedColumns;return true;
    }

    bool clippedRootsV5(I y,L leftEdge,L rightEdge,std::array<I,4>& roots,unsigned& rootCount) const {
        const auto& g=geometry;const I n=g.a+g.c*y;
        std::array<I,2> candidates{};unsigned candidateCount=0;
        if(exactZero(g.h)) {
            if(g.b.lo<=0&&g.b.hi>=0) {
                const I range=n+g.b*tanIV5(I(leftEdge,rightEdge));
                return range.valid()&&(range.lo>0||range.hi<0);
            }
            candidates[candidateCount++]=-n/g.b;
        } else {
            const I hh=g.h*g.h,aa=g.b*g.b-hh*g.g11;
            const I bb=I(2)*(n*g.b-hh*(g.g01+g.g12*y));
            const I cc=n*n-hh*(g.g00+I(2)*g.g02*y+g.g22*y*y);
            if(!aa.valid()||(aa.lo<=0&&aa.hi>=0))return false;
            const I discriminant=bb*bb-I(4)*aa*cc;
            if(!discriminant.valid())return false;
            if(discriminant.hi<0)return true;
            if(discriminant.lo<=0)return false; // Uncertain tangent: native fallback.
            const I radical=sqrtI(discriminant);
            if(!radical.valid())return false;
            const I denom=I(2)*aa;
            candidates[candidateCount++]=(-bb-radical)/denom;
            candidates[candidateCount++]=(-bb+radical)/denom;
        }
        for(unsigned idx=0;idx<candidateCount;++idx) {
            const I candidate=candidates[idx];
            const I angle=atanIV5(candidate);
            if(!angle.valid())return false;
            if(angle.hi<=leftEdge||angle.lo>=rightEdge)continue;
            const I numerator=n+g.b*candidate;
            if(!exactZero(g.h)) {
                if((g.h.lo>0&&numerator.hi<0)||(g.h.hi<0&&numerator.lo>0))continue;
                if(!((g.h.lo>0&&numerator.lo>0)||(g.h.hi<0&&numerator.hi<0)))return false;
            }
            if(rootCount>=4)return false;
            roots[rootCount++]=intersect(angle,leftEdge,rightEdge);
        }
        return true;
    }

    bool sumV5(unsigned row,unsigned col,unsigned degree,
        const std::array<Mid,8>& mass,Mid lower,double& outField,double& outError) const noexcept {
        const auto& column=columns[col];
        const auto& coeffs=column.coefficients[row];
        double numerator=0,absolute=0,error=0;
        {
            const double c0v=coeffs[0].value-lower.value;
            const double c0e=(coeffs[0].error+lower.error+
                depsilon*(std::abs(coeffs[0].value)+std::abs(lower.value))+dtiny)*(1.0+32.0*depsilon);
            const double prod=c0v*mass[0].value;
            numerator+=prod;
            absolute+=std::abs(prod);
            error+=std::abs(c0v)*mass[0].error+std::abs(mass[0].value)*c0e+c0e*mass[0].error;
        }
        for(unsigned n=1;n<=degree;++n) {
            const double cv=coeffs[n].value,ce=coeffs[n].error;
            const double mv=mass[n].value,me=mass[n].error;
            const double prod=cv*mv;
            numerator+=prod;
            absolute+=std::abs(prod);
            error+=std::abs(cv)*me+std::abs(mv)*ce+ce*me;
        }
        const double operations=2.0*static_cast<double>(degree+1)+4.0;
        const double gamma=operations*depsilon/(1.0-operations*depsilon);
        error+=gamma*absolute+operations*dtiny;
        const double normalizer=signature.pixelPitchRad*signature.pixelPitchRad;
        const double value=numerator/normalizer;
        const double totalErr=error*invAreaHiD+depsilon*std::abs(value)+dtiny+
            (degree==3?column.remainder4[row]:column.remainder8[row])+extraTailD;
        const double clamped=std::clamp(value,0.0,1.0);
        const double bound=degree==3?upperPositiveD<256>(totalErr+std::abs(value-clamped)):
            upperPositiveD<384>(totalErr+std::abs(value-clamped));
        if(!std::isfinite(value)||!std::isfinite(bound)||bound>fieldBudget)return false;
        outField=clamped;
        outError=bound;
        return true;
    }

    bool clippedFastColumnV5(unsigned col,L x0,L x1,L bottomEdge,L topEdge,I tanBottom,I tanTop,
        const std::array<I,4>& strips,unsigned stripCount,
        const unsigned* activeRows,unsigned activeRowCount,
        const Mid* lower,const Mid* height,
        double* colField,double* colError,Work* work) const {
        struct Piece {I domain;bool ground=false;};
        std::array<Piece,5> pieces{};unsigned pieceCount=0;
        const auto append=[&](L a,L b) {
            if(!(b>a))return true;
            const I x=tanPointV5(.5L*a+.5L*b);
            const I below=geometry.level(x,tanBottom),above=geometry.level(x,tanTop);
            if(!below.valid()||!above.valid())return false;
            // All possible boundary roots are enclosed in strips. A strict
            // midpoint sign therefore certifies the whole remaining piece.
            if(above.hi<geometry.h.lo)pieces[pieceCount++]={I(a,b),true};
            else if(below.lo>geometry.h.hi){} // Entire piece is sky.
            else if(below.hi<geometry.h.lo&&above.lo>geometry.h.hi)pieces[pieceCount++]={I(a,b),false};
            else return false;
            return true;
        };
        L next=x0;
        for(unsigned idx=0;idx<stripCount;++idx) {if(!append(next,strips[idx].lo))return false;next=strips[idx].hi;}
        if(!append(next,x1))return false;
        const auto& column=columns[col];
        Ball groundBall(0.0);
        bool horizon=false;
        for(unsigned idx=0;idx<pieceCount;++idx)if(pieces[idx].ground) {
            std::array<Mid,8> mass{};
            if(!fastMomentsMidV5(pieces[idx].domain.lo,pieces[idx].domain.hi,column.pixelLow,column.pixelHigh,signature.psfSigmaRad,column.anchor,0,nullptr,mass,work))
                return false;
            groundBall=groundBall+Ball(mass[0].value,mass[0].error);
        } else horizon=true;
        std::array<Ball,128> stripFields{};
        const Ball areaBall=ball(area);
        for(unsigned idx=0;idx<stripCount;++idx) {
            const I strip=strips[idx];
            const I root=geometry.horizon0V5(strip);
            if(!root.valid())return false;
            const L low=std::clamp(root.lo,bottomEdge,topEdge),high=std::clamp(root.hi,bottomEdge,topEdge);
            const L at=.5L*low+.5L*high;
            const I distance=I(std::max(magnitude(I(at)-I(low)),magnitude(I(high)-I(at))));
            const I wedge=(I(strip.hi)-I(strip.lo))*distance/area;
            if(!wedge.valid())return false;
            std::array<Mid,8> mass{};
            if(!fastMomentsMidV5(strip.lo,strip.hi,column.pixelLow,column.pixelHigh,signature.psfSigmaRad,column.anchor,0,nullptr,mass,work))
                return false;
            const Ball massBall(mass[0].value,mass[0].error);
            const double wedgeHi=nextUpPositiveD(static_cast<double>(wedge.hi));
            const Ball atBall=ball(I(at));
            const double sigmaD=signature.psfSigmaRad;
            bool hasPrev=false;double prevHigh=0;Ball prevPrim;
            for(unsigned r=0;r<activeRowCount;++r) {
                const unsigned row=activeRows[r];
                const double rowLow=(row-.5*signature.height)*signature.pixelPitchRad,rowHigh=rowLow+signature.pixelPitchRad;
                const Ball pa=(hasPrev&&rowLow==prevHigh)?prevPrim:fastGaussianPrimitive(divPosExact(atBall-Ball(rowLow),sigmaD),work);
                const Ball pb=fastGaussianPrimitive(divPosExact(atBall-Ball(rowHigh),sigmaD),work);
                hasPrev=true;prevHigh=rowHigh;prevPrim=pb;
                const Mid vertMid=clampBallToMid(mulPosExact(pa-pb,sigmaD),0.0,signature.pixelPitchRad);
                if(!std::isfinite(vertMid.error))return false;
                Ball value=massBall*(Ball(vertMid.value,vertMid.error)-Ball(lower[row].value,lower[row].error))/areaBall;
                value.radius=radiusUpper(value.radius+wedgeHi);
                stripFields[row]=stripFields[row]+value;
            }
        }
        const auto evaluate=[&](unsigned degree)->bool {
            std::array<Ball,8> accum{};
            if(horizon)for(unsigned idx=0;idx<pieceCount;++idx)if(!pieces[idx].ground) {
                std::array<Mid,8> mass{};
                if(!fastMomentsMidV5(pieces[idx].domain.lo,pieces[idx].domain.hi,column.pixelLow,column.pixelHigh,signature.psfSigmaRad,column.anchor,degree,nullptr,mass,work))
                    return false;
                for(unsigned n=0;n<=degree;++n)accum[n]=accum[n]+Ball(mass[n].value,mass[n].error);
            }
            std::array<Mid,8> stored{};
            if(horizon)for(unsigned n=0;n<=degree;++n){
                if(!accum[n].valid())return false;
                stored[n]={accum[n].value,nextUpPositiveD(accum[n].radius)};
            }
            for(unsigned r=0;r<activeRowCount;++r) {
                const unsigned row=activeRows[r];
                Ball value=groundBall*Ball(height[row].value,height[row].error)/areaBall+stripFields[row];
                if(horizon) {
                    double partVal=0,partErr=0;
                    if(!sumV5(row,col,degree,stored,lower[row],partVal,partErr))return false;
                    value=value+Ball(partVal,partErr);
                }
                if(!value.valid())return false;
                const double clamped=std::clamp(value.value,0.0,1.0);
                const double bound=upperPositiveD<128>(radiusUpper(value.radius+extraTailD)+std::abs(value.value-clamped));
                if(!std::isfinite(bound)||bound>fieldBudget)return false;
                colField[row]=clamped;
                colError[row]=bound;
            }
            return true;
        };
        if(evaluate(3))return true;
        if(horizon&&!ensure(col,7,work))return false;
        return evaluate(7);
    }

    template<bool fastPadding=false>
    std::optional<std::pair<double,double>> finish(double value,L error) const {
        if(!std::isfinite(value)||!(error>=0)||!std::isfinite(error))return std::nullopt;
        const double clamped=std::clamp(value,0.,1.);
        error+=std::abs(static_cast<L>(value)-clamped);
        const double bound=upperPositive<fastPadding>(error,64);
        if(!std::isfinite(bound)||bound>fieldBudget)return std::nullopt;
        return std::make_pair(clamped,bound);
    }

    template<bool fastPadding=false>
    std::optional<std::pair<double,double>> sum(unsigned row,unsigned col,unsigned degree,
        const std::array<I,8>& integral,Mid lower,const std::array<Mid,8>* stored=nullptr) const {
        const auto& column=columns[col];
        std::array<Mid,8> mass{};
        if(stored)mass=*stored;
        else for(unsigned n=0;n<=degree;++n){mass[n]=midpoint<fastPadding>(integral[n]);if(!std::isfinite(mass[n].error))return std::nullopt;}
        double numerator=0;L absolute=0,error=0;
        for(unsigned n=0;n<=degree;++n){
            auto coefficient=column.coefficients[row][n];
            if(n==0){
                const double difference=coefficient.value-lower.value;
                coefficient.error=upperPositive<fastPadding>(static_cast<L>(coefficient.error)+lower.error+
                    depsilon*(std::abs(static_cast<L>(coefficient.value))+std::abs(static_cast<L>(lower.value)))+dtiny);
                coefficient.value=difference;
            }
            numerator+=coefficient.value*mass[n].value;
            absolute+=std::abs(static_cast<L>(coefficient.value)*mass[n].value);
            error+=std::abs(static_cast<L>(coefficient.value))*mass[n].error+
                std::abs(static_cast<L>(mass[n].value))*coefficient.error+
                static_cast<L>(coefficient.error)*mass[n].error;
        }
        const L operations=2*static_cast<L>(degree+1)+4,gamma=operations*depsilon/(1-operations*depsilon);
        error+=gamma*absolute+operations*dtiny;
        const double normalizer=signature.pixelPitchRad*signature.pixelPitchRad;
        const double value=numerator/normalizer;
        error=error/area.lo+depsilon*std::abs(static_cast<L>(value))+dtiny+
            (degree==3?column.remainder4[row]:column.remainder8[row])+extraTail;
        return finish<fastPadding>(value,upperPositive<fastPadding>(error,32*(degree+1)+64));
    }

    bool evaluateFastCoreV5(const std::array<double,4>& region,
        double* fieldOut,double* errorOut,double& maxErrorOut,
        unsigned* activeRowsOut,unsigned& activeRowCountOut,
        unsigned& colBeginOut,unsigned& colEndOut,Work* work) const {
        if(work)*work={};
        const auto refuse=[&](Refusal reason)->bool{if(work)work->refusal=reason;return false;};
        if(!arithmeticAvailable())return refuse(Refusal::ArithmeticEnvironment);
        if(!std::all_of(region.begin(),region.end(),[](double x){return std::isfinite(x)&&std::abs(x)<=1.55;})||
            !(region[1]>region[0])||!(region[3]>region[2]))return refuse(Refusal::Input);
        const auto& p=signature;
        const L regLeft=std::max(left,static_cast<L>(region[0])),regRight=std::min(right,static_cast<L>(region[1]));
        const L regBottom=std::max(bottom,static_cast<L>(region[2])),regTop=std::min(top,static_cast<L>(region[3]));
        const double defaultError=upperPositiveD<32>(extraTailD);
        maxErrorOut=defaultError;
        activeRowCountOut=0;
        colBeginOut=p.width;colEndOut=0;
        if(!(regRight>regLeft&&regTop>regBottom))return true;
        std::array<Mid,128> lower{},height{};
        std::array<unsigned,128> activeRows{};unsigned activeRowCount=0;
        std::array<double,128> omitted{};
        for(unsigned row=0;row<p.height;++row)omitted[row]=-1.0;
        GaussianCacheV5* rowCache=activeGaussianCacheV5;
        GaussianKeyV5 rowKey;rowKey.kind=8;
        double storedBottom=0,storedTop=0;
        if(rowCache) {
            if(!samePointV5(I(regBottom),storedBottom)||!samePointV5(I(regTop),storedTop))rowCache=nullptr;
            else {rowKey.bits[0]=gaussianBitsV5(storedBottom);rowKey.bits[1]=gaussianBitsV5(storedTop);}
        }
        const auto saved=rowCache?rowCache->get(rowKey):GaussianCacheV5::Entry{};
        if(saved.rows) {
            for(unsigned row=0;row<p.height;++row){lower[row]=saved.rows->lower[row];height[row]=saved.rows->height[row];omitted[row]=saved.rows->omitted[row];}
            activeRowCount=static_cast<unsigned>(saved.rows->active.size());
            for(unsigned r=0;r<activeRowCount;++r)activeRows[r]=saved.rows->active[r];
        } else {
            const double sigmaD=p.psfSigmaRad,pitchD=p.pixelPitchRad;
            const Ball bottomBall=ball(I(regBottom)),topBall=ball(I(regTop));
            const double q12Hi=fromBits(table::q12.high);
            const double gap12=12.0*sigmaD*(1.0+16.0*depsilon);
            const double regSpanHi=nextUpPositiveD(static_cast<double>(up(regTop-regBottom)));
            const double tail12Hi=(regSpanHi*q12Hi/pitchD)*(1.0+32.0*depsilon);
            const bool omitAt12=std::isfinite(tail12Hi)&&tail12Hi<=5e-19;
            const double omitted12Err=omitAt12?upperPositiveD<32>(tail12Hi+extraTailD):-1.0;
            if(omitAt12&&(!std::isfinite(omitted12Err)||omitted12Err>fieldBudget))return refuse(Refusal::ErrorBudget);
            const double regBottomD=static_cast<double>(regBottom),regTopD=static_cast<double>(regTop);
            bool hasPrev=false;double prevHigh=0;Ball prevBotPrim,prevTopPrim;
            for(unsigned row=0;row<p.height;++row){
                const double low=(row-.5*p.height)*pitchD,high=low+pitchD;
                if(omitAt12&&(low-regTopD>=gap12||regBottomD-high>=gap12)) {
                    omitted[row]=omitted12Err;
                    hasPrev=false;
                    continue;
                }
                I gap;
                if(regTop<low)gap=I(low)-I(regTop);
                else if(regBottom>high)gap=I(regBottom)-I(high);
                if(gap.lo>0) {
                    const Ball dist=divPosExact(ball(I(gap.lo)),sigmaD);
                    const Ball qTail=fastNormalAtMagnitude(dist,true,work);
                    const double qHi=qTail.value+qTail.radius;
                    const double tailHi=(regSpanHi*qHi/pitchD)*(1.0+32.0*depsilon);
                    if(qTail.valid()&&std::isfinite(tailHi)&&tailHi<=5e-19) {
                        const double error=upperPositiveD<32>(tailHi+extraTailD);
                        if(!std::isfinite(error)||error>fieldBudget)return refuse(Refusal::ErrorBudget);
                        omitted[row]=error;
                        hasPrev=false;
                        continue;
                    }
                }
                activeRows[activeRowCount++]=row;
                const Ball botA=(hasPrev&&row==activeRows[activeRowCount-2]+1&&low==prevHigh)
                    ?prevBotPrim:fastGaussianPrimitive(divPosExact(bottomBall-Ball(low),sigmaD),work);
                const Ball botB=fastGaussianPrimitive(divPosExact(bottomBall-Ball(high),sigmaD),work);
                const Ball topA=(hasPrev&&row==activeRows[activeRowCount-2]+1&&low==prevHigh)
                    ?prevTopPrim:fastGaussianPrimitive(divPosExact(topBall-Ball(low),sigmaD),work);
                const Ball topB=fastGaussianPrimitive(divPosExact(topBall-Ball(high),sigmaD),work);
                hasPrev=true;prevHigh=high;prevBotPrim=botB;prevTopPrim=topB;
                const Ball aBall=mulPosExact(botA-botB,sigmaD);
                const Ball bBall=mulPosExact(topA-topB,sigmaD);
                lower[row]=clampBallToMid(aBall,0.0,pitchD);
                height[row]=clampBallToMid(bBall-aBall,0.0,pitchD);
                if(!std::isfinite(lower[row].error)||!std::isfinite(height[row].error))return refuse(Refusal::Nonfinite);
            }
            if(rowCache) {
                GaussianRowsV5 cachedRows;
                cachedRows.lower.assign(lower.begin(),lower.begin()+p.height);
                cachedRows.height.assign(height.begin(),height.begin()+p.height);
                cachedRows.active.assign(activeRows.begin(),activeRows.begin()+activeRowCount);
                cachedRows.omitted.assign(omitted.begin(),omitted.begin()+p.height);
                GaussianCacheV5::Entry value;
                value.charge=sizeof(value)+sizeof(rowKey)+sizeof(GaussianRowsV5)+256+
                    2*p.height*sizeof(Mid)+cachedRows.active.size()*sizeof(unsigned)+p.height*sizeof(double);
                value.rows=std::make_shared<const GaussianRowsV5>(std::move(cachedRows));
                rowCache->put(rowKey,std::move(value));
            }
        }
        for(unsigned row=0;row<p.height;++row)if(omitted[row]>=0) {
            if(omitted[row]>maxErrorOut)maxErrorOut=omitted[row];
            if(errorOut) {
                double* rowErr=errorOut+static_cast<std::size_t>(row)*p.width;
                for(unsigned col=0;col<p.width;++col)rowErr[col]=omitted[row];
            }
        }
        if(activeRowsOut) {
            for(unsigned r=0;r<activeRowCount;++r)activeRowsOut[r]=activeRows[r];
        }
        activeRowCountOut=activeRowCount;
        if(activeRowCount==0)return true;

        bool edgesPrepared=false,edgesValid=false;
        I tanBottom,tanTop;
        std::array<I,4> regionStrips{};unsigned regionStripCount=0;
        const auto prepareEdges=[&]() {
            if(edgesPrepared)return;
            edgesPrepared=true;
            tanBottom=tanPointV5(regBottom);tanTop=tanPointV5(regTop);
            if(!tanBottom.valid()||!tanTop.valid())return;
            std::array<I,4> roots{};unsigned rootCount=0;
            if(!clippedRootsV5(tanBottom,regLeft,regRight,roots,rootCount)||
               !clippedRootsV5(tanTop,regLeft,regRight,roots,rootCount))return;
            for(unsigned i=1;i<rootCount;++i)for(unsigned j=i;j>0&&roots[j].lo<roots[j-1].lo;--j)std::swap(roots[j],roots[j-1]);
            regionStripCount=0;
            for(unsigned idx=0;idx<rootCount;++idx) {
                if(regionStripCount>0&&roots[idx].lo<=regionStrips[regionStripCount-1].hi)
                    regionStrips[regionStripCount-1].hi=std::max(regionStrips[regionStripCount-1].hi,roots[idx].hi);
                else regionStrips[regionStripCount++]=roots[idx];
            }
            edgesValid=true;
        };

        const double normalizer=p.pixelPitchRad*p.pixelPitchRad;
        std::array<double,128> colField{},colError{};
        ColumnEdgeCacheV5 edgeCache;
        for(unsigned col=0;col<p.width;++col){
            auto& column=columns[col];
            const L x0=std::max(regLeft,column.xDomain.lo),x1=std::min(regRight,column.xDomain.hi);
            if(!(x1>x0))continue;
            if(!ensure(col,3,work))return refuse(Refusal::Root);
            auto mask=column.mask;
            if(mask==Column::Mask::Horizon){
                const I range=column.horizonRange;
                if(regTop<=range.lo)mask=Column::Mask::Ground;
                else if(regBottom>=range.hi)mask=Column::Mask::Sky;
                else if(!(regBottom<range.lo&&regTop>range.hi)) {
                    prepareEdges();
                    bool resolvedTopology=false;
                    std::array<I,4> colStrips{};unsigned colStripCount=0;
                    if(edgesValid) {
                        for(unsigned idx=0;idx<regionStripCount;++idx) {
                            if(regionStrips[idx].hi>x0&&regionStrips[idx].lo<x1)
                                colStrips[colStripCount++]=intersect(regionStrips[idx],x0,x1);
                        }
                        if(colStripCount==0) {
                            const I xm=tanPointV5(.5L*x0+.5L*x1);
                            const I below=geometry.level(xm,tanBottom),above=geometry.level(xm,tanTop);
                            if(below.valid()&&above.valid()) {
                                if(above.hi<geometry.h.lo){mask=Column::Mask::Ground;resolvedTopology=true;}
                                else if(below.lo>geometry.h.hi){mask=Column::Mask::Sky;resolvedTopology=true;}
                                else if(below.hi<geometry.h.lo&&above.lo>geometry.h.hi){resolvedTopology=true;}
                            }
                        }
                    }
                    if(!resolvedTopology&&colStripCount==0) {
                        const I local=geometry.horizon0V5(I(x0,x1));
                        if(!local.valid())return refuse(Refusal::Root);
                        if(regTop<=local.lo){mask=Column::Mask::Ground;resolvedTopology=true;}
                        else if(regBottom>=local.hi){mask=Column::Mask::Sky;resolvedTopology=true;}
                        else if(regBottom<local.lo&&regTop>local.hi){resolvedTopology=true;}
                        else if(!edgesValid) {
                            if(!tanBottom.valid()||!tanTop.valid())return refuse(Refusal::Topology);
                            std::array<I,4> roots{};unsigned rootCount=0;
                            if(!clippedRootsV5(tanBottom,x0,x1,roots,rootCount)||
                               !clippedRootsV5(tanTop,x0,x1,roots,rootCount))return refuse(Refusal::Topology);
                            for(unsigned i=1;i<rootCount;++i)for(unsigned j=i;j>0&&roots[j].lo<roots[j-1].lo;--j)std::swap(roots[j],roots[j-1]);
                            for(unsigned idx=0;idx<rootCount;++idx) {
                                if(colStripCount>0&&roots[idx].lo<=colStrips[colStripCount-1].hi)
                                    colStrips[colStripCount-1].hi=std::max(colStrips[colStripCount-1].hi,roots[idx].hi);
                                else colStrips[colStripCount++]=roots[idx];
                            }
                        }
                    }
                    if(!resolvedTopology) {
                        if(!clippedFastColumnV5(col,x0,x1,regBottom,regTop,tanBottom,tanTop,colStrips,colStripCount,
                            activeRows.data(),activeRowCount,lower.data(),height.data(),colField.data(),colError.data(),work))
                            return refuse(Refusal::Topology);
                        if(col<colBeginOut)colBeginOut=col;
                        if(col+1>colEndOut)colEndOut=col+1;
                        for(unsigned r=0;r<activeRowCount;++r) {
                            const unsigned row=activeRows[r];
                            const auto at=static_cast<std::size_t>(row)*p.width+col;
                            fieldOut[at]=colField[row];
                            if(errorOut)errorOut[at]=colError[row];
                            if(colError[row]>maxErrorOut)maxErrorOut=colError[row];
                        }
                        continue;
                    }
                }
            }
            if(mask==Column::Mask::Sky){if(work)++work->analyticColumns;continue;}
            const bool uncutColumn=!activeGaussianCacheV5&&!work&&x0==column.xDomain.lo&&x1==column.xDomain.hi;
            if(mask==Column::Mask::Ground){
                Mid horizontal{};
                if(uncutColumn&&column.uncut0Valid) {
                    horizontal=column.uncutMass0;
                } else {
                    std::array<Mid,8> integral{};
                    if(!fastMomentsMidV5(x0,x1,column.pixelLow,column.pixelHigh,p.psfSigmaRad,column.anchor,0,&edgeCache,integral,work))
                        return refuse(Refusal::Moment);
                    horizontal=integral[0];
                }
                if(col<colBeginOut)colBeginOut=col;
                if(col+1>colEndOut)colEndOut=col+1;
                const double hv=horizontal.value,he=horizontal.error,absHv=std::abs(hv);
                for(unsigned r=0;r<activeRowCount;++r){
                    const unsigned row=activeRows[r];
                    const double yv=height[row].value,ye=height[row].error;
                    const double value=hv*yv/normalizer;
                    const double err=(absHv*ye+std::abs(yv)*he+he*ye+
                        4.0*depsilon*std::abs(hv*yv)+4.0*dtiny)*invAreaHiD+
                        extraTailD+depsilon*std::abs(value);
                    const double clamped=std::clamp(value,0.0,1.0);
                    const double bound=upperPositiveD<64>(err+std::abs(value-clamped));
                    if(!std::isfinite(value)||!std::isfinite(bound)||bound>fieldBudget)return refuse(Refusal::ErrorBudget);
                    const auto at=static_cast<std::size_t>(row)*p.width+col;
                    fieldOut[at]=clamped;
                    if(errorOut)errorOut[at]=bound;
                    if(bound>maxErrorOut)maxErrorOut=bound;
                }
                if(work)++work->analyticColumns;
                continue;
            }
            const auto evaluateDegree=[&](unsigned degree)->bool {
                if(uncutColumn&&(degree==3?column.uncut3Valid:column.uncut7Valid)) {
                    const auto& preVec=degree==3?column.uncutHoriz3:column.uncutHoriz7;
                    const auto& remVec=degree==3?column.remainder4:column.remainder8;
                    const Mid m0=degree==3?column.uncutMass3[0]:column.uncutMass7[0];
                    const double m0v=m0.value,m0e=m0.error,absM0=std::abs(m0v);
                    const double operations=2.0*static_cast<double>(degree+2)+4.0;
                    const double gamma=operations*depsilon/(1.0-operations*depsilon);
                    for(unsigned r=0;r<activeRowCount;++r) {
                        const unsigned row=activeRows[r];
                        const auto& pre=preVec[row];
                        const double lv=lower[row].value,le=lower[row].error;
                        const double subProd=lv*m0v;
                        const double numerator=pre.value-subProd;
                        const double absolute=pre.absSum+std::abs(subProd);
                        const double error=pre.errSum+std::abs(lv)*m0e+absM0*le+le*m0e+gamma*absolute+operations*dtiny;
                        const double value=numerator/normalizer;
                        const double totalErr=error*invAreaHiD+depsilon*std::abs(value)+dtiny+remVec[row]+extraTailD;
                        const double clamped=std::clamp(value,0.0,1.0);
                        const double bound=degree==3?upperPositiveD<256>(totalErr+std::abs(value-clamped)):
                            upperPositiveD<384>(totalErr+std::abs(value-clamped));
                        if(!std::isfinite(value)||!std::isfinite(bound)||bound>fieldBudget)return false;
                        colField[row]=clamped;colError[row]=bound;
                    }
                } else {
                    std::array<Mid,8> stored{};
                    if(!fastMomentsMidV5(x0,x1,column.pixelLow,column.pixelHigh,p.psfSigmaRad,column.anchor,degree,&edgeCache,stored,work))
                        return false;
                    for(unsigned r=0;r<activeRowCount;++r){
                        const unsigned row=activeRows[r];
                        double val=0,err=0;
                        if(!sumV5(row,col,degree,stored,lower[row],val,err))return false;
                        colField[row]=val;colError[row]=err;
                    }
                }
                if(col<colBeginOut)colBeginOut=col;
                if(col+1>colEndOut)colEndOut=col+1;
                for(unsigned r=0;r<activeRowCount;++r){
                    const unsigned row=activeRows[r];
                    const auto at=static_cast<std::size_t>(row)*p.width+col;
                    fieldOut[at]=colField[row];
                    if(errorOut)errorOut[at]=colError[row];
                    if(colError[row]>maxErrorOut)maxErrorOut=colError[row];
                }
                return true;
            };
            if(evaluateDegree(3)){if(work)++work->degree3Columns;continue;}
            if(!ensure(col,7,work)||!evaluateDegree(7))return refuse(Refusal::ErrorBudget);
            if(work)++work->degree7Columns;
        }
        return true;
    }
};

Bank::Bank(std::unique_ptr<Impl> implementation):implementation_(std::move(implementation)){}
Bank::~Bank()=default;
Bank::Bank(Bank&&) noexcept=default;
Bank& Bank::operator=(Bank&&) noexcept=default;

std::unique_ptr<Bank> Bank::prepareSharedV5(const Request& request,Work* work) {
    auto bank=prepare(request,work);
    if(bank)bank->implementation_->sharedColumns=std::make_unique<Impl::SharedColumn[]>(request.width);
    return bank;
}

std::unique_ptr<Bank> Bank::prepare(const Request& request,Work* work){
    if(work)*work={};
    const auto refuse=[&](Refusal reason)->std::unique_ptr<Bank>{if(work)work->refusal=reason;return nullptr;};
    if(!arithmeticAvailable())return refuse(Refusal::ArithmeticEnvironment);
    const double pitch=request.pixelPitchRad,sigma=request.psfSigmaRad;
    if(request.width<8||request.width>128||request.height<8||request.height>128||
        !std::isfinite(pitch)||pitch<1e-7||pitch*request.width>=1.4||pitch*request.height>=1.4||
        !std::isfinite(sigma)||sigma/pitch<.125||sigma/pitch>8||
        !std::isfinite(request.cutoff)||std::abs(request.cutoff)>1||
        !std::all_of(request.pointing.begin(),request.pointing.end(),[](double x){return std::isfinite(x);})||
        !std::all_of(request.up.begin(),request.up.end(),[](double x){return std::isfinite(x);}))return refuse(Refusal::Input);
    using V=weapon::V3;
    const V input{request.pointing[0],request.pointing[1],request.pointing[2]},inputUp{request.up[0],request.up[1],request.up[2]};
    if(!(measured::norm(input)>0)||std::abs(measured::norm(inputUp)-1)>1e-6)return refuse(Refusal::Input);
    const auto forward=measured::unit(input),upVector=measured::unit(inputUp);
    auto right=measured::unit(measured::cross(forward,upVector));
    if(measured::norm(right)<.5)right=measured::unit(measured::cross(forward,std::abs(forward.z)<.9?V{0,0,1}:V{0,1,0}));
    const auto vertical=measured::unit(measured::cross(right,forward));
    auto implementation=std::make_unique<Impl>();implementation->signature=request;
    auto& g=implementation->geometry;
    g.f={{I(forward.x),I(forward.y),I(forward.z)}};g.r={{I(right.x),I(right.y),I(right.z)}};
    g.v={{I(vertical.x),I(vertical.y),I(vertical.z)}};g.u={{I(upVector.x),I(upVector.y),I(upVector.z)}};
    const auto inner=[](const auto& a,const auto& b){I sum;for(unsigned n=0;n<3;++n)sum=sum+a[n]*b[n];return sum;};
    g.a=inner(g.f,g.u);g.b=inner(g.r,g.u);g.c=inner(g.v,g.u);g.h=I(request.cutoff);
    g.g00=inner(g.f,g.f);g.g01=inner(g.f,g.r);g.g02=inner(g.f,g.v);
    g.g11=inner(g.r,g.r);g.g12=inner(g.r,g.v);g.g22=inner(g.v,g.v);
    const double margin=9*sigma;
    implementation->left=std::max(-1.55,-.5*request.width*pitch-margin);implementation->right=std::min(1.55,.5*request.width*pitch+margin);
    implementation->bottom=std::max(-1.55,-.5*request.height*pitch-margin);implementation->top=std::min(1.55,.5*request.height*pitch+margin);
    implementation->pitch=I(pitch);implementation->sigma=I(sigma);implementation->area=I(pitch*pitch);
    implementation->invAreaHiD=nextUpPositiveD(static_cast<double>(up(1.L/implementation->area.lo)));
    if(!g.monotone(tanI({implementation->left,implementation->right}),tanI({implementation->bottom,implementation->top})))return refuse(Refusal::Cone);
    implementation->columns.resize(request.width);
    for(unsigned col=0;col<request.width;++col){auto& column=implementation->columns[col];
        column.pixelLow=(col-.5*request.width)*pitch;column.pixelHigh=column.pixelLow+pitch;
        column.anchor=.5L*column.pixelLow+.5L*column.pixelHigh;
        column.xDomain={std::max(implementation->left,static_cast<L>(column.pixelLow-margin)),std::min(implementation->right,static_cast<L>(column.pixelHigh+margin))};
    }
    return std::unique_ptr<Bank>(new Bank(std::move(implementation)));
}

std::optional<Result> Bank::evaluate(const std::array<double,4>& region,Work* work) const {
    if(work)*work={};
    const auto refuse=[&](Refusal reason)->std::optional<Result>{if(work)work->refusal=reason;return std::nullopt;};
    if(!arithmeticAvailable())return refuse(Refusal::ArithmeticEnvironment);
    if(!std::all_of(region.begin(),region.end(),[](double x){return std::isfinite(x)&&std::abs(x)<=1.55;})||
        !(region[1]>region[0])||!(region[3]>region[2]))return refuse(Refusal::Input);
    const auto& state=*implementation_;const auto& p=state.signature;
    const L left=std::max(state.left,static_cast<L>(region[0])),right=std::min(state.right,static_cast<L>(region[1]));
    const L bottom=std::max(state.bottom,static_cast<L>(region[2])),top=std::min(state.top,static_cast<L>(region[3]));
    Result result;const auto cells=static_cast<std::size_t>(p.width)*p.height;
    result.field.assign(cells,0);result.absoluteError.assign(cells,upperPositive(extraTail));
    if(!(right>left&&top>bottom))return result;
    std::vector<Mid> lower(p.height),height(p.height);
    for(unsigned row=0;row<p.height;++row){const double low=(row-.5*p.height)*p.pixelPitchRad,high=low+p.pixelPitchRad;
        const I a=gaussianCumulative(I(bottom),I(low),I(high),state.sigma,work);
        const I b=gaussianCumulative(I(top),I(low),I(high),state.sigma,work);
        lower[row]=midpoint(a);height[row]=midpoint(intersect(b-a,0,(I(high)-I(low)).hi));
        if(!std::isfinite(lower[row].error)||!std::isfinite(height[row].error))return refuse(Refusal::Nonfinite);
    }
    for(unsigned col=0;col<p.width;++col){
        auto& column=state.columns[col];
        const L x0=std::max(left,column.xDomain.lo),x1=std::min(right,column.xDomain.hi);
        if(!(x1>x0))continue;
        if(!state.ensure(col,3,work))return refuse(Refusal::Root);
        auto mask=column.mask;
        if(mask==Impl::Column::Mask::Horizon){
            if(top<=column.horizonRange.lo)mask=Impl::Column::Mask::Ground;
            else if(bottom>=column.horizonRange.hi)mask=Impl::Column::Mask::Sky;
            else if(!(bottom<column.horizonRange.lo&&top>column.horizonRange.hi))return refuse(Refusal::Topology);
        }
        if(mask==Impl::Column::Mask::Sky){if(work)++work->analyticColumns;continue;}
        if(mask==Impl::Column::Mask::Ground){
            const auto integral=moments(I(x0),I(x1),I(column.pixelLow),I(column.pixelHigh),state.sigma,I(column.anchor),0,work);
            const Mid horizontal=midpoint(integral[0]);if(!std::isfinite(horizontal.error))return refuse(Refusal::Moment);
            for(unsigned row=0;row<p.height;++row){
                const double value=horizontal.value*height[row].value/(p.pixelPitchRad*p.pixelPitchRad);
                const L error=(std::abs(static_cast<L>(horizontal.value))*height[row].error+
                    std::abs(static_cast<L>(height[row].value))*horizontal.error+
                    static_cast<L>(horizontal.error)*height[row].error+
                    4*depsilon*std::abs(static_cast<L>(horizontal.value)*height[row].value)+4*dtiny)/state.area.lo+
                    extraTail+depsilon*std::abs(static_cast<L>(value));
                const auto final=state.finish(value,error);if(!final)return refuse(Refusal::ErrorBudget);
                const auto at=static_cast<std::size_t>(row)*p.width+col;result.field[at]=final->first;result.absoluteError[at]=final->second;
            }
            if(work)++work->analyticColumns;
            continue;
        }
        const auto evaluateDegree=[&](unsigned degree)->bool {
            const auto integral=moments(I(x0),I(x1),I(column.pixelLow),I(column.pixelHigh),state.sigma,I(column.anchor),degree,work);
            for(unsigned n=0;n<=degree;++n)if(!integral[n].valid())return false;
            for(unsigned row=0;row<p.height;++row){const auto final=state.sum(row,col,degree,integral,lower[row]);if(!final)return false;
                const auto at=static_cast<std::size_t>(row)*p.width+col;result.field[at]=final->first;result.absoluteError[at]=final->second;}
            return true;
        };
        if(evaluateDegree(3)){if(work)++work->degree3Columns;continue;}
        if(!state.ensure(col,7,work)||!evaluateDegree(7))return refuse(Refusal::ErrorBudget);
        if(work)++work->degree7Columns;
    }
    return result;
}
std::optional<Result> Bank::evaluateFastMoments(const std::array<double,4>& region,Work* work) const {
    const auto& p=implementation_->signature;
    const auto cells=static_cast<std::size_t>(p.width)*p.height;
    Result result;
    result.field.assign(cells,0.0);
    result.absoluteError.assign(cells,upperPositiveD<32>(extraTailD));
    double maxError=0;unsigned activeCount=0,colBegin=0,colEnd=0;
    if(!implementation_->evaluateFastCoreV5(region,result.field.data(),result.absoluteError.data(),
        maxError,nullptr,activeCount,colBegin,colEnd,work))return std::nullopt;
    return result;
}

bool Bank::evaluateFastIntoV5(const std::array<double,4>& region,FieldWorkspaceV5& workspace,Work* work) const {
    const auto& p=implementation_->signature;
    const auto cells=static_cast<std::size_t>(p.width)*p.height;
    if(workspace.width!=p.width||workspace.height!=p.height||workspace.field.size()!=cells) {
        workspace.field.assign(cells,0.0);
        workspace.width=p.width;workspace.height=p.height;
        workspace.activeRowCount=0;workspace.colBegin=0;workspace.colEnd=0;workspace.dirtyAll=false;
    } else if(workspace.dirtyAll) {
        std::fill(workspace.field.begin(),workspace.field.end(),0.0);
        workspace.activeRowCount=0;workspace.colBegin=0;workspace.colEnd=0;workspace.dirtyAll=false;
    } else if(workspace.activeRowCount>0&&workspace.colEnd>workspace.colBegin) {
        for(unsigned r=0;r<workspace.activeRowCount;++r) {
            double* rowPtr=workspace.field.data()+static_cast<std::size_t>(workspace.activeRows[r])*p.width;
            for(unsigned col=workspace.colBegin;col<workspace.colEnd;++col)rowPtr[col]=0.0;
        }
        workspace.activeRowCount=0;workspace.colBegin=0;workspace.colEnd=0;
    }
    workspace.dirtyAll=true;
    const bool ok=implementation_->evaluateFastCoreV5(region,workspace.field.data(),nullptr,
        workspace.maxError,workspace.activeRows.data(),workspace.activeRowCount,
        workspace.colBegin,workspace.colEnd,work);
    if(ok)workspace.dirtyAll=false;
    return ok;
}

bool Bank::ensureColumnV5(unsigned col) const {
    if(!implementation_||col>=implementation_->signature.width)return false;
    return implementation_->ensure(col,3,nullptr);
}

std::optional<Result> Bank::evaluateFastMemoV5(const std::array<double,4>& region,GaussianMemoV5& memo,Work* work) const {
    const auto& a=implementation_->signature;const auto& b=memo.implementation_->policy;
    if(!arithmeticAvailable()||a.width!=b.width||a.height!=b.height||
       gaussianBitsV5(a.pixelPitchRad)!=gaussianBitsV5(b.pixelPitchRad)||
       gaussianBitsV5(a.psfSigmaRad)!=gaussianBitsV5(b.psfSigmaRad))return evaluateFastMoments(region,work);
    GaussianScopeV5 scope(memo.implementation_.get());
    return evaluateFastMoments(region,work);
}

} // namespace fsim::ms::components::conic_moments_v4
