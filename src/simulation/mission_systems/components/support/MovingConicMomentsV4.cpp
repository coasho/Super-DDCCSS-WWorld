#include "MovingConicMomentsV4.h"
#include "CertifiedNormalTableV4.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <cfenv>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
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

double fromBits(std::uint64_t raw) {double value;std::memcpy(&value,&raw,sizeof(value));return value;}
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
double upperPositive(L value,unsigned operations=32){
    if(!(value>=0)||!std::isfinite(value))return std::numeric_limits<double>::infinity();
    const L n=operations,gamma=n*lepsilon/(1-n*lepsilon);
    if(!(gamma<1))return std::numeric_limits<double>::infinity();
    const L bound=(value+n*ltiny)/(1-gamma);
    const double stored=static_cast<double>(bound);
    return std::nextafter(stored,std::numeric_limits<double>::infinity());
}
struct Mid {double value=0,error=0;};
Mid midpoint(I a){
    if(!a.valid())return {0,std::numeric_limits<double>::infinity()};
    const double centre=static_cast<double>(.5L*a.lo+.5L*a.hi);
    return {centre,upperPositive(std::max(std::abs(a.lo-centre),std::abs(a.hi-centre)),8)};
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
I hermite(unsigned n,I z){I previous(1);if(n==0)return previous;I current=z;
    for(unsigned k=1;k<n;++k){const I next=z*current-I(k)*previous;previous=current;current=next;}return current;}
I densityDerivative(unsigned n,I z,Work* work){I value=hermite(n,z)*pdfI(z,work);return n%2?-value:value;}

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
};

Jet composeGaussian(const Jet& horizon,I low,I high,I sigma,unsigned degree,bool valueNeeded,Work* work){
    std::array<I,maximumDegree+1> derivatives{};
    const I z0=(horizon[0]-low)/sigma,z1=(horizon[0]-high)/sigma;
    if(valueNeeded)derivatives[0]=gaussianCumulative(horizon[0],low,high,sigma,work);
    derivatives[1]=cdfDifference(z0,z1,work);
    I sigmaPower=sigma;
    for(unsigned n=2;n<=degree;++n){derivatives[n]=(densityDerivative(n-2,z0,work)-densityDerivative(n-2,z1,work))/sigmaPower;sigmaPower=sigmaPower*sigma;}
    Jet delta=horizon;delta[0]=I(0);Jet result=constant(derivatives[0]),term=constant(I(1));I factorial(1);
    for(unsigned n=1;n<=degree;++n){term=times(term,delta,degree);factorial=factorial*I(n);result=plus(result,scaled(term,derivatives[n]/factorial,degree),degree);}return result;
}

unsigned binomial(unsigned n,unsigned k){unsigned result=1;for(unsigned i=1;i<=k;++i)result=result*(n+1-i)/i;return result;}
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
    L radius=0;
    Ball()=default;
    explicit Ball(double x):value(x){}
    Ball(double x,L r):value(x),radius(r){}
    bool valid() const {return std::isfinite(value)&&std::isfinite(radius)&&radius>=0;}
};
Ball badBall(){return {0,infinity};}
L radiusUpper(L x) {
    // Each caller expression has at most 16 positive basic operations.
    // The additional factor covers this inflation's own rounded operations.
    if(!(x>=0)||!std::isfinite(x))return infinity;
    constexpr L factor=1+64*lepsilon;
    const L result=(x+64*ltiny)*factor;
    return std::isfinite(result)?result:infinity;
}
L centreError(double x){return depsilon*std::abs(static_cast<L>(x))+dtiny;}
Ball ball(I x) {
    if(!x.valid())return badBall();
    const double centre=static_cast<double>(.5L*x.lo+.5L*x.hi);
    return {centre,radiusUpper(std::max(std::abs(x.lo-centre),std::abs(x.hi-centre)))};
}
I interval(Ball x) {
    if(!x.valid())return invalid();
    return {down(static_cast<L>(x.value)-x.radius),up(static_cast<L>(x.value)+x.radius)};
}
Ball operator-(Ball a){return {-a.value,a.radius};}
Ball operator+(Ball a,Ball b) {
    if(!a.valid()||!b.valid())return badBall();
    const double centre=a.value+b.value;
    return {centre,radiusUpper(a.radius+b.radius+centreError(centre))};
}
Ball operator-(Ball a,Ball b){return a+-b;}
Ball operator*(Ball a,Ball b) {
    if(!a.valid()||!b.valid())return badBall();
    const double centre=a.value*b.value;
    return {centre,radiusUpper(std::abs(static_cast<L>(a.value))*b.radius+
        std::abs(static_cast<L>(b.value))*a.radius+a.radius*b.radius+centreError(centre))};
}
Ball operator/(Ball a,Ball b) {
    if(!a.valid()||!b.valid())return badBall();
    const double centre=a.value/b.value;
    const L difference=std::abs(static_cast<L>(b.value))-b.radius;
    if(!(difference>0)||!std::isfinite(centre))return badBall();
    // A strict positive lower bound is required. Absolute underflow padding
    // enters the numerator BEFORE division, so small divisors amplify it.
    const L denominator=(difference-64*ltiny)*(1-64*lepsilon);
    if(!(denominator>0))return badBall();
    const L numerator=radiusUpper(a.radius+std::abs(static_cast<L>(centre))*b.radius+
        centreError(centre)*std::abs(static_cast<L>(b.value)));
    return {centre,radiusUpper(numerator/denominator)};
}
std::array<I,8> fastMoments(I low,I high,I detectorLow,I detectorHigh,I sigma,I anchor,unsigned degree,Work* work){
#if FLT_EVAL_METHOD != 0
    (void)low;(void)high;(void)detectorLow;(void)detectorHigh;(void)sigma;(void)anchor;(void)degree;(void)work;
    std::array<I,8> refused{};refused.fill(invalid());return refused;
#else
    std::array<Ball,8> answer{};
    const Ball scale=ball(sigma),reference=ball(anchor),bottom=ball(low),top=ball(high);
    for(unsigned edge=0;edge<2;++edge){
        const Ball pixel=ball(edge?detectorHigh:detectorLow);
        std::array<std::array<Ball,8>,2> primitives{};
        for(unsigned side=0;side<2;++side){
            const Ball z=(pixel-(side?top:bottom))/scale;
            const I enclosed=interval(z);
            const Ball cdf=ball(cdfI(enclosed,work)),density=ball(pdfI(enclosed,work));
            std::array<Ball,10> zpower{};zpower[0]=Ball(1);
            for(unsigned n=1;n<=degree+1;++n)zpower[n]=zpower[n-1]*z;
            std::array<Ball,10> normal{};normal[0]=cdf;normal[1]=-density;
            for(unsigned n=2;n<=degree+1;++n)normal[n]=-zpower[n-1]*density+Ball(n-1)*normal[n-2];
            for(unsigned n=0;n<=degree;++n)primitives[side][n]=(zpower[n+1]*cdf-normal[n+1])/Ball(n+1);
        }
        std::array<Ball,8> offset{},spread{};offset[0]=Ball(1);spread[0]=Ball(1);
        for(unsigned n=1;n<=degree;++n){offset[n]=offset[n-1]*(pixel-reference);spread[n]=spread[n-1]*(-scale);}
        for(unsigned n=0;n<=degree;++n){
            Ball value;
            for(unsigned k=0;k<=n;++k)value=value+Ball(binomial(n,k))*offset[n-k]*spread[k]*(primitives[0][k]-primitives[1][k]);
            value=scale*value;answer[n]=edge?answer[n]+value:answer[n]-value;
        }
    }
    std::array<I,8> result{};for(unsigned n=0;n<=degree;++n)result[n]=interval(answer[n]);
    result[0]=intersect(result[0],0,(detectorHigh-detectorLow).hi);
    for(unsigned n=2;n<=degree;n+=2)result[n]=intersect(result[n],0,infinity);
    return result;
#endif
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
    struct Column {
        enum class Mask {Unknown,Sky,Ground,Horizon};
        Mask mask=Mask::Unknown;
        unsigned degree=0;
        double pixelLow=0,pixelHigh=0;
        L anchor=0;
        I xDomain,horizonRange;
        std::vector<std::array<Mid,8>> coefficients;
        std::vector<double> remainder4,remainder8;
    };
    mutable std::vector<Column> columns;

    bool ensure(unsigned index,unsigned degree,Work* work) const {
        auto& column=columns[index];
        if(column.mask==Column::Mask::Sky||column.mask==Column::Mask::Ground)return true;
        if(column.degree>=degree)return true;
        const unsigned jetDegree=degree==3?4u:8u;
        if(column.mask==Column::Mask::Unknown){
            const I xx=tanI(column.xDomain);
            const I low=geometry.level(xx,tanI(I(bottom))),high=geometry.level(xx,tanI(I(top)));
            if(!low.valid()||!high.valid())return false;
            if(low.lo>=geometry.h.hi){column.mask=Column::Mask::Sky;return true;}
            if(high.hi<geometry.h.lo){column.mask=Column::Mask::Ground;return true;}
            if(!(low.hi<geometry.h.lo&&high.lo>geometry.h.hi))return false;
            column.mask=Column::Mask::Horizon;
        }
        const auto point=geometry.horizon(I(column.anchor),jetDegree);
        const auto range=geometry.horizon(column.xDomain,jetDegree);
        if(!valid(point,jetDegree)||!valid(range,jetDegree)||range[0].lo<bottom||range[0].hi>top)return false;
        column.horizonRange=range[0];
        if(column.coefficients.empty())column.coefficients.resize(signature.height);
        const I width=I(column.pixelHigh)-I(column.pixelLow);
        const I mean=(I(column.pixelLow)+I(column.pixelHigh))/I(2);
        const I offset=mean-I(column.anchor);
        const I mu=weightedEvenMoment(jetDegree,width,sigma,offset);
        auto& errors=degree==3?column.remainder4:column.remainder8;errors.resize(signature.height);
        for(unsigned row=0;row<signature.height;++row){
            const double low=(row-.5*signature.height)*signature.pixelPitchRad,high=low+signature.pixelPitchRad;
            const auto polynomial=composeGaussian(point,I(low),I(high),sigma,jetDegree,true,work);
            const auto bound=composeGaussian(range,I(low),I(high),sigma,jetDegree,false,work);
            if(!valid(polynomial,jetDegree)||!valid(bound,jetDegree))return false;
            // Higher-degree construction must not replace lower coefficients:
            // exact-zero jet identities keep their operation prefix identical.
            for(unsigned n=column.degree==0?0:column.degree+1;n<=degree;++n){
                column.coefficients[row][n]=midpoint(polynomial[n]);
                if(!std::isfinite(column.coefficients[row][n].error))return false;
            }
            errors[row]=upperPositive((I(magnitude(bound[jetDegree]))*width*mu/area).hi);
        }
        column.degree=degree;if(work)++work->preparedColumns;return true;
    }

    std::optional<std::pair<double,double>> finish(double value,L error) const {
        if(!std::isfinite(value)||!(error>=0)||!std::isfinite(error))return std::nullopt;
        const double clamped=std::clamp(value,0.,1.);
        error+=std::abs(static_cast<L>(value)-clamped);
        const double bound=upperPositive(error,64);
        if(!std::isfinite(bound)||bound>fieldBudget)return std::nullopt;
        return std::make_pair(clamped,bound);
    }

    std::optional<std::pair<double,double>> sum(unsigned row,unsigned col,unsigned degree,
        const std::array<I,8>& integral,Mid lower) const {
        const auto& column=columns[col];
        std::array<Mid,8> mass{};for(unsigned n=0;n<=degree;++n){mass[n]=midpoint(integral[n]);if(!std::isfinite(mass[n].error))return std::nullopt;}
        double numerator=0;L absolute=0,error=0;
        for(unsigned n=0;n<=degree;++n){
            auto coefficient=column.coefficients[row][n];
            if(n==0){
                const double difference=coefficient.value-lower.value;
                coefficient.error=upperPositive(static_cast<L>(coefficient.error)+lower.error+
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
        return finish(value,upperPositive(error,32*(degree+1)+64));
    }
};

Bank::Bank(std::unique_ptr<Impl> implementation):implementation_(std::move(implementation)){}
Bank::~Bank()=default;
Bank::Bank(Bank&&) noexcept=default;
Bank& Bank::operator=(Bank&&) noexcept=default;

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
            const auto integral=fastMoments(I(x0),I(x1),I(column.pixelLow),I(column.pixelHigh),state.sigma,I(column.anchor),0,work);
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
            const auto integral=fastMoments(I(x0),I(x1),I(column.pixelLow),I(column.pixelHigh),state.sigma,I(column.anchor),degree,work);
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

} // namespace fsim::ms::components::conic_moments_v4
