#include "OperatingPointControlV1.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <cstring>
#include <new>
#include <utility>

namespace fsim::ms::weapon {
namespace {
using A3=std::array<double,3>;
bool finite(double x){return std::isfinite(x);}
bool finite(const A3& x){return std::all_of(x.begin(),x.end(),[](double v){return finite(v);});}
bool valid(const OperatingPointInputV1& in) {
    if(!in.plant.measured||!finite(in.dtS)||in.dtS<=0||!in.actuatorSubsteps||in.actuatorSubsteps>1024||
       !finite(in.plant.actuatorLagS)||in.plant.actuatorLagS<=0||in.dtS/in.actuatorSubsteps>2*in.plant.actuatorLagS||in.fins.empty()||in.fins.size()>32||
       !finite(in.channelInputRad)||!finite(in.channelLagRad)||!finite(in.channelActualRad)||
       !finite(in.plant.measured->momentBodyNm))return false;
    for(unsigned axis=0;axis<3;++axis)
        if(!finite(in.stops.lowerRad[axis])||!finite(in.stops.upperRad[axis])||in.stops.lowerRad[axis]>0||
           in.stops.upperRad[axis]<0||in.stops.lowerRad[axis]>in.stops.upperRad[axis]||
           !finite(in.plant.inertiaKgM2[axis])||in.plant.inertiaKgM2[axis]<=0||
           in.channelInputRad[axis]<in.stops.lowerRad[axis]||in.channelInputRad[axis]>in.stops.upperRad[axis]||
           in.channelLagRad[axis]<in.stops.lowerRad[axis]-1e-12||in.channelLagRad[axis]>in.stops.upperRad[axis]+1e-12||
           in.channelActualRad[axis]<in.stops.lowerRad[axis]-1e-12||in.channelActualRad[axis]>in.stops.upperRad[axis]+1e-12)return false;
    if(!finite(in.stops.channelRateRadS)||in.stops.channelRateRadS<=0)return false;
    for(const auto& fin:in.fins) {
        if(!finite(fin.actualRad)||!finite(fin.minimumRad)||!finite(fin.maximumRad)||fin.minimumRad>fin.maximumRad||
           fin.actualRad<fin.minimumRad-1e-10||fin.actualRad>fin.maximumRad+1e-10||!finite(fin.rateRadS)||fin.rateRadS<=0||
           !finite(fin.mixer)||fin.anglesRad.size()<2||fin.anglesRad.size()>65||fin.momentsNm.size()!=fin.anglesRad.size())return false;
        for(std::size_t k=0;k<fin.anglesRad.size();++k)
            if(!finite(fin.anglesRad[k])||!finite(fin.momentsNm[k])||(k&&fin.anglesRad[k]<=fin.anglesRad[k-1]))return false;
        if(fin.anglesRad.front()>fin.minimumRad||fin.anglesRad.back()<fin.maximumRad)return false;
    }
    return true;
}
// An affine state a+d*u, where u is the varied normalized channel.
// Event classification and the corresponding interval use the SAME direct
// root. Point-slope reconstruction at a rounded probe can stall at an event.
struct D {double a=0,d=0;};
D operator+(D x,D y){return {x.a+y.a,x.d+y.d};}
D operator-(D x,D y){return {x.a-y.a,x.d-y.d};}
D operator*(D x,double y){return {x.a*y,x.d*y};}
struct Window {
    double at=0,low=-1,high=1;
    int compare(D x,double bound) const {
        if(x.d==0)return x.a<bound?-1:x.a>bound?1:0;
        const double root=(bound-x.a)/x.d;
        const int side=at<root?-1:at>root?1:0;
        return x.d>0?side:-side;
    }
    double evaluate(D x) const {return x.a+x.d*at;}
    void below(D x,double bound) {
        if(x.d>0)high=std::min(high,(bound-x.a)/x.d);
        else if(x.d<0)low=std::max(low,(bound-x.a)/x.d);
    }
    void above(D x,double bound){below({-x.a,-x.d},-bound);}
};
D clip(D x,D low,D high,Window& w) {
    const auto l=x-low,h=x-high;
    const int lower=w.compare(l,0),upper=w.compare(h,0);
    if(lower<0||(lower==0&&l.d<0)){w.below(l,0);return low;}
    if(upper>0||(upper==0&&h.d>0)){w.above(h,0);return high;}
    w.above(l,0);w.below(h,0);return x;
}
D malfunction(D input,const OperatingFailureV1& failure,double low,double high,Window& w) {
    if(failure.zero)input={};
    if(failure.hardover) {
        if(w.compare(input,0)<0){w.below(input,0);input={low,0};}
        else {w.above(input,0);input={high,0};}
    }
    return input;
}
std::array<D,3> curve(const OperatingFinV1& fin,D x,Window& w) {
    std::size_t k=0;
    for(;k<fin.anglesRad.size();++k) {
        const int side=w.compare(x,fin.anglesRad[k]);
        if(side<0||(side==0&&x.d<0))break;
    }
    k=k==0?0:std::min(k-1,fin.anglesRad.size()-2);
    w.above(x,fin.anglesRad[k]);w.below(x,fin.anglesRad[k+1]);
    std::array<D,3> result;
    const double width=fin.anglesRad[k+1]-fin.anglesRad[k];
    for(unsigned axis=0;axis<3;++axis) {
        const double slope=(fin.momentsNm[k+1][axis]-fin.momentsNm[k][axis])/width;
        result[axis]={fin.momentsNm[k][axis]+slope*(x.a-fin.anglesRad[k]),slope*x.d};
    }
    return result;
}
double clip(double x,double low,double high,Window&) {
    if(x-low<0)return low;
    if(x-high>0)return high;
    return x;
}
double malfunction(double input,const OperatingFailureV1& failure,double low,double high,Window&) {
    if(failure.zero)input=0;
    if(failure.hardover)input=input<0?low:high;
    return input;
}
A3 curve(const OperatingFinV1& fin,double x,Window&) {
    std::size_t k=static_cast<std::size_t>(std::upper_bound(fin.anglesRad.begin(),fin.anglesRad.end(),x)-fin.anglesRad.begin());
    k=k==0?0:std::min(k-1,fin.anglesRad.size()-2);
    A3 result;
    const double width=fin.anglesRad[k+1]-fin.anglesRad[k];
    for(unsigned axis=0;axis<3;++axis) {
        const double slope=(fin.momentsNm[k+1][axis]-fin.momentsNm[k][axis])/width;
        result[axis]=fin.momentsNm[k][axis]+slope*(x-fin.anglesRad[k]);
    }
    return result;
}
struct Choice {A3 command{};double cost=std::numeric_limits<double>::infinity(),effort=std::numeric_limits<double>::infinity();};
struct ChannelCache {
    unsigned varied=0;
    std::array<A3,1024> actual{};
};
struct MemoSlot {bool used=false;std::array<std::uint64_t,3> key{};Choice result;};
struct SearchContext {
    const OperatingPointSearchOptionsV1& options;
    OperatingPointSearchStatsV1* stats;
    const ChannelCache* channels=nullptr;
    std::array<MemoSlot,1024> memo;
    SearchContext(const OperatingPointSearchOptionsV1& o,OperatingPointSearchStatsV1* s):options(o),stats(s){}
};
std::uint64_t bits(double x){std::uint64_t result;std::memcpy(&result,&x,sizeof result);return result;}
bool same(D x,D y){return bits(x.a)==bits(y.a)&&bits(x.d)==bits(y.d);}
void audit(SearchContext& context,const Choice& candidate) {
    if(context.stats)++context.stats->candidates;
    if(context.options.candidateAudit)context.options.candidateAudit->push_back({candidate.command,candidate.cost,candidate.effort});
}
ChannelCache channelCache(const OperatingPointInputV1& in,const A3& command,unsigned varied) {
    ChannelCache result;result.varied=varied;
    A3 input{},previous=in.channelInputRad,lag=in.channelLagRad,actual=in.channelActualRad;
    for(unsigned axis=0;axis<3;++axis)input[axis]=command[axis]*(command[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis]);
    const double dt=in.dtS/in.actuatorSubsteps,k=dt/in.plant.actuatorLagS,ca=k/(2+k),cb=(2-k)/(2+k);
    Window unused;
    for(unsigned step=0;step<in.actuatorSubsteps;++step)for(unsigned axis=0;axis<3;++axis)if(axis!=varied) {
        lag[axis]=(input[axis]+previous[axis])*ca+lag[axis]*cb;previous[axis]=input[axis];
        if(!in.channelFailure[axis].stuck) {
            const double demand=malfunction(lag[axis],in.channelFailure[axis],in.stops.lowerRad[axis],in.stops.upperRad[axis],unused);
            actual[axis]=clip(demand,actual[axis]-in.stops.channelRateRadS*dt,actual[axis]+in.stops.channelRateRadS*dt,unused);
            actual[axis]=clip(actual[axis],in.stops.lowerRad[axis],in.stops.upperRad[axis],unused);
        }
        result.actual[step][axis]=actual[axis];
    }
    return result;
}
template<class Number> struct Prediction {std::array<Number,3> moment;std::array<double,32> fins{};double right=1;};
template<bool Differential> auto predict(const OperatingPointInputV1& in,const A3& command,int varied=-1,OperatingPointSearchStatsV1* stats=nullptr,const ChannelCache* channels=nullptr) {
    if(stats){if constexpr(Differential)++stats->differentialPredictions;else ++stats->scalarPredictions;}
    using Number=std::conditional_t<Differential,D,double>;
    Window w;w.at=varied<0?0:command[static_cast<std::size_t>(varied)];
    std::array<Number,3> input,lag,channel,previous;
    for(unsigned axis=0;axis<3;++axis) {
        Number u{command[axis]};
        bool negative=command[axis]<0;
        if constexpr(Differential) {
            if(varied==static_cast<int>(axis))u={0,1};
            const int side=w.compare(u,0);negative=side<0||(side==0&&u.d<0);
            if(negative)w.below(u,0);else w.above(u,0);
        }
        input[axis]=u*(negative?-in.stops.lowerRad[axis]:in.stops.upperRad[axis]);
        lag[axis]=Number{in.channelLagRad[axis]};channel[axis]=Number{in.channelActualRad[axis]};previous[axis]=Number{in.channelInputRad[axis]};
    }
    std::array<Number,32> fins{};
    for(std::size_t i=0;i<in.fins.size();++i)fins[i]=Number{in.fins[i].actualRad};
    const double dt=in.dtS/in.actuatorSubsteps,k=dt/in.plant.actuatorLagS,ca=k/(2+k),cb=(2-k)/(2+k);
    for(unsigned step=0;step<in.actuatorSubsteps;++step) {
        for(unsigned axis=0;axis<3;++axis) {
            if constexpr(!Differential)if(channels&&axis!=channels->varied) {
                channel[axis]=channels->actual[step][axis];continue;
            }
            lag[axis]=(input[axis]+previous[axis])*ca+lag[axis]*cb;previous[axis]=input[axis];
            // The separate native lag_filter continues updating even when its
            // downstream channel actuator is stuck.
            if(!in.channelFailure[axis].stuck) {
                const auto demand=malfunction(lag[axis],in.channelFailure[axis],in.stops.lowerRad[axis],in.stops.upperRad[axis],w);
                channel[axis]=clip(demand,channel[axis]+Number{-in.stops.channelRateRadS*dt},channel[axis]+Number{in.stops.channelRateRadS*dt},w);
                channel[axis]=clip(channel[axis],Number{in.stops.lowerRad[axis]},Number{in.stops.upperRad[axis]},w);
            }
        }
        for(std::size_t i=0;i<in.fins.size();++i) {
            const auto& f=in.fins[i];
            if(f.failure.stuck)continue;
            Number mix{};
            for(unsigned axis=0;axis<3;++axis)mix=mix+channel[axis]*f.mixer[axis];
            mix=clip(mix,Number{f.minimumRad},Number{f.maximumRad},w);
            mix=malfunction(mix,f.failure,f.minimumRad,f.maximumRad,w);
            fins[i]=clip(mix,fins[i]+Number{-f.rateRadS*dt},fins[i]+Number{f.rateRadS*dt},w);
            fins[i]=clip(fins[i],Number{f.minimumRad},Number{f.maximumRad},w);
        }
    }
    Prediction<Number> out;for(unsigned axis=0;axis<3;++axis)out.moment[axis]=Number{in.plant.measured->momentBodyNm[axis]};
    for(std::size_t i=0;i<in.fins.size();++i) {
        Window unused;const auto before=curve(in.fins[i],Number{in.fins[i].actualRad},unused),after=curve(in.fins[i],fins[i],w);
        for(unsigned axis=0;axis<3;++axis)out.moment[axis]=out.moment[axis]+after[axis]-before[axis];
        if constexpr(Differential)out.fins[i]=w.evaluate(fins[i]);else out.fins[i]=fins[i];
    }
    out.right=w.high;return out;
}
Choice choice(const OperatingPointInputV1& in,const A3& target,const A3& u,SearchContext& context) {
    const std::array<std::uint64_t,3> key={bits(u[0]),bits(u[1]),bits(u[2])};
    std::uint64_t hash=14695981039346656037ull;for(const auto word:key)hash=(hash^word)*1099511628211ull;
    hash^=hash>>32;
    auto& slot=context.memo[hash%context.memo.size()];
    const bool memoize=context.options.memoize&&!context.options.referenceTracer;
    if(memoize&&slot.used&&slot.key==key) {
        if(context.stats)++context.stats->memoHits;
        audit(context,slot.result);return slot.result;
    }
    const auto predicted=predict<false>(in,u,-1,context.stats,context.channels);
    Choice result;result.command=u;result.cost=0;result.effort=0;
    for(unsigned axis=0;axis<3;++axis) {
        const double error=(predicted.moment[axis]-target[axis])/in.plant.inertiaKgM2[axis];result.cost+=error*error;
        const double extent=in.channelInputRad[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis];
        const double old=extent>0?in.channelInputRad[axis]/extent:0;result.effort+=(u[axis]-old)*(u[axis]-old);
    }
    if(memoize){slot.used=true;slot.key=key;slot.result=result;}
    audit(context,result);return result;
}
bool better(const Choice& a,const Choice& b) {
    const double tolerance=64*std::numeric_limits<double>::epsilon()*std::max(1.,std::min(a.cost,b.cost));
    return a.cost<b.cost-tolerance||(std::abs(a.cost-b.cost)<=tolerance&&a.effort<b.effort);
}
Choice traceLine(const OperatingPointInputV1& in,const A3& target,Choice best,unsigned axis,SearchContext& context) {
    auto* stats=context.stats;
    double at=-1;std::size_t segments=0,maxSegments=128+in.actuatorSubsteps*(24+8*in.fins.size());
    for(const auto& fin:in.fins)maxSegments+=fin.anglesRad.size();
    while(at<1&&segments<maxSegments) {
        ++segments;if(stats)++stats->lineSegments;
        A3 probe=best.command;probe[axis]=std::nextafter(at,1.);
        if(probe[axis]==1) {
            const auto endpoint=choice(in,target,probe,context);if(better(endpoint,best))best=endpoint;
            at=1;break; // No representable open interval remains before +1.
        }
        const auto p=predict<true>(in,probe,static_cast<int>(axis),stats);
        const double end=std::clamp(p.right,probe[axis],1.);double slope=0,curvature=0;
        if(stats&&end<=probe[axis])++stats->boundaryRetries;
        for(unsigned k=0;k<3;++k) {
            const double e=(p.moment[k].a+p.moment[k].d*probe[axis]-target[k])/in.plant.inertiaKgM2[k],d=p.moment[k].d/in.plant.inertiaKgM2[k];
            slope+=e*d;curvature+=d*d;
        }
        const double extent=in.channelInputRad[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis];
        const double prior=extent>0?in.channelInputRad[axis]/extent:0;
        A3 u=best.command;u[axis]=curvature>0?std::clamp(probe[axis]-slope/curvature,at,end):std::clamp(prior,at,end);
        const auto candidate=choice(in,target,u,context);if(better(candidate,best))best=candidate;
        u[axis]=end;const auto boundary=choice(in,target,u,context);if(better(boundary,best))best=boundary;
        if(end<=at)break;
        at=end;
    }
    if(stats) {
        stats->maxLineSegments=std::max(stats->maxLineSegments,static_cast<std::uint64_t>(segments));
        if(at<1)++stats->budgetExhaustions;else ++stats->completedLines;
    }
    return best;
}
struct MapCapacity {};
struct Piece {double low=-1,high=1;D expression;};
using Map=std::vector<Piece>;
struct MapBuilder {
    std::size_t capacity;
    OperatingPointSearchStatsV1* stats;
    std::vector<double> events{-1,1};
    MapBuilder(std::size_t limit,OperatingPointSearchStatsV1* counters):capacity(limit),stats(counters) {
        if(capacity<2)throw MapCapacity{};
    }
    void event(double x) {
        if(!(x>-1&&x<1))return;
        const auto found=std::lower_bound(events.begin(),events.end(),x);
        if(found!=events.end()&&*found==x)return;
        if(events.size()>=capacity)throw MapCapacity{};
        events.insert(found,x);
    }
    void append(Map& output,double low,double high,D expression) {
        if(!(low<high))return;
        if(!finite(expression.a)||!finite(expression.d))throw MapCapacity{};
        if(stats)++stats->mapPieces;
        if(!output.empty()&&output.back().high==low&&same(output.back().expression,expression))output.back().high=high;
        else {
            if(output.size()>=capacity)throw MapCapacity{};
            output.push_back({low,high,expression});
        }
    }
    Map constant(double x) {return {{-1,1,{x,0}}};}
    Map scale(const Map& input,double gain) {
        Map output;output.reserve(input.size());
        for(const auto& p:input)append(output,p.low,p.high,p.expression*gain);
        return output;
    }
    Map shift(const Map& input,double offset,bool subtract=false) {
        Map output;output.reserve(input.size());
        for(const auto& p:input)append(output,p.low,p.high,subtract?p.expression-D{offset,0}:p.expression+D{offset,0});
        return output;
    }
    Map add(const Map& x,const Map& y) {
        Map output;output.reserve(std::min(capacity,x.size()+y.size()));
        std::size_t i=0,j=0;
        while(i<x.size()&&j<y.size()) {
            const double low=std::max(x[i].low,y[j].low),high=std::min(x[i].high,y[j].high);
            append(output,low,high,x[i].expression+y[j].expression);
            if(x[i].high==high)++i;
            if(y[j].high==high)++j;
        }
        return output;
    }
    void root(std::array<double,67>& cuts,std::size_t& count,D x,double bound,double low,double high) {
        if(x.d==0)return;
        const double crossing=(bound-x.a)/x.d;
        if(crossing>low&&crossing<high){cuts[count++]=crossing;event(crossing);}
    }
    template<class Function> void pieces(Map& output,std::array<double,67>& cuts,std::size_t count,Function expression) {
        std::sort(cuts.begin(),cuts.begin()+static_cast<std::ptrdiff_t>(count));
        count=static_cast<std::size_t>(std::unique(cuts.begin(),cuts.begin()+static_cast<std::ptrdiff_t>(count))-cuts.begin());
        for(std::size_t k=1;k<count;++k) {
            Window w;w.at=std::nextafter(cuts[k-1],cuts[k]);
            append(output,cuts[k-1],cuts[k],expression(w));
        }
    }
    Map clipped(const Map& x,const Map& lowMap,const Map& highMap) {
        Map output;output.reserve(std::min(capacity,x.size()+lowMap.size()+highMap.size()));
        std::size_t i=0,j=0,k=0;
        while(i<x.size()&&j<lowMap.size()&&k<highMap.size()) {
            const double low=std::max({x[i].low,lowMap[j].low,highMap[k].low});
            const double high=std::min({x[i].high,lowMap[j].high,highMap[k].high});
            std::array<double,67> cuts{};cuts[0]=low;cuts[1]=high;std::size_t count=2;
            root(cuts,count,x[i].expression-lowMap[j].expression,0,low,high);
            root(cuts,count,x[i].expression-highMap[k].expression,0,low,high);
            pieces(output,cuts,count,[&](Window& w){return clip(x[i].expression,lowMap[j].expression,highMap[k].expression,w);});
            if(x[i].high==high)++i;
            if(lowMap[j].high==high)++j;
            if(highMap[k].high==high)++k;
        }
        return output;
    }
    Map stopped(const Map& input,double low,double high) {return clipped(input,constant(low),constant(high));}
    Map failed(Map input,const OperatingFailureV1& failure,double low,double high) {
        if(failure.zero)input=constant(0);
        if(!failure.hardover)return input;
        Map output;output.reserve(std::min(capacity,input.size()*2));
        for(const auto& p:input) {
            std::array<double,67> cuts{};cuts[0]=p.low;cuts[1]=p.high;std::size_t count=2;
            root(cuts,count,p.expression,0,p.low,p.high);
            pieces(output,cuts,count,[&](Window& w){return malfunction(p.expression,failure,low,high,w);});
        }
        return output;
    }
    Map moment(const Map& input,const OperatingFinV1& fin,unsigned axis) {
        Map output;output.reserve(std::min(capacity,input.size()+fin.anglesRad.size()));
        for(const auto& p:input) {
            std::array<double,67> cuts{};cuts[0]=p.low;cuts[1]=p.high;std::size_t count=2;
            for(double knot:fin.anglesRad)root(cuts,count,p.expression,knot,p.low,p.high);
            pieces(output,cuts,count,[&](Window& w){return curve(fin,p.expression,w)[axis];});
        }
        return output;
    }
};
Choice mapLine(const OperatingPointInputV1& in,const A3& target,Choice best,unsigned axis,SearchContext& context) {
    auto* stats=context.stats;
    std::size_t limit=128+in.actuatorSubsteps*(24+8*in.fins.size());
    for(const auto& fin:in.fins)limit+=fin.anglesRad.size();
    if(context.options.mapCapacity)limit=std::min(limit,context.options.mapCapacity);
    const auto cached=channelCache(in,best.command,axis);
    struct Restore {
        SearchContext& owner;const ChannelCache* saved;
        ~Restore(){owner.channels=saved;}
    } restore{context,context.channels};
    if(context.options.cacheNonVaried)context.channels=&cached;
    if(stats)stats->cachedChannelSteps+=2*in.actuatorSubsteps;
    try {
        MapBuilder build(limit,stats);build.event(0);
        Map input;build.append(input,-1,0,{0,-in.stops.lowerRad[axis]});build.append(input,0,1,{0,in.stops.upperRad[axis]});
        Map previous=build.constant(in.channelInputRad[axis]),lag=build.constant(in.channelLagRad[axis]);
        Map channel=build.constant(in.channelActualRad[axis]);
        std::vector<Map> fins;fins.reserve(in.fins.size());for(const auto& fin:in.fins)fins.push_back(build.constant(fin.actualRad));
        const double dt=in.dtS/in.actuatorSubsteps,k=dt/in.plant.actuatorLagS,ca=k/(2+k),cb=(2-k)/(2+k);
        for(unsigned step=0;step<in.actuatorSubsteps;++step) {
            lag=build.add(build.scale(build.add(input,previous),ca),build.scale(lag,cb));previous=input;
            if(!in.channelFailure[axis].stuck) {
                const auto demand=build.failed(lag,in.channelFailure[axis],in.stops.lowerRad[axis],in.stops.upperRad[axis]);
                channel=build.clipped(demand,build.shift(channel,-in.stops.channelRateRadS*dt),build.shift(channel,in.stops.channelRateRadS*dt));
                channel=build.stopped(channel,in.stops.lowerRad[axis],in.stops.upperRad[axis]);
            }
            for(std::size_t i=0;i<in.fins.size();++i) {
                const auto& fin=in.fins[i];if(fin.failure.stuck)continue;
                Map mix=build.constant(0);
                for(unsigned component=0;component<3;++component)
                    mix=build.add(mix,build.scale(component==axis?channel:build.constant(cached.actual[step][component]),fin.mixer[component]));
                mix=build.stopped(mix,fin.minimumRad,fin.maximumRad);
                mix=build.failed(std::move(mix),fin.failure,fin.minimumRad,fin.maximumRad);
                fins[i]=build.clipped(mix,build.shift(fins[i],-fin.rateRadS*dt),build.shift(fins[i],fin.rateRadS*dt));
                fins[i]=build.stopped(fins[i],fin.minimumRad,fin.maximumRad);
            }
        }
        std::array<Map,3> moments;
        for(unsigned component=0;component<3;++component) {
            moments[component]=build.constant(in.plant.measured->momentBodyNm[component]);
            for(std::size_t i=0;i<in.fins.size();++i) {
                Window unused;const auto before=curve(in.fins[i],D{in.fins[i].actualRad,0},unused);
                moments[component]=build.shift(build.add(moments[component],build.moment(fins[i],in.fins[i],component)),before[component].a,true);
            }
        }
        std::array<std::size_t,3> indices{};std::size_t segments=0;
        for(std::size_t event=1;event<build.events.size();++event) {
            ++segments;if(stats)++stats->lineSegments;
            const double at=build.events[event-1],end=build.events[event];
            A3 probe=best.command;probe[axis]=std::nextafter(at,1.);
            if(probe[axis]==1) {
                const auto endpoint=choice(in,target,probe,context);if(better(endpoint,best))best=endpoint;
                break;
            }
            double slope=0,curvature=0;
            for(unsigned component=0;component<3;++component) {
                auto& index=indices[component];const auto& map=moments[component];
                while(index+1<map.size()&&probe[axis]>=map[index].high)++index;
                const auto d=map[index].expression;
                const double error=(d.a+d.d*probe[axis]-target[component])/in.plant.inertiaKgM2[component];
                const double derivative=d.d/in.plant.inertiaKgM2[component];slope+=error*derivative;curvature+=derivative*derivative;
            }
            const double extent=in.channelInputRad[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis];
            const double prior=extent>0?in.channelInputRad[axis]/extent:0;
            A3 u=best.command;u[axis]=curvature>0?std::clamp(probe[axis]-slope/curvature,at,end):std::clamp(prior,at,end);
            const auto candidate=choice(in,target,u,context);if(better(candidate,best))best=candidate;
            u[axis]=end;const auto boundary=choice(in,target,u,context);if(better(boundary,best))best=boundary;
        }
        if(stats){++stats->mapLines;++stats->completedLines;stats->maxLineSegments=std::max(stats->maxLineSegments,static_cast<std::uint64_t>(segments));}
        return best;
    } catch(const MapCapacity&) {
        if(stats)++stats->fallbackLines;
        return traceLine(in,target,best,axis,context);
    } catch(const std::bad_alloc&) {
        if(stats)++stats->fallbackLines;
        return traceLine(in,target,best,axis,context);
    }
}
Choice line(const OperatingPointInputV1& in,const A3& target,Choice best,unsigned axis,SearchContext& context) {
    return context.options.referenceTracer?traceLine(in,target,best,axis,context):mapLine(in,target,best,axis,context);
}

using Jacobian=std::array<A3,3>; // row=angular acceleration, column=normalized channel
// Small pivoted Householder QR solves augmented least squares without normal
// equations. Numerical damping is sqrt(64*eps) after effectiveness scaling;
// every proposed step is judged by the original UNDAMPED native cost.
bool faceStep(const Jacobian& jacobian,const A3& error,const A3& low,const A3& high,
    const A3& preferred,double scale,unsigned face,A3& step,double& cost) {
    if(!finite(scale)||!finite(error))return false;
    const double damping=std::sqrt(64*std::numeric_limits<double>::epsilon());
    double design[6][3]{};std::array<double,6> rhs{};
    for(unsigned row=0;row<3;++row) {
        rhs[row]=-error[row]/scale;rhs[row+3]=damping*preferred[row];
        for(unsigned column=0;column<3;++column)design[row][column]=jacobian[row][column]/scale;
        design[row+3][row]=damping;
    }
    std::array<unsigned,3> free{};unsigned count=0;
    for(unsigned axis=0;axis<3;++axis) {
        const unsigned state=face%3;face/=3;
        if(state==0){free[count++]=axis;step[axis]=0;}
        else {
            step[axis]=state==1?low[axis]:high[axis];
            for(unsigned row=0;row<6;++row)rhs[row]-=design[row][axis]*step[axis];
        }
    }
    double matrix[6][3]{};
    for(unsigned row=0;row<6;++row)for(unsigned column=0;column<count;++column)matrix[row][column]=design[row][free[column]];
    for(unsigned column=0;column<count;++column) {
        unsigned pivot=column;double largest=-1;
        for(unsigned candidate=column;candidate<count;++candidate) {
            // The matrix is scaled to |entry|<=1 before orthogonal
            // transformations; squared norms cannot overflow or underflow.
            double squared=0;for(unsigned row=column;row<6;++row)squared+=matrix[row][candidate]*matrix[row][candidate];
            const double norm=std::sqrt(squared);
            if(norm>largest){largest=norm;pivot=candidate;}
        }
        if(!(largest>0)||!finite(largest))return false;
        if(pivot!=column) {
            std::swap(free[pivot],free[column]);
            for(unsigned row=0;row<6;++row)std::swap(matrix[row][pivot],matrix[row][column]);
        }
        const double diagonal=-std::copysign(largest,matrix[column][column]);
        std::array<double,6> reflector{};
        for(unsigned row=column;row<6;++row)reflector[row]=matrix[row][column];
        reflector[column]-=diagonal;
        double squared=0;for(unsigned row=column;row<6;++row)squared+=reflector[row]*reflector[row];
        if(!(squared>0)||!finite(squared))return false;
        const double beta=2/squared;
        for(unsigned target=column;target<count;++target) {
            double dot=0;for(unsigned row=column;row<6;++row)dot+=reflector[row]*matrix[row][target];
            for(unsigned row=column;row<6;++row)matrix[row][target]-=beta*reflector[row]*dot;
        }
        double dot=0;for(unsigned row=column;row<6;++row)dot+=reflector[row]*rhs[row];
        for(unsigned row=column;row<6;++row)rhs[row]-=beta*reflector[row]*dot;
        matrix[column][column]=diagonal;
    }
    A3 solution{};
    for(unsigned reverse=count;reverse>0;--reverse) {
        const unsigned row=reverse-1;double value=rhs[row];
        for(unsigned column=row+1;column<count;++column)value-=matrix[row][column]*solution[column];
        solution[row]=value/matrix[row][row];
        if(!finite(solution[row]))return false;
    }
    for(unsigned column=0;column<count;++column)step[free[column]]=solution[column];
    for(unsigned axis=0;axis<3;++axis) {
        const double tolerance=256*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(low[axis]),std::abs(high[axis])});
        if(step[axis]<low[axis]-tolerance||step[axis]>high[axis]+tolerance)return false;
        step[axis]=std::clamp(step[axis],low[axis],high[axis]);
    }
    cost=0;
    for(unsigned row=0;row<3;++row) {
        double residual=error[row]/scale;
        for(unsigned axis=0;axis<3;++axis)residual+=jacobian[row][axis]*step[axis]/scale;
        cost+=residual*residual;
        const double movement=damping*(step[row]-preferred[row]);cost+=movement*movement;
    }
    return finite(cost);
}
A3 boxStep(const Jacobian& positive,const Jacobian& negative,const A3& error,const A3& current,
    const A3& prior,OperatingPointSearchStatsV1* stats) {
    A3 selected{};double minimum=std::numeric_limits<double>::infinity(),scale=1;
    for(const auto* side:{&positive,&negative})for(const auto& row:*side)for(double x:row)scale=std::max(scale,std::abs(x));
    bool smooth=true;Jacobian common{};
    for(unsigned axis=0;axis<3;++axis)for(unsigned row=0;row<3;++row) {
        if(current[axis]<=-1)common[row][axis]=positive[row][axis];
        else if(current[axis]>=1)common[row][axis]=negative[row][axis];
        else {
            common[row][axis]=positive[row][axis];
            if(positive[row][axis]!=negative[row][axis])smooth=false;
        }
    }
    for(unsigned orthant=0;orthant<(smooth?1u:8u);++orthant) {
        Jacobian jacobian{};A3 low{},high{},preferred{};
        for(unsigned axis=0;axis<3;++axis) {
            const bool forward=(orthant&(1u<<axis))!=0;
            low[axis]=smooth||!forward?-1-current[axis]:0;
            high[axis]=smooth||forward?1-current[axis]:0;
            preferred[axis]=prior[axis]-current[axis];
            for(unsigned row=0;row<3;++row)jacobian[row][axis]=smooth?common[row][axis]:(forward?positive[row][axis]:negative[row][axis]);
        }
        for(unsigned face=0;face<27;++face) {
            if(stats)++stats->qpFaces;
            A3 step{};double cost=0;
            if(faceStep(jacobian,error,low,high,preferred,scale,face,step,cost)&&cost<minimum){minimum=cost;selected=step;}
        }
    }
    return selected;
}
Choice sequential(const OperatingPointInputV1& in,const A3& target,const A3& wanted,Choice best,SearchContext& context) {
    A3 prior{};
    for(unsigned axis=0;axis<3;++axis) {
        const double extent=in.channelInputRad[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis];
        prior[axis]=extent>0?std::clamp(in.channelInputRad[axis]/extent,-1.,1.):0;
    }
    std::array<A3,11> starts{};starts[0]=prior;starts[2]=wanted;
    for(unsigned corner=0;corner<8;++corner)starts[corner+3]={corner&1?1.:-1.,corner&2?1.:-1.,corner&4?1.:-1.};
    for(const auto& start:starts) {
        if(context.stats)++context.stats->seeds;
        auto current=choice(in,target,start,context);
        for(unsigned iteration=0;iteration<4;++iteration) {
            if(context.stats)++context.stats->sequentialIterations;
            const auto actual=predict<false>(in,current.command,-1,context.stats);
            A3 error{};Jacobian positive{},negative{};
            for(unsigned row=0;row<3;++row)error[row]=(actual.moment[row]-target[row])/in.plant.inertiaKgM2[row];
            for(unsigned axis=0;axis<3;++axis)for(unsigned side=0;side<2;++side) {
                if((side==0&&current.command[axis]>=1)||(side==1&&current.command[axis]<=-1))continue;
                A3 probe=current.command;probe[axis]=std::nextafter(probe[axis],side==0?1.:-1.);
                const auto derivative=predict<true>(in,probe,static_cast<int>(axis),context.stats);
                for(unsigned row=0;row<3;++row)(side==0?positive:negative)[row][axis]=derivative.moment[row].d/in.plant.inertiaKgM2[row];
            }
            const auto step=boxStep(positive,negative,error,current.command,prior,context.stats);
            bool accepted=false;double fraction=1;
            for(unsigned backtrack=0;backtrack<8;++backtrack) {
                A3 command{};for(unsigned axis=0;axis<3;++axis)command[axis]=std::clamp(current.command[axis]+fraction*step[axis],-1.,1.);
                if(context.stats)++context.stats->backtracks;
                const auto candidate=choice(in,target,command,context);
                if(better(candidate,current)){current=candidate;accepted=true;break;}
                fraction*=.5;
            }
            if(!accepted) {
                auto polled=current;
                for(unsigned axis=0;axis<3;++axis)for(double bound:{-1.,1.}) {
                    A3 command=current.command;command[axis]=bound;
                    if(context.stats)++context.stats->polls;
                    const auto candidate=choice(in,target,command,context);if(better(candidate,polled))polled=candidate;
                }
                if(!better(polled,current))break;
                current=polled;
            }
        }
        if(better(current,best))best=current;
    }
    return best;
}

}
std::array<double,3> operatingPointBodyMomentV1(const std::array<double,6>& load,
    double alpha,double beta,const A3& offset) noexcept {
    const double ca=std::cos(alpha),sa=std::sin(alpha),cb=std::cos(beta),sb=std::sin(beta);
    const A3 force={-load[0]*ca*cb-load[1]*ca*sb+load[2]*sa,
        -load[0]*sb+load[1]*cb,-load[0]*sa*cb-load[1]*sa*sb-load[2]*ca};
    return {load[3]-offset[1]*force[2]+offset[2]*force[1],
        load[4]-offset[2]*force[0]+offset[0]*force[2],
        load[5]-offset[0]*force[1]+offset[1]*force[0]};
}
std::array<double,3> operatingPointDirectionalMomentV1(const OperatingPointInputV1& in,const A3& motion) noexcept {
    A3 result{};if(!valid(in)||!finite(motion))return result;
    for(const auto& fin:in.fins) {
        if(fin.failure.stuck||fin.failure.zero||fin.failure.hardover)continue;
        double movement=0;for(unsigned axis=0;axis<3;++axis) {
            const auto& failed=in.channelFailure[axis];
            if(!failed.stuck&&!failed.zero&&!failed.hardover)movement+=fin.mixer[axis]*motion[axis];
        }
        if((fin.actualRad<=fin.minimumRad&&movement<0)||(fin.actualRad>=fin.maximumRad&&movement>0))movement=0;
        Window unused;const auto moment=curve(fin,{fin.actualRad,movement},unused);
        for(unsigned axis=0;axis<3;++axis)result[axis]+=moment[axis].d;
    }
    return result;
}
static OperatingPointResultV1 operatingPointCommandsImpl(const OperatingPointInputV1& in,const FinControlConfigV1& config,
    OperatingPointSearchStatsV1* stats,const OperatingPointSearchOptionsV1& options,bool inputStage=false) noexcept {
    try {
    SearchContext context(options,stats);
    OperatingPointResultV1 result;if(!valid(in))return result;
    const auto nominal=finCommands(in.plant,config,in.stops);if(!nominal.available)return result;
    const A3 wanted={nominal.aileron,nominal.elevator,nominal.rudder};
    A3 virtualEndpoint{};
    if(inputStage) {
        // Excluded-axis index3 excludes none of the three channels. Reuse the
        // exact native predictor, including previous INPUT, finite virtual
        // lag/rate/stops and malfunction order at the existing native clock.
        // valid(in) has already bounded the endpoint index to [0,1023].
        const auto channels=channelCache(in,wanted,3);
        virtualEndpoint=channels.actual[in.actuatorSubsteps-1];
    }
    const A3 gain={in.plant.rollMomentPerRad,in.plant.pitch.momentControl,in.plant.yaw.momentControl};
    for(unsigned axis=0;axis<3;++axis) {
        const double angle=inputStage?virtualEndpoint[axis]:
            wanted[axis]*(wanted[axis]<0?-in.stops.lowerRad[axis]:in.stops.upperRad[axis]);
        result.requestedMomentNm[axis]=in.plant.measured->momentBodyNm[axis]+gain[axis]*(angle-in.plant.measured->controlRad[axis]);
    }
    if(!finite(result.requestedMomentNm))return {};
    Choice best=choice(in,result.requestedMomentNm,{},context);
    if(options.referenceTracer||options.exhaustiveReference) {
        std::array<A3,10> starts{};starts[1]=wanted;
        for(int corner=0;corner<8;++corner)starts[static_cast<std::size_t>(corner)+2]={corner&1?1.:-1.,corner&2?1.:-1.,corner&4?1.:-1.};
        for(auto start:starts) {
            auto current=choice(in,result.requestedMomentNm,start,context);
            for(std::size_t sweep=0;sweep<2*in.fins.size()+3;++sweep) {
                const auto before=current;
                for(unsigned axis=0;axis<3;++axis)current=line(in,result.requestedMomentNm,current,axis,context);
                if(!better(current,before))break;
            }
            if(better(current,best))best=current;
        }
    } else best=sequential(in,result.requestedMomentNm,wanted,best,context);
    if(!finite(best.cost)||!finite(best.command))return {};
    const auto p=predict<false>(in,best.command,-1,stats);
    result.predictedFinRad.assign(p.fins.begin(),p.fins.begin()+static_cast<std::ptrdiff_t>(in.fins.size()));
    result.residualAccelerationNorm=std::sqrt(best.cost);
    for(unsigned axis=0;axis<3;++axis)result.predictedMomentNm[axis]=p.moment[axis];
    result.commands={best.command[0],best.command[1],best.command[2],true};return result;
    } catch(...) {
        // The native noexcept boundary refuses unavailable storage instead of
        // allowing bounded scratch-vector allocation failure to terminate it.
        return {};
    }
}
OperatingPointResultV1 operatingPointCommandsV1(const OperatingPointInputV1& in,const FinControlConfigV1& config) noexcept {
    return operatingPointCommandsImpl(in,config,nullptr,{});
}
OperatingPointResultV1 operatingPointCommandsV1(const OperatingPointInputV1& in,const FinControlConfigV1& config,
    OperatingPointSearchStatsV1& stats) noexcept {
    stats={};return operatingPointCommandsImpl(in,config,&stats,{});
}

OperatingPointResultV1 operatingPointCommandsV1(const OperatingPointInputV1& in,const FinControlConfigV1& config,
    OperatingPointSearchStatsV1& stats,const OperatingPointSearchOptionsV1& options) noexcept {
    stats={};return operatingPointCommandsImpl(in,config,&stats,options);
}

OperatingPointResultV1 operatingPointInputCommandsV1(const OperatingPointInputV1& in,const FinControlConfigV1& config) noexcept {
    return operatingPointCommandsImpl(in,config,nullptr,{},true);
}

OperatingPointResultV1 operatingPointPredictionV1(const OperatingPointInputV1& in,const A3& command) noexcept {
    try {
        OperatingPointResultV1 result;
        if(!valid(in)||!finite(command)||std::any_of(command.begin(),command.end(),[](double x){return std::abs(x)>1;}))return result;
        const auto prediction=predict<false>(in,command);
        result.predictedFinRad.assign(prediction.fins.begin(),prediction.fins.begin()+static_cast<std::ptrdiff_t>(in.fins.size()));
        result.predictedMomentNm=prediction.moment;
        if(!finite(result.predictedMomentNm))return {};
        result.commands={command[0],command[1],command[2],true};return result;
    } catch(...) {return {};}
}

}
