#pragma once
// Conservative track-to-track fusion with unknown cross-correlation.
// Covariance intersection (Julier/Uhlmann); no independent-noise assumption,
// no target-truth association, no public inbox consumption.
#include "fsim/Fusion.h"
#include "simulation/mission_systems/track/Filters.h"
#include <algorithm>
#include <map>
#include <tuple>
#include <limits>

namespace fsim::ms::track {

inline double logDet(const Covar& p) {
    if(std::any_of(p.a.begin(),p.a.end(),[](double v){return !std::isfinite(v);}))return std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<6;++i)for(std::size_t j=0;j<i;++j)
        if(std::fabs(p(i,j)-p(j,i))>1e-9*std::max({1.,std::fabs(p(i,i)),std::fabs(p(j,j))}))return std::numeric_limits<double>::infinity();
    Covar l;if(!cholesky(p,l))return std::numeric_limits<double>::infinity();
    double v=0;for(std::size_t i=0;i<6;++i)v+=2*std::log(l(i,i));return v;
}

inline bool intersect(const Gaussian& a,const Gaussian& b,Gaussian& out) {
    Covar ia,ib;
    if(a.t!=b.t || !std::isfinite(logDet(a.p)) || !std::isfinite(logDet(b.p)) || !inverse(a.p,ia) || !inverse(b.p,ib))return false;
    auto covariance=[&](double w) {Covar p;if(!inverse(w*ia+(1-w)*ib,p))p.a.fill(std::numeric_limits<double>::quiet_NaN());return p;};
    // Fixed bounded grid plus endpoints. The optimized objective is logdet,
    // invariant to a common coordinate/unit transform; this is not a fit scale.
    double best=std::numeric_limits<double>::infinity(),weight=0;
    for(int i=0;i<=32;++i){const double w=i/32.;const double score=logDet(covariance(w));if(score<best)best=score,weight=w;}
    if(!std::isfinite(best))return false;
    out.p=covariance(weight);out.x=out.p*(weight*(ia*a.x)+(1-weight)*(ib*b.x));out.t=a.t;
    return true;
}

class Fusion {
public:
    struct Observation {FusionSourceV1 source;Gaussian estimate;};
    struct Settings {
        double maximumAgeS=12.; ///< matches the existing default delivered-track lifetime; explicit engineering policy
        double processNoise=100.; ///< declared continuation diffusion, not equipment calibration
        double gateSquared=16.811893829770927; ///< chi2.ppf(.99,6), frozen association rule
        std::size_t capacity=512;
        std::size_t sourcesPerEntity=64;
    };
    Fusion()=default;
    explicit Fusion(Settings s):settings_(s) {
        if(!std::isfinite(s.maximumAgeS)||s.maximumAgeS<=0||!std::isfinite(s.processNoise)||s.processNoise<0||
           !std::isfinite(s.gateSquared)||s.gateSquared<=0||s.capacity==0||s.sourcesPerEntity==0)throw std::invalid_argument("Invalid fusion policy");
    }
    void advance(double now,std::vector<Observation> observations) {
        if(!std::isfinite(now)||now<now_)throw std::invalid_argument("Fusion clock moved backwards");
        now_=now;
        // A bounded lost-record tail preserves explicit loss transitions.
        // Release expired retention before enforcing this tick's admission
        // capacity. Otherwise an already expired row can refuse a new source.
        rows_.erase(std::remove_if(rows_.begin(),rows_.end(),[&](const auto& r){return now_-r.lastReceived>2*settings_.maximumAgeS;}),rows_.end());
        std::stable_sort(observations.begin(),observations.end(),[](const auto& a,const auto& b){return key(a.source)<key(b.source);});
        for(const auto& observation:observations)ingest(observation);
    }
    std::vector<FusedEntityV1> snapshot() const {
        std::vector<FusedEntityV1> result;
        for(const auto& row:rows_) {
            FusedEntityV1 e;e.id=row.id;e.timeS=now_;e.lastUpdateS=row.lastSample;e.validUntilS=row.lastReceived+settings_.maximumAgeS;
            e.validUntilS=std::min(e.validUntilS,row.lastSample+settings_.maximumAgeS);
            e.state=now_>e.validUntilS?FusionStateV1::Lost:row.updatedAt==now_?FusionStateV1::Live:FusionStateV1::Coasting;
            const auto g=project(row.estimate,now_);
            std::copy(g.x.a.begin(),g.x.a.end(),e.stateEcef);std::copy(g.p.a.begin(),g.p.a.end(),e.covarianceEcef);
            for(const auto& s:row.sources)e.sources.push_back(s.second);
            result.push_back(std::move(e));
        }
        return result;
    }
    std::uint64_t rejected()const noexcept{return rejected_;}
    std::uint64_t capacityRefusals()const noexcept{return capacityRefusals_;}

private:
    using Key=std::tuple<std::uint32_t,std::uint32_t,std::uint32_t,std::uint64_t>;
    static Key key(const FusionSourceV1& s){return {static_cast<std::uint32_t>(s.kind),s.system,s.payload,s.track};}
    static bool sameSensor(const FusionSourceV1& a,const FusionSourceV1& b){return a.kind==b.kind&&a.system==b.system&&a.payload==b.payload;}
    struct Row {
        std::uint64_t id=0;Gaussian estimate;double lastSample=0,lastReceived=0,updatedAt=-1;
        std::map<Key,FusionSourceV1> sources;
    };
    Gaussian project(const Gaussian& g,double t) const {ConstantVelocity m;for(double& q:m.q)q=settings_.processNoise;return predict(g,t,m);}
    double distance(const Gaussian& a,const Gaussian& b)const {
        Covar inverseSum;
        // Var(A-B)<=2(Pa+Pb) for any unknown cross-correlation: conservative
        // association covariance rather than invented independent errors.
        if(!inverse(2.*(a.p+b.p),inverseSum))return std::numeric_limits<double>::infinity();
        const auto d=a.x-b.x;return (transpose(d)*inverseSum*d).a[0];
    }
    void ingest(const Observation& o) {
        const auto& s=o.source;
        if(!s.track || !std::isfinite(s.sampleTimeS)||!std::isfinite(s.receivedTimeS)||
           s.sampleTimeS>s.receivedTimeS || s.receivedTimeS>now_ || o.estimate.t>now_ ||
           !std::isfinite(o.estimate.t) || o.estimate.t<s.sampleTimeS || now_-s.sampleTimeS>settings_.maximumAgeS ||
           !std::isfinite(logDet(o.estimate.p)) ||
           std::any_of(o.estimate.x.a.begin(),o.estimate.x.a.end(),[](double v){return !std::isfinite(v);})) {++rejected_;return;}
        const Key k=key(s);const auto observed=project(o.estimate,now_);Row* best=nullptr;
        Row* owner=nullptr;
        double newest=-std::numeric_limits<double>::infinity();
        for(auto& r:rows_) {
            auto found=r.sources.find(k);
            if(found==r.sources.end())continue;
            if(found->second.sampleTimeS>newest)owner=&r,newest=found->second.sampleTimeS;
        }
        // A sensor can recycle a track number while the old entity coasts.
        // Retain its admitted historical provenance and route later reports to
        // the newest source epoch. Refused admission never edits an old row.
        if(s.sampleTimeS<=newest)return;
        if(owner&&distance(project(owner->estimate,now_),observed)<=settings_.gateSquared)best=owner;
        if(!best) {
            double nearest=settings_.gateSquared;
            for(auto& r:rows_) {
                if(now_>std::min(r.lastReceived,r.lastSample)+settings_.maximumAgeS)continue;
                bool conflict=false;
                for(const auto& source:r.sources)
                    if(sameSensor(source.second,s)&&key(source.second)!=k && now_-source.second.sampleTimeS<=settings_.maximumAgeS)conflict=true;
                if(conflict)continue;
                const double d=distance(project(r.estimate,now_),observed);
                if(d<nearest)nearest=d,best=&r;
            }
        }
        if(!best) {
            if(rows_.size()>=settings_.capacity){++capacityRefusals_;return;}
            Row r;r.id=nextId_++;r.estimate=observed;rows_.push_back(std::move(r));best=&rows_.back();
        } else {
            // Retain recent contributor provenance; reclaim expired history
            // only when a new live contributor needs its bounded slot. A live
            // entity must not become permanently full of dead source keys.
            const bool reclaim=!best->sources.count(k)&&best->sources.size()>=settings_.sourcesPerEntity;
            auto expired=[&](const auto& source){return now_-source.second.sampleTimeS>settings_.maximumAgeS;};
            if(reclaim&&static_cast<std::size_t>(std::count_if(best->sources.begin(),best->sources.end(),
                [&](const auto& source){return !expired(source);}))>=settings_.sourcesPerEntity){++capacityRefusals_;return;}
            Gaussian combined;
            if(!intersect(project(best->estimate,now_),observed,combined)){++rejected_;return;}
            if(reclaim)for(auto it=best->sources.begin();it!=best->sources.end();) {
                if(expired(*it))it=best->sources.erase(it);else ++it;
            }
            best->estimate=combined;
        }
        best->sources[k]=s;best->lastSample=std::max(best->lastSample,s.sampleTimeS);
        best->lastReceived=std::max(best->lastReceived,s.receivedTimeS);best->updatedAt=now_;
    }
    Settings settings_;
    std::vector<Row> rows_;
    std::uint64_t nextId_=1,rejected_=0,capacityRefusals_=0;
    double now_=0;
};

} // namespace fsim::ms::track
