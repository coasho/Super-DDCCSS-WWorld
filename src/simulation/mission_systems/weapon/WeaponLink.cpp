#include "WeaponLink.h"
#include <cmath>
#include <stdexcept>

namespace fsim::ms::weapon {
WeaponChannelV1::WeaponChannelV1(WeaponLinkProfileV1 p,std::uint64_t seed):profile_(std::move(p)),rng_(seed) {
    const double positive[]={profile_.updatePeriodS,profile_.feedbackPeriodS,profile_.dataRateBps,
        profile_.rangeM,profile_.frequencyHz,profile_.maximumAgeS,profile_.assessmentTimeoutS,profile_.retentionS};
    for(double x:positive)if(!std::isfinite(x)||x<=0)throw std::invalid_argument("Invalid weapon-link policy");
    if(!std::isfinite(profile_.latencyS)||profile_.latencyS<0||!std::isfinite(profile_.maximumQueueS)||profile_.maximumQueueS<0||
       !std::isfinite(profile_.jitterS)||profile_.jitterS<0||
       static_cast<std::uint32_t>(profile_.direction)>static_cast<std::uint32_t>(WeaponLinkDirectionV1::TwoWay)||
       !std::isfinite(profile_.lossProbability)||profile_.lossProbability<0||profile_.lossProbability>1||
       !profile_.queueCapacity||!profile_.reportCapacity||!profile_.assessmentCapacity)
        throw std::invalid_argument("Invalid weapon-link capacity/loss policy");
}
bool WeaponChannelV1::valid(const FusedEntityV1& e,double now) {
    if(!e.id||e.sources.empty()||e.sources.size()>64||!std::isfinite(e.timeS)||!std::isfinite(e.lastUpdateS)||
       !std::isfinite(e.validUntilS)||e.lastUpdateS>e.timeS||e.timeS>now||e.validUntilS<e.lastUpdateS)return false;
    if(e.state!=FusionStateV1::Live&&e.state!=FusionStateV1::Coasting&&e.state!=FusionStateV1::Lost)return false;
    if(e.state!=FusionStateV1::Lost&&now>e.validUntilS)return false;
    for(double x:e.stateEcef)if(!std::isfinite(x))return false;
    for(double x:e.covarianceEcef)if(!std::isfinite(x))return false;
    // Cholesky rejects non-positive and asymmetric covariance before mutation.
    double l[36]={};
    for(int i=0;i<6;++i)for(int j=0;j<=i;++j) {
        if(std::fabs(e.covarianceEcef[6*i+j]-e.covarianceEcef[6*j+i])>1e-8*std::max(1.,std::fabs(e.covarianceEcef[6*i+i])))return false;
        double x=e.covarianceEcef[6*i+j];for(int k=0;k<j;++k)x-=l[6*i+k]*l[6*j+k];
        if(i==j){if(!(x>0))return false;l[6*i+j]=std::sqrt(x);}else l[6*i+j]=x/l[6*j+j];
    }
    for(const auto& s:e.sources)if(!s.track||!std::isfinite(s.sampleTimeS)||!std::isfinite(s.receivedTimeS)||
        s.sampleTimeS>s.receivedTimeS||s.receivedTimeS>e.timeS||s.sampleTimeS>e.lastUpdateS||
        (s.kind!=FusionSourceKindV1::LocalRadar&&s.kind!=FusionSourceKindV1::DeliveredLink)||
        (s.kind==FusionSourceKindV1::DeliveredLink&&!s.system))return false;
    return true;
}
bool WeaponChannelV1::transmit(std::size_t bytes,double now,const WeaponLinkPathV1& path,bool returning,double& due,WeaponLinkResourceV1* shared) {
    if(!std::isfinite(now)||now<now_||!std::isfinite(path.distanceM)||path.distanceM<0){++stats_.rejected;return false;}
    if(!path.clear){++stats_.masked;return false;}
    if(path.distanceM>profile_.rangeM){++stats_.outOfRange;return false;}
    auto& resource=shared?*shared:returning?return_:uplink_;
    const double start=std::max(now,resource.busyUntilS),duration=8.*static_cast<double>(bytes)/profile_.dataRateBps;
    if(!std::isfinite(start)||!std::isfinite(duration)||
       !std::isfinite(start+duration+profile_.latencyS+profile_.jitterS+path.distanceM/299792458.)) {
        ++stats_.rejected;return false;
    }
    if(packets_.size()>=profile_.queueCapacity||start-now>profile_.maximumQueueS){++stats_.queueRefused;return false;}
    resource.busyUntilS=start+duration;++stats_.sent;stats_.bytes+=bytes;
    if(rng_.uniform()<profile_.lossProbability){++stats_.lost;return false;}
    const double jitter=profile_.jitterS>0?rng_.uniform(-profile_.jitterS,profile_.jitterS):0;
    due=resource.busyUntilS+std::max(0.,profile_.latencyS+jitter)+path.distanceM/299792458.;return true;
}
bool WeaponChannelV1::sendInput(const FusedEntityV1& e,double now,const WeaponLinkPathV1& path,WeaponLinkResourceV1* shared) {
    if(profile_.direction==WeaponLinkDirectionV1::Unsupported||!valid(e,now)||(boundFusedId_&&e.id!=boundFusedId_)||e.lastUpdateS<lastSentSample_||
       (e.lastUpdateS==lastSentSample_&&(lastSentLost_||e.state!=FusionStateV1::Lost))){++stats_.rejected;return false;}
    double due;
    // Fixed canonical binary payload sizes, including full covariance and each provenance tuple.
    if(!transmit(384+48*e.sources.size(),now,path,false,due,shared))return false;
    boundFusedId_=e.id;lastSentSample_=e.lastUpdateS;lastSentLost_=e.state==FusionStateV1::Lost;
    packets_.push_back({due,e,std::nullopt});return true;
}
void WeaponChannelV1::advanceStates(double at) {
    for(auto& pair:assessments_) {
        auto& status=pair.second.status;
        if(status.state==WeaponAssessmentStateV1::Canceled||status.state==WeaponAssessmentStateV1::EndOfFlight||
           status.state==WeaponAssessmentStateV1::Lost||status.state==WeaponAssessmentStateV1::Expired)continue;
        const double age=at-status.lastReceivedTimeS.value_or(status.requestedTimeS);
        if(at>status.expiresTimeS)status.state=WeaponAssessmentStateV1::Expired,status.reason=WeaponLinkReasonV1::Expired;
        else if(age>profile_.assessmentTimeoutS)status.state=WeaponAssessmentStateV1::Lost;
        else if(age>profile_.maximumAgeS)status.state=WeaponAssessmentStateV1::SoftLost;
    }
}
void WeaponChannelV1::advance(double now) {
    if(!std::isfinite(now)||now<now_)throw std::invalid_argument("Weapon-link clock moved backwards");
    std::stable_sort(packets_.begin(),packets_.end(),[](const auto& a,const auto& b){return a.due<b.due;});
    while(!packets_.empty()&&packets_.front().due<=now) {
        const auto packet=std::move(packets_.front());packets_.pop_front();
        // Apply receiver deadlines at the physical packet epoch, not only at
        // this caller's later read/decision boundary. Batching cannot heal loss.
        advanceStates(packet.due);
        if(packet.input) {
            const auto& entity=*packet.input;
            if(entity.state!=FusionStateV1::Lost&&
               (packet.due>entity.validUntilS||packet.due-entity.lastUpdateS>profile_.maximumAgeS)) {
                ++stats_.expired;continue;
            }
            if(input_&&(entity.lastUpdateS<input_->lastUpdateS||
               (entity.lastUpdateS==input_->lastUpdateS&&(input_->state==FusionStateV1::Lost||entity.state!=FusionStateV1::Lost)))) {
                ++stats_.rejected;continue;
            }
            input_=entity;inputReceived_=packet.due;++stats_.delivered;
        } else if(packet.assessmentStart) {
            const auto it=assessments_.find(packet.assessmentStart);
            if(it!=assessments_.end()&&!it->second.cancelDelivered&&packet.due<=it->second.status.expiresTimeS)
                it->second.onboard=true;
            ++stats_.delivered;
        } else if(packet.assessmentCancel) {
            const auto it=assessments_.find(packet.assessmentCancel);
            if(it!=assessments_.end()){it->second.cancelDelivered=true;it->second.onboard=false;}
            ++stats_.delivered;
        } else if(packet.feedback) {
            const auto& feedback=*packet.feedback;
            const auto it=assessments_.find(feedback.assessment);
            if(it==assessments_.end())continue;
            auto& assessment=it->second;const auto state=assessment.status.state;
            if(state==WeaponAssessmentStateV1::Canceled||state==WeaponAssessmentStateV1::SoftLost||
               state==WeaponAssessmentStateV1::Lost||state==WeaponAssessmentStateV1::Expired||
               state==WeaponAssessmentStateV1::EndOfFlight||packet.due>assessment.status.expiresTimeS||packet.due>feedback.validUntilS) {
                ++stats_.expired;continue;
            }
            if(!assessment.reports.empty()&&feedback.sequence<=assessment.reports.back().sequence){++stats_.rejected;continue;}
            auto received=feedback;received.receivedTimeS=packet.due;
            assessment.reports.push_back(received);
            if(assessment.reports.size()>profile_.reportCapacity)assessment.reports.pop_front();
            assessment.status.lastReceivedTimeS=packet.due;
            assessment.status.state=feedback.endOfFlight?WeaponAssessmentStateV1::EndOfFlight:WeaponAssessmentStateV1::Active;
            ++stats_.delivered;
        }
    }
    advanceStates(now);now_=now;
}

std::optional<FusedEntityV1> WeaponChannelV1::input()const {
    if(!input_||input_->state==FusionStateV1::Lost||now_>input_->validUntilS||now_-input_->lastUpdateS>profile_.maximumAgeS)return std::nullopt;
    return input_;
}
WeaponAssessmentReceiptV1 WeaponChannelV1::assess(const WeaponAssessmentRequestV1& q,double now,const WeaponLinkPathV1& path,WeaponLinkResourceV1* shared) {
    if(const auto previous=retry(q))return *previous;
    WeaponAssessmentReceiptV1 r;r.requestId=q.requestId;
    if(profile_.direction!=WeaponLinkDirectionV1::TwoWay)r.reason=WeaponLinkReasonV1::Unsupported;
    else if(!q.requestId||!std::isfinite(now)||now<now_||!std::isfinite(q.periodS)||q.periodS<profile_.feedbackPeriodS||
            !std::isfinite(q.durationS)||q.durationS<=0||q.durationS>3600)r.reason=WeaponLinkReasonV1::Invalid;
    else if(assessments_.size()>=profile_.assessmentCapacity||requests_.size()>=4*profile_.assessmentCapacity)r.reason=WeaponLinkReasonV1::Capacity;
    else {
        r.accepted=true;r.assessment=nextAssessment_++;r.reason=WeaponLinkReasonV1::None;
        Assessment a;a.request=q;a.nextSample=now;
        a.status.assessment=r.assessment;a.status.requestId=q.requestId;a.status.reason=WeaponLinkReasonV1::None;
        a.status.requestedTimeS=now;a.status.expiresTimeS=now+q.durationS;
        assessments_.emplace(r.assessment,std::move(a));
        double due;
        if(transmit(64,now,path,false,due,shared))packets_.push_back({due,std::nullopt,std::nullopt,r.assessment});
    }
    // Refusals are bounded too. Full capacity never evicts an accepted request and re-executes its retry.
    if(requests_.size()<4*profile_.assessmentCapacity)requests_.emplace(q.requestId,std::make_pair(q,r));
    return r;
}
std::optional<WeaponAssessmentReceiptV1> WeaponChannelV1::retry(const WeaponAssessmentRequestV1& q)const {
    const auto old=requests_.find(q.requestId);
    if(old==requests_.end())return std::nullopt;
    const auto& p=old->second.first;
    if(p.instance==q.instance&&p.weapon==q.weapon&&p.periodS==q.periodS&&p.durationS==q.durationS)return old->second.second;
    return WeaponAssessmentReceiptV1{false,0,q.requestId,WeaponLinkReasonV1::RequestConflict};
}
bool WeaponChannelV1::cancel(std::uint64_t id) {return cancel(id,now_,{});}
bool WeaponChannelV1::cancel(std::uint64_t id,double now,const WeaponLinkPathV1& path,WeaponLinkResourceV1* shared) {
    auto it=assessments_.find(id);if(it==assessments_.end()||!std::isfinite(now)||now<now_)return false;
    auto& status=it->second.status;
    if(status.state==WeaponAssessmentStateV1::Canceled)return true;
    if(status.state==WeaponAssessmentStateV1::EndOfFlight||status.state==WeaponAssessmentStateV1::Lost||status.state==WeaponAssessmentStateV1::Expired)return false;
    // Closing the local subscription is immediate. The remote emitter stops
    // only when its separately transported cancellation command arrives.
    status.state=WeaponAssessmentStateV1::Canceled;
    double due;
    if(transmit(64,now,path,false,due,shared))packets_.push_back({due,std::nullopt,std::nullopt,0,id});
    return true;
}

bool WeaponChannelV1::sample(WeaponFeedbackV1 f,double now,const WeaponLinkPathV1& path) {
    if(!std::isfinite(f.sampleTimeS)||f.sampleTimeS>now||now-f.sampleTimeS>profile_.maximumAgeS)return false;
    if(f.navigationValid) {
        if(!std::isfinite(f.navigationTimeS)||f.navigationTimeS>f.sampleTimeS)return false;
        for(double v:f.stateEcef)if(!std::isfinite(v))return false;
        for(double v:f.navigationCovariance)if(!std::isfinite(v))return false;
    }
    for(const auto x:{f.timeToActiveS,f.timeToInterceptS})if(x&&(!std::isfinite(*x)||*x<0))return false;
    bool sent=false;
    for(auto& pair:assessments_) {
        auto& a=pair.second;
        if(!a.onboard||a.cancelDelivered||a.terminalSampled||now>a.status.expiresTimeS||
           (!f.endOfFlight&&(now<a.nextSample||f.sampleTimeS<=a.lastSample)))continue;
        a.lastSample=f.sampleTimeS;
        a.nextSample=now+a.request.periodS;
        auto report=f;report.assessment=pair.first;report.sequence=++a.sequence;report.sentTimeS=now;
        report.validUntilS=std::min(a.status.expiresTimeS,f.sampleTimeS+profile_.maximumAgeS);
        double due;
        if(transmit(512,now,path,true,due)){packets_.push_back({due,std::nullopt,report});sent=true;}
        if(f.endOfFlight)a.terminalSampled=true;
    }
    return sent;
}
WeaponAssessmentStatusV1 WeaponChannelV1::status(std::uint64_t id)const {
    const auto it=assessments_.find(id);return it==assessments_.end()?WeaponAssessmentStatusV1{}:it->second.status;
}
std::vector<WeaponFeedbackV1> WeaponChannelV1::feedback(std::uint64_t id,std::uint64_t since)const {
    std::vector<WeaponFeedbackV1> out;const auto it=assessments_.find(id);
    if(it!=assessments_.end())for(const auto& r:it->second.reports)if(r.sequence>since)out.push_back(r);
    return out;
}
bool WeaponChannelV1::needsSample(double now,bool terminal)const {
    for(const auto& pair:assessments_) {
        const auto& a=pair.second;
        if(a.onboard&&!a.cancelDelivered&&!a.terminalSampled&&now<=a.status.expiresTimeS&&
           (terminal||now>=a.nextSample))return true;
    }
    return false;
}
bool WeaponChannelV1::finished(double now)const {
    if(!packets_.empty())return false;
    for(const auto& p:assessments_)if(now<=p.second.status.expiresTimeS+profile_.retentionS)return false;
    return true;
}
}
