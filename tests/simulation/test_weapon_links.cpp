#include "simulation/mission_systems/weapon/WeaponLink.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace fsim::ms;
using namespace fsim::ms::weapon;
namespace {
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
FusedEntityV1 observation(double sample=0) {
    FusedEntityV1 e;e.id=7;e.state=FusionStateV1::Live;e.timeS=sample;
    e.lastUpdateS=sample;e.validUntilS=10;
    for(int i=0;i<6;++i)e.covarianceEcef[6*i+i]=1;
    FusionSourceV1 source;source.track=3;source.sampleTimeS=sample;source.receivedTimeS=sample;
    e.sources.push_back(source);return e;
}
void finite_delivery() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::TwoWay;
    p.latencyS=.25;p.dataRateBps=8000;p.lossProbability=0;p.maximumAgeS=3;
    WeaponChannelV1 c(p,123);WeaponLinkPathV1 path;path.distanceM=0;
    const auto input=observation();
    check(c.sendInput(input,0,path),"An admitted fused input must enter transport");
    c.advance(.249);check(!c.input(),"Queued input cannot refresh guidance before delivery");
    c.advance(1);check(c.input().has_value(),"Finite transport must deliver");
    check(c.input()->sources[0].sampleTimeS==0,"Delivery must preserve the original measurement epoch");
    check(std::fabs(c.inputReceivedS()-.682)<1e-12,"432-byte input at 8000 bit/s plus .25 s must deliver at .682 s");
    check(c.stats().bytes==432&&c.stats().sent==1&&c.stats().delivered==1,"Canonical input byte accounting is incorrect");
    check(!c.sendInput(input,1,path),"A repeated retained observation cannot renew its epoch");
    auto older=input;older.lastUpdateS=-1;
    check(!c.sendInput(older,1,path),"An older input must be rejected");
    c.advance(4);check(!c.input(),"An expired delivered input cannot remain guidance support");
}
void assessment_delivery() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::TwoWay;p.latencyS=.25;
    WeaponChannelV1 c(p,1);WeaponAssessmentRequestV1 q;q.requestId=11;q.durationS=10;
    const auto receipt=c.assess(q,0);
    check(receipt.accepted,"A supported assessment must be accepted");
    check(c.assess(q,0).assessment==receipt.assessment,"Identical request retry must be idempotent");
    q.durationS=9;check(!c.assess(q,0).accepted,"Changed-body request reuse must be refused");
    WeaponFeedbackV1 f;f.sampleTimeS=0;f.endOfFlight=true;
    WeaponLinkPathV1 path;
    check(!c.sample(f,0,path),"Assessment command must reach the weapon before publication");
    c.advance(1);f.sampleTimeS=1;
    check(c.sample(f,1,path),"Onboard final report must enter return transport");
    check(c.feedback(receipt.assessment).empty(),"Undelivered final report is not actor knowledge");
    c.advance(2);
    check(c.status(receipt.assessment).state==WeaponAssessmentStateV1::EndOfFlight,"Delivered terminal report must close assessment");
    check(c.feedback(receipt.assessment).size()==1,"Final report must survive onboard removal");
    check(std::fabs(c.feedback(receipt.assessment)[0].receivedTimeS-1.314)<1e-12,"512-byte return report plus .25 s must deliver at 1.314 s");
    check(c.stats().bytes==576,"Assessment command and covariance-bearing report must charge all bytes");
    check(c.feedback(receipt.assessment).size()==1,"Report reads must not consume");
}
void unsupported_loss_and_cancel() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::Uplink;
    WeaponChannelV1 one(p,2);WeaponAssessmentRequestV1 q;q.requestId=1;
    check(!one.assess(q,0).accepted,"One-way equipment cannot acquire a return transmitter");
    p.direction=WeaponLinkDirectionV1::TwoWay;p.lossProbability=1;
    WeaponChannelV1 loss(p,3);const auto receipt=loss.assess(q,0);
    WeaponFeedbackV1 f;WeaponLinkPathV1 path;
    check(!loss.sample(f,0,path),"Total modeled loss must suppress delivery");loss.advance(1);
    check(loss.feedback(receipt.assessment).empty(),"Loss cannot create a report");
    check(loss.cancel(receipt.assessment),"Owner cancellation must close an assessment");
    check(loss.status(receipt.assessment).state==WeaponAssessmentStateV1::Canceled,"Canceled status must be explicit");
}
void invalid_and_explicit_loss() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::Uplink;p.maximumAgeS=10;
    WeaponChannelV1 c(p,4);WeaponLinkPathV1 path;
    auto e=observation();e.covarianceEcef[0]=-1;
    check(!c.sendInput(e,0,path),"Invalid covariance must refuse before mutation");
    e=observation();e.sources[0].receivedTimeS=1;
    check(!c.sendInput(e,0,path),"Future delivery provenance must refuse");
    e=observation();check(c.sendInput(e,0,path),"Valid input after refusal must still be admitted");
    c.advance(1);check(c.input().has_value(),"Valid input did not arrive");
    auto foreign=observation(1);foreign.id=8;
    check(!c.sendInput(foreign,1,path),"A different fused target cannot silently rebind the channel");
    e.state=FusionStateV1::Lost;
    check(c.sendInput(e,1,path),"Explicit loss with original epoch must enter transport");
    c.advance(2);check(!c.input(),"Delivered explicit loss must stop support");
}
void assessment_timeouts_and_expiry() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::TwoWay;p.maximumAgeS=.5;p.assessmentTimeoutS=2;
    WeaponAssessmentRequestV1 q;q.requestId=1;q.durationS=10;
    WeaponChannelV1 c(p,5);const auto r=c.assess(q,0);
    c.advance(1);check(c.status(r.assessment).state==WeaponAssessmentStateV1::SoftLost,"Missing reports must become soft lost");
    c.advance(3);check(c.status(r.assessment).state==WeaponAssessmentStateV1::Lost,"Communication timeout must stop publication");
    WeaponChannelV1 expiry(p,6);q.durationS=.3;const auto expired=expiry.assess(q,0);expiry.advance(1);
    check(expiry.status(expired.assessment).state==WeaponAssessmentStateV1::Expired,"Requested lifetime must expire independently");
}
void cancellation_and_finite_capacity() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::TwoWay;p.assessmentCapacity=1;
    WeaponAssessmentRequestV1 q;q.requestId=1;
    WeaponChannelV1 c(p,7);const auto r=c.assess(q,0);c.advance(1);
    WeaponFeedbackV1 f;f.sampleTimeS=1;WeaponLinkPathV1 path;
    check(c.sample(f,1,path),"Active onboard assessment must sample");
    check(c.cancel(r.assessment),"Cancellation must succeed before return delivery");c.advance(2);
    check(c.feedback(r.assessment).empty(),"Cancellation cannot admit a queued report");
    q.requestId=2;check(c.assess(q,2).reason==WeaponLinkReasonV1::Capacity,"Full retained assessment capacity must refuse");
    q.requestId=1;check(c.assess(q,2).assessment==r.assessment,"Capacity must preserve accepted retry identity");
}
void shared_byte_resource_and_path() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::Uplink;p.dataRateBps=8000;p.maximumQueueS=0;
    WeaponChannelV1 a(p,8),b(p,9);WeaponLinkResourceV1 resource;WeaponLinkPathV1 path;
    check(a.sendInput(observation(),0,path,&resource),"First launcher packet must reserve byte resource");
    check(!b.sendInput(observation(),0,path,&resource),"Another weapon must share launcher serialization capacity");
    check(b.stats().queueRefused==1,"Resource refusal needs native accounting");
    WeaponChannelV1 masked(p,10);path.clear=false;
    check(!masked.sendInput(observation(),0,path),"Masked path cannot deliver");
    path.clear=true;path.distanceM=p.rangeM+1;
    check(!masked.sendInput(observation(),0,path),"Out-of-range path cannot deliver");
}
void deterministic_seeded_loss() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::Uplink;p.lossProbability=.5;p.maximumAgeS=10;
    WeaponChannelV1 a(p,11),b(p,11);WeaponLinkPathV1 path;
    for(int i=0;i<20;++i){const double t=i;auto e=observation(t);e.validUntilS=t+10;
        check(a.sendInput(e,t,path)==b.sendInput(e,t,path),"Seeded loss must be independent of scheduling");a.advance(t+1);b.advance(t+1);}
    check(a.stats().sent==b.stats().sent&&a.stats().lost==b.stats().lost,"Seeded accounting differs");
    check(a.stats().lost>0&&a.stats().delivered>0,"Loss fixture must exercise both delivery outcomes");
}
void soft_loss_publication_cannot_reopen() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::TwoWay;p.maximumAgeS=.5;p.assessmentTimeoutS=3;
    WeaponChannelV1 c(p,12);WeaponAssessmentRequestV1 q;q.requestId=1;
    const auto r=c.assess(q,0);c.advance(.2);
    WeaponFeedbackV1 f;f.sampleTimeS=.2;WeaponLinkPathV1 path;
    check(c.sample(f,.2,path),"Delivered assessment command must activate onboard emission");c.advance(.4);
    check(c.status(r.assessment).state==WeaponAssessmentStateV1::Active,"Initial report must publish");
    const auto count=c.feedback(r.assessment).size();c.advance(1);
    check(c.status(r.assessment).state==WeaponAssessmentStateV1::SoftLost,"Freshness loss must be explicit");
    f.sampleTimeS=1;c.sample(f,1,path);c.advance(1.2);
    check(c.status(r.assessment).state==WeaponAssessmentStateV1::SoftLost,"Late reports cannot reopen SOFT_LOST publication");
    check(c.feedback(r.assessment).size()==count,"SOFT_LOST must stop assessment publication");
}
void cancellation_is_a_modeled_command() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::TwoWay;
    WeaponChannelV1 c(p,13);WeaponAssessmentRequestV1 q;q.requestId=1;
    const auto r=c.assess(q,0);c.advance(1);WeaponFeedbackV1 f;f.sampleTimeS=1;WeaponLinkPathV1 path;
    check(c.sample(f,1,path),"Onboard publication must start after request delivery");
    const auto bytes=c.stats().bytes;check(c.cancel(r.assessment),"Local cancellation must be admitted");
    check(c.stats().bytes==bytes+64,"Cancel must traverse finite uplink and charge its command bytes");
    check(c.needsSample(1.01,true),"Onboard emitter cannot know receiver cancellation before command delivery");
    c.advance(2);
    check(!c.needsSample(2,true),"Delivered cancel command must stop onboard emission");
    check(c.feedback(r.assessment).empty(),"Canceled subscription cannot publish in-flight old reports");
}
void batched_receivers_preserve_physical_epochs() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::Uplink;p.maximumAgeS=.5;
    WeaponChannelV1 frequent(p,14),batched(p,14);WeaponLinkPathV1 path;
    check(frequent.sendInput(observation(),0,path)&&batched.sendInput(observation(),0,path),"Both transports must admit identical input");
    frequent.advance(.2);frequent.advance(1);batched.advance(1);
    check(frequent.stats().delivered==1&&batched.stats().delivered==1,"A late read cannot retroactively erase an on-time physical delivery");
    check(!frequent.input()&&!batched.input(),"An aged delivery must remain unavailable to both readers");
    p.direction=WeaponLinkDirectionV1::TwoWay;p.assessmentTimeoutS=3;
    WeaponChannelV1 first(p,15),second(p,15);WeaponAssessmentRequestV1 q;q.requestId=1;
    const auto one=first.assess(q,0),two=second.assess(q,0);first.advance(.2);second.advance(.2);
    WeaponFeedbackV1 report;report.sampleTimeS=.2;
    check(first.sample(report,.2,path)&&second.sample(report,.2,path),"Initial onboard samples must send");
    first.advance(.4);second.advance(.4);report.sampleTimeS=2;
    first.advance(1);
    check(first.sample(report,2,path)&&second.sample(report,2,path),"Onboard emission is independent of receiver read schedule");
    first.advance(2.2);second.advance(2.2);
    check(first.feedback(one.assessment).size()==1&&second.feedback(two.assessment).size()==1,"Batching cannot heal a gap that closed publication");
    check(first.status(one.assessment).state==WeaponAssessmentStateV1::SoftLost&&second.status(two.assessment).state==WeaponAssessmentStateV1::SoftLost,"Receiver publication clocks must agree");
}
void jitter_reordering_never_regresses_input() {
    WeaponLinkProfileV1 p;p.direction=WeaponLinkDirectionV1::Uplink;p.maximumAgeS=10;p.latencyS=.2;p.jitterS=.19;
    // Deliberate synthetic network fault, not an admitted equipment jitter.
    WeaponChannelV1 channel(p,14);WeaponLinkPathV1 path;
    for(int i=0;i<12;++i){const double time=i*.01;check(channel.sendInput(observation(time),time,path),"Synthetic reorder fixture must admit each input");}
    channel.advance(2);const auto latest=channel.input();
    check(latest&&std::fabs(latest->lastUpdateS-.11)<1e-12,"A reordered late packet cannot regress accepted measurement time");
    check(channel.stats().rejected>0,"The deterministic fixture must actually exercise reordered rejection");
}

}
int main(){
    const std::pair<const char*,void(*)()> cases[]={
        {"finite input delivery",finite_delivery},{"assessment terminal delivery",assessment_delivery},
        {"unsupported/loss/cancel",unsupported_loss_and_cancel},{"input validation/loss",invalid_and_explicit_loss},
        {"assessment timeout/expiry",assessment_timeouts_and_expiry},{"cancel/capacity",cancellation_and_finite_capacity},
        {"shared resource/path",shared_byte_resource_and_path},{"seeded loss",deterministic_seeded_loss},
        {"soft-loss cessation",soft_loss_publication_cannot_reopen},{"finite cancellation",cancellation_is_a_modeled_command},
        {"batched physical epochs",batched_receivers_preserve_physical_epochs},{"jitter reordering",jitter_reordering_never_regresses_input}};
    std::size_t passed=0;
    for(const auto& test:cases){try{test.second();++passed;}catch(const std::exception& e){std::cerr<<test.first<<": "<<e.what()<<'\n';}}
    std::cout<<"weapon links: "<<passed<<"/"<<std::size(cases)<<" cases passed\n";
    return passed==std::size(cases)?0:1;
}
