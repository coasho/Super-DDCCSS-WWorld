#include "simulation/mission_systems/components/implementations/seeker/RadioTemporalCoincidenceV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using namespace fsim::ms;
using namespace components;
radar::signal::Waveform program(){return radar::signal::mediumPrf(10e9,.09,3,3,150,.1);}
RadioTemporalSegmentV1 capturedSegment(unsigned interval,double time,double rate=180) {
    RadioTemporalSegmentV1 segment;segment.programInterval=interval;segment.timeS=time;segment.rateResolutionMs=1;
    segment.waveform=program().intervals.at(interval);
    RadioTemporalPeakV1 peak;const double range=4000+rate*time;
    peak.foldedRangeM=std::fmod(range,segment.waveform.ambiguousRangeM);
    peak.foldedRateMs=std::remainder(rate,segment.waveform.ambiguousRateMs);
    peak.snr=100;peak.angularVarianceRad2=1e-6;
    peak.direction={std::cos(.02+.1*time),std::sin(.02+.1*time),0};peak.refereeAttributionIndex=interval;
    segment.peaks.push_back(peak);return segment;
}
radar::signal::Settings settings(){radar::signal::Settings s;s.rangeScaleM=6000;s.minRateMs=-300;s.maxRateMs=300;s.coincidence=1;return s;}
}

TEST_CASE("Short SARH listening window follows actual long sequential transmitter program", "[weapon-radio-temporal]") {
    auto p=program();const radar::IlluminationClockV1 clock{true,0,.09};
    const auto window=radioPulseWindowV1(p,clock,0,.024,.012);REQUIRE(window);
    REQUIRE(window->waveform.intervals.size()==1);CHECK(window->programIntervals.at(0)==0);
    CHECK(window->waveform.m==3); // Never compress three transmitter PRFs into 12 ms.
    const auto& native=p.intervals.at(0);const auto& heard=window->waveform.intervals.at(0);
    CHECK(heard.prfHz==native.prfHz);CHECK(heard.pulseS==native.pulseS);CHECK(heard.gates==native.gates);
    unsigned oracle=0;
    for(int n=0;n<native.pulses;++n){const double t=n/native.prfHz;if(t>=.012&&t+native.pulseS<=.024)++oracle;}
    CHECK(heard.pulses==static_cast<int>(oracle));
    CHECK(window->integratedTimeS==Catch::Approx(static_cast<double>(oracle)/native.prfHz).epsilon(1e-12));
    CHECK(window->startTimeS.at(0)>=.012);CHECK(window->endTimeS.at(0)<=.024);
    CHECK(window->energyFraction.at(0)==1);
    CHECK(window->waveform.rateResolutionMs==Catch::Approx(p.wavelengthM*native.prfHz/(2*heard.pulses)).epsilon(1e-12));
    const auto first=radioPulseWindowV1(p,clock,.024,.024,.012);REQUIRE(first);CHECK(first->waveform.intervals.empty());
    auto phase=clock;phase.epochS=.09;
    const auto same=radioPulseWindowV1(p,phase,0,.024,.012);REQUIRE(same);
    CHECK(same->startTimeS==window->startTimeS);CHECK(same->endTimeS==window->endTimeS);
    CHECK_FALSE(radioPulseWindowV1(p,{false,0,.09},0,.024,.012));
    CHECK_FALSE(radioPulseWindowV1(p,clock,.025,.024,.012));
    const double edge=200/p.intervals.at(0).prfHz+p.intervals.at(0).pulseS;
    const auto at=radioPulseWindowV1(p,clock,0,edge,.012);
    const auto before=radioPulseWindowV1(p,clock,0,std::nextafter(edge,0.),.012);
    REQUIRE(at);REQUIRE(before);REQUIRE(at->waveform.intervals.size()==1);REQUIRE(before->waveform.intervals.size()==1);
    CHECK(at->waveform.intervals.at(0).pulses==before->waveform.intervals.at(0).pulses+1);
    CHECK(at->endTimeS.at(0)<=edge);CHECK(before->endTimeS.at(0)<edge);
}

TEST_CASE("Temporal SARH coincidence aligns measured migration and bearings without lowering M", "[weapon-radio-temporal]") {
    RadioTemporalCoincidenceV1 tracker;std::vector<RadioTemporalDetectionV1> out;
    REQUIRE(tracker.update(.012,.12,.1,3,settings(),1,{capturedSegment(0,.012)},out));CHECK(out.empty());
    REQUIRE(tracker.update(.042,.12,.1,3,settings(),1,{capturedSegment(1,.042)},out));CHECK(out.empty());
    REQUIRE(tracker.update(.072,.12,.1,3,settings(),1,{capturedSegment(2,.072)},out));REQUIRE(out.size()==1);
    const auto& detection=out.at(0);CHECK(detection.intervals==3);
    CHECK(detection.firstObservationS==.012);CHECK(detection.lastObservationS==.072);
    CHECK(std::atan2(detection.direction.y,detection.direction.x)==Catch::Approx(.0272).margin(1e-7));
    CHECK(detection.directionRate.y==Catch::Approx(.1*std::cos(.0272)).margin(1e-5));
    CHECK(detection.angularVarianceRad2>0);CHECK(detection.refereeAttributionIndex==2);
    REQUIRE_FALSE(detection.hypotheses.empty());
    CHECK(std::any_of(detection.hypotheses.begin(),detection.hypotheses.end(),[](const auto& h){
        return std::abs(h[0]-4012.96)<1e-8&&std::abs(h[1]-180)<1e-8;
    }));
    REQUIRE(tracker.update(.08,.12,.1,3,settings(),1,{},out));CHECK(out.empty()); // No stale delivery renewed as a capture.
}

TEST_CASE("Temporal coincidence rejects stale repeated inconsistent and invalid observations", "[weapon-radio-temporal]") {
    RadioTemporalCoincidenceV1 tracker;std::vector<RadioTemporalDetectionV1> out;
    REQUIRE(tracker.update(.012,.12,.1,3,settings(),1,{capturedSegment(0,.012)},out));
    REQUIRE(tracker.update(.042,.12,.1,3,settings(),1,{capturedSegment(0,.042)},out));CHECK(out.empty());CHECK(tracker.retainedSegments()==1);
    auto invalid=capturedSegment(1,.05);invalid.peaks.at(0).snr=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(tracker.update(.05,.12,.1,3,settings(),1,{invalid},out));CHECK(tracker.retainedSegments()==1);
    REQUIRE(tracker.update(.05,.12,.1,3,settings(),1,{capturedSegment(1,.05)},out));
    auto unrelated=capturedSegment(2,.072);unrelated.peaks.at(0).direction={0,1,0};
    REQUIRE(tracker.update(.072,.12,.1,3,settings(),1,{unrelated},out));CHECK(out.empty());
    auto negative=capturedSegment(0,.08);negative.peaks.clear();
    REQUIRE(tracker.update(.08,.12,.1,3,settings(),1,{negative},out));CHECK(out.empty());
    REQUIRE(tracker.update(.25,.12,.1,3,settings(),1,{capturedSegment(2,.25)},out));CHECK(out.empty());CHECK(tracker.retainedSegments()==1);
    tracker.reset();CHECK(tracker.retainedSegments()==0);
    REQUIRE(tracker.update(.3,.12,.1,3,settings(),1,{capturedSegment(2,.3)},out));CHECK(out.empty());
    auto overflow=capturedSegment(1,.31);overflow.peaks.resize(4097,overflow.peaks.at(0));
    CHECK_FALSE(tracker.update(.31,.12,.1,3,settings(),1,{overflow},out));CHECK(tracker.retainedSegments()==1);
}

TEST_CASE("Temporal association does not consult referee attribution metadata", "[weapon-radio-temporal]") {
    RadioTemporalCoincidenceV1 a,b;std::vector<RadioTemporalDetectionV1> x,y;
    for(unsigned interval=0;interval<3;++interval) {
        const double time=.012+.03*interval;auto one=capturedSegment(interval,time),two=one;
        two.peaks.at(0).refereeAttributionIndex.reset();
        REQUIRE(a.update(time,.12,.1,3,settings(),1,{one},x));
        REQUIRE(b.update(time,.12,.1,3,settings(),1,{two},y));
        REQUIRE(x.size()==y.size());
        for(std::size_t i=0;i<x.size();++i) {
            CHECK(x.at(i).direction.x==y.at(i).direction.x);CHECK(x.at(i).direction.y==y.at(i).direction.y);
            CHECK(x.at(i).snr==y.at(i).snr);CHECK(x.at(i).hypotheses==y.at(i).hypotheses);
            CHECK(x.at(i).angularVarianceRad2==y.at(i).angularVarianceRad2);
        }
    }
    REQUIRE(x.size()==1);CHECK(x.at(0).refereeAttributionIndex==2);CHECK_FALSE(y.at(0).refereeAttributionIndex);
}

TEST_CASE("Pulse listener retains actual native PRF indices beyond a five-PRF own design", "[weapon-radio-temporal]") {
    const auto p=radar::signal::mediumPrf(10e9,.02,32,3,150,.1);
    const auto window=radioPulseWindowV1(p,{true,0,.02},0,.02,.012);REQUIRE(window);
    CHECK(window->waveform.intervals.size()>5);REQUIRE_FALSE(window->programIntervals.empty());
    CHECK(window->programIntervals.back()>5);CHECK(window->programIntervals.back()<32);
    for(std::size_t i=0;i<window->programIntervals.size();++i) {
        const auto original=window->programIntervals.at(i);const auto& iv=p.intervals.at(original);
        CHECK(window->waveform.intervals.at(i).prfHz==iv.prfHz);
        CHECK(window->startTimeS.at(i)>=.008);CHECK(window->endTimeS.at(i)<=.02);
    }
}
