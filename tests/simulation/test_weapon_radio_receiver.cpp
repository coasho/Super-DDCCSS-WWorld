#include "simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#include "simulation/mission_systems/components/implementations/seeker/ActiveRadarSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/SemiActiveRadarSeekerV2.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using namespace fsim::ms;
using namespace fsim::ms::components;
RadioReceiverConfigV1 policy() {
    RadioReceiverConfigV1 c;c.version=1;c.frequencyHz=10e9;c.temperatureK=290;c.noiseFigureDb=6;c.systemLossDb=6;
    c.transmitAveragePowerW=1000;c.bandwidthHz=1e6;c.monopulseSlope=1.5;c.clutterRateSigmaMs=2;c.associationGateRad=.1;
    c.temporalMemoryS=.12;c.observationWindowS=.012;
    c.aperture.shape=rf::ApertureShape::Circular;c.aperture.taper=rf::Taper::Uniform;
    c.aperture.widthM=c.aperture.heightM=.12;c.aperture.efficiency=.6;c.aperture.peakGainDbi=0;
    c.aperture.floorDbi=-30;c.aperture.backDbi=-40;
    c.waveform=radar::signal::mediumPrf(c.frequencyHz,.012,3,3,150,.05);
    c.processing.pfaInterval=1e-6;c.processing.cfarCells=16;c.processing.cfarGuard=2;
    c.processing.coincidence=1;c.processing.rangeScaleM=20000;c.processing.minRateMs=-1000;c.processing.maxRateMs=1000;
    c.processing.notchMs=0;c.processing.knownNoiseThreshold=false;c.processing.reportAmbiguities=true;
    return c;
}
const std::array<weapon::V3,3> axes={{{1,0,0},{0,1,0},{0,0,1}}};
RadioSceneFrameV1 scene(double time=0) {
    RadioSceneFrameV1 s;s.timeS=time;s.frequencyHz=10e9;s.up={0,0,1};
    RadioEchoSceneV1 e;e.direction={std::cos(.02),std::sin(.02),0};e.receiverRangeM=e.delayRangeM=4000;
    e.rcsM2=5;e.dopplerHz=2*200/(299792458./s.frequencyHz);
    s.echoes.push_back(e);s.refereeAttributionIndices.push_back(9);return s;
}
}

TEST_CASE("Physical radio energy uses separate spreading legs and receiver aperture gain", "[weapon-radio-receiver]") {
    auto c=policy();auto e=scene().echoes[0];const double gain=200,duration=.012,lambda=299792458./c.frequencyHz;
    const double reference=c.transmitAveragePowerW*gain*gain*e.rcsM2*lambda*lambda*duration/
        (std::pow(4*std::acos(-1.),3)*std::pow(4000.,4)*1.380649e-23*290*std::pow(10.,1.2));
    CHECK(RadioReceiverV1::energySnr(c,e,gain,c.frequencyHz,duration,false)==Catch::Approx(reference).epsilon(1e-12));
    e.transmitterRangeM=8000;e.transmitterPowerW=2000;e.transmitterGainLinear=1000;
    const double bistatic=RadioReceiverV1::energySnr(c,e,gain,c.frequencyHz,duration,true);
    e.transmitterRangeM*=2;
    CHECK(RadioReceiverV1::energySnr(c,e,gain,c.frequencyHz,duration,true)==Catch::Approx(bistatic/4).epsilon(1e-12));
    rf::Pattern aperture(c.aperture,c.frequencyHz);const double on[3]={1,0,0},off[3]={std::cos(.5),std::sin(.5),0};
    CHECK(aperture.gain(off)<aperture.gain(on)*.1);
}

TEST_CASE("Native dwell supplies measured cells peaks monopulse and ambiguity hypotheses", "[weapon-radio-receiver]") {
    RadioReceiverV1 receiver;auto c=policy();std::string error;REQUIRE(receiver.configure(c,error));
    auto s=scene();auto result=receiver.expose(s,{1,0,0},axes,17,false);
    REQUIRE(result.valid);REQUIRE(result.spectrum.kept);CHECK(result.stats.cells>0);
    CHECK(result.spectrum.power.size()==static_cast<std::size_t>(result.spectrum.rows*result.spectrum.filters));
    CHECK(result.stats.peaks>0);REQUIRE_FALSE(result.measurements.empty());
    const auto target=std::find_if(result.measurements.begin(),result.measurements.end(),[](const auto& m){return m.refereeAttributionIndex==9;});
    REQUIRE(target!=result.measurements.end());CHECK(target->direction.y>0);CHECK(target->angularVarianceRad2>0);
    REQUIRE_FALSE(result.hits.empty());
    CHECK(std::any_of(result.hits.begin(),result.hits.end(),[](const auto& h){
        return std::abs(h.rangeM-4000)<150&&std::abs(h.rangeRateMs+200)<10;
    }));
    for(float value:result.spectrum.power)CHECK((std::isfinite(value)&&value>=0));
}

TEST_CASE("Empty receiver creates actual measurement cells and unowned false peaks", "[weapon-radio-receiver]") {
    RadioReceiverV1 receiver;auto c=policy();c.waveform=radar::signal::mediumPrf(c.frequencyHz,.004,1,1,150,.05);
    c.processing.pfaInterval=.005;c.processing.coincidence=0;c.processing.rangeScaleM=14900;
    c.processing.minRateMs=-149;c.processing.maxRateMs=149;std::string error;REQUIRE(receiver.configure(c,error));
    auto s=scene();s.echoes.clear();s.refereeAttributionIndices.clear();std::uint64_t peaks=0,hits=0;
    for(unsigned i=0;i<8;++i) {
        s.timeS=i*.01;auto result=receiver.expose(s,{1,0,0},axes,47,false);REQUIRE(result.valid);
        CHECK(result.stats.cells>1000);peaks+=result.stats.peaks;hits+=result.hits.size();
        for(const auto& m:result.measurements)CHECK_FALSE(m.refereeAttributionIndex);
        for(const auto& h:result.hits)CHECK(h.truth==0);
    }
    CHECK(peaks>1);CHECK(hits>1);
}

TEST_CASE("Physical radio spherical angles recover diagonal rays in both receiver paths", "[weapon-radio-receiver][radio-diagonal-ray]") {
    // The source direction is constructed independently in a rotated physical
    // frame. Strong test-only illumination isolates the angle convention from
    // detector noise; production cards, thresholds and signal processing stay
    // untouched. Both angular signs must survive the native monopulse channel.
    const std::array<weapon::V3,3> rotated={{{0,1,0},{0,0,1},{1,0,0}}};
    for(const bool semi:{false,true})for(const double az:{-.15,.15})for(const double el:{-.12,.12}) {
        CAPTURE(semi,az,el);
        auto c=policy();c.transmitAveragePowerW=1e7;
        c.waveform=radar::signal::mediumPrf(c.frequencyHz,.004,1,1,150,.05);
        RadioReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
        auto s=scene();auto& echo=s.echoes.at(0);
        const double forward=std::cos(el)*std::cos(az),right=std::cos(el)*std::sin(az),down=-std::sin(el);
        echo.direction={down,forward,right};echo.rcsM2=1e8;echo.fluctuation=0;
        if(semi) {
            s.illuminationAvailable=true;s.illuminationWaveform=c.waveform;
            s.illuminationClock=radar::IlluminationClockV1{true,0,RadioReceiverV1::durationS(c.waveform)};
            SeekerNavigationPoseV1 nav;nav.valid=true;nav.bodyToEcef=rotated;s.navigation=nav;
            echo.transmitterRangeM=8000;echo.transmitterPowerW=1e7;echo.transmitterGainLinear=1000;
            echo.delayRangeM=.5*(echo.receiverRangeM+echo.transmitterRangeM);
            const auto first=receiver.expose(s,rotated[0],rotated,83,true);REQUIRE(first.valid);
            s.timeS=.02;s.navigation->timeS=s.timeS;
        }
        const auto result=receiver.expose(s,rotated[0],rotated,83,semi);REQUIRE(result.valid);
        const RadioMeasurementV1* strongest=nullptr;
        for(const auto& measurement:result.measurements)if(measurement.refereeAttributionIndex==9&&
            (!strongest||measurement.signalToNoise>strongest->signalToNoise))strongest=&measurement;
        REQUIRE(strongest);
        CHECK(strongest->direction.x==Catch::Approx(echo.direction.x).margin(1e-5));
        CHECK(strongest->direction.y==Catch::Approx(echo.direction.y).margin(1e-5));
        CHECK(strongest->direction.z==Catch::Approx(echo.direction.z).margin(1e-5));
        if(semi){CHECK_FALSE(strongest->rangeM);CHECK_FALSE(strongest->rangeRateMs);}
    }
}

TEST_CASE("Ground clutter and modeled interference alter actual received spectrum", "[weapon-radio-receiver]") {
    auto c=policy();RadioReceiverV1 a,b,jammer;std::string error;
    REQUIRE(a.configure(c,error));REQUIRE(b.configure(c,error));REQUIRE(jammer.configure(c,error));
    auto clean=scene();auto ground=clean;auto patch=ground.echoes[0];patch.rcsM2=100000;patch.dopplerHz=0;
    ground.groundPatches.push_back(patch);auto ra=a.expose(clean,{1,0,0},axes,6,false),rb=b.expose(ground,{1,0,0},axes,6,false);
    REQUIRE(ra.valid);REQUIRE(rb.valid);
    const auto total=[](const auto& r){double value=0;for(float p:r.spectrum.power)value+=static_cast<double>(p);return value;};
    CHECK(total(rb)>total(ra)*2);
    clean.interferencePowerW=1e-7;auto rj=jammer.expose(clean,{1,0,0},axes,6,false);REQUIRE(rj.valid);
    CHECK(total(rj)>total(ra)*100);
    CHECK(std::none_of(rj.measurements.begin(),rj.measurements.end(),[](const auto& m){return m.refereeAttributionIndex==9;}));
}

TEST_CASE("SARH measures Doppler cells but cannot publish Cartesian range", "[weapon-radio-receiver]") {
    auto c=policy();RadioReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto s=scene();s.illuminationAvailable=true;s.illuminationWaveform=c.waveform;
    s.illuminationClock=radar::IlluminationClockV1{true,0,RadioReceiverV1::durationS(c.waveform)};
    SeekerNavigationPoseV1 nav;nav.valid=true;nav.bodyToEcef=axes;s.navigation=nav;
    auto& e=s.echoes[0];e.transmitterRangeM=8000;e.transmitterPowerW=5000;e.transmitterGainLinear=1000;
    e.delayRangeM=(e.receiverRangeM+e.transmitterRangeM)/2;
    auto result=receiver.expose(s,{1,0,0},axes,18,true);REQUIRE(result.valid);
    CHECK(result.measurements.empty()); // No pre-observed transmitter history.
    s.timeS=.02;s.navigation->timeS=s.timeS;
    result=receiver.expose(s,{1,0,0},axes,18,true);REQUIRE(result.valid);REQUIRE_FALSE(result.measurements.empty());
    for(const auto& m:result.measurements){CHECK_FALSE(m.rangeM);CHECK_FALSE(m.rangeRateMs);}
    receiver.reset();s.illuminationAvailable=false;s.illuminationWaveform.reset();
    auto off=receiver.expose(s,{1,0,0},axes,18,true);REQUIRE(off.valid);
    for(const auto& m:off.measurements)CHECK_FALSE(m.refereeAttributionIndex);
}

TEST_CASE("Radio source-order reset and invalid-policy controls retain physics and RNG", "[weapon-radio-receiver]") {
    auto c=policy();RadioReceiverV1 a,b;std::string error;REQUIRE(a.configure(c,error));REQUIRE(b.configure(c,error));
    auto s=scene();auto second=s.echoes[0];second.receiverRangeM=second.delayRangeM=5500;second.direction={1,0,0};
    s.echoes.push_back(second);s.refereeAttributionIndices.push_back(10);
    auto reversed=s;std::reverse(reversed.echoes.begin(),reversed.echoes.end());std::reverse(reversed.refereeAttributionIndices.begin(),reversed.refereeAttributionIndices.end());
    auto first=a.expose(s,{1,0,0},axes,4,false),other=b.expose(reversed,{1,0,0},axes,4,false);
    REQUIRE(first.valid);CHECK(first.spectrum.power==other.spectrum.power);
    CHECK(a.expose(s,{1,0,0},axes,900,false).spectrum.power==first.spectrum.power);
    a.reset();CHECK(a.expose(s,{1,0,0},axes,4,false).spectrum.power==first.spectrum.power);
    auto invalid=c;invalid.waveform.intervals[0].pulses=257;CHECK_FALSE(a.configure(invalid,error));
    auto duplicate=c.waveform;duplicate.intervals.at(1)=duplicate.intervals.at(0);
    CHECK_FALSE(RadioReceiverV1::validIlluminationProgram(duplicate));
    invalid=c;invalid.processing.knownNoiseThreshold=true;CHECK_FALSE(a.configure(invalid,error));
    auto bad=s;bad.timeS=.02;bad.echoes[0].rcsM2=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(a.expose(bad,{1,0,0},axes,4,false).valid);
    s.timeS=.02;reversed.timeS=.02;
    CHECK(a.expose(s,{1,0,0},axes,4,false).spectrum.power==b.expose(reversed,{1,0,0},axes,4,false).spectrum.power);
}

TEST_CASE("Navigation-coupled RF wrapper preserves actual spectrum and changes reconstructed LOS", "[weapon-radio-receiver]") {
    weapon::Spec spec;spec.seeker="active_radar";spec.executionVersion=2;
    spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::ActiveRadar;spec.seekerModel.engineering.version=3;
    EngineeringSeekerV3 a(spec,std::make_unique<ActiveRadarSeekerV2>(spec,std::nullopt)),b(spec,std::make_unique<ActiveRadarSeekerV2>(spec,std::nullopt));
    std::string error;REQUIRE(a.configureRadioV1(policy(),error));REQUIRE(b.configureRadioV1(policy(),error));
    SeekerFrameV2 f;f.up={0,0,1};f.boresight={1,0,0};f.bodyAxes=axes;f.velocity={250,0,0};f.dtS=.01;f.receiverSeed=17;
    auto x=scene(),y=x;SeekerNavigationPoseV1 nav;nav.valid=true;nav.velocity=f.velocity;nav.bodyToEcef=axes;
    x.navigation=nav;nav.bodyToEcef={{{std::cos(.15),std::sin(.15),0},{-std::sin(.15),std::cos(.15),0},{0,0,1}}};y.navigation=nav;
    a.sampleRadioV1(f,x);b.sampleRadioV1(f,y);
    CHECK_FALSE(a.radioSpectrumV1());CHECK_FALSE(b.radioSpectrumV1());
    f.timeS=f.ageS=.02;x.timeS=y.timeS=.02;x.navigation->timeS=y.navigation->timeS=.02;
    a.sampleRadioV1(f,x);b.sampleRadioV1(f,y);
    const auto* spectrumA=a.radioSpectrumV1();const auto* spectrumB=b.radioSpectrumV1();
    REQUIRE(spectrumA);REQUIRE(spectrumB);if(!spectrumA||!spectrumB)return;
    CHECK(spectrumA->spectrum.power==spectrumB->spectrum.power);
    f.timeS=f.ageS=.03;x.timeS=y.timeS=.03;x.navigation->timeS=y.navigation->timeS=.03;
    auto ra=a.sampleRadioV1(f,x),rb=b.sampleRadioV1(f,y);REQUIRE(ra.track);REQUIRE(rb.track);
    CHECK(std::atan2(rb.track->direction.y,rb.track->direction.x)-std::atan2(ra.track->direction.y,ra.track->direction.x)==Catch::Approx(.15).margin(1e-10));
    y.navigation.reset();CHECK_FALSE(b.sampleRadioV1(f,y).valid);
}

TEST_CASE("Opt-in RF diagnostic planes preserve native dwell samples and decisions", "[weapon-radio-receiver]") {
    auto c=policy();radar::signal::Echo echo;echo.rangeM=4000;echo.rangeRateMs=-200;
    echo.snr=10000;echo.fluctuation=0;echo.monopulseAz=.1;echo.truth=3;
    const std::vector<radar::signal::Echo> echoes{echo};radar::signal::Stream oldStream(91),newStream(91);
    radar::signal::Snapshot oldSnapshot,newSnapshot;radar::signal::Stats oldStats,newStats;
    radar::signal::CellDiagnosticsV1 diagnostic;
    const auto oldHits=radar::signal::dwell(c.waveform,echoes,c.processing,oldStream,&oldStats,nullptr,&oldSnapshot);
    const auto newHits=radar::signal::dwellWithDiagnosticsV1(c.waveform,echoes,c.processing,newStream,&newStats,nullptr,&newSnapshot,&diagnostic);
    CHECK(oldSnapshot.power==newSnapshot.power);CHECK(oldSnapshot.peaks==newSnapshot.peaks);
    CHECK(oldHits.size()==newHits.size());CHECK(oldStats.cells==newStats.cells);
    CHECK(oldStream.next()==newStream.next());REQUIRE(diagnostic.kept);
    REQUIRE(diagnostic.tested.size()==newSnapshot.power.size());
    REQUIRE(diagnostic.decisionThresholdRelative.size()==newSnapshot.power.size());
    for(std::size_t i=0;i<oldHits.size();++i) {
        CHECK(oldHits[i].rangeM==newHits[i].rangeM);CHECK(oldHits[i].rangeRateMs==newHits[i].rangeRateMs);
        CHECK(oldHits[i].monopulseAz==newHits[i].monopulseAz);CHECK(oldHits[i].truth==newHits[i].truth);
    }
    for(const auto& peak:newSnapshot.peaks) {
        const auto index=static_cast<std::size_t>(peak.first)*static_cast<std::size_t>(newSnapshot.filters)+static_cast<std::size_t>(peak.second);
        REQUIRE(index<diagnostic.tested.size());CHECK(diagnostic.tested.at(index)==1);
        CHECK(newSnapshot.power.at(index)>diagnostic.decisionThresholdRelative.at(index));
        CHECK(diagnostic.localNoiseRelative.at(index)>0.f);
    }
}

TEST_CASE("SARH long native dwell acquires by real temporal PRF coincidence and finite delivery", "[weapon-radio-receiver]") {
    auto c=policy();auto s=scene();s.illuminationAvailable=true;
    s.illuminationWaveform=radar::signal::mediumPrf(c.frequencyHz,.09,3,3,150,.1);
    s.illuminationClock=radar::IlluminationClockV1{true,0,.09};
    SeekerNavigationPoseV1 nav;nav.valid=true;nav.bodyToEcef=axes;nav.velocity={250,0,0};s.navigation=nav;
    auto& echo=s.echoes.at(0);echo.transmitterRangeM=8000;echo.transmitterPowerW=5000;echo.transmitterGainLinear=1000;echo.delayRangeM=6000;
    REQUIRE(RadioReceiverV1::durationS(*s.illuminationWaveform)>.08);
    REQUIRE(s.illuminationWaveform->intervals.at(0).pulses>256);
    weapon::Spec spec;spec.seeker="semi_active_radar";spec.executionVersion=2;
    spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::SemiActiveRadar;spec.seekerModel.engineering.version=3;
    EngineeringSeekerV3 receiver(spec,std::make_unique<SemiActiveRadarSeekerV2>(spec));
    std::string error;REQUIRE(receiver.configureRadioV1(c,error));
    SeekerFrameV2 frame;frame.up=s.up;frame.boresight={1,0,0};frame.bodyAxes=axes;frame.velocity=nav.velocity;
    frame.dtS=.01;frame.receiverSeed=18;frame.illuminated=true;frame.configuredIlluminator=7;
    bool acquired=false,coincided=false;double firstTrack=-1;
    for(unsigned i=0;i<=18;++i) {
        frame.timeS=frame.ageS=i*.01;s.timeS=frame.timeS;s.navigation->timeS=s.timeS;
        // A constant physical path-rate over the finite observation history.
        echo.delayRangeM=6000-200*s.timeS;
        const auto delivered=receiver.sampleRadioV1(frame,s);
        const auto* raw=receiver.radioSpectrumV1();REQUIRE(raw);if(!raw)return;
        CHECK(raw->valid);CHECK(raw->waveform.m==3);
        CHECK(raw->waveform.intervals.size()<3); // No compressed transmitter switching.
        if(raw->spectrum.kept){CHECK(raw->observationEndTimeS<=frame.timeS);CHECK(raw->observationEndTimeS-raw->observationStartTimeS<=.012+1e-12);}
        if(raw->coincidenceIntervals>=3) {
            coincided=true;CHECK(raw->coincidenceLastTimeS-raw->coincidenceFirstTimeS<=c.temporalMemoryS);
            for(const auto& measured:raw->measurements){CHECK_FALSE(measured.rangeM);CHECK_FALSE(measured.rangeRateMs);CHECK(measured.navigationReconstructed);}
        }
        if(delivered.tracking) {
            acquired=true;if(firstTrack<0)firstTrack=frame.timeS;
            REQUIRE(delivered.measurementTimeS);REQUIRE(delivered.processingReadyTimeS);REQUIRE(delivered.deliveredTimeS);
            CHECK(*delivered.processingReadyTimeS==Catch::Approx(*delivered.measurementTimeS+.01).margin(1e-12));
            CHECK(*delivered.deliveredTimeS>=*delivered.processingReadyTimeS);
            REQUIRE(delivered.track);CHECK_FALSE(delivered.track->rangeM);CHECK_FALSE(delivered.track->rangeRateMs);
        }
    }
    CHECK(coincided);CHECK(acquired);CHECK(firstTrack>=.07);
    // A phase discontinuity starts a new observation history, even though the
    // transmitter still has the same power, PRFs, program and scene geometry.
    frame.timeS=frame.ageS=.20;s.timeS=.20;s.navigation->timeS=.20;s.illuminationClock->epochS=.001;
    receiver.sampleRadioV1(frame,s);const auto* changed=receiver.radioSpectrumV1();REQUIRE(changed);if(!changed)return;
    CHECK(changed->measurements.empty());CHECK_FALSE(changed->spectrum.kept);
}

TEST_CASE("Observed native intervals use declared energy shares without altering the old path", "[weapon-radio-receiver]") {
    auto c=policy();auto w=c.waveform;w.intervals.resize(1);w.m=1;
    radar::signal::Echo echo;echo.rangeM=4000;echo.rangeRateMs=-200;echo.snr=10000;echo.fluctuation=0;
    radar::signal::Stream reference(91),observed(91);radar::signal::Snapshot a,b;
    radar::signal::DwellObservationV1 observation;observation.intervalEnergyFraction={1};observation.intervalTimeS={.04};
    const auto original=radar::signal::dwell(w,{echo},c.processing,reference,nullptr,nullptr,&a);
    REQUIRE(radar::signal::dwellObservedV1(w,{echo},c.processing,observed,nullptr,nullptr,&b,nullptr,observation));
    CHECK(a.power==b.power);CHECK(a.peaks==b.peaks);CHECK(reference.next()==observed.next());
    REQUIRE_FALSE(original.empty());REQUIRE_FALSE(observation.peaks.empty());
    for(const auto& p:observation.peaks){CHECK(p.timeS==.04);CHECK(p.interval==0);CHECK(p.snr>0);}
    const auto before=observed;observation.intervalEnergyFraction={.5};
    CHECK_FALSE(radar::signal::dwellObservedV1(w,{echo},c.processing,observed,nullptr,nullptr,&b,nullptr,observation));
    auto copy=before;CHECK(copy.next()==observed.next());
}

TEST_CASE("Observed unequal PRF energy matches independent native single-interval processing", "[weapon-radio-receiver]") {
    auto c=policy();auto w=c.waveform;w.intervals.resize(2);
    radar::signal::Echo echo;echo.rangeM=4000;echo.rangeRateMs=-200;echo.snr=10000;echo.fluctuation=0;
    radar::signal::Stream reference(72),observed(72);radar::signal::Snapshot expected,actual;
    for(std::size_t j=0;j<2;++j) {
        auto single=w;single.intervals={w.intervals.at(j)};single.m=1;
        auto portion=echo;portion.snr*=j==0?.25:.75;
        (void)radar::signal::dwell(single,{portion},c.processing,reference,nullptr,nullptr,&expected);
    }
    actual.interval=1;
    radar::signal::DwellObservationV1 input;input.intervalEnergyFraction={.25,.75};input.intervalTimeS={.04,.04};
    REQUIRE(radar::signal::dwellObservedV1(w,{echo},c.processing,observed,nullptr,nullptr,&actual,nullptr,input));
    CHECK(expected.power==actual.power);CHECK(expected.peaks==actual.peaks);CHECK(reference.next()==observed.next());
}

TEST_CASE("SARH listens to native eight-PRF programs without expanding its own design or cells", "[weapon-radio-receiver]") {
    auto c=policy();auto native=radar::signal::mediumPrf(c.frequencyHz,.09,8,3,150,.1);
    REQUIRE(RadioReceiverV1::validIlluminationProgram(native));
    auto wrongOwn=c;wrongOwn.waveform=native;CHECK_FALSE(RadioReceiverV1::validConfig(wrongOwn));
    RadioReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto s=scene();s.illuminationAvailable=true;s.illuminationWaveform=native;
    s.illuminationClock=radar::IlluminationClockV1{true,0,.09};
    SeekerNavigationPoseV1 nav;nav.valid=true;nav.bodyToEcef=axes;s.navigation=nav;
    auto& echo=s.echoes.at(0);echo.transmitterRangeM=8000;echo.transmitterPowerW=5000;echo.transmitterGainLinear=1000;
    bool detected=false,measuredCells=false;
    for(unsigned i=0;i<=9;++i) {
        s.timeS=i*.02;s.navigation->timeS=s.timeS;echo.delayRangeM=6000-200*s.timeS;
        const auto result=receiver.expose(s,{1,0,0},axes,27,true);REQUIRE(result.valid);
        CHECK(result.waveform.m==3);CHECK(result.stats.cells<=131072);
        if(result.spectrum.kept)measuredCells=true;
        if(!result.measurements.empty()) {
            detected=true;CHECK(result.coincidenceIntervals>=3);
            for(const auto& m:result.measurements){CHECK_FALSE(m.rangeM);CHECK_FALSE(m.rangeRateMs);}
        }
    }
    CHECK(measuredCells);CHECK(detected);
}

TEST_CASE("ARH standby preserves authorized launch estimate and consumes no receiver energy or noise", "[weapon-radio-receiver]") {
    weapon::Spec spec;spec.seeker="active_radar";spec.executionVersion=2;spec.acquireM=10000;spec.activationRatio=.5;
    spec.launcherUpdates=true;spec.inertialMidcourse=true;
    spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::ActiveRadar;spec.seekerModel.engineering.version=3;
    weapon::Observation far;far.position={20000,0,0};weapon::Observation near;near.position={4000,0,0};
    EngineeringSeekerV3 held(spec,std::make_unique<ActiveRadarSeekerV2>(spec,far));
    EngineeringSeekerV3 immediate(spec,std::make_unique<ActiveRadarSeekerV2>(spec,near));
    std::string error;REQUIRE(held.configureRadioV1(policy(),error));REQUIRE(immediate.configureRadioV1(policy(),error));
    SeekerFrameV2 frame;frame.up={0,0,1};frame.boresight={1,0,0};frame.bodyAxes=axes;frame.velocity={250,0,0};
    frame.dtS=.01;frame.receiverSeed=57;auto physical=scene();
    SeekerNavigationPoseV1 nav;nav.valid=true;nav.bodyToEcef=axes;nav.velocity=frame.velocity;physical.navigation=nav;
    const auto sample=[&](EngineeringSeekerV3& receiver,double time) {
        frame.timeS=frame.ageS=time;physical.timeS=time;physical.navigation->timeS=time;
        return receiver.sampleRadioV1(frame,physical);
    };
    for(unsigned i=0;i<=10;++i) {
        const auto result=sample(held,i*.01);REQUIRE(result.valid);
        CHECK_FALSE(result.active);CHECK_FALSE(result.tracking);CHECK_FALSE(result.measurementTimeS);
        CHECK_FALSE(held.radioSpectrumV1());
        REQUIRE(result.midcourseEstimate);CHECK(result.midcourseEstimate->position.x==20000);
    }
    frame.supported=true;frame.launcherEstimate=LauncherEstimateV2{{4000,0,0},{0,0,0},.12};
    const auto activation=sample(held,.12);REQUIRE(activation.valid);CHECK(activation.active);
    CHECK_FALSE(held.radioSpectrumV1());CHECK_FALSE(activation.measurementTimeS);
    sample(held,.13);CHECK_FALSE(held.radioSpectrumV1());
    const auto capturing=sample(held,.14);CHECK_FALSE(capturing.tracking);
    const auto* actual=held.radioSpectrumV1();REQUIRE(actual);if(!actual)return;
    CHECK(actual->stats.cells>0);CHECK(actual->observationStartTimeS>=.12);
    const auto power=actual->spectrum.power;
    const auto delivered=sample(held,.15);REQUIRE(delivered.tracking);REQUIRE(delivered.measurementTimeS);
    REQUIRE(delivered.processingReadyTimeS);REQUIRE(delivered.deliveredTimeS);
    CHECK(*delivered.measurementTimeS==Catch::Approx(.14).margin(1e-12));
    CHECK(*delivered.processingReadyTimeS==Catch::Approx(.15).margin(1e-12));
    CHECK(*delivered.deliveredTimeS==Catch::Approx(.15).margin(1e-12));
    frame.supported=false;frame.launcherEstimate.reset();
    sample(immediate,0);CHECK_FALSE(immediate.radioSpectrumV1());
    sample(immediate,.01);CHECK_FALSE(immediate.radioSpectrumV1());
    sample(immediate,.02);const auto* control=immediate.radioSpectrumV1();REQUIRE(control);if(!control)return;
    CHECK(power==control->spectrum.power); // Standby did not consume any RF random draws.
}
