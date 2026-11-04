#include "RadioReceiverV1.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>
#include <chrono>

namespace fsim::ms::components {
using namespace measured;
namespace {
constexpr double pi=3.14159265358979323846,light=299792458.,boltzmann=1.380649e-23;
bool positive(double x){return std::isfinite(x)&&x>0;}
bool nonnegative(double x){return std::isfinite(x)&&x>=0;}
double linear(double db){return std::pow(10.,db/10.);}
bool close(double a,double b){return std::abs(a-b)<=1e-6*std::max(std::abs(b),1.);}
struct Beam {V forward,right,down;};
Beam beamFor(V pointing,const std::array<V,3>& axes) {
    Beam b;b.forward=unit(pointing);b.right=unit(subtract(axes[1],scale(b.forward,dot(axes[1],b.forward))));
    if(norm(b.right)<.5)b.right=unit(cross(axes[2],b.forward));
    b.down=unit(cross(b.forward,b.right));return b;
}
V measuredBeamRay(const Beam& beam,double az,double el) {
    // The difference channels encode azimuth in the forward/right plane and
    // elevation above that plane. Invert those spherical coordinates in
    // this physical beam before any navigation-frame reconstruction.
    return unit(add(scale(beam.forward,std::cos(el)*std::cos(az)),
        add(scale(beam.right,std::cos(el)*std::sin(az)),scale(beam.down,-std::sin(el)))));
}
auto physicalKey(const RadioEchoSceneV1& e) {
    return std::tuple{e.direction.x,e.direction.y,e.direction.z,e.receiverRangeM,e.transmitterRangeM,e.delayRangeM,
        e.dopplerHz,e.rcsM2,e.receiverExcessLossDb,e.transmitterExcessLossDb,e.transmitterPowerW,e.transmitterGainLinear,e.fluctuation};
}
bool sameProgram(const radar::signal::Waveform& a,const radar::signal::Waveform& b) {
    if(a.m!=b.m||a.gateM!=b.gateM||a.wavelengthM!=b.wavelengthM||a.intervals.size()!=b.intervals.size())return false;
    for(std::size_t i=0;i<a.intervals.size();++i) {
        const auto& x=a.intervals[i];const auto& y=b.intervals[i];
        if(std::tie(x.prfHz,x.pulses,x.gates,x.blankedGates,x.ambiguousRangeM,x.ambiguousRateMs,x.pulseS)!=
            std::tie(y.prfHz,y.pulses,y.gates,y.blankedGates,y.ambiguousRangeM,y.ambiguousRateMs,y.pulseS))return false;
    }
    return true;
}
V navigationRay(V ray,const std::array<V,3>& actual,const SeekerNavigationPoseV1& nav) {
    V result{};for(unsigned i=0;i<3;++i)result=add(result,scale(nav.bodyToEcef[i],dot(actual[i],ray)));return unit(result);
}
}
double RadioReceiverV1::durationS(const radar::signal::Waveform& w) noexcept {
    double result=0;for(const auto& interval:w.intervals)result+=interval.pulses/interval.prfHz;return result;
}
static bool validWaveformDomainV1(const radar::signal::Waveform& w,const radar::signal::Settings& p,std::size_t maximumIntervals) noexcept {
    if(w.intervals.empty()||w.intervals.size()>maximumIntervals||w.m<1||static_cast<std::size_t>(w.m)>w.intervals.size()||
        !positive(w.gateM)||w.gateM<.01||w.gateM>5000||!positive(w.rateResolutionMs)||w.rateResolutionMs<1e-6||!positive(w.wavelengthM)||
        !positive(p.rangeScaleM)||p.rangeScaleM>500000||!std::isfinite(p.minRateMs)||!std::isfinite(p.maxRateMs)||
        p.minRateMs>=p.maxRateMs||std::abs(p.minRateMs)>10000||std::abs(p.maxRateMs)>10000||
        !positive(p.pfaInterval)||p.pfaInterval>=.1||p.cfarCells<4||p.cfarCells>62||p.cfarCells%2||
        p.cfarGuard<0||p.cfarGuard>8||p.coincidence<0||p.coincidence>2||p.knownNoiseThreshold||
        !nonnegative(p.notchMs)||p.notchMs>1000)return false;
    std::uint64_t cells=0;
    for(const auto& iv:w.intervals) {
        if(!positive(iv.prfHz)||iv.prfHz>1e7||iv.pulses<4||iv.pulses>256||iv.gates<8||iv.gates>2048||
            iv.blankedGates<0||iv.blankedGates>iv.gates-8||!positive(iv.pulseS)||iv.pulseS*iv.prfHz>=1||
            !positive(iv.ambiguousRangeM)||!positive(iv.ambiguousRateMs)||
            !close(iv.ambiguousRangeM,light/(2*iv.prfHz))||!close(iv.ambiguousRateMs,w.wavelengthM*iv.prfHz/2))return false;
        const double expectedGates=std::floor(iv.ambiguousRangeM/w.gateM);
        const double expectedBlank=std::ceil(light*iv.pulseS/(2*w.gateM));
        if(!close(iv.gates,expectedGates)||!close(iv.blankedGates,expectedBlank))return false;
        cells+=static_cast<std::uint64_t>(iv.gates-iv.blankedGates)*static_cast<std::uint64_t>(iv.pulses);
    }
    const double grid=std::ceil(p.rangeScaleM/w.gateM)*std::ceil((p.maxRateMs-p.minRateMs)/w.rateResolutionMs)*
        std::pow(2*p.coincidence+1,2);
    return cells<=131072&&std::isfinite(grid)&&grid<=64.*1024*1024&&positive(RadioReceiverV1::durationS(w))&&RadioReceiverV1::durationS(w)<=.25;
}
bool RadioReceiverV1::validWaveform(const radar::signal::Waveform& w,const radar::signal::Settings& p) noexcept {return validWaveformDomainV1(w,p,5);}
bool RadioReceiverV1::validObservedWaveform(const radar::signal::Waveform& w,const radar::signal::Settings& p) noexcept {return validWaveformDomainV1(w,p,32);}
bool RadioReceiverV1::validConfig(const RadioReceiverConfigV1& c) noexcept {
    const auto& a=c.aperture;
    const bool taper=a.shape==rf::ApertureShape::Rectangular?
        a.taper==rf::Taper::Uniform||a.taper==rf::Taper::Cosine||a.taper==rf::Taper::CosineSquared||a.taper==rf::Taper::Taylor:
        a.shape==rf::ApertureShape::Circular&&(a.taper==rf::Taper::Uniform||a.taper==rf::Taper::Parabolic||a.taper==rf::Taper::ParabolicSquared);
    return c.version==1&&positive(c.frequencyHz)&&c.frequencyHz>=1e6&&c.frequencyHz<=1e12&&
        positive(c.temperatureK)&&c.temperatureK>=1&&c.temperatureK<=2000&&nonnegative(c.noiseFigureDb)&&c.noiseFigureDb<=100&&
        nonnegative(c.systemLossDb)&&c.systemLossDb<=100&&positive(c.transmitAveragePowerW)&&c.transmitAveragePowerW<=1e7&&
        positive(c.bandwidthHz)&&c.bandwidthHz>=1&&c.bandwidthHz<=1e10&&positive(c.monopulseSlope)&&c.monopulseSlope>=1e-6&&c.monopulseSlope<=100&&
        positive(c.clutterRateSigmaMs)&&c.clutterRateSigmaMs>=1e-6&&c.clutterRateSigmaMs<=1000&&positive(c.associationGateRad)&&c.associationGateRad<1.5&&
        positive(c.temporalMemoryS)&&c.temporalMemoryS<=5&&
        positive(c.observationWindowS)&&c.observationWindowS<=.25&&
        taper&&positive(a.widthM)&&a.widthM<=2&&positive(a.heightM)&&a.heightM<=2&&
        nonnegative(a.pedestal)&&a.pedestal<=1&&positive(a.efficiency)&&a.efficiency<=1&&
        positive(a.taylorSidelobeDb)&&a.taylorSidelobeDb<=100&&a.taylorNbar>=2&&a.taylorNbar<=16&&
        std::isfinite(a.peakGainDbi)&&a.peakGainDbi<=70&&std::isfinite(a.floorDbi)&&a.floorDbi>=-150&&a.floorDbi<=30&&
        std::isfinite(a.backDbi)&&a.backDbi>=-150&&a.backDbi<=30&&nonnegative(a.scanExponent)&&a.scanExponent<=10&&
        validWaveform(c.waveform,c.processing)&&close(c.waveform.wavelengthM,light/c.frequencyHz);
}
bool RadioReceiverV1::validIlluminationProgram(const radar::signal::Waveform& w) noexcept {
    if(w.intervals.empty()||w.intervals.size()>32||!positive(w.wavelengthM)||!positive(w.gateM)||
        w.m<1||static_cast<std::size_t>(w.m)>w.intervals.size())return false;
    for(const auto& iv:w.intervals) {
        if(!positive(iv.prfHz)||iv.prfHz>1e7||iv.pulses<4||iv.pulses>65536||iv.gates<8||iv.gates>2048||
            iv.blankedGates<0||iv.blankedGates>iv.gates-8||!positive(iv.pulseS)||iv.pulseS*iv.prfHz>=1||
            !close(iv.ambiguousRangeM,light/(2*iv.prfHz))||!close(iv.ambiguousRateMs,w.wavelengthM*iv.prfHz/2)||
            !close(iv.gates,std::floor(iv.ambiguousRangeM/w.gateM))||
            !close(iv.blankedGates,std::ceil(light*iv.pulseS/(2*w.gateM))))return false;
    }
    for(std::size_t i=0;i<w.intervals.size();++i)for(std::size_t j=0;j<i;++j)
        if(w.intervals[i].prfHz==w.intervals[j].prfHz)return false;
    return positive(durationS(w))&&durationS(w)<=5;
}
bool RadioReceiverV1::validScene(const RadioSceneFrameV1& s,const RadioReceiverConfigV1& c,bool semi) noexcept {
    if(!nonnegative(s.timeS)||!positive(s.frequencyHz)||s.frequencyHz<1e6||s.frequencyHz>1e12||
        (!semi&&!close(s.frequencyHz,c.frequencyHz))||
        !nonnegative(s.interferencePowerW)||s.interferencePowerW>1e7||!finite(s.up)||std::abs(norm(s.up)-1)>1e-6||
        s.echoes.size()>128||s.groundPatches.size()>512||s.refereeAttributionIndices.size()!=s.echoes.size()||
        (s.mainBeamClutterRateMs&&!std::isfinite(*s.mainBeamClutterRateMs))||
        (s.altitudeM&&(!nonnegative(*s.altitudeM)||*s.altitudeM>1e6)))return false;
    if(semi&&s.illuminationAvailable&&(!s.illuminationWaveform||!validIlluminationProgram(*s.illuminationWaveform)||
        !close(s.illuminationWaveform->wavelengthM,light/s.frequencyHz)||!s.illuminationClock||!s.illuminationClock->illuminating||
        !std::isfinite(s.illuminationClock->epochS)||s.illuminationClock->epochS<0||
        !positive(s.illuminationClock->periodS)||s.illuminationClock->periodS>5||
        !s.navigation||!s.navigation->valid||s.navigation->timeS!=s.timeS||!finite(s.navigation->position)||!finite(s.navigation->velocity)))return false;
    if(semi&&s.illuminationAvailable) {
        const auto& axes=s.navigation->bodyToEcef;
        for(const auto axis:axes)if(!finite(axis)||std::abs(norm(axis)-1)>1e-6)return false;
        if(std::abs(dot(axes[0],axes[1]))>1e-6||norm(subtract(cross(axes[0],axes[1]),axes[2]))>1e-6)return false;
    }
    const auto valid=[&](const auto& e){return finite(e.direction)&&std::abs(norm(e.direction)-1)<=1e-6&&
        positive(e.receiverRangeM)&&nonnegative(e.delayRangeM)&&std::isfinite(e.dopplerHz)&&std::abs(e.dopplerHz)<=1e8&&
        nonnegative(e.rcsM2)&&e.rcsM2<=1e12&&nonnegative(e.receiverExcessLossDb)&&e.receiverExcessLossDb<=1000&&
        nonnegative(e.transmitterExcessLossDb)&&e.transmitterExcessLossDb<=1000&&e.fluctuation<=2&&
        nonnegative(e.transmitterRangeM)&&nonnegative(e.transmitterPowerW)&&nonnegative(e.transmitterGainLinear)&&
        (!semi||!s.illuminationAvailable||(positive(e.transmitterRangeM)&&positive(e.transmitterPowerW)&&positive(e.transmitterGainLinear)));};
    return std::all_of(s.echoes.begin(),s.echoes.end(),valid)&&std::all_of(s.groundPatches.begin(),s.groundPatches.end(),valid);
}
double RadioReceiverV1::energySnr(const RadioReceiverConfigV1& c,const RadioEchoSceneV1& e,
    double receiveGain,double frequency,double dwell,bool semi) noexcept {
    if(!positive(receiveGain)||!positive(frequency)||!positive(dwell)||!positive(e.receiverRangeM)||!positive(e.rcsM2))return 0;
    const double txRange=semi?e.transmitterRangeM:e.receiverRangeM;
    const double txGain=semi?e.transmitterGainLinear:receiveGain;
    const double power=semi?e.transmitterPowerW:c.transmitAveragePowerW;
    if(!positive(txRange)||!positive(txGain)||!positive(power))return 0;
    const double wavelength=light/frequency;
    const double loss=c.systemLossDb+e.receiverExcessLossDb+(semi?e.transmitterExcessLossDb:e.receiverExcessLossDb);
    const double value=power*txGain*receiveGain*e.rcsM2*wavelength*wavelength*dwell/
        (std::pow(4*pi,3)*txRange*txRange*e.receiverRangeM*e.receiverRangeM*linear(loss)*
         boltzmann*c.temperatureK*linear(c.noiseFigureDb));
    return nonnegative(value)?value:std::numeric_limits<double>::quiet_NaN();
}
bool RadioReceiverV1::configure(const RadioReceiverConfigV1& c,std::string& error) {
    if(!validConfig(c)){error="invalid explicit pulse-domain RF receiver policy";return false;}
    rf::Pattern pattern(c.aperture,c.frequencyHz);
    if(!positive(pattern.beamwidthRad(false))||!positive(pattern.beamwidthRad(true))||
        pattern.beamwidthRad(false)>=pi/2||pattern.beamwidthRad(true)>=pi/2){error="invalid RF aperture pattern";return false;}
    config_=c;pattern_=std::move(pattern);patternFrequencyHz_=c.frequencyHz;reset();return true;
}
void RadioReceiverV1::reset() noexcept {
    lastTimeS_=-1;seeded_=false;cached_={};temporal_.reset();illuminationProgram_.reset();illuminationClock_.reset();
    firstIlluminationS_=lastIlluminationS_=-1;
}
void RadioReceiverV1::observeIlluminationV1(const RadioSceneFrameV1& scene) {
    if(!scene.illuminationAvailable) {
        temporal_.reset();illuminationProgram_.reset();illuminationClock_.reset();firstIlluminationS_=lastIlluminationS_=-1;return;
    }
    if(!validScene(scene,config_,true)||scene.timeS<lastIlluminationS_)return;
    bool changed=!illuminationProgram_||!illuminationClock_||!sameProgram(*illuminationProgram_,*scene.illuminationWaveform);
    if(!changed) {
        const auto& old=*illuminationClock_;const auto& current=*scene.illuminationClock;
        const double cycles=(current.epochS-old.epochS)/current.periodS;
        const double residual=current.epochS-old.epochS-std::round(cycles)*current.periodS;
        // Native Radar::step accepts/clamps equality within 1e-12 s. This
        // tolerance identifies the same scheduler phase; pulse endpoints
        // themselves still use exact causal inequalities in the cropper.
        changed=current.periodS!=old.periodS||std::abs(residual)>1e-12+
            32*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(current.epochS),std::abs(old.epochS)})||
            scene.timeS-lastIlluminationS_>config_.temporalMemoryS;
    }
    if(changed){temporal_.reset();firstIlluminationS_=scene.timeS;illuminationProgram_=scene.illuminationWaveform;}
    illuminationClock_=scene.illuminationClock;lastIlluminationS_=scene.timeS;
}
RadioReceiverResultV1 RadioReceiverV1::expose(const RadioSceneFrameV1& scene,V pointing,
    const std::array<V,3>& axes,std::uint64_t seed,bool semi) {
    const auto& c=config_;
    if(!validConfig(c)||!validScene(scene,c,semi)||scene.timeS<lastTimeS_||!finite(pointing)||!positive(norm(pointing)))return {};
    for(const auto axis:axes)if(!finite(axis)||std::abs(norm(axis)-1)>1e-6)return {};
    if(std::abs(dot(axes[0],axes[1]))>1e-6||norm(subtract(cross(axes[0],axes[1]),axes[2]))>1e-6)return {};
    if(scene.timeS==lastTimeS_)return cached_;
    if(semi)observeIlluminationV1(scene);
    if(patternFrequencyHz_!=scene.frequencyHz){pattern_=rf::Pattern(c.aperture,scene.frequencyHz);patternFrequencyHz_=scene.frequencyHz;}
    std::optional<RadioPulseWindowV1> observed;
    if(semi&&scene.illuminationAvailable) {
        observed=radioPulseWindowV1(*scene.illuminationWaveform,*scene.illuminationClock,firstIlluminationS_,scene.timeS,c.observationWindowS);
        if(!observed)return {};
        if(observed->waveform.intervals.empty()) {
            RadioReceiverResultV1 result;result.valid=true;result.semiActive=true;result.timeS=scene.timeS;
            result.pointing=pointing;result.config=c;result.waveform=observed->waveform;
            result.observationStartTimeS=observed->observationStartTimeS;result.observationEndTimeS=scene.timeS;
            lastTimeS_=scene.timeS;cached_=result;return result;
        }
    }
    auto waveform=observed?observed->waveform:c.waveform;
    const double wavelength=light/scene.frequencyHz,ratio=wavelength/waveform.wavelengthM;
    waveform.wavelengthM=wavelength;waveform.rateResolutionMs*=ratio;
    for(auto& iv:waveform.intervals)iv.ambiguousRateMs=wavelength*iv.prfHz/2;
    // The physical short window may not contain M distinct transmitter PRFs.
    // Validate its cells with a feasible local count; M is enforced by the
    // measured temporal processor, never silently reduced for homing.
    auto shape=waveform;shape.m=std::min(shape.m,static_cast<int>(shape.intervals.size()));
    if(observed?!validObservedWaveform(shape,c.processing):!validWaveform(shape,c.processing))return {};
    if(observed)waveform.m=c.waveform.m;
    const double dwell=durationS(waveform);const auto beam=beamFor(pointing,axes);
    const double bwAz=pattern_.beamwidthRad(false),bwEl=pattern_.beamwidthRad(true);
    if(bwAz>=pi/2||bwEl>=pi/2)return {};
    const auto gain=[&](V direction){
        const double face[3]={dot(direction,c.aperture.electronic?axes[0]:beam.forward),
            dot(direction,c.aperture.electronic?axes[1]:beam.right),dot(direction,c.aperture.electronic?axes[2]:beam.down)};
        return pattern_.gain(face,c.aperture.electronic?dot(beam.forward,axes[1]):0,
            c.aperture.electronic?dot(beam.forward,axes[2]):0);
    };
    std::vector<std::size_t> order(scene.echoes.size());std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](auto a,auto b){return physicalKey(scene.echoes[a])<physicalKey(scene.echoes[b]);});
    std::vector<radar::signal::Echo> echoes;std::vector<std::size_t> original;
    if(!semi||scene.illuminationAvailable)for(const auto index:order) {
        const auto& source=scene.echoes[index];
        const double snr=energySnr(c,source,gain(source.direction),scene.frequencyHz,dwell,semi);
        if(!std::isfinite(snr)||snr>1e25)return {};
        if(!(snr>0))continue;
        radar::signal::Echo e;e.rangeM=source.delayRangeM;e.rangeRateMs=-source.dopplerHz*wavelength/2;e.snr=snr;
        e.fluctuation=static_cast<int>(source.fluctuation);
        const double forward=dot(source.direction,beam.forward),right=dot(source.direction,beam.right),down=dot(source.direction,beam.down);
        e.monopulseAz=c.monopulseSlope*std::atan2(right,forward)/bwAz;
        e.monopulseEl=c.monopulseSlope*std::atan2(-down,std::hypot(forward,right))/bwEl;
        e.truth=static_cast<std::uint32_t>(original.size()+1);echoes.push_back(e);original.push_back(index);
    }
    const auto clutterStarted=std::chrono::steady_clock::now();
    radar::signal::ClutterMap clutter(waveform.intervals.size());
    const double actualBandwidth=observed?light/(2*waveform.gateM):c.bandwidthHz;
    const double thermalNoise=boltzmann*c.temperatureK*linear(c.noiseFigureDb)*actualBandwidth;
    const double jamRatio=scene.interferencePowerW/thermalNoise;
    for(std::size_t j=0;j<waveform.intervals.size();++j) {
        const auto& iv=waveform.intervals[j];const int rows=iv.gates-iv.blankedGates;
        auto& map=clutter[j];map.resize(static_cast<std::size_t>(rows)*static_cast<std::size_t>(iv.pulses));
        const double chips=iv.pulseS*light/(2*waveform.gateM),period=iv.ambiguousRangeM/waveform.gateM;
        for(int row=0;row<rows;++row) {
            const double value=iv.pulses*radar::signal::noiseShare(row+iv.blankedGates,chips,period)*jamRatio;
            for(int k=0;k<iv.pulses;++k)map[static_cast<std::size_t>(row)*static_cast<std::size_t>(iv.pulses)+static_cast<std::size_t>(k)]=value;
        }
    }
    std::vector<std::size_t> groundOrder(scene.groundPatches.size());std::iota(groundOrder.begin(),groundOrder.end(),0);
    std::sort(groundOrder.begin(),groundOrder.end(),[&](auto a,auto b){return physicalKey(scene.groundPatches[a])<physicalKey(scene.groundPatches[b]);});
    if(!semi||scene.illuminationAvailable)for(const auto index:groundOrder) {
        const auto& patch=scene.groundPatches[index];
        const double snr=energySnr(c,patch,gain(patch.direction),scene.frequencyHz,dwell,semi);
        if(!std::isfinite(snr)||snr>1e25)return {};
        if(!(snr>0))continue;
        for(std::size_t j=0;j<waveform.intervals.size();++j) {
            const auto& iv=waveform.intervals[j];const int m=iv.pulses;
            const double range=std::fmod(patch.delayRangeM,iv.ambiguousRangeM)/waveform.gateM;
            const int first=static_cast<int>(std::floor(range));
            const double centre=(patch.dopplerHz/iv.prfHz-std::floor(patch.dopplerHz/iv.prfHz))*m;
            const double sigma=2*c.clutterRateSigmaMs/wavelength/iv.prfHz*m;
            std::vector<double> profile(static_cast<std::size_t>(m));double total=0;
            for(int k=0;k<m;++k) {
                double fraction=1./m;
                if(sigma<m) {
                    fraction=0;const int images=static_cast<int>(std::ceil(8*sigma/m));
                    for(int image=-images;image<=images;++image)fraction+=.5*(
                        std::erf((k+.5-centre+image*m)/(std::sqrt(2.)*sigma))-
                        std::erf((k-.5-centre+image*m)/(std::sqrt(2.)*sigma)));
                }
                profile[static_cast<std::size_t>(k)]=std::max(0.,fraction);total+=std::max(0.,fraction);
            }
            const double chips=iv.pulseS*light/(2*waveform.gateM),period=iv.ambiguousRangeM/waveform.gateM;
            for(int gate=first;gate<=first+1;++gate)if(gate>=iv.blankedGates&&gate<iv.gates&&total>0) {
                const double response=radar::signal::gateResponse(range,gate,chips,period);
                const double energy=observed?snr*observed->energyFraction.at(j)*m*response*response:
                    snr/static_cast<double>(waveform.intervals.size())*m*response*response;
                for(int k=0;k<m;++k)clutter[j][static_cast<std::size_t>(gate-iv.blankedGates)*static_cast<std::size_t>(m)+static_cast<std::size_t>(k)]+=
                    energy*profile[static_cast<std::size_t>(k)]/total;
            }
        }
    }
    auto settings=c.processing;
    // A high-PRF receiver can coincide directly in its unambiguous Doppler
    // domain. With medium PRFs, use the actual measured path-delay cells to
    // resolve Doppler aliases; that delay is never a passive Cartesian range.
    const double rateSpan=2*std::max(std::abs(settings.minRateMs),std::abs(settings.maxRateMs));
    const auto& ambiguityProgram=observed?scene.illuminationWaveform->intervals:waveform.intervals;
    settings.velocityOnly=semi&&std::all_of(ambiguityProgram.begin(),ambiguityProgram.end(),
        [&](const auto& interval){return interval.ambiguousRateMs>=rateSpan;});
    settings.reportAmbiguities=true;
    settings.mainBeamRateMs=scene.mainBeamClutterRateMs.value_or(std::numeric_limits<double>::quiet_NaN());
    settings.altitudeM=scene.altitudeM.value_or(std::numeric_limits<double>::quiet_NaN());
    const auto streamBefore=stream_;const bool seededBefore=seeded_;
    if(!seeded_){stream_.reseed(seed^0x52465349474e414cull);seeded_=true;}
    RadioReceiverResultV1 result;result.valid=true;result.semiActive=semi;result.timeS=scene.timeS;
    result.pointing=pointing;result.waveform=waveform;
    result.config=c;result.config.bandwidthHz=actualBandwidth;
    result.observationStartTimeS=observed?observed->observationStartTimeS:scene.timeS-dwell;
    result.observationEndTimeS=scene.timeS;result.integratedTimeS=dwell;
    if(observed){result.programIntervals=observed->programIntervals;result.intervalStartTimeS=observed->startTimeS;
        result.intervalEndTimeS=observed->endTimeS;result.intervalEnergyFraction=observed->energyFraction;}
    result.stats.clutterS=std::chrono::duration<double>(std::chrono::steady_clock::now()-clutterStarted).count();
    if(observed) {
        radar::signal::DwellObservationV1 measured;
        measured.intervalEnergyFraction=observed->energyFraction;measured.intervalTimeS.assign(waveform.intervals.size(),scene.timeS);
        if(!radar::signal::dwellObservedV1(waveform,echoes,settings,stream_,&result.stats,&clutter,&result.spectrum,&result.cells,measured)) {
            stream_=streamBefore;seeded_=seededBefore;return {};
        }
        const auto coincidenceStarted=std::chrono::steady_clock::now();
        std::vector<RadioTemporalSegmentV1> segments;
        for(std::size_t j=0;j<waveform.intervals.size();++j) {
            RadioTemporalSegmentV1 segment;segment.programInterval=observed->programIntervals.at(j);segment.timeS=scene.timeS;
            segment.waveform=waveform.intervals[j];segment.rateResolutionMs=wavelength*segment.waveform.prfHz/(2*segment.waveform.pulses);
            for(const auto& peak:measured.peaks)if(peak.interval==j&&positive(peak.snr)&&
                std::isfinite(peak.monopulseAz)&&std::isfinite(peak.monopulseEl)) {
                const double az=std::clamp(peak.monopulseAz/c.monopulseSlope,-1.,1.)*bwAz;
                const double el=std::clamp(peak.monopulseEl/c.monopulseSlope,-1.,1.)*bwEl;
                RadioTemporalPeakV1 p;p.foldedRangeM=peak.rangeM;p.foldedRateMs=peak.rateMs;p.snr=peak.snr;
                const auto physical=measuredBeamRay(beam,az,el);
                p.direction=navigationRay(physical,axes,*scene.navigation);
                p.angularVarianceRad2=std::max(bwAz*bwAz,bwEl*bwEl)/(2*c.monopulseSlope*c.monopulseSlope*peak.snr);
                if(peak.truth>0&&peak.truth<=original.size())p.refereeAttributionIndex=scene.refereeAttributionIndices[original[peak.truth-1]];
                segment.peaks.push_back(p);
            }
            segments.push_back(std::move(segment));
        }
        std::vector<RadioTemporalDetectionV1> detections;
        if(!temporal_.update(scene.timeS,c.temporalMemoryS,c.associationGateRad,static_cast<unsigned>(c.waveform.m),
            settings,waveform.gateM,segments,detections)){stream_=streamBefore;seeded_=seededBefore;return {};}
        result.coincidenceFirstTimeS=scene.timeS;result.coincidenceLastTimeS=scene.timeS;
        for(const auto& detection:detections) {
            RadioMeasurementV1 m;m.direction=detection.direction;m.navigationReconstructed=true;
            m.signalToNoise=detection.snr;m.angularVarianceRad2=detection.angularVarianceRad2;
            m.refereeAttributionIndex=detection.refereeAttributionIndex;
            // Neither a measured bistatic path-delay alias nor its derivative
            // is a Cartesian passive range/range-rate channel.
            result.measurements.push_back(m);
            radar::signal::Hit hit;hit.snr=detection.snr;hit.intervals=static_cast<int>(detection.intervals);hit.hypotheses=detection.hypotheses;
            if(!hit.hypotheses.empty()){hit.rangeM=hit.hypotheses.front()[0];hit.rangeRateMs=hit.hypotheses.front()[1];}
            result.hits.push_back(std::move(hit));
            result.coincidenceFirstTimeS=std::min(result.coincidenceFirstTimeS,detection.firstObservationS);
            result.coincidenceIntervals=std::max(result.coincidenceIntervals,detection.intervals);
        }
        result.stats.processS+=std::chrono::duration<double>(std::chrono::steady_clock::now()-coincidenceStarted).count();
        result.stats.hits=result.hits.size();lastTimeS_=scene.timeS;cached_=result;return result;
    }
    result.hits=radar::signal::dwellWithDiagnosticsV1(waveform,echoes,settings,stream_,&result.stats,&clutter,&result.spectrum,&result.cells);
    for(const auto& hit:result.hits) {
        if(!positive(hit.snr)||!std::isfinite(hit.monopulseAz)||!std::isfinite(hit.monopulseEl))continue;
        const double az=std::clamp(hit.monopulseAz/c.monopulseSlope,-1.,1.)*bwAz;
        const double el=std::clamp(hit.monopulseEl/c.monopulseSlope,-1.,1.)*bwEl;
        RadioMeasurementV1 measurement;measurement.direction=measuredBeamRay(beam,az,el);
        measurement.signalToNoise=hit.snr;
        measurement.angularVarianceRad2=std::max(bwAz*bwAz,bwEl*bwEl)/(2*c.monopulseSlope*c.monopulseSlope*hit.snr);
        if(!semi&&hit.hypotheses.size()<=1){measurement.rangeM=hit.rangeM;measurement.rangeRateMs=hit.rangeRateMs;}
        if(hit.truth>0&&hit.truth<=original.size())measurement.refereeAttributionIndex=scene.refereeAttributionIndices[original[hit.truth-1]];
        result.measurements.push_back(measurement);
    }
    lastTimeS_=scene.timeS;cached_=result;return result;
}
} // namespace fsim::ms::components
