#include "EngineeringSeekerV3.h"
#include "ActiveRadarSeekerV2.h"
#include <algorithm>
#include <cmath>
#include <tuple>
#include <iterator>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include "simulation/mission_systems/radar/Detection.h"

namespace fsim::ms::components {
using namespace measured;
namespace {

// Optional referee-only observation; all storage belongs to this sample call.
// No registry, receiver state, random draw, transport or actor API is added.
struct MeasuredPriorJoinTraceV2 {
    std::uint64_t seed=0;
    std::optional<std::uint64_t> localId;
    double newRead=0,newStart=0,newCapture=0,newEnd=0;
    std::optional<double> oldRead,oldCapture,trackCapture,available;
    bool rawComplete=false,rawPrecise=false,rawUnique=false,rawFit=false,currentPose=false,constructed=false;
    std::optional<bool> trackPresent,trackIdentity,trackRate,trackPrecise,epochMatch,readInOrder;
    std::optional<bool> memory,trackValid,previousPose,ready,pointing;
    std::optional<ThermalReticleMeasuredPriorV2> prior;
};
const char* measuredPriorTraceBoolV2(std::optional<bool> value) noexcept {
    return value?(*value?"true":"false"):"null";
}
bool measuredPriorTraceNumberV2(std::optional<double> value,char (&out)[48]) noexcept {
    if(!value||!std::isfinite(*value)){out[0]='n';out[1]='u';out[2]='l';out[3]='l';out[4]=0;return true;}
    // to_chars is locale-independent and retains the sign of an exact zero.
    const auto result=std::to_chars(out,out+47,*value,std::chars_format::general,std::numeric_limits<double>::max_digits10);
    if(result.ec!=std::errc{})return false;
    *result.ptr=0;return true;
}
void publishMeasuredPriorJoinTraceV2(const MeasuredPriorJoinTraceV2& row) noexcept {
    char read[48],start[48],capture[48],end[48],oldRead[48],oldCapture[48],trackCapture[48],available[48];
    bool valid=measuredPriorTraceNumberV2(row.newRead,read)&&measuredPriorTraceNumberV2(row.newStart,start)&&
        measuredPriorTraceNumberV2(row.newCapture,capture)&&measuredPriorTraceNumberV2(row.newEnd,end)&&
        measuredPriorTraceNumberV2(row.oldRead,oldRead)&&measuredPriorTraceNumberV2(row.oldCapture,oldCapture)&&
        measuredPriorTraceNumberV2(row.trackCapture,trackCapture)&&measuredPriorTraceNumberV2(row.available,available);
    char localId[32]="null";
    if(row.localId)std::snprintf(localId,sizeof(localId),"\"%llu\"",static_cast<unsigned long long>(*row.localId));
    const bool priorFinite=row.prior&&finite(row.prior->direction)&&finite(row.prior->directionRate)&&
        std::isfinite(row.prior->captureTimeS)&&std::isfinite(row.prior->availableTimeS)&&
        std::isfinite(row.prior->widthRad)&&std::isfinite(row.prior->heightRad)&&std::isfinite(row.prior->cutoff);
    char direction[192]="null",rate[192]="null",support[192]="null";
    if(priorFinite) {
        char x[48],y[48],z[48],vx[48],vy[48],vz[48],width[48],height[48],cutoff[48];
        valid=valid&&measuredPriorTraceNumberV2(row.prior->direction.x,x)&&measuredPriorTraceNumberV2(row.prior->direction.y,y)&&
            measuredPriorTraceNumberV2(row.prior->direction.z,z)&&measuredPriorTraceNumberV2(row.prior->directionRate.x,vx)&&
            measuredPriorTraceNumberV2(row.prior->directionRate.y,vy)&&measuredPriorTraceNumberV2(row.prior->directionRate.z,vz)&&
            measuredPriorTraceNumberV2(row.prior->widthRad,width)&&measuredPriorTraceNumberV2(row.prior->heightRad,height)&&
            measuredPriorTraceNumberV2(row.prior->cutoff,cutoff);
        if(valid) {
            std::snprintf(direction,sizeof(direction),"[%s,%s,%s]",x,y,z);
            std::snprintf(rate,sizeof(rate),"[%s,%s,%s]",vx,vy,vz);
            std::snprintf(support,sizeof(support),"[%s,%s,%s]",width,height,cutoff);
        }
    }
    char line[4096];
    const int size=valid?std::snprintf(line,sizeof(line),
        "{\"kind\":\"reticle_measured_prior_join_v2\",\"version\":2,\"proposal_only\":true,"
        "\"receiver_seed\":\"%llu\",\"local_id\":%s,\"new_read_time_s\":%s,\"new_exposure_start_s\":%s,"
        "\"new_capture_time_s\":%s,\"new_exposure_end_s\":%s,\"old_read_time_s\":%s,\"old_capture_time_s\":%s,"
        "\"track_capture_time_s\":%s,\"old_available_time_s\":%s,\"raw_complete\":%s,\"raw_precise\":%s,"
        "\"raw_unique\":%s,\"raw_fit\":%s,\"current_pose_available\":%s,\"track_present\":%s,"
        "\"track_identity_nonzero\":%s,\"track_rate_available\":%s,\"track_precise_predecessor\":%s,"
        "\"capture_epochs_match\":%s,\"old_read_not_future\":%s,\"within_memory\":%s,\"track_valid\":%s,"
        "\"previous_pose_available\":%s,\"ready_before_exposure\":%s,\"previous_pointing_matches\":%s,"
        "\"prior_constructed\":%s,\"prior_values_finite\":%s,\"prior_direction\":%s,\"prior_direction_rate\":%s,"
        "\"prior_width_height_cutoff_guesses\":%s}\n",
        static_cast<unsigned long long>(row.seed),localId,read,start,capture,end,oldRead,oldCapture,trackCapture,available,
        measuredPriorTraceBoolV2(row.rawComplete),measuredPriorTraceBoolV2(row.rawPrecise),measuredPriorTraceBoolV2(row.rawUnique),
        measuredPriorTraceBoolV2(row.rawFit),measuredPriorTraceBoolV2(row.currentPose),measuredPriorTraceBoolV2(row.trackPresent),
        measuredPriorTraceBoolV2(row.trackIdentity),measuredPriorTraceBoolV2(row.trackRate),measuredPriorTraceBoolV2(row.trackPrecise),
        measuredPriorTraceBoolV2(row.epochMatch),measuredPriorTraceBoolV2(row.readInOrder),measuredPriorTraceBoolV2(row.memory),
        measuredPriorTraceBoolV2(row.trackValid),measuredPriorTraceBoolV2(row.previousPose),measuredPriorTraceBoolV2(row.ready),
        measuredPriorTraceBoolV2(row.pointing),measuredPriorTraceBoolV2(row.constructed),
        measuredPriorTraceBoolV2(row.prior?std::optional<bool>(priorFinite):std::nullopt),direction,rate,support):-1;
    if(size>0&&static_cast<std::size_t>(size)<sizeof(line))std::fwrite(line,1,static_cast<std::size_t>(size),stderr);
    else std::fprintf(stderr,"{\"kind\":\"reticle_measured_prior_join_v2\",\"receiver_seed\":\"%llu\",\"diagnostic_refused\":true}\n",
        static_cast<unsigned long long>(row.seed));
}

constexpr double pi=3.14159265358979323846, boltzmann=1.380649e-23;
V toward(V from,V to,double limit) {
    from=unit(from);to=unit(to);
    const double angle=std::acos(std::clamp(dot(from,to),-1.,1.));
    if(angle<=limit)return to;
    V tangent=subtract(to,scale(from,dot(from,to)));
    if(norm(tangent)<1e-12)tangent=cross(from,std::abs(from.z)<.9?V{0,0,1}:V{0,1,0});
    return unit(add(scale(from,std::cos(limit)),scale(unit(tangent),std::sin(limit))));
}
double linear(double db) {return std::pow(10.,db/10.);}
bool positive(double x) {return std::isfinite(x)&&x>0;}
bool validTrack(const SeekerTrackV2& track,double minimumRange) {
    return track.localId&&std::isfinite(track.sampleTimeS)&&track.sampleTimeS>=0&&
        finite(track.direction)&&positive(norm(track.direction))&&
        (!track.rateAvailable||(finite(track.directionRate)&&std::isfinite(norm(track.directionRate))))&&
        (!track.rangeM||(std::isfinite(*track.rangeM)&&*track.rangeM>=minimumRange))&&
        (!track.rangeRateMs||std::isfinite(*track.rangeRateMs))&&
        (!track.imageExtent||validExtent(*track.imageExtent));
}
bool validNavigation(const std::optional<SeekerNavigationPoseV1>& nav,double time) {
    if(!nav||!nav->valid||nav->timeS!=time||!finite(nav->position)||!finite(nav->velocity))return false;
    for(const auto axis:nav->bodyToEcef)if(!finite(axis)||std::abs(norm(axis)-1)>1e-6)return false;
    return std::abs(dot(nav->bodyToEcef[0],nav->bodyToEcef[1]))<=1e-6&&
        norm(subtract(cross(nav->bodyToEcef[0],nav->bodyToEcef[1]),nav->bodyToEcef[2]))<=1e-6;
}
V rotateFrame(V ray,const std::array<V,3>& from,const std::array<V,3>& to) {
    V result{};for(unsigned i=0;i<3;++i)result=add(result,scale(to[i],dot(ray,from[i])));return unit(result);
}
}
EngineeringSeekerV3::EngineeringSeekerV3(weapon::Spec spec,std::unique_ptr<MeasuredSeekerV2> family)
    :spec_(std::move(spec)),family_(std::move(family)) {}
const weapon::SeekerConfigV1& EngineeringSeekerV3::config() const noexcept {return spec_.seekerModel;}
double EngineeringSeekerV3::infraredRange(double a,double b,bool c) const noexcept {return family_->infraredRange(a,b,c);}
double EngineeringSeekerV3::radarRange(double a,double b,bool c) const noexcept {return family_->radarRange(a,b,c);}
bool EngineeringSeekerV3::notched(double a,double b) const noexcept {return family_->notched(a,b);}
double EngineeringSeekerV3::decoySignal(double a,bool b) const noexcept {return family_->decoySignal(a,b);}
bool EngineeringSeekerV3::illuminated(const double a[3],double b,double c,double d,double e) const noexcept {return family_->illuminated(a,b,c,d,e);}
double EngineeringSeekerV3::normal() {
    // Explicit Box-Muller avoids implementation-defined normal_distribution state.
    const auto uniform=[&]{return (static_cast<double>(rng_()>>11)+.5)/9007199254740992.;};
    return std::sqrt(-2*std::log(uniform()))*std::cos(2*pi*uniform());
}
double EngineeringSeekerV3::sourceAspectCosineV3(V ray,V forward) noexcept {
    if(!finite(ray)||!finite(forward)||!positive(norm(ray))||!positive(norm(forward)))
        return std::numeric_limits<double>::quiet_NaN();
    return std::clamp(dot(unit(ray),unit(forward)),-1.,1.);
}
double EngineeringSeekerV3::rfSnr(const weapon::SeekerConfigV1& c,const SeekerSceneV3& s,bool semi,double exposure) noexcept {
    const auto& p=c.engineering;
    if(!positive(s.rangeM)||!positive(s.rcsM2)||!positive(exposure)||!positive(p.temperatureK)||
       !std::isfinite(s.receiverExcessLossDb)||s.receiverExcessLossDb<0)return 0;
    const double frequency=semi?s.transmitterFrequencyHz:p.frequencyHz;
    const double txRange=semi?s.transmitterRangeM:s.rangeM;
    const double power=semi?s.transmitterPowerW:c.emitterPowerW;
    const double txGain=semi?s.transmitterGainLinear:linear(c.emitterPeakGainDbi);
    if(!positive(frequency)||!positive(txRange)||!positive(power)||!positive(txGain)||
       !std::isfinite(s.transmitterExcessLossDb)||s.transmitterExcessLossDb<0)return 0;
    const double wavelength=299792458./frequency;
    // Bistatic radar equation and coherent integration energy / kTF. Bandwidth
    // cancels through matched-filter processing gain B*T; no magic range gate.
    const double received=power*txGain*linear(c.emitterPeakGainDbi)*s.rcsM2*wavelength*wavelength/
        (std::pow(4*pi,3)*txRange*txRange*s.rangeM*s.rangeM*
         linear(p.systemLossDb+s.receiverExcessLossDb+(semi?s.transmitterExcessLossDb:s.receiverExcessLossDb)));
    const double noisePower=boltzmann*p.temperatureK*p.bandwidthHz*linear(p.noiseFigureDb);
    const double processingGain=p.bandwidthHz*exposure;
    const double snr=received*processingGain/noisePower;
    return std::isfinite(snr)&&snr>0?snr:0;
}
double EngineeringSeekerV3::bandFraction(double temperature,double low,double high) noexcept {
    if(!positive(temperature)||!positive(low)||!positive(high)||high<=low)return 0;
    // Dimensionless Planck integral. A uniform wavelength grid can completely
    // miss the spectral peak for broad bands. The x>80 tail is <2e-29.
    const double lower=std::min(80.,.01438776877/(high*1e-6*temperature));
    const double upper=std::min(80.,.01438776877/(low*1e-6*temperature));
    if(!(upper>lower))return 0;
    unsigned intervals=std::max(64u,static_cast<unsigned>(std::ceil((upper-lower)*16)));
    if(intervals%2)++intervals;
    const double step=(upper-lower)/intervals;
    double integral=0;
    for(unsigned i=0;i<=intervals;++i) {
        const double x=lower+i*step;
        const double radiance=x>0?x*x*x/std::expm1(x):0;
        integral+=(i==0||i==intervals?1:i%2?4:2)*radiance;
    }
    const double fraction=integral*step/3*15/std::pow(pi,4);
    return std::isfinite(fraction)?std::clamp(fraction,0.,1.):0;
}
double EngineeringSeekerV3::irSnr(const weapon::SeekerConfigV1& c,const SeekerSceneV3& s,double strength) noexcept {
    const auto& p=c.engineering;
    if(!positive(s.rangeM)||!positive(strength)||!positive(p.irNoiseWm2)||!positive(p.backgroundWm2)||
       !std::isfinite(s.receiverExcessLossDb)||s.receiverExcessLossDb<0||!std::isfinite(s.temperatureK)||s.temperatureK<0)return 0;
    const double temperature=s.temperatureK>0?s.temperatureK:p.irSourceTemperatureK;
    const double fraction=bandFraction(temperature,p.irBandLowUm,p.irBandHighUm);
    const double irradiance=p.irIntensityWSr*std::pow(temperature/p.irSourceTemperatureK,4)*strength*fraction*std::exp(-p.irExtinctionPerM*s.rangeM)/
        (s.rangeM*s.rangeM*linear(s.receiverExcessLossDb));
    const double snr=irradiance/std::hypot(p.irNoiseWm2,p.backgroundWm2);
    return std::isfinite(snr)&&snr>0?snr:0;
}
double EngineeringSeekerV3::detectionProbability(double snr,double threshold,bool heat) noexcept {
    if(!std::isfinite(snr)||snr<0||!positive(threshold))return 0;
    // Native steady-target envelope detector; IR uses Gaussian irradiance noise.
    return heat?.5*std::erfc((threshold-snr)/std::sqrt(2.)):
        radar::probabilityOfDetection(snr,std::exp(-threshold),radar::Fluctuation::Steady);
}
bool EngineeringSeekerV3::measuredAssociationV1(V direction,const SeekerTrackV2& track,double captureTimeS,double readTimeS,double gateRad) noexcept {
    if(!finite(direction)||!finite(track.direction)||!std::isfinite(captureTimeS)||!std::isfinite(track.sampleTimeS)||
        !std::isfinite(readTimeS)||captureTimeS>readTimeS||!std::isfinite(gateRad)||gateRad<=0)return false;
    const auto rate=track.rateAvailable?track.directionRate:V{};
    if(!finite(rate))return false;
    const auto predicted=unit(add(track.direction,scale(rate,std::max(0.,captureTimeS-track.sampleTimeS))));
    return dot(direction,predicted)>=std::cos(gateRad);
}
void EngineeringSeekerV3::seedTrack(const SeekerTrackV2& track) {
    // This is an actual on-rail measurement, not an instantaneous detector cue.
    if(!validTrack(track,config().minimumRangeM)||(lastTimeS_>=0&&track.sampleTimeS>lastTimeS_))return;
    auto admitted=track;admitted.direction=unit(admitted.direction);
    // An unavailable channel has no payload authority. In particular, never
    // read a NaN placeholder as an angular velocity during later prediction.
    if(!admitted.rateAvailable)admitted.directionRate={};
    if(radarNavigationTrack_&&radarNavigationTrack_!=admitted.localId){
        radarNavigation_.reset();radarNavigationSupportPrior_.reset();radarSupportEpochS_=-1;radarNavigationTrack_=0;
    }
    family_->seedTrack(admitted);filtered_=admitted;pointing_=admitted.direction;
    // A track-only handoff cannot distinguish a resolved first point from a
    // clipped quadrant centering ray. An unavailable rate supplies neither
    // permission to difference that ray nor a resolved-position certificate.
    filteredQuadrantUnresolved_=spec_.seeker=="infrared"&&thermalConfig_&&!admitted.rateAvailable;
}
bool EngineeringSeekerV3::seedLaunchNavigationV1(const weapon::Observation& estimate) {
    if((!spec_.inertialMidcourse&&!spec_.launcherUpdates)||!finite(estimate.position)||!finite(estimate.velocity))return false;
    family_->seedNavigationEstimate(estimate);return true;
}
void EngineeringSeekerV3::resetTracking() noexcept {
    family_->resetTracking();pending_.clear();filtered_.reset();filteredQuadrantUnresolved_=false;quadrantCentering_.reset();quadrantBounds_={};cached_={};
    pointing_={};lastTimeS_=-1;nextSampleS_=0;firstTimeS_=0;seeded_=false;previousAxes_.reset();
    thermal_.reset();thermalDual_.reset();reticle_.reset();opticalPoseHistory_.clear();
    radio_.reset();
    radioActiveSinceS_.reset();
    radarNavigation_.reset();radarNavigationTrack_=0;
    radarNavigationSupportPrior_.reset();radarSupportEpochS_=-1;
}
SeekerResultV2 EngineeringSeekerV3::sample(const SeekerFrameV2& f) {
    if(physicalConfig_||dualConfig_||reticleConfig_)return {}; // Explicit new binding requires its actual scene/navigation input.
    return sampleImpl(f,nullptr);
}
std::optional<EngineeringSeekerV3::OpticalPose> EngineeringSeekerV3::opticalPoseAt(double time) const {
    const auto at=std::lower_bound(opticalPoseHistory_.begin(),opticalPoseHistory_.end(),time,
        [](const OpticalPose& p,double t){return p.timeS<t;});
    if(at==opticalPoseHistory_.end())return std::nullopt;
    if(at->timeS==time)return *at;
    if(at==opticalPoseHistory_.begin())return std::nullopt;
    const auto& a=*std::prev(at);const auto& b=*at;const double duration=b.timeS-a.timeS;
    if(duration<=0||duration>std::max(.25,2*config().engineering.samplePeriodS))return std::nullopt;
    const double w=(time-a.timeS)/duration;
    const auto interpolate=[&](V x,V y){return add(scale(x,1-w),scale(y,w));};
    const auto axes=[&](const auto& x,const auto& y)->std::optional<std::array<V,3>> {
        auto first=interpolate(x[0],y[0]);
        if(norm(first)<.5)return std::nullopt;
        first=unit(first);
        auto second=interpolate(x[1],y[1]);second=subtract(second,scale(first,dot(first,second)));
        if(norm(second)<.5)return std::nullopt;
        second=unit(second);
        return std::array<V,3>{first,second,cross(first,second)};
    };
    const auto actual=axes(a.physicalAxes,b.physicalAxes),estimated=axes(a.navigation.bodyToEcef,b.navigation.bodyToEcef);
    if(!actual||!estimated)return std::nullopt;
    auto result=a;result.timeS=time;result.navigation.timeS=time;
    result.navigation.position=interpolate(a.navigation.position,b.navigation.position);
    result.navigation.velocity=interpolate(a.navigation.velocity,b.navigation.velocity);
    result.navigation.bodyToEcef=*estimated;result.physicalAxes=*actual;
    result.pointing=unit(interpolate(a.pointing,b.pointing));result.up=unit(interpolate(a.up,b.up));
    if(norm(result.pointing)<.5||norm(result.up)<.5)return std::nullopt;
    return result;
}
bool EngineeringSeekerV3::configureReceiverV3(const ReceiverConfigurationV3& receiver,std::string& error) {
    if(!receiver.reticle)return configureReceiverV2(receiver.inherited,error);
    if(!configureThermalReticleV1(receiver.reticle->policy,error))return false;
    reticleReceiverConfig_=receiver.reticle;return true;
}
bool EngineeringSeekerV3::configureThermalReticleV1(const ThermalReticleConfigV1& policy,std::string& error) {
    if(spec_.seeker!="infrared"||!ThermalReticleReceiverV1::validConfig(policy)||
        1/policy.frequencyHz>config().engineering.samplePeriodS||
        .5*std::hypot(static_cast<double>(policy.detector.width),static_cast<double>(policy.detector.height))*policy.detector.pixelPitchRad>config().engineering.fieldHalfAngleRad) {
        error="reticle scan does not fit selected finite sensor cadence/field";return false;
    }
    if(!reticle_.configure(policy,error))return false;
    resetTracking();reticleConfig_=policy;thermalConfig_=policy.detector;radioConfig_.reset();dualConfig_.reset();
    physicalConfig_.reset();receiverConfigV2_.reset();reticleReceiverConfig_.reset();return true;
}
SeekerResultV2 EngineeringSeekerV3::sampleThermalReticleV1(const SeekerFrameV2& frame,const ThermalSceneFrameV1& scene) {
    if(!reticleConfig_||scene.timeS!=frame.timeS||!validNavigation(scene.navigation,frame.timeS)||
        !ThermalImageReceiverV1::validScene(scene,reticleConfig_->detector))return {};
    return sampleImpl(frame,&scene);
}
bool EngineeringSeekerV3::configureReceiverV1(const ReceiverConfigurationV1& receiver,std::string& error) {
    bool accepted=false;
    if(receiver.kind==PhysicalReceiverKindV1::Image||receiver.kind==PhysicalReceiverKindV1::Quadrant)
        accepted=configureThermalV1(receiver.thermal,error);
    if(receiver.kind==PhysicalReceiverKindV1::Radio)accepted=configureRadioV1(receiver.radio,error);
    if(!accepted)return false;
    physicalConfig_=receiver;return true;
}
bool EngineeringSeekerV3::configureReceiverV2(const ReceiverConfigurationV2& receiver,std::string& error) {
    if(!receiver.dual) {
        if(!configureReceiverV1(receiver.single,error))return false;
    } else {
        if(!receiver.secondBandScene||receiver.single.kind!=PhysicalReceiverKindV1::Image) {
            error="dual receiver needs both physical band scenes";return false;
        }
        if(!configureThermalDualBandV1(*receiver.dual,error))return false;
        physicalConfig_=receiver.single;
    }
    receiverConfigV2_=receiver;return true;
}
bool EngineeringSeekerV3::configureThermalDualBandV1(const ThermalDualBandConfigV1& policy,std::string& error) {
    if(spec_.seeker!="imaging_infrared"||!ThermalDualBandReceiverV1::validConfig(policy)) {
        error="dual-waveband receiver requires valid imaging infrared policy";return false;
    }
    for(const auto& c:policy.channels)if(c.integrationTimeS>config().engineering.samplePeriodS||
        .5*std::hypot(static_cast<double>(c.width),static_cast<double>(c.height))*c.pixelPitchRad>config().engineering.fieldHalfAngleRad) {
        error="dual thermal policy does not fit selected exposure/field";return false;
    }
    if(!thermalDual_.configure(policy,error))return false;
    resetTracking();reticleConfig_.reset();reticleReceiverConfig_.reset();thermalConfig_=policy.channels[0];dualConfig_=policy;radioConfig_.reset();return true;
}
SeekerResultV2 EngineeringSeekerV3::sampleThermalDualBandV1(const SeekerFrameV2& frame,const ThermalDualBandSceneFrameV1& scene) {
    if(!dualConfig_||!ThermalDualBandReceiverV1::validScene(scene,*dualConfig_))return {};
    for(const auto& channel:scene.channels)
        if(channel.timeS!=frame.timeS||!validNavigation(channel.navigation,frame.timeS))return {};
    const auto& a=*scene.channels[0].navigation;const auto& b=*scene.channels[1].navigation;
    if(a.position.x!=b.position.x||a.position.y!=b.position.y||a.position.z!=b.position.z||
       a.velocity.x!=b.velocity.x||a.velocity.y!=b.velocity.y||a.velocity.z!=b.velocity.z)return {};
    for(unsigned i=0;i<3;++i)if(a.bodyToEcef[i].x!=b.bodyToEcef[i].x||a.bodyToEcef[i].y!=b.bodyToEcef[i].y||a.bodyToEcef[i].z!=b.bodyToEcef[i].z)return {};
    return sampleImpl(frame,&scene.channels[0],nullptr,&scene);
}
bool EngineeringSeekerV3::configureThermalV1(const ThermalImageConfigV1& policy,std::string& error) {
    if((spec_.seeker!="infrared"&&spec_.seeker!="imaging_infrared")||
        policy.integrationTimeS>config().engineering.samplePeriodS||
        .5*std::hypot(static_cast<double>(policy.width),static_cast<double>(policy.height))*policy.pixelPitchRad>
            config().engineering.fieldHalfAngleRad) {
        error="thermal policy does not fit selected receiver exposure/field";return false;
    }
    const bool reconfigure=thermalConfig_.has_value()||radioConfig_.has_value()||lastTimeS_>=0;
    if(!thermal_.configure(policy,error))return false;
    thermalConfig_=policy;dualConfig_.reset();reticleConfig_.reset();reticleReceiverConfig_.reset();receiverConfigV2_.reset();if(reconfigure)resetTracking();return true;
}
SeekerResultV2 EngineeringSeekerV3::sampleThermalV1(const SeekerFrameV2& frame,const ThermalSceneFrameV1& scene) {
    if(dualConfig_||reticleConfig_||!thermalConfig_||scene.timeS!=frame.timeS||!validNavigation(scene.navigation,frame.timeS)||
        !ThermalImageReceiverV1::validScene(scene,*thermalConfig_))return {};
    return sampleImpl(frame,&scene);
}
bool EngineeringSeekerV3::configureRadioV1(const RadioReceiverConfigV1& policy,std::string& error) {
    if((spec_.seeker!="active_radar"&&spec_.seeker!="semi_active_radar")||
        RadioReceiverV1::durationS(policy.waveform)>config().engineering.samplePeriodS||
        policy.observationWindowS>config().engineering.samplePeriodS) {
        error="RF waveform does not fit selected finite receiver exposure";return false;
    }
    auto selected=policy;selected.temporalMemoryS=config().engineering.memoryS;
    const bool reconfigure=thermalConfig_.has_value()||radioConfig_.has_value()||lastTimeS_>=0;
    if(!radio_.configure(selected,error))return false;
    radioConfig_=selected;dualConfig_.reset();reticleConfig_.reset();reticleReceiverConfig_.reset();receiverConfigV2_.reset();if(reconfigure)resetTracking();return true;
}
SeekerResultV2 EngineeringSeekerV3::sampleRadioV1(const SeekerFrameV2& frame,const RadioSceneFrameV1& scene) {
    const bool semi=spec_.seeker=="semi_active_radar";
    if(!radioConfig_||scene.timeS!=frame.timeS||!validNavigation(scene.navigation,frame.timeS)||
        !RadioReceiverV1::validScene(scene,*radioConfig_,semi)||
        (semi&&scene.illuminationAvailable!=(frame.illuminated&&frame.configuredIlluminator!=0)))return {};
    return sampleImpl(frame,nullptr,&scene);
}
SeekerResultV2 EngineeringSeekerV3::sampleImpl(const SeekerFrameV2& f,const ThermalSceneFrameV1* thermalFrame,const RadioSceneFrameV1* radioFrame,const ThermalDualBandSceneFrameV1* dualFrame) {
    const auto& p=config().engineering;
    if(p.version!=3||!weapon::validSeekerConfigV1(config())||!std::isfinite(f.timeS)||f.timeS<0||f.timeS<lastTimeS_||
       !std::isfinite(f.ageS)||f.ageS<0||!std::isfinite(f.dtS)||f.dtS<0||
       !finite(f.position)||!finite(f.velocity)||!finite(f.up)||!finite(f.boresight)||norm(f.boresight)==0||!f.bodyAxes)return {};
    if(f.launcherEstimate&&(!std::isfinite(f.launcherEstimate->sampleTimeS)||f.launcherEstimate->sampleTimeS<0||
       f.launcherEstimate->sampleTimeS>f.timeS||!finite(f.launcherEstimate->position)||!finite(f.launcherEstimate->velocity)))return {};
    if(f.bodyAxes) {
        for(const auto& axis:*f.bodyAxes)if(!finite(axis)||std::abs(norm(axis)-1)>1e-6)return {};
        if(std::abs(dot((*f.bodyAxes)[0],(*f.bodyAxes)[1]))>1e-6||
           norm(subtract(cross((*f.bodyAxes)[0],(*f.bodyAxes)[1]),(*f.bodyAxes)[2]))>1e-6)return {};
    }
    if(f.timeS==lastTimeS_)return cached_;
    if(radioFrame&&spec_.seeker=="semi_active_radar")radio_.observeIlluminationV1(*radioFrame);
    if(!seeded_) {rng_.seed(f.receiverSeed^0x53454e534f525633ull);backgroundRng_.seed(f.receiverSeed^0x4241434b47524f55ull);seeded_=true;nextSampleS_=firstTimeS_=f.timeS;}
    const double dt=lastTimeS_<0?0:f.timeS-lastTimeS_;
    const auto* navigation=thermalFrame?&*thermalFrame->navigation:radioFrame?&*radioFrame->navigation:nullptr;
    const V ownPosition=navigation?navigation->position:f.position;
    if(norm(pointing_)==0)pointing_=unit(f.boresight);
    else if(previousAxes_&&f.bodyAxes) {
        V moved{};
        for(unsigned i=0;i<3;++i)moved=add(moved,scale((*f.bodyAxes)[i],dot(pointing_,(*previousAxes_)[i])));
        pointing_=unit(moved);
    }
    V desired=unit(f.boresight);
    bool navigationCue=false;
    const double detectorHalfAngle=thermalFrame?.5*std::min(thermalConfig_->width,thermalConfig_->height)*thermalConfig_->pixelPitchRad:
        radioFrame?.5*std::min(radio_.beamwidthRad(false),radio_.beamwidthRad(true)):p.fieldHalfAngleRad;
    if(cached_.track&&f.timeS-cached_.track->sampleTimeS<=p.memoryS) {desired=unit(add(cached_.track->direction,
        scale(cached_.track->directionRate,std::max(0.,f.timeS-cached_.track->sampleTimeS))));navigationCue=true;}
    else if(spec_.launcherUpdates&&f.supported&&f.launcherEstimate) {desired=unit(subtract(add(f.launcherEstimate->position,
        scale(f.launcherEstimate->velocity,f.timeS-f.launcherEstimate->sampleTimeS)),ownPosition));navigationCue=true;}
    else if(cached_.midcourseEstimate) {desired=unit(subtract(add(cached_.midcourseEstimate->position,
        scale(cached_.midcourseEstimate->velocity,f.timeS-cached_.midcourseEstimate->sampleTimeS)),ownPosition));navigationCue=true;}
    else if(cached_.valid&&(!reticleConfig_||cached_.measurementTimeS.has_value())&&f.bodyAxes&&dt>0&&config().gimbalRad>detectorHalfAngle) {
        // Interleaved central raster: row spacing <= one detector diameter,
        // center revisited every two sweeps. Geometry derives the bar count;
        // search phase belongs to this sensor, including before launch exists.
        const double limit=config().gimbalRad;
        const double sweep=2*limit/std::min(p.searchRateRadS,p.gimbalRateRadS);
        const double bars=std::ceil(limit/(2*detectorHalfAngle));
        const double elapsed=f.timeS-firstTimeS_;
        const double index=std::floor(std::fmod(elapsed,4*bars*sweep)/sweep);
        const double fraction=std::fmod(elapsed,sweep)/sweep;
        const bool central=std::fmod(index,2)==0;
        const double row=std::floor(index/2);
        const double el=central?0:(std::fmod(row,2)==0?1:-1)*(std::floor(row/2)+1)*(limit-detectorHalfAngle)/bars;
        const double az=(central?-1:1)*limit+(central?1:-1)*2*limit*fraction;
        desired=add(scale((*f.bodyAxes)[0],std::cos(az)*std::cos(el)),
            add(scale((*f.bodyAxes)[1],std::sin(az)*std::cos(el)),scale((*f.bodyAxes)[2],-std::sin(el))));
    }
    // A clipped quadrant ratio carries a centering direction rather than an
    // accurate angular magnitude. Close that optical loop using its delayed
    // measured error, with the same finite servo and mechanical stops below.
    // This device-local error is already in the physical optical frame.
    if(thermalFrame&&spec_.seeker=="infrared"&&quadrantCentering_) {
        desired=*quadrantCentering_;navigationCue=false;
    }
    if(navigation&&navigationCue)desired=rotateFrame(desired,navigation->bodyToEcef,*f.bodyAxes);
    const auto body=unit(f.boresight);
    // Project command/physical stops to the actual body cone. Transport above
    // accounts for finite stabilization work during body pitch/yaw/roll.
    desired=toward(body,desired,config().gimbalRad);
    const double error=std::acos(std::clamp(dot(pointing_,desired),-1.,1.));
    pointing_=toward(pointing_,desired,std::min(p.gimbalRateRadS*dt,error*(1-std::exp(-dt/p.gimbalLagS))));
    pointing_=toward(body,pointing_,config().gimbalRad);
    previousAxes_=f.bodyAxes;
    if(reticleConfig_&&navigation) {
        opticalPoseHistory_.push_back({f.timeS,*navigation,*f.bodyAxes,pointing_,f.up});
        const double oldest=f.timeS-std::max(.25,2*p.samplePeriodS);
        while(opticalPoseHistory_.size()>2&&opticalPoseHistory_[1].timeS<oldest)opticalPoseHistory_.pop_front();
        while(opticalPoseHistory_.size()>4096)opticalPoseHistory_.pop_front();
    }
    if(f.timeS+1e-12>=nextSampleS_) {
        SeekerFrameV2 exposure=f;exposure.samples.clear();
        if(navigation){exposure.position=navigation->position;exposure.velocity=navigation->velocity;
            exposure.boresight=navigation->bodyToEcef[0];exposure.bodyAxes=navigation->bodyToEcef;}
        std::vector<std::size_t> indices;
        std::vector<std::size_t> dualFeatureIndices;
        std::vector<double> angularVariances;
        bool quadrantUnresolved=false;
        bool receiverCaptureReady=true;
        double captureTimeS=f.timeS,processingStartS=f.timeS;
        std::optional<OpticalPose> reticlePose;
        std::optional<V> quadrantCentering;
        const bool heat=spec_.seeker=="infrared"||spec_.seeker=="imaging_infrared";
        const bool semi=spec_.seeker=="semi_active_radar";
        bool radioReady=true;
        if(radioFrame&&!semi) {
            const auto* active=dynamic_cast<const ActiveRadarSeekerV2*>(family_.get());
            if(!active)return {};
            const auto phase=active->previewPhaseV1(exposure);if(!phase.valid)return {};
            if(phase.active&&!radioActiveSinceS_)radioActiveSinceS_=f.timeS;
            const double completeWindow=std::max(radioConfig_->observationWindowS,RadioReceiverV1::durationS(radioConfig_->waveform));
            radioReady=phase.active&&radioActiveSinceS_&&f.timeS-*radioActiveSinceS_>=completeWindow;
        }
        // Canonical observable ordering ensures seeded draws do not depend on
        // World container order. Return indices still refer to original frame.
        std::vector<std::size_t> order;
        for(std::size_t i=0;i<f.samples.size();++i) {
            const auto& s=f.samples[i];
            if(s.scene&&finite(s.direction)&&positive(norm(s.direction))&&positive(s.signal)&&
               positive(s.scene->rangeM)&&std::isfinite(s.scene->rcsM2))order.push_back(i);
        }
        std::stable_sort(order.begin(),order.end(),[&](auto a,auto b){
            const auto key=[](const auto& s) {
                const auto safe=[](double x){return std::isfinite(x)?x:std::numeric_limits<double>::infinity();};
                const auto& physics=*s.scene;
                return std::tuple{s.direction.x,s.direction.y,s.direction.z,s.signal,physics.rangeM,physics.rcsM2,
                    safe(physics.receiverExcessLossDb),safe(physics.transmitterRangeM),safe(physics.transmitterPowerW),safe(physics.transmitterGainLinear),
                    safe(physics.transmitterExcessLossDb),safe(physics.transmitterFrequencyHz),safe(physics.temperatureK),
                    s.rangeRateMs.has_value(),safe(s.rangeRateMs.value_or(0)),s.imageExtent.has_value(),
                    safe(s.imageExtent?s.imageExtent->widthRad:0),safe(s.imageExtent?s.imageExtent->heightRad:0),
                    s.illuminator,s.transmitterPathClear,s.receiverPathClear};
            };
            return key(f.samples[a])<key(f.samples[b]);
        });
        for(const auto i:order) {
            if(thermalFrame||radioFrame)break;
            auto sample=f.samples[i];
            if(!sample.scene||!finite(sample.direction)||!positive(norm(sample.direction))||!positive(sample.signal)||
               dot(unit(sample.direction),pointing_)<std::cos(p.fieldHalfAngleRad))continue;
            if(semi&&(!f.illuminated||!f.configuredIlluminator||sample.illuminator!=f.configuredIlluminator||
                !sample.transmitterPathClear||!sample.receiverPathClear))continue;
            double snr=heat?irSnr(config(),*sample.scene,sample.signal):
                rfSnr(config(),*sample.scene,semi,p.samplePeriodS)*sample.signal;
            // A declared reduced-order ground clutter screening penalty, driven
            // only by the measured scene look direction, never a target label.
            if(!heat&&dot(unit(sample.direction),unit(f.up))<config().radarLookdownCosine)
                snr/=linear(p.lookdownClutterDb);
            const double draw=(static_cast<double>(rng_()>>11)+.5)/9007199254740992.;
            if(snr<=0||draw>=detectionProbability(snr,p.detectionSnr,heat))continue;
            const double sigma=std::hypot(p.angleNoiseRad,p.angleResolutionRad/std::sqrt(12*snr));
            if(!std::isfinite(sigma))continue;
            const auto ray=unit(sample.direction);const auto u=unit(cross(ray,std::abs(ray.z)<.9?V{0,0,1}:V{0,1,0}));
            const auto v=cross(ray,u);
            sample.direction=unit(add(ray,add(scale(u,normal()*sigma),scale(v,normal()*sigma))));
            const double sceneRange=sample.scene->rangeM;
            sample.signal=snr;sample.scene.reset();
            if(sample.imageExtent) {
                if(sample.imageExtent->widthRad<p.angleResolutionRad||sample.imageExtent->heightRad<p.angleResolutionRad)sample.imageExtent.reset();
                else {
                    sample.imageExtent->widthRad=std::round(sample.imageExtent->widthRad/p.angleResolutionRad)*p.angleResolutionRad;
                    sample.imageExtent->heightRad=std::round(sample.imageExtent->heightRad/p.angleResolutionRad)*p.angleResolutionRad;
                }
            }
            if(!heat&&!semi) {
                sample.rangeM=std::max(config().minimumRangeM,sceneRange)+normal()*p.rangeNoiseM;
                if(sample.rangeM&&*sample.rangeM<config().minimumRangeM)sample.rangeM=config().minimumRangeM;
                if(sample.rangeRateMs)sample.rangeRateMs=*sample.rangeRateMs+normal()*p.rangeRateNoiseMs;
            } else {sample.rangeM.reset();sample.rangeRateMs.reset();}
            exposure.samples.push_back(sample);indices.push_back(i);angularVariances.push_back(sigma*sigma);
        }
        if(thermalFrame) {
            std::vector<ThermalImageFeatureV1> features;
            if(reticleConfig_) {
                const auto window=reticle_.captureWindowV1(f.timeS);
                if(window)reticlePose=opticalPoseAt((*window)[1]);
                if(window&&!reticlePose)return {};
                auto scene=*thermalFrame;
                if(reticlePose)scene.up=reticlePose->up;
                std::optional<ThermalReticleMeasuredPriorV2> measuredPrior;
                const auto* previous=reticle_.latest();
                const char* joinFlag=std::getenv("FSIM_RETICLE_PRIOR_JOIN_TRACE_V2");
                const bool traceJoin=joinFlag&&joinFlag[0]=='1'&&joinFlag[1]==0&&window&&previous&&
                    previous->measurementTimeS<(*window)[1];
                MeasuredPriorJoinTraceV2 joinTrace;
                const auto observeJoinGate=[&](std::optional<bool>& observed,bool value) {
                    if(traceJoin)observed=value;
                    return value; // Exact original short-circuit gate result.
                };

                // A raw fit alone is not an accepted receiver track. The unique
                // precise raw feature and filtered result must share the EXACT
                // capture epoch; never join current tracking to another scan.
                if(window&&reticlePose&&previous&&previous->complete&&previous->preciseBearing&&
                   previous->features.size()==1&&previous->horizonDiagnostic.fitAvailable&&
                   filtered_&&!filteredQuadrantUnresolved_&&filtered_->localId&&filtered_->rateAvailable&&
                   filtered_->sampleTimeS==previous->measurementTimeS&&
                   previous->measurementTimeS<(*window)[1]&&previous->timeS<=f.timeS&&
                   observeJoinGate(joinTrace.memory,f.timeS-filtered_->sampleTimeS<=p.memoryS)&&
                   observeJoinGate(joinTrace.trackValid,validTrack(*filtered_,config().minimumRangeM))) {
                    const auto previousPose=opticalPoseAt(previous->measurementTimeS);
                    const double available=std::max(previous->timeS,previous->exposureEndTimeS+p.processingDelayS);
                    if(traceJoin){joinTrace.previousPose=previousPose.has_value();joinTrace.available=available;}
                    if(previousPose&&observeJoinGate(joinTrace.ready,available<=(*window)[0])&&
                       observeJoinGate(joinTrace.pointing,previous->pointing.x==previousPose->pointing.x&&
                           previous->pointing.y==previousPose->pointing.y&&previous->pointing.z==previousPose->pointing.z)) {
                        // filtered_ is already in the navigation Cartesian
                        // frame, mapped from the matching previous capture at
                        // admission above. Map that inertial vector ONCE into
                        // the current capture's physical coordinate frame.
                        // Applying old body axes again would add ego motion.
                        const auto direction=rotateFrame(filtered_->direction,reticlePose->navigation.bodyToEcef,reticlePose->physicalAxes);
                        // rotateFrame() normalizes a RAY; never use it for a
                        // derivative. This linear map retains zero/magnitude.
                        V rate{};for(unsigned axis=0;axis<3;++axis)
                            rate=add(rate,scale(reticlePose->physicalAxes[axis],dot(filtered_->directionRate,reticlePose->navigation.bodyToEcef[axis])));
                        const auto& shape=previous->horizonDiagnostic.shape;
                        // Previous axis-aligned support and cutoff are finite
                        // initial guesses only. Ego rotation can change the
                        // current support; all five coordinates are re-fit.
                        const auto& support=previous->features.front().extent;
                        measuredPrior=ThermalReticleMeasuredPriorV2{previous->measurementTimeS,available,direction,rate,support.widthRad,support.heightRad,shape[4]};
                    }
                }

                if(traceJoin) {
                    // Copy BEFORE expose replaces cached_: previous aliases that
                    // receiver-owned value. Unreached gate values stay null.
                    joinTrace.seed=f.receiverSeed;joinTrace.newRead=f.timeS;joinTrace.newStart=(*window)[0];
                    joinTrace.newCapture=(*window)[1];joinTrace.newEnd=(*window)[2];
                    joinTrace.oldRead=previous->timeS;
                    if(previous->complete)joinTrace.oldCapture=previous->measurementTimeS;
                    joinTrace.rawComplete=previous->complete;joinTrace.rawPrecise=previous->preciseBearing;
                    joinTrace.rawUnique=previous->features.size()==1;joinTrace.rawFit=previous->horizonDiagnostic.fitAvailable;
                    joinTrace.currentPose=reticlePose.has_value();joinTrace.trackPresent=filtered_.has_value();
                    joinTrace.readInOrder=previous->timeS<=f.timeS;
                    if(filtered_) {
                        joinTrace.localId=filtered_->localId;joinTrace.trackCapture=filtered_->sampleTimeS;
                        joinTrace.trackIdentity=filtered_->localId!=0;joinTrace.trackRate=filtered_->rateAvailable;
                        joinTrace.trackPrecise=!filteredQuadrantUnresolved_;
                        if(previous->complete)joinTrace.epochMatch=filtered_->sampleTimeS==previous->measurementTimeS;
                    }
                    joinTrace.constructed=measuredPrior.has_value();joinTrace.prior=measuredPrior;
                }
                const auto signal=reticle_.exposeWithMeasuredPriorV2(scene,reticlePose?reticlePose->pointing:pointing_,
                    f.receiverSeed,measuredPrior?&*measuredPrior:nullptr);
                // A completed newer capture updates latest(), naturally
                // preventing duplicate records for rereads of the same scan.
                // Refused/incomplete exposures emit no join/admission claim.
                if(traceJoin&&signal.valid&&signal.complete&&signal.measurementTimeS==joinTrace.newCapture)
                    publishMeasuredPriorJoinTraceV2(joinTrace);
                if(!signal.valid)return {};
                receiverCaptureReady=signal.complete;
                if(signal.complete) {
                    captureTimeS=signal.measurementTimeS;processingStartS=signal.exposureEndTimeS;
                    exposure.timeS=captureTimeS;exposure.ageS=std::max(0.,f.ageS-(f.timeS-captureTimeS));
                    exposure.position=reticlePose->navigation.position;exposure.velocity=reticlePose->navigation.velocity;
                    exposure.bodyAxes=reticlePose->navigation.bodyToEcef;exposure.boresight=reticlePose->navigation.bodyToEcef[0];
                    // A finite coarse centering direction is not a resolved
                    // angular sample. Only consecutive precise captures can
                    // grant LOS-rate authority; keep coarse detections usable
                    // for association/centering with their actual error bound.
                    features=signal.features;quadrantUnresolved=!signal.preciseBearing;
                }
            } else if(dualFrame) {
                const auto image=thermalDual_.expose(*dualFrame,pointing_,f.receiverSeed);
                if(!image.valid)return {};
                for(const auto& feature:image.features)features.push_back(feature);
            } else {
                const auto image=thermal_.expose(*thermalFrame,pointing_,f.receiverSeed,spec_.seeker=="imaging_infrared");
                if(!image.valid)return {};
                features=image.features;
                if(spec_.seeker=="infrared"&&features.size()==1) {
                    const auto& feature=features.front();const auto& q=image.quadrantElectrons;
                    const double total=q[0]+q[1]+q[2]+q[3];
                    const double ratioNoise=2/feature.signalToNoise;
                    const double uncertainty=image.config.detectionSigma*ratioNoise;
                    const auto clipped=[&](double ratio) {
                        return std::isfinite(ratio)&&std::abs(ratio)>uncertainty&&std::abs(ratio)+uncertainty>=1;
                    };
                    const double dx=(q[1]+q[3]-q[0]-q[2])/total,dy=(q[0]+q[1]-q[2]-q[3])/total;
                    const bool xClipped=clipped(dx),yClipped=clipped(dy);
                    quadrantUnresolved=xClipped||yClipped;
                    {
                        const auto origin=unit(image.pointing),ray=unit(feature.direction);
                        auto right=unit(cross(origin,unit(thermalFrame->up)));
                        if(norm(right)<.5)right=unit(cross(origin,std::abs(origin.z)<.9?V{0,0,1}:V{0,1,0}));
                        const auto up=unit(cross(right,origin));
                        while(!quadrantBounds_.empty()&&f.timeS-quadrantBounds_.front().timeS>p.memoryS)
                            quadrantBounds_.pop_front();
                        const auto constrain=[&](double ratio,bool unresolved,V direction,double halfField) {
                            if(std::abs(ratio)<=uncertainty)return;
                            // A quadrant constrains a hemisphere, not one point.
                            // erf(z)<=2*z/sqrt(pi) gives a conservative minimum
                            // offset when the magnitude is unresolved.
                            const double minimum=unresolved?std::copysign(std::min(halfField,
                                std::sqrt(pi/2)*image.config.psfSigmaRad*(std::abs(ratio)-uncertainty)),ratio):0;
                            const auto normal=scale(subtract(direction,scale(origin,std::tan(minimum))),std::copysign(1.,ratio));
                            quadrantBounds_.push_back({normal,f.timeS});
                        };
                        const double halfX=.5*image.width*image.config.pixelPitchRad,halfY=.5*image.height*image.config.pixelPitchRad;
                        constrain(dx,xClipped,right,halfX);constrain(dy,yClipped,up,halfY);
                        while(quadrantBounds_.size()>2048)quadrantBounds_.pop_front();
                        double x=std::atan2(dot(ray,right),dot(ray,origin)),y=std::atan2(dot(ray,up),dot(ray,origin));
                        if(xClipped||yClipped) {
                            struct Vertex {double x,y;};std::vector<Vertex> polygon;
                            for(;;) {
                                const double ax=std::tan(halfX),ay=std::tan(halfY);
                                polygon={{-ax,-ay},{ax,-ay},{ax,ay},{-ax,ay}};
                                for(const auto& constraint:quadrantBounds_) {
                                    if(polygon.empty())break;
                                    const double a=dot(constraint.normal,right),b=dot(constraint.normal,up),c=dot(constraint.normal,origin);
                                    std::vector<Vertex> intersection;auto previous=polygon.back();
                                    double previousValue=a*previous.x+b*previous.y+c;
                                    for(const auto current:polygon) {
                                        const double value=a*current.x+b*current.y+c;
                                        if((value>=0)!=(previousValue>=0)) {
                                            const double fraction=previousValue/(previousValue-value);
                                            intersection.push_back({previous.x+fraction*(current.x-previous.x),previous.y+fraction*(current.y-previous.y)});
                                        }
                                        if(value>=0)intersection.push_back(current);
                                        previous=current;previousValue=value;
                                    }
                                    polygon=std::move(intersection);
                                }
                                if(!polygon.empty())break;
                                // Source motion or changed aggregate contrast
                                // contradicts old constraints. Drop the oldest;
                                // no source identity supplies a replacement.
                                quadrantBounds_.pop_front();
                            }
                            double centreX=0,centreY=0;
                            for(const auto vertex:polygon){centreX+=vertex.x;centreY+=vertex.y;}
                            x=std::atan(centreX/static_cast<double>(polygon.size()));y=std::atan(centreY/static_cast<double>(polygon.size()));
                        }
                        quadrantCentering=unit(add(origin,add(scale(right,std::tan(x)),scale(up,std::tan(y)))));
                    }
                } else if(spec_.seeker=="infrared")quadrantBounds_={};
            }
            for(std::size_t featureIndex=0;featureIndex<features.size();++featureIndex) {
                const auto& feature=features[featureIndex];
                const auto reconstructed=reticlePose?rotateFrame(feature.direction,reticlePose->physicalAxes,reticlePose->navigation.bodyToEcef):
                    rotateFrame(feature.direction,*f.bodyAxes,navigation->bodyToEcef);
                if(cached_.tracking&&cached_.track&&!measuredAssociationV1(reconstructed,*cached_.track,
                    captureTimeS,f.timeS,thermalConfig_->associationGateRad))continue;
                SeekerSampleV2 measured;measured.direction=reconstructed;measured.signal=feature.signalToNoise;
                if(spec_.seeker=="imaging_infrared")measured.imageExtent=feature.extent;
                exposure.samples.push_back(measured);
                if(dualFrame)dualFeatureIndices.push_back(featureIndex);
                indices.push_back(feature.refereeAttributionIndex.value_or(std::numeric_limits<std::size_t>::max()));
                angularVariances.push_back(feature.angularVarianceRad2);
            }
        }
        if(radioFrame&&radioReady) {
            const auto signal=radio_.expose(*radioFrame,pointing_,*f.bodyAxes,f.receiverSeed,semi);
            if(!signal.valid)return {};
            for(const auto& feature:signal.measurements) {
                const auto reconstructed=feature.navigationReconstructed?feature.direction:
                    rotateFrame(feature.direction,*f.bodyAxes,navigation->bodyToEcef);
                if(cached_.tracking&&cached_.track&&dot(reconstructed,unit(add(cached_.track->direction,
                    scale(cached_.track->directionRate,std::max(0.,f.timeS-cached_.track->sampleTimeS)))))<
                    std::cos(radioConfig_->associationGateRad))continue;
                SeekerSampleV2 measured;measured.direction=reconstructed;measured.signal=feature.signalToNoise;
                if(!semi){measured.rangeM=feature.rangeM;measured.rangeRateMs=feature.rangeRateMs;}
                if(semi){measured.illuminator=f.configuredIlluminator;measured.transmitterPathClear=true;measured.receiverPathClear=true;}
                exposure.samples.push_back(measured);indices.push_back(feature.refereeAttributionIndex.value_or(std::numeric_limits<std::size_t>::max()));
                angularVariances.push_back(feature.angularVarianceRad2);
            }
        }
        // One effective resolution-cell noise trial, independent of scene
        // candidates. A noise contact has no source identity or attribution.
        const auto uniform=[&]{return (static_cast<double>(backgroundRng_()>>11)+.5)/9007199254740992.;};
        const double falseAlarm=heat?.5*std::erfc(p.detectionSnr/std::sqrt(2.)):std::exp(-p.detectionSnr);
        if(!thermalFrame&&!radioFrame&&uniform()<falseAlarm) {
            SeekerSampleV2 noise;
            const auto u=unit(cross(pointing_,std::abs(pointing_.z)<.9?V{0,0,1}:V{0,1,0}));
            noise.direction=unit(add(pointing_,add(scale(u,(2*uniform()-1)*p.angleResolutionRad),
                scale(cross(pointing_,u),(2*uniform()-1)*p.angleResolutionRad))));
            noise.signal=p.detectionSnr;
            if(!heat&&!semi) {
                noise.rangeM=config().minimumRangeM+uniform()*spec_.acquireM;
                noise.rangeRateMs=(2*uniform()-1)*norm(f.velocity);
            }
            if(!semi||f.illuminated) {
                if(semi) {noise.illuminator=f.configuredIlluminator;noise.transmitterPathClear=true;noise.receiverPathClear=true;}
                exposure.samples.push_back(noise);indices.push_back(std::numeric_limits<std::size_t>::max());
                angularVariances.push_back(p.angleResolutionRad*p.angleResolutionRad/3);
            }
        }
        // A missed dwell is not a new target. Preserve the measured angular
        // association/filter inside its authored memory, while continuing to
        // publish the actual absence of a detection. Expiry forgets only that
        // association, never the authorized navigation or active-radar phase.
        if(filtered_) {
            if(f.timeS-filtered_->sampleTimeS<=p.memoryS)family_->seedTrack(*filtered_);
            else {family_->forgetTrack();filtered_.reset();filteredQuadrantUnresolved_=false;}
        }
        auto result=receiverCaptureReady?family_->sample(exposure):SeekerResultV2{};
        if(!receiverCaptureReady){cached_={};cached_.valid=true;}
        if(result.valid) {
            if(dualFrame&&result.tracking) {
                // At the raw family-return seam this is the ACTUAL selected
                // index into exposure.samples. It has not been converted to
                // referee provenance, and no filtered-track guess is used.
                const auto selectedMeasuredSample=result.attributionIndex;
                if(!selectedMeasuredSample||*selectedMeasuredSample>=dualFeatureIndices.size()||
                    !thermalDual_.commitSelectedMeasurementV1(f.timeS,dualFeatureIndices[*selectedMeasuredSample]))return {};
            }
            if(result.attributionIndex&&*result.attributionIndex<indices.size()) {
                const auto index=indices[*result.attributionIndex];
                if(index==std::numeric_limits<std::size_t>::max())result.attributionIndex.reset();else result.attributionIndex=index;
            }
            if(result.tracking&&result.track) {
                auto track=*result.track;
                if(!quadrantUnresolved&&!filteredQuadrantUnresolved_&&filtered_&&track.localId==filtered_->localId&&track.sampleTimeS>filtered_->sampleTimeS) {
                    const double elapsed=track.sampleTimeS-filtered_->sampleTimeS;
                    const double alpha=1-std::exp(-elapsed/p.trackLagS),beta=alpha*alpha;
                    const auto predicted=unit(add(filtered_->direction,scale(filtered_->directionRate,elapsed)));
                    const auto residual=subtract(track.direction,predicted);
                    track.direction=unit(add(predicted,scale(residual,alpha)));
                    track.directionRate=add(filtered_->directionRate,scale(residual,beta/elapsed));
                    track.directionRate=subtract(track.directionRate,scale(track.direction,dot(track.direction,track.directionRate)));
                    track.rateAvailable=true;
                }
                if(quadrantUnresolved||filteredQuadrantUnresolved_) {
                    // Saturated quadrant ratios constrain a centering region,
                    // not a resolved angular position. Differencing their
                    // inverse-erf clamp would turn servo motion into target
                    // velocity. Two resolved exposures must bracket a rate.
                    track.directionRate={};track.rateAvailable=false;
                }
                std::optional<LauncherEstimateV2> supportPrior;
                if(radioFrame&&!semi&&spec_.inertialMidcourse&&spec_.launcherUpdates&&f.supported&&
                   f.launcherEstimate&&track.rangeM&&track.rangeRateMs&&track.rateAvailable&&navigation&&
                   track.sampleTimeS==navigation->timeS&&
                   f.launcherEstimate->sampleTimeS>=(radarNavigationTrack_==track.localId?radarSupportEpochS_:-1)) {
                    const auto& prior=*f.launcherEstimate;
                    const auto cue=subtract(add(prior.position,scale(prior.velocity,
                        track.sampleTimeS-prior.sampleTimeS)),navigation->position);
                    if(finite(cue)&&positive(norm(cue))&&dot(unit(cue),track.direction)>=std::cos(spec_.cueRad)) {
                        // A supported kinematic rate, not an independent
                        // Doppler-derived transverse velocity measurement.
                        // Keep the original prior epoch/provenance separately.
                        const auto relativeVelocity=subtract(prior.velocity,navigation->velocity);
                        track.directionRate=scale(subtract(relativeVelocity,
                            scale(track.direction,dot(relativeVelocity,track.direction))),1 / *track.rangeM);
                        supportPrior=prior;
                    }
                }
                if(!validTrack(track,config().minimumRangeM)) {
                    // Keep the family's current finite detector state, but do
                    // not publish a failed derived estimate or retain it for
                    // the next prediction. No target truth supplies a repair.
                    filtered_.reset();filteredQuadrantUnresolved_=false;result.tracking=false;result.track.reset();result.attributionIndex.reset();
                } else {
                    filtered_=track;filteredQuadrantUnresolved_=quadrantUnresolved;family_->seedTrack(track);result.track=track;
                    if(radioFrame&&!semi&&spec_.inertialMidcourse) {
                        if(radarNavigationTrack_!=track.localId) {
                            radarNavigation_.reset();radarNavigationTrack_=track.localId;
                            radarNavigationSupportPrior_.reset();radarSupportEpochS_=-1;
                        }
                        if(track.rangeM&&track.rangeRateMs&&track.rateAvailable&&
                           navigation&&track.sampleTimeS==navigation->timeS) {
                            // Range and LOS share this capture's own-navigation
                            // origin. Keep its correlated error during inertial
                            // coast instead of switching to launcher coordinates.
                            LauncherEstimateV2 measured;
                            measured.position=add(navigation->position,scale(track.direction,*track.rangeM));
                            measured.velocity=add(navigation->velocity,add(scale(track.direction,*track.rangeRateMs),
                                scale(track.directionRate,*track.rangeM)));
                            measured.sampleTimeS=track.sampleTimeS;
                            if(finite(measured.position)&&finite(measured.velocity)) {
                                radarNavigation_=measured;radarNavigationSupportPrior_=supportPrior;
                                if(supportPrior)radarSupportEpochS_=supportPrior->sampleTimeS;
                            }
                        }
                    }
                    // Conservative detector-error bound also covers unresolved
                    // intensity centroids; no fitted tracker covariance is claimed.
                    result.angularVarianceRad2=*std::max_element(angularVariances.begin(),angularVariances.end());
                    result.covarianceAvailable=true;
                }
            }
            if(radioFrame&&!semi&&spec_.inertialMidcourse&&radarNavigation_)
                result.midcourseEstimate=radarNavigation_;
            if(radioFrame&&!semi&&!radioReady) {
                // Native phase/navigation decision, not a nonexistent RF
                // exposure. Standby/warm-up consumes no receiver noise or
                // transmit energy and carries no fabricated detector stamp.
                result.measurementTimeS.reset();result.processingReadyTimeS.reset();result.deliveredTimeS.reset();
                cached_=result;
            } else {
                result.measurementTimeS=captureTimeS;result.processingReadyTimeS=processingStartS+p.processingDelayS;
                pending_.push_back({processingStartS+p.processingDelayS,result,quadrantCentering});
            }
        }
        nextSampleS_=f.timeS+p.samplePeriodS;
    }
    while(!pending_.empty()&&pending_.front().deliverS<=f.timeS+1e-12) {
        quadrantCentering_=pending_.front().quadrantCentering;
        cached_=std::move(pending_.front().result);cached_.deliveredTimeS=f.timeS;pending_.pop_front();
    }
    if(pending_.size()>2048) {pending_.clear();quadrantCentering_.reset();quadrantBounds_={};cached_={};}
    if(cached_.measurementTimeS&&f.timeS-*cached_.measurementTimeS>p.memoryS+p.processingDelayS) {
        cached_.tracking=false;cached_.attributionIndex.reset();
        quadrantCentering_.reset();
        quadrantBounds_={};
    }
    cached_.pointingDirection=pointing_;lastTimeS_=f.timeS;return cached_;
}
}
