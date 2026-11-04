#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include "contracts/world_stream/WorldLayout.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>
#include <new>
#include <vector>
#ifndef _WIN32
#include <sys/mman.h>
#endif

namespace fsim::ipc {
namespace {
constexpr std::uint32_t magic=0x57444731,logMagic=0x57444c31,endian=0x01020304;
constexpr std::size_t frameWords=sizeof(WeaponDiagnosticFrameV1)/4;
constexpr std::size_t selectionWords=sizeof(WeaponDiagnosticSelectionV1)/4;
static_assert(std::atomic<std::uint32_t>::is_always_lock_free&&std::atomic<std::uint64_t>::is_always_lock_free,
              "Weapon diagnostics require lock-free shared integer atomics");
struct alignas(64) AtomicFrame {
    std::atomic<std::uint64_t> guard{0};
    std::atomic<std::uint32_t> words[frameWords]{};
};
struct LogHeader {
    std::uint32_t magic=logMagic,version=1,frameBytes=sizeof(WeaponDiagnosticFrameV1),byteOrder=endian;
    std::uint64_t pid=0;
    std::uint32_t incarnation=0,reserved=0;
};
static_assert(sizeof(LogHeader)==32);
struct LogRecord {std::uint32_t bytes=sizeof(WeaponDiagnosticFrameV1),crc=0;};
static_assert(sizeof(LogRecord)==8);
bool validStatus(WeaponDiagnosticStatusV1 s){return static_cast<std::uint32_t>(s)<=7;}
template<std::size_t N> bool terminated(const char (&text)[N]){return std::find(text,text+N,'\0')!=text+N;}
template<std::size_t N> bool finite(const double (&data)[N]){return std::all_of(data,data+N,[](double v){return std::isfinite(v);});}
bool nonnegative(double v){return std::isfinite(v)&&v>=0;}
bool validStamp(const WeaponDiagnosticStampV1& s,const WeaponDiagnosticFrameV1& f){
    if(!validStatus(s.status)||!terminated(s.cause))return false;
    const double times[]={s.sampleTimeS,s.processingTimeS,s.deliveredTimeS,s.validUntilS};
    if(!finite(times))return false;
    if(s.status!=WeaponDiagnosticStatusV1::Available)return true;
    return f.entity&&f.generation&&s.sourceFrame&&s.sourceFrame<=f.worldFrame&&s.sampleTimeS>=0&&
        s.processingTimeS>=s.sampleTimeS&&s.deliveredTimeS>=s.processingTimeS&&
        s.deliveredTimeS<=f.simTimeS&&s.validUntilS>=s.sampleTimeS;
}
bool unitQuaternion(const double (&q)[4]){
    if(!finite(q))return false;
    double n=0;for(double v:q)n+=v*v;return std::abs(n-1)<1e-4;
}
std::string segmentName(const std::string& world,std::uint64_t pid,std::uint32_t incarnation){
    return "fsim.weapon-diagnostics.v1."+world+"."+std::to_string(pid)+"."+std::to_string(incarnation);
}
bool validWorld(const std::string& world){
    return !world.empty()&&world.size()<=160&&world.find('\0')==std::string::npos&&world.find('/')==std::string::npos&&world.find('\\')==std::string::npos;
}
template<class T,std::size_t N> void atomicWrite(std::atomic<std::uint32_t> (&words)[N],const T& value){
    static_assert(sizeof(T)==4*N);const auto* bytes=reinterpret_cast<const unsigned char*>(&value);
    for(std::size_t i=0;i<N;++i){std::uint32_t word;std::memcpy(&word,bytes+4*i,4);words[i].store(word,std::memory_order_release);}
}
template<class T,std::size_t N> void atomicRead(const std::atomic<std::uint32_t> (&words)[N],T& value){
    static_assert(sizeof(T)==4*N);auto* bytes=reinterpret_cast<unsigned char*>(&value);
    for(std::size_t i=0;i<N;++i){const auto word=words[i].load(std::memory_order_acquire);std::memcpy(bytes+4*i,&word,4);}
}
std::uint32_t crc32(const void* data,std::size_t size){
    const auto* bytes=static_cast<const unsigned char*>(data);std::uint32_t crc=0xffffffffu;
    for(std::size_t i=0;i<size;++i){crc^=bytes[i];for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((crc&1)?0xedb88320u:0u);}
    return ~crc;
}
using Packet=std::vector<unsigned char>;
void appendBytes(Packet& out,const void* value,std::size_t bytes){const auto* p=static_cast<const unsigned char*>(value);out.insert(out.end(),p,p+bytes);}
Packet pack(const WeaponDiagnosticFrameV1& f){
    Packet out;out.reserve(32768);
    appendBytes(out,&f,offsetof(WeaponDiagnosticFrameV1,roster));appendBytes(out,f.roster,f.rosterCount*sizeof(f.roster[0]));
    const auto& t=f.thermal;appendBytes(out,&t,offsetof(WeaponDiagnosticThermalV1,expectedElectrons));const auto pixels=static_cast<std::size_t>(t.width)*t.height;
    if(t.flags&WeaponDiagnosticExpectedPixelsV1)appendBytes(out,t.expectedElectrons,pixels*sizeof(float));
    appendBytes(out,t.measuredElectrons,pixels*sizeof(float));if(t.flags&WeaponDiagnosticSegmentationV1)appendBytes(out,t.segmentation,pixels*sizeof(std::uint32_t));
    const auto& r=f.rf;appendBytes(out,&r,offsetof(WeaponDiagnosticRfV1,powerNoiseRelative));const auto cells=static_cast<std::size_t>(r.rows)*r.columns;
    appendBytes(out,r.powerNoiseRelative,cells*sizeof(float));if(r.flags&WeaponDiagnosticRfThresholdsV1)appendBytes(out,r.thresholdNoiseRelative,cells*sizeof(float));
    if(r.flags&WeaponDiagnosticRfLocalNoiseV1)appendBytes(out,r.localNoiseRelative,cells*sizeof(float));
    if(r.flags&WeaponDiagnosticRfTestedCellsV1)appendBytes(out,r.tested,cells);
    appendBytes(out,r.peaks,r.peakCount*sizeof(r.peaks[0]));
    appendBytes(out,&f.navigation,sizeof(f.navigation));appendBytes(out,&f.guidance,sizeof(f.guidance));
    appendBytes(out,&f.link,offsetof(WeaponDiagnosticLinkV1,sources));appendBytes(out,f.link.sources,f.link.sourceCount*sizeof(f.link.sources[0]));
    appendBytes(out,&f.control,offsetof(WeaponDiagnosticControlV1,fins));appendBytes(out,f.control.fins,f.control.finCount*sizeof(f.control.fins[0]));
    appendBytes(out,&f.propulsion,sizeof(f.propulsion));appendBytes(out,&f.effects,offsetof(WeaponDiagnosticEffectsV1,fragments));
    appendBytes(out,f.effects.fragments,f.effects.fragmentCount*sizeof(f.effects.fragments[0]));appendBytes(out,f.effects.damage,f.effects.damageCount*sizeof(f.effects.damage[0]));
    appendBytes(out,f.events,f.eventCount*sizeof(f.events[0]));return out;
}
bool unpack(const Packet& packet,WeaponDiagnosticFrameV1& f){
    std::size_t at=0;
    const auto take=[&](void* out,std::size_t bytes){if(bytes>packet.size()-at)return false;std::memcpy(out,packet.data()+at,bytes);at+=bytes;return true;};
    if(!take(&f,offsetof(WeaponDiagnosticFrameV1,roster))||f.rosterCount>128||f.eventCount>128||!take(f.roster,f.rosterCount*sizeof(f.roster[0])))return false;
    auto& t=f.thermal;if(!take(&t,offsetof(WeaponDiagnosticThermalV1,expectedElectrons))||t.width>128||t.height>128)return false;
    const auto pixels=static_cast<std::size_t>(t.width)*t.height;
    if((t.flags&WeaponDiagnosticExpectedPixelsV1)&&!take(t.expectedElectrons,pixels*sizeof(float)))return false;
    if(!take(t.measuredElectrons,pixels*sizeof(float)))return false;
    if((t.flags&WeaponDiagnosticSegmentationV1)&&!take(t.segmentation,pixels*sizeof(std::uint32_t)))return false;
    auto& r=f.rf;if(!take(&r,offsetof(WeaponDiagnosticRfV1,powerNoiseRelative))||r.rows>2048||r.columns>256||r.peakCount>64)return false;
    const auto cells=static_cast<std::size_t>(r.rows)*r.columns;if(cells>kWeaponDiagnosticRfCellsV1||!take(r.powerNoiseRelative,cells*sizeof(float)))return false;
    if((r.flags&WeaponDiagnosticRfThresholdsV1)&&!take(r.thresholdNoiseRelative,cells*sizeof(float)))return false;
    if((r.flags&WeaponDiagnosticRfLocalNoiseV1)&&!take(r.localNoiseRelative,cells*sizeof(float)))return false;
    if((r.flags&WeaponDiagnosticRfTestedCellsV1)&&!take(r.tested,cells))return false;
    if(!take(r.peaks,r.peakCount*sizeof(r.peaks[0]))||!take(&f.navigation,sizeof(f.navigation))||!take(&f.guidance,sizeof(f.guidance)))return false;
    if(!take(&f.link,offsetof(WeaponDiagnosticLinkV1,sources))||f.link.sourceCount>64||!take(f.link.sources,f.link.sourceCount*sizeof(f.link.sources[0])))return false;
    if(!take(&f.control,offsetof(WeaponDiagnosticControlV1,fins))||f.control.finCount>32||!take(f.control.fins,f.control.finCount*sizeof(f.control.fins[0])))return false;
    if(!take(&f.propulsion,sizeof(f.propulsion))||!take(&f.effects,offsetof(WeaponDiagnosticEffectsV1,fragments))||f.effects.fragmentCount>128||f.effects.damageCount>64)return false;
    return take(f.effects.fragments,f.effects.fragmentCount*sizeof(f.effects.fragments[0]))&&take(f.effects.damage,f.effects.damageCount*sizeof(f.effects.damage[0]))&&
        take(f.events,f.eventCount*sizeof(f.events[0]))&&at==packet.size();
}
}

const char* weaponDiagnosticStatusTextV1(WeaponDiagnosticStatusV1 status) noexcept {
    switch(status){
    case WeaponDiagnosticStatusV1::Unavailable:return "unavailable";
    case WeaponDiagnosticStatusV1::Available:return "available";
    case WeaponDiagnosticStatusV1::NotApplicable:return "not applicable";
    case WeaponDiagnosticStatusV1::Pending:return "pending processing/delivery";
    case WeaponDiagnosticStatusV1::Expired:return "expired";
    case WeaponDiagnosticStatusV1::Removed:return "removed / old generation";
    case WeaponDiagnosticStatusV1::CapacityExceeded:return "native data exceed sidecar capacity";
    case WeaponDiagnosticStatusV1::Rejected:return "native processing rejected";
    }
    return "invalid status";
}
bool weaponDiagnosticFreshV1(const WeaponDiagnosticStampV1& s,double time) noexcept {
    return s.status==WeaponDiagnosticStatusV1::Available&&std::isfinite(time)&&
        nonnegative(s.sampleTimeS)&&std::isfinite(s.deliveredTimeS)&&std::isfinite(s.validUntilS)&&
        s.sampleTimeS<=s.deliveredTimeS&&s.deliveredTimeS<=time&&time<=s.validUntilS;
}
bool validateWeaponDiagnosticsV1(const WeaponDiagnosticFrameV1& f,const char** reason) noexcept {
    const auto fail=[&](const char* why){if(reason)*reason=why;return false;};if(reason)*reason=nullptr;
    if(f.version!=1||!f.publisherPid||!f.publisherIncarnation||!f.worldFrame||!nonnegative(f.simTimeS)||f.reserved||
       !validStatus(f.selectionStatus)||!terminated(f.name)||!terminated(f.store)||
       ((f.entity==0)!=(f.generation==0))||
       (f.selectionStatus==WeaponDiagnosticStatusV1::Available&&(!f.entity||!f.generation)))return fail("frame identity or version");
    if(f.rosterCount>kWeaponDiagnosticRosterV1||f.totalWeaponCount<f.rosterCount||f.eventCount>kWeaponDiagnosticEventsV1)return fail("frame capacity");
    for(std::uint32_t i=0;i<f.rosterCount;++i){
        const auto& row=f.roster[i];if(!row.entity||!row.generation||!terminated(row.name)||!terminated(row.store))return fail("roster identity");
        for(std::uint32_t j=0;j<i;++j)if(f.roster[j].entity==row.entity&&f.roster[j].generation==row.generation)return fail("duplicate roster identity");
    }
    const WeaponDiagnosticStampV1* stamps[]={&f.thermal.stamp,&f.rf.stamp,&f.navigation.stamp,&f.guidance.stamp,&f.guidance.measurementStamp,&f.link.stamp,&f.control.stamp,&f.propulsion.stamp,&f.effects.stamp};
    for(const auto* stamp:stamps)if(!validStamp(*stamp,f))return fail("subsystem epoch, status or identity");
    const auto& t=f.thermal;
    if(t.width>128||t.height>128||((t.width==0)!=(t.height==0))||
       (t.stamp.status==WeaponDiagnosticStatusV1::Available&&!t.width))return fail("thermal dimensions");
    const double thermalScalars[]={t.horizontalHalfAngleRad,t.verticalHalfAngleRad,t.centroidColumn,t.centroidRow,t.centroidVarianceRad2,t.detectionSigma,t.readNoiseElectrons,t.darkCurrentElectronsS,t.exposureS,t.fullWellElectrons,t.displayMinimumElectrons,t.displayMaximumElectrons,t.centroidSnrAmplitude,t.centroidSignedContrastElectrons,t.centroidWidthRad,t.centroidHeightRad,t.quadrantGuardBackgroundElectrons,t.quadrantVarianceElectrons2};
    if(!finite(thermalScalars)||!finite(t.pointingEcef)||!finite(t.quadrantContrastElectrons)||t.centroidVarianceRad2<0||t.readNoiseElectrons<0||t.darkCurrentElectronsS<0||t.adcBits>32||t.reserved)return fail("thermal scalar");
    if(t.stamp.status==WeaponDiagnosticStatusV1::Available&&(!(t.exposureS>0)||!(t.fullWellElectrons>0)||!t.adcBits))return fail("thermal detector policy");
    if((t.flags&WeaponDiagnosticNativeDisplayScaleV1)&&!(t.displayMaximumElectrons>t.displayMinimumElectrons))return fail("thermal display scale");
    if(t.width&&(!(t.horizontalHalfAngleRad>0&&t.horizontalHalfAngleRad<1.5707963267948966)||
                 !(t.verticalHalfAngleRad>0&&t.verticalHalfAngleRad<1.5707963267948966)))return fail("thermal angular field");
    if((t.flags&WeaponDiagnosticCentroidV1)&&(!t.width||t.centroidColumn<0||t.centroidColumn>=t.width||t.centroidRow<0||t.centroidRow>=t.height))return fail("thermal centroid");
    for(std::size_t i=0;i<static_cast<std::size_t>(t.width)*t.height;++i)if(!std::isfinite(t.measuredElectrons[i])||((t.flags&WeaponDiagnosticExpectedPixelsV1)&&!std::isfinite(t.expectedElectrons[i])))return fail("thermal pixel");
    const auto& r=f.rf;
    if(r.rows>kWeaponDiagnosticRfRowsV1||r.columns>kWeaponDiagnosticRfColumnsV1||
       static_cast<std::size_t>(r.rows)*r.columns>kWeaponDiagnosticRfCellsV1||((r.rows==0)!=(r.columns==0))||r.peakCount>64||
       (r.stamp.status==WeaponDiagnosticStatusV1::Available&&!r.rows))return fail("RF dimensions");
    const double rfScalars[]={r.firstRangeM,r.rangeBinM,r.firstClosingMs,r.closingBinMs,r.frequencyHz,r.prfHz,r.bandwidthHz,r.ambiguousRangeM,r.ambiguousClosingMs,r.clutterWallS,r.processingWallS};
    if(!finite(rfScalars)||!finite(r.pointingEcef)||
       ((r.flags&WeaponDiagnosticRangeAxisV1)&&(!nonnegative(r.firstRangeM)||r.rangeBinM<=0))||
       ((r.flags&WeaponDiagnosticDopplerAxisV1)&&r.closingBinMs<=0))return fail("RF axes");
    for(std::size_t i=0;i<static_cast<std::size_t>(r.rows)*r.columns;++i)if(!nonnegative(r.powerNoiseRelative[i])||!nonnegative(r.thresholdNoiseRelative[i])||!nonnegative(r.localNoiseRelative[i]))return fail("RF cell");
    if(r.flags&WeaponDiagnosticRfTestedCellsV1)for(std::size_t i=0;i<static_cast<std::size_t>(r.rows)*r.columns;++i)if(r.tested[i]>1)return fail("RF tested-cell mask");
    for(std::uint32_t i=0;i<r.peakCount;++i){
        const auto& p=r.peaks[i];if(p.row>=r.rows||p.column>=r.columns||p.reserved||!nonnegative(p.powerNoiseRelative)||!nonnegative(p.thresholdNoiseRelative)||
            !nonnegative(p.snrLinear)||!std::isfinite(p.rangeM)||!std::isfinite(p.closingMs))return fail("RF peak");
    }
    const auto& n=f.navigation;
    if(n.errorStateCount!=0&&n.errorStateCount!=15&&n.errorStateCount!=21)return fail("navigation state count");
    if(!finite(n.positionEcefM)||!finite(n.velocityEcefMs)||!finite(n.accelerometerBiasBodyMs2)||!finite(n.gyroBiasBodyRadS)||
       !finite(n.measuredGyroBodyRadS)||!finite(n.measuredSpecificForceBodyMs2)||!std::isfinite(n.lastImuTimeS)||!std::isfinite(n.lastGnssTimeS))return fail("navigation scalar");
    if(n.stamp.status==WeaponDiagnosticStatusV1::Available&&(!n.errorStateCount||!unitQuaternion(n.quaternionBodyToEcef)))return fail("navigation attitude");
    if(n.flags&WeaponDiagnosticOwnReferenceV1){if(!finite(n.referencePositionEcefM)||!finite(n.referenceVelocityEcefMs)||!unitQuaternion(n.referenceQuaternionBodyToEcef))return fail("navigation referee reference");}
    for(std::uint32_t i=0;i<n.errorStateCount;++i)for(std::uint32_t j=0;j<n.errorStateCount;++j){
        const double value=n.errorCovariance[n.errorStateCount*i+j];if(!std::isfinite(value)||(i==j&&value<0))return fail("navigation covariance");
    }
    const auto& g=f.guidance;
    const double guidanceScalars[]={g.weaponAgeS,g.trackTimeS,g.rangeM,g.rangeRateMs,g.angularVarianceRad2,g.timeToGoS,g.timeToActiveS};
    if(g.reserved||!finite(guidanceScalars)||!finite(g.losDirectionEcef)||!finite(g.losDirectionRatePerS)||g.weaponAgeS<0||g.angularVarianceRad2<0||
       ((g.flags&WeaponDiagnosticLosV1)&&(g.trackTimeS<0||g.trackTimeS>f.simTimeS))||
       ((g.flags&WeaponDiagnosticMeasuredRangeV1)&&g.rangeM<0)||((g.flags&WeaponDiagnosticTimeToGoV1)&&g.timeToGoS<0)||
       ((g.flags&WeaponDiagnosticTimeToActiveV1)&&g.timeToActiveS<0))return fail("guidance measurement");
    const auto& l=f.link;
    const double linkScalars[]={l.stateTimeS,l.lastUpdateS,l.inputReceivedS,l.inputValidUntilS,l.updatePeriodS,l.feedbackPeriodS,l.latencyS,l.dataRateBps,l.rangeM,l.frequencyHz,l.resourceBusyUntilS};
    if(l.direction>2||l.sourceCount>64||!terminated(l.directionSource)||!finite(linkScalars)||!finite(l.stateEcef)||!finite(l.covarianceEcef))return fail("link scalar or source capacity");
    if((l.flags&WeaponDiagnosticInputKnownV1)&&(!l.fusedEntity||l.stateTimeS<0||l.stateTimeS>l.inputReceivedS||l.inputReceivedS>f.simTimeS||l.inputValidUntilS<0))return fail("uplink original epochs");
    if((l.flags&WeaponDiagnosticInputValidV1)&&(!(l.flags&WeaponDiagnosticInputKnownV1)||l.fusionState>=2||l.inputValidUntilS<f.simTimeS))return fail("uplink stale validity");
    for(std::uint32_t i=0;i<l.sourceCount;++i){const auto& s=l.sources[i];if(s.kind>1||!s.track||!nonnegative(s.sampleTimeS)||!nonnegative(s.receivedTimeS)||s.sampleTimeS>s.receivedTimeS||s.receivedTimeS>f.simTimeS)return fail("fusion source provenance");}
    const auto& c=f.control;
    const double controlScalars[]={c.controllerTimeS,c.navigationTimeS,c.tvcDemandPitchRad,c.tvcDemandYawRad,c.tvcActualPitchRad,c.tvcActualYawRad,c.tvcLimitRad,c.tvcRateRadS,c.allocatorResidual};
    if(c.finCount>32||!finite(controlScalars)||!finite(c.demandedAccelerationBodyMs2)||!finite(c.achievedAccelerationBodyMs2)||
       !finite(c.commands)||!finite(c.demandedMomentBodyNm)||!finite(c.achievedMomentBodyNm)||
       ((c.flags&WeaponDiagnosticControllerV1)&&(c.controllerTimeS<0||c.controllerTimeS>f.simTimeS)))return fail("control scalar");
    if((c.flags&WeaponDiagnosticControlNavigationEpochV1)&&(!(c.flags&WeaponDiagnosticControllerV1)||c.navigationTimeS<0||c.navigationTimeS>c.controllerTimeS))return fail("control navigation epoch");
    for(std::uint32_t i=0;i<c.finCount;++i){
        const auto& fin=c.fins[i];const double scalars[]={fin.demandRad,fin.actualRad,fin.minimumRad,fin.maximumRad,fin.rateRadS};
        if(!fin.id||fin.reserved||!finite(scalars)||((fin.flags&1)&&(fin.minimumRad>=fin.maximumRad||fin.rateRadS<0)))return fail("fin");
        for(std::uint32_t j=0;j<i;++j)if(fin.id==c.fins[j].id||fin.index==c.fins[j].index)return fail("duplicate native fin");
    }
    const auto& p=f.propulsion;
    const double motorScalars[]={p.fuelKg,p.nativeMotorAgeS,p.secondIgnitionAgeS,p.valveDemand,p.valveActual,p.inletAirKgS,p.inletTemperatureK,p.combustorTemperatureK,p.netThrustN};
    if(p.engineCount>4||!finite(p.thrustN)||!finite(p.fuelFlowKgS)||!finite(motorScalars)||p.fuelKg<0||p.nativeMotorAgeS<0)return fail("propulsion scalar");
    for(std::uint32_t i=0;i<p.engineCount;++i)if(p.fuelFlowKgS[i]<0)return fail("negative fuel flow");
    const auto& e=f.effects;
    const double effectScalars[]={e.armingTimeS,e.burstTimeS,e.fuzeRadiusM,e.explosiveKg,e.caseMassKg};
    if(e.fragmentCount>128||e.totalFragmentCount<e.fragmentCount||e.damageCount>64||e.totalDamageCount<e.damageCount||e.reserved||
       !finite(effectScalars)||!finite(e.burstPositionEcefM)||e.fuzeRadiusM<0||e.explosiveKg<0||e.caseMassKg<0)return fail("effects scalar or capacity");
    for(std::uint32_t i=0;i<e.fragmentCount;++i){const auto& v=e.fragments[i];if(!nonnegative(v.sampleTimeS)||v.sampleTimeS>f.simTimeS||!finite(v.positionEcefM)||!finite(v.velocityEcefMs)||!nonnegative(v.massKg)||!nonnegative(v.areaM2))return fail("fragment");}
    for(std::uint32_t i=0;i<e.damageCount;++i){const auto& v=e.damage[i];if(!v.victim||!v.victimGeneration||v.reserved||!nonnegative(v.sampleTimeS)||v.sampleTimeS>f.simTimeS||!nonnegative(v.penetratedEnergyJ)||!std::isfinite(v.health)||!nonnegative(v.removedMassKg))return fail("damage component");}
    for(std::uint32_t i=0;i<f.eventCount;++i){const auto& v=f.events[i];if(((v.flags&WeaponDiagnosticEventSourceGenerationV1)&&(!v.source||!v.sourceGeneration))||((v.flags&WeaponDiagnosticEventTargetGenerationV1)&&(!v.target||!v.targetGeneration))||!v.serial||!nonnegative(v.timeS)||v.timeS>f.simTimeS||!std::isfinite(v.value)||!terminated(v.cause)||!terminated(v.unit)||(i&&v.serial<=f.events[i-1].serial))return fail("native timeline");}
    return true;
}

struct alignas(64) WeaponDiagnosticsV1::Shared {
    std::atomic<std::uint32_t> ready{0},magicValue{0},version{0},frameBytes{0},incarnation{0};
    std::atomic<std::uint64_t> pid{0},latest{0},selectionGuard{0};
    std::atomic<std::uint32_t> selection[selectionWords]{};
    AtomicFrame slots[3];
};
WeaponDiagnosticsV1::~WeaponDiagnosticsV1(){close();}
void WeaponDiagnosticsV1::close() noexcept {
    if(owner_&&shared_&&shared_->pid.load(std::memory_order_acquire)==pid_&&shared_->incarnation.load(std::memory_order_acquire)==incarnation_)shared_->ready.store(0,std::memory_order_release);
#ifndef _WIN32
    if(owner_&&memory_.valid()){
        char name[256];const int length=std::snprintf(name,sizeof(name),"/%s",memory_.name().c_str());
        if(length>0&&static_cast<std::size_t>(length)<sizeof(name))::shm_unlink(name);
    }
#endif
    shared_=nullptr;owner_=false;pid_=lastFrame_=0;incarnation_=0;lastTimeS_=0;memory_.close();
}
bool WeaponDiagnosticsV1::valid() const noexcept {
    return shared_&&shared_->ready.load(std::memory_order_acquire)==1&&shared_->magicValue.load(std::memory_order_relaxed)==magic&&
        shared_->version.load(std::memory_order_relaxed)==1&&shared_->frameBytes.load(std::memory_order_relaxed)==sizeof(WeaponDiagnosticFrameV1)&&
        shared_->pid.load(std::memory_order_relaxed)==pid_&&shared_->incarnation.load(std::memory_order_relaxed)==incarnation_&&platform::processAlive(pid_);
}
bool WeaponDiagnosticsV1::create(const std::string& world,std::uint64_t pid,std::uint32_t incarnation){
    close();if(!validWorld(world)||pid!=platform::currentProcessId()||!incarnation||!memory_.create(segmentName(world,pid,incarnation),sizeof(Shared)))return false;
    auto* h=static_cast<Shared*>(memory_.data());
    // Identity is part of the mapping name: never reconstruct atomics under
    // readers retaining a prior publisher's generation or a concurrent request.
    if(!memory_.created()){memory_.close();return false;}
    new(h)Shared{};h->ready.store(2,std::memory_order_release);
    h->magicValue.store(magic);h->version.store(1);h->frameBytes.store(sizeof(WeaponDiagnosticFrameV1));h->pid.store(pid);h->incarnation.store(incarnation);h->latest.store(0);
    h->selectionGuard.store(0);for(auto& word:h->selection)word.store(0);
    for(auto& slot:h->slots){slot.guard.store(0);for(auto& word:slot.words)word.store(0);}
    shared_=h;pid_=pid;incarnation_=incarnation;owner_=true;h->ready.store(1,std::memory_order_release);return true;
}
bool WeaponDiagnosticsV1::open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation){
    close();if(!validWorld(world)||!pid||!incarnation||!memory_.open(segmentName(world,pid,incarnation),sizeof(Shared)))return false;
    shared_=static_cast<Shared*>(memory_.data());pid_=pid;incarnation_=incarnation;
    if(!valid()){close();return false;}return true;
}
bool WeaponDiagnosticsV1::publish(const WeaponDiagnosticFrameV1& f) noexcept {
    if(!owner_||!valid()||f.publisherPid!=pid_||f.publisherIncarnation!=incarnation_||f.worldFrame<=lastFrame_||f.simTimeS<lastTimeS_||!validateWeaponDiagnosticsV1(f))return false;
    auto& slot=shared_->slots[f.worldFrame%3];const auto guard=slot.guard.load(std::memory_order_relaxed);
    if((guard&1)||guard>std::numeric_limits<std::uint64_t>::max()-2)return false;
    slot.guard.store(guard+1,std::memory_order_seq_cst);atomicWrite(slot.words,f);slot.guard.store(guard+2,std::memory_order_seq_cst);
    lastFrame_=f.worldFrame;lastTimeS_=f.simTimeS;shared_->latest.store(f.worldFrame,std::memory_order_release);return true;
}
bool WeaponDiagnosticsV1::read(std::uint64_t sequence,WeaponDiagnosticFrameV1& out) const noexcept {
    if(!sequence||!valid())return false;
    const auto& slot=shared_->slots[sequence%3];
    std::unique_ptr<WeaponDiagnosticFrameV1> pending;
    try{pending=std::make_unique<WeaponDiagnosticFrameV1>();}catch(...){return false;}
    for(unsigned attempt=0;attempt<8;++attempt){
        const auto before=slot.guard.load(std::memory_order_seq_cst);if(!before||(before&1))continue;
        atomicRead(slot.words,*pending);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if(before!=slot.guard.load(std::memory_order_seq_cst))continue;
        if(!valid()||pending->worldFrame!=sequence||pending->publisherPid!=pid_||pending->publisherIncarnation!=incarnation_||!validateWeaponDiagnosticsV1(*pending))return false;
        out=*pending;return true;
    }
    return false;
}

bool weaponDiagnosticWorldIdentityV1(const std::string& world,std::uint64_t& pid,std::uint32_t& incarnation,std::uint64_t& frame){
    if(!validWorld(world))return false;
    platform::SharedMemory probe;if(!probe.open(std::string(kWorldPrefix)+world,sizeof(WorldHeader)))return false;
    const auto* header=static_cast<const WorldHeader*>(probe.data());
    for(unsigned attempt=0;attempt<8;++attempt){
        const auto before=header->incarnation.load(std::memory_order_acquire);
        if(!before||header->magic!=kMagic||header->layoutVersion!=kLayoutVersion||header->headerSize<sizeof(WorldHeader)||!header->alive.load(std::memory_order_acquire))return false;
        const auto sourcePid=header->publisherPid,sequence=header->publishSequence.load(std::memory_order_acquire);
        if(header->incarnation.load(std::memory_order_acquire)!=before)continue;
        if(!sourcePid||!sequence||!platform::processAlive(sourcePid))return false;
        pid=sourcePid;incarnation=before;frame=sequence;return true;
    }
    return false;
}
bool WeaponDiagnosticsV1::readLatest(WeaponDiagnosticFrameV1& out) const noexcept {
    if(!valid())return false;
    return read(shared_->latest.load(std::memory_order_acquire),out);
}
bool WeaponDiagnosticsV1::requestSelection(std::uint32_t entity,std::uint64_t generation,std::uint64_t frame,std::uint64_t* request) noexcept {
    if(owner_||!valid()||!frame||((entity==0)!=(generation==0)))return false;
    for(unsigned attempt=0;attempt<8;++attempt){
        auto guard=shared_->selectionGuard.load(std::memory_order_seq_cst);
        if((guard&1)||guard>std::numeric_limits<std::uint64_t>::max()-2)continue;
        if(!shared_->selectionGuard.compare_exchange_weak(guard,guard+1,std::memory_order_seq_cst))continue;
        WeaponDiagnosticSelectionV1 next;atomicRead(shared_->selection,next);
        if(next.request==std::numeric_limits<std::uint64_t>::max()||!valid()){shared_->selectionGuard.store(guard+2,std::memory_order_seq_cst);return false;}
        next={next.request+1,frame,generation,entity,0};atomicWrite(shared_->selection,next);shared_->selectionGuard.store(guard+2,std::memory_order_seq_cst);
        if(request)*request=next.request;
        return true;
    }
    return false;
}
bool WeaponDiagnosticsV1::selection(std::uint64_t frame,WeaponDiagnosticSelectionV1& out,std::uint64_t maximumAge) const noexcept {
    if(!owner_||!valid()||!frame)return false;
    for(unsigned attempt=0;attempt<8;++attempt){
        const auto before=shared_->selectionGuard.load(std::memory_order_seq_cst);if(before&1)continue;
        WeaponDiagnosticSelectionV1 pending;atomicRead(shared_->selection,pending);std::atomic_thread_fence(std::memory_order_seq_cst);
        if(shared_->selectionGuard.load(std::memory_order_seq_cst)!=before)continue;
        if(!valid()||!pending.request||!pending.worldFrame||pending.reserved||((pending.entity==0)!=(pending.generation==0))||pending.worldFrame>frame||frame-pending.worldFrame>maximumAge)return false;
        out=pending;return true;
    }
    return false;
}

bool WeaponDiagnosticsRecorderV1::open(const std::string& path,std::uint64_t pid,std::uint32_t incarnation){
    close();if(!pid||!incarnation)return false;stream_.open(path,std::ios::binary|std::ios::trunc);if(!stream_)return false;
    LogHeader header;header.pid=pid;header.incarnation=incarnation;stream_.write(reinterpret_cast<const char*>(&header),sizeof(header));
    if(!stream_){close();return false;}pid_=pid;incarnation_=incarnation;return true;
}
bool WeaponDiagnosticsRecorderV1::append(const WeaponDiagnosticFrameV1& frame){
    if(!stream_||frame.publisherPid!=pid_||frame.publisherIncarnation!=incarnation_||frame.worldFrame<=lastFrame_||frame.simTimeS<lastTimeS_||!validateWeaponDiagnosticsV1(frame))return false;
    const auto packet=pack(frame);LogRecord record;record.bytes=static_cast<std::uint32_t>(packet.size());record.crc=crc32(packet.data(),packet.size());
    stream_.write(reinterpret_cast<const char*>(&record),sizeof(record));stream_.write(reinterpret_cast<const char*>(packet.data()),static_cast<std::streamsize>(packet.size()));
    if(!stream_)return false;
    lastFrame_=frame.worldFrame;lastTimeS_=frame.simTimeS;return true;
}
bool WeaponDiagnosticsRecorderV1::close(){
    bool ok=true;if(stream_.is_open()){stream_.flush();ok=static_cast<bool>(stream_);stream_.close();ok=ok&&static_cast<bool>(stream_);}
    stream_.clear();pid_=lastFrame_=0;incarnation_=0;lastTimeS_=0;return ok;
}
bool WeaponDiagnosticsRecorderV1::finishCaptureV1(WeaponDiagnosticCaptureCompletionV1& completion){
    const auto pid=pid_;const auto incarnation=incarnation_;
    const bool opened=stream_.is_open();const auto position=opened?stream_.tellp():std::streampos(-1);
    const bool counted=position>=std::streampos(0);
    const auto bytes=counted?static_cast<std::uint64_t>(static_cast<std::streamoff>(position)):0;
    const bool closed=close();
    if(!opened||!counted||!pid||!incarnation||!closed)return false;
    completion.diagnosticsClosedSuccessfully=true;completion.diagnosticsBytes=bytes;
    completion.publisherPid=pid;completion.publisherIncarnation=incarnation;return true;
}
bool WeaponDiagnosticsReplayV1::open(const std::string& path){
    close();stream_.open(path,std::ios::binary);if(!stream_)return false;
    LogHeader header;stream_.read(reinterpret_cast<char*>(&header),sizeof(header));
    if(!stream_||header.magic!=logMagic||header.version!=1||header.frameBytes!=sizeof(WeaponDiagnosticFrameV1)||header.byteOrder!=endian||header.reserved||!header.pid||!header.incarnation){close();return false;}
    pid_=header.pid;incarnation_=header.incarnation;return true;
}
void WeaponDiagnosticsReplayV1::close(){stream_.close();stream_.clear();pid_=lastFrame_=0;incarnation_=0;lastTimeS_=0;invalid_=false;lookahead_.reset();requestedTimeS_=-1;}
bool WeaponDiagnosticsReplayV1::rewind(){
    if(!stream_.is_open())return false;
    stream_.clear();stream_.seekg(sizeof(LogHeader));lastFrame_=0;lastTimeS_=0;invalid_=false;lookahead_.reset();requestedTimeS_=-1;return static_cast<bool>(stream_);
}
WeaponDiagnosticsReplayResultV1 WeaponDiagnosticsReplayV1::next(WeaponDiagnosticFrameV1& out){
    if(!stream_.is_open()||invalid_)return WeaponDiagnosticsReplayResultV1::Invalid;
    LogRecord record;stream_.read(reinterpret_cast<char*>(&record),sizeof(record));
    if(stream_.gcount()==0&&stream_.eof())return WeaponDiagnosticsReplayResultV1::End;
    if(!stream_||!record.bytes||record.bytes>sizeof(WeaponDiagnosticFrameV1)){invalid_=true;return WeaponDiagnosticsReplayResultV1::Invalid;}
    Packet packet(record.bytes);stream_.read(reinterpret_cast<char*>(packet.data()),static_cast<std::streamsize>(packet.size()));
    auto pending=std::make_unique<WeaponDiagnosticFrameV1>();
    if(!stream_||record.crc!=crc32(packet.data(),packet.size())||!unpack(packet,*pending)||pending->publisherPid!=pid_||pending->publisherIncarnation!=incarnation_||
       pending->worldFrame<=lastFrame_||pending->simTimeS<lastTimeS_||!validateWeaponDiagnosticsV1(*pending)){invalid_=true;return WeaponDiagnosticsReplayResultV1::Invalid;}
    lastFrame_=pending->worldFrame;lastTimeS_=pending->simTimeS;out=*pending;return WeaponDiagnosticsReplayResultV1::Frame;
}
bool WeaponDiagnosticsReplayV1::atTime(double time,WeaponDiagnosticFrameV1& out){
    if(!nonnegative(time)||!stream_.is_open())return false;
    if(time<requestedTimeS_&&!rewind())return false;
    requestedTimeS_=time;
    for(;;){
        if(lookahead_){
            if(lookahead_->simTimeS==time){out=*lookahead_;return true;}
            if(lookahead_->simTimeS>time)return false;
        }
        auto pending=std::make_unique<WeaponDiagnosticFrameV1>();
        if(next(*pending)!=WeaponDiagnosticsReplayResultV1::Frame)return false;
        lookahead_=std::move(pending);
    }
}
}
