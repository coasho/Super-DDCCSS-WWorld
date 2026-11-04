// Referee-only encoding of actual detector arrays. Never generates pixels,
// invokes a detector, renews a measurement epoch or exposes actor knowledge.
#include "World.h"
#include "simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h"
#include <algorithm>
#include <cmath>
#include <new>
#include <type_traits>

namespace fsim::session {
namespace {
template<std::size_t N>void label(char (&out)[N],const char* value){ipc::weaponDiagnosticTextV1(out,value);}
void rawStamp(ipc::WeaponDiagnosticStampV1& stamp,const ipc::WeaponDiagnosticFrameV1& frame,double epoch,const char* cause) {
    stamp={};stamp.status=ipc::WeaponDiagnosticStatusV1::Available;stamp.sourceFrame=frame.worldFrame;stamp.sourceId=frame.entity;
    stamp.sampleTimeS=epoch;stamp.processingTimeS=epoch;stamp.deliveredTimeS=frame.simTimeS;stamp.validUntilS=frame.simTimeS;
    label(stamp.cause,cause);
}
void vector(double (&out)[3],ms::weapon::V3 value){out[0]=value.x;out[1]=value.y;out[2]=value.z;}
template<class T>void resetInPlace(T& value) {
    static_assert(std::is_trivially_destructible_v<T>);
    value.~T();::new(static_cast<void*>(&value))T{};
}
}
const ms::components::ThermalReticleResultV1* World::weaponReticleObservationV1(std::uint32_t weapon) const noexcept {
    const auto* e=entry(weapon);
    if(!e||e->info.role!=ms::Role::Weapon||!e->seekerProvider)return nullptr;
    const auto* receiver=dynamic_cast<const ms::components::ThermalReticleSeekerExecutionV1*>(e->seekerProvider.get());
    return receiver?receiver->thermalReticleV1():nullptr;
}
void World::fillWeaponSensorDiagnosticsV1(const Entry& e,ipc::WeaponDiagnosticFrameV1& out) {
    using namespace ipc;
    resetInPlace(out.thermal);resetInPlace(out.rf);
    out.thermal.stamp.status=out.rf.stamp.status=WeaponDiagnosticStatusV1::Unavailable;
    if(!e.seekerProvider){label(out.thermal.stamp.cause,"No installed seeker");label(out.rf.stamp.cause,"No installed seeker");return;}
    const auto* thermal=dynamic_cast<const ms::components::ThermalSeekerExecutionV1*>(e.seekerProvider.get());
    const auto* image=thermal?thermal->thermalImageV1():nullptr;
    const auto* thermalPolicy=thermal?thermal->thermalPolicyV1():nullptr;
    const auto* dual=dynamic_cast<const ms::components::ThermalDualBandSeekerExecutionV1*>(e.seekerProvider.get());
    const auto* dualPolicy=dual?dual->thermalDualBandPolicyV1():nullptr;
    unsigned band=0;
    if(dualPolicy) {
        if(weaponThermalBandV1_)weaponThermalBandV1_->selection(out.worldFrame,e.info.id,e.info.generation,band);
        const auto* images=dual->thermalDualBandV1();
        thermalPolicy=&dualPolicy->channels[band];image=images&&images->valid?&images->channels[band]:nullptr;
    }
    if(thermalPolicy) {
        auto& d=out.thermal;
        if(!image||!image->valid){d.stamp.status=WeaponDiagnosticStatusV1::Pending;label(d.stamp.cause,"No accepted native thermal exposure yet");}
        else {
            const auto count=static_cast<std::size_t>(image->width)*image->height;
            if(image->width>kWeaponDiagnosticThermalSideV1||image->height>kWeaponDiagnosticThermalSideV1||
                count>kWeaponDiagnosticThermalPixelsV1||image->measuredElectrons.size()!=count) {
                d.stamp.status=WeaponDiagnosticStatusV1::CapacityExceeded;label(d.stamp.cause,"Native image shape exceeds diagnostic contract");
            } else {
                rawStamp(d.stamp,out,image->timeS,"Actual raw detector capture; referee publication, not actor delivery");
                if(dualPolicy)label(d.stamp.cause,band==0?"Actual raw detector capture, channel 1; display selection only":"Actual raw detector capture, channel 2; display selection only");
                const auto& finite=e.seekerProvider->config().engineering;
                d.stamp.validUntilS=image->timeS+finite.processingDelayS+finite.memoryS+std::max(finite.samplePeriodS,options_.dt*options_.frameSkip);
                if(out.simTimeS>d.stamp.validUntilS)d.stamp.status=WeaponDiagnosticStatusV1::Expired;
                d.width=image->width;d.height=image->height;const auto& c=image->config;
                d.horizontalHalfAngleRad=.5*c.width*c.pixelPitchRad;d.verticalHalfAngleRad=.5*c.height*c.pixelPitchRad;
                d.adcBits=c.adcBits;d.exposureS=c.integrationTimeS;d.fullWellElectrons=c.fullWellElectrons;
                d.detectionSigma=c.detectionSigma;d.readNoiseElectrons=c.readNoiseElectrons;d.darkCurrentElectronsS=c.darkCurrentElectronsS;
                d.displayMinimumElectrons=0;d.displayMaximumElectrons=c.fullWellElectrons;d.flags|=WeaponDiagnosticNativeDisplayScaleV1;
                vector(d.pointingEcef,image->pointing);
                if(e.seekerResultV2.tracking)d.flags|=WeaponDiagnosticTrackingV1;
                const bool expected=image->expectedElectrons.size()==count;
                const bool segmented=image->segmentationAvailable&&image->segmentation.size()==count;
                if(expected)d.flags|=WeaponDiagnosticExpectedPixelsV1;
                if(segmented)d.flags|=WeaponDiagnosticSegmentationV1;
                for(unsigned row=0;row<image->height;++row)for(unsigned column=0;column<image->width;++column) {
                    const auto to=static_cast<std::size_t>(row)*image->width+column;
                    const auto from=static_cast<std::size_t>(image->height-1-row)*image->width+column;
                    d.measuredElectrons[to]=static_cast<float>(image->measuredElectrons[from]);
                    if(expected)d.expectedElectrons[to]=static_cast<float>(image->expectedElectrons[from]);
                    if(segmented)d.segmentation[to]=image->segmentation[from];
                }
                if(!image->features.empty()) {
                    const auto& f=image->features.front();
                    if(f.centroidColumn>=0&&f.centroidColumn<image->width&&f.centroidRow>=0&&f.centroidRow<image->height) {
                        // The raw detector sorts by current detection strength.
                        // It does not expose the family's delayed association;
                        // selectedBlob therefore remains explicitly unavailable.
                        d.flags|=WeaponDiagnosticCentroidV1;d.selectedBlob=0;
                        d.centroidColumn=f.centroidColumn;d.centroidRow=image->height-1-f.centroidRow;
                        d.centroidVarianceRad2=f.angularVarianceRad2;d.centroidSnrAmplitude=f.signalToNoise;d.centroidSignedContrastElectrons=f.signedContrastElectrons;
                        if(image->imageProcessingAvailable){d.flags|=WeaponDiagnosticThermalExtentV1;d.centroidWidthRad=f.extent.widthRad;d.centroidHeightRad=f.extent.heightRad;}
                    }
                }
                if(!image->imageProcessingAvailable) {
                    d.flags|=WeaponDiagnosticQuadrantsV1;
                    std::copy(image->quadrantElectrons.begin(),image->quadrantElectrons.end(),d.quadrantContrastElectrons);
                    d.quadrantGuardBackgroundElectrons=image->quadrantBackgroundElectrons;d.quadrantVarianceElectrons2=image->quadrantVarianceElectrons2;
                }
            }
        }
        out.rf.stamp.status=WeaponDiagnosticStatusV1::NotApplicable;label(out.rf.stamp.cause,"Selected thermal receiver");
    }
    const auto* radio=dynamic_cast<const ms::components::RadioSeekerExecutionV1*>(e.seekerProvider.get());
    const auto* result=radio?radio->radioSpectrumV1():nullptr;
    const auto* radioPolicy=radio?radio->radioPolicyV1():nullptr;
    if(!radioPolicy)return;
    out.thermal.stamp.status=WeaponDiagnosticStatusV1::NotApplicable;label(out.thermal.stamp.cause,"Selected RF receiver");
    auto& d=out.rf;
    if(!result||!result->valid||!result->spectrum.kept){d.stamp.status=WeaponDiagnosticStatusV1::Pending;label(d.stamp.cause,"No accepted native RF dwell yet");return;}
    const auto& s=result->spectrum;const auto rows=static_cast<std::size_t>(s.rows),columns=static_cast<std::size_t>(s.filters),count=rows*columns;
    if(s.rows<=0||s.filters<=0||rows>kWeaponDiagnosticRfRowsV1||columns>kWeaponDiagnosticRfColumnsV1||
        count>kWeaponDiagnosticRfCellsV1||s.power.size()!=count) {
        d.stamp.status=WeaponDiagnosticStatusV1::CapacityExceeded;label(d.stamp.cause,"Native spectrum exceeds diagnostic contract");return;
    }
    rawStamp(d.stamp,out,result->timeS,"Actual native waveform/cells; referee publication, not actor delivery");
    const auto& finite=e.seekerProvider->config().engineering;
    d.stamp.validUntilS=result->timeS+finite.processingDelayS+finite.memoryS+std::max(finite.samplePeriodS,options_.dt*options_.frameSkip);
    if(out.simTimeS>d.stamp.validUntilS)d.stamp.status=WeaponDiagnosticStatusV1::Expired;
    d.rows=static_cast<std::uint32_t>(rows);d.columns=static_cast<std::uint32_t>(columns);
    d.frequencyHz=299792458./result->waveform.wavelengthM;d.prfHz=s.prfHz;d.bandwidthHz=result->config.bandwidthHz;
    d.firstRangeM=s.firstGate*s.gateM;d.rangeBinM=s.gateM;d.ambiguousRangeM=s.ambiguousRangeM;d.ambiguousClosingMs=s.ambiguousRateMs;
    const auto shift=(columns+1)/2;
    d.firstClosingMs=(static_cast<double>(shift)/static_cast<double>(columns)-1)*s.ambiguousRateMs;
    d.closingBinMs=s.ambiguousRateMs/static_cast<double>(columns);
    d.flags|=WeaponDiagnosticDopplerAxisV1|WeaponDiagnosticRfStatsV1;
    d.flags|=result->semiActive?WeaponDiagnosticBistaticDelayAxisV1:WeaponDiagnosticRangeAxisV1;
    if(result->semiActive&&e.seekerResultV2.supported)d.flags|=WeaponDiagnosticIlluminatedV1;
    if(e.seekerResultV2.tracking)d.flags|=WeaponDiagnosticTrackingV1;
    d.processedCells=result->stats.cells;d.thresholdPeakCount=result->stats.peaks;d.acceptedHitCount=result->stats.hits;
    d.clutterWallS=result->stats.clutterS;d.processingWallS=result->stats.processS;vector(d.pointingEcef,result->pointing);
    const auto& cells=result->cells;
    const bool planes=cells.kept&&cells.tested.size()==count&&cells.localNoiseRelative.size()==count&&cells.decisionThresholdRelative.size()==count;
    if(planes)d.flags|=WeaponDiagnosticRfTestedCellsV1|WeaponDiagnosticRfLocalNoiseV1|WeaponDiagnosticRfThresholdsV1;
    for(std::size_t row=0;row<rows;++row)for(std::size_t column=0;column<columns;++column) {
        const auto to=row*columns+column,from=row*columns+(column+shift)%columns;
        d.powerNoiseRelative[to]=s.power[from];
        if(planes){d.tested[to]=cells.tested[from];d.localNoiseRelative[to]=cells.localNoiseRelative[from];d.thresholdNoiseRelative[to]=cells.decisionThresholdRelative[from];}
    }
    for(const auto& peak:s.peaks) {
        if(d.peakCount>=kWeaponDiagnosticPeaksV1)break;
        if(peak.first<0||peak.second<0||peak.first>=s.rows||peak.second>=s.filters)continue;
        auto& p=d.peaks[d.peakCount++];p.row=static_cast<std::uint32_t>(peak.first);
        p.column=static_cast<std::uint32_t>((static_cast<std::size_t>(peak.second)+columns-shift)%columns);
        const auto index=static_cast<std::size_t>(p.row)*columns+p.column;
        p.powerNoiseRelative=static_cast<double>(d.powerNoiseRelative[index]);p.thresholdNoiseRelative=planes?static_cast<double>(d.thresholdNoiseRelative[index]):0;
        p.rangeM=d.firstRangeM+p.row*d.rangeBinM;p.closingMs=d.firstClosingMs+p.column*d.closingBinMs;
        if(planes&&d.localNoiseRelative[index]>0.f)p.snrLinear=std::max(0.,static_cast<double>(d.powerNoiseRelative[index])/static_cast<double>(d.localNoiseRelative[index])-1)*
            ms::radar::signal::Window::noiseGain()/(ms::radar::signal::Window::a[0]*ms::radar::signal::Window::a[0]);
    }
}
} // namespace fsim::session
