#pragma once
// Referee/Viewer diagnostic sidecar only. Never an Agent or actor report.
// Additive contract: WorldLayout v9 and fsrec v8 are unchanged.
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace fsim::ipc {
constexpr std::uint32_t kWeaponDiagnosticsVersionV1=1;
constexpr std::size_t kWeaponDiagnosticRosterV1=128;
constexpr std::size_t kWeaponDiagnosticThermalSideV1=128;
constexpr std::size_t kWeaponDiagnosticThermalPixelsV1=128*128;
constexpr std::size_t kWeaponDiagnosticRfRowsV1=2048,kWeaponDiagnosticRfColumnsV1=256;
constexpr std::size_t kWeaponDiagnosticRfCellsV1=131072;
constexpr std::size_t kWeaponDiagnosticPeaksV1=64,kWeaponDiagnosticFinsV1=32;
constexpr std::size_t kWeaponDiagnosticFragmentsV1=128,kWeaponDiagnosticDamageV1=64;
constexpr std::size_t kWeaponDiagnosticEventsV1=128;

enum class WeaponDiagnosticStatusV1 : std::uint32_t {
    Unavailable=0,Available=1,NotApplicable=2,Pending=3,Expired=4,
    Removed=5,CapacityExceeded=6,Rejected=7
};
struct WeaponDiagnosticStampV1 {
    WeaponDiagnosticStatusV1 status=WeaponDiagnosticStatusV1::Unavailable;
    std::uint32_t flags=0;
    std::uint64_t sourceFrame=0,sourceId=0;
    double sampleTimeS=0,processingTimeS=0,deliveredTimeS=0,validUntilS=0;
    char cause[96]{}; ///< Native processing/refusal cause, never a UI-invented outcome.
};
struct WeaponDiagnosticSelectionV1 {
    std::uint64_t request=0,worldFrame=0,generation=0;
    std::uint32_t entity=0,reserved=0; ///< entity=generation=0 clears selection.
};
struct WeaponDiagnosticRosterEntryV1 {
    std::uint32_t entity=0,flags=0;
    std::uint64_t generation=0;
    char name[64]{},store[32]{};
};
enum WeaponDiagnosticSensorFlagsV1 : std::uint32_t {
    WeaponDiagnosticTrackingV1=1,WeaponDiagnosticCentroidV1=2,
    WeaponDiagnosticRangeAxisV1=4,WeaponDiagnosticDopplerAxisV1=8,
    WeaponDiagnosticIlluminatedV1=16,WeaponDiagnosticSegmentationV1=32,
    WeaponDiagnosticRfThresholdsV1=64,WeaponDiagnosticRfLocalNoiseV1=128,
    WeaponDiagnosticRfStatsV1=256,WeaponDiagnosticBistaticDelayAxisV1=512,
    WeaponDiagnosticNativeDisplayScaleV1=1024,WeaponDiagnosticExpectedPixelsV1=2048,
    WeaponDiagnosticThermalExtentV1=4096,WeaponDiagnosticQuadrantsV1=8192,
    WeaponDiagnosticRfTestedCellsV1=16384
};
struct WeaponDiagnosticThermalV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t width=0,height=0,flags=0,selectedBlob=0;
    std::uint32_t adcBits=0,reserved=0;
    double horizontalHalfAngleRad=0,verticalHalfAngleRad=0;
    double centroidColumn=0,centroidRow=0,centroidVarianceRad2=0;
    double centroidSnrAmplitude=0,centroidSignedContrastElectrons=0;
    double centroidWidthRad=0,centroidHeightRad=0;
    double quadrantContrastElectrons[4]{},quadrantGuardBackgroundElectrons=0,quadrantVarianceElectrons2=0;
    double detectionSigma=0,readNoiseElectrons=0,darkCurrentElectronsS=0;
    double exposureS=0,fullWellElectrons=0;
    double displayMinimumElectrons=0,displayMaximumElectrons=0;
    double pointingEcef[3]{};
    // Row zero is the TOP of the native image. Values are actual detector
    // electrons/pixel/exposure (float32 diagnostic encoding), NOT irradiance.
    // Expected response and segmentation require their availability flags.
    // Segmentation is actual processor labels; zero means background.
    // Labels are not simulation target identities.
    float expectedElectrons[kWeaponDiagnosticThermalPixelsV1]{};
    float measuredElectrons[kWeaponDiagnosticThermalPixelsV1]{};
    std::uint32_t segmentation[kWeaponDiagnosticThermalPixelsV1]{};
};
struct WeaponDiagnosticRfPeakV1 {
    std::uint32_t row=0,column=0,flags=0,reserved=0;
    double powerNoiseRelative=0,thresholdNoiseRelative=0,rangeM=0,closingMs=0,snrLinear=0;
};
struct WeaponDiagnosticRfV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t rows=0,columns=0,peakCount=0,flags=0;
    double firstRangeM=0,rangeBinM=0,firstClosingMs=0,closingBinMs=0;
    double frequencyHz=0,prfHz=0,bandwidthHz=0;
    double ambiguousRangeM=0,ambiguousClosingMs=0;
    std::uint64_t processedCells=0,thresholdPeakCount=0,acceptedHitCount=0;
    double clutterWallS=0,processingWallS=0;
    double pointingEcef[3]{};
    // Row zero is the LOWEST range bin. Passive unknown axes remain
    // unavailable by flags; no range is manufactured by this transport.
    // Actual native Signal::Snapshot units: linear power / mean thermal
    // noise per filter. These are NOT watts. Optional local-CFAR arrays
    // require explicit flags; absence must not become zero thresholds.
    float powerNoiseRelative[kWeaponDiagnosticRfCellsV1]{};
    float thresholdNoiseRelative[kWeaponDiagnosticRfCellsV1]{};
    float localNoiseRelative[kWeaponDiagnosticRfCellsV1]{};
    std::uint8_t tested[kWeaponDiagnosticRfCellsV1]{}; ///< 0 excluded/untested, 1 actually tested.
    WeaponDiagnosticRfPeakV1 peaks[kWeaponDiagnosticPeaksV1];
};
enum WeaponDiagnosticNavigationFlagsV1 : std::uint32_t {
    WeaponDiagnosticGnssFittedV1=1,WeaponDiagnosticGnssAidedV1=2,
    WeaponDiagnosticOwnReferenceV1=4,WeaponDiagnosticGnssEverSampledV1=8
};
struct WeaponDiagnosticNavigationV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t flags=0,errorStateCount=0;
    double quaternionBodyToEcef[4]{}; ///< Hamilton [w,x,y,z].
    double positionEcefM[3]{},velocityEcefMs[3]{};
    double accelerometerBiasBodyMs2[3]{},gyroBiasBodyRadS[3]{};
    double measuredGyroBodyRadS[3]{},measuredSpecificForceBodyMs2[3]{};
    double lastImuTimeS=0,lastGnssTimeS=0;
    // Full native covariance, row major, with actual state count 15 or21:
    // dp,dv,dtheta_ECEF,dba_body,dbg_body,[held accelerometer,held gyro].
    double errorCovariance[21*21]{};
    // Optional explicitly REFEREE own-reference comparison, never correction.
    double referencePositionEcefM[3]{},referenceVelocityEcefMs[3]{};
    double referenceQuaternionBodyToEcef[4]{};
};
struct WeaponDiagnosticFinV1 {
    std::uint32_t id=0,index=0,flags=0,reserved=0;
    double demandRad=0,actualRad=0,minimumRad=0,maximumRad=0,rateRadS=0;
};
enum WeaponDiagnosticControlFlagsV1 : std::uint32_t {
    WeaponDiagnosticControllerV1=1,WeaponDiagnosticTvcV1=2,WeaponDiagnosticAllocationConvergedV1=4,
    WeaponDiagnosticAccelerationDemandV1=8,WeaponDiagnosticAccelerationAchievedV1=16,
    WeaponDiagnosticMomentDemandV1=32,WeaponDiagnosticMomentAchievedV1=64,
    WeaponDiagnosticAllocatorMetricsV1=128,WeaponDiagnosticControlNavigationEpochV1=256
};
struct WeaponDiagnosticControlV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t finCount=0,flags=0,allocatorIterations=0,allocatorReason=0;
    double controllerTimeS=0; ///< Available only with WeaponDiagnosticControllerV1.
    double navigationTimeS=0; ///< Actual posterior read by that control; own availability flag.
    double demandedAccelerationBodyMs2[3]{},achievedAccelerationBodyMs2[3]{};
    double commands[3]{},demandedMomentBodyNm[3]{},achievedMomentBodyNm[3]{};
    double tvcDemandPitchRad=0,tvcDemandYawRad=0,tvcActualPitchRad=0,tvcActualYawRad=0;
    double tvcLimitRad=0,tvcRateRadS=0,allocatorResidual=0;
    WeaponDiagnosticFinV1 fins[kWeaponDiagnosticFinsV1];
};
struct WeaponDiagnosticPropulsionV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t engineCount=0,flags=0,pulseState=0,propulsionKind=0;
    double thrustN[4]{},fuelFlowKgS[4]{},fuelKg=0,nativeMotorAgeS=0;
    double secondIgnitionAgeS=0,valveDemand=0,valveActual=0;
    double inletAirKgS=0,inletTemperatureK=0,combustorTemperatureK=0;
    double netThrustN=0; ///< Actual body +X propulsion force incl. native ram/inventory reactions, NOT engine sum.
};
enum WeaponDiagnosticPropulsionFlagsV1 : std::uint32_t {
    WeaponDiagnosticBurningV1=1,WeaponDiagnosticSecondCommandV1=2,
    WeaponDiagnosticSecondIgnitedV1=4,WeaponDiagnosticInletLitV1=8,
    WeaponDiagnosticFuelV1=16,WeaponDiagnosticValveV1=32,
    WeaponDiagnosticInletV1=64,WeaponDiagnosticFuelFlowV1=128,
    WeaponDiagnosticMotorClockV1=256,WeaponDiagnosticPulsePolicyV1=512,
    WeaponDiagnosticNetAxialForceV1=1024
};
enum WeaponDiagnosticGuidanceFlagsV1 : std::uint32_t {
    WeaponDiagnosticSeekerActiveV1=1,WeaponDiagnosticGuidanceTrackingV1=2,
    WeaponDiagnosticSupportedV1=4,WeaponDiagnosticLosV1=8,
    WeaponDiagnosticLosRateV1=16,WeaponDiagnosticMeasuredRangeV1=32,
    WeaponDiagnosticMeasuredRangeRateV1=64,WeaponDiagnosticMidcourseV1=128,
    WeaponDiagnosticTimeToGoV1=256,WeaponDiagnosticTimeToActiveV1=512,
    WeaponDiagnosticAngularVarianceV1=1024
};
struct WeaponDiagnosticGuidanceV1 {
    WeaponDiagnosticStampV1 stamp; ///< Current native guidance/phase decision.
    WeaponDiagnosticStampV1 measurementStamp; ///< Original seeker capture/processing/delivery.
    std::uint32_t phase=0,flags=0,localTrackId=0,reserved=0;
    double weaponAgeS=0,trackTimeS=0;
    double losDirectionEcef[3]{},losDirectionRatePerS[3]{};
    double rangeM=0,rangeRateMs=0,angularVarianceRad2=0,timeToGoS=0,timeToActiveS=0;
};
enum WeaponDiagnosticLinkFlagsV1 : std::uint32_t {
    WeaponDiagnosticInputKnownV1=1,WeaponDiagnosticInputValidV1=2,
    WeaponDiagnosticChannelOpenV1=4,WeaponDiagnosticChannelStatsV1=8,
    WeaponDiagnosticRadioResourceV1=16
};
struct WeaponDiagnosticFusionSourceV1 {
    std::uint32_t kind=0,system=0,payload=0,flags=0;
    std::uint64_t track=0;
    double sampleTimeS=0,receivedTimeS=0;
};
struct WeaponDiagnosticLinkV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t direction=0,flags=0,fusionState=0,sourceCount=0;
    std::uint64_t fusedEntity=0;
    double stateTimeS=0,lastUpdateS=0,inputReceivedS=0,inputValidUntilS=0;
    double stateEcef[6]{},covarianceEcef[36]{}; ///< Accepted delivered estimate only, interleaved x,vx,y,vy,z,vz.
    double updatePeriodS=0,feedbackPeriodS=0,latencyS=0,dataRateBps=0;
    double rangeM=0,frequencyHz=0,resourceBusyUntilS=0;
    std::uint64_t sent=0,delivered=0,lost=0,masked=0,outOfRange=0;
    std::uint64_t queueRefused=0,expired=0,rejected=0,bytes=0;
    char directionSource[256]{};
    WeaponDiagnosticFusionSourceV1 sources[64];
};
enum WeaponDiagnosticEffectsFlagsV1 : std::uint32_t {
    WeaponDiagnosticArmedV1=1,WeaponDiagnosticBurstV1=2,WeaponDiagnosticConnectedRingV1=4,
    WeaponDiagnosticFuzeV1=8,WeaponDiagnosticWarheadV1=16
};
struct WeaponDiagnosticFragmentV1 {
    std::uint64_t id=0;
    double sampleTimeS=0,positionEcefM[3]{},velocityEcefMs[3]{},massKg=0,areaM2=0;
    std::uint32_t flags=0,material=0;
};
struct WeaponDiagnosticDamageV1 {
    std::uint32_t victim=0,component=0,kind=0,flags=0;
    std::uint64_t victimGeneration=0;
    double sampleTimeS=0,penetratedEnergyJ=0,health=0,removedMassKg=0;
    std::uint32_t hitCount=0,reserved=0;
};
enum WeaponDiagnosticDamageFlagsV1 : std::uint32_t {
    WeaponDiagnosticImpactCountV1=1,WeaponDiagnosticImpactEnergyV1=2,
    WeaponDiagnosticComponentHealthV1=4,WeaponDiagnosticRemovedMassV1=8
};
struct WeaponDiagnosticEffectsV1 {
    WeaponDiagnosticStampV1 stamp;
    std::uint32_t flags=0,fuzeAlgorithm=0,warheadTopology=0,fragmentCount=0;
    std::uint32_t totalFragmentCount=0,damageCount=0,totalDamageCount=0,reserved=0;
    double armingTimeS=0,burstTimeS=0,fuzeRadiusM=0,burstPositionEcefM[3]{};
    double explosiveKg=0,caseMassKg=0;
    WeaponDiagnosticFragmentV1 fragments[kWeaponDiagnosticFragmentsV1];
    WeaponDiagnosticDamageV1 damage[kWeaponDiagnosticDamageV1];
};
enum WeaponDiagnosticEventFlagsV1 : std::uint32_t {
    WeaponDiagnosticEventSourceGenerationV1=1,WeaponDiagnosticEventTargetGenerationV1=2,
    WeaponDiagnosticEventValueV1=4,WeaponDiagnosticEventCodeV1=8
};
struct WeaponDiagnosticEventV1 {
    std::uint64_t serial=0;
    std::uint64_t sourceGeneration=0,targetGeneration=0;
    double timeS=0,value=0;
    std::uint32_t code=0,source=0,target=0,flags=0;
    char unit[16]{},cause[96]{};
};
struct WeaponDiagnosticFrameV1 {
    std::uint32_t version=kWeaponDiagnosticsVersionV1,publisherIncarnation=0;
    std::uint64_t publisherPid=0,worldFrame=0,selectionRequest=0;
    double simTimeS=0;
    std::uint32_t entity=0,rosterCount=0,totalWeaponCount=0,eventCount=0;
    std::uint64_t generation=0;
    WeaponDiagnosticStatusV1 selectionStatus=WeaponDiagnosticStatusV1::Unavailable;
    std::uint32_t reserved=0;
    char store[32]{},name[64]{};
    WeaponDiagnosticRosterEntryV1 roster[kWeaponDiagnosticRosterV1];
    WeaponDiagnosticThermalV1 thermal;
    WeaponDiagnosticRfV1 rf;
    WeaponDiagnosticNavigationV1 navigation;
    WeaponDiagnosticGuidanceV1 guidance;
    WeaponDiagnosticLinkV1 link;
    WeaponDiagnosticControlV1 control;
    WeaponDiagnosticPropulsionV1 propulsion;
    WeaponDiagnosticEffectsV1 effects;
    WeaponDiagnosticEventV1 events[kWeaponDiagnosticEventsV1];
};
static_assert(std::is_trivially_copyable_v<WeaponDiagnosticFrameV1>);
static_assert(sizeof(WeaponDiagnosticFrameV1)%4==0);

template<std::size_t N> bool weaponDiagnosticTextV1(char (&out)[N],std::string_view text) noexcept {
    if(text.size()>=N)return false;
    for(std::size_t i=0;i<text.size();++i)if(text[i]=='\0')return false;
    for(std::size_t i=0;i<N;++i)out[i]=i<text.size()?text[i]:'\0';
    return true;
}
/// Pure schema validation; no simulation, resampling, image synthesis or repair.
bool validateWeaponDiagnosticsV1(const WeaponDiagnosticFrameV1&,const char** reason=nullptr) noexcept;
bool weaponDiagnosticFreshV1(const WeaponDiagnosticStampV1&,double simTimeS) noexcept;
const char* weaponDiagnosticStatusTextV1(WeaponDiagnosticStatusV1) noexcept;
}
