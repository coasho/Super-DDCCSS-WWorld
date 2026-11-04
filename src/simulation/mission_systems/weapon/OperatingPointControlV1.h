#pragma once
#include "Autopilot.h"
#include <cstdint>

namespace fsim::ms::weapon {
/// A new private snapshot; retained AutopilotInput/provider layouts are unchanged.
struct OperatingFailureV1 {
    bool stuck=false,zero=false,hardover=false;
};
struct OperatingFinV1 {
    double actualRad=0,minimumRad=0,maximumRad=0,rateRadS=0;
    OperatingFailureV1 failure;
    std::array<double,3> mixer{}; ///< aileron, elevator, rudder
    std::vector<double> anglesRad;
    std::vector<std::array<double,3>> momentsNm; ///< current-incidence/Mach moments about current CG
};
struct OperatingPointInputV1 {
    AutopilotInput plant;
    ControlStopsV1 stops;
    double dtS=0;
    unsigned actuatorSubsteps=1;
    std::array<double,3> channelInputRad{},channelLagRad{},channelActualRad{};
    std::array<OperatingFailureV1,3> channelFailure;
    std::vector<OperatingFinV1> fins;
};
struct OperatingPointResultV1 {
    FinCommands commands;
    std::array<double,3> requestedMomentNm{},predictedMomentNm{};
    std::vector<double> predictedFinRad;
    double residualAccelerationNorm=0;
};
/// Optional private diagnostics; no shared state, actor report or public ABI.
struct OperatingPointSearchStatsV1 {
    std::uint64_t scalarPredictions=0,differentialPredictions=0,lineSegments=0;
    std::uint64_t boundaryRetries=0,budgetExhaustions=0,completedLines=0,maxLineSegments=0;
    std::uint64_t memoHits=0,mapLines=0,fallbackLines=0,mapPieces=0,cachedChannelSteps=0,candidates=0;
    std::uint64_t sequentialIterations=0,qpFaces=0,backtracks=0,polls=0,seeds=0;
};
struct OperatingPointCandidateV1 {
    std::array<double,3> command{};
    double cost=0,effort=0;
};
/// Private verification controls; production selects bounded sequential allocation.
struct OperatingPointSearchOptionsV1 {
    bool referenceTracer=false,exhaustiveReference=false,memoize=true,cacheNonVaried=true;
    std::size_t mapCapacity=0; ///< zero uses the retained tracer's finite event budget
    std::vector<OperatingPointCandidateV1>* candidateAudit=nullptr;
};
/// Fixed-command private native-predictor oracle; no forces are applied.
OperatingPointResultV1 operatingPointPredictionV1(const OperatingPointInputV1&,
                                               const std::array<double,3>& command) noexcept;
/// Wind loads D,Y,L (N) and body moments (Nm) about ARP, shifted to current CG.
std::array<double,3> operatingPointBodyMomentV1(const std::array<double,6>& loads,
    double alphaRad,double betaRad,const std::array<double,3>& cgFromArpM) noexcept;
/// Feasible one-sided physical differential, including current fin stops.
std::array<double,3> operatingPointDirectionalMomentV1(const OperatingPointInputV1&,
                                                     const std::array<double,3>& motion) noexcept;
/// Full finite native-actuator predictor and bounded sequential allocation.
/// Numerical search does not imply a global nonlinear optimum or fitted hardware accuracy.
OperatingPointResultV1 operatingPointCommandsV1(const OperatingPointInputV1&,const FinControlConfigV1&) noexcept;
OperatingPointResultV1 operatingPointCommandsV1(const OperatingPointInputV1&,const FinControlConfigV1&,
                                               OperatingPointSearchStatsV1&) noexcept;
OperatingPointResultV1 operatingPointCommandsV1(const OperatingPointInputV1&,const FinControlConfigV1&,
    OperatingPointSearchStatsV1&,const OperatingPointSearchOptionsV1&) noexcept;
/// Distinct private input-stage contract. finCommands supplies an authored FCS
/// INPUT, not an instantaneous surface angle. Advance that nominal input
/// through the captured native virtual-channel lag/rate/stops/failures first;
/// allocate its finite endpoint moment with the existing physical-fin model.
/// The retained operatingPointCommandsV1 instantaneous-moment contract above
/// is unchanged. No additional horizon, control state or ideal force is added.
OperatingPointResultV1 operatingPointInputCommandsV1(const OperatingPointInputV1&,const FinControlConfigV1&) noexcept;
}
