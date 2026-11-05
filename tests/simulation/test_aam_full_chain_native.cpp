// One actual release lifetime per maintained AAM. Observations never replace
// providers, feed target truth to guidance, or force a terminal outcome.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"
#include "simulation/mission_systems/weapon/ControlObservationV1.h"
#include "simulation/mission_systems/weapon/OperatingPointControlV1.h"
#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h"
#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "contracts/equipment/EquipmentModels.h"
#include "ComponentReferenceRuntimeV1.h"
#include <picosha2.h>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace fsim;
constexpr double pi=3.14159265358979323846,deg=pi/180.;
constexpr std::array<const char*,9> slots{{"seeker","guidance","autopilot","allocator","actuator","aerodynamics","propulsion","fuze","warhead"}};
void allowedCapturePath(const std::filesystem::path& path) {
    for(const auto& part:path) {
        auto name=part.wstring();
        std::transform(name.begin(),name.end(),name.begin(),[](wchar_t c){return static_cast<wchar_t>(std::towlower(c));});
        REQUIRE(name!=L".codex");
    }
}
std::string captureHash(const std::filesystem::path& path) {
    allowedCapturePath(path);REQUIRE(std::filesystem::file_size(path)<=512ULL*1024*1024);
    std::ifstream file(path,std::ios::binary);REQUIRE(file.good());
    picosha2::hash256_one_by_one hash;char buffer[65536];
    while(file.read(buffer,sizeof(buffer))||file.gcount()>0)hash.process(buffer,buffer+file.gcount());
    REQUIRE(file.eof());hash.finish();return picosha2::get_hash_hex_string(hash);
}
fsim::core::Json captureExecutable() {
    std::filesystem::path executable;
#if defined(_WIN32)
    std::array<wchar_t,32768> buffer{};
    const auto count=GetModuleFileNameW(nullptr,buffer.data(),static_cast<DWORD>(buffer.size()));
    REQUIRE(count>0);REQUIRE(count<buffer.size());
    executable=std::filesystem::path(std::wstring(buffer.data(),count));
#else
    executable=std::filesystem::canonical("/proc/self/exe");
#endif
    fsim::core::Json value;value.set("path",executable.generic_string()).set("sha256",captureHash(executable));return value;
}
ms::WeaponRecord record(const session::World& w,std::uint32_t id) {
    for(const auto& row:w.weapons())if(row.id==id)return row;
    FAIL("Actual released weapon history missing");return {};
}
std::uint16_t payload(const session::World& w,std::uint32_t id,ms::Family family) {
    REQUIRE(w.missionSystems(id));
    for(const auto& p:w.missionSystems(id)->payloads())if(p.family==family)return p.index;
    FAIL("Required native platform payload missing");return 0;
}
void operate(session::World& w,std::uint32_t id,std::uint16_t p) {
    const auto r=w.submit(id,ms::StateCommand{p,ms::PayloadState::Operate});
    REQUIRE((r.accepted()||r.reason==ms::Reason::AlreadyInState));
    for(unsigned i=0;i<1350&&w.missionSystems(id)->payloads()[p].state!=ms::PayloadState::Operate;++i)w.step();
    REQUIRE(w.missionSystems(id)->payloads()[p].state==ms::PayloadState::Operate);
}
void finiteWorld(const session::World& w) {
    for(const auto id:w.vehicleIds()) {
        const auto* s=w.vehicleState(id);REQUIRE(s);REQUIRE_FALSE(s->diverged);
        for(const auto v:s->positionEcef)REQUIRE(std::isfinite(v));
        for(const auto v:s->velocityBodyMs)REQUIRE(std::isfinite(v));
        for(const auto v:s->angularRateBodyRadS)REQUIRE(std::isfinite(v));
    }
}
struct Snapshot {
    ipc::WeaponDiagnosticsV1 reader;
    std::unique_ptr<ipc::WeaponDiagnosticFrameV1> frame=std::make_unique<ipc::WeaponDiagnosticFrameV1>();
    bool read(session::World& w,const std::string& name,std::uint32_t id,std::uint64_t generation) {
        w.publishNow();std::uint64_t pid=0,epoch=0;std::uint32_t incarnation=0;
        if(!ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,epoch))return false;
        if(!reader.valid()&&!reader.open(name,pid,incarnation))return false;
        if(!reader.requestSelection(id,generation,epoch))return false;
        w.publishNow();
        if(!ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,epoch)||!reader.read(epoch,*frame))return false;
        REQUIRE(frame->entity==id);REQUIRE(frame->generation==generation);
        REQUIRE(frame->worldFrame==epoch);
        return frame->store[0]!='\0'; // Retention expiry remains unavailable.
    }
};
// Observational POST-step property handles: bound once for this release.
// Empty fields mean unavailable/nonfinite, never a synthetic native zero.
struct NativeControlObservations {
    std::vector<std::pair<std::string,PropertyHandle>> properties;
    explicit NativeControlObservations(sim::FlightModel& model) {
        const auto bind=[&](const std::string& name) {properties.emplace_back(name,model.property(name));};
        for(const auto* name:{"aero/qbar-psf","metrics/Sw-sqft","metrics/cbarw-ft","metrics/bw-ft",
            "fsim/missile-control/version","fsim/missile-control/reference-version","fsim/missile-control/aero-reference-x-m",
            "fsim/missile-control/lag-coefficient-s-1","fsim/missile-control/normalization-version",
            "fsim/missile-control/actuator-version","fsim/missile-control/channel-rate-rad_s",
            "fsim/missile-control/fin-count","fsim/integration-v1/substeps","fsim/control-operating-point-v1/version",
            "accelerations/pdot-rad_sec2","accelerations/qdot-rad_sec2","accelerations/rdot-rad_sec2",
            "forces/fbx-total-lbs","forces/fby-total-lbs","forces/fbz-total-lbs",
            "forces/fbx-prop-lbs","forces/fby-prop-lbs","forces/fbz-prop-lbs",
            "moments/l-total-lbsft","moments/m-total-lbsft","moments/n-total-lbsft",
            "moments/l-prop-lbsft","moments/m-prop-lbsft","moments/n-prop-lbsft",
            "fsim/thrust-vector-control/version","fsim/thrust-vector-control/limit_rad","fsim/thrust-vector-control/rate_rad_s",
            "fsim/thrust-vector-control/pitch_gain","fsim/thrust-vector-control/yaw_gain","fsim/thrust-vector-control/exit_x_body_m",
            "propulsion/engine/pitch-angle-rad","propulsion/engine/yaw-angle-rad"})bind(name);
        for(const auto* axis:{"ixx","iyy","izz","ixy","ixz","iyz"})bind(std::string("inertia/")+axis+"-slugs_ft2");
        for(const auto* coefficient:{"cz-alpha","cy-beta","cm-alpha","cn-beta","cm-q","cn-r","cl-p",
            "cz-elevator","cy-rudder","cm-elevator","cn-rudder","cl-aileron"})bind(std::string("fsim/control/")+coefficient);
        for(const auto* axis:{"aileron","elevator","rudder"}) {
            for(const auto* suffix:{"-control","-lag-state","-response-rad"})bind(std::string("fcs/")+axis+suffix);
            for(const auto* suffix:{"-limit-rad","-min-rad","-max-rad"})bind(std::string("fsim/missile-control/")+axis+suffix);
        }
        const auto count=model.property("fsim/missile-control/fin-count");
        if(!count.valid())return;
        const auto fins=count.get();
        if(!std::isfinite(fins)||fins<1||fins>ipc::kWeaponDiagnosticFinsV1||fins!=std::floor(fins))return;
        for(std::size_t i=0;i<static_cast<std::size_t>(fins);++i) {
            const auto name="fin-"+std::to_string(i);
            bind("fcs/"+name+"-delta-rad");
            for(const auto* failure:{"fail_stuck","fail_zero","fail_hardover"})bind("fcs/"+name+"-actuator/malfunction/"+failure);
            for(const auto* suffix:{"-min-rad","-max-rad","-rate-rad_s"})bind("fsim/missile-control/"+name+suffix);
            const auto prefix="fsim/control-operating-point-v1/"+name+"/";
            for(const auto* axis:{"aileron","elevator","rudder"})bind(prefix+"mix-"+axis);
            bind(prefix+"knot-count");const auto& knotsHandle=properties.back().second;
            if(!knotsHandle.valid())continue;
            const auto knots=knotsHandle.get();
            if(!std::isfinite(knots)||knots<2||knots>65||knots!=std::floor(knots))continue;
            for(std::size_t j=0;j<static_cast<std::size_t>(knots);++j) {
                const auto knot=prefix+"knot-"+std::to_string(j);bind(knot+"-rad");
                for(const auto* load:{"CD","CY","CL","Cl","Cm","Cn"})bind(knot+"/"+load);
            }
        }
    }
    void header(std::ostream& out) const {for(const auto& item:properties)out<<",native/"<<item.first;}
    void row(std::ostream& out) const {
        for(const auto& item:properties) {
            out<<',';
            if(item.second.valid()) {const auto value=item.second.get();if(std::isfinite(value))out<<value;}
        }
    }
};
void observedScalar(std::ostream& out,double value,bool available) {
    out<<',';if(available&&std::isfinite(value))out<<value;
}
// Exact PRE-integration controller packets, grouped only by the existing CSV
// publication row. Every intervening authoritative call is retained; the POST
// fields above remain unchanged and are never substituted for these inputs.
core::Json controlNumber(double value) {return std::isfinite(value)?core::Json(value):core::Json();}
template<class Values> core::Json controlArray(const Values& values) {
    core::Json::Array result;
    for(const auto value:values)result.push_back(controlNumber(value));
    return core::Json(std::move(result));
}
core::Json controlAxis(const ms::weapon::ControlAxis& axis) {
    core::Json j;j.set("force_angle_n_rad",controlNumber(axis.forceAngle)).set("force_control_n_rad",controlNumber(axis.forceControl))
        .set("moment_angle_nm_rad",controlNumber(axis.momentAngle)).set("moment_control_nm_rad",controlNumber(axis.momentControl))
        .set("moment_rate_nm_s_rad",controlNumber(axis.momentRate));return j;
}
core::Json controlFailure(const ms::weapon::OperatingFailureV1& f) {
    core::Json j;j.set("stuck",f.stuck).set("zero",f.zero).set("hardover",f.hardover);return j;
}
struct ExactControlObservations {
    const sim::FlightModel* model;
    std::uint32_t entity;
    std::uint64_t generation,sequence=0;
    std::vector<core::Json> pending;
    bool failed=false;
    double lastTimeS=0;
    PropertyHandle totalAxialForce;
    bool axialContractFailed=false;
    std::uint64_t axialContractChecks=0;
    static void capture(void* context,const ms::weapon::ControlObservationV1& event) noexcept {
        auto& self=*static_cast<ExactControlObservations*>(context);
        try {
            if(&event.model!=self.model||event.entity!=self.entity||event.generation!=self.generation||self.pending.size()>=4096) {
                self.failed=true;return;
            }
            const auto& in=event.input;const auto& c=event.config;const auto& stops=event.stops;
            core::Json axial;
            const bool axialRequired=c.version==1&&event.mechanismCode>0;
            axial.set("required",axialRequired);
            if(axialRequired) {
                ++self.axialContractChecks;
                const double expected=self.totalAxialForce.valid()?self.totalAxialForce.get()*4.4482216152605:
                    std::numeric_limits<double>::quiet_NaN();
                const bool matches=std::isfinite(expected)&&in.measured&&in.measured->forceBodyN[0]==expected;
                self.axialContractFailed=self.axialContractFailed||!matches;
                axial.set("native_total_nongravity_fx_n",controlNumber(expected)).set("matches",matches)
                    .set("passed_fx_n",in.measured?controlNumber(in.measured->forceBodyN[0]):core::Json());
            }
            core::Json input;input.set("demand_body_m_s2",controlArray(in.demandBody)).set("gravity_body_m_s2",controlArray(in.gravityBody))
                .set("rates_rad_s",controlArray(in.ratesRadS)).set("inertia_kg_m2",controlArray(in.inertiaKgM2))
                .set("mass_kg",controlNumber(in.massKg)).set("speed_m_s",controlNumber(in.speedMs))
                .set("alpha_rad",controlNumber(in.alphaRad)).set("beta_rad",controlNumber(in.betaRad))
                .set("limits_rad",controlArray(in.limitsRad)).set("actuator_lag_s",controlNumber(in.actuatorLagS))
                .set("pitch",controlAxis(in.pitch)).set("yaw",controlAxis(in.yaw))
                .set("roll_moment_per_rad",controlNumber(in.rollMomentPerRad)).set("roll_damping",controlNumber(in.rollDamping));
            core::Json measured;
            if(in.measured)measured.set("force_body_n",controlArray(in.measured->forceBodyN))
                .set("moment_body_nm",controlArray(in.measured->momentBodyNm)).set("control_rad",controlArray(in.measured->controlRad));
            input.set("measured",std::move(measured));
            core::Json config;config.set("version",static_cast<int>(c.version)).set("algorithm",static_cast<int>(c.algorithm))
                .set("allocator",static_cast<int>(c.allocator)).set("max_incidence_rad",controlNumber(c.maxIncidenceRad))
                .set("rate_time_min_s",controlNumber(c.rateTimeMinS)).set("rate_lag_multiplier",controlNumber(c.rateLagMultiplier))
                .set("incidence_time_min_s",controlNumber(c.incidenceTimeMinS)).set("incidence_rate_multiplier",controlNumber(c.incidenceRateMultiplier))
                .set("actuator_lag_s",controlNumber(c.actuatorLagS)).set("actuator_rate_rad_s",controlNumber(c.actuatorRateRadS))
                .set("thrust_vector",c.thrustVector).set("thrust_vector_limit_rad",controlNumber(c.thrustVectorLimitRad))
                .set("thrust_vector_rate_rad_s",controlNumber(c.thrustVectorRateRadS));
            core::Json bounds;bounds.set("version",static_cast<int>(stops.version)).set("actuator_version",static_cast<int>(stops.actuatorVersion))
                .set("lower_rad",controlArray(stops.lowerRad)).set("upper_rad",controlArray(stops.upperRad))
                .set("channel_rate_rad_s",controlNumber(stops.channelRateRadS)).set("fin_rates_rad_s",controlArray(stops.finRateRadS));
            core::Json point;
            if(const auto* p=event.operatingPoint) {
                point.set("dt_s",controlNumber(p->dtS)).set("actuator_substeps",static_cast<int>(p->actuatorSubsteps))
                    .set("channel_input_rad",controlArray(p->channelInputRad)).set("channel_lag_rad",controlArray(p->channelLagRad))
                    .set("channel_actual_rad",controlArray(p->channelActualRad));
                core::Json::Array channelFailures,fins;
                for(const auto& f:p->channelFailure)channelFailures.push_back(controlFailure(f));
                for(const auto& f:p->fins) {
                    core::Json fin;fin.set("actual_rad",controlNumber(f.actualRad)).set("minimum_rad",controlNumber(f.minimumRad))
                        .set("maximum_rad",controlNumber(f.maximumRad)).set("rate_rad_s",controlNumber(f.rateRadS))
                        .set("mixer",controlArray(f.mixer)).set("failure",controlFailure(f.failure)).set("angles_rad",controlArray(f.anglesRad));
                    core::Json::Array moments;for(const auto& moment:f.momentsNm)moments.push_back(controlArray(moment));
                    fin.set("moments_nm",core::Json(std::move(moments)));fins.push_back(std::move(fin));
                }
                point.set("channel_failures",core::Json(std::move(channelFailures))).set("fins",core::Json(std::move(fins)));
            }
            core::Json commands;commands.set("available",event.commands.available)
                .set("aileron",controlNumber(event.commands.aileron)).set("elevator",controlNumber(event.commands.elevator))
                .set("rudder",controlNumber(event.commands.rudder));
            core::Json packet;packet.set("sequence",std::to_string(++self.sequence)).set("entity",std::to_string(event.entity))
                .set("generation",std::to_string(event.generation)).set("time_s",controlNumber(event.timeS)).set("dt_s",controlNumber(event.dtS))
                .set("mechanism_code",static_cast<int>(event.mechanismCode)).set("mechanism",event.mechanism).set("dispatched",event.dispatched)
                .set("input",std::move(input)).set("config",std::move(config)).set("stops",std::move(bounds))
                .set("operating_point",std::move(point)).set("commands",std::move(commands)).set("axial_force_contract",std::move(axial));
            self.pending.push_back(std::move(packet));self.lastTimeS=event.timeS;
        } catch(...) {self.failed=true;}
    }
    void row(std::ostream& out) {
        out<<','<<pending.size()<<',';if(sequence)out<<lastTimeS;
        out<<','<<failed<<",\"";
        core::Json::Array batch;batch.swap(pending);
        for(const auto c:core::Json(std::move(batch)).dump()) {if(c=='\"')out<<'\"';out<<c;}
        out<<'\"';
    }
};
// A passive hinge has no driven demand or FGActuator malfunction ports.
// Validate its actual separate state; preserve V1's unavailable driven fields.
void passiveFin(session::World& w,std::uint32_t weapon,const ipc::WeaponDiagnosticFinV1& fin) {
    auto* model=w.model(weapon);REQUIRE(model);
    const auto read=[&](const std::string& name){const auto p=model->property(name);REQUIRE(p.valid());const double v=p.get();REQUIRE(std::isfinite(v));return v;};
    const auto generic="fsim/missile-control/fin-"+std::to_string(fin.index)+"-";
    REQUIRE(read(generic+"actuation-kind-v1")==2);
    REQUIRE(read("fsim/passive-rolleron-v1/version")==1);
    const double count=read("fsim/passive-rolleron-v1/count");REQUIRE(count>0);REQUIRE(count<=32);REQUIRE(count==std::floor(count));
    bool matched=false;
    for(unsigned k=0;k<static_cast<unsigned>(count);++k) {
        const auto prefix="fsim/passive-rolleron-v1/fin-"+std::to_string(k)+"/";
        if(read(prefix+"fin-index")!=fin.index)continue;
        REQUIRE_FALSE(matched);matched=true;REQUIRE(read(prefix+"fin-id")==fin.id);
        REQUIRE(std::abs(read(prefix+"native-deflection-sign"))==1);
        REQUIRE(read(prefix+"angle-rad")==fin.actualRad);
        REQUIRE(read(prefix+"limit-rad")==fin.maximumRad);REQUIRE(fin.minimumRad==-fin.maximumRad);
        REQUIRE(std::abs(read(prefix+"rate-rad-s"))<=fin.rateRadS+1e-9);
        read(prefix+"wheel-rad-s");
    }
    REQUIRE(matched);
}
}

TEST_CASE("Each maintained AAM executes one identifiable native release to actual fuze warhead and damage","[aam-full-chain][native-engagement]") {
    const char* chosen=std::getenv("FSIM_AAM_CHAIN_CARD");REQUIRE(chosen);const std::string id=chosen;
    const char* output=std::getenv("FSIM_AAM_CHAIN_DIR");REQUIRE(output);
    const auto folder=std::filesystem::path(output)/id;allowedCapturePath(folder);std::filesystem::create_directories(folder);
    // Invalidate completion before truncating any old data. Ordinary CTest
    // remains diagnostic unless a producer source epoch was supplied BEFORE
    // execution; the collector never admits an unbound capture.
    std::filesystem::remove(folder/"capture.json");
    const char* sourceEpoch=std::getenv("FSIM_AAM_CHAIN_EPOCH");
    const bool boundEpoch=sourceEpoch&&*sourceEpoch;
    const std::string sourceHash=boundEpoch?captureHash(sourceEpoch):std::string{};
    const auto producer=captureExecutable();
    const auto runtime=fsim::testing::componentReferenceRuntimeV1(captureHash,true);
    std::ofstream trace(folder/"trace.csv",std::ios::binary|std::ios::trunc),summary(folder/"summary.csv",std::ios::binary|std::ios::trunc);
    std::ofstream fins(folder/"fins.csv",std::ios::binary|std::ios::trunc),kinematics(folder/"kinematics.csv",std::ios::binary|std::ios::trunc),events(folder/"events.csv",std::ios::binary|std::ios::trunc);
    REQUIRE(trace.good());REQUIRE(summary.good());trace<<std::setprecision(17);summary<<std::setprecision(17);
    REQUIRE(fins.good());REQUIRE(kinematics.good());REQUIRE(events.good());
    fins<<std::setprecision(17);kinematics<<std::setprecision(17);events<<std::setprecision(17);
    fins<<"card,weapon,generation,time,fin_id,index,driven_available,actual_rad,demand_rad,min_rad,max_rad,rate_limit_rad_s\n";
    kinematics<<"card,weapon,generation,time,age,wx,wy,wz,wvx,wvy,wvz,tx,ty,tz,tvx,tvy,tvz,range_m,guidance_flags,measured_range_m,measured_range_rate_ms,track_epoch,nav_status,nav_epoch,nav_x,nav_y,nav_z,nav_vx,nav_vy,nav_vz,los_x,los_y,los_z,los_rate_x,los_rate_y,los_rate_z,pointing_x,pointing_y,pointing_z,thermal_flags,thermal_epoch,centroid_variance_rad2,centroid_snr,quadrant0,quadrant1,quadrant2,quadrant3,quadrant_variance_electrons2,demand_ax,demand_ay,demand_az,command_roll,command_pitch,command_yaw";
    kinematics<<",physical_p_rad_s,physical_q_rad_s,physical_r_rad_s";
    for(unsigned k=0;k<9;++k)kinematics<<",physical_body_to_ecef_"<<k;
    kinematics<<",nav_qw,nav_qx,nav_qy,nav_qz,nav_gyro_x_rad_s,nav_gyro_y_rad_s,nav_gyro_z_rad_s,nav_gyro_bias_x_rad_s,nav_gyro_bias_y_rad_s,nav_gyro_bias_z_rad_s,nav_last_imu_time_s,control_navigation_time_s,native_post_time_s,cg_body_x_m,cg_body_y_m,cg_body_z_m,physical_aileron_rad,physical_elevator_rad,physical_rudder_rad";
    events<<"card,weapon,generation,serial,time,kind,reason,source,target,destroyed\n";
    trace<<"card,weapon,generation,time,age,phase,tracking,burning,thermal_status,rf_status,guidance_status,guidance_flags,control_status,controller_time,fin_count,propulsion_flags,fuel_kg,thrust_n,fuze_flags,burst_time,fragments,damage_records,component_energy_j,alpha,beta,speed,fx,fy,fz,l,m,n,propulsion_status,propulsion_epoch,mass_kg,fuel_flow_kg_s,native_motor_age_s,pulse_state,valve_actual,inlet_air_kg_s,inlet_temperature_k,combustor_temperature_k,net_thrust_n,control_flags,achieved_ax,achieved_ay,achieved_az,tvc_demand_pitch_rad,tvc_demand_yaw_rad,tvc_actual_pitch_rad,tvc_actual_yaw_rad,control_pre_count,control_pre_last_time_s,control_pre_error,control_pre_json,reticle_scalar_json\n";
    summary<<"card,weapon,generation,released,seeker,guidance,autopilot,allocator,actuator,aerodynamics,propulsion,fuze,warhead,damage,terminal,terminal_time,samples,burst_time,fragments,damage_records,component_energy_j\n";
    session::WorldOptions o;o.name="aam-full-chain-"+id;o.seed=17;o.workers=1;o.pinWorkers=false;
    o.publish=true;o.dt=1./120;o.frameSkip=4;o.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    session::World w(o);
    std::vector<std::string> errors;const auto cards=ms::readStoreLibrary(FSIM_TEST_STORE_DIR,errors);REQUIRE(errors.empty());
    const ms::StoreCard* card=nullptr;
    std::set<std::string> roster;
    for(const auto& [name,row]:cards)if(row->storeClass=="aam") {roster.insert(name);if(name==id)card=row.get();}
    REQUIRE(roster.size()==24);REQUIRE(card);
    ms::components::ProfileV1 profile;std::string error;
    REQUIRE(ms::components::readProfileV1(std::filesystem::path(FSIM_TEST_STORE_DIR)/(id+".json"),*card,profile,error));
    std::ofstream mechanisms(folder/"components.csv",std::ios::binary|std::ios::trunc);REQUIRE(mechanisms.good());
    mechanisms<<"card,component,implementation,mechanism\n";
    for(const auto* kind:slots) {
        const auto* descriptor=profile.find(kind);REQUIRE(descriptor);
        REQUIRE(descriptor->availability==ms::components::AvailabilityV1::Available);
        mechanisms<<id<<','<<kind<<','<<descriptor->implementationId<<','<<equipment::mechanism(descriptor->implementationId)<<'\n';
    }
    mechanisms.flush();REQUIRE(mechanisms.good());
    session::VehicleSpec own;own.name="launcher";own.type=card->seeker=="semi_active_radar"?"jsbsim:fa18c":"jsbsim:f16c";
    own.initial.latitudeDeg=40;own.initial.longitudeDeg=0;own.initial.altitudeMslM=6000;
    own.initial.airspeedTrueMs=250;own.initial.headingDeg=90;
    const auto launcher=w.createVehicle(own);REQUIRE(launcher);
    control::VelocityCommand hold;hold.airspeedMs=250;hold.verticalSpeedMs=0;hold.headingRad=90*deg;
    REQUIRE(w.submit(launcher,hold).accepted());
    const auto station=payload(w,launcher,ms::Family::StoreStation),radar=payload(w,launcher,ms::Family::AirRadar);
    const auto* stationCard=w.missionSystems(launcher)->card(station);REQUIRE(stationCard);
    int mount=0;const double loaded=card->value("mass_kg",0)+card->value("fuel_kg",0);
    for(const auto& fit:stationCard->stations)if(fit.maxKg>=loaded&&std::find(fit.takes.begin(),fit.takes.end(),"aam")!=fit.takes.end()){mount=fit.number;break;}
    REQUIRE(mount>0);REQUIRE(w.loadStore(launcher,station,mount,id)==ms::Reason::None);
    operate(w,launcher,station);operate(w,launcher,radar);w.step(300);
    const auto* origin=w.vehicleState(launcher);REQUIRE(origin);
    session::VehicleSpec opponent;opponent.name="opponent";opponent.type="jsbsim:f16c";
    opponent.initial.latitudeDeg=origin->latitudeRad/deg;
    opponent.initial.longitudeDeg=origin->longitudeRad/deg+5000/(6371000*std::cos(origin->latitudeRad))/deg;
    opponent.initial.altitudeMslM=6000;opponent.initial.airspeedTrueMs=250;opponent.initial.headingDeg=270;
    const auto target=w.createVehicle(opponent);REQUIRE(target);hold.headingRad=270*deg;REQUIRE(w.submit(target,hold).accepted());
    ms::AmtiCommand search;search.payload=radar;search.bars=1;search.azimuthHalfWidthRad=15*deg;search.rangeScaleM=60000;
    REQUIRE(w.submit(launcher,search).accepted());std::uint64_t track=0;
    for(unsigned i=0;i<600&&!track;++i) {
        w.step();for(const auto& t:w.amtiTracks(launcher,radar))if(t.updates>=3){track=t.id;break;}
    }
    REQUIRE(track);const auto designation=w.designate(launcher,radar,track);REQUIRE(designation>=session::World::kDesignations);
    ms::AmtiCommand stt;stt.payload=radar;stt.task=ms::AmtiTask::Track;stt.mode=ms::AmtiMode::SingleTargetTrack;
    stt.track=track;stt.rangeScaleM=60000;stt.illuminate=card->seeker=="semi_active_radar";
    REQUIRE(w.submit(launcher,stt).accepted());
    ms::StrikeCommand strike;strike.payload=station;strike.store=id;strike.target=designation;
    const auto request=w.submit(launcher,strike);REQUIRE(request.accepted());REQUIRE(w.weapons().empty());
    for(unsigned i=0;i<1350;++i) {
        const auto status=w.strikeStatus(request.activity);REQUIRE(status);
        if(status->phase==ms::StrikePhase::AwaitingConsent)break;
        w.step();
    }
    REQUIRE(w.strikeStatus(request.activity)->phase==ms::StrikePhase::AwaitingConsent);
    REQUIRE(w.weapons().empty());REQUIRE(w.strikeConsent(request.activity,true)==ms::Reason::None);
    for(unsigned i=0;i<600&&w.weapons().empty();++i)w.step();
    REQUIRE(w.weapons().size()==1);const auto weapon=w.weapons().front().id;
    REQUIRE(record(w,weapon).store==id);REQUIRE_FALSE(record(w,weapon).jettisoned);
    REQUIRE(w.info(weapon));const auto generation=w.info(weapon)->generation;
    REQUIRE(w.info(weapon)->type==card->model);
    const auto* seekerDescriptor=profile.find("seeker");REQUIRE(seekerDescriptor);
    const auto receiverBinding=seekerDescriptor->bindings.find("receiver_model");
    const bool scalarReceiver=receiverBinding!=seekerDescriptor->bindings.end()&&receiverBinding->second=="thermal_conscan_reticle.v1";
    std::optional<double> lastReticlePacket;
    const double launch=record(w,weapon).launchTimeS;Snapshot snapshot;
    const NativeControlObservations nativeControl(*w.model(weapon));
    ExactControlObservations controlPre{w.model(weapon),weapon,generation,0,{},false,0,
        w.model(weapon)->property("forces/fbx-total-lbs"),false,0};
    const ms::weapon::ScopedControlObserverV1 observeControl{{w.model(weapon),&controlPre,&ExactControlObservations::capture}};
    nativeControl.header(kinematics);kinematics<<'\n';
    std::array<bool,9> executed{};unsigned samples=0;bool actualDamage=false;
    std::set<std::uint32_t> verifiedPassive;
    double burstTime=0,energy=0;std::uint32_t fragments=0,damageRecords=0;
    const std::array<const char*,6> axes{{"forces/fbx-aero-lbs","forces/fby-aero-lbs","forces/fbz-aero-lbs","moments/l-aero-lbsft","moments/m-aero-lbsft","moments/n-aero-lbsft"}};
    while(w.simTime()<launch+40) {
        w.step();finiteWorld(w);
        if(snapshot.read(w,o.name,weapon,generation)) {
            const auto& f=*snapshot.frame;REQUIRE(std::string(f.store)==id);++samples;
            const auto available=ipc::WeaponDiagnosticStatusV1::Available;
            const auto* reticle=scalarReceiver?w.weaponReticleObservationV1(weapon):nullptr;
            if(scalarReceiver) {
                // The selected mechanism has one scalar detector, not image
                // pixels. Require its actual resolved post-release capture
                // together with authoritative tracking, preserving stage meaning.
                if(reticle&&reticle->valid&&reticle->complete) {
                    REQUIRE(reticle->phases.size()==reticle->measuredElectrons.size());
                    REQUIRE(reticle->phases.size()==reticle->expectedElectrons.size());
                    REQUIRE(reticle->phases.size()==reticle->incidentElectrons.size());
                    REQUIRE(reticle->phases.size()<=512);
                    executed[0]=executed[0]||(reticle->bearingResolved&&!reticle->features.empty()&&
                        !reticle->measuredElectrons.empty()&&reticle->measurementTimeS>=launch&&
                        reticle->timeS<=w.simTime()&&record(w,weapon).tracking);
                }
            } else {
                executed[0]=executed[0]||((f.thermal.stamp.status==available||f.rf.stamp.status==available)&&record(w,weapon).tracking);
            }
            executed[1]=executed[1]||(f.guidance.stamp.status==available&&(f.guidance.flags&ipc::WeaponDiagnosticLosV1));
            executed[2]=executed[2]||(f.control.stamp.status==available&&(f.control.flags&ipc::WeaponDiagnosticControllerV1));
            executed[3]=executed[3]||(f.control.stamp.status==available&&f.control.finCount>0);
            for(std::uint32_t k=0;k<f.control.finCount;++k) {
                const auto& fin=f.control.fins[k];
                if((fin.flags&1u)==0) {
                    if(f.control.stamp.status==available) {REQUIRE(w.alive(weapon));passiveFin(w,weapon,fin);verifiedPassive.insert(fin.id);}
                    else {REQUIRE(f.control.stamp.status==ipc::WeaponDiagnosticStatusV1::Removed);REQUIRE(verifiedPassive.count(fin.id)==1);}
                }
                REQUIRE(std::isfinite(fin.actualRad));
                REQUIRE(fin.actualRad>=fin.minimumRad-1e-9);REQUIRE(fin.actualRad<=fin.maximumRad+1e-9);
                executed[4]=executed[4]||((fin.flags&1u)!=0);
                fins<<id<<','<<weapon<<','<<generation<<','<<w.simTime()<<','<<fin.id<<','<<fin.index<<','<<((fin.flags&1u)!=0)<<','<<fin.actualRad<<',';
                if(fin.flags&1u)fins<<fin.demandRad;
                fins<<','<<fin.minimumRad<<','<<fin.maximumRad<<','<<fin.rateRadS<<'\n';
            }
            executed[6]=executed[6]||(f.propulsion.stamp.status==available&&(f.propulsion.flags&ipc::WeaponDiagnosticBurningV1)&&f.propulsion.fuelKg>=0);
            executed[7]=executed[7]||((f.effects.flags&ipc::WeaponDiagnosticArmedV1)&&(f.effects.flags&ipc::WeaponDiagnosticFuzeV1));
            executed[8]=executed[8]||((f.effects.flags&ipc::WeaponDiagnosticBurstV1)&&(f.effects.flags&ipc::WeaponDiagnosticWarheadV1)&&f.effects.totalFragmentCount>0);
            fragments=std::max(fragments,f.effects.totalFragmentCount);damageRecords=std::max(damageRecords,f.effects.totalDamageCount);
            burstTime=std::max(burstTime,f.effects.burstTimeS);double measuredEnergy=0;
            for(std::uint32_t k=0;k<f.effects.damageCount;++k)if(f.effects.damage[k].victim==target)measuredEnergy+=f.effects.damage[k].penetratedEnergyJ;
            energy=std::max(energy,measuredEnergy);
            std::array<double,6> aero{};double alpha=0,beta=0,speed=0,mass=0;
            if(const auto* state=w.vehicleState(weapon);state&&w.alive(weapon)) {
                bool valid=true;for(std::size_t k=0;k<axes.size();++k) {
                    const auto p=w.model(weapon)->property(axes[k]);valid=valid&&p.valid();if(p.valid()){aero[k]=p.get();REQUIRE(std::isfinite(aero[k]));}
                }
                executed[5]=executed[5]||valid;alpha=state->alphaRad;beta=state->betaRad;speed=state->airspeedTrueMs;
                mass=w.model(weapon)->energy().massKg;
                const auto* targetState=w.vehicleState(target);REQUIRE(targetState);
                kinematics<<id<<','<<weapon<<','<<generation<<','<<w.simTime()<<','<<w.simTime()-launch;
                for(const auto* s:{state,targetState}) {
                    for(const auto v:s->positionEcef)kinematics<<','<<v;
                    for(unsigned a=0;a<3;++a)kinematics<<','<<(s->rotationBodyToEcef[3*a]*s->velocityBodyMs[0]+s->rotationBodyToEcef[3*a+1]*s->velocityBodyMs[1]+s->rotationBodyToEcef[3*a+2]*s->velocityBodyMs[2]);
                }
                kinematics<<','<<std::hypot(state->positionEcef[0]-targetState->positionEcef[0],state->positionEcef[1]-targetState->positionEcef[1],state->positionEcef[2]-targetState->positionEcef[2])
                    <<','<<f.guidance.flags<<','<<f.guidance.rangeM<<','<<f.guidance.rangeRateMs<<','<<f.guidance.trackTimeS<<','<<static_cast<unsigned>(f.navigation.stamp.status)<<','<<f.navigation.stamp.sampleTimeS;
                for(const auto v:f.navigation.positionEcefM)kinematics<<','<<v;
                for(const auto v:f.navigation.velocityEcefMs)kinematics<<','<<v;
                for(const auto v:f.guidance.losDirectionEcef)kinematics<<','<<v;
                for(const auto v:f.guidance.losDirectionRatePerS)kinematics<<','<<v;
                for(const auto v:f.thermal.pointingEcef)kinematics<<','<<v;
                kinematics<<','<<f.thermal.flags<<','<<f.thermal.stamp.sampleTimeS<<','<<f.thermal.centroidVarianceRad2<<','<<f.thermal.centroidSnrAmplitude;
                for(const auto v:f.thermal.quadrantContrastElectrons)kinematics<<','<<v;
                kinematics<<','<<f.thermal.quadrantVarianceElectrons2;
                for(const auto v:f.control.demandedAccelerationBodyMs2)kinematics<<','<<v;
                for(const auto v:f.control.commands)kinematics<<','<<v;
                for(const auto v:state->angularRateBodyRadS)kinematics<<','<<v;
                for(const auto v:state->rotationBodyToEcef)kinematics<<','<<v;
                const bool navigationAvailable=f.navigation.stamp.status==available;
                for(const auto v:f.navigation.quaternionBodyToEcef)observedScalar(kinematics,v,navigationAvailable);
                for(const auto v:f.navigation.measuredGyroBodyRadS)observedScalar(kinematics,v,navigationAvailable);
                for(const auto v:f.navigation.gyroBiasBodyRadS)observedScalar(kinematics,v,navigationAvailable);
                observedScalar(kinematics,f.navigation.lastImuTimeS,navigationAvailable);
                observedScalar(kinematics,f.control.navigationTimeS,(f.control.flags&ipc::WeaponDiagnosticControlNavigationEpochV1)!=0);
                kinematics<<','<<w.simTime();
                double cg[3];const bool cgAvailable=w.model(weapon)->cgFromOriginBodyM(cg);
                for(unsigned a=0;a<3;++a)observedScalar(kinematics,cgAvailable?cg[a]:0,cgAvailable);
                for(const auto v:{state->aileronRad,state->elevatorRad,state->rudderRad})observedScalar(kinematics,v,true);
                nativeControl.row(kinematics);
                kinematics<<'\n';
            }
            double thrust=0;for(std::uint32_t k=0;k<f.propulsion.engineCount;++k)thrust+=f.propulsion.thrustN[k];
            trace<<id<<','<<weapon<<','<<generation<<','<<w.simTime()<<','<<w.simTime()-launch<<','<<f.guidance.phase<<','
                <<record(w,weapon).tracking<<','<<record(w,weapon).burning<<','<<static_cast<unsigned>(f.thermal.stamp.status)<<','
                <<static_cast<unsigned>(f.rf.stamp.status)<<','<<static_cast<unsigned>(f.guidance.stamp.status)<<','<<f.guidance.flags<<','
                <<static_cast<unsigned>(f.control.stamp.status)<<','<<f.control.controllerTimeS<<','<<f.control.finCount<<','
                <<f.propulsion.flags<<','<<f.propulsion.fuelKg<<','<<thrust<<','<<f.effects.flags<<','<<f.effects.burstTimeS<<','
                <<f.effects.totalFragmentCount<<','<<f.effects.totalDamageCount<<','<<measuredEnergy<<','<<alpha<<','<<beta<<','<<speed;
            for(const auto value:aero)trace<<','<<value;
            double flow=0;for(std::uint32_t k=0;k<f.propulsion.engineCount;++k)flow+=f.propulsion.fuelFlowKgS[k];
            trace<<','<<static_cast<unsigned>(f.propulsion.stamp.status)<<','<<f.propulsion.stamp.sampleTimeS<<','<<mass<<','<<flow<<','
                <<f.propulsion.nativeMotorAgeS<<','<<f.propulsion.pulseState<<','<<f.propulsion.valveActual<<','<<f.propulsion.inletAirKgS<<','
                <<f.propulsion.inletTemperatureK<<','<<f.propulsion.combustorTemperatureK<<','<<f.propulsion.netThrustN;
            trace<<','<<f.control.flags;
            for(const auto v:f.control.achievedAccelerationBodyMs2)observedScalar(trace,v,(f.control.flags&ipc::WeaponDiagnosticAccelerationAchievedV1)!=0);
            for(const auto v:{f.control.tvcDemandPitchRad,f.control.tvcDemandYawRad,f.control.tvcActualPitchRad,f.control.tvcActualYawRad})
                observedScalar(trace,v,(f.control.flags&ipc::WeaponDiagnosticTvcV1)!=0);
            controlPre.row(trace);
            trace<<',';
            if(reticle&&reticle->valid&&(!lastReticlePacket||*lastReticlePacket!=reticle->timeS)) {
                core::Json packet;packet.set("model",id).set("weapon",std::to_string(weapon)).set("generation",std::to_string(generation))
                    .set("mechanism","thermal_conscan_reticle.v1").set("valid",true).set("complete",reticle->complete)
                    .set("time_s",controlNumber(reticle->timeS));
                if(reticle->complete) {
                    packet.set("capture_time_s",controlNumber(reticle->measurementTimeS)).set("exposure_start_s",controlNumber(reticle->exposureStartTimeS))
                        .set("exposure_end_s",controlNumber(reticle->exposureEndTimeS)).set("bearing_resolved",reticle->bearingResolved)
                        .set("precise_bearing",reticle->preciseBearing)
                        .set("saturated",reticle->saturated).set("fit_snr",controlNumber(reticle->fitSnr))
                        .set("fit_residual",controlNumber(reticle->fitResidual)).set("adc_step_electrons",controlNumber(reticle->adcStepElectrons))
                        .set("variance_bound_electrons2",controlNumber(reticle->varianceBoundElectrons2))
                        .set("pointing_x",controlNumber(reticle->pointing.x)).set("pointing_y",controlNumber(reticle->pointing.y)).set("pointing_z",controlNumber(reticle->pointing.z))
                        .set("feature_count",static_cast<double>(reticle->features.size()))
                        .set("phase_rad",controlArray(reticle->phases)).set("incident_electrons",controlArray(reticle->incidentElectrons))
                        .set("expected_electrons",controlArray(reticle->expectedElectrons)).set("measured_electrons",controlArray(reticle->measuredElectrons));
                    const bool fitAvailable=std::isfinite(reticle->compactFitSnr)&&reticle->compactFitSnr>0;
                    packet.set("compact_fit_available",fitAvailable)
                        .set("compact_fit_snr",controlNumber(reticle->compactFitSnr))
                        .set("compact_fit_residual",controlNumber(reticle->compactFitResidual))
                        .set("fitted_centre_x_rad",fitAvailable?controlNumber(reticle->fittedCentreXRad):core::Json())
                        .set("fitted_centre_y_rad",fitAvailable?controlNumber(reticle->fittedCentreYRad):core::Json())
                        .set("fitted_width_rad",fitAvailable?controlNumber(reticle->fittedWidthRad):core::Json())
                        .set("fitted_height_rad",fitAvailable?controlNumber(reticle->fittedHeightRad):core::Json());
                    // Observer only: exact native own-pose and inverse refusal
                    // state. Unvisited stages retain explicit unavailable/null.
                    const auto& horizon=reticle->horizonDiagnostic;
                    const bool horizonAttempted=horizon.refusal!=0;
                    const bool nullAvailable=horizon.refusal>=4&&std::isfinite(horizon.nullResidual);
                    const bool localVarianceAvailable=horizon.refusal>=17&&std::isfinite(horizon.localVariance)&&horizon.localVariance>0;
                    core::Json inverse;
                    inverse.set("attempted",horizonAttempted)
                        .set("refusal_code",horizonAttempted?core::Json(static_cast<double>(horizon.refusal)):core::Json())
                        .set("own_up_x",horizonAttempted?controlNumber(horizon.ownUp.x):core::Json())
                        .set("own_up_y",horizonAttempted?controlNumber(horizon.ownUp.y):core::Json())
                        .set("own_up_z",horizonAttempted?controlNumber(horizon.ownUp.z):core::Json())
                        .set("null_fit_available",nullAvailable)
                        .set("null_cutoff",nullAvailable?controlNumber(horizon.nullCutoff):core::Json())
                        .set("null_residual",nullAvailable?controlNumber(horizon.nullResidual):core::Json())
                        .set("source_fit_available",horizon.fitAvailable)
                        .set("source_shape_x_y_width_height_cutoff",horizon.fitAvailable?controlArray(horizon.shape):core::Json())
                        .set("source_residual",horizon.fitAvailable?controlNumber(horizon.sourceResidual):core::Json())
                        .set("raw_budget_available",horizon.budgetAvailable)
                        .set("raw_budget_sky_ground_source",horizon.budgetAvailable?controlArray(horizon.rawBudget):core::Json())
                        .set("information_available",horizon.informationAvailable)
                        .set("information_determinant",horizon.informationAvailable?controlNumber(horizon.informationDeterminant):core::Json())
                        .set("information_trace",horizon.informationAvailable?controlNumber(horizon.informationTrace):core::Json())
                        .set("local_variance_available",localVarianceAvailable)
                        .set("local_variance_rad2",localVarianceAvailable?controlNumber(horizon.localVariance):core::Json())
                        .set("alternative_available",horizon.alternativeAvailable)
                        .set("alternative_shape_x_y_width_height_cutoff",horizon.alternativeAvailable?controlArray(horizon.alternativeShape):core::Json())
                        .set("alternative_delta_cost_sigma2",horizon.alternativeAvailable?controlNumber(horizon.alternativeDelta):core::Json())
                        .set("alternative_separation_rad",horizon.alternativeAvailable?controlNumber(horizon.alternativeSeparation):core::Json());
                    packet.set("horizon_inference",std::move(inverse));
                    if(!reticle->features.empty()) {
                        const auto& feature=reticle->features.front();
                        packet.set("feature_x",controlNumber(feature.direction.x)).set("feature_y",controlNumber(feature.direction.y)).set("feature_z",controlNumber(feature.direction.z))
                            .set("feature_width_rad",controlNumber(feature.extent.widthRad)).set("feature_height_rad",controlNumber(feature.extent.heightRad))
                            .set("feature_variance_rad2",controlNumber(feature.angularVarianceRad2));
                    }
                }
                trace<<'"';for(const auto ch:packet.dump()){if(ch=='"')trace<<'"';trace<<ch;}trace<<'"';
                lastReticlePacket=reticle->timeS;
            }
            trace<<'\n';
        }
        for(const auto& event:w.engagementEvents())if(event.entity==weapon&&event.target==target&&event.kind=="damage")actualDamage=true;
    }
    const auto final=record(w,weapon);
    REQUIRE_FALSE(controlPre.failed);
    REQUIRE_FALSE(controlPre.axialContractFailed);
    REQUIRE(controlPre.axialContractChecks>0);
    REQUIRE(controlPre.sequence>0);
    REQUIRE(controlPre.pending.empty());
    for(const auto& e:w.engagementEvents())if(e.entity==weapon||e.target==target)
        events<<id<<','<<weapon<<','<<generation<<','<<e.serial<<','<<e.timeS<<','<<e.kind<<','<<std::quoted(e.reason)<<','<<e.entity<<','<<e.target<<','<<e.destroyed<<'\n';
    summary<<id<<','<<weapon<<','<<generation<<",1";for(const bool value:executed)summary<<','<<value;
    summary<<','<<actualDamage<<','<<ms::weaponEndName(final.end)<<',';
    if(std::isfinite(final.endTimeS))summary<<final.endTimeS;
    summary<<','<<samples<<','<<burstTime<<','<<fragments<<','<<damageRecords<<','<<energy<<'\n';
    trace.flush();summary.flush();REQUIRE(trace.good());REQUIRE(summary.good());
    fins.flush();kinematics.flush();events.flush();REQUIRE(fins.good());REQUIRE(kinematics.good());REQUIRE(events.good());
    INFO("model="<<id<<" terminal="<<ms::weaponEndName(final.end)<<" samples="<<samples);
    for(std::size_t k=0;k<slots.size();++k){INFO("component="<<slots[k]);CHECK(executed[k]);}
    CHECK(final.end==ms::WeaponEnd::Detonated);CHECK(actualDamage);CHECK(energy>0);
    // Publish only after every output stream successfully closes. These are
    // provenance observations; all original encounter assertions stay above.
    for(auto* stream:{&trace,&summary,&mechanisms,&fins,&kinematics,&events}) {
        stream->close();REQUIRE_FALSE(stream->fail());
    }
    core::Json::Array files;
    for(const auto* name:{"summary.csv","trace.csv","components.csv","fins.csv","kinematics.csv","events.csv"}) {
        const auto path=folder/name;core::Json file;
        file.set("path",name).set("sha256",captureHash(path)).set("bytes",static_cast<double>(std::filesystem::file_size(path)));
        files.push_back(std::move(file));
    }
    core::Json capture;capture.set("schema","fsim.sim-m03.aam-chain-capture.v1").set("complete",true)
        .set("admission_eligible",boundEpoch).set("input_epoch_sha256",sourceHash)
        .set("model",id).set("native_model",card->model).set("weapon",std::to_string(weapon))
        .set("generation",std::to_string(generation)).set("terminal",ms::weaponEndName(final.end))
        .set("launch_time_s",launch).set("samples",static_cast<double>(samples))
        .set("executable",producer).set("runtime_modules",runtime).set("files",core::Json(std::move(files)));
    if(std::isfinite(final.endTimeS))capture.set("terminal_time_s",final.endTimeS);
    else capture.set("terminal_time_s",core::Json{});
    std::ofstream marker(folder/"capture.json",std::ios::binary|std::ios::trunc);REQUIRE(marker.good());
    marker<<capture.dump()<<'\n';marker.flush();REQUIRE(marker.good());marker.close();REQUIRE_FALSE(marker.fail());
}
