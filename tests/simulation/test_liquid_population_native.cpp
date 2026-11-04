// Complete native material receiver population. Referee diagnostic point sources
// qualify capture/transit/reset, not flight fitness, warhead lethality or calibration.
#include "simulation/runtime/world/World.h"
#include "simulation/runtime/world/WorldLiquidMaterials.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/weapon/MaterialPoint.h"
#include "foundation/core/Json.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
namespace {
using namespace fsim;
namespace damage=ms::damage;
namespace components=ms::components;
using damage::V3;
using core::Json;
namespace fs=std::filesystem;
constexpr double lbsToKg=.45359237,gravityMs2=9.80665;
constexpr double pointMassKg=.01,pointAreaM2=.0001,pointAirCd=1.3,relativeSpeedMs=5;
const std::vector<std::string> designs={"a10c","b52h","c130j","c172","c17a","cf2","e3g","e7a","ea18g","ec130h","f15c","f16c","f22a","f35a","fa18c","gripen","h6k","iris","j10a","j20a","kc135r","kc46a","mig29a","mirage2000","rafale","rc135w","rq4b","skua","su25","su27s","su57","typhoon","u2s","uh1h","uh60"};
V3 vector(const double* p){return {p[0],p[1],p[2]};}
double coordinate(V3 p,unsigned axis){return axis==0?p.x:(axis==1?p.y:p.z);}
void coordinate(V3& p,unsigned axis,double value){if(axis==0)p.x=value;else if(axis==1)p.y=value;else p.z=value;}
double norm(V3 p){return std::hypot(p.x,p.y,p.z);}
V3 rotate(const sim::VehicleState& s,V3 p) {
    const auto* r=s.rotationBodyToEcef;
    return {r[0]*p.x+r[1]*p.y+r[2]*p.z,r[3]*p.x+r[4]*p.y+r[5]*p.z,r[6]*p.x+r[7]*p.y+r[8]*p.z};
}
V3 body(const sim::VehicleState& s,V3 p) {
    const auto* r=s.rotationBodyToEcef;
    return {r[0]*p.x+r[3]*p.y+r[6]*p.z,r[1]*p.x+r[4]*p.y+r[7]*p.z,r[2]*p.x+r[5]*p.y+r[8]*p.z};
}
V3 up(const sim::VehicleState& s) {
    return body(s,{std::cos(s.latitudeRad)*std::cos(s.longitudeRad),
        std::cos(s.latitudeRad)*std::sin(s.longitudeRad),std::sin(s.latitudeRad)});
}
void close(V3 a,V3 b,double absolute) {CHECK(norm(a-b)<=absolute);}

// Independent projected-uniform-box CDF, not the native clipping/free-surface
// implementation. Tiny projected dimensions are dropped with an explicit
// spatial bound; selected wet/dry chords must clear that bound by >1000x.
struct CdfVolume {double volume=0,planeUncertaintyM=0;};
CdfVolume referenceVolume(damage::fluid::Aabb box,V3 normal,double offset) {
    const V3 size=box.high-box.low;
    const std::array<double,3> widths{std::abs(normal.x)*size.x,std::abs(normal.y)*size.y,std::abs(normal.z)*size.z};
    const double maximum=*std::max_element(widths.begin(),widths.end());
    const V3 corner{normal.x>=0?box.low.x:box.high.x,normal.y>=0?box.low.y:box.high.y,normal.z>=0?box.low.z:box.high.z};
    long double t=static_cast<long double>(offset)-static_cast<long double>(normal.x)*corner.x-
        static_cast<long double>(normal.y)*corner.y-static_cast<long double>(normal.z)*corner.z;
    std::array<long double,3> active{};unsigned count=0;double uncertainty=0;
    for(const double width:widths) {
        if(width<=maximum*1e-10)uncertainty+=width;
        else active[count++]=width;
    }
    REQUIRE(count>0);
    long double extent=0,denominator=1;
    for(unsigned i=0;i<count;++i){extent+=active[i];denominator*=active[i]*static_cast<long double>(i+1);}
    const double full=size.x*size.y*size.z;
    if(t<=0)return {0,uncertainty};
    if(t>=extent)return {full,uncertainty};
    long double numerator=0;
    for(unsigned mask=0;mask<(1u<<count);++mask) {
        long double x=t;unsigned bits=0;
        for(unsigned i=0;i<count;++i)if(mask&(1u<<i)){x-=active[i];++bits;}
        if(x<=0)continue;
        long double power=1;for(unsigned i=0;i<count;++i)power*=x;
        numerator+=(bits%2?-power:power);
    }
    const double fraction=static_cast<double>(numerator/denominator);
    REQUIRE(std::isfinite(fraction));REQUIRE(fraction>=-1e-8);REQUIRE(fraction<=1+1e-8);
    return {std::clamp(fraction,0.,1.)*full,uncertainty};
}
double planeAtMass(const session::LiquidMaterialDatasetV1& data,int tank,double mass,V3 normal,V3 shift) {
    const auto found=std::find_if(data.tanks.begin(),data.tanks.end(),[&](const auto& row){return row.index==tank;});
    REQUIRE(found!=data.tanks.end());
    double low=std::numeric_limits<double>::infinity(),high=-low,total=0;
    for(const auto& cell:data.cells)if(cell.nativeTank==tank) {
        const auto a=cell.bodyBounds.low-shift,b=cell.bodyBounds.high-shift;
        const V3 lo{normal.x>=0?a.x:b.x,normal.y>=0?a.y:b.y,normal.z>=0?a.z:b.z};
        const V3 hi{normal.x>=0?b.x:a.x,normal.y>=0?b.y:a.y,normal.z>=0?b.z:a.z};
        low=std::min(low,damage::dot(normal,lo));high=std::max(high,damage::dot(normal,hi));
        total+=(b.x-a.x)*(b.y-a.y)*(b.z-a.z);
    }
    const double requested=mass/found->densityKgM3;REQUIRE(requested>=0);REQUIRE(requested<=total+1e-8);
    if(requested==0)return low;
    if(requested>=total)return high;
    for(unsigned iteration=0;iteration<100;++iteration) {
        const double level=low+(high-low)*.5;long double volume=0;
        for(const auto& cell:data.cells)if(cell.nativeTank==tank)
            volume+=referenceVolume({cell.bodyBounds.low-shift,cell.bodyBounds.high-shift},normal,level).volume;
        if(volume<requested)low=level;else high=level;
    }
    return low+(high-low)*.5;
}

double pointTriangle(V3 p,V3 a,V3 b,V3 c) {
    const V3 ab=b-a,ac=c-a,ap=p-a;
    const double d1=damage::dot(ab,ap),d2=damage::dot(ac,ap);
    if(d1<=0&&d2<=0)return norm(ap);
    const V3 bp=p-b;const double d3=damage::dot(ab,bp),d4=damage::dot(ac,bp);
    if(d3>=0&&d4<=d3)return norm(bp);
    const double vc=d1*d4-d3*d2;
    if(vc<=0&&d1>=0&&d3<=0)return norm(p-(a+ab*(d1/(d1-d3))));
    const V3 cp=p-c;const double d5=damage::dot(ab,cp),d6=damage::dot(ac,cp);
    if(d6>=0&&d5<=d6)return norm(cp);
    const double vb=d5*d2-d1*d6;
    if(vb<=0&&d2>=0&&d6<=0)return norm(p-(a+ac*(d2/(d2-d6))));
    const double va=d3*d6-d5*d4;
    if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)return norm(p-(b+(c-b)*((d4-d3)/((d4-d3)+(d5-d6)))));
    const V3 n=damage::cross(ab,ac);const double length=norm(n);
    REQUIRE(length>0);return std::abs(damage::dot(ap,n))/length;
}
double segmentDistance(V3 a,V3 b,V3 c,V3 d) {
    const V3 u=b-a,v=d-c,w=a-c;
    const double aa=damage::dot(u,u),bb=damage::dot(u,v),cc=damage::dot(v,v),dd=damage::dot(u,w),ee=damage::dot(v,w);
    double s=0,t=0;
    if(aa==0)return norm(a-(c+v*(cc>0?std::clamp(ee/cc,0.,1.):0)));
    if(cc==0)return norm(a+u*std::clamp(-dd/aa,0.,1.)-c);
    const double denominator=aa*cc-bb*bb;
    if(denominator>0)s=std::clamp((bb*ee-cc*dd)/denominator,0.,1.);
    t=(bb*s+ee)/cc;
    if(t<0){t=0;s=std::clamp(-dd/aa,0.,1.);}
    else if(t>1){t=1;s=std::clamp((bb-dd)/aa,0.,1.);}
    return norm(w+u*s-v*t);
}
double triangleClearance(V3 start,V3 end,V3 a,V3 b,V3 c) {
    const V3 direction=end-start,n=damage::cross(b-a,c-a);
    const double denominator=damage::dot(n,direction);
    if(denominator!=0) {
        const double t=damage::dot(n,a-start)/denominator;
        if(t>=0&&t<=1&&pointTriangle(start+direction*t,a,b,c)<1e-10)return 0;
    }
    return std::min({pointTriangle(start,a,b,c),pointTriangle(end,a,b,c),
        segmentDistance(start,end,a,b),segmentDistance(start,end,b,c),segmentDistance(start,end,c,a)});
}
bool clearOfMaterial(const damage::AttachedGeometryView& geometry,V3 start,V3 end,double margin) {
    const auto& frame=geometry.airframe();REQUIRE(frame.contactGeometry);
    const auto clear=[&](const damage::ContactMesh* mesh) {
        REQUIRE(mesh);
        for(unsigned axis=0;axis<3;++axis)
            if(std::min(coordinate(start,axis),coordinate(end,axis))>coordinate(mesh->high,axis)+margin||
               std::max(coordinate(start,axis),coordinate(end,axis))<coordinate(mesh->low,axis)-margin)return true;
        for(std::size_t i=0;i<mesh->triangles.size();++i) {
            if(!mesh->triangleNeighbors.empty()&&mesh->triangleNeighbors[i])continue;
            const auto& triangle=mesh->triangles[i];
            if(triangleClearance(start,end,mesh->vertices[triangle[0]],mesh->vertices[triangle[1]],mesh->vertices[triangle[2]])<=margin)return false;
        }
        return true;
    };
    for(const auto index:geometry.panelIndices()) {
        if(const auto* mesh=frame.contactGeometry->panel(index)) {if(!clear(mesh))return false;}
        else {
            // Authored lifting surfaces use the native analytic quad, not a
            // closed body mesh. Retain its actual two triangle boundaries.
            const auto& panel=frame.panels[index];REQUIRE(panel.surface);
            const auto& p=panel.corners;
            if(triangleClearance(start,end,p[0],p[1],p[2])<=margin||
               triangleClearance(start,end,p[0],p[2],p[3])<=margin)return false;
        }
    }
    for(const auto index:geometry.componentIndices())if(frame.components[index].kind!="fuel") {
        if(const auto* mesh=frame.contactGeometry->component(index))if(!clear(mesh))return false;
        // Also exclude the maintained damage probe, which can be a stricter
        // assessment boundary than the closed component mesh.
        const auto& component=frame.components[index];
        damage::fluid::Aabb box{component.centre-component.half-V3{margin,margin,margin},component.centre+component.half+V3{margin,margin,margin}};
        const auto clip=damage::fluid::clipImmersedLine(box,{{1,0,0},box.high.x},start,end);
        REQUIRE(clip.valid);if(clip.intersects)return false;
    }
    return true;
}
struct Probe {unsigned calls=0;bool captureOnly=false;};
class OnePoint final:public damage::WarheadAdapter,public damage::WarheadSceneExecutionV2 {
public:
    OnePoint(damage::Warhead model,std::string mechanism,std::shared_ptr<Probe> probe):model_(std::move(model)),mechanism_(std::move(mechanism)),probe_(std::move(probe)) {
        model_.modelVersion=2;model_.fragmentSource=damage::Warhead::FragmentSource::FiniteZonedCaseV2;
        model_.kind=damage::Warhead::Kind::Fragmentation;
        model_.caseKg=pointMassKg;model_.totalKg=pointMassKg;model_.explosiveKg=0;model_.tntKg=0;model_.fragments=1;
        model_.fragmentKg=pointMassKg;model_.fragmentAreaM2=pointAreaM2;model_.speedMs=relativeSpeedMs;
    }
    std::string_view implementationId() const noexcept override{return mechanism_;}
    const damage::Warhead& model() const noexcept override{return model_;}
    damage::Impact assess(const damage::Burst&,const damage::AttachedGeometryView&,const damage::Target&,Rng&) const override {
        FAIL("Diagnostic point fixture must use the production shared causal scene");return {};
    }
    damage::FragmentFieldV1 emitScene(const damage::Burst& burst,Rng&) const override {
        ++probe_->calls;damage::FragmentFieldV1 field;field.valid=true;field.burst=burst;field.caseKg=pointMassKg;field.applyBlast=false;
        if(probe_->captureOnly){field.caseKg=0;return field;}
        damage::FragmentFieldV1::Projectile point;point.sourceOrdinalV1=1;point.massKg=pointMassKg;point.areaM2=pointAreaM2;
        point.cd=pointAirCd;point.at=burst.at;point.velocity=burst.velocity;field.projectiles.push_back(point);return field;
    }
private:
    damage::Warhead model_;std::string mechanism_;std::shared_ptr<Probe> probe_;
};
components::ProviderRegistryV1 pointRegistry(std::shared_ptr<Probe> probe) {
    std::vector<std::string> errors;const auto path=std::filesystem::path(FSIM_TEST_STORE_DIR)/"aim-120c.json";
    const auto card=ms::readStoreCard(path,errors);REQUIRE(card);REQUIRE(errors.empty());
    components::ProfileV1 profile;std::string error;REQUIRE(components::readProfileV1(path,*card,profile,error));
    const auto* descriptor=profile.find("warhead");REQUIRE(descriptor);
    auto registry=components::builtInProvidersV1();const auto original=registry.warhead;
    REQUIRE(registry.warhead.registerFactory(descriptor->implementationId,std::string(equipment::mechanism(descriptor->implementationId)),
        [original,probe](const auto& row,const auto& context,auto& why)->std::unique_ptr<damage::WarheadAdapter> {
            const auto inner=original.create(row,context,why);if(!inner)return {};
            return std::make_unique<OnePoint>(inner->model(),std::string(inner->implementationId()),probe);
        },true,error));return registry;
}

Json binding(const fs::path& path) {
    ipc::WeaponDiagnosticFileDigestV1 digest;std::string error;
    REQUIRE(ipc::weaponDiagnosticFileDigestV1(path,digest,error));
    return Json::Object{{"path",fs::absolute(path).generic_string()},{"sha256",digest.sha256}};
}
Json runtime(const std::string& name) {
    Json result=Json::Object{{"kind","runtime"},{"design",name},{"schema","fsim.liquid-native-population.v1"}};
#ifdef _WIN32
    for(const auto& item:std::initializer_list<std::pair<const char*,const wchar_t*>>{{"binary",nullptr},{"jsbsim",L"libJSBSim.dll"}}) {
        const auto module=GetModuleHandleW(item.second);REQUIRE(module);wchar_t path[32768]{};
        const auto count=GetModuleFileNameW(module,path,32768);REQUIRE(count>0);REQUIRE(count<32768);
        result.set(item.first,binding(fs::path(path)));
    }
#else
    FAIL("Actual maintained native provenance requires the Windows presets");
#endif
    const auto folder=fs::path(FSIM_TEST_AIRCRAFT_DIR)/name;
    for(const auto& suffix:{".xml",".liquid-v1.json"})result.set(suffix,binding(folder/(name+suffix)));
    if(fs::is_regular_file(folder/(name+".damage.json")))result.set(".damage.json",binding(folder/(name+".damage.json")));
    return result;
}
struct Native {
    session::WorldOptions config;
    session::World world;
    std::shared_ptr<const damage::Airframe> frame;
    std::shared_ptr<const session::LiquidMaterialDatasetV1> data;
    std::uint32_t parent=0;
    static session::WorldOptions options() {
        static std::atomic<unsigned> sequence{0};session::WorldOptions out;
        out.name="liquid-population-"+std::to_string(sequence.fetch_add(1));out.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
        out.seed=17;out.dt=.0001;out.frameSkip=1;out.workers=1;out.pinWorkers=false;return out;
    }
    explicit Native(const std::string& name,bool captureOnly=false):config(options()),world(config) {
        auto probe=std::make_shared<Probe>();probe->captureOnly=captureOnly;
        std::string error;REQUIRE(world.setPayloadProvidersV1(pointRegistry(probe),error));
        const auto folder=fs::path(FSIM_TEST_AIRCRAFT_DIR)/name;
        if(fs::is_regular_file(folder/(name+".damage.json"))) {
            const auto f=damage::readAirframe(folder/(name+".damage.json"),error);INFO(error);REQUIRE(f);
            frame=std::make_shared<const damage::Airframe>(*f);
        }
        bool declared=false;data=session::readLiquidMaterialDatasetV1(folder,name,frame.get(),declared,error);
        INFO(error);REQUIRE(data);REQUIRE(error.empty());
        if(!data->tanks.empty()&&frame)REQUIRE(declared);
        session::VehicleSpec spec;spec.name="material receiver";spec.type="jsbsim:"+name;
        spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=0;spec.initial.enginesRunning=false;
        parent=world.createVehicle(spec);REQUIRE(parent);world.enableLiquidMaterialDiagnosticV1(true);world.step();
        REQUIRE_FALSE(world.liquidMaterialDiagnosticV1());
    }
    double duration()const{return world.dt()*static_cast<double>(world.frameSkip());}
    V3 cg() {double value[3]{};REQUIRE(world.cgFromOriginBodyM(parent,value));return vector(value);}
    std::vector<std::optional<double>> contents() {
        std::vector<std::optional<double>> result;
        for(const auto& tank:data->tanks) {
            const auto id=static_cast<std::size_t>(tank.index);if(result.size()<=id)result.resize(id+1);
            const auto property=world.model(parent)->property("propulsion/tank["+std::to_string(tank.index)+"]/contents-lbs");
            REQUIRE(property.valid());result[id]=property.get()*lbsToKg;REQUIRE(std::isfinite(*result[id]));REQUIRE(*result[id]>=0);
        }
        return result;
    }
    void fuel(double fraction) {
        for(const auto& tank:data->tanks) {
            auto property=world.model(parent)->property("propulsion/tank["+std::to_string(tank.index)+"]/contents-lbs");REQUIRE(property.valid());
            property.set(tank.capacityKg*fraction/lbsToKg);
        }
    }
    std::uint32_t release() {
        ms::WeaponLaunch launch;launch.type="jsbsim:aim120c";launch.offsetBodyM[1]=30;
        const auto source=world.launchWeapon(parent,launch);REQUIRE(source);world.step();return source;
    }
};
struct Choice {std::uint64_t cell=0;V3 start,direction;double margin=0;};
Choice select(const Native& f,int tank,double mass,const sim::VehicleState& state,bool wet) {
    const V3 normal=up(state);const double level=planeAtMass(*f.data,tank,mass,normal,{});
    Choice best;const double travel=relativeSpeedMs*f.duration()*2+gravityMs2*f.duration()*f.duration()*4;
    for(const auto& cell:f.data->cells)if(cell.nativeTank==tank) {
        const auto a=cell.bodyBounds.low,b=cell.bodyBounds.high,size=b-a;
        for(const double x:{.25,.5,.75})for(const double y:{.25,.5,.75})for(const double z:{.25,.5,.75}) {
            const V3 start{a.x+x*size.x,a.y+y*size.y,a.z+z*size.z};
            double clearance=std::min({x*size.x,(1-x)*size.x,y*size.y,(1-y)*size.y,z*size.z,(1-z)*size.z});
            const double signedHeight=damage::dot(normal,start)-level;
            if(mass>0)clearance=std::min(clearance,wet?-signedHeight:signedHeight);
            if(!(clearance>travel*2)||clearance<=best.margin)continue;
            const V3 direction=size.x>=size.y&&size.x>=size.z?V3{1,0,0}:size.y>=size.z?V3{0,1,0}:V3{0,0,1};
            if(f.frame&&!clearOfMaterial(damage::AttachedGeometryView(*f.frame),start,start+direction*travel,travel))continue;
            best={cell.stableCellId,start,direction,clearance};
        }
    }
    REQUIRE(best.cell!=0);REQUIRE(best.margin>travel*2);return best;
}
void checkFreeze(const Native& f,const session::LiquidMaterialDiagnosticV1& actual,
    const std::vector<std::optional<double>>& kg) {
    REQUIRE(actual.recipient.identity==f.parent);REQUIRE(actual.recipient.generation==f.world.info(f.parent)->generation);
    CHECK(actual.unsectioned==!bool(f.frame));REQUIRE(actual.recipient.tanks.size()==f.data->tanks.size());
    REQUIRE(actual.recipient.cells.size()==f.data->cells.size());
    std::set<std::uint64_t> ids;
    for(const auto& cell:actual.recipient.cells) {REQUIRE(ids.insert(cell.stableCellId).second);
        const auto source=std::find_if(f.data->cells.begin(),f.data->cells.end(),[&](const auto& c){return c.stableCellId==cell.stableCellId;});REQUIRE(source!=f.data->cells.end());
        CHECK(source->nativeTank==cell.nativeTank);close(source->bodyBounds.low,cell.bodyBounds.low,0);close(source->bodyBounds.high,cell.bodyBounds.high,0);
    }
    for(const auto& tank:actual.recipient.tanks) {
        const auto index=static_cast<std::size_t>(tank.nativeTank);REQUIRE(index<kg.size());REQUIRE(kg[index]);CHECK(std::abs(tank.actualContentsKg-*kg[index])<=1e-9);
        long double wet=0,full=0;double projectionError=0;
        for(const auto& cell:actual.recipient.cells)if(cell.nativeTank==tank.nativeTank) {
            const auto value=referenceVolume(cell.bodyBounds,tank.freeSurfaceBody.normal,tank.freeSurfaceBody.offset);
            wet+=value.volume;const auto size=cell.bodyBounds.high-cell.bodyBounds.low;full+=size.x*size.y*size.z;
            // Independent near-axis omission error <= area of a box projection
            // times omitted plane distance. No native clipping call is used.
            projectionError+=value.planeUncertaintyM*(size.x*size.y+size.x*size.z+size.y*size.z)*4;
        }
        const double target=tank.actualContentsKg/tank.densityKgM3;
        CHECK(std::abs(static_cast<double>(wet)-target)<=1e-9+1e-6*static_cast<double>(full)+projectionError);
    }
}
std::shared_ptr<const session::LiquidMaterialDiagnosticV1> burst(Native& f,std::uint32_t source,const Choice& selected,bool requirePoint=true) {
    const auto state=*f.world.vehicleState(f.parent);const auto point=ms::weapon::materialPoint(state,f.cg(),selected.start);REQUIRE(point.valid);
    session::Detonation d;d.store="aim-120c";d.weapon=source;d.stepFraction=1;
    const auto velocity=point.velocity+rotate(state,selected.direction*relativeSpeedMs),axis=rotate(state,selected.direction);
    for(unsigned i=0;i<3;++i){d.positionEcef[i]=coordinate(point.position,i);d.velocityEcef[i]=coordinate(velocity,i);d.axisEcef[i]=coordinate(axis,i);}
    const auto generation=f.world.info(source)->generation;f.world.detonateCapturedV1(d);
    const auto frozen=f.world.liquidMaterialDiagnosticV1();REQUIRE(frozen);REQUIRE_FALSE(frozen->assessed);
    REQUIRE(f.world.removeVehicle(source));f.world.step();const auto actual=f.world.liquidMaterialDiagnosticV1();REQUIRE(actual);
    REQUIRE(actual->assessed);CHECK(actual->source==source);CHECK(actual->sourceGeneration==generation);
    if(requirePoint){REQUIRE(actual->projectileCount==1);REQUIRE(actual->firstProjectile);CHECK(actual->firstProjectile->sourceOrdinalV1==1);}
    else {CHECK(actual->projectileCount==0);CHECK_FALSE(actual->firstProjectile);}
    CHECK(actual->fieldAgeS==f.duration());return actual;
}
V3 inertial(Native& f){const auto& state=*f.world.vehicleState(f.parent);return rotate(state,vector(state.velocityBodyMs));}
Json transit(const std::string& name,int tank,double fraction,bool wet) {
    Native active(name),control(name);const auto source=active.release(),controlSource=control.release();
    active.fuel(fraction);control.fuel(fraction);active.world.step();control.world.step();
    const auto kg=active.contents();const auto state=*active.world.vehicleState(active.parent);
    const auto choice=select(active,tank,*kg.at(static_cast<std::size_t>(tank)),state,wet);
    const auto point=ms::weapon::materialPoint(state,active.cg(),choice.start);REQUIRE(point.valid);
    const V3 incoming=point.velocity+rotate(state,choice.direction*relativeSpeedMs);
    const auto generation=active.world.info(source)->generation;const auto actual=burst(active,source,choice);
    REQUIRE(control.world.removeVehicle(controlSource));control.world.step();checkFreeze(active,*actual,kg);
    double energy=0;unsigned walls=0;
    for(const auto& row:actual->tankWork){CHECK(row.nativeTank==tank);CHECK(row.identity==active.parent);energy+=row.energyJ;walls+=row.wallContacts;}
    CHECK(walls==0);if(wet)CHECK(energy>0);else CHECK(energy==0);
    const auto& final=*active.world.vehicleState(active.parent);const auto origin=ms::weapon::materialPoint(final,active.cg(),{});REQUIRE(origin.valid);
    const V3 local=body(final,actual->firstProjectile->at-origin.position);
    const auto cell=std::find_if(active.data->cells.begin(),active.data->cells.end(),[&](const auto& c){return c.stableCellId==choice.cell;});REQUIRE(cell!=active.data->cells.end());
    for(unsigned axis=0;axis<3;++axis){CHECK(coordinate(local,axis)>coordinate(cell->bodyBounds.low,axis));CHECK(coordinate(local,axis)<coordinate(cell->bodyBounds.high,axis));}
    const V3 reaction=(inertial(active)-inertial(control))*control.world.model(control.parent)->energy().massKg;
    const V3 gravity=rotate(state,up(state))*(-gravityMs2);
    const V3 lost=(incoming-actual->firstProjectile->velocity+gravity*active.duration())*pointMassKg;
    const double error=norm(reaction-(wet?lost:V3{}));CHECK(error<=2e-5);
    CHECK(norm(actual->firstProjectile->at-point.position)>0);
    const auto material=ms::weapon::materialPoint(final,active.cg(),choice.start);REQUIRE(material.valid);
    const auto declared=std::find_if(active.data->tanks.begin(),active.data->tanks.end(),[&](const auto& r){return r.index==tank;});REQUIRE(declared!=active.data->tanks.end());
    const double k=declared->densityKgM3*active.data->dragCoefficient*pointAreaM2/(2*pointMassKg);
    const V3 expected=point.velocity+rotate(state,choice.direction*(relativeSpeedMs/(1+k*relativeSpeedMs*active.duration())));
    const double perturbation=gravityMs2*active.duration()+2*norm(material.velocity-point.velocity)+1e-4;
    if(wet)close(actual->firstProjectile->velocity,expected,perturbation);
    CHECK_FALSE(control.world.liquidMaterialDiagnosticV1()); // No cross-World observation.
    return Json::Object{{"kind","transit"},{"design",name},{"tank",tank},{"fraction",fraction},{"wet",wet},{"source",std::to_string(source)},
        {"source_generation",std::to_string(generation)},{"victim",std::to_string(active.parent)},
        {"victim_generation",std::to_string(actual->recipient.generation)},{"dt_s",active.duration()},{"cell",std::to_string(choice.cell)},
        {"actual_kg",*kg.at(static_cast<std::size_t>(tank))},{"capacity_kg",declared->capacityKg},{"density_kg_m3",declared->densityKgM3},
        {"captured_tanks",static_cast<double>(actual->recipient.tanks.size())},{"captured_cells",static_cast<double>(actual->recipient.cells.size())},
        {"energy_j",energy},{"wall_contacts",static_cast<double>(walls)},{"reaction_error_ns",error},{"reaction_budget_ns",2e-5},
        {"source_ordinal","1"},{"advanced",true},{"inside_selected_cell",true},{"freeze_oracle",true},{"unsectioned",actual->unsectioned}};
}
}
TEST_CASE("All maintained native liquid recipients conserve actual contents transit and reset", "[damage][liquid-population-native]") {
    const auto* selected=std::getenv("FSIM_LIQUID_NATIVE_DESIGN");REQUIRE(selected);const std::string name=selected;
    REQUIRE(std::find(designs.begin(),designs.end(),name)!=designs.end());
    const auto* output=std::getenv("FSIM_LIQUID_NATIVE_DIR");REQUIRE(output);fs::create_directories(output);
    std::ofstream log(fs::path(output)/(name+".jsonl"),std::ios::binary|std::ios::trunc);REQUIRE(log);
    const auto emit=[&](const Json& row){log<<row.dump()<<'\n'<<std::flush;REQUIRE(log);};emit(runtime(name));
    Native initial(name);
    if(initial.data->tanks.empty()) {
        CHECK(initial.data->cells.empty());const auto kg=initial.contents();CHECK(kg.empty());
        REQUIRE(initial.world.resetVehicle(initial.parent));CHECK_FALSE(initial.world.liquidMaterialDiagnosticV1());
        emit(Json::Object{{"kind","no-liquid"},{"design",name},{"loaded",true},{"reset",true},{"native_tanks",0.}});return;
    }
    for(const auto& tank:initial.data->tanks)for(const auto mode:std::initializer_list<std::pair<double,bool>>{{0,false},{.5,false},{.5,true},{1,true}})
        emit(transit(name,tank.index,mode.first,mode.second));
    // A physical reset retains World authority, clears pose-bound observations,
    // and restores native XML contents. It is compared with a fresh native body.
    Native restored(name,true),fresh(name,true);restored.fuel(.5);restored.world.step();
    const auto source=restored.release();const auto oldKg=restored.contents();
    const auto chosen=select(restored,restored.data->tanks.front().index,*oldKg.at(static_cast<std::size_t>(restored.data->tanks.front().index)),*restored.world.vehicleState(restored.parent),true);
    burst(restored,source,chosen,false);REQUIRE(restored.world.resetVehicle(restored.parent));CHECK_FALSE(restored.world.liquidMaterialDiagnosticV1());
    restored.world.enableLiquidMaterialDiagnosticV1(false);restored.world.enableLiquidMaterialDiagnosticV1(true);
    restored.world.step();const auto a=restored.contents(),b=fresh.contents();
    const auto resetCapture=restored.world.liquidMaterialDiagnosticV1();REQUIRE(resetCapture);checkFreeze(restored,*resetCapture,a);
    const auto freshSource=fresh.release();const auto freshKg=fresh.contents();
    const auto freshChoice=select(fresh,fresh.data->tanks.front().index,*freshKg.at(static_cast<std::size_t>(fresh.data->tanks.front().index)),*fresh.world.vehicleState(fresh.parent),true);
    const auto freshCapture=burst(fresh,freshSource,freshChoice,false);checkFreeze(fresh,*freshCapture,freshKg);
    REQUIRE(resetCapture->recipient.cells.size()==freshCapture->recipient.cells.size());
    for(const auto& tank:restored.data->tanks){const auto i=static_cast<std::size_t>(tank.index);REQUIRE(a.at(i));REQUIRE(b.at(i));CHECK(std::abs(*a[i]-*b[i])<=1e-9);
        const auto resetTank=std::find_if(resetCapture->recipient.tanks.begin(),resetCapture->recipient.tanks.end(),[&](const auto& t){return t.nativeTank==tank.index;});
        const auto freshTank=std::find_if(freshCapture->recipient.tanks.begin(),freshCapture->recipient.tanks.end(),[&](const auto& t){return t.nativeTank==tank.index;});
        REQUIRE(resetTank!=resetCapture->recipient.tanks.end());REQUIRE(freshTank!=freshCapture->recipient.tanks.end());
        CHECK(std::abs(resetTank->actualContentsKg-freshTank->actualContentsKg)<=1e-9);
        close(resetTank->freeSurfaceBody.normal,freshTank->freeSurfaceBody.normal,1e-10);
        CHECK(std::abs(resetTank->freeSurfaceBody.offset-freshTank->freeSurfaceBody.offset)<=1e-9);
        emit(Json::Object{{"kind","reset"},{"design",name},{"tank",tank.index},{"actual_kg",*a[i]},{"fresh_kg",*b[i]},
            {"kg_budget",1e-9},{"stale_capture_refused",true},{"fresh_freeze",true},{"captured_tanks",static_cast<double>(resetCapture->recipient.tanks.size())},
            {"captured_cells",static_cast<double>(resetCapture->recipient.cells.size())}});}
    // Removed receiver IDs cannot resolve a stale diagnostic snapshot.
    REQUIRE(restored.world.removeVehicle(restored.parent));CHECK_FALSE(restored.world.liquidMaterialDiagnosticV1());
}

namespace {
damage::FragmentFieldV1 field(bool rod=false) {
    damage::FragmentFieldV1 out;out.valid=true;out.caseKg=1;out.emittedCount=1;out.applyBlast=false;
    damage::FragmentFieldV1::Projectile p;p.sourceOrdinalV1=1;p.massKg=1;p.areaM2=.1;p.cd=1;
    p.at={-.2,0,0};p.velocity={2,0,0};p.segment=rod;
    if(rod){p.at.y=-.1;p.end={-.2,.1,0};p.endVelocity=p.velocity;p.breakupAgeS=2;}
    out.projectiles.push_back(p);return out;
}
damage::SceneLiquidRecipientV1 material(std::uint64_t id,double x) {
    damage::SceneLiquidRecipientV1 out;out.identity=id;out.generation=3;
    out.tanks.push_back({0,10,1,16,{{0,0,1},2}});
    out.cells.push_back({1,0,0,-1,{{x,-1,-1},{x+.4,1,1}}});return out;
}
damage::SceneEnvironmentV1 environment() {
    damage::SceneEnvironmentV1 out;out.airDensity=0;out.maxFlightTimeS=1;return out;
}
damage::Target target() {damage::Target out;out.bodyToEcef={V3{1,0,0},V3{0,1,0},V3{0,0,1}};return out;}
damage::Airframe absorber(double x) {
    damage::Airframe out;out.aircraft="independent-solid";damage::Airframe::Component c;
    c.kind="engine";c.shape="box";c.centre={x,0,0};c.half={.1,1,1};out.components.push_back(c);return out;
}
double work(const damage::SceneLiquidAssessmentV2& result) {double out=0;for(const auto& row:result.liquidOnlyWork)out+=row.energyJ;return out;}
}
TEST_CASE("Liquid-only receivers and real solids share one ordered residual field", "[liquid-only-methods]") {
    for(const bool rod:{false,true})for(const bool upstreamSolid:{false,true}) {
        const auto solid=absorber(upstreamSolid?-.6:.7);const auto geometry=damage::AttachedGeometryView(solid);
        auto source=field(rod);if(upstreamSolid){source.projectiles[0].at.x=-1;if(rod)source.projectiles[0].end.x=-1;}
        const std::vector<damage::SceneVictimV1> solids{{42,geometry,target(),3}};
        const std::vector<damage::SceneLiquidOnlyVictimV2> only{{17,3,target(),{}}};
        damage::SceneLiquidInputsV1 inputs;inputs.recipients.push_back(material(17,0));
        const auto result=damage::assessSceneLiquidsV2(source,solids,only,environment(),inputs);REQUIRE(result.scene.valid);
        const auto hit=std::find_if(result.scene.victims.begin(),result.scene.victims.end(),[](const auto& r){return r.identity==42;});REQUIRE(hit!=result.scene.victims.end());
        REQUIRE(hit->impact.components.size()==1);const double absorbed=hit->impact.components[0].energyJ;
        if(upstreamSolid){CHECK(work(result)==0);CHECK(std::abs(absorbed-2)<1e-9);}
        else {const double speed=2*std::exp(-.5*.4);CHECK(std::abs(absorbed-.5*speed*speed)<2e-5);CHECK(work(result)>0);}
        CHECK(std::abs(work(result)+absorbed-2)<2e-5);close(result.scene.transferredMomentumEcef,{2,0,0},2e-5);
        const auto liquid=std::find_if(result.scene.victims.begin(),result.scene.victims.end(),[](const auto& r){return r.identity==17;});REQUIRE(liquid!=result.scene.victims.end());
        CHECK(liquid->impact.components.empty());CHECK(liquid->impact.panels.empty());CHECK(result.scene.continuation.projectiles.empty());
    }
}
TEST_CASE("Private liquid-only V2 preserves the old wrapper and rejects ambiguous identities atomically", "[liquid-only-methods]") {
    const auto solid=absorber(.7);const std::vector<damage::SceneVictimV1> solids{{42,damage::AttachedGeometryView(solid),target(),3}};
    const auto source=field();damage::SceneLiquidInputsV1 empty;
    const auto old=damage::assessSceneLiquidsV1(source,solids,environment(),empty);
    const auto same=damage::assessSceneLiquidsV2(source,solids,{},environment(),empty);REQUIRE(old.valid);REQUIRE(same.scene.valid);
    close(old.transferredMomentumEcef,same.scene.transferredMomentumEcef,0);CHECK(old.absorbedMassKg==same.scene.absorbedMassKg);
    CHECK(old.victims[0].impact.components[0].energyJ==same.scene.victims[0].impact.components[0].energyJ);
    damage::SceneLiquidInputsV1 inputs;inputs.recipients.push_back(material(17,0));
    for(unsigned fault=0;fault<4;++fault) {
        auto actual=inputs;std::vector<damage::SceneLiquidOnlyVictimV2> only{{17,3,target(),{}}};
        if(fault==0)only[0].identity=42;
        if(fault==1)actual.recipients[0].generation=4;
        if(fault==2)actual.recipients.clear();
        if(fault==3)only.push_back({17,9,target(),{}});
        const auto failed=damage::assessSceneLiquidsV2(source,solids,only,environment(),actual);CHECK_FALSE(failed.scene.valid);CHECK(failed.liquidOnlyWork.empty());
        CHECK(source.projectiles.size()==1);CHECK(source.projectiles[0].velocity.x==2);
    }
}
TEST_CASE("Liquid-only native tank and cell keys remain recipient scoped", "[liquid-only-methods]") {
    damage::SceneLiquidInputsV1 inputs;inputs.recipients={material(17,0),material(18,.8)};
    std::vector<damage::SceneLiquidOnlyVictimV2> only{{17,3,target(),{}},{18,3,target(),{}}};
    const auto first=damage::assessSceneLiquidsV2(field(),{},only,environment(),inputs);REQUIRE(first.scene.valid);
    std::reverse(only.begin(),only.end());std::reverse(inputs.recipients.begin(),inputs.recipients.end());
    const auto reverse=damage::assessSceneLiquidsV2(field(),{},only,environment(),inputs);REQUIRE(reverse.scene.valid);
    REQUIRE(first.liquidOnlyWork.size()==2);REQUIRE(reverse.liquidOnlyWork.size()==2);
    for(std::size_t i=0;i<2;++i){CHECK(first.liquidOnlyWork[i].identity==17+i);CHECK(first.liquidOnlyWork[i].nativeTank==0);CHECK(first.liquidOnlyWork[i].energyJ>0);
        CHECK(first.liquidOnlyWork[i].energyJ==reverse.liquidOnlyWork[i].energyJ);CHECK(first.liquidOnlyWork[i].wallContacts==1);}
    close(first.scene.remainingMomentumEcef,reverse.scene.remainingMomentumEcef,0);
}
