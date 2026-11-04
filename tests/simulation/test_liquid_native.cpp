// Referee diagnostic material fixture, NOT an authored-warhead lethality test.
// Production World capture, native tank kg, certified occupancy, causal source
// identity and native impulses remain authoritative. No actor/Range API is used.
#include "simulation/runtime/world/World.h"
#include "simulation/runtime/world/WorldLiquidMaterials.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/weapon/MaterialPoint.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace fsim;
namespace damage=ms::damage;
namespace components=ms::components;
using damage::V3;
constexpr double lbsToKg=0.45359237;
constexpr double gravityMs2=9.80665;
constexpr double pointMassKg=0.01,pointAreaM2=0.0001,pointAirCd=1.3,relativeSpeedMs=5;
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
struct Chord {int tank=-1;std::uint64_t cell=0;V3 start,direction;double margin=0,level=0;};
struct PathBounds {V3 low,high;};
PathBounds referencePath(V3 direction,V3 carrierVelocity,double airDensity,double liquidK,double elapsed,bool wet) {
    REQUIRE(std::isfinite(airDensity));REQUIRE(airDensity>=0);
    REQUIRE(std::isfinite(liquidK));REQUIRE(liquidK>0);
    REQUIRE(std::isfinite(elapsed));REQUIRE(elapsed>0);
    PathBounds bounds;
    if(wet) {
        // Relative to the initially co-moving material point. Ambient air is
        // absent while immersed. Gravity/native motion retain the old margin.
        const V3 last=direction*(std::log1p(liquidK*relativeSpeedMs*elapsed)/liquidK);
        for(unsigned axis=0;axis<3;++axis) {
            coordinate(bounds.low,axis,std::min(0.,coordinate(last,axis)));
            coordinate(bounds.high,axis,std::max(0.,coordinate(last,axis)));
        }
        return bounds;
    }
    // Calm ambient air is stationary in ECEF. In the frozen body frame the
    // projectile's initial air-relative velocity includes the ACTUAL native
    // material-point velocity, not merely the diagnostic 5 m/s increment.
    const V3 initial=carrierVelocity+direction*relativeSpeedMs;
    const double k=airDensity*pointAirCd*pointAreaM2/(2*pointMassKg),rate=k*norm(initial);
    REQUIRE(std::isfinite(rate));
    const auto displacement=[&](double time) {
        const double flow=rate>0?std::log1p(rate*time)/rate:time;
        return initial*flow-carrierVelocity*time;
    };
    const auto include=[&](V3 point) {
        for(unsigned axis=0;axis<3;++axis) {
            coordinate(bounds.low,axis,std::min(coordinate(bounds.low,axis),coordinate(point,axis)));
            coordinate(bounds.high,axis,std::max(coordinate(bounds.high,axis),coordinate(point,axis)));
        }
    };
    include(displacement(elapsed));
    // Each coordinate derivative is initial_i/(1+rate*t)-carrier_i.
    // Besides 0/T there is at most one stationary point per coordinate.
    if(rate>0)for(unsigned axis=0;axis<3;++axis) {
        const double carrier=coordinate(carrierVelocity,axis);
        if(carrier==0)continue;
        const double time=(coordinate(initial,axis)/carrier-1)/rate;
        if(time>0&&time<elapsed)include(displacement(time));
    }
    return bounds;
}
Chord chooseChord(const session::LiquidMaterialDatasetV1& data,const damage::AttachedGeometryView& geometry,
    const std::vector<std::optional<double>>& contents,V3 normal,V3 shift,double duration,bool wet,
    const sim::VehicleState& state,V3 cg,double airDensity,int owner=-2) {
    // Fixed test material and fixed two-interval duration; selection depends
    // solely on source geometry/current kg, never on native damage outcomes.
    const double margin=.002+2*gravityMs2*duration*duration;
    std::map<int,double> levels;
    for(const auto& cell:data.cells) {
        if(owner!=-2&&cell.section!=owner)continue;
        if(std::find(geometry.componentIndices().begin(),geometry.componentIndices().end(),cell.componentIndex)==geometry.componentIndices().end())continue;
        REQUIRE(cell.nativeTank>=0);const auto id=static_cast<std::size_t>(cell.nativeTank);
        REQUIRE(id<contents.size());REQUIRE(contents[id]);
        auto found=levels.find(cell.nativeTank);
        if(found==levels.end())found=levels.emplace(cell.nativeTank,planeAtMass(data,cell.nativeTank,*contents[id],normal,shift)).first;
        const double level=found->second;
        const auto lo=cell.bodyBounds.low-shift,hi=cell.bodyBounds.high-shift,size=hi-lo;
        REQUIRE(referenceVolume({lo,hi},normal,level).planeUncertaintyM*1000<margin);
        const auto tank=std::find_if(data.tanks.begin(),data.tanks.end(),[&](const auto& row){return row.index==cell.nativeTank;});
        REQUIRE(tank!=data.tanks.end());
        const double liquidK=tank->densityKgM3*data.dragCoefficient*pointAreaM2/(2*pointMassKg);
        const V3 centre=(lo+hi)*.5;
        for(unsigned axis=0;axis<3;++axis) {
            V3 direction{};coordinate(direction,axis,1);
            V3 anchor=centre;
            for(unsigned k=0;k<3;++k)if(k!=axis)coordinate(anchor,k,coordinate(anchor,k)+
                (wet?-1:1)*(coordinate(normal,k)>=0?1:-1)*coordinate(size,k)*.25);
            // Center the complete physical span, including any dry turning
            // point. Rotation makes initial material velocity depend weakly
            // on the seed; re-center using captured rigid kinematics only.
            // The final independent bounds are always recomputed at the
            // actual seed, so iteration completion never grants containment.
            V3 start=anchor;
            for(unsigned iteration=0;iteration<8;++iteration) {
                const auto material=ms::weapon::materialPoint(state,cg,start);REQUIRE(material.valid);
                const auto path=referencePath(direction,body(state,material.velocity),airDensity,liquidK,2*duration,wet);
                const V3 next=anchor-(path.low+path.high)*.5;
                if(next.x==start.x&&next.y==start.y&&next.z==start.z)break;
                start=next;
            }
            const auto material=ms::weapon::materialPoint(state,cg,start);REQUIRE(material.valid);
            const auto path=referencePath(direction,body(state,material.velocity),airDensity,liquidK,2*duration,wet);
            const V3 pathLow=start+path.low,pathHigh=start+path.high;
            bool inside=true;
            for(unsigned k=0;k<3;++k)inside=inside&&coordinate(pathLow,k)-coordinate(lo,k)>margin&&
                coordinate(hi,k)-coordinate(pathHigh,k)>margin;
            if(!inside)continue;
            const V3 planeLow{normal.x>=0?pathLow.x:pathHigh.x,normal.y>=0?pathLow.y:pathHigh.y,normal.z>=0?pathLow.z:pathHigh.z};
            const V3 planeHigh{normal.x>=0?pathHigh.x:pathLow.x,normal.y>=0?pathHigh.y:pathLow.y,normal.z>=0?pathHigh.z:pathLow.z};
            const double near=damage::dot(normal,planeLow),far=damage::dot(normal,planeHigh);
            if(wet?!(far<level-margin):!(near>level+margin))continue;
            // Enclose the whole curved path in a primary line plus a derived
            // transverse radius. This radius is geometry, not a loosened
            // physical-motion tolerance; dry endpoints alone are insufficient.
            const V3 span=pathHigh-pathLow,middle=(pathLow+pathHigh)*.5;
            unsigned primary=0;for(unsigned k=1;k<3;++k)if(coordinate(span,k)>coordinate(span,primary))primary=k;
            V3 lineStart=middle,lineEnd=middle,transverse=span*.5;
            coordinate(lineStart,primary,coordinate(pathLow,primary));coordinate(lineEnd,primary,coordinate(pathHigh,primary));
            coordinate(transverse,primary,0);
            if(!clearOfMaterial(geometry,lineStart,lineEnd,margin+norm(transverse)))continue;
            std::printf("SIM_M03_LIQUID_NATIVE_PATH cell=%llu wet=%u air_density_kg_m3=%.17g low_m=%.17g,%.17g,%.17g high_m=%.17g,%.17g,%.17g transverse_radius_m=%.17g\n",
                static_cast<unsigned long long>(cell.stableCellId),wet?1u:0u,airDensity,pathLow.x,pathLow.y,pathLow.z,
                pathHigh.x,pathHigh.y,pathHigh.z,norm(transverse));
            return {cell.nativeTank,cell.stableCellId,start,direction,margin,level};
        }
    }
    FAIL("Required certified clear wet/dry two-interval chord is absent; do not substitute vulnerability probes or tune an outcome");
    return {};
}

struct Probe {unsigned calls=0;};
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
session::WorldOptions options(int skip,unsigned workers) {
    static std::atomic<unsigned> sequence{0};session::WorldOptions out;
    out.name="liquid-native-"+std::to_string(sequence.fetch_add(1));out.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    out.seed=17;out.frameSkip=skip;out.workers=workers;out.pinWorkers=false;out.publish=true;out.publishIntervalSeconds=0;return out;
}
struct Native {
    std::shared_ptr<Probe> probe=std::make_shared<Probe>();session::WorldOptions config;
    session::World world;std::shared_ptr<const damage::Airframe> frame;
    std::shared_ptr<const session::LiquidMaterialDatasetV1> materials;
    std::uint32_t parent=0;ipc::WeaponDiagnosticsV1 diagnostic;
    Native(int skip,unsigned workers):config(options(skip,workers)),world(config) {
        std::string error;REQUIRE(world.setPayloadProvidersV1(pointRegistry(probe),error));
        const auto directory=std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"f16c";
        REQUIRE(std::filesystem::is_regular_file(directory/"f16c.liquid-v1.json"));
        const auto source=damage::readAirframe(directory/"f16c.damage.json",error);INFO(error);REQUIRE(source);
        frame=std::make_shared<const damage::Airframe>(*source);REQUIRE(frame->contactGeometry);
        bool declared=false;materials=session::readLiquidMaterialDatasetV1(directory,"f16c",frame.get(),declared,error);
        INFO(error);REQUIRE(declared);REQUIRE(materials);REQUIRE_FALSE(materials->cells.empty());
        session::VehicleSpec spec;spec.name="native liquid recipient";spec.type="jsbsim:f16c";
        spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=220;spec.initial.headingDeg=90;
        parent=world.createVehicle(spec);REQUIRE(parent);world.step();
    }
    double duration()const{return world.dt()*static_cast<double>(world.frameSkip());}
    void fuel(double fraction) {
        for(const auto& tank:materials->tanks) {
            auto property=world.model(parent)->property("propulsion/tank["+std::to_string(tank.index)+"]/contents-lbs");
            REQUIRE(property.valid());property.set(tank.capacityKg*fraction/lbsToKg);
        }
    }
    std::vector<std::optional<double>> contents(std::uint32_t recipient) {
        std::vector<std::optional<double>> result;
        for(const auto& tank:materials->tanks) {
            const auto index=static_cast<std::size_t>(tank.index);if(result.size()<=index)result.resize(index+1);
            if(world.info(recipient)->role==ms::Role::Debris) {
                REQUIRE(world.damage(recipient));result[index]=world.damage(recipient)->detachedFuelKg(tank.index);
            } else {
                auto property=world.model(recipient)->property("propulsion/tank["+std::to_string(tank.index)+"]/contents-lbs");
                REQUIRE(property.valid());result[index]=property.get()*lbsToKg;
            }
        }
        return result;
    }
    std::uint32_t release() {
        ms::WeaponLaunch launch;launch.type="jsbsim:aim120c";launch.offsetBodyM[1]=30;
        const auto id=world.launchWeapon(parent,launch);REQUIRE(id);world.step();return id;
    }
    std::unique_ptr<ipc::WeaponDiagnosticFrameV1> snapshot(std::uint32_t source,std::uint64_t generation) {
        world.publishNow();std::uint64_t pid=0,frameNumber=0;std::uint32_t incarnation=0;
        REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(config.name,pid,incarnation,frameNumber));
        if(!diagnostic.valid())REQUIRE(diagnostic.open(config.name,pid,incarnation));
        REQUIRE(diagnostic.requestSelection(source,generation,frameNumber));world.publishNow();
        auto result=std::make_unique<ipc::WeaponDiagnosticFrameV1>();REQUIRE(diagnostic.readLatest(*result));
        REQUIRE(result->entity==source);REQUIRE(result->generation==generation);return result;
    }
    V3 cg(std::uint32_t recipient) {double value[3]{};REQUIRE(world.cgFromOriginBodyM(recipient,value));return vector(value);}
};
double nativeAirDensity(Native& fixture,std::uint32_t source) {
    // World freezes the released source's actual atmosphere for its field.
    // Read that same native physical input; this is not a fitted air density.
    const auto property=fixture.world.model(source)->property("atmosphere/rho-slugs_ft3");REQUIRE(property.valid());
    const double density=property.get()*515.378818;REQUIRE(std::isfinite(density));REQUIRE(density>=0);return density;
}
struct Observation {V3 birthVelocity,velocity,position,mediumVelocity,reaction;double energy=0;unsigned hits=0;};
Observation observe(const ipc::WeaponDiagnosticFrameV1& frame,std::uint32_t recipient,std::uint64_t generation) {
    const auto& effects=frame.effects;REQUIRE(effects.stamp.status==ipc::WeaponDiagnosticStatusV1::Available);
    REQUIRE(effects.fragmentCount==1);REQUIRE(effects.totalFragmentCount==1);REQUIRE(effects.fragments[0].id==1);
    REQUIRE(effects.fragments[0].massKg==pointMassKg);REQUIRE(effects.fragments[0].areaM2==pointAreaM2);
    Observation result;result.position=vector(effects.fragments[0].positionEcefM);result.velocity=vector(effects.fragments[0].velocityEcefMs);
    for(std::uint32_t i=0;i<effects.damageCount;++i) {
        const auto& row=effects.damage[i];REQUIRE(row.victim==recipient);REQUIRE(row.victimGeneration==generation);
        result.energy+=row.penetratedEnergyJ;result.hits+=row.hitCount;
    }
    return result;
}
session::Detonation initiate(Native& fixture,std::uint32_t recipient,std::uint32_t source,const Chord& chord,bool remote=false) {
    const auto& state=*fixture.world.vehicleState(recipient);
    const auto point=ms::weapon::materialPoint(state,fixture.cg(recipient),chord.start);REQUIRE(point.valid);
    session::Detonation burst;burst.store="aim-120c";burst.weapon=source;burst.stepFraction=1;
    const V3 at=point.position+(remote?rotate(state,{0,0,-1000}):V3{});
    const V3 velocity=point.velocity+rotate(state,chord.direction*relativeSpeedMs);
    const V3 axis=rotate(state,chord.direction);
    for(unsigned i=0;i<3;++i){burst.positionEcef[i]=coordinate(at,i);burst.velocityEcef[i]=coordinate(velocity,i);burst.axisEcef[i]=coordinate(axis,i);}
    const unsigned before=fixture.probe->calls;
    const auto immediate=fixture.world.detonateCapturedV1(burst);REQUIRE(fixture.probe->calls==before+1);
    CHECK(std::none_of(immediate.begin(),immediate.end(),[](const auto& event){return event.kind==static_cast<std::uint8_t>(ipc::EngagementKind::Hit);}));
    return burst;
}
V3 inertialVelocity(Native& fixture,std::uint32_t id) {
    const auto& state=*fixture.world.vehicleState(id);return rotate(state,vector(state.velocityBodyMs));
}
Observation runIntervalPair(Native& active,Native& reference,std::uint32_t recipient,std::uint32_t controlRecipient,
    std::uint32_t source,std::uint32_t controlSource,const Chord& chord,bool wet) {
    const auto generation=active.world.info(source)->generation,recipientGeneration=active.world.info(recipient)->generation;
    const auto burst=initiate(active,recipient,source,chord);initiate(reference,controlRecipient,controlSource,chord,true);
    const auto initial=active.snapshot(source,generation);const auto start=observe(*initial,recipient,recipientGeneration);
    CHECK(start.energy==0);CHECK(start.hits==0);CHECK(initial->effects.fragments[0].sampleTimeS==active.world.simTime());
    const auto state=*active.world.vehicleState(recipient);const V3 normal=up(state);
    const V3 gravity=rotate(state,normal)*(-gravityMs2);const V3 incoming=vector(burst.velocityEcef);
    const auto material=ms::weapon::materialPoint(state,active.cg(recipient),chord.start);REQUIRE(material.valid);
    bool recordedRelease=false;
    for(const auto& row:active.world.weapons())if(row.id==source) {
        CHECK(row.launcher==active.parent);CHECK(row.type=="jsbsim:aim120c");
        CHECK(row.store.empty()); // Existing platform launch has no consumed inventory/store identity.
        recordedRelease=true;
    }
    REQUIRE(recordedRelease);
    // Remove the two real releases after capturing their emissions. Retained
    // fields must retain the original ID/generation without a live Entry, and
    // a later autonomous fuze cannot add a second diagnostic source population.
    REQUIRE(active.world.removeVehicle(source));REQUIRE(reference.world.removeVehicle(controlSource));
    const double time=active.duration();active.world.step();reference.world.step();
    const auto firstFrame=active.snapshot(source,generation);auto first=observe(*firstFrame,recipient,recipientGeneration);
    // The source is strictly inside the preselected capacity cell. Real fluid
    // work is observable immediately; no bladder wall was crossed here.
    CHECK(first.hits==0);
    CHECK(firstFrame->effects.totalDamageCount>=firstFrame->effects.damageCount);
    if(wet)CHECK(firstFrame->effects.damageCount>0);
    else CHECK(firstFrame->effects.totalDamageCount==0);
    if(const auto* damaged=active.world.damage(recipient))
        for(const auto& tank:damaged->tanks())CHECK(tank.second.holeM2==0);
    CHECK(firstFrame->effects.fragments[0].sampleTimeS==active.world.simTime());
    CHECK(firstFrame->effects.stamp.sourceId==source);CHECK(firstFrame->effects.burstTimeS==initial->effects.burstTimeS);
    const double mass=reference.world.model(controlRecipient)->energy().massKg;REQUIRE(mass>0);
    const V3 nativeReaction=(inertialVelocity(active,recipient)-inertialVelocity(reference,controlRecipient))*mass;
    const V3 lostMomentum=(incoming-first.velocity+gravity*time)*pointMassKg;
    std::printf("SIM_M03_LIQUID_NATIVE_INTERVAL source=%u source_generation=%llu victim=%u victim_generation=%llu dt_s=%.17g wet=%u energy_j=%.17g hits=%u reaction_error_ns=%.17g\n",
        source,static_cast<unsigned long long>(generation),recipient,static_cast<unsigned long long>(recipientGeneration),
        time,wet?1u:0u,first.energy,first.hits,norm(nativeReaction-lostMomentum));
    // The first interval's FDM trajectory is identical in the paired World;
    // impulse application occurs after that batch. In a wet chord ambient air
    // must not be charged again. Published double roundoff and finite liquid
    // integration tolerances bound this impulse comparison, not a kill score.
    if(wet)close(nativeReaction,lostMomentum,2e-5);
    else {CHECK(first.energy==0);close(nativeReaction,{},2e-5);}
    CHECK((wet?first.energy>0:first.energy==0));
    // Constant-medium analytic reference with explicit gravity/recipient
    // acceleration perturbation bound. This is weaker than the independent
    // exact primitive tests; it checks the actual production capture path.
    const auto& after=*active.world.vehicleState(recipient);
    const auto endMaterial=ms::weapon::materialPoint(after,active.cg(recipient),chord.start);REQUIRE(endMaterial.valid);
    const auto tank=std::find_if(active.materials->tanks.begin(),active.materials->tanks.end(),[&](const auto& row){return row.index==chord.tank;});
    REQUIRE(tank!=active.materials->tanks.end());
    const double k=tank->densityKgM3*active.materials->dragCoefficient*pointAreaM2/(2*pointMassKg);
    const V3 expected=material.velocity+rotate(state,chord.direction*(relativeSpeedMs/(1+k*relativeSpeedMs*time)));
    const double perturbation=gravityMs2*time+2*norm(endMaterial.velocity-material.velocity)+1e-4;
    if(wet)close(first.velocity,expected,perturbation);
    // A second interval contributes real energy with no second material hit.
    active.world.step();reference.world.step();const auto laterFrame=active.snapshot(source,generation);
    auto later=observe(*laterFrame,recipient,recipientGeneration);CHECK(later.hits==first.hits);
    if(wet)CHECK(later.energy>first.energy);else CHECK(later.energy==0);
    const auto& finalState=*active.world.vehicleState(recipient);
    const auto origin=ms::weapon::materialPoint(finalState,active.cg(recipient),{});REQUIRE(origin.valid);
    const V3 bodyPosition=body(finalState,later.position-origin.position);
    const auto cell=std::find_if(active.materials->cells.begin(),active.materials->cells.end(),[&](const auto& row){return row.stableCellId==chord.cell;});
    REQUIRE(cell!=active.materials->cells.end());
    // Departure fixtures verify their translated geometry separately; attached
    // coordinates use the original fixed material origin directly.
    if(active.world.info(recipient)->role==ms::Role::Aircraft) {
        for(unsigned axis=0;axis<3;++axis) {
            CHECK(coordinate(bodyPosition,axis)>coordinate(cell->bodyBounds.low,axis));
            CHECK(coordinate(bodyPosition,axis)<coordinate(cell->bodyBounds.high,axis));
        }
    }
    later.birthVelocity=incoming;later.reaction=nativeReaction;return later;
}
}

TEST_CASE("Real native F16 fuel captures use empty partial and full certified occupancy", "[damage][world][liquid-native]") {
    for(const int skip:{1,4})for(const unsigned workers:{1u,2u})for(const double fraction:{0.,.5,1.}) {
        INFO("skip="<<skip<<" workers="<<workers<<" fraction="<<fraction);
        for(const bool wet:{false,true}) {
            if((fraction==0&&wet)||(fraction==1&&!wet))continue;
            Native active(skip,workers),reference(skip,workers);active.fuel(fraction);reference.fuel(fraction);
            const auto source=active.release(),controlSource=reference.release();
            const auto state=*active.world.vehicleState(active.parent);const auto kg=active.contents(active.parent);
            const auto chord=chooseChord(*active.materials,damage::AttachedGeometryView(*active.frame),kg,up(state),{},active.duration(),wet,
                state,active.cg(active.parent),nativeAirDensity(active,source));
            INFO("preselected cell="<<chord.cell<<" tank="<<chord.tank<<" wet="<<wet);
            std::printf("SIM_M03_LIQUID_NATIVE_PRESELECT skip=%d workers=%u fraction=%.17g wet=%u source=%u cell=%llu tank=%d actual_kg=%.17g start_m=%.17g,%.17g,%.17g level_m=%.17g clearance_m=%.17g\n",
                skip,workers,fraction,wet?1u:0u,source,static_cast<unsigned long long>(chord.cell),chord.tank,
                *kg[static_cast<std::size_t>(chord.tank)],chord.start.x,chord.start.y,chord.start.z,chord.level,chord.margin);
            const auto result=runIntervalPair(active,reference,active.parent,reference.parent,source,controlSource,chord,wet);
            CHECK(result.hits==0);CHECK(std::isfinite(result.energy));
        }
    }
}

TEST_CASE("Real left wing departure transfers actual kg and only the child receives later liquid work", "[damage][world][liquid-native][detached-effects]") {
    Native active(1,1),reference(1,1);active.fuel(.5);reference.fuel(.5);active.world.step();reference.world.step();
    const auto before=active.contents(active.parent);const int owner=active.frame->section("left wing");REQUIRE(owner>=0);
    REQUIRE(active.world.separateSection(active.parent,"left wing"));REQUIRE(reference.world.separateSection(reference.parent,"left wing"));
    const auto findPiece=[](Native& f) {
        for(const auto id:f.world.vehicleIds())if(f.world.info(id)->role==ms::Role::Debris)return id;
        FAIL("Native separation did not create the actual piece model");return std::uint32_t{0};
    };
    const auto piece=findPiece(active),controlPiece=findPiece(reference);REQUIRE(active.world.damage(piece));
    for(const auto& tank:active.frame->sections[static_cast<std::size_t>(owner)].tanks) {
        const auto id=static_cast<std::size_t>(tank.index);REQUIRE(id<before.size());REQUIRE(before[id]);
        const auto actual=active.world.damage(piece)->detachedFuelKg(tank.index);REQUIRE(actual);
        CHECK(std::abs(*actual-*before[id])<=1e-9);
        const auto remaining=active.contents(active.parent);REQUIRE(remaining[id]);CHECK(*remaining[id]==0);
    }
    const auto source=active.release(),controlSource=reference.release();
    const auto geometry=active.world.damage(piece)->airframe(); // Own the copy across the later physical reset.
    const V3 shift=active.frame->sections[static_cast<std::size_t>(owner)].cg;
    const auto chord=chooseChord(*active.materials,active.world.damage(piece)->attachedGeometry(),active.contents(piece),
        up(*active.world.vehicleState(piece)),shift,active.duration(),true,*active.world.vehicleState(piece),active.cg(piece),
        nativeAirDensity(active,source),owner);
    const auto cell=std::find_if(active.materials->cells.begin(),active.materials->cells.end(),[&](const auto& row){return row.stableCellId==chord.cell;});REQUIRE(cell!=active.materials->cells.end());
    close(geometry.components[cell->componentIndex].centre,active.frame->components[cell->componentIndex].centre-shift,1e-12);
    runIntervalPair(active,reference,piece,controlPiece,source,controlSource,chord,true);
    // The observer rejects any component energy attributed to the parent in
    // runIntervalPair; original victim generation and source ordinal remain.
    const unsigned expiry=static_cast<unsigned>(std::ceil(1/active.duration()))+1;
    active.world.step(expiry);reference.world.step(expiry); // Existing independent fields expire before reset.
    REQUIRE(active.world.resetVehicle(piece));REQUIRE(reference.world.resetVehicle(controlPiece));
    CHECK(active.world.damage(piece)==nullptr);CHECK(reference.world.damage(controlPiece)==nullptr);
    for(const auto pair:{std::pair<Native*,std::uint32_t>{&active,piece},{&reference,controlPiece}}) {
        const auto fuel=pair.first->world.model(pair.second)->property("inertia/pointmass-weight-lbs");
        REQUIRE(fuel.valid());CHECK(fuel.get()==0); // The generated native piece is loaded dry.
    }
    const auto resetSource=active.release(),resetControlSource=reference.release();
    std::vector<std::optional<double>> dryContents=before;
    for(const auto& tank:active.frame->sections[static_cast<std::size_t>(owner)].tanks)
        dryContents[static_cast<std::size_t>(tank.index)]=0;
    damage::VehicleDamage dryGeometry(std::make_shared<const damage::Airframe>(geometry),owner);
    const auto dry=chooseChord(*active.materials,dryGeometry.attachedGeometry(),dryContents,
        up(*active.world.vehicleState(piece)),shift,active.duration(),false,*active.world.vehicleState(piece),active.cg(piece),
        nativeAirDensity(active,resetSource),owner);
    runIntervalPair(active,reference,piece,controlPiece,resetSource,resetControlSource,dry,false);
    REQUIRE(active.world.damage(piece));
    for(const auto& tank:active.frame->sections[static_cast<std::size_t>(owner)].tanks) {
        const auto actual=active.world.damage(piece)->detachedFuelKg(tank.index);REQUIRE(actual);CHECK(*actual==0);
    }
}

TEST_CASE("Physical parent reset restores the same liquid capture as a fresh native World", "[damage][world][liquid-native][reset]") {
    Native restored(1,1),control(1,1);restored.fuel(.5);control.fuel(.5);
    const auto oldSource=restored.release(),oldControlSource=control.release();
    const auto warm=chooseChord(*restored.materials,damage::AttachedGeometryView(*restored.frame),restored.contents(restored.parent),
        up(*restored.world.vehicleState(restored.parent)),{},restored.duration(),true,*restored.world.vehicleState(restored.parent),
        restored.cg(restored.parent),nativeAirDensity(restored,oldSource));
    runIntervalPair(restored,control,restored.parent,control.parent,oldSource,oldControlSource,warm,true);
    const unsigned expiry=static_cast<unsigned>(std::ceil(1/restored.duration()))+1;
    restored.world.step(expiry);control.world.step(expiry); // Physical reset does not cancel another source's field.
    REQUIRE(restored.world.resetVehicle(restored.parent));REQUIRE(control.world.resetVehicle(control.parent));
    CHECK(restored.world.damage(restored.parent)==nullptr);CHECK(control.world.damage(control.parent)==nullptr);
    restored.world.step();control.world.step(); // Same first post-IC native interval as Native's constructor.
    Native fresh(1,1),freshControl(1,1);
    const auto actualKg=restored.contents(restored.parent),freshKg=fresh.contents(fresh.parent);
    REQUIRE(actualKg.size()==freshKg.size());
    for(const auto& tank:restored.materials->tanks) {
        const auto i=static_cast<std::size_t>(tank.index);REQUIRE(actualKg[i]);REQUIRE(freshKg[i]);
        CHECK(std::abs(*actualKg[i]-*freshKg[i])<=1e-9);
    }
    const auto source=restored.release(),controlSource=control.release();
    const auto freshSource=fresh.release(),freshControlSource=freshControl.release();
    const auto restoredChord=chooseChord(*restored.materials,damage::AttachedGeometryView(*restored.frame),restored.contents(restored.parent),
        up(*restored.world.vehicleState(restored.parent)),{},restored.duration(),true,*restored.world.vehicleState(restored.parent),
        restored.cg(restored.parent),nativeAirDensity(restored,source));
    const auto freshChord=chooseChord(*fresh.materials,damage::AttachedGeometryView(*fresh.frame),fresh.contents(fresh.parent),
        up(*fresh.world.vehicleState(fresh.parent)),{},fresh.duration(),true,*fresh.world.vehicleState(fresh.parent),
        fresh.cg(fresh.parent),nativeAirDensity(fresh,freshSource));
    CHECK(restoredChord.cell==freshChord.cell);CHECK(restoredChord.tank==freshChord.tank);
    close(restoredChord.start,freshChord.start,1e-9);
    const auto actual=runIntervalPair(restored,control,restored.parent,control.parent,source,controlSource,restoredChord,true);
    const auto expected=runIntervalPair(fresh,freshControl,fresh.parent,freshControl.parent,freshSource,freshControlSource,freshChord,true);
    CHECK(actual.hits==expected.hits);CHECK(std::abs(actual.energy-expected.energy)<=1e-10);
    close(actual.velocity,expected.velocity,1e-8);
}
