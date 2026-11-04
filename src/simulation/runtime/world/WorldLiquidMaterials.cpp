#include "WorldLiquidMaterials.h"
#include "World.h"
#include "foundation/core/Json.h"
#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fsim::session {
namespace {
using namespace ms::damage;
constexpr std::size_t maxCells=65536,maxTanks=65536;
bool finite(V3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
void need(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
core::Json document(const std::filesystem::path& path) {
    need(std::filesystem::is_regular_file(path),"Required liquid material document is absent");
    need(std::filesystem::file_size(path)<=64*1024*1024,"Liquid material document exceeds its bounded size");
    std::ifstream stream(path,std::ios::binary);need(bool(stream),"Liquid material document cannot be opened");
    std::stringstream text;text<<stream.rdbuf();need(!stream.bad(),"Liquid material document read failed");
    return core::Json::parse(text.str(),path.string());
}
V3 vector(const core::Json& row) {
    const auto& values=row.asArray();need(values.size()==3,"Liquid vector requires exactly three values");
    V3 out{values[0].asNumber(),values[1].asNumber(),values[2].asNumber()};
    need(finite(out),"Liquid vector is nonfinite");return out;
}
std::size_t integer(const core::Json& row,std::size_t upper) {
    const double value=row.asNumber();
    need(std::isfinite(value)&&value>=0&&std::floor(value)==value&&value<static_cast<double>(upper),"Invalid liquid integer identity");
    return static_cast<std::size_t>(value);
}
double positive(const core::Json& row) {
    const double value=row.asNumber();need(std::isfinite(value)&&value>0,"Liquid physical value must be finite and positive");return value;
}
double volume(fluid::Aabb box) {
    const V3 size=box.high-box.low;need(finite(size)&&size.x>0&&size.y>0&&size.z>0,"Liquid cell has invalid dimensions");
    const double out=size.x*size.y*size.z;need(std::isfinite(out)&&out>0,"Liquid cell has invalid volume");return out;
}
bool overlap(fluid::Aabb a,fluid::Aabb b) {
    return std::max(a.low.x,b.low.x)<std::min(a.high.x,b.high.x)&&
        std::max(a.low.y,b.low.y)<std::min(a.high.y,b.high.y)&&
        std::max(a.low.z,b.low.z)<std::min(a.high.z,b.high.z);
}
int section(const Airframe* frame,const std::string& name) {
    if(!frame){need(name=="airframe","Unsectioned material has an unknown owner");return -1;}
    const int result=frame->section(name);
    need(result>=0||name=="airframe","Liquid owner does not exist in the source airframe");return result;
}
std::size_t componentFor(const Airframe* frame,int tank,int owner,V3 point) {
    if(!frame)return std::numeric_limits<std::size_t>::max();
    std::size_t selected=std::numeric_limits<std::size_t>::max();double distance=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<frame->components.size();++i) {
        const auto& c=frame->components[i];if(c.kind!="fuel"||c.tank!=tank||c.section!=owner)continue;
        const double next=length(c.centre-point);if(next<distance){selected=i;distance=next;}
    }
    need(selected!=std::numeric_limits<std::size_t>::max(),"Native liquid tank has no matching owned fuel component");return selected;
}
double projected(V3 p,V3 n){return dot(p,n);}
double lower(fluid::Aabb box,V3 n) {
    return projected({n.x>=0?box.low.x:box.high.x,n.y>=0?box.low.y:box.high.y,n.z>=0?box.low.z:box.high.z},n);
}
double upper(fluid::Aabb box,V3 n) {
    return projected({n.x>=0?box.high.x:box.low.x,n.y>=0?box.high.y:box.low.y,n.z>=0?box.high.z:box.low.z},n);
}
void vesselKeys(const core::Json& row,std::initializer_list<const char*> names) {
    const auto& fields=row.asObject();need(fields.size()==names.size(),"Inferred vessel has unknown/missing declaration fields");
    std::set<std::string> found;
    for(const auto& field:fields) {
        need(found.insert(field.first).second,"Inferred vessel has duplicate declaration fields");
        need(std::any_of(names.begin(),names.end(),[&](const char* name){return field.first==name;}),
            "Inferred vessel has unknown declaration field");
    }
}
void validateVesselPolicy(const core::Json& doc,const std::string& design) {
    const auto& cells=doc.child("cells").asArray();
    if(!doc.has("inferred_vessels_v1")) {
        need(!doc.child("ledger").boolean("per_tank_vessel_eligibility_certified",false),
            "Undeclared inferred vessel ledger policy");
        for(const auto& cell:cells)need(!cell.has("eligibility_v1"),"Undeclared inferred vessel cell policy");
        return;
    }
    need(design=="rq4b"&&!doc.child("tanks").asArray().empty(),"Inferred vessel policy is not supported for this model");
    const auto& receipt=doc.child("inferred_vessels_v1");const auto& policy=receipt.child("declaration");
    vesselKeys(policy,{"schema","design","basis","geometry","attachment","tanks"});
    need(policy.child("schema").asString()=="fsim.inferred-liquid-vessels.v1"&&
        policy.child("design").asString()==design&&policy.child("basis").asString()=="theoretical_inference"&&
        policy.child("geometry").asString()=="original_source_wing_root_strip_v1"&&
        policy.child("attachment").asString()=="sealed_whole_native_tank_wing_module_v1"&&
        receipt.child("attachment").asString()==policy.child("attachment").asString(),"Unsupported inferred vessel method/attachment");
    const auto& selected=policy.child("tanks").asArray();need(selected.size()==2,"Inferred vessel requires precisely TK0/TK1");
    const auto& summaries=receipt.child("tanks").asArray();need(summaries.size()==2,"Inferred vessel native summaries are incomplete");
    for(std::size_t i=0;i<selected.size();++i) {
        vesselKeys(selected[i],{"index","section","surface"});const char* owner=i==0?"left wing":"right wing";
        need(integer(selected[i].child("index"),2)==i&&selected[i].child("section").asString()==owner&&
            selected[i].child("surface").asString()=="wing","Inferred vessel departs its supported native tank/surface");
        need(integer(summaries[i].child("index"),2)==i&&summaries[i].child("section").asString()==owner,
            "Inferred vessel summary departs its native mechanical owner");
    }
    need(!doc.child("ledger").child("native_owner_interval_certified").asBool()&&
        doc.child("ledger").child("per_tank_vessel_eligibility_certified").asBool(),
        "Inferred vessel cannot claim original singleton skin ownership");
    const auto& mesh=receipt.child("source_wing");
    need(mesh.child("method").asString()=="original_uninflated_wing_primitive_root_clip_v1"&&
        mesh.child("surface").asString()=="wing"&&mesh.child("skin_inflation_m").asNumber()==0&&
        mesh.child("root_extension_m").asNumber()==0,"Inferred vessel requires original uninflated source wing");
    const double cut=positive(receipt.child("structural_root_cut_design_y_m"));
    const double originY=vector(doc.child("method").child("origin_design_m")).y;
    for(const auto& cell:cells) {
        const auto& kind=cell.child("eligibility_v1").asString();
        need(kind=="native_owner_interval_v1"||kind=="inferred_original_wing_root_v1","Unknown inferred vessel cell eligibility");
        if(kind=="native_owner_interval_v1")continue;
        const auto index=integer(cell.child("tank"),2);const char* owner=index==0?"left wing":"right wing";
        need(cell.child("section").asString()==owner,"Inferred root vessel has wrong native mechanical owner");
        const V3 lo=vector(cell.child("low_m")),hi=vector(cell.child("high_m"));
        const double lower=index==0?-cut-originY:-originY,upper=index==0?-originY:cut-originY;
        need(lower<=lo.y&&lo.y<hi.y&&hi.y<=upper,"Inferred root vessel leaves its own source half-strip");
    }
}
}

std::shared_ptr<const LiquidMaterialDatasetV1> readLiquidMaterialDatasetV1(
    const std::filesystem::path& directory,const std::string& design,
    const ms::damage::Airframe* frame,bool& declared,std::string& error) {
    declared=std::filesystem::is_regular_file(directory/(design+".liquid-v1.json"));error.clear();
    try {
        const auto damagePath=directory/(design+".damage.json");
        if(std::filesystem::is_regular_file(damagePath)) {
            const auto damage=document(damagePath);
            if(damage.has("liquid_geometry_v1")) {
                declared=true;
                const auto& reference=damage.child("liquid_geometry_v1");
                need(reference.child("required").asBool(),"Current liquid declaration must require its inputs");
                need(reference.child("file").asString()==design+".liquid-v1.json"&&
                    reference.child("schema").asString()=="fsim.liquid-occupancy.v1","Invalid liquid material declaration");
            }
        }
        const auto path=directory/(design+".liquid-v1.json");
        if(!declared&&!std::filesystem::is_regular_file(path))return {};
        declared=true;const auto doc=document(path);
        need(doc.child("schema").asString()=="fsim.liquid-occupancy.v1"&&
            doc.child("frame").asString()=="body_origin_forward_right_down_m"&&
            doc.child("design").asString()==design,"Liquid material identity/frame/schema mismatch");
        auto result=std::make_shared<LiquidMaterialDatasetV1>();result->design=design;
        if(frame)need(length(vector(doc.child("method").child("origin_design_m"))-frame->originDesign)<=1e-9,
            "Liquid geometry origin differs from its native structural frame");
        const auto& tanks=doc.child("tanks").asArray();const auto& cells=doc.child("cells").asArray();
        need(tanks.size()<=maxTanks&&cells.size()<=maxCells,"Liquid material population exceeds its bounds");
        validateVesselPolicy(doc,design);
        if(tanks.empty()) {
            if(frame)for(const auto& owner:frame->sections)need(owner.tanks.empty(),"Liquid-inapplicable data omits native tanks");
            need(cells.empty()&&!doc.child("no_liquid_reason").asString().empty(),"Liquid-inapplicable data needs its explicit reason");return result;
        }
        const auto& policy=doc.child("fluid_policy_v1");
        need(policy.child("law").asString()=="quadratic_unresolved_tumbling_v1"&&
            policy.child("basis").asString()=="assumed","Unsupported liquid material law/basis");
        result->dragCoefficient=positive(policy.child("drag_coefficient"));
        positive(policy.child("screening_uncertainty_fraction"));
        std::map<int,std::size_t> byTank;std::map<int,long double> totals;
        for(const auto& row:tanks) {
            LiquidMaterialDatasetV1::Tank tank;tank.index=static_cast<int>(integer(row.child("index"),maxTanks));
            tank.sectionName=row.child("section").asString();tank.section=section(frame,tank.sectionName);
            tank.capacityKg=positive(row.child("capacity_kg"));tank.densityKgM3=positive(row.child("density_kg_m3"));
            tank.anchorBodyM=vector(row.child("anchor_body_m"));
            need(byTank.emplace(tank.index,result->tanks.size()).second,"Duplicate native liquid tank");
            if(frame) {
                unsigned matches=0;
                for(std::size_t s=0;s<frame->sections.size();++s)for(const auto& native:frame->sections[s].tanks)
                    if(native.index==tank.index) {
                        ++matches;need(static_cast<int>(s)==tank.section&&std::abs(native.capacityKg-tank.capacityKg)<=5.1e-6,
                            "Liquid capacity/ownership differs from native section ownership");
                    }
                need(matches==1,"Native liquid tank has missing/duplicate ownership");
            }
            result->tanks.push_back(std::move(tank));
        }
        if(frame)for(const auto& owner:frame->sections)for(const auto& native:owner.tanks)
            need(byTank.count(native.index)==1,"Liquid material omits a native tank");
        for(const auto& row:cells) {
            LiquidCapacityCellV1 cell;const auto id=integer(row.child("id"),maxCells);
            need(id==result->cells.size(),"Liquid capacity cell IDs are not stable sequential source identities");
            cell.stableCellId=static_cast<std::uint64_t>(id)+1;
            cell.nativeTank=static_cast<int>(integer(row.child("tank"),maxTanks));
            const auto found=byTank.find(cell.nativeTank);need(found!=byTank.end(),"Liquid cell names an unknown native tank");
            const auto& tank=result->tanks[found->second];
            need(row.child("section").asString()==tank.sectionName,"Liquid cell departs its native tank owner");cell.section=tank.section;
            cell.bodyBounds={vector(row.child("low_m")),vector(row.child("high_m"))};
            const double actual=volume(cell.bodyBounds),given=positive(row.child("volume_m3"));
            need(std::abs(actual-given)<=256*std::numeric_limits<double>::epsilon()*std::max(actual,given),"Liquid cell volume differs from its spatial box");
            cell.componentIndex=componentFor(frame,cell.nativeTank,cell.section,(cell.bodyBounds.low+cell.bodyBounds.high)*.5);
            totals[cell.nativeTank]+=actual;result->cells.push_back(cell);
        }
        // X sweep bounds comparisons to simultaneously active spatial slabs.
        std::vector<std::size_t> order(result->cells.size());
        for(std::size_t i=0;i<order.size();++i)order[i]=i;
        std::sort(order.begin(),order.end(),[&](auto a,auto b){return result->cells[a].bodyBounds.low.x<result->cells[b].bodyBounds.low.x;});
        for(std::size_t i=0;i<order.size();++i)for(std::size_t j=i+1;j<order.size();++j) {
            const auto a=result->cells[order[i]].bodyBounds,b=result->cells[order[j]].bodyBounds;
            if(b.low.x>=a.high.x)break;
            need(!overlap(a,b),"Liquid material cells overlap");
        }
        for(const auto& tank:result->tanks) {
            const double expected=tank.capacityKg/tank.densityKgM3;
            const double errorVolume=std::abs(static_cast<double>(totals[tank.index])-expected);
            need(std::isfinite(expected)&&expected>0&&errorVolume<=std::max(1e-9,1024*std::numeric_limits<double>::epsilon()*expected*static_cast<double>(cells.size()+1)),
                "Liquid cells do not conserve declared capacity at physical density");
        }
        return result;
    } catch(const std::exception& refused) {error=refused.what();return {};}
}

static std::optional<ms::damage::SceneLiquidRecipientV1> freezeLiquidImpl(
    const LiquidMaterialDatasetV1& data,const ms::damage::AttachedGeometryView* attached,
    std::uint64_t identity,std::uint64_t generation,const std::vector<std::optional<double>>& contents,
    ms::damage::V3 shift,ms::damage::V3 up,const ms::damage::SceneLiquidControlsV1& controls,std::string& error) {
    error.clear();
    try {
        need(std::isfinite(controls.absoluteVolumeToleranceM3)&&controls.absoluteVolumeToleranceM3>=0&&
            std::isfinite(controls.relativeVolumeTolerance)&&controls.relativeVolumeTolerance>=0,
            "Invalid liquid volume tolerances");
        need(identity&&generation&&finite(shift)&&finite(up)&&length(up)>0,"Invalid captured liquid recipient identity/frame");
        up=normalized(up);SceneLiquidRecipientV1 out;out.identity=identity;out.generation=generation;
        std::set<int> needed;
        if(attached)for(const auto index:attached->componentIndices()) {
            const auto& component=attached->airframe().components[index];
            if(component.kind=="fuel")needed.insert(component.tank);
        } else for(const auto& tank:data.tanks) {
            need(tank.section==-1&&tank.sectionName=="airframe","Standalone liquid cannot infer structural ownership");
            needed.insert(tank.index);
        }
        for(const auto& source:data.cells) {
            if(attached&&std::find(attached->componentIndices().begin(),attached->componentIndices().end(),source.componentIndex)==attached->componentIndices().end())continue;
            auto cell=source;
            if(!attached) {
                need(cell.section==-1&&needed.count(cell.nativeTank)==1,"Standalone liquid cell has an unknown native owner");
                // A V2 liquid-only key names a native tank, never an Airframe component.
                cell.componentIndex=static_cast<std::size_t>(cell.nativeTank);
            }
            cell.bodyBounds.low=cell.bodyBounds.low-shift;cell.bodyBounds.high=cell.bodyBounds.high-shift;out.cells.push_back(cell);
        }
        for(const int tankId:needed) {
            const auto found=std::find_if(data.tanks.begin(),data.tanks.end(),[&](const auto& row){return row.index==tankId;});
            need(found!=data.tanks.end()&&tankId>=0&&static_cast<std::size_t>(tankId)<contents.size()&&contents[static_cast<std::size_t>(tankId)].has_value(),
                "Required captured native fuel contents are unavailable");
            const double mass=*contents[static_cast<std::size_t>(tankId)];need(std::isfinite(mass)&&mass>=0,"Captured native fuel is invalid");
            LiquidTankStateV1 tank;tank.nativeTank=tankId;tank.densityKgM3=found->densityKgM3;tank.actualContentsKg=mass;
            // Explicit isotropic/tumbling pressure-drag engineering policy.
            // It is a separate liquid assumption, not the source's air Cd or
            // a fitted FAA/OEM coefficient. See the liquid method document.
            tank.dragCoefficient=data.dragCoefficient;tank.freeSurfaceBody.normal=up;
            std::vector<fluid::Aabb> boxes;long double capacity=0;double low=std::numeric_limits<double>::infinity(),high=-low;
            for(const auto& cell:out.cells)if(cell.nativeTank==tankId) {
                boxes.push_back(cell.bodyBounds);capacity+=volume(cell.bodyBounds);
                low=std::min(low,lower(cell.bodyBounds,up));high=std::max(high,upper(cell.bodyBounds,up));
            }
            const double target=mass/tank.densityKgM3,total=static_cast<double>(capacity);
            const double tolerance=controls.absoluteVolumeToleranceM3+controls.relativeVolumeTolerance*std::max(target,total);
            need(std::isfinite(target)&&std::isfinite(tolerance)&&tolerance>=0&&target<=total+tolerance,"Native fuel exceeds its owned geometric capacity");
            if(boxes.empty()){need(mass==0,"Live fuel has no local capacity after ownership filtering");tank.freeSurfaceBody.offset=0;}
            else if(target<=0)tank.freeSurfaceBody.offset=low;
            else if(target>=total)tank.freeSurfaceBody.offset=high;
            else {
                bool solved=false;double level=low;
                for(unsigned iteration=0;iteration<100;++iteration) {
                    level=low+(high-low)*.5;long double wet=0;
                    for(const auto box:boxes) {const auto value=fluid::immersedVolume(box,{up,level});need(value.has_value(),"Liquid free-surface volume evaluation failed");wet+=*value;}
                    const double actual=static_cast<double>(wet);
                    if(std::abs(actual-target)<=tolerance){solved=true;break;}
                    if(actual<target)low=level;else high=level;
                }
                need(solved,"Liquid free-surface solve exhausted its numerical budget");tank.freeSurfaceBody.offset=level;
            }
            out.tanks.push_back(tank);
        }
        return out;
    } catch(const std::exception& refused){error=refused.what();return {};}
}

std::optional<ms::damage::SceneLiquidRecipientV1> freezeLiquidRecipientV1(
    const LiquidMaterialDatasetV1& data,const ms::damage::AttachedGeometryView& attached,
    std::uint64_t identity,std::uint64_t generation,const std::vector<std::optional<double>>& contents,
    ms::damage::V3 shift,ms::damage::V3 up,const ms::damage::SceneLiquidControlsV1& controls,std::string& error) {
    return freezeLiquidImpl(data,&attached,identity,generation,contents,shift,up,controls,error);
}
std::optional<ms::damage::SceneLiquidRecipientV1> freezeUnsectionedLiquidRecipientV2(
    const LiquidMaterialDatasetV1& data,std::uint64_t identity,std::uint64_t generation,
    const std::vector<std::optional<double>>& contents,ms::damage::V3 up,
    const ms::damage::SceneLiquidControlsV1& controls,std::string& error) {
    return freezeLiquidImpl(data,nullptr,identity,generation,contents,{},up,controls,error);
}
void World::enableLiquidMaterialDiagnosticV1(bool enabled) {
    liquidMaterialDiagnosticStateV1_=enabled?std::make_shared<LiquidMaterialDiagnosticStateV1>():nullptr;
}
std::shared_ptr<const LiquidMaterialDiagnosticV1> World::liquidMaterialDiagnosticV1() const {
    if(!liquidMaterialDiagnosticStateV1_||!liquidMaterialDiagnosticStateV1_->last)return {};
    const auto& last=liquidMaterialDiagnosticStateV1_->last;
    if(last->sampleTimeS!=simTime_||last->recipient.identity>std::numeric_limits<std::uint32_t>::max())return {};
    const auto* e=entry(static_cast<std::uint32_t>(last->recipient.identity));
    if(!e||e->info.generation!=last->recipient.generation||e->engagementPath.empty())return {};
    return last;
}
void World::captureLiquidAssessmentV1(const Entry& e,const ms::damage::Impact& impact,
    std::uint64_t source,std::uint64_t sourceGeneration,const std::vector<ms::damage::SceneLiquidOnlyWorkV2>& only,
    const ms::damage::FragmentFieldV1& field) {
    if(!liquidMaterialDiagnosticStateV1_||!liquidMaterialDiagnosticStateV1_->last)return;
    const auto& old=liquidMaterialDiagnosticStateV1_->last;
    if(old->recipient.identity!=e.info.id||old->recipient.generation!=e.info.generation||old->sampleTimeS!=simTime_)return;
    auto snapshot=std::make_shared<LiquidMaterialDiagnosticV1>(*old);
    snapshot->source=source;snapshot->sourceGeneration=sourceGeneration;snapshot->assessed=true;
    snapshot->impulseBody=impact.impulseBody;snapshot->angularImpulseBody=impact.angularImpulseBody;
    snapshot->fieldAgeS=field.ageS;snapshot->projectileCount=field.projectiles.size();
    if(!field.projectiles.empty())snapshot->firstProjectile=field.projectiles.front();
    if(snapshot->unsectioned) {
        for(const auto& row:only)if(row.identity==e.info.id&&row.generation==e.info.generation)snapshot->tankWork.push_back(row);
    } else for(const auto& component:impact.components) {
        const auto cell=std::find_if(snapshot->recipient.cells.begin(),snapshot->recipient.cells.end(),[&](const auto& c){
            return component.component>=0&&c.componentIndex==static_cast<std::size_t>(component.component);});
        if(cell==snapshot->recipient.cells.end())continue;
        auto row=std::find_if(snapshot->tankWork.begin(),snapshot->tankWork.end(),[&](const auto& r){return r.nativeTank==cell->nativeTank;});
        if(row==snapshot->tankWork.end()){snapshot->tankWork.push_back({e.info.id,e.info.generation,cell->nativeTank,0,0});row=std::prev(snapshot->tankWork.end());}
        row->energyJ+=component.energyJ;if(component.hits>0)row->wallContacts+=static_cast<unsigned>(component.hits);
    }
    liquidMaterialDiagnosticStateV1_->last=std::move(snapshot);
}

bool World::appendLiquidMaterialV1(const Entry& e,const sim::VehicleState& state,const Entry::Matter& matter,
    const ms::damage::AttachedGeometryView& geometry,ms::damage::SceneLiquidInputsV1& inputs,std::string& error) const {
    if(!e.liquidMaterials)return true; // Retained legacy/custom data have no new declaration.
    const auto& data=*e.liquidMaterials;
    if(data.tanks.empty()) {
        for(const auto index:geometry.componentIndices())if(geometry.airframe().components[index].kind=="fuel") {
            error="Liquid-inapplicable product still declares a fuel component";return false;
        }
        return true;
    }
    const V3 up{std::cos(state.latitudeRad)*std::cos(state.longitudeRad),
        std::cos(state.latitudeRad)*std::sin(state.longitudeRad),std::sin(state.latitudeRad)};
    const auto* r=state.rotationBodyToEcef;
    const V3 body{r[0]*up.x+r[3]*up.y+r[6]*up.z,r[1]*up.x+r[4]*up.y+r[7]*up.z,r[2]*up.x+r[5]*up.y+r[8]*up.z};
    auto recipient=freezeLiquidRecipientV1(data,geometry,e.info.id,e.info.generation,matter.tankFuelKg,
        e.liquidOriginShiftBodyM,body,inputs.controls,error);
    if(!recipient)return false;
    if(liquidMaterialDiagnosticStateV1_) {
        auto snapshot=std::make_shared<LiquidMaterialDiagnosticV1>();snapshot->sampleTimeS=simTime_;
        snapshot->recipient=*recipient;liquidMaterialDiagnosticStateV1_->last=std::move(snapshot);
    }
    if(!recipient->tanks.empty())inputs.recipients.push_back(std::move(*recipient));
    return true;
}

bool World::appendUnsectionedLiquidMaterialV2(const Entry& e,const sim::VehicleState& state,const Entry::Matter& matter,
    ms::damage::SceneLiquidInputsV1& inputs,std::string& error) const {
    if(!e.liquidMaterials){error="Standalone liquid source is unavailable";return false;}
    const V3 up{std::cos(state.latitudeRad)*std::cos(state.longitudeRad),
        std::cos(state.latitudeRad)*std::sin(state.longitudeRad),std::sin(state.latitudeRad)};
    const auto* r=state.rotationBodyToEcef;
    const V3 body{r[0]*up.x+r[3]*up.y+r[6]*up.z,r[1]*up.x+r[4]*up.y+r[7]*up.z,r[2]*up.x+r[5]*up.y+r[8]*up.z};
    auto recipient=freezeUnsectionedLiquidRecipientV2(*e.liquidMaterials,e.info.id,e.info.generation,
        matter.tankFuelKg,body,inputs.controls,error);
    if(!recipient)return false;
    if(liquidMaterialDiagnosticStateV1_) {
        auto snapshot=std::make_shared<LiquidMaterialDiagnosticV1>();snapshot->sampleTimeS=simTime_;snapshot->unsectioned=true;
        snapshot->recipient=*recipient;liquidMaterialDiagnosticStateV1_->last=std::move(snapshot);
    }
    if(!recipient->tanks.empty())inputs.recipients.push_back(std::move(*recipient));
    return true;
}
}
