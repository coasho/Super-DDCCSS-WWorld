#include "Profile.h"
#include "foundation/core/Json.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fsim::ms::components {
namespace {
const std::set<std::string> kinds={"seeker","guidance","autopilot","allocator","actuator",
                                    "fuze","warhead","propulsion","aerodynamics"};
void fields(const core::Json& object,std::initializer_list<const char*> allowed,const std::string& at) {
    if (!object.isObject()) throw std::runtime_error(at+" must be an object");
    std::set<std::string> seen;
    for (const auto& [key,value]:object.asObject()) {
        (void)value;
        if (!seen.insert(key).second) throw std::runtime_error(at+" duplicate field '"+key+"'");
        if (std::none_of(allowed.begin(),allowed.end(),[&](const char* name){return key==name;}))
            throw std::runtime_error(at+" unknown field '"+key+"'");
    }
}
std::string requiredString(const core::Json& object,const char* name,const std::string& at) {
    const auto* value=object.find(name);
    if (!value || !value->isString() || value->asString().empty())
        throw std::runtime_error(at+" needs nonempty string '"+name+"'");
    return value->asString();
}
void parameter(const Parameter& p,const StoreCard* card,const std::string& at) {
    if (!std::isfinite(p.value) || p.unit.empty() || !std::isfinite(p.uncertainty) || p.uncertainty<0)
        throw std::runtime_error(at+" needs finite value, unit and nonnegative finite uncertainty");
    if (p.basis!=Basis::Published && p.basis!=Basis::Derived && p.basis!=Basis::Assumed)
        throw std::runtime_error(at+" has invalid provenance basis");
    if (p.basis!=Basis::Assumed && (p.source.empty() || (card && !card->sources.count(p.source))))
        throw std::runtime_error(at+" needs a listed source");
    if (p.basis!=Basis::Published && p.note.empty())
        throw std::runtime_error(at+" needs a derivation/assumption note");
}
std::map<std::string,std::string> strings(const core::Json& object,const std::string& at) {
    if (!object.isObject()) throw std::runtime_error(at+" must be an object");
    std::map<std::string,std::string> result;
    for (const auto& [key,value]:object.asObject()) {
        if (key.empty() || !value.isString() || value.asString().empty())
            throw std::runtime_error(at+" needs nonempty string keys and values");
        if (!result.emplace(key,value.asString()).second) throw std::runtime_error(at+" duplicate key '"+key+"'");
    }
    return result;
}
}
const DescriptorV1* ProfileV1::find(const std::string& kind) const noexcept {
    const auto it=components.find(kind); return it==components.end()?nullptr:&it->second;
}
bool parseProfileV1(const core::Json& document,const StoreCard& card,ProfileV1& output,std::string& error) {
    error.clear();
    try {
        if (!document.isObject()) throw std::runtime_error("store document must be an object");
        ProfileV1 next; next.storeId=card.id;
        const auto* block=document.find("component_models");
        if (!block) {output=std::move(next);return true;}
        // The underlying JSON representation retains duplicates. Never allow
        // first-member lookup to conceal a second executable declaration.
        std::set<std::string> rootFields;
        for (const auto& [name,value]:document.asObject()) {
            (void)value;
            if (!rootFields.insert(name).second) throw std::runtime_error("duplicate store field '"+name+"'");
        }
        if (document.string("id","")!=card.id || document.string("model","")!=card.model)
            throw std::runtime_error("component profile store/model differs from its loaded card");
        fields(*block,{"schema_version","components"},"component_models");
        if (!block->child("schema_version").isNumber() || block->number("schema_version",0)!=1)
            throw std::runtime_error("component_models needs supported schema_version 1");
        next.schemaVersion=1;
        const auto& list=block->child("components");
        if (!list.isObject()) throw std::runtime_error("component_models.components must be an object");
        for (const auto& [kind,node]:list.asObject()) {
            if (!kinds.count(kind)) throw std::runtime_error("unknown payload component '"+kind+"'");
            if (next.components.count(kind)) throw std::runtime_error("duplicate payload component '"+kind+"'");
            const auto at="component '"+kind+"'";
            DescriptorV1 d; d.kind=kind;
            const auto availability=requiredString(node,"availability",at);
            if (availability=="available") {
                fields(node,{"availability","implementation_id","configuration_version","parameter_refs",
                    "bindings","basis","source","note","validity"},at);
                d.availability=AvailabilityV1::Available;
                d.implementationId=requiredString(node,"implementation_id",at);
                if (!node.child("configuration_version").isNumber() || node.number("configuration_version",0)!=1)
                    throw std::runtime_error(at+" needs supported configuration_version 1");
                d.note=requiredString(node,"note",at); d.validity=requiredString(node,"validity",at);
                const auto basis=requiredString(node,"basis",at);
                if (basis=="published") d.basis=Basis::Published;
                else if (basis=="derived") d.basis=Basis::Derived;
                else if (basis=="assumed") d.basis=Basis::Assumed;
                else throw std::runtime_error(at+" has unknown basis");
                d.source=node.string("source","");
                if ((!d.source.empty() && !card.sources.count(d.source)) ||
                    (d.basis!=Basis::Assumed && d.source.empty())) throw std::runtime_error(at+" needs a listed source");
                d.parameterRefs=strings(node.child("parameter_refs"),at+".parameter_refs");
                d.bindings=strings(node.child("bindings"),at+".bindings");
                for (const auto& [role,ref]:d.parameterRefs) {
                    const auto p=card.parameters.find(ref);
                    if (p==card.parameters.end()) throw std::runtime_error(at+" missing parameter reference '"+ref+"'");
                    parameter(p->second,&card,at+" parameter '"+ref+"'");
                    d.parameters.emplace(role,p->second);
                }
            } else {
                fields(node,{"availability","reason"},at);
                if (availability=="unavailable") d.availability=AvailabilityV1::Unavailable;
                else if (availability=="not_applicable") d.availability=AvailabilityV1::NotApplicable;
                else throw std::runtime_error(at+" has unknown availability");
                d.reason=requiredString(node,"reason",at);
            }
            next.components.emplace(kind,std::move(d));
        }
        if (next.components.size()!=kinds.size()) throw std::runtime_error("explicit payload profile needs all nine component roles");
        output=std::move(next); return true;
    } catch (const std::exception& ex) {error=ex.what();return false;}
}
bool readProfileV1(const std::filesystem::path& file,const StoreCard& card,ProfileV1& output,std::string& error) {
    try {
        std::ifstream in(file,std::ios::binary);
        if (!in) throw std::runtime_error("cannot read payload profile '"+file.string()+"'");
        std::stringstream buffer; buffer<<in.rdbuf();
        const auto doc=core::Json::parse(buffer.str(),file.string()); ProfileV1 next;
        if (!parseProfileV1(doc,card,next,error)) return false;
        // Resolve against the already loaded card, but also reject a later
        // edited file supplying descriptors for a different parameter snapshot.
        for (const auto& [kind,d]:next.components) for (const auto& [role,ref]:d.parameterRefs) {
            const auto& authored=doc.child("parameters").child(ref); const auto& p=d.parameters.at(role);
            const auto basis=p.basis==Basis::Published?"published":p.basis==Basis::Derived?"derived":"assumed";
            if (authored.number("value",std::numeric_limits<double>::quiet_NaN())!=p.value ||
                authored.string("unit","")!=p.unit || authored.number("uncertainty",-1)!=p.uncertainty ||
                authored.string("basis","")!=basis || authored.string("source","")!=p.source || authored.string("note","")!=p.note)
                throw std::runtime_error("component '"+kind+"' reference '"+ref+"' differs from loaded card snapshot");
        }
        output=std::move(next);return true;
    } catch (const std::exception& ex) {error=ex.what();return false;}
}
bool parameterV1(const DescriptorV1& d,const std::string& role,const std::string& unit,double& output,std::string& error) {
    error.clear();
    try {
        if (d.availability!=AvailabilityV1::Available || d.configurationVersion!=1)
            throw std::runtime_error(d.kind+" configuration unavailable or unsupported");
        const auto p=d.parameters.find(role);
        if (p==d.parameters.end()) throw std::runtime_error(d.kind+" missing role '"+role+"'");
        parameter(p->second,nullptr,d.kind+" role '"+role+"'");
        if (p->second.unit!=unit) throw std::runtime_error(d.kind+" role '"+role+"' needs unit '"+unit+"'");
        output=p->second.value;return true;
    } catch (const std::exception& ex) {error=ex.what();return false;}
}
bool slotsV1(const DescriptorV1& d,std::initializer_list<const char*> parameters,std::initializer_list<const char*> bindings,std::string& error) {
    error.clear();
    for (const auto& [role,value]:d.parameters) {
        (void)value;
        if (std::none_of(parameters.begin(),parameters.end(),[&](const char* name){return role==name;})) {
            error=d.kind+" unknown parameter role '"+role+"'";return false;
        }
    }
    for (const auto& [role,value]:d.bindings) {
        (void)value;
        if (std::none_of(bindings.begin(),bindings.end(),[&](const char* name){return role==name;})) {
            error=d.kind+" unknown binding '"+role+"'";return false;
        }
    }
    return true;
}
}
