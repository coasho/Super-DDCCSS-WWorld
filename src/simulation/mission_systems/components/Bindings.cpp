#include "Bindings.h"
namespace fsim::ms::components {
namespace {
bool same(const DescriptorV1&,const DescriptorV1&);
bool same(const Parameter& a,const Parameter& b) {
    return a.value==b.value && a.unit==b.unit && a.uncertainty==b.uncertainty && a.basis==b.basis && a.source==b.source && a.note==b.note;
}
bool same(const Source& a,const Source& b) {
    return a.title==b.title && a.publisher==b.publisher && a.url==b.url && a.date==b.date;
}
template<class T> bool sameMap(const std::map<std::string,T>& a,const std::map<std::string,T>& b) {
    if (a.size()!=b.size()) return false;
    auto i=a.begin(),j=b.begin();
    for (;i!=a.end();++i,++j) if (i->first!=j->first || !same(i->second,j->second)) return false;
    return true;
}
bool same(const StoreCard& a,const StoreCard& b) {
    const auto& x=a.guidanceCapabilities;const auto& y=b.guidanceCapabilities;
    return a.id==b.id && a.designation==b.designation && a.summary==b.summary && a.maker==b.maker && a.storeClass==b.storeClass &&
        a.model==b.model && a.seeker==b.seeker && a.powerable==b.powerable && a.jettisonable==b.jettisonable &&
        a.warhead==b.warhead && a.warheadKind==b.warheadKind && a.explosive==b.explosive &&
        x.version==y.version && x.inertialMidcourse==y.inertialMidcourse && x.launcherUpdates==y.launcherUpdates &&
        x.basis==y.basis && x.source==y.source && x.note==y.note && sameMap(a.parameters,b.parameters) && sameMap(a.sources,b.sources);
}
bool same(const DescriptorV1& a,const DescriptorV1& b) {
    return a.kind==b.kind && a.implementationId==b.implementationId && a.configurationVersion==b.configurationVersion &&
        a.availability==b.availability && sameMap(a.parameters,b.parameters) && a.parameterRefs==b.parameterRefs &&
        a.bindings==b.bindings && a.basis==b.basis && a.source==b.source && a.note==b.note && a.validity==b.validity && a.reason==b.reason;
}
bool same(const ProfileV1& a,const ProfileV1& b) {
    return a.storeId==b.storeId && a.schemaVersion==b.schemaVersion && sameMap(a.components,b.components);
}
}
bool BindingsV1::bind(std::shared_ptr<const StoreCard> card,std::shared_ptr<const ProfileV1> profile,
    const std::filesystem::path& origin,std::string& error) {
    error.clear();
    if (!card) {error="cannot bind an absent loaded card";return false;}
    const auto address=card.get();
    const auto deny=[&](const char* reason) {error=reason;refuse(card,origin,error);return false;};
    if (!profile || card->id.empty() || profile->storeId!=card->id || origin.empty()) return deny("invalid loaded card/profile/source binding");
    if (const auto previous=loaded_.find(address);previous!=loaded_.end()) {
        if (!previous->second.profile) {error=previous->second.error;return false;}
        if (previous->second.origin!=origin || !same(*previous->second.profile,*profile))
            return deny("an immutable loaded card binding cannot be replaced");
        return true;
    }
    if (const auto identity=identities_.find(card->id);identity!=identities_.end()) {
        const auto& first=loaded_.at(identity->second);
        if (first.origin!=origin) return deny("same store identity comes from a different component library");
        if (!same(*first.card,*card) || !same(*first.profile,*profile)) return deny("same store identity has a different card or component declaration");
    } else identities_.emplace(card->id,address);
    loaded_.emplace(address,Binding{std::move(card),std::move(profile),origin,{}});
    return true;
}
void BindingsV1::refuse(std::shared_ptr<const StoreCard> card,const std::filesystem::path& origin,const std::string& error) {
    if (!card) return;
    const auto address=card.get();
    loaded_.emplace(address,Binding{std::move(card),{},origin,error.empty()?"component binding refused":error});
}
std::shared_ptr<const ProfileV1> BindingsV1::find(const StoreCard& card,std::string& error) const {
    const auto binding=loaded_.find(&card);
    if (binding==loaded_.end()) {error="component source was not retained with this loaded card object";return {};}
    error=binding->second.error;return binding->second.profile;
}
}
