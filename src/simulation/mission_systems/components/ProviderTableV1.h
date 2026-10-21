#pragma once

#include "ProviderContextV1.h"
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

namespace fsim::ms::components {

/// Catalogue copies freeze membership; callbacks must own their captures.
template<class Provider> class ProviderTableV1 {
public:
    using Factory = std::function<std::unique_ptr<Provider>(
        const DescriptorV1&, const ProviderContextV1&, std::string&)>;
    explicit ProviderTableV1(std::string kind) : kind_(std::move(kind)) {}
    bool registerFactory(std::string id, std::string schemaImplementationId, Factory factory,
                         bool replace, std::string& error) {
        if(id.empty() || schemaImplementationId.empty() || !factory) {
            error="provider registration needs identity, schema and factory";return false;
        }
        if(!replace && entries_.count(id)) {error="provider identity is already registered";return false;}
        entries_.insert_or_assign(std::move(id), Registration{std::move(schemaImplementationId),std::move(factory)});
        error.clear();return true;
    }
    bool descriptor(const DescriptorV1& input, DescriptorV1& output, std::string& error) const {
        if(input.kind!=kind_ || input.configurationVersion!=1 || input.availability!=AvailabilityV1::Available) {
            error="provider selection has wrong role/version/availability";return false;
        }
        const auto found=entries_.find(input.implementationId);
        if(found==entries_.end()) {error="unregistered "+kind_+" provider '"+input.implementationId+"'";return false;}
        auto next=input;next.implementationId=found->second.schema;output=std::move(next);error.clear();return true;
    }
    std::unique_ptr<Provider> create(const DescriptorV1& input, const ProviderContextV1& context,
                                     std::string& error) const {
        DescriptorV1 selected;
        if(!descriptor(input,selected,error)) return {};
        try {
            auto result=entries_.at(input.implementationId).factory(selected,context,error);
            if(!result) {if(error.empty()) error="selected "+kind_+" factory returned no instance";return {};}
            if(!error.empty()) return {};
            return result;
        } catch(const std::exception& ex) {error="selected "+kind_+" factory failed: "+ex.what();return {};}
        catch(...) {error="selected "+kind_+" factory failed with an unknown exception";return {};}
    }
    std::size_t size() const noexcept { return entries_.size(); }
private:
    struct Registration { std::string schema; Factory factory; };
    std::string kind_;
    std::map<std::string,Registration> entries_;
};

} // namespace fsim::ms::components
