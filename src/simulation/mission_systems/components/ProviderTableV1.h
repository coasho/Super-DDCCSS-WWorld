#pragma once

#include "ProviderContextV1.h"
#include "contracts/equipment/EquipmentModels.h"
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
    bool registerEquipmentModel(std::string_view selection,std::string& error) {
        const auto* model=equipment::component(selection);
        if(!model||model->kind!=kind_) {error="equipment model does not identify this component role";return false;}
        const auto backend=entries_.find(std::string(model->mechanismId));
        if(backend==entries_.end()||!backend->second.factory||backend->second.schema!=model->mechanismId) {
            error="equipment model's physical mechanism is not registered";return false;
        }
        if(entries_.count(std::string(selection))) {error="equipment model is already registered";return false;}
        // An empty factory is an explicit link to this catalogue's current
        // mechanism entry. Copies remain independent, and replacing a backend
        // or a model-specific factory keeps the existing replacement semantics.
        entries_.emplace(std::string(selection),Registration{std::string(model->mechanismId),{}});
        error.clear();return true;
    }
    bool registerFactory(std::string id, std::string schemaImplementationId, Factory factory,
                         bool replace, std::string& error) {
        if(id.empty() || schemaImplementationId.empty() || !factory) {
            error="provider registration needs identity, schema and factory";return false;
        }
        if(id.rfind("equipment.",0)==0) {
            const auto* model=equipment::component(id);
            if(!model||model->kind!=kind_||schemaImplementationId!=model->mechanismId) {
                error="equipment namespace requires an exact catalogue model, role and mechanism schema";return false;
            }
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
        if(const auto* model=equipment::component(input.implementationId);model&&
           (model->kind!=kind_||found->second.schema!=model->mechanismId)) {
            error="named equipment component has a different mechanism schema";return false;
        }
        auto next=input;next.implementationId=found->second.schema;output=std::move(next);error.clear();return true;
    }
    std::unique_ptr<Provider> create(const DescriptorV1& input, const ProviderContextV1& context,
                                     std::string& error) const {
        DescriptorV1 selected;
        if(!descriptor(input,selected,error)) return {};
        if(const auto* part=equipment::component(input.implementationId)) {
            const auto* model=equipment::model(part->storeId);
            if(!model||context.card.id!=model->storeId||context.profile.storeId!=model->storeId||
               context.card.model!=model->nativeModel||context.card.designation!=model->cardDesignation) {
                error="named component belongs to another equipment or native model";return {};
            }
        }
        try {
            const auto* registration=&entries_.at(input.implementationId);
            if(!registration->factory) {
                const auto backend=entries_.find(registration->schema);
                if(backend==entries_.end()||!backend->second.factory||backend->second.schema!=selected.implementationId) {
                    error="named equipment mechanism is unavailable or has changed schema";return {};
                }
                registration=&backend->second;
            }
            auto result=registration->factory(selected,context,error);
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
