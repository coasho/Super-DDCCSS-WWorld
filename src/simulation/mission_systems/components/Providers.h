#pragma once

#include "Profile.h"
#include "simulation/mission_systems/weapon/Guidance.h"
#include "simulation/mission_systems/weapon/FuzeAdapter.h"
#include "simulation/mission_systems/weapon/PlantAdapter.h"
#include "simulation/mission_systems/damage/WarheadAdapter.h"
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

namespace fsim::ms::components {

/// Native source-extension interfaces. No simulation state or RNG is global.
/// Providers see only the inputs admitted by the World owner.
class SeekerProviderV1 {
public:
    virtual ~SeekerProviderV1() = default;
    virtual const weapon::SeekerConfigV1& config() const noexcept = 0;
    virtual double infraredRange(double base, double aspect, bool afterburner) const noexcept = 0;
    virtual double radarRange(double base, double crossSection, bool modelled) const noexcept = 0;
    virtual bool notched(double vertical, double radialSpeed) const noexcept = 0;
    virtual double decoySignal(double age, bool infrared) const noexcept = 0;
    virtual bool illuminated(const double direction[3], double azimuth, double elevation,
                             double azimuthWidth, double elevationWidth) const noexcept = 0;
};

class GuidanceProviderV1 {
public:
    virtual ~GuidanceProviderV1() = default;
    virtual weapon::Spec spec() const noexcept = 0;
    virtual weapon::Output update(const weapon::Input&) = 0;
};

/// Each native plant role has an independently selected provider. Backend
/// integration remains in JSBSim; command stages cannot emit forces/moments.
class NativeBindingProviderV1 {
public:
    virtual ~NativeBindingProviderV1() = default;
    virtual std::string_view nativeModel() const noexcept = 0;
    virtual bool available(const weapon::NativePlantMetadataV1& expected,
                           const weapon::NativePlantMetadataV1& actual) const noexcept = 0;
};
class AutopilotProviderV1 : public NativeBindingProviderV1 {
public:
    virtual weapon::FinCommands commands(const weapon::AutopilotInput&,
                                         const weapon::ControlStopsV1&) const noexcept = 0;
};
class AllocatorProviderV1 : public NativeBindingProviderV1 {
public:
    virtual weapon::FinCommands allocate(weapon::FinCommands,
                                         const weapon::ControlStopsV1&) const noexcept = 0;
};
class ActuatorProviderV1 : public NativeBindingProviderV1 {
public:
    virtual bool accepts(const weapon::AutopilotInput&,
                         const weapon::ControlStopsV1&) const noexcept = 0;
};
class PropulsionProviderV1 : public NativeBindingProviderV1 {
public:
    virtual bool acceptsEngineCount(unsigned) const noexcept = 0;
    virtual bool burning(double nativeThrustN) const noexcept = 0;
};
class AerodynamicsProviderV1 : public NativeBindingProviderV1 {
public:
    virtual std::string_view coefficientSource() const noexcept = 0;
};

struct ProviderContextV1 {
    const StoreCard& card;
    const ProfileV1& profile;
    weapon::Spec spec;
    std::optional<weapon::Observation> initialEstimate;
};

/// Uniform typed factories. Catalogue copies freeze registration membership;
/// callbacks must own their captures and be safe for independent Worlds.
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

struct PlantProvidersV1 {
    std::unique_ptr<AutopilotProviderV1> autopilot;
    std::unique_ptr<AllocatorProviderV1> allocator;
    std::unique_ptr<ActuatorProviderV1> actuator;
    std::unique_ptr<PropulsionProviderV1> propulsion;
    std::unique_ptr<AerodynamicsProviderV1> aerodynamics;
    bool available(const weapon::PlantAdapterV1&, const weapon::NativePlantIdentityV1&) const noexcept;
    weapon::FinCommands commands(const weapon::PlantAdapterV1&, const weapon::AutopilotInput&,
        const weapon::ControlStopsV1&, const weapon::NativePlantIdentityV1&) const noexcept;
};

class ProviderRegistryV1 {
public:
    ProviderTableV1<SeekerProviderV1> seeker{"seeker"};
    ProviderTableV1<GuidanceProviderV1> guidance{"guidance"};
    ProviderTableV1<weapon::FuzeAdapter> fuze{"fuze"};
    ProviderTableV1<damage::WarheadAdapter> warhead{"warhead"};
    ProviderTableV1<AutopilotProviderV1> autopilot{"autopilot"};
    ProviderTableV1<AllocatorProviderV1> allocator{"allocator"};
    ProviderTableV1<ActuatorProviderV1> actuator{"actuator"};
    ProviderTableV1<PropulsionProviderV1> propulsion{"propulsion"};
    ProviderTableV1<AerodynamicsProviderV1> aerodynamics{"aerodynamics"};
    bool validationProfile(const ProfileV1&, ProfileV1&, std::string& error) const;
    bool createPlant(const ProfileV1&, const ProviderContextV1&, const weapon::PlantAdapterV1&,
                     PlantProvidersV1&, std::string& error) const;
};

ProviderRegistryV1 builtInProvidersV1();
} // namespace fsim::ms::components
