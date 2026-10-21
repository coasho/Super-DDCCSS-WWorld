#pragma once

#include "simulation/mission_systems/Cards.h"

#include <initializer_list>
#include <map>
#include <string>

namespace fsim::core { class Json; }
namespace fsim::ms::components {

enum class AvailabilityV1 { Available, Unavailable, NotApplicable };

/// Resolved immutable-by-ownership sidecar. Values and provenance are copied;
/// no instance retains pointers into a mutable card or parsed JSON document.
struct DescriptorV1 {
    std::string kind, implementationId;
    unsigned configurationVersion = 1;
    AvailabilityV1 availability = AvailabilityV1::Unavailable;
    std::map<std::string, Parameter> parameters;
    std::map<std::string, std::string> parameterRefs, bindings;
    Basis basis = Basis::Assumed;
    std::string source, note, validity, reason;
};

struct ProfileV1 {
    unsigned schemaVersion = 0; ///< zero identifies an absent legacy declaration
    std::string storeId;
    std::map<std::string, DescriptorV1> components;
    const DescriptorV1* find(const std::string& kind) const noexcept;
    bool explicitProfile() const noexcept { return schemaVersion != 0; }
};

/// Atomic parse/resolve. Absence succeeds as an explicitly identifiable legacy
/// profile. An invalid explicit block never substitutes legacy behavior.
bool parseProfileV1(const core::Json& document, const StoreCard& card,
                    ProfileV1& output, std::string& error);
bool readProfileV1(const std::filesystem::path& file, const StoreCard& card,
                   ProfileV1& output, std::string& error);

/// Required finite scalar with exact unit and validated provenance. Failure
/// leaves output unchanged. A provider must additionally check its own domain.
bool parameterV1(const DescriptorV1&, const std::string& role,
                 const std::string& unit, double& output, std::string& error);

/// Reject unrecognized slots/bindings. Providers check required slots with
/// parameterV1 and required bindings explicitly; optional slots remain possible.
bool slotsV1(const DescriptorV1&, std::initializer_list<const char*> parameters,
             std::initializer_list<const char*> bindings, std::string& error);

} // namespace fsim::ms::components
