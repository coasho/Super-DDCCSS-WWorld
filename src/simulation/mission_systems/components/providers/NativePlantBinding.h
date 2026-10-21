#pragma once
#include "Builtins.h"

namespace fsim::ms::components {
/// Shared metadata admission only. Concrete roles own registration and command
/// dispatch; JSBSim owns each selected backend's integration.
template<class Interface> class NativePlantBindingV1 : public Interface {
public:
    std::string_view nativeModel() const noexcept final {return model_;}
    bool available(const weapon::NativePlantMetadataV1& expected,
                   const weapon::NativePlantMetadataV1& actual) const noexcept final {
        return actual.schemaVersion==expected.schemaVersion && actual.implementationCode==expected.implementationCode &&
               actual.fingerprint==expected.fingerprint;
    }
protected:
    explicit NativePlantBindingV1(std::string model) : model_(std::move(model)) {}
private:
    std::string model_;
};
}
