#include "JsbsimGeneratedStoreAerodynamicsV1.h"
#include <utility>

namespace fsim::ms::components {

JsbsimGeneratedStoreAerodynamicsV1::JsbsimGeneratedStoreAerodynamicsV1(std::string model,std::string source)
    : NativePlantBindingV1(std::move(model)),source_(std::move(source)) {}

std::string_view JsbsimGeneratedStoreAerodynamicsV1::coefficientSource() const noexcept {return source_;}

} // namespace fsim::ms::components
