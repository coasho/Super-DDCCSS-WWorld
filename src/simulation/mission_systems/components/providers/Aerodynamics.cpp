#include "NativePlantBinding.h"
namespace fsim::ms::components {
namespace {
class JsbsimGeneratedStoreAerodynamicsV1 final : public NativePlantBindingV1<AerodynamicsProviderV1> {
public:
    JsbsimGeneratedStoreAerodynamicsV1(std::string model,std::string source)
        : NativePlantBindingV1(std::move(model)),source_(std::move(source)) {}
    std::string_view coefficientSource() const noexcept override {return source_;}
private:
    std::string source_;
};
}
void registerNativeAerodynamicsV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.aerodynamics.registerFactory("fsim.aerodynamics.jsbsim_generated_store","fsim.aerodynamics.jsbsim_generated_store",
        [](const DescriptorV1& d,const ProviderContextV1& ctx,std::string& e) -> std::unique_ptr<AerodynamicsProviderV1> {
            const auto source=d.bindings.find("source");
            if(source==d.bindings.end()){e="native aerodynamic source missing";return {};}
            e.clear();return std::make_unique<JsbsimGeneratedStoreAerodynamicsV1>(ctx.card.model,source->second);
        },false,error);
}
}
