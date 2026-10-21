#include "Builtins.h"
namespace fsim::ms::components {
namespace {
class EncounterSeekerV1 final : public SeekerProviderV1 {
public:
    explicit EncounterSeekerV1(weapon::SeekerConfigV1 config) : config_(config) {}
    const weapon::SeekerConfigV1& config() const noexcept override {return config_;}
    double infraredRange(double base,double aspect,bool burner) const noexcept override {
        return weapon::infraredAcquisitionRangeM(base,aspect,burner,config_);
    }
    double radarRange(double base,double cross,bool modelled) const noexcept override {
        return weapon::radarAcquisitionRangeM(base,cross,modelled,config_);
    }
    bool notched(double vertical,double radial) const noexcept override {return weapon::radarNotched(vertical,radial,config_);}
    double decoySignal(double age,bool infrared) const noexcept override {return weapon::decoySignal(age,infrared,config_);}
    bool illuminated(const double direction[3],double azimuth,double elevation,double aw,double ew) const noexcept override {
        return weapon::withinIlluminationBeam(direction,azimuth,elevation,aw,ew,config_);
    }
private:
    weapon::SeekerConfigV1 config_;
};
}
std::unique_ptr<SeekerProviderV1> nativeSeekerProviderV1(weapon::SeekerConfigV1 config) {return std::make_unique<EncounterSeekerV1>(config);}
void registerNativeSeekerV1(ProviderRegistryV1& registry) {
    std::string error;
    registry.seeker.registerFactory("fsim.seeker.encounter.v1","fsim.seeker.encounter.v1",
        [](const DescriptorV1& d,const ProviderContextV1&,std::string& e) -> std::unique_ptr<SeekerProviderV1> {
            weapon::SeekerConfigV1 config;if(!weapon::configureSeeker(d,config,e))return {};
            return nativeSeekerProviderV1(config);
        },false,error);
}
}
