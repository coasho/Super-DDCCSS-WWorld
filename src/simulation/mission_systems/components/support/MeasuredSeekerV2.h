#pragma once
#include "simulation/mission_systems/components/interfaces/SeekerProviderV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerExecutionV2.h"

namespace fsim::ms::components {
namespace measured {
using V=weapon::V3;
bool finite(V) noexcept;
double norm(V) noexcept;
double dot(V,V) noexcept;
V add(V,V) noexcept;
V subtract(V,V) noexcept;
V scale(V,double) noexcept;
V cross(V,V) noexcept;
V unit(V) noexcept;
bool validExtent(const ImageExtentV2&) noexcept;
}

class MeasuredSeekerV2 : public SeekerProviderV1, public SeekerExecutionV2 {
public:
    explicit MeasuredSeekerV2(weapon::Spec);
    const weapon::SeekerConfigV1& config() const noexcept override;
    double infraredRange(double,double,bool) const noexcept override;
    double radarRange(double,double,bool) const noexcept override;
    bool notched(double,double) const noexcept override;
    double decoySignal(double,bool) const noexcept override;
    bool illuminated(const double[3],double,double,double,double) const noexcept override;
    void seedTrack(const SeekerTrackV2&) override;
    void resetTracking() noexcept override;
protected:
    enum class Admission { New, Repeated, Invalid };
    Admission admit(const SeekerFrameV2&) const noexcept;
    std::vector<std::size_t> eligible(const SeekerFrameV2&) const;
    weapon::V3 reference(const SeekerFrameV2&) const noexcept;
    std::optional<std::size_t> nearest(const SeekerFrameV2&,const std::vector<std::size_t>&) const;
    SeekerResultV2 finish(const SeekerFrameV2&,std::optional<std::size_t>,
                          std::optional<weapon::V3> aggregate=std::nullopt);
    weapon::Spec spec_;
    std::optional<SeekerTrackV2> track_;
    SeekerResultV2 cached_;
private:
    double lastSampleS_=std::numeric_limits<double>::quiet_NaN();
    std::uint32_t nextTrack_=1;
};
}
