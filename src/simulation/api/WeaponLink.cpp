#include "fsim/WeaponLink.h"
#include "fsim/Agent.h"
#include "fsim/World.h"
#include "simulation/runtime/world/World.h"
#include <stdexcept>
namespace fsim {
struct WeaponLinksV1::Access {std::shared_ptr<session::WeaponLinkAccessV1> native;};
namespace {
template<class A,class F> auto checked(const std::shared_ptr<A>& a,F&& function) {
    if(!a||!a->native)throw std::runtime_error("WeaponLinksV1 native World is closed");
    std::lock_guard<std::mutex> lock(a->native->mutex);
    if(!a->native->world)throw std::runtime_error("WeaponLinksV1 native World is closed");
    return function(*a->native->world);
}
}
WeaponLinksV1::WeaponLinksV1(const Agent& a):vehicle_(a.vehicle_),controller_(a.controller_) {
    if(!a.world_)throw std::invalid_argument("WeaponLinksV1 requires a bound Agent");
    access_=std::make_shared<Access>();access_->native=a.world_->impl().weaponLinkAccessV1();
}
WeaponLinksV1::WeaponLinksV1(const Vehicle& v):vehicle_(v.id_) {
    if(!v.world_)throw std::invalid_argument("WeaponLinksV1 requires a bound Vehicle");
    access_=std::make_shared<Access>();access_->native=v.world_->impl().weaponLinkAccessV1();
}
std::uint64_t WeaponLinksV1::instance()const{return checked(access_,[](auto& w){return w.weaponLinkInstanceV1();});}
ms::WeaponLinkCapabilityV1 WeaponLinksV1::capability(std::uint32_t w)const{return checked(access_,[&](auto& world){return world.weaponLinkCapabilityV1(vehicle_,controller_,w);});}
std::uint32_t WeaponLinksV1::designateFused(std::uint64_t i,std::uint64_t e){return checked(access_,[&](auto& w){return w.designateFusedV1(vehicle_,i,e);});}
std::vector<ms::FusedDesignationV1> WeaponLinksV1::fusedDesignations()const{return checked(access_,[&](auto& w){return w.fusedDesignationsV1(vehicle_);});}
ms::WeaponAssessmentReceiptV1 WeaponLinksV1::assess(const ms::WeaponAssessmentRequestV1& q){return checked(access_,[&](auto& w){return w.assessWeaponV1(vehicle_,controller_,q);});}
bool WeaponLinksV1::cancel(std::uint64_t i,std::uint32_t w,std::uint64_t a){return checked(access_,[&](auto& world){return world.cancelWeaponAssessmentV1(vehicle_,controller_,i,w,a);});}
ms::WeaponAssessmentStatusV1 WeaponLinksV1::status(std::uint32_t w,std::uint64_t a)const{return checked(access_,[&](auto& world){return world.weaponAssessmentStatusV1(vehicle_,controller_,w,a);});}
std::vector<ms::WeaponFeedbackV1> WeaponLinksV1::feedback(std::uint32_t w,std::uint64_t a,std::uint64_t s)const{return checked(access_,[&](auto& world){return world.weaponFeedbackV1(vehicle_,controller_,w,a,s);});}
}
