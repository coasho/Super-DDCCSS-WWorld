#pragma once
#include "Profile.h"

namespace fsim::ms::components {

/// One World's bindings of actual loaded immutable card objects. A rejected
/// second source cannot later retrieve the first source's same-ID profile.
class BindingsV1 {
public:
    bool bind(std::shared_ptr<const StoreCard>,std::shared_ptr<const ProfileV1>,
              const std::filesystem::path& origin,std::string& error);
    void refuse(std::shared_ptr<const StoreCard>,const std::filesystem::path& origin,const std::string& error);
    std::shared_ptr<const ProfileV1> find(const StoreCard&,std::string& error) const;
private:
    struct Binding {
        std::shared_ptr<const StoreCard> card;
        std::shared_ptr<const ProfileV1> profile;
        std::filesystem::path origin;
        std::string error;
    };
    std::map<const StoreCard*,Binding> loaded_;
    std::map<std::string,const StoreCard*> identities_;
};
}
