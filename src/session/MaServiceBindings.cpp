#include "session/MaServiceBindings.h"

#include <algorithm>

namespace fsim::session {

bool MaServiceBinding::owns(std::string_view capability) const noexcept {
    if (all()) return true;
    return std::find(capabilities.begin(), capabilities.end(), capability) != capabilities.end();
}

std::string MaServiceBindings::refusal(const MaServiceBinding& b) const {
    if (b.serviceId == 0) return "a service id is not 0";
    if (b.controller == 0) return "a service is bound to a controller the world minted";
    for (std::size_t i = 0; i < b.capabilities.size(); ++i) {
        const std::string& c = b.capabilities[i];
        if (c.empty()) return "a command capability has no name";
        if (c == "*" && b.capabilities.size() != 1) return "\"*\" stands alone";
        if (std::find(b.capabilities.begin(), b.capabilities.begin() + static_cast<std::ptrdiff_t>(i), c) != b.capabilities.begin() + static_cast<std::ptrdiff_t>(i))
            return "command capability " + c + " is listed twice";
    }
    for (const MaServiceBinding& other : bindings_) {
        if (other.serviceId == b.serviceId) return "service " + std::to_string(b.serviceId) + " is bound already";
        if (other.controller == b.controller)
            return "controller " + std::to_string(b.controller) + " is service " + std::to_string(other.serviceId) + "'s already";
        if (other.platform != b.platform) continue;
        const std::string theirs = "service " + std::to_string(other.serviceId) + " of the platform";
        if (b.ownsDmsIngress && other.ownsDmsIngress) return theirs + " owns its DMS ingress";
        if (b.ownsDmsEgress && other.ownsDmsEgress) return theirs + " owns its DMS egress";
        if (b.capabilities.empty() || other.capabilities.empty()) continue;
        if (b.all() || other.all()) return theirs + (other.all() ? " owns every command capability" : " owns command capabilities, so \"*\" cannot be given");
        for (const std::string& c : b.capabilities)
            if (other.owns(c)) return theirs + " owns " + c;
    }
    return {};
}

void MaServiceBindings::add(MaServiceBinding binding) {
    const auto at = std::lower_bound(bindings_.begin(), bindings_.end(), binding.serviceId,
                                     [](const MaServiceBinding& b, std::uint64_t id) { return b.serviceId < id; });
    bindings_.insert(at, std::move(binding));
}

bool MaServiceBindings::bound(std::uint32_t platform) const noexcept {
    for (const MaServiceBinding& b : bindings_)
        if (b.platform == platform) return true;
    return false;
}

const MaServiceBinding* MaServiceBindings::ofController(control::ControllerId controller) const noexcept {
    for (const MaServiceBinding& b : bindings_)
        if (b.controller == controller) return &b;
    return nullptr;
}

const MaServiceBinding* MaServiceBindings::ofService(std::uint64_t serviceId) const noexcept {
    for (const MaServiceBinding& b : bindings_)
        if (b.serviceId == serviceId) return &b;
    return nullptr;
}

control::ControllerId MaServiceBindings::owner(std::uint32_t platform, std::string_view capability) const noexcept {
    for (const MaServiceBinding& b : bindings_)
        if (b.platform == platform && b.owns(capability)) return b.controller;
    return 0;
}

control::ControllerId MaServiceBindings::ingressOwner(std::uint32_t platform) const noexcept {
    for (const MaServiceBinding& b : bindings_)
        if (b.platform == platform && b.ownsDmsIngress) return b.controller;
    return 0;
}

control::ControllerId MaServiceBindings::egressOwner(std::uint32_t platform) const noexcept {
    for (const MaServiceBinding& b : bindings_)
        if (b.platform == platform && b.ownsDmsEgress) return b.controller;
    return 0;
}

void MaServiceBindings::release(std::uint32_t platform) noexcept {
    bindings_.erase(std::remove_if(bindings_.begin(), bindings_.end(), [platform](const MaServiceBinding& b) { return b.platform == platform; }),
                    bindings_.end());
}

} // namespace fsim::session
