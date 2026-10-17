#pragma once

// Explicit MA-service bindings (docs/sdk/mission-training.md, section 9; ADR-31). A platform may carry several MA
// services, each one Agent with a controller the range minted. Where the range binds them, a binding - not the order
// its Agents were made in - says which service owns each command capability, the platform's DMS ingress and its DMS
// egress. A platform with no binding keeps the legacy behaviour untouched.
//
// This is the record alone: what is bound and who owns what. The World validates a binding against its entities,
// agents and capabilities before it is added, and applies it (World::bindMaService).

#include "fsim/Capability.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::session {

struct MaServiceBinding {
    std::uint64_t serviceId = 0;           ///< nonzero, unique in the world
    std::uint32_t platform = 0;            ///< the vehicle's or C2 node's entity id
    control::ControllerId controller = 0;  ///< the controller the world minted for this service's Agent
    std::vector<std::string> capabilities; ///< discovery ids exactly as the platform reports them, or the single entry "*"
    bool ownsDmsIngress = false;           ///< its inbox and subscriptions
    bool ownsDmsEgress = false;            ///< its sends, native publications, task answers and C2 messages
    /// Every command capability of its platform.
    bool all() const noexcept { return capabilities.size() == 1 && capabilities.front() == "*"; }
    bool owns(std::string_view capability) const noexcept;
};

class MaServiceBindings {
public:
    /// Why `binding` cannot join what is bound - a service or controller bound already, a capability or a DMS side
    /// another service of its platform owns, a list that repeats itself or mixes "*" with names; empty: it can.
    std::string refusal(const MaServiceBinding& binding) const;
    /// Add one `refusal` accepted: kept in ascending service id.
    void add(MaServiceBinding binding);

    /// The platform has at least one bound service: its ownership is configured, not legacy.
    bool bound(std::uint32_t platform) const noexcept;
    const MaServiceBinding* ofController(control::ControllerId controller) const noexcept;
    const MaServiceBinding* ofService(std::uint64_t serviceId) const noexcept;
    /// The controller that owns a capability of a platform (0: no bound service does).
    control::ControllerId owner(std::uint32_t platform, std::string_view capability) const noexcept;
    control::ControllerId ingressOwner(std::uint32_t platform) const noexcept;
    control::ControllerId egressOwner(std::uint32_t platform) const noexcept;
    /// Every binding, in ascending service id: an order no creation order changes.
    const std::vector<MaServiceBinding>& all() const noexcept { return bindings_; }
    /// A platform gone: its bindings with it.
    void release(std::uint32_t platform) noexcept;

private:
    std::vector<MaServiceBinding> bindings_;
};

} // namespace fsim::session
