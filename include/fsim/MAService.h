#pragma once

// fsim SDK: explicit MA-service binding (docs/sdk/mission-training.md, section 9; ADR-31).
//
// An Agent is one permission-scoped endpoint on a platform. A platform may carry several - several MA services - and
// then something must say which of them owns each command capability, the platform's DMS inbox and its DMS sends. A
// binding says it, for one service. It is the range's to make: it names a controller the World minted for one of the
// platform's Agents and creates no identity of its own. Ownership follows the bindings alone, never the order the
// Agents were made in.
//
// A platform the range binds nothing on behaves exactly as before.

#include "fsim/Capability.h"
#include "fsim/Export.h"

#include <cstdint>
#include <string>
#include <vector>

namespace fsim {

class World;

/// One MA service of a platform (a vehicle or a C2 node) and what it owns there.
struct MAServiceBindingV1 {
    std::uint64_t serviceId = 0;          ///< nonzero, unique in the world
    std::uint32_t platform = 0;           ///< the vehicle's or C2 node's entity id
    control::ControllerId controller = 0; ///< as World::createAgent minted it for this service's Agent (Agent::controller)
    /// The command capabilities it owns, each by its discovery id exactly as the platform reports it - a flight
    /// capability's ("fsim.guidance.hsa") or a Mission Systems capability instance's ("fsim.ms.amti:radar") - or the
    /// single entry "*": every command capability of the platform. A capability belongs to at most one service.
    std::vector<std::string> commandCapabilities;
    bool ownsDmsIngress = false; ///< its system's inbox and subscriptions: at most one service of a platform
    bool ownsDmsEgress = false;  ///< its sends, native publications, task answers and C2 messages: at most one
};

/// Bind one MA service. The range's: an Agent cannot reach it. False - `error`, if given, saying why - and nothing
/// changed for a controller this world did not mint or minted for another platform, a platform that is gone, a
/// capability the platform does not offer for command, and anything a service bound earlier already owns.
///
/// A platform's first binding puts it under control::ControlMode::Granted, the existing mode in which only a grant's
/// holder commands a capability, and asks for the grants of what each binding names. From then on:
///  - an Agent of that platform is granted only a capability bound to it (Agent::requestControl of another's:
///    NotAllowed), so only its owner commands it, and one service does not update or cancel another's activity;
///  - a C2 control designation's grants, a C2 override applied to a live HSA and a lost-communication plan activation
///    go to the bound owners, not to the Agent made last;
///  - the platform's inbox and subscriptions answer its ingress owner alone, and its sends - free-form, native
///    publications, task answers, C2 messages - its egress owner alone (comm::DmsReason::NotOwner for another);
///  - every Agent of the platform still reads the platform's reports alike: sharing a report grants no command.
/// Removing the platform ends its bindings.
FSIM_API bool bindMAService(World& world, const MAServiceBindingV1& binding, std::string* error = nullptr);

/// Every binding of the world, in ascending service id. The range's.
FSIM_API std::vector<MAServiceBindingV1> maServiceBindings(World& world);

/// The controller that owns a command capability of a platform: 0 where no bound service does, or the platform has
/// no binding. The range's.
FSIM_API control::ControllerId maCapabilityOwner(World& world, std::uint32_t platform, const std::string& capability);

} // namespace fsim
