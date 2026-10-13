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

class Agent;
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
/// A binding accepted, the platform is consistent with every binding made on it so far:
///  - it is under control::ControlMode::Granted, the existing mode in which only a grant's holder commands a
///    capability, and stays there (Vehicle::setControlMode to another: NotAllowed);
///  - each command capability's grant is its bound owner's or no one's, and an unowned capability's is no one's;
///  - no live or waiting policy activity is a controller's that does not own its capability.
/// What stood in the way before the binding - a grant another controller held, the default policy's (controller 0)
/// among them, or a policy activity flown without one while the platform was open - is taken back as the platform
/// takes anything back: the grant revoked, that capability's policy activities ended Canceled(Revoked), flight and
/// Mission Systems alike. A revoke is of a capability, not of a controller: it ends every policy activity of that
/// capability, the newly bound owner's own among them. So the owner's activities go on under its grant only on a
/// capability no other controller held a grant of or had a live or waiting policy activity on; where one did, the
/// owner's activities of that capability end Canceled(Revoked) with the other's, and the owner, granted, commands
/// anew. A capability the platform does not allow or cannot give at that moment stays ungranted, its owner's to
/// request later. Bind every service of a platform before any of them acts, and nothing is taken from anyone. From
/// then on:
///  - only a capability's owner is granted it (Agent::requestControl or Vehicle::requestControl by any other
///    controller: NotAllowed), so only its owner commands it, and one service does not update or cancel another's
///    activity; the platform's own sources (autopilot, override) need no grant, as ever;
///  - a C2 control designation's grants, a C2 override applied to a live HSA and a lost-communication plan activation
///    go to the bound owners, not to the Agent made last;
///  - the platform's inbox and subscriptions answer its ingress owner alone, and its sends - free-form, native
///    publications, task answers, C2 messages - its egress owner alone (comm::DmsReason::NotOwner for another);
///  - every Agent of the platform still reads the platform's reports alike: sharing a report grants no command.
/// Removing the platform ends its bindings.
FSIM_API bool bindMAService(World& world, const MAServiceBindingV1& binding, std::string* error = nullptr);

/// The same binding for the service whose Agent is `agent`: its platform and its controller are the Agent's own. Two
/// Worlds mint the same vehicle ids and controller numbers, so the numbers of an Agent another World made would name
/// an Agent of this one; this form refuses an Agent this World did not make (and an empty one), nothing changed.
/// Prefer it wherever the Agent itself is at hand.
FSIM_API bool bindMAService(World& world, const Agent& agent, std::uint64_t serviceId, const std::vector<std::string>& commandCapabilities,
                            bool ownsDmsIngress, bool ownsDmsEgress, std::string* error = nullptr);

/// Every binding of the world, in ascending service id. The range's.
FSIM_API std::vector<MAServiceBindingV1> maServiceBindings(World& world);

/// The controller that owns a command capability of a platform: 0 where no bound service does, or the platform has
/// no binding. The range's.
FSIM_API control::ControllerId maCapabilityOwner(World& world, std::uint32_t platform, const std::string& capability);

} // namespace fsim
