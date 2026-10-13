#pragma once

// ADR-31 (docs/agent-boundary.md): what an agent's handle may do, in one place - the C ABI's agent handles and the C++
// fsim::Agent apply the same rules. An agent is bound to one vehicle and commands as a controller of its own: it may not
// claim another source or controller (L2), address another vehicle's activities, name another aircraft by its entity id
// (L4: a target by track comes with AB-2b), or read the truth labels in its reports (L3).

#include "fsim/Agent.h"
#include "fsim/Capability.h"
#include "fsim/Effects.h"
#include "fsim/Control.h"
#include "fsim/MissionSystems.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fsim::session {

class World;

struct AgentIdentity {
    std::uint32_t vehicle = 0;
    control::ControllerId controller = 0;
    std::string name;
    std::size_t record = 0; ///< its record among the world's agents (World::agents): where its audit is kept
};

/// The audit's calls (ADR-31, AB-2d): each entry point an agent calls has an index, process-wide, given on its first call
/// (its name: the C function's or the C++ Agent's member).
std::uint16_t auditCall(const char* name);
/// The names, by index.
std::vector<std::string> auditNames();

/// Its options as an agent's: the policy's source and its own controller. NotAllowed where they claim another
/// (a source above the policy's, a controller not 0 and not its own); 0 means its own.
control::Reason agentOptions(const AgentIdentity& agent, control::CommandOptions& options) noexcept;
ms::Reason agentOptions(const AgentIdentity& agent, ms::CommandOptions& options) noexcept;
/// A declared caller (UPDATE, CANCEL, an activity command) as the agent's, by the same rule.
control::Reason agentCaller(const AgentIdentity& agent, control::Caller& caller) noexcept;
/// A source and controller given as numbers (the C ABI's _as and _by forms), by the same rule.
bool agentCaller(const AgentIdentity& agent, int& source, std::uint32_t& controller) noexcept;
/// An activity of its own vehicle (a flight or a Mission Systems activity: the vehicle is in the id's high word).
bool agentActivity(const AgentIdentity& agent, std::uint64_t activity, const World* world = nullptr) noexcept;
/// A command that names no other aircraft by its entity id: a behaviour's target, a must fly's entity. NotAllowed where
/// it does. Its own vehicle may be named, and a target its own vehicle designated by track (`world`'s designations,
/// AB-2b).
control::Reason agentCommand(const AgentIdentity& agent, const control::Command& command, const World* world) noexcept;
control::Reason agentMustFly(const AgentIdentity& agent, const control::MustFlyCommand& mustFly, const World* world) noexcept;
/// A strike whose target is none or its own designation, and that asks for consent rather than giving it: release
/// consent is C2's (A-GRA's weapons authorization policy, A9; the range stands in for C2 until AB-5).
ms::Reason agentStrike(const AgentIdentity& agent, const ms::StrikeCommand& strike, const World* world) noexcept;
/// The DMS of a platform with configured MA services (docs/sdk/mission-training.md, section 9): its inbox and its
/// subscriptions are its ingress owner's, its sends - free-form, native publications, task answers, C2 messages - its
/// egress owner's. True for the owner, and for any agent of a platform the range bound no service on.
bool agentDmsIngress(const AgentIdentity& agent, const World& world) noexcept;
bool agentDmsEgress(const AgentIdentity& agent, const World& world) noexcept;
/// A report as an agent reads it: the entity behind a detection or a track is the referee's (0).
void agentReport(ms::AmtiDetection& detection) noexcept;
void agentReport(ms::AmtiTrack& track) noexcept;

/// Its own ship from its sensed state: what a platform reports of it.
OwnShip ownShipOf(const effects::SensedState& sensed) noexcept;

/// The message every refusal of an agent's handle gives (fsim_last_error, the C++ Agent's exceptions).
std::string notAgents(const char* call);

} // namespace fsim::session
