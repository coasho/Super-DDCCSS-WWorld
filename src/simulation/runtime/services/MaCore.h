#pragma once

// The MA core (ADR-31, AB-5): A-GRA's rules an MA must apply, for its agent - never the agent's decisions. It takes what
// C2 sends a platform over the links (its DMS's core queue), authorizes each message by its sender's declared role in
// the platform's RBAC configuration (C2 1.2.9), answers what is unauthorized and not acted on, applies control
// designations (1.2.7.23: forwarded to FA - the capabilities MA may command, granted to MA's controller) and executes
// direct flight commands (1.2.7.8-9) under MA's secondary control as C2's: while one flies, the agent's own commands
// on its axes are refused (authority_held; section 9 item 5). It reports each command's status back to its sender.

#include "fsim/Capability.h"
#include "fsim/Comm.h"
#include "simulation/runtime/services/MaConsentLedger.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsim::session {

class World;

class MaCore {
public:
    /// The controllers C2's direct commands fly under: one per C2 system, clear of agents' and programs' own.
    static constexpr control::ControllerId kC2Controllers = 0x30000000u;

    struct Operator {
        comm::SystemId system = comm::kNoSystem;
        std::uint32_t roleId = 0;
        std::string callsign;
    };
    struct C2Activity {
        control::ActivityId activity = 0;
        comm::SystemId system = comm::kNoSystem; ///< the C2 system whose command it flies
        std::uint64_t commandId = 0;
    };
    /// What an MA's agent may read of the authority over it: who declared which role, C2's designation (if any) and
    /// the direct commands C2 flies on it.
    struct Authority {
        std::vector<Operator> operators;
        bool designated = false;               ///< C2 has designated its flight modes
        std::uint32_t designatedModes = 0;     ///< 1 << control::FlightMode each
        comm::SystemId designatedBy = comm::kNoSystem;
        double designatedS = 0.0;
        std::vector<C2Activity> c2;
        // lost communication (AB-5c)
        bool lostCommsSet = false;
        comm::LostCommsSettings lostComms;
        comm::SystemId lostCommsBy = comm::kNoSystem;
        double lostCommsSetS = 0.0;
        bool lost = false;          ///< the contingency raised: nothing heard from C2 for the threshold
        double lostSinceS = 0.0;
        std::uint32_t alerts = 0;   ///< contingency alerts raised
        // its team (AB-5c)
        std::uint32_t packageId = 0;
        comm::SystemId leader = comm::kNoSystem;
        std::vector<comm::SystemId> members;
    };

    /// The platform's RBAC configuration (the range's: mission planning). A-GRA's defaults until it is set.
    void setRbac(std::uint32_t entity, const comm::RbacConfig& config) { state(entity).rbac = config; }
    comm::RbacConfig rbac(std::uint32_t entity) const;
    /// The same in place (the C ABI's names): the defaults kept for an entity not yet configured.
    const comm::RbacConfig& rbacOf(std::uint32_t entity) { return state(entity).rbac; }
    const Authority* authority(std::uint32_t entity) const noexcept;
    /// The C2 system an activity flies for (0: none - not a C2 command's).
    comm::SystemId c2Of(control::ActivityId activity) const noexcept;

    /// After the DMS's step: each platform's C2 messages handled, each C2 activity that ended reported, its strikes'
    /// release consent asked for as its policy says (AB-5b), expired overrides cleared.
    void step(World& world);
    /// A removed entity keeps its public records in place; only its private consent storage is released now.
    /// Its existing C2, override and lost-communication transitions still settle on their original steps.
    void retireEntity(std::uint32_t entity) noexcept;

    // AB-5b: tasks to the agent, overrides on its flight, release consent over the link
    const std::vector<comm::ReceivedTask>* tasks(std::uint32_t entity) const noexcept;
    /// The agent's answer to a task (accepted, rejected - `code` its reason) or its report (completed, failed, progress):
    /// sent to the C2 system that gave it. False for a task it does not hold, or one already ended.
    bool answerTask(World& world, std::uint32_t entity, comm::SystemId from, std::uint64_t taskId, comm::ReceivedTask::State state,
                    std::uint32_t code, double percent);
    const std::vector<comm::ActiveOverride>* overrides(std::uint32_t entity) const noexcept;
    bool hasOverrides(std::uint32_t entity) const noexcept;
    /// An agent's flight command as the overrides in force have it fly.
    void applyOverrides(std::uint32_t entity, control::HsaCommand& hsa) const noexcept;
    void applyOverrides(std::uint32_t entity, std::vector<control::Waypoint>& points) const noexcept;
    /// As a C2 system: a task, an override (or its cancel), an answer to a consent request it holds.
    std::uint64_t sendTask(World& world, std::uint32_t entity, comm::SystemId ma, comm::TaskOrder task, comm::DmsReason& why);
    std::uint64_t sendOverride(World& world, std::uint32_t entity, comm::SystemId ma, comm::OverrideCommand command, comm::DmsReason& why);
    bool answerConsent(World& world, std::uint32_t entity, comm::SystemId ma, std::uint64_t requestId, bool granted, comm::DmsReason& why);

    // A C2 system's sends (its agent's: C2 1.2.9.4, 1.2.7.23, 1.2.7.8-9) - typed, through its DMS; the command's id.
    std::uint64_t declareRole(World& world, std::uint32_t entity, std::uint32_t roleId, const std::string& callsign,
                              const std::vector<comm::SystemId>& destinations, comm::DmsReason& why);
    std::uint64_t designateControl(World& world, std::uint32_t entity, comm::SystemId ma, std::uint32_t modes, comm::DmsReason& why);
    std::uint64_t flightCommand(World& world, std::uint32_t entity, comm::SystemId ma, comm::FlightCommand command, comm::DmsReason& why);
    /// AB-5c, as a C2 system: the lost-communication contingency, a team (to each system it names; dissolved).
    std::uint64_t sendLostComms(World& world, std::uint32_t entity, comm::SystemId ma, comm::LostCommsSettings settings, comm::DmsReason& why);
    std::uint64_t sendPackage(World& world, std::uint32_t entity, comm::PackageCommand command, const std::vector<comm::SystemId>& to,
                              comm::DmsReason& why);
    /// A team formed - sent to its leader and each member but the sender - or dissolved (to `systems` but the sender).
    std::uint64_t formPackage(World& world, std::uint32_t entity, std::uint32_t packageId, comm::SystemId leader,
                              const std::vector<comm::SystemId>& members, comm::DmsReason& why);
    std::uint64_t dissolvePackage(World& world, std::uint32_t entity, std::uint32_t packageId, const std::vector<comm::SystemId>& systems,
                                  comm::DmsReason& why);

private:
    struct State {
        comm::RbacConfig rbac = comm::RbacConfig::agraDefaults();
        Authority authority;
        std::uint64_t nextCommand = 1; ///< (as a C2 system: its commands' ids)
        std::vector<comm::ReceivedTask> tasks;          ///< (AB-5b) C2's tasks for its agent
        std::vector<comm::ActiveOverride> overrides;
        MaConsentLedger consents;                     ///< unanswered requests, including ended strikes' late answers
        std::uint64_t nextRequest = 1;
        bool retired = false;
    };
    static bool needsMaintenance(const State& s) noexcept {
        return !s.retired || !s.overrides.empty() || !s.authority.c2.empty() || (s.authority.lostCommsSet && !s.authority.lost);
    }
    void consents(World& world, std::uint32_t entity, State& s);
    void watchComms(World& world, std::uint32_t entity, State& s);
    /// What a team's leader may send its members: anything but role configuration (Peer 1.1).
    static const comm::Role& leaderRole();
    void overrideLive(World& world, std::uint32_t entity, const State& s);
    State& state(std::uint32_t entity) { return states_[entity]; }
    control::ControllerId controllerFor(comm::SystemId system);
    const comm::Role* roleOf(const State& s, comm::SystemId sender) const noexcept;
    void handle(World& world, std::uint32_t entity, const comm::DmsReceived& m);
    void reply(World& world, std::uint32_t entity, comm::SystemId to, const comm::CommandStatus& status);
    void designate(World& world, std::uint32_t entity, State& s, comm::SystemId by, std::uint32_t modes);
    void fly(World& world, std::uint32_t entity, State& s, comm::SystemId by, const comm::FlightCommand& c);

    std::unordered_map<std::uint32_t, State> states_;
    std::unordered_map<comm::SystemId, control::ControllerId> controllers_;
};

} // namespace fsim::session
