#pragma once

// Systems, nets and each system's Decentralized Messaging Service (ADR-31, AB-4; A-GRA MS 1.2.5): what one platform or
// C2 node may tell another, and how it gets there. The world owns one; the range makes nets and puts systems on them;
// a system's DMS sends - published on a topic, or reliably to given systems - and receives, stamped by the sender's
// SystemID. Delivery reads truth (positions) as physics does; nothing of it reaches an agent but what arrives.

#include "fsim/Comm.h"
#include "fsim/GroundProvider.h"
#include "fsim/Rng.h"

#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsim::session {

class Dms {
public:
    /// Where an entity is now (truth: the delivery's range and line of sight); false for one gone.
    class Locator {
    public:
        virtual ~Locator() = default;
        virtual bool locate(std::uint32_t entity, double& latitudeRad, double& longitudeRad, double& heightM) const = 0;
    };

    /// No net: nothing to step (a world without nets steps as before).
    bool active() const noexcept { return !nets_.empty(); }

    // --- the range's -----------------------------------------------------------------------------------------------
    comm::NetId createNet(const comm::NetSpec& spec);
    /// Put an entity (a vehicle or a C2 node) on a net; its system is assigned the next free SystemID if it has none.
    bool join(std::uint32_t entity, comm::NetId net);
    bool leave(std::uint32_t entity, comm::NetId net);
    /// An entity gone: off every net, its waiting sends failed; its SystemID stays the referee's to map.
    void removeEntity(std::uint32_t entity);
    /// Assign an entity's SystemID (`want` 0: the next free) and tail number: its id, or 0 where `want` is another
    /// system's or the entity already has another.
    comm::SystemId assign(std::uint32_t entity, comm::SystemId want, const std::string& tailNumber);

    /// A system's Data Dissemination Plan (AB-4b): what its MA core publishes on its own, how long its picture keeps
    /// what it heard. False for an entity with no system.
    bool setPlan(std::uint32_t entity, const comm::DisseminationPlan& plan, double now);
    comm::DisseminationPlan plan(std::uint32_t entity) const noexcept;
    /// What each system's plan has due now (its next times moved on): the world builds them from its platform's reports.
    struct Due {
        std::uint32_t entity = 0;
        bool position = false, status = false, tracks = false;
    };
    std::vector<Due> due(double now);

    // --- a system's (its agent's, through its own handle) -----------------------------------------------------------
    comm::SystemId systemOf(std::uint32_t entity) const noexcept;
    std::vector<comm::NetInfo> netsOf(std::uint32_t entity) const;
    /// Send as the entity's system: its message number, or 0 refused (`why`). `native`: the MA core's, a native type.
    std::uint64_t send(std::uint32_t entity, const comm::DmsMessage& message, double now, comm::DmsReason& why, bool native = false);
    bool subscribe(std::uint32_t entity, comm::Topic topic, bool on);
    bool subscribeAll(std::uint32_t entity, bool on);
    std::vector<comm::Topic> subscriptions(std::uint32_t entity) const;
    bool subscribedAll(std::uint32_t entity) const noexcept;
    /// What arrived and waits to be taken (at most comm::kInboxCapacity: the oldest dropped first), oldest first.
    const std::deque<comm::DmsReceived>* inbox(std::uint32_t entity) const noexcept;
    /// Take the first `count` waiting (all: SIZE_MAX).
    std::vector<comm::DmsReceived> take(std::uint32_t entity, std::size_t count = static_cast<std::size_t>(-1));
    std::optional<comm::DmsTransmission> transmission(std::uint32_t entity, std::uint64_t message) const;
    /// Its picture (AB-4b): each peer it heard - its last position report and heartbeat - and each track received.
    std::vector<comm::PeerReport> peers(std::uint32_t entity) const;
    /// How each of a peer's two reports arrived (docs/sdk/mission-training.md, section 10). comm::PeerReport has one
    /// `net` and one `receivedS`, those of whichever message was heard last; a position delivered over one net and a
    /// later heartbeat over another are two deliveries, kept apart here.
    struct PeerProvenance {
        comm::NetId positionNet = 0;   ///< the net its held position report came by (0: none held)
        double positionReceivedS = std::numeric_limits<double>::quiet_NaN();
        bool heartbeat = false;        ///< a heartbeat was heard
        comm::NetId heartbeatNet = 0;  ///< the net its last heartbeat came by
        double heartbeatReceivedS = std::numeric_limits<double>::quiet_NaN();
        double heartbeatTimeS = std::numeric_limits<double>::quiet_NaN(); ///< the time that heartbeat states
    };
    /// Beside peers(entity), index for index.
    std::vector<PeerProvenance> peerProvenance(std::uint32_t entity) const;
    std::vector<comm::LinkTrack> linkTracks(std::uint32_t entity) const;
    /// What arrived for its MA core (topics 10 to 49: C2's commands, AB-5) - taken by the core, never the agent's inbox.
    std::vector<comm::DmsReceived> takeCore(std::uint32_t entity);
    /// Every entity with a system on a net now.
    std::vector<std::uint32_t> entities() const;
    /// The entities whose MA core has messages waiting.
    std::vector<std::uint32_t> coreWaiting() const;
    /// When it last heard anything from a system (AB-5c: lost communication), NaN never.
    double lastHeard(std::uint32_t entity, comm::SystemId from) const noexcept;
    /// The strikes' consent requests it received (AB-5b), answered or not; its own to mark answered.
    std::vector<comm::PendingConsent>* consentsOf(std::uint32_t entity) noexcept;
    /// The last status of each C2 command it sent, by the MA that answered (AB-5).
    const std::vector<comm::CommandStatusEntry>* commandStatuses(std::uint32_t entity) const noexcept;
    /// The same in place (the C ABI's): valid until the next step.
    const std::vector<comm::PeerReport>* peersOf(std::uint32_t entity) const noexcept;
    const std::vector<comm::LinkTrack>* linkTracksOf(std::uint32_t entity) const noexcept;
    const comm::LinkTrack* linkTrack(std::uint32_t entity, comm::SystemId source, std::uint32_t payload, std::uint64_t track) const noexcept;

    // --- the referee's ---------------------------------------------------------------------------------------------
    std::uint32_t entityOf(comm::SystemId system) const noexcept;
    std::string tailNumber(comm::SystemId system) const;
    std::vector<comm::NetInfo> nets() const;
    std::size_t netCount() const noexcept { return nets_.size(); }
    /// A net's info in place: it stays where it is (its name for the C ABI) for the world's life.
    const comm::NetInfo* netInfo(comm::NetId net) const noexcept { return net >= 1 && net <= nets_.size() ? &nets_[net - 1].info : nullptr; }
    comm::NetStats stats(comm::NetId net) const noexcept;

    /// Route what was sent, deliver what is due by `now`: the world's step, after the flight.
    void step(double now, const Locator& where, const sim::GroundProvider* ground, Rng& rng);

private:
    /// A send as it travels: shared by its copies in flight.
    struct Outgoing {
        std::size_t sender = 0; ///< its system's index
        std::uint64_t id = 0;
        comm::DmsMessage message;
        double timeSent = 0.0;
        bool routed = false;    ///< (published) its one attempt made
        struct Dest {
            comm::SystemId system = comm::kNoSystem;
            comm::Delivery state = comm::Delivery::Pending;
            double nextAttempt = 0.0;
            bool inFlight = false;
        };
        std::vector<Dest> dests;                 ///< reliable: each destination
        std::vector<comm::SystemId> deliveredTo; ///< first arrivals (later copies dropped)
        std::uint32_t inFlight = 0;
        bool queueDropped = false;
    };
    struct InFlight {
        std::shared_ptr<Outgoing> message;
        std::size_t receiver = 0;
        comm::NetId net = 0;
        double deliverAt = 0.0;
    };
    struct System {
        comm::SystemId id = comm::kNoSystem;
        std::uint32_t entity = 0;
        std::string tail;
        bool present = true;
        std::vector<comm::NetId> nets;
        std::vector<double> busyUntil;           ///< per net in `nets`: when its share is free
        bool all = true;
        std::vector<comm::Topic> topics;
        std::deque<comm::DmsReceived> inbox;
        std::uint64_t nextMessage = 1;
        std::vector<comm::DmsTransmission> sent; ///< its last sends' statuses (bounded)
        comm::DisseminationPlan plan;
        double nextPosition = 0.0, nextStatus = 0.0, nextTracks = 0.0;
        std::vector<comm::PeerReport> peers;     ///< its picture (AB-4b)
        std::vector<PeerProvenance> peerFrom;    ///< beside `peers`, index for index: how each one's reports arrived
        std::vector<comm::LinkTrack> linkTracks;
        std::deque<comm::DmsReceived> core;              ///< for its MA core (AB-5)
        std::vector<comm::CommandStatusEntry> commands;  ///< its commands' last statuses (AB-5)
        std::vector<comm::PendingConsent> consents;      ///< consent requests it received (AB-5b)
        std::vector<std::pair<comm::SystemId, double>> heard; ///< when it last heard each sender (AB-5c)
    };
    /// A native message delivered: into the receiver's picture.
    void picture(System& to, const comm::DmsReceived& r);
    struct Net {
        comm::NetInfo info;
        std::vector<std::size_t> members; ///< system indices
        comm::NetStats stats;
    };
    struct PathKey {
        std::uint32_t a, b;
        double frequencyHz;
        bool operator==(const PathKey& o) const noexcept { return a == o.a && b == o.b && frequencyHz == o.frequencyHz; }
    };
    struct PathKeyHash {
        std::size_t operator()(const PathKey& k) const noexcept {
            return std::hash<std::uint64_t>()((std::uint64_t(k.a) << 32) ^ k.b) ^ std::hash<double>()(k.frequencyHz);
        }
    };
    struct Sight {
        double timeS = -1e300;
        bool clear = true;
    };

    System* system(std::uint32_t entity) noexcept;
    const System* system(std::uint32_t entity) const noexcept;
    std::size_t index(std::uint32_t entity) const noexcept;
    Net* net(comm::NetId id) noexcept;
    bool subscribed(const System& s, comm::Topic topic) const noexcept;
    comm::DmsTransmission* status(System& s, std::uint64_t id) noexcept;
    /// One transmission by `sender` on its i-th net, asked for at `at`: when it ends, or a negative number when its
    /// share could not take it in time.
    double transmit(System& sender, std::size_t i, const Outgoing& m, double at);
    /// Can `net` carry a message from one system to another now (range, line of sight, loss)? Counts why not.
    bool closes(Net& net, const System& from, const System& to, double now, const Locator& where, const sim::GroundProvider* ground, Rng& rng);
    void deliver(const InFlight& f, double now);

    std::vector<System> systems_;
    std::unordered_map<std::uint32_t, std::size_t> byEntity_;
    std::unordered_map<comm::SystemId, std::size_t> bySystem_;
    std::deque<Net> nets_; ///< (a deque: each net stays in place)
    std::vector<std::shared_ptr<Outgoing>> outgoing_; ///< sent, not yet settled
    std::vector<InFlight> inflight_;
    std::unordered_map<PathKey, Sight, PathKeyHash> sight_;
    comm::SystemId nextSystem_ = 0x53000001u;
};

} // namespace fsim::session
