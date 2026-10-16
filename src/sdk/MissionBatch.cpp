// Batches of mission episodes (fsim/MissionBatch.h; docs/sdk/mission-training.md, section 18; M7 core).
//
// One public MissionEpisode per slot, nothing shared between slots: each has its own configuration, World, judge,
// gateway and seed streams, and its own count of local episodes. Slots are stepped side by side by `workers` threads,
// each slot by one thread at a time; which thread steps which slot decides nothing.
//
// One call at a time holds the whole batch - its slots' episodes, statuses, reset counts, pending resets and final
// documents - for the whole operation: a call that finds it held fails busy at once (mission::Busy: FSIM_ERROR, and
// fsim.Error in Python, where reset, reset_slot and step release the GIL) and changes and reads nothing, so no reader
// sees a status, a final document or an endpoint between two halves of a step or a reset.
//
// A step or reset also holds every slot's episode (env::EpisodeHold), all of them before any changes: an endpoint or
// referee opened earlier takes only its episode's boundary, and while one of its calls holds a slot's episode the
// batch call fails busy whole, nothing changed. Under those holds each slot's transition is one operation no other
// call enters - the native step, its outcome, the ended generation's final documents and its autoreset - and is
// committed to the slot's status and final documents once read whole. A SameStep reset that fails after its episode
// ended leaves the slot explicitly ended with its reset pending: the next step resets it first, as NextStep does.

#include "fsim/MissionBatch.h"

#include "env/MissionEpisode.h"
#include "sdk/MissionConfig.h"
#include "sdk/MissionEpisodeImpl.h"
#include "sdk/MissionGateway.h"
#include "sdk/MissionJson.h"

#include <algorithm>
#include <atomic>
#include <exception>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

namespace fsim {

namespace {

std::uint64_t idMember(const core::Json& document, const char* name) {
    const core::Json* v = document.find(name);
    if (!v || !v->isString()) throw std::runtime_error(std::string("mission batch: no ") + name);
    return *mission::decimal(v->asString(), ~std::uint64_t{0});
}

/// A native episode refusal as the public episode reports it (MissionEpisode.cpp, section 5).
[[noreturn]] void refused(env::EpisodeStatus status, const std::string& call) {
    if (status == env::EpisodeStatus::Busy) throw mission::Busy("mission batch: " + call + ": busy - another call holds the episode");
    if (status == env::EpisodeStatus::NotReset) throw std::runtime_error("mission batch: " + call + ": the episode has no World (not reset)");
    throw std::runtime_error("mission batch: " + call + ": the episode is closed");
}

} // namespace

struct MissionBatch::Impl {
    struct Slot {
        std::shared_ptr<const mission::Config> config;
        std::unique_ptr<MissionEpisode> episode;
        std::uint64_t resets = 0; ///< generations begun in this slot
        MissionSlotStatus status;
        bool begun = false;        ///< reset at least once
        bool pendingReset = false; ///< an episode ended (NextStep, or a SameStep reset that failed): the next step resets first
        std::map<std::uint64_t, std::string> finalObservations;
        std::string finalOutcome, finalTruth, finalCosts;
    };

    MissionBatchOptions options;
    std::vector<Slot> slots;
    std::atomic<bool> held{false}; ///< a call holds the batch

    /// The batch for one call, or mission::Busy without waiting.
    class Hold {
    public:
        Hold(Impl& m, const char* call) : m_(m) {
            bool expected = false;
            if (!m.held.compare_exchange_strong(expected, true, std::memory_order_acquire))
                throw mission::Busy(std::string("mission batch: ") + call + ": busy - another call holds the batch");
        }
        ~Hold() { m_.held.store(false, std::memory_order_release); }
        Hold(const Hold&) = delete;
        Hold& operator=(const Hold&) = delete;

    private:
        Impl& m_;
    };

    /// One final document of a slot, read under the batch's hold.
    std::string finalDocument(std::size_t i, std::string Slot::*member, const char* call) {
        const Hold hold(*this, call);
        return slot(i).*member;
    }

    Slot& slot(std::size_t i) {
        if (i >= slots.size()) throw std::invalid_argument("mission batch: no slot " + std::to_string(i));
        return slots[i];
    }

    /// f(slot index) for every slot, `workers` at a time; the first failure (by slot) is rethrown after all have run.
    template <typename F>
    void each(F&& f) {
        const std::size_t n = slots.size(), threads = std::min<std::size_t>(std::max(1u, options.workers), n);
        std::vector<std::exception_ptr> failures(n);
        std::atomic<std::size_t> next{0};
        const auto work = [&] {
            for (std::size_t i = next++; i < n; i = next++) {
                try {
                    f(i);
                } catch (...) {
                    failures[i] = std::current_exception();
                }
            }
        };
        std::vector<std::thread> pool;
        for (std::size_t t = 1; t < threads; ++t) pool.emplace_back(work);
        work();
        for (std::thread& t : pool) t.join();
        for (const std::exception_ptr& e : failures)
            if (e) std::rethrow_exception(e);
    }

    /// The native episode behind a slot (MissionBatch is MissionEpisode's friend).
    static env::MissionEpisode& native(Slot& s) { return s.episode->impl_->episode; }

    /// Every slot's episode held for one operation, each boundary taken at once in slot order - or mission::Busy with
    /// none held and nothing changed: a call of an endpoint or referee opened earlier holds a slot's episode.
    std::vector<std::unique_ptr<env::EpisodeHold>> holdAll(const std::string& call) {
        std::vector<std::unique_ptr<env::EpisodeHold>> holds;
        holds.reserve(slots.size());
        for (std::size_t i = 0; i < slots.size(); ++i) {
            holds.push_back(std::make_unique<env::EpisodeHold>(native(slots[i])));
            if (!holds.back()->held())
                throw mission::Busy("mission batch: " + call + ": busy - another call holds slot " + std::to_string(i) +
                                    "'s episode (an endpoint's or a referee's)");
        }
        return holds;
    }

    /// A fresh generation of one slot at its next local episode index, under its hold.
    void resetOne(Slot& s, env::EpisodeHold& hold) {
        const std::uint64_t local = options.firstLocalEpisodeIndex + s.resets;
        std::string key;
        try {
            hold.reset(local, mission::resetHooks(s.episode->impl_->minting, local, key));
        } catch (const env::EpisodeError& e) {
            if (e.status() == env::EpisodeStatus::Busy) refused(e.status(), "reset");
            throw std::runtime_error(e.what());
        }
        ++s.resets;
        const core::Json k = mission::parseStrict(key, "episode key");
        MissionSlotStatus status;
        status.generation = idMember(k, "episode_generation");
        status.localEpisodeIndex = local;
        status.reset = true;
        s.status = status;
        s.begun = true, s.pendingReset = false;
    }

    /// One decision tick of one slot, under its hold: the native step, its outcome and - when its episode ends - the
    /// ended generation's final documents, all read before any of the slot's metadata changes; then its autoreset.
    void stepOne(Slot& s, env::EpisodeHold& hold) {
        if (s.pendingReset) {
            // the ended generation does nothing more - no request of it is dispatched, no World of it steps
            resetOne(s, hold);
            return;
        }
        if (s.status.episodeDone) { // Disabled: the ended generation holds - nothing of it is dispatched or stepped
            s.status.ended = s.status.reset = false;
            return;
        }
        if (const env::EpisodeStatus status = hold.step(); status != env::EpisodeStatus::Ok) refused(status, "step");
        // The tick is closed natively. Everything below reads the generation's gateway under the same hold: no other
        // call can enter, so nothing here fails busy, and the slot's metadata changes only once all of it is read.
        auto* gateway = dynamic_cast<mission::Gateway*>(hold.judge());
        if (!gateway) throw std::runtime_error("mission batch: step: the generation has no gateway");
        const core::Json outcome = mission::parseStrict(gateway->outcome(), "ma.outcome.v1");
        const core::Json* d = outcome.find("episode_done");
        const bool done = d && d->isBool() && d->asBool();
        MissionSlotStatus status = s.status;
        ++status.tick;
        status.episodeDone = status.ended = done;
        status.reset = false;
        if (!done) {
            s.status = status;
            return;
        }
        std::map<std::uint64_t, std::string> observations;
        for (const mission::ServiceConfig& service : s.config->services) observations[service.serviceId] = gateway->observation(service.serviceId);
        std::string finalOutcome = gateway->outcome(), finalTruth = gateway->truth(), finalCosts = gateway->costs();
        // commit: the ended generation's status and final documents, by principal, and its reset pending
        s.status = status;
        s.finalObservations = std::move(observations);
        s.finalOutcome = std::move(finalOutcome), s.finalTruth = std::move(finalTruth), s.finalCosts = std::move(finalCosts);
        s.pendingReset = options.autoreset != MissionAutoreset::Disabled;
        if (options.autoreset == MissionAutoreset::SameStep) {
            resetOne(s, hold); // (a failure leaves the slot ended, its reset pending for the next step)
            s.status.ended = true;
        }
    }
};

MissionBatch::MissionBatch(const std::vector<std::string>& configsJsonV1, const MissionBatchOptions& options) {
    if (configsJsonV1.empty()) throw std::invalid_argument("mission batch: no configuration");
    if (options.workers < 1) throw std::invalid_argument("mission batch: workers is at least 1");
    if (options.autoreset != MissionAutoreset::Disabled && options.autoreset != MissionAutoreset::NextStep &&
        options.autoreset != MissionAutoreset::SameStep)
        throw std::invalid_argument("mission batch: an autoreset mode it does not have");
    auto impl = std::make_unique<Impl>();
    impl->options = options;
    std::set<std::uint64_t> envIds;
    for (std::size_t i = 0; i < configsJsonV1.size(); ++i) {
        Impl::Slot s;
        s.config = mission::parseConfig(configsJsonV1[i]);
        if (!envIds.insert(s.config->envId).second)
            throw std::invalid_argument("mission batch: slot " + std::to_string(i) + " repeats env_id " + std::to_string(s.config->envId) +
                                        " - each slot's random streams are its own");
        s.episode = std::make_unique<MissionEpisode>(configsJsonV1[i]);
        impl->slots.push_back(std::move(s));
    }
    impl_ = impl.release();
}

MissionBatch::~MissionBatch() { delete impl_; }
MissionBatch::MissionBatch(MissionBatch&& other) noexcept : impl_(other.impl_) { other.impl_ = nullptr; }
MissionBatch& MissionBatch::operator=(MissionBatch&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = other.impl_;
        other.impl_ = nullptr;
    }
    return *this;
}

// (the live batch, inside a member: a moved-from batch answers nothing)
#define FSIM_MISSION_BATCH_LIVE (impl_ ? *impl_ : throw std::runtime_error("mission batch: moved from"))

std::size_t MissionBatch::size() const { return FSIM_MISSION_BATCH_LIVE.slots.size(); }

void MissionBatch::reset() {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "reset");
    auto holds = m.holdAll("reset"); // (every slot's episode, before any resets)
    m.each([&m, &holds](std::size_t i) { m.resetOne(m.slots[i], *holds[i]); });
}

void MissionBatch::resetSlot(std::size_t slot) {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "reset_slot");
    Impl::Slot& s = m.slot(slot);
    env::EpisodeHold episode(Impl::native(s));
    if (!episode.held()) refused(env::EpisodeStatus::Busy, "reset_slot");
    m.resetOne(s, episode);
}

void MissionBatch::step() {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "step");
    for (std::size_t i = 0; i < m.slots.size(); ++i) // (before any slot steps: a refused step changes nothing)
        if (!m.slots[i].begun) throw std::runtime_error("mission batch: step: slot " + std::to_string(i) + " is not reset");
    auto holds = m.holdAll("step"); // (every slot's episode, before any steps)
    m.each([&m, &holds](std::size_t i) { m.stepOne(m.slots[i], *holds[i]); });
}

MissionSlotStatus MissionBatch::status(std::size_t slot) const {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "status");
    return m.slot(slot).status;
}

MissionEndpoint MissionBatch::endpoint(std::size_t slot, std::uint64_t serviceId) {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "endpoint");
    return m.slot(slot).episode->endpoint(serviceId);
}

MissionReferee MissionBatch::referee(std::size_t slot) {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "referee");
    return m.slot(slot).episode->referee();
}

std::string MissionBatch::finalObservationJsonV1(std::size_t slot, std::uint64_t serviceId) const {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "final observation");
    Impl::Slot& s = m.slot(slot);
    if (!s.config->service(serviceId)) throw std::invalid_argument("mission batch: slot " + std::to_string(slot) + " has no service " + std::to_string(serviceId));
    const auto it = s.finalObservations.find(serviceId);
    return it == s.finalObservations.end() ? std::string() : it->second;
}

std::string MissionBatch::finalOutcomeJsonV1(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.finalDocument(slot, &Impl::Slot::finalOutcome, "final outcome"); }

std::string MissionBatch::finalTruthJsonV1(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.finalDocument(slot, &Impl::Slot::finalTruth, "final truth"); }

std::string MissionBatch::finalCostsJsonV1(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.finalDocument(slot, &Impl::Slot::finalCosts, "final costs"); }

bool MissionBatch::removePlatform(std::size_t slot, std::string_view platform) {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    const Impl::Hold hold(m, "remove_platform");
    return m.slot(slot).episode->removePlatform(platform);
}

} // namespace fsim
