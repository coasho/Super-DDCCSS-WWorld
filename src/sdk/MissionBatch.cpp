// Batches of mission episodes (fsim/MissionBatch.h; docs/sdk/mission-training.md, section 18; M7 core).
//
// One public MissionEpisode per slot, nothing shared between slots: each has its own configuration, World, judge,
// gateway and seed streams, and its own count of local episodes. Slots are stepped side by side by `workers` threads,
// each slot by one thread at a time; which thread steps which slot decides nothing.

#include "fsim/MissionBatch.h"

#include "sdk/MissionConfig.h"
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

} // namespace

struct MissionBatch::Impl {
    struct Slot {
        std::shared_ptr<const mission::Config> config;
        std::unique_ptr<MissionEpisode> episode;
        std::uint64_t resets = 0; ///< generations begun in this slot
        MissionSlotStatus status;
        bool begun = false;        ///< reset at least once
        bool pendingReset = false; ///< NextStep: an episode ended; the next step resets first
        std::map<std::uint64_t, std::string> finalObservations;
        std::string finalOutcome, finalTruth, finalCosts;
    };

    MissionBatchOptions options;
    std::vector<Slot> slots;

    Slot& slot(std::size_t i) {
        if (i >= slots.size()) throw std::invalid_argument("mission batch: no slot " + std::to_string(i));
        return slots[i];
    }

    /// f(slot) for every slot, `workers` at a time; the first failure (by slot) is rethrown after all have run.
    template <typename F>
    void each(F&& f) {
        const std::size_t n = slots.size(), threads = std::min<std::size_t>(std::max(1u, options.workers), n);
        std::vector<std::exception_ptr> failures(n);
        std::atomic<std::size_t> next{0};
        const auto work = [&] {
            for (std::size_t i = next++; i < n; i = next++) {
                try {
                    f(slots[i]);
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

    /// A fresh generation of one slot at its next local episode index.
    void resetOne(Slot& s) {
        const std::uint64_t local = options.firstLocalEpisodeIndex + s.resets;
        const std::string key = s.episode->reset(local);
        ++s.resets;
        const core::Json k = mission::parseStrict(key, "episode key");
        s.status = MissionSlotStatus{};
        s.status.generation = idMember(k, "episode_generation");
        s.status.localEpisodeIndex = local;
        s.status.reset = true;
        s.begun = true, s.pendingReset = false;
    }

    /// The ended generation's final documents, by principal: each service's last snapshot; the referee's outcome, costs.
    void keepFinals(Slot& s) {
        s.finalObservations.clear();
        for (const mission::ServiceConfig& service : s.config->services)
            s.finalObservations[service.serviceId] = s.episode->endpoint(service.serviceId).observationJsonV1();
        MissionReferee referee = s.episode->referee();
        s.finalOutcome = referee.outcomeJsonV1();
        s.finalTruth = referee.truthJsonV1();
        s.finalCosts = referee.costsJsonV1();
    }

    void stepOne(Slot& s) {
        if (!s.begun) throw std::runtime_error("mission batch: a slot is not reset");
        s.status.ended = s.status.reset = false;
        if (s.pendingReset) {
            // NextStep: the ended generation does nothing more - no request of it is dispatched, no World of it steps
            resetOne(s);
            return;
        }
        if (s.status.episodeDone) return; // Disabled: the ended generation holds - nothing of it is dispatched or stepped
        s.episode->step();
        ++s.status.tick;
        const core::Json outcome = mission::parseStrict(s.episode->referee().outcomeJsonV1(), "ma.outcome.v1");
        const bool done = outcome.find("episode_done") && outcome.find("episode_done")->isBool() && outcome.find("episode_done")->asBool();
        const bool ended = done && !s.status.episodeDone;
        s.status.episodeDone = done;
        if (!ended) return;
        s.status.ended = true;
        keepFinals(s);
        if (options.autoreset == MissionAutoreset::NextStep) {
            s.pendingReset = true;
        } else if (options.autoreset == MissionAutoreset::SameStep) {
            resetOne(s);
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
    m.each([&m](Impl::Slot& s) { m.resetOne(s); });
}

void MissionBatch::resetSlot(std::size_t slot) {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    m.resetOne(m.slot(slot));
}

void MissionBatch::step() {
    Impl& m = FSIM_MISSION_BATCH_LIVE;
    m.each([&m](Impl::Slot& s) { m.stepOne(s); });
}

MissionSlotStatus MissionBatch::status(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.slot(slot).status; }

MissionEndpoint MissionBatch::endpoint(std::size_t slot, std::uint64_t serviceId) { return FSIM_MISSION_BATCH_LIVE.slot(slot).episode->endpoint(serviceId); }

MissionReferee MissionBatch::referee(std::size_t slot) { return FSIM_MISSION_BATCH_LIVE.slot(slot).episode->referee(); }

std::string MissionBatch::finalObservationJsonV1(std::size_t slot, std::uint64_t serviceId) const {
    Impl::Slot& s = FSIM_MISSION_BATCH_LIVE.slot(slot);
    if (!s.config->service(serviceId)) throw std::invalid_argument("mission batch: slot " + std::to_string(slot) + " has no service " + std::to_string(serviceId));
    const auto it = s.finalObservations.find(serviceId);
    return it == s.finalObservations.end() ? std::string() : it->second;
}

std::string MissionBatch::finalOutcomeJsonV1(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.slot(slot).finalOutcome; }

std::string MissionBatch::finalTruthJsonV1(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.slot(slot).finalTruth; }

std::string MissionBatch::finalCostsJsonV1(std::size_t slot) const { return FSIM_MISSION_BATCH_LIVE.slot(slot).finalCosts; }

bool MissionBatch::removePlatform(std::size_t slot, std::string_view platform) { return FSIM_MISSION_BATCH_LIVE.slot(slot).episode->removePlatform(platform); }

} // namespace fsim
