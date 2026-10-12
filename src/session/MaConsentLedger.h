#pragma once

// Private MA requests whose approving senders may still answer. The DMS owns
// the separate, publicly readable consent history.
#include "fsim/Capability.h"
#include "fsim/Comm.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace fsim::session {

class MaConsentLedger {
public:
    struct Request {
        control::ActivityId strike = 0;
        std::uint64_t requestId = 0;
        std::vector<comm::SystemId> asked;
    };

    const Request* findStrike(control::ActivityId strike) const noexcept {
        const auto it = std::find_if(requests_.begin(), requests_.end(),
                                     [&](const Request& request) { return request.strike == strike; });
        return it == requests_.end() ? nullptr : &*it;
    }

    const Request* findRequest(std::uint64_t requestId) const noexcept {
        const auto it = std::find_if(requests_.begin(), requests_.end(),
                                     [&](const Request& request) { return request.requestId == requestId; });
        return it == requests_.end() ? nullptr : &*it;
    }

    void record(Request request) {
        if (request.asked.empty()) return; // Automatic policies have no approving sender to authorize.
        const auto it = std::find_if(requests_.begin(), requests_.end(),
                                     [&](const Request& held) { return held.strike == request.strike; });
        if (it != requests_.end()) *it = std::move(request);
        else requests_.push_back(std::move(request));
    }

    bool complete(std::uint64_t requestId) noexcept {
        const auto it = std::find_if(requests_.begin(), requests_.end(),
                                     [&](const Request& request) { return request.requestId == requestId; });
        if (it == requests_.end()) return false;
        requests_.erase(it); // First answer retires every approver; later answers are unauthorized.
        return true;
    }

    void release() noexcept { std::vector<Request>().swap(requests_); }

    std::size_t size() const noexcept { return requests_.size(); }
    std::size_t capacity() const noexcept { return requests_.capacity(); }

private:
    std::vector<Request> requests_;
};

} // namespace fsim::session
