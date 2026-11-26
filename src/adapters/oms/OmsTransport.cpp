#include "adapters/oms/OmsTransport.h"

#include <algorithm>
#include <limits>

namespace fsim::oms {

const char* frameErrorName(FrameError err) noexcept {
    switch (err) {
    case FrameError::None: return "none";
    case FrameError::EmptyPayload: return "empty_payload";
    case FrameError::OversizedFrame: return "oversized_frame";
    case FrameError::MalformedFrame: return "malformed_frame";
    case FrameError::TransportClosed: return "transport_closed";
    }
    return "unknown";
}

bool encodeFrame(std::string_view payload,
                 FrameMode mode,
                 std::string& outBytes,
                 FrameError* error,
                 std::size_t maxFrameBytes) {
    auto fail = [&](FrameError err) {
        if (error) *error = err;
        return false;
    };
    if (payload.empty()) return fail(FrameError::EmptyPayload);
    if (payload.size() > maxFrameBytes) return fail(FrameError::OversizedFrame);

    outBytes.clear();
    if (mode == FrameMode::Ndjson) {
        if (payload.find('\n') != std::string_view::npos || payload.find('\r') != std::string_view::npos) {
            return fail(FrameError::MalformedFrame);
        }
        outBytes.reserve(payload.size() + 1);
        outBytes.append(payload.data(), payload.size());
        outBytes.push_back('\n');
    } else {
        if (payload.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
            return fail(FrameError::OversizedFrame);
        }
        const auto len = static_cast<std::uint32_t>(payload.size());
        outBytes.reserve(4 + payload.size());
        outBytes.push_back(static_cast<char>((len >> 24) & 0xFFu));
        outBytes.push_back(static_cast<char>((len >> 16) & 0xFFu));
        outBytes.push_back(static_cast<char>((len >> 8) & 0xFFu));
        outBytes.push_back(static_cast<char>(len & 0xFFu));
        outBytes.append(payload.data(), payload.size());
    }
    if (error) *error = FrameError::None;
    return true;
}

FrameDecoder::FrameDecoder(FrameMode mode, std::size_t maxFrameBytes) noexcept
    : mode_(mode), maxFrameBytes_(maxFrameBytes) {}

bool FrameDecoder::feed(std::string_view bytes, std::vector<std::string>& outFrames, FrameError* error) {
    auto fail = [&](FrameError err) {
        buffer_.clear();
        if (error) *error = err;
        return false;
    };
    if (error) *error = FrameError::None;
    if (bytes.empty()) return true;

    const std::size_t maxBuffered = maxFrameBytes_ + (mode_ == FrameMode::Ndjson ? 2u : 4u);
    buffer_.append(bytes.data(), bytes.size());

    if (mode_ == FrameMode::Ndjson) {
        std::size_t start = 0;
        while (start < buffer_.size()) {
            const std::size_t nl = buffer_.find('\n', start);
            if (nl == std::string::npos) {
                const std::size_t rem = buffer_.size() - start;
                if (rem > maxFrameBytes_) return fail(FrameError::OversizedFrame);
                break;
            }
            std::size_t lineEnd = nl;
            if (lineEnd > start && buffer_[lineEnd - 1] == '\r') --lineEnd;
            const std::size_t lineLen = lineEnd - start;
            if (lineLen == 0) return fail(FrameError::EmptyPayload);
            if (lineLen > maxFrameBytes_) return fail(FrameError::OversizedFrame);
            outFrames.emplace_back(buffer_.data() + start, lineLen);
            start = nl + 1;
        }
        if (start > 0) buffer_.erase(0, start);
        if (buffer_.size() > maxBuffered) return fail(FrameError::OversizedFrame);
        return true;
    }

    // LengthPrefixed: 4-byte big-endian unsigned length header.
    std::size_t offset = 0;
    while (buffer_.size() - offset >= 4u) {
        const auto b0 = static_cast<std::uint8_t>(buffer_[offset]);
        const auto b1 = static_cast<std::uint8_t>(buffer_[offset + 1]);
        const auto b2 = static_cast<std::uint8_t>(buffer_[offset + 2]);
        const auto b3 = static_cast<std::uint8_t>(buffer_[offset + 3]);
        const std::uint32_t len = (static_cast<std::uint32_t>(b0) << 24) |
                                  (static_cast<std::uint32_t>(b1) << 16) |
                                  (static_cast<std::uint32_t>(b2) << 8) |
                                  static_cast<std::uint32_t>(b3);
        if (len == 0u) return fail(FrameError::EmptyPayload);
        if (len > maxFrameBytes_) return fail(FrameError::OversizedFrame);
        if (buffer_.size() - offset < 4u + static_cast<std::size_t>(len)) break;
        outFrames.emplace_back(buffer_.data() + offset + 4u, len);
        offset += 4u + static_cast<std::size_t>(len);
    }
    if (offset > 0) buffer_.erase(0, offset);
    if (buffer_.size() > maxBuffered) return fail(FrameError::OversizedFrame);
    return true;
}

LoopbackOmsTransport::Pair LoopbackOmsTransport::createPair() {
    auto ch = std::make_shared<Channel>();
    Pair pair;
    pair.adapterSide = std::shared_ptr<LoopbackOmsTransport>(new LoopbackOmsTransport(ch, true));
    pair.clientSide = std::shared_ptr<LoopbackOmsTransport>(new LoopbackOmsTransport(ch, false));
    return pair;
}

bool LoopbackOmsTransport::connected() const noexcept {
    return channel_ && channel_->open;
}

bool LoopbackOmsTransport::sendBytes(std::string_view bytes) {
    if (!connected()) return false;
    std::string& dst = isSideA_ ? channel_->aToB : channel_->bToA;
    dst.append(bytes.data(), bytes.size());
    bytesSent_ += bytes.size();
    return true;
}

std::string LoopbackOmsTransport::receiveBytes(std::size_t maxBytes) {
    if (!channel_) return {};
    std::string& src = isSideA_ ? channel_->bToA : channel_->aToB;
    if (src.empty() || maxBytes == 0) return {};
    const std::size_t n = std::min(maxBytes, src.size());
    std::string out(src.data(), n);
    src.erase(0, n);
    bytesReceived_ += n;
    return out;
}

void LoopbackOmsTransport::close() {
    if (channel_) channel_->open = false;
}

std::size_t LoopbackOmsTransport::pendingReadBytes() const noexcept {
    if (!channel_) return 0;
    const std::string& src = isSideA_ ? channel_->bToA : channel_->aToB;
    return src.size();
}

} // namespace fsim::oms
