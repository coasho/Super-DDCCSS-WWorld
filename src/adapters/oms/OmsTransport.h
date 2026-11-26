#pragma once

// SIM-M08 (MS-12): External OMS / A-GRA / UCI byte-stream framing and transport abstraction.
//
// External OMS transport carries software-bus envelopes between an onboard Mission Computing
// Environment (or external OMS client) and an Agent-bound adapter. It is strictly separate
// from modeled RF DMS/C2 delivery (World::createNet / Dms).

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim::oms {

/// Wire framing mode for serialized OMS JSON envelopes.
enum class FrameMode : std::uint8_t {
    Ndjson,         ///< UTF-8 JSON line terminated by '\n' (RFC 8259 / application/x-ndjson)
    LengthPrefixed, ///< 4-byte big-endian uint32 payload length followed by UTF-8 JSON bytes
};

/// Result of framing or decoding a wire frame.
enum class FrameError : std::uint8_t {
    None = 0,
    EmptyPayload,
    OversizedFrame,
    MalformedFrame,
    TransportClosed,
};

const char* frameErrorName(FrameError err) noexcept;

/// Default maximum serialized frame size (256 KiB).
inline constexpr std::size_t kDefaultMaxFrameBytes = 262144u;

/// Encode a single JSON payload into a wire frame. Returns false and sets `error`
/// if the payload is empty, exceeds `maxFrameBytes`, or (for Ndjson) contains raw '\n' or '\r'.
bool encodeFrame(std::string_view payload,
                 FrameMode mode,
                 std::string& outBytes,
                 FrameError* error = nullptr,
                 std::size_t maxFrameBytes = kDefaultMaxFrameBytes);

/// Incremental byte-stream frame decoder supporting arbitrary chunk fragmentation.
class FrameDecoder {
public:
    explicit FrameDecoder(FrameMode mode = FrameMode::Ndjson,
                          std::size_t maxFrameBytes = kDefaultMaxFrameBytes) noexcept;

    FrameMode mode() const noexcept { return mode_; }
    std::size_t maxFrameBytes() const noexcept { return maxFrameBytes_; }
    std::size_t bufferedBytes() const noexcept { return buffer_.size(); }

    /// Append incoming bytes and extract all complete frames into `outFrames`.
    /// Returns false on framing violation (e.g. OversizedFrame, MalformedFrame).
    bool feed(std::string_view bytes, std::vector<std::string>& outFrames, FrameError* error = nullptr);

    void reset() noexcept { buffer_.clear(); }

private:
    FrameMode mode_ = FrameMode::Ndjson;
    std::size_t maxFrameBytes_ = kDefaultMaxFrameBytes;
    std::string buffer_;
};

/// Abstract external OMS byte-stream transport. Upper-layer bridges (DDS, ZeroMQ,
/// local sockets/pipes) implement this interface above fsim::oms without coupling
/// simulation core targets to networking libraries.
class IOmsTransport {
public:
    virtual ~IOmsTransport() = default;
    virtual bool connected() const noexcept = 0;
    virtual bool sendBytes(std::string_view bytes) = 0;
    virtual std::string receiveBytes(std::size_t maxBytes = kDefaultMaxFrameBytes) = 0;
    virtual void close() = 0;
};

/// Deterministic in-memory duplex transport endpoint for unit and protocol fixtures.
/// Supports chunked reads/writes and explicit disconnect injection.
class LoopbackOmsTransport final : public IOmsTransport {
public:
    struct Pair {
        std::shared_ptr<LoopbackOmsTransport> adapterSide;
        std::shared_ptr<LoopbackOmsTransport> clientSide;
    };

    static Pair createPair();

    bool connected() const noexcept override;
    bool sendBytes(std::string_view bytes) override;
    std::string receiveBytes(std::size_t maxBytes = kDefaultMaxFrameBytes) override;
    void close() override;

    std::size_t pendingReadBytes() const noexcept;
    std::uint64_t bytesSent() const noexcept { return bytesSent_; }
    std::uint64_t bytesReceived() const noexcept { return bytesReceived_; }

private:
    struct Channel {
        std::string aToB;
        std::string bToA;
        bool open = true;
    };

    LoopbackOmsTransport(std::shared_ptr<Channel> channel, bool isSideA) noexcept
        : channel_(std::move(channel)), isSideA_(isSideA) {}

    std::shared_ptr<Channel> channel_;
    bool isSideA_ = true;
    std::uint64_t bytesSent_ = 0;
    std::uint64_t bytesReceived_ = 0;
};

} // namespace fsim::oms
