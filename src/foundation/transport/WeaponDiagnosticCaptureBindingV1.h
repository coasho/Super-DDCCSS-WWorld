#pragma once

// Private additive recording provenance. No fsrec, shared-memory, diagnostic
// frame, SDK class layout or plugin interface is changed by this companion.
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace fsim::ipc {

struct WeaponDiagnosticFrameV1;
enum class WeaponDiagnosticsReplayResultV1;

struct WeaponDiagnosticFileDigestV1 {
    std::uint64_t bytes=0;
    std::string sha256;
};

struct WeaponDiagnosticCaptureBindingV1 {
    std::uint64_t publisherPid=0;
    std::uint32_t publisherIncarnation=0;
    WeaponDiagnosticFileDigestV1 recording,diagnostics;
};

struct WeaponDiagnosticCaptureLimitsV1 {
    // Diagnostic admission bound, not a limit on legacy flight recording.
    // Hashing retains one 64 KiB block, regardless of the accepted file size.
    std::uint64_t maximumFileBytes=std::uint64_t{1}<<40;
    std::size_t maximumManifestBytes=4096;
};

struct WeaponDiagnosticCaptureCompletionV1 {
    // These are the actual writers' checked flush/CLOSE results and intended
    // byte counts, captured before their handles are discarded. File presence
    // or an existing partial file cannot substitute for successful completion.
    bool recordingClosedSuccessfully=false,diagnosticsClosedSuccessfully=false;
    std::uint64_t recordingBytes=0,diagnosticsBytes=0;
    std::uint64_t publisherPid=0;
    std::uint32_t publisherIncarnation=0;
};

std::filesystem::path weaponDiagnosticCaptureBindingPathV1(
    const std::filesystem::path& diagnosticsPath);

// Transactional outputs: failure leaves the supplied digest/binding unchanged.
// Symlinks, non-regular inputs, read errors, size/mtime changes and byte-count
// overflow are refused. This is content provenance, not sender authentication.
bool weaponDiagnosticFileDigestV1(const std::filesystem::path&,
    WeaponDiagnosticFileDigestV1&,std::string& error,
    const WeaponDiagnosticCaptureLimitsV1& limits={});

// Call BEFORE truncating either output of a new capture. Removal is limited to
// this exact completion marker; no source file or stale temporary directory is
// removed. Failure must not be treated as a fresh binding session.
bool invalidateWeaponDiagnosticCaptureBindingV1(
    const std::filesystem::path& diagnosticsPath,std::string& error);

// Call only after successful invalidation and both checked writer closes.
// Refuses an existing completion marker. The small temporary manifest is
// closed beside the final marker, then published by an atomic no-replace hard
// link LAST; only our exact temporary link/directory are cleaned up. A
// filesystem without hard-link support explicitly refuses diagnostic binding.
// Failure never creates a completed binding. Bytes, not paths, are bound.
bool finalizeWeaponDiagnosticCaptureBindingV1(
    const std::filesystem::path& recordingPath,
    const std::filesystem::path& diagnosticsPath,
    const WeaponDiagnosticCaptureCompletionV1&,
    std::string& error,const WeaponDiagnosticCaptureLimitsV1& limits={});

// A missing/unbound legacy companion is unavailable to joined flight replay.
// Raw legacy fsrec replay is independent and remains usable. The caller must
// use BoundWeaponDiagnosticsReplayV1 for guarded per-record CRC/schema
// validation and exact native-time lookup after this file-pair verification.
bool verifyWeaponDiagnosticCaptureBindingV1(
    const std::filesystem::path& recordingPath,
    const std::filesystem::path& diagnosticsPath,
    WeaponDiagnosticCaptureBindingV1&,std::string& error,
    const WeaponDiagnosticCaptureLimitsV1& limits={});

// New sibling rather than a layout change to the original diagnostic reader.
// Full hashes are verified once on open. Every subsequent operation guards the
// original regular-file sizes/mtimes and exact bounded completion-marker bytes
// before AND after reading. Ordinary overwrite/new-capture invalidation closes
// this reader and leaves caller output unchanged. This is not authentication
// against a malicious same-size edit with deliberately preserved metadata.
class BoundWeaponDiagnosticsReplayV1 {
public:
    BoundWeaponDiagnosticsReplayV1();
    ~BoundWeaponDiagnosticsReplayV1();
    BoundWeaponDiagnosticsReplayV1(const BoundWeaponDiagnosticsReplayV1&)=delete;
    BoundWeaponDiagnosticsReplayV1& operator=(const BoundWeaponDiagnosticsReplayV1&)=delete;
    bool open(const std::filesystem::path& recordingPath,
        const std::filesystem::path& diagnosticsPath,std::string& error,
        const WeaponDiagnosticCaptureLimitsV1& limits={});
    void close() noexcept;
    bool rewind();
    WeaponDiagnosticsReplayResultV1 next(WeaponDiagnosticFrameV1&);
    bool atTime(double simTimeS,WeaponDiagnosticFrameV1&);
    // Open at the most recent operation, not a substitute for guarded reads.
    bool isOpen() const noexcept;
    std::uint64_t publisherPid() const noexcept;
    std::uint32_t publisherIncarnation() const noexcept;
    const std::string& errorV1() const noexcept {return error_;}
private:
    struct State;
    std::unique_ptr<State> state_;
    std::string error_;
    bool checkCurrentV1();
};

} // namespace fsim::ipc
