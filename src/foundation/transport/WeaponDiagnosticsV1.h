#pragma once
#include "contracts/world_stream/WeaponDiagnosticsV1.h"
#include "foundation/platform/SharedMemory.h"
#include <fstream>
#include <memory>
#include <string>

namespace fsim::ipc {
struct WeaponDiagnosticCaptureCompletionV1;
/// Read only the existing world header. Outputs change together only for a
/// live compatible identity with a completed native world frame.
bool weaponDiagnosticWorldIdentityV1(const std::string& world,std::uint64_t& pid,
                                     std::uint32_t& incarnation,std::uint64_t& frame);
/// Separate referee/Viewer sidecar: fsim.weapon-diagnostics.v1.<world>.<pid>.<incarnation>.
/// One publisher and a last-request-wins Viewer selection. All payload words
/// are atomic; bounded retries never wait for, or lock, native simulation.
class WeaponDiagnosticsV1 {
public:
    WeaponDiagnosticsV1()=default;
    ~WeaponDiagnosticsV1();
    WeaponDiagnosticsV1(const WeaponDiagnosticsV1&)=delete;
    WeaponDiagnosticsV1& operator=(const WeaponDiagnosticsV1&)=delete;
    bool create(const std::string& world,std::uint64_t pid,std::uint32_t incarnation);
    bool open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation);
    void close() noexcept;
    bool valid() const noexcept;
    /// The caller stamps the exact native worldFrame/simTime and publisher
    /// identity. Invalid/oversized/reordered frames do not replace old data.
    bool publish(const WeaponDiagnosticFrameV1&) noexcept;
    bool read(std::uint64_t worldFrame,WeaponDiagnosticFrameV1& out) const noexcept;
    bool readLatest(WeaponDiagnosticFrameV1& out) const noexcept;
    bool requestSelection(std::uint32_t entity,std::uint64_t generation,
                          std::uint64_t currentWorldFrame,std::uint64_t* request=nullptr) noexcept;
    /// Producer must also resolve entity+generation against its OWN native
    /// state. A selection is display demand, never a modeled weapon command.
    bool selection(std::uint64_t currentWorldFrame,WeaponDiagnosticSelectionV1& out,
                   std::uint64_t maximumAgeFrames=300) const noexcept;
private:
    struct Shared;
    platform::SharedMemory memory_;
    Shared* shared_=nullptr;
    std::uint64_t pid_=0,lastFrame_=0;
    std::uint32_t incarnation_=0;
    double lastTimeS_=0;
    bool owner_=false;
};

/// Optional companion file, never a change to fsrec v8. Every record contains
/// the original bounded frame and CRC32; no resimulation or image rebuilding.
class WeaponDiagnosticsRecorderV1 {
public:
    bool open(const std::string& path,std::uint64_t pid,std::uint32_t incarnation);
    bool append(const WeaponDiagnosticFrameV1&);
    bool close();
    /// Capture original publisher/actual intended length, then check flush AND
    /// close. Failure closes the writer and leaves completion unchanged.
    bool finishCaptureV1(WeaponDiagnosticCaptureCompletionV1& completion);
private:
    std::ofstream stream_;
    std::uint64_t pid_=0,lastFrame_=0;
    std::uint32_t incarnation_=0;
    double lastTimeS_=0;
};
enum class WeaponDiagnosticsReplayResultV1 {Frame,End,Invalid};
class WeaponDiagnosticsReplayV1 {
public:
    bool open(const std::string& path);
    void close();
    bool rewind();
    WeaponDiagnosticsReplayResultV1 next(WeaponDiagnosticFrameV1& out);
    /// Exact simulation-time join for legacy fsrec v8, which has no source
    /// world-frame field. Missing times stay unavailable; never interpolate.
    bool atTime(double simTimeS,WeaponDiagnosticFrameV1& out);
    std::uint64_t publisherPid() const noexcept{return pid_;}
    std::uint32_t publisherIncarnation() const noexcept{return incarnation_;}
private:
    std::ifstream stream_;
    std::uint64_t pid_=0,lastFrame_=0;
    std::uint32_t incarnation_=0;
    double lastTimeS_=0;
    bool invalid_=false;
    std::unique_ptr<WeaponDiagnosticFrameV1> lookahead_;
    double requestedTimeS_=-1;
};
}
