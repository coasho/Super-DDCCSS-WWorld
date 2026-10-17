#include "applications/viewer/DemoLink.h"

#include "foundation/core/Log.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <new>
#include <thread>

#ifdef _WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <windows.h>
#    include <io.h>
#else
#    include <csignal>
#    include <spawn.h>
#    include <sys/wait.h>
#    include <unistd.h>
extern char** environ;
#endif

namespace fsim::app {

namespace {

#ifdef _WIN32
/// One argument quoted as CommandLineToArgvW reads it back.
std::wstring quoted(const std::string& argument) {
    const std::wstring a = std::filesystem::path(argument).wstring(); // (the narrow text as the process code page reads it)
    if (!a.empty() && a.find_first_of(L" \t\n\v\"") == std::wstring::npos) return a;
    std::wstring out = L"\"";
    for (auto it = a.begin();; ++it) {
        std::size_t backslashes = 0;
        while (it != a.end() && *it == L'\\') { ++it; ++backslashes; }
        if (it == a.end()) { out.append(backslashes * 2, L'\\'); break; }
        if (*it == L'"') out.append(backslashes * 2 + 1, L'\\');
        else out.append(backslashes, L'\\');
        out.push_back(*it);
    }
    out.push_back(L'"');
    return out;
}

/// A standard stream's handle as the C runtime has it now (a console, or the log file the viewer reopened it to),
/// made inheritable: the publisher's log goes where the viewer's goes.
HANDLE inheritable(FILE* stream) {
    const intptr_t h = _get_osfhandle(_fileno(stream));
    if (h == -1 || h == -2) return nullptr;
    const auto handle = reinterpret_cast<HANDLE>(h);
    SetHandleInformation(handle, HANDLE_FLAG_INHERIT, HANDLE_FLAG_INHERIT);
    return handle;
}
#endif

} // namespace

struct DemoLink::Process {
#ifdef _WIN32
    PROCESS_INFORMATION info{};
#else
    pid_t pid = 0;
#endif
    bool running = false;

    bool start(const std::filesystem::path& program, const std::vector<std::string>& arguments) {
#ifdef _WIN32
        std::wstring command = quoted(program.string());
        for (const auto& a : arguments) command += L" " + quoted(a);
        STARTUPINFOW si{};
        si.cb = sizeof si;
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = nullptr;
        si.hStdOutput = inheritable(stdout);
        si.hStdError = inheritable(stderr);
        // (no window of its own: a console program started from the GUI viewer would open one)
        if (!CreateProcessW(program.wstring().c_str(), command.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                            nullptr, &si, &info))
            return false;
        CloseHandle(info.hThread);
        info.hThread = nullptr;
#else
        std::vector<std::string> args{program.string()};
        args.insert(args.end(), arguments.begin(), arguments.end());
        std::vector<char*> argv;
        for (auto& a : args) argv.push_back(a.data());
        argv.push_back(nullptr);
        if (posix_spawn(&pid, args.front().c_str(), nullptr, nullptr, argv.data(), environ) != 0) return false;
#endif
        running = true;
        return true;
    }

    /// Still running; once it is not, `exitCode` is its exit status.
    bool alive(int& exitCode) {
        if (!running) return false;
#ifdef _WIN32
        if (WaitForSingleObject(info.hProcess, 0) == WAIT_TIMEOUT) return true;
        DWORD code = 0;
        GetExitCodeProcess(info.hProcess, &code);
        exitCode = static_cast<int>(code);
#else
        int status = 0;
        if (waitpid(pid, &status, WNOHANG) == 0) return true;
        exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
#endif
        running = false;
        return false;
    }

    /// Wait up to `seconds` for it to exit, then end it: whether it exited by itself.
    bool finish(double seconds) {
        if (!running) return true;
        bool exited = true;
#ifdef _WIN32
        if (WaitForSingleObject(info.hProcess, static_cast<DWORD>(seconds * 1000.0)) != WAIT_OBJECT_0) {
            LOG_WARN("demo") << "the demo publisher did not stop; ending it";
            TerminateProcess(info.hProcess, 1);
            WaitForSingleObject(info.hProcess, 2000);
            exited = false;
        }
        CloseHandle(info.hProcess);
        info.hProcess = nullptr;
#else
        int exitCode = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (alive(exitCode) && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (running) {
            LOG_WARN("demo") << "the demo publisher did not stop; ending it";
            kill(pid, SIGKILL);
            waitpid(pid, nullptr, 0);
            exited = false;
        }
#endif
        running = false;
        return exited;
    }

    ~Process() { finish(0.0); }
};

DemoLink::DemoLink() : process_(std::make_unique<Process>()) {}

DemoLink::~DemoLink() { stop(); }

std::unique_ptr<DemoLink> DemoLink::launch(const std::filesystem::path& program, std::vector<std::string> arguments,
                                           double timeFactor, bool paused, double timeoutSeconds) {
#ifdef _WIN32
    const std::uint64_t pid = GetCurrentProcessId();
#else
    const std::uint64_t pid = static_cast<std::uint64_t>(getpid());
#endif
    const std::string world = "demo-" + std::to_string(pid);
    std::unique_ptr<DemoLink> link(new DemoLink());
    // the control block first: the publisher opens it as it starts
    if (!link->controlMemory_.create(ipc::demoControlName(world), sizeof(ipc::DemoControl))) {
        LOG_ERROR("demo") << "cannot create the demo control block '" << ipc::demoControlName(world) << "'";
        return nullptr;
    }
    link->control_ = new (link->controlMemory_.data()) ipc::DemoControl();
    link->control_->timeFactor.store(timeFactor);
    link->control_->paused.store(paused ? 1u : 0u);
    link->control_->version = ipc::kDemoControlVersion;
    link->control_->magic = ipc::kDemoControlMagic;

    arguments.insert(arguments.end(), {"--world", world, "--control", "--parent-pid", std::to_string(pid)});
    if (!std::filesystem::is_regular_file(program) || !link->process_->start(program, arguments)) {
        LOG_ERROR("demo") << "cannot start the demo publisher " << program.string();
        return nullptr;
    }
    LOG_INFO("demo") << "demo publisher started: world '" << world << "'";
    // its world, and its first snapshot (with each vehicle's CG offset in its flight status)
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSeconds);
    while (std::chrono::steady_clock::now() < deadline) {
        int exitCode = 0;
        if (!link->process_->alive(exitCode)) {
            LOG_ERROR("demo") << "the demo publisher exited (code " << exitCode << ") before publishing its world";
            return nullptr;
        }
        if (link->mirror_.valid() || link->mirror_.open(world)) {
            link->mirror_.pollTable();
            if (link->mirror_.pollSnapshot(link->batch_)) {
                link->fresh_ = true;
                return link;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    LOG_ERROR("demo") << "the demo publisher did not publish world '" << world << "' within " << timeoutSeconds << " s";
    return nullptr;
}

std::size_t DemoLink::vehicles() const noexcept { return mirror_.capacity(); }

void DemoLink::cgFromOriginBodyM(std::size_t i, double* out) const {
    const auto& status = mirror_.status();
    for (std::size_t k = 0; k < 3; ++k) out[k] = i < status.size() ? status[i].derived[ipc::kDerivedCgFromOriginM + k] : 0.0;
}

void DemoLink::setPaused(bool paused) noexcept { control_->paused.store(paused ? 1u : 0u, std::memory_order_relaxed); }

void DemoLink::setTimeFactor(double factor) noexcept { control_->timeFactor.store(factor, std::memory_order_relaxed); }

void DemoLink::singleStep() noexcept { control_->stepRequests.fetch_add(1, std::memory_order_relaxed); }

const sim::SnapshotBatch* DemoLink::acquire() noexcept {
    if (mirror_.pollSnapshot(batch_)) fresh_ = true;
    if (!fresh_) return nullptr;
    fresh_ = false;
    batch_.states.resize(vehicles()); // (the mirror's copy has its capacity's slots: the demo's vehicles)
    return &batch_;
}

double DemoLink::throughput() const noexcept { return control_->throughput.load(std::memory_order_relaxed); }

bool DemoLink::stop() noexcept {
    if (control_) control_->quit.store(1, std::memory_order_relaxed);
    return process_->finish(5.0);
}

} // namespace fsim::app
