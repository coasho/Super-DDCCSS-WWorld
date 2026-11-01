#pragma once

// Private tooling seam. Observers read already evaluated native properties;
// they do not replace integration, forces, commands or the public-step result.
// Trusted synchronous tooling only: read-only use is a callback contract, not
// an access-control boundary. Cross-thread observation is not supported.
namespace fsim::sim {
class JsbsimModel;
struct NativeStepObserverV1 {
    const JsbsimModel* model = nullptr;
    void* context = nullptr;
    void (*observe)(void*, unsigned substep, unsigned substeps) noexcept = nullptr;
};
inline thread_local NativeStepObserverV1 nativeStepObserverV1;

class ScopedNativeStepObserverV1 {
public:
    explicit ScopedNativeStepObserverV1(NativeStepObserverV1 observer) noexcept
        : previous_(nativeStepObserverV1) { nativeStepObserverV1=observer; }
    ~ScopedNativeStepObserverV1() { nativeStepObserverV1=previous_; }
    ScopedNativeStepObserverV1(const ScopedNativeStepObserverV1&)=delete;
    ScopedNativeStepObserverV1& operator=(const ScopedNativeStepObserverV1&)=delete;
private:
    NativeStepObserverV1 previous_;
};
} // namespace fsim::sim
