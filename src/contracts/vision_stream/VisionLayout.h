#pragma once

// The camera-image segment's layout (design 8.4 "sensor preview"): VisionHeader | image bytes (RGB, tightly packed) at
// each camera's offset, seqlocks per camera. A neutral contract (src/contracts, ARCH-02): the publisher (fsim_vision)
// and the reader (the viewer's preview) include it through the transport's segment (ipc/VisionSegment.h). Its names
// and namespace are those the definitions had in src/ipc.

#include <atomic>
#include <cstdint>

namespace fsim::ipc {

inline constexpr std::uint32_t kVisionMagic = 0x49565346u; // "FSVI"
inline constexpr std::uint32_t kVisionVersion = 1;
inline constexpr unsigned kMaxVisionCameras = 64;
inline constexpr unsigned kVisionLabelLength = 32;

struct VisionCameraRecord {
    std::uint32_t vehicleId = 0;
    std::uint32_t width = 0, height = 0;
    std::uint32_t channels = 3;
    std::uint64_t offset = 0;                ///< image bytes from the segment start
    std::atomic<std::uint64_t> sequence{0}; ///< odd while being written; even = consistent
    char label[kVisionLabelLength] = {};     ///< vehicle name (or "cam<i>")
};

struct VisionHeader {
    std::uint32_t magic = 0, version = 0;
    std::uint64_t totalBytes = 0;
    std::uint64_t pid = 0;
    std::atomic<std::uint64_t> generation{0}; ///< bumped when the camera table is rewritten (odd while rewriting)
    std::atomic<std::uint64_t> frame{0};      ///< bumped after every published frame
    std::uint32_t cameraCount = 0;
    VisionCameraRecord cameras[kMaxVisionCameras];
};

} // namespace fsim::ipc
