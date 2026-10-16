#pragma once

// Camera images of a world in shared memory (design 8.4 "sensor preview"):
// fsim_vision publishes what each camera rendered into "fsim.vision.<world>"
// and the viewer shows the frames next to the world it mirrors. Same
// principles as the world segment: the publisher never blocks, the reader
// polls, seqlocks per camera, nothing in the trainer waits for a viewer.
//
// Layout: VisionHeader | image bytes (RGB, tightly packed) at each camera's
// offset. The segment is created once per (world, capacity); a change of the
// camera set rewrites the table under a new generation.

#include "contracts/vision_stream/VisionLayout.h"
#include "platform/SharedMemory.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fsim::ipc {

/// A camera as the publisher describes it.
struct VisionCameraDesc {
    std::uint32_t vehicleId = 0;
    unsigned width = 0, height = 0;
    std::string label;
};

class VisionPublisher {
public:
    /// Create "fsim.vision.<world>" with room for `capacityBytes` of images.
    bool create(const std::string& worldName, std::size_t capacityBytes = 32u << 20);
    void close();
    bool valid() const noexcept { return shm_.valid(); }

    /// Rewrite the camera table (drops cameras that do not fit); returns how many were kept.
    unsigned setCameras(const std::vector<VisionCameraDesc>& cameras);
    /// Write one camera's RGB pixels (width*height*3 bytes).
    void write(unsigned camera, const std::uint8_t* rgb, std::size_t bytes);
    /// Mark the end of a frame (the reader's "new images" signal).
    void endFrame();

private:
    VisionHeader* header() noexcept { return static_cast<VisionHeader*>(shm_.data()); }
    platform::SharedMemory shm_;
    unsigned count_ = 0;
};

/// Read side; polls without ever blocking the publisher.
class VisionMirror {
public:
    bool open(const std::string& worldName);
    void close();
    bool valid() const noexcept { return shm_.valid(); }

    struct Camera {
        std::uint32_t vehicleId = 0;
        unsigned width = 0, height = 0;
        std::string label;
    };
    /// Re-read the camera table if it changed; returns true when it did.
    bool pollTable();
    const std::vector<Camera>& cameras() const noexcept { return cameras_; }
    std::uint64_t frame() const noexcept;          ///< the publisher's frame counter
    bool publisherAlive() const noexcept;

    /// Copy one camera's pixels (consistent snapshot); false when it is being written right now.
    bool read(unsigned camera, std::vector<std::uint8_t>& rgb) const;

private:
    const VisionHeader* header() const noexcept { return static_cast<const VisionHeader*>(shm_.data()); }
    platform::SharedMemory shm_;
    std::vector<Camera> cameras_;
    std::uint64_t generation_ = 0;
};

std::string visionSegmentName(const std::string& worldName);

} // namespace fsim::ipc
