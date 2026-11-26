#pragma once

#include "foundation/core/Span.h"
#include "fsim/VehicleState.h"

#include <vsg/all.h>

#include <vector>

namespace fsim::world {

/// Per-vehicle flight-path trails (design 8.2 "Effects"): a line strip over the
/// last N sampled positions, fading out along its length. Positions are kept
/// relative to a per-trail origin so float vertices stay precise.
class Trails {
public:
    /// Where along the airframe each trail attaches (B-52): Origin (default, vehicle state origin),
    /// CenterOfGravity (shifted by optional per-slot body offset), or WingTips (wing-tip ribbon origin).
    enum class Attachment { Origin, CenterOfGravity, WingTips };

    Trails(std::size_t vehicles, std::size_t points, double sampleIntervalS, vsg::ref_ptr<const vsg::Options> options);

    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Append a sample per vehicle when `simTime` has advanced by the sample
    /// interval; rewrites the GPU arrays (cheap: N points per vehicle).
    void update(Span<const sim::VehicleState> states, double simTime);

    void setSelected(int index) { selected_ = index; }
    void setAttachment(Attachment attachment) noexcept { attachment_ = attachment; }
    Attachment attachment() const noexcept { return attachment_; }
    void setBodyOffset(std::size_t index, const vsg::dvec3& bodyOffsetM);
    void setVisible(bool on) { root_->setAllChildren(on); }
    /// Disable (and clear) a vehicle's trail, e.g. for slots without a live vehicle.
    void setEnabled(std::size_t index, bool enabled);

private:
    struct Trail {
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec4Array> colors;
        vsg::ref_ptr<vsg::VertexDraw> draw;
        std::vector<vsg::dvec3> ring; ///< O(1) circular buffer of size points_
        std::size_t head = 0;         ///< index of oldest sample in ring
        std::size_t count = 0;        ///< active samples in ring (0..points_)
        vsg::dvec3 bodyOffset{0.0, 0.0, 0.0};
        vsg::dvec3 origin;
        bool originSet = false;
        bool enabled = true;
        bool lastSelected = false;
    };
    void rewrite(Trail& t, bool selected);

    vsg::ref_ptr<vsg::Switch> root_;
    std::vector<Trail> trails_;
    std::size_t points_;
    double interval_;
    double lastSample_ = -1.0;
    int selected_ = -1;
    Attachment attachment_ = Attachment::Origin;
};

} // namespace fsim::world
