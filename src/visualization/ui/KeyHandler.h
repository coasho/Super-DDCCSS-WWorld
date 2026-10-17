#pragma once

#include "visualization/ui/ViewerControls.h"

#include <vsg/all.h>

#include <memory>

namespace fsim::ui {

/// Keyboard bindings for the viewer (design 9.6). Registered after ImGui's
/// event forwarder so text fields keep their keys.
///
///   space        pause / resume (on its release, unless it was held for a drag)    .   single step (while paused)
///   tab / s+tab  next / previous vehicle
///   c            cycle camera mode (chase, orbit, overview, free)     - / =   zoom out / in
///   r            reset view            mouse      left drag rotate, space + left drag pan (detached only), wheel zoom
///   l            toggle vehicle list   m          toggle monitor
///   n            toggle name labels    t          toggle trails
///   [ / ]        time factor / 2, x 2  esc        quit
class KeyHandler : public vsg::Inherit<vsg::Visitor, KeyHandler> {
public:
    explicit KeyHandler(std::shared_ptr<ViewerControls> controls) : controls_(std::move(controls)) {}

    void apply(vsg::KeyPressEvent& e) override;
    void apply(vsg::KeyReleaseEvent& e) override;
    void apply(vsg::ButtonPressEvent& e) override;

private:
    std::shared_ptr<ViewerControls> controls_;
    bool spaceDown_ = false, spaceUsed_ = false; ///< space held; a mouse button pressed while it was (a pan, not a pause)
};

} // namespace fsim::ui
