#include "visualization/ui/KeyHandler.h"

#include <algorithm>

namespace fsim::ui {

void KeyHandler::apply(vsg::KeyPressEvent& e) {
    if (e.handled) return; // ImGui consumed it
    auto& c = *controls_;
    const bool shift = (e.keyModifier & vsg::MODKEY_Shift) != 0;

    switch (e.keyBase) {
    case vsg::KEY_Space: // (pause on its release: held for a drag, it pans)
        if (!spaceDown_) spaceDown_ = true, spaceUsed_ = false;
        break;
    case vsg::KEY_Home:
        c.seekTo.store(0.0, std::memory_order_relaxed);
        break;
    case vsg::KEY_Period:
        c.singleStep.store(true, std::memory_order_relaxed);
        break;
    case vsg::KEY_Tab:
        c.selectStep.store(shift ? -1 : 1, std::memory_order_relaxed); // resolved by the loop (live vehicles only)
        break;
    case vsg::KEY_c:
    case vsg::KEY_C:
        c.cameraMode.store((c.cameraMode.load(std::memory_order_relaxed) + 1) % 4, std::memory_order_relaxed);
        break;
    case vsg::KEY_Minus:
        c.cameraZoom.store(1.25, std::memory_order_relaxed);
        break;
    case vsg::KEY_Equals:
    case vsg::KEY_Plus:
        c.cameraZoom.store(0.8, std::memory_order_relaxed);
        break;
    case vsg::KEY_r:
    case vsg::KEY_R:
        c.cameraReset.store(true, std::memory_order_relaxed);
        break;
    case vsg::KEY_Escape:
        c.quit.store(true, std::memory_order_relaxed);
        break;
    default:
        return;
    }
    e.handled = true;
}

void KeyHandler::apply(vsg::KeyReleaseEvent& e) {
    if (e.keyBase != vsg::KEY_Space || !spaceDown_) return;
    spaceDown_ = false;
    if (!spaceUsed_) controls_->paused.store(!controls_->paused.load(std::memory_order_relaxed), std::memory_order_relaxed);
    e.handled = true;
}

void KeyHandler::apply(vsg::ButtonPressEvent&) {
    if (spaceDown_) spaceUsed_ = true; // (space + drag: the camera pans)
}

} // namespace fsim::ui
