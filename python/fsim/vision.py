"""Cameras on vehicles (docs/sdk/vision.md): offscreen RGB, depth and
segmentation images of the same Earth the viewer draws, as numpy arrays
viewing the renderer's own buffers.

Imported on its own (``import fsim.vision``) because it opens a Vulkan
device: ``import fsim`` works on a machine without a GPU driver, this does
not.

    import fsim, fsim.vision
    world = fsim.World("cams")
    red = world.create_vehicle("red", altitude_msl_m=800, airspeed_ms=60)
    vision = fsim.vision.Vision(world)
    nose = vision.add_camera(red, width=256, height=192, fov_deg=70)
    world.step(); vision.render()
    rgb = nose.image()            # (192, 256, 3) uint8, valid until the next render
"""
import numpy as np

from . import _vision
from ._contracts import options as _options
from ._visualization import cameras as _cameras
from ._visualization.cameras import Camera, Vision, _camera_spec
from .vecenv import VecEnv
from .world import Vehicle, World

# The camera implementation takes these public wrappers without importing them (ARCH-07).
_cameras.bind(World, Vehicle)


# The optional training-image composition (ARCH-07): a VecEnv's batch with a camera per vehicle. It composes the public
# VecEnv and the camera implementation here, outside both the camera core and the training core.
class VisionBatch:
    """One camera per batch vehicle of a VecEnv, images as tensors:
    ``rgb`` (N, H, W, 3) uint8, ``depth`` (N, H, W) float32 and
    ``segmentation`` (N, H, W) uint16 - views of the renderer's buffers,
    rewritten by every render."""

    def __init__(self, env, *, width=None, height=None, fov_deg=None, offset_body_m=None, yaw_deg=None, pitch_deg=None,
                 roll_deg=None, hide_own_vehicle=None, depth=None, segmentation=None, **vision_options):
        if not isinstance(env, VecEnv):
            raise TypeError("VisionBatch(env): a fsim.VecEnv")
        spec = _camera_spec(width, height, fov_deg, offset_body_m, yaw_deg, pitch_deg, roll_deg, hide_own_vehicle, depth,
                            segmentation)
        if segmentation:
            vision_options.setdefault("segmentation", True)
        self._h = _vision.VisionBatch(env._h, spec, _options(**vision_options))
        self.count = self._h.count()
        self.sensors = Vision(env.world, _handle=self._h.sensors())
        _, self.height, self.width = self.sensors._h.image(0)
        self._views()

    def _views(self):
        rgb, depth, seg = self._h.tensors()
        n, h, w = self.count, self.height, self.width
        self.rgb = np.frombuffer(rgb, dtype=np.uint8).reshape(n, h, w, 3)
        self.depth = None if depth is None else np.frombuffer(depth, dtype=np.float32).reshape(n, h, w)
        self.segmentation = None if seg is None else np.frombuffer(seg, dtype=np.uint16).reshape(n, h, w)

    def render(self):
        """Draw every batch camera; returns ``rgb``."""
        self._h.render()
        return self.rgb
