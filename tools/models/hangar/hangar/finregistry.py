"""Physical-fin identity shared by native actuators and visual hinge nodes."""
import json
import zlib


def identity(surface, control, side):
    # Authored names, rather than enumeration/mesh-patch order, establish identity.
    key = json.dumps([surface.name, control.name, int(side)], ensure_ascii=True, separators=(",", ":"))
    value = zlib.crc32(key.encode("ascii")) & 0xffffffff
    if value == 0:
        raise ValueError("missile fin identity is reserved: " + key)
    return value


def visual_gain(surface, piece, axis):
    import numpy as np
    # Native fin delta uses BODY fin normal (-u.x,u.y,-u.z). A positive
    # local hinge turn moves the trailing edge towards -u in design axes.
    # The visual hinge helper may reverse that axis for its primary channel.
    hinge = piece["hinge"][0]
    return 1.0 if np.dot(np.cross(axis, piece["te"]-hinge), -piece["u"]) > 0 else -1.0
