"""Delivery-derived negative controls for aero_wake_containment.py (wake/body containment).

CONTAINMENT_TOOL selects the implementation (default: the real tool). The RED evidence ran these tests against
native/numerics/naive_containment.py, which only verifies that the binary64 wake points round to the packed
binary32 points, as the accepted coordinate-matching check does, and therefore passes every wake.

Fixture: the released alpha 0 VSPGEOM and VKEY with the archived alpha 0 ADB (`native/numerics/archive-alpha0`), copied read
only below TMP. Mutations rewrite the internal binary64 wake points and their packed binary32 copies together, so each
case stays self-consistent. The ADB offsets are found here by an independent walk of the pinned 7.2.2 layout.

The reference result is `native/numerics/strength-summary.json`, the LAB owner's source-qualified classification. At alpha 0
it lists three definite interior points on active trails: 14/node 2, 20/node 2 and 21/node 3.

Run from the project root:
    fsim.cmd --preset ucrt64-headless python tools/validation/missile/test_aero_wake_containment.py -v
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
FIXTURES = Path(__file__).with_name("native") / "numerics"
TARGET = Path(os.environ.get("CONTAINMENT_TOOL", str(Path(__file__).with_name("aero_wake_containment.py"))))
spec = importlib.util.spec_from_file_location("containment_under_test", TARGET)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)
print("containment tool under test: %s sha256=%s" % (TARGET, hashlib.sha256(TARGET.read_bytes()).hexdigest()))
CASE = FIXTURES / "archive-alpha0"
REFERENCE = json.loads((FIXTURES / "strength-summary.json").read_text(encoding="utf-8"))["cases"]["alpha0"]


def layout(raw):
    """Offsets of the solve-level circulation, the wake records and the packed binary32 wake points."""
    offset = 0

    def take(fmt, count=1):
        nonlocal offset
        size = struct.calcsize("<" + fmt)
        rows = [struct.unpack_from("<" + fmt, raw, offset + k * size) for k in range(count)]
        offset += size * count
        return rows
    _m, _mo, _s, _u, loops, nodes, triangles, edges = take("8i")[0]
    take("6f")
    take("i100si", take("i")[0][0])
    take("6if", triangles)
    take("3f", nodes)
    take("i", 2)
    levels = take("i")[0][0]
    packed_offset, packed_count = None, None
    for level in range(1, levels + 1):
        nnode, nedge = take("2i")[0]
        if level == 1:
            packed_offset, packed_count = offset, nnode
        take("3f", nnode)
        take("4i", nedge)
    take("i", take("i")[0][0])
    take("i", take("i")[0][0])
    take("i")
    take("5f")
    gamma_offset = offset
    take("2d", loops)
    take("d", 3 * edges)
    take("d", 3 * loops)
    take("f", 3 * triangles)
    records = []
    for index in range(take("i")[0][0]):
        node, _span, count = take("idi")[0]
        records.append({"node": node, "count": count, "offset": offset})
        take("3d", count)
    wake_nodes = max(record["count"] for record in records)
    surface = packed_count - len(records) * wake_nodes
    return {"gamma": gamma_offset, "records": records, "packed": packed_offset, "surface": surface, "wake_nodes": wake_nodes}


class Containment(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp(prefix="containment-", dir=os.environ.get("TMP")))
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        for name in ("reference_body.adb", "reference_body.vspgeom", "reference_body.vkey"):
            shutil.copyfile(CASE / name, self.root / name)
        self.raw = bytearray((self.root / "reference_body.adb").read_bytes())
        self.map = layout(bytes(self.raw))

    def point(self, trail, node):
        record = self.map["records"][trail - 1]
        return list(struct.unpack_from("<3d", self.raw, record["offset"] + 24 * (node - 1)))

    def set_point(self, trail, node, xyz):
        record = self.map["records"][trail - 1]
        struct.pack_into("<3d", self.raw, record["offset"] + 24 * (node - 1), *xyz)
        packed = self.map["packed"] + 12 * (self.map["surface"] + (trail - 1) * self.map["wake_nodes"] + node - 1)
        struct.pack_into("<3f", self.raw, packed, *xyz)

    def write(self):
        (self.root / "reference_body.adb").write_bytes(bytes(self.raw))
        return tool.assess(self.root)

    def clear_wake(self):
        """Every interior node of every non-concave trail moved radially to four times its root radius."""
        for trail, record in enumerate(self.map["records"], 1):
            if record["count"] == 1:
                continue
            root = self.point(trail, 1)
            for node in range(2, record["count"] + 1):
                x = self.point(trail, node)[0]
                self.set_point(trail, node, [x, 4 * root[1], 4 * root[2]])

    def failing_points(self, result):
        return sorted((item["trail"], point["node"]) for item in result["failures"] for point in item["points"]
                      if point["state"] == "inside")

    def test_the_archived_alpha0_interior_points_of_the_reference_are_found(self):
        result = self.write()
        self.assertFalse(result["passed"])
        self.assertEqual(self.failing_points(result), [(14, 2), (20, 2), (21, 3)])
        strength = {trail["trail"]: trail for trail in result["trail_details"]}
        for trail in REFERENCE["trails"]:
            self.assertEqual(abs(strength[trail["trail"]]["strength"]), abs(trail["derived_streamwise_gamma"]))
            self.assertEqual(strength[trail["trail"]]["concave"], trail["concave_internal_record"])
            self.assertEqual(strength[trail["trail"]]["active"], trail["effective_induction_active"])

    def test_a_wake_clear_of_the_body_passes_although_disabled_trails_stay_inside(self):
        self.clear_wake()
        result = self.write()
        self.assertTrue(result["passed"], result["failures"][:2])
        self.assertEqual(sorted(item["trail"] for item in result["disabled_not_outside"]), [1, 7, 13, 19])

    def test_one_active_node_inside_the_body_fails(self):
        self.clear_wake()
        x = self.point(2, 3)[0]
        self.set_point(2, 3, [x, 0.0, 0.0])
        result = self.write()
        self.assertFalse(result["passed"])
        self.assertEqual(self.failing_points(result), [(2, 3)])

    def test_a_node_on_the_body_surface_is_not_certified(self):
        self.clear_wake()
        # a body vertex at x = 2.0 on the cylinder: distance 0, inside the binary32 uncertainty
        vertex = next(point for point in tool.geometry_reader.read_vspgeom(self.root / "reference_body.vspgeom")["points"]
                      if abs(point[0] - 2.0) < 0.3 and 0.088 < (point[1] ** 2 + point[2] ** 2) ** 0.5 < 0.09)
        self.set_point(2, 3, list(vertex))
        result = self.write()
        self.assertFalse(result["passed"])
        states = [point["state"] for item in result["failures"] for point in item["points"]]
        self.assertIn("uncertain", states)

    def test_a_segment_through_the_body_between_outside_nodes_fails(self):
        self.clear_wake()
        self.set_point(2, 2, [2.0, 0.0, 0.3])
        self.set_point(2, 3, [2.1, 0.0, -0.3])
        result = self.write()
        self.assertFalse(result["passed"])
        failure = next(item for item in result["failures"] if item["trail"] == 2)
        self.assertEqual(failure["points"], [])
        self.assertTrue(any(segment["inside_samples"] for segment in failure["segments"]))

    def test_a_zero_strength_trail_is_disabled_data(self):
        # equal adjacent strip values make trail 14 carry no circulation; its interior node no longer fails
        strips = {tuple(strip["te_nodes"]): strip for strip in REFERENCE["strips"]}
        chain = [976, 985]
        left = next(strip for key, strip in strips.items() if key[1] == 976)
        right = strips[tuple(chain)]
        (left_owner,), (right_owner,) = left["owners"], right["owners"]
        left_gamma = struct.unpack_from("<d", self.raw, self.map["gamma"] + 16 * (left_owner["loop"] - 1))[0]
        value = left_owner["direction"] * left_gamma * right_owner["direction"]
        struct.pack_into("<d", self.raw, self.map["gamma"] + 16 * (right_owner["loop"] - 1), value)
        result = self.write()
        trail = next(item for item in result["trail_details"] if item["trail"] == 14)
        self.assertEqual(trail["strength"], 0.0)
        self.assertFalse(trail["active"])
        self.assertEqual(self.failing_points(result), [(20, 2), (21, 3)])

    def test_an_internal_point_that_differs_from_its_packed_copy_is_refused(self):
        record = self.map["records"][1]
        struct.pack_into("<d", self.raw, record["offset"] + 24 + 8, self.point(2, 2)[1] + 1e-3)
        with self.assertRaises(ValueError):
            self.write()

    def test_nonfinite_circulation_is_refused(self):
        struct.pack_into("<d", self.raw, self.map["gamma"], float("nan"))
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.write()


if __name__ == "__main__":
    unittest.main()
