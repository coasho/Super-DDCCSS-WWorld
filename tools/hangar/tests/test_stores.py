"""The stores' models (hangar/stores.py): each a valid glTF binary, the same bytes each time it is written - the one
in assets/models is what `python -m hangar store` makes."""
import os
import struct
import tempfile
import unittest

from hangar import pipeline, stores


class StoresTest(unittest.TestCase):
    def test_written_as_committed(self):
        for name in stores.STORES:
            with tempfile.TemporaryDirectory() as d:
                path = stores.write_store(name, os.path.join(d, name + ".glb"))
                data = open(path, "rb").read()
            magic, version, length = struct.unpack("<III", data[:12])
            self.assertEqual(magic, 0x46546C67)
            self.assertEqual(version, 2)
            self.assertEqual(length, len(data))
            committed = os.path.join(pipeline.repo_root(), "assets", "models", name + ".glb")
            with open(committed, "rb") as f:
                self.assertEqual(f.read(), data, f"{committed} is not what `python -m hangar store {name}` writes")


if __name__ == "__main__":
    unittest.main()
