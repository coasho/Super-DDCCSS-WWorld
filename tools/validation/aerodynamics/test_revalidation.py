"""Reject stale authored inputs, products and numerical generation methods."""
import hashlib
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0,str(Path(__file__).resolve().parent))
import engineering_revalidate as revalidation


class RevalidationInputs(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root=Path(self.temp.name)
        self.current=root/'current';self.source=root/'generation'
        paths={'aircraft/example/example.toml':b'current authored geometry',
               'tools/models/hangar/hangar/aero/model.py':b'current numerical method',
               'aircraft/example/example.xml':b'current generated coefficients'}
        for base in (self.current,self.source):
            for relative,value in paths.items():
                p=base/relative;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(value)
        digest=lambda relative:hashlib.sha256(paths[relative]).hexdigest()
        self.row=dict(input_sha256={'aircraft/example/example.toml':digest('aircraft/example/example.toml')},
            generator_sha256={'tools/models/hangar/hangar/aero/model.py':digest('tools/models/hangar/hangar/aero/model.py')},
            products={'example/example.xml':digest('aircraft/example/example.xml')})
        patch=mock.patch.object(revalidation,'ROOT',self.current);patch.start();self.addCleanup(patch.stop)

    def test_unchanged_generation_is_eligible_without_relabeling_measurement(self):
        self.assertEqual(revalidation.verify_generation(self.row,self.source),[])

    def test_changed_authored_geometry_rejects_reuse(self):
        (self.current/'aircraft/example/example.toml').write_bytes(b'different geometry')
        with self.assertRaisesRegex(ValueError,'Changed authored input'):
            revalidation.verify_generation(self.row,self.source)

    def test_stale_runtime_product_rejects_reuse(self):
        (self.current/'aircraft/example/example.xml').write_bytes(b'stale product')
        with self.assertRaisesRegex(ValueError,'Changed generated product'):
            revalidation.verify_generation(self.row,self.source)

    def test_changed_solver_requires_regeneration(self):
        (self.current/'tools/models/hangar/hangar/aero/model.py').write_bytes(b'new method')
        with self.assertRaisesRegex(ValueError,'requires regeneration'):
            revalidation.verify_generation(self.row,self.source)

    def test_mutated_generation_source_cannot_back_a_receipt(self):
        (self.source/'tools/models/hangar/hangar/aero/model.py').write_bytes(b'forged history')
        with self.assertRaisesRegex(ValueError,'Historical generation source drift'):
            revalidation.verify_generation(self.row,self.source)


if __name__=='__main__':unittest.main()
