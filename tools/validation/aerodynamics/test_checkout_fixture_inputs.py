"""Only explicit project-local inputs can support a Git-only fixture."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('verification',Path(__file__).with_name('engineering_verify.py'))
verification=importlib.util.module_from_spec(spec);spec.loader.exec_module(verification)


class CheckoutFixtureInputs(unittest.TestCase):
    def test_reads_the_unchanged_fixture_declaration(self):
        root=Path(__file__).resolve().parents[3]
        source=(root/'tools/models/hangar/tests/test_bootstrap_snapshot.py').read_text(encoding='utf-8')
        paths=verification.fixture_input_paths(source)
        self.assertEqual(len(paths),4)
        self.assertTrue(all((root/path).is_file() for path in paths))

    def test_rejects_missing_duplicate_and_executable_declarations(self):
        for source in ('value=1','paths=("a",)\npaths=("b",)','paths=tuple(read_paths())'):
            with self.subTest(source=source),self.assertRaises((ValueError,TypeError)):
                verification.fixture_input_paths(source)

    def test_rejects_nonlocal_or_excluded_inputs_without_accessing_them(self):
        for path in ('../escape','D:/outside','docs/.CoDeX/forbidden','docs\\.CoDeX\\forbidden','/absolute','D:relative','','.'):
            with self.subTest(path=path),self.assertRaises(ValueError):
                verification.fixture_input_paths('paths='+repr((path,)))

    def test_rejects_nonstring_empty_and_unbounded_declarations(self):
        for paths in ((1,),(),tuple('entry'+str(i) for i in range(33))):
            with self.subTest(paths=paths),self.assertRaises(ValueError):
                verification.fixture_input_paths('paths='+repr(paths))


if __name__=='__main__':unittest.main()
