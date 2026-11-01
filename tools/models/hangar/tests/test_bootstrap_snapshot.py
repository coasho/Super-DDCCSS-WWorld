"""A source export uses clone checkout rules without changing user staging."""
import hashlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from tests.paths import TEMP_ROOT

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tools/bootstrap'))
import snapshot


class SourceCheckout(unittest.TestCase):
    def test_git_checkout_preserves_byte_bound_products_and_the_real_index(self):
        import subprocess
        if not (ROOT/'.git').is_dir():
            self.skipTest('Git source-export fixture requires its original repository')
        git_root=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','--show-toplevel'],text=True).strip()
        if Path(git_root).resolve()!=ROOT:
            self.skipTest('The tested export intentionally has no Git metadata')
        index=ROOT/'.git/index'
        before=hashlib.sha256(index.read_bytes()).hexdigest()
        paths=('.gitattributes','README.md','tools/bootstrap/dependencies.lock.json','aircraft/c17a/c17a.xml')
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            destination=Path(folder)/'checkout'
            with patch.object(snapshot,'SOURCE_PATHS',paths):record=snapshot.snapshot(ROOT,destination)
            self.assertEqual(record['file_count'],len(paths))
            self.assertEqual((ROOT/paths[-1]).read_bytes(),(destination/paths[-1]).read_bytes())
            self.assertNotIn(b'\r\n',(destination/'tools/bootstrap/dependencies.lock.json').read_bytes())
        self.assertEqual(hashlib.sha256(index.read_bytes()).hexdigest(),before)


if __name__=='__main__':unittest.main()
