"""Normal Python and CMake staging retire only obsolete generated piece XML."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from hangar import runtime_products

SOURCE=Path(__file__).resolve().parents[4]


class RuntimePieceEpoch(unittest.TestCase):
    def test_both_normal_stagers_propagate_removal_and_preserve_neighbor_files(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp') as temporary:
            root=Path(temporary);source=root/'aircraft/plane';source.mkdir(parents=True)
            (source/'plane.toml').write_text('[aircraft]\ncategory="fighter"\n',encoding='utf-8')
            (source/'plane.xml').write_text('current flight',encoding='utf-8')
            (source/'plane.damage.json').write_text('{}',encoding='utf-8')
            (source/'plane.glb').write_bytes(b'current model')
            (source/'plane.glb.manifest').write_text('current manifest',encoding='utf-8')
            (source/'plane.liquid-v1.json').write_text('{}',encoding='utf-8')
            nose=source/'pieces/plane-nose/plane-nose.xml';nose.parent.mkdir(parents=True);nose.write_text('current nose',encoding='utf-8')
            data=root/'build/runtime/share/flightsim';data.mkdir(parents=True)
            target=data/'aircraft/plane'
            tail=target/'pieces/plane-tail/plane-tail.xml';tail.parent.mkdir(parents=True)
            note=tail.with_name('user-notes.txt');note.write_text('keep',encoding='utf-8')
            unrelated=target/'pieces/user.xml';unrelated.write_text('keep',encoding='utf-8')
            for route in ('python','cmake'):
                with self.subTest(route=route):
                    tail.write_text('obsolete generated section',encoding='utf-8')
                    if route=='python':
                        with patch.object(runtime_products.pipeline,'repo_root',return_value=str(root)):
                            runtime_products.stage_runtime_products(source/'plane.toml',data)
                    else:
                        subprocess.run(['D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/cmake.exe',
                            '-DSRC='+str(root/'aircraft'),'-DDEST='+str(data/'aircraft'),'-P',str(SOURCE/'cmake/StageDesigns.cmake')],
                            check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=30)
                    self.assertFalse(tail.exists())
                    self.assertEqual(note.read_text(encoding='utf-8'),'keep')
                    self.assertEqual(unrelated.read_text(encoding='utf-8'),'keep')
                    self.assertEqual((target/'pieces/plane-nose/plane-nose.xml').read_text(encoding='utf-8'),'current nose')
                    self.assertEqual((target/'plane.liquid-v1.json').read_text(encoding='utf-8'),'{}')


if __name__=='__main__':unittest.main(verbosity=2)
