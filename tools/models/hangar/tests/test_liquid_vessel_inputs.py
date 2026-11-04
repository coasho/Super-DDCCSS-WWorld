"""Optional vessel declaration addition/edit/removal must invalidate reuse."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from hangar import engineering
from hangar import method_backends
import test_generation_input_epoch as existing_fixtures


class VesselInputEpochTests(unittest.TestCase):
    def test_absence_addition_edit_and_removal_are_distinct_current_input_states(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp') as temporary:
            root = Path(temporary); folder = root/'aircraft/rq4b'; folder.mkdir(parents=True)
            source = folder/'rq4b.toml'; source.write_text('[aircraft]\nname="rq4b"\n', encoding='utf-8')
            anchor = root/'tools/models/hangar/hangar/engineering.py'; anchor.parent.mkdir(parents=True)
            anchor.with_name('public_data.json').write_text('{}', encoding='utf-8')
            record = dict(path=str(source)); key = 'aircraft/rq4b/rq4b.liquid-geometry-v1.json'
            with patch.object(engineering, 'ROOT', root), patch.object(engineering, '__file__', str(anchor)):
                absent = engineering.generation_inputs(record)
                self.assertFalse(absent[1][key]); self.assertNotIn(key, absent[0])
                policy = folder/'rq4b.liquid-geometry-v1.json'; policy.write_text('version A', encoding='utf-8')
                first = engineering.generation_inputs(record); self.assertTrue(first[1][key])
                policy.write_text('version B', encoding='utf-8')
                second = engineering.generation_inputs(record); self.assertNotEqual(first[0][key], second[0][key])
                policy.unlink(); self.assertEqual(engineering.generation_inputs(record), absent)

    def test_reuse_independently_refuses_changed_or_omitted_optional_vessel_state(self):
        fixture = existing_fixtures.GenerationInputEpoch('test_paint_absence_addition_edit_and_removal_are_input_states')
        fixture.setUp(); self.addCleanup(fixture.doCleanups)
        path = fixture.design/'plane.liquid-geometry-v1.json'
        with patch.object(method_backends, 'verify', return_value=True) as backend:
            folder, _ = fixture.receipt(); self.assertIsNotNone(engineering.reusable_generation(folder, fixture.record))
            for change in ('addition', 'edit', 'removal'):
                if change != 'addition': path.write_text('old source policy', encoding='utf-8')
                folder, _ = fixture.receipt(); backend.reset_mock()
                if change == 'removal': path.unlink()
                else: path.write_text('new source policy', encoding='utf-8')
                self.assertIsNone(engineering.reusable_generation(folder, fixture.record)); backend.assert_not_called()
            import json
            folder, row = fixture.receipt(); row['input_presence'].pop('aircraft/plane/plane.liquid-geometry-v1.json')
            (folder/'generation.json').write_text(json.dumps(row), encoding='utf-8'); backend.reset_mock()
            self.assertIsNone(engineering.reusable_generation(folder, fixture.record)); backend.assert_not_called()


if __name__ == '__main__': unittest.main(verbosity=2)
