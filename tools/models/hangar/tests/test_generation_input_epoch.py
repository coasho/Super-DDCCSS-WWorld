"""Changed optional model inputs invalidate byte-bound generation reuse."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from hangar import engineering, method_backends


class GenerationInputEpoch(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory(dir='D:/ENV/tmp');self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name);self.design=self.root/'aircraft/plane';self.design.mkdir(parents=True)
        self.source=self.design/'plane.toml';self.source.write_text('[aircraft]\nname="plane"\ncategory="fighter"\n',encoding='utf-8')
        self.record=dict(name='plane',category='fixed-wing',path=str(self.source))
        self.scope=patch.object(engineering,'ROOT',self.root);self.scope.start();self.addCleanup(self.scope.stop)
        anchor=self.root/'tools/models/hangar/hangar/engineering.py';anchor.parent.mkdir(parents=True)
        anchor.with_name('public_data.json').write_text('{}',encoding='utf-8')
        module_scope=patch.object(engineering,'__file__',str(anchor));module_scope.start();self.addCleanup(module_scope.stop)

    def receipt(self):
        # Only the file-freshness mechanism is exercised here. These are
        # opaque test payloads, not flight/material or native admission data.
        folder=self.root/'receipt';folder.mkdir(exist_ok=True)
        products={}
        for suffix in ('.xml','.damage.json','.glb','.glb.manifest','.liquid-v1.json'):
            product=self.design/('plane'+suffix);product.write_text('test payload '+suffix,encoding='utf-8')
            products[product.relative_to(self.root/'aircraft').as_posix()]=engineering.sha(product)
        library=self.root/'build/ucrt64-release/python/fsim/bin/libfsim.dll';library.parent.mkdir(parents=True,exist_ok=True)
        library.write_bytes(b'explicit test double')
        reference=folder/'reference.json';reference.write_text('{}',encoding='utf-8')
        inputs,presence=engineering.generation_inputs(self.record)
        row=dict(self.record,products=products,checks=[dict(passed=True)],input_sha256=inputs,input_presence=presence,
            generator_sha256={},native_dll_sha256=engineering.sha(library),
            reference_bundle=dict(name=reference.name,sha256=engineering.sha(reference)),
            normal_model=dict(regenerated=True,checks=[dict(name='fixture',status='info')],method_backends={'test':True},
                products=['plane/plane'+suffix for suffix in ('.glb','.glb.manifest','.liquid-v1.json')]))
        (folder/'generation.json').write_text(json.dumps(row),encoding='utf-8');return folder,row

    def test_paint_absence_addition_edit_and_removal_are_input_states(self):
        absent,absent_presence=engineering.generation_inputs(self.record)
        key='aircraft/plane/paint.toml';self.assertFalse(absent_presence[key]);self.assertNotIn(key,absent)
        paint=self.design/'paint.toml';paint.write_text('colour="red"\n',encoding='utf-8')
        red,presence=engineering.generation_inputs(self.record);self.assertTrue(presence[key]);self.assertIn(key,red)
        paint.write_text('colour="blue"\n',encoding='utf-8')
        blue,_=engineering.generation_inputs(self.record);self.assertNotEqual(red[key],blue[key])
        paint.unlink();self.assertEqual(engineering.generation_inputs(self.record),(absent,absent_presence))

    def test_reuse_independently_reconciles_optional_inputs_before_backend_probe(self):
        paint=self.design/'paint.toml'
        with patch.object(method_backends,'verify',return_value=True) as backend:
            folder,_=self.receipt();self.assertIsNotNone(engineering.reusable_generation(folder,self.record))
            for change in ('add','edit','remove'):
                if change!='add':paint.write_text('old painting',encoding='utf-8')
                folder,_=self.receipt();backend.reset_mock()
                if change=='remove':paint.unlink()
                else:paint.write_text('new painting',encoding='utf-8')
                self.assertIsNone(engineering.reusable_generation(folder,self.record));backend.assert_not_called()
            folder,row=self.receipt();row['input_presence'].pop('aircraft/plane/paint.toml')
            (folder/'generation.json').write_text(json.dumps(row),encoding='utf-8')
            self.assertIsNone(engineering.reusable_generation(folder,self.record))

    def test_reuse_rejects_unnamed_checks_and_unhashed_required_model(self):
        with patch.object(method_backends,'verify',return_value=True):
            for fault in ('missing name','blank name','unhashed liquid'):
                folder,row=self.receipt()
                if fault=='missing name':row['normal_model']['checks'][0].pop('name')
                elif fault=='blank name':row['normal_model']['checks'][0]['name']='  '
                else:row['products'].pop('plane/plane.liquid-v1.json')
                (folder/'generation.json').write_text(json.dumps(row),encoding='utf-8')
                with self.subTest(fault=fault):self.assertIsNone(engineering.reusable_generation(folder,self.record))


if __name__=='__main__':unittest.main(verbosity=2)
