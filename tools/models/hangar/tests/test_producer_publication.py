"""Whole-file producer I/O fixtures; no aircraft/model/native SDK is executed."""
import ast
import copy
import ctypes
import errno
import io
import json
import os
from pathlib import Path
import pickle
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from hangar import damage, files, model3d, pieces, pipeline, rotorcraft
from hangar.rotorcraft import heli
from test_glb_publication import mapped_reader

SOURCE = Path(__file__).resolve().parents[1]/'hangar'
OWNERS = {
    'pipeline.py': {'write_text', 'Design.save', 'Design.build_with'},
    'damage.py': {'write'}, 'pieces.py': {'write'},
    'rotorcraft/heli.py': {'_write'},
    'rotorcraft/__init__.py': {'Rotorcraft.save', 'Rotorcraft.write_hover'},
    'storestages.py': {'main'}, 'model3d.py': {'refresh_engine_lines'},
}


def writer_contexts():
    result = []
    for name, allowed in OWNERS.items():
        class Visitor(ast.NodeVisitor):
            stack = []
            def visit_ClassDef(self, node):
                self.stack.append(node.name); self.generic_visit(node); self.stack.pop()
            def visit_FunctionDef(self, node):
                self.stack.append(node.name); self.generic_visit(node); self.stack.pop()
            def visit_With(self, node):
                scope = '.'.join(self.stack)
                if scope in allowed:
                    for item in node.items:
                        call = item.context_expr
                        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id in ('open_write', 'atomic_write_stream'):
                            result.append((name, scope, node))
                self.generic_visit(node)
        Visitor().visit(ast.parse((SOURCE/name).read_text(encoding='utf-8')))
    return result


@unittest.skipUnless(os.name == 'nt', 'Actual Windows mapped-file publication fixtures')
class ProducerPublication(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir='D:/ENV/tmp')
        self.addCleanup(self.temp.cleanup); self.root = Path(self.temp.name)
        self.old = b'old complete producer bytes\n'*4
        # Bound only the negative fixture's retry wait. Production remains20s.
        for patch in (mock.patch.object(files, 'PATIENCE_S', .005),
                      mock.patch.object(files, 'PAUSE_S', .001),
                      mock.patch.object(files.time, 'sleep')):
            patch.start(); self.addCleanup(patch.stop)

    def mapped_call(self, path, function, expected=None, delete=True):
        path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(self.old)
        with mapped_reader(path, delete) as view:
            result = function()
            if expected is not None: self.assertEqual(path.read_bytes(), expected)
            self.assertEqual(ctypes.string_at(view, len(self.old)), self.old)
        self.assertFalse(list(path.parent.glob('*.tmp')))
        return result

    def test_actual_atomic_text_retains_utf8_lf_bytes_on_mapped_publication(self):
        path = self.root/'material.json'; text = '{"value":"\u03bc","kg":1.234567890123}\n'
        self.mapped_call(path, lambda: files.atomic_write_text(path, text), text.encode('utf-8'))

    def test_legacy_replace_preserves_original_refusal_identity_deadline_and_nonsharing_errors(self):
        source = self.root/'source.tmp'; source.write_bytes(b'closed stage')
        target = self.root/'target'; target.write_bytes(self.old)
        original = ctypes.WinError(5); unsupported = ctypes.WinError(50)
        with mock.patch.object(files.os, 'replace', side_effect=original) as ordinary, \
             mock.patch.object(files, '_windows_rename_closed', side_effect=unsupported), \
             mock.patch.object(files.time, 'monotonic', side_effect=[0., 0., 1.]):
            with self.assertRaises(OSError) as refused: files.replace(source, target)
        self.assertIs(refused.exception, original); self.assertEqual(ordinary.call_count, 2)
        io_error = OSError(errno.ENOSPC, 'independent disk failure')
        with mock.patch.object(files.os, 'replace', side_effect=io_error), \
             mock.patch.object(files, '_windows_rename_closed') as native:
            with self.assertRaises(OSError) as refused: files.replace(source, target)
        self.assertIs(refused.exception, io_error); native.assert_not_called()
        self.assertEqual(target.read_bytes(), self.old); self.assertEqual(source.read_bytes(), b'closed stage')

    def test_real_nondelete_sharing_refuses_legacy_text_and_cleans_owned_stage(self):
        path = self.root/'material.json'; path.write_bytes(self.old)
        with mapped_reader(path, False) as view:
            with self.assertRaises(OSError): files.atomic_write_text(path, 'new complete text\n')
            self.assertEqual(path.read_bytes(), self.old)
            self.assertEqual(ctypes.string_at(view, len(self.old)), self.old)
        self.assertEqual([p.name for p in self.root.iterdir()], ['material.json'])

    def test_actual_pipeline_text_shortcut_return_and_binary_cache_bytes(self):
        path = self.root/'airframe.xml'; text = '<mass unit="KG">1.234567890123</mass>\n'
        self.assertIs(self.mapped_call(path, lambda: pipeline.write_text(str(path), text), text.encode()), True)
        with mapped_reader(path, True), mock.patch.object(pipeline, 'atomic_write_stream', create=True,
                                                         side_effect=AssertionError('unchanged file must not publish')):
            self.assertIs(pipeline.write_text(str(path), text), False)
        data = {'tables': [1., 2.**-40, -3.25], 'schema': 'fixture'}
        cache = self.root/'tables.pkl'
        self.mapped_call(cache, lambda: pipeline._write_table_cache(str(cache), data), pickle.dumps(data))

    def namespace(self, directory):
        xml = '<fixture label="\u03bc">1.234567890123</fixture>\n'
        render = lambda *args: xml
        return dict(os=os, json=json, float=float, _json=pipeline._json, open_write=files.open_write,
            atomic_write_stream=files.atomic_write_stream,
            path=str(directory/'fixture.xml'), xml_path=str(directory/'airframe.xml'),
            manifest=str(directory/'fixture.manifest'), eng_dir=str(directory), ename='engine', pname='prop',
            self=SimpleNamespace(out=str(directory)), stage='fixture', data={'value':1.234567890123, 'name':'\u03bc'},
            text=xml, e=SimpleNamespace(name='engine', type='piston'), a=SimpleNamespace(name='airframe'), i=0,
            p=SimpleNamespace(tables=lambda: [], mach_tables=lambda tab: []), tab=[], mass_e=13.,
            turbofan_xml=render, nozzle_xml=render, turboprop_xml=render, constant_speed_propeller_xml=render,
            piston_xml=render, electric_xml=render, propeller_xml=render, blades_mass=lambda e: 3.,
            stages=['model'], forced=True, jobs=2, rows=[{'name':'fixture', 'mass':1.234567890123}],
            names=['fixture'], results={'fixture':0}, failed=0)

    def test_all16_actual_complete_writer_contexts_retain_serializer_bytes(self):
        contexts = writer_contexts(); self.assertEqual(len(contexts), 16)
        for number, (name, scope, node) in enumerate(contexts):
            with self.subTest(source=name, scope=scope, line=node.lineno):
                directory = self.root/str(number); directory.mkdir(); namespace = self.namespace(directory)
                call = node.items[0].context_expr
                path = Path(eval(compile(ast.Expression(copy.deepcopy(call.args[0])), name, 'eval'), namespace))
                oracle = copy.deepcopy(node); oracle.items[0].context_expr.func.id = 'open_write'
                exec(compile(ast.fix_missing_locations(ast.Module(body=[oracle], type_ignores=[])), name, 'exec'), namespace)
                expected = path.read_bytes()
                self.mapped_call(path, lambda: exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(node)], type_ignores=[])), name, 'exec'), namespace), expected)
                self.assertEqual(call.func.id, 'atomic_write_stream')

    def test_damage_serialization_retains_physical_precision_rounding_and_shortcut(self):
        data = dict(origin_design_m=[1.234567890123, 0., 0.], mass_partition={'kg':1.234567890123},
                    sections=[dict(mass_kg=1.234567890123, cg=[.1234567890123, 0., 0.],
                                   inertia=[1.234567890123]*6, strength=1.234567890123)], other=1.234567890123)
        expected = copy.deepcopy(data); expected['other']=1.23457; expected['sections'][0]['strength']=1.23457
        text = json.dumps(expected, indent=1, allow_nan=False)+'\n'; path = self.root/'fixture.damage.json'
        with mock.patch.object(damage, 'damage_data', return_value=data):
            self.assertEqual(self.mapped_call(path, lambda: damage.write(None, str(path), None), text.encode()), expected)
            with mapped_reader(path, True), mock.patch.object(damage, 'atomic_write_stream', create=True,
                                                             side_effect=AssertionError('unchanged damage must not publish')):
                self.assertEqual(damage.write(None, str(path), None), expected)

    def test_piece_publication_retains_owned_path_and_unchanged_return(self):
        data = dict(aircraft='fixture'); text='<piece kg="1.234567890123"/>\n'
        name = pieces.piece_name('fixture','nose'); path=self.root/'pieces'/name/(name+'.xml')
        with mock.patch.object(pieces, 'piece_xml', side_effect=lambda data, section: text if section=='nose' else None):
            expected=[str(path)]
            self.assertEqual(self.mapped_call(path, lambda: pieces.write(data,str(self.root)), text.encode()), expected)
            with mapped_reader(path, True), mock.patch.object(pieces, 'atomic_write_stream', create=True,
                                                             side_effect=AssertionError('unchanged piece must not publish')):
                self.assertEqual(pieces.write(data,str(self.root)), expected)

    def test_rotor_xml_and_existing_manifest_refresh_keep_bytes_and_returns(self):
        path=self.root/'rotor.xml';text='<rotor kg="1.234567890123"/>\n'
        self.assertIsNone(self.mapped_call(path,lambda:heli._write(str(path),text),text.encode()))
        manifest=self.root/'fixture.manifest';manifest.write_text('forward -y\nengine 0 1 1\nother\n',encoding='utf-8')
        expected=b'forward -y\nengine 0 0.5 0\nother\n'; old=manifest.read_bytes()
        with mock.patch.object(model3d,'engine_lines',return_value=['engine 0 0.5 0']):
            with mapped_reader(manifest,True) as view:
                self.assertIs(model3d.refresh_engine_lines(None,str(manifest)),True)
                self.assertEqual(manifest.read_bytes(),expected);self.assertEqual(ctypes.string_at(view,len(old)),old)
            self.assertIs(model3d.refresh_engine_lines(None,str(manifest)),False)

    def test_both_save_methods_keep_metadata_returns_and_default_json_newlines(self):
        for index, (kind, module) in enumerate(((pipeline.Design,pipeline),(rotorcraft.Rotorcraft,rotorcraft))):
            with self.subTest(kind=kind.__name__):
                directory=self.root/str(index);directory.mkdir();writer=object.__new__(kind);writer.out=str(directory)
                data={'mass_kg':1.234567890123,'label':'\u03bc','count':2}
                expected=dict(data,stage='mass',hangar=module.__version__,time='2000-01-02 03:04:05')
                buffer=io.BytesIO();text=io.TextIOWrapper(buffer,encoding='utf-8',newline=None)
                text.write(json.dumps(expected,indent=1));text.flush();raw=buffer.getvalue();text.detach()
                with mock.patch.object(module.time,'strftime',return_value=expected['time']):
                    self.assertEqual(self.mapped_call(directory/'mass.json',lambda:writer.save('mass',data),raw),expected)

    def test_hover_writer_keeps_exact_text_and_none_return(self):
        writer=object.__new__(rotorcraft.Rotorcraft);writer.dir=str(self.root);writer.name='fixture'
        section={'altitude_m':1234.5,'mass_kg':1000.123456789,'roll_attitude_deg':1.25,'pitch_attitude_deg':-2.5}
        for command in ('throttle','aileron','elevator','rudder'):section[command+'_trim']=.123456789
        for axis in writer.HOVER_AXES:
            for field in ('power','damping','lag_s'):section[axis+'/'+field]=.123456789
        with mock.patch.object(rotorcraft.time,'strftime',return_value='2000-01-02 03:04'):
            self.assertIsNone(writer.write_hover(section));path=self.root/'hover.toml';expected=path.read_bytes()
            self.assertIsNone(self.mapped_call(path,lambda:writer.write_hover(section),expected))

    def test_serializer_failure_preserves_old_complete_stage_and_cleans_owned_temp(self):
        writer=object.__new__(rotorcraft.Rotorcraft);writer.out=str(self.root)
        path=self.root/'mass.json';path.write_bytes(self.old)
        with mapped_reader(path,True):
            with self.assertRaises(TypeError):writer.save('mass',{'unsupported':object()})
        self.assertEqual(path.read_bytes(),self.old)
        self.assertEqual([p.name for p in self.root.iterdir()],['mass.json'])


if __name__ == '__main__': unittest.main()
