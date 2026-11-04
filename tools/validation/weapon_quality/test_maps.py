"""Independent map provenance fixtures; no SDK, downloads or native execution."""
import base64
from contextlib import contextmanager
import json
import os
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import maps

PNG=base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=')


class MapProvenance(unittest.TestCase):
    def test_reserved_superscript_device_names_are_refused(self):
        for device in ('COM','LPT'):
            for suffix in ('\u00b9','\u00b2','\u00b3'):
                with self.subTest(device=device,suffix=suffix),self.assertRaises(ValueError):
                    maps.identity('cache/'+device+suffix+'.png')
        self.assertEqual(maps.identity('cache/tile-\u00b9.png'),'cache/tile-\u00b9.png')

    def fixture(self,directory):
        root=Path(directory)/'maps';root.mkdir()
        for prefix in maps.REQUIRED.values():
            tile=root/prefix/'0/0/0.png';tile.parent.mkdir(parents=True);tile.write_bytes(PNG)
        return root

    def temporary(self):
        return tempfile.TemporaryDirectory(dir='D:/ENV/tmp' if os.name=='nt' else None)

    def test_exact_frozen_tree_retains_required_image_inputs(self):
        with self.temporary() as directory:
            root=self.fixture(directory);start=maps.snapshot(root)
            start_path=Path(directory)/'maps-start.json';start_path.write_bytes(maps.canonical(start))
            final=maps.freeze(root,start,start_path)
            receipt=Path(directory)/'maps.json';receipt.write_bytes(maps.canonical(final))
            actual=maps.verify_frozen(root,receipt)
            self.assertEqual(actual['file_count'],2)
            self.assertEqual(actual['total_bytes'],2*len(PNG))
            self.assertTrue(all(row['files']==1 for row in actual['required_inputs'].values()))
            self.assertEqual(actual['inventory_sha256'],actual['start_inventory_sha256'])

    def test_empty_or_missing_required_input_tree_is_refused(self):
        with self.temporary() as directory:
            root=Path(directory)/'maps';root.mkdir()
            with self.assertRaises(ValueError):maps.snapshot(root)
            tile=root/maps.REQUIRED['terrain']/'0/0/0.png';tile.parent.mkdir(parents=True);tile.write_bytes(PNG)
            with self.assertRaises(ValueError):maps.snapshot(root)

    def test_freeze_refuses_changed_or_added_map_bytes(self):
        with self.temporary() as directory:
            root=self.fixture(directory);start=maps.snapshot(root)
            start_path=Path(directory)/'maps-start.json';start_path.write_bytes(maps.canonical(start))
            tile=root/maps.REQUIRED['terrain']/'0/0/0.png';tile.write_bytes(PNG+b'changed')
            with self.assertRaises(ValueError):maps.freeze(root,start,start_path)
            tile.write_bytes(PNG);(root/'added.bin').write_bytes(b'unobserved')
            with self.assertRaises(ValueError):maps.freeze(root,start,start_path)

    def test_invalid_required_image_header_is_refused(self):
        with self.temporary() as directory:
            root=self.fixture(directory)
            (root/maps.REQUIRED['imagery']/'0/0/0.png').write_bytes(b'not an image')
            with self.assertRaisesRegex(ValueError,'image input'):maps.snapshot(root)

    def test_excluded_names_are_filtered_before_type_or_path_access(self):
        with self.temporary() as directory:
            root=self.fixture(directory);real_scan=os.scandir
            class Excluded:
                def __init__(self,name):self.name=name
                def stat(self,*args,**kwargs):raise AssertionError('Excluded entry was inspected')
            @contextmanager
            def guarded(path):
                with real_scan(path) as entries:
                    yield [*entries,Excluded('.codex'),Excluded('.git')]
            with patch('maps.os.scandir',guarded):result=maps.snapshot(root)
            self.assertEqual(result['file_count'],2)

    def test_cached_directory_entry_zero_identifiers_do_not_replace_file_identity(self):
        with self.temporary() as directory:
            root=self.fixture(directory);real_scan=os.scandir
            class Entry:
                def __init__(self,entry):self.entry=entry;self.name=entry.name;self.path=entry.path
                def stat(self,**kwargs):
                    value=self.entry.stat(**kwargs)
                    return SimpleNamespace(st_dev=0,st_ino=0,st_mode=value.st_mode,
                        st_file_attributes=getattr(value,'st_file_attributes',0))
            @contextmanager
            def cached(path):
                with real_scan(path) as entries:yield [Entry(entry) for entry in entries]
            with patch('maps.os.scandir',cached):result=maps.snapshot(root)
            self.assertEqual(result['file_count'],2)

    def test_reparse_entry_is_refused_before_opening(self):
        with self.temporary() as directory:
            root=self.fixture(directory)
            entry=SimpleNamespace(name='link',path=str(root/'link'),
                stat=lambda **kwargs:SimpleNamespace(st_mode=stat.S_IFLNK,st_file_attributes=0))
            @contextmanager
            def linked_scan(path):yield [entry]
            with patch('maps.os.scandir',linked_scan),self.assertRaisesRegex(ValueError,'links/reparse'):
                maps.snapshot(root)

    def test_initial_receipt_and_same_bytes_different_root_are_not_substitutable(self):
        with self.temporary() as directory:
            root=self.fixture(directory);start=maps.snapshot(root)
            start_path=Path(directory)/'maps-start.json';start_path.write_bytes(maps.canonical(start))
            final=maps.freeze(root,start,start_path);receipt=Path(directory)/'maps.json';receipt.write_bytes(maps.canonical(final))
            other=Path(directory)/'other';other.mkdir()
            copy=self.fixture(other)
            with self.assertRaises(ValueError):maps.verify_frozen(copy,receipt)
            start_path.write_bytes(maps.canonical(dict(start,unreviewed=True)))
            with self.assertRaisesRegex(ValueError,'receipt bytes changed'):maps.verify_frozen(root,receipt)

    def test_campaign_binding_requires_exact_original_root_policy_and_start(self):
        with self.temporary() as directory:
            root=self.fixture(directory);start=maps.snapshot(root)
            start_path=Path(directory)/'maps-start.json';start_path.write_bytes(maps.canonical(start))
            final=maps.freeze(root,start,start_path);receipt=Path(directory)/'maps.json';receipt.write_bytes(maps.canonical(final))
            binding=dict(root=str(root.resolve()),policy=maps.POLICY,start=final['start_receipt'])
            self.assertTrue(maps.verify_campaign(root,receipt,binding)['frozen'])
            for malformed in (None,{},dict(binding,policy='different'),dict(binding,root=None),
                    dict(binding,start=None),dict(binding,start={}),
                    dict(binding,start=dict(final['start_receipt'],sha256='0'*64)),
                    dict(binding,start=dict(final['start_receipt'],path=str(Path(directory)/'another.json')))):
                with self.subTest(binding=malformed),self.assertRaises(ValueError):
                    maps.verify_campaign(root,receipt,malformed)


if __name__=='__main__':unittest.main(verbosity=2)
