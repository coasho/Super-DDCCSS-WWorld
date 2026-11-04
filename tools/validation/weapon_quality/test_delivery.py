"""Integrity tests use small ordinary fixture repositories; no native execution.

Fixtures are retained under D:/ENV/tmp for inspection. No recursive deletion.
"""
import importlib.util
import base64
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('weapon_delivery', Path(__file__).with_name('delivery.py'))
delivery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(delivery)


class DeliveryTests(unittest.TestCase):
    def fixture(self):
        base = Path('D:/ENV/tmp') if os.name == 'nt' else Path(tempfile.gettempdir())
        base.mkdir(parents=True, exist_ok=True)
        folder = Path(tempfile.mkdtemp(prefix='m03-delivery-test-', dir=base)).resolve()
        repo = folder / 'repo'
        repo.mkdir()
        delivery.git(repo, 'init', '--initial-branch=main')
        for name, content in {'.gitattributes': b'*.txt text eol=lf\n', 'tracked.txt': b'head\n',
                              'unrelated.txt': b'unchanged head\n', 'aircraft/model/model.xml': b'original generated model', 'deleted.bin': b'\0head\xff'}.items():
            (repo / name).parent.mkdir(parents=True,exist_ok=True)
            (repo / name).write_bytes(content)
        delivery.git(repo, 'add', '--all')
        delivery.git(repo, '-c', 'user.name=Delivery Fixture', '-c', 'user.email=fixture@example.invalid',
                     'commit', '-m', 'Create immutable fixture')
        (repo / 'unrelated.txt').write_bytes(b'unrelated staged\r\n')
        delivery.git(repo, 'add', '--', 'unrelated.txt')
        (repo / 'untracked.txt').write_bytes(b'unrelated untracked\n')
        (repo / 'tracked.txt').write_bytes(b'exact CRLF\r\nsecond\r\n')
        (repo / 'deleted.bin').unlink()
        (repo / 'new.bin').write_bytes(b'\0new binary\xff\r\n')
        allowlist = folder / 'reviewed.json'
        allowlist.write_text(json.dumps(['tracked.txt', 'new.bin', 'deleted.bin']), encoding='utf-8')
        return folder, repo, allowlist

    def prepared(self):
        folder, repo, allowlist = self.fixture()
        before = delivery.digest(repo / '.git/index')
        receipt = delivery.prepare(repo, allowlist, folder / 'proof', folder / 'export', temporary_root=folder)
        self.assertTrue(receipt['passed'])
        self.assertEqual(before, delivery.digest(repo / '.git/index'))
        return folder, repo, receipt

    def completed(self, folder):
        def write(path, value):
            path.parent.mkdir(parents=True, exist_ok=True)
            delivery.save(path, value)
            return {'path': str(path), 'sha256': delivery.digest(path)}
        export=folder/'export';sdk=folder/'sdk';campaign_dir=folder/'evidence'
        provenance=[]
        for name in ('bin/libfsim.dll', 'fsim/_native.pyd'):
            source=export/'build'/name;target=sdk/name
            source.parent.mkdir(parents=True,exist_ok=True);target.parent.mkdir(parents=True,exist_ok=True)
            source.write_bytes(name.encode());target.write_bytes(source.read_bytes())
            provenance.append(dict(source=str(source),installed=str(target),sha256=delivery.digest(source)))
        model=sdk/'share/flightsim/aircraft/model/model.xml'
        model.parent.mkdir(parents=True);model.write_bytes(b'generated model')
        producer=write(campaign_dir/'production/installed-production-manifest.json',dict(
            schema='fsim.sim-m02.production-install.v1',prefix=str(sdk),native_dll_sha256=delivery.digest(sdk/'bin/libfsim.dll'),
            product_count=1,products={'model/model.xml':delivery.digest(model)}))
        installed=write(campaign_dir/'installed.json',dict(schema='fsim.sim-m03.installed-consumers.v1',passed=True,
            prefix=str(sdk),producer=producer,provenance=provenance))
        maps=folder/'maps'
        png=base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=')
        names=['s3.amazonaws.com/elevation-tiles-prod/terrarium/0/0/0.png',
               'server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/0/0/0.png']
        files=[]
        for name in names:
            path=maps/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(png)
            files.append(dict(path=name,bytes=len(png),sha256=delivery.digest(path)))
        inventory=delivery.sha(delivery.json_bytes(files))
        map_start=write(campaign_dir/'maps-start.json',dict(schema='fsim.sim-m03.maps.v1',passed=True,
            root=str(maps),files=files,inventory_sha256=inventory,policy='offline-frozen-exact-tree'))
        map_receipt=write(campaign_dir/'maps.json',dict(schema='fsim.sim-m03.maps.v1',passed=True,
            root=str(maps),files=files,inventory_sha256=inventory,policy='offline-frozen-exact-tree',
            frozen=True,start_receipt=map_start,start_inventory_sha256=inventory))
        campaign=write(campaign_dir/'campaign.json',dict(schema='fsim.sim-m03.complete-campaign.v1',
            automated_passed=True,source_root=str(export),prefix=str(sdk),maps=map_receipt,
            map_inputs=dict(root=str(maps),policy='offline-frozen-exact-tree',start=map_start),
            production={'path':str(campaign_dir/'production')}))
        receipt=folder/'proof/prepare.json'
        report=folder/'completed.json'
        source_review=write(folder/'reviews/source.json',dict(passed=True))
        visual_review=write(folder/'reviews/visual.json',dict(passed=True))
        (campaign_dir/'native-raw.log').write_bytes(b'raw native evidence')
        write(report,dict(schema='fsim.sim-m03.complete-acceptance.v1',passed=True,engineering_acceptance=True,
            source_root=str(export),campaign=campaign,maps=map_receipt,installed_consumer=installed,source_review=source_review,visual_review=visual_review,source_manifest={'path':str(receipt),'sha256':delivery.digest(receipt)}))
        return report

    def test_empty_or_substituted_maps_cannot_be_packaged(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        empty=folder/'empty-maps';empty.mkdir()
        substituted=folder/'substituted-maps';substituted.mkdir()
        (substituted/'different.bin').write_bytes(b'unrelated map bytes')
        for index,maps in enumerate((empty,substituted)):
            with self.assertRaises(ValueError):
                delivery.package(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],
                    folder/'sdk',maps,folder/('invalid-maps-'+str(index)),scope_roots=(folder,))

    def test_changed_accepted_map_bytes_cannot_be_packaged(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        tile=folder/'maps/s3.amazonaws.com/elevation-tiles-prod/terrarium/0/0/0.png'
        tile.write_bytes(tile.read_bytes()+b'changed after acceptance')
        with self.assertRaises(ValueError):
            delivery.package(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],
                folder/'sdk',folder/'maps',folder/'changed-maps',scope_roots=(folder,))

    def test_missing_accepted_map_receipt_cannot_be_packaged(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        value=json.loads(report.read_text());value.pop('maps');delivery.save(report,value)
        with self.assertRaises(ValueError):
            delivery.package(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],
                folder/'sdk',folder/'maps',folder/'unbound-maps',scope_roots=(folder,))

    def test_valid_substitute_map_proofs_cannot_replace_pre_execution_campaign_inputs(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        campaign_path=folder/'evidence/campaign.json';campaign=json.loads(campaign_path.read_text(encoding='utf-8'))
        original_inputs=campaign['map_inputs']
        substitute=folder/'substitute';substitute.mkdir()
        for row in json.loads((folder/'evidence/maps.json').read_text(encoding='utf-8'))['files']:
            target=substitute/row['path'];target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((folder/'maps'/row['path']).read_bytes())
        import maps
        start=maps.snapshot(substitute);start_path=folder/'evidence/maps-start.json'
        start_path.write_bytes(maps.canonical(start))
        final_path=folder/'evidence/maps.json';final_path.write_bytes(maps.canonical(maps.freeze(substitute,start,start_path)))
        campaign['maps']['sha256']=delivery.digest(final_path)
        self.assertEqual(campaign['map_inputs'],original_inputs)
        delivery.save(campaign_path,campaign)
        completed=json.loads(report.read_text(encoding='utf-8'))
        completed['campaign']['sha256']=delivery.digest(campaign_path);completed['maps']=campaign['maps'];delivery.save(report,completed)
        with self.assertRaisesRegex(ValueError,'campaign.*map|map.*campaign'):
            delivery.package(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],
                folder/'sdk',substitute,folder/'substituted-proof',scope_roots=(folder,))

    def test_safe_archive_and_allowlist_identities(self):
        for name in ('../escape', '/absolute', 'C:/escape', 'C:escape', 'x\\y', 'a/.CoDeX/private',
                     'a/../b', 'a//b', 'a/.git/config', 'a/NUL.txt', 'a/name.', 'a/name ', 'a:stream', 'a\nname',
                     'a/COM\u00b9.txt','a/COM\u00b2.txt','a/COM\u00b3.txt',
                     'a/LPT\u00b9.txt','a/LPT\u00b2.txt','a/LPT\u00b3.txt'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                delivery.relative_name(name)
        self.assertEqual(delivery.relative_name('proof files/model+data.json'), 'proof files/model+data.json')

    def test_crlf_binary_deletions_and_real_index_are_exact(self):
        folder, repo, receipt = self.prepared()
        export = folder / 'export'
        self.assertEqual((export / 'tracked.txt').read_bytes(), b'exact CRLF\r\nsecond\r\n')
        self.assertEqual((export / 'new.bin').read_bytes(), b'\0new binary\xff\r\n')
        self.assertFalse((export / 'deleted.bin').exists())
        self.assertEqual((export / 'unrelated.txt').read_bytes(), b'unchanged head\n')
        self.assertFalse((export / 'untracked.txt').exists())
        self.assertFalse((export / '.git').exists())
        self.assertTrue(receipt['real_index_unchanged'])
        staged = delivery.git(repo, 'show', ':unrelated.txt')
        self.assertEqual(staged, b'unrelated staged\n')
        self.assertFalse((folder / 'proof/private-index').exists())

    def test_complete_large_reviewed_population_replays_without_command_length_limit(self):
        folder, repo, allowlist = self.fixture()
        names = json.loads(allowlist.read_text(encoding='utf-8'))
        for index in range(384):
            name = 'aircraft/model-'+str(index).zfill(4)+'/aero/static-grid-v2-'+format(index, '064x')+'.json'
            path = repo/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(('actual product '+str(index)+'\r\n').encode())
            names.append(name)
        allowlist.write_text(json.dumps(names), encoding='utf-8')
        before = delivery.digest(repo/'.git/index')
        receipt = delivery.prepare(repo, allowlist, folder/'proof', folder/'export', temporary_root=folder)
        self.assertTrue(receipt['passed'])
        self.assertEqual(set(receipt['allowlist']), set(names))
        self.assertEqual(before, delivery.digest(repo/'.git/index'))
        for name in names:
            with self.subTest(name=name):
                source, exported = repo/name, folder/'export'/name
                if source.exists():
                    self.assertEqual(exported.read_bytes(), source.read_bytes())
                else:
                    self.assertFalse(exported.exists())
        self.assertEqual((folder/'export/unrelated.txt').read_bytes(), b'unchanged head\n')
        self.assertFalse((folder/'export/untracked.txt').exists())
        self.assertEqual(delivery.git(repo, 'show', ':unrelated.txt'), b'unrelated staged\n')

    def test_deterministic_archive_and_bundled_verifier(self):
        folder, repo, _ = self.prepared()
        report = self.completed(folder)
        args = (folder / 'proof/prepare.json', report, [('evidence', folder / 'evidence')], folder / 'sdk', folder / 'maps')
        first = delivery.package(*args, folder / 'package-a', scope_roots=(folder,))
        second = delivery.package(*args, folder / 'package-b', scope_roots=(folder,))
        self.assertEqual(first['archive_sha256'], second['archive_sha256'])
        extracted = folder / 'unpacked'
        extracted.mkdir()
        with tarfile.open(first['archive']) as archive:
            for member in archive:
                delivery.relative_name(member.name)
                self.assertTrue(member.isfile())
                self.assertEqual(member.mtime, 0)
                destination = extracted / member.name
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(archive.extractfile(member).read())
        result = subprocess.run([os.sys.executable, str(extracted / 'VERIFY.py')], capture_output=True, text=True, check=True)
        self.assertTrue(json.loads(result.stdout)['passed'])
        self.assertEqual((extracted/'artifacts/evidence/native-raw.log').read_bytes(),b'raw native evidence')
        manifest=json.loads((extracted/'manifest.json').read_text())
        for row in manifest['provenance_bindings']:
            self.assertEqual(delivery.digest(extracted/row['archive_path']),row['sha256'])
        self.assertEqual((extracted / 'source/tracked.txt').read_bytes(), b'exact CRLF\r\nsecond\r\n')
        self.assertFalse((extracted / 'source/deleted.bin').exists())

    def test_source_tampering_and_failed_report_cannot_be_packaged(self):
        folder, _, _ = self.prepared()
        report = self.completed(folder)
        valid=json.loads(report.read_text())
        args = (folder / 'proof/prepare.json', report, [('evidence',folder/'evidence')], folder / 'sdk', folder / 'maps')
        for field,value in (('passed',False),('schema','fsim.regression.v1'),('engineering_acceptance',False),
                            ('source_root',str(folder/'repo'))):
            invalid=dict(valid);invalid[field]=value;delivery.save(report,invalid)
            with self.subTest(field=field),self.assertRaises(ValueError):
                delivery.package(*args, folder / ('invalid-'+field), scope_roots=(folder,))
        delivery.save(report,valid)
        (folder / 'export/tracked.txt').write_bytes(b'tampered')
        with self.assertRaisesRegex(ValueError,'Prepared source bytes changed'):
            delivery.package(*args, folder / 'invalid-source', scope_roots=(folder,))

    def test_wrong_sdk_or_changed_current_producer_is_rejected(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        wrong=folder/'wrong-sdk';wrong.mkdir()
        with self.assertRaises(ValueError):
            delivery.package(folder/'proof/prepare.json',report,[],wrong,folder/'maps',folder/'invalid-sdk',scope_roots=(folder,))
        (folder/'sdk/bin/libfsim.dll').write_bytes(b'wrong dll')
        with self.assertRaises(ValueError):
            delivery.package(folder/'proof/prepare.json',report,[],folder/'sdk',folder/'maps',folder/'invalid-dll',scope_roots=(folder,))

    def test_produced_overlay_retains_original_source_and_requires_exact_proof(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        path=folder/'export/aircraft/model/model.xml'
        original=path.read_bytes();path.write_bytes(b'generated model')
        overlay=folder/'overlay.json'
        value=dict(schema='fsim.sim-m03.produced-overlay.v1',
            prepare_receipt_sha256=delivery.digest(folder/'proof/prepare.json'),
            campaign_sha256=delivery.digest(folder/'evidence/campaign.json'),
            producer_sha256=delivery.digest(folder/'evidence/production/installed-production-manifest.json'),
            files=[dict(path='aircraft/model/model.xml',prepared_sha256=delivery.sha(original),produced_sha256=delivery.digest(path))])
        delivery.save(overlay,value)
        args=(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],folder/'sdk',folder/'maps')
        with self.assertRaises(ValueError):
            delivery.package(*args,folder/'without-overlay',scope_roots=(folder,))
        completed=json.loads(report.read_text());completed['produced_overlay']={'path':str(overlay),'sha256':delivery.digest(overlay)}
        delivery.save(report,completed)
        result=delivery.package(*args,folder/'with-overlay',scope_roots=(folder,),produced_overlay=overlay)
        with tarfile.open(result['archive']) as archive:
            self.assertEqual(archive.extractfile('source/aircraft/model/model.xml').read(),original)
            self.assertEqual(archive.extractfile('produced-overlay/aircraft/model/model.xml').read(),b'generated model')
        alternative=folder/'alternative-overlay.json';alternative.write_bytes(overlay.read_bytes())
        with self.assertRaises(ValueError):
            delivery.package(*args,folder/'unreviewed-overlay',scope_roots=(folder,),produced_overlay=alternative)
        value['files'][0]['prepared_sha256']='0'*64;delivery.save(overlay,value)
        with self.assertRaises(ValueError):
            delivery.package(*args,folder/'bad-overlay',scope_roots=(folder,),produced_overlay=overlay)

    def test_missing_raw_campaign_tree_is_rejected(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        args=(folder/'proof/prepare.json',report)
        with self.assertRaises(ValueError):
            delivery.package(*args,[],folder/'sdk',folder/'maps',folder/'missing-raw',scope_roots=(folder,))

    def test_changed_installed_receipt_is_rejected(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        args=(folder/'proof/prepare.json',report)
        installed=folder/'evidence/installed.json'
        value=json.loads(installed.read_text());value['unreviewed_change']=True;delivery.save(installed,value)
        with self.assertRaises(ValueError):
            delivery.package(*args,[('evidence',folder/'evidence')],folder/'sdk',folder/'maps',folder/'changed-installed',scope_roots=(folder,))

    def test_native_windows_producer_keys_bind_posix_produced_overlay(self):
        folder,_,_=self.prepared();report=self.completed(folder)
        producer_path=folder/'evidence/production/installed-production-manifest.json'
        producer=json.loads(producer_path.read_text(encoding='utf-8'))
        producer['products']={name.replace('/','\\'):digest for name,digest in producer['products'].items()}
        delivery.save(producer_path,producer)
        installed_path=folder/'evidence/installed.json';installed=json.loads(installed_path.read_text(encoding='utf-8'))
        installed['producer']['sha256']=delivery.digest(producer_path);delivery.save(installed_path,installed)
        path=folder/'export/aircraft/model/model.xml';original=path.read_bytes();path.write_bytes(b'generated model')
        overlay=folder/'overlay-windows.json'
        delivery.save(overlay,dict(schema='fsim.sim-m03.produced-overlay.v1',
            prepare_receipt_sha256=delivery.digest(folder/'proof/prepare.json'),
            campaign_sha256=delivery.digest(folder/'evidence/campaign.json'),producer_sha256=delivery.digest(producer_path),
            files=[dict(path='aircraft/model/model.xml',prepared_sha256=delivery.sha(original),produced_sha256=delivery.digest(path))]))
        completed=json.loads(report.read_text(encoding='utf-8'))
        completed['installed_consumer']['sha256']=delivery.digest(installed_path)
        completed['produced_overlay']=dict(path=str(overlay),sha256=delivery.digest(overlay));delivery.save(report,completed)
        result=delivery.package(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],folder/'sdk',
            folder/'maps',folder/'windows-overlay',scope_roots=(folder,),produced_overlay=overlay)
        self.assertTrue(result['passed'])

    def test_unsafe_or_colliding_native_producer_keys_are_refused_before_path_access(self):
        for products in ({'..\\escape.xml':'a'*64},
                         {'model/model.xml':'a'*64,'model\\model.xml':'a'*64}):
            folder,_,_=self.prepared();report=self.completed(folder)
            producer_path=folder/'evidence/production/installed-production-manifest.json'
            producer=json.loads(producer_path.read_text(encoding='utf-8'))
            producer['products']=products;producer['product_count']=len(products);delivery.save(producer_path,producer)
            installed_path=folder/'evidence/installed.json';installed=json.loads(installed_path.read_text(encoding='utf-8'))
            installed['producer']['sha256']=delivery.digest(producer_path);delivery.save(installed_path,installed)
            completed=json.loads(report.read_text(encoding='utf-8'))
            completed['installed_consumer']['sha256']=delivery.digest(installed_path);delivery.save(report,completed)
            with self.assertRaises(ValueError):
                delivery.package(folder/'proof/prepare.json',report,[('evidence',folder/'evidence')],folder/'sdk',
                    folder/'maps',folder/'unsafe-producer',scope_roots=(folder,))


if __name__ == '__main__':
    unittest.main(verbosity=2)
