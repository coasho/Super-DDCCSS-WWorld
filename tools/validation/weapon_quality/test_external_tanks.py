"""Synthetic negative admission controls; not native execution/equipment proof."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import external_tanks as et

TMP_PARENT='D:/ENV/tmp' if os.name=='nt' else None

class ExternalTankAdmission(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=TMP_PARENT);self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.card=self.root/'assets/stores/fixture.json';self.card.parent.mkdir(parents=True);self.card.write_text('{}',encoding='utf-8')
        self.fit=self.root/'aircraft/carrier/payloads.json';self.fit.parent.mkdir(parents=True);self.fit.write_text('{}',encoding='utf-8')
        self.carrier_xml=self.fit.with_name('carrier.xml');self.carrier_xml.write_text('<fdm_config/>',encoding='utf-8')
        self.xml=self.root/'aircraft/stores/fixturetank/fixturetank.xml';self.xml.parent.mkdir(parents=True)
        self.xml.write_text('<fdm_config><mass_balance><emptywt unit="LBS">'+str(10/.45359237)+'</emptywt>'
            '<location name="CG" unit="M"><x>2</x><y>0</y><z>0</z></location>'
            '<pointmass name="fuel"><weight unit="LBS">0</weight><location unit="M"><x>3</x><y>0</y><z>0</z></location>'
            '</pointmass></mass_balance></fdm_config>',encoding='utf-8')
        self.expected=dict(card='fixture',model='jsbsim:fixturetank',model_name='fixturetank',fuel_kg=100.,dry_kg=10.,
            card_path=str(self.card),mounts=[dict(carrier='carrier',station=1,fit=str(self.fit))])
        self.row=dict(kind='tank',card='fixture',model='jsbsim:fixturetank',card_source=et.bind(self.card),fit_source=et.bind(self.fit),
                      capacity_kg=100.,dry_card_mass_kg=10.,wet=self.run_row(False),empty=self.run_row(True))
        wet,dry=self.row['wet'],self.row['empty'];self.row.update(wet_empty_mass_error_kg=0.,
            wet_empty_mass_budget_kg=1e-7*wet['released_mass_kg'],wet_empty_moment_error_kg_m=0.,
            wet_empty_moment_budget_kg_m=1e-7*max(wet['released_mass_kg']*et.norm(wet['released_cg_structural_m']),
                                               dry['released_mass_kg']*et.norm(dry['released_cg_structural_m'])))

    def run_row(self,empty):
        before_fuel=200. if empty else 111.;after_fuel=200. if empty else 112.
        remaining=0. if empty else 89.;fuel=0. if empty else 88.;store=10+remaining
        mb=700+before_fuel+store;ma=700+after_fuel
        cb=[(1400+before_fuel+4*store)/mb,0,0];ca=[(1400+after_fuel)/ma,0,0]
        mass=10+fuel;cg=[(20+3*fuel)/mass,0,0]
        return dict(empty_control=empty,frame_skip=1,carrier='carrier',station=1,fit_basis='authored_loadout',
            initial_altitude_m=120,initial_airspeed_ms=180,initial_pitch_deg=-60,release_down_ms=155.,
            carrier_xml=et.bind(self.carrier_xml),
            external_before_transfer_kg=100.,external_after_transfer_kg=0. if empty else 90.,
            internal_before_transfer_kg=100.,internal_after_transfer_kg=200. if empty else 110.,internal_capacity_kg=1000.,
            transfer_duration_s=11. if empty else 1.,native_dt_s=.01,transfer_calls=1100 if empty else 100,external_before_release_kg=remaining,
            before_release=dict(mass_kg=mb,cg_structural_m=cb,internal_fuel_kg=before_fuel,internal_fuel_moment_kg_m=[before_fuel,0,0]),
            after_release=dict(mass_kg=ma,cg_structural_m=ca,internal_fuel_kg=after_fuel,internal_fuel_moment_kg_m=[after_fuel,0,0]),
            store_before_release_kg=store,station_structural_m=[4.,0,0],release_mass_error_kg=0.,
            release_mass_budget_kg=1e-6*max(mb,ma),release_moment_error_kg_m=0.,
            release_moment_budget_kg_m=2e-6*max(mb*et.norm(cb),ma*et.norm(ca)),
            released_fuel_kg=fuel,released_mass_kg=mass,released_cg_structural_m=cg,fuel_point_structural_m=[3.,0,0],
            impact_count=1,fire_count=0 if empty else 1,fire_kg=0. if empty else 17.6,continuing_kg_s=0.,detonation_count=0,
            impact_energy_j=10000.,impact_time_s=5.,loaded_xml=et.bind(self.xml))

    def check(self,row=None,expected=None):return et.validate_tank(row or self.row,expected or self.expected,self.root)

    def test_independent_native_mass_moment_and_fuel_control_example_is_admitted(self):
        self.assertTrue(self.check()['passed'])

    def test_actual_transfer_must_reach_native_internal_fuel(self):
        self.row['wet']['internal_after_transfer_kg']-=.1
        with self.assertRaisesRegex(ValueError,'Transferred mass conservation'):self.check()

    def test_transfer_uses_fixed_law_not_reported_success(self):
        self.row['wet']['external_after_transfer_kg']=91.
        with self.assertRaisesRegex(ValueError,'transfer law'):self.check()

    def test_release_first_moment_is_recomputed_from_native_snapshots(self):
        self.row['wet']['after_release']['cg_structural_m'][1]=.1
        with self.assertRaises(ValueError):self.check()

    def test_reported_larger_tolerance_cannot_admit_mass_loss(self):
        self.row['wet']['release_mass_budget_kg']=100.
        with self.assertRaisesRegex(ValueError,'Fixed release mass budget'):self.check()

    def test_released_fuel_cannot_exceed_actual_host_remaining(self):
        self.row['wet']['released_fuel_kg']=100.
        with self.assertRaisesRegex(ValueError,'Release created fuel'):self.check()

    def test_empty_native_control_cannot_burn_or_detonate(self):
        for key,value in (('fire_count',1),('fire_kg',1.),('detonation_count',1)):
            with self.subTest(key=key):
                row=copy.deepcopy(self.row);row['empty'][key]=value
                with self.assertRaises(ValueError):self.check(row)

    def test_wet_fire_cannot_exceed_remaining_fuel(self):
        self.row['wet']['fire_kg']=1000.
        with self.assertRaisesRegex(ValueError,'fire exceeds'):self.check()

    def test_impact_and_native_finite_energy_are_required(self):
        for key,value in (('impact_count',0),('impact_energy_j',0.),('impact_time_s',22.),('continuing_kg_s',1.)):
            with self.subTest(key=key):
                row=copy.deepcopy(self.row);row['wet'][key]=value
                with self.assertRaises(ValueError):self.check(row)

    def test_nonfinite_booleans_and_nonintegral_native_quantities_are_refused(self):
        for key,value in (('released_fuel_kg',float('nan')),('impact_count',True),('station',True),('frame_skip',1.5)):
            with self.subTest(key=key):
                row=copy.deepcopy(self.row);row['wet'][key]=value
                with self.assertRaises(ValueError):self.check(row)

    def test_destructive_contact_premise_uses_preterminal_native_velocity(self):
        self.row['empty']['release_down_ms']=99.
        with self.assertRaisesRegex(ValueError,'hard-impact premise'):self.check()

    def test_actual_loaded_xml_and_source_bytes_are_bound(self):
        self.xml.write_text(self.xml.read_text(encoding='utf-8').replace('<x>3</x>','<x>4</x>'),encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'bytes changed'):self.check()

    def test_current_generated_xml_composite_cg_is_independent(self):
        self.row['wet']['released_cg_structural_m'][0]+=.01
        with self.assertRaisesRegex(ValueError,'composite CG'):self.check()

    def test_an_unauthored_mount_is_never_relabelled_an_oem_fit(self):
        expected=copy.deepcopy(self.expected);expected['mounts']=[]
        with self.assertRaisesRegex(ValueError,'Diagnostic mount'):self.check(expected=expected)

    def test_exact_population_refuses_missing_duplicate_and_foreign_rows(self):
        for rows in ([{'card':'a'}],[{'card':'a'},{'card':'a'}],[{'card':'a'},{'card':'c'}]):
            with self.assertRaises(ValueError):et.reconcile_population({'a','b'},rows,('card',))
        with self.assertRaises(ValueError):et.reconcile_population(et.PRESETS,[{'preset':'ucrt64-release'}],('preset',))

    def test_partial_native_population_never_has_a_completion_receipt(self):
        path=self.root/'partial.ndjson';path.write_text(json.dumps(dict(kind='runtime',schema='fsim.external-tank-functions.v1',seed='17'))+'\n',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'partial'):et.raw(path)

    def test_nonzero_exact_native_case_is_required(self):
        path=self.root/'report.xml';path.write_text('<Catch2TestRun><TestCase name="'+et.CASE+'"><OverallResult success="true"/></TestCase>'
            '<OverallResults successes="0" failures="0"/></Catch2TestRun>',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'Nonzero'):et.actual_test_report(path)

    def test_completed_population_cannot_substitute_for_a_passing_native_report(self):
        path=self.root/'failed.xml';path.write_text('<Catch2TestRun><TestCase name="'+et.CASE+'"><OverallResult success="false"/></TestCase>'
            '<OverallResults successes="100" failures="1"/></Catch2TestRun>',encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'test failed'):et.actual_test_report(path)

    def test_missing_or_stale_mode_flights_header_invalidates_source_epoch(self):
        name='tests/simulation/mode_flights.h';self.assertIn(name,et.SOURCE_PATHS)
        path=self.root/name;path.parent.mkdir(parents=True);path.write_text('original fixture options',encoding='utf-8')
        epoch=dict(schema='fsim.external-tanks.source-epoch.v1',main_head='head',coverage_sha256='coverage',sources=[],presets=[])
        with mock.patch.object(et,'git_epoch',return_value='head'),mock.patch.object(et,'source_paths',return_value={name}):
            with self.assertRaisesRegex(ValueError,'Complete unique exact'):et.admit({},'coverage',epoch,self.root)
            epoch['sources']=[dict(path=name,sha256=et.sha(path))];path.write_text('changed fixture interval',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'bytes changed'):et.admit({},'coverage',epoch,self.root)

    def test_campaign_join_sources_are_bound_before_current_native_execution(self):
        for name in ('campaign.py','report.py','README.md'):
            self.assertIn('tools/validation/weapon_quality/'+name,et.SOURCE_PATHS)

    def test_each_of_19_real_tanks_gets_exact_receipt_without_changing_old_gates(self):
        import contract
        import report
        cards=[json.loads(p.read_text(encoding='utf-8')) for p in (et.ROOT/'assets/stores').glob('*.json')]
        tanks={c['id'] for c in cards if c.get('class')=='tank'}
        self.assertEqual(len(tanks),19)
        path=self.root/'external-tanks.json';path.write_text('{"passed":true}',encoding='utf-8')
        expected=[report.binding(path)]
        self.assertEqual(contract.INERT_GATES,('applicability',))
        for card in tanks:
            self.assertEqual([e for g in contract.INERT_GATES for e in report.external_tank_gate_evidence(card,g,tanks,path)],expected)
            for gate in ('native','methods','installed','public','visual'):
                self.assertEqual(report.external_tank_gate_evidence(card,gate,tanks,path),[])
        for c in cards:
            if c['id'] not in tanks:
                for gate in set(contract.AAM_GATES+contract.EFFECT_GATES+contract.INERT_GATES):
                    self.assertEqual(report.external_tank_gate_evidence(c['id'],gate,tanks,path),[])

    def test_report_independent_join_refuses_missing_stale_and_wrong_receipts(self):
        import report
        (self.root/'coverage.json').write_text('{}',encoding='utf-8')
        epoch=self.root/'external-tanks-source-epoch.json';epoch.write_text('{}',encoding='utf-8')
        path=self.root/'external-tanks.json';fresh=dict(schema='fixture-join',passed=True,models=[{'card':'tank'}])
        good=dict(**fresh,source_epoch=et.bind(epoch))
        def write(value):path.write_text(json.dumps(value),encoding='utf-8')
        with mock.patch.object(report,'admit_external_tanks',return_value=fresh) as admission:
            write(good);self.assertEqual(report.join_external_tanks({},self.root,self.root,self.root),path)
            self.assertEqual(admission.call_count,1)
            path.unlink()
            with self.assertRaises(FileNotFoundError):report.join_external_tanks({},self.root,self.root,self.root)
            write({**good,'models':[]})
            with self.assertRaisesRegex(ValueError,'joined receipt changed'):report.join_external_tanks({},self.root,self.root,self.root)
            foreign=self.root/'foreign-epoch.json';foreign.write_text('{}',encoding='utf-8')
            write({**good,'source_epoch':et.bind(foreign)})
            with self.assertRaisesRegex(ValueError,'another campaign'):report.join_external_tanks({},self.root,self.root,self.root)
            write(good);epoch.write_text('{"changed":true}',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'changed'):report.join_external_tanks({},self.root,self.root,self.root)

if __name__=='__main__':unittest.main()
