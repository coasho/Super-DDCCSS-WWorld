"""Independent admission negatives; no occupancy generation or native execution."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import liquid_population as lp


def expected():
    return dict(design='fixture', origin=[0., 0., 0.], tanks=[dict(index=0, capacity_kg=800.,
                density_kg_m3=800., anchor_body_m=[0., 0., 0.], section='centre')])


def document():
    checks = [dict(tank=0, required_m3=1., actual_m3=1., error_m3=0., tolerance_m3=1024*lp.EPS)]
    flags = dict(capacity_conserved=True, containment_certified=True, exclusions_certified=True, disjoint_certified=True)
    section = dict(section='centre', **flags, tanks=copy.deepcopy(checks), screened_cells=10, assignment_edges=2)
    ledger = dict(**flags, single_native_tank_owner=True, authoritative_boundary_certified=True,
                  native_owner_interval_certified=True, global_aabb_nonoverlap_verified=True,
                  tanks=checks, sections=[section], screened_cells=10, assignment_edges=2)
    return dict(schema='fsim.liquid-occupancy.v1', design='fixture', frame='body_origin_forward_right_down_m',
                fluid_policy_v1=dict(lp.POLICY), method=dict(version=1, origin_design_m=[0., 0., 0.]),
                tanks=expected()['tanks'], cells=[dict(id=i, tank=0, section='centre', low_m=[i*.5, 0., 0.],
                    high_m=[(i+1)*.5, 1., 1.], volume_m3=.5) for i in range(2)], ledger=ledger, exclusions=[],
                uncertainty=dict(distribution='estimated internal distribution', boundary='delivered original GLB',
                    density='estimated physical reference density', dynamics='full contents only', maximum_cell_diagonal_m=1.5),
                limits=dict(max_cells=65536, max_screened_cells=1048576, max_assignment_edges=2097152, max_seconds=300.))


class LiquidAdmissionTests(unittest.TestCase):
    def validate(self, doc=None, exp=None): return lp.validate_document(doc or document(), exp or expected())

    def test_independent_spatial_capacity_and_touching_boundaries_pass(self):
        result = self.validate(); self.assertEqual(result['cells'], 2); self.assertEqual(result['tanks'], 1)
        self.assertIsNone(result['measured_liquid_wall_seconds'])

    def test_true_flags_cannot_hide_overlapping_material_at_conserved_mass(self):
        doc = document(); doc['cells'][1]['low_m'][0] = .4; doc['cells'][1]['high_m'][0] = .9
        with self.assertRaisesRegex(ValueError, 'overlap'): self.validate(doc)

    def test_source_native_identity_density_anchor_and_owner_are_required(self):
        mutations = [lambda d: d.update(design='another'), lambda d: d.update(frame='gltf'),
                     lambda d: d['tanks'][0].update(index=1), lambda d: d['tanks'][0].update(density_kg_m3=1600.),
                     lambda d: d['tanks'][0].update(capacity_kg=801.), lambda d: d['tanks'][0].update(section='left wing'),
                     lambda d: d['tanks'][0].update(anchor_body_m=[1, 0, 0]),
                     lambda d: d['method'].update(origin_design_m=[1, 0, 0])]
        for mutation in mutations:
            doc = document(); mutation(doc)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): self.validate(doc)

    def test_missing_duplicate_reordered_unknown_and_cross_owner_cells_fail(self):
        mutations = [lambda d: d.update(cells=[]), lambda d: d['cells'][1].update(id=0),
                     lambda d: d['cells'][0].update(tank=9), lambda d: d['cells'][0].update(section='tail'),
                     lambda d: d['tanks'].append(copy.deepcopy(d['tanks'][0]))]
        for mutation in mutations:
            doc = document(); mutation(doc)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): self.validate(doc)

    def test_cell_volume_and_fixed_roundoff_ledger_cannot_be_forged(self):
        for path, value in [(('cells', 'volume_m3'), .6), (('ledger', 'tolerance_m3'), 1e30),
                            (('ledger', 'actual_m3'), .9), (('ledger', 'error_m3'), .1)]:
            doc = document()
            (doc['cells'][0] if path[0] == 'cells' else doc['ledger']['tanks'][0])[path[1]] = value
            with self.subTest(path=path), self.assertRaises(ValueError): self.validate(doc)

    def test_false_global_and_owner_geometry_receipts_fail(self):
        for field in ['capacity_conserved', 'containment_certified', 'exclusions_certified', 'disjoint_certified',
                      'single_native_tank_owner', 'authoritative_boundary_certified', 'native_owner_interval_certified',
                      'global_aabb_nonoverlap_verified']:
            doc = document(); doc['ledger'][field] = False
            with self.subTest(field=field), self.assertRaises(ValueError): self.validate(doc)
        doc = document(); doc['ledger']['sections'][0]['containment_certified'] = False
        with self.assertRaises(ValueError): self.validate(doc)
        doc = document(); doc['ledger']['sections'] = []
        with self.assertRaises(ValueError): self.validate(doc)

    def test_declared_equipment_exclusion_is_numerically_independent(self):
        doc = document(); doc['exclusions'] = [dict(low_m=[.1, .1, .1], high_m=[.2, .2, .2], basis='estimated crew')]
        with self.assertRaisesRegex(ValueError, 'exclusion'): self.validate(doc)

    def test_policy_uncertainty_and_resource_failures_are_not_accuracy_receipts(self):
        for mutation in [lambda d: d['fluid_policy_v1'].update(drag_coefficient=.5),
                         lambda d: d['fluid_policy_v1'].update(basis='published'),
                         lambda d: d['uncertainty'].update(distribution='OEM calibrated'),
                         lambda d: d['uncertainty'].update(maximum_cell_diagonal_m=0.),
                         lambda d: d['ledger'].update(screened_cells=0),
                         lambda d: d['limits'].update(max_cells=1)]:
            doc = document(); mutation(doc)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): self.validate(doc)

    def test_nonfinite_and_boolean_numbers_are_rejected(self):
        for value in (float('nan'), float('inf'), True):
            doc = document(); doc['cells'][0]['low_m'][0] = value
            with self.subTest(value=value), self.assertRaises(ValueError): self.validate(doc)

    def test_no_liquid_requires_empty_population_and_explicit_reason(self):
        doc = document(); exp = expected(); exp['tanks'] = []
        doc.update(tanks=[], cells=[], no_liquid_reason='Electric propulsion has no liquid capacity')
        doc['ledger']['tanks'] = []; doc['uncertainty']['distribution'] = 'liquid-inapplicable'
        self.assertFalse(self.validate(doc, exp)['liquid'])
        doc['cells'] = [document()['cells'][0]]
        with self.assertRaises(ValueError): self.validate(doc, exp)

    def test_native_rotor_pounds_capacity_and_aggregate_identity(self):
        spec = dict(aircraft=dict(kind='helicopter'), mass=dict(cg_in=[140., 0., 55.]), fuel=dict(capacity_lb=1400.))
        xml = ET.fromstring('<fdm_config><mass_balance><location name="CG" unit="IN"><x>140</x><y>0</y><z>55</z></location>'
                            '</mass_balance><propulsion><tank type="FUEL"><location unit="IN"><x>140</x><y>0</y><z>55</z></location>'
                            '<capacity unit="LBS">1400</capacity></tank></propulsion></fdm_config>')
        actual = lp.expected_model('rotor', spec, xml, None)
        self.assertEqual(len(actual['tanks']), 1); self.assertEqual(actual['tanks'][0]['index'], 0)
        self.assertAlmostEqual(actual['tanks'][0]['capacity_kg'], 635.029318, places=9)
        self.assertEqual(actual['tanks'][0]['anchor_body_m'], [0., 0., 0.])
        xml.find('propulsion/tank/capacity').set('unit', 'GAL')
        with self.assertRaises(ValueError): lp.expected_model('rotor', spec, xml, None)

    def test_spatial_sweep_checks_interior_overlap_with_many_disjoint_boxes(self):
        boxes = [([float(i), 0., 0.], [float(i+1), 1., 1.]) for i in range(128)]
        lp.nonoverlap(boxes)
        boxes.append(([127.5, .1, .1], [128.5, .9, .9]))
        with self.assertRaises(ValueError): lp.nonoverlap(boxes)

    def test_json_duplicates_and_nonfinite_tokens_fail_before_admission(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp' if os.name == 'nt' else None) as folder:
            path = Path(folder)/'product.json'
            for text in ('{"design":"a","design":"b"}', '{"value":NaN}', '{"value":Infinity}', '{"value":1e999}'):
                path.write_text(text)
                with self.assertRaises(ValueError): lp.read(path)

    def test_actual_native_nonfuel_geometry_is_independently_hash_bound(self):
        component = dict(kind='control', shape='box', name='actual actuator', centre=[2., 2., 2.], half=[.1, .1, .1])
        native = dict(origin_design_m=[0., 0., 0.], components=[component])
        doc = document(); doc['exclusions'] = [dict(native_component=0, low_m=[1.9, 1.9, 1.9],
                                                  high_m=[2.1, 2.1, 2.1], basis='actual native control')]
        payload = lp.validate_native_exclusions(doc, native, [0., 0., 0.])
        self.assertEqual(payload, dict(origin_design_m=[0., 0., 0.], native_damage_present=True, components=[[0, component]]))

    def test_actual_native_control_cannot_hide_behind_clearance_receipt_or_frame(self):
        component = dict(kind='control', shape='box', name='actual actuator', centre=[.25, .5, .5], half=[.1, .1, .1])
        native = dict(origin_design_m=[0., 0., 0.], components=[component])
        doc = document(); doc['exclusions'] = [dict(native_component=0, low_m=[.15, .4, .4],
                                                  high_m=[.35, .6, .6], basis='actual native control')]
        with self.assertRaises(ValueError): lp.validate_native_exclusions(doc, native, [0., 0., 0.])
        native['origin_design_m'] = [1., 0., 0.]
        with self.assertRaises(ValueError): lp.validate_native_exclusions(doc, native, [0., 0., 0.])

    def test_population_is_independently_enumerated_and_rejects_coverage_omissions(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp' if os.name == 'nt' else None) as folder:
            root = Path(folder); (root/'aircraft').mkdir(); fits = []
            for i in range(35):
                name = 'fixture%02d' % i; category = 'fixed-wing' if i < 31 else 'rotorcraft'
                design = root/'aircraft'/name; design.mkdir()
                design.joinpath(name+'.toml').write_text('[aircraft]\nname="'+name+'"\n'+('kind="multirotor"\n' if i >= 31 else ''))
                fits.append(dict(design=name, category=category))
            self.assertEqual(len(lp.population(dict(fits=fits), root)), 35)
            for altered in (fits[:-1], fits+[fits[0]], fits[:-1]+[fits[0]]):
                with self.assertRaises(ValueError): lp.population(dict(fits=altered), root)

    def test_source_receipt_binds_exact_glb_partition_and_producer_epochs(self):
        expected = {'producer': dict(path='liquid.py', sha256='a'*64),
                    'glb': dict(path='fixture.glb', sha256='b'*64),
                    'native_partition_method': dict(path='breakup.py', sha256='c'*64),
                    'effective_design_spec': dict(path='canonical-design-spec', sha256='d'*64)}
        rows = [dict(role=role, **binding) for role, binding in expected.items()]
        self.assertEqual(set(lp.verify_sources(rows, expected)), set(expected))

    def test_stale_mixed_duplicate_missing_or_wrong_path_source_receipts_fail(self):
        expected = {'producer': dict(path='liquid.py', sha256='a'*64),
                    'glb': dict(path='fixture.glb', sha256='b'*64),
                    'native_partition_method': dict(path='breakup.py', sha256='c'*64)}
        rows = [dict(role=role, **binding) for role, binding in expected.items()]
        for role in expected:
            changed = copy.deepcopy(rows); next(r for r in changed if r['role'] == role)['sha256'] = 'f'*64
            with self.subTest(role=role), self.assertRaises(ValueError): lp.verify_sources(changed, expected)
        for changed in [rows[:-1], rows+[rows[0]], rows+[dict(role='extra', path='x', sha256='d'*64)],
                        [dict(rows[0], path='different.py'), *rows[1:]],
                        [dict(rows[0], sha256='not-a-hash'), *rows[1:]]]:
            with self.assertRaises(ValueError): lp.verify_sources(changed, expected)


if __name__ == '__main__': unittest.main()
