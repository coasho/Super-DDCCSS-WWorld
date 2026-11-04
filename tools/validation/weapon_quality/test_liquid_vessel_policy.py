"""Independent typed-vessel receipt controls, without producer imports."""
import copy
import math
import unittest

import liquid_population as lp


def policy():
    return dict(schema='fsim.inferred-liquid-vessels.v1', design='rq4b', basis='theoretical_inference',
                geometry='original_source_wing_root_strip_v1', attachment='sealed_whole_native_tank_wing_module_v1',
                tanks=[dict(index=0, section='left wing', surface='wing'),
                       dict(index=1, section='right wing', surface='wing')])


def fixture():
    tanks = []; cells = []; rows = []
    for i, side in ((0, -1), (1, 1)):
        section = 'left wing' if i == 0 else 'right wing'; anchor = [.5, side, .5]
        tanks.append(dict(index=i, section=section, anchor_body_m=anchor, capacity_kg=800., density_kg_m3=800.))
        for kind, y0, y1 in [('inferred_original_wing_root_v1', .4, .8), ('native_owner_interval_v1', 1., 1.6)]:
            lo, hi = sorted((side*y0, side*y1))
            for x in range(128):
                cells.append(dict(id=len(cells), tank=i, section=section, eligibility_v1=kind,
                                  low_m=[x/128, lo, 0.], high_m=[(x+1)/128, hi, 1.], volume_m3=(y1-y0)/128))
        bounds = sorted((side*.4, side*1.6))
        def summary(fraction):
            z = .5 if fraction == 1 else .75; dz = z-.5
            return dict(volume_m3=fraction, centroid_body_m=[.5, side*1.02, z],
                        offset_from_native_lump_m=[0., side*.02, dz], offset_norm_m=math.hypot(.02, dz),
                        low_m=[0., bounds[0], 0. if fraction == 1 else .5], high_m=[1., bounds[1], 1.],
                        surface_body_z_m=0. if fraction == 1 else .5,
                        reference_up_body=[0., 0., -1.], contents_fraction=fraction)
        rows.append(dict(index=i, section=section, ordinary_m3=.6, root_m3=.4,
                         full=summary(1.), half_reference_up=summary(.5)))
    for i, side in ((2, -1), (3, 1)):
        section = 'left wing' if side < 0 else 'right wing'
        tanks.append(dict(index=i, section=section, anchor_body_m=[2.5, side*1.5, .5], capacity_kg=400., density_kg_m3=800.))
        for x in range(128):
            lo, hi = sorted((side*1., side*1.5))
            cells.append(dict(id=len(cells), tank=i, section=section, eligibility_v1='native_owner_interval_v1',
                              low_m=[2+x/128, lo, 0.], high_m=[2+(x+1)/128, hi, 1.], volume_m3=.5/128))
    expected = dict(design='rq4b', origin=[0., 0., 0.], tanks=tanks, vessel_declaration=policy(), vessel_root_cut=1.)
    receipt = dict(declaration=policy(), attachment=policy()['attachment'], structural_root_cut_design_y_m=1.,
                   centroid_scope='Actual spatial material; native point masses unchanged', tanks=rows,
                   source_wing=dict(method='original_uninflated_wing_primitive_root_clip_v1', surface='wing',
                       skin_inflation_m=0., root_extension_m=0., vertices_sha256='a'*64, triangles_sha256='b'*64,
                       vertices=8, triangles=12, cell_m=.01, error_m=.0025, volume_m3=2.))
    receipt['packing_v1'] = dict(method='nested_eligible_shared_pool_v1', assignment_order=[2, 3, 0, 1],
        definite_eligibility_masks=True,
        target_volume_m3_by_tank=[dict(tank=i, target_m3=(1. if i < 2 else .5)/64) for i in range(4)],
        nested_certified_feasibility=[dict(restricted_tank=r, flexible_tank=f, ordinary_certified_m3=1.1,
                                          total_certified_m3=1.5, restricted_required_m3=.5, total_required_m3=1.5)
                                     for r, f in ((2, 0), (3, 1))])
    return dict(cells=cells, inferred_vessels_v1=receipt,
                ledger=dict(native_owner_interval_certified=False, per_tank_vessel_eligibility_certified=True),
                uncertainty=dict(inferred_vessel_attachment='theoretical sealed module', native_lump_discrepancy='estimated spatial/lump offset')), expected


class VesselPolicyAdmissionTests(unittest.TestCase):
    def test_shared_pool_identity_resolution_and_both_nested_conditions_are_mandatory(self):
        doc, expected = fixture(); lp.validate_vessel_receipt(doc, expected)
        changes = [lambda d: d['inferred_vessels_v1'].pop('packing_v1'),
                   lambda d: d['inferred_vessels_v1']['packing_v1'].update(method='serial_prior_subtraction'),
                   lambda d: d['inferred_vessels_v1']['packing_v1'].update(assignment_order=[0, 1, 2, 3]),
                   lambda d: d['inferred_vessels_v1']['packing_v1'].update(definite_eligibility_masks=False),
                   lambda d: d['inferred_vessels_v1']['packing_v1'].update(assignment_order=[2, 3, 0, True]),
                   lambda d: d['inferred_vessels_v1']['packing_v1'].update(assumed_oem_layout=True),
                   lambda d: d['inferred_vessels_v1']['packing_v1']['target_volume_m3_by_tank'][0].update(target_m3=.1),
                   lambda d: d['inferred_vessels_v1']['packing_v1']['nested_certified_feasibility'][0].update(ordinary_certified_m3=.4),
                   lambda d: d['inferred_vessels_v1']['packing_v1']['nested_certified_feasibility'][0].update(total_certified_m3=1.4),
                   lambda d: d['inferred_vessels_v1']['packing_v1']['nested_certified_feasibility'][0].update(restricted_tank=0),
                   lambda d: d['inferred_vessels_v1']['packing_v1']['nested_certified_feasibility'][0].update(flexible_tank=False),
                   lambda d: d['inferred_vessels_v1']['packing_v1']['nested_certified_feasibility'][0].update(ordinary_certified_m3=.6),
                   lambda d: d['cells'][0].update(volume_m3=.1)]
        for change in changes:
            bad = copy.deepcopy(doc); change(bad)
            with self.subTest(change=change), self.assertRaises(ValueError): lp.validate_vessel_receipt(bad, expected)

    def test_exact_source_policy_is_model_and_native_identity_specific(self):
        self.assertEqual(lp.validate_vessel_declaration(policy(), 'rq4b'), policy())
        for mutate in (lambda d: d.update(design='other'), lambda d: d.update(oem=True),
                       lambda d: d['tanks'][0].update(index=2), lambda d: d['tanks'][0].update(index=True),
                       lambda d: d['tanks'][0].update(section='centre'), lambda d: d.update(geometry='fuselage')):
            bad = copy.deepcopy(policy()); mutate(bad)
            with self.subTest(bad=bad), self.assertRaises(ValueError): lp.validate_vessel_declaration(bad, 'rq4b')

    def test_marker_cannot_replace_required_original_boundary_and_source_roles(self):
        expected = {'inferred_vessel_declaration': dict(path='rq4b.liquid-geometry-v1.json', sha256='a'*64),
                    'inferred_vessel_method': dict(path='liquid_vessels.py', sha256='b'*64),
                    'inferred_vessel_airfoil_coordinates': dict(path='lrn1015.dat', sha256='c'*64),
                    'inferred_vessel_airfoil_method': dict(path='airfoil.py', sha256='d'*64)}
        good = [dict(role=k, **v) for k, v in expected.items()]
        lp.verify_sources(good, expected)
        for bad in (good[:-1], good+[good[0]], [dict(good[0], sha256='b'*64), *good[1:]],
                    [r for r in good if r['role'] != 'inferred_vessel_airfoil_coordinates'],
                    [dict(r, sha256='e'*64) if r['role'] == 'inferred_vessel_airfoil_coordinates' else r for r in good],
                    good+[dict(role='unreviewed_geometry', path='x', sha256='c'*64)]):
            with self.assertRaises(ValueError): lp.verify_sources(bad, expected)

    def test_actual_root_split_and_reference_half_moments_are_independently_reconciled(self):
        doc, expected = fixture(); lp.validate_vessel_receipt(doc, expected)
        changes = [lambda d: d.pop('inferred_vessels_v1'),
                   lambda d: d['ledger'].update(native_owner_interval_certified=True),
                   lambda d: d['ledger'].update(per_tank_vessel_eligibility_certified=False),
                   lambda d: d['cells'][0].update(tank=2),
                   lambda d: d['cells'][0].update(eligibility_v1='any_fuselage'),
                   lambda d: d['cells'][0].update(high_m=[1., .2, 1.]),
                   lambda d: d['inferred_vessels_v1']['tanks'][0].update(root_m3=.8),
                   lambda d: d['inferred_vessels_v1']['tanks'][0]['full'].update(centroid_body_m=[.5, -1., .5]),
                   lambda d: d['inferred_vessels_v1']['tanks'][0]['half_reference_up'].update(volume_m3=.25),
                   lambda d: d['inferred_vessels_v1']['source_wing'].update(skin_inflation_m=.1),
                   lambda d: d['uncertainty'].pop('native_lump_discrepancy')]
        for change in changes:
            bad = copy.deepcopy(doc); change(bad)
            with self.subTest(change=change), self.assertRaises(ValueError): lp.validate_vessel_receipt(bad, expected)
        with self.assertRaises(ValueError): lp.validate_vessel_receipt(doc, dict(expected, vessel_declaration=None))


if __name__ == '__main__': unittest.main(verbosity=2)
