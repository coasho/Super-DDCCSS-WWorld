"""Independent, bounded Manifold fixtures for capacity-conserving liquid cells."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import time
import tomllib
from types import SimpleNamespace
import unittest

import manifold3d as mf
import numpy as np

from hangar import liquid


def tank(index, volume, anchor):
    return dict(index=index, capacity_kg=800.0 * volume,
                density_kg_m3=800.0, anchor_body_m=list(anchor))


def cube(size, low=(0, 0, 0)):
    return mf.Manifold.cube(size).translate(low)


def exported_design_vertices(points):
    from hangar import model3d
    return model3d._to_gltf(points, np.zeros(3))


def partition_aircraft(tail=False):
    def surface(kind, x, chord, span=5.):
        return SimpleNamespace(kind=kind, mirror=True, sections=[
            SimpleNamespace(le=np.array([x, 0., 0.]), chord=chord),
            SimpleNamespace(le=np.array([x, span, 0.]), chord=chord)])
    surfaces = [surface('wing', 0., 1.), surface('canard', 3., .4)]
    if tail: surfaces.append(surface('htail', 8., 1., 2.))
    return SimpleNamespace(surfaces=surfaces, bodies=[])


def glb(path, meshes, nodes=None, vertex_transform=None, authoritative=None):
    """A GLB writer independent of the production importer; retain split vertices."""
    binary = bytearray(); accessors = []; views = []; records = []
    for solid in meshes:
        mesh = solid.to_mesh64(); vertices = np.asarray(mesh.vert_properties)[:, :3]
        triangles = np.asarray(mesh.tri_verts)
        if vertex_transform is not None: vertices = vertex_transform(vertices)
        # Deliberate per-face material/normal splits: exact-coordinate welding is required.
        vertices = vertices[triangles.reshape(-1)].astype('<f4')
        triangles = np.arange(len(vertices), dtype='<u4').reshape(-1, 3)
        indices = []
        for data, typ, component in [(vertices, 'VEC3', 5126), (triangles.reshape(-1), 'SCALAR', 5125)]:
            views.append(dict(buffer=0, byteOffset=len(binary), byteLength=data.nbytes))
            binary.extend(data.tobytes())
            accessors.append(dict(bufferView=len(views)-1, componentType=component,
                                  count=len(data), type=typ))
            indices.append(len(accessors)-1)
        records.append(dict(primitives=[dict(attributes={'POSITION': indices[0]}, indices=indices[1])]))
    if nodes is None:
        nodes = [dict(name='airframe', mesh=0)]
    doc = dict(asset={'version': '2.0'}, scene=0, scenes=[{'nodes': [0]}], nodes=nodes,
               meshes=records, accessors=accessors, bufferViews=views,
               buffers=[{'byteLength': len(binary)}])
    if authoritative is not None:
        mesh = authoritative.to_mesh64(); vertices = np.asarray(mesh.vert_properties)[:, :3]
        if vertex_transform is not None: vertices = vertex_transform(vertices)
        indices = []
        for data, typ, component in [(vertices.astype('<f4'), 'VEC3', 5126),
                                      (np.asarray(mesh.tri_verts).astype('<u4').reshape(-1), 'SCALAR', 5125)]:
            views.append(dict(buffer=0, byteOffset=len(binary), byteLength=data.nbytes))
            binary.extend(data.tobytes()); accessors.append(dict(bufferView=len(views)-1, componentType=component,
                                                                count=len(data), type=typ)); indices.append(len(accessors)-1)
        doc['buffers'][0]['byteLength'] = len(binary)
        doc['extras'] = dict(fsim_liquid_boundary_v1=dict(schema='fsim.liquid-boundary.v1',
                             frame='gltf_model_origin_m', positions=indices[0], indices=indices[1], node=0))
    js = json.dumps(doc).encode(); js += b' ' * (-len(js) % 4)
    binary += b'\0' * (-len(binary) % 4)
    total = 12 + 8 + len(js) + 8 + len(binary)
    path.write_bytes(struct.pack('<III', 0x46546c67, 2, total) +
                     struct.pack('<II', len(js), 0x4e4f534a) + js +
                     struct.pack('<II', len(binary), 0x004e4942) + binary)


class LiquidTests(unittest.TestCase):
    def setUp(self):
        self.t0 = time.perf_counter()
        self.tmp = tempfile.TemporaryDirectory(dir='D:/ENV/tmp' if os.name == 'nt' else None)
        self.directory = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def check_cells(self, result, hull, excluded=None):
        self.assertGreater(len(result['cells']), 0)
        by_tank = {t['index']: [] for t in result['tanks']}
        boxes = []
        for i, cell in enumerate(result['cells']):
            self.assertEqual(cell['id'], i)
            low, high = np.array(cell['low_m']), np.array(cell['high_m'])
            volume = math.prod(high-low)
            self.assertGreater(volume, 0)
            self.assertAlmostEqual(volume, cell['volume_m3'], places=12)
            # Read the stored endpoints directly, independently of lo+size
            # arithmetic in a transformed cube (which can round past hi).
            reference = mf.Manifold.cube().to_mesh64()
            vertices = np.where(np.asarray(reference.vert_properties)[:, :3] == 0, low, high)
            solid = mf.Manifold(mf.Mesh64(np.array(vertices, dtype=np.float64, order='C', copy=True),
                                         np.array(reference.tri_verts, dtype=np.uint64, order='C', copy=True)))
            self.assertEqual((solid-hull).status(), mf.Error.NoError)
            self.assertTrue((solid-hull).is_empty())
            if excluded is not None:
                self.assertTrue((solid ^ excluded).is_empty())
            for other_low, other_high in boxes:
                # Exact AABB interior intersection; common boundary is legal.
                self.assertTrue(np.any(np.minimum(high, other_high) <= np.maximum(low, other_low)))
            boxes.append((low, high)); by_tank[cell['tank']].append(volume)
        for t in result['tanks']:
            self.assertAlmostEqual(math.fsum(by_tank[t['index']]),
                                   t['capacity_kg']/t['density_kg_m3'], places=11)

    def test_full_capacity_disjoint_and_partial_slabs(self):
        hull = cube((4, 2, 2))
        tanks = [tank(0, 1.31, (.2, 1, 1)), tank(1, 2.17, (3.8, 1, 1))]
        result = liquid.pack('analytic-box', hull, tanks)
        self.check_cells(result, hull)
        self.assertEqual(result['schema'], 'fsim.liquid-occupancy.v1')
        self.assertEqual(result['frame'], 'body_origin_forward_right_down_m')
        self.assertTrue(result['ledger']['capacity_conserved'])
        self.assertTrue(result['ledger']['containment_certified'])
        self.assertEqual(result, liquid.pack('analytic-box', hull, tanks))
        means = [np.mean([(np.array(c['low_m'])+c['high_m'])/2 for c in result['cells']
                          if c['tank'] == i], axis=0)[0] for i in (0, 1)]
        self.assertLess(means[0], means[1])

    def test_excluded_engine_and_crew_cavities_remain_empty(self):
        hull = cube((3, 3, 3))
        excluded = cube((1, 1, 3), (1, 1, 0)) + cube((.8, 3, .6), (0, 0, 2.4))
        result = liquid.pack('cavities', hull, [tank(0, 7.25, (1, 1, 1))], excluded=excluded)
        self.check_cells(result, hull, excluded)

    def test_actual_native_control_boxes_cannot_share_physical_liquid(self):
        path = self.directory/'native-controls.toml'; path.write_text('[aircraft]\nname="native-controls"\n')
        model = self.directory/'native-controls.glb'
        glb(model, [cube((1, 1, 1))], vertex_transform=exported_design_vertices, authoritative=cube((1, 1, 1)))
        component = dict(kind='control', shape='box', section='airframe', name='actual native actuator',
                         centre=[-.5, .5, -.5], half=[.25, .25, .25], channel='aileron')
        native = dict(aircraft='native-controls', origin_design_m=[0., 0., 0.], sections=[], components=[component])
        model.with_suffix('.damage.json').write_text(json.dumps(native))
        a = SimpleNamespace(name='native-controls', path=str(path), bodies=[], engines=[], surfaces=[],
                            spec={'mass': {'tank': [dict(capacity=400., position=[.5, .5, .5])]}})
        result = liquid.produce(a, [0, 0, 0], glb=model)
        lo = np.array(component['centre'])-component['half']; hi = np.array(component['centre'])+component['half']
        for cell in result['cells']:
            self.assertTrue(np.any(np.minimum(cell['high_m'], hi) <= np.maximum(cell['low_m'], lo)))
        payload = dict(origin_design_m=[0., 0., 0.], native_damage_present=True, components=[[0, component]])
        digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        self.assertEqual(next(s['sha256'] for s in result['sources'] if s['role'] == 'native_material_exclusion'), digest)

    def test_native_body_frame_exclusion_is_not_shifted_and_partial_crop_is_real(self):
        aircraft = SimpleNamespace(spec={}, bodies=[], engines=[])
        component = dict(kind='control', shape='box', section='centre', name='edge actuator',
                         centre=[-1., 0., 0.], half=[.5, .2, .2])
        native = dict(origin_design_m=[8., 1., 2.], components=[component]); before = copy.deepcopy(native)
        excluded, rows = liquid._exclusions(aircraft, np.array([8., 1., 2.]), native_damage=native)
        row = next(r for r in rows if r.get('native_component') == 0)
        self.assertEqual(row['low_m'], [-1.5, -.2, -.2]); self.assertEqual(row['high_m'], [-.5, .2, .2])
        hull = cube((2, 2, 2), (-1, -1, -1))
        self.assertAlmostEqual((excluded ^ hull).volume(), .08, places=12)
        result = liquid.pack('partial-native-box', hull, [tank(0, .25, (-.8, 0, 0))], excluded=excluded)
        self.check_cells(result, hull, excluded); self.assertEqual(native, before)

    def test_native_exclusion_refuses_wrong_origin_and_unknown_or_malformed_solids(self):
        aircraft = SimpleNamespace(spec={}, bodies=[], engines=[])
        component = dict(kind='control', shape='box', section='centre', name='actuator',
                         centre=[0., 0., 0.], half=[.1, .1, .1])
        for mutation in [dict(kind='unimplemented'), dict(shape='unknown'), dict(half=[0., .1, .1]),
                         dict(half=[-.1, .1, .1]), dict(centre=[float('nan'), 0, 0]), dict(centre=[0, 0])]:
            native = dict(origin_design_m=[0., 0., 0.], components=[dict(component, **mutation)])
            with self.subTest(mutation=mutation), self.assertRaises(liquid.LiquidError):
                liquid._exclusions(aircraft, np.zeros(3), native_damage=native)
        with self.assertRaises(liquid.LiquidError):
            liquid._exclusions(aircraft, np.zeros(3), native_damage=dict(origin_design_m=[1., 0., 0.], components=[component]))

    def test_native_exclusion_exact_tangent_face_and_one_ulp_penetration(self):
        for mirror in (False, True):
            lo = np.array([.2, 0., .2]); hi = np.array([.8, 1., .8])
            a = np.array([0., 1., 0.]); b = np.array([1., 2., 1.])
            if mirror:
                lo[1], hi[1] = -hi[1], -lo[1]; a[1], b[1] = -b[1], -a[1]
            rows = [dict(low_m=a.tolist(), high_m=b.tolist()), dict(low_m=[3, 3, 3], high_m=[4, 4, 4])]
            for ordered in (rows, rows[::-1]):
                self.assertTrue(liquid._clear_of_exclusion_boxes(lo, hi, ordered))
                penetrated_lo, penetrated_hi = lo.copy(), hi.copy()
                if mirror: penetrated_lo[1] = math.nextafter(lo[1], -math.inf)
                else: penetrated_hi[1] = math.nextafter(hi[1], math.inf)
                self.assertFalse(liquid._clear_of_exclusion_boxes(penetrated_lo, penetrated_hi, ordered))

    def test_nonconvex_hull_is_not_certified_by_its_corners(self):
        hull = cube((3, 3, 3))-cube((.2, .2, 3), (1.4, 1.4, 0))
        result = liquid.pack('internal-hole', hull, [tank(0, 2.5, (1.5, 1.5, 1))])
        self.check_cells(result, hull)
        self.assertGreater(result['ledger']['screened_cells'], 1)

    def test_insufficient_volume_fails_without_density_or_capacity_adjustment(self):
        inputs = [tank(0, 1.1, (0, 0, 0))]; before = copy.deepcopy(inputs)
        with self.assertRaisesRegex(liquid.LiquidError, 'insufficient'):
            liquid.pack('too-large', cube((1, 1, 1)), inputs)
        self.assertEqual(inputs, before)

    def test_finite_budget_failure_is_explicit(self):
        with self.assertRaisesRegex(liquid.LiquidError, 'budget'):
            liquid.pack('budget', cube((2, 2, 2))-cube((.2, .2, 2), (.9, .9, 0)),
                        [tank(0, 3, (1, 1, 1))], limits={'max_screened_cells': 1})

    def test_full_box_capacity_closes_without_reserve_volume(self):
        hull = cube((1, 1, 1))
        self.check_cells(liquid.pack('full', hull, [tank(0, 1, (.5, .5, .5))]), hull)

    def test_rotated_thin_geometry_has_real_contained_volume(self):
        angle = .31; c, s = math.cos(angle), math.sin(angle)
        hull = cube((5, .8, .06)).transform([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0]])
        self.check_cells(liquid.pack('thin', hull, [tank(0, .08, (2, .8, .03))]), hull)

    def test_invalid_resource_limits_fail_closed(self):
        for limits in [{'max_cells': 0}, {'max_cells': 1.5}, {'max_cells': 'many'},
                       {'max_seconds': float('inf')}, {'unbounded': True}]:
            with self.subTest(limits=limits), self.assertRaises(liquid.LiquidError):
                liquid.pack('bad-budget', cube((1, 1, 1)), [tank(0, .5, (0, 0, 0))], limits=limits)

    def test_fluid_drag_policy_is_explicit_assumed_and_not_calibration(self):
        expected = dict(law='quadratic_unresolved_tumbling_v1', drag_coefficient=1.0, basis='assumed',
                        screening_uncertainty_fraction=1.0,
                        note='Unresolved tumbling pressure-drag engineering policy; not FAA/OEM calibration or air Cd.')
        wet = liquid.pack('policy', cube((1, 1, 1)), [tank(0, .25, (.5, .5, .5))])
        self.assertEqual(wet['fluid_policy_v1'], expected)
        design = self.directory/'electric.toml'; design.write_text('[aircraft]\nname="electric"\n')
        a = SimpleNamespace(name='electric', spec={'mass': {}}, path=str(design), engines=[])
        dry = liquid.produce(a, np.zeros(3), glb=self.directory/'absent.glb')
        self.assertEqual(dry['fluid_policy_v1'], expected)
        self.assertEqual(dry['tanks'], [])
        self.assertEqual(dry['method']['version'], 1)
        self.assertEqual(dry['method']['origin_design_m'], [0., 0., 0.])

    def test_invalid_and_duplicate_tank_contracts_are_rejected(self):
        for inputs in [[tank(0, 1, (0, 0, 0)), tank(0, 1, (0, 0, 0))],
                       [tank(0, float('nan'), (0, 0, 0))],
                       [dict(tank(0, 1, (0, 0, 0)), density_kg_m3=0)]]:
            with self.subTest(inputs=inputs), self.assertRaises(liquid.LiquidError):
                liquid.pack('invalid', cube((2, 2, 2)), inputs)

    def test_glb_all_ancestor_transforms_and_equipment_exclusion(self):
        path = self.directory/'transform.glb'
        # Raw GLB axes: actual producer maps body = [+g.z, -g.x, -g.y].
        # Parent matrix translates +10 glTF z; child TRS rotates then translates.
        parent = np.eye(4); parent[2, 3] = 10
        nodes = [dict(name='root', matrix=parent.reshape(-1, order='F').tolist(), children=[1, 2]),
                 dict(name='airframe', mesh=0, translation=[2, 3, 4],
                      rotation=[0, 0, math.sqrt(.5), math.sqrt(.5)], scale=[2, 1, 1]),
                 dict(name='fsim:propeller:0', mesh=1, translation=[100, 100, 100])]
        glb(path, [cube((1, 1, 1)), cube((20, 20, 20))], nodes)
        solid, receipt = liquid.read_glb(path)
        self.assertAlmostEqual(solid.volume(), 2, places=10)
        np.testing.assert_allclose(solid.bounding_box(), [14, -2, -5, 15, -1, -3], atol=1e-12)
        self.assertEqual(receipt['structural_nodes'], 1)

    def test_actual_model3d_native_body_roundtrip_with_origin_and_ancestors(self):
        from hangar import model3d
        origin = np.array([10., 2., 6.])
        # Integer affine matrix and exact 180-degree quaternion keep the GLB
        # float32 coordinates exactly representable. The 1e-12 assertion covers
        # only binary64 transform arithmetic, not a geometry/volume allowance.
        parent = np.array([[0., 0., 1., 4.], [0., 1., 0., -3.],
                           [-1., 0., 0., 2.], [0., 0., 0., 1.]])
        child = np.array([[-2., 0., 0., 1.], [0., -.5, 0., 2.],
                          [0., 0., 4., -1.], [0., 0., 0., 1.]])
        inverse = np.linalg.inv(parent@child)
        nodes = [dict(name='ancestor', matrix=parent.reshape(-1, order='F').tolist(), children=[1]),
                 dict(name='airframe', mesh=0, translation=[1., 2., -1.],
                      rotation=[0., 0., 1., 0.], scale=[2., .5, 4.])]
        tetra = mf.Manifold(mf.Mesh64(np.array([[12., 5., 9.], [13., 5., 9.],
                                                [12., 7., 9.], [12., 5., 12.]]),
                                    np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]], dtype=np.uint64)))
        shapes = [cube((.5, 1., 2.), (12., 5., 9.)), tetra]

        def export_local(design_points):
            # Call the actual normal model producer's frame function. Do not
            # duplicate its sign convention in the test GLB writer.
            global_gltf = model3d._to_gltf(design_points, origin)
            return global_gltf@inverse[:3, :3].T+inverse[:3, 3]

        for i, shape in enumerate(shapes):
            with self.subTest(shape=('asymmetric-box', 'asymmetric-tetrahedron')[i]):
                path = self.directory/('roundtrip-%d.glb' % i)
                glb(path, [shape], nodes, vertex_transform=export_local)
                imported, _ = liquid.read_glb(path)
                design_points = np.asarray(shape.to_mesh64().vert_properties)[:, :3]
                expected = np.column_stack((origin[0]-design_points[:, 0],
                                            design_points[:, 1]-origin[1],
                                            origin[2]-design_points[:, 2]))
                actual = np.asarray(imported.to_mesh64().vert_properties)[:, :3]
                np.testing.assert_allclose(np.unique(actual, axis=0), np.unique(expected, axis=0),
                                           atol=1e-12, rtol=0.)
                self.assertAlmostEqual(imported.volume(), shape.volume(), places=12)

    def test_glb_union_closed_sections_handles_internal_caps(self):
        path = self.directory/'sections.glb'
        nodes = [dict(name='root', children=[1, 2]), dict(name='fsim:section:nose', mesh=0),
                 dict(name='fsim:section:centre', mesh=1, translation=[1, 0, 0])]
        glb(path, [cube((1, 1, 1)), cube((1, 1, 1))], nodes, authoritative=cube((2, 1, 1)))
        solid, receipt = liquid.read_glb(path)
        self.assertAlmostEqual(solid.volume(), 2, places=10)
        self.assertEqual(receipt['structural_nodes'], 2)

    def test_sectioned_glb_requires_original_unsplit_boundary(self):
        path = self.directory/'missing-original.glb'
        glb(path, [cube((1, 1, 1))], [dict(name='fsim:section:nose', mesh=0)])
        with self.assertRaisesRegex(liquid.LiquidError, 'authoritative|unsplit'):
            liquid.read_glb(path)

    def test_original_boundary_extra_controls_material_hull_not_owner_cap(self):
        path = self.directory/'original.glb'
        glb(path, [cube((2, 2, 2))], authoritative=cube((1, 1, 1)))
        hull, receipt = liquid.read_glb(path)
        self.assertAlmostEqual(hull.volume(), 1., places=12)
        self.assertEqual(receipt['authority'], 'original_unsplit_airframe_accessors_v1')

    def test_model3d_boundary_extras_do_not_add_rendered_meshes_or_nodes(self):
        from hangar import model3d
        B = model3d._Builder(); solid = cube((2, 3, 4), (11, 5, 9)); origin = np.array([10, 2, 6])
        mesh = solid.to_mesh64(); vertices = model3d._to_gltf(np.asarray(mesh.vert_properties)[:, :3], origin)
        triangles = np.asarray(mesh.tri_verts)
        material = B.material('fixture', (.2, .3, .4))
        node = B.node('airframe', B.mesh('airframe', [(vertices, triangles, material)]))
        B.liquid_boundary_v1 = dict(schema='fsim.liquid-boundary.v1', frame='gltf_model_origin_m',
                                    positions=B.accessor(vertices, 'vec3'),
                                    indices=B.accessor(triangles.reshape(-1), 'index'))
        path = self.directory/'actual-finish.glb'
        model3d.finish(B, [node], 'fixture', str(path))
        raw = path.read_bytes(); size = struct.unpack_from('<I', raw, 12)[0]; doc = json.loads(raw[20:20+size])
        extra = doc['extras']['fsim_liquid_boundary_v1']
        self.assertEqual(extra['schema'], 'fsim.liquid-boundary.v1')
        self.assertEqual(extra['node'], doc['scenes'][0]['nodes'][0])
        self.assertEqual(len(doc['meshes']), 1); self.assertEqual(len(doc['nodes']), 2)
        self.assertEqual(doc['nodes'][extra['node']]['children'], [node])
        imported, _ = liquid.read_glb(path)
        self.assertAlmostEqual(imported.volume(), solid.volume(), places=12)
        expected = np.array([[-3, 3, -7], [-1, 6, -3]])
        np.testing.assert_allclose(np.array(imported.bounding_box()).reshape(2, 3), expected, rtol=0., atol=1e-12)

    def test_overlapping_owner_bounds_allocate_unique_actual_cells(self):
        authority = cube((3, 2, 2))
        sections = {'centre': cube((2, 2, 2)), 'left wing': cube((2, 2, 2), (1, 0, 0))}
        tanks = [dict(tank(0, 1.25, (.5, 1, 1)), section='centre'),
                 dict(tank(1, 1.25, (2.5, 1, 1)), section='left wing')]
        result = liquid._pack_owned('overlap', authority, sections, tanks, None, None)
        self.check_cells(result, authority)
        for cell in result['cells']:
            lo, hi = np.array(cell['low_m']), np.array(cell['high_m'])
            self.assertTrue((cube(hi-lo, lo)-sections[cell['section']]).is_empty())

    def test_owner_cap_cannot_supply_capacity_outside_original_skin(self):
        tanks = [dict(tank(0, 1.1, (.5, .5, .5)), section='centre')]
        with self.assertRaisesRegex(liquid.LiquidError, 'insufficient'):
            liquid._pack_owned('outside', cube((1, 1, 1)), {'centre': cube((2, 2, 2))}, tanks, None, None)

    def test_interval_owner_detects_region_missed_by_all_corners_and_centroid(self):
        from hangar.shape import breakup
        aircraft = partition_aircraft()
        low = np.array([-8., 2., -.02]); high = np.array([-2., 3., .02])
        design_points = np.array([[-x, y, -z] for x in (low[0], high[0]) for y in (low[1], high[1])
                                  for z in (low[2], high[2])]+[[5., 2.5, 0.]])
        self.assertEqual(set(breakup.classify(aircraft, design_points)), {breakup.SECTIONS.index('centre')})
        self.assertEqual(breakup.SECTIONS[int(breakup.classify(aircraft, np.array([[3.2, 2.5, 0.]]))[0])], 'nose')
        self.assertEqual(liquid.owner_candidates(aircraft, low, high, [0, 0, 0]), frozenset({'centre', 'nose'}))

    def test_interval_owner_encloses_frozen_classifier_at_one_ulp_boundaries(self):
        from hangar.shape import breakup
        aircraft = partition_aircraft(tail=True)
        for threshold in (-.6, -.05, 1.05, 2.95, 3.45, 7.7, 7.95, 9.05):
            for x in (math.nextafter(threshold, -math.inf), threshold, math.nextafter(threshold, math.inf)):
                for y in (-.05, .05, -2.5, 2.5):
                    point = np.array([x, y, 0.]); body = point*np.array([-1., 1., -1.])
                    expected = breakup.SECTIONS[int(breakup.classify(aircraft, point[None])[0])]
                    with self.subTest(x=x, y=y):
                        self.assertIn(expected, liquid.owner_candidates(aircraft, body, body, [0, 0, 0]))
        for point, expected in [([-1., 0, 0], 'nose'), ([2., 0, 0], 'centre'),
                                ([.5, -2.5, 0], 'left wing'), ([.5, 2.5, 0], 'right wing'), ([10., 0, 0], 'tail')]:
            body = np.array(point)*[-1., 1., -1.]
            self.assertEqual(liquid.owner_candidates(aircraft, body-.001, body+.001, [0, 0, 0]), frozenset({expected}))

    def test_real_owner_partition_never_uses_inferred_cap_solids(self):
        aircraft = partition_aircraft()
        authority = cube((2., 1., .2), (-7., 2., -.1))
        tanks = [dict(tank(0, .05, (-6., 2.5, 0)), section='centre')]
        # No cap solids exist in this seam: only native owner names matter.
        result = liquid._pack_owned('native-partition', authority, {'centre': None}, tanks, None, None,
                                    aircraft=aircraft, origin=[0, 0, 0])
        self.check_cells(result, authority)
        for c in result['cells']:
            self.assertEqual(liquid.owner_candidates(aircraft, c['low_m'], c['high_m'], [0, 0, 0]), frozenset({'centre'}))

    def test_native_owner_shortage_cannot_borrow_centre_material(self):
        authority = cube((1., 1., .2), (-6., 2., -.1))
        tanks = [dict(tank(0, .01, (-5.5, 2.5, 0)), section='nose')]
        with self.assertRaisesRegex(liquid.LiquidError, 'insufficient'):
            liquid._pack_owned('no-borrow', authority, {'nose': None}, tanks, None, None,
                                aircraft=partition_aircraft(), origin=[0, 0, 0])

    def test_certified_hull_boundary_cannot_starve_coarser_owner_refinement(self):
        hull = mf.Manifold.sphere(1., 32)
        def certificate(lo, hi):
            # All material has this owner. Its conservative certificate resolves
            # the left cap early, but needs .5m resolution elsewhere. The left
            # spherical cap alone cannot supply the requested1m3; its curved
            # boundary must not monopolize refinement ahead of pending boxes.
            return 1 if hi[0] <= -.5 or max(hi-lo) <= .5 else 0
        result = liquid.pack('fair-refinement', hull, [tank(0, 1., (0., 0., 0.))],
                             owner_certificate=certificate, limits={'max_screened_cells': 4096})
        self.check_cells(result, hull)

    def test_candidate_bounds_enclose_frozen_mirrored_dihedral_and_override_partition(self):
        from hangar.shape import breakup
        aircraft = partition_aircraft(tail=True)
        aircraft.surfaces[0].sections[1].le[2] = .7
        classifier = liquid._OwnerClassifier(aircraft, [0, 0, 0])
        low = np.array([-11., -7., -3.]); high = np.array([2., 7., 1.])
        points = np.array([[x, y, z] for x in (-1., -.05, .5, 1.05, 2., 3.2, 7.7, 8.5, 10.)
                           for y in (-6., -2.5, -.05, 0., .05, 2.5, 6.) for z in (-.2, 0., .5, 1., 2.)])
        labels = breakup.classify(aircraft, points)
        for owner in breakup.SECTIONS:
            lo, hi = classifier.candidate_bounds(owner, low, high)
            for point in points[labels == breakup.SECTIONS.index(owner)]:
                body = point*[-1., 1., -1.]
                self.assertTrue(np.all(body >= lo) and np.all(body <= hi), (owner, point, lo, hi))
            # Whole-box interval witnesses, not merely sampled point labels.
            for point in points:
                body = point*[-1., 1., -1.]; a, b = body-1e-5, body+1e-5
                if classifier.possible(a, b) == frozenset({owner}):
                    self.assertTrue(np.all(a >= lo) and np.all(b <= hi), (owner, a, b, lo, hi))

    def test_candidate_bounds_are_necessary_planes_not_owner_cap_aabbs(self):
        classifier = liquid._OwnerClassifier(partition_aircraft(tail=True), [0, 0, 0])
        low = np.array([-11., -7., -3.]); high = np.array([2., 7., 1.])
        lo, hi = classifier.candidate_bounds('centre', low, high)
        self.assertLessEqual(lo[0], -7.7); self.assertGreaterEqual(hi[0], .6)
        self.assertGreater(lo[0], low[0]); self.assertLess(hi[0], high[0])
        lo, hi = classifier.candidate_bounds('left wing', low, high)
        self.assertLess(hi[1], 0.)
        self.assertLess(np.prod(hi-lo), np.prod(high-low)/4.)

    def test_candidate_box_union_covers_frozen_owner_and_keeps_original_boundary_guard(self):
        from hangar.shape import breakup
        aircraft = partition_aircraft(tail=True)
        aircraft.surfaces[0].sections[1].le[2] = .7
        classifier = liquid._OwnerClassifier(aircraft, [0, 0, 0])
        low = np.array([-11., -7., -3.]); high = np.array([2., 7., 1.])
        points = np.array([[x, y, z] for x in (-1., .5, 2., 3.2, 8.5, 10.)
                           for y in (-6., -2.5, 0., 2.5, 6.) for z in (0., .5, 1., 2.)])
        labels = breakup.classify(aircraft, points)
        for owner in breakup.SECTIONS:
            boxes = classifier.candidate_boxes(owner, low, high)
            for point in points[labels == breakup.SECTIONS.index(owner)]:
                body = point*[-1., 1., -1.]
                self.assertTrue(any(np.all(body >= a) and np.all(body <= b) for a, b in boxes), (owner, point))
        authority = cube((2., 1., .2), (-7., 2., -.1))
        tanks = [dict(tank(0, .05, (-6., 2.5, 0.)), section='centre')]
        result = liquid._pack_owned('local-candidate', authority, {'centre': None}, tanks, None, None,
                                    aircraft=aircraft, origin=[0, 0, 0])
        self.check_cells(result, authority)

    def test_glb_bad_mesh_and_accessor_fail_closed(self):
        path = self.directory/'bad.glb'; glb(path, [cube((1, 1, 1))])
        data = bytearray(path.read_bytes()); n = struct.unpack_from('<I', data, 12)[0]
        doc = json.loads(data[20:20+n]); doc['accessors'][1]['count'] -= 3
        js = json.dumps(doc).encode(); js += b' ' * (-len(js) % 4)
        binary = data[20+n:]
        path.write_bytes(struct.pack('<III', 0x46546c67, 2, 20+len(js)+len(binary))+
                         struct.pack('<II', len(js), 0x4e4f534a)+js+binary)
        with self.assertRaisesRegex(liquid.LiquidError, 'manifold|closed'):
            liquid.read_glb(path)
        path.write_bytes(b'not a GLB')
        with self.assertRaises(liquid.LiquidError): liquid.read_glb(path)

    def test_fixed_and_rotor_native_tank_identity_units(self):
        fixed = SimpleNamespace(spec={'mass': {'tank': [dict(capacity=800, position=[2, 3, 4])]}}, engines=[])
        self.assertEqual(liquid.tank_records(fixed, [1, 1, 1]), [tank(0, 1, (-1, 2, -3))])
        rotor = SimpleNamespace(spec={'aircraft': {'kind': 'helicopter'}, 'fuel': {'capacity_lb': 2340},
                                     'mass': {'cg_in': [360.4, 0, 247.2]}}, engines=[])
        records = liquid.tank_records(rotor, np.array([360.4, 0, 247.2])*.0254)
        self.assertEqual(len(records), 1); self.assertEqual(records[0]['index'], 0)
        self.assertAlmostEqual(records[0]['capacity_kg'], 1061.4061458, places=9)
        self.assertEqual(records[0]['anchor_body_m'], [0, 0, 0])
        fixed.engines = [SimpleNamespace(type='piston')]
        self.assertEqual(liquid.tank_records(fixed, [0, 0, 0])[0]['density_kg_m3'], 720.)

    def test_authored_all35_applicability_and_native_tank_population(self):
        root = Path(__file__).resolve().parents[4]/'aircraft'
        names = ('a10c b52h c130j c172 c17a e3g e7a ea18g ec130h f15c f16c f22a f35a fa18c '
                 'gripen h6k j10a j20a kc135r kc46a mig29a mirage2000 rafale rc135w rq4b skua '
                 'su25 su27s su57 typhoon u2s cf2 iris uh1h uh60').split()
        counts = []; absent = []
        for name in names:
            spec = tomllib.loads((root/name/(name+'.toml')).read_text(encoding='utf-8'))
            records = liquid.tank_records(SimpleNamespace(spec=spec, engines=[]), [0, 0, 0])
            if records: counts.append(len(records))
            else: absent.append(name)
        self.assertEqual(len(names), 35); self.assertEqual(len(counts), 32)
        self.assertEqual(sum(counts), 149); self.assertEqual(absent, ['skua', 'cf2', 'iris'])

    def test_no_liquid_is_explicit_and_does_not_require_mesh(self):
        path = self.directory/'electric.toml'; path.write_text('[aircraft]\nname="electric"\n')
        a = SimpleNamespace(name='electric', spec={'mass': {}}, path=str(path), engines=[])
        out = self.directory/'electric.liquid-v1.json'
        result = liquid.write(a, out, np.zeros(3), glb=self.directory/'absent.glb')
        self.assertEqual(result['tanks'], []); self.assertEqual(result['cells'], [])
        self.assertTrue(result['no_liquid_reason'])
        self.assertEqual(json.loads(out.read_text()), result)
        physical = dict(aircraft='electric', retained_float=123.12345678901234,
                        liquid_geometry_v1={'required': True})
        (self.directory/'absent.damage.json').write_text(json.dumps(physical))
        liquid.write(a, out, np.zeros(3), glb=self.directory/'absent.glb')
        physical.pop('liquid_geometry_v1')
        self.assertEqual(json.loads((self.directory/'absent.damage.json').read_text()), physical)

    def test_no_liquid_multirotor_binds_actual_rotor_model_stage(self):
        path = self.directory/'electric-rotor.toml'; path.write_text('[aircraft]\nname="electric-rotor"\nkind="multirotor"\n')
        a = SimpleNamespace(name='electric-rotor', path=str(path), engines=[],
                            spec={'aircraft': {'kind': 'multirotor'}, 'mass': {}})
        result = liquid.produce(a, [0, 0, 0], glb=self.directory/'absent.glb')
        source = next(s for s in result['sources'] if s['role'] == 'normal_model_stage')
        self.assertEqual(source['path'], '__init__.py')

    def test_normal_writer_hashes_inputs_and_is_byte_deterministic(self):
        design = self.directory/'fixture.toml'; design.write_text('[aircraft]\nname="fixture"\n')
        model = self.directory/'fixture.glb'
        glb(model, [cube((2, 2, 2))], vertex_transform=exported_design_vertices)
        a = SimpleNamespace(name='fixture', path=str(design), engines=[], bodies=[], surfaces=[],
                            spec={'mass': {'tank': [dict(capacity=800, position=[1, 1, 1])]}})
        output = self.directory/'fixture.liquid-v1.json'
        result = liquid.write(a, output, [0, 0, 0], glb=model); first = output.read_bytes()
        liquid.write(a, output, [0, 0, 0], glb=model)
        self.assertEqual(first, output.read_bytes())
        self.assertTrue(all(len(source['sha256']) == 64 for source in result['sources']))
        self.assertIn('glb', [source['role'] for source in result['sources']])

    def section_fixture(self, capacities=(200., 200.)):
        design = self.directory/'owned.toml'; design.write_text('[aircraft]\nname="owned"\n')
        model = self.directory/'owned.glb'
        glb(model, [cube((1, 1, 1)), cube((1, 1, 1))],
            [dict(name='root', children=[1, 2]), dict(name='fsim:section:nose', mesh=0),
             dict(name='fsim:section:centre', mesh=1, translation=[0, 0, -1])],
            vertex_transform=exported_design_vertices, authoritative=cube((2, 1, 1)))
        physical = {'aircraft': 'owned', 'origin_design_m': [0., 0., 0.],
                    'components': [],
                    'design_mass_kg': 1234.5678901234567,
                    'sections': [dict(name=name, mass_kg=98.12345678901234,
                                      tanks=[dict(index=i, capacity_kg=capacity)])
                                 for i, (name, capacity) in enumerate(zip(('nose', 'centre'), capacities))]}
        model.with_suffix('.damage.json').write_text(json.dumps(physical))
        # Frozen native x_nose = wing LE1.6 - .6 =1.0; the high wing is
        # outside these fuel boxes, preserving nose/centre physical ownership.
        wing = SimpleNamespace(kind='wing', mirror=True, sections=[
            SimpleNamespace(le=np.array([1.6, 0., 10.]), chord=1.),
            SimpleNamespace(le=np.array([1.6, 5., 10.]), chord=1.)])
        a = SimpleNamespace(name='owned', path=str(design), bodies=[], engines=[], surfaces=[wing],
                            spec={'mass': {'tank': [dict(capacity=c, position=[.5+i, .5, .5])
                                                   for i, c in enumerate(capacities)]}})
        return a, model, physical

    def test_single_native_tank_owner_and_additive_damage_declaration(self):
        from hangar.resources import thread_reservation
        a, model, before = self.section_fixture()
        with thread_reservation(1):
            result = liquid.write(a, self.directory/'owned.liquid-v1.json', [0, 0, 0], glb=model)
        self.assertEqual([t['section'] for t in result['tanks']], ['nose', 'centre'])
        self.assertTrue(result['ledger']['single_native_tank_owner'])
        hull, _, sections = liquid.read_glb(model, with_sections=True)
        self.check_cells(result, hull)
        for cell in result['cells']:
            low, high = np.array(cell['low_m']), np.array(cell['high_m'])
            self.assertTrue((cube(high-low, low)-sections[cell['section']]).is_empty())
            self.assertEqual(cell['section'], result['tanks'][cell['tank']]['section'])
        after = json.loads(model.with_suffix('.damage.json').read_text())
        self.assertEqual(after.pop('liquid_geometry_v1'),
                         dict(file='owned.liquid-v1.json', schema=liquid.SCHEMA, required=True))
        self.assertEqual(after, before)

    def test_owner_shortage_cannot_borrow_other_section_volume(self):
        from hangar.resources import thread_reservation
        a, model, before = self.section_fixture((880., 80.))
        output = self.directory/'owned.liquid-v1.json'
        with thread_reservation(1), self.assertRaisesRegex(liquid.LiquidError, 'section nose.*insufficient'):
            liquid.write(a, output, [0, 0, 0], glb=model)
        self.assertFalse(output.exists())
        self.assertEqual(json.loads(model.with_suffix('.damage.json').read_text()), before)

    def test_duplicate_owner_is_rejected_and_rotor_does_not_get_fake_damage(self):
        a, model, before = self.section_fixture()
        before['sections'][1]['tanks'].append(dict(index=0, capacity_kg=200.))
        model.with_suffix('.damage.json').write_text(json.dumps(before))
        with self.assertRaisesRegex(liquid.LiquidError, 'Duplicate'):
            liquid.produce(a, [0, 0, 0], glb=model)
        a.name = 'unsectioned'; a.spec = {'mass': {'tank': [dict(capacity=200., position=[.5, .5, .5])]}}; a.surfaces = []
        model = self.directory/'unsectioned.glb'
        glb(model, [cube((1, 1, 1))], vertex_transform=exported_design_vertices)
        result = liquid.write(a, self.directory/'unsectioned.liquid-v1.json', [0, 0, 0], glb=model)
        self.assertEqual(result['tanks'][0]['section'], 'airframe')
        self.assertFalse(model.with_suffix('.damage.json').exists())


if __name__ == '__main__': unittest.main()
