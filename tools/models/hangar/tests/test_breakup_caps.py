"""Boundary-edge and annular-domain regressions for delivered breakup caps."""
from collections import Counter
import json
from pathlib import Path
import struct
import unittest

import manifold3d as mf
import numpy as np

from hangar.shape import breakup


def edge_counts(points, triangles):
    _, inverse = np.unique(points, axis=0, return_inverse=True)
    faces = inverse[triangles]
    directed = Counter((int(u), int(v)) for a, b, c in faces for u, v in ((a, b), (b, c), (c, a)))
    return directed


def closed(points, triangles):
    edges = edge_counts(points, triangles)
    return all(n == 1 and edges[(v, u)] == 1 for (u, v), n in edges.items())


def capped(points, triangles):
    vertices, faces, centre = breakup.caps(points, triangles)
    return np.concatenate((points, vertices)), np.concatenate((triangles, faces+len(points))), vertices, faces


def open_box(low=(0, 0, 0), high=(1, 1, 1)):
    x0, y0, z0 = low; x1, y1, z1 = high
    points = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                       [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    faces = np.array([[0, 2, 1], [0, 3, 2], [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    return points, faces


class BreakupCapTests(unittest.TestCase):
    def test_separate_caps_cannot_reuse_a_welded_internal_diagonal(self):
        # Missing quad faces share opposite U/V vertices, but no skin edge.
        # Independent projected triangulations both choose the short U--V
        # diagonal. Coordinate welding must not give that edge four cap faces.
        points = np.array([[-1., 0., 0.], [1., 0., 0.],
                           [0., -2., 0.], [0., 2., 0.],
                           [0., 0., -2.], [0., 0., 2.],
                           [0., 0., 3.], [0., 3., 0.]])
        for start_a in range(4):
            for start_b in range(4):
                for reverse_storage in (False, True):
                    with self.subTest(start_a=start_a, start_b=start_b,
                                      reverse_storage=reverse_storage):
                        chunks = []
                        for ring, start, apex in zip(([0, 2, 1, 3], [0, 4, 1, 5]),
                                                     (start_a, start_b), (6, 7)):
                            ring = ring[start:]+ring[:start]
                            chunks.append([[apex, ring[(k+1) % 4], ring[k]] for k in range(4)])
                        if reverse_storage:
                            chunks.reverse()
                        faces = np.array(sum(chunks, []), dtype=np.int64)
                        before_points, before_faces = points.copy(), faces.copy()
                        ap, at, cv, ct = capped(points, faces)
                        np.testing.assert_array_equal(points, before_points)
                        np.testing.assert_array_equal(faces, before_faces)
                        self.assertTrue(closed(ap, at))
                        # Both original projected diamond domains and winding
                        # survive; no extra patch closes the shared diagonal.
                        normals = np.cross(cv[ct[:, 1]]-cv[ct[:, 0]],
                                           cv[ct[:, 2]]-cv[ct[:, 0]])
                        self.assertAlmostEqual(.5*np.linalg.norm(normals, axis=1).sum(), 8., places=12)
                        for triangle, normal in zip(cv[ct], normals):
                            if np.all(triangle[:, 2] == 0.):
                                self.assertGreater(normal[2], 0.)
                            else:
                                self.assertTrue(np.all(triangle[:, 1] == 0.))
                                self.assertLess(normal[1], 0.)

    def test_touching_boundary_loops_consume_edges_not_vertices(self):
        # Two tetrahedra have separate missing triangular faces that touch only
        # at vertex 0. Visiting one loop's vertex must not erase the other's edge.
        points = np.array([[0., 0, 0], [1., 0, 0], [0, 1., 0], [0, 0, 1.],
                           [-1., 0, 0], [0, -1., 0], [0, 0, -1.]])
        faces = np.array([[0, 1, 3], [0, 3, 2], [1, 2, 3],
                          [0, 6, 4], [0, 5, 6], [4, 6, 5]])
        all_points, all_faces, _, _ = capped(points, faces)
        self.assertTrue(closed(all_points, all_faces))

    def test_annular_cap_preserves_hole_and_unsigned_area(self):
        outer = np.array([[-2., -2.], [2., -2.], [2., 2.], [-2., 2.]])
        inner = outer*.5
        points = np.array([[x, y, z] for z in (0., 1.) for ring in (outer, inner) for x, y in ring])
        faces = []
        for i in range(4):
            j = (i+1) % 4
            faces += [(i, j, 8+j), (i, 8+j, 8+i)]  # outer wall
            faces += [(4+i, 12+j, 4+j), (4+i, 12+i, 12+j)]  # inner wall
            faces += [(i, 4+j, j), (i, 4+i, 4+j)]  # bottom annulus
        faces = np.array(faces)
        all_points, all_faces, cv, ct = capped(points, faces)
        self.assertTrue(closed(all_points, all_faces))
        a, b, c = cv[ct[:, 0]], cv[ct[:, 1]], cv[ct[:, 2]]
        area = .5*np.linalg.norm(np.cross(b-a, c-a), axis=1).sum()
        self.assertAlmostEqual(area, 12., places=12)
        # No triangle's interior may cross the open square hole.
        for triangle in cv[ct]:
            centre = triangle.mean(axis=0)
            self.assertFalse(abs(centre[0]) < 1. and abs(centre[1]) < 1.)

    def test_parallel_disjoint_caps_do_not_bridge(self):
        p0, t0 = open_box(); p1, t1 = open_box((0, 0, 2), (1, 1, 3))
        p = np.concatenate((p0, p1)); t = np.concatenate((t0, t1+len(p0)))
        ap, at, cv, ct = capped(p, t)
        self.assertTrue(closed(ap, at))
        self.assertTrue(all(np.ptp(cv[tri][:, 2]) == 0 for tri in ct))

    def test_nonplanar_boundary_endpoints_remain_exact(self):
        p, t = open_box(); p[4:, 2] += [0., .1, 0., .1]
        original = p.copy(); ap, at, cv, ct = capped(p, t)
        np.testing.assert_array_equal(p, original)
        self.assertTrue(closed(ap, at))
        for point in cv:
            self.assertTrue(np.any(np.all(original == point, axis=1)))

    def test_actual_f16_exported_skin_boundaries_close_without_vertex_motion(self):
        path = Path(__file__).resolve().parents[4]/'aircraft/f16c/f16c.glb'
        raw = path.read_bytes(); length = struct.unpack_from('<I', raw, 12)[0]
        doc = json.loads(raw[20:20+length]); binary = raw[28+length:]

        def accessor(index):
            a = doc['accessors'][index]; view = doc['bufferViews'][a['bufferView']]
            dtype = np.dtype({5123: '<u2', 5125: '<u4', 5126: '<f4'}[a['componentType']])
            width = {'VEC3': 3, 'SCALAR': 1}[a['type']]
            return np.ndarray((a['count'], width), dtype=dtype, buffer=binary,
                              offset=view.get('byteOffset', 0)+a.get('byteOffset', 0),
                              strides=(view.get('byteStride', dtype.itemsize*width), dtype.itemsize)).copy()

        count = 0
        for node in doc['nodes']:
            if not node.get('name', '').startswith('fsim:section:'): continue
            count += 1
            with self.subTest(section=node['name']):
                points = []; faces = []; start = 0
                for primitive in doc['meshes'][node['mesh']]['primitives']:
                    if doc['materials'][primitive['material']]['name'] == 'break': continue
                    p = accessor(primitive['attributes']['POSITION']).astype(float)
                    t = accessor(primitive['indices']).reshape(-1, 3).astype(np.int64)
                    points.append(p); faces.append(t+start); start += len(p)
                p = np.concatenate(points); t = np.concatenate(faces); before = p.copy()
                ap, at, cv, ct = capped(p, t)
                np.testing.assert_array_equal(p, before)
                self.assertTrue(closed(ap, at))
                unique, inverse = np.unique(ap, axis=0, return_inverse=True)
                mesh = mf.Mesh64(np.ascontiguousarray(unique), np.ascontiguousarray(inverse[at], dtype=np.uint64))
                self.assertEqual(mf.Manifold(mesh).status(), mf.Error.NoError)
        self.assertEqual(count, 5)


if __name__ == '__main__': unittest.main()
