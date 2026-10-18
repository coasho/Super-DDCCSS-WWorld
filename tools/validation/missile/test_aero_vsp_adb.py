"""Independent binary-writer fixtures; default tests never run native tools."""
import copy
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).parent))
from aero_vsp_frame import paired_conditions
from aero_vsp_adb import assess_adb


def fixture(*,concave=False):
    """Literal pinned writer layout, independent of the production reader."""
    points=[(9.,9.,9.),(0.,0.,0.),(1.,0.,0.),(1.,1.,0.),(0.,1.,0.)]
    trails=[[(1.,float(y),0.),(2.,float(y),.1),(3.,float(y),.2),(100.,float(y),.3)] for y in (0,1)]
    if concave:
        trails[0][1]=trails[0][0]
    fine=points+[p for trail in trails for p in trail]
    coarse=points[1:]+[p for trail in trails for p in trail]
    tris=[(2,3,4,1,1,0,.5),(2,4,5,1,1,0,.5)]
    for index in range(3):
        a,b,c,d=6+index,10+index,11+index,7+index
        tris.extend([(a,b,c,0,0,index+1,0. if concave and index==0 else .5),
                     (a,c,d,0,0,index+1,0. if concave and index==0 else .5)])
    edges=[(1,0,1,2),(1,0,2,3),(1,0,3,4),(1,0,4,1)]
    pairs=set()
    for index in range(3):
        face=[5+index,9+index,10+index,6+index]
        pairs.update(tuple(sorted((a,b))) for a,b in zip(face,face[1:]+face[:1]))
    edges += [(0,1,a,b) for a,b in sorted(pairs)]
    raw=bytearray();offsets={}
    def put(name,fmt,*values):
        if name:
            offsets[name]=len(raw)
        raw.extend(struct.pack('<'+fmt,*values))
    put('header','8i',-123789453,2,0,0,4,len(fine),len(tris),len(edges))
    # Pinned native order is Sref, Cref, Bref, then the three CG coordinates.
    put('refs','6f',.6,.3,.8,.25,.13,-.2)
    put('surface_count','i',1)
    put('surface_id','i',1);put(None,'100s',b'Independent rectangular fixture');put(None,'i',1)
    for index,tri in enumerate(tris):
        put('tri'+str(index),'6if',*tri)
    for index,point in enumerate(fine):
        put('fine'+str(index+1),'3f',*point)
    put('rotors','i',0);put('nozzles','i',0)
    put('levels','i',1);put('level_counts','2i',len(coarse),len(edges))
    for index,point in enumerate(coarse):
        put('coarse'+str(index+1),'3f',*point)
    for index,edge in enumerate(edges):
        put('edge'+str(index),'4i',*edge)
    put('kutta_edge_count','i',1);put('kutta_edge','i',2)
    put('kutta_count','i',2);put('kutta','2i',2,3)
    put('controls','i',0)
    pair=paired_conditions(.123)[1]
    put('conditions','5f',pair['native']['mach'],math.radians(pair['native']['alpha_deg']),
        math.radians(pair['native']['beta_deg']),-1.,1.)
    for label,size,fmt in [('gamma',8,'d'),('force',3*len(edges),'d'),('velocity',12,'d'),('pressure',3*len(tris),'f')]:
        put(label,str(size)+fmt,*([.25]*size))
    put('wake_count','i',2)
    for index,trail in enumerate(trails):
        put('wake_node'+str(index),'i',2+index)
        put(None,'d',float(index))
        exported=trail[:1] if concave and index==0 else trail
        put('wake_size'+str(index),'i',len(exported))
        for j,point in enumerate(exported):
            put('wake%d_point%d'%(index,j),'3d',*point)
    geometry='\n'.join(['# vspgeom v3','1','5 1 1',*(' '.join(map(str,p)) for p in points),
        '1','4 2 3 4 5','1 1 0 0 1 0 1 1 0 1','1 1','1','2 1 3 4',
        '1 2 2 3 4 2 4 5','1 1 1 0 0 1 0 1 1 0 0 1 1 0 1'])+'\n'
    return raw,geometry,offsets,pair


class AdbGeometryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder=Path(self.temp.name)
        self.raw,self.geometry,self.offsets,self.pair=fixture()

    def assess(self,*,raw=None,geometry=None,**changes):
        binary=self.folder/'case.adb';source=self.folder/'case.vspgeom'
        binary.write_bytes(self.raw if raw is None else raw)
        source.write_text(self.geometry if geometry is None else geometry)
        arguments=dict(reference_area=.6,bref=.8,cref=.3,native_cg=[.25,.13,-.2],wake_nodes=4)
        arguments.update(changes)
        return assess_adb(binary,source,self.pair,**arguments)

    def mutate(self,name,fmt,*values):
        raw=self.raw.copy();struct.pack_into('<'+fmt,raw,self.offsets[name],*values);return raw

    def test_independent_valid_fixture_with_unused_input_vertex(self):
        result=self.assess()
        self.assertTrue(result['passed'])
        self.assertFalse(result['model_accepted'])
        self.assertEqual(result['decoded_bytes'],len(self.raw))
        self.assertEqual(result['physical_source_nodes'],4)
        self.assertEqual(result['native_kutta_nodes'],2)

    def test_unequal_reference_lengths_and_nonzero_native_cg(self):
        self.assertTrue(self.assess()['passed'])
        for changes in (dict(bref=.3,cref=.8),dict(native_cg=[.25,0,-.2]),dict(reference_area=.61)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.assess(**changes)

    def test_detached_root_rejected_with_correct_physical_surface(self):
        result=self.assess(raw=self.mutate('wake0_point0','d',1.125))
        self.assertFalse(result['passed'])
        self.assertTrue(result['checks']['physical_surface_points'])
        self.assertFalse(result['checks']['every_internal_root_attached'])

    def test_interior_point_rejected_with_attached_root(self):
        result=self.assess(raw=self.mutate('wake1_point1','3d',2.,1.125,.1))
        self.assertFalse(result['passed'])
        self.assertTrue(result['checks']['every_internal_root_attached'])
        self.assertFalse(result['checks']['every_exported_internal_point_matches_mesh'])

    def test_surface_motion_rejected_independently_of_wakes(self):
        result=self.assess(raw=self.mutate('fine2','3f',.125,0,0))
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['physical_surface_points'])
        self.assertTrue(result['checks']['every_internal_root_attached'])

    def test_surface_coordinate_multiplicity_is_preserved(self):
        raw=self.mutate('fine1','3f',0,0,0)
        struct.pack_into('<i',raw,self.offsets['tri1'],1)
        self.assertFalse(self.assess(raw=raw)['checks']['physical_surface_points'])

    def test_surface_connectivity_change_cannot_hide_behind_same_point_multiset(self):
        raw=self.mutate('fine2','3f',0,1,0)
        struct.pack_into('<3f',raw,self.offsets['fine5'],0,0,0)
        result=self.assess(raw=raw)
        self.assertTrue(result['checks']['physical_surface_points'])
        self.assertFalse(result['passed'])

    def test_authored_te_set_is_independent_of_internal_attachment(self):
        raw=self.mutate('coarse2','3f',1.125,0,0)
        struct.pack_into('<3d',raw,self.offsets['wake0_point0'],1.125,0,0)
        result=self.assess(raw=raw)
        self.assertFalse(result['checks']['complete_authored_te_points'])
        self.assertTrue(result['checks']['every_internal_root_attached'])

    def test_in_range_non_te_kutta_edge_is_rejected(self):
        result=self.assess(raw=self.mutate('kutta_edge','i',1))
        self.assertFalse(result['passed'])
        self.assertTrue(result['checks']['complete_authored_te_points'])

    def test_concave_record_only_claims_its_exported_root(self):
        raw,geometry,_,_=fixture(concave=True)
        result=self.assess(raw=raw,geometry=geometry)
        self.assertTrue(result['passed'])
        self.assertEqual(result['concave_single_node_records'],1)
        self.assertEqual(result['exported_internal_points'],5)

    def test_zero_physical_area_rejected_but_zero_wake_area_allowed(self):
        with self.assertRaises(ValueError):
            self.assess(raw=self.mutate('tri0','6if',2,3,4,1,1,0,0.))
        self.assertTrue(self.assess(raw=self.mutate('tri2','6if',6,10,11,0,0,1,0.))['passed'])

    def test_cached_positive_area_cannot_conceal_collinear_physical_coordinates(self):
        with self.assertRaises(ValueError):
            self.assess(raw=self.mutate('fine4','3f',2.,0.,0.))

    def test_wrong_condition_or_unpaired_supplied_wind_rejected(self):
        with self.assertRaises(ValueError):
            self.assess(raw=self.mutate('conditions','f',.7))
        raw=self.raw.copy();struct.pack_into('<f',raw,self.offsets['conditions']+4,2.)
        with self.assertRaises(ValueError):
            self.assess(raw=raw)
        self.pair=copy.deepcopy(self.pair);self.pair['native']['beta_deg']=0.
        with self.assertRaises(ValueError):
            self.assess()

    def test_reversed_native_pressure_range_is_malformed(self):
        raw=self.raw.copy()
        struct.pack_into('<2f',raw,self.offsets['conditions']+12,1.,-1.)
        with self.assertRaises(ValueError):
            self.assess(raw=raw)

    def test_missing_duplicate_or_invalid_native_kutta_association(self):
        for raw in (self.mutate('kutta','2i',2,2),self.mutate('wake_node1','i',2),
                    self.mutate('kutta_edge','i',99999),self.mutate('wake_count','i',1)):
            with self.subTest(),self.assertRaises(ValueError):
                self.assess(raw=raw)

    def test_all_payload_and_coordinate_fields_must_be_finite(self):
        for label,fmt in (('gamma','d'),('force','d'),('velocity','d'),('pressure','f'),
                          ('fine2','f'),('coarse2','f'),('wake0_point0','d'),('conditions','f'),('refs','f')):
            with self.subTest(label=label),self.assertRaises(ValueError):
                self.assess(raw=self.mutate(label,fmt,float('nan')))

    def test_negative_huge_or_truncated_counts_fail_before_allocation(self):
        for offset in (16,20,24,28,self.offsets['surface_count'],self.offsets['levels'],self.offsets['level_counts'],
                       self.offsets['kutta_count'],self.offsets['kutta_edge_count'],self.offsets['wake_count'],self.offsets['wake_size0']):
            for value in (-1,2147483647):
                raw=self.raw.copy();struct.pack_into('<i',raw,offset,value)
                with self.subTest(offset=offset,value=value),self.assertRaises(ValueError):
                    self.assess(raw=raw)

    def test_invalid_native_geometry_ids_rejected(self):
        for raw in (self.mutate('tri0','i',99999),self.mutate('edge0','4i',1,0,0,2),
                    self.mutate('surface_id','i',2)):
            with self.subTest(),self.assertRaises(ValueError):
                self.assess(raw=raw)

    def test_truncation_extra_case_and_trailing_bytes_rejected(self):
        for raw in (self.raw[:-1],self.raw[:64],self.raw+b'x',self.raw+self.raw):
            with self.subTest(),self.assertRaises(ValueError):
                self.assess(raw=raw)

    def test_optional_unsteady_symmetry_rotor_nozzle_control_profiles_rejected(self):
        mutations=[(12,1),(8,1),(self.offsets['rotors'],1),(self.offsets['nozzles'],1),(self.offsets['controls'],1)]
        for offset,value in mutations:
            raw=self.raw.copy();struct.pack_into('<i',raw,offset,value)
            with self.subTest(offset=offset),self.assertRaises(ValueError):
                self.assess(raw=raw)

    def test_legacy_layout_and_other_declared_solver_versions_rejected(self):
        raw=self.raw.copy();del raw[self.offsets['tri0']+20:self.offsets['tri0']+24]
        with self.assertRaises(ValueError):
            self.assess(raw=raw)
        with self.assertRaises(ValueError):
            self.assess(solver_version='6.4.8')

    def test_source_header_counts_parts_parents_uv_and_wakes_are_validated(self):
        corruptions=[self.geometry.replace('# vspgeom v3','# vspgeom v2'),
            self.geometry.replace('\n1\n5 1 1','\n2\n5 1 1',1),
            self.geometry.replace('5 1 1','2147483647 1 1',1),
            self.geometry.replace('4 2 3 4 5','4 2 3 3 5',1),
            self.geometry.replace('1 1 0 0 1 0 1 1 0 1','0 1 0 0 1 0 1 1 0 1',1),
            self.geometry.replace('\n1 1\n1\n2 1','\n1 2\n1\n2 1',1),
            self.geometry.replace('2 1 3 4','2 1 3 3',1),
            self.geometry.replace('1 2 2 3 4 2 4 5','1 2 2 3 99999 2 4 5',1),
            self.geometry.replace('1 1 1 0 0','1 1 1 nan 0',1),self.geometry+'unexpected\n']
        for geometry in corruptions:
            with self.subTest(geometry=geometry[-80:]),self.assertRaises(ValueError):
                self.assess(geometry=geometry)

    def test_invalid_api_references_and_wake_size_rejected(self):
        for changes in (dict(wake_nodes=2),dict(wake_nodes=True),dict(bref=0),dict(cref=float('inf')),
                        dict(native_cg=[0,0]),dict(reference_area=True)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.assess(**changes)


def retained_native(root):
    """Explicit file-only campaign audit, excluded from default unit tests."""
    base=Path(root).resolve()/'build/architecture-reviews/sim-m02/aero'
    records=[]
    for label,relative,expected in (
        ('v1','vsp-closure/root-mapping-fix/followup/full-alpha-',False),
        ('v2','vsp-closure/wake-mapping-fix-v2/followup/full-alpha-',True),
        ('joint-v2','vsp-closure/wake-mapping-fix-v2/joint-core-control/alpha-',True)):
        for pair in paired_conditions(.123):
            alpha=pair['physical']['alpha_deg'];folder=base/(relative+str(alpha))
            result=assess_adb(folder/'missile.adb',folder/'missile.vspgeom',pair,
                reference_area=math.pi*.1778**2/4,bref=.1778,cref=.1778,native_cg=[1.83,0,0],wake_nodes=8)
            if result['passed']!=expected:
                raise AssertionError('Unexpected retained native classification: '+label+str(alpha))
            records.append(dict(label=label,alpha=alpha,expected=expected,**result))
    folder=base/'source-built-campaign/vsp-frame-alpha-0'
    result=assess_adb(folder/'missile.adb',folder/'missile.vspgeom',paired_conditions(.123)[0],
        reference_area=math.pi*.1778**2/4,bref=.1778,cref=.1778,native_cg=[1.83,0,0],wake_nodes=8)
    if not result['passed']:
        raise AssertionError('The identity-numbered all-thick geometry must pass attachment')
    records.append(dict(label='original-all-thick',alpha=0.,expected=True,**result))
    print(json.dumps(dict(cases=records,all_expected_classifications=True,model_accepted=False),indent=2))


if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--retained-native':
        retained_native(sys.argv[2])
    else:
        unittest.main()
