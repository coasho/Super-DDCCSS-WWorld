"""Independent mass/first/second-moment partition checks, without flight execution."""
import itertools
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from hangar import pipeline
from hangar.geometry.aircraft import Aircraft
from hangar.mass import MassModel, _gear_mass_points

ROOT=Path(pipeline.repo_root())


def point_tensor(items,origin):
    """Direct particle definition; independent of MassModel.empty/damage helpers."""
    mass=0.;first=np.zeros(3);tensor=np.zeros((3,3))
    for _,amount,points,weights,_ in items:
        for point,weight in zip(points,weights):
            value=amount*weight;offset=point-origin
            mass+=value;first+=value*offset
            tensor+=value*(np.dot(offset,offset)*np.eye(3)-np.outer(offset,offset))
    return mass,first,tensor


def declared_tensor(empty):
    return np.array([[empty['ixx'],-empty['ixy'],-empty['ixz']],
                     [-empty['ixy'],empty['iyy'],-empty['iyz']],
                     [-empty['ixz'],-empty['iyz'],empty['izz']]])


class AuthoredQuadrature(unittest.TestCase):
    def test_f16_source_diagonal_targets_are_retained(self):
        model=MassModel(Aircraft.load(str(ROOT/'aircraft/f16c/f16c.toml')))
        value=model.empty()
        np.testing.assert_allclose([value[k] for k in ('ixx','iyy','izz')],
            [11236.47875796528,72065.343134412,78834.83737123005],rtol=1e-14)

    def test_clean_gear_uses_authored_hinge_rotation_and_preserves_fixed_gear(self):
        aircraft=Aircraft.load(str(ROOT/'aircraft/f16c/f16c.toml'))
        for gear in aircraft.gear:
            # The wheel twist is about the hinge-to-axle strut itself, so it
            # cannot move this representative mass point. Only swing acts.
            expected=[]
            for _,contact in gear.positions():
                axle=contact+np.array([0.,0.,gear.wheel_diameter/2])
                hinge=gear.attach.copy()
                if contact[1]*hinge[1]<0:hinge[1]*=-1
                direction=gear.spec.get('retract','aft' if gear.steerable else 'forward')
                axes={'forward':[0.,1.,0.],'aft':[0.,-1.,0.]}
                axis=np.array(axes[direction]);angle=np.radians(gear.spec.get('retract_deg',90))
                r=axle-hinge
                expected.append(hinge+r*np.cos(angle)+np.cross(axis,r)*np.sin(angle)+axis*np.dot(axis,r)*(1-np.cos(angle)))
            np.testing.assert_allclose(_gear_mass_points(gear,True),expected,atol=1e-14)
            unchanged=copy.copy(gear);unchanged.retractable=False
            np.testing.assert_array_equal(_gear_mass_points(unchanged,True),_gear_mass_points(unchanged,False))

    @staticmethod
    def synthetic():
        model=MassModel.__new__(MassModel)
        points=np.array(list(itertools.product((-1.,0.,1.),repeat=3)))
        prior=np.ones(len(points))/len(points)
        density=np.exp(.2*points[:,0]-.15*points[:,1]**2+.1*points[:,0]*points[:,2]);density/=density.sum()
        fixed=np.array([[.3,-.2,.1]])
        target_items=[('structure',9.,points,density,'structure'),('engine',1.,fixed,np.ones(1),'engine')]
        mass,first,_=point_tensor(target_items,np.zeros(3));origin=first/mass
        _,_,J=point_tensor(target_items,origin)
        model.items=[('structure',9.,points,prior,'structure'),('engine',1.,fixed,np.ones(1),'engine')]
        model.aircraft=SimpleNamespace(name='synthetic',gear=[],bodies=[])
        model._authored_empty=dict(mass=mass,cg=origin,ixx=J[0,0],iyy=J[1,1],izz=J[2,2],
            ixy=-J[0,1],ixz=-J[0,2],iyz=-J[1,2])
        return model

    def test_reconciliation_preserves_items_mounts_and_all_additive_moments(self):
        model=self.synthetic();original=copy.deepcopy(model.items)
        model._reconcile_authored_distribution()
        other=self.synthetic();other._reconcile_authored_distribution()
        for before,after,repeat in zip(original,model.items,other.items):
            self.assertEqual(before[1],after[1]);np.testing.assert_array_equal(before[2],after[2])
            self.assertGreaterEqual(after[3].min(),0);self.assertAlmostEqual(after[3].sum(),1)
            np.testing.assert_array_equal(after[3],repeat[3])
            if before[4]=='engine':np.testing.assert_array_equal(before[3],after[3])
        _,first,J=point_tensor(model.items,model._authored_empty['cg'])
        np.testing.assert_allclose(first,0,atol=1e-10)
        np.testing.assert_allclose(J,declared_tensor(model._authored_empty),atol=1e-10)

    def test_incompatible_declared_tensor_is_refused_without_fallback(self):
        for diagonal in ([100.,1.,1.],[100.,100.,100.]):
            model=self.synthetic()
            for name,value in zip(('ixx','iyy','izz'),diagonal):model._authored_empty[name]=value
            with self.assertRaises(ValueError):model._reconcile_authored_distribution()

    def test_all_source_stores_construct_and_keep_their_declared_basis(self):
        from hangar.engineering import roster
        count=0
        for record in roster():
            if record['category']!='store':continue
            model=MassModel(Aircraft.load(record['path']));value=model.empty();count+=1
            self.assertFalse(hasattr(model,'mass_partition'))
            if model.spec.get('gyration_reference')=='body-length-v1':
                length=max(float(b.x[-1]) for b in model.aircraft.bodies)-min(float(b.x[0]) for b in model.aircraft.bodies)
                expected=value['mass']*(np.asarray(model.spec['gyration'])*length)**2
                np.testing.assert_allclose([value[k] for k in ('ixx','iyy','izz')],expected,rtol=1e-14)
        self.assertEqual(count,53)

    def test_unknown_or_nonfinite_store_basis_is_refused(self):
        for bad in ('unknown',float('nan')):
            aircraft=Aircraft.load(str(ROOT/'aircraft/r77m/r77m.toml'))
            if isinstance(bad,str):aircraft.spec['mass']['gyration_reference']=bad
            else:aircraft.spec['mass']['gyration'][0]=bad
            with self.assertRaises(ValueError):MassModel(aircraft).empty()

    def test_body_length_basis_matches_independent_cylinder_volume_and_rod_limit(self):
        # The expected tensor is integrated from a uniform physical volume,
        # without MassModel's inertia or gyration conversion helpers. A very
        # small radius supplies the uniform-rod limiting case independently.
        nodes,weights=np.polynomial.legendre.leggauss(8)
        for radius in (.1,1e-6):
            aircraft=Aircraft.load(str(ROOT/'aircraft/r77m/r77m.toml'))
            stations=[s['x'] for body in aircraft.spec['body'] for s in body['stations']]
            length=max(stations)-min(stations);mass=float(aircraft.spec['mass']['empty'])
            aircraft.spec['mass']['gyration']=[radius/(np.sqrt(2)*length),
                np.sqrt(length**2/12+radius**2/4)/length,
                np.sqrt(length**2/12+radius**2/4)/length]
            expected=np.zeros((3,3))
            for x,wx in zip(nodes,weights):
                for u,wu in zip((nodes+1)/2,weights/2):
                    for phi in np.arange(16)*2*np.pi/16:
                        point=np.array([x*length/2,radius*np.sqrt(u)*np.cos(phi),radius*np.sqrt(u)*np.sin(phi)])
                        dm=mass*wx/2*wu/16
                        expected+=dm*(np.dot(point,point)*np.eye(3)-np.outer(point,point))
            value=MassModel(aircraft).empty()
            self.assertAlmostEqual(value['mass'],mass,places=10)
            np.testing.assert_allclose(declared_tensor(value),expected,rtol=1e-12,atol=1e-12)

    def test_f16_particles_realize_the_authoritative_full_empty_tensor(self):
        aircraft=Aircraft.load(str(ROOT/'aircraft/f16c/f16c.toml'))
        model=MassModel(aircraft);empty=model.empty()
        amount,first,tensor=point_tensor(model.items,np.asarray(empty['cg']))
        self.assertAlmostEqual(amount,empty['mass'],places=8)
        np.testing.assert_allclose(first,np.zeros(3),atol=1e-7)
        np.testing.assert_allclose(tensor,declared_tensor(empty),rtol=1e-9,atol=1e-7)

    def test_existing_f16_generated_sections_partition_xml_mass_and_full_tensor(self):
        import json
        frame=json.loads((ROOT/'aircraft/f16c/f16c.damage.json').read_text(encoding='utf-8'))
        balance=ET.parse(ROOT/'aircraft/f16c/f16c.xml').getroot().find('mass_balance')
        self.assertIsNotNone(balance)
        self.assertEqual(balance.attrib['negated_crossproduct_inertia'],'false')
        mass=float(balance.find('emptywt').text)
        # XML stores structural-frame plain products; damage records store body
        # plain products. Rotation diag(-1,1,-1) fixes xy/yz and retains xz.
        names=('ixx','iyy','izz','ixy','ixz','iyz');v=[float(balance.find(k).text) for k in names]
        target=np.array([[v[0],v[3],-v[4]],[v[3],v[1],v[5]],[-v[4],v[5],v[2]]])
        matrices=[];first=np.zeros(3);total_mass=0.
        for section in frame['sections']:
            amount=section['mass_kg'];r=np.asarray(section['cg']);a=section['inertia']
            own=np.array([[a[0],-a[3],-a[4]],[-a[3],a[1],-a[5]],[-a[4],-a[5],a[2]]])
            self.assertGreaterEqual(float(np.linalg.eigvalsh(own).min()),0)
            matrices.append(own+amount*(np.dot(r,r)*np.eye(3)-np.outer(r,r)))
            first+=amount*r;total_mass+=amount
        # The existing records round mass/coordinates/tensors to five decimal
        # places. These bounds cover that declared serialization precision;
        # they are tiny beside the observed 27912 kg m^2 pitch discrepancy.
        self.assertAlmostEqual(total_mass,mass,delta=1e-4)
        np.testing.assert_allclose(first,np.zeros(3),atol=.1)
        np.testing.assert_allclose(sum(matrices),target,rtol=0,atol=1.)
        for size in range(len(matrices)):
            for removed in itertools.combinations(range(len(matrices)),size):
                remaining=mass-sum(frame['sections'][i]['mass_kg'] for i in removed)
                if remaining<=0:continue
                retained_first=-sum((frame['sections'][i]['mass_kg']*np.asarray(frame['sections'][i]['cg']) for i in removed),np.zeros(3))
                r=retained_first/remaining
                remainder=target-sum((matrices[i] for i in removed),np.zeros((3,3)))-remaining*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
                self.assertGreaterEqual(float(np.linalg.eigvalsh(remainder).min()),-1e-6)
                self.assertGreaterEqual(float(np.linalg.eigvalsh(.5*np.trace(remainder)*np.eye(3)-remainder).min()),-1e-6)


class SerializedOracle(unittest.TestCase):
    @staticmethod
    def write_zero_mass_fixture(directory, tiny_mass):
        xml=Path(directory)/'mass.xml';damage=Path(directory)/'mass.damage.json'
        xml.write_text('<fdm_config><mass_balance negated_crossproduct_inertia="false"><emptywt unit="KG">1</emptywt><ixx unit="KG*M2">2</ixx><iyy unit="KG*M2">2</iyy><izz unit="KG*M2">2</izz><location name="CG" unit="M"><x>0</x><y>0</y><z>0</z></location></mass_balance></fdm_config>',encoding='utf-8')
        sections=[dict(name='massive',mass_kg=1.,cg=[0.,0.,0.],inertia=[1.,1.,1.,0.,0.,0.]),
                  dict(name='massless',mass_kg=tiny_mass,cg=[0.,0.,0.],inertia=[1.,1.,1.,0.,0.,0.])]
        damage.write_text(json.dumps(dict(origin_design_m=[0.,0.,0.],sections=sections)),encoding='utf-8')
        return xml,damage

    def test_zero_mass_section_cannot_carry_finite_own_inertia(self):
        from hangar.engineering import mass_partition_audit
        with tempfile.TemporaryDirectory() as directory:
            xml,damage=self.write_zero_mass_fixture(directory,0.)
            # Full additive moments and PSD tensors agree, but the second
            # serialized section has no mass with which to realize its inertia.
            with self.assertRaisesRegex(ValueError,'mass|inertia'):
                mass_partition_audit(xml,damage)
            # A genuinely empty serialized section remains valid. Adjust the
            # independent whole tensor to retain exact additive consistency.
            data=json.loads(damage.read_text(encoding='utf-8'));data['sections'][1]['inertia']=[0.]*6
            damage.write_text(json.dumps(data),encoding='utf-8')
            document=ET.parse(xml)
            for name in ('ixx','iyy','izz'):document.getroot().find('mass_balance/'+name).text='1'
            document.write(xml,encoding='utf-8')
            self.assertTrue(mass_partition_audit(xml,damage)['passed'])

    def test_zero_retained_native_mass_cannot_hide_finite_complement_inertia(self):
        from hangar.engineering import mass_partition_audit
        with tempfile.TemporaryDirectory() as directory:
            xml,damage=self.write_zero_mass_fixture(directory,1e-300)
            # Each section's encoded mass is positive. Native subtraction of
            # the first section nevertheless leaves exactly1-1=0, while the
            # retained tensor is I. It must be checked before the zero-mass skip.
            with self.assertRaisesRegex(ValueError,'mass|complement'):
                mass_partition_audit(xml,damage)

    def test_malformed_numeric_budget_metadata_cannot_bypass_conservation(self):
        from hangar.engineering import mass_partition_audit
        with tempfile.TemporaryDirectory() as directory:
            xml=Path(directory)/'mass.xml';damage=Path(directory)/'mass.damage.json'
            xml.write_text('<fdm_config><mass_balance><emptywt unit="KG">1</emptywt><ixx unit="KG*M2">1</ixx><iyy unit="KG*M2">1</iyy><izz unit="KG*M2">1</izz><location name="CG" unit="M"><x>0</x><y>0</y><z>0</z></location></mass_balance></fdm_config>',encoding='utf-8')
            for scale in ([float('nan'),1.,1.],[float('inf'),1.,1.],[1.,1.],[0.,1.,1.]):
                data=dict(origin_design_m=[0.,0.,0.],sections=[],mass_partition=dict(
                    normalization_scale_m=scale,dimensionless_moment_tolerance=2e-12))
                damage.write_text(json.dumps(data),encoding='utf-8')
                with self.assertRaisesRegex(ValueError,'normalization scale'):
                    mass_partition_audit(xml,damage)

    def test_independent_artifact_reader_rejects_nonphysical_whole_and_partition(self):
        from hangar.engineering import mass_partition_audit
        with tempfile.TemporaryDirectory() as directory:
            xml=Path(directory)/'mass.xml';damage=Path(directory)/'mass.damage.json'
            template='<fdm_config><mass_balance negated_crossproduct_inertia="false"><emptywt unit="KG">2</emptywt><ixx unit="KG*M2">2</ixx><iyy unit="KG*M2">%s</iyy><izz unit="KG*M2">4</izz><location name="CG" unit="M"><x>0</x><y>0</y><z>0</z></location></mass_balance></fdm_config>'
            sections=[dict(name=str(x),mass_kg=1.,cg=[x,0.,0.],inertia=[1.,1.,1.,0.,0.,0.]) for x in (-1.,1.)]
            # Independent two isotropic bodies, parallel-axis shift along x.
            xml.write_text(template%'4',encoding='utf-8')
            damage.write_text(json.dumps(dict(origin_design_m=[0,0,0],sections=sections)),encoding='utf-8')
            result=mass_partition_audit(xml,damage)
            self.assertEqual(result['complement_count'],3)
            sections[0]['inertia'][1]=2.
            damage.write_text(json.dumps(dict(origin_design_m=[0,0,0],sections=sections)),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'conserve'):mass_partition_audit(xml,damage)
            xml.write_text(template%'40',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'physical'):mass_partition_audit(xml)

    def test_finite_extreme_scale_cannot_overflow_a_mismatched_tensor_budget(self):
        from hangar.engineering import mass_partition_audit
        with tempfile.TemporaryDirectory() as directory:
            xml=Path(directory)/'mass.xml';damage=Path(directory)/'mass.damage.json'
            xml.write_text('<fdm_config><mass_balance><emptywt unit="KG">1</emptywt><ixx unit="KG*M2">1</ixx><iyy unit="KG*M2">1</iyy><izz unit="KG*M2">1</izz><location name="CG" unit="M"><x>0</x><y>0</y><z>0</z></location></mass_balance></fdm_config>',encoding='utf-8')
            for epsilon in (0.,2e-12):
                # Finite inputs and matching mass/first moment. Section tensor
                # is deliberately ten times the native whole tensor.
                section=dict(name='mismatch',mass_kg=1.,cg=[0.,0.,0.],inertia=[10.,10.,10.,0.,0.,0.])
                data=dict(origin_design_m=[0.,0.,0.],sections=[section],mass_partition=dict(
                    normalization_scale_m=[1e308,1.,1.],dimensionless_moment_tolerance=epsilon))
                damage.write_text(json.dumps(data),encoding='utf-8')
                with self.assertRaises(ValueError):mass_partition_audit(xml,damage)

    def test_finite_scale_inflation_is_refused_against_actual_geometry(self):
        from hangar.engineering import mass_partition_audit
        with tempfile.TemporaryDirectory() as directory:
            xml=Path(directory)/'mass.xml';damage=Path(directory)/'mass.damage.json'
            xml.write_text('<fdm_config><mass_balance><emptywt unit="KG">1</emptywt><ixx unit="KG*M2">1</ixx><iyy unit="KG*M2">1</iyy><izz unit="KG*M2">1</izz><location name="CG" unit="M"><x>0</x><y>0</y><z>0</z></location></mass_balance></fdm_config>',encoding='utf-8')
            section=dict(name='mismatch',mass_kg=1.,cg=[0.,0.,0.],inertia=[10.,10.,10.,0.,0.,0.])
            data=dict(origin_design_m=[0.,0.,0.],sections=[section],
                panels=[dict(corners=list(itertools.product((-1.,1.),repeat=3)))],
                components=[],mass_partition=dict(normalization_scale_m=[1e100,1.,1.],dimensionless_moment_tolerance=2e-12))
            damage.write_text(json.dumps(data),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'geometry'):
                mass_partition_audit(xml,damage)
            data['mass_partition']['normalization_scale_m']=[1.,1.,1.]
            section['inertia']=[1.,1.,1.,0.,0.,0.]
            damage.write_text(json.dumps(data),encoding='utf-8')
            self.assertTrue(mass_partition_audit(xml,damage)['passed'])
            del data['panels']
            damage.write_text(json.dumps(data),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'geometry proof'):
                mass_partition_audit(xml,damage)


if __name__=='__main__':unittest.main()
