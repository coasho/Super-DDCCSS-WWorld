"""Typed skin allocation and independent serialized rigid-mass invariants.

These small synthetic engineering fixtures do not generate native aircraft,
fit coefficients or claim an actual aircraft's measured skin thickness.
"""
import copy
import itertools
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from hangar import damage
from hangar.shape.breakup import SECTIONS


def surface(kind,x,span,chord):
    return SimpleNamespace(kind=kind,mirror=True,sections=[
        SimpleNamespace(le=[x,0,0],chord=chord),SimpleNamespace(le=[x,span,0],chord=chord)])


def aircraft():
    return SimpleNamespace(name='typed-skin-fixture',spec={'flight_control':{'n_max':3},'mass':{}},
        surfaces=[surface('wing',2,4,2),surface('htail',6,2,1)],
        bodies=[SimpleNamespace(kind='fuselage',x=np.array([0.,8.]),w=np.array([1.,1.]))])


def item(name,mass,kind,points,weights,sections):
    return dict(name=name,mass=mass,kind=kind,points=np.asarray(points,float),
        weights=np.asarray(weights,float),sections=sections)


def fixture():
    return [
        item('outer structure',100,'structure',[[0,0,.2],[3,2,.1]],[.25,.75],['nose','right wing']),
        item('left structure',30,'structure',[[3,-2,.1]],[1],['left wing']),
        item('tail structure',20,'structure',[[6.5,0,.1]],[1],['tail']),
    ]


def equipment(kind,mass=1000):
    return item('heavy '+kind,mass,kind,[[3,0,.2],[3,2,.1]],[.75,.25],['centre','right wing'])


def fake_mass(items):
    return SimpleNamespace(items=[(i['name'],i['mass'],i['points'].copy(),i['weights'].copy(),i['kind']) for i in items],
        mtow=5000.,spec={},mass_partition={'version':'synthetic immutable physical distribution'})


def build(items,*,serialized=False):
    mass=fake_mass(items);model=SimpleNamespace(S=10.)
    shares={s:dict(normal_share=.2,side_share=0.,normal_cp=[1.,1.,0.],side_cp=None) for s in SECTIONS}
    roots={s:dict(at=[0.,0.,0.],axes=[[1.,0.,0.],[0.,1.,0.]]) for s in SECTIONS}
    panels=[dict(kind='body',section=s,body='synthetic shell',centre=[0.,0.,0.],half=[1.,1.,1.],area_m2=10.) for s in SECTIONS]
    with patch('hangar.vulnerability.components',return_value=[]),patch('hangar.damage._shares',return_value=shares),\
            patch('hangar.damage._roots',return_value=roots),patch('hangar.damage._panels',return_value=panels),\
            patch('hangar.damage.contact_geometry',return_value={'version':1,'meshes':[]}),\
            patch('hangar.carriage.stations',return_value=None),patch('hangar.mass.MassModel',return_value=mass):
        if not serialized:return damage.damage_data(aircraft(),model=model,mass_model=mass,origin=np.zeros(3))
        parent='D:/ENV/tmp' if os.name=='nt' else None
        with tempfile.TemporaryDirectory(dir=parent) as folder:
            path=Path(folder)/'typed-skin.damage.json';damage.write(aircraft(),str(path),np.zeros(3),model=model)
            return json.loads(path.read_text(encoding='utf-8'))


def expected_skin(items):
    # The fixture states each particle's intended physical ownership directly.
    # No production classifier or allocation helper computes this reference.
    result={s:0. for s in SECTIONS}
    for row in items:
        if row['kind']=='structure':
            for weight,section in zip(row['weights'],row['sections']):result[section]+=.35*row['mass']*weight
    return result


def strip_skin(document):
    result=copy.deepcopy(document)
    for section in result['sections']:section.pop('skin_kg')
    return result


def physical_moments(items,retained):
    particles=[]
    for row in items:
        for weight,point,section in zip(row['weights'],row['points'],row['sections']):
            if section in retained:particles.append((row['mass']*weight,point*np.array([-1,1,-1])))
    mass=sum(m for m,_ in particles)
    first=sum((m*p for m,p in particles),np.zeros(3))
    second=sum((m*(np.eye(3)*np.dot(p,p)-np.outer(p,p)) for m,p in particles),np.zeros((3,3)))
    return mass,first,second


def serialized_moments(sections):
    mass=0.;first=np.zeros(3);second=np.zeros((3,3))
    for row in sections:
        m=row['mass_kg'];cg=np.asarray(row['cg']);ixx,iyy,izz,pxy,pxz,pyz=row['inertia']
        centre=np.array([[ixx,-pxy,-pxz],[-pxy,iyy,-pyz],[-pxz,-pyz,izz]])
        mass+=m;first+=m*cg;second+=centre+m*(np.eye(3)*np.dot(cg,cg)-np.outer(cg,cg))
    return mass,first,second


class TypedSkinMass(unittest.TestCase):
    def test_heavy_nonstructural_equipment_cannot_thicken_a_skin(self):
        base=fixture();expected=expected_skin(base);self.assertEqual(damage.SKIN_SHARE,.35)
        for kind in ('engine','systems','gear','item'):
            for amount in (10.,1000.,10000.):
                with self.subTest(kind=kind,mass=amount):
                    actual=build(base+[equipment(kind,amount)])
                    self.assertEqual({s['name']:s['skin_kg'] for s in actual['sections']},expected)

    def test_relocating_structure_moves_skin_by_its_actual_spatial_weights(self):
        before=fixture();after=fixture();after[0]['points'][1]=[3,-2,.1];after[0]['sections'][1]='left wing'
        actual=build(after);expected=expected_skin(after)
        self.assertEqual({s['name']:s['skin_kg'] for s in actual['sections']},expected)
        old={s['name']:s['skin_kg'] for s in build(before)['sections']}
        self.assertEqual(old['right wing']-expected['right wing'],.35*75)
        self.assertEqual(expected['left wing']-old['left wing'],.35*75)
        self.assertEqual(sum(old.values()),sum(expected.values()))

    def test_same_mass_particles_and_flight_structure_differ_only_in_skin_type_allocation(self):
        items=fixture()+[equipment(kind) for kind in ('engine','systems','gear','item')]
        typed=build(items);all_structure=copy.deepcopy(items)
        for row in all_structure:row['kind']='structure'
        relabeled=build(all_structure)
        self.assertEqual(strip_skin(typed),strip_skin(relabeled))
        self.assertNotEqual([s['skin_kg'] for s in typed['sections']],[s['skin_kg'] for s in relabeled['sections']])

    def test_serialized_all_retained_complements_preserve_full_physical_moments(self):
        items=fixture()+[equipment(kind) for kind in ('engine','systems','gear','item')]
        result=build(items,serialized=True)
        self.assertEqual({s['name']:s['skin_kg'] for s in result['sections']},expected_skin(items))
        for size in range(1,len(SECTIONS)+1):
            for names in itertools.combinations(SECTIONS,size):
                with self.subTest(retained=names):
                    actual=serialized_moments([s for s in result['sections'] if s['name'] in names])
                    expected=physical_moments(items,names)
                    self.assertAlmostEqual(actual[0],expected[0],places=11)
                    np.testing.assert_allclose(actual[1],expected[1],rtol=1e-13,atol=1e-10)
                    np.testing.assert_allclose(actual[2],expected[2],rtol=1e-13,atol=1e-10)
                    self.assertGreaterEqual(float(np.linalg.eigvalsh(actual[2]).min()),-1e-10)


if __name__=='__main__':unittest.main(verbosity=2)
