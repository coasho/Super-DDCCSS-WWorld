"""Conservation, generated native algebra and launch-mass checks for the cycle."""
import copy
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from hangar import components,ductedrocket,storebodies
from tests.test_storecontrol import Expressions

ROOT=Path(__file__).resolve().parents[4]
VALUES=dict(capture_area_m2=.0119,fuel_flow_max_kg_s=.3,heating_value_j_kg=40e6,
    combustion_efficiency=.9,pressure_recovery=.85,air_cp_j_kg_k=1004.5,gas_cp_j_kg_k=1150.,
    gas_gamma=1.33,fuel_air_ratio_max=.08,lightoff_mach=1.5,target_mach=3.5,
    valve_minimum=.15,speed_gain=.5,valve_lag_s=.1,valve_rate_s_1=10.)


def fixture():
    card=json.loads((ROOT/'assets/stores/meteor.json').read_text(encoding='utf-8'))
    row=card['component_models']['components']['propulsion'];row['implementation_id']=ductedrocket.IMPLEMENTATION
    for key,unit in ductedrocket.ROLES.items():
        ref='ducted_'+key;row['parameter_refs'][key]=ref
        card['parameters'][ref]=dict(value=VALUES[key],unit=unit,basis='assumed',uncertainty=abs(VALUES[key])*.5,
            note='Synthetic cycle fixture; not measured Meteor performance.')
    return card


class NativeExpressions(Expressions):
    def eval(self,node):
        if node.tag=='ifthen':return self.eval(node[1] if self.eval(node[0]) else node[2])
        if node.tag=='and':return float(all(self.eval(c) for c in node))
        if node.tag in ('ge','gt'):
            a,b=(self.eval(c) for c in node);return float(a>=b if node.tag=='ge' else a>b)
        if node.tag=='pow':return self.eval(node[0])**self.eval(node[1])
        if node.tag=='sqrt':return math.sqrt(self.eval(node[0]))
        if node.tag=='min':return min(self.eval(c) for c in node)
        return super().eval(node)


class DuctedRocket(unittest.TestCase):
    def test_negative_cycle_wind_loads_reconstruct_only_axial_body_force(self):
        model=ET.fromstring(storebodies.files(fixture())['meteor.xml'])
        for alpha in (-2.4,-.5,0.,.5,2.4):
            for beta in (-1.1,0.,.7):
                for net in (-100.,0.,100.):
                    with self.subTest(alpha=alpha,beta=beta,net=net):
                        evaluator=NativeExpressions(model,{
                            'aero/alpha-rad':alpha,'aero/beta-rad':beta,
                            'fsim/ducted-rocket/engine[1]/net-thrust-n':net})
                        drag,side,lift=(evaluator.property('fsim/ducted-rocket/ram-drag-'+axis)
                                        *ductedrocket.LBF for axis in ('drag','side','lift'))
                        ca,sa,cb,sb=math.cos(alpha),math.sin(alpha),math.cos(beta),math.sin(beta)
                        # FGAuxiliary::UpdateBodyMatrices, followed by
                        # FGAerodynamics::Run's negative DRAG/LIFT convention.
                        actual=(-ca*cb*drag-ca*sb*side+sa*lift,
                                -sb*drag+cb*side,
                                -sa*cb*drag-sa*sb*side-ca*lift)
                        for value,expected in zip(actual,(min(net,0.),0.,0.)):
                            self.assertAlmostEqual(value,expected,delta=1e-11)

    def test_split_grain_preserves_native_launch_mass_cg_and_inertia(self):
        card=fixture();legacy=copy.deepcopy(card)
        row=legacy['component_models']['components']['propulsion'];row['implementation_id']=components.PROPULSION
        for key in ductedrocket.ROLES:del row['parameter_refs'][key]
        def ledger(xml):
            model=ET.fromstring(xml);balance=model.find('mass_balance')
            empty=float(balance.findtext('emptywt'))*.45359237
            cg=float(balance.findtext("location[@name='CG']/x"))*.0254
            tanks=[]
            for tank in model.findall('propulsion/tank'):
                tanks.append(tuple(float(tank.findtext(path))*factor for path,factor in (
                    ('contents',.45359237),('location/x',.0254),('radius',.0254),
                    ('grain_config/bore_diameter',.0127),('grain_config/length',.0254))))
            mass=empty+sum(t[0] for t in tanks)
            center=(empty*cg+sum(m*x for m,x,*_ in tanks))/mass
            ixx=float(balance.findtext('ixx'))*1.3558179483314
            iyy=float(balance.findtext('iyy'))*1.3558179483314+empty*(cg-center)**2
            for m,x,r,b,l in tanks:
                ixx+=.5*m*(r*r+b*b)
                iyy+=m*((3*(r*r+b*b)+l*l)/12+(x-center)**2)
            return mass,center,ixx,iyy
        from unittest import mock
        from hangar import storeaero
        old_geometry=copy.deepcopy(storeaero.geometry(legacy))
        old_geometry['design']['motor'].pop('kind',None)
        with mock.patch.object(storeaero,'geometry',return_value=old_geometry):
            before=ledger(storebodies.files(legacy)['meteor.xml'])
        after=ledger(storebodies.files(card)['meteor.xml'])
        for a,b,tolerance in zip(before,after,(1e-5,1e-5,.002,.002)):
            self.assertAlmostEqual(a,b,delta=tolerance)

    def test_cycle_requires_air_and_fuel_and_has_finite_oxygen(self):
        for density,mach,fuel in ((0.,2.,.3),(.65,1.,.3),(.65,2.,0.)):
            result=ductedrocket.cycle(VALUES,density=density,speed=600.,temperature=250.,mach=mach,fuel_flow=fuel)
            self.assertEqual(result['net_thrust_n'],0.)
        rich=ductedrocket.cycle(VALUES,density=.01,speed=600.,temperature=250.,mach=2.,fuel_flow=.3)
        self.assertAlmostEqual(rich['burned_fuel_kg_s'],rich['air_kg_s']*VALUES['fuel_air_ratio_max'])

    def test_cycle_preserves_first_law_and_subtracts_ram_momentum(self):
        for density,speed,temperature,mach in ((1.225,680.,288.15,2.),(.65,950.,249.,3.),(.2,1200.,220.,4.)):
            for fuel in (.05,.15,.3):
                r=ductedrocket.cycle(VALUES,density=density,speed=speed,temperature=temperature,mach=mach,fuel_flow=fuel)
                air=density*speed*VALUES['capture_area_m2']
                energy=air*VALUES['air_cp_j_kg_k']*temperature*(1+.2*mach**2)+r['burned_fuel_kg_s']*VALUES['combustion_efficiency']*VALUES['heating_value_j_kg']
                self.assertAlmostEqual((air+fuel)*VALUES['gas_cp_j_kg_k']*r['total_temperature_k']/energy,1.,places=12)
                self.assertAlmostEqual(r['net_thrust_n'],(air+fuel)*r['exit_speed_m_s']-air*speed,places=10)
        low=ductedrocket.cycle(VALUES,density=.65,speed=950.,temperature=249.,mach=3.,fuel_flow=.1)
        high=ductedrocket.cycle(VALUES,density=.65,speed=950.,temperature=249.,mach=3.,fuel_flow=.3)
        self.assertGreater(high['net_thrust_n'],low['net_thrust_n'])

    def test_emitted_native_cycle_matches_si_conditions_without_double_pressure_subtraction(self):
        card=fixture();original=copy.deepcopy(card);products=storebodies.files(card)
        self.assertEqual(card,original,'pure generation must not publish or modify a card')
        xml=ET.fromstring(products['meteor.xml']);self.assertEqual(len(xml.findall('propulsion/engine')),2)
        engine=ET.fromstring(products['Engines/meteor_sustainer.xml'])
        for function in engine.findall('function'):
            self.assertIn('#',function.get('name'),'JSBSim numbered engine functions require index substitution')
            function.set('name',function.get('name').replace('#','1'))
        self.assertIsNone(engine.find('thrust_table'))
        self.assertEqual(float(ET.fromstring(products['Engines/meteor_sustainer_nozzle.xml']).findtext('area')),0.)
        self.assertEqual(sum(float(n.findtext('contents')) for n in xml.findall('propulsion/tank')),
                         sum(float(n.findtext('capacity')) for n in xml.findall('propulsion/tank')))
        for density,mach,speed,temp,fuel in ((.65,2.,630.,249.,.3),(.2,3.,900.,220.,.1),(0.,3.,900.,220.,.3)):
            evaluator=NativeExpressions(engine,{'velocities/mach':mach,'velocities/vt-fps':speed/.3048,
                'atmosphere/rho-slugs_ft3':density/515.3788183931961,'atmosphere/T-R':temp*9/5,
                'propulsion/engine[1]/fuel-flow-rate-pps':fuel/.45359237})
            expected=ductedrocket.cycle(VALUES,density=density,speed=speed,temperature=temp,mach=mach,fuel_flow=fuel)
            actual=evaluator.property('fsim/ducted-rocket/engine[1]/net-thrust-n')
            self.assertAlmostEqual(actual,expected['net_thrust_n'],places=8)
            isp=evaluator.eval(engine.find('isp/function'))
            self.assertAlmostEqual(isp*fuel*9.80665,max(0.,actual),places=8)
        code=next(n for n in xml.findall('flight_control/property') if n.text=='fsim/components/propulsion/implementation-code')
        self.assertEqual(code.get('value'),'2')

    def test_explicit_ducted_design_requires_its_declared_cycle(self):
        design={'motor':{'kind':'ducted_rocket'}}
        with self.assertRaises(ValueError):components.motor({},design)
        card=fixture();card['component_models']['components']['propulsion']['implementation_id']=components.PROPULSION
        with self.assertRaises(ValueError):components.motor(card,design)

    def test_missing_or_invalid_cycle_inputs_do_not_fall_back_to_solid(self):
        for key in ductedrocket.ROLES:
            card=fixture();del card['parameters']['ducted_'+key]
            with self.subTest(key=key),self.assertRaises(ValueError):components.motor(card,{'motor':{}})
        for key,value in (('pressure_recovery',1.1),('gas_gamma',1.),('lightoff_mach',.8),('fuel_flow_max_kg_s',0.)):
            card=fixture();card['parameters']['ducted_'+key]['value']=value
            with self.subTest(key=key),self.assertRaises(ValueError):ductedrocket.inputs(card)


if __name__=='__main__':unittest.main()
