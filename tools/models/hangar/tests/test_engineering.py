"""Failures must survive the engineering gate; no equipment-name exemptions."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from hangar import airframe, engineering
from hangar.aero.tables import COEFFS, CHANNEL_COEFFS


def fixture():
    a=SimpleNamespace(spec={'analysis':{'max_mach':.7}},engines=[],b=2.,c=1.,channels=lambda:[])
    t=dict(alpha=np.array([-180.,0.,180.]),beta=np.array([-90.,0.,90.]),
        base={k:np.zeros((3,3)) for k in (*COEFFS,'CDi')},controls={},native_controls=[],
        rates={r:{k:np.zeros(3) for k in (*COEFFS,'CDi')} for r in ('p','q','r')},
        alphadot={k:0. for k in COEFFS},ground_effect={'h_b':[0.,1.1],'lift':[1.,1.],'drag':[1.,1.]},
        mach={'mach':[0.,.7],'K_L':[1.,1.],'K_Y':[1.,1.],'dCm_dCL':[0.,0.],
              'dCD0':[0.,0.],'induced_factor':[1.,1.],'K0':0.})
    return t,a


class EngineeringGate(unittest.TestCase):
    def test_declared_availability_has_one_applicability_and_check_truth_table(self):
        for availability,applicability,present,passed in [
                ('available','required',True,True),
                ('unavailable','required',False,False),
                ('not_applicable','not_applicable',False,True)]:
            with self.subTest(availability=availability):
                descriptor=dict(availability=availability,reason='Explicit project configuration declaration')
                state,check=engineering.declared_component_state(descriptor)
                self.assertEqual(state['applicability'],applicability)
                self.assertEqual(state['present'],present)
                self.assertEqual(check,passed)

    def test_missing_implementation_is_required_even_if_reason_mentions_inert_release(self):
        state,passed=engineering.declared_component_state(dict(availability='unavailable',
            reason='Armed effects are not implemented; inert release only.'))
        self.assertEqual(state['applicability'],'required')
        self.assertFalse(state['present'])
        self.assertFalse(passed)

    def test_all_authored_store_component_checks_match_applicability(self):
        import tomllib
        from hangar import components
        declarations=0
        for record in engineering.roster():
            if record['category']!='store':continue
            identity=tomllib.loads(Path(record['path']).read_text(encoding='utf-8'))['aircraft']
            card=json.loads((engineering.ROOT/'assets/stores'/(identity['card']+'.json')).read_text(encoding='utf-8'))
            product=engineering.ROOT/'aircraft/stores'/record['name']
            states,checks,_=engineering.structure((product/(record['name']+'.xml')).read_text(encoding='utf-8'),product,card,'store',bool(card.get('powerable')))
            by_name={row['name']:row['passed'] for row in checks}
            for kind in card['component_models']['components']:
                descriptor=components.resolve(card,kind)
                name='declared-'+kind if kind in ('autopilot','allocator','actuator','propulsion','aerodynamics') else kind
                with self.subTest(design=record['name'],component=kind):
                    state,passed=engineering.declared_component_state(descriptor)
                    self.assertEqual(states[name]['applicability'],state['applicability'])
                    self.assertEqual(states[name]['present'],state['present'])
                    self.assertEqual(by_name[name],passed)
                    declarations+=1
        self.assertGreater(declarations,0)

    def test_both_complete_statuses_are_acceptable_and_measurement_absence_is_not_blocking(self):
        self.assertEqual(engineering.engineering_status([]),'engineering-complete-uncalibrated')
        self.assertEqual(engineering.engineering_status([],calibration_complete=True),'engineering-complete')
        for calibrated in (False,True):
            self.assertEqual(engineering.engineering_status(['missing required component'],calibration_complete=calibrated),'blocked')
            self.assertEqual(engineering.engineering_status([],timed_out=True,calibration_complete=calibrated),'blocked')

    def test_deleting_project_role_declarations_cannot_create_an_inert_configuration(self):
        import tomllib
        record=next(r for r in engineering.roster() if r['category']=='store')
        identity=tomllib.loads(Path(record['path']).read_text(encoding='utf-8'))['aircraft']
        card=json.loads((engineering.ROOT/'assets/stores'/(identity['card']+'.json')).read_text(encoding='utf-8'))
        product=engineering.ROOT/'aircraft/stores'/record['name']
        for kind in ('seeker','guidance','fuze','warhead'):
            altered=copy.deepcopy(card)
            del altered['component_models']['components'][kind]
            states,checks,_=engineering.structure((product/(record['name']+'.xml')).read_text(encoding='utf-8'),product,altered,'store',bool(card.get('powerable')))
            self.assertEqual(states[kind]['applicability'],'required')
            self.assertFalse(states[kind]['present'])
            self.assertFalse(next(c['passed'] for c in checks if c['name']==kind))

    def test_roster_is_complete_and_disjoint(self):
        rows=engineering.roster()
        self.assertEqual(len(rows),88)
        self.assertEqual(len({r['name'] for r in rows}),88)

    def test_all_control_channels_have_six_coefficients(self):
        self.assertTrue(all(set(v)==set(COEFFS) for v in CHANNEL_COEFFS.values()))

    def test_generic_reference_suite_passes_valid_limiting_case(self):
        t,a=fixture();inventory,checks=engineering.table_audit(t,a)
        self.assertTrue(all(c['passed'] for c in checks))
        self.assertTrue(all(x['basis'] in ('simulated','estimated') and x['version'] for x in inventory))

    def test_nonfinite_cell_is_rejected(self):
        t,a=fixture();t['base']['Cn'][1,1]=np.nan
        _,checks=engineering.table_audit(t,a)
        self.assertFalse(next(c['passed'] for c in checks if c['name']=='base/Cn'))

    def test_missing_sixth_coefficient_is_rejected(self):
        t,a=fixture();del t['base']['Cn']
        with self.assertRaises(KeyError):engineering.table_audit(t,a)

    def test_missing_row_is_rejected(self):
        t,a=fixture();t['rates']['q']['Cm']=np.zeros(2)
        with self.assertRaises(ValueError):engineering.table_audit(t,a)

    def test_duplicate_axis_is_rejected(self):
        t,a=fixture();t['alpha'][1]=-180
        with self.assertRaisesRegex(ValueError,'grid'):engineering.table_audit(t,a)

    def test_uncovered_mach_endpoint_is_rejected(self):
        t,a=fixture();a.spec['analysis']['max_mach']=.8
        _,checks=engineering.table_audit(t,a)
        self.assertFalse(next(c['passed'] for c in checks if c['name']=='declared Mach domain'))

    def test_body_coordinate_transform_preserves_force_magnitude(self):
        t,a=fixture();t['base']['CD'][:]=.3;t['base']['CL'][:]=1.2;t['base']['CY'][:]=.2
        engineering.table_audit(t,a)
        body=t['body_base']
        np.testing.assert_allclose(body['CA']**2+body['CN']**2+body['CY']**2,.3**2+1.2**2+.2**2)

    def test_rotor_actuation_has_real_lag_rate_and_stops(self):
        fcs=ET.fromstring('<flight_control><channel><summer><input>fcs/elevator-cmd-norm</input></summer></channel></flight_control>')
        airframe._rotor_command_actuators(fcs)
        actuator=fcs.find('channel/actuator')
        self.assertEqual(actuator.findtext('lag'),'10')
        self.assertEqual(actuator.findtext('rate_limit'),'2')
        self.assertEqual(actuator.findtext('input'),'fcs/elevator-cmd-norm')
        self.assertEqual(fcs.find('channel[2]/summer/input').text,'fsim/engineering-actuator-v1/elevator-cmd-norm')

    def test_expired_deadline_never_starts_command(self):
        import subprocess
        with self.assertRaises(subprocess.TimeoutExpired):
            engineering.run_command(['cannot-run'],Path('unused'),0)

    def test_existing_native_response_does_not_receive_a_second_command_pole(self):
        fcs=ET.fromstring('<flight_control><channel><lag_filter name="motor-response"><input>fcs/elevator-cmd-norm</input><c1>5</c1></lag_filter></channel></flight_control>')
        airframe._rotor_command_actuators(fcs,additional_lag_s=0.)
        self.assertEqual(len(fcs.findall('.//lag_filter')),1)
        self.assertEqual(fcs.find('channel[2]/lag_filter/c1').text,'5')
        self.assertIsNone(fcs.find('channel/actuator/lag'))
        self.assertEqual(fcs.find("property[.='fsim/engineering-actuator-v1/additional-lag-s']").get('value'),'0')
        self.assertEqual(fcs.find('channel/actuator/rate_limit').text,'2')

    def test_writer_preserves_small_damping_instead_of_rounding_to_zero(self):
        from hangar import jsbsim
        xml=jsbsim._table1([-1.,1.],[1.e-9,-1.e-9],'aero/alpha-deg',0)
        values=[list(map(float,line.split())) for line in ET.fromstring(xml).findtext('tableData').splitlines() if line.strip()]
        self.assertEqual([row[1] for row in values],[1.e-9,-1.e-9])

    def test_wingless_tail_has_no_fictitious_downwash_delay(self):
        from hangar.aero.tables import alphadot
        tail=SimpleNamespace(kind='htail')
        model=SimpleNamespace(aircraft=SimpleNamespace(surfaces=[tail],wing=tail))
        self.assertEqual(alphadot(model)['Cm'],0.)

    def test_integration_resolution_tracks_inertia_without_a_name_rule(self):
        from hangar import integration
        t,a=fixture();a.S=1.
        a.spec['integration']={'version':1,'mode':'derived'}
        for rate,k in [('p','Cl'),('q','Cm'),('r','Cn')]:t['rates'][rate][k][:]=-1.
        root=ET.fromstring('<fdm_config><mass_balance><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></mass_balance></fdm_config>')
        small=integration.settings(root,a,t)
        for key in ('ixx','iyy','izz'):root.find('mass_balance/'+key).text='1000'
        large=integration.settings(root,a,t)
        self.assertGreater(small['substeps'],large['substeps'])
        self.assertEqual(large['substeps'],1)

    def test_integration_default_is_identity_and_options_are_versioned(self):
        from hangar import integration
        t,a=fixture();a.S=1.
        root=ET.fromstring('<fdm_config><mass_balance><ixx>1</ixx><iyy>1</iyy><izz>1</izz></mass_balance></fdm_config>')
        self.assertEqual(integration.settings(root,a,t)['substeps'],1)
        a.spec['integration']={'mode':'derived'}
        with self.assertRaisesRegex(ValueError,'version'):integration.settings(root,a,t)

    def test_invalid_inertia_cannot_get_integration_metadata(self):
        from hangar import integration
        t,a=fixture();a.S=1.
        root=ET.fromstring('<fdm_config><mass_balance><ixx>-1</ixx><iyy>1</iyy><izz>1</izz></mass_balance></fdm_config>')
        with self.assertRaisesRegex(ValueError,'inertia'):integration.settings(root,a,t)

    def test_version_two_resolution_includes_force_arm_and_gyro_without_equipment_rules(self):
        from hangar import integration
        t,a=fixture();a.S=1.;a.spec['integration']={'version':2,'mode':'derived'}
        t['base']['CY'][:]=np.radians(t['beta'])[None,:]
        root=ET.fromstring('<fdm_config><metrics><location name="AERORP" unit="M"><x>0</x><y>0</y><z>0</z></location></metrics><mass_balance><ixx>1</ixx><iyy>2</iyy><izz>2</izz><location name="CG" unit="M"><x>0</x><y>0</y><z>0</z></location></mass_balance></fdm_config>')
        centered=integration.settings(root,a,t)
        root.find("mass_balance/location/x").text='2'
        transferred=integration.settings(root,a,t)
        self.assertEqual(transferred['rotational_rate_method'],4)
        self.assertGreater(transferred['stiffness_bound'],centered['stiffness_bound'])
        self.assertGreater(transferred['reference_transfer']['gyro_rate_bound_per_s'],0)
        self.assertEqual(transferred['reference_transfer']['cg_hull_structural_m'],[[2.,0.,0.]])

    def test_ab3_reference_modal_semicircle_is_inside_absolute_stability_region(self):
        # Native recurrence: y[n+1]=y[n]+h*(23f[n]-16f[n-1]+5f[n-2])/12.
        # This checks the numerical method's existing0.25modal increment;
        # it does not assert nonlinear aircraft stability or alter flight gates.
        for angle in np.linspace(np.pi/2,3*np.pi/2,257):
            z=.25*np.exp(1j*angle)
            roots=np.roots([1.,-1.-23*z/12,16*z/12,-5*z/12])
            self.assertLessEqual(float(max(abs(roots))),1.+1.e-12)

    def test_inertia_reader_matches_both_native_crossproduct_frame_conventions(self):
        from hangar import integration
        root=ET.fromstring('<fdm_config><mass_balance><ixx unit="KG*M2">20</ixx><iyy unit="KG*M2">30</iyy><izz unit="KG*M2">40</izz><ixy unit="KG*M2">2</ixy><ixz unit="KG*M2">3</ixz><iyz unit="KG*M2">4</iyz></mass_balance></fdm_config>')
        np.testing.assert_array_equal(integration.inertia_matrix(root),[[20,-2,3],[-2,30,-4],[3,-4,40]])
        root.find('mass_balance').set('negated_crossproduct_inertia','false')
        np.testing.assert_array_equal(integration.inertia_matrix(root),[[20,2,-3],[2,30,4],[-3,4,40]])

    def test_version_two_cg_hull_includes_native_tank_drain_endpoint(self):
        from hangar import integration
        t,a=fixture();a.S=1.;a.spec['integration']={'version':2,'mode':'derived'}
        root=ET.fromstring('<fdm_config><metrics><location name="AERORP" unit="M"/></metrics><mass_balance><ixx>1</ixx><iyy>1</iyy><izz>1</izz><location name="CG" unit="M"/></mass_balance><propulsion><tank><location unit="M"><x>1</x></location><drain_location unit="M"><x>2</x></drain_location></tank></propulsion></fdm_config>')
        result=integration.settings(root,a,t)
        self.assertEqual(result['reference_transfer']['cg_hull_structural_m'],[[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])

    def test_numerical_version_rejects_boolean_and_noninteger_aliases(self):
        from hangar import integration
        t,a=fixture();a.S=1.
        root=ET.fromstring('<fdm_config><mass_balance><ixx>1</ixx><iyy>1</iyy><izz>1</izz></mass_balance></fdm_config>')
        for version in (True,1.,'2'):
            a.spec['integration']={'version':version,'mode':'identity'}
            with self.subTest(version=version),self.assertRaisesRegex(ValueError,'version'):integration.settings(root,a,t)


if __name__=='__main__':unittest.main()
