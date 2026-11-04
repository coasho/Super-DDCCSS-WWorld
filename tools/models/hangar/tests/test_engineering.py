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

    @staticmethod
    def r77_actuator_gate(root):
        product=engineering.ROOT/'aircraft/stores/r77m'
        card=json.loads((engineering.ROOT/'assets/stores/r-77m.json').read_text(encoding='utf-8'))
        _,checks,_=engineering.structure(ET.tostring(root,encoding='unicode'),product,card,'store',True)
        return next(check['passed'] for check in checks if check['name']=='actuator-lag-rate-stops')

    @staticmethod
    def r77_native_xml():
        return ET.parse(engineering.ROOT/'aircraft/stores/r77m/r77m.xml').getroot()

    def test_actual_r77_native_serial_channel_lag_rate_and_stops_are_admitted(self):
        root=self.r77_native_xml()
        for axis in ('elevator','rudder','aileron'):
            lag=root.find(".//lag_filter[@name='fcs/%s-operating-lag']"%axis)
            actuator=root.find(".//actuator[@name='fcs/%s-actuator']"%axis)
            self.assertIsNotNone(lag)
            self.assertEqual(actuator.findtext('input'),lag.findtext('output'))
            self.assertIsNone(actuator.find('lag'),'The finite channel lag executes upstream; physical fins do not need another pole')
        self.assertTrue(self.r77_actuator_gate(root))

    def test_serial_native_lag_requires_each_connected_positive_resolved_channel(self):
        defects=('missing-filter','unrelated-output','unrelated-input','missing-pole','unknown-pole',
                 'zero-pole','negative-pole','nan-pole','infinite-pole','tiny-pole',
                 'unrelated-named-function','duplicate-pole-property','wrong-positive-pole')
        for defect in defects:
            with self.subTest(defect=defect):
                root=self.r77_native_xml();channel=root.find('flight_control/channel')
                lag=channel.find("lag_filter[@name='fcs/elevator-operating-lag']")
                pole=root.find("flight_control/property[.='fsim/missile-control/lag-coefficient-s-1']")
                if defect=='missing-filter':channel.remove(lag)
                elif defect=='unrelated-output':lag.find('output').text='fcs/unrelated-lag-state'
                elif defect=='unrelated-input':lag.find('input').text='fcs/rudder-control'
                elif defect=='missing-pole':lag.remove(lag.find('c1'))
                elif defect=='unknown-pole':lag.find('c1').text='fsim/missing-lag-pole'
                elif defect=='unrelated-named-function':
                    lag.find('c1').text='fsim/not-a-scalar-pole';ET.SubElement(root,'function',name='fsim/not-a-scalar-pole')
                elif defect=='duplicate-pole-property':root.find('flight_control').append(copy.deepcopy(pole))
                elif defect=='wrong-positive-pole':lag.find('c1').text='50'
                else:pole.set('value',{'zero-pole':'0','negative-pole':'-1','nan-pole':'nan','infinite-pole':'inf','tiny-pole':'5e-324'}[defect])
                self.assertFalse(self.r77_actuator_gate(root))

    def test_serial_native_actuator_chain_rejects_broken_inputs_rates_stops_and_order(self):
        defects=('actuator-input','actuator-output','missing-rate','zero-rate','negative-rate','nan-rate','infinite-rate',
                 'unknown-rate','wrong-positive-rate','directional-rate','missing-min','missing-max','inverted-stops',
                 'nan-stop','infinite-stop','different-stop','scale-input','scale-range','actuator-before-lag',
                 'duplicate-filter','second-lag','missing-version','physical-fin-rate','physical-fin-stop')
        for defect in defects:
            with self.subTest(defect=defect):
                root=self.r77_native_xml();channel=root.find('flight_control/channel')
                actuator=channel.find("actuator[@name='fcs/elevator-actuator']")
                lag=channel.find("lag_filter[@name='fcs/elevator-operating-lag']")
                scale=channel.find("aerosurface_scale[@name='fcs/elevator-control']")
                rate=root.find("flight_control/property[.='fsim/missile-control/channel-rate-rad_s']")
                if defect=='actuator-input':actuator.find('input').text='fcs/elevator-control'
                elif defect=='actuator-output':actuator.find('output').text='fcs/unrelated-response'
                elif defect=='missing-rate':actuator.remove(actuator.find('rate_limit'))
                elif defect in ('zero-rate','negative-rate','nan-rate','infinite-rate'):
                    rate.set('value',{'zero-rate':'0','negative-rate':'-1','nan-rate':'nan','infinite-rate':'inf'}[defect])
                elif defect=='unknown-rate':actuator.find('rate_limit').text='fsim/missing-rate'
                elif defect=='wrong-positive-rate':actuator.find('rate_limit').text='8'
                elif defect=='directional-rate':actuator.find('rate_limit').set('sense','incr')
                elif defect in ('missing-min','missing-max'):actuator.find('clipto').remove(actuator.find('clipto/'+defect.removeprefix('missing-')))
                elif defect=='inverted-stops':actuator.find('clipto/min').text='1'
                elif defect=='nan-stop':actuator.find('clipto/min').text='nan'
                elif defect=='infinite-stop':actuator.find('clipto/max').text='inf'
                elif defect=='different-stop':actuator.find('clipto/max').text='.5'
                elif defect=='scale-input':scale.find('input').text='fcs/rudder-cmd-norm'
                elif defect=='scale-range':scale.find('range/max').text='.5'
                elif defect=='actuator-before-lag':channel.remove(actuator);channel.insert(list(channel).index(lag),actuator)
                elif defect=='duplicate-filter':channel.append(copy.deepcopy(lag))
                elif defect=='second-lag':ET.SubElement(actuator,'lag').text='25'
                elif defect=='missing-version':root.find('flight_control').remove(root.find("flight_control/property[.='fsim/control-operating-point-v1/version']"))
                elif defect=='physical-fin-rate':channel.find("actuator[@name='fcs/fin-0-actuator']/rate_limit").text='0'
                elif defect=='physical-fin-stop':channel.find("actuator[@name='fcs/fin-0-actuator']/clipto/max").text='nan'
                self.assertFalse(self.r77_actuator_gate(root))

    def test_version_one_serial_scale_accepts_only_implicit_or_explicit_default_normalization(self):
        for centered in ('true','1'):
            root=self.r77_native_xml()
            for scale in root.findall('.//aerosurface_scale'):
                domain=ET.SubElement(scale,'domain');ET.SubElement(domain,'min').text='-1';ET.SubElement(domain,'max').text='1'
                ET.SubElement(scale,'gain').text='1';ET.SubElement(scale,'zero_centered').text=centered
            with self.subTest(centered=centered):self.assertTrue(self.r77_actuator_gate(root))

    def test_version_one_serial_scale_refuses_normalization_overrides_and_ambiguous_fields(self):
        defects=('domain-wide','domain-missing-min','domain-nan','domain-inf','domain-property','domain-duplicate','domain-duplicate-min',
                 'gain-two','gain-zero','gain-negative','gain-nan','gain-inf','gain-empty','gain-property','gain-duplicate',
                 'centered-false','centered-zero','centered-garbage','centered-nan','centered-empty','centered-duplicate',
                 'range-duplicate','range-duplicate-min')
        for defect in defects:
            with self.subTest(defect=defect):
                root=self.r77_native_xml();scale=root.find(".//aerosurface_scale[@name='fcs/elevator-control']")
                if defect.startswith('domain-'):
                    domain=ET.SubElement(scale,'domain');ET.SubElement(domain,'min').text='-1';ET.SubElement(domain,'max').text='1'
                    if defect=='domain-wide':domain.find('min').text='-2';domain.find('max').text='2'
                    elif defect=='domain-missing-min':domain.remove(domain.find('min'))
                    elif defect=='domain-nan':domain.find('min').text='nan'
                    elif defect=='domain-inf':domain.find('max').text='inf'
                    elif defect=='domain-property':domain.find('min').text='fsim/missile-control/elevator-min-rad'
                    elif defect=='domain-duplicate':scale.append(copy.deepcopy(domain))
                    elif defect=='domain-duplicate-min':domain.append(copy.deepcopy(domain.find('min')))
                elif defect.startswith('gain-'):
                    text={'gain-two':'2','gain-zero':'0','gain-negative':'-1','gain-nan':'nan','gain-inf':'inf',
                          'gain-empty':'','gain-property':'fsim/unit-gain','gain-duplicate':'1'}[defect]
                    node=ET.SubElement(scale,'gain');node.text=text
                    if defect=='gain-property':ET.SubElement(root.find('flight_control'),'property',value='1').text='fsim/unit-gain'
                    if defect=='gain-duplicate':scale.append(copy.deepcopy(node))
                elif defect.startswith('centered-'):
                    text={'centered-false':'false','centered-zero':'0','centered-garbage':'maybe','centered-nan':'nan',
                          'centered-empty':'','centered-duplicate':'true'}[defect]
                    node=ET.SubElement(scale,'zero_centered');node.text=text
                    if defect=='centered-duplicate':scale.append(copy.deepcopy(node))
                elif defect=='range-duplicate':scale.append(copy.deepcopy(scale.find('range')))
                elif defect=='range-duplicate-min':scale.find('range').append(copy.deepcopy(scale.find('range/min')))
                self.assertFalse(self.r77_actuator_gate(root))

    def test_version_one_serial_components_and_mixer_match_the_predictor_shape(self):
        defects=('bias','deadband','hysteresis','delay','lag-c2','duplicate-scale-input','duplicate-lag-input','duplicate-c1',
                 'duplicate-actuator-input','duplicate-rate','duplicate-clip','duplicate-output','mixed-coefficient','mixed-axis',
                 'mixed-extra-value','mixed-offset','mixed-nonlinear','mixed-metadata','duplicate-mixer-function','mixed-clip')
        for defect in defects:
            with self.subTest(defect=defect):
                root=self.r77_native_xml();channel=root.find('flight_control/channel')
                scale=channel.find("aerosurface_scale[@name='fcs/elevator-control']")
                lag=channel.find("lag_filter[@name='fcs/elevator-operating-lag']")
                actuator=channel.find("actuator[@name='fcs/elevator-actuator']")
                mixer=channel.find("fcs_function[@name='fcs/fin-0-demand-rad']")
                if defect in ('bias','deadband','hysteresis','delay'):
                    tag={'bias':'bias','deadband':'deadband_width','hysteresis':'hysteresis_width','delay':'delay'}[defect]
                    ET.SubElement(actuator,tag).text='.01'
                elif defect=='lag-c2':ET.SubElement(lag,'c2').text='2'
                elif defect=='duplicate-scale-input':scale.append(copy.deepcopy(scale.find('input')))
                elif defect=='duplicate-lag-input':lag.append(copy.deepcopy(lag.find('input')))
                elif defect=='duplicate-c1':lag.append(copy.deepcopy(lag.find('c1')))
                elif defect=='duplicate-actuator-input':actuator.append(copy.deepcopy(actuator.find('input')))
                elif defect=='duplicate-rate':actuator.append(copy.deepcopy(actuator.find('rate_limit')))
                elif defect=='duplicate-clip':actuator.append(copy.deepcopy(actuator.find('clipto')))
                elif defect=='duplicate-output':actuator.append(copy.deepcopy(actuator.find('output')))
                elif defect=='mixed-coefficient':mixer.find('function/sum/product/value').text='2'
                elif defect=='mixed-axis':mixer.find('function/sum/product/property').text='fcs/rudder-response-rad'
                elif defect=='mixed-extra-value':ET.SubElement(mixer.find('function/sum/product'),'value').text='2'
                elif defect=='mixed-offset':ET.SubElement(mixer.find('function/sum'),'value').text='.1'
                elif defect=='mixed-nonlinear':mixer.find('function/sum').tag='product'
                elif defect=='mixed-metadata':root.find("flight_control/property[.='fsim/control-operating-point-v1/fin-0/mix-elevator']").set('value','2')
                elif defect=='duplicate-mixer-function':mixer.append(copy.deepcopy(mixer.find('function')))
                elif defect=='mixed-clip':mixer.append(copy.deepcopy(mixer.find('clipto')))
                self.assertFalse(self.r77_actuator_gate(root))

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
