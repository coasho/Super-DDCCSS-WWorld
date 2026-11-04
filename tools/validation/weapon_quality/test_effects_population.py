"""Negative admission controls, not native simulation or equipment evidence."""
import os
from pathlib import Path
import tempfile
import unittest

import effects_population as ep

TMP_PARENT='D:/ENV/tmp' if os.name=='nt' else None


def particle(segment=False):
    values=[1,2,1e-5,1.3,int(segment),0,0,0,1100,20,-30,0,0,0,1100,20,-30,0,0]
    return dict(zip(ep.SOURCE_FIELDS,map(str,values)))


def definition():
    return dict(card='fixture',selection='equipment.fixture.warhead',mechanism='native.warhead.zoned_case.v1',
        version=0,case_kg=2.,rod=False,count=1,speed=1000.,diameter=2.,length=2.,arming_s=1.,tnt_kg=1.,cd=1.3)


def population_row():
    ref=ep.source_reference([particle()]);budget=512*ep.sys.float_info.epsilon
    values=['fixture','equipment.fixture.warhead','native.warhead.zoned_case.v1',0,0,0,1,2,2,budget*2,
        ref['energy'],ref['energy'],budget*ref['energy'],0,budget*ref['momentum_scale'],2,1,1,2,1,.001,1,1]
    return dict(zip(ep.POPULATION_FIELDS,map(str,values)))


def terminal_rows():
    result=[]
    for skip in (1,4):
        for armed in (0,1):
            values=['cj-10a',skip,armed,1-armed,1,1.25,armed,armed,armed,2*armed,2,3 if armed else 1,4]
            result.append(dict(zip(ep.TERMINAL_FIELDS,map(str,values))))
    return result


class ExactEffectsAdmission(unittest.TestCase):
    def check_model(self,row=None,method=None,native=None,expected=None):
        return ep.validate_model(row or population_row(),expected or definition(),
            method if method is not None else [particle()],native if native is not None else [particle()])

    def test_independent_uniform_line_energy_includes_internal_motion(self):
        line=particle(True);line['evx']='1300'
        actual=ep.source_reference([line]);mean=(1100+1300)/2-17
        expected=.5*2*(mean*mean+28*28+33*33)+2*200*200/24
        self.assertEqual(actual['energy'],expected)
        self.assertEqual(actual['momentum'],[2*mean,56,-66])

    def test_exact_method_and_actual_native_component_evidence_passes(self):
        result=self.check_model();self.assertTrue(result['passed']);self.assertEqual(result['native_fragment_hits'],1)

    def test_raw_source_mass_energy_speed_and_topology_are_independently_required(self):
        for key,value in (('mass_kg','3'),('vx','1101'),('x','9'),('segment','1'),
                          ('ordinal','2'),('area_m2','0'),('cd','nan')):
            source=particle();source[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.check_model(method=[source])
        for values in ([],[particle(),particle()]):
            with self.assertRaises(ValueError):self.check_model(method=values)

    def test_blast_only_native_loss_cannot_satisfy_component_execution(self):
        for key,value in (('native_fragment_hits','0'),('native_fuel_hole_m2','0'),('native_finite','0'),
                          ('native_scene_calls','0'),('native_scene_calls','2'),('native_emitted','2'),
                          ('native_emitted_kg','4'),('absorbed_energy_j','0'),('absorbed_kg','0'),
                          ('momentum_error_ns','1'),('energy_tolerance_j','1e99')):
            row=population_row();row[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):self.check_model(row=row)

    def test_selected_identity_and_actual_native_material_cannot_be_replaced(self):
        for key in ('card','selection','mechanism','version'):
            row=population_row();row[key]='1' if key=='version' else 'other'
            with self.subTest(key=key),self.assertRaises(ValueError):self.check_model(row=row)
        for values in ([],[particle(),particle()]):
            with self.assertRaises(ValueError):self.check_model(native=values)
        native=particle();native['mass_kg']='4'
        with self.assertRaises(ValueError):self.check_model(native=[native])

    def test_cruise_requires_both_cadences_actual_armed_initiation_and_inert_control(self):
        expected=definition();expected['card']='cj-10a'
        self.assertEqual(len(ep.validate_terminal(terminal_rows(),expected)),4)
        for rows in (terminal_rows()[:-1],terminal_rows()+[terminal_rows()[0]],
                     terminal_rows()[:-1]+[terminal_rows()[0]]):
            with self.assertRaises(ValueError):ep.validate_terminal(rows,expected)
        for index,key,value in ((1,'detonations','0'),(1,'terminal','0'),(0,'detonations','1'),
                                (0,'emitted','1'),(1,'scene_calls','0'),(1,'emitted','2'),
                                (1,'impact_age_s','.5'),(1,'arming_s','0'),(1,'finite_worlds','0')):
            rows=terminal_rows();rows[index][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):ep.validate_terminal(rows,expected)

    def test_raw_schema_does_not_admit_truncated_or_extra_fields(self):
        with tempfile.TemporaryDirectory(dir=TMP_PARENT) as folder:
            path=Path(folder)/'raw.csv'
            for text in ('a,b\n1\n','a,b\n1,2,3\n','a,b\n'):
                path.write_text(text,encoding='utf-8')
                with self.assertRaises(ValueError):ep.read_csv(path,('a','b'))

    def test_actual_test_receipt_cannot_hide_a_failed_skipped_empty_or_partial_run(self):
        with tempfile.TemporaryDirectory(dir=TMP_PARENT) as folder:
            path=Path(folder)/'junit.xml'
            prefix='<testsuites><testcase name="weapon_effects_population">'
            success='<system-out>All tests passed (20 assertions in 2 test cases)</system-out>'
            suffix='</testcase></testsuites>'
            path.write_text(prefix+success+suffix,encoding='utf-8');self.assertEqual(ep.actual_test_report(path),'ctest-junit')
            for body in (success+'<failure/>',success+'<skipped/>',success.replace('20 assertions','0 assertions'),
                         success.replace('2 test cases','1 test case'),'<system-out/>'):
                path.write_text(prefix+body+suffix,encoding='utf-8')
                with self.assertRaises(ValueError):ep.actual_test_report(path)
            for wrong in ('other',''):
                path.write_text((prefix+success+suffix).replace('weapon_effects_population',wrong),encoding='utf-8')
                with self.assertRaises(ValueError):ep.actual_test_report(path)

    def test_artifact_changed_after_capture_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=TMP_PARENT) as folder:
            path=Path(folder)/'source.csv';path.write_text('original',encoding='utf-8');binding=ep.bind(path)
            self.assertEqual(ep.bound(binding),path.resolve());path.write_text('changed',encoding='utf-8')
            with self.assertRaises(ValueError):ep.bound(binding)

    def test_actual_process_module_path_cannot_be_replaced_by_staged_file_presence(self):
        with tempfile.TemporaryDirectory(dir=TMP_PARENT) as folder:
            root=Path(folder);binary=root/'fsim_weapon_effects_population_test.exe';binary.write_bytes(b'native')
            module=root/'libJSBSim.dll';module.write_bytes(b'loaded')
            observed=[dict(role='binary',path=str(binary)),dict(role='jsbsim',path=str(module))]
            runtime=[ep.bind(module)]
            self.assertIn('statically linked',ep.validate_runtime(observed,binary,runtime)['sdk'])
            for rows in (observed[:-1],observed+[observed[0]],
                         [dict(role='binary',path=str(module)),observed[1]],
                         [observed[0],dict(role='jsbsim',path=str(root/'libfsim.dll'))]):
                with self.assertRaises(ValueError):ep.validate_runtime(rows,binary,runtime)
            other=root/'other';other.mkdir();staged=other/'libJSBSim.dll';staged.write_bytes(b'loaded')
            with self.assertRaises(ValueError):ep.validate_runtime(observed,binary,[ep.bind(staged)])


if __name__=='__main__':unittest.main(verbosity=2)
