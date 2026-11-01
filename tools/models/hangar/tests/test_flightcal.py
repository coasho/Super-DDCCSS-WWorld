"""A clone must retain admitted flight metadata without temporary reports."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from hangar import flightcal,profile
from hangar.rotorcraft import performance
from tests.paths import TEMP_ROOT


class FlightCalibration(unittest.TestCase):
    def test_native_sampler_retains_speed_on_each_replacement_setpoint(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        vehicle=Mock(state=SimpleNamespace(altitude_msl_m=100.))
        update=performance._native_level_hold(vehicle,14.)
        vehicle.state.altitude_msl_m=101.
        update()
        fields=vehicle.submit.return_value.update.call_args.kwargs
        self.assertEqual(fields['airspeed_ms'],14.)
        self.assertEqual(fields['heading_rad'],0.)
        self.assertAlmostEqual(fields['vertical_speed_ms'],-.4)

    def test_production_rotor_controller_is_a_sampling_dependency(self):
        flightcal.methods_hash.cache_clear()
        before=flightcal.methods_hash('rotorcraft')
        fixed=flightcal.methods_hash('fixed-wing')
        target=Path(flightcal.pipeline.repo_root())/'src/simulation/flight_control/Laws.cpp'
        original=Path.read_bytes
        def changed(path):
            data=original(path)
            return data+b'\n// Changed production rotor control\n' if path.resolve()==target.resolve() else data
        try:
            with patch.object(Path,'read_bytes',changed):
                flightcal.methods_hash.cache_clear()
                self.assertNotEqual(before,flightcal.methods_hash('rotorcraft'))
                self.assertEqual(fixed,flightcal.methods_hash('fixed-wing'))
        finally:flightcal.methods_hash.cache_clear()

    def test_rotor_receipt_depends_on_native_rotor_patch(self):
        flightcal.methods_hash.cache_clear()
        before=flightcal.methods_hash('rotorcraft')
        target=Path(flightcal.pipeline.repo_root())/'cmake/JsbsimPatches.cmake'
        original=Path.read_bytes
        def changed(path):
            data=original(path)
            return data.replace(b'# classical rotor forces.',b'# classical rotor forces. Changed runtime equation.') if path.resolve()==target.resolve() else data
        try:
            with patch.object(Path,'read_bytes',changed):
                flightcal.methods_hash.cache_clear()
                self.assertNotEqual(before,flightcal.methods_hash('rotorcraft'))
        finally:
            flightcal.methods_hash.cache_clear()

    def test_fixed_wing_receipt_does_not_depend_on_unused_rotor_sampler(self):
        flightcal.methods_hash.cache_clear()
        before=flightcal.methods_hash('fixed-wing')
        rotor_before=flightcal.methods_hash('rotorcraft')
        target=Path(performance.__file__).resolve()
        original=Path.read_bytes
        def changed(path):
            data=original(path)
            return data+b'\n# changed rotor sampling procedure\n' if path.resolve()==target else data
        try:
            with patch.object(Path,'read_bytes',changed):
                flightcal.methods_hash.cache_clear()
                self.assertEqual(before,flightcal.methods_hash('fixed-wing'))
                self.assertNotEqual(rotor_before,flightcal.methods_hash('rotorcraft'))
        finally:flightcal.methods_hash.cache_clear()

    def test_rotor_top_speed_records_achieved_speed_instead_of_request(self):
        runs=performance._Runs.__new__(performance._Runs)
        runs.run=lambda height,speed,fraction:dict(tas_ms=.98*speed)
        runs.within=lambda observation,speed:speed<=10.
        minimum,maximum=runs.bracket(100.,1.)
        self.assertEqual(minimum,0.)
        self.assertAlmostEqual(maximum,9.8)

    def test_rotor_transient_overspeed_does_not_extend_tested_bound(self):
        runs=performance._Runs.__new__(performance._Runs)
        runs.run=lambda height,speed,fraction:dict(tas_ms=1.02*speed)
        runs.within=lambda observation,speed:speed<=10.
        self.assertAlmostEqual(runs.bracket(100.,1.)[1],10.)

    def test_export_survives_missing_reports_and_rejects_changed_source(self):
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            root=Path(folder)/'sample';(root/'out').mkdir(parents=True)
            design=root/'sample.toml';design.write_text('[aircraft]\nname="sample"\n',encoding='utf-8')
            (root/'calibration.toml').write_text('[calibration]\n',encoding='utf-8')
            flown={'results':{'design':{'max_speed_ms':62.}}}
            tables={'altitude_m':[100.], 'weight_kg':[900.], 'max_tas_ms':[[62.]]}
            (root/'out/fly.json').write_text(json.dumps(flown),encoding='utf-8')
            (root/'out/performance.json').write_text(json.dumps({'tables':tables}),encoding='utf-8')
            (root/'out/ground.json').write_text(json.dumps({'turn_radius_m':6.71,'yaw_accel_rad_s2':.69}),encoding='utf-8')
            flightcal.export(root)
            (root/'out/fly.json').unlink();(root/'out/performance.json').unlink()
            self.assertEqual(profile.fly_results(str(root)),flown['results']['design'])
            self.assertEqual(profile.performance_tables(str(root)),tables)
            self.assertEqual(flightcal.read(root)['ground'],{'turn_radius_m':6.71,'yaw_accel_rad_s2':.69})
            design.write_text('[aircraft]\nname="changed"\n',encoding='utf-8')
            with self.assertWarnsRegex(RuntimeWarning,'stale'):
                self.assertEqual(profile.performance_tables(str(root)),{})

    def test_rotor_top_speed_honors_authored_calibrated_airspeed_limit(self):
        runs=performance._Runs.__new__(performance._Runs)
        runs.heli=True;runs.rpm=300.;runs.tilt=.35;runs.cas_max_kt=100.
        sample=dict(vz_ms=0.,tas_ms=50.,pitch_deg=0.,power=.8,rotor_rpm=300.,calibrated_speed_kt=99.)
        self.assertTrue(runs.within(sample,50.))
        sample['calibrated_speed_kt']=101.
        self.assertFalse(runs.within(sample,50.))
