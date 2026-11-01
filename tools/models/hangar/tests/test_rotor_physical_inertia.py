"""Physical drive energy must not be inferred from a handling-response fit."""
import copy
from pathlib import Path
import tomllib
import unittest
from hangar.rotorcraft.heli import rotor_data


class RotorPhysicalInertia(unittest.TestCase):
    def test_physical_inertia_is_independent_of_effective_handling_response(self):
        path = Path(__file__).resolve().parents[4] / 'aircraft/uh1h/uh1h.toml'
        with path.open('rb') as stream:
            spec = tomllib.load(stream)
        actual = rotor_data(spec)
        legacy_spec = copy.deepcopy(spec)
        del legacy_spec['rotor']['main']['physical_inertia']
        legacy = rotor_data(legacy_spec)
        self.assertEqual(actual['ib'], legacy['ib'])
        self.assertEqual(actual['flap_lag'], legacy['flap_lag'])
        self.assertAlmostEqual(actual['physical_ib'], .5 * legacy['ib'])
        reflected_tail = spec['rotor']['tail']['blades'] * actual['tr_ib'] * (actual['tr_omega'] / actual['omega'])**2
        self.assertAlmostEqual(actual['j'], 1.1 * spec['rotor']['main']['blades'] * actual['physical_ib'] + reflected_tail)
        self.assertEqual(legacy['ib'], legacy['physical_ib'])

    def test_changing_handling_time_does_not_change_declared_physical_energy(self):
        path = Path(__file__).resolve().parents[4] / 'aircraft/uh1h/uh1h.toml'
        with path.open('rb') as stream:
            spec = tomllib.load(stream)
        before = rotor_data(spec)
        spec['rotor']['main']['flap_lag_s'] *= 1.5
        after = rotor_data(spec)
        self.assertEqual(after['physical_ib'], before['physical_ib'])
        self.assertEqual(after['j'], before['j'])
        self.assertNotEqual(after['flap_lag'], before['flap_lag'])


if __name__ == '__main__':
    unittest.main()
