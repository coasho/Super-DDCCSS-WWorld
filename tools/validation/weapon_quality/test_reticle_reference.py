"""Independent analytic controls for sampled conical-scan reticle methods."""
import math
import unittest

try:
    import reticle_reference as reference
except ImportError:
    reference = None


class ReticleReferenceTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(reference, 'The numerical reticle reference must exist')

    def test_primary_stationary_reticle_carrier(self):
        # Birchenall et al., DOI10.1016/j.infrared.2010.07.001,
        # tracking-loop/pre-amplifier sections: 100Hz,12 periods =>1200Hz.
        self.assertEqual(reference.carrier_hz(100, 12), 1200)
        for phase in (.1, .4, 1.7, 3.2, 5.6):
            self.assertAlmostEqual(reference.angular_speed((0, 0), .024, phase, 100), 200*math.pi)

    def test_offset_frequency_exact_differential(self):
        # Independent centred finite difference of atan2, away from branch cuts.
        for point in ((.002, -.003), (-.004, .001), (.01, .002)):
            for phase in (.3, 1.1, 2.4):
                step = 1e-8
                def angle(t):
                    return math.atan2(point[1]+.024*math.sin(t), point[0]+.024*math.cos(t))
                numerical = (angle(phase+step)-angle(phase-step))/(2*step)*200*math.pi
                self.assertAlmostEqual(reference.angular_speed(point, .024, phase, 100), numerical, delta=2e-5)

    def test_mask_is_physical_and_phase_periodic(self):
        for point in ((.002, -.003), (-.004, .001), (.01, .002)):
            for phase in (.3, 1.1, 2.4):
                value = reference.mask(point, phase, .024, .048, 12)
                self.assertIn(value, (0., 1.))
                self.assertEqual(value, reference.mask(point, phase+2*math.pi, .024, .048, 12))
        self.assertEqual(reference.mask((1, 1), 0, .024, .048, 12), 0)

    def test_photon_energy_and_first_order_response(self):
        expected = 2e-12*1e-5*.6*4e-6/(6.62607015e-34*299792458)
        self.assertAlmostEqual(reference.electrons(2e-12, 1e-5, .6, 4e-6), expected, delta=1e-12)
        self.assertAlmostEqual(reference.response(0, 10, 1e-4, 1e-4), 10*(1-math.exp(-1)))
        self.assertEqual(reference.response(3, 3, .1, .01), 3)

    def test_inference_is_declared_and_not_reference_hardware(self):
        policy = reference.design(64, 64, .0015, .00075, .0001)
        self.assertEqual(policy['periods'], 12)
        self.assertAlmostEqual(policy['frequency_hz'], 132.62911924324612)
        self.assertNotEqual(policy['frequency_hz'], 100)
        self.assertEqual(policy['basis'], 'assumed')

    def test_scalar_equivalent_area_and_correlated_noise_sensitivity(self):
        independent = reference.scalar_electronics(64, 64, 1e6, 1, 12)
        correlated = reference.scalar_electronics(64, 64, 1e6, 1, 12, 1)
        self.assertEqual(independent['full_well'], 4096000000)
        self.assertEqual(independent['dark_current'], 4096)
        self.assertEqual(independent['read_noise'], 768)
        self.assertEqual(correlated['read_noise'], 49152)
        self.assertEqual(correlated['read_noise']**2/independent['read_noise']**2, 4096)

    def test_invalid_domains_refuse(self):
        for frequency, periods in ((0, 12), (100, 0), (float('nan'), 12)):
            with self.assertRaises(ValueError):
                reference.carrier_hz(frequency, periods)
        with self.assertRaises(ValueError):
            reference.angular_speed((-.024, 0), .024, 0, 100)

    def waveform(self):
        rows = []
        previous = previous_time = 0.
        for i in range(128):
            phase = 2*math.pi*(i+.5)/128
            incident = 1e8+1e7*math.cos(12*phase)
            time = .01+(i+.5)/12800
            previous = reference.response(previous, incident, time-previous_time, .0001)
            previous_time = time
            adc = 4096*1e6/65535
            rows.append(dict(phase_rad=phase, incident_electrons=incident, expected_electrons=previous,
                             measured_electrons=round(previous/adc)*adc))
        return rows

    def test_numeric_admission_accepts_independent_analytic_control(self):
        result = reference.compare_native_waveform(self.waveform())
        self.assertTrue(result['passed'])
        self.assertEqual(result['execution_admission'], 'separate_current_native_run_receipt_required')

    def test_numeric_admission_refuses_missing_wrong_and_nonphysical_data(self):
        with self.assertRaises(ValueError):
            reference.compare_native_waveform(self.waveform()[:-1])
        for key, value in (('phase_rad', -1), ('incident_electrons', -1), ('expected_electrons', 0),
                           ('measured_electrons', .1), ('incident_electrons', float('nan'))):
            rows = self.waveform()
            rows[17][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                reference.compare_native_waveform(rows)


if __name__ == '__main__':
    unittest.main()
