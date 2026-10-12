"""A3 control and Mission binding contracts before moving native handlers."""
import array
import pickle
import types
import unittest

import fsim
import fsim._native as native
import fsim.world as world_module


CONTROL_METHODS = (
    "command_batch", "submit", "submit_batch", "submit_behavior", "submit_support",
    "submit_mode", "submit_route", "submit_curve", "submit_nurbs", "command_task",
    "active_level", "use_controller", "controller_parameter", "last_findings",
)
MISSION_METHODS = (
    "payloads", "ms_capabilities", "ms_capability_status", "payload_state",
    "payload_bit", "ms_cancel", "ms_activity", "ms_activities", "payload_fail",
)
MODULE_METHODS = (
    "command_field_count", "command_field_count_full", "reason_name",
    "ms_reason_name", "ms_reason_description", "payload_family_name",
    "payload_state_name", "ms_capability_type_id",
)
PUBLIC_RECORDS = {
    "PayloadInfo": "index name card designation summary family state count source",
    "MissionCapability": "index payload type id support stage",
    "MissionActivity": "id capability payload source controller command_id state reason start_time end_time percent",
}


class DomainBindingCompatibility(unittest.TestCase):
    def setUp(self):
        self.world = fsim.World("domain-binding-compatibility", publish=False, workers=1,
                                pin_workers=False, terrain=False)
        self.addCleanup(self.world.close)

    def viper(self):
        return self.world.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0,
                                         altitude_msl_m=3000.0, airspeed_ms=160.0)

    def test_descriptors_and_module_functions_keep_their_owners(self):
        self.assertIs(fsim.World, world_module.World)
        self.assertEqual((native.World.__module__, native.World.__qualname__),
                         ("fsim._native", "World"))
        for name in CONTROL_METHODS + MISSION_METHODS:
            with self.subTest(method=name):
                descriptor = vars(native.World)[name]
                self.assertIsInstance(descriptor, types.MethodDescriptorType)
                self.assertIs(descriptor.__objclass__, native.World)
                self.assertEqual((descriptor.__name__, descriptor.__qualname__),
                                 (name, "World." + name))
                self.assertIs(getattr(self.world._h, name).__self__, self.world._h)
                with self.assertRaisesRegex(TypeError, "takes no keyword arguments"):
                    getattr(self.world._h, name)(unused=0)
        for name in MODULE_METHODS:
            with self.subTest(module_function=name):
                function = getattr(native, name)
                self.assertIsInstance(function, types.BuiltinFunctionType)
                self.assertEqual((function.__module__, function.__name__), ("fsim._native", name))
                with self.assertRaisesRegex(TypeError, "takes no keyword arguments"):
                    function(unused=0)

    def test_arity_and_conversion_errors_remain_native(self):
        cases = (
            ("command_batch", (), TypeError, "command_batch() takes 3 argument(s) (0 given)"),
            ("payload_state", (), TypeError, "payload_state() takes 3 to 6 arguments (0 given)"),
            ("payloads", ("bad",), TypeError, "an integer is required"),
            ("ms_capability_status", (1, 2), TypeError, "capability id must be a string"),
        )
        for name, args, error, message in cases:
            with self.subTest(method=name, args=args):
                with self.assertRaises(error) as raised:
                    getattr(self.world._h, name)(*args)
                self.assertEqual(str(raised.exception), message)

    def test_native_failure_keeps_the_shared_error_type(self):
        self.assertIs(fsim.Error, native.Error)
        with self.assertRaises(fsim.Error) as raised:
            self.world._h.payload_state(0, 0, 99)
        self.assertIs(type(raised.exception), fsim.Error)
        self.assertIn("fsim_vehicle_payload_state", str(raised.exception))
        restored = pickle.loads(pickle.dumps(raised.exception))
        self.assertIs(type(restored), fsim.Error)
        self.assertEqual(str(restored), str(raised.exception))

    def test_payload_tuple_placeholder_and_public_records(self):
        vehicle = self.viper()
        native_payloads = self.world._h.payloads(vehicle.id)
        self.assertEqual(len(native_payloads), 6)
        self.assertEqual(len(native_payloads[0]), 10)
        self.assertEqual(native_payloads[0][5], "")
        self.assertEqual(native_payloads[0][1], "radar")
        public_payload = vehicle.payload("radar")
        self.assertIs(type(public_payload), fsim.PayloadInfo)
        self.assertEqual(public_payload.family, "air_radar")
        self.assertEqual(public_payload.state, fsim.PayloadState.OFF)
        self.assertEqual(public_payload.source, native_payloads[0][9])
        for name, fields in PUBLIC_RECORDS.items():
            with self.subTest(record=name):
                record = getattr(fsim, name)
                self.assertIs(record, getattr(world_module, name))
                self.assertEqual((record.__module__, record.__qualname__), ("fsim.world", name))
                self.assertEqual(record._fields, tuple(fields.split()))
                value = record(*range(len(record._fields)))
                for protocol in (0, pickle.HIGHEST_PROTOCOL):
                    restored = pickle.loads(pickle.dumps(value, protocol=protocol))
                    self.assertIs(type(restored), record)
                    self.assertEqual(restored, value)

    def test_control_and_mission_results_share_one_world(self):
        vehicle = self.viper()
        values = (0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0)
        control = self.world._h.submit(vehicle.id, 0, values)
        self.assertIsInstance(control, tuple)
        self.assertGreaterEqual(len(control), 6)
        findings = self.world._h.last_findings()
        mission = self.world._h.payload_state(vehicle.id, 0, int(fsim.PayloadState.STANDBY), 0, 0, 1234)
        self.assertEqual(len(mission), 4)
        self.assertEqual(mission[3], 1234)
        self.assertEqual(self.world._h.last_findings(), findings)
        self.assertEqual(self.world._h.ms_activities(vehicle.id)[0][5], 1234)

    def test_command_batch_checks_dtype_shape_and_releases_buffers(self):
        vehicle = self.viper()
        ids = array.array("I", [vehicle.id])
        values = array.array("d", [0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0])
        self.world._h.command_batch(0, ids, values)
        ids.append(vehicle.id)  # the successful native call released both views
        values.append(0.0)
        with self.assertRaises((TypeError, ValueError, BufferError)):
            self.world._h.command_batch(0, array.array("H", [vehicle.id]), values)
        with self.assertRaises((TypeError, ValueError, BufferError)):
            self.world._h.command_batch(0, ids, memoryview(values)[::2])
        with self.assertRaisesRegex(ValueError, "need"):
            self.world._h.command_batch(0, ids, values)
        ids.append(vehicle.id)  # early error paths released the first view too
        values.append(0.0)


if __name__ == "__main__":
    unittest.main()
