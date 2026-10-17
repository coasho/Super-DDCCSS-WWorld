"""A4 World, Scenario and Environment contracts before native extraction."""
import array
import gc
import json
import math
import struct
import types
import unittest

import fsim
import fsim._native as native
import fsim.world as world_module


WORLD_METHODS = (
    "step", "agent_audit", "agent_identity", "info", "time", "create_vehicle",
    "remove_vehicle", "focus_viewer", "reset_vehicle", "find_vehicle", "vehicle_ids",
    "vehicle_info", "gather_states", "get_property", "set_property", "comm_create_node",
    "comm_send", "comm_inbox", "comm_set_medium", "comm_attach_protocol",
    "comm_attach_udp_bridge", "_capsule", "borrowed",
)
ENVIRONMENT_METHODS = (
    "get_environment", "set_environment", "add_effect", "clear_effects", "terrain", "magnetic_year",
)
WORLD_DEFAULTS = {
    "name": "default", "dt": 1.0 / 120.0, "frame_skip": 4, "workers": 0,
    "pin_workers": 1, "seed": 0, "capacity": 256, "publish": 1,
    "publish_interval_s": 1.0 / 60.0, "jsbsim_root": None, "terrain": 0,
    "terrain_url": None, "terrain_zoom": 15, "record_path": None,
    "record_interval_s": 0.0, "terrain_offline": 1, "store_drag": 1,
}
ENVIRONMENT_DEFAULTS = {
    "epoch_utc_seconds": 0.0, "time_factor": 1.0, "temperature_sl_k": 288.15,
    "pressure_sl_pa": 101325.0, "humidity": 0.0, "wind_direction_deg": 0.0,
    "wind_speed_ms": 0.0, "wind_gust_ms": 0.0, "turbulence": 0.0,
    "visibility_m": 50000.0, "cloud_base_m": 2000.0, "cloud_cover": 0.0,
    "precipitation": 0.0,
}


class WorldBindingCompatibility(unittest.TestCase):
    def setUp(self):
        self.world = fsim.World("world-binding-compatibility", publish=False, workers=1,
                                pin_workers=False, terrain=False)
        self.addCleanup(self.world.close)

    def test_moved_descriptors_and_types_keep_identity_and_call_flags(self):
        self.assertIs(fsim.World, world_module.World)
        self.assertIs(fsim.Scenario, world_module.Scenario)
        for owner, names, size in ((native.World, WORLD_METHODS + ENVIRONMENT_METHODS, 9),
                                   (native.Scenario, ("world_options", "vehicle_count"), 3)):
            with self.subTest(owner=owner.__name__):
                self.assertEqual((owner.__module__, owner.__qualname__),
                                 ("fsim._native", owner.__name__))
                self.assertEqual(owner.__basicsize__, size * struct.calcsize("P"))
                self.assertEqual(owner.__itemsize__, 0)
                # Heap types, without subclassing, GC or managed instance attributes.
                mask = (1 << 4) | (1 << 9) | (1 << 10) | (1 << 14)
                self.assertEqual(owner.__flags__ & mask, 1 << 9)
                with self.assertRaisesRegex(TypeError, "not an acceptable base type"):
                    type("Derived", (owner,), {})
            instance = self.world._h if owner is native.World else native.Scenario(None, "{}", "descriptor")
            for name in names:
                with self.subTest(owner=owner.__name__, method=name):
                    descriptor = vars(owner)[name]
                    self.assertIsInstance(descriptor, types.MethodDescriptorType)
                    self.assertIs(descriptor.__objclass__, owner)
                    self.assertEqual((descriptor.__name__, descriptor.__qualname__),
                                     (name, owner.__name__ + "." + name))
                    self.assertIs(getattr(instance, name).__self__, instance)
                    with self.assertRaisesRegex(TypeError, "takes no keyword arguments"):
                        getattr(instance, name)(unused=0)
        for name in ("magnetic_field", "decimal_year"):
            function = getattr(native, name)
            self.assertIsInstance(function, types.BuiltinFunctionType)
            self.assertEqual((function.__module__, function.__name__), ("fsim._native", name))
            with self.assertRaisesRegex(TypeError, "takes no keyword arguments"):
                function(unused=0)

    def test_constructor_arity_and_field_conversion_errors(self):
        cases = (
            (native.World, ([],), TypeError, "World options must be a dict"),
            (self.world._h.create_vehicle, ({"unknown": 1},), TypeError, "unknown vehicle option 'unknown'"),
            (self.world._h.get_property, (1, 7), TypeError, "property path must be a string"),
            (self.world._h.set_environment, ({"unknown": 1},), TypeError, "unknown environment option 'unknown'"),
            (self.world._h.terrain, ([0.0], []), ValueError, "terrain: as many longitudes as latitudes"),
            (self.world._h.apply_scenario, ({},), TypeError, "apply_scenario: expected a Scenario"),
            (native.decimal_year, (), TypeError, "decimal_year() takes 1 argument(s) (0 given)"),
        )
        for function, args, error, message in cases:
            with self.subTest(function=function, args=args):
                with self.assertRaises(error) as raised:
                    function(*args)
                self.assertEqual(str(raised.exception), message)
        with self.assertRaisesRegex(TypeError, "Scenario\\(path\\) or Scenario\\(None, json, source\\)"):
            native.Scenario(json="{}")
        with self.assertRaises(fsim.Error) as raised:
            native.Scenario(None, "{", "compatibility-bad.json")
        self.assertIs(type(raised.exception), native.Error)
        self.assertIs(native.Error, fsim.Error)
        self.assertIn("compatibility-bad.json", str(raised.exception))

    def test_default_metadata_and_scenario_options_are_independent_copies(self):
        scenario = fsim.Scenario(json="{}")
        options = scenario._h.world_options()
        self.assertEqual(options, WORLD_DEFAULTS)
        self.assertEqual(list(options), list(WORLD_DEFAULTS))
        self.assertEqual(scenario.vehicle_count, 0)
        public = scenario.world_options
        self.assertNotIn("jsbsim_root", public)
        self.assertIs(public["publish"], True)
        self.assertIs(public["terrain"], False)
        self.assertIs(type(public["terrain_offline"]), int)
        options["name"] = "caller-copy"
        self.assertEqual(scenario._h.world_options()["name"], "default")
        retained = fsim.Scenario(json=json.dumps({"world": {"name": "retained", "record_path": "retained/file"}}))
        copied = retained._h.world_options()
        del retained
        gc.collect()
        self.assertEqual((copied["name"], copied["record_path"]), ("retained", "retained/file"))
        self.assertEqual(self.world.environment, ENVIRONMENT_DEFAULTS)
        self.assertEqual(list(self.world.environment), list(ENVIRONMENT_DEFAULTS))
        self.assertFalse(self.world._h.borrowed())
        self.assertIsNone(self.world._h.agent_identity())

    def test_scenario_applies_environment_vehicles_and_initial_control(self):
        scenario = fsim.Scenario(json=json.dumps({
            "environment": {"wind": {"direction_deg": 180, "speed_ms": 5}},
            "vehicles": [{"name": "pair", "count": 2,
                          "initial": {"alt_msl_m": 1200, "airspeed_ms": 60},
                          "command": {"level": "attitude", "pitch_deg": 3}}],
        }))
        raw = self.world._h.apply_scenario(scenario._h)
        self.assertIs(type(raw), bytes)
        ids = array.array("I")
        ids.frombytes(raw)
        self.assertEqual(len(ids), scenario.vehicle_count)
        self.assertEqual(len(ids), 2)
        self.assertEqual(self.world._h.vehicle_ids(), raw)
        self.assertEqual(self.world._h.vehicle_info(ids[0]), ("pair-1", "jsbsim:c172x"))
        self.assertEqual(self.world._h.find_vehicle("pair-2"), ids[1])
        self.assertEqual(self.world.vehicle(ids[0]).active_level, fsim.Level.ATTITUDE)
        self.assertEqual(self.world.environment["wind_speed_ms"], 5.0)
        self.assertEqual(self.world.environment["wind_direction_deg"], 180.0)
        del scenario
        gc.collect()
        self.world.step(1)
        self.assertEqual(self.world._h.info()["vehicle_count"], 2)
        self.assertAlmostEqual(self.world.vehicle(ids[0]).state.sim_time, self.world.time)
        agent = self.world._h.create_agent(ids[0], "adapter-agent")
        self.assertEqual(agent.agent_identity()[::2], (ids[0], "adapter-agent"))
        with self.assertRaises(fsim.Error) as raised:
            agent.get_environment()
        self.assertIs(type(raised.exception), native.Error)
        self.assertIn("fsim_world_get_environment", str(raised.exception))
        self.assertTrue(math.isnan(agent.magnetic_year()))
        self.assertEqual(agent.terrain([], []), [])

    def test_environment_updates_and_magnetic_tuple_contract(self):
        self.world.set_environment(wind_speed_ms=7.0, cloud_cover=0.4)
        changed = dict(ENVIRONMENT_DEFAULTS, wind_speed_ms=7.0, cloud_cover=0.4)
        self.assertEqual(self.world.environment, changed)
        self.assertEqual(self.world._h.terrain([], []), [])
        self.assertEqual(self.world._h.terrain([0.1, 0.2], [0.2, 0.3]), [0.0, 0.0])
        self.assertEqual(self.world.magnetic_year, 2025.0)
        field = native.magnetic_field(0.1, 0.2, 0.0, 2025.0)
        self.assertIs(type(field), tuple)
        self.assertEqual(len(field), 7)
        self.assertTrue(all(type(value) is float and math.isfinite(value) for value in field))
        north, east, down, horizontal, total, declination, inclination = field
        self.assertAlmostEqual(horizontal, math.hypot(north, east))
        self.assertAlmostEqual(total, math.hypot(horizontal, down))
        self.assertAlmostEqual(declination, math.atan2(east, north))
        self.assertAlmostEqual(inclination, math.atan2(down, horizontal))
        with self.assertRaisesRegex(ValueError, "a place or date not finite"):
            native.magnetic_field(float("nan"), 0.0, 0.0, 2025.0)
        self.assertEqual(native.decimal_year(1735689600.0), 2025.0)

    def test_borrowed_world_retains_owner_and_gather_releases_buffers(self):
        env = fsim.VecEnv(1, publish=False, workers=1, terrain=False)
        self.addCleanup(env.close)
        env.reset()
        borrowed = env.world
        self.addCleanup(borrowed.close)
        self.assertIs(type(borrowed._h), native.World)
        self.assertTrue(borrowed._h.borrowed())
        vehicle_id = int(env.vehicle_ids[0])
        env.close()
        gc.collect()
        self.assertEqual(borrowed._h.vehicle_info(vehicle_id)[0], "env0/0")
        ids = array.array("I", [vehicle_id])
        out = bytearray(fsim.vehicle_state_dtype.itemsize)
        self.assertIsNone(borrowed._h.gather_states(ids, out))
        self.assertEqual(bytes(out), bytes(memoryview(borrowed._h.state_buffer(vehicle_id))))
        ids.append(vehicle_id)
        out.append(0)  # neither input nor output remains exported after success
        with self.assertRaisesRegex(ValueError, "out holds"):
            borrowed._h.gather_states(ids, out)
        ids.append(vehicle_id)
        out.append(0)  # early size failures release both views too
        with self.assertRaisesRegex(TypeError, "writable C-contiguous array"):
            borrowed._h.gather_states(ids, bytes(out))
        ids.append(vehicle_id)


if __name__ == "__main__":
    unittest.main()
