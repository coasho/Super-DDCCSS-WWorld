"""The native links adapter keeps its existing Python identities and call contract."""
import pickle
import types
import unittest

import fsim
import fsim._native as native
import fsim.world as world_module


LINK_METHODS = (
    "create_c2_node", "remove_c2_node", "create_net", "join_net", "leave_net", "assign_system",
    "dms_system", "dms_nets", "dms_send", "dms_inbox_take", "dms_subscribe", "dms_subscribe_all",
    "dms_subscriptions", "dms_inbox", "dms_transmission", "system_entity", "entity_system", "net_count",
    "net_stats", "set_strike_policy", "strike_policy", "agent_c2_tasks", "agent_answer_task", "agent_overrides",
    "c2_send_task", "c2_set_override", "c2_consents", "c2_answer_consent", "set_rbac", "rbac", "agent_authority",
    "agent_comms", "c2_set_lost_comms", "c2_package", "c2_declare_role", "c2_designate_control", "c2_command_hsa",
    "c2_command_route", "c2_cancel_command", "c2_command_statuses", "set_dissemination", "dms_publish", "dms_peers",
    "dms_link_tracks", "designate_link", "designation_source", "link_track_truth",
)

RECORD_FIELDS = {
    "DmsReceived": "message source topic net time_sent time_delivered data",
    "DmsTransmission": "message topic time_sent published queue_dropped reached destinations",
    "NetStats": "transmissions delivered duplicates out_of_range masked lost queue_dropped failed overflowed",
    "PeerReport": "system net received_s tail_number has_position time_s latitude_rad longitude_rad altitude_msl_m "
                  "velocity_ned_ms heading_rad airspeed_true_ms airspeed_calibrated_ms mach",
    "Role": "id name permissions approves_strikes",
    "C2Authority": "designated designated_modes designated_by designated_s operators commands",
    "CommandStatus": "source received_s command_id topic state reason activity time_s percent",
    "C2Task": "source received_s state percent task_id type priority latitude_rad longitude_rad altitude_m radius_m "
              "start_s end_s target_source target_payload target_track text",
    "ActiveOverride": "source kind value expiry_s",
    "CommsState": "lost_comms_set threshold_s plan lost lost_since_s alerts package_id leader members",
    "ConsentRequest": "source received_s answered request_id strike store quantity target_source target_payload "
                      "target_track latitude_rad longitude_rad altitude_m time_s",
    "LinkTrack": "source net received_s track payload updates time_s latitude_rad longitude_rad height_m "
                 "velocity_ned_ms position_sigma_m velocity_sigma_ms",
}


class BindingCompatibilityTest(unittest.TestCase):
    def setUp(self):
        self.world = fsim.World("py-binding-compatibility", publish=False, workers=1, pin_workers=False, terrain=False)
        self.addCleanup(self.world.close)

    def test_native_descriptors_remain_on_the_original_world_type(self):
        self.assertIs(fsim.World, world_module.World)
        self.assertEqual((native.World.__module__, native.World.__qualname__), ("fsim._native", "World"))
        for name in LINK_METHODS:
            with self.subTest(method=name):
                descriptor = vars(native.World)[name]
                self.assertIsInstance(descriptor, types.MethodDescriptorType)
                self.assertIs(descriptor.__objclass__, native.World)
                self.assertEqual((descriptor.__name__, descriptor.__qualname__), (name, "World." + name))
                self.assertIs(getattr(self.world._h, name).__self__, self.world._h)

    def test_native_links_methods_reject_keyword_arguments(self):
        for name in LINK_METHODS:
            with self.subTest(method=name):
                with self.assertRaisesRegex(TypeError, "takes no keyword arguments"):
                    getattr(self.world._h, name)(unused=0)

    def test_argument_errors_keep_the_original_conversion_order(self):
        cases = (
            ("create_c2_node", (), TypeError, "create_c2_node() takes 4 argument(s) (0 given)"),
            ("dms_send", (), TypeError, "dms_send() takes 4 argument(s) (0 given)"),
            ("c2_command_hsa", (), TypeError, "c2_command_hsa() takes 8 argument(s) (0 given)"),
            ("dms_system", ("wrong",), TypeError, "an integer is required"),
            ("create_c2_node", (1, 0.0, 0.0, 0.0), TypeError, "name must be a string"),
            ("c2_command_route", (1, 2, []), ValueError, "c2_command_route: 1 to 16 points"),
            ("create_net", ("invalid", {"unknown": 1}), ValueError, "create_net: no net field 'unknown'"),
        )
        for name, args, error, message in cases:
            with self.subTest(method=name, args=args):
                with self.assertRaises(error) as raised:
                    getattr(self.world._h, name)(*args)
                self.assertEqual(str(raised.exception), message)

    def test_native_failure_uses_the_existing_error_class(self):
        self.assertIs(fsim.Error, native.Error)
        self.assertEqual((fsim.Error.__module__, fsim.Error.__qualname__), ("fsim", "Error"))
        with self.assertRaises(fsim.Error) as raised:
            self.world._h.c2_command_hsa(0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertIs(type(raised.exception), fsim.Error)
        self.assertEqual(str(raised.exception), "fsim_c2_command_hsa: not sent")
        restored = pickle.loads(pickle.dumps(raised.exception))
        self.assertIs(type(restored), fsim.Error)
        self.assertEqual(str(restored), str(raised.exception))

    def test_public_record_fields_and_pickle_identity_remain_in_world(self):
        for name, fields in RECORD_FIELDS.items():
            with self.subTest(record=name):
                record = getattr(fsim, name)
                self.assertIs(record, getattr(world_module, name))
                self.assertEqual((record.__module__, record.__qualname__), ("fsim.world", name))
                self.assertEqual(record._fields, tuple(fields.split()))
                value = record(*range(len(fields.split())))
                for protocol in (0, pickle.HIGHEST_PROTOCOL):
                    restored = pickle.loads(pickle.dumps(value, protocol=protocol))
                    self.assertIs(type(restored), record)
                    self.assertEqual(restored, value)

    def test_dms_copies_a_contiguous_buffer_and_retains_tuple_order(self):
        sender = self.world.create_c2_node("sender", 0.0, 0.0)
        receiver = self.world.create_c2_node("receiver", 0.0, 0.0)
        net = self.world.create_net("contract", latency_s=0.0)
        for node, system in ((sender, 0x3001), (receiver, 0x3002)):
            self.assertEqual(self.world.assign_system(node, system), system)
            self.assertTrue(self.world.join_net(node, net))
        source = bytearray(b"payload\x00bytes")
        view = memoryview(source)
        message = self.world._h.dms_send(sender, 1001, view, [0x3002])
        view.release()
        source[:] = b"xxxxxxxxxxxxx"
        self.world.step(20)
        (received,) = self.world._h.dms_inbox(receiver)
        self.assertEqual(len(received), 7)
        self.assertEqual(received[:4], (message, 0x3001, 1001, net))
        self.assertGreaterEqual(received[5], received[4])
        self.assertEqual(received[6], b"payload\x00bytes")
        decoded = fsim.DmsReceived(*received)
        self.assertEqual((decoded.source, decoded.topic, decoded.data), (0x3001, 1001, b"payload\x00bytes"))
        status = fsim.DmsTransmission(*self.world._h.dms_transmission(sender, message))
        self.assertEqual((status.message, status.topic, status.reached), (message, 1001, 1))
        self.assertEqual(status.destinations, [(0x3002, fsim.DELIVERY_DELIVERED)])
        self.assertIs(type(pickle.loads(pickle.dumps(decoded))), fsim.DmsReceived)
        self.assertIsNone(self.world._h.dms_transmission(sender, message + 1000))

    def test_dms_rejects_a_strided_payload_buffer(self):
        with self.assertRaises(BufferError):
            self.world._h.dms_send(0, 1001, memoryview(b"abcdef")[::2], [])


if __name__ == "__main__":
    unittest.main()
