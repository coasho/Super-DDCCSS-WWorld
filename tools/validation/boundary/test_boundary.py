"""Principal-guard scanner regressions; no native build or SDK import is needed."""

import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import boundary


class GuardCoverageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="fsim-boundary-")
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name)
        self.write("src/sdk/fsim.cpp", "")
        self.write("src/sdk/fsim_world_c.cpp", "")
        self.write("include/fsim/fsim_c.h", "")
        self.result = {}
        self.data = {"rule": []}

    def write(self, name, text):
        path = self.repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def exported(self, name="fsim_vehicle_query", principal="agent",
                 params="const fsim_world* world"):
        self.result["c", name] = len(self.data["rule"])
        self.data["rule"].append({"principal": principal})
        self.write("include/fsim/fsim_c.h", "FSIM_API int %s(%s);\n" % (name, params))
        return name

    def errors(self):
        with patch.object(boundary, "REPO", os.fspath(self.repo)):
            return boundary.guards(self.result, self.data)

    def test_new_adapter_is_discovered_and_its_missing_guard_is_rejected(self):
        name = self.exported()
        self.write("src/sdk/fsim_mission_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { return 0; }" % name)
        self.assertTrue(any(name in e and "does not open" in e for e in self.errors()))

    def test_adapter_outside_sdk_directory_is_discovered(self):
        name = self.exported()
        self.write("src/comm/fsim_comm_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { return 0; }" % name)
        self.assertTrue(any(name in e and "does not open" in e for e in self.errors()))

    def test_exported_world_input_requires_an_inspected_definition(self):
        name = self.exported()
        self.assertTrue(any(name in e and "no inspected definition" in e for e in self.errors()))

    def test_duplicate_exported_world_input_definitions_are_rejected(self):
        name = self.exported()
        body = "FSIM_API int %s(const fsim_world* world) { FSIM_AGENT_CALL(world, -1); return 0; }" % name
        self.write("src/sdk/fsim_world_c.cpp", body)
        self.write("src/sdk/fsim_mission_c.cpp", body)
        self.assertTrue(any(name in e and "duplicate" in e for e in self.errors()))

    def test_missing_first_guard_is_rejected_in_existing_adapter(self):
        name = self.exported()
        self.write("src/sdk/fsim_world_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { return 0; }" % name)
        self.assertTrue(any(name in e and "does not open" in e for e in self.errors()))

    def test_guard_later_in_body_is_not_the_first_guard(self):
        name = self.exported()
        self.write("src/sdk/fsim_world_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { int n = 0; FSIM_AGENT_CALL(world, -1); return n; }" % name)
        self.assertTrue(any(name in e and "does not open" in e for e in self.errors()))

    def test_wrong_principal_guard_is_rejected(self):
        name = self.exported()
        self.write("src/sdk/fsim_world_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { FSIM_NOT_AGENT(world, -1); return 0; }" % name)
        self.assertTrue(any(name in e and "opens with FSIM_NOT_AGENT" in e for e in self.errors()))

    def test_agent_guard_is_rejected_for_referee(self):
        name = self.exported(principal="referee")
        self.write("src/sdk/fsim_world_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { FSIM_AGENT_CALL(world, -1); return 0; }" % name)
        self.assertTrue(any(name in e and "opens with FSIM_AGENT_CALL" in e for e in self.errors()))

    def test_valid_new_adapter_passes(self):
        name = self.exported()
        self.write("src/sdk/fsim_mission_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { FSIM_AGENT_CALL(world, -1); return 0; }" % name)
        self.assertEqual([], self.errors())

    def test_world_output_and_value_exports_do_not_require_guards(self):
        self.exported(name="fsim_world_create", principal="range", params="fsim_world** out")
        self.write("src/sdk/fsim_world_c.cpp",
                   "FSIM_API int fsim_world_create(fsim_world** out) { return 0; }")
        self.assertEqual([], self.errors())

    def test_world_pointer_return_does_not_require_a_guard(self):
        name = self.exported(name="fsim_vecenv_world", principal="range", params="fsim_vecenv* env")
        self.write("src/sdk/fsim.cpp", "FSIM_API fsim_world* %s(fsim_vecenv* env) { return 0; }" % name)
        self.assertEqual([], self.errors())

    def test_const_world_output_does_not_require_a_guard(self):
        name = self.exported(name="fsim_world_create", principal="range", params="fsim_world* const* out")
        self.write("src/sdk/fsim_world_c.cpp", "FSIM_API int %s(fsim_world* const* out) { return 0; }" % name)
        self.assertEqual([], self.errors())

    def test_cv_qualified_world_inputs_require_first_guard(self):
        for params in ("const fsim_world* world", "fsim_world const* world",
                       "volatile fsim_world* world", "fsim_world volatile* world",
                       "const volatile fsim_world* const volatile world",
                       "fsim_world const volatile*const volatile world",
                       "const struct fsim_world* const world"):
            with self.subTest(params=params):
                name = self.exported(params=params)
                self.write("src/sdk/fsim_world_c.cpp", "FSIM_API int %s(%s) { return 0; }" % (name, params))
                self.assertTrue(any(name in e and "does not open" in e for e in self.errors()))

    def test_cv_qualified_world_inputs_require_inspected_definitions(self):
        for params in ("fsim_world const* world", "fsim_world volatile* const world",
                       "fsim_world const volatile*const volatile world"):
            with self.subTest(params=params):
                name = self.exported(params=params)
                self.assertTrue(any(name in e and "no inspected definition" in e for e in self.errors()))

    def test_cv_qualified_world_outputs_do_not_require_guard(self):
        for params in ("fsim_world const** out", "fsim_world const* const* out",
                       "fsim_world volatile*volatile* out", "fsim_world* const volatile* out",
                       "const volatile struct fsim_world* const volatile* out"):
            with self.subTest(params=params):
                name = self.exported(name="fsim_world_create", principal="range", params=params)
                self.write("src/sdk/fsim_world_c.cpp", "FSIM_API int %s(%s) { return 0; }" % (name, params))
                self.assertEqual([], self.errors())

    def test_claims_release_and_unknown_agent_macros_are_not_principal_guards(self):
        for guard in ("FSIM_AGENT_AS", "FSIM_AGENT_RELEASE", "FSIM_AGENT_UNKNOWN"):
            with self.subTest(guard=guard):
                name = self.exported()
                self.write("src/sdk/fsim_world_c.cpp",
                           "FSIM_API int %s(fsim_world* world) { %s(world, -1); return 0; }" % (name, guard))
                self.assertTrue(any(name in e and "does not open" in e for e in self.errors()))

    def test_release_is_allowed_for_existing_lifetime_and_identity_functions(self):
        for name in ("fsim_world_destroy", "fsim_agent_identity"):
            with self.subTest(name=name):
                self.exported(name=name)
                self.write("src/sdk/fsim_world_c.cpp",
                           "FSIM_API int %s(fsim_world* world) { FSIM_AGENT_RELEASE(world); return 0; }" % name)
                self.assertEqual([], self.errors())

    def test_optional_vision_exports_are_not_blanket_guard_coverage(self):
        name = self.exported(name="fsim_vision_create", principal="range")
        self.write("include/fsim/fsim_c.h", "")
        self.write("include/fsim/fsim_vision_c.h", "FSIM_VISION_API int %s(fsim_world* world);" % name)
        self.write("src/vision/fsim_vision_c.cpp", "FSIM_VISION_API int %s(fsim_world* world) { return 0; }" % name)
        self.assertEqual([], self.errors())

    def test_unexported_private_definition_does_not_require_guard(self):
        self.write("src/sdk/fsim_world_c.cpp", "FSIM_API int fsim_private(fsim_world* world) { return 0; }")
        self.assertEqual([], self.errors())

    def test_comments_before_guard_are_not_statements(self):
        name = self.exported()
        self.write("src/sdk/fsim_world_c.cpp",
                   "FSIM_API int %s(const fsim_world* world) { /* scoped call */ FSIM_AGENT_CALL(world, -1); return 0; }" % name)
        self.assertEqual([], self.errors())

    def test_commented_definition_cannot_satisfy_coverage(self):
        name = self.exported()
        self.write("src/sdk/fsim_world_c.cpp",
                   "/* FSIM_API int %s(const fsim_world* world) { FSIM_AGENT_CALL(world, -1); return 0; } */" % name)
        self.assertTrue(any(name in e and "no inspected definition" in e for e in self.errors()))

    def test_cpp_digit_separators_do_not_hide_definitions(self):
        name = self.exported()
        self.write("src/sdk/fsim_world_c.cpp",
                   "constexpr auto mask = 0x4000'0000; "
                   "FSIM_API int %s(const fsim_world* world) { FSIM_AGENT_CALL(world, -1); return 0; } "
                   "constexpr char label = 'x';" % name)
        self.assertEqual([], self.errors())


if __name__ == "__main__":
    unittest.main()
