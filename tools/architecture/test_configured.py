"""Executable negative fixtures for the configured-graph gate (configured.py): test_check's literal source tree, with a
CMake File API reply written beside it as a configure would, then a violation planted in that reply."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_check import FixtureRepository  # noqa: E402

CONFIGURED = Path(__file__).with_name("configured.py")


def include(entry):
    """A code model include directory: a path, or (path,) for a system one."""
    return {"path": entry[0], "isSystem": True} if isinstance(entry, tuple) else {"path": entry}


class ConfiguredFixtures(FixtureRepository):
    """Each module target configured with its own sources, its declared links and the include views of its closure;
    the cache enables both branches. Tests then plant a violation in this reply."""

    SOURCES = {"flow": ["src/world/FlowSolver.cpp", "src/world/FlowSolver.h"], "viewer": ["src/app/viewer_main.cpp"]}

    def setUp(self):
        super().setUp()
        self.build = self.repo / "build"
        self.cache = {"FSIM_WITH_RENDER": "ON", "FSIM_BUILD_PYTHON": "ON", "CMAKE_BUILD_TYPE": "Release"}
        self.version = (2, 11)
        self.cmake = (4, 4, 3)
        self.source = self.repo
        self.reply_build = self.build
        self.targets = {}
        for name, module in self.policy["modules"].items():
            self.targets[module["target"]] = {
                "sources": self.SOURCES.get(name, [f"src/{name}/Header.h"]),
                "links": [self.policy["modules"][d]["target"] for d in module["allowed_dependencies"]],
                "deps": [self.policy["modules"][d]["target"] for d in module["allowed_dependencies"]],
                "includes": [str(self.build / "include-views" / module["target"]), str(self.repo / "include")],
            }

    def target(self, name):
        return self.targets[self.policy["modules"][name]["target"]]

    def write_reply(self):
        reply = self.build / ".cmake" / "api" / "v1" / "reply"
        reply.mkdir(parents=True, exist_ok=True)
        ids = {name: f"{name}::@fixture" for name in self.targets}
        entries = []
        for name, target in self.targets.items():
            data = {"name": name, "id": ids[name], "type": target.get("type", "STATIC_LIBRARY"),
                    "sources": [{"path": path} for path in target["sources"]],
                    "linkLibraries": [{"id": ids[link]} if link in ids else {"fragment": link} for link in target["links"]],
                    "interfaceLinkLibraries": [{"id": ids[link]} for link in target.get("interface", [])],
                    "dependencies": [{"id": ids[dependency]} for dependency in target["deps"] if dependency in ids],
                    "compileGroups": [{"language": "CXX", "includes": [include(entry) for entry in target["includes"]]}]}
            (reply / f"target-{name}.json").write_text(json.dumps(data), encoding="utf-8")
            entries.append({"name": name, "id": ids[name], "jsonFile": f"target-{name}.json"})
        model = {"kind": "codemodel", "version": {"major": self.version[0], "minor": self.version[1]},
                 "paths": {"source": str(self.source), "build": str(self.reply_build)},
                 "configurations": [{"name": "Release", "targets": entries}]}
        (reply / "codemodel-v2-fixture.json").write_text(json.dumps(model), encoding="utf-8")
        cache = {"kind": "cache", "entries": [{"name": key, "value": value, "type": "STRING"} for key, value in self.cache.items()]}
        (reply / "cache-v2-fixture.json").write_text(json.dumps(cache), encoding="utf-8")
        index = {"cmake": {"version": dict(zip(("major", "minor", "patch"), self.cmake),
                                           string=".".join(map(str, self.cmake)))},
                 "reply": {"codemodel-v2": {"jsonFile": "codemodel-v2-fixture.json"},
                           "cache-v2": {"jsonFile": "cache-v2-fixture.json"}}}
        (reply / "index-2026-10-17T00-00-00-0000.json").write_text(json.dumps(index), encoding="utf-8")

    def run_configured(self, reply=True):
        self.write("policy.json", json.dumps(self.policy))
        if reply:
            self.write_reply()
        result = subprocess.run([sys.executable, "-B", str(CONFIGURED), "--repo", str(self.repo), "--build", str(self.build),
                                 "--policy", str(self.repo / "policy.json")], capture_output=True, text=True, check=False)
        return result.returncode, result.stdout + result.stderr

    def rejects(self, code, detail):
        status, output = self.run_configured()
        self.assertEqual(status, 1, output)
        self.assertIn(code, output)
        self.assertIn(detail, output)

    def input_error(self, detail, reply=True):
        status, output = self.run_configured(reply)
        self.assertEqual(status, 2, output)
        self.assertIn("configured graph input error", output)
        self.assertIn(detail, output)

    # --- accepted ------------------------------------------------------------------------------------------------------

    def test_a_configure_matching_the_source_graph_is_accepted(self):
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)
        self.assertIn("12 module targets configured; absent: none", output)
        self.assertIn("application sources compiled by tests: none", output)

    def test_the_renderer_modules_may_be_absent_with_the_renderer_off(self):
        self.cache["FSIM_WITH_RENDER"] = "OFF"
        for name in ("render", "world", "flow", "viewer"):
            del self.targets[self.policy["modules"][name]["target"]]
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)
        self.assertIn("render (FSIM_WITH_RENDER is off)", output)

    def test_a_python_extension_module_may_be_absent_with_the_python_sdk_off(self):
        self.write("src/adapters/sdk/python/CMakeLists.txt", "add_library(fsim_python MODULE _native.c)\n")
        self.write("src/adapters/sdk/python/_native.c", "/* native */\n")
        self.policy["modules"]["python_native"] = {
            "target": "fsim_python", "aliases": [], "domain": "adapters", "kind": "assembly", "directories": [],
            "destinations": ["src/adapters/sdk/python"], "files": [], "allowed_dependencies": [], "headless": True}
        self.cache["FSIM_BUILD_PYTHON"] = "OFF"
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)
        self.assertIn("python_native (FSIM_BUILD_PYTHON is off)", output)
        self.cache["FSIM_BUILD_PYTHON"] = "ON"
        self.rejects("unconfigured-module", "python_native: not configured, though its branch is enabled here")

    def test_a_test_target_may_compile_an_applications_sources(self):
        self.write("tests/simulation/test_viewer.cpp", "\n")
        self.targets["fsim_viewer_test"] = {"sources": ["tests/simulation/test_viewer.cpp", "src/app/viewer_main.cpp"],
                                            "links": [], "deps": [], "includes": []}
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)
        self.assertIn("application sources compiled by tests: fsim_viewer_test (viewer)", output)

    def test_registered_external_links_system_and_outside_includes_are_accepted(self):
        self.policy["external_targets"].append("libExternal")
        self.targets["libExternal"] = {"sources": ["third_party/external/a.c"], "links": [], "deps": [], "includes": []}
        self.target("sim")["links"].append("libExternal")
        self.target("sim")["includes"] += [(str(self.repo / "generated"),), str(self.repo / "third_party" / "external"),
                                           str(Path(sys.executable).resolve().parent)]
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)
        self.assertIn("external: libExternal", output)

    def test_a_modules_own_and_its_providers_directories_may_be_included(self):
        self.target("core")["includes"] += [str(self.repo / "src" / "core"), str(self.repo / "src" / "platform"),
                                            str(self.build / "include-views" / "fsim_platform")]
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)

    # --- targets and sources -------------------------------------------------------------------------------------------

    def test_an_unregistered_target_compiling_a_library_source_is_rejected(self):
        self.targets["fsim_extra"] = {"sources": ["src/sim/Header.h"], "links": [], "deps": [], "includes": []}
        self.rejects("unregistered-target", "fsim_extra: compiles sources under src (src/sim/Header.h)")

    def test_a_test_target_compiling_a_library_source_is_rejected(self):
        self.write("tests/simulation/test_x.cpp", "\n")
        self.targets["fsim_x_test"] = {"sources": ["tests/simulation/test_x.cpp", "src/sim/Header.h"], "links": [], "deps": [],
                                       "includes": []}
        self.rejects("unregistered-target", "fsim_x_test: compiles sources under src (src/sim/Header.h)")

    def test_a_non_test_target_compiling_an_applications_source_is_rejected(self):
        self.targets["fsim_tool"] = {"sources": ["tools/models/tool.cpp", "src/app/viewer_main.cpp"], "links": [], "deps": [],
                                     "includes": []}
        self.rejects("unregistered-target", "fsim_tool: compiles sources under src (src/app/viewer_main.cpp)")

    def test_a_module_target_compiling_another_modules_source_is_rejected(self):
        self.target("core")["sources"].append("src/sim/Header.h")
        self.rejects("foreign-source", "fsim_core: src/sim/Header.h is owned by sim, not core")

    # --- links and dependencies ----------------------------------------------------------------------------------------

    def test_a_configured_link_the_source_graph_lacks_is_rejected(self):
        self.target("core")["links"].append("fsim_ms")
        self.rejects("unknown-configured-link", "fsim_core: links fsim_ms (ms), which the source graph does not declare")

    def test_an_interface_link_the_source_graph_lacks_is_rejected(self):
        self.target("core")["interface"] = ["fsim_sim"]
        self.rejects("unknown-configured-link", "fsim_core: links fsim_sim (sim)")

    def test_a_link_to_an_unregistered_target_is_rejected(self):
        self.targets["helper"] = {"sources": ["tools/models/helper.cpp"], "links": [], "deps": [], "includes": []}
        self.target("sim")["links"].append("helper")
        self.rejects("unregistered-link", "fsim_sim: links helper, neither a module nor a registered external target")

    def test_a_build_order_dependency_outside_the_closure_is_rejected(self):
        self.target("core")["deps"].append("fsim_sim")
        self.rejects("hidden-dependency", "fsim_core: depends on fsim_sim (sim) outside its source closure")

    def test_a_source_edge_the_configure_dropped_is_rejected(self):
        self.target("session")["links"].remove("fsim_ms")
        self.rejects("missing-configured-edge", "session: the source graph's link to ms is not configured here")

    def test_a_missing_module_with_its_branch_enabled_is_rejected(self):
        del self.targets["fsim_ms"]
        status, output = self.run_configured()
        self.assertEqual(status, 1, output)
        self.assertIn("unconfigured-module: ms: not configured, though its branch is enabled here", output)
        self.assertIn("unconfigured-provider: session: its provider ms is not configured here", output)

    def test_a_renderer_module_missing_with_the_renderer_on_is_rejected(self):
        del self.targets["fsim_flow"]
        self.rejects("unconfigured-module", "flow: not configured, though its branch is enabled here (FSIM_WITH_RENDER ON")

    # --- include directories -------------------------------------------------------------------------------------------

    def test_src_and_its_ancestors_are_broad_include_roots(self):
        for root in ("src", "."):
            with self.subTest(root=root):
                self.setUp()
                self.target("core")["includes"].append(str((self.repo / root).resolve()))
                self.rejects("broad-include-root", "fsim_core: includes")

    def test_a_directory_outside_the_closure_is_rejected(self):
        self.target("core")["includes"].append(str(self.repo / "src" / "sim"))
        self.rejects("include-outside-closure", "fsim_core: includes src/sim, outside its and its providers' destinations")

    def test_an_include_view_outside_the_closure_is_rejected(self):
        self.target("core")["includes"].append(str(self.build / "include-views" / "fsim_sim"))
        self.rejects("include-outside-closure", "fsim_core: includes sim's view outside its source closure")

    def test_an_unknown_include_view_is_rejected(self):
        self.target("core")["includes"].append(str(self.build / "include-views" / "not_a_target"))
        self.rejects("unknown-include-view", "fsim_core: includes the view not_a_target")

    def test_an_unclassified_repository_directory_is_rejected(self):
        self.target("core")["includes"].append(str(self.repo / "generated"))
        self.rejects("unclassified-include", "fsim_core: includes generated")

    def test_a_system_include_of_a_foreign_src_directory_is_rejected(self):
        # HC-38: SYSTEM changes compiler diagnostics, not ownership
        self.target("core")["includes"].append((str(self.repo / "src" / "sim"),))
        self.rejects("include-outside-closure", "fsim_core: includes src/sim, outside its and its providers' destinations")

    def test_a_system_include_of_a_foreign_include_view_is_rejected(self):
        self.target("core")["includes"].append((str(self.build / "include-views" / "fsim_sim"),))
        self.rejects("include-outside-closure", "fsim_core: includes sim's view outside its source closure")

    def test_system_includes_of_owned_vendor_and_toolchain_directories_are_accepted(self):
        self.target("core")["includes"] += [(str(self.repo / "src" / "platform"),),
                                            (str(self.build / "include-views" / "fsim_platform"),),
                                            (str(self.repo / "third_party" / "external"),),
                                            (str(self.build / "deps" / "install" / "include"),),
                                            (str(Path(sys.executable).resolve().parent),)]
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)

    # --- branches: absent where disabled, present where enabled (HC-42) --------------------------------------------------

    def test_renderer_modules_configured_with_the_renderer_off_are_rejected(self):
        self.cache["FSIM_WITH_RENDER"] = "OFF"
        status, output = self.run_configured()
        self.assertEqual(status, 1, output)
        for module in ("render", "world", "flow", "viewer"):
            self.assertIn(f"disabled-branch-module: {module}: configured although FSIM_WITH_RENDER is off", output)

    def test_one_renderer_module_left_with_the_renderer_off_is_rejected(self):
        self.cache["FSIM_WITH_RENDER"] = "OFF"
        for name in ("render", "world", "viewer"):
            del self.targets[self.policy["modules"][name]["target"]]
        self.rejects("disabled-branch-module", "flow: configured although FSIM_WITH_RENDER is off")

    def test_a_python_extension_module_configured_with_the_python_sdk_off_is_rejected(self):
        self.write("src/adapters/sdk/python/CMakeLists.txt", "add_library(fsim_python MODULE _native.c)\n")
        self.write("src/adapters/sdk/python/_native.c", "/* native */\n")
        self.policy["modules"]["python_native"] = {
            "target": "fsim_python", "aliases": [], "domain": "adapters", "kind": "assembly", "directories": [],
            "destinations": ["src/adapters/sdk/python"], "files": [], "allowed_dependencies": [], "headless": True}
        self.targets["fsim_python"] = {"sources": ["src/adapters/sdk/python/_native.c"], "links": [], "deps": [],
                                       "includes": []}
        self.cache["FSIM_BUILD_PYTHON"] = "OFF"
        self.rejects("disabled-branch-module", "python_native: configured although FSIM_BUILD_PYTHON is off")
        self.cache["FSIM_BUILD_PYTHON"] = "ON"
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)

    # --- the source graph and the reply itself -------------------------------------------------------------------------

    def test_the_source_graphs_own_violations_are_reported(self):
        self.append("src/ms/Header.h", '#include "sim/Header.h"\n')
        self.rejects("source-graph: undeclared-include", "src/ms/Header.h")

    def test_without_a_reply_the_preset_is_pending_not_passed(self):
        self.input_error("configured-graph acceptance is pending, not passed", reply=False)

    def test_without_a_reply_the_configuring_cmake_version_is_named(self):
        # (an older CMake answers only query files, from its next configure, and never with direct link items)
        self.write("build/CMakeCache.txt", "CMAKE_CACHE_MAJOR_VERSION:INTERNAL=3\nCMAKE_CACHE_MINOR_VERSION:INTERNAL=25\n"
                                           "CMAKE_CACHE_PATCH_VERSION:INTERNAL=0\n")
        status, output = self.run_configured(reply=False)
        self.assertEqual(status, 2, output)
        self.assertIn("this build was configured with CMake 3.25.0", output)
        self.assertIn("architecture_configured needs CMake 4.2 or later (native builds remain supported from CMake 3.25)",
                      output)

    def test_a_reply_from_cmake_older_than_4_2_is_pending_with_an_actionable_diagnostic(self):
        for cmake, codemodel in (((3, 25, 0), (2, 4)), ((3, 31, 6), (2, 7)), ((4, 1, 2), (2, 9))):
            with self.subTest(cmake=cmake):
                self.cmake, self.version = cmake, codemodel
                status, output = self.run_configured()
                self.assertEqual(status, 2, output)
                self.assertIn("this build's reply is from CMake %d.%d.%d" % cmake, output)
                self.assertIn("configure this preset with CMake 4.2+ to check its configured graph", output)
                self.assertIn("configured-graph acceptance is pending, not passed", output)

    def test_cmake_4_2_is_accepted(self):
        self.cmake = (4, 2, 0)
        status, output = self.run_configured()
        self.assertEqual(status, 0, output)
        self.assertIn("CMake 4.2.0", output)

    def test_a_reply_of_another_build_is_an_input_error(self):
        # HC-42: the reply must be the requested build's, not another preset's
        self.reply_build = self.repo / "other-build"
        self.input_error("the reply is of the build")

    def test_a_code_model_without_direct_link_items_is_an_input_error(self):
        self.version = (2, 8)
        self.input_error("code model 2.8 has no direct link items")

    def test_a_reply_of_another_checkout_is_an_input_error(self):
        self.source = self.repo / "elsewhere"
        self.input_error("the reply is of")

    def test_a_cache_without_the_branch_options_is_an_input_error(self):
        del self.cache["FSIM_BUILD_PYTHON"]
        self.input_error("the configure's cache has no FSIM_BUILD_PYTHON")


if __name__ == "__main__":
    unittest.main()
