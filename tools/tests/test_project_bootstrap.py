"""Bootstrap safety and reproducibility tests; no compilers or solvers run."""
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import tarfile
import time
import unittest
import zipfile
from contextlib import contextmanager
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("project_bootstrap", ROOT / "tools/bootstrap/project.py")
bootstrap = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bootstrap)



class BootstrapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (ROOT / ".local/tmp").mkdir(parents=True, exist_ok=True)

    def run_argument_binding(self, *, named_root=False, explicit_arguments=False):
        """Run real PowerShell binding with an owned entry-layer stub only."""
        shells=list(dict.fromkeys(path for name in ('pwsh','powershell') if (path:=shutil.which(name))))
        if not shells:self.skipTest('PowerShell is required for actual script argument binding')
        quote=lambda value:"'"+str(value).replace("'","''")+"'"
        with tempfile.TemporaryDirectory(prefix='run-binding-',dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve()
            script_root=root/'script root';script_root.mkdir()
            shutil.copyfile(ROOT/'run.ps1',script_root/'run.ps1')
            entry=script_root/'tools/bootstrap/entry.ps1';entry.parent.mkdir(parents=True)
            entry.write_text(r'''function Initialize-ProjectBootstrap([string]$ProjectRoot) {
    if (-not (Test-Path -LiteralPath (Join-Path $ProjectRoot 'CMakeLists.txt') -PathType Leaf)) {
        throw 'ProjectRoot must contain a complete project checkout; missing CMakeLists.txt.'
    }
    $script:ObservedProjectRoot = [IO.Path]::GetFullPath($ProjectRoot)
}
function Get-ManifestPython { return 'manufactured-python-not-executed' }
function Invoke-ProjectBootstrap([string]$Program, [string[]]$Values) {
    $record = @{ project_root = $script:ObservedProjectRoot; program = $Program; arguments = @($Values) }
    [Console]::WriteLine(($record | ConvertTo-Json -Compress))
    return 0
}
''',encoding='utf-8')
            (script_root/'CMakeLists.txt').write_text('# Manufactured project marker; never configured')
            selected=root/'explicit project root' if named_root else script_root
            if named_root:
                selected.mkdir();(selected/'CMakeLists.txt').write_text('# Explicit root marker only')
            target=root/'scripts with spaces'/'diagnostic script.py'
            target.parent.mkdir();target.write_text('# Forwarding identity only; never executed')
            forwarded=[str(target),'--literal-flag','value with spaces',"literal 'quote'",'-7']
            invocation='& '+quote(script_root/'run.ps1')+' -Mode python -Preset ucrt64-headless'
            if named_root:invocation+=' -ProjectRoot '+quote(selected)+' -Background'
            if explicit_arguments:invocation+=' -Arguments @('+','.join(map(quote,forwarded))+')'
            else:invocation+=' '+' '.join(map(quote,forwarded))
            launcher=root/'invoke.ps1';launcher.write_text(invocation+'\n',encoding='utf-8')
            for shell in shells:
                with self.subTest(shell=shell,named_root=named_root,explicit_arguments=explicit_arguments):
                    env=dict(os.environ,TMP=str(ROOT/'.local/tmp'),TEMP=str(ROOT/'.local/tmp'))
                    completed=subprocess.run([shell,'-NoLogo','-NoProfile','-NonInteractive',
                        '-ExecutionPolicy','Bypass','-File',str(launcher)],cwd=root,env=env,
                        capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=20)
                    self.assertEqual(completed.returncode,0,completed.stdout+completed.stderr)
                    record=json.loads(completed.stdout.strip())
                    self.assertEqual(Path(record['project_root']),selected)
                    self.assertEqual(record['program'],'manufactured-python-not-executed')
                    expected=['run','--mode','python','--preset','ucrt64-headless']
                    if named_root:expected.append('--background')
                    self.assertEqual(record['arguments'],expected+['--']+forwarded)

    def test_run_positional_script_and_literal_arguments_do_not_bind_project_root(self):
        self.run_argument_binding()

    def test_run_explicit_project_root_and_named_flags_preserve_positional_arguments(self):
        self.run_argument_binding(named_root=True)

    def test_run_explicit_arguments_array_preserves_existing_named_contract(self):
        self.run_argument_binding(explicit_arguments=True)


    def test_all_write_paths_stay_inside_project(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root = Path(temporary)
            self.assertEqual(bootstrap.inside(root, ".local/tmp"), root.resolve() / ".local/tmp")
            for value in ("../escape", ".local/../../escape"):
                with self.assertRaises(ValueError):
                    bootstrap.inside(root, value)

    def test_missing_pinned_vendor_source_is_installed_and_then_reused_offline(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root=Path(temporary);archive=root/'fixture.cpp';archive.write_bytes(b'// upstream\n')
            digest=bootstrap.sha256(archive)
            item=dict(name='required-vendor-source',version='1.0',destination='third_party/vendor/src/bin/bin.cpp',
                      filename='source.cpp',sha256=digest,size=archive.stat().st_size,url='https://example.invalid/pinned')
            with patch.object(bootstrap,'fetch',return_value=archive) as fetched:
                records=bootstrap.install_sources(root,{'source_files':[item]},False)
                fetched.assert_called_once()
            self.assertEqual((root/item['destination']).read_bytes(),archive.read_bytes())
            self.assertEqual(records[0]['sha256'],digest)
            with patch.object(bootstrap,'fetch',side_effect=AssertionError('Matching sources must be reused')):
                self.assertEqual(bootstrap.install_sources(root,{'source_files':[item]},True),records)

    def test_changed_vendor_source_is_preserved_and_fails_explicitly(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root=Path(temporary);target=root/'vendor/source.cpp';target.parent.mkdir();target.write_bytes(b'local change')
            item=dict(name='source',destination='vendor/source.cpp',sha256='0'*64)
            with patch.object(bootstrap,'fetch',side_effect=AssertionError('Do not overwrite authored changes')):
                with self.assertRaisesRegex(ValueError,'source.*changed'):
                    bootstrap.install_sources(root,{'source_files':[item]},False)
            self.assertEqual(target.read_bytes(),b'local change')

    def test_only_locked_alternate_vendor_bytes_are_reused(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root=Path(temporary);target=root/'source.cpp';target.write_bytes(b'// upstream\r\n')
            digest=bootstrap.sha256(target)
            item=dict(name='source',destination='source.cpp',sha256='0'*64,accepted_sha256=[digest])
            with patch.object(bootstrap,'fetch',side_effect=AssertionError('Approved CRLF source must be reused')):
                records=bootstrap.install_sources(root,{'source_files':[item]},True)
            self.assertEqual(records[0]['sha256'],digest)

    def test_archive_path_traversal_and_symlinks_are_rejected(self):
        for name in ("../evil", "/absolute", "C:/evil", "ucrt64/../../evil", "ucrt64\\..\\evil"):
            with self.assertRaises(ValueError, msg=name):
                bootstrap.safe_member(name)
        self.assertEqual(bootstrap.safe_member("ucrt64/bin/gcc.exe"), "ucrt64/bin/gcc.exe")

    def test_process_pid_reuse_and_foreign_root_never_match(self):
        row = {"pid": 123, "created": 987, "executable": "D:/work/python.exe", "root": "D:/project", "token": "a" * 32}
        live = {"pid": 123, "created": 987, "executable": "d:/work/PYTHON.exe"}
        self.assertTrue(bootstrap.identity_matches(row, live, "D:/project"))
        self.assertFalse(bootstrap.identity_matches(row, dict(live, created=988), "D:/project"))
        self.assertFalse(bootstrap.identity_matches(row, live, "D:/other"))
        self.assertFalse(bootstrap.identity_matches(row, dict(live, executable="D:/other/python.exe"), "D:/project"))

    def test_lock_has_pinned_hashes_and_transitive_native_closure(self):
        lock = bootstrap.load_lock(ROOT)
        self.assertEqual(lock["python"]["version"], "3.13.12")
        self.assertGreaterEqual(len(lock["native_packages"]), 15)
        names = {item["name"] for item in lock["native_packages"]}
        for item in lock["native_packages"]:
            self.assertRegex(item["sha256"], r"^[0-9a-f]{64}$")
            self.assertTrue(item["url"].startswith("https://repo.msys2.org/mingw/ucrt64/"))
            self.assertTrue(set(item["resolved_dependencies"]).issubset(names))
        for wheel in lock["python_packages"]:
            self.assertRegex(wheel["sha256"], r"^[0-9a-f]{64}$")
            self.assertIn(".whl", wheel["filename"])

    def test_manifest_rejects_relocation_and_changed_tool(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root = Path(temporary)
            executable = root / "python.exe"
            executable.write_bytes(b"original")
            manifest = {"schema": 1, "project_root": str(root), "tools": {"python": {"path": str(executable), "sha256": bootstrap.sha256(executable)}}}
            bootstrap.validate_manifest(root, manifest)
            executable.write_bytes(b"changed")
            with self.assertRaises(ValueError):
                bootstrap.validate_manifest(root, manifest)
            manifest["project_root"] = str(root / "other")
            with self.assertRaises(ValueError):
                bootstrap.validate_manifest(root, manifest)

    def test_build_uses_explicit_local_toolchain_and_temp(self):
        root = ROOT
        manifest = {"tools": {"python": {"path": "X:/existing/python.exe"}, "cmake": {"path": "X:/ucrt64/bin/cmake.exe"}, "ninja": {"path": "X:/ucrt64/bin/ninja.exe"}}, "ucrt64_root": "X:/ucrt64"}
        args = bootstrap.configure_arguments(root, manifest, "ucrt64-release")
        self.assertIn("-DFSIM_UCRT64_ROOT=X:/ucrt64", args)
        self.assertIn("-DFSIM_PYTHON_EXECUTABLE=X:/existing/python.exe", args)
        self.assertIn("-DFSIM_WITH_RENDER=ON", args)
        env = bootstrap.project_environment(root, manifest, {})
        for key in ("PIP_CACHE_DIR", "MPLCONFIGDIR", "XDG_CACHE_HOME"):
            self.assertTrue(Path(env[key]).is_relative_to(root))
        for key in ("TMP", "TEMP", "TORCH_EXTENSIONS_DIR"):
            self.assertEqual(env[key], "D:/ENV/tmp")


    def test_model_enumeration_is_generic_and_never_enters_excluded_directory(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root = Path(temporary)
            for name in ("alpha", "bravo"):
                folder = root / "aircraft" / name
                folder.mkdir(parents=True)
                (folder / (name + ".toml")).write_text("[aircraft]\n")
            class ExcludedEntry:
                name='.codex'
                def is_dir(self):raise AssertionError('Excluded entry was inspected')
            entries=list((root/'aircraft').iterdir())+[ExcludedEntry()]
            # Use an inert directory entry; never create or inspect an actual
            # excluded directory, including inside a disposable fixture.
            with patch.object(Path,'iterdir',return_value=iter(entries)):
                self.assertEqual(bootstrap.discover_models(root), ["alpha", "bravo"])
            with self.assertRaises(ValueError):
                bootstrap.select_models(root, ["missing"])





    def test_local_python_overlay_precedes_shared_environment(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root = Path(temporary)
            info = root / ".local/packages/pip-99.0.dist-info"
            info.mkdir(parents=True)
            (info / "METADATA").write_text("Metadata-Version: 2.1\nName: pip\nVersion: 99.0\n", encoding="utf-8")
            self.assertEqual(bootstrap.package_versions(root)["pip"], "99.0")

    def test_gmsh_native_wheel_library_is_preserved_for_real_import(self):
        with tempfile.TemporaryDirectory(dir=ROOT / ".local/tmp") as temporary:
            root = Path(temporary)
            archive = root / "gmsh.whl"
            with zipfile.ZipFile(archive, "w") as wheel:
                wheel.writestr("gmsh-4.15.0.data/data/lib/gmsh-4.15.dll", b"native-library")
            target = bootstrap.install_gmsh_library(root, archive)
            self.assertEqual(target.read_bytes(), b"native-library")
            self.assertEqual(target.parent, root.resolve() / ".local/packages")

    def test_job_completion_waits_for_all_owned_processes(self):
        active = iter((3, 1, 0))
        observed = []
        class Api:
            def QueryInformationJobObject(self, job, kind, pointer, size, returned):
                observed.append(next(active))
                bootstrap.ctypes.cast(pointer, bootstrap.ctypes.POINTER(bootstrap._JobAccounting)).contents.ActiveProcesses = observed[-1]
                return True
        with patch.object(bootstrap.time, "sleep") as sleep:
            bootstrap._wait_job_empty(Api(), 123)
        self.assertEqual(observed, [3, 1, 0])
        self.assertEqual(sleep.call_count, 2)

    def test_job_completion_timeout_cannot_report_success(self):
        class Api:
            def QueryInformationJobObject(self, job, kind, pointer, size, returned):
                bootstrap.ctypes.cast(pointer, bootstrap.ctypes.POINTER(bootstrap._JobAccounting)).contents.ActiveProcesses = 1
                return True
        with self.assertRaisesRegex(TimeoutError, "active processes"):
            bootstrap._wait_job_empty(Api(), 123, timeout=0)

    def test_job_completion_query_failure_cannot_report_success(self):
        class Api:
            def QueryInformationJobObject(self, *args):
                return False
        with self.assertRaisesRegex(OSError, "verify owned project job"):
            bootstrap._wait_job_empty(Api(), 123)

    @unittest.skipUnless(os.name == "nt", "Windows job-object ownership contract")
    def test_stop_reaps_owned_descendants_and_preserves_unregistered_process(self):
        with tempfile.TemporaryDirectory(prefix="bootstrap isolation ", dir=ROOT / ".local/tmp") as temporary:
            root = Path(temporary).resolve()
            script = root / "tools/bootstrap/project.py"
            script.parent.mkdir(parents=True)
            shutil.copy2(ROOT / "tools/bootstrap/project.py", script)
            pidfile = root / "owned-pids.json"
            child_code = "import time; time.sleep(120)"
            parent_code = "import subprocess,sys,os,time,json,pathlib; child=subprocess.Popen([sys.executable,'-c'," + repr(child_code) + "]); pathlib.Path(" + repr(str(pidfile)) + ").write_text(json.dumps([os.getpid(),child.pid])); time.sleep(120)"
            manifest = {"ucrt64_root": str(root / "toolchain"), "tools": {"python": {"path": sys.executable}}}
            foreign = subprocess.Popen([sys.executable, "-B", "-c", child_code], creationflags=subprocess.CREATE_NO_WINDOW)
            try:
                self.assertEqual(bootstrap.launch(root, manifest, "test", [[sys.executable, "-B", "-c", parent_code]], True), 0)
                self.assertEqual(len(list((root / ".local/state/processes").glob("*.json"))), 1,
                                 "Background launch returned before stop could identify its owner.")
                deadline = time.monotonic() + 10
                while not pidfile.is_file() and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertTrue(pidfile.is_file(), "Registered worker did not launch its child tree.")
                pids = json.loads(pidfile.read_text())
                records = list((root / ".local/state/processes").glob("*.json"))
                original = json.loads(records[0].read_text())
                forged = dict(original, **bootstrap.process_identity(foreign.pid))
                bootstrap.write_json(root / ".local/state/processes/000-forged.json", forged)
                self.assertEqual(bootstrap.stop(root), 0)
                deadline = time.monotonic() + 5
                while any(bootstrap.process_running(pid) for pid in pids) and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertFalse(any(bootstrap.process_running(pid) for pid in pids))
                self.assertIsNone(foreign.poll(), "stop killed an unregistered process using the same interpreter.")
            finally:
                if foreign.poll() is None:
                    foreign.terminate()
                foreign.wait(timeout=5)

    @unittest.skipUnless(os.name == "nt", "Windows job-object forced-worker-exit contract")
    def test_forced_worker_exit_reaps_its_child_and_grandchild_only(self):
        with tempfile.TemporaryDirectory(prefix="bootstrap forced exit ",dir=ROOT/".local/tmp") as temporary:
            root=Path(temporary).resolve();script=root/'tools/bootstrap/project.py'
            script.parent.mkdir(parents=True);shutil.copy2(ROOT/'tools/bootstrap/project.py',script)
            pidfile=root/'owned-pids.json';record=None;owner=None;owned_handles=[]
            child_code='import time; time.sleep(120)'
            parent_code=('import subprocess,sys,os,time,json,pathlib; child=subprocess.Popen([sys.executable,"-c",'
                +repr(child_code)+']); pathlib.Path('+repr(str(pidfile))
                +').write_text(json.dumps([os.getpid(),child.pid])); time.sleep(120)')
            manifest=dict(ucrt64_root=str(root/'toolchain'),tools=dict(python=dict(path=sys.executable)))
            foreign=subprocess.Popen([sys.executable,'-B','-c',child_code],creationflags=subprocess.CREATE_NO_WINDOW)
            try:
                bootstrap.launch(root,manifest,'test',[[sys.executable,'-B','-c',parent_code]],True)
                records=list((root/'.local/state/processes').glob('*.json'))
                self.assertEqual(len(records),1);record=json.loads(records[0].read_text())
                deadline=time.monotonic()+10
                while not pidfile.is_file() and time.monotonic()<deadline:time.sleep(.02)
                self.assertTrue(pidfile.is_file(),'Owned child tree did not start')
                pids=json.loads(pidfile.read_text())
                api=bootstrap.windows_api()
                api.WaitForSingleObject.argtypes=[bootstrap.wintypes.HANDLE,bootstrap.wintypes.DWORD]
                api.TerminateProcess.argtypes=[bootstrap.wintypes.HANDLE,bootstrap.wintypes.UINT]
                for pid in pids:
                    handle=api.OpenProcess(0x101001,False,pid) # SYNCHRONIZE | QUERY_LIMITED_INFORMATION | TERMINATE
                    self.assertTrue(handle,'Cannot retain an exact fixture-child process handle')
                    owned_handles.append(handle)
                owner=next(p for p in bootstrap._BACKGROUND_CHILDREN if p.pid==record['pid'])
                self.assertTrue(bootstrap.identity_matches(record,bootstrap.process_identity(owner.pid),root))
                owner.terminate();owner.wait(timeout=5)
                deadline=time.monotonic()+5
                while any(bootstrap.process_running(pid) for pid in pids) and time.monotonic()<deadline:time.sleep(.02)
                self.assertFalse(any(bootstrap.process_running(pid) for pid in pids),
                    'Forced worker exit left its child/grandchild running')
                self.assertIsNone(foreign.poll(),'Unregistered interpreter was terminated')
                self.assertEqual(bootstrap.stop(root),0)
                self.assertIsNone(foreign.poll(),'Stale owned-worker record affected a foreign process')
            finally:
                # RED runs need cleanup through the exact test-owned named job;
                # never target a PID/name that was not created by this fixture.
                if record and record['job']==bootstrap.job_name(root,record['token']):
                    job_api=bootstrap.windows_api();job=job_api.OpenJobObjectW(0x000C,False,record['job'])
                    if job:
                        try:job_api.TerminateJobObject(job,130)
                        finally:job_api.CloseHandle(job)
                for handle in owned_handles:
                    try:
                        api.TerminateProcess(handle,130)
                        self.assertEqual(api.WaitForSingleObject(handle,5000),0,'Fixture child cleanup timed out')
                    finally:api.CloseHandle(handle)
                if owner is not None:
                    owner.wait(timeout=5)
                    if owner in bootstrap._BACKGROUND_CHILDREN:bootstrap._BACKGROUND_CHILDREN.remove(owner)
                if foreign.poll() is None:foreign.terminate()
                foreign.wait(timeout=5)

    @unittest.skipUnless(os.name == "nt", "Windows job-object foreground exit-status contract")
    def test_foreground_job_preserves_success_and_failure_exit_status(self):
        with tempfile.TemporaryDirectory(prefix="bootstrap foreground ",dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve();script=root/'tools/bootstrap/project.py'
            script.parent.mkdir(parents=True);shutil.copy2(ROOT/'tools/bootstrap/project.py',script)
            manifest=dict(ucrt64_root=str(root/'toolchain'),tools=dict(python=dict(path=sys.executable)))
            for code in (0,7):
                with self.subTest(exit_code=code):
                    status=bootstrap.launch(root,manifest,'test',[[sys.executable,'-B','-c',f'raise SystemExit({code})']])
                    self.assertEqual(status,code)
                    record=json.loads((root/'.local/state/last-test.json').read_text())
                    self.assertEqual(record['exit_code'],code)
                    self.assertEqual(record['state'],'completed' if code==0 else 'failed')


class CMakeArgumentPaths(unittest.TestCase):
    def test_backslash_native_root_is_normalized_only_for_cmake_environment(self):
        manifest=dict(ucrt64_root=r'X:\locked\ucrt64',tools=dict(python=dict(path=r'X:\python\python.exe')))
        environment=bootstrap.project_environment(ROOT,manifest,{})
        self.assertEqual(environment['FSIM_UCRT64_ROOT'],'X:/locked/ucrt64')
        self.assertEqual(environment['PATH'].split(os.pathsep)[0],str(Path(manifest['ucrt64_root'])/'bin'))
        self.assertEqual(environment['FSIM_PYTHON_EXECUTABLE'],manifest['tools']['python']['path'])

    def test_all_cmake_definitions_use_posix_paths_but_tool_commands_keep_native_paths(self):
        from types import SimpleNamespace
        manifest=dict(ucrt64_root=r'X:\locked\ucrt64',tools={name:dict(path=str(Path(r'X:\tools')/(name+'.exe')))
            for name in ('python','cmake','ninja','ctest')})
        arguments=SimpleNamespace(jobs=2,test=False,model=['all'])
        commands=bootstrap.build_commands(ROOT,manifest,arguments)
        self.assertTrue(any(command[0]==manifest['tools']['cmake']['path'] for command in commands))
        fields=('CMAKE_TOOLCHAIN_FILE','FSIM_UCRT64_ROOT','CMAKE_MAKE_PROGRAM','FSIM_PYTHON_EXECUTABLE',
                'CMAKE_PREFIX_PATH','FETCHCONTENT_BASE_DIR','FETCHCONTENT_SOURCE_DIR_CATCH2','FSIM_DEPS_INSTALL_PREFIX')
        definitions=[value for command in commands for value in command if value.startswith(tuple('-D'+key+'=' for key in fields))]
        self.assertGreater(len(definitions),10)
        self.assertTrue(all('\\' not in value for value in definitions),definitions)
        self.assertTrue(any(value=='-DFSIM_UCRT64_ROOT=X:/locked/ucrt64' for value in definitions))

    def test_normal_build_has_hangar_and_no_external_solver(self):
        with tempfile.TemporaryDirectory(dir=ROOT / '.local/tmp') as directory:
            root=Path(directory)
            (root/'aircraft/f16c').mkdir(parents=True)
            (root/'aircraft/f16c/f16c.toml').write_text('[aircraft]\nname="f16c"\n')
            manifest=dict(ucrt64_root='X:/ucrt64',tools={name:dict(path='X:/tools/'+name+'.exe') for name in ('python','cmake','ninja','ctest')})
            from types import SimpleNamespace
            commands=bootstrap.build_commands(root,manifest,SimpleNamespace(model=['f16c'],jobs=2,memory_limit_mib=2048,test=False))
            flattened=' '.join(str(x) for command in commands for x in command)
            self.assertIn('hangar.fleet_build',flattened)
            self.assertNotIn('SU2',flattened)
            self.assertNotIn('hangar.cfd',flattened)


class GeometryArchiveProvisioning(unittest.TestCase):
    def test_missing_headers_are_repaired_and_existing_headers_are_reused(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary);archive=root/'geometry.tar.xz'
            with tarfile.open(archive,'w:xz') as stream:
                for name,contents in [('geometry/include/required.h',b'required'),('geometry/include/other.h',b'other')]:
                    member=tarfile.TarInfo(name);member.size=len(contents);stream.addfile(member,io.BytesIO(contents))
            item=dict(name='manufactured',sha256=bootstrap.sha256(archive),destination='.local/geometry/kernel',
                      archive_root='geometry',format='xz',required='include/required.h')
            with patch.object(bootstrap,'fetch',return_value=archive) as fetch:
                result=bootstrap.install_geometry(root,dict(geometry_archives=[item]),True)
                self.assertEqual(fetch.call_count,1)
                second=bootstrap.install_geometry(root,dict(geometry_archives=[item]),True)
                self.assertEqual(fetch.call_count,1);self.assertEqual(result,second)
                required=root/item['destination']/'include/required.h'
                other=required.with_name('other.h');stamp=other.stat().st_mtime_ns
                required.unlink()
                bootstrap.install_geometry(root,dict(geometry_archives=[item]),True)
                self.assertEqual(fetch.call_count,2);self.assertEqual(required.read_bytes(),b'required')
                self.assertEqual(other.stat().st_mtime_ns,stamp)

    def test_archive_member_cannot_leave_the_project_owned_prefix(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary);archive=root/'bad.tar.xz'
            with tarfile.open(archive,'w:xz') as stream:
                member=tarfile.TarInfo('geometry/../../outside.h');member.size=1;stream.addfile(member,io.BytesIO(b'x'))
            item=dict(name='manufactured',sha256=bootstrap.sha256(archive),destination='.local/geometry/kernel',
                      archive_root='geometry',format='xz',required='include/required.h')
            with patch.object(bootstrap,'fetch',return_value=archive),self.assertRaises(ValueError):
                bootstrap.install_geometry(root,dict(geometry_archives=[item]),True)
            self.assertFalse((root/'outside.h').exists())


class NativePackageRepair(unittest.TestCase):
    """Synthetic payloads only; no executable, installer or network is run."""
    def setUp(self):
        (ROOT/'.local/tmp').mkdir(parents=True,exist_ok=True)
        self.temporary=tempfile.TemporaryDirectory(prefix='native-repair-',dir=ROOT/'.local/tmp')
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name).resolve()
        self.payloads={'gcc':{'ucrt64/bin/gcc.exe':b'not executable gcc','ucrt64/include/compiler.h':b'header'},
            'cmake':{'ucrt64/bin/cmake.exe':b'not executable cmake','ucrt64/bin/ctest.exe':b'not executable ctest'},
            'runtime':{'ucrt64/bin/libfixture.dll':b'not executable runtime'}}
        self.items=[]
        for name,payload in self.payloads.items():
            archive=self.archive(name,payload)
            self.items.append(dict(name=name,version='1-1',filename=archive.name,sha256=bootstrap.sha256(archive),
                size=archive.stat().st_size,url='https://example.invalid/'+archive.name,resolved_dependencies=[]))
        self.lock={'native_packages':self.items}
        self.lock_path=self.root/'tools/bootstrap/dependencies.lock.json'
        self.lock_path.parent.mkdir(parents=True);self.lock_path.write_text(json.dumps(self.lock))
        self.local=self.root/'.local/msys2';self.marker=self.local/'bootstrap-packages.json'

    def archive(self,name,payload,links=()):
        sys.path.insert(0,str(ROOT/'.local/packages'))
        try:import zstandard
        finally:sys.path.pop(0)
        content=io.BytesIO()
        with tarfile.open(fileobj=content,mode='w') as stream:
            for path,data in {'.PKGINFO':b'pkgname = fixture\n',**payload}.items():
                member=tarfile.TarInfo(path);member.size=len(data);stream.addfile(member,io.BytesIO(data))
            for path,target in links:
                member=tarfile.TarInfo(path);member.type=tarfile.SYMTYPE;member.linkname=target;stream.addfile(member)
        destination=self.root/'.local/cache/archives'/(name+'.pkg.tar.zst')
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(zstandard.ZstdCompressor().compress(content.getvalue()))
        return destination

    def populated(self,base):
        for payload in self.payloads.values():
            for name,data in payload.items():
                path=base/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)

    def record(self,schema=2):
        self.populated(self.local)
        packages=[dict(name=item['name'],version=item['version'],archive_sha256=item['sha256'],
                       files=sorted(self.payloads[item['name']])) for item in self.items]
        if schema==1:
            packages=[dict(name=item['name'],version=item['version'],action='downloaded-pinned',sha256=item['sha256'])
                      for item in self.items]
        bootstrap.write_json(self.marker,dict(schema=schema,lock_sha256=bootstrap.sha256(self.lock_path),packages=packages))

    def fingerprint(self,base):
        return {str(path.relative_to(base)):(path.read_bytes(),path.stat().st_mtime_ns)
                for path in base.rglob('*') if path.is_file()}

    def repair(self):
        with patch.object(bootstrap,'native_candidates',return_value=[]), \
                patch.object(bootstrap.urllib.request,'urlopen',side_effect=AssertionError('No network in fixture')), \
                patch.object(bootstrap,'fetch',wraps=bootstrap.fetch) as fetched:
            result=bootstrap.install_native(self.root,self.lock,None,True)
        return result,[call.args[1]['name'] for call in fetched.call_args_list]

    def test_matching_local_record_repairs_only_missing_cmake(self):
        self.record();(self.local/'ucrt64/bin/cmake.exe').unlink();before=self.fingerprint(self.local/'ucrt64')
        result,fetched=self.repair()
        self.assertEqual((self.local/'ucrt64/bin/cmake.exe').read_bytes(),self.payloads['cmake']['ucrt64/bin/cmake.exe'])
        self.assertEqual(fetched,['cmake']);self.assertEqual(result[0],self.local/'ucrt64')
        after=self.fingerprint(self.local/'ucrt64')
        self.assertTrue(all(after[name]==value for name,value in before.items()))

    def test_all_tools_present_still_repairs_missing_runtime_dll(self):
        self.record();(self.local/'ucrt64/bin/libfixture.dll').unlink()
        _,fetched=self.repair()
        self.assertEqual(fetched,['runtime']);self.assertTrue((self.local/'ucrt64/bin/libfixture.dll').is_file())

    def test_missing_gcc_does_not_reinstall_other_complete_packages(self):
        self.record();(self.local/'ucrt64/bin/gcc.exe').unlink();before=self.fingerprint(self.local/'ucrt64')
        _,fetched=self.repair();self.assertEqual(fetched,['gcc'])
        self.assertTrue(all(self.fingerprint(self.local/'ucrt64')[name]==value for name,value in before.items()))

    def test_complete_local_rerun_performs_no_installation_work(self):
        self.record();before=self.fingerprint(self.local/'ucrt64')
        with patch.object(bootstrap,'native_candidates',side_effect=AssertionError('No candidate scan')), \
                patch.object(bootstrap,'fetch',side_effect=AssertionError('No fetch')), \
                patch.object(bootstrap,'extract_native_package',side_effect=AssertionError('No extract')), \
                patch.object(bootstrap,'copy_installed_package',side_effect=AssertionError('No copy')):
            bootstrap.install_native(self.root,self.lock,None,True)
        self.assertEqual(before,self.fingerprint(self.local/'ucrt64'))

    def add_unrelated_source_pin(self):
        self.lock['source_files']=[dict(name='required-license',sha256='1'*64)]
        self.lock_path.write_text(json.dumps(self.lock))

    def test_schema2_unrelated_lock_update_reuses_complete_native_inventory(self):
        self.record();before=self.fingerprint(self.local/'ucrt64');self.add_unrelated_source_pin()
        with patch.object(bootstrap,'native_candidates',side_effect=AssertionError('No candidate scan')), \
                patch.object(bootstrap,'fetch',side_effect=AssertionError('No fetch')), \
                patch.object(bootstrap,'extract_native_package',side_effect=AssertionError('No extract')), \
                patch.object(bootstrap,'copy_installed_package',side_effect=AssertionError('No copy')):
            bootstrap.install_native(self.root,self.lock,None,True)
        self.assertEqual(before,self.fingerprint(self.local/'ucrt64'))
        self.assertEqual(json.loads(self.marker.read_text())['lock_sha256'],bootstrap.sha256(self.lock_path))

    def test_schema2_unrelated_lock_update_still_repairs_only_missing_payload(self):
        self.record();self.add_unrelated_source_pin();(self.local/'ucrt64/bin/libfixture.dll').unlink()
        before=self.fingerprint(self.local/'ucrt64');_,fetched=self.repair()
        self.assertEqual(fetched,['runtime'])
        self.assertTrue(all(self.fingerprint(self.local/'ucrt64')[name]==value for name,value in before.items()))
        self.assertEqual(json.loads(self.marker.read_text())['lock_sha256'],bootstrap.sha256(self.lock_path))

    def test_changed_native_identity_rejects_lock_evolution_before_writes(self):
        for kind in ('roster','version','archive'):
            self.record();old=self.marker.read_bytes();before=self.fingerprint(self.local/'ucrt64')
            changed=json.loads(json.dumps(self.lock))
            if kind=='roster':changed['native_packages']=changed['native_packages'][:-1]
            elif kind=='version':changed['native_packages'][0]['version']='2-1'
            else:changed['native_packages'][0]['sha256']='2'*64
            self.lock_path.write_text(json.dumps(changed))
            with self.subTest(kind=kind),patch.object(bootstrap,'fetch',side_effect=AssertionError('No fetch')):
                with self.assertRaises(ValueError):bootstrap.install_native(self.root,changed,None,True)
            self.assertEqual(self.marker.read_bytes(),old);self.assertEqual(before,self.fingerprint(self.local/'ucrt64'))
            self.lock_path.write_text(json.dumps(self.lock))

    def test_legacy_record_remains_byte_strict_after_unrelated_lock_update(self):
        self.record(schema=1);old=self.marker.read_bytes();before=self.fingerprint(self.local/'ucrt64')
        self.add_unrelated_source_pin()
        with self.assertRaises(ValueError):self.repair()
        self.assertEqual(self.marker.read_bytes(),old);self.assertEqual(before,self.fingerprint(self.local/'ucrt64'))

    def test_schema2_lock_evolution_failure_preserves_previous_marker(self):
        self.record();old=self.marker.read_bytes();self.add_unrelated_source_pin()
        (self.local/'ucrt64/bin/libfixture.dll').unlink()
        (self.root/'.local/cache/archives'/self.items[-1]['filename']).unlink()
        before=self.fingerprint(self.local/'ucrt64')
        with self.assertRaises(ValueError):self.repair()
        self.assertEqual(self.marker.read_bytes(),old);self.assertEqual(before,self.fingerprint(self.local/'ucrt64'))

    def test_schema2_malformed_previous_lock_digest_is_rejected(self):
        self.record();record=json.loads(self.marker.read_text());record['lock_sha256']='unknown'
        bootstrap.write_json(self.marker,record);old=self.marker.read_bytes()
        with self.assertRaises(ValueError):self.repair()
        self.assertEqual(self.marker.read_bytes(),old)

    def external(self):
        base=self.root/'read-only-reference/msys2';self.populated(base)
        for item in self.items:
            directory=base/'var/lib/pacman/local'/(item['name']+'-'+item['version']);directory.mkdir(parents=True)
            (directory/'desc').write_text('%NAME%\n'+item['name']+'\n\n%VERSION%\n'+item['version']+'\n')
            (directory/'files').write_text('%FILES%\n'+'\n'.join(self.payloads[item['name']])+'\n')
        return base

    def test_incomplete_external_versions_are_rejected_and_reference_is_read_only(self):
        reference=self.external();(reference/'ucrt64/bin/libfixture.dll').unlink()
        before=self.fingerprint(reference)
        self.assertFalse(bootstrap.native_complete(reference/'ucrt64',self.lock))
        with patch.object(bootstrap,'native_candidates',return_value=[str(reference/'ucrt64')]), \
                patch.object(bootstrap,'fetch',wraps=bootstrap.fetch) as fetched:
            result=bootstrap.install_native(self.root,self.lock,None,True)
        self.assertEqual(result[0],self.local/'ucrt64');self.assertTrue((result[0]/'bin/libfixture.dll').is_file())
        self.assertEqual([call.args[1]['name'] for call in fetched.call_args_list],['runtime'])
        self.assertEqual(before,self.fingerprint(reference))

    def test_complete_external_exact_versions_are_reused_without_copy(self):
        reference=self.external();before=self.fingerprint(reference)
        with patch.object(bootstrap,'native_candidates',return_value=[str(reference/'ucrt64')]), \
                patch.object(bootstrap,'fetch',side_effect=AssertionError('No fetch')):
            result=bootstrap.install_native(self.root,self.lock,None,True)
        self.assertEqual(result[0],reference/'ucrt64');self.assertFalse(self.local.exists())
        self.assertEqual(before,self.fingerprint(reference))

    def test_legacy_cached_inventory_migrates_and_repairs_missing_member(self):
        self.record(schema=1);(self.local/'ucrt64/bin/libfixture.dll').unlink()
        self.repair();record=json.loads(self.marker.read_text())
        self.assertEqual(record['schema'],2);self.assertTrue((self.local/'ucrt64/bin/libfixture.dll').is_file())
        self.assertEqual({row['name']:row['files'] for row in record['packages']},
                         {name:sorted(files) for name,files in self.payloads.items()})

    def test_unknown_legacy_inventory_offline_preserves_old_marker(self):
        self.record(schema=1);old=self.marker.read_bytes();before=self.fingerprint(self.local/'ucrt64')
        for item in self.items:(self.root/'.local/cache/archives'/item['filename']).unlink()
        with self.assertRaisesRegex(ValueError,'(?i)inventory.*gcc|gcc.*inventory'):self.repair()
        self.assertEqual(self.marker.read_bytes(),old);self.assertEqual(before,self.fingerprint(self.local/'ucrt64'))

    def test_legacy_inventory_uses_exact_database_and_repairs_local_prefix_only(self):
        reference=self.external();self.record(schema=1);(self.local/'ucrt64/bin/libfixture.dll').unlink()
        before=self.fingerprint(reference);kept=self.fingerprint(self.local/'ucrt64')
        with patch.object(bootstrap,'native_candidates',return_value=[str(reference/'ucrt64')]), \
                patch.object(bootstrap,'fetch',side_effect=AssertionError('Exact database source is available')):
            result=bootstrap.install_native(self.root,self.lock,None,True)
        self.assertEqual(result[0],self.local/'ucrt64');self.assertEqual(json.loads(self.marker.read_text())['schema'],2)
        self.assertTrue((self.local/'ucrt64/bin/libfixture.dll').is_file())
        self.assertEqual(before,self.fingerprint(reference))
        self.assertTrue(all(self.fingerprint(self.local/'ucrt64')[name]==value for name,value in kept.items()))

    def test_fresh_cached_install_records_the_complete_payload_inventory(self):
        _,fetched=self.repair();record=json.loads(self.marker.read_text())
        self.assertEqual(record['schema'],2)
        self.assertEqual({row['name']:row['files'] for row in record['packages']},
                         {name:sorted(files) for name,files in self.payloads.items()})
        self.assertEqual(set(fetched),set(self.payloads))

    def test_archive_inventory_disagreement_is_rejected_before_missing_write(self):
        self.record();record=json.loads(self.marker.read_text());record['packages'][0]['files']=['ucrt64/bin/gcc.exe']
        bootstrap.write_json(self.marker,record);old=self.marker.read_bytes();(self.local/'ucrt64/bin/gcc.exe').unlink()
        with self.assertRaises(ValueError):self.repair()
        self.assertFalse((self.local/'ucrt64/bin/gcc.exe').exists());self.assertEqual(self.marker.read_bytes(),old)

    def test_extract_preserves_every_existing_payload_file(self):
        self.populated(self.local);target=self.local/'ucrt64/bin/gcc.exe';target.write_bytes(b'preserve existing bytes')
        before=self.fingerprint(self.local)
        bootstrap.extract_native_package(self.root,self.root/'.local/cache/archives'/self.items[0]['filename'],self.local)
        self.assertTrue(all(self.fingerprint(self.local)[name]==value for name,value in before.items()))

    def test_entire_archive_and_link_graph_are_checked_before_any_write(self):
        for name,links in (('escape',[('ucrt64/bin/link','../../../outside')]),
                           ('cycle',[('ucrt64/bin/link','other'),('ucrt64/bin/other','link')])):
            archive=self.archive(name,{'ucrt64/bin/otherwise-safe.exe':b'not executable'},links)
            destination=self.root/name
            with self.subTest(name=name),self.assertRaises(ValueError):
                bootstrap.extract_native_package(self.root,archive,destination)
            self.assertFalse((destination/'ucrt64/bin/otherwise-safe.exe').exists())

    def test_existing_parent_file_conflict_is_detected_before_any_write(self):
        archive=self.archive('conflict',{'ucrt64/bin/otherwise-safe.exe':b'new','ucrt64/conflict/child':b'new'})
        destination=self.root/'conflict-prefix';existing=destination/'ucrt64/conflict'
        existing.parent.mkdir(parents=True);existing.write_bytes(b'preserve parent file')
        with self.assertRaises(ValueError):bootstrap.extract_native_package(self.root,archive,destination)
        self.assertFalse((destination/'ucrt64/bin/otherwise-safe.exe').exists())
        self.assertEqual(existing.read_bytes(),b'preserve parent file')

    def test_archive_payload_ancestor_conflicts_are_rejected_before_any_write(self):
        for label,parent,link in (('regular','parent',False),('case-alias','PARENT',False),('link','PARENT',True)):
            payload={'ucrt64/bin/aaa-safe.exe':b'new','ucrt64/bin/parent/child':b'new'}
            links=[]
            if link:
                payload['ucrt64/bin/source']=b'new';links=[('ucrt64/bin/'+parent,'source')]
            else:payload['ucrt64/bin/'+parent]=b'new'
            archive=self.archive(label,payload,links);destination=self.root/(label+'-prefix');destination.mkdir()
            marker=destination/'bootstrap-packages.json';marker.write_bytes(b'preserve previous marker')
            error=None
            try:bootstrap.extract_native_package(self.root,archive,destination)
            except (ValueError,OSError) as caught:error=caught
            with self.subTest(label=label):
                self.assertFalse((destination/'ucrt64/bin/aaa-safe.exe').exists())
                self.assertIsInstance(error,ValueError)
                self.assertEqual(marker.read_bytes(),b'preserve previous marker')

    def test_safe_relative_package_links_are_materialized_without_overwriting_existing(self):
        archive=self.archive('valid-links',{'ucrt64/lib/target':b'package bytes'},
                             [('ucrt64/bin/first','../lib/target'),('ucrt64/bin/second','first')])
        destination=self.root/'valid-links';existing=destination/'ucrt64/bin/first'
        existing.parent.mkdir(parents=True);existing.write_bytes(b'preserve existing link copy')
        bootstrap.extract_native_package(self.root,archive,destination)
        self.assertEqual(existing.read_bytes(),b'preserve existing link copy')
        self.assertEqual((destination/'ucrt64/bin/second').read_bytes(),b'package bytes')

    def test_failed_later_repair_never_updates_success_marker(self):
        self.record();(self.local/'ucrt64/bin/gcc.exe').unlink();(self.local/'ucrt64/bin/libfixture.dll').unlink()
        archive=self.archive('runtime',self.payloads['runtime'],[('ucrt64/bin/loop','loop')])
        self.items[-1].update(sha256=bootstrap.sha256(archive),size=archive.stat().st_size)
        self.lock_path.write_text(json.dumps(self.lock));record=json.loads(self.marker.read_text())
        record['lock_sha256']=bootstrap.sha256(self.lock_path);record['packages'][-1]['archive_sha256']=self.items[-1]['sha256']
        bootstrap.write_json(self.marker,record);old=self.marker.read_bytes()
        with self.assertRaises(ValueError):self.repair()
        self.assertEqual(self.marker.read_bytes(),old);self.assertFalse((self.local/'ucrt64/bin/libfixture.dll').exists())


class SourceSnapshotMetadata(unittest.TestCase):
    @contextmanager
    def exported_root(self):
        with tempfile.TemporaryDirectory(prefix='git-free-metadata-',dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve()
            for name in ('tools/bootstrap/snapshot.py','.github/workflows/ci.yml','.claude/CLAUDE.md'):
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((ROOT/name).read_bytes())
            (root/'.local/tmp').mkdir(parents=True)
            with patch.dict(globals(),ROOT=root):yield root

    def test_git_free_export_under_ignored_parent_preserves_contract_metadata(self):
        with self.exported_root():self.test_tracked_contract_metadata_is_selected_and_exported()

    def test_git_free_export_without_git_preserves_contract_metadata(self):
        with self.exported_root(),patch.object(subprocess,'check_output',side_effect=FileNotFoundError('No Git')):
            self.test_tracked_contract_metadata_is_selected_and_exported()

    def test_tracked_contract_metadata_is_selected_and_exported(self):
        with patch.dict(sys.modules,{'project':bootstrap}):
            spec=importlib.util.spec_from_file_location('metadata_snapshot',ROOT/'tools/bootstrap/snapshot.py')
            snapshot=importlib.util.module_from_spec(spec);spec.loader.exec_module(snapshot)
        command=['git','-C',str(ROOT),'ls-files','--cached','-z','--',*snapshot.SOURCE_PATHS,
                 ':(exclude,glob)**/.codex/**']
        expected={'.github/workflows/ci.yml','.claude/CLAUDE.md'}
        for name in expected:
            self.assertTrue((ROOT/name).is_file(),name)
            self.assertTrue(any(Path(name).is_relative_to(selector) for selector in snapshot.SOURCE_PATHS),name)
        try:
            git_root=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','--show-toplevel'],
                                             text=True,stderr=subprocess.DEVNULL).strip()
        except (OSError,subprocess.CalledProcessError):git_root=None
        if git_root and Path(git_root).resolve()==ROOT.resolve():
            selected=set(subprocess.check_output(command).decode('utf-8').split('\0'))
            self.assertTrue(expected<=selected,expected-selected)
        self.assertNotIn('.claude',snapshot.SOURCE_PATHS)
        self.assertIn('.claude/CLAUDE.md',snapshot.SOURCE_PATHS)
        with tempfile.TemporaryDirectory(prefix='snapshot-metadata-',dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve();destination=root/'export'
            for name in expected:
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((ROOT/name).read_bytes())
            staged=[]
            def export_git(arguments,**kwargs):
                # This fixture tests metadata selection without a repository.
                # The separate source-checkout regression exercises real Git.
                self.assertTrue(Path(kwargs['env']['GIT_INDEX_FILE']).is_relative_to(root))
                if 'add' in arguments:
                    inventory_path=next(value.split('=',1)[1] for value in arguments if value.startswith('--pathspec-from-file='))
                    staged[:]=Path(inventory_path).read_bytes().decode('utf-8').strip('\0').split('\0')
                elif 'checkout-index' in arguments:
                    prefix=Path(next(value.split('=',1)[1] for value in arguments if value.startswith('--prefix=')))
                    for name in staged:
                        target=prefix/name;target.parent.mkdir(parents=True,exist_ok=True)
                        target.write_bytes((root/name).read_bytes())
                else:self.assertIn('read-tree',arguments)
                return subprocess.CompletedProcess(arguments,0)
            with patch.object(snapshot.subprocess,'check_output',side_effect=[
                    ('\0'.join(sorted(expected))+'\0').encode(), 'manufactured-head\n']) as inventory, \
                    patch.object(snapshot.subprocess,'run',side_effect=export_git), \
                    patch('sys.stdout',new=io.StringIO()):
                record=snapshot.snapshot(root,destination)
            self.assertIn(':(exclude,glob)**/.codex/**',inventory.call_args_list[0].args[0])
            self.assertEqual({row['path'] for row in record['files']},expected)
            for name in expected:self.assertEqual((destination/name).read_bytes(),(ROOT/name).read_bytes())


class JsbsimLicenseProvisioning(unittest.TestCase):
    """Pinned metadata plus manufactured installer transport; no downloads."""
    def license_pin(self):
        matches=[item for item in bootstrap.load_lock(ROOT).get('source_files',[])
                 if item.get('destination')=='third_party/jsbsim/COPYING']
        self.assertEqual(len(matches),1,'Required JSBSim license must be provisioned for Git-free exports')
        return matches[0]

    def test_lock_pins_exact_release_license_and_only_approved_line_endings(self):
        item=self.license_pin();revision='3b25f25e49b42d0489c04ac805674fc1450ca579'
        self.assertEqual(item['version'],'1.3.1')
        self.assertEqual(item['url'],'https://raw.githubusercontent.com/JSBSim-Team/jsbsim/'+revision+'/COPYING')
        self.assertEqual(item['upstream_commit'],revision)
        self.assertEqual(item['sha256'],'1e7e6bae5a5bde32f1ae5a7c37a082d1ab03cf89354f7f936ac40be9e39a6531')
        self.assertEqual(item['accepted_sha256'],['dc626520dcd53a22f727af3ee42c770e56c97a64fe3adb063799d8ab032fe551'])
        self.assertEqual(item['size'],27032)

    def fixture_pin(self):
        # Synthetic text exercises transport without copying upstream license text into the test.
        item=dict(self.license_pin());crlf=b'Manufactured license transport\r\nSecond line\r\n';lf=crlf.replace(b'\r\n',b'\n')
        item.update(sha256=bootstrap.hashlib.sha256(crlf).hexdigest(),size=len(crlf),
                    accepted_sha256=[bootstrap.hashlib.sha256(lf).hexdigest()])
        return item,crlf,lf

    def test_missing_export_license_is_restored_from_pinned_cache_offline(self):
        item,data,_=self.fixture_pin()
        with tempfile.TemporaryDirectory(prefix='license-cache-',dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve();cache=root/'.local/cache/archives'/item['filename']
            cache.parent.mkdir(parents=True);cache.write_bytes(data)
            with patch.object(bootstrap.urllib.request,'urlopen',side_effect=AssertionError('No network')):
                records=bootstrap.install_sources(root,{'source_files':[item]},True)
            self.assertEqual((root/item['destination']).read_bytes(),data)
            self.assertEqual(records[0]['sha256'],item['sha256'])

    def test_missing_export_license_uses_exact_pinned_online_request(self):
        item,data,_=self.fixture_pin()
        with tempfile.TemporaryDirectory(prefix='license-online-stub-',dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve()
            with patch.object(bootstrap.urllib.request,'urlopen',return_value=io.BytesIO(data)) as request, \
                    patch('sys.stdout',new=io.StringIO()):
                bootstrap.install_sources(root,{'source_files':[item]},False)
            self.assertEqual(request.call_args.args[0].full_url,item['url'])
            self.assertEqual((root/item['destination']).read_bytes(),data)

    def test_existing_crlf_and_lf_license_bytes_are_preserved_without_fetch(self):
        item,crlf,lf=self.fixture_pin()
        for data in (crlf,lf):
            with self.subTest(line_ending='CRLF' if data==crlf else 'LF'), \
                    tempfile.TemporaryDirectory(prefix='license-reuse-',dir=ROOT/'.local/tmp') as temporary:
                root=Path(temporary).resolve();target=root/item['destination'];target.parent.mkdir(parents=True);target.write_bytes(data)
                before=(target.read_bytes(),target.stat().st_mtime_ns)
                with patch.object(bootstrap,'fetch',side_effect=AssertionError('No fetch')):
                    bootstrap.install_sources(root,{'source_files':[item]},True)
                self.assertEqual((target.read_bytes(),target.stat().st_mtime_ns),before)

    def test_altered_existing_license_is_preserved_and_rejected(self):
        item,_,_=self.fixture_pin()
        with tempfile.TemporaryDirectory(prefix='license-altered-',dir=ROOT/'.local/tmp') as temporary:
            root=Path(temporary).resolve();target=root/item['destination'];target.parent.mkdir(parents=True);target.write_bytes(b'local edit')
            before=(target.read_bytes(),target.stat().st_mtime_ns)
            with patch.object(bootstrap,'fetch',side_effect=AssertionError('No fetch')):
                with self.assertRaises(ValueError):bootstrap.install_sources(root,{'source_files':[item]},False)
            self.assertEqual((target.read_bytes(),target.stat().st_mtime_ns),before)


if __name__ == "__main__":
    unittest.main()
