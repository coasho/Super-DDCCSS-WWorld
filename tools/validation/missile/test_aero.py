"""Strict static-aerodynamic comparison contract; synthetic parser fixtures only."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import test_aero_history
import test_aero_mesh
import test_aero_panels
import test_aero_grid
import test_aero_vsp_frame
import test_aero_vsp_case
import test_aero_vsp_native
import test_aero_vsp_adb
import test_aero_hangar
import test_aero_schema
import types

_SPEC = importlib.util.spec_from_file_location("missile_aero", Path(__file__).with_name("aero.py"))
aero = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(aero)


def su2_fixture(cd, cl, cm, rho=-9):
    return ('"Inner_Iter","rms[Rho]","CD","CL","CMy","Cauchy[CD]","Cauchy[CL]","Cauchy[CMy]"\n' +
            "".join('%d,%g,%.17g,%.17g,%.17g,0,0,0\n' % (i, rho, cd, cl, cm) for i in range(200)))


def vsp_log(wake_counts=(5, 5), maximum=5):
    return ("NumberOfKuttaNodes after trimming: 48\nNumberOfWakeLoops: 280\n" +
            "".join("Solving... Mach: .6 ... Alpha: %g ... Beta: 0\n" % alpha +
                    "".join("Wake Iter: %d / %d ... GMRES Iter: 4 ... Red: -1.1 / -1 ... Max: .1 / 1.31\n" % (i, maximum)
                            for i in range(1, count+1)) for alpha, count in zip((0, 2), wake_counts)))


class StaticAero(unittest.TestCase):
    def setUp(self):
        # Process-orchestration tests isolate the binary parser, whose independent
        # byte fixtures are registered below and never invoke a native process.
        patch=mock.patch.object(aero,"assess_adb",side_effect=self.synthetic_adb,create=True)
        self.adb_call=patch.start()
        self.addCleanup(patch.stop)
        # These orchestration fixtures contain synthetic ADB bytes. The separate
        # delivery-derived containment suite reads actual retained native bytes.
        wake=mock.patch.object(aero,"assess_wake",return_value={"passed":True},create=True)
        self.wake_call=wake.start()
        self.addCleanup(wake.stop)

    @staticmethod
    def synthetic_adb(path,geometry,pair,**arguments):
        data=Path(path).read_bytes()
        return {"passed":not data.startswith(b"bad"),"model_accepted":False,
                "raw_files":{"adb":{"path":str(Path(path).resolve()),"sha256":aero.sha256(path)},
                             "geometry":{"path":str(Path(geometry).resolve()),"sha256":aero.sha256(geometry)}}}

    def test_explicit_initial_model_compares_both_current_solvers_without_changing_inviscid_gates(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)
            reference=dict(area_m2=.3048**2,length_m=.6096,moment_x_m=.3048)
            model=output/'initial.xml';model.write_text(test_aero_hangar.fixture())
            def process(command,directory,log,timeout=120):
                if '--mesh-worker' in command:aero.save(output/'mesh-record.json',dict(reference=reference))
                else:self.write_paired_campaign(output,reference)
                return dict(exit_code=0,status='executed')
            def cfd(args,directory,ref,alpha):
                return dict(result=dict(mach=.6,alpha_deg=alpha,wind_cd=.1,
                    coefficients=aero.body_coefficients(.1,0 if alpha==0 else .4,0 if alpha==0 else -.1,alpha)))
            argv=['aero.py','--design',str(output/'source.toml'),'--output',str(output),
                  '--initial-model',str(model)]
            with mock.patch.object(sys,'argv',argv),mock.patch.object(aero,'bounded',side_effect=process), \
                 mock.patch.object(aero,'su2_case',side_effect=cfd):
                self.assertEqual(aero.main(),0)
                result=__import__('json').loads((output/'report.json').read_text())
                self.assertTrue(result['gates']['solver_static_residuals'])
                self.assertTrue(result['gates']['initial_model_comparison_complete'])
                for label in ('su2','vspaero'):
                    row=result['initial_model_comparison']['solvers'][label]
                    self.assertEqual([(r['mach'],r['alpha_deg']) for r in row['cases']],[(.6,0),(.6,2)])
                    self.assertFalse(row['model_accepted'])
                    self.assertFalse(row['like_for_like_inviscid'])
                    self.assertGreater(row['cases'][1]['difference']['ca']['absolute'],.3)
                model.unlink()
                self.assertEqual(aero.main(),1)
                failed=__import__('json').loads((output/'report.json').read_text())
                self.assertEqual(failed['initial_model_comparison']['status'],'unavailable')
                self.assertFalse(failed['gates']['initial_model_comparison_complete'])
                self.assertNotIn('solvers',failed['initial_model_comparison'])

    def write_paired_campaign(self,output,reference,*,roll=0,maximum=(-3.5,-3.5),zero_cm=0,cl=.4,cm=-.1):
        (output/"missile.vspgeom").write_text(test_aero_panels.document())
        rows=[]
        for index,pair in enumerate(aero.paired_conditions(roll)):
            folder=output/("vsp-frame-alpha-%g"%pair["physical"]["alpha_deg"])
            folder.mkdir(exist_ok=True)
            (folder/"missile.vspgeom").write_bytes((output/"missile.vspgeom").read_bytes())
            (folder/"missile.vspaero").write_text("NumWakeNodes = 8\n")
            (folder/"missile.adb").write_bytes(b"synthetic current native case")
            data=test_aero_vsp_case.fixture(pair,bref=reference["length_m"],cref=reference["length_m"],
                 area=reference["area_m2"],cg=(reference["moment_x_m"],0,0),maximum=maximum[index])
            alpha=math.radians(pair["physical"]["alpha_deg"])
            lift=0 if index==0 else cl;moment=zero_cm if index==0 else cm
            ca=.1*math.cos(alpha)-lift*math.sin(alpha);cn=.1*math.sin(alpha)+lift*math.cos(alpha)
            angle=math.radians(roll);a=math.radians(pair["native"]["alpha_deg"]);b=math.radians(pair["native"]["beta_deg"])
            inviscid={"CFix":ca,"CFiy":-math.sin(angle)*cn,"CFiz":math.cos(angle)*cn,
                      "CMix":0.,"CMiy":math.cos(angle)*moment,"CMiz":math.sin(angle)*moment}
            inviscid.update(CLi=-ca*math.sin(a)+inviscid["CFiz"]*math.cos(a),
                CDi=(ca*math.cos(a)+inviscid["CFiz"]*math.sin(a))*math.cos(b)-inviscid["CFiy"]*math.sin(b))
            for key in ("polar","history"):
                lines=[]
                columns=test_aero_vsp_case.POLAR if key=="polar" else test_aero_vsp_case.HISTORY
                for line in data[key].splitlines():
                    fields=line.split()
                    if len(fields)==len(columns) and fields[0].replace("-","").replace(".","").isdigit():
                        values=dict(zip(columns,fields));values.update(inviscid)
                        line=" ".join("%.12f"%float(values[name]) for name in columns)
                    lines.append(line)
                (folder/("missile."+key)).write_text("\n".join(lines)+"\n")
            (folder/"solver.log").write_text(data["log"])
            row=aero.assess_case(folder/"missile.polar",folder/"missile.history",folder/"solver.log",pair,
                bref=reference["length_m"],cref=reference["length_m"],reference_area=reference["area_m2"],
                native_cg=[reference["moment_x_m"],0,0])
            row.update(geometry_sha256=aero.sha256(output/"missile.vspgeom"),
                native_config_sha256=aero.sha256(folder/"missile.vspaero"),
                wake_containment={"passed":True},
                actual_geometry=self.synthetic_adb(folder/"missile.adb",folder/"missile.vspgeom",pair))
            rows.append(row)
        aero.save(output/"vsp-frame-cases.json",{"status":"computed","result":rows,"model_accepted":False})

    def test_finite_current_adb_mutation_cannot_hide_behind_saved_worker_summary(self):
        for roll in (0.,.123):
            with self.subTest(roll=roll),tempfile.TemporaryDirectory() as folder:
                output=Path(folder);reference={"moment_x_m":1,"length_m":.5,"area_m2":.1}
                def process(command,directory,log,timeout=120):
                    if "--mesh-worker" in command:aero.save(output/"mesh-record.json",{"reference":reference})
                    else:
                        self.write_paired_campaign(output,reference,roll=roll)
                        with (output/"vsp-frame-alpha-2/missile.adb").open("ab") as stream:stream.write(b"changed finite payload")
                    return {"exit_code":0,"status":"executed"}
                def cfd(args,directory,ref,alpha):
                    return {"result":{"mach":.6,"alpha_deg":alpha,"wind_cd":.1,
                        "coefficients":aero.body_coefficients(.1,0 if alpha==0 else .4,0 if alpha==0 else -.1,alpha)}}
                argv=["aero.py","--design",str(output/"source.toml"),"--output",str(output),"--frame-roll-deg",str(roll)]
                with mock.patch.object(sys,"argv",argv),mock.patch.object(aero,"bounded",side_effect=process), \
                     mock.patch.object(aero,"su2_case",side_effect=cfd):
                    self.assertEqual(aero.main(),1)
                report=__import__("json").loads((output/"report.json").read_text())
                self.assertFalse(report["gates"]["vsp_actual_geometry"])
                self.assertIn("changed",report["runs"]["vspaero"]["reason"])

    def test_single_case_requires_one_native_wake_count_and_actual_adb(self):
        with tempfile.TemporaryDirectory() as folder:
            case=Path(folder);(case/"missile.vspgeom").write_bytes(b"geometry")
            (case/"missile.adb").write_bytes(b"current native binary")
            pair=aero.paired_conditions(0)[0];ref={"moment_x_m":1,"length_m":.5,"area_m2":.1}
            (case/"missile.vspaero").write_text("NumWakeNodes = 16\n")
            with mock.patch.object(aero,"assess_case",return_value={}):
                row=aero.assess_single_vsp_case(case,pair,ref)
                self.assertTrue(row["actual_geometry"]["passed"])
                self.assertEqual(self.adb_call.call_args.kwargs["wake_nodes"],16)
                with mock.patch.object(aero,"assess_wake",return_value={"passed":False,"failures":[{"trail":2}]}):
                    with self.assertRaisesRegex(ValueError,"containment"):
                        aero.assess_single_vsp_case(case,pair,ref)
                for config in ("missing\n","NumWakeNodes = 8\nNumWakeNodes = 16\n","NumWakeNodes = 8.0\n"):
                    (case/"missile.vspaero").write_text(config)
                    with self.subTest(config=config),self.assertRaises(ValueError):aero.assess_single_vsp_case(case,pair,ref)
                (case/"missile.vspaero").write_text("NumWakeNodes = 8\n")
                (case/"missile.adb").write_bytes(b"bad detached native wake")
                with self.assertRaises(ValueError):aero.assess_single_vsp_case(case,pair,ref)

    def test_explicit_native_panel_solver_selection_requires_the_requested_file(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            binary = output / ("vspaero.exe" if aero.os.name == "nt" else "vspaero")
            binary.write_bytes(b"synthetic source-built solver")
            native = types.SimpleNamespace(SetVSPAEROPath=mock.Mock(return_value=True),
                                           GetVSPAEROPath=lambda: str(output))
            record = aero.select_vsp_solver(native, binary, output)
            native.SetVSPAEROPath.assert_called_once_with(str(output.resolve()))
            self.assertEqual(record["sha256"], aero.sha256(binary))
            self.assertEqual(record["path"], str(binary.resolve()))
            for accepted, selected in ((False, output), (True, output / "different")):
                native.SetVSPAEROPath.return_value = accepted
                native.GetVSPAEROPath = lambda: str(selected)
                with self.subTest(accepted=accepted, selected=selected), self.assertRaises(ValueError):
                    aero.select_vsp_solver(native, binary, output)
                self.assertFalse((output / "vsp-solver-record.json").exists())
            binary.unlink()
            with self.assertRaises(ValueError):
                aero.select_vsp_solver(native, binary, output)

    def test_solver_directory_cannot_hash_a_decoy_basename(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            actual = output / ("vspaero.exe" if aero.os.name == "nt" else "vspaero")
            decoy = output / ("vspaero" if aero.os.name == "nt" else "vspaero.exe")
            actual.write_bytes(b"native platform command")
            decoy.write_bytes(b"different unexecuted decoy")
            native = types.SimpleNamespace(SetVSPAEROPath=mock.Mock(return_value=True),
                                           GetVSPAEROPath=lambda: str(output))
            with self.assertRaises(ValueError):
                aero.select_vsp_solver(native, decoy, output)
            native.SetVSPAEROPath.assert_not_called()

    def test_default_flow_requires_history_and_linear_log_to_end_at_the_same_wake(self):
        for final, expected_status in (("mismatch", 1), ("valid", 0), ("reversed", 1)):
            with self.subTest(final=final), tempfile.TemporaryDirectory() as folder:
                output = Path(folder)

                def solver(command, directory, log, timeout=120):
                    if "--mesh-worker" in command:
                        (output / "mesh.su2").write_bytes(b"synthetic mesh")
                        aero.save(output / "mesh-record.json", {"reference": {"area_m2": .1, "length_m": .1, "moment_x_m": 1}})
                    elif "-t" in command:
                        zero = "alpha-0.cfg" in command[-1]
                        (Path(directory) / "history.csv").write_text(su2_fixture(.1, 0 if zero else .4, 0 if zero else -.1))
                    else:
                        self.write_paired_campaign(output,{"area_m2":.1,"length_m":.1,"moment_x_m":1})
                        if final=="mismatch":
                            history=output/"vsp-frame-alpha-2/missile.history"
                            history.write_text("\n".join(history.read_text().splitlines()[:-1])+"\n")
                        elif final=="reversed":
                            path=output/"vsp-frame-cases.json";data=__import__("json").loads(path.read_text())
                            data["result"].reverse();aero.save(path,data)
                    return {"exit_code": 0, "status": "executed"}

                argv = ["aero.py", "--design", str(output / "design.toml"), "--output", str(output),
                        "--gmsh", "gmsh", "--openvsp", "vsp", "--su2", "su2"]
                with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=solver):
                    self.assertEqual(aero.main(), expected_status)

    def test_bounded_su2_continuation_uses_only_its_own_checkpoint(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            (output / "mesh.su2").write_bytes(b"synthetic fixed mesh")
            calls = []

            def solver(command, directory, log, timeout=120):
                directory = Path(directory)
                config = Path(command[-1]).read_text()
                calls.append(config)
                self.assertEqual((directory / "mesh.su2").read_bytes(), b"synthetic fixed mesh")
                if len(calls) == 1:
                    self.assertIn("RESTART_SOL= NO", config)
                    self.assertFalse((directory / "restart.dat").exists())
                else:
                    self.assertIn("RESTART_SOL= YES", config)
                    self.assertEqual((directory / "restart.dat").read_bytes(), b"first pass native state")
                (directory / "history.csv").write_text(su2_fixture(.1, .2, -.3, -7 if len(calls) == 1 else -9))
                (directory / "restart-out.dat").write_bytes(b"first pass native state")
                return {"exit_code": 0, "status": "executed", "seconds": 1}

            args = types.SimpleNamespace(su2=Path("solver"), iterations=2200, su2_passes=3, solver_timeout=300)
            reference = {"area_m2": .1, "length_m": .1, "moment_x_m": 1}
            with mock.patch.object(aero, "bounded", side_effect=solver):
                row = aero.su2_case(args, output, reference, 2)
            self.assertEqual(row["result"]["status"], "converged")
            self.assertEqual(row["result"]["mach"], .6)
            self.assertEqual(len(row["passes"]), 2)
            self.assertEqual(row["passes"][0]["status"], "not_converged")
            self.assertEqual(row["passes"][1]["input_restart_sha256"], aero.sha256(output / "su2-alpha-2/pass-0/restart-out.dat"))

    def test_failed_or_malformed_su2_pass_never_qualifies_for_continuation(self):
        for exit_code, invalid in ((None, False), (1, False), (0, True)):
            with self.subTest(exit_code=exit_code, invalid=invalid), tempfile.TemporaryDirectory() as folder:
                output = Path(folder)
                (output / "mesh.su2").write_bytes(b"synthetic mesh")

                def solver(command, directory, log, timeout=120):
                    directory = Path(directory)
                    data = su2_fixture(.1, .2, -.3)
                    (directory / "history.csv").write_text(data.replace("40,-9,", "40,nan,") if invalid else data)
                    (directory / "restart-out.dat").write_bytes(b"must not be reused")
                    return {"exit_code": exit_code, "status": "executed"}

                args = types.SimpleNamespace(su2=Path("solver"), iterations=2200, su2_passes=3, solver_timeout=300)
                with mock.patch.object(aero, "bounded", side_effect=solver) as run:
                    row = aero.su2_case(args, output, {"area_m2": .1, "length_m": .1, "moment_x_m": 1}, 2)
                self.assertEqual(run.call_count, 1)
                self.assertNotIn("result", row)

    def test_linear_convergence_uses_case_banners_and_all_observed_wakes(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "solver.log"
            path.write_text(vsp_log((1, 3)))
            row = aero.vsp_convergence(path)
            self.assertTrue(row["gmres_targets_met_every_wake"])
            self.assertEqual([r["wake"] for r in row["final_wake_targets"]], [1, 3])
            path.write_text(vsp_log((1, 3)).replace("Wake Iter: 2 / 5", "Wake Iter: 4 / 5"))
            self.assertFalse(aero.vsp_convergence(path)["gmres_targets_met_every_wake"])

    def test_su2_fixed_pseudotime_and_all_force_convergence_are_explicit(self):
        cfg = aero.su2_config({"area_m2": .1, "length_m": .1, "moment_x_m": 1}, 2)
        for value in ("CFL_NUMBER= 1.0", "CFL_ADAPT= NO", "CONV_RESIDUAL_MINVAL= -8",
                      "CONV_FIELD= ( RMS_DENSITY, DRAG, LIFT, MOMENT_Y )", "CONV_STARTITER= 500",
                      "CONV_CAUCHY_ELEMS= 100", "CONV_CAUCHY_EPS= 1E-10"):
            with self.subTest(value=value):
                self.assertIn(value, cfg)

    def test_worker_rejects_open_native_panels_before_flow_sweep(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            calls = []
            native = types.SimpleNamespace(SET_ALL=0, SET_NONE=-1)
            for name in ("VSPCheckSetup", "ClearVSPModel", "WriteVSPFile", "SetAnalysisInputDefaults",
                         "SetIntAnalysisInput", "SetDoubleAnalysisInput"):
                setattr(native, name, lambda *args: None)
            native.GetVSPVersion = lambda: "fixture"
            native.ErrorMgrSingleton = types.SimpleNamespace(getInstance=lambda: types.SimpleNamespace(GetNumTotalErrors=lambda: 0))

            def compute(name):
                calls.append(name)
                (output / "missile.vspgeom").write_text(test_aero_panels.document(
                    faces=[(0, 2, 1), (0, 1, 3), (0, 3, 2)]))

            native.ExecAnalysis = compute
            with mock.patch.dict(sys.modules, {"openvsp": native}), mock.patch.object(sys, "path", list(sys.path)), \
                    mock.patch.object(aero, "parametric_vsp"):
                with self.assertRaisesRegex(ValueError, "closed outward"):
                    aero.vsp_worker("unused", output, {}, Path("source.toml"))
            self.assertEqual(calls, ["VSPAEROComputeGeometry"])
            self.assertFalse(__import__("json").loads((output / "vsp-panel-record.json").read_text())["closed_oriented"])

    def paired_worker_fixture(self, output, mode="valid",roll=.123):
        state={"calls":[],"cases":[],"folder":output,"inputs":{},"errors":[]}
        native=types.SimpleNamespace(SET_ALL=0,SET_NONE=-1)
        for name in ("VSPCheckSetup","ClearVSPModel","WriteVSPFile","SetParmVal","Update"):
            setattr(native,name,lambda *args:None)
        native.GetVSPVersion=lambda:"fixture"
        def pop_error():
            message=state["errors"].pop()
            return types.SimpleNamespace(GetErrorString=lambda:message)
        manager=types.SimpleNamespace(GetNumTotalErrors=lambda:len(state["errors"]),PopLastError=pop_error)
        native.ErrorMgrSingleton=types.SimpleNamespace(getInstance=lambda:manager)
        native.SetVSP3FileName=lambda path:state.update(folder=Path(path).parent)
        native.SetAnalysisInputDefaults=lambda name:state.update(inputs={})
        def value(name,key,values):state["inputs"][key]=values[0]
        native.SetIntAnalysisInput=native.SetDoubleAnalysisInput=native.SetStringAnalysisInput=value
        def execute(name):
            state["calls"].append(name)
            if name=="VSPAEROComputeGeometry":
                (output/"missile.vspgeom").write_text(test_aero_panels.document())
                (output/"missile.vkey").write_text("native key retained verbatim\n")
                return "native geometry"
            self.assertEqual(name,"VSPAEROSweep")
            settings=state["inputs"].copy();state["cases"].append(settings)
            for key in ("AlphaNpts","BetaNpts","MachNpts"):self.assertEqual(settings[key],1)
            self.assertEqual(settings["WakeNumIter"],5)
            for key in ("WakeRelax","CoreSizeFactor","FreezeWakeAtIteration","ImplicitWake"):
                self.assertNotIn(key,settings)
            index=len(state["cases"])-1;pair=aero.paired_conditions(roll)[index]
            self.assertAlmostEqual(settings["AlphaStart"],pair["native"]["alpha_deg"])
            self.assertAlmostEqual(settings["BetaStart"],pair["native"]["beta_deg"])
            if mode=="stale" and index==1:return "native case"
            if mode=="wrong_beta" and index==1:
                pair=__import__("copy").deepcopy(pair);pair["native"]["beta_deg"]=0
            data=test_aero_vsp_case.fixture(pair,bref=.5,cref=.5,cg=(1.1,0,0) if mode=="wrong_cg" and index==1 else (1,0,0))
            if mode=="cartesian" and index==1:data["polar"]+=data["polar"]
            folder=state["folder"]
            (folder/"missile.vspaero").write_text("NumWakeNodes = 8\n")
            if mode!="missing_adb" or index!=1:
                (folder/"missile.adb").write_bytes(b"synthetic current native case")
            (folder/"missile.polar").write_text(data["polar"])
            (folder/"missile.history").write_text(data["history"])
            Path(settings["RedirectFile"]).write_text(data["log"])
            if mode=="api_error" and index==1:state["errors"].append("native case API failure")
            return "native case"
        native.ExecAnalysis=execute
        return native,state

    def test_zero_roll_worker_also_uses_two_single_cases(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);native,state=self.paired_worker_fixture(output,roll=0)
            with mock.patch.dict(sys.modules,{"openvsp":native}),mock.patch.object(sys,"path",list(sys.path)), \
                 mock.patch.object(aero,"parametric_vsp",return_value=[]):
                aero.vsp_worker("unused",output,{"moment_x_m":1,"length_m":.5,"area_m2":.1},Path("source.toml"))
            self.assertEqual(state["calls"],["VSPAEROComputeGeometry","VSPAEROSweep","VSPAEROSweep"])

    def test_stale_adb_is_removed_before_native_single_case(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);native,state=self.paired_worker_fixture(output,mode="missing_adb")
            stale=output/"vsp-frame-alpha-2";stale.mkdir();(stale/"missile.adb").write_bytes(b"old apparent valid native case")
            with mock.patch.dict(sys.modules,{"openvsp":native}),mock.patch.object(sys,"path",list(sys.path)), \
                 mock.patch.object(aero,"parametric_vsp",return_value=[{"id":"body","parent_id":"","origin_xyz":[0,0,0],"rotation_xyz":[0,0,0]}]),self.assertRaises((ValueError,OSError)):
                aero.vsp_worker("unused",output,{"moment_x_m":1,"length_m":.5,"area_m2":.1},Path("source.toml"),frame_roll=.123)
            self.assertFalse((stale/"missile.adb").exists())

    def test_rolled_geometry_uses_two_single_paired_native_executions(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            native,state=self.paired_worker_fixture(output)
            frames = [{"id": "body", "parent_id": "", "origin_xyz": [0, 0, 0], "rotation_xyz": [0, 0, 0]}]
            with mock.patch.dict(sys.modules, {"openvsp": native}), mock.patch.object(sys, "path", list(sys.path)), \
                    mock.patch.object(aero, "parametric_vsp", return_value=frames):
                aero.vsp_worker("unused",output,{"moment_x_m":1,"length_m":.5,"area_m2":.1},Path("source.toml"),frame_roll=.123)
            self.assertEqual(state["calls"],["VSPAEROComputeGeometry","VSPAEROSweep","VSPAEROSweep"])
            record = __import__("json").loads((output / "vsp-frame-record.json").read_text())
            self.assertEqual(len(record["paired_conditions"]), 2)
            cases=__import__("json").loads((output/"vsp-frame-cases.json").read_text())
            self.assertEqual(len(cases["result"]),2)
            self.assertFalse(cases["model_accepted"])
            self.assertEqual(len({row["geometry_sha256"] for row in cases["result"]}),1)

    def test_paired_orchestration_rejects_wrong_beta_extra_cases_and_stale_second_output(self):
        for mode in ("wrong_beta","cartesian","stale","wrong_cg","api_error"):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as folder:
                output=Path(folder);native,state=self.paired_worker_fixture(output,mode)
                stale=output/"vsp-frame-alpha-2";stale.mkdir()
                data=test_aero_vsp_case.fixture(aero.paired_conditions(.123)[1],bref=.5,cref=.5)
                for key,name in (("polar","missile.polar"),("history","missile.history"),("log","solver.log")):
                    (stale/name).write_text(data[key])
                frames=[{"id":"body","parent_id":"","origin_xyz":[0,0,0],"rotation_xyz":[0,0,0]}]
                with mock.patch.dict(sys.modules,{"openvsp":native}),mock.patch.object(sys,"path",list(sys.path)), \
                     mock.patch.object(aero,"parametric_vsp",return_value=frames),self.assertRaises(ValueError):
                    aero.vsp_worker("unused",output,{"moment_x_m":1,"length_m":.5,"area_m2":.1},Path("source.toml"),frame_roll=.123)
                self.assertEqual(len(state["cases"]),2)
                result=__import__("json").loads((output/"vsp-frame-cases.json").read_text())
                self.assertEqual(result["status"],"failed")
                self.assertFalse(result["model_accepted"])

    def test_normal_campaign_propagates_explicit_roll_and_reassesses_raw_single_cases(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);native,state=self.paired_worker_fixture(output)
            reference={"moment_x_m":1,"length_m":.5,"area_m2":.1}
            frames=[{"id":"body","parent_id":"","origin_xyz":[0,0,0],"rotation_xyz":[0,0,0]}]
            def process(command,directory,log,timeout=120):
                if "--mesh-worker" in command:aero.save(output/"mesh-record.json",{"reference":reference})
                else:
                    self.assertIn("--frame-roll-deg",command)
                    self.assertEqual(float(command[command.index("--frame-roll-deg")+1]),.123)
                    aero.vsp_worker("unused",output,reference,Path("source.toml"),frame_roll=.123)
                return {"exit_code":0,"status":"executed"}
            def cfd(args,directory,ref,alpha):
                return {"result":{"mach":.6,"alpha_deg":alpha,"wind_cd":.12*math.cos(math.radians(alpha))+.4*math.sin(math.radians(alpha)),
                                  "coefficients":{"ca":.12,"cn":.4,"cm":-.1}}}
            argv=["aero.py","--design",str(output/"source.toml"),"--output",str(output),"--openvsp","unused","--frame-roll-deg",".123"]
            with mock.patch.dict(sys.modules,{"openvsp":native}),mock.patch.object(sys,"path",list(sys.path)), \
                 mock.patch.object(aero,"parametric_vsp",return_value=frames),mock.patch.object(aero,"bounded",side_effect=process), \
                 mock.patch.object(aero,"su2_case",side_effect=cfd),mock.patch.object(sys,"argv",argv):
                self.assertEqual(aero.main(),1) # Deliberate zero-alpha force/pitch bias still rejects this fixture.
            report=__import__("json").loads((output/"report.json").read_text())
            self.assertEqual(len(state["cases"]),2)
            self.assertTrue(report["gates"]["nonlinear_converged"])
            self.assertTrue(report["gates"]["vsp_panels_closed"])
            self.assertTrue(report["gates"]["solver_static_residuals"])
            self.assertFalse(report["model_accepted"])

    def test_su2_rejects_bad_intermediate_history_and_unsettled_loads(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "history.csv"
            header = su2_fixture(.1, .2, -.3).splitlines()[0] + "\n"
            stable = "".join('%d,-9,.1,.2,-.3,0,0,0\n' % i for i in range(200))
            path.write_text(header + stable)
            self.assertEqual(aero.parse_su2(path, 2)["status"], "converged")
            for bad in (stable.replace('40,-9,.1', '40,-9,nan'),
                        stable.replace('40,-9,.1', '39,-9,.1'),
                        "".join('%d,-9,%g,.2,-.3,0,0,0\n' % (i, .1+i*.0001) for i in range(200))):
                path.write_text(header + bad)
                with self.subTest(bad=bad[:80]), self.assertRaises(ValueError):
                    aero.parse_su2(path, 2)

    def test_su2_native_cauchy_targets_remain_mandatory(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "history.csv"
            data = su2_fixture(.1, .2, -.3)
            path.write_text(data.replace(",0,0,0\n", ",1e-9,0,0\n"))
            with self.assertRaisesRegex(ValueError, "Cauchy"):
                aero.parse_su2(path, 2)

    def test_cli_rejects_unconverged_outer_flow_despite_matching_plausible_loads(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)

            def solver(command, directory, log, timeout=120):
                if "--mesh-worker" in command:
                    (output / "mesh.su2").write_bytes(b"synthetic mesh")
                    aero.save(output / "mesh-record.json", {"reference": {"area_m2": .1, "length_m": .1, "moment_x_m": 1}})
                elif "-t" in command:
                    zero = "alpha-0.cfg" in command[-1]
                    cl, cm = (0, 0) if zero else (.4, -.1)
                    (Path(directory) / "history.csv").write_text(su2_fixture(.1, cl, cm))
                else:
                        self.write_paired_campaign(output,{"area_m2":.1,"length_m":.1,"moment_x_m":1},
                                                   maximum=(.851015563695,1.410963213734))
                return {"exit_code": 0, "status": "executed"}

            argv = ["aero.py", "--design", str(output / "design.toml"), "--output", str(output),
                    "--gmsh", "gmsh", "--openvsp", "vsp", "--su2", "su2"]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=solver):
                self.assertEqual(aero.main(), 1)
            report = __import__("json").loads((output / "report.json").read_text())
            self.assertTrue(report["gates"]["solver_static_residuals"])
            self.assertTrue(report["gates"]["gmres_targets_met_every_wake"])
            self.assertFalse(report["gates"]["nonlinear_converged"])

            def converged(command, directory, log, timeout=120):
                result = solver(command, directory, log, timeout)
                if "--vsp-parametric-worker" in command:
                    self.write_paired_campaign(output,{"area_m2":.1,"length_m":.1,"moment_x_m":1},maximum=(-3.,-3.))
                return result

            with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=converged):
                self.assertEqual(aero.main(), 0)
            report = __import__("json").loads((output / "report.json").read_text())
            self.assertTrue(report["gates"]["nonlinear_converged"])
            self.assertFalse(report["model_accepted"])

    def test_cli_rejects_shared_zero_alpha_pitch_bias_despite_solver_agreement(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)

            def solver(command, directory, log, timeout=120):
                if "--mesh-worker" in command:
                    (output / "mesh.su2").write_bytes(b"synthetic mesh")
                    aero.save(output / "mesh-record.json", {"reference": {"area_m2": .1, "length_m": .1, "moment_x_m": 1}})
                elif "-t" in command:
                    alpha_zero = "alpha-0.cfg" in command[-1]
                    cl, cm = (0, .3) if alpha_zero else (.4, -.2)
                    (Path(directory) / "history.csv").write_text(su2_fixture(.1, cl, cm))
                else:
                    self.write_paired_campaign(output,{"area_m2":.1,"length_m":.1,"moment_x_m":1},zero_cm=.3,cm=-.2)
                return {"exit_code": 0, "status": "executed"}

            argv = ["aero.py", "--design", str(output / "design.toml"), "--output", str(output),
                    "--gmsh", "gmsh", "--openvsp", "vsp", "--su2", "su2"]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=solver):
                self.assertEqual(aero.main(), 1)
            report = __import__("json").loads((output / "report.json").read_text())
            self.assertEqual(report["status"], "failed_static_comparison")
            self.assertTrue(report["gates"]["solver_static_residuals"])
            self.assertTrue(report["gates"]["su2_zero_alpha_symmetry"])
            self.assertFalse(report["gates"]["su2_zero_alpha_pitch_symmetry"])
            self.assertFalse(report["gates"]["vsp_zero_alpha_pitch_symmetry"])
            self.assertFalse(report["model_accepted"])

    def test_cli_rerun_replaces_prior_archived_histories(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            for alpha in (0, 2):
                (output / ("history-alpha-%g.csv" % alpha)).write_text("old evidence\n")

            def solver(command, directory, log, timeout=120):
                if "--mesh-worker" in command:
                    (output / "mesh.su2").write_bytes(b"synthetic mesh")
                    aero.save(output / "mesh-record.json", {"reference": {"area_m2": .1, "length_m": .1, "moment_x_m": 1}})
                elif "-t" in command:
                    (Path(directory) / "history.csv").write_text(su2_fixture(-.1, .3, -.2))
                return {"exit_code": 0, "status": "executed"}

            argv = ["aero.py", "--design", str(output / "design.toml"), "--output", str(output),
                    "--gmsh", "gmsh", "--openvsp", "vsp", "--su2", "su2"]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=solver):
                self.assertEqual(aero.main(), 1)
            for alpha in (0, 2):
                self.assertNotIn("old evidence", (output / ("history-alpha-%g.csv" % alpha)).read_text())
            report = __import__("json").loads((output / "report.json").read_text())
            self.assertEqual(report["status"], "failed_static_comparison")

    def test_cli_records_unexpected_helper_failure_instead_of_stale_success(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            aero.save(output / "report.json", {"status": "static_comparison_passed", "gates": {"old": True}})
            argv = ["aero.py", "--design", str(output / "design.toml"), "--output", str(output)]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=RuntimeError("synthetic launch failure")):
                self.assertEqual(aero.main(), 1)
            report = __import__("json").loads((output / "report.json").read_text())
            self.assertEqual(report["status"], "failed")
            self.assertFalse(report["model_accepted"])
            self.assertIn("synthetic launch failure", report["reason"])
            self.assertNotIn("gates", report)

    def test_cli_uses_lifting_geometry_and_rejects_converged_unphysical_results(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            observed = []

            def solver(command, directory, log, timeout=120):
                observed.append(command)
                if "--mesh-worker" in command:
                    (output / "mesh.su2").write_bytes(b"synthetic mesh")
                    aero.save(output / "mesh-record.json", {"reference": {"area_m2": .1, "length_m": .1, "moment_x_m": 1}})
                elif "-t" in command:
                    (Path(directory) / "history.csv").write_text(su2_fixture(-.1, .3, -.2))
                else:
                    self.assertIn("--vsp-parametric-worker", command)
                    (output / "missile.polar").write_text("Mach AoA CLi CDi CMiy\n.6 0 .3 -.1 -.2\n.6 2 .4 -.1 -.2\n")
                    Path(log).write_text(vsp_log())
                return {"exit_code": 0, "status": "executed"}

            argv = ["aero.py", "--design", str(output / "design.toml"), "--output", str(output),
                    "--gmsh", "gmsh", "--openvsp", "vsp", "--su2", "su2"]
            with mock.patch.object(sys, "argv", argv), mock.patch.object(aero, "bounded", side_effect=solver):
                self.assertEqual(aero.main(), 1)
            report = __import__("json").loads((output / "report.json").read_text())
            self.assertEqual(report["status"], "failed_static_comparison")
            self.assertTrue(report["gates"]["su2_density_converged"])
            self.assertFalse(report["gates"]["su2_zero_alpha_symmetry"])
            self.assertFalse(report["gates"]["vsp_nonnegative_wind_drag"])
            self.assertFalse(report["model_accepted"])
            self.assertEqual(len(observed), 4)

    def test_physical_gates_do_not_accept_residual_convergence_alone(self):
        rows = [{"alpha_deg": 0, "wind_cd": -.05, "coefficients": {"cn": .3, "cm": .3}},
                {"alpha_deg": 2, "wind_cd": .01, "coefficients": {"cn": .4, "cm": -.2}}]
        self.assertEqual(aero.physical_gates(rows), {"conditions_match": True, "nonnegative_wind_drag": False,
                                                   "zero_alpha_symmetry": False, "zero_alpha_pitch_symmetry": False})
        self.assertFalse(aero.physical_gates([])["conditions_match"])

    def test_zero_alpha_pitch_tolerance_is_finite_symmetric_and_explicit(self):
        rows = [{"alpha_deg": 0, "wind_cd": .1, "coefficients": {"cn": 0, "cm": 0}},
                {"alpha_deg": 2, "wind_cd": .1, "coefficients": {"cn": .4, "cm": -.2}}]
        for pitch, expected in ((0, True), (.02, True), (-.02, True), (.020001, False), (-.020001, False)):
            with self.subTest(pitch=pitch):
                rows[0]["coefficients"]["cm"] = pitch
                self.assertEqual(aero.physical_gates(rows)["zero_alpha_pitch_symmetry"], expected)
        rows[0]["coefficients"]["cm"] = math.nan
        with self.assertRaises(ValueError):
            aero.physical_gates(rows)

    def test_lifting_convergence_requires_topology_and_each_printed_solver_target(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "solver.log"
            data = vsp_log()
            path.write_text(data)
            result = aero.vsp_convergence(path)
            self.assertTrue(result["lifting_wakes_present"])
            self.assertTrue(result["gmres_targets_met_every_wake"])
            path.write_text(data.replace(": 48", ": 0").replace("-1.1 /", "-.8 /"))
            result = aero.vsp_convergence(path)
            self.assertFalse(result["lifting_wakes_present"])
            self.assertFalse(result["gmres_targets_met_every_wake"])

    def test_failed_and_timed_out_processes_remain_failures(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "run.log"
            result = aero.bounded([sys.executable, "-c", "raise SystemExit(3)"], folder, log, 5)
            self.assertEqual((result["status"], result["exit_code"]), ("failed", 3))
            result = aero.bounded([sys.executable, "-c", "import time; time.sleep(2)"], folder, log, 0.05)
            self.assertEqual((result["status"], result["exit_code"]), ("timeout", None))

    def test_closed_mesh_orientation_is_repaired_and_open_mesh_rejected(self):
        points = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        faces = [(0, 1, 2), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
        result, metrics = aero.orient_closed_surface(points, faces)
        self.assertEqual(len(result), 4)
        self.assertAlmostEqual(metrics["volume_m3"], 1 / 6)
        self.assertLess(metrics["area_vector_norm_m2"], 1e-15)
        with self.assertRaises(ValueError):
            aero.orient_closed_surface(points, faces[:-1])

    def test_cart3d_translation_preserves_geometry_and_rejects_invalid_indices(self):
        with tempfile.TemporaryDirectory() as folder:
            source, target = Path(folder) / "source.tri", Path(folder) / "target.tri"
            geometry = "4 4\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 3 2\n1 2 4\n1 4 3\n2 3 4\n"
            source.write_text(geometry + "1\n" * 4)
            aero.cart3d_without_tags(source, target)
            self.assertEqual(target.read_text(), geometry)
            for bad in (geometry.replace("2 3 4", "0 3 4"), geometry.replace("2 3 4", "2 3 5"),
                        geometry.replace("1 0 0", "nan 0 0"), geometry[:-6]):
                source.write_text(bad)
                with self.subTest(bad=bad), self.assertRaises(ValueError):
                    aero.cart3d_without_tags(source, target)

    def test_force_frame_converts_wind_to_body_without_normalization_change(self):
        values = aero.body_coefficients(0.1, 0.5, -0.2, 30.0)
        self.assertAlmostEqual(values["ca"], 0.1 * math.cos(math.pi / 6) - 0.25)
        self.assertAlmostEqual(values["cn"], 0.05 + 0.5 * math.cos(math.pi / 6))
        self.assertEqual(values["cm"], -0.2)

    def test_su2_finite_converged_history_is_required(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "history.csv"
            path.write_text(su2_fixture(.1, .2, -.3).replace('"CMy"', '"CMY"'))
            row = aero.parse_su2(path, 2.0)
            self.assertEqual(row["status"], "converged")
            self.assertEqual(row["residual_log10"], -9)
            path.write_text(su2_fixture(.1, .2, -.3))
            self.assertEqual(aero.parse_su2(path, 2.0)["coefficients"], row["coefficients"])
            for bad in ('"Inner_Iter","rms[Rho]","CD","CL","CMY"\n1,-4,0.1,0.2,-0.3\n',
                        '"Inner_Iter","rms[Rho]","CD","CL","CMY"\n1,-6,nan,0.2,-0.3\n',
                        '"Inner_Iter","CD","CL","CMY"\n1,0.1,0.2,-0.3\n'):
                path.write_text(bad)
                with self.subTest(bad=bad), self.assertRaises(ValueError):
                    aero.parse_su2(path, 2.0)

    def test_missing_or_empty_solver_outputs_fail(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "missing"
            with self.assertRaises((ValueError, OSError)):
                aero.parse_su2(path, 0.0)
            path.write_text("")
            with self.assertRaises(ValueError):
                aero.parse_vspaero(path)

    def test_vspaero_polar_rejects_nonfinite_and_wrong_shape(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "case.polar"
            path.write_text("Mach AoA Beta CL CDtot CMm\n0.6 2 0 0.4 0.1 -0.2\n")
            rows = aero.parse_vspaero(path)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["alpha_deg"], 2)
            for text in ("Mach AoA CL CDtot CMm\n0.6 2 nan .1 -.2\n", "Mach AoA CL\n.6 2 .4\n"):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    aero.parse_vspaero(path)

    def test_vspaero_uses_only_near_surface_inviscid_loads_for_euler_comparison(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "case.polar"
            path.write_text("Mach AoA CLi CDi CMiy CLtot CDtot CMytot\n0.6 0 .4 .1 -.2 .5 .2 -.3\n")
            row = aero.parse_vspaero(path)[0]
            self.assertEqual(row["basis"], "near_surface_inviscid")
            self.assertFalse(row["convergence_assessed"])
            self.assertEqual(row["coefficients"], {"ca": .1, "cn": .4, "cm": -.2})

    def test_static_comparison_exposes_residual_and_cannot_fill_missing_channels(self):
        result = aero.static_comparison({"cn": 0.5, "ca": 0.1, "cm": -0.2},
                                        {"cn": 0.4, "ca": 0.12, "cm": -0.1})
        self.assertAlmostEqual(result["cn"]["relative"], 0.2)
        self.assertEqual(set(result), {"ca", "cn", "cm"})
        with self.assertRaises(ValueError):
            aero.static_comparison({"cn": float("inf")}, {"cn": 1.0})


def load_tests(loader, tests, pattern):
    tests.addTests(loader.loadTestsFromModule(test_aero_history))
    tests.addTests(loader.loadTestsFromModule(test_aero_mesh))
    tests.addTests(loader.loadTestsFromModule(test_aero_panels))
    tests.addTests(loader.loadTestsFromModule(test_aero_grid))
    tests.addTests(loader.loadTestsFromModule(test_aero_vsp_frame))
    tests.addTests(loader.loadTestsFromModule(test_aero_vsp_case))
    tests.addTests(loader.loadTestsFromModule(test_aero_vsp_native))
    tests.addTests(loader.loadTestsFromModule(test_aero_vsp_adb))
    tests.addTests(loader.loadTestsFromModule(test_aero_hangar))
    tests.addTests(loader.loadTestsFromModule(test_aero_schema))
    return tests


if __name__ == "__main__":
    unittest.main()
