"""Strict raw single-case/frame regression fixtures, without native solvers."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parent))
from aero_vsp_frame import paired_conditions

_SPEC = importlib.util.spec_from_file_location("missile_aero_vsp_case", Path(__file__).with_name("aero_vsp_case.py"))
case = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(case)

HISTORY = ("Iter Mach AoA Beta CLo CLi CLtot CDo CDi CDtot CSo CSi CStot L/D E "
           "CMox CMoy CMoz CMix CMiy CMiz CMxtot CMytot CMztot CFox CFoy CFoz "
           "CFix CFiy CFiz CFxtot CFytot CFztot CLwtot CDwtot CSwtot CLiw CDiw "
           "CSiw CFwxtot CFwytot CFwztot CFiwx CFiwy CFiwz LoDw Ew T/QS "
           "L2Residual MaxResidual Wall_Time").split()
POLAR = ("Beta Mach AoA Re/1e6 CLo CLi CLtot CDo CDi CDtot CSo CSi CStot L/D E "
         "CMox CMoy CMoz CMix CMiy CMiz CMxtot CMytot CMztot CFox CFoy CFoz "
         "CFix CFiy CFiz CFxtot CFytot CFztot CLwtot CDwtot CSwtot CLiw CDiw "
         "CSiw CFwxtot CFwytot CFwztot CFiwx CFiwy CFiwz LoDw Ew StallFactor").split()


def fixture(pair, maximum=-3.5, *, bref=.5, cref=.2, area=.1, cg=(1,0,0)):
    roll = math.radians(pair["roll_deg"])
    c, s = math.cos(roll), math.sin(roll)
    # Independent forward rotation; references deliberately differ.
    fx, fy, fz = .12, -.04, .4
    mx, my, mz = .03*bref, -.1*cref, .07*bref
    raw = {"CFix": fx, "CFiy": c*fy-s*fz, "CFiz": s*fy+c*fz,
           "CMix": mx/bref, "CMiy": (c*my-s*mz)/cref, "CMiz": (s*my+c*mz)/bref}
    native = pair["native"]
    a, b = math.radians(native["alpha_deg"]), math.radians(native["beta_deg"])
    cl = -raw["CFix"]*math.sin(a)+raw["CFiz"]*math.cos(a)
    cd = (raw["CFix"]*math.cos(a)+raw["CFiz"]*math.sin(a))*math.cos(b)-raw["CFiy"]*math.sin(b)
    values = dict.fromkeys(set(HISTORY+POLAR), 0.)
    values.update(raw, Mach=native["mach"], AoA=native["alpha_deg"], Beta=native["beta_deg"],
                  CLi=cl, CDi=cd, L2Residual=-4, MaxResidual=maximum)
    polar = " ".join(POLAR)+"\n"+" ".join("%.12f" % values[k] for k in POLAR)+"\n"
    metadata = "\n".join("%s %.7f %s" % (key, value, unit) for key, value, unit in
                         (("Sref_", area, "Lunit^2"), ("Xcg_",cg[0],"Lunit"),("Ycg_",cg[1],"Lunit"),("Zcg_",cg[2],"Lunit"),
                          ("Cref_", cref, "Lunit"), ("Bref_", bref, "Lunit"),
                          ("Mach_", native["mach"], "no_unit"), ("AoA_", native["alpha_deg"], "deg"),
                          ("Beta_", native["beta_deg"], "deg")))
    header = " ".join(HISTORY).replace("L2Residual", "L2 Residual").replace("MaxResidual", "Max Residual")
    rows = []
    for iteration in (1, 2, 3):
        values.update(Iter=iteration, Wall_Time=iteration/5)
        rows.append(" ".join("%.12f" % values[k] for k in HISTORY))
    history = "\n".join(("*"*30, "# Name Value Units", metadata, "Solver Case: 1", "", header, *rows))+"\n"
    log = "VSPAERO v.7.2.2 --- Compiled on: pinned fixture\nNumberOfKuttaNodes after trimming: 46\nNumberOfWakeLoops: 266\n"
    log += "Solving... Mach: %.6f ... Alpha: %.6f ... Beta: %.6f\n" % (native["mach"], native["alpha_deg"], native["beta_deg"])
    log += "".join("Wake Iter: %d / 5 ... GMRES Iter: 4 ... Red: -6 / -5 ... Max: -8 / 1.31\n" % i for i in (1,2,3))
    log += "Total setup and solve time: 1 seconds\n"
    return {"polar": polar, "history": history, "log": log}


class SingleNativeCase(unittest.TestCase):
    def assess(self, data, pair=None, **kwargs):
        pair = paired_conditions(.123)[1] if pair is None else pair
        with tempfile.TemporaryDirectory() as directory:
            paths = {key: Path(directory)/key for key in data}
            for key, path in paths.items(): path.write_text(data[key])
            before = {key: path.read_bytes() for key, path in paths.items()}
            result = case.assess_case(paths["polar"], paths["history"], paths["log"], pair,
                                      bref=kwargs.get("bref", .5), cref=kwargs.get("cref", .2),
                                      **{key:kwargs[key] for key in ("reference_area","native_cg") if key in kwargs})
            self.assertEqual(before, {key: path.read_bytes() for key, path in paths.items()})
            return result

    def test_paired_raw_case_recovers_all_six_loads_without_rewriting_files(self):
        pair = paired_conditions(.123)[1]
        result = self.assess(fixture(pair), pair)
        self.assertTrue(result["numerically_converged"])
        self.assertFalse(result["model_accepted"])
        self.assertEqual((result["mach"],result["alpha_deg"],result["beta_deg"]), (.6,2.,0.))
        for key, expected in zip(("CFix","CFiy","CFiz","CMix","CMiy","CMiz"), (.12,-.04,.4,.03,-.1,.07)):
            self.assertAlmostEqual(result["loads"]["original"][key], expected, places=10)
        self.assertAlmostEqual(result["coefficients"]["ca"], .12, places=10)
        self.assertAlmostEqual(result["coefficients"]["cn"], .4, places=10)
        self.assertAlmostEqual(result["coefficients"]["cm"], -.1, places=10)
        self.assertAlmostEqual(result["wind_cd"], .12*math.cos(math.radians(2))+.4*math.sin(math.radians(2)), places=10)

    def test_quarter_turn_checks_beta_sign_and_unequal_moment_normalization(self):
        pair = paired_conditions(90)[1]
        result = self.assess(fixture(pair), pair)
        self.assertAlmostEqual(result["loads"]["original"]["CMiy"], -.1, places=11)
        self.assertAlmostEqual(result["loads"]["original"]["CMiz"], .07, places=11)
        self.assertAlmostEqual(result["coefficients"]["cn"], .4, places=11)

    def test_actual_native_polar_explanatory_banners_are_not_numeric_rows(self):
        pair=paired_conditions(.123)[1];data=fixture(pair)
        data["polar"]=("Surface Integration Forces and Moments --> Wake Induced Forces -->\n"
                       "Surf-Surf-Surf-Surf Wake-Wake-Wake-Wake\n"+data["polar"])
        self.assertTrue(self.assess(data,pair)["numerically_converged"])

    def test_native_timing_diagnostic_does_not_define_iteration_order(self):
        pair=paired_conditions(.123)[1];data=fixture(pair)
        lines=data["history"].splitlines();fields=lines[-1].split()
        fields[HISTORY.index("Wall_Time")]="0.1";lines[-1]=" ".join(fields)
        data["history"]="\n".join(lines)+"\n"
        self.assertTrue(self.assess(data,pair)["numerically_converged"])

    def test_unpaired_beta_is_rejected_even_if_all_three_raw_files_agree(self):
        pair = paired_conditions(.123)[1]
        wrong = copy.deepcopy(pair);wrong["native"]["beta_deg"] = 0
        with self.assertRaises(ValueError): self.assess(fixture(wrong), pair)
        with self.assertRaises(ValueError): self.assess(fixture(wrong), wrong)

    def test_stale_nonstatic_six_load_columns_cannot_hide_behind_matching_cn_cm(self):
        pair = paired_conditions(.123)[1]
        for key in ("CFiy", "CMix", "CMiz"):
            data = fixture(pair);lines = data["history"].splitlines();fields = lines[-1].split()
            fields[HISTORY.index(key)] = str(float(fields[HISTORY.index(key)])+.01)
            lines[-1] = " ".join(fields);data["history"] = "\n".join(lines)+"\n"
            with self.subTest(key=key), self.assertRaises(ValueError): self.assess(data,pair)

    def test_cartesian_extra_cases_are_rejected_in_each_raw_source(self):
        pair = paired_conditions(.123)[1]
        for key in ("polar", "history", "log"):
            data = fixture(pair);data[key] += data[key]
            with self.subTest(key=key), self.assertRaises(ValueError): self.assess(data,pair)

    def test_printed_metadata_precision_is_deliberate_but_numeric_rows_remain_strict(self):
        pair = paired_conditions(.123)[1]
        self.assertTrue(self.assess(fixture(pair),pair)["numerically_converged"])
        data=fixture(pair);lines=data["history"].splitlines()
        for i,line in enumerate(lines):
            if line.startswith("Beta_"):lines[i]="Beta_ %.7f deg" % (pair["native"]["beta_deg"]+1e-7)
        data["history"]="\n".join(lines)+"\n"
        with self.assertRaises(ValueError):self.assess(data,pair)
        data=fixture(pair);lines=data["polar"].splitlines();values=lines[1].split()
        values[POLAR.index("AoA")]=str(pair["native"]["alpha_deg"]+1e-8);lines[1]=" ".join(values);data["polar"]="\n".join(lines)+"\n"
        with self.assertRaises(ValueError):self.assess(data,pair)

    def test_native_outer_and_every_inner_target_are_independent_gates(self):
        pair=paired_conditions(.123)[1]
        result=self.assess(fixture(pair,maximum=-2.9),pair)
        self.assertTrue(result["gmres_targets_met_every_wake"]);self.assertFalse(result["nonlinear_converged"])
        self.assertFalse(result["numerically_converged"])
        data=fixture(pair);data["log"]=data["log"].replace("Red: -6 / -5","Red: -4 / -5",1)
        result=self.assess(data,pair)
        self.assertTrue(result["nonlinear_converged"]);self.assertFalse(result["gmres_targets_met_every_wake"])
        self.assertFalse(result["numerically_converged"])

    def test_truncated_or_wrong_version_log_and_mismatched_last_iteration_fail(self):
        pair=paired_conditions(.123)[1]
        for log in (fixture(pair)["log"].replace("7.2.2","6.4.8"),
                    fixture(pair)["log"].split("Total setup")[0],
                    fixture(pair)["log"].replace("Wake Iter: 3 / 5", "Wake Iter: 4 / 5")):
            data=fixture(pair);data["log"]=log
            with self.subTest(log=log),self.assertRaises(ValueError):self.assess(data,pair)

    def test_stale_early_completion_or_nonfinite_duration_is_not_completion(self):
        pair=paired_conditions(.123)[1]
        data=fixture(pair);completion="Total setup and solve time: 1 seconds\n"
        data["log"]=data["log"].replace(completion,"").replace("Wake Iter: 1",completion+"Wake Iter: 1",1)
        with self.assertRaises(ValueError):self.assess(data,pair)
        data=fixture(pair);data["log"]=data["log"].replace(completion,"Total setup and solve time: 1e999 seconds\n")
        with self.assertRaises(ValueError):self.assess(data,pair)

    def test_moment_references_and_complete_headers_are_required(self):
        pair=paired_conditions(.123)[1]
        with self.assertRaises(ValueError):self.assess(fixture(pair),pair,cref=.3)
        for key in ("polar","history"):
            data=fixture(pair);data[key]=data[key].replace("CFiy ","missing ",1)
            with self.subTest(key=key),self.assertRaises(ValueError):self.assess(data,pair)

    def test_explicit_area_and_rotated_moment_origin_reject_stale_native_reference(self):
        pair=paired_conditions(.123)[1];data=fixture(pair)
        result=self.assess(data,pair,reference_area=.1,native_cg=[1,0,0])
        self.assertTrue(result["reference_check"]["passed"])
        for area,cg in ((.2,[1,0,0]),(.1,[1.01,0,0]),(.1,[1,.001,0]),(.1,[1,0,-.001])):
            with self.subTest(area=area,cg=cg),self.assertRaises(ValueError):
                self.assess(data,pair,reference_area=area,native_cg=cg)

    def test_nonfinite_unused_history_column_and_changed_wind_scalar_fail(self):
        pair=paired_conditions(.123)[1]
        data=fixture(pair);lines=data["history"].splitlines();values=lines[-1].split();values[HISTORY.index("CLiw")]="nan"
        lines[-1]=" ".join(values);data["history"]="\n".join(lines)+"\n"
        with self.assertRaises(ValueError):self.assess(data,pair)
        data=fixture(pair)
        for key,columns in (("polar",POLAR),("history",HISTORY)):
            lines=data[key].splitlines()
            for i,line in enumerate(lines):
                fields=line.split()
                if len(fields)==len(columns) and fields[0][0].isdigit():
                    fields[columns.index("CDi")]=str(float(fields[columns.index("CDi")])+.1);lines[i]=" ".join(fields)
            data[key]="\n".join(lines)+"\n"
        with self.assertRaises(ValueError):self.assess(data,pair)


if __name__ == "__main__":unittest.main()
