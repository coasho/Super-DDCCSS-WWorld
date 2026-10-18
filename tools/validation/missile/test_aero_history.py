"""Pinned VSPAERO steady-history safeguards; no optional solver dependencies."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest


_SPEC = importlib.util.spec_from_file_location("missile_aero_history", Path(__file__).with_name("aero_history.py"))
history = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(history)

# Independently transcribed from the bundled 7.2.2 .history output. Its two
# residual names each contain a space, unlike the individual numeric columns.
HEADER = ("Iter Mach AoA Beta CLo CLi CLtot CDo CDi CDtot CSo CSi CStot L/D E "
          "CMox CMoy CMoz CMix CMiy CMiz CMxtot CMytot CMztot CFox CFoy CFoz "
          "CFix CFiy CFiz CFxtot CFytot CFztot CLwtot CDwtot CSwtot CLiw CDiw "
          "CSiw CFwxtot CFwytot CFwztot CFiwx CFiwy CFiwz LoDw Ew T/QS "
          "L2 Residual Max Residual Wall_Time")


def polar():
    result = []
    for alpha in (0.0, 2.0):
        cd, cl, cm = .1, .2 * alpha, -.05 * alpha
        angle = math.radians(alpha)
        result.append({"mach": .6, "alpha_deg": alpha, "wind_cd": cd,
                       "coefficients": {"ca": cd * math.cos(angle) - cl * math.sin(angle),
                                        "cn": cl * math.cos(angle) + cd * math.sin(angle), "cm": cm}})
    return result


def fixture(maxima=(-3.5, -3.5), *, iterations=(1, 5), alpha=(0.0, 2.0)):
    columns = HEADER.replace("L2 Residual", "L2Residual").replace("Max Residual", "MaxResidual").split()
    blocks = []
    for case, (angle, maximum) in enumerate(zip(alpha, maxima), 1):
        rows = []
        for iteration in iterations:
            values = dict.fromkeys(columns, 0.0)
            values.update(Iter=iteration, Mach=.6, AoA=angle, Beta=0, CDi=.1, CLi=.2 * angle,
                          CMiy=-.05 * angle, L2Residual=-4, MaxResidual=maximum, Wall_Time=1)
            rows.append(" ".join(str(values[key]) for key in columns))
        blocks.append("\n".join(("*" * 40, "# Name Value Units", "Mach_ 0.6000000 no_unit",
                                 "AoA_ %.7f deg" % angle, "Beta_ 0.0000000 deg", "",
                                 "Solver Case: %d" % case, "", "Surface Integration Forces and Moments -->",
                                 "Surf-Surf-Surf-Surf Wake-Wake-Wake-Wake", HEADER, *rows)))
    return "\n".join(blocks) + "\n"


class VspaeroHistory(unittest.TestCase):
    def parse(self, text, rows=None, **kwargs):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "missile.history"
            path.write_text(text, encoding="utf-8")
            return history.parse_history(path, polar() if rows is None else rows, **kwargs)

    def test_linear_targets_can_pass_while_final_nonlinear_residual_fails(self):
        # These are the real failed trial's final logarithmic maximum residuals.
        # A successful inner linear solve cannot substitute for this outer gate.
        spec = importlib.util.spec_from_file_location("missile_aero_linear_fixture", Path(__file__).with_name("aero.py"))
        aero = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(aero)
        linear_log = "NumberOfKuttaNodes after trimming: 48\nNumberOfWakeLoops: 280\n"
        linear_log += "".join("Solving... Mach: .6 ... Alpha: %g ... Beta: 0\n" % alpha +
                              "".join("Wake Iter: %d / 5 ... GMRES Iter: 4 ... Red: -1.1 / -1.0 ... Max: .1 / 1.31\n" % wake
                                      for wake in range(1, 6)) for alpha in (0, 2))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "vspaero.log"
            path.write_text(linear_log, encoding="utf-8")
            linear = aero.vsp_convergence(path)
            self.assertTrue(linear["lifting_wakes_present"])
            self.assertTrue(linear["gmres_targets_met_every_wake"])
        result = self.parse(fixture((.851015563695, 1.410963213734)))
        self.assertFalse(result["nonlinear_converged"])
        self.assertEqual([row["max_residual_log10"] for row in result["cases"]],
                         [.851015563695, 1.410963213734])

    def test_frozen_threshold_and_matching_polar_pass(self):
        result = self.parse(fixture((-3.0, -3.5)))
        self.assertTrue(result["nonlinear_converged"])
        self.assertEqual(result["threshold_log10"], -3.0)
        self.assertEqual([row["alpha_deg"] for row in result["cases"]], [0.0, 2.0])
        self.assertTrue(all(row["iterations"] == 2 and row["final_iteration"] == 5 for row in result["cases"]))

    def test_threshold_is_inclusive_and_uses_final_row(self):
        self.assertFalse(self.parse(fixture((-3.0, -2.999999)))["nonlinear_converged"])
        data = fixture((-4.0, -4.0)).splitlines()
        for index, line in enumerate(data):
            if line.startswith("5 "):
                fields = line.split()
                fields[-2] = "-2"
                data[index] = " ".join(fields)
        self.assertFalse(self.parse("\n".join(data))["nonlinear_converged"])

    def test_stricter_finite_threshold_can_be_declared(self):
        self.assertFalse(self.parse(fixture((-3.5, -3.5)), threshold=-4)["nonlinear_converged"])
        for value in (True, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.parse(fixture(), threshold=value)

    def test_missing_and_empty_histories_are_invalid(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                history.parse_history(Path(folder) / "missing.history", polar())
        with self.assertRaises(ValueError):
            self.parse("")

    def test_all_history_numbers_must_be_finite(self):
        for field in ("CLi", "CDi", "CMiy", "MaxResidual", "L2Residual", "CFox", "Wall_Time"):
            for bad in ("nan", "inf", "-inf"):
                lines = fixture().splitlines()
                columns = HEADER.replace("L2 Residual", "L2Residual").replace("Max Residual", "MaxResidual").split()
                index = next(i for i, line in enumerate(lines) if line.startswith("1 "))
                values = lines[index].split()
                values[columns.index(field)] = bad
                lines[index] = " ".join(values)
                with self.subTest(field=field, bad=bad), self.assertRaises(ValueError):
                    self.parse("\n".join(lines))

    def test_missing_or_duplicate_columns_and_truncated_rows_are_invalid(self):
        for data in (fixture().replace("CLi CLtot", "CLi CLi"),
                     fixture().replace("L2 Residual ", ""),
                     fixture().replace("Wall_Time", "Unknown"),
                     fixture().replace("-3.5 1", "-3.5", 1),
                     fixture().replace("Iter Mach", "Time Iter Mach")):
            with self.subTest(data=data[:80]), self.assertRaises(ValueError):
                self.parse(data)

    def test_nonincreasing_or_invalid_iteration_is_rejected(self):
        for iterations in ((1, 1), (5, 1), (0, 5), (1, 1.5)):
            with self.subTest(iterations=iterations), self.assertRaises(ValueError):
                self.parse(fixture(iterations=iterations))

    def test_missing_extra_and_duplicate_cases_are_invalid(self):
        data = fixture()
        split = data.index("*" * 40, 1)
        for bad in (data[:split], data + data, data.replace("Solver Case: 2", "Solver Case: 1"),
                    fixture(alpha=(0, 0)), fixture(alpha=(0, 3)), data + "Solver Case: 3\n",
                    data + "\n" + "*" * 40 + "\nSolver Case: invalid\n"):
            with self.subTest(data=bad[-90:]), self.assertRaises(ValueError):
                self.parse(bad)

    def test_empty_table_or_missing_case_header_is_invalid(self):
        for bad in (fixture().replace("Solver Case: 2", ""),
                    fixture().replace(HEADER, "", 1), fixture().split(HEADER, 1)[0]):
            with self.subTest(data=bad[-90:]), self.assertRaises(ValueError):
                self.parse(bad)

    def test_frozen_conditions_and_metadata_must_match_every_row(self):
        for bad in (fixture().replace("Mach_ 0.6000000", "Mach_ 0.7000000", 1),
                    fixture().replace("AoA_ 2.0000000", "AoA_ 0.0000000"),
                    fixture().replace("Beta_ 0.0000000", "Beta_ 1.0000000", 1),
                    fixture().replace("Mach_ 0.6000000 no_unit\n", "", 1),
                    fixture().replace("Mach_ 0.6000000 no_unit", "Mach_ 0.6000000 no_unit\nMach_ 0.6000000 no_unit", 1),
                    fixture().replace("1 0.6 0.0 0", "1 0.7 0.0 0", 1),
                    fixture().replace("1 0.6 0.0 0", "1 0.6 0.0 1", 1)):
            with self.subTest(data=bad[:100]), self.assertRaises(ValueError):
                self.parse(bad)

    def test_final_loads_must_match_current_polar(self):
        for column in ("wind_cd", "ca", "cn", "cm"):
            rows = polar()
            if column == "wind_cd":
                rows[1][column] += .01
            else:
                rows[1]["coefficients"][column] += .01
            with self.subTest(column=column), self.assertRaises(ValueError):
                self.parse(fixture(), rows)

    def test_polar_must_contain_exactly_the_two_frozen_finite_cases(self):
        for rows in ([], polar()[:1], polar() + polar()[:1], [polar()[0], polar()[0]]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.parse(fixture(), rows)
        for key, value in (("mach", .7), ("alpha_deg", 3), ("wind_cd", float("nan")), ("beta_deg", 1)):
            rows = polar()
            rows[0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.parse(fixture(), rows)


if __name__ == "__main__":
    unittest.main()
