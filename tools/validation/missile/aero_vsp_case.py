"""Assess one unmodified VSPAERO 7.2.2 case in a declared rigid frame.

The caller executes each paired alpha/beta condition separately. This parser
never concatenates, relabels or rewrites native files. Native history/polar
conditions print 12 decimals, history metadata 7, and log banners 6; each is
checked at its own output resolution against the same intended condition.
Converged numerical output is never equipment-model admission.
"""
from collections.abc import Mapping
import hashlib
import math
from pathlib import Path
import re

from aero_history import _COLUMNS
from aero_vsp_frame import paired_conditions, recover_loads


_SIX = ("CFix", "CFiy", "CFiz", "CMix", "CMiy", "CMiz")
_POLAR = ["Beta", "Mach", "AoA", "Re/1e6", *_COLUMNS[4:47], "StallFactor"]
_NUMBER = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
_CONDITIONS = (("Mach", "mach"), ("AoA", "alpha_deg"), ("Beta", "beta_deg"))
_NONLINEAR_LIMIT = -3.0


def _finite(value, label):
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(label + " must be finite") from error
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(label + " must be finite")
    return result


def _printed(actual, expected, decimals):
    return abs(actual-expected) <= .5001 * 10**(-decimals)


def _loads_match(left, right):
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=1e-10)


def _intended(pair):
    try:
        roll = _finite(pair["roll_deg"], "roll")
        physical, native = pair["physical"], pair["native"]
        alpha = _finite(physical["alpha_deg"], "physical alpha")
        expected = next(case for case in paired_conditions(roll) if case["physical"]["alpha_deg"] == alpha)
        for label, supplied in (("physical", physical), ("native", native)):
            for key in ("mach", "alpha_deg", "beta_deg"):
                if not _printed(_finite(supplied[key], label+" "+key), expected[label][key], 12):
                    raise ValueError("supplied condition is not one complete paired frame condition")
            wind = supplied["wind_xyz"]
            if not isinstance(wind, (list, tuple)) or len(wind) != 3:
                raise ValueError("paired wind must have three coordinates")
            if any(not _printed(_finite(a, "paired wind"), b, 12) for a,b in zip(wind,expected[label]["wind_xyz"])):
                raise ValueError("supplied wind does not match paired condition")
        return expected
    except (KeyError, TypeError, AttributeError, StopIteration) as error:
        raise ValueError("one complete paired_conditions entry is required") from error


def _read(path):
    try:
        path = Path(path)
        raw = path.read_bytes()
        return raw.decode("utf-8"), {"path": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest()}
    except (OSError, UnicodeError, TypeError) as error:
        raise ValueError("missing or unreadable native case file") from error


def _condition(row, expected, decimals):
    for column, key in _CONDITIONS:
        if not _printed(row[column], expected[key], decimals):
            raise ValueError("native case has an unpaired or unexpected "+column)


def _polar(text, expected):
    lines = [line.split() for line in text.splitlines() if line.strip()]
    if lines and lines[0][:5] == ["Surface","Integration","Forces","and","Moments"]:
        if len(lines)<2 or not lines[1][0].startswith("Surf-Surf"):
            raise ValueError("incomplete native polar explanatory banner")
        lines = lines[2:]
    if len(lines) != 2 or lines[0] != _POLAR or len(lines[1]) != len(_POLAR):
        raise ValueError("exactly one complete pinned 7.2.2 polar row is required")
    row = {key:_finite(value,"polar "+key) for key,value in zip(_POLAR,lines[1])}
    _condition(row,expected,12)
    angle, beta = math.radians(row["AoA"]), math.radians(row["Beta"])
    lift = -row["CFix"]*math.sin(angle)+row["CFiz"]*math.cos(angle)
    drag = (row["CFix"]*math.cos(angle)+row["CFiz"]*math.sin(angle))*math.cos(beta)-row["CFiy"]*math.sin(beta)
    if not _loads_match(lift,row["CLi"]) or not _loads_match(drag,row["CDi"]):
        raise ValueError("native polar wind coefficients disagree with full body force")
    return row


def _history(text, expected, bref, cref, reference=None):
    metadata, rows = {}, []
    started = header = False
    fields_expected = {"Mach_":("mach","no_unit"),"AoA_":("alpha_deg","deg"),"Beta_":("beta_deg","deg"),
                       "Bref_":("bref","Lunit"),"Cref_":("cref","Lunit")}
    values = {**expected,"bref":bref,"cref":cref}
    if reference is not None:
        fields_expected.update({"Sref_":("area","Lunit^2"),"Xcg_":("x","Lunit"),
                                "Ycg_":("y","Lunit"),"Zcg_":("z","Lunit")})
        values.update(reference)
    for line in text.splitlines():
        textline = line.strip()
        if not textline:
            continue
        if set(textline) == {"*"}:
            if started:
                raise ValueError("extra history case or trailing case separator")
            continue
        fields = textline.split()
        if fields[0] in fields_expected:
            key = fields[0]; valuekey,unit = fields_expected[key]
            if started or key in metadata or len(fields)!=3 or fields[2]!=unit:
                raise ValueError("invalid or duplicate native condition/reference metadata")
            value = _finite(fields[1],"history "+key)
            if not _printed(value,values[valuekey],7):
                raise ValueError("history condition/reference metadata disagrees with intended case")
            metadata[key] = value
            continue
        if textline.startswith("Solver Case:"):
            if started or textline != "Solver Case: 1" or set(metadata)!=set(fields_expected):
                raise ValueError("exactly one complete native Solver Case: 1 is required")
            started = True
            continue
        if not started:
            if fields[0] in ("Iter","Time") or re.match(r"^[+-]?(?:\d|\.\d)",textline):
                raise ValueError("orphan history numeric table")
            continue
        if fields[0] in ("Iter","Time"):
            normalized = textline.replace("L2 Residual","L2Residual").replace("Max Residual","MaxResidual").split()
            if header or normalized != _COLUMNS:
                raise ValueError("unsupported or duplicate pinned steady history header")
            header = True
            continue
        if not header:
            if textline.startswith(("Surface Integration Forces","Surf-Surf")):
                continue
            raise ValueError("native history has no supported steady table")
        if len(fields)!=len(_COLUMNS):
            raise ValueError("incomplete or extra native history row")
        row = {key:_finite(value,"history "+key) for key,value in zip(_COLUMNS,fields)}
        # The native timing diagnostic can decrease between adjacent rows;
        # actual iteration IDs, not this clock, establish table ordering.
        if row["Iter"] != len(rows)+1 or row["Wall_Time"]<0:
            raise ValueError("history requires contiguous native iterations and nonnegative timing")
        _condition(row,expected,12)
        rows.append(row)
    if not started or not header or not rows:
        raise ValueError("one complete steady history is required")
    return rows,metadata


def _log(text, expected, last_iteration):
    versions = re.findall(r"^VSPAERO v\.(\S+)",text,re.M)
    if versions != ["7.2.2"] or "DLB Error" in text:
        raise ValueError("one error-free pinned VSPAERO 7.2.2 log is required")
    banner = r"Solving\.\.\.\s*Mach:\s*"+_NUMBER+r"\s*\.\.\.\s*Alpha:\s*"+_NUMBER+r"\s*\.\.\.\s*Beta:\s*"+_NUMBER
    starts = list(re.finditer(banner,text))
    if len(starts)!=1 or text.count("Solving...")!=1:
        raise ValueError("exactly one native solver condition is required")
    _condition({key:_finite(starts[0][i],"log condition") for i,(key,_) in enumerate(_CONDITIONS,1)},expected,6)
    kutta = [int(v) for v in re.findall(r"NumberOfKuttaNodes after trimming:\s*(\d+)",text)]
    wakes = [int(v) for v in re.findall(r"NumberOfWakeLoops:\s*(\d+)",text)]
    if len(kutta)!=1 or len(wakes)!=1 or min(kutta+wakes)<=0:
        raise ValueError("single native lifting-wake setup is missing")
    pattern = (r"Wake Iter:\s*(\d+)\s*/\s*(\d+).*?GMRES Iter:\s*(\d+).*?Red:\s*"+_NUMBER+
               r"\s*/\s*"+_NUMBER+r".*?Max:\s*"+_NUMBER+r"\s*/\s*"+_NUMBER)
    groups,previous,totals = {},0,set()
    for line in text[starts[0].end():].splitlines():
        if "Wake Iter:" not in line:
            continue
        match = re.search(pattern,line)
        if not match:
            raise ValueError("malformed or nonfinite native GMRES row")
        wake,total,iteration = map(int,match.group(1,2,3))
        if wake not in (previous,previous+1) or wake<1 or total<1 or wake>total:
            raise ValueError("native wake solves are missing, duplicated or out of order")
        if wake in groups and iteration<=groups[wake]["gmres_iteration"]:
            raise ValueError("native GMRES iterations are not increasing")
        reduction,target,maximum,max_target = (_finite(match[i],"GMRES residual") for i in range(4,8))
        groups[wake] = {"wake":wake,"total":total,"gmres_iteration":iteration,"reduction":reduction,
                        "reduction_target":target,"maximum":maximum,"maximum_target":max_target}
        previous = wake; totals.add(total)
    endings = list(re.finditer(r"Total setup and solve time:\s*"+_NUMBER+r"\s*seconds",text))
    if (len(totals)!=1 or set(groups)!=set(range(1,last_iteration+1)) or len(endings)!=1
            or endings[0].start()<text.rfind("Wake Iter:")):
        raise ValueError("truncated log or history/GMRES iteration mismatch")
    if _finite(endings[0][1],"native elapsed time")<0:
        raise ValueError("native elapsed time must be nonnegative")
    passed = all(row["reduction"]<=row["reduction_target"] and row["maximum"]<=row["maximum_target"] for row in groups.values())
    return {"gmres_targets_met_every_wake":passed,"kutta_nodes":kutta[0],"wake_loops":wakes[0],
            "requested_wake_iterations":next(iter(totals)),"completed_wake_iterations":last_iteration,
            "final_wake_targets":list(groups.values())}


def assess_case(polar_path, history_path, log_path, paired_condition, *, bref, cref, reference_area=None, native_cg=None):
    """Validate one native case; structurally valid nonconvergence returns false.

    Invalid, extra, stale or incorrectly paired input raises ValueError. The
    sole native nonlinear criterion stays -3, corresponding to factor one.
    Caller provenance must also retain the actually used config and process
    result; a parser cannot prove filesystem freshness or successful launch.
    """
    intended = _intended(paired_condition)
    bref,cref = _finite(bref,"Bref"),_finite(cref,"Cref")
    if bref<=0 or cref<=0:
        raise ValueError("positive moment reference lengths are required")
    reference = None
    if reference_area is not None or native_cg is not None:
        area = _finite(reference_area,"Sref")
        if area<=0 or not isinstance(native_cg,(list,tuple)) or len(native_cg)!=3:
            raise ValueError("area and the complete native moment origin are required together")
        reference = dict(zip(("x","y","z"),(_finite(value,"native CG") for value in native_cg)))
        reference["area"] = area
    files,texts = {},{}
    for name,path in (("polar",polar_path),("history",history_path),("log",log_path)):
        texts[name],files[name] = _read(path)
    polar = _polar(texts["polar"],intended["native"])
    rows,metadata = _history(texts["history"],intended["native"],bref,cref,reference)
    last = rows[-1]
    for key in (*_SIX,"CLi","CDi"):
        if not _loads_match(last[key],polar[key]):
            raise ValueError("current history and polar inviscid loads disagree: "+key)
    linear = _log(texts["log"],intended["native"],int(last["Iter"]))
    loads = recover_loads({key:polar[key] for key in _SIX},intended["roll_deg"],bref=bref,cref=cref)
    original = loads["original"]
    cd = math.fsum(original[key]*wind for key,wind in zip(_SIX[:3],intended["physical"]["wind_xyz"]))
    if not _loads_match(cd,polar["CDi"]):
        raise ValueError("recovered wind drag does not preserve native force/wind covariance")
    nonlinear = last["MaxResidual"]<=_NONLINEAR_LIMIT
    return {"status":"computed","basis":"near_surface_inviscid","model_accepted":False,
            "mach":intended["physical"]["mach"],"alpha_deg":intended["physical"]["alpha_deg"],"beta_deg":0.0,
            "coefficients":{"ca":original["CFix"],"cn":original["CFiz"],"cm":original["CMiy"]},
            "wind_cd":cd,"loads":loads,"paired_condition":intended,"raw_files":files,
            "reference_check":None if reference is None else {"passed":True,"expected":reference,
                "native_metadata":{key:metadata[key] for key in ("Sref_","Xcg_","Ycg_","Zcg_")}},
            "convergence_assessed":True,"nonlinear_converged":nonlinear,
            "gmres_targets_met_every_wake":linear["gmres_targets_met_every_wake"],
            "numerically_converged":nonlinear and linear["gmres_targets_met_every_wake"],"linear":linear,
            "history":{"iterations":len(rows),"final_iteration":int(last["Iter"]),"metadata":metadata,
                       "l2_residual_log10":last["L2Residual"],"max_residual_log10":last["MaxResidual"],
                       "threshold_log10":_NONLINEAR_LIMIT,"final_native_inviscid_loads":{key:last[key] for key in _SIX}}}
