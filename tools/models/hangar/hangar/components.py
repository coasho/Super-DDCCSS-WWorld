"""Frozen payload component inputs. These declarations do not calibrate hardware."""
import copy
import math
from pathlib import PurePosixPath
import struct
AUTOPILOT = "fsim.control.load_feedback_cascade"
ACTUATOR_AWARE_AUTOPILOT = "fsim.control.actuator_aware_cascade.v1"
ALLOCATOR = "fsim.allocator.jsbsim_authored_fin_mixer"
ACTUATOR = "fsim.actuator.jsbsim_lag_rate_stops"
PROPULSION = "fsim.propulsion.jsbsim_generated_solid_rocket"
DUCTED_PROPULSION = "fsim.propulsion.jsbsim_ducted_rocket.v1"
TURBOFAN_PROPULSION = "fsim.propulsion.jsbsim_turbofan.v1"
AERODYNAMICS = "fsim.aerodynamics.jsbsim_generated_store"

def resolve(card, kind):
    if "component_models" not in card:
        return None
    block = card["component_models"]
    if not isinstance(block, dict) or set(block)!={"schema_version", "components"} or type(block.get("schema_version")) is not int or block["schema_version"] != 1:
        raise ValueError("components: unsupported component_models schema")
    rows = block.get("components")
    if not isinstance(rows, dict) or kind not in rows or not isinstance(rows[kind], dict):
        raise ValueError("components: missing explicit " + kind)
    row = copy.deepcopy(rows[kind])
    if row.get("availability") not in ("available", "unavailable", "not_applicable"):
        raise ValueError("components: invalid availability")
    if row["availability"] != "available":
        if set(row) != {"availability", "reason"} or not isinstance(row.get("reason"), str) or not row["reason"]:
            raise ValueError("components: unavailable/not-applicable component needs only a reason")
        return row | {"kind": kind, "configuration_version": 1, "implementation_id": "", "parameters": {},
                      "parameter_refs": {}, "bindings": {}}
    if type(row.get("configuration_version")) is not int or row["configuration_version"] != 1:
        raise ValueError("components: unsupported configuration_version")
    if set(row)-{"availability", "implementation_id", "configuration_version", "parameter_refs", "bindings", "basis", "source", "note", "validity"}:
        raise ValueError("components: unknown component field")
    if row.get("basis") not in ("published", "derived", "assumed") or any(
            not isinstance(row.get(key), str) or not row[key] for key in ("implementation_id", "note", "validity")):
        raise ValueError("components: available implementation needs provenance and validity")
    refs, bindings = row.get("parameter_refs", {}), row.get("bindings", {})
    if not isinstance(refs, dict) or not isinstance(bindings, dict) or any(
            not isinstance(k, str) or not k or not isinstance(v, str) or not v for m in (refs, bindings) for k, v in m.items()):
        raise ValueError("components: references and bindings must be nonempty strings")
    if row["availability"] != "available" and not row.get("reason"):
        raise ValueError("components: unavailable/not-applicable component needs a reason")
    values = {}
    for role, ref in refs.items():
        parameter = card.get("parameters", {}).get(ref)
        if not isinstance(parameter, dict):
            raise ValueError("components: missing referenced parameter " + ref)
        value = parameter.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("components: parameter must be finite numeric")
        try:
            value = float(value)
        except (OverflowError, ValueError) as error:
            raise ValueError("components: parameter cannot be represented") from error
        if not math.isfinite(value) or not isinstance(parameter.get("unit"), str) or not parameter["unit"]:
            raise ValueError("components: parameter needs finite value and unit")
        values[role] = copy.deepcopy(parameter) | {"value": value}
    row.update(kind=kind, parameters=values, parameter_refs=refs, bindings=bindings)
    return row


def _available(row, implementation, roles, bindings, card):
    if row["availability"] != "available":
        raise ValueError("components: %s is %s: %s" % (row["kind"], row["availability"], row["reason"]))
    if row.get("implementation_id") != implementation:
        raise ValueError("components: unsupported " + row["kind"] + " implementation")
    if set(row["parameter_refs"]) != set(roles) or set(row["bindings"]) != set(bindings):
        raise ValueError("components: unexpected or missing parameter/binding slots for " + row["kind"])
    if row["bindings"].get("model") != card.get("model") or not card.get("model", "").startswith("jsbsim:"):
        raise ValueError("components: native model identity mismatch")
    for key in ("design", "motor", "nozzle", "table"):
        if key in row["bindings"]:
            path = row["bindings"][key]
            if ("\\" in path or ":" in path or PurePosixPath(path).is_absolute() or
                any(p in ("", ".", "..") or p.casefold() == ".codex" for p in path.split("/"))):
                raise ValueError("components: asset binding must be a canonical permitted relative path")
    name = card["model"][7:]
    if "design" in row["bindings"] and row["bindings"]["design"] != "aircraft/%s/%s.toml" % (name, name):
        raise ValueError("components: authored design identity mismatch")


def number(descriptor, role, unit):
    row = descriptor["parameters"].get(role)
    if row is None or row["unit"] != unit:
        raise ValueError("components: %s needs %s" % (role, unit))
    return row["value"]


def fingerprint(descriptor):
    """FNV-1a32, length-prefixed UTF-8 and little-endian IEEE binary64."""
    data = bytearray()
    def text(value):
        raw = value.encode("utf-8")
        data.extend(struct.pack("<I", len(raw))); data.extend(raw)
    text(descriptor["kind"]); text(descriptor["implementation_id"])
    data.extend(struct.pack("<I", descriptor["configuration_version"]))
    values = descriptor["parameters"]
    data.extend(struct.pack("<I", len(values)))
    for role, row in sorted(values.items()):
        text(role); text(row["unit"]); data.extend(struct.pack("<d", row["value"]))
    for key in ("parameter_refs", "bindings"):
        data.extend(struct.pack("<I", len(descriptor[key])))
        for role, value in sorted(descriptor[key].items()): text(role); text(value)
    value = 2166136261
    for byte in data: value = ((value ^ byte) * 16777619) & 0xffffffff
    return value

def actuator(card):
    row = resolve(card, "actuator")
    if row is None:
        return None
    _available(row, ACTUATOR, ("lag_s", "rate_rad_s"), ("model",), card)
    lag, rate = number(row, "lag_s", "s"), number(row, "rate_rad_s", "rad/s")
    if lag <= 0 or rate <= 0 or not math.isfinite(1 / lag):
        raise ValueError("components: actuator lag/rate must be positive representable values")
    return {"version": 1, "lag_coefficient": 1 / lag, "rate_rad_s": rate,
            "sha256": "%08x" % fingerprint(row)}


def control(card):
    row = resolve(card, "autopilot")
    if row is None: return None
    mode = row["bindings"].get("execution_mode")
    roles = {"max_incidence_deg": "deg", "rate_time_min_s": "s", "rate_lag_multiplier": "1",
             "incidence_time_min_s": "s", "incidence_rate_multiplier": "1"} if mode == "configured" else {}
    bindings=['model','execution_mode']
    if 'thrust_vector' in row['bindings']:
        from . import thrustvector
        thrustvector.inputs(card)
        roles.update(tvc_limit_rad='rad',tvc_rate_rad_s='rad/s');bindings.append('thrust_vector')
    implementation=row['implementation_id']
    if implementation not in (AUTOPILOT,ACTUATOR_AWARE_AUTOPILOT):
        raise ValueError('components: unsupported autopilot')
    _available(row, implementation, roles, bindings, card)
    if implementation==ACTUATOR_AWARE_AUTOPILOT and mode!='configured':
        raise ValueError('components: actuator-aware control requires configured native dynamics')
    if mode not in ("legacy", "configured") or (mode == "legacy" and "weapon_execution_version" in card.get("parameters", {})):
        raise ValueError("components: invalid or contradictory autopilot execution mode")
    for role, unit in roles.items():
        value = number(row, role, unit)
        if value <= 0 or (role == "max_incidence_deg" and value >= 90):
            raise ValueError("components: autopilot value outside supported domain")
    _available(resolve(card, "allocator"), ALLOCATOR, (), ("model", "design"), card)
    return row

def migration(card, design):
    additions, rows = {}, {}
    model = card.get("model", "")
    name = model.removeprefix("jsbsim:")
    source = "aircraft/%s/%s.toml" % (name, name)
    def descriptor(kind, implementation, refs=None, bindings=None, availability="available", reason=""):
        if availability != "available":
            rows[kind] = {"availability": availability, "reason": reason}
            return
        rows[kind] = {"availability": availability, "implementation_id": implementation,
            "configuration_version": 1, "parameter_refs": refs or {}, "bindings": bindings or {},
            "basis": "assumed", "note": "Existing simulator implementation and unchanged authored inputs; no equipment calibration.",
            "validity": "Reduced-order engineering model; hardware performance unvalidated."}
    def parameter(key, value, unit):
        if key not in card.get("parameters", {}):
            additions[key] = {"value": value, "unit": unit, "uncertainty": 0,
                "basis": "assumed", "note": "Transcribed existing consumed simulator input; uncertainty is not quantified, zero does not assert measurement accuracy."}
        return key
    controlled = any(s.get("controls") for s in (design or {}).get("surface", []))
    if controlled:
        versioned = card.get("parameters", {}).get("weapon_execution_version", {}).get("value") == 1
        existing = card.get("component_models", {}).get("components", {}).get("autopilot", {})
        current_refs = existing.get("parameter_refs", {}) if existing.get("bindings", {}).get("execution_mode") == "configured" else {}
        # Transcribe the current generic controller defaults, not equipment
        # identification or a new execution/physical model. Separate names
        # avoid inventing a partial legacy top-level execution block.
        defaults = {"max_incidence_deg": (15., "deg"), "rate_time_min_s": (.12, "s"),
                    "rate_lag_multiplier": (3., "1"), "incidence_time_min_s": (.4, "s"),
                    "incidence_rate_multiplier": (3.333333333333333, "1")}
        roles = {role: parameter(current_refs.get(role, ("control_" if versioned else "component_control_") + role), value, unit)
                 for role, (value, unit) in defaults.items()}
        descriptor("autopilot", AUTOPILOT, roles, {"model": model, "execution_mode": "configured"})
        descriptor("allocator", ALLOCATOR, bindings={"model": model, "design": source})
        refs = {"lag_s": parameter("control_actuator_lag_s" if versioned else "plant_actuator_lag_s", .04, "s"),
                "rate_rad_s": parameter("control_actuator_rate_rad_s" if versioned else "plant_actuator_rate_rad_s", 4., "rad/s")}
        descriptor("actuator", ACTUATOR, refs, {"model": model})
    elif card.get("class") in ("aam","cruise_missile") or card.get("seeker"):
        for kind in ("autopilot", "allocator", "actuator"):
            descriptor(kind, "", availability="unavailable", reason="No executing authored control plant is available for this guided store.")
    else:
        for kind in ("autopilot", "allocator", "actuator"):
            descriptor(kind, "", availability="not_applicable", reason="No authored native control surfaces on this released store.")
    native_motor = (design or {}).get("motor")
    if native_motor and native_motor.get("phase"):
        values = {"isp_s": (255. if native_motor.get("smoke") == "aluminised" else 245., "s"),
            "coast_thrust_n": (2., "N"), "buildup_s": (.05, "s"), "grain_density_kg_m3": (1750., "kg/m^3"),
            "grain_radius_fraction": (.9, "1"), "grain_port_area_fraction": (.15, "1"), "grain_aft_clearance_m": (.03, "m"),
            "phase_count": (len(native_motor["phase"]), "1"), "exit_diameter_m": (native_motor["exit_diameter"], "m"),
            "throat_diameter_m": (native_motor.get("throat_diameter", .4*native_motor["exit_diameter"]), "m"),
            "cone_length_m": (native_motor.get("cone_length", 1.2*native_motor["exit_diameter"]), "m")}
        for axis, value in zip("xyz", native_motor["exit"]): values["exit_%s_m" % axis] = (value, "m")
        for i, phase in enumerate(native_motor["phase"]):
            values["phase_%d_duration_s" % i] = (phase["duration_s"], "s")
            values["phase_%d_thrust_kn" % i] = (phase["thrust_kn"], "kN")
        refs = {role: parameter("plant_motor_" + role, value, unit) for role, (value, unit) in values.items()}
        descriptor("propulsion", PROPULSION, refs, {"model": model, "design": source, "motor": "Engines/%s_motor.xml" % name,
                   "nozzle": "Engines/%s_nozzle.xml" % name, "smoke": native_motor.get("smoke", "reduced")})
    elif (design or {}).get('engine') or 'store_propulsion' in (design or {}):
        descriptor('propulsion','',availability='unavailable',
            reason='An authored air-breathing engine requires an explicit versioned propulsion component.')
    elif design is None and card.get("class") in ("aam","cruise_missile"):
        descriptor("propulsion", "", availability="unavailable", reason="No authored native motor design is available.")
    else:
        descriptor("propulsion", "", availability="not_applicable", reason="The authored released configuration has no rocket motor or air-breathing engine.")
    if model.startswith("jsbsim:") and (design is not None or card.get("class") in ("tank", "pod", "cargo")):
        aero_source = "vlm"
        bindings = {"model": model, "design": source, "source": aero_source}
        descriptor("aerodynamics", AERODYNAMICS, bindings=bindings)
    else:
        descriptor("aerodynamics", "", availability="unavailable", reason="No generated native aerodynamic model is available.")
    # Migration fills missing declarations; it cannot downgrade a selected
    # physical backend or discard optional native vector/cycle inputs.
    existing=card.get('component_models',{}).get('components',{})
    for kind in rows:
        if kind in existing:rows[kind]=copy.deepcopy(existing[kind])
    if native_motor and native_motor.get('kind')=='ducted_rocket' and rows['propulsion'].get('implementation_id')!=DUCTED_PROPULSION:
        descriptor('propulsion','',availability='unavailable',
            reason='An explicitly typed ducted rocket requires declared cycle and controller inputs.')
    if (design or {}).get('engine') or 'store_propulsion' in (design or {}):
        config=(design or {}).get('store_propulsion',{});engines=(design or {}).get('engine',[])
        if ((design or {}).get('motor') or not isinstance(config,dict) or type(config.get('version')) is not int or config['version']!=1 or
                not isinstance(engines,list) or len(engines)!=1 or not isinstance(engines[0],dict) or engines[0].get('type')!='turbofan' or
                rows['propulsion'].get('availability')!='available' or
                rows['propulsion'].get('implementation_id')!=TURBOFAN_PROPULSION):
            descriptor('propulsion','',availability='unavailable',
                reason='Authored engine hardware requires a supported versioned source and its explicit propulsion component.')
    return {"id": card.get("id", ""), "parameters": additions, "components": rows}

def motor(card, design):
    native_motor=design.get('motor',{})
    if not isinstance(native_motor,dict):
        raise ValueError('motor: declaration must be a mapping')
    row = resolve(card, "propulsion")
    if design.get('engine') or 'store_propulsion' in design:
        from . import turbofanstore
        turbofanstore.inputs(card,design)
        return design,None
    if native_motor.get('kind')=='ducted_rocket' and (row is None or row.get('implementation_id')!=DUCTED_PROPULSION):
        raise ValueError('components: explicitly typed ducted rocket requires its declared cycle/controller profile')
    if row is None: return design, None
    if row.get('implementation_id')==TURBOFAN_PROPULSION:
        from . import turbofanstore
        turbofanstore.inputs(card,design)
        return design,None
    if row["availability"] == "not_applicable":
        if native_motor.get("phase"): raise ValueError("components: executing motor cannot be not_applicable")
        return design, None
    values = row["parameters"]
    count = number(row, "phase_count", "1")
    if count < 1 or count > 1024 or count != int(count): raise ValueError("components: invalid phase_count")
    roles = {"isp_s", "coast_thrust_n", "buildup_s", "grain_density_kg_m3", "grain_radius_fraction",
        "grain_port_area_fraction", "grain_aft_clearance_m", "phase_count", "exit_x_m", "exit_y_m", "exit_z_m",
        "exit_diameter_m", "throat_diameter_m", "cone_length_m"}
    roles.update("phase_%d_%s" % (i, suffix) for i in range(int(count)) for suffix in ("duration_s", "thrust_kn"))
    ducted=row.get('implementation_id')==DUCTED_PROPULSION
    if ducted:
        from . import ductedrocket
        roles.update(ductedrocket.ROLES)
        if count!=2:raise ValueError('components: ducted propulsion needs booster and fuel-ledger phases')
        ductedrocket.inputs(card)
    _available(row, DUCTED_PROPULSION if ducted else PROPULSION, roles, ("model", "design", "motor", "nozzle", "smoke"), card)
    name = card["model"][7:]
    if row["bindings"]["motor"] != "Engines/%s_motor.xml" % name or row["bindings"]["nozzle"] != "Engines/%s_nozzle.xml" % name:
        raise ValueError("components: motor/nozzle asset identity mismatch")
    units = {"isp_s": "s", "coast_thrust_n": "N", "buildup_s": "s", "grain_density_kg_m3": "kg/m^3",
        "grain_radius_fraction": "1", "grain_port_area_fraction": "1", "grain_aft_clearance_m": "m"}
    settings = {key: number(row, key, unit) for key, unit in units.items()}
    if any(value <= 0 for key, value in settings.items() if key != "grain_aft_clearance_m") or settings["grain_aft_clearance_m"] < 0:
        raise ValueError("components: motor settings outside supported domain")
    if settings["grain_radius_fraction"] > 1 or settings["grain_port_area_fraction"] >= 1:
        raise ValueError("components: invalid cylindrical grain fractions")
    out = copy.deepcopy(design)
    selected = copy.deepcopy(out.get("motor", {}))
    if ducted:selected['kind']='ducted_rocket'
    selected.update(exit=[number(row, "exit_%s_m" % axis, "m") for axis in "xyz"],
        exit_diameter=number(row, "exit_diameter_m", "m"), throat_diameter=number(row, "throat_diameter_m", "m"),
        cone_length=number(row, "cone_length_m", "m"), smoke=row["bindings"]["smoke"],
        phase=[{"duration_s": number(row, "phase_%d_duration_s" % i, "s"),
                "thrust_kn": number(row, "phase_%d_thrust_kn" % i, "kN")} for i in range(int(count))])
    if selected["exit"][1:] != [0., 0.]:
        raise ValueError("components: generated axial rocket does not support transverse nozzle offsets")
    from .motorinputs import validate_motor
    validate_motor(selected)
    out["motor"] = selected
    return out, settings


def contact_effects_v1(card,*,arming_s,life_s,source,terrain_contact=False):
    """Author explicit contact-only native effects from supplied source inputs.

    No equipment identity, charge estimate or applicability is inferred here.
    Callers supply the physical warhead parameters and project configuration.
    Native factories still validate/admit execution at actual release.
    """
    if source not in card.get('sources',{}):raise ValueError('Effects require an explicit source/derivation record')
    if card.get('warhead_kind') not in ('fragmentation','rod'):raise ValueError('Effects require an authored warhead kind')
    out=copy.deepcopy(card);parameters=out.setdefault('parameters',{})
    for key,unit in (('warhead_kg','kg'),('explosive_kg','kg'),('gurney_kms','km/s'),('diameter_m','m')):
        p=parameters.get(key,{})
        if p.get('unit')!=unit or not isinstance(p.get('value'),(int,float)) or not math.isfinite(float(p['value'])):
            raise ValueError('Missing canonical effects input '+key)
    if not 0<parameters['explosive_kg']['value']<parameters['warhead_kg']['value'] or parameters['gurney_kms']['value']<=0 or parameters['diameter_m']['value']<=0:
        raise ValueError('Effects mass/geometry inputs outside their supported domain')
    if not math.isfinite(arming_s) or not math.isfinite(life_s) or not 0<=arming_s<life_s:raise ValueError('Invalid contact fuze timing')
    refs={}
    for role,value,unit in (('model_version',0,'1'),('algorithm',0,'1'),('arming_s',arming_s,'s'),('life_s',life_s,'s'),('radius_m',0,'m')):
        key='component_contact_fuze_'+role;refs[role]=key
        parameters[key]=dict(value=value,unit=unit,basis='assumed',uncertainty=0 if role in ('model_version','algorithm','radius_m') else value,
            source=source,note='Explicit project contact-only model setting; estimated, not measured hardware; zero is a selection/absent proximity channel, not an accuracy claim.')
    common=dict(availability='available',configuration_version=1,basis='assumed',source=source,
        note='Explicit native contact/fragment-blast engineering configuration.',
        validity='Engineering virtual configuration only; no fitted fuze timing or measured fragment-pattern accuracy.')
    rows=out['component_models']['components']
    rows['fuze']=dict(common,implementation_id='native.fuze.swept_path.v1',parameter_refs=refs,
        bindings=dict(sensor_reference='captured_weapon_cg',target_reference='captured_target_cg',
            contact_reference='captured_authored_nose',contact='native.authored_point_contact.v1',
            history='native.captured_piecewise_linear.v1',delay='none'))
    rows['warhead']=dict(common,implementation_id='native.warhead.zoned_case.v1',
        parameter_refs={key:key for key in ('warhead_kg','explosive_kg','gurney_kms','diameter_m')},
        bindings=dict(burst_reference='captured_weapon_cg',blast='kingery_bulmash.swisdak_1994.free_air.v1',
            penetration='thor.aluminium_2024_t3.v1',assessment='native.fragment_blast.v1'))
    if type(terrain_contact) is not bool:raise ValueError('Terrain contact is an explicit versioned sensor selection')
    if terrain_contact:
        rows['fuze']['bindings']['terrain_contact']='native.structure_contact.v1'
        rows['fuze']['note']+=' Authored STRUCTURE-support contact initiates the same one-shot channel; this is an estimated configuration, not a nose-only or case-only hardware claim.'
    return out


def coefficients(card, geo):
    from . import airframe
    row=resolve(card,'aerodynamics')
    if row is not None:
        _available(row,AERODYNAMICS,(),('model','design','source'),card)
        if row['bindings']['source']!='vlm':
            raise ValueError('components: the aircraft VLM source is mandatory')
    return airframe.coefficients(geo['design_path'])


def diagnostic_source(card, design_path):
    row=resolve(card,'aerodynamics')
    if row is not None:
        _available(row,AERODYNAMICS,(),('model','design','source'),card)
        if row['bindings'].get('source')!='vlm':
            raise ValueError('components: the aircraft VLM source is mandatory')
    return 'vlm'


def diagnostic_coefficients(card,geo):
    return coefficients(card,geo)


def stamp(xml, card):
    """Publish each selected native descriptor once, including empty FCS shells."""
    if 'component_models' not in card:return xml
    import xml.etree.ElementTree as ET
    root=ET.fromstring(xml);fcs=root.find('flight_control')
    if fcs is None:fcs=ET.SubElement(root,'flight_control',name='Native component metadata')
    for node in list(fcs):
        if node.tag=='property' and (node.text or '').startswith('fsim/components/'):
            fcs.remove(node)
    for kind,implementation in zip(('autopilot','allocator','actuator','propulsion','aerodynamics'),
                                  (AUTOPILOT,ALLOCATOR,ACTUATOR,PROPULSION,AERODYNAMICS)):
        row=resolve(card,kind)
        if row['availability']!='available':continue
        code=1
        if kind=='autopilot' and row['implementation_id']==ACTUATOR_AWARE_AUTOPILOT:
            implementation=ACTUATOR_AWARE_AUTOPILOT;code=2
        if kind=='propulsion' and row['implementation_id']==DUCTED_PROPULSION:
            implementation=DUCTED_PROPULSION;code=2
        if kind=='propulsion' and row['implementation_id']==TURBOFAN_PROPULSION:
            implementation=TURBOFAN_PROPULSION;code=3
        if row['implementation_id']!=implementation:
            raise ValueError('components: unsupported generated native implementation')
        for leaf,value in (('schema-version',1),('implementation-code',code),('profile-fingerprint',fingerprint(row))):
            ET.SubElement(fcs,'property',value=str(value)).text='fsim/components/'+kind+'/'+leaf
    ET.indent(root,space='    ')
    return ET.tostring(root,encoding='unicode')


def proposals(root):
    """Read maintained sources and return proposed additions; never mutate cards."""
    import json
    import re
    import tomllib
    from pathlib import Path
    root = Path(root)
    result = []
    for path in sorted((root / "assets" / "stores").glob("*.json")):
        raw = json.loads(path.read_text(encoding="utf-8"))
        name = raw.get("model", "").removeprefix("jsbsim:")
        design = None
        if re.fullmatch(r"[A-Za-z0-9_-]+", name):
            source = root / "aircraft" / name / (name + ".toml")
            if source.is_file(): design = tomllib.loads(source.read_text(encoding="utf-8"))
        result.append(migration(raw, design))
    return {"schema_version": 1, "stores": result}


if __name__ == "__main__":
    import argparse
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser(description="Propose unchanged payload plant card inputs without editing cards")
    parser.add_argument("--migration-out", required=True)
    args = parser.parse_args()
    output = Path(args.migration_out)
    if not output.is_absolute(): output = Path.cwd() / output
    root = Path(__file__).resolve().parents[4]
    allowed = root / "build" / "architecture-reviews"
    if not output.resolve().is_relative_to(allowed.resolve()):
        raise SystemExit("migration output must remain in build/architecture-reviews")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(proposals(root), indent=2) + "\n", encoding="utf-8")
