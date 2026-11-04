"""Admit 24 identifiable native AAM chains in both current build variants."""
import argparse
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import stat
import xml.etree.ElementTree as ET

from acceptance import allowed, verify_artifact
from component_matrix import COMPONENTS, binding, canonical, derive_cells
from component_reference_run import observed_modules, staged_runtime

ROOT = Path(__file__).resolve().parents[3]
SCHEMA = 'fsim.sim-m03.component-native.v1'
PRESETS = ('ucrt64-headless', 'ucrt64-release')
FILES = ('summary.csv', 'trace.csv', 'components.csv', 'fins.csv', 'kinematics.csv', 'events.csv')
GENERATION_SOURCE_PATHS = (
    'cmake/JsbsimPatches.cmake', 'cmake/Dependencies.cmake',
    'tools/models/hangar/hangar/__main__.py', 'tools/models/hangar/hangar/engineering.py',
    'tools/models/hangar/hangar/storebodies.py',
    'tools/models/hangar/hangar/ductedrocket.py', 'tools/models/hangar/hangar/storemodels.py',
    'tools/models/hangar/hangar/storestages.py', 'tools/models/hangar/hangar/jsbsim.py',
    'tools/models/hangar/hangar/airframe.py', 'tools/models/hangar/hangar/components.py',
    'tools/models/hangar/hangar/motorinputs.py', 'tools/models/hangar/hangar/storecontrol.py',
    'tools/models/hangar/hangar/rolleron.py', 'tools/models/hangar/hangar/pipeline.py',
    'tools/models/hangar/hangar/pieces.py', 'tools/models/hangar/hangar/damage.py')


def require(value, message):
    if not value:
        raise ValueError(message)


def checked_path(value, root=None):
    value=allowed(value)
    if root is not None and not value.is_absolute():value=allowed(root)/value
    value=allowed(value.resolve())
    if root is not None:
        require(value.is_relative_to(allowed(root).resolve()) or value.is_relative_to(Path('D:/ENV').resolve()),
                'Foreign native evidence root')
    return value


def digest(path):
    return hashlib.sha256(checked_path(path).read_bytes()).hexdigest()


def read(path):
    def unique(pairs):
        result={}
        for key,value in pairs:
            require(key not in result,'Duplicate native JSON field: '+key)
            result[key]=value
        return result
    return json.loads(checked_path(path).read_text(encoding='utf-8'),object_pairs_hook=unique,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite native JSON: '+value)))


def bound(item,root):
    require(type(item) is dict and set(item)=={'path','sha256'},'Exact native path/hash binding required')
    value=checked_path(item['path'],root)
    verify_artifact(value,item['sha256'])
    return value


def rows(path):
    with checked_path(path).open(encoding='utf-8', newline='') as stream:
        reader=csv.DictReader(stream)
        require(reader.fieldnames and len(set(reader.fieldnames))==len(reader.fieldnames),'Duplicate/missing native CSV header')
        result = list(reader)
    require(result, 'Empty actual native output: '+str(path))
    require(all(None not in row and None not in row.values() for row in result), 'Malformed actual native CSV')
    return result


def number(row, key):
    value = float(row[key])
    require(math.isfinite(value), 'Nonfinite native '+key)
    return value


def validate_case(model, summary, trace, mechanisms, cells):
    """Reconstruct every stage from actual observations, not summary booleans."""
    require(len(summary) == 1, 'Exactly one native release summary required')
    s = summary[0]
    require(s['card'] == model and s['released'] == '1', 'Wrong/unreleased native model')
    require(len(trace) >= 2 and int(s['samples']) == len(trace), 'Incomplete native sample count')
    weapon, generation = int(s['weapon']), int(s['generation'])
    require(weapon > 0 and generation > 0, 'Actual native lifetime identity required')
    require(len(mechanisms) == 9 and {r['component'] for r in mechanisms} == set(COMPONENTS),
            'Missing/duplicate component selection')
    expected = {c['component']: c for c in cells if c['model'] == model}
    require(len(expected) == 9, 'Missing actual component population')
    for item in mechanisms:
        c = expected[item['component']]
        require(item['card'] == model and item['implementation'] == c['implementation_id']
                and item['mechanism'] == c['mechanism_id'], 'Foreign component execution identity')
    observed = {c: False for c in COMPONENTS}
    times = []
    launch = None
    for row in trace:
        require(row['card'] == model and int(row['weapon']) == weapon
                and int(row['generation']) == generation, 'Cross-model/lifetime trace')
        time, age = number(row, 'time'), number(row, 'age')
        times.append(time)
        require(age >= 0, 'Negative weapon age')
        if launch is None:
            launch = time-age
        require(abs((time-age)-launch) <= 1e-9, 'Inconsistent native release epoch')
        for field in ('alpha', 'beta', 'speed', 'fx', 'fy', 'fz', 'l', 'm', 'n',
                      'thrust_n', 'fuel_kg', 'component_energy_j', 'controller_time'):
            number(row, field)
        live = row['control_status'] == '1'
        if live:
            decision = number(row, 'controller_time')
            require(launch <= decision <= time and time-decision <= 1/120+1e-9,
                    'Future/prelaunch/stale native controller decision')
        observed['seeker'] |= row['tracking'] == '1' and (row['thermal_status'] == '1' or row['rf_status'] == '1')
        observed['guidance'] |= row['guidance_status'] == '1' and bool(int(row['guidance_flags']) & 8)
        observed['autopilot'] |= live and number(row, 'controller_time') > 0
        observed['allocator'] |= live and int(row['fin_count']) > 0
        observed['actuator'] |= live and int(row['fin_count']) > 0
        observed['aerodynamics'] |= live and number(row, 'speed') > 0
        observed['propulsion'] |= bool(int(row['propulsion_flags']) & 1) and number(row, 'thrust_n') > 0
        observed['fuze'] |= int(row['fuze_flags']) & (1 | 8) == (1 | 8)
        observed['warhead'] |= int(row['fuze_flags']) & (2 | 16) == (2 | 16) and int(row['fragments']) > 0
    require(all(b > a for a, b in zip(times, times[1:])), 'Reordered/duplicate native epochs')
    require(all(s[c] == '1' and observed[c] for c in COMPONENTS), 'Unexecuted actual component stage')
    require(s['terminal'] == 'detonated' and s['damage'] == '1', 'Native chain did not reach target damage')
    require(number(s, 'component_energy_j') > 0 and max(number(r, 'component_energy_j') for r in trace) > 0,
            'No actual deposited target component work')
    require(int(s['fragments']) > 0 and int(s['damage_records']) > 0
            and number(s, 'burst_time') > times[0]-number(trace[0], 'age'), 'Missing causal burst source')
    number(s, 'terminal_time')
    return dict(model=model, weapon=weapon, generation=generation, samples=len(trace),
                first_time_s=times[0], last_time_s=times[-1],
                burst_time_s=number(s, 'burst_time'), fragments=int(s['fragments']),
                component_work_j=number(s, 'component_energy_j'),
                numeric_trace_sha256=hashlib.sha256(canonical([
                    {k: v for k, v in r.items() if k not in ('card', 'weapon', 'generation', 'time')}
                    for r in trace])).hexdigest())


def executed_case(junit, model):
    found = [c for c in ET.parse(checked_path(junit)).getroot().findall('.//testcase')
             if c.get('name') == 'aam_full_chain.'+model]
    require(len(found) == 1, 'Missing/duplicate native CTest model: '+model)
    case = found[0]
    require(case.get('status')=='run','Explicit executed actual CTest status required')
    require(not any(case.find(k) is not None for k in ('failure', 'error', 'skipped')),
            'Failed/skipped actual native chain: '+model)
    text = case.findtext('system-out', '')
    match = re.search(r'All tests passed \(([\d,]+) assertions? in ([\d,]+) test cases?\)', text)
    require(match is not None, 'Nonzero actual nested native count required: '+model)
    assertions, tests = (int(v.replace(',', '')) for v in match.groups())
    require(assertions > 0 and tests == 1, 'Exactly one positive native Catch case required')
    elapsed=float(case.get('time'))
    require(math.isfinite(elapsed) and elapsed>=0,'Invalid native execution duration')
    return dict(case=case.get('name'), tests=tests, assertions=assertions,
                failures=0, errors=0, skipped=0, elapsed_s=elapsed)


def scene_input_paths(coverage, root=ROOT, airframes=('f16c','fa18c')):
    """Bound actual named scene resources without directory enumeration.

    Resolution follows the maintained Windows JSBSim model/Engines/Systems
    search order, then its configured vendored engine/systems root. Only
    explicit XML links, fit/card names, section-piece names and known optional
    runtime filenames are inspected. This helper also serves public scenes
    with a separately derived explicit carrier set.
    """
    root=allowed(root);root=allowed(root.resolve());names=set();pending=[];visited=set()
    def path(name,required=True):
        name=allowed(name)
        require(not name.is_absolute() and not name.drive and '..' not in name.parts,
                'Scene source must be a contained relative path')
        current=root
        for part in name.parts:
            current=allowed(current/part)
            try:info=current.lstat()
            except FileNotFoundError:
                require(not required,'Missing scene input: '+str(name));return None
            require(not stat.S_ISLNK(info.st_mode) and not getattr(info,'st_file_attributes',0)&0x400,
                    'Scene source reparse point refused')
        resolved=allowed(current.resolve());require(resolved.is_relative_to(root),'Scene input escapes source root')
        require(resolved.is_file(),'Scene input is not a file: '+str(name))
        return resolved
    def add(name,required=True):
        value=path(name,required)
        if value is not None:
            relative=value.relative_to(root).as_posix();names.add(relative)
            require(len(names)<=4096,'Scene source closure exceeds its file bound')
            return relative
        return None
    def identity(value):
        allowed(value)
        require(type(value) is str and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',value),
                'Invalid scene model/card identity')
        return value
    def document(name):
        value=path(name);require(value.stat().st_size<=64*1024*1024,'Scene JSON exceeds its size bound')
        return read(value)
    def xml(name,model_directory):
        relative=add(name);pending.append((relative,model_directory))
    native_pending=[];native_seen=set()
    def native_model(model,directory,required_fit=False):
        native_pending.append((model,directory,required_fit))
    def design_inputs(model):
        directory='aircraft/'+model
        add(directory+'/'+model+'.toml')
        # Normal Aircraft.load/design reads this optional calibration input.
        # Binding it makes no claim of equipment calibration admission.
        add(directory+'/calibration.toml',False)
    # Bind every card read by the native store library, not only the launched
    # AAM. Only AAM native models are loaded by this scene.
    stores=coverage['stores'];require(type(stores) is list and 0<len(stores)<=128,'Bounded actual store inventory required')
    for row in stores:
        card=identity(row['card']);relative=row['card_path']
        require(relative=='assets/stores/'+card+'.json','Foreign scene store card path');add(relative)
        if row['class']!='aam':continue
        model=identity(row['model']);design_inputs(model)
        directory='aircraft/stores/'+model;native_model(model,directory)
        # StoreFlightBinding consumes exactly the source owned by this typed
        # marker, not any other similarly named source/card or table file.
        doc=document(relative)
        require(doc.get('id')==card and doc.get('model')=='jsbsim:'+model,'Carriage source model ownership mismatch')
        marker=doc.get('parameters',{}).get('airframe_aerodynamics_version')
        require(type(marker) is dict and type(marker.get('value')) in (int,float) and marker['value']==1
                and marker.get('unit')=='1','Required native shared carriage marker missing/malformed')
        owner=marker.get('source');sources=doc.get('sources',{})
        require(type(owner) is str and owner and type(sources) is dict and type(sources.get(owner)) is dict,
                'Carriage marker source owner missing')
        link=sources[owner].get('url')
        require(type(link) is str,'Carriage resource URL required');allowed(link)
        match=re.fullmatch(r'aero/vlm-carriage-([0-9a-f]{64})\.json',link)
        require(match is not None,'Foreign native carriage resource URL')
        table=add(directory+'/'+link)
        require(digest(path(table))==match.group(1),'Carriage content-addressed file hash mismatch')
    require(type(airframes) in (tuple,list,set) and len(airframes)<=88,'Bounded explicit carrier/target set required')
    for design in sorted(set(airframes)):
        design=identity(design);directory='aircraft/'+design
        design_inputs(design);native_model(design,directory,True)
    while native_pending:
        design,directory,required_fit=native_pending.pop()
        key=(design,directory)
        if key in native_seen:continue
        native_seen.add(key);require(len(native_seen)<=4096,'Loaded model closure exceeds bound')
        xml(directory+'/'+design+'.xml',directory)
        fit_path=directory+'/payloads.json'
        if add(fit_path,required_fit):
            fit=document(fit_path);require(fit.get('aircraft')==design,'Scene fit identity mismatch')
            payloads=fit.get('payloads',[]);require(type(payloads) is list and len(payloads)<=1024,'Scene fit exceeds payload bound')
            for payload in payloads:add('assets/payloads/'+identity(payload['card'])+'.json')
        # Native World reads damage/material inputs beside the model; damage
        # geometry is embedded. New optional inputs change the next epoch too.
        damage=directory+'/'+design+'.damage.json';material=directory+'/'+design+'.liquid-v1.json'
        if add(damage,False):
            doc=document(damage);declaration=doc.get('liquid_geometry_v1')
            if declaration is not None:
                reference=allowed(declaration['file'])
                require(str(reference)==design+'.liquid-v1.json','Foreign native liquid declaration')
                add(material)
            sections=doc.get('sections',[])
            require(type(sections) is list and len(sections)<=256,'Scene structural sections exceed bound')
            for section in sections:
                slug=identity(section['name'].replace(' ','-'));piece=design+'-'+slug
                piece_dir=directory+'/pieces/'+piece
                if path(piece_dir+'/'+piece+'.xml',False):native_model(piece,piece_dir)
        add(material,False)
        for optional in (directory+'/rcs.bin',directory+'/'+design+'.rcs.bin',
                         'assets/models/'+design+'.rcs.bin','models/'+design+'.rcs.bin',
                         directory+'/'+design+'.glb',directory+'/'+design+'.glb.manifest'):
            add(optional,False)
    while pending:
        name,directory=pending.pop();key=(name,directory)
        if key in visited:continue
        visited.add(key);require(len(visited)<=4096,'Scene XML resource closure exceeds bound')
        value=path(name);require(value.stat().st_size<=64*1024*1024,'Scene XML exceeds its size bound')
        tree=ET.parse(value)
        for node in tree.iter():
            if 'file' not in node.attrib:continue
            reference=allowed(node.get('file'));require(str(reference) and not reference.is_absolute() and
                not reference.drive and '..' not in reference.parts,'Foreign scene XML resource')
            file=str(reference) if reference.suffix=='.xml' else str(reference)+'.xml'
            tag=node.tag.rsplit('}',1)[-1]
            if tag in ('engine','thruster'):
                candidates=[directory+'/Engines/'+file,directory+'/engine/'+file,'third_party/jsbsim/engine/'+file]
            elif tag=='system':
                candidates=[directory+'/'+file,directory+'/Systems/'+file,'third_party/jsbsim/systems/'+file]
            else:candidates=[directory+'/'+file]
            chosen=next((candidate for candidate in candidates if path(candidate,False)),None)
            require(chosen is not None,'Missing scene XML resource: '+file);xml(chosen,directory)
    # Imports are parsed, never executed. Follow only named local Python files
    # of these actual generation roots, with a bounded queue and no glob.
    package='tools/models/hangar/hangar';python=[];seen=set()
    for name in GENERATION_SOURCE_PATHS:
        relative=add(name)
        if relative.endswith('.py'):python.append(relative)
    while python:
        name=python.pop()
        if name in seen:continue
        seen.add(name);require(len(seen)<=512,'Normal generator source closure exceeds bound')
        tree=ast.parse(path(name).read_text(encoding='utf-8'),filename=name)
        parent=Path(name).parent
        for node in ast.walk(tree):
            if not isinstance(node,ast.ImportFrom) or not node.level:continue
            base=parent
            for _ in range(node.level-1):base=base.parent
            if node.module:base=base.joinpath(*node.module.split('.'))
            require(base.is_relative_to(Path(package)),'Generator relative import escapes package')
            candidates=([base] if node.module else [])+[base/alias.name for alias in node.names if alias.name!='*']
            for candidate in candidates:
                for suffix in (candidate.with_suffix('.py'),candidate/'__init__.py'):
                    relative=add(suffix,False)
                    if relative and relative not in seen:python.append(relative)
    return names


def freeze_sources(coverage, root=ROOT):
    root=checked_path(root)
    cells = derive_cells(coverage, root)
    names = {r['path'] for cell in cells for r in cell['sources']}
    names.update(scene_input_paths(coverage,root))
    # Companion-header closure cannot discover separate World translation
    # units. Bind the actual maintained static-session target's named inputs.
    runtime_cmake='src/simulation/runtime/CMakeLists.txt'
    names.add(runtime_cmake)
    runtime_text=(root/runtime_cmake).read_text(encoding='utf-8')
    target=re.findall(r'add_library\(fsim_session\s+STATIC\s+(.*?)\)',runtime_text,re.S)
    require(len(target)==1,'Exact native session source declaration required')
    units=re.findall(r'(?m)^\s*([A-Za-z0-9_/]+\.(?:cpp|h))\s*\)?\s*$',target[0])
    require(0<len(units)<=256,'Bounded native session source declaration required')
    names.update('src/simulation/runtime/'+name for name in units)
    names |= {'tests/simulation/test_aam_full_chain_native.cpp', 'tests/CMakeLists.txt',
              'tests/simulation/ComponentReferenceRuntimeV1.h',
              'src/foundation/transport/third_party/picosha2/picosha2.h',
              'src/foundation/transport/third_party/picosha2/LICENSE',
              'src/foundation/transport/third_party/picosha2/provenance.json',
              'tools/validation/weapon_quality/component_reference_run.py',
              'tools/validation/weapon_quality/acceptance.py',
              'tools/validation/weapon_quality/campaign.py',
              'tools/validation/weapon_quality/report.py',
              'tools/validation/aerodynamics/sim_m02_closure.py',
              'tools/validation/aerodynamics/closure_evidence.py',
              'tools/validation/weapon_quality/full_chain.py',
              'tools/validation/weapon_quality/component_matrix.py',
              'src/simulation/runtime/world/World.cpp',
              'src/simulation/runtime/world/WorldEngagements.cpp',
              'src/simulation/runtime/world/WorldDamage.cpp',
              'src/simulation/runtime/world/World.h',
              'src/simulation/runtime/world/WorldWeaponControls.cpp',
              'src/contracts/world_stream/WeaponDiagnosticsV1.h',
              'tools/validation/weapon_quality/test_full_chain.py',
              'src/simulation/runtime/world/WorldWeaponDiagnostics.cpp',
              'src/simulation/runtime/world/WorldWeaponSensorDiagnostics.cpp',
              'src/simulation/runtime/world/WorldWeaponEffectsDiagnostics.cpp'}
    # Follow only quoted dependencies of the explicitly selected native
    # inputs, including companion implementations. No directory enumeration.
    pending=[root/name for name in names if Path(name).suffix in ('.cpp','.h')]
    visited=set()
    while pending:
        path=checked_path(pending.pop(),root)
        if path in visited:continue
        visited.add(path)
        require(len(visited)<=2048,'Bounded native include closure exceeded')
        for name in re.findall(r'^\s*#include\s+"([^"]+)"',path.read_text(encoding='utf-8'),re.M):
            include=allowed(name);require(not include.is_absolute(),'Absolute native include')
            for candidate in (path.parent/include,root/'src'/include,root/'include'/include,root/'third_party/jsbsim/src'/include):
                allowed(candidate)
                if not candidate.is_file():continue
                candidate=checked_path(candidate,root)
                require(candidate.is_relative_to(root),'Native include escapes current source root')
                relative=candidate.relative_to(root).as_posix()
                if relative not in names:names.add(relative);pending.append(candidate)
                implementation=candidate.with_suffix('.cpp')
                if implementation.is_file():
                    relative=implementation.relative_to(root).as_posix()
                    if relative not in names:names.add(relative);pending.append(implementation)
                break
    return dict(schema='fsim.sim-m03.full-chain-source-epoch.v1',
                sources=[binding(root/name, root) for name in sorted(names)])


def prepare_preset(epoch_path,preset,root=ROOT):
    """Before execution: stage identities are expectations, not loaded code."""
    require(preset in PRESETS,'Unknown actual native preset')
    root=checked_path(root);epoch_path=checked_path(epoch_path,root)
    executable=root/'build'/preset/'tests/fsim_aam_full_chain_native_test.exe'
    prepared=staged_runtime(executable,'kinematics')
    require(len(prepared)==3,'All three staged full-chain products required')
    return dict(source_epoch=binding(epoch_path,root),
        directory=str(root/'build'/preset/'validation/aam_full_chain'),
        runtime=[binding(checked_path(item['path']),root) for item in prepared])


def validate_capture(model,directory,run,epoch,cells,root):
    """Admit only self-bound output from the exact current native invocation."""
    directory=checked_path(directory,root)
    path=directory/'capture.json';capture=read(path)
    require(capture.get('schema')=='fsim.sim-m03.aam-chain-capture.v1'
            and capture.get('complete') is True and capture.get('admission_eligible') is True,
            'Current complete epoch-bound native capture required')
    require(capture.get('model')==model and capture.get('input_epoch_sha256')==digest(epoch),
            'Old/foreign native capture epoch/model')
    expected_model=next(c['native_model'] for c in cells if c['model']==model)
    require(capture.get('native_model')==expected_model,'Wrong actual native model binding')
    executable=root/'build'/run['preset']/'tests/fsim_aam_full_chain_native_test.exe'
    require(bound(capture['executable'],root)==executable,'Foreign actual native executable')
    # fsim_session is statically linked in this native fixture. Its staged
    # SDK DLL remains an expected product, never an invented loaded module.
    observed_modules(capture,run['runtime'],executable,require_jsbsim=True,root=root)
    files=capture.get('files')
    require(type(files) is list and len(files)==6,'Exactly six native output bindings required')
    require(all(type(item) is dict and set(item)=={'path','sha256','bytes'} for item in files),
            'Exact native output records required')
    for item in files:allowed(item['path'])
    require({item['path'] for item in files}==set(FILES),'Missing/foreign/duplicate native data file')
    raw=[];data={}
    for item in files:
        file=checked_path(directory/item['path'],root)
        require(type(item['bytes']) is int and item['bytes']>0 and file.stat().st_size==item['bytes'],
                'Native data size changed')
        verify_artifact(file,item['sha256']);raw.append(binding(file,root));data[item['path']]=rows(file)
    observation=validate_case(model,data['summary.csv'],data['trace.csv'],data['components.csv'],cells)
    summary=data['summary.csv'][0];trace=data['trace.csv']
    require(capture.get('weapon')==summary['weapon'] and capture.get('generation')==summary['generation'],
            'Native capture lifetime mismatch')
    require(capture.get('samples')==len(trace) and type(capture.get('samples')) is int,
            'Native capture sample count mismatch')
    require(capture.get('terminal')==summary['terminal'] and
            type(capture.get('terminal_time_s')) in (int,float) and
            capture['terminal_time_s']==number(summary,'terminal_time'), 'Native capture terminal mismatch')
    require(type(capture.get('launch_time_s')) in (int,float) and math.isfinite(capture['launch_time_s']) and
            abs(capture['launch_time_s']-(number(trace[0],'time')-number(trace[0],'age')))<=1e-9,
            'Native capture release epoch mismatch')
    trace_times={row['time'] for row in trace}
    fin_population={time:[] for time in trace_times}
    for name in ('fins.csv','kinematics.csv','events.csv'):
        for row in data[name]:
            require(row.get('card')==model and row.get('weapon')==summary['weapon'] and
                    row.get('generation')==summary['generation'],'Foreign auxiliary native model/lifetime')
            number(row,'time')
            if name!='events.csv':require(row['time'] in trace_times,'Auxiliary output has no actual trace epoch')
            if name=='fins.csv':fin_population[row['time']].append(row['fin_id'])
    require(all(len(fin_population[row['time']])==int(row['fin_count']) and
                len(set(fin_population[row['time']]))==len(fin_population[row['time']]) for row in trace),
            'Missing/duplicate actual fin observations')
    raw.append(binding(path,root))
    observation.update(capture=binding(path,root),executable=capture['executable'],
        runtime_modules=capture['runtime_modules'])
    return observation,raw


def admit(coverage, source_epoch, presets, root=ROOT):
    root=checked_path(root)
    require(source_epoch == freeze_sources(coverage, root), 'Current native chain source epoch changed')
    require(set(presets) == set(PRESETS), 'Both native presets required')
    cells = derive_cells(coverage, root)
    models = sorted({c['model'] for c in cells})
    proof = dict(schema=SCHEMA, passed=False, sources=source_epoch['sources'], raw=[], models=[])
    observations = {}
    counts = {}
    epoch_path=None
    for preset in PRESETS:
        run = presets[preset]
        current_epoch=bound(run['source_epoch'],root)
        require(read(current_epoch)==source_epoch,'Prepared epoch bytes do not match current sources')
        require(epoch_path is None or epoch_path==current_epoch,'Presets used different source epoch files')
        epoch_path=current_epoch
        junit, directory = checked_path(run['junit'],root), checked_path(run['directory'],root)
        require(directory==root/'build'/preset/'validation/aam_full_chain','Foreign native output directory')
        require(type(run['runtime']) is list and len(run['runtime'])==3,'Three before-execution products required')
        require(len({item['path'] for item in run['runtime']})==3,'Duplicate before-execution product')
        proof['raw'].append(binding(junit, root))
        for binary in ('tests/fsim_aam_full_chain_native_test.exe', 'tests/libfsim.dll', 'tests/libJSBSim.dll'):
            path = root/'build'/preset/binary
            item = next((r for r in run['runtime'] if r['path'] == path.relative_to(root).as_posix()), None)
            require(item is not None, 'Missing actual native binary before-execution binding')
            require(bound(item,root)==path,'Foreign staged product')
            proof['raw'].append(item)
        observations[preset] = {}
        for model in models:
            observations[preset][model],captured=validate_capture(model,directory/model,
                dict(run,preset=preset),epoch_path,cells,root)
            counts[preset, model] = executed_case(junit, model)
            proof['raw'] += captured
    proof['source_epoch']=binding(epoch_path,root)
    proof['raw'].append(binding(epoch_path,root))
    proof['runs']=presets
    for cell in cells:
        model = cell['model']
        selected = {k: cell[k] for k in ('model', 'component', 'implementation_id', 'mechanism_id', 'card_sha256', 'parameter_sha256')}
        selected.update(passed=True, native=dict(case='aam_full_chain.'+model,
            tests=sum(counts[p, model]['tests'] for p in PRESETS),
            assertions=sum(counts[p, model]['assertions'] for p in PRESETS),
            failures=0, errors=0, skipped=0, per_model_execution=True,
            native_model=cell['native_model'], coupled_components=list(COMPONENTS), presets=list(PRESETS),
            observations={p: observations[p][model] for p in PRESETS}))
        proof['models'].append(selected)
    proof['passed'] = True
    return proof


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('coverage', 'source-epoch', 'runs', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    value = admit(read(a.coverage), read(a.source_epoch), read(a.runs))
    allowed(a.output).write_bytes(json.dumps(value, indent=2, allow_nan=False).encode('utf-8')+b'\n')


if __name__ == '__main__':
    main()
