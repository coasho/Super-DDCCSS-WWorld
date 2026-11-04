"""A real Agent/C2 weapon scene through the configured normal Python launcher."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from acceptance import allowed
from audit import audit
from fleet import scene

ROOT=Path(__file__).resolve().parents[3]


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    output=allowed(a.output).resolve()
    if output.exists():raise ValueError('Fresh normal-entry receipt required')
    import fsim
    package=Path(fsim.__file__).resolve().parent
    if package!=ROOT/'build/ucrt64-release/python/fsim':raise ValueError('Normal release launcher selected another Python package')
    sys.path.insert(0,str(ROOT/'tools/validation/radar'))
    from provenance import loaded_modules
    coverage=audit()
    if not coverage['inventory_passed']:raise ValueError('Incomplete current source applicability')
    row=next(r for r in coverage['stores'] if r['card']=='aim-120d')
    card=json.loads((ROOT/row['card_path']).read_text(encoding='utf-8'))
    scene.prefix=package;scene.artifacts=output.parent/'normal-entry-observations'
    value=dict(schema='fsim.sim-m03.normal-entry.v1',passed=False,python_package=str(package),
        native_python=str(Path(fsim._native.__file__).resolve()))
    def save():
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    try:
        value['case']=scene(fsim,row,card,dict(carrier='f16c',station=1,declared_loadout=False,
            fit_scope='Exact AFMC-documented F-16 AIM-120D integration'),1,duration=12,countermeasure=False)
        modules=loaded_modules(ROOT/'build/ucrt64-release')
        for module in modules:
            expected=hashlib.sha256((ROOT/'build/ucrt64-release/bin'/module['name']).read_bytes()).hexdigest()
            if module['sha256']!=expected:raise ValueError('Normal entry loaded an old native binary')
        native=Path(fsim._native.__file__).resolve()
        if not native.is_relative_to(package):raise ValueError('Normal Python extension outside configured package')
        value.update(modules=modules,native_python_sha256=hashlib.sha256(native.read_bytes()).hexdigest(),passed=True)
        save();print(json.dumps(value))
    except Exception as error:value['error']=str(error);save();raise
