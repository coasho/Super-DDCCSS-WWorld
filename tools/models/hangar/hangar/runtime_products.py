"""Publish generated aircraft and cards into already-built project SDKs."""
import json
from pathlib import Path
import re
import shutil
import tomllib

from . import pipeline
from .files import atomic_write_text


def stage_runtime_products(source,data):
    """Keep XML, cards and content-addressed carriage tables in one epoch."""
    from . import storestages
    root=Path(pipeline.repo_root()).resolve();source=Path(source).resolve();data=Path(data).resolve()
    if not data.is_relative_to(root) or not data.is_dir():
        raise ValueError('Publication requires the built project SDK data directory')
    store=storestages.is_store(source)
    relative=Path('stores')/source.stem if store else Path(source.stem)
    products=root/'aircraft'/relative;destination=data/'aircraft'/relative
    files=list(products.glob('*.xml'))
    files.extend(products.glob('*.damage.json'))
    if (products/'pieces').is_dir():
        for piece in (products/'pieces').iterdir():
            if piece.name.casefold()=='.codex' or piece.is_symlink() or not piece.is_dir():continue
            files.extend(p for p in piece.iterdir() if p.name.casefold()!='.codex' and not p.is_symlink() and p.is_file() and p.suffix=='.xml')
    for folder in ('Engines','Systems','aero'):
        if (products/folder).is_dir():
            files.extend(p for p in (products/folder).iterdir() if p.is_file() and p.suffix in ('.xml','.json'))
    for path in files:
        target=destination/path.relative_to(products);target.parent.mkdir(parents=True,exist_ok=True)
        if not target.is_file() or path.read_bytes()!=target.read_bytes():shutil.copyfile(path,target)
    legacy=destination/'engineering-aero-v1.json'
    if legacy.is_file() and not (products/legacy.name).is_file():
        old=json.loads(legacy.read_text(encoding='utf-8'))
        if old.get('schema')=='fsim.sim-m02.engineering.v1' and old.get('name')==source.stem:legacy.unlink()
    if store:
        card_id=tomllib.loads(source.read_text(encoding='utf-8'))['aircraft']['card']
        card=json.loads((root/'assets/stores'/(card_id+'.json')).read_text(encoding='utf-8'))
        atomic_write_text(data/'stores'/(card['id']+'.json'),json.dumps(card,indent=2,ensure_ascii=False)+'\n')
        for old in (destination/'aero').glob('vlm-carriage-*.json'):
            if re.fullmatch(r'vlm-carriage-[0-9a-f]{64}\.json',old.name) and not (products/'aero'/old.name).is_file():old.unlink()
    return dict(data_directory=str(data),generated_files=len(files))


def stage_built_project_sdks(source):
    """A model-only build remains possible before any SDK has been built."""
    root=Path(pipeline.repo_root());receipts=[]
    for preset in ('ucrt64-headless','ucrt64-release'):
        for relative in ('share/flightsim','python/fsim/share/flightsim'):
            data=root/'build'/preset/relative
            if data.is_dir():receipts.append(stage_runtime_products(source,data))
    return receipts
