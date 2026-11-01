"""Publish carried-store data from the aircraft VLM tables."""
import json
from pathlib import Path
from . import airframe,pipeline


def write(cards_dir):
    root=Path(pipeline.repo_root());rows=[]
    for path in sorted(Path(cards_dir).glob('*.json')):
        card=json.loads(path.read_text(encoding='utf-8'))
        name=card.get('model','').removeprefix('jsbsim:')
        source=root/'aircraft'/name/(name+'.toml')
        if not source.is_file():raise ValueError('Store has no authored VLM design: '+card['id'])
        from . import storestages
        d=airframe.design(source)
        storestages.build(d)
        card=json.loads(path.read_text(encoding="utf-8"))
        rows.append(dict(card=card['id'],method='vlm',drag_area_m2=card['parameters']['drag_area_m2']['value']))
    return rows


def main():
    for row in write(Path(pipeline.repo_root())/'assets/stores'):
        print(json.dumps(row))


if __name__=='__main__':main()
