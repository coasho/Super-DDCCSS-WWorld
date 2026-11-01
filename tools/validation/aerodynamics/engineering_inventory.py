"""Strict generic XML table inventory, including JSBSim internal lookup tables.

Grammar authority: the bundled third_party/jsbsim/src/math/FGTable.cpp,
InferLeafDimension. Missing independentVar does not mean one-dimensional.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))
from hangar.engineering_tables import VERSION, table


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect(path):
    path=Path(path);doc=ET.parse(path);checksum=sha(path);result=[]
    for index,node in enumerate(doc.findall('.//table')+doc.findall('.//thrust_table')):
        record=dict(index=index,name=node.get('name'),file=str(path),file_sha256=checksum,
            method='current shared Hangar generated XML; data grammar inferred by bundled JSBSim FGTable rules',
            basis='estimated',version=VERSION,source='third_party/jsbsim/src/math/FGTable.cpp:InferLeafDimension; original per-table derivations remain in engineering-aero-v1.json')
        try:record.update(table(node));record['passed']=True
        except (ValueError,KeyError) as error:record.update(passed=False,error=str(error))
        result.append(record)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--matrix',required=True);p.add_argument('--aircraft',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();started=time.monotonic();matrix=json.loads((Path(args.matrix)/'matrix.json').read_text(encoding='utf-8'));rows=[]
    for design in matrix['designs']:
        tables=[]
        for relative,expected in design.get('products',{}).items():
            if Path(relative).suffix!='.xml':continue
            path=Path(args.aircraft)/relative
            if sha(path)!=expected:raise ValueError('Native table inventory refused stale product: '+str(path))
            tables.extend(inspect(path))
        rows.append(dict(name=design['name'],table_count=len(tables),passed=bool(tables) and all(t['passed'] for t in tables),tables=tables))
    elapsed=time.monotonic()-started
    result=dict(schema=VERSION,design_count=len(rows),table_count=sum(r['table_count'] for r in rows),source_sha256=sha(__file__),elapsed_seconds=elapsed,
        deadline_seconds=1200,timeout=elapsed>1200,designs=rows)
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return int(elapsed>1200 or not all(r['passed'] for r in rows))


if __name__=='__main__':raise SystemExit(main())
