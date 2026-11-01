"""One strict table grammar for generation and independent XML inventory.

Follows bundled JSBSim FGTable.cpp:InferLeafDimension, including internal
tables with no independentVar. No design name or tolerance is involved.
"""
import math

VERSION='fsim.sim-m02.native-table-inventory.v1'


def axis(values):
    if not values or not all(math.isfinite(v) for v in values):raise ValueError('Empty/nonfinite table axis')
    if any(b<=a for a,b in zip(values,values[1:])):raise ValueError('Duplicate or unordered table axis')
    return dict(points=len(values),minimum=values[0],maximum=values[-1],extrapolation='clamped end knots')


def data(node):
    children=node.findall('tableData')
    if children:
        points=[float(n.attrib['breakPoint']) for n in children]
        parsed=[data(n) for n in children]
        if len({p['dimension'] for p in parsed})!=1:raise ValueError('Inconsistent nested table dimensions')
        return dict(dimension=parsed[0]['dimension']+1,axes=[axis(points)],slices=parsed)
    rows=[[float(value) for value in line.split()] for line in (node.text or '').splitlines() if line.strip()]
    if not rows or any(not math.isfinite(v) for row in rows for v in row):raise ValueError('Empty/nonfinite table data')
    width=len(rows[0]);two=len(rows)>1 and len(rows[1])==width+1
    if two:
        if any(len(row)!=width+1 for row in rows[1:]):raise ValueError('Missing row/column in 2D table')
        return dict(dimension=2,shape=[len(rows)-1,width],axes=[axis([row[0] for row in rows[1:]]),axis(rows[0])],value_count=(len(rows)-1)*width)
    if width!=2 or any(len(row)!=2 for row in rows):raise ValueError('Missing row/column in 1D table')
    return dict(dimension=1,shape=[len(rows),1],axes=[axis([row[0] for row in rows])],value_count=len(rows))


def table(node):
    leaves=node.findall('tableData')
    if not leaves:raise ValueError('Table has no tableData')
    if len(leaves)==1 and leaves[0].get('breakPoint') is None:
        result=data(leaves[0])
    else:
        points=[float(n.attrib['breakPoint']) for n in leaves]
        slices=[data(n) for n in leaves]
        if len({p['dimension'] for p in slices})!=1:raise ValueError('Inconsistent table slice dimensions')
        result=dict(dimension=slices[0]['dimension']+1,axes=[axis(points)],slices=slices)
    independent=[dict(n.attrib,property=(n.text or '').strip()) for n in node.findall('independentVar')]
    if independent and len(independent)!=result['dimension']:raise ValueError('Declared dimension disagrees with table data')
    if node.get('type')=='internal' and independent:raise ValueError('Internal and independentVar specifications are mutually exclusive')
    return result|dict(independent_variables=independent,internal=node.get('type')=='internal' or node.tag=='thrust_table',
        interpolation='piecewise linear / multilinear according to dimension',version=VERSION)
