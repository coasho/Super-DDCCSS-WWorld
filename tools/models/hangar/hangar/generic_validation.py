"""SDK comparison with a public generic geometry, never an equipment fit."""
import argparse,hashlib,json,math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from . import __version__,pipeline,jsbsim
from .aero import tables
from .geometry import Aircraft
from .reference_campaign import errors
from .sdk_measurements import Measurements,native_library

SOURCE='https://www.mas.bg.ac.rs/_media/istrazivanje/fme/vol48/4/5_d_damljanovic_et_al.pdf'
# (Mach, alpha_deg, CLf, CDf, Cm); Tables 2--10, printed pp.766--768.
# Only these 9 disjoint reference conditions are used; no parameters are fitted.
REFERENCE=[(.77,2.,.097,.0152,.0177),(.77,6.,.308,.0401,.0545),(.77,10.,.530,.0978,.0912),
    (1.,2.,.108,.0242,.0107),(1.,6.,.339,.0524,.0316),(1.,10.,.581,.1152,.0537),
    (1.17,2.,.106,.0328,.0079),(1.17,6.,.321,.0609,.0254),(1.17,10.,.533,.1190,.0474)]

def normal_force(lift,drag,alpha_deg):
    a=math.radians(alpha_deg)
    return lift*math.cos(a)+drag*math.sin(a)

def geometry():
    # Figure 1, p.762: D=115.8 mm; cubic nose, 60-degree delta,
    # 4% symmetric biconvex section. No support sting is simulated.
    d=.1158;chord=2*math.sqrt(3)*d;mac=4*math.sqrt(3)*d/3
    root_le=4.5*d-chord/3;reference=4.5*d+.5*mac
    stations=[]
    for k in range(25):
        x=3*d*k/24;y=x/3*(1-(x/d)**2/9+(x/d)**3/54)
        stations.append(dict(x=x,w=2*y,top=y,bottom=-y))
    stations.append(dict(x=8.5*d,w=d,top=d/2,bottom=-d/2))
    return dict(aircraft=dict(name='generic-agard-b',category='research'),
        analysis=dict(speed=260.,max_mach=1.2),
        body=[dict(name='standard body',kind='fuselage',stations=stations)],
        surface=[dict(name='delta',kind='wing',mirror=True,airfoil='biconvex4',sections=[
            dict(le=[root_le,0.,0.],chord=chord),dict(le=[root_le+chord,2*d,0.],chord=1.e-7)])],
        reference=dict(area=4*math.sqrt(3)*d*d,span=4*d,chord=mac,aero_point=[reference,0.,0.]))

def run(scratch):
    import fsim
    fsim.set_log_level('off');scratch=Path(scratch).resolve();scratch.mkdir(parents=True,exist_ok=True)
    spec=geometry();(scratch/'geometry.json').write_text(json.dumps(spec,indent=2)+'\n')
    a=Aircraft(spec,str(scratch/'generic-agard-b.toml'))
    from .aero.model import AeroModel
    table=tables.build(AeroModel(a),alpha=[-2.,0.,2.,6.,10.,12.],beta=[-3.,0.,3.])
    # The same shared writer and SDK measurement path as equipment campaigns.
    # The neutral fixture retains a real aircraft container, replacing only
    # aerodynamic geometry/reference expressions; no client force integration.
    original=Path(pipeline.repo_root())/'aircraft/c172/c172.xml'
    xml=ET.parse(original);root=xml.getroot();root.remove(root.find('aerodynamics'))
    root.append(ET.fromstring(jsbsim.aerodynamics_xml(table,a)))
    metrics=root.find('metrics')
    for tag,value in [('wingarea',a.S/.3048**2),('wingspan',a.b/.3048),('chord',a.c/.3048)]:metrics.find(tag).text=str(value)
    arp=metrics.find("location[@name='AERORP']");arp.set('unit','M')
    for key,value in zip('xyz',a.aero_point):arp.find(key).text=str(value)
    m=Measurements.__new__(Measurements)
    m.name='generic-agard-b';m.root=Path(pipeline.repo_root());m.scratch=scratch
    m.source=scratch/'generic-agard-b.toml';m.kind='research';m.product=original;m.initial={};m.last=[]
    m.original=ET.tostring(root,encoding='unicode')
    m.design=SimpleNamespace(aircraft=a,tables=lambda:table,calibrated=lambda value:value)
    cases=[]
    for mach,alpha,cl,cd,cm in REFERENCE:
        for metric,value in [('CZ',-normal_force(cl,cd,alpha)),('Cm',cm)]:
            cases.append(dict(metric=metric,value=value,conditions=dict(alpha_deg=alpha,beta_deg=0.,mach=mach,altitude_m=1000.)))
    predicted=m.measure({},cases)
    rows=[dict(case,sdk_value=value,signed_error=value-case['value'],actual=m.last[i]) for i,(case,value) in enumerate(zip(cases,predicted))]
    result=dict(schema='fsim.generic-vlm-validation.v1',basis='simulated',tool='Hangar shared aircraft VLM',version=__version__,
        source_url=SOURCE,source_locator='Figure 1 p.762; Tables 2--10 pp.766--768',geometry=spec,
        native_library=native_library(),fixture_xml_sha256=m.fixture_sha256,observations=rows,
        metrics={metric:errors([r['sdk_value'] for r in rows if r['metric']==metric],
                              [r['value'] for r in rows if r['metric']==metric]) for metric in ('CZ','Cm')},
        assessed_domain=dict(mach=[.77,1.,1.17],alpha_deg=[2.,6.,10.],beta_deg=[0.]),
        equipment_accuracy_claim=False,parameters_fitted=False,
        limitations=['Generic AGARD-B geometry, not any of the 88 authored equipment models.',
            'The publication pools facilities/model sizes and includes M0.8/M1.2 runs with the M0.77/M1.17 groups; no common Reynolds condition is supplied.',
            'Body-normal force is derived as CN=CLf*cos(alpha)+CDf*sin(alpha), cancelling axial base-pressure force; reported CZ=-CN.',
            'The 4% biconvex wing uses the existing shared section; its tiny specified leading-edge radius is not separately resolved.',
            'Density/Reynolds and sting/wall interference differences remain. Residuals are not an equipment confidence interval.',
            'No beta, dynamic derivative, powered flow or extreme-angle qualification follows from these conditions.'])
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=run(args.out.parent/'generic-runtime')
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result['metrics']))
    return 0

if __name__=='__main__':raise SystemExit(main())
