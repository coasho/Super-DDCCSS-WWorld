"""Limited reference comparisons, measured through SDK instances of both models.

F16A reference data and the approximate authored F16C are different variants.
The MK82 reference likewise differs in exact geometry. These measurements are
not full-envelope equipment accuracy or independent validation of fitted data.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from vlm_sdk import ROOT,FT,LBF,PSF,condition,native_library,load_native,sha


def fixture(source,folder,name):
    tree=ET.parse(source);root=tree.getroot()
    for old in root.findall('flight_control'):root.remove(old)
    control=ET.SubElement(root,'flight_control',name='Declared neutral reference fixture')
    # Forces, moments, geometry and mass are retained byte-for-byte as XML
    # expressions. Only actuator outputs are held at a declared neutral state.
    for key in ('elevator','left-aileron','right-aileron','rudder','flap','speedbrake'):
        for unit in ('rad','deg','norm'):
            ET.SubElement(control,'property',value='0').text='fcs/'+key+'-pos-'+unit
    declared={n.text for n in control.findall('property')}
    for node in root.findall('./aerodynamics//property')+root.findall('./aerodynamics//independentVar'):
        property_name=(node.text or '').strip().removeprefix('-')
        if property_name.startswith('fcs/') and property_name not in declared:
            ET.SubElement(control,'property',value='0').text=property_name
            declared.add(property_name)
    directory=folder/'aircraft'/name;directory.mkdir(parents=True,exist_ok=True)
    engines=directory/'Engines';engines.mkdir(exist_ok=True)
    for node in root.findall('./propulsion/engine')+root.findall('./propulsion/engine/thruster'):
        name_file=node.get('file')
        if not name_file:continue
        for candidate in (source.parent/'Engines'/(name_file+'.xml'),ROOT/'third_party/jsbsim/engine'/(name_file+'.xml')):
            if candidate.is_file():shutil.copyfile(candidate,engines/candidate.name);break
        else:raise ValueError('Missing declared engine dependency: '+name_file)
    for node in root.findall('system')+root.findall('autopilot'):
        file=node.get('file')
        if not file:continue
        source_file=next((p for p in (source.parent/'Systems'/(file+'.xml'),ROOT/'third_party/jsbsim/systems'/(file+'.xml')) if p.is_file()),None)
        if source_file is None:raise ValueError('Missing declared system dependency: '+file)
        shutil.copyfile(source_file,folder/'systems'/source_file.name)
    path=directory/(name+'.xml');tree.write(path,encoding='utf-8',xml_declaration=True)
    return path


def capture(model,root,conditions):
    import fsim
    world=fsim.World('reference-'+model,dt=1.e-5,frame_skip=1,publish=False,workers=1,
                     pin_workers=False,seed=3,terrain=False,jsbsim_root=str(root))
    rows=[]
    try:
        vehicle=world.create_vehicle('probe','jsbsim:'+model,latitude_deg=37.6,longitude_deg=-122.4,
                                     altitude_msl_m=10000.,airspeed_ms=150.)
        vehicle.set_protection('off')
        g=vehicle.get_property
        for alpha,beta,mach in conditions:
            condition(world,vehicle,alpha,beta,mach)
            force=np.array([g('forces/fb'+axis+'-aero-lbs') for axis in 'xyz'])*LBF
            moment=np.array([g('moments/'+axis+'-aero-lbsft') for axis in 'lmn'])*LBF*FT
            qs=g('aero/qbar-psf')*PSF*g('metrics/Sw-sqft')*FT**2
            b,c=g('metrics/bw-ft')*FT,g('metrics/cbarw-ft')*FT
            # SDK moments are about the actual CG. Compare coefficients about
            # each source model's declared aerodynamic reference point.
            tree=ET.parse(root/'aircraft'/model/(model+'.xml'))
            location=tree.find("metrics/location[@name='AERORP']")
            scale={'M':1.,'IN':.0254,'FT':FT}[location.get('unit','IN')]
            arp=np.array([float(location.findtext(axis)) for axis in 'xyz'])*scale
            cg=np.array([g('inertia/cg-'+axis+'-in') for axis in 'xyz'])*.0254
            moment-=np.cross((arp-cg)*np.array([-1.,1.,-1.]),force)
            rows.append(dict(requested=dict(alpha_deg=alpha,beta_deg=beta,mach=mach),
                actual=dict(alpha_deg=g('aero/alpha-deg'),beta_deg=g('aero/beta-deg'),mach=g('velocities/mach')),
                force_coefficient=(force/qs).tolist(),moment_coefficient=(moment/(qs*np.array([b,c,b]))).tolist(),
                drag_area_m2=-float(np.dot(force,[math.cos(g('aero/alpha-rad'))*math.cos(g('aero/beta-rad')),
                    math.sin(g('aero/beta-rad')),math.sin(g('aero/alpha-rad'))*math.cos(g('aero/beta-rad'))]))/(g('aero/qbar-psf')*PSF)))
    finally:world.close()
    return rows


def nrmse(value,reference):
    value,reference=np.asarray(value),np.asarray(reference)
    return float(100.*np.linalg.norm(value-reference)/np.linalg.norm(reference))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--native-bin',type=Path)
    args=parser.parse_args()
    load_native(args.native_bin)
    import fsim
    fsim.set_log_level('off')
    folder=args.out.parent/'reference-fixture'
    for name in ('engine','systems'):(folder/name).mkdir(parents=True,exist_ok=True)
    sources=dict(candidate=ROOT/'aircraft/f16c/f16c.xml',reference=ROOT/'third_party/jsbsim/aircraft/f16/f16.xml',mk82=ROOT/'aircraft/stores/mk82/mk82.xml')
    paths={key:fixture(source,folder,'vlm-'+key) for key,source in sources.items()}
    conditions=[(alpha,0.,.3) for alpha in (-5.,0.,5.,10.,15.,20.)]
    candidate=capture('vlm-candidate',folder,conditions)
    reference=capture('vlm-reference',folder,conditions)
    combine=lambda rows:[r['force_coefficient']+r['moment_coefficient'] for r in rows]
    mk82=capture('vlm-mk82',folder,[(0.,0.,mach) for mach in (.6,.9,1.2)])
    ref_area=.05747202904793593
    measured=[r['drag_area_m2']/ref_area for r in mk82]
    published=[.1635,.1434,.3574]
    report=dict(schema='fsim.limited-reference-sdk.v1',sdk_executed=True,native_library=native_library(args.native_bin),
        source_sha256={k:sha(p) for k,p in sources.items()},fixture_sha256={k:sha(p) for k,p in paths.items()},
        f16=dict(candidate_model='Authored F16C',reference_model='JSBSim F16A Block32, NASA-TP1538-derived',
            source_url='https://ntrs.nasa.gov/citations/19800005879',
            scope='Six neutral-control low-speed cases; variant/reference-origin differences remain. Model-reference comparison, not matched equipment accuracy.',
            nrmse_percent=nrmse(combine(candidate),combine(reference)),candidate=candidate,reference=reference),
        mk82=dict(source_url='https://apps.dtic.mil/sti/pdfs/ADA329921.pdf',reference_area_m2=ref_area,
            scope='Three clean axial drag cases from DSTO-TR0554; authored approximate geometry differs. No fitting to these values.',
            reference_cd=published,measured_sdk_cd=measured,nrmse_percent=nrmse(measured,published),rows=mk82))
    args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k]['nrmse_percent'] for k in ('f16','mk82')}))


if __name__=='__main__':main()
