"""Generated, versioned numerical resolution from geometry, inertia and tables.

No equipment-name branch and no force/measurement fit. Default (absent metadata)
is one native step. A fixed step per model also preserves JSBSim's multistep
integrator and FCS coefficient assumptions; changing dt after load is invalid.
"""
import math
import itertools
from pathlib import Path
import re
import tomllib
import xml.etree.ElementTree as ET

import numpy as np

VERSION = 'fsim.integration.fixed-substeps.v1'
VERSION2 = 'fsim.integration.fixed-substeps.v2'


def _location(node):
    if node is None:raise ValueError('Missing required numerical reference/CG location')
    scale={'M':1.,'IN':.0254,'FT':.3048,'CM':.01}.get(node.get('unit','IN').upper())
    if scale is None:raise ValueError('Unsupported numerical reference location unit')
    result=np.array([float(node.findtext(k,'0')) for k in 'xyz'])*scale
    if not np.isfinite(result).all():raise ValueError('Nonfinite numerical reference location')
    return result


def inertia_matrix(root):
    """Match JSBSim FGMassBalance::ReadInertiaMatrix, including frame signs."""
    balance=root.find('mass_balance')
    def number(key):
        node=balance.find(key)
        value=float(node.text) if node is not None else 0.
        return value*(1.3558179483314004 if node is not None and node.get('unit','').upper() in ('SLUG*FT2','SLUG*FT^2') else 1.)
    ixx,iyy,izz=(number(k) for k in ('ixx','iyy','izz'))
    ixy,ixz,iyz=(number(k) for k in ('ixy','ixz','iyz'))
    sign=1. if balance.get('negated_crossproduct_inertia')=='false' else -1.
    return np.array([[ixx,sign*ixy,-sign*ixz],[sign*ixy,iyy,sign*iyz],[-sign*ixz,sign*iyz,izz]])


def _transferred_modes(root,aircraft,tables,inertia,inverse,vmax):
    """Grid-based modal estimate including the native reference/CG transfer.

    Positive source masses keep CG within the hull of the dry CG, tank and
    point-mass locations. This is an estimated neutral-control resolution,
    not a global nonlinear stability or hardware-envelope certificate.
    """
    arp=_location(root.find("metrics/location[@name='AERORP']"))
    cg=[_location(root.find("mass_balance/location[@name='CG']"))]
    cg.extend(_location(tank.find('location')) for tank in root.findall('propulsion/tank'))
    cg.extend(_location(tank.find('drain_location')) for tank in root.findall('propulsion/tank') if tank.find('drain_location') is not None)
    cg.extend(_location(point.find('location')) for point in root.findall('mass_balance/pointmass')
              if float(point.findtext('weight','0'))>0)
    arms=[(arp-point)*np.array([-1.,1.,-1.]) for point in cg]
    alpha=np.radians(tables['alpha']);beta=np.radians(tables['beta'])
    ca,sa=np.cos(alpha)[:,None],np.sin(alpha)[:,None]
    cb,sb=np.cos(beta)[None,:],np.sin(beta)[None,:]
    na,nb=len(alpha),len(beta);base=tables['base'];length=np.array([aircraft.b,aircraft.c,aircraft.b]);S=aircraft.S
    ground=tables['ground_effect']
    corners=itertools.product((min(ground['lift']),max(ground['lift'])),(min(ground['drag']),max(ground['drag'])))
    ground_corners=list(set(corners));damping=stiffness=0.
    def forces(cd,cy,cl):
        return S*np.stack(np.broadcast_arrays(-cd*ca*cb-cy*ca*sb+cl*sa,-cd*sb+cy*cb,-cd*sa*cb-cy*sa*sb-cl*ca))
    for index in range(len(tables['mach']['mach'])):
        mach=tables['mach'];kl=float(mach['K_L'][index]);ky=float(mach['K_Y'][index]);ki=float(mach['induced_factor'][index]);pitch=float(mach['dCm_dCL'][index])
        for gl,gd in ground_corners:
            cd=np.asarray(base['CD'])+float(mach['dCD0'][index])+np.asarray(base['CDi'])*(ki*gd-1.)
            force=forces(cd,np.asarray(base['CY'])*ky,np.asarray(base['CL'])*kl*gl)
            moment=S*length[:,None,None]*np.stack((np.asarray(base['Cl'])*ky,np.asarray(base['Cm'])+np.asarray(base['CL'])*pitch,np.asarray(base['Cn'])*ky))
            rate_force=[];rate_moment=[]
            for k,rate in enumerate(('p','q','r')):
                row=tables['rates'][rate];factor=length[k]/2*kl
                rate_force.append(forces((np.asarray(row['CD'])+np.asarray(row['CDi'])*(ki*gd-1.))[:,None],np.asarray(row['CY'])[:,None],np.asarray(row['CL'])[:,None])*factor)
                rate_moment.append(np.broadcast_to(S*length[:,None,None]*np.stack([np.asarray(row[key]) for key in ('Cl','Cm','Cn')])[:,:,None]*factor,(3,na,nb)))
            for arm in arms:
                transferred=moment+np.cross(arm,force,axisa=0,axisb=0,axisc=0)
                acceleration=np.einsum('ij,jab->iab',inverse,transferred)
                slopes=[]
                for axis,coordinates in ((1,alpha),(2,beta)):
                    delta=np.diff(acceleration,axis=axis)/np.diff(coordinates).reshape((1,-1,1) if axis==1 else (1,1,-1))
                    slopes.append(float(np.max(np.linalg.norm(delta,axis=0))))
                stiffness=max(stiffness,math.hypot(*slopes))
                matrix=np.stack([m+np.cross(arm,f,axisa=0,axisb=0,axisc=0) for m,f in zip(rate_moment,rate_force)],axis=-1)
                angular=np.einsum('ij,jabk->iabk',inverse,matrix)
                damping=max(damping,float(np.max(np.sum(np.abs(angular),axis=-1))))
    # Include torque-free Euler coupling at the same explicitly declared
    # reduced-rate reference interval as the generated derivative tables.
    omega=.06*vmax/length;gyro=0.;eye=np.eye(3)
    for signs in itertools.product((-1.,1.),repeat=3):
        w=omega*np.array(signs)
        jacobian=-inverse@np.column_stack([np.cross(e,inertia@w)+np.cross(w,inertia@e) for e in eye])
        gyro=max(gyro,float(np.linalg.norm(jacobian,ord=np.inf)))
    return damping,stiffness,dict(cg_hull_structural_m=[p.tolist() for p in cg],aero_reference_structural_m=arp.tolist(),
        gyro_rate_bound_per_s=gyro,reduced_rate_reference_interval=[-.03,.03],
        scope='Neutral-control table grid, source mass-CG hull including drain endpoints and dry-inertia torque-free Jacobian; estimated fixed resolution, not nonlinear global stability or a bound on arbitrary loaded anisotropic inertia')


def source_options(path):
    """Preserve an author's versioned numerical options in a design rewrite.

    Geometry/card writers own their physical template. The optional TOML
    integration section is an independently authored numerical setting; its
    absence remains identity for every design.
    """
    path=Path(path)
    if not path.is_file():return ''
    text=path.read_text(encoding='utf-8')
    config=tomllib.loads(text).get('integration')
    if config is None:return ''
    if not isinstance(config,dict) or type(config.get('version')) is not int or config.get('version') not in (1,2) or set(config)-({'version','mode','refinement'} if config.get('version')==2 else {'version','mode'}) or config.get('mode','identity') not in ('identity','derived'):
        raise ValueError('Integration settings require supported version 1/2 and mode')
    if type(config.get('refinement',1)) is not int or not 1<=config.get('refinement',1)<=8:
        raise ValueError('Integration refinement must be a bounded integer1..8')
    lines=text.splitlines(keepends=True)
    begin=next(i for i,line in enumerate(lines) if re.fullmatch(r'\s*\[integration\]\s*(?:#.*)?\n?',line))
    end=next((i for i in range(begin+1,len(lines)) if re.match(r'\s*\[',lines[i])),len(lines))
    while begin and (not lines[begin-1].strip() or lines[begin-1].lstrip().startswith('#')):begin-=1
    return '\n'+''.join(lines[begin:end]).strip('\n')+'\n'


def settings(root, aircraft, tables):
    inertia=inertia_matrix(root)
    if not np.isfinite(inertia).all() or min(np.linalg.eigvalsh(inertia))<=0:
        raise ValueError('Missing positive definite required dry inertia tensor')
    inverse=np.linalg.inv(inertia)
    length=np.array([aircraft.b,aircraft.c,aircraft.b])
    S=aircraft.S
    # The declared Mach domain at ISA sea level bounds the common numerical
    # resolution. It is not a calibrated operating envelope.
    maximum_mach=float(tables['mach']['mach'][-1])
    vmax=max(340.294*maximum_mach,1.)
    qmax=.5*1.225*vmax*vmax
    kl=max(abs(float(x)) for x in tables['mach']['K_L'])
    damping=0.
    for index in range(len(tables['alpha'])):
        matrix=np.array([[tables['rates'][rate][k][index] for rate in ('p','q','r')] for k in ('Cl','Cm','Cn')])
        matrix=np.diag(length)@matrix@np.diag(length)*S/2
        damping=max(damping,float(np.linalg.norm(inverse@matrix,ord=np.inf))*kl)
    # Angle stiffness from the full alpha/beta moment grid; row-sum norm is
    # a conservative bound on the linearized modal frequency squared.
    moments=np.stack([np.asarray(tables['base'][k])*length[i]*S for i,k in enumerate(('Cl','Cm','Cn'))])
    gradients=[np.gradient(moments,np.radians(tables['alpha']),axis=1),
               np.gradient(moments,np.radians(tables['beta']),axis=2)]
    stiffness=0.
    for gradient in gradients:
        acceleration=np.einsum('ij,jab->iab',inverse,gradient)
        stiffness=max(stiffness,float(np.max(np.sum(np.abs(acceleration),axis=0)))*kl)
    # Fourfold margin on decay and 0.25 rad modal increment for the existing
    # explicit multistep propagation; a numerical safety factor, never an
    # aerodynamic scale. 120 Hz is the existing SDK base clock.
    rate=max(damping*qmax/vmax,math.sqrt(stiffness*qmax))
    recommended=max(1,int(math.ceil(rate/(120.*.25))))
    config=aircraft.spec.get('integration',{})
    if not isinstance(config,dict) or config and (type(config.get('version')) is not int or config.get('version') not in (1,2) or set(config)-({'version','mode','refinement'} if config.get('version')==2 else {'version','mode'})):
        raise ValueError('Integration settings require supported version 1/2 and the declared mode only')
    mode=config.get('mode','identity')
    if mode not in ('identity','derived'):raise ValueError('Unsupported integration resolution mode')
    # A numerical option must not silently change existing first-tick sampling
    # semantics. All designs use the same explicit, versioned option; default
    # identity preserves existing native products and public-step behavior.
    version2=config.get('version')==2 and mode=='derived';transfer={}
    refinement=config.get('refinement',1)
    if type(refinement) is not int or not 1<=refinement<=8:raise ValueError('Integration refinement must be a bounded integer1..8')
    if version2:
        damping,stiffness,transfer=_transferred_modes(root,aircraft,tables,inertia,inverse,vmax)
        rate=damping*qmax/vmax+math.sqrt(stiffness*qmax)+transfer['gyro_rate_bound_per_s']
        recommended=max(1,int(math.ceil(rate/(120.*.25))))
    substeps=recommended*refinement if mode=='derived' else 1
    if substeps>4096:
        raise ValueError('Missing bounded integration resolution: required substeps exceeds 4096')
    return dict(version=VERSION2 if version2 else VERSION,basis='estimated',substeps=substeps,mode=mode,recommended_substeps=recommended,
        rotational_rate_method=4 if version2 else 1,reference_transfer=transfer,refinement=refinement,
        dry_inertia_body_kg_m2=inertia.tolist(),
        reference_outer_dt_s=1/120,maximum_internal_dt_s=(1/120)/substeps,
        derivation=('Native AERORP-to-source-CG-hull transfer of six-axis loads and rate derivatives; sum of aerodynamic-rate, angle-frequency and dry-inertia gyro estimates at reduced-rate reference ±0.03; AB3 rotational rates; ' if version2 else '')+'row-sum norm of I^-1 dM/domega, and sqrt(norm(I^-1 dM/dangle)), from generated dry inertia and aerodynamic tables; 0.25 reference modal increment at ISA sea-level maximum declared Mach; convergence/actual flight required, not a global stability certificate',
        damping_bound=damping,stiffness_bound=stiffness,declared_qbar_pa=qmax,declared_speed_ms=vmax,
        extrapolation='Fixed resolution for declared numerical domain; higher density/Mach/rates require a new declared envelope and regeneration; no dynamic dt switches',
        source='integration.py:settings; JSBSim FGPropagate.h integrator enum/property and FGPropagate.cpp AB3 recurrence; FGFDMExec.cpp RunIC derivative-history initialization')


def install(xml, aircraft, tables):
    root=ET.fromstring(xml)
    values=settings(root,aircraft,tables)
    fcs=root.find('flight_control')
    if fcs is None:fcs=ET.SubElement(root,'flight_control',name='Numerical integration metadata')
    version2=values['version']==VERSION2
    fields=[('version',2 if version2 else 1),('substeps',values['substeps'])]
    if version2:fields.append(('rotational-rate-method',4))
    for key,value in fields:
        name='fsim/integration-v1/'+key
        node=next((n for n in fcs.findall('property') if (n.text or '').strip()==name),None)
        if node is None:node=ET.SubElement(fcs,'property')
        node.set('value',str(value));node.text=name
    ET.indent(root,space='    ')
    root.insert(0,ET.Comment(' Written by hangar; shared geometry/card aerodynamic pipeline; integration metadata v%d. '%(2 if version2 else 1)))
    return '<?xml version="1.0"?>\n'+ET.tostring(root,encoding='unicode')+'\n'
