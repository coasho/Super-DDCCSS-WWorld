"""Independent matrix-exponential oracle for the sampled resolved INS model.

SciPy's matrix exponential is independent of the native polynomial transition.
This is a mathematical reference, never an equipment accuracy measurement.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import scipy
from scipy.linalg import expm
from acceptance import allowed


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    output=allowed(a.output)
    if output.exists():raise ValueError('Fresh independent reference required')
    system=np.array([[0.,1.,0.],[0.,0.,-1.],[0.,0.,0.]])
    intensity=np.diag([0.,.03**2,.0001**2]);zero=np.zeros((3,3));dt=2.3
    block=expm(np.block([[system,intensity],[zero,-system.T]])*dt)
    transition=block[:3,:3];q=block[:3,3:]@transition.T
    posterior=transition@np.diag([25.,.25,.01**2])@transition.T+q
    sampled_system=np.zeros((4,4));sampled_system[0,1]=1
    sampled_system[1,2:]=-1
    period=.01;step=expm(sampled_system*period);reset=np.diag([1.,1.,1.,0.])
    refresh=np.diag([0.,0.,.0001**2*period,.03**2/period])
    sampled=np.diag([25.,.25,.01**2,.03**2/period])
    for _ in range(230):sampled=reset@step@sampled@step.T@reset.T+refresh
    convergence=[]
    for exposure in (.02,.01,.005):
        f=expm(sampled_system*exposure)
        value=np.diag([25.,.25,.01**2,.03**2/exposure])
        renewal=np.diag([0.,0.,.0001**2*exposure,.03**2/exposure])
        for _ in range(round(dt/exposure)):value=reset@f@value@f.T@reset.T+renewal
        convergence.append(dict(imu_period_s=exposure,position_velocity_covariance=value[:2,:2].tolist(),
            continuum_error_norm=float(np.linalg.norm(value[:2,:2]-posterior[:2,:2]))))
    if not all(a['continuum_error_norm']>b['continuum_error_norm'] for a,b in zip(convergence,convergence[1:])):
        raise ValueError('Sampled covariance did not converge to independent continuum limit')
    result=dict(schema='fsim.sim-m03.navigation-reference.v1',scipy_version=scipy.__version__,dt_s=dt,
        system=system.tolist(),noise_intensity=intensity.tolist(),transition=transition.tolist(),
        process_covariance=q.tolist(),propagated_covariance=posterior.tolist(),
        sampled_system=sampled_system.tolist(),imu_period_s=period,
        sampled_transition=step.tolist(),sampled_refresh=refresh.tolist(),
        sampled_covariance=sampled.tolist(),
        continuum_convergence=convergence,
        method='Independent expm of augmented ZOH system plus exact discrete exposure refresh; Van Loan gives the separate continuum limit',
        equipment_accuracy=False)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result))
