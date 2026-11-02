"""Generate a new independent IMM/UKF fixture with pinned FilterPy 1.4.5.

No existing Stone Soup/DREV fixture is edited. This is numerical-method
agreement, not actual radar measurement calibration.
"""
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys


def generate(reference_path):
    if any(p.casefold()=='.codex' for p in reference_path.parts):raise ValueError('Excluded reference directory')
    sys.path.insert(0,str(reference_path))
    import numpy as np
    import filterpy
    from filterpy.kalman import IMMEstimator, UnscentedKalmanFilter, MerweScaledSigmaPoints
    from scipy.linalg import cholesky
    from scipy.stats import Covariance, multivariate_normal
    if filterpy.__version__!='1.4.5':raise ValueError('Reference version must remain pinned to1.4.5')
    def h(x):
        p=x[[0,2,4]];v=x[[1,3,5]];r=np.linalg.norm(p)
        return np.array([math.atan2(p[2],math.hypot(p[0],p[1])),math.atan2(p[1],p[0]),r,np.dot(p,v)/r])
    def f(x,dt):
        y=x.copy();y[[0,2,4]]+=dt*x[[1,3,5]];return y
    def residual_z(a,b):
        d=a-b
        d[:2]=(d[:2]+np.pi)%(2*np.pi)-np.pi
        return d
    def mean_z(sigmas,weights):
        result=np.dot(weights,sigmas)/sum(weights)
        for axis in (0,1):
            result[axis]=math.atan2(np.dot(weights,np.sin(sigmas[:,axis]))/sum(weights),
                                    np.dot(weights,np.cos(sigmas[:,axis]))/sum(weights))
        return result
    def process(q,dt):
        matrix=np.zeros((6,6))
        for i in (0,2,4):matrix[i:i+2,i:i+2]=q*np.array([[dt**3/3,dt**2/2],[dt**2/2,dt]])
        return matrix
    initial=np.array([10000.,50.,2000.,-20.,3000.,2.]);cov=np.diag([100.,10.,100.,10.,100.,10.])
    noise=np.diag([1e-8,1e-8,16.,1.]);q=[2.,60.];sojourn=4.
    filters=[]
    class ReferenceUkf(UnscentedKalmanFilter):
        # FilterPy1.4.5 IMM calls predict(u), while its UKF takes predict(dt).
        # The native additive-noise UKF applies h to the full prior including
        # Q. FilterPy caches process sigma points before adding Q, so use its
        # documented sigma regeneration operation at dt=0 before update.
        # This adapter and the circular measurement callbacks are explicit;
        # upstream IMM/UKF still perform mixing, transforms and updates.
        reference_dt=1.
        def predict(self,u=None):
            super().predict(dt=self.reference_dt)
            self.compute_process_sigmas(dt=0.)
        @property
        def log_likelihood(self):
            # Explicit SPD covariance representation avoids a pseudoinverse
            # rank decision for mixed units (radians, metres, m/s).
            return multivariate_normal.logpdf(self.y,mean=np.zeros(4),
                cov=Covariance.from_cholesky(cholesky(self.S,lower=True)))
    for _ in q:
        ukf=ReferenceUkf(6,4,1,h,f,MerweScaledSigmaPoints(6,alpha=.5,beta=2,kappa=-3),
                         z_mean_fn=mean_z,residual_z=residual_z)
        ukf.x=initial.copy();ukf.P=cov.copy();ukf.R=noise.copy();filters.append(ukf)
    bank=IMMEstimator(filters,[.5,.5],np.eye(2))
    steps=[];time=0
    for i,dt in enumerate([1.,2.,.5,3.,1.]):
        time+=dt;stay=.5+.5*math.exp(-2*dt/sojourn);bank.M=np.array([[stay,1-stay],[1-stay,stay]])
        bank._compute_mixing_probabilities()
        for ukf,diffusion in zip(filters,q):ukf.Q=process(diffusion,dt);ukf.reference_dt=dt
        bank.predict()
        truth=f(initial,time);truth[[0,2,4]]+=np.array([3.,-2.,.5])*time*time
        truth[[1,3,5]]+=np.array([6.,-4.,1.])*time
        z=h(truth)+np.array([1e-5*(-1)**i,-2e-5,2*(-1)**i,.3])
        bank.update(z)
        steps.append(dict(t=time,z=z.tolist(),x=bank.x.tolist(),p=bank.P.ravel().tolist(),probabilities=bank.mu.tolist(),
            likelihood_backends=[dict(cholesky=u.log_likelihood,eigen=multivariate_normal.logpdf(u.y,cov=u.S),
                innovation_covariance=u.S.ravel().tolist(),residual=u.y.tolist()) for u in filters]))
    files={inspect.getfile(c) for c in (IMMEstimator,UnscentedKalmanFilter,MerweScaledSigmaPoints)}
    return dict(schema='fsim.track.imm-filterpy-reference.v3',basis='simulated',equipment_calibration=False,
        convention='additive process Q included in measurement sigma cloud; circular elevation/bearing mean and residual',
        adapter='UKF.predict(u) boundary; upstream compute_process_sigmas(dt=0); circular mean_z/residual_z; SciPy SPD Cholesky covariance logpdf',
        reference_version=filterpy.__version__,reference_source='https://github.com/rlabbe/filterpy/tree/1.4.5',
        source_files=[dict(path=p,sha256=hashlib.sha256(Path(p).read_bytes()).hexdigest()) for p in sorted(files)],
        numerical_tolerances=dict(mean_absolute=1e-8,covariance_absolute=1e-7,probability_absolute=1e-10),
        sojourn_s=sojourn,process_noise=q,initial=dict(x=initial.tolist(),p=cov.ravel().tolist()),measurement_noise=noise.ravel().tolist(),steps=steps)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference-path',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if any(x.casefold()=='.codex' for x in a.output.parts):raise ValueError('Excluded output directory')
    if a.output.exists():raise ValueError('Do not overwrite a frozen reference fixture')
    data=generate(a.reference_path);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('FilterPy1.4.5 independent fixture:5 sequential nonlinear IMM updates; no hardware calibration')


if __name__=='__main__':main()
