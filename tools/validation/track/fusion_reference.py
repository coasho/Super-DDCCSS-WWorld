"""Generate CI software-method fixtures using the pinned Stone Soup reducer.

The native 33-weight logdet grid is evaluated by Stone Soup, including endpoints.
This does not test equipment accuracy or assert continuous-optimum weights.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np
import stonesoup
from stonesoup.mixturereducer.gaussianmixture import CovarianceIntersection
from stonesoup.types.state import GaussianState


def generate():
    if stonesoup.__version__!='1.9.1':
        raise ValueError('The independent reference remains pinned to Stone Soup 1.9.1')
    rng=np.random.default_rng(12021)
    rows=[]
    for index in range(12):
        factors=[rng.normal(size=(6,6)) for _ in range(2)]
        covariances=[f@f.T+np.diag([1.,2.,3.,4.,5.,6.]) for f in factors]
        if index==0:covariances[1]=covariances[0].copy()
        if index==1:covariances[1]=4*covariances[0]
        if index==2:covariances[0]=4*covariances[1]
        states=[GaussianState(rng.normal(size=6)*10+np.array([4e6,100.,3e6,40.,2e6,1.]),p)
                for p in covariances]
        if index==0:states[1]=GaussianState(states[0].state_vector.copy(),covariances[1])
        merged=[CovarianceIntersection.merge_components(*states,weights=[i/32,1-i/32]) for i in range(33)]
        choice=min(range(33),key=lambda i:np.linalg.slogdet(merged[i].covar)[1])
        # Equal covariances make every weight a legal minimizer. Choose the
        # native deterministic first endpoint rather than relying on SVD roundoff.
        if index==0:choice=0
        result=merged[choice]
        rows.append(dict(name=f'ci-{index}',weight=choice/32,
            inputs=[dict(x=np.asarray(s.state_vector).ravel().tolist(),p=np.asarray(s.covar).ravel().tolist()) for s in states],
            x=np.asarray(result.state_vector).ravel().tolist(),p=np.asarray(result.covar).ravel().tolist()))
    source=Path(inspect.getsourcefile(CovarianceIntersection))
    return dict(schema='fsim.track.ci-stonesoup-reference.v1',basis='simulated',equipment_calibration=False,
        reference_version=stonesoup.__version__,reference_source='https://github.com/dstl/Stone-Soup/tree/v1.9.1',
        reference_source_file='stonesoup/mixturereducer/gaussianmixture.py',
        reference_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        method='Stone Soup CovarianceIntersection.merge_components; native 33-point logdet grid including endpoints',
        tolerances=dict(mean_absolute=1e-7,covariance_absolute=1e-9),cases=rows)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('Preserve existing references; use a new output')
    args.output.write_text(json.dumps(generate(),indent=2,allow_nan=False)+'\n',encoding='utf-8')
