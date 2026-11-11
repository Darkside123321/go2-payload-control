from pathlib import Path
import json,time
import numpy as np
from locomotion_control import LocomotionController,solve_qp
c=LocomotionController();d=c.d;feet=np.array([d.site_xpos[s] for s in c.sids])
state=np.r_[c.initial_com,np.zeros(9)]
errors=[];cached=[];fresh=[]
for i in range(20):
    wt=.2 if i%2 else -.1
    begin=time.perf_counter()
    force,r=c.mpc.solve(state,feet,c.initial_inertia,wt,c.initial_com,np.array([.1,0,0]),c.mpc.mass)
    cached.append((time.perf_counter()-begin)*1000)
    P,q,A,l,u=c.mpc.last_problem
    begin=time.perf_counter();x,receipt=solve_qp(P,q,A,l,u,eps=1e-7)
    fresh.append((time.perf_counter()-begin)*1000)
    assert force is not None and x is not None
    off=11*12;err=float(abs(force.ravel()-x[off:off+12]).max());errors.append(err)
    assert err<.05,err
out={'passed':True,'cases':20,'max_first_action_difference_N':max(errors),
'cached_full_planner_p50_ms':float(np.median(cached[1:])),
'fresh_QP_setup_and_solve_p50_ms':float(np.median(fresh)),
'interpretation':'fresh cost excludes Python model/constraint assembly; measurements are not end-to-end controller guarantees'}
(Path(__file__).parent/'reports/cache_checks.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out))
