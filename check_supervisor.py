from pathlib import Path
from concurrent.futures import Future
import json,numpy as np
from locomotion_control import LocomotionController
checks=[]
for case in ['exception','stale_solution']:
    c=LocomotionController();q=c.qref.copy();v=np.zeros(18)
    # Avoid submitting real jobs in this deterministic supervisor fixture.
    c.tick=1
    for t in [5.,5.004]:
        f=Future()
        if case=='exception':f.set_exception(RuntimeError('injected worker exception'))
        else:f.set_result((t-.2,np.zeros((4,3)),{'status':'solved'},1.))
        c.future=f
        tau,r=c.step(t,q,v,np.zeros(12))
        assert tau is not None and np.isfinite(tau).all() and max(abs(tau)/c.limits)<1.0001
    tau,r=c.step(5.008,q,v,np.zeros(12))
    assert c.mode=='STOPPED' and c.mpc_rejected==2,(case,c.mode)
    c.worker.shutdown(wait=True)
    checks.append({'case':case,'passed':True,'rejections':c.mpc_rejected,'final_mode':c.mode})
c=LocomotionController();q=c.qref.copy();q[0]=np.nan
tau,r=c.step(0.,q,np.zeros(18),np.zeros(12))
assert tau is None and r['mode']=='ABORT';c.worker.shutdown(wait=True)
checks.append({'case':'nonfinite_input','passed':True,'mode':'ABORT'})
(Path(__file__).parent/'reports/supervisor_checks.json').write_text(json.dumps(checks,indent=2))
print(json.dumps(checks))
