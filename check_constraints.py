"""Targeted acceptance checks for active constraints and invalid measurements."""
import json,time
import numpy as np
from standing_control import *
def main():
    m,limits=make_model();q=initial_state(m);v=np.zeros(18);checks=[]
    for name,offset,scale in [('friction_boundary',.5,1.),('torque_boundary',0.,.05)]:
        ref=q.copy();ref[0]+=offset
        controller=StandingController(m,limits*scale,ref)
        t=time.perf_counter();tau,r=controller.step(q,v);elapsed=(time.perf_counter()-t)*1000
        assert r['success'] and r['active_solver'] and tau is not None,r
        f=r.pop('forces')
        friction=np.abs(f[:,:2]).sum(axis=1)-MU*f[:,2]
        utilization=float(np.max(np.abs(tau)/(limits*scale)))
        assert f[:,2].min()>=-1e-6 and friction.max()<1e-6 and utilization<=1+1e-6
        if name=='friction_boundary':assert max(friction)>-1e-5
        if name=='torque_boundary':assert utilization>.999
        r.update(name=name,passed=True,wall_ms=elapsed,max_torque_utilization=utilization,
                 max_friction_pyramid_excess_N=float(friction.max()),forces_N=f.tolist(),
                 scope='one QP solve at reference pose; not a dynamic load experiment')
        checks.append(r)
    controller=StandingController(m,limits,q)
    invalid=q.copy();invalid[0]=np.nan
    tau,r=controller.step(invalid,v)
    assert tau is None and not r['success']
    checks.append({'name':'nonfinite_state_rejected','passed':True,'reason':r['reason']})
    (ROOT/'reports/constraint_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps([{'name':r['name'],'passed':r['passed']} for r in checks]))
if __name__=='__main__':main()
