"""Numerical parity and trajectory/scheduler regression checks for the C++ control core."""
from pathlib import Path
import json
import numpy as np
from cpp_core import model_B,schedule,swing,PERIOD,DUTY
rng=np.random.default_rng(2026);worst=0.
for _ in range(20):
    mass=float(rng.uniform(10,25));F=rng.normal(size=(3,3));inv=np.linalg.inv(F@F.T+np.eye(3))
    arms=rng.normal(size=(4,3));dt=.02;ref=np.zeros((12,12))
    for i,a in enumerate(arms):
        S=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
        ref[:3,3*i:3*i+3]=.5*dt*dt*np.eye(3)/mass
        ref[3:6,3*i:3*i+3]=dt*np.eye(3)/mass
        ref[6:9,3*i:3*i+3]=.5*dt*dt*inv@S
        ref[9:12,3*i:3*i+3]=dt*inv@S
    worst=max(worst,float(abs(model_B(mass,inv,arms,dt)-ref).max()))
assert worst<1e-12
a=np.array([.1,-.1,.022]);b=a+np.array([.05,.02,0.])
T=PERIOD*(1-DUTY)
for phase in [.2,.5,.8]:
    p,v,acc=swing(a,b,phase);eps=1e-5
    pp,vp,_=swing(a,b,phase+eps);pm,vm,_=swing(a,b,phase-eps)
    assert np.max(abs((pp-pm)/(2*eps*T)-v))<1e-6
    assert np.max(abs((vp-vm)/(2*eps*T)-acc))<1e-4
assert np.allclose(swing(a,b,0)[0],a) and np.allclose(swing(a,b,1)[0],b)
assert np.allclose(swing(a,b,0)[1],0) and np.allclose(swing(a,b,1)[1],0)
for t in np.linspace(0,PERIOD,101):
    contact,phase=schedule(float(t))
    assert sum(contact) in [2,4]
    assert contact[0]==contact[3] and contact[1]==contact[2]
    assert np.all((phase>=0)&(phase<=1))
out={'passed':True,'seed':2026,'cpp_prediction_parity_cases':20,'B_max_abs_error':worst,
'swing_derivative_phases':[.2,.5,.8],'schedule_cases':101}
(Path(__file__).parent/'reports/cpp_checks.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
