"""Run the fixed evaluation manifest sequentially; reuse matching completed cases."""
from pathlib import Path
import json,os,subprocess,sys,hashlib
ROOT=Path(__file__).resolve().parent
cfg=json.loads((ROOT/'configs/walking_eval.json').read_text())
results=[]
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
for c in cfg['cases']:
    path=ROOT/'reports/walking_async_final'/c['name']
    if not (path/'summary.json').exists():
        cmd=[sys.executable,str(ROOT/'run_walking.py'),'--name',c['name'],'--load',str(c['load'])]
        if not c['adaptive']:cmd+=['--baseline']
        if c.get('stress'):cmd+=['--stress']
        if c.get('fault'):cmd+=['--fault']
        if 'fault_start' in c:cmd+=['--fault-start',str(c['fault_start'])]
        with (ROOT/f"{c['name']}.log").open('w') as f:
            p=subprocess.run(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
        if p.returncode:raise RuntimeError(f"See {c['name']}.log")
    setting=json.loads((path/'config.json').read_text())
    assert setting['load']==c['load'] and setting['baseline']==(not c['adaptive'])
    assert setting['seconds']==12. and setting['velocity']==.1
    assert bool(setting['stress'])==bool(c.get('stress',False)) and bool(setting['fault'])==bool(c.get('fault',False))
    assert setting.get('fault_start',6.)==c.get('fault_start',6.)
    s=json.loads((path/'summary.json').read_text());s['case']=c
    if c.get('fault'):
        g=cfg['fault_acceptance']
        s['acceptance_passed']=bool(s['completed'] and s['final_mode']==g['final_mode'] and
            s['final_speed_mps']<=g['final_speed_mps'] and s['peak_tilt_deg']<=g['tilt_deg'])
    else:
        g=cfg['acceptance']
        s['acceptance_passed']=bool(s['completed'] and s['velocity_rmse_mps']<=g['velocity_rmse_mps'] and
            s['height_rmse_mm']<=g['height_rmse_mm'] and s['peak_tilt_deg']<=g['tilt_deg'] and
            s['max_torque_utilization']<=g['torque_utilization'] and s['distance_m']>=g['minimum_travel_m'] and
            s['max_measured_foot_clearance_m']>=g['minimum_foot_clearance_m'] and
            s['actual_contact_changes']>=g['minimum_contact_changes'])
    results.append(s)
    print(json.dumps({'case':c['name'],'completed':s['completed'],'passed':s['acceptance_passed'],
       'height_mm':s['height_rmse_mm'],'velocity_rmse':s['velocity_rmse_mps'],'mode':s['final_mode']}),flush=True)
bundle={'protocol':cfg,'results':results,'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
 for p in [*ROOT.glob('*.py'),ROOT/'cpp/control_core.cpp']},
 'scope':'flat-ground 0.1m/s forward trot, one initial state per case; ideal state feedback',
 'cpp':'centroidal input matrix, trot scheduler, quintic/quartic swing trajectory',
 'timing':'wall-paced WSL simulation, asynchronous MPC worker + servo thread; compute percentiles and late cycles reported separately; no hard-real-time guarantee'}
(ROOT/'reports/walking_async_final/benchmark.json').write_text(json.dumps(bundle,indent=2)+'\n')
