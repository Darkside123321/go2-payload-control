from pathlib import Path
import argparse,csv,json,time,hashlib
import numpy as np,mujoco
from locomotion_control import LocomotionController,STAND_SECONDS
from standing_control import make_model,DT
p=argparse.ArgumentParser()
p.add_argument('--name',required=True);p.add_argument('--report-root',default='walking_async_final');p.add_argument('--load',type=float,default=0.)
p.add_argument('--baseline',action='store_true');p.add_argument('--seconds',type=float,default=12.)
p.add_argument('--velocity',type=float,default=.1);p.add_argument('--fault',action='store_true');p.add_argument('--fault-start',type=float,default=6.)
p.add_argument('--stress',action='store_true')
args=p.parse_args();ROOT=Path(__file__).parent;out=ROOT/'reports'/args.report_root/args.name
out.mkdir(parents=True,exist_ok=True)
if (out/'summary.json').exists():raise SystemExit('Refusing overwrite '+str(out))
controller=LocomotionController(adaptive=not args.baseline,velocity=args.velocity)
plant,limits=make_model();nom=plant.body_mass.sum();base=plant.body('base_link').id
plant.body_mass[base]+=nom*args.load
d=mujoco.MjData(plant);mujoco.mj_setConst(plant,d);d.qpos[:]=controller.qref;mujoco.mj_forward(plant,d)
config=vars(args).copy();config.update({'control_dt':DT,'physics_dt':.002,'initial_stand_s':STAND_SECONDS,
'state':'ideal qpos/qvel; measured actual actuator torques','friction':.6,'horizon':10,'mpc_dt':.02,
'payload_location':'base link inertial COM, point mass','cpp_source_sha256':hashlib.sha256((ROOT/'cpp/control_core.cpp').read_bytes()).hexdigest()})
(out/'config.json').write_text(json.dumps(config,indent=2))
rows=[];failure=None;start=time.perf_counter();mpc_statuses={};late_cycles=0
rng=np.random.default_rng(2026);contact_counts=[];max_swing_clearance=0.;actual_contact_changes=0;previous_actual=None
for k in range(round(args.seconds/DT)):
    t=float(d.time)
    begin=time.perf_counter()
    inject=args.fault and args.fault_start<=t<args.fault_start+.06
    tau,rec=controller.step(t,d.qpos.copy(),d.qvel.copy(),d.qfrc_actuator[6:].copy()+(rng.normal(0.,.03,12) if args.stress else 0.),inject)
    control_ms=(time.perf_counter()-begin)*1000
    mpc_statuses[rec.get('mpc_status','none')]=mpc_statuses.get(rec.get('mpc_status','none'),0)+1
    if tau is None:
        failure=rec;break
    d.ctrl[:]=np.clip(tau,-limits,limits)
    for _ in range(2):
        d.xfrc_applied[base,:]=0.
        if args.stress and 7.<=d.time<7.15:d.xfrc_applied[base,0]=20.
        mujoco.mj_step(plant,d)
    mujoco.mj_forward(plant,d)
    tilt=float(np.degrees(np.arccos(np.clip(d.xmat[base].reshape(3,3)[2,2],-1,1))))
    actual=set()
    for contact in d.contact:
        for geom in [contact.geom1,contact.geom2]:
            name=plant.geom(geom).name
            if name.endswith('_foot_collision'):actual.add(name[:2])
    actual_tuple=tuple(sorted(actual))
    if previous_actual is not None and actual_tuple!=previous_actual:actual_contact_changes+=1
    previous_actual=actual_tuple;contact_counts.append(len(actual))
    footz=[float(d.site_xpos[plant.site(leg).id,2]) for leg in ['FL','FR','RL','RR']]
    max_swing_clearance=max(max_swing_clearance,max(footz)-.022)
    rows.append({'time_s':float(d.time),'x_m':float(d.qpos[0]),'vx_mps':float(d.qvel[0]),'vy_mps':float(d.qvel[1]),
        'height_error_m':float(d.qpos[2]-controller.qref[2]),'tilt_deg':tilt,
        'payload_estimate_kg':controller.payload,'mode':rec['mode'],'control_ms':control_ms,'mpc_ms':rec['mpc_ms'],
        'qp_violation':rec['qp_violation'],'mpc_age_s':rec['mpc_age_s'],'torque_utilization':rec['joint_torque_utilization'],
        'scheduled_support':sum(rec['contacts']),'actual_support':len(actual),
        **{f'{leg}_z_m':footz[i] for i,leg in enumerate(['FL','FR','RL','RR'])},
        **{f'tau_{i}_Nm':float(tau[i]) for i in range(12)},
        **{f'force_{i}_N':float(v) for i,v in enumerate(rec['planned_forces'].ravel())}})
    if not np.isfinite(d.qpos).all() or d.qpos[2]<.14 or tilt>30.:
        failure={'reason':'pose or numerical failure','time_s':float(d.time)};break
    deadline=start+(k+1)*DT
    remaining=deadline-time.perf_counter()
    if remaining>0:time.sleep(remaining)
    else:late_cycles+=1
controller.worker.shutdown(wait=True)
if rows:
    with (out/'trace.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
scored=[r for r in rows if r['time_s']>=STAND_SECONDS+1.]
mpctimes=[r['mpc_ms'] for r in rows if r['mpc_ms']>0]
s={'completed':failure is None and len(rows)==round(args.seconds/DT),'failure':failure,
'sim_seconds':float(d.time),'wall_seconds':time.perf_counter()-start,'distance_m':float(d.qpos[0]-controller.qref[0]),
'height_rmse_mm':float(np.sqrt(np.mean([r['height_error_m']**2 for r in scored]))*1000) if scored else None,
'velocity_rmse_mps':float(np.sqrt(np.mean([(r['vx_mps']-args.velocity)**2+r['vy_mps']**2 for r in scored]))) if scored else None,
'peak_tilt_deg':max((r['tilt_deg'] for r in rows),default=None),
'max_torque_utilization':max((r['torque_utilization'] for r in rows),default=None),
'max_qp_violation':max((r['qp_violation'] for r in rows),default=None),
'control_p99_ms':float(np.percentile([r['control_ms'] for r in rows],99)) if rows else None,
'mpc_p99_ms':float(np.percentile(mpctimes,99)) if mpctimes else None,'mpc_status_counts':mpc_statuses,
'final_payload_estimate_kg':controller.payload,'final_mode':controller.mode,
'actual_contact_changes':actual_contact_changes,'minimum_actual_support':min(contact_counts) if rows else 0,
'max_measured_foot_clearance_m':max_swing_clearance,
'final_speed_mps':float(np.linalg.norm(d.qvel[:3])),
'mpc_accepted_updates':controller.mpc_updates,'mpc_rejected_updates':controller.mpc_rejected,
'late_cycles':late_cycles,'cycle_count':len(rows),'realtime_pacing':True}
(out/'summary.json').write_text(json.dumps(s,indent=2));print(json.dumps(s),flush=True)
