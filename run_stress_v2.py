"""One 10 s gravity-compensated run per load, paired with preserved standing_v1."""
from pathlib import Path
import csv,hashlib,json,time
import mujoco,numpy as np
from standing_control import make_model,initial_state,StandingController,DT,FEET,SOURCE
from compensated_control import CompensatedStandingController

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'reports/stress_v2'
OUT.mkdir(parents=True,exist_ok=True)
if (OUT/'summary.json').exists():raise SystemExit('Existing completed stress_v2; refusing accidental rerun')
cfg={'duration_s':10.,'control_dt_s':DT,'physics_dt_s':.002,'warmup_s':1.,
 'load_fractions':[.2],'load_location':'point payload at nominal base-link inertial COM; no added rotational inertia about that point',
 'state_input':'ideal qpos/qvel; no state estimator','stance':'four scheduled point-contact feet at site centers',
 'controller_mass':'fixed nominal inertia, gravity compensated using estimated payload',
 'acceleration_gate':'linear norm<0.2m/s2; angular norm<1rad/s2; max joint<2rad/s2; backward velocity difference',
 'estimator_inputs':'ideal qpos/qvel and joint torque measurement with Gaussian noise; no plant mass/contact-force access',
 'torque_noise_sigma_Nm':.03,'noise_seed':2026,
 'push_world_N':[20.,0.,0.],'push_start_s':4.,'push_end_s':4.15,
 'fail_height_m':.14,'fail_tilt_deg':30.,'accept_height_rmse_m':.02,'accept_peak_tilt_deg':5.,
 'effort_limits':'upstream configuration, not hardware-validated ratings',
 'upstream_asset_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest()}
(OUT/'config.json').write_text(json.dumps(cfg,indent=2)+'\n')
summaries=[]
for fraction in cfg['load_fractions']:
    nominal,limits=make_model();plant,_=make_model()
    qref=initial_state(nominal)
    nominal_mass=float(nominal.body_mass.sum())
    base=plant.body('base_link').id
    added=nominal_mass*fraction
    plant.body_mass[base]+=added
    # Payload is an ideal point mass at the existing link COM; inertial axes/COM stay fixed.
    d=mujoco.MjData(plant);mujoco.mj_setConst(plant,d)
    d.qpos[:]=qref;mujoco.mj_forward(plant,d)
    ctrl=CompensatedStandingController(nominal,limits,qref)
    rows=[];durations=[];maxeq=0.;maxineq=0.;qp_fail=0;active=0;reason=None
    rng=np.random.default_rng(2026)
    t0=time.perf_counter()
    for tick in range(round(cfg['duration_s']/DT)):
        before=time.perf_counter_ns()
        tau,receipt=ctrl.step(d.qpos.copy(),d.qvel.copy(),d.qfrc_actuator[6:].copy()+rng.normal(0.,.03,12))
        elapsed=(time.perf_counter_ns()-before)/1e6;durations.append(elapsed)
        active+=int(receipt.get('active_solver',False))
        if tau is None:
            qp_fail+=1;reason=receipt['reason']
            # Failure path terminates this experiment; emergency clipped damping is applied once.
            d.ctrl[:]=np.clip(-2*d.qvel[6:],-limits,limits);mujoco.mj_step(plant,d);break
        maxeq=max(maxeq,receipt['eq_residual']);maxineq=max(maxineq,receipt['ineq_violation'])
        d.ctrl[:]=tau
        for _ in range(2):
            d.xfrc_applied[base,:]=0.
            if 4.<=d.time<4.15:d.xfrc_applied[base,0]=20.
            mujoco.mj_step(plant,d)
        mujoco.mj_forward(plant,d)
        tilt=float(np.degrees(np.arccos(np.clip(d.xmat[base].reshape(3,3)[2,2],-1,1))))
        finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
        feet_in_contact=set()
        for c in d.contact:
            for g in [c.geom1,c.geom2]:
                name=plant.geom(g).name
                if name.endswith('_foot_collision'):feet_in_contact.add(name)
        rows.append({'time_s':float(d.time),'height_m':float(d.qpos[2]),'height_error_m':float(d.qpos[2]-qref[2]),
          'tilt_deg':tilt,'base_xy_error_m':float(np.linalg.norm(d.qpos[:2]-qref[:2])),
          'max_torque_utilization':float(np.max(np.abs(tau)/limits)),
          'saturated_joints':int(np.sum(np.abs(tau)>=limits-1e-6)),
          'foot_contacts':len(feet_in_contact),'solve_ms':elapsed,
          'eq_residual':receipt['eq_residual'],'ineq_violation':receipt['ineq_violation'],
          'estimated_payload_kg':ctrl.observer.estimate_kg,'estimator_updated':int(ctrl.observation['updated']),
          'estimator_reason':ctrl.observation.get('reason','updated'),
          **{f'tau_{i}_Nm':float(tau[i]) for i in range(12)}})
        if not finite or d.qpos[2]<cfg['fail_height_m'] or tilt>cfg['fail_tilt_deg']:
            reason='nonfinite or pose failure';break
    name='nominal' if fraction==0 else f'payload{int(fraction*100)}'
    if rows:
        with (OUT/f'{name}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    scored=[r for r in rows if r['time_s']>=cfg['warmup_s']]
    heights=np.array([r['height_error_m'] for r in scored]);tilts=np.array([r['tilt_deg'] for r in scored])
    full=bool(reason is None and len(rows)==round(cfg['duration_s']/DT))
    s={'scenario':name,'added_payload_kg':added,'nominal_controller_mass_kg':float(ctrl.model.body_mass.sum()),
       'completed':full,'sim_seconds':float(d.time),'wall_seconds':time.perf_counter()-t0,
       'height_rmse_m':float(np.sqrt(np.mean(heights**2))) if len(heights) else None,
       'steady_height_bias_m':float(np.mean(heights)) if len(heights) else None,
       'peak_tilt_deg':max((r['tilt_deg'] for r in rows),default=None),
       'max_torque_utilization':max((r['max_torque_utilization'] for r in rows),default=None),
       'saturation_fraction_joint_ticks':sum(r['saturated_joints'] for r in rows)/max(1,len(rows)*12),
       'four_foot_contact_fraction':sum(r['foot_contacts']==4 for r in rows)/max(1,len(rows)),
       'solve_p50_ms':float(np.percentile(durations,50)),'solve_p95_ms':float(np.percentile(durations,95)),
       'solve_p99_ms':float(np.percentile(durations,99)),'solve_max_ms':max(durations),
       'over_4ms_fraction':float(np.mean(np.array(durations)>4)),
       'max_equality_residual':maxeq,'max_inequality_violation':maxineq,
       'qp_failures':qp_fail,'active_inequality_solves':active,'failure_reason':reason,
       'final_estimated_payload_kg':ctrl.observer.estimate_kg,
       'final_estimation_error_kg':ctrl.observer.estimate_kg-added,
       'estimator_update_fraction':sum(r['estimator_updated'] for r in rows)/max(1,len(rows))}
    s['acceptance_passed']=bool(full and s['height_rmse_m']<=cfg['accept_height_rmse_m'] and
        s['peak_tilt_deg']<=cfg['accept_peak_tilt_deg'] and maxineq<1e-6)
    summaries.append(s)
    print(json.dumps(s),flush=True)
    if not full:break
(OUT/'summary.json').write_text(json.dumps({'config':cfg,'results':summaries,
    'stress_passed':bool(len(summaries)==1 and all(s['acceptance_passed'] for s in summaries))},indent=2)+'\n')
