"""Read-only upstream assets; in-memory torque actuators for a dynamics audit."""
from pathlib import Path
import hashlib,json,subprocess,xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
UPSTREAM=ROOT.parent/'unitree_rl_mjlab'
SOURCE=UPSTREAM/'src/assets/robots/unitree_go2/xmls/go2.xml'
tree=ET.parse(SOURCE)
xml=tree.getroot()
xml.find('compiler').set('meshdir',str(SOURCE.parent/'assets'))
option=xml.find('option')
if option is None:option=ET.SubElement(xml,'option')
option.set('timestep','0.002')
option.set('gravity','0 0 -9.81')
world=xml.find('worldbody')
ET.SubElement(world,'geom',name='audit_floor',type='plane',size='0 0 .05',friction='.6 .005 .0001')
act=xml.find('actuator')
if act is not None:xml.remove(act)
act=ET.SubElement(xml,'actuator')
joints=[j for j in xml.findall('.//joint') if j.get('name','').endswith('_joint')]
assert len(joints)==12
limits=[]
for j in joints:
    limit=45. if 'calf' in j.get('name') else 23.5
    limits.append(limit)
    ET.SubElement(act,'motor',name=j.get('name')+'_motor',joint=j.get('name'),gear='1',
                  ctrllimited='true',ctrlrange=f'{-limit} {limit}')
model=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))
data=mujoco.MjData(model)
assert (model.nq,model.nv,model.nu)==(19,18,12)
data.qpos[2]=.35
data.qpos[3:7]=[1,0,0,0]
for jid in range(model.njnt):
    name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,jid)
    if model.jnt_type[jid]==mujoco.mjtJoint.mjJNT_HINGE:
        data.qpos[model.jnt_qposadr[jid]]=.9 if 'thigh' in name else -1.8 if 'calf' in name else 0.
mujoco.mj_forward(model,data)
feet=['FL','FR','RL','RR']
gids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f'{f}_foot_collision') for f in feet]
data.qpos[2]-=min(data.geom_xpos[g,2]-model.geom_size[g,0] for g in gids)
mujoco.mj_forward(model,data)
J=[]
for foot in feet:
    sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,foot)
    jp=np.zeros((3,model.nv));jr=jp.copy()
    mujoco.mj_jacSite(model,data,jp,jr,sid)
    J.append(jp)
J=np.vstack(J)
bias=data.qfrc_bias.copy()
forces=np.linalg.lstsq(J[:,:6].T,bias[:6],rcond=None)[0]
tau=bias[6:]-J[:,6:].T@forces
residual=np.linalg.norm(J.T@forces+np.r_[np.zeros(6),tau]-bias)
fz=forces.reshape(4,3)[:,2]
friction_margin=.6*fz-np.linalg.norm(forces.reshape(4,3)[:,:2],axis=1)
# Check one Jacobian column via tangent-space finite difference of foot-site positions.
q0=data.qpos.copy()
sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'FL')
err=[]
for k in range(model.nv):
    dv=np.zeros(model.nv);dv[k]=1.
    pts=[]
    for sign in [1.,-1.]:
        data.qpos[:]=q0
        mujoco.mj_integratePos(model,data.qpos,dv,sign*1e-7)
        mujoco.mj_forward(model,data)
        pts.append(data.site_xpos[sid].copy())
    err.append(np.max(np.abs((pts[0]-pts[1])/2e-7-J[:3,k])))
data.qpos[:]=q0;mujoco.mj_forward(model,data)
M=np.zeros((model.nv,model.nv))
mujoco.mj_fullM(model,M,data.qM)
# Unit-gain motor interface audit.
data.ctrl[:]=1.;mujoco.mj_forward(model,data)
act_error=float(np.max(np.abs(data.qfrc_actuator-np.r_[np.zeros(6),np.ones(12)])))
data.ctrl[:]=tau
height0=float(data.qpos[2])
for _ in range(50):mujoco.mj_step(model,data)
mass=float(model.body_mass.sum())
report={
    'scope':'Static ideal point-contact equilibrium and 0.1 s open-loop interface smoke check; not a standing controller or MPC result',
    'upstream_commit':subprocess.check_output(['git','-C',str(UPSTREAM),'rev-parse','HEAD'],text=True).strip(),
    'asset_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'mujoco':mujoco.__version__,'nq':model.nq,'nv':model.nv,'actuated_joints':model.nu,
    'mass_kg':mass,'gravity_weight_N':mass*9.81,
    'mass_matrix_min_eigenvalue':float(np.linalg.eigvalsh(M).min()),
    'foot_order':feet,'contact_model':'ideal point forces at original foot site centers',
    'foot_forces_world_N':forces.reshape(4,3).tolist(),
    'vertical_force_sum_N':float(fz.sum()),'equilibrium_residual_norm':float(residual),
    'nominal_joint_torques_Nm':tau.tolist(),'actuator_order':[j.get('name') for j in joints],
    'configured_effort_limits_Nm':limits,
    'effort_limit_provenance':'upstream go2_constants.py configuration; not verified hardware ratings',
    'max_torque_limit_utilization':float(np.max(np.abs(tau)/limits)),
    'min_friction_margin_N':float(friction_margin.min()),
    'jacobian_fd_max_abs_error':float(max(err)),'unit_motor_mapping_error':act_error,
    'physics_seconds':float(data.time),'base_height_start_m':height0,'base_height_end_m':float(data.qpos[2]),
    'last_contact_count':data.ncon,'finite_state':bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
}
report['passed']=bool(residual<1e-7 and min(err)<1e-5 and max(err)<1e-5 and act_error<1e-10 and
                     fz.min()>0 and friction_margin.min()>0 and max(np.abs(tau)/limits)<1 and
                     report['finite_state'] and report['mass_matrix_min_eigenvalue']>0)
(ROOT/'reports').mkdir(exist_ok=True)
(ROOT/'reports/dynamics_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['passed','mass_kg','actuated_joints','gravity_weight_N','vertical_force_sum_N','equilibrium_residual_norm','jacobian_fd_max_abs_error','unit_motor_mapping_error','max_torque_limit_utilization','physics_seconds','finite_state']},indent=2))
assert report['passed']
