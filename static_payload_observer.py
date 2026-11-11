"""Near-static support-force observer, using nominal dynamics and joint torque measurements."""
import numpy as np
import mujoco
from learner_core import payload_from_support,lowpass,weight_compensation
from standing_control import make_model,FEET,DT
class StaticPayloadObserver:
    def __init__(self):
        self.model,_=make_model();self.data=mujoco.MjData(self.model)
        self.nominal_mass=float(self.model.body_mass.sum())
        self.estimate_kg=0.;self.still_time=0.
        self.alpha=1.-np.exp(-DT/.5)
        self.previous_velocity=None
    def update(self,qpos,qvel,measured_joint_torques):
        if not all(np.isfinite(v).all() for v in [qpos,qvel,measured_joint_torques]):
            return {'updated':False,'reason':'invalid measurement','estimate_kg':self.estimate_kg}
        # Quasi-static only: do not interpret accelerating-body support as payload.
        acc=np.zeros(18) if self.previous_velocity is None else (qvel-self.previous_velocity)/DT
        self.previous_velocity=qvel.copy()
        low_speed=np.linalg.norm(qvel[:3])<.02 and np.linalg.norm(qvel[3:6])<.05 and np.max(abs(qvel[6:]))<.1
        low_acc=np.linalg.norm(acc[:3])<.2 and np.linalg.norm(acc[3:6])<1. and np.max(abs(acc[6:]))<2.
        still=low_speed and low_acc
        self.still_time=self.still_time+DT if still else 0.
        if self.still_time<.3:
            return {'updated':False,'reason':'waiting for near-static window','estimate_kg':self.estimate_kg}
        m=self.model;d=self.data;d.qpos[:]=qpos;d.qvel[:]=qvel;mujoco.mj_forward(m,d)
        J=[]
        for foot in FEET:
            jp=np.zeros((3,18));jr=jp.copy()
            mujoco.mj_jacSite(m,d,jp,jr,m.site(foot).id);J.append(jp[:,6:])
        A=np.vstack(J).T
        if np.linalg.cond(A)>1e4:
            return {'updated':False,'reason':'ill-conditioned leg Jacobian','estimate_kg':self.estimate_kg}
        # At near rest: h_joint = measured_tau + J_joint.T @ contact_force.
        f=np.linalg.solve(A,d.qfrc_bias[6:]-measured_joint_torques).reshape(4,3)
        if np.any(f[:,2]<0):
            return {'updated':False,'reason':'invalid reconstructed support','estimate_kg':self.estimate_kg}
        total=float(f[:,2].sum())
        raw=payload_from_support(total,self.nominal_mass,9.81)
        bounded=float(np.clip(raw,0.,.3*self.nominal_mass))
        self.estimate_kg=float(lowpass(self.estimate_kg,bounded,self.alpha))
        return {'updated':True,'estimate_kg':self.estimate_kg,'raw_payload_kg':float(raw),
                'estimated_support_N':total,'extra_support_N':float(weight_compensation(self.estimate_kg,9.81))}
