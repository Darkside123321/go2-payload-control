"""Nominal-model inverse-dynamics standing QP; no plant parameters enter controller."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import minimize

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/'unitree_rl_mjlab/src/assets/robots/unitree_go2/xmls/go2.xml'
FEET=['FL','FR','RL','RR']
DT=.004
MU=.6

def make_model(*, imu_at_base=False):
    xml=ET.parse(SOURCE).getroot()
    if imu_at_base:
        xml.find(".//site[@name='imu']").set('pos', '0 0 0')
    xml.find('compiler').set('meshdir',str(SOURCE.parent/'assets'))
    option=xml.find('option')
    if option is None: option=ET.SubElement(xml,'option')
    option.set('timestep','0.002');option.set('gravity','0 0 -9.81')
    option.set('integrator','implicitfast')
    ET.SubElement(xml.find('worldbody'),'geom',name='floor',type='plane',size='0 0 .05',
                  friction=f'{MU} .005 .0001',condim='3',priority='1')
    for g in xml.findall('.//geom'):
        if g.get('name','').endswith('_foot_collision'):
            g.set('friction',f'{MU} .005 .0001');g.set('condim','3');g.set('priority','1')
    act=xml.find('actuator')
    if act is not None:xml.remove(act)
    act=ET.SubElement(xml,'actuator');limits=[]
    for j in xml.findall('.//joint'):
        if not j.get('name','').endswith('_joint'):continue
        limit=45. if 'calf' in j.get('name') else 23.5
        limits.append(limit)
        ET.SubElement(act,'motor',name=j.get('name')+'_motor',joint=j.get('name'),gear='1',
                      ctrllimited='true',ctrlrange=f'{-limit} {limit}')
    model=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))
    return model,np.array(limits)

def initial_state(model):
    d=mujoco.MjData(model);d.qpos[2]=.35;d.qpos[3:7]=[1,0,0,0]
    for i in range(model.njnt):
        if model.jnt_type[i]!=mujoco.mjtJoint.mjJNT_HINGE:continue
        name=model.joint(i).name
        d.qpos[model.jnt_qposadr[i]]=.9 if 'thigh' in name else -1.8 if 'calf' in name else 0.
    mujoco.mj_forward(model,d)
    gids=[model.geom(f+'_foot_collision').id for f in FEET]
    d.qpos[2]-=min(d.geom_xpos[g,2]-model.geom_size[g,0] for g in gids)
    mujoco.mj_forward(model,d)
    return d.qpos.copy()

class StandingController:
    def __init__(self,model,limits,qref):
        self.model=model;self.data=mujoco.MjData(model);self.limits=limits;self.qref=qref.copy()
        self.prev_J=None
        self.sids=[model.site(f).id for f in FEET]
        self.H=np.r_[np.full(6,100.),np.ones(12),np.full(12,1e-4)]
        self.force_ref=np.tile([0.,0.,model.body_mass.sum()*9.81/4],4)
        self.kp=np.r_[[40.,40.,160.,100.,100.,80.],np.full(12,20.)]
        self.kd=np.r_[[12.,12.,25.,18.,18.,15.],np.full(12,4.)]
        self.mass=np.zeros((18,18))
        self.friction=np.zeros((20,30))
        for i in range(4):
            for j,(sx,sy) in enumerate([(1,1),(1,-1),(-1,1),(-1,-1)]):
                self.friction[5*i+j,18+3*i:18+3*i+3]=[sx,sy,-MU]
            self.friction[5*i+4,18+3*i+2]=-1.
    def bias_force(self):
        return self.data.qfrc_bias.copy()
    def step(self,qpos,qvel):
        m=self.model;d=self.data
        if np.shape(qpos)!=(19,) or np.shape(qvel)!=(18,) or not np.isfinite(qpos).all() or not np.isfinite(qvel).all():
            return None,{'success':False,'reason':'invalid state','active_solver':False}
        d.qpos[:]=qpos;d.qvel[:]=qvel;mujoco.mj_forward(m,d)
        J=[]
        for sid in self.sids:
            jp=np.zeros((3,18));jr=jp.copy();mujoco.mj_jacSite(m,d,jp,jr,sid);J.append(jp)
        J=np.vstack(J)
        jdotv=np.zeros(12) if self.prev_J is None else ((J-self.prev_J)/DT)@qvel
        self.prev_J=J.copy()
        mujoco.mj_fullM(m,self.mass,d.qM)
        M=self.mass;h=self.bias_force()
        error=np.zeros(18);mujoco.mj_differentiatePos(m,error,1.,qpos,self.qref)
        aref=self.kp*error-self.kd*qvel
        ref=np.r_[aref,self.force_ref]
        A=np.vstack([np.hstack([M[:6],-J[:,:6].T]),np.hstack([J,np.zeros((12,12))])])
        b=np.r_[-h[:6],-jdotv]
        T=np.hstack([M[6:],-J[:,6:].T])
        G=np.vstack([self.friction,T,-T])
        upper=np.r_[np.zeros(20),self.limits-h[6:],self.limits+h[6:]]
        # Exact equality-constrained optimum; if feasible, it is also the constrained QP optimum.
        AH=A/self.H
        try:
            x=ref+AH.T@np.linalg.solve(AH@A.T,b-A@ref)
        except np.linalg.LinAlgError:
            return None,{'success':False,'reason':'singular dynamics','active_solver':False}
        active_solver=False;success=True;reason='feasible equality optimum'
        if np.max(G@x-upper)>1e-7:
            active_solver=True
            fun=lambda z:.5*np.dot(self.H*(z-ref),z-ref)
            jac=lambda z:self.H*(z-ref)
            sol=minimize(fun,x,jac=jac,method='SLSQP',
                constraints=[{'type':'eq','fun':lambda z:A@z-b,'jac':lambda z:A},
                             {'type':'ineq','fun':lambda z:upper-G@z,'jac':lambda z:-G}],
                options={'maxiter':60,'ftol':1e-9})
            x=sol.x;success=bool(sol.success);reason=str(sol.message)
        eq=float(np.max(np.abs(A@x-b)));ineq=float(max(0.,np.max(G@x-upper)))
        success=success and eq<1e-5 and ineq<1e-6 and np.isfinite(x).all()
        tau=T@x+h[6:]
        return (tau if success else None),{'success':bool(success),'reason':reason,
            'eq_residual':eq,'ineq_violation':ineq,'active_solver':active_solver,
            'forces':x[18:].reshape(4,3)}
