"""Optional graphical demo; closes after the requested duration."""
import argparse,time
import numpy as np,mujoco
from locomotion_control import LocomotionController
from standing_control import make_model,DT
def main():
    p=argparse.ArgumentParser()
    p.add_argument('--load',type=float,default=.2)
    p.add_argument('--velocity',type=float,default=.1)
    p.add_argument('--seconds',type=float,default=12.)
    a=p.parse_args()
    import mujoco.viewer
    c=LocomotionController(adaptive=True,velocity=a.velocity)
    plant,limits=make_model();base=plant.body('base_link').id
    plant.body_mass[base]+=plant.body_mass.sum()*a.load
    d=mujoco.MjData(plant);mujoco.mj_setConst(plant,d)
    d.qpos[:]=c.qref;mujoco.mj_forward(plant,d)
    try:
        with mujoco.viewer.launch_passive(plant,d) as viewer:
            viewer.cam.distance=1.8;viewer.cam.azimuth=135;viewer.cam.elevation=-20
            start=time.perf_counter()
            for k in range(round(a.seconds/DT)):
                if not viewer.is_running():break
                tau,r=c.step(float(d.time),d.qpos.copy(),d.qvel.copy(),d.qfrc_actuator[6:].copy())
                if tau is None:raise RuntimeError(r)
                d.ctrl[:]=np.clip(tau,-limits,limits)
                for _ in range(2):mujoco.mj_step(plant,d)
                mujoco.mj_forward(plant,d)
                viewer.cam.lookat[:]=d.qpos[:3]
                if k%8==0:viewer.sync()
                wait=start+(k+1)*DT-time.perf_counter()
                if wait>0:time.sleep(wait)
    finally:c.worker.shutdown(wait=True)
if __name__=='__main__':main()
