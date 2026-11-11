"""Estimated-payload gravity compensation; nominal inertia and fixed gains retained."""
import numpy as np
import mujoco
from standing_control import StandingController
from static_payload_observer import StaticPayloadObserver
from learner_core import weight_compensation

class CompensatedStandingController(StandingController):
    def __init__(self,model,limits,qref):
        super().__init__(model,limits,qref)
        self.observer=StaticPayloadObserver()
        self.observation={'estimate_kg':0.,'updated':False,'reason':'initial'}
    def step(self,qpos,qvel,measured_joint_torques):
        self.observation=self.observer.update(qpos,qvel,measured_joint_torques)
        return super().step(qpos,qvel)
    def bias_force(self):
        h=super().bias_force()
        jp=np.zeros((3,18));jr=jp.copy()
        mujoco.mj_jacBodyCom(self.model,self.data,jp,jr,self.model.body('base_link').id)
        # Add estimated payload gravity at the declared attachment point.
        # Do not change nominal mass/inertia or use actual plant payload.
        h+=jp.T@np.array([0.,0.,weight_compensation(self.observer.estimate_kg,9.81)])
        return h
