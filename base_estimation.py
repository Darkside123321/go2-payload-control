"""Sensor extraction and C++ contact-inertial estimator for the flat simulation."""
import ctypes
from dataclasses import dataclass

import mujoco
import numpy as np

from cpp_core import Arr, Iarr, LIB
from standing_control import DT, FEET

LIB.base_estimator_create.argtypes = [Arr, Arr]
LIB.base_estimator_create.restype = ctypes.c_void_p
LIB.base_estimator_destroy.argtypes = [ctypes.c_void_p]
LIB.base_estimator_destroy.restype = None
LIB.base_estimator_step.argtypes = [
    ctypes.c_void_p, ctypes.c_double, Arr, Arr, Arr, Arr, Iarr, Arr, Arr,
    ctypes.POINTER(ctypes.c_int),
]
LIB.base_estimator_step.restype = ctypes.c_int

SENSOR_NOISE = {
    "gyro_std_rad_s": 0.0005,
    "accel_std_m_s2": 0.015,
    "joint_position_std_rad": 0.00005,
    "joint_velocity_std_rad_s": 0.0005,
    "gyro_bias_rad_s": (0.0005, -0.0004, 0.0003),
    "accel_bias_m_s2": (0.02, -0.015, 0.03),
}


@dataclass(frozen=True)
class Measurements:
    gyro: np.ndarray
    specific_force: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    joint_torque: np.ndarray
    contacts: np.ndarray


class SimSensors:
    def __init__(self, model, seed):
        self.model = model
        self.rng = np.random.default_rng(seed)
        self.floor = model.geom("floor").id
        self.foot_geoms = {model.geom(foot + "_foot_collision").id: i
                           for i, foot in enumerate(FEET)}
        self.gyro_start = model.sensor("imu_ang_vel").adr.item()
        self.accel_start = model.sensor("imu_lin_acc").adr.item()
        self.joint_names = [model.joint(j).name for j in model.actuator_trnid[:, 0]]

    def read(self, data):
        normal_force = np.zeros(4)
        wrench = np.zeros(6)
        for index, contact in enumerate(data.contact):
            first, second = contact.geom
            if self.floor not in (first, second):
                continue
            foot = self.foot_geoms.get(second if first == self.floor else first)
            if foot is not None:
                mujoco.mj_contactForce(self.model, data, index, wrench)
                normal_force[foot] += wrench[0]
        noise = SENSOR_NOISE
        return Measurements(
            data.sensordata[self.gyro_start:self.gyro_start + 3]
            + noise["gyro_bias_rad_s"] + self.rng.normal(0, noise["gyro_std_rad_s"], 3),
            data.sensordata[self.accel_start:self.accel_start + 3]
            + noise["accel_bias_m_s2"] + self.rng.normal(0, noise["accel_std_m_s2"], 3),
            data.qpos[7:] + self.rng.normal(0, noise["joint_position_std_rad"], 12),
            data.qvel[6:] + self.rng.normal(0, noise["joint_velocity_std_rad_s"], 12),
            data.qfrc_actuator[6:].copy(),
            (normal_force >= 8.0).astype(np.int32),
        )


class ContactInertialEstimator:
    def __init__(self, nominal_model, initial_position):
        self.model = nominal_model
        self.data = mujoco.MjData(nominal_model)
        self.data.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        self.sites = [nominal_model.site(foot).id for foot in FEET]
        ground_heights = np.array([
            nominal_model.geom(foot + "_foot_collision").size[0] for foot in FEET
        ])
        self.handle = LIB.base_estimator_create(initial_position, ground_heights)
        if self.handle is None:
            raise ValueError("invalid initial estimator position or ground heights")
        self.jacobian = np.zeros((3, nominal_model.nv))
        self.foot_position = np.zeros((4, 3))
        self.foot_velocity = np.zeros((4, 3))

    def __enter__(self):
        return self

    def __exit__(self, *_):
        LIB.base_estimator_destroy(self.handle)
        self.handle = None

    def update(self, sample):
        self.data.qpos[7:] = sample.joint_position
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        for i, site in enumerate(self.sites):
            self.foot_position[i] = self.data.site_xpos[site]
            mujoco.mj_jacSite(self.model, self.data, self.jacobian, None, site)
            self.foot_velocity[i] = self.jacobian[:, 6:] @ sample.joint_velocity
        qpos = np.empty(19)
        qvel = np.empty(18)
        support = ctypes.c_int()
        status = LIB.base_estimator_step(
            self.handle, DT, sample.gyro, sample.specific_force, self.foot_position,
            self.foot_velocity, sample.contacts, qpos[:7], qvel[:6], ctypes.byref(support),
        )
        if status != 0:
            raise ValueError("invalid inertial or leg kinematics sample")
        qpos[7:] = sample.joint_position
        qvel[6:] = sample.joint_velocity
        return qpos, qvel, support.value
