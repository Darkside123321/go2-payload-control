"""Run Go2 control using measured sensors; ground truth is used for scoring only."""
import argparse
from contextlib import ExitStack
import csv
import json
from pathlib import Path
import socket
import time

import mujoco
import numpy as np

from base_estimation import ContactInertialEstimator, SENSOR_NOISE, SimSensors
from local_transport import (
    COMMAND_PORT, TELEMETRY_PORT, CommandWatchdog, bind_udp, send, speed_valid,
)
from locomotion_control import LocomotionController, STAND_SECONDS
from standing_control import DT, make_model


def telemetry(t, sensors, sample, qpos, qvel, support, controller, watchdog):
    return {
        "time_s": t, "joint_names": sensors.joint_names,
        "joint_position": sample.joint_position.tolist(),
        "joint_velocity": sample.joint_velocity.tolist(),
        "joint_torque": sample.joint_torque.tolist(),
        "gyro": sample.gyro.tolist(), "specific_force": sample.specific_force.tolist(),
        "pose": qpos[:7].tolist(), "velocity": qvel[:6].tolist(),
        "contacts": sample.contacts.tolist(), "trusted_support": support,
        "mode": controller.mode, "payload_kg": controller.payload,
        "requested_forward_mps": controller.velocity if controller.mode == "RUN" else 0.0,
        "stop_reason": watchdog.stop_reason, "rejected_commands": watchdog.rejected,
    }


def run(args, output):
    with ExitStack() as stack:
        watchdog = None
        speed = args.velocity
        if args.ros:
            command_socket = stack.enter_context(bind_udp(args.command_port))
            telemetry_socket = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            watchdog = CommandWatchdog(command_socket)
            print("Waiting for ROS 2 forward velocity command", flush=True)
            wait_until = time.monotonic() + 15.0
            while watchdog.speed is None and time.monotonic() < wait_until:
                watchdog.poll()
                time.sleep(0.01)
            if watchdog.speed is None:
                return {"completed": False, "failure": "No ROS 2 start command within 15 s"}
            speed = watchdog.speed

        controller = LocomotionController(adaptive=True, velocity=speed)
        stack.callback(controller.worker.shutdown)
        plant, limits = make_model(imu_at_base=True)
        base = plant.body("base_link").id
        added_mass = float(plant.body_mass.sum() * args.load)
        plant.body_mass[base] += added_mass
        data = mujoco.MjData(plant)
        mujoco.mj_setConst(plant, data)
        data.qpos[:] = controller.qref
        mujoco.mj_forward(plant, data)
        sensors = SimSensors(plant, args.seed)
        estimator = stack.enter_context(ContactInertialEstimator(
            controller.model, controller.initial_base
        ))

        rows = []
        failure = None
        late_cycles = 0
        max_telemetry_bytes = 0
        ticks = round(args.seconds / DT)
        start = time.perf_counter()
        for tick in range(ticks):
            t = float(data.time)
            begin = time.perf_counter()
            if watchdog:
                watchdog.poll()
                if watchdog.stop_reason:
                    controller.request_stop()

            sample = sensors.read(data)
            try:
                qpos, qvel, support = estimator.update(sample)
            except ValueError as error:
                failure = str(error)
                break
            estimate_ms = (time.perf_counter() - begin) * 1000
            tau, receipt = controller.step(t, qpos, qvel, sample.joint_torque)
            cycle_ms = (time.perf_counter() - begin) * 1000
            if tau is None:
                failure = receipt["reason"]
                break

            # Evaluation uses the simulator reference at the same timestamp as the estimate.
            quaternion_dot = abs(float(qpos[3:7] @ data.qpos[3:7]))
            tilt_cos = data.xmat[base].reshape(3, 3)[2, 2]
            rows.append({
                "time_s": t, "mode": receipt["mode"], "true_x_m": data.qpos[0],
                "true_height_m": data.qpos[2], "true_forward_mps": data.qvel[0],
                "true_speed_mps": np.linalg.norm(data.qvel[:3]),
                "estimated_x_m": qpos[0], "estimated_height_m": qpos[2],
                "estimated_forward_mps": qvel[0],
                "height_error_mm": (qpos[2] - data.qpos[2]) * 1000,
                "position_error_m": np.linalg.norm(qpos[:3] - data.qpos[:3]),
                "velocity_error_mps": np.linalg.norm(qvel[:3] - data.qvel[:3]),
                "attitude_error_deg": np.degrees(2 * np.arccos(np.clip(quaternion_dot, 0, 1))),
                "true_tilt_deg": np.degrees(np.arccos(np.clip(tilt_cos, -1, 1))),
                "trusted_support": support, "estimated_payload_kg": controller.payload,
                "estimate_ms": estimate_ms, "cycle_ms": cycle_ms,
                "torque_utilization": receipt["joint_torque_utilization"],
            })
            if data.qpos[2] < 0.14 or rows[-1]["true_tilt_deg"] > 30:
                failure = "plant safety threshold exceeded"
                break
            if watchdog and tick % 10 == 0:
                packet_bytes = send(telemetry_socket, args.telemetry_port, telemetry(
                    t, sensors, sample, qpos, qvel, support, controller, watchdog
                ))
                max_telemetry_bytes = max(max_telemetry_bytes, packet_bytes)
            data.ctrl[:] = np.clip(tau, -limits, limits)
            for _ in range(2):
                mujoco.mj_step(plant, data)
            mujoco.mj_forward(plant, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                failure = "non-finite plant state"
                break
            remaining = start + (tick + 1) * DT - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                late_cycles += 1

        if rows:
            with (output / "trace.csv").open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

        walking = [row for row in rows if row["time_s"] >= STAND_SECONDS + 0.5
                   and row["mode"] == "RUN"]
        stopped = [row for row in rows if row["mode"] == "STOPPED"]
        def rms(values):
            return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None

        return {
            "completed": failure is None and len(rows) == ticks, "failure": failure,
            "sim_seconds": float(data.time), "wall_seconds": time.perf_counter() - start,
            "control_hz": 1 / DT, "physics_hz": 1 / plant.opt.timestep,
            "cycle_count": len(rows), "late_cycles": late_cycles,
            "speed_command_mps": speed, "load_fraction": args.load,
            "actual_payload_kg": added_mass, "estimated_payload_kg": controller.payload,
            "distance_m": float(data.qpos[0] - controller.initial_base[0]),
            "walking_samples": len(walking),
            "walking_speed_rmse_mps": rms([row["true_forward_mps"] - speed for row in walking]),
            "walking_height_estimate_rmse_mm": rms([row["height_error_mm"] for row in walking]),
            "walking_position_estimate_rmse_m": rms([row["position_error_m"] for row in walking]),
            "walking_velocity_estimate_rmse_mps": rms([row["velocity_error_mps"] for row in walking]),
            "walking_attitude_estimate_rmse_deg": rms([row["attitude_error_deg"] for row in walking]),
            "peak_tilt_deg": float(max((row["true_tilt_deg"] for row in rows), default=0)),
            "peak_torque_utilization": float(max((row["torque_utilization"] for row in rows), default=0)),
            "estimate_p99_ms": float(np.percentile([row["estimate_ms"] for row in rows], 99)) if rows else None,
            "cycle_p99_ms": float(np.percentile([row["cycle_ms"] for row in rows], 99)) if rows else None,
            "mpc_accepted_updates": controller.mpc_updates,
            "mpc_rejected_updates": controller.mpc_rejected,
            "final_mode": controller.mode,
            "stop_reason": watchdog.stop_reason if watchdog else None,
            "first_stopped_s": stopped[0]["time_s"] if stopped else None,
            "final_speed_mps": float(np.linalg.norm(data.qvel[:3])),
            "rejected_commands": watchdog.rejected if watchdog else None,
            "max_telemetry_datagram_bytes": max_telemetry_bytes if watchdog else None,
            "seed": args.seed, "sensor_noise": SENSOR_NOISE,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--load", type=float, choices=[0.0, 0.1, 0.2], default=0.2)
    parser.add_argument("--velocity", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--command-port", type=int, default=COMMAND_PORT)
    parser.add_argument("--telemetry-port", type=int, default=TELEMETRY_PORT)
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= STAND_SECONDS + 0.5:
        parser.error("seconds must be finite and greater than 4.5")
    if not speed_valid(args.velocity) or args.velocity == 0:
        parser.error("forward speed must be in [0.02, 0.10] m/s")
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "summary.json").exists() or (args.output / "trace.csv").exists():
        parser.error("output already contains a run")
    summary = run(args, args.output)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)
    return 0 if summary["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
