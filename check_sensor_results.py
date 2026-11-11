"""Score stored sensor and ROS 2 acceptance runs against fixed limits."""
import argparse
import json
from pathlib import Path


CASES = ("nominal", "loaded", "ros_timeout", "ros_zero")


def limits_for(case):
    is_ros = case.startswith("ros_")
    return {
        "walking_samples": (400 if is_ros else 1800, None),
        "walking_speed_rmse_mps": (0, 0.05),
        "walking_height_estimate_rmse_mm": (0, 5.0),
        "walking_position_estimate_rmse_m": (0, 0.10),
        "walking_velocity_estimate_rmse_mps": (0, 0.04),
        "walking_attitude_estimate_rmse_deg": (0, 3.0),
        "peak_tilt_deg": (0, 10.0),
        "peak_torque_utilization": (0, 1.001),
        "estimate_p99_ms": (0, 2.0),
        "mpc_accepted_updates": (100, None),
        "distance_m": (0.18 if is_ros else 0.65, None),
    }


def score(case, result, dds=None):
    failures = []
    for metric, (low, high) in limits_for(case).items():
        value = result.get(metric)
        if value is None or not (low <= value and (high is None or value <= high)):
            failures.append(metric)
    if not result["completed"]:
        failures.append("incomplete simulation")
    if abs(result["estimated_payload_kg"] - result["actual_payload_kg"]) > 0.15:
        failures.append("payload estimate")
    if dds is not None:
        expected_reason = "command timeout" if case == "ros_timeout" else "zero velocity command"
        if not (dds["passed"] and result["stop_reason"] == expected_reason
                and result["final_mode"] == "STOPPED" and result["final_speed_mps"] <= 0.02
                and dds["first_stop_request_to_stopped_sim_s"] <= 0.52):
            failures.append("DDS or controlled stop")
    elif result["final_mode"] != "RUN":
        failures.append("unrequested stop")
    return {"passed": not failures, "failures": failures, "result": result, "dds": dds}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    cases = {}
    for case in CASES:
        folder = args.root / case
        result = json.loads((folder / "summary.json").read_text())
        dds = json.loads((folder / "dds.json").read_text()) if case.startswith("ros_") else None
        cases[case] = score(case, result, dds)
    combined = {
        "passed": all(item["passed"] for item in cases.values()),
        "limits": {case: limits_for(case) for case in CASES},
        "extra_limits": {
            "absolute_payload_error_kg": 0.15,
            "final_ros_speed_mps": 0.02,
            "stop_transition_sim_s": 0.52,
            "dds_minimum_messages_per_topic": 100,
            "dds_last_positive_to_stop_max_wall_s": 1.2,
        },
        "cases": cases,
    }
    if args.output:
        args.output.write_text(json.dumps(combined, indent=2) + "\n")
    for case, entry in cases.items():
        print(case, "PASS" if entry["passed"] else "FAIL", *entry["failures"])
    return 0 if combined["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
