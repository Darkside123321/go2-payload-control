"""Independent ROS 2 DDS client for command and telemetry acceptance tests."""
import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String


class Probe(Node):
    def __init__(self, stop):
        super().__init__("go2_acceptance_probe")
        self.stop = stop
        self.counts = dict(imu=0, joints=0, odom=0, status=0)
        self.status = None
        self.last_motion_command = None
        self.first_stop_wall = None
        self.first_stop_sim = None
        self.first_stop_requested_sim = None
        self.first_stop_requested_wall = None
        self.odom_x = []
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Imu, "/go2/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/go2/joint_states", self.on_joints, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/go2/odom", self.on_odom, 10)
        self.create_subscription(String, "/go2/status", self.on_status, 10)
        self.create_timer(0.10, self.send_command)

    def on_imu(self, msg):
        assert msg.header.frame_id == "imu_link" and msg.orientation_covariance[0] == -1
        assert math.isfinite(msg.linear_acceleration.z)
        self.counts["imu"] += 1

    def on_joints(self, msg):
        assert len(set(msg.name)) == len(msg.position) == len(msg.velocity) == len(msg.effort) == 12
        assert all(math.isfinite(value) for value in (*msg.position, *msg.velocity, *msg.effort))
        self.counts["joints"] += 1

    def on_odom(self, msg):
        assert msg.header.frame_id == "odom" and msg.child_frame_id == "base_link"
        pose = msg.pose.pose
        norm = math.sqrt(sum(component**2 for component in
                             (pose.orientation.w, pose.orientation.x, pose.orientation.y,
                              pose.orientation.z)))
        assert abs(norm - 1.0) < 1e-6
        assert math.isfinite(pose.position.x)
        self.odom_x.append(pose.position.x)
        self.counts["odom"] += 1

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        assert len(self.status["contacts"]) == 4
        assert self.status["mode"] in {"RUN", "STOPPING", "STOPPED"}
        self.counts["status"] += 1
        if self.first_stop_requested_wall is None and self.status["stop_reason"]:
            self.first_stop_requested_wall = time.monotonic()
            self.first_stop_requested_sim = self.status["time_s"]
        if self.first_stop_wall is None and self.status["mode"] == "STOPPED":
            self.first_stop_wall = time.monotonic()
            self.first_stop_sim = self.status["time_s"]

    def matched(self):
        return self.cmd.get_subscription_count() > 0 and all(
            self.count_publishers(topic) > 0
            for topic in ("/go2/imu", "/go2/joint_states", "/go2/odom", "/go2/status")
        )

    def send_command(self):
        if not self.matched():
            return
        unsupported = Twist()
        unsupported.linear.y = 0.1
        self.cmd.publish(unsupported)
        t = self.status["time_s"] if self.status else 0.0
        if t < 6.5:
            msg = Twist()
            msg.linear.x = 0.10
            self.last_motion_command = time.monotonic()
            self.cmd.publish(msg)
        elif self.stop == "zero" and self.first_stop_wall is None:
            self.cmd.publish(Twist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stop", required=True, choices=("timeout", "zero"))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.stop)
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if probe.status and probe.first_stop_sim is not None and probe.status["time_s"] >= 9.0:
                break
        assert probe.status and probe.first_stop_wall and probe.last_motion_command, (
            "no ROS 2 stop status received", probe.matched(), probe.counts, probe.status
        )
        latency = probe.first_stop_wall - probe.last_motion_command
        assert probe.status["stop_reason"] == {
            "timeout": "command timeout", "zero": "zero velocity command"
        }[args.stop], probe.status
        assert probe.status["rejected_twists"] > 0 and probe.status["requested_forward_mps"] == 0
        assert min(probe.counts.values()) >= 100 and max(probe.odom_x) - min(probe.odom_x) >= 0.18
        assert latency <= 1.2 and (args.stop == "zero" or latency >= 0.55), latency
        result = {
            "passed": True, "stop_test": args.stop, "counts": probe.counts,
            "first_stop_requested_sim_s": probe.first_stop_requested_sim,
            "first_stopped_sim_s": probe.first_stop_sim,
            "last_positive_command_to_stopped_wall_s": latency,
            "first_stop_request_to_stopped_sim_s": probe.first_stop_sim - probe.first_stop_requested_sim,
            "odometry_forward_span_m": max(probe.odom_x) - min(probe.odom_x),
            "status": probe.status,
        }
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
