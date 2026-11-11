"""ROS 2 Humble bridge for the local Go2 sensor-control simulation."""
import argparse
import json
import socket
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from local_transport import COMMAND_PORT, TELEMETRY_PORT, bind_udp, packets, send, speed_valid


def xyz(target, values):
    target.x, target.y, target.z = values


def world_to_body(vector, quaternion):
    # Apply q* v q with the scalar-first body-to-world quaternion q.
    w, x, y, z = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2 * (z * vy - y * vz), 2 * (x * vz - z * vx), 2 * (y * vx - x * vy)
    return (
        vx + w * tx + z * ty - y * tz,
        vy + w * ty + x * tz - z * tx,
        vz + w * tz + y * tx - x * ty,
    )


class Go2Bridge(Node):
    def __init__(self, command_port, telemetry_socket, command_socket):
        super().__init__("go2_sim_bridge")
        self.command_port = command_port
        self.telemetry_socket = telemetry_socket
        self.command_socket = command_socket
        self.rejected_twists = 0
        self.imu = self.create_publisher(Imu, "/go2/imu", qos_profile_sensor_data)
        self.joints = self.create_publisher(JointState, "/go2/joint_states", qos_profile_sensor_data)
        self.odom = self.create_publisher(Odometry, "/go2/odom", 10)
        self.status = self.create_publisher(String, "/go2/status", 10)
        self.create_subscription(Twist, "/cmd_vel", self.command, 10)
        self.create_timer(0.02, self.publish_telemetry)

    def command(self, message):
        unsupported = (message.linear.y, message.linear.z, message.angular.x,
                       message.angular.y, message.angular.z)
        if any(axis != 0.0 for axis in unsupported) or not speed_valid(message.linear.x):
            self.rejected_twists += 1
            return
        send(self.command_socket, self.command_port,
             {"forward_mps": message.linear.x, "monotonic_s": time.monotonic()})

    def publish_telemetry(self):
        sample = None
        for sample in packets(self.telemetry_socket):
            pass
        if sample is None or "time_s" not in sample:
            return
        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0
        xyz(imu.angular_velocity, sample["gyro"])
        xyz(imu.linear_acceleration, sample["specific_force"])

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = sample["joint_names"]
        joints.position = sample["joint_position"]
        joints.velocity = sample["joint_velocity"]
        joints.effort = sample["joint_torque"]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        pose = odom.pose.pose
        xyz(pose.position, sample["pose"][:3])
        quat = sample["pose"][3:]
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = quat
        xyz(odom.twist.twist.linear, world_to_body(sample["velocity"][:3], quat))
        xyz(odom.twist.twist.angular, sample["velocity"][3:])

        status = {key: sample[key] for key in (
            "time_s", "contacts", "trusted_support", "mode", "payload_kg",
            "requested_forward_mps", "stop_reason", "rejected_commands",
        )}
        status["rejected_twists"] = self.rejected_twists
        self.imu.publish(imu)
        self.joints.publish(joints)
        self.odom.publish(odom)
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-port", type=int, default=COMMAND_PORT)
    parser.add_argument("--telemetry-port", type=int, default=TELEMETRY_PORT)
    args = parser.parse_args()
    rclpy.init()
    with bind_udp(args.telemetry_port) as telemetry_socket, socket.socket(
        socket.AF_INET, socket.SOCK_DGRAM
    ) as command_socket:
        node = Go2Bridge(args.command_port, telemetry_socket, command_socket)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
