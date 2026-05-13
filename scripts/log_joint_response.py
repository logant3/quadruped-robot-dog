#!/usr/bin/env python3

import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointResponseLogger(Node):
    def __init__(self, hip_cmd: float, knee_cmd: float, move_time: float, log_time: float):
        super().__init__("log_joint_response")

        self.hip_cmd = hip_cmd
        self.knee_cmd = knee_cmd
        self.move_time = move_time
        self.log_time = log_time

        self.start_time = None
        self.latest_joint_state = None
        self.rows = []

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = Path.home() / "ros2_ws" / f"joint_response_{timestamp}.csv"

        self.get_logger().info(f"Logging to: {self.log_path}")

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = [
            "front_right_hip_joint",
            "front_right_knee_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [self.hip_cmd, self.knee_cmd]
        point.time_from_start.sec = int(self.move_time)
        point.time_from_start.nanosec = int((self.move_time - int(self.move_time)) * 1e9)

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Commanded hip={self.hip_cmd:.4f} rad, knee={self.knee_cmd:.4f} rad over {self.move_time:.2f}s"
        )

    def get_joint_value(self, name: str):
        if self.latest_joint_state is None:
            return None, None, None

        if name not in self.latest_joint_state.name:
            return None, None, None

        idx = self.latest_joint_state.name.index(name)

        pos = self.latest_joint_state.position[idx] if idx < len(self.latest_joint_state.position) else None
        vel = self.latest_joint_state.velocity[idx] if idx < len(self.latest_joint_state.velocity) else None
        eff = self.latest_joint_state.effort[idx] if idx < len(self.latest_joint_state.effort) else None

        return pos, vel, eff

    def run_test(self):
        # Wait for publisher/subscriber connections and first joint state.
        self.get_logger().info("Waiting for joint states...")
        wait_start = time.time()

        while rclpy.ok() and self.latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - wait_start > 5.0:
                self.get_logger().error("No /joint_states received. Are Gazebo and controllers running?")
                return False

        time.sleep(0.5)
        self.send_command()

        self.start_time = time.time()
        end_time = self.start_time + self.log_time

        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)

            now = time.time()
            t = now - self.start_time

            hip_pos, hip_vel, hip_eff = self.get_joint_value("front_right_hip_joint")
            knee_pos, knee_vel, knee_eff = self.get_joint_value("front_right_knee_joint")

            if hip_pos is None or knee_pos is None:
                continue

            hip_err = self.hip_cmd - hip_pos
            knee_err = self.knee_cmd - knee_pos

            self.rows.append({
                "time_s": t,
                "hip_cmd_rad": self.hip_cmd,
                "hip_actual_rad": hip_pos,
                "hip_error_rad": hip_err,
                "hip_actual_deg": math.degrees(hip_pos),
                "hip_error_deg": math.degrees(hip_err),
                "hip_velocity_rad_s": hip_vel if hip_vel is not None else "",
                "hip_effort": hip_eff if hip_eff is not None else "",
                "knee_cmd_rad": self.knee_cmd,
                "knee_actual_rad": knee_pos,
                "knee_error_rad": knee_err,
                "knee_actual_deg": math.degrees(knee_pos),
                "knee_error_deg": math.degrees(knee_err),
                "knee_velocity_rad_s": knee_vel if knee_vel is not None else "",
                "knee_effort": knee_eff if knee_eff is not None else "",
            })

        self.write_csv()

        if self.rows:
            final = self.rows[-1]
            self.get_logger().info(
                f"Final hip actual={final['hip_actual_rad']:.4f} rad, error={final['hip_error_rad']:.4f} rad"
            )
            self.get_logger().info(
                f"Final knee actual={final['knee_actual_rad']:.4f} rad, error={final['knee_error_rad']:.4f} rad"
            )

        return True

    def write_csv(self):
        if not self.rows:
            self.get_logger().warn("No rows logged.")
            return

        with open(self.log_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f"Saved {len(self.rows)} samples to {self.log_path}")


def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run robot_dog_description log_joint_response.py HIP_RAD KNEE_RAD [MOVE_TIME_SEC] [LOG_TIME_SEC]")
        print("Example: ros2 run robot_dog_description log_joint_response.py 0.0 1.5708 2.0 5.0")
        return

    hip_cmd = float(sys.argv[1])
    knee_cmd = float(sys.argv[2])
    move_time = float(sys.argv[3]) if len(sys.argv) >= 4 else 2.0
    log_time = float(sys.argv[4]) if len(sys.argv) >= 5 else move_time + 3.0

    rclpy.init()
    node = JointResponseLogger(hip_cmd, knee_cmd, move_time, log_time)

    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
