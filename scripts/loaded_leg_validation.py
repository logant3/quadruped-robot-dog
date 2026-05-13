#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Geometry in mm
L1_MM = 187.5
L2_MM = 172.0

# Old absolute home pose before home was baked into URDF zero
OLD_HOME_HIP_RAD = 1.03
OLD_HOME_KNEE_RAD = -0.669
OLD_KNEE_STRAIGHT_RAD = 1.5708

# Home-relative limits
HIP_MIN_RAD = -4.17159
HIP_MAX_RAD = 2.11159
KNEE_MIN_RAD = -0.131
KNEE_MAX_RAD = 4.569

# Actuator/belt limits
HIP_PEAK_TORQUE_NM = 17.0
KNEE_PEAK_TORQUE_NM = 25.5


class LoadedLegValidation(Node):
    def __init__(self):
        super().__init__("loaded_leg_validation")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10,
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.latest_joint_state = None
        self.current_target = None
        self.rows = []

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = Path.home() / "ros2_ws" / f"loaded_leg_validation_{timestamp}.csv"

        self.get_logger().info(f"Logging to: {self.log_path}")

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def get_joint_value(self, name):
        if self.latest_joint_state is None:
            return None, None, None

        if name not in self.latest_joint_state.name:
            return None, None, None

        idx = self.latest_joint_state.name.index(name)

        pos = self.latest_joint_state.position[idx] if idx < len(self.latest_joint_state.position) else None
        vel = self.latest_joint_state.velocity[idx] if idx < len(self.latest_joint_state.velocity) else None
        eff = self.latest_joint_state.effort[idx] if idx < len(self.latest_joint_state.effort) else None

        return pos, vel, eff

    def solve_ik(self, x_mm, z_mm):
        x = x_mm
        y = -z_mm  # positive downward

        d = math.sqrt(x * x + y * y)
        max_reach = L1_MM + L2_MM
        min_reach = abs(L1_MM - L2_MM)

        if d > max_reach:
            raise ValueError(f"Target too far: {d:.1f} mm > {max_reach:.1f} mm")
        if d < min_reach:
            raise ValueError(f"Target too close: {d:.1f} mm < {min_reach:.1f} mm")

        cos_delta = (d * d - L1_MM * L1_MM - L2_MM * L2_MM) / (2.0 * L1_MM * L2_MM)
        cos_delta = max(-1.0, min(1.0, cos_delta))

        delta = -math.acos(cos_delta)

        a = L1_MM + L2_MM * math.cos(delta)
        b = L2_MM * math.sin(delta)

        gamma = math.atan2(b, a)
        hip_abs = math.atan2(x, y) - gamma
        knee_abs = OLD_KNEE_STRAIGHT_RAD + delta

        hip_cmd = hip_abs - OLD_HOME_HIP_RAD
        knee_cmd = knee_abs - OLD_HOME_KNEE_RAD

        if not (HIP_MIN_RAD <= hip_cmd <= HIP_MAX_RAD):
            raise ValueError(f"Hip command {hip_cmd:.4f} outside safe range.")
        if not (KNEE_MIN_RAD <= knee_cmd <= KNEE_MAX_RAD):
            raise ValueError(f"Knee command {knee_cmd:.4f} outside safe range.")

        return hip_cmd, knee_cmd

    def send_foot_target(self, label, x_mm, z_mm, move_time):
        hip_cmd, knee_cmd = self.solve_ik(x_mm, z_mm)
        self.current_target = {
            "label": label,
            "foot_x_mm": x_mm,
            "foot_z_mm": z_mm,
            "hip_cmd_rad": hip_cmd,
            "knee_cmd_rad": knee_cmd,
        }

        msg = JointTrajectory()
        msg.joint_names = [
            "front_right_hip_joint",
            "front_right_knee_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [hip_cmd, knee_cmd]
        point.time_from_start.sec = int(move_time)
        point.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(
            f"{label}: foot_x={x_mm:.1f} mm, foot_z={z_mm:.1f} mm "
            f"-> hip={hip_cmd:.3f}, knee={knee_cmd:.3f}, time={move_time:.2f}s"
        )

    def log_for(self, duration_s):
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration_s:
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.latest_joint_state is None or self.current_target is None:
                continue

            hip_pos, hip_vel, hip_eff = self.get_joint_value("front_right_hip_joint")
            knee_pos, knee_vel, knee_eff = self.get_joint_value("front_right_knee_joint")
            rail_pos, rail_vel, rail_eff = self.get_joint_value("vertical_rail_joint")

            if hip_pos is None or knee_pos is None:
                continue

            hip_eff_val = hip_eff if hip_eff is not None else 0.0
            knee_eff_val = knee_eff if knee_eff is not None else 0.0

            self.rows.append({
                "time_s": time.time(),
                "phase": self.current_target["label"],
                "foot_x_cmd_mm": self.current_target["foot_x_mm"],
                "foot_z_cmd_mm": self.current_target["foot_z_mm"],
                "hip_cmd_rad": self.current_target["hip_cmd_rad"],
                "hip_actual_rad": hip_pos,
                "hip_error_rad": self.current_target["hip_cmd_rad"] - hip_pos,
                "hip_velocity_rad_s": hip_vel if hip_vel is not None else "",
                "hip_effort_nm": hip_eff if hip_eff is not None else "",
                "hip_effort_pct_peak": abs(hip_eff_val) / HIP_PEAK_TORQUE_NM * 100.0,
                "knee_cmd_rad": self.current_target["knee_cmd_rad"],
                "knee_actual_rad": knee_pos,
                "knee_error_rad": self.current_target["knee_cmd_rad"] - knee_pos,
                "knee_velocity_rad_s": knee_vel if knee_vel is not None else "",
                "knee_effort_nm": knee_eff if knee_eff is not None else "",
                "knee_effort_pct_peak": abs(knee_eff_val) / KNEE_PEAK_TORQUE_NM * 100.0,
                "rail_pos_m": rail_pos if rail_pos is not None else "",
                "rail_velocity_m_s": rail_vel if rail_vel is not None else "",
                "rail_effort": rail_eff if rail_eff is not None else "",
            })

    def run_test(self):
        self.get_logger().info("Waiting for /joint_states...")
        wait_start = time.time()
        while rclpy.ok() and self.latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - wait_start > 5.0:
                self.get_logger().error("No /joint_states received. Are Gazebo and controllers running?")
                return

        # Loaded validation sequence.
        # x=0 keeps foot under hip.
        sequence = [
            ("home_hold", 0.0, -157.0, 2.0, 2.0),
            ("compress", 0.0, -140.0, 2.0, 2.0),
            ("home_return", 0.0, -157.0, 2.0, 2.0),
            ("extend_push", 0.0, -220.0, 1.5, 2.0),
            ("home_final", 0.0, -157.0, 2.0, 2.0),
        ]

        for label, x_mm, z_mm, move_time, settle_time in sequence:
            self.send_foot_target(label, x_mm, z_mm, move_time)
            self.log_for(move_time + settle_time)

        self.write_csv()
        self.print_summary()

    def write_csv(self):
        if not self.rows:
            self.get_logger().warn("No rows logged.")
            return

        with open(self.log_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f"Saved {len(self.rows)} samples to {self.log_path}")

    def print_summary(self):
        if not self.rows:
            return

        hip_efforts = [
            abs(r["hip_effort_nm"]) for r in self.rows
            if isinstance(r["hip_effort_nm"], float)
        ]
        knee_efforts = [
            abs(r["knee_effort_nm"]) for r in self.rows
            if isinstance(r["knee_effort_nm"], float)
        ]
        rail_positions = [
            r["rail_pos_m"] for r in self.rows
            if isinstance(r["rail_pos_m"], float)
        ]

        max_hip = max(hip_efforts) if hip_efforts else 0.0
        max_knee = max(knee_efforts) if knee_efforts else 0.0

        self.get_logger().info("===== Loaded leg validation summary =====")
        self.get_logger().info(f"Peak hip effort:  {max_hip:.3f} N·m ({max_hip / HIP_PEAK_TORQUE_NM * 100.0:.1f}% of 17 N·m)")
        self.get_logger().info(f"Peak knee effort: {max_knee:.3f} N·m ({max_knee / KNEE_PEAK_TORQUE_NM * 100.0:.1f}% of 25.5 N·m)")

        if rail_positions:
            self.get_logger().info(f"Rail min height: {min(rail_positions):.4f} m")
            self.get_logger().info(f"Rail max height: {max(rail_positions):.4f} m")
            self.get_logger().info(f"Rail travel:     {max(rail_positions) - min(rail_positions):.4f} m")


def main():
    rclpy.init()
    node = LoadedLegValidation()

    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
