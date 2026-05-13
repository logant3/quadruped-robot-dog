#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


L1_MM = 187.5
L2_MM = 172.0

OLD_HOME_HIP_RAD = 1.03
OLD_HOME_KNEE_RAD = -0.669
OLD_KNEE_STRAIGHT_RAD = 1.5708

HIP_MIN_RAD = -4.17159
HIP_MAX_RAD = 2.11159
KNEE_MIN_RAD = -0.131
KNEE_MAX_RAD = 4.569


class GroundBounceIK(Node):
    def __init__(self):
        super().__init__("ground_bounce_ik")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10,
        )

        time.sleep(1.0)

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

    def send_foot_target(self, x_mm, z_mm, move_time):
        hip_cmd, knee_cmd = self.solve_ik(x_mm, z_mm)

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
            f"foot_x={x_mm:.1f} mm, foot_z={z_mm:.1f} mm "
            f"-> hip={hip_cmd:.3f}, knee={knee_cmd:.3f}, time={move_time:.2f}s"
        )

        time.sleep(move_time + 0.4)


def main():
    rclpy.init()
    node = GroundBounceIK()

    try:
        # foot_x = 0 keeps foot under hip.
        # foot_z is vertical position relative to hip.
        sequence = [
            (0.0, -157.0, 1.5),  # home
            (0.0, -140.0, 1.0),  # compress near safe knee limit
            (0.0, -220.0, 0.5),  # extend/push
            (0.0, -157.0, 1.2),  # return home
        ]

        for x_mm, z_mm, move_time in sequence:
            node.send_foot_target(x_mm, z_mm, move_time)

        node.get_logger().info("Ground bounce IK test complete.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
