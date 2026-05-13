#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Geometry in mm
L1_MM = 187.5   # hip center to knee center
L2_MM = 172.0   # knee center to bottom of foot

# Old absolute home pose before we baked home into URDF zero
OLD_HOME_HIP_RAD = 1.03
OLD_HOME_KNEE_RAD = -0.669

# Old knee convention
OLD_KNEE_STRAIGHT_RAD = 1.5708

# New home-relative software limits after home was baked into zero
HIP_MIN_RAD = -4.17159
HIP_MAX_RAD = 2.11159
KNEE_MIN_RAD = -0.131
KNEE_MAX_RAD = 4.4569


class IKFootPose(Node):
    def __init__(self, foot_x_mm: float, foot_z_mm: float, move_time: float):
        super().__init__("ik_foot_pose")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10
        )

        self.foot_x_mm = foot_x_mm
        self.foot_z_mm = foot_z_mm
        self.move_time = move_time

        time.sleep(1.0)

    def solve_ik(self, x_mm: float, z_mm: float):
        """
        x_mm: horizontal foot position relative to hip.
              x = 0 means foot directly under hip.

        z_mm: vertical foot position relative to hip.
              Negative means below hip.
              Example: z = -157 is current home height.
        """

        x = x_mm
        y = -z_mm  # positive downward

        d = math.sqrt(x * x + y * y)

        max_reach = L1_MM + L2_MM
        min_reach = abs(L1_MM - L2_MM)

        if d > max_reach:
            raise ValueError(
                f"Target too far: distance={d:.1f} mm, max={max_reach:.1f} mm"
            )

        if d < min_reach:
            raise ValueError(
                f"Target too close: distance={d:.1f} mm, min={min_reach:.1f} mm"
            )

        # Link angle difference.
        # delta = 0 means straight.
        # For our folding direction, delta is negative.
        cos_delta = (d * d - L1_MM * L1_MM - L2_MM * L2_MM) / (2.0 * L1_MM * L2_MM)
        cos_delta = max(-1.0, min(1.0, cos_delta))

        delta = -math.acos(cos_delta)

        # Solve upper-leg angle.
        a = L1_MM + L2_MM * math.cos(delta)
        b = L2_MM * math.sin(delta)

        gamma = math.atan2(b, a)
        hip_abs = math.atan2(x, y) - gamma

        # Convert back to old absolute knee convention.
        knee_abs = OLD_KNEE_STRAIGHT_RAD + delta

        # Convert old absolute pose to new home-relative command.
        hip_cmd = hip_abs - OLD_HOME_HIP_RAD
        knee_cmd = knee_abs - OLD_HOME_KNEE_RAD

        return hip_cmd, knee_cmd, hip_abs, knee_abs

    def send_pose(self):
        hip_cmd, knee_cmd, hip_abs, knee_abs = self.solve_ik(
            self.foot_x_mm,
            self.foot_z_mm
        )

        self.get_logger().info(
            f"IK target: foot_x={self.foot_x_mm:.1f} mm, foot_z={self.foot_z_mm:.1f} mm"
        )
        self.get_logger().info(
            f"Old absolute solution: hip={hip_abs:.4f} rad, knee={knee_abs:.4f} rad"
        )
        self.get_logger().info(
            f"New home-relative command: hip={hip_cmd:.4f} rad, knee={knee_cmd:.4f} rad"
        )

        if not (HIP_MIN_RAD <= hip_cmd <= HIP_MAX_RAD):
            raise ValueError(
                f"Hip command {hip_cmd:.4f} rad outside safe range [{HIP_MIN_RAD}, {HIP_MAX_RAD}]"
            )

        if not (KNEE_MIN_RAD <= knee_cmd <= KNEE_MAX_RAD):
            raise ValueError(
                f"Knee command {knee_cmd:.4f} rad outside safe range [{KNEE_MIN_RAD}, {KNEE_MAX_RAD}]"
            )

        msg = JointTrajectory()
        msg.joint_names = [
            "front_right_hip_joint",
            "front_right_knee_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [hip_cmd, knee_cmd]
        point.time_from_start.sec = int(self.move_time)
        point.time_from_start.nanosec = int((self.move_time - int(self.move_time)) * 1e9)

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Sent IK pose over {self.move_time:.2f} seconds."
        )


def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run robot_dog_description ik_foot_pose.py FOOT_X_MM FOOT_Z_MM [MOVE_TIME_SEC]")
        print("Example home: ros2 run robot_dog_description ik_foot_pose.py 0 -157 2.0")
        print("Example compress: ros2 run robot_dog_description ik_foot_pose.py 0 -140 2.0")
        print("Example extend: ros2 run robot_dog_description ik_foot_pose.py 0 -210 1.0")
        return

    foot_x_mm = float(sys.argv[1])
    foot_z_mm = float(sys.argv[2])
    move_time = float(sys.argv[3]) if len(sys.argv) >= 4 else 2.0

    rclpy.init()
    node = IKFootPose(foot_x_mm, foot_z_mm, move_time)

    try:
        node.send_pose()
        time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
