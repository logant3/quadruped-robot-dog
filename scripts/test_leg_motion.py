#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LegMotionTester(Node):
    def __init__(self):
        super().__init__("test_leg_motion")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10
        )

        # Give publisher time to connect
        time.sleep(1.0)

    def send_pose(self, hip_rad, knee_rad, move_time):
        msg = JointTrajectory()
        msg.joint_names = [
            "front_right_hip_joint",
            "front_right_knee_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [hip_rad, knee_rad]
        point.time_from_start.sec = int(move_time)
        point.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Commanded hip={hip_rad:.3f} rad, knee={knee_rad:.3f} rad over {move_time:.1f}s"
        )

        time.sleep(move_time + 0.75)


def main():
    rclpy.init()
    node = LegMotionTester()

    try:
        # Known convention:
        # knee = 1.5708 rad is approximately straight leg
        # knee = 1.0 rad is a bent pose

        sequence = [
            (0.0, 1.5708, 2.0),   # straight
            (0.0, 1.0, 2.0),      # bend knee
            (0.0, 1.5708, 2.0),   # back straight
            (0.25, 1.5708, 2.0),  # small hip forward/back test
            (-0.25, 1.5708, 2.0),
            (0.0, 1.5708, 2.0),   # neutral straight
        ]

        for hip, knee, duration in sequence:
            node.send_pose(hip, knee, duration)

        node.get_logger().info("Safe leg motion test complete.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
