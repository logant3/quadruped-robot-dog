#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# These are the known good home-relative IK outputs:
# foot_x = 0, foot_z = -157 mm -> home
# foot_x = 0, foot_z = -140 mm -> compress
# foot_x = 0, foot_z = -220 mm -> extend
POSES = {
    "home":     (0.002, -0.002),
    "compress": (0.041, -0.106),
    "extend":  (-0.172, 0.413),
}


class LoadedLegCycleTest(Node):
    def __init__(self):
        super().__init__("loaded_leg_cycle_test")
        self.pub = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10
        )
        time.sleep(1.0)

    def send_pose(self, name, duration):
        hip, knee = POSES[name]

        msg = JointTrajectory()
        msg.joint_names = [
            "front_right_hip_joint",
            "front_right_knee_joint",
        ]

        pt = JointTrajectoryPoint()
        pt.positions = [hip, knee]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        msg.points.append(pt)
        self.pub.publish(msg)

        self.get_logger().info(
            f"{name}: hip={hip:.3f}, knee={knee:.3f}, time={duration:.2f}s"
        )
        time.sleep(duration + 0.35)


def main():
    rclpy.init()
    node = LoadedLegCycleTest()

    try:
        cycles = 10

        node.send_pose("home", 1.5)

        for i in range(cycles):
            node.get_logger().info(f"Cycle {i + 1}/{cycles}")
            node.send_pose("compress", 1.0)
            node.send_pose("home", 0.8)
            node.send_pose("extend", 1.0)
            node.send_pose("home", 0.8)

        node.get_logger().info("Loaded cycle test complete.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
