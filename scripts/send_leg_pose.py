#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LegPoseSender(Node):
    def __init__(self, hip_rad: float, knee_rad: float, move_time: float):
        super().__init__("send_leg_pose")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/front_right_leg_controller/joint_trajectory",
            10
        )

        self.hip_rad = hip_rad
        self.knee_rad = knee_rad
        self.move_time = move_time

        self.timer = self.create_timer(0.5, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return

        msg = JointTrajectory()
        msg.joint_names = [
            "front_right_hip_joint",
            "front_right_knee_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [self.hip_rad, self.knee_rad]
        point.time_from_start.sec = int(self.move_time)
        point.time_from_start.nanosec = int((self.move_time - int(self.move_time)) * 1e9)

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Sent pose: hip={self.hip_rad:.4f} rad, knee={self.knee_rad:.4f} rad, time={self.move_time:.2f}s"
        )

        self.sent = True
        rclpy.shutdown()


def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run robot_dog_description send_leg_pose.py HIP_RAD KNEE_RAD [MOVE_TIME_SEC]")
        print("Example: ros2 run robot_dog_description send_leg_pose.py 0.0 1.5708 2.0")
        return

    hip_rad = float(sys.argv[1])
    knee_rad = float(sys.argv[2])
    move_time = float(sys.argv[3]) if len(sys.argv) >= 4 else 2.0

    rclpy.init()
    node = LegPoseSender(hip_rad, knee_rad, move_time)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
    
