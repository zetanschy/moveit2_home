#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class ArmTester(Node):
    def __init__(self):
        """Initialize the arm tester."""
        super().__init__("yam_arm_tester")
        self.controller_name = "arm_controller"
        self.topic = f"/{self.controller_name}/joint_trajectory"
        
        self.publisher = self.create_publisher(
            JointTrajectory, self.topic, 10
        )

        # Joint names for YAM 6-DOF arm
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        self.get_logger().info("YAM Arm Tester initialized")
        # Wait for the publisher to be ready
        time.sleep(1)

    def send_trajectory(self, positions, duration_sec=3.0):
        """Send a joint trajectory command."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9)
        )

        msg.points = [point]

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent trajectory: {positions}")

    def test_sequence(self):
        """Run a test sequence of movements."""
        self.get_logger().info("Starting YAM arm test sequence...")

        # Home position
        self.get_logger().info("Moving to home position...")
        self.send_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
        time.sleep(4)

        # Test position 1
        self.get_logger().info("Moving to test position 1...")
        self.send_trajectory([0.5, -0.5, 0.3, -0.7, 0.2, 0.4], 3.0)
        time.sleep(4)

        # Test position 2
        self.get_logger().info("Moving to test position 2...")
        self.send_trajectory([-0.3, 0.7, -0.4, 0.5, -0.2, 0.6], 3.0)
        time.sleep(4)

        # Back to home
        self.get_logger().info("Returning to home...")
        self.send_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
        time.sleep(4)

        self.get_logger().info("YAM arm test sequence complete!")


def main(args=None):
    rclpy.init(args=args)

    tester = ArmTester()

    try:
        # Run the test sequence
        tester.test_sequence()

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
