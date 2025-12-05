#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition
from sensor_msgs.msg import JointState


class RobotControlWithFK(Node):
    def __init__(self):
        super().__init__('robot_control_with_fk')

        # === DH parameters in mm (same as your C++ code) ===
        self.L1 = 96.326   # Base to joint2 (z-offset)
        self.L2 = math.sqrt(128.0*128.0 + 24.0*24.0) # Joint2 to joint3
        self.L3 = 124.0  # Joint3 to joint4
        self.L4 = 133.4  # Joint4 to end-effector

        # Current joint angles (rad)
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        self.have_joint_state = False

        # --- Create client for joint-space service ---
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')

        # --- Subscribe to /joint_states to get current joint angles ---
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info('Robot Control with FK Node Started')
        self.print_dh_parameters()

    # ------------------------------------------------------------------
    # Utility print
    # ------------------------------------------------------------------
    def print_dh_parameters(self):
        print("\n=== DH Parameters ===")
        print(f"L1 (base height): {self.L1} mm")
        print(f"L2 (link 1):      {self.L2} mm")
        print(f"L3 (link 2):      {self.L3} mm")
        print(f"L4 (link 3):      {self.L4} mm")
        print("=====================\n")

    # ------------------------------------------------------------------
    # Forward kinematics (Python version of your C++ calculateFK)
    # ------------------------------------------------------------------
    def calculate_fk(self, t1, t2, t3, t4):
        # Planar cumulative angle for joints 2, 3, 4
        theta234 = t2 + t3 + t4

        # r = radial distance in the plane of the arm
        r = (
            self.L2 * math.cos(t2) +
            self.L3 * math.cos(t2 + t3) +
            self.L4 * math.cos(theta234)
        )
        # z = height
        z = (
            self.L1 +
            self.L2 * math.sin(t2) +
            self.L3 * math.sin(t2 + t3) +
            self.L4 * math.sin(theta234)
        )

        # Rotate about base joint (theta1) to get 3D (x, y)
        x = r * math.cos(t1)
        y = r * math.sin(t1)

        return x, y, z

    # ------------------------------------------------------------------
    # /joint_states callback: update joint angles & print FK
    # ------------------------------------------------------------------
    def joint_callback(self, msg: JointState):
        if len(msg.position) < 4:
            self.get_logger().warn('Not enough joint angles received in /joint_states')
            return

        self.theta1 = msg.position[0]
        self.theta2 = msg.position[1]
        self.theta3 = msg.position[2]
        self.theta4 = msg.position[3]
        self.have_joint_state = True

        x, y, z = self.calculate_fk(self.theta1, self.theta2, self.theta3, self.theta4)

        print("===========================================")
        print("Joint Angles (rad):")
        print(f"  theta1 = {self.theta1:.4f}")
        print(f"  theta2 = {self.theta2:.4f}")
        print(f"  theta3 = {self.theta3:.4f}")
        print(f"  theta4 = {self.theta4:.4f}")
        print("-------------------------------------------")
        print("End Effector Position (mm):")
        print(f"  x = {x:.3f}")
        print(f"  y = {y:.3f}")
        print(f"  z = {z:.3f}")
        print("===========================================\n")

    # ------------------------------------------------------------------
    # Send a single joint command (Python version of sendJointCommand)
    # ------------------------------------------------------------------
    def send_joint_command(self, positions, path_time: float = 5.0) -> bool:
        """
        positions: [j1, j2, j3, j4, gripper] in radians (gripper in whatever units your stack expects)
        path_time: duration for the motion in seconds
        """
        if len(positions) != 5:
            raise ValueError(
                "positions must be a list/tuple of length 5: "
                "[joint1, joint2, joint3, joint4, gripper]"
            )

        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = list(positions)
        request.path_time = float(path_time)

        self.get_logger().info('Sending joint command...')
        future = self.client.call_async(request)

        # Block until service responds
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Joint command sent successfully')

            # Expected FK for the *commanded* configuration
            x, y, z = self.calculate_fk(
                positions[0], positions[1], positions[2], positions[3]
            )
            print("\nExpected End-Effector Position (commanded FK, mm):")
            print(f"  x = {x:.3f}")
            print(f"  y = {y:.3f}")
            print(f"  z = {z:.3f}\n")
            return True
        else:
            self.get_logger().error(f'Service call failed: {future.exception()!r}')
            return False

    # ------------------------------------------------------------------
    # Send multiple joint commands in sequence (like C++ sendMultipleCommands)
    # ------------------------------------------------------------------
    def send_multiple_commands(self, command_sequence, path_time: float = 5.0):
        for i, cmd in enumerate(command_sequence, start=1):
            self.get_logger().info(
                f'Executing command {i} of {len(command_sequence)}'
            )
            ok = self.send_joint_command(cmd, path_time)
            if not ok:
                break
            # simple wait so the motion can complete
            time.sleep(path_time)


def main(args=None):
    rclpy.init(args=args)

    node = RobotControlWithFK()

    try:
        # Example 1: send a single command (like your C++ example)
        # target_positions = [1.0, 0.0, 0.0, 0.0, 0.0]  # rad, rad, rad, rad, gripper
        # node.send_joint_command(target_positions, path_time=5.0)

        # Example 2: multiple commands (uncomment if you want)
        commands = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.01, -0.04, 0.0, 0.0],
            [0.025, 0.5, 0.035, 0.045, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        node.send_multiple_commands(commands, path_time=5.0)

        # Spin to keep receiving /joint_states and printing FK
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()