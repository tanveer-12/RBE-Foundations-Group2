import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import math
import time

Position = [100, 100, 100]
speed = 1.0  # Robot Path travel time (Sec)


class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')

        # TWO SEPARATE CLIENTS
        self.joint_client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.gripper_client = self.create_client(SetJointPosition, 'goal_tool_control')

        # Wait for services
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for joint movement service...")

        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper service...")

        self.get_logger().info("All services ready.")

        # Test: move and close gripper
        self.base()
        time.sleep(speed + 0.1)
        self.go_down()
        time.sleep(speed + 0.1)
        self.open_gripper()
        time.sleep(speed + 0.1)
        self.close_gripper()
        time.sleep(speed + 0.1)
        self.base()


    # -------------------------------
    # Arm motion (4 joints)
    # -------------------------------
    def base(self):
        req = SetJointPosition.Request()
        req.planning_group = ''
        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        req.joint_position.position = [0.0, 0.0, 0.0, 0.0]
        req.path_time = speed

        self.future = self.joint_client.call_async(req)

    def go_down(self):
        req = SetJointPosition.Request()
        req.planning_group = ''
        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        req.joint_position.position = [0.0, 0.0, 0.0, 90.0]
        req.path_time = speed

        self.future = self.joint_client.call_async(req)


    # -------------------------------
    # GRIPPER CONTROL (current-based)
    # -------------------------------
    def open_gripper(self):
        req = SetJointPosition.Request()
        req.planning_group = 'gripper'
        req.joint_position.joint_name = ['gripper']
        req.joint_position.position = [0.01]  # open
        req.path_time = 1.0

        self.future = self.gripper_client.call_async(req)

    def close_gripper(self):
        req = SetJointPosition.Request()
        req.planning_group = 'gripper'
        req.joint_position.joint_name = ['gripper']
        req.joint_position.position = [-0.01]  # close
        req.path_time = 1.0

        self.future = self.gripper_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()