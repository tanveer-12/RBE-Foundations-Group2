#!/usr/bin/env python3
import sys
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from open_manipulator_msgs.srv import SetJointPosition
from fkik_srv.srv import InvKinService


class InverseKinematicsNode(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # === DH Geometry (mm) identical to FK ===
        self.L0 = self.declare_parameter('L0_mm', 36.076).get_parameter_value().double_value
        self.L1 = self.declare_parameter('L1_mm', 60.25).get_parameter_value().double_value
        self.L2z = self.declare_parameter('L2z_mm', 128.0).get_parameter_value().double_value
        self.L2x = self.declare_parameter('L2x_mm', 24.0).get_parameter_value().double_value
        self.L3 = self.declare_parameter('L3_mm', 124.0).get_parameter_value().double_value
        self.L4 = self.declare_parameter('L4_mm', 133.4).get_parameter_value().double_value

        # Keep the same conventions as FK node
        self.current_q = np.zeros(4)

        # === Service Server ===
        self.srv = self.create_service(
            InvKinService,
            '/get_ik',
            self.handle_ik
        )

        # === Joint command clients ===
        self.client = self.create_client(SetJointPosition, '/goal_joint_space_path')
        self.gr_client = self.create_client(SetJointPosition, '/goal_tool_control')

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /goal_joint_space_path ...")

        self.get_logger().info("IK Node Ready. Service: /get_ik")

    # ================================================================
    #   Forward Kinematics (must match exactly your FK implementation)
    # ================================================================
    def DH(self, theta, d, a, alpha):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, q):
        theta1, theta2, theta3, theta4 = q

        # FK MUST MATCH your FK script exactly
        M_PI = math.pi
        l1 = 96.326
        l2 = 130.23
        l3 = 124.0
        l4 = 133.4

        vals = np.array([
            [theta1,                             0.0,  l1, -M_PI/2],
            [-(M_PI/2 - math.radians(10.62)) + theta2, l2, 0.0, 0.0],
            [theta3 + (M_PI/2 - math.radians(10.62)),   l3, 0.0, 0.0],
            [theta4,                         l4,  0.0,  M_PI/2]
        ])

        T = np.eye(4)
        for (theta, a, d, alpha) in vals:
            T = T @ self.DH(theta, d, a, alpha)

        return T[:3, 3]   # (x,y,z) in mm

    # ================================================================
    #   Numerical Jacobian (3×4)
    # ================================================================
    def jacobian(self, q, eps=1e-4):
        J = np.zeros((3, 4))
        for i in range(4):
            dq = np.zeros(4)
            dq[i] = eps
            p_plus = self.forward_kinematics(q + dq)
            p_minus = self.forward_kinematics(q - dq)
            J[:, i] = (p_plus - p_minus) / (2 * eps)
        return J

    # ================================================================
    #     Damped Least Squares IK Solver
    # ================================================================
    def solve_ik(self, target_pos_mm):
        q = self.current_q.copy()
        max_iters = 200
        tol = 0.5  # mm
        lam = 0.01

        for i in range(max_iters):
            p = self.forward_kinematics(q)
            err = target_pos_mm - p
            if np.linalg.norm(err) < tol:
                self.current_q = q
                return q, True, "Converged"

            J = self.jacobian(q)
            JJt = J @ J.T
            inv = np.linalg.inv(JJt + lam * lam * np.eye(3))
            dq = J.T @ (inv @ err)

            # Limit delta step for stability
            if np.linalg.norm(dq) > 0.15:
                dq = dq / np.linalg.norm(dq) * 0.15

            q = q + dq

        return q, False, "Did not converge"

    # ================================================================
    #       SERVICE CALLBACK
    # ================================================================
    def handle_ik(self, request, response):
        pos = request.pose_values.position
        target = np.array([pos.x, pos.y, pos.z], dtype=float)

        q, success, message = self.solve_ik(target)

        js = JointState()
        js.name = ["joint1", "joint2", "joint3", "joint4"]
        js.position = q.tolist()
        response.joint_values = js

        if success:
            self.send_joint_command(q)
            time.sleep(2.0)
            self.send_gripper(request.gripper_value)

        return response

    # ================================================================
    #     SEND JOINTS + GRIPPER
    # ================================================================
    def send_joint_command(self, q):
        req = SetJointPosition.Request()
        req.planning_group = 'arm'
        req.joint_position.joint_name = ['joint1','joint2','joint3','joint4','gripper']
        req.joint_position.position = [*q.tolist(), 0.5]
        req.path_time = 2.0
        self.client.call_async(req)

    def send_gripper(self, width):
        req = SetJointPosition.Request()
        req.planning_group = 'gripper'
        req.joint_position.joint_name = ['gripper']
        req.joint_position.position = [width]
        req.path_time = 2.0
        self.gr_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()