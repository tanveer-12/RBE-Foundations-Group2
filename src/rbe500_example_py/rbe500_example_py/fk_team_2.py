#!/usr/bin/env python3
import time
import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


class ForwardKinematicsNode(Node):
    """
    Subscribes to /joint_states
    Computes the forward kinematics
    Publishes end-effector pose on /ee_pose
    """

    def __init__(self):
        super().__init__('forward_kinematics_node')

        # --- Subscribers ---
        # self.subscription = self.create_subscription(
        #     JointState, '/joint_states', self.joint_cb, 10
        # )

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.forward_kinematics_from_joint_state_cb, 10
        )

        # --- Publisher ---
        self.pose_pub = self.create_publisher(Pose, '/ee_pose', 10)

        # --- Robot geometry ---
        self.L0 = self.declare_parameter('L0_mm', 36.076).get_parameter_value().double_value
        self.L1 = self.declare_parameter('L1_mm', 60.25).get_parameter_value().double_value
        self.L2z = self.declare_parameter('L2z_mm', 128.0).get_parameter_value().double_value
        self.L2x = self.declare_parameter('L2x_mm', 24.0).get_parameter_value().double_value
        self.L2 = self.declare_parameter('L2_mm', 130.23).get_parameter_value().double_value
        self.L3 = self.declare_parameter('L3_mm', 124.0).get_parameter_value().double_value
        self.L4 = self.declare_parameter('L4_mm', 133.4).get_parameter_value().double_value

        self.get_logger().info("FK Node Ready. Listening to /joint_states ...")

    def compute_DH_matrix(self, theta, d, a, alpha):
        """
        Create a standard Denavit–Hartenberg (DH) transformation matrix.

        Parameters:
            theta : float   Joint angle (rad) - rotation about z-axis
            d     : float   Link offset (m)  - translation along z-axis
            a     : float   Link length (m)  - translation along x-axis
            alpha : float   Twist angle (rad) - rotation about x-axis

        Returns:
            4x4 numpy array representing the homogeneous transformation matrix
        """

        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        T = np.array([
            [ ct, -st*ca,  st*sa, a*ct ],
            [ st,  ct*ca, -ct*sa, a*st ],
            [  0,     sa,     ca,    d ],
            [  0,      0,      0,    1 ]
        ])

        return T
        
    def forward_kinematics_from_joint_state_cb(self, msg):
        # ----------------------------------------------------
        # 1) Extract joint angles
        # ----------------------------------------------------
        theta1, theta2, theta3, theta4 = msg.position[:4]

        # ----------------------------------------------------
        # 2) DH Parameters (converted to radians where needed)
        #    Note: lengths are in *mm*, convert to meters if required.
        # ----------------------------------------------------
        M_PI = math.pi
        offset = math.radians(79.38)
        l1 = 96.326
        l2 = 130.23
        l3 = 124
        l4 = 133.4

        vals = np.array([
            [theta1,                          0.0,       l1,     -M_PI/2],                      # A1
            [-(M_PI/2 - math.radians(10.62)) + theta2, l2,    0.0,          0.0],               # A2
            [theta3 + (M_PI/2 - math.radians(10.62)),   l3,    0.0,          0.0],              # A3
            [theta4,        l4,      0.0,       M_PI/2]                                         # A4
        ])

        # ----------------------------------------------------
        # 3) Compute individual A_i matrices
        # ----------------------------------------------------
        A = []
        for (theta, a, d, alpha) in vals:
            A.append(self.compute_DH_matrix(theta, d, a, alpha))

        # ----------------------------------------------------
        # 4) Multiply them together:  T = A1 * A2 * A3 * A4
        # ----------------------------------------------------
        T = np.eye(4)
        for Ai in A:
            T = T @ Ai

        # ----------------------------------------------------
        # 5) Extract rotation (3x3) and position (3x1)
        # ----------------------------------------------------
        R = T[:3, :3]
        p = T[:3, 3]

        # Quaternion
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

        # Publish pose ----------------------------------------------
        pose_msg = Pose()
        pose_msg.position.x = float(p[0])
        pose_msg.position.y = float(p[1])
        pose_msg.position.z = float(p[2])
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # Log (optional)
        self.get_logger().info(
            f"EE Position: x={p[0]:.2f}, y={p[1]:.2f}, z={p[2]:.2f}"
        )

    def rotation_matrix_to_quaternion(self, R):
        t = np.trace(R)
        if t > 0:
            S = math.sqrt(t + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S

        return qx, qy, qz, qw


def main():
    rclpy.init()
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()