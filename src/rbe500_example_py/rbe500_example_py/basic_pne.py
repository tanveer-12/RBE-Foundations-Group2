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
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10
        )

        # --- Publisher ---
        self.pose_pub = self.create_publisher(Pose, '/ee_pose', 10)

        # --- Robot geometry ---
        self.L0 = self.declare_parameter('L0_mm', 36.076).get_parameter_value().double_value
        self.L1 = self.declare_parameter('L1_mm', 60.25).get_parameter_value().double_value
        self.L2z = self.declare_parameter('L2z_mm', 128.0).get_parameter_value().double_value
        self.L2x = self.declare_parameter('L2x_mm', 24.0).get_parameter_value().double_value
        self.L3 = self.declare_parameter('L3_mm', 124.0).get_parameter_value().double_value
        self.L4 = self.declare_parameter('L4_mm', 133.4).get_parameter_value().double_value

        self.get_logger().info("FK Node Ready. Listening to /joint_states ...")

    # ------------------------------------------------------------
    # Quaternion Conversion
    # ------------------------------------------------------------
    def rotmat_to_quat(self, R):
        t = np.trace(R)
        if t > 0:
            s = math.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
            if i == 0:
                s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
                w = (R[2, 1] - R[1, 2]) / s
            elif i == 1:
                s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
                w = (R[0, 2] - R[2, 0]) / s
            else:
                s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
                w = (R[1, 0] - R[0, 1]) / s
        return float(x), float(y), float(z), float(w)

    # ------------------------------------------------------------
    # Joint State Callback
    # ------------------------------------------------------------
    def joint_cb(self, msg: JointState):
        if len(msg.position) < 4:
            return

        # Read joint angles in radians directly
        rq1, rq2, rq3, rq4 = msg.position[:4]

        # Compute FK ------------------------------------------------
        T01 = np.array([
            [math.cos(rq1), -math.sin(rq1), 0, 0],
            [math.sin(rq1),  math.cos(rq1), 0, 0],
            [0,              0,            1, self.L0],
            [0, 0, 0, 1]
        ])

        T12 = np.array([
            [math.cos(rq2), 0, math.sin(rq2), 0],
            [0,             1, 0,             0],
            [-math.sin(rq2),0, math.cos(rq2), self.L1],
            [0, 0, 0, 1]
        ])

        T23 = np.array([
            [math.cos(rq3), 0, math.sin(rq3), self.L2x],
            [0,             1, 0,             0],
            [-math.sin(rq3),0, math.cos(rq3), self.L2z],
            [0, 0, 0, 1]
        ])

        T34 = np.array([
            [math.cos(rq4), 0, math.sin(rq4), self.L3],
            [0,             1, 0,            0],
            [-math.sin(rq4),0, math.cos(rq4),0],
            [0, 0, 0, 1]
        ])

        T4ee = np.array([
            [0, 0, 1, self.L4],
            [0, 1, 0, 0],
            [-1,0, 0, 0],
            [0, 0, 0, 1]
        ])

        # Full transform
        T = T01 @ T12 @ T23 @ T34 @ T4ee
        R = T[:3, :3]
        p = T[:3, 3]

        # Quaternion
        qx, qy, qz, qw = self.rotmat_to_quat(R)

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


def main():
    rclpy.init()
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
