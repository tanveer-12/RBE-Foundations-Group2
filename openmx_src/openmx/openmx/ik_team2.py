#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from scipy.spatial.transform import Rotation as R

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--debug-x", type=float, default=None)
parser.add_argument("--debug-y", type=float, default=None)
parser.add_argument("--debug-z", type=float, default=None)
parser.add_argument("--debug-pitch", type=float, default=None)
debug_args, _ = parser.parse_known_args()

class AnalyticIKNode(Node):

    
    def __init__(self):
        # MUST BE FIRST
        super().__init__("inverse_kinematics_node")

        # ---- Debug Mode Setup ----
        self.debug_pose = None
        if debug_args.debug_x is not None:
            self.debug_pose = np.array([
                debug_args.debug_x,
                debug_args.debug_y,
                debug_args.debug_z
            ])
            self.debug_pitch = debug_args.debug_pitch

            self.get_logger().warn(
                f"[DEBUG MODE] Using debug pose {self.debug_pose}, pitch={self.debug_pitch}"
            )
        else:
            self.debug_pitch = None

        # ---- ROS Subscribers / Publishers ----
        self.pose_sub = self.create_subscription(
            Pose,
            '/ee_pose',
            self.pose_callback,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/ik_joint_angles',
            10
        )

        self.get_logger().info("Analytic IK node ready.")
    # ==========================================================
    #                    CALLBACK: EE POSE
    # ==========================================================
    def pose_callback(self, msg: Pose):

        # Convert pose to mm
        x_c = msg.position.x
        y_c = msg.position.y
        z_c = msg.position.z

        # Read orientation quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Quaternion → Euler angles (roll, pitch, yaw)
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        self.get_logger().info(f"{qx:.4f} rad")
        self.get_logger().info(f"{qy:.4f} rad")
        self.get_logger().info(f"{qz:.4f} rad")
        self.get_logger().info(f"{qw:.4f} rad")
        
        self.get_logger().info(f"Pitch = {pitch:.4f} rad")


        # Compute IK
        q = self.analytic_ik([x_c, y_c, z_c], qx, qy, qz, qw, pitch)

        # Publish joint angles
        js = JointState()
        js.name = ["joint1", "joint2", "joint3", "joint4"]
        js.position = q.tolist()

        self.joint_pub.publish(js)
        theta1_deg = math.degrees(q[0])
        theta2_deg = math.degrees(q[1])
        theta3_deg = math.degrees(q[2])
        theta4_deg = math.degrees(q[3])

        self.get_logger().info(
            f"IK Joint angles (deg)→ "
            f"θ1={theta1_deg:.3f}, "
            f"θ2={theta2_deg:.3f}, "
            f"θ3={theta3_deg:.3f}, "
            f"θ4={theta4_deg:.3f}"
        )

    # ==========================================================
    #                 ANALYTIC IK IMPLEMENTATION
    # ==========================================================
    def analytic_ik(self, target, qx, qy, qz, qw, pitch):
        """
        target = [x, y, z] in mm
        """

        x_c, y_c, z_c = target

        # Constants
        l1 = 96.326
        l2 = 130.23
        l3 = 124.0
        l4 = 133.4
        alpha = math.radians(10.62)
        M_PI = math.pi
        
        offset = math.radians(10.62)
        
        # calculate the wrist position backtracking from the quaternion defined rotation matrix
        x_wc = x_c - l4*(1-2*(qy**2+qz**2))
        y_wc = y_c - l4*(2*(qx*qy+qz*qw))
        z_wc = z_c - l4*(2*(qx*qz-qy*qw))

        # θ1
        theta1 = math.atan2(y_c, x_c)

        # radial distance
        R = math.sqrt(x_wc**2 + y_wc**2)
        
        # vertical distance
        H = z_wc - l1

        S = (R**2+H**2-l2**2-l3**2) / (2*l2*l3)
        
        phi = math.atan2(R,H)
        
        psi = math.acos((l2**2+R**2+H**2-l3**2) / (2*l2*math.sqrt(R**2+H**2)))
        
        theta2 = phi - psi - offset
        
        theta3 = offset - math.pi/2.0 + math.acos(S)

        # θ4 (using pitch)
        theta4 = pitch - theta2 - theta3

        return np.array([theta1, theta2, theta3, theta4])


def main(args=None):
    rclpy.init(args=args)
    node = AnalyticIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
