"""
================================================================================
JACOBIAN VELOCITY KINEMATICS MODULE - 4-DOF Robotic Manipulator
================================================================================

PROJECT: 4-DOF Robotic Manipulator Kinematics
FRAMEWORK: ROS2 (Robot Operating System 2)
LANGUAGE: Python 3

TEAM MEMBERS (GROUP 2):
    - Rajdeep Banerjee
    - John Hoang Do
    - Hunter Sanford
    - Tanveer Kaur

================================================================================
EXECUTIVE SUMMARY
================================================================================

This module implements JACOBIAN-BASED VELOCITY KINEMATICS for a 4-degree-of-
freedom (4-DOF) robotic manipulator. It provides bidirectional conversion 
between joint velocities and end-effector velocities using the manipulator 
Jacobian matrix.

KEY FEATURES:
    - Forward velocity kinematics: joint velocities → EE velocities
    - Inverse velocity kinematics: EE velocities → joint velocities
    - Analytical Jacobian computation using DH parameters
    - Proper handling of the manipulator Jacobian pseudoinverse
    - ROS2 service interface for real-time velocity transformations
    - Comprehensive error handling and singularity detection

ROBOT CONFIGURATION:
    - 4 revolute joints (1 base rotation + 3 planar arm joints)
    - Jacobian dimension: 6x4 (6-DOF velocity, 4-DOF joint space)

================================================================================
TECHNICAL OVERVIEW
================================================================================

JACOBIAN MATRIX:
    The manipulator Jacobian J relates joint velocities to end-effector
    velocities:
        V = J(q) * q_dot
    
    where:
        V = [vx, vy, vz, wx, wy, wz]^T  (EE linear + angular velocity)
        q_dot = [θ1_dot, θ2_dot, θ3_dot, θ4_dot]^T  (joint velocities)
        J(q) = 6x4 Jacobian matrix (configuration-dependent)

JACOBIAN COMPUTATION:
    For each revolute joint i:
        Linear velocity contribution:  J_vi = z_{i-1} x (o_n - o_{i-1})
        Angular velocity contribution: J_ωi = z_{i-1}
    
    where:
        z_{i-1} = rotation axis of joint i (in base frame)
        o_n = end-effector position
        o_{i-1} = position of joint i

FORWARD VELOCITY KINEMATICS:
    V = J(q) * q_dot
    Direct matrix multiplication

INVERSE VELOCITY KINEMATICS:
    q_dot = J^+ * V
    where J^+ = (J^T * J)^(-1) * J^T is the pseudoinverse
    
    This provides the minimum-norm solution for the overdetermined system
    (6 EE velocity DOF, 4 joint DOF)

ROS2 SERVICES:
    Services:
        - /joint_to_ee_velocity: Forward velocity kinematics
        - /ee_to_joint_velocity: Inverse velocity kinematics
    
    Service Types:
        - JointToEEVelocity.srv
        - EEToJointVelocity.srv

================================================================================
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np

# Import robot parameters
try:
    from .openmx_params import get_robot_params
except ImportError:
    from openmx_params import get_robot_params

from openmx_interfaces.srv import JointToEEVelocity, EEToJointVelocity


class JacobianVelocityKinematics(Node):
    """
    ROS2 node for Jacobian-based velocity transformations.
    
    This node provides two services:
    1. joint_to_ee_velocity: Converts joint velocities to end-effector velocities
    2. ee_to_joint_velocity: Converts end-effector velocities to joint velocities
    
    The node uses the manipulator Jacobian computed from forward kinematics
    to perform these transformations.
    """

    def __init__(self):
        super().__init__('jacobian_velocity_kinematics')

        # Get robot parameters
        params = get_robot_params()
        self.L0 = params['L0']
        self.L1 = params['L1']
        self.L2 = params['L2']  # This is actually L2_total = sqrt(L2V^2 + L2H^2)
        self.L3 = params['L3']
        self.L4 = params['L4']
        self.L2V = params['L2V']
        self.L2H = params['L2H']
        
        # Calculate the offset angle for the L-shaped link
        self.offset = math.atan2(self.L2H, self.L2V)  # ~10.62 degrees
        
        # Link lengths for DH parameters
        self.a2 = self.L2  # ~0.130 m
        self.a3 = self.L3  # 0.124 m
        self.a4 = self.L4  # 0.1334 m

        # Create service servers
        self.joint_to_ee_srv = self.create_service(
            JointToEEVelocity,
            'joint_to_ee_velocity',
            self.joint_to_ee_vel_callback
        )
        
        self.ee_to_joint_srv = self.create_service(
            EEToJointVelocity,
            'ee_to_joint_velocity',
            self.ee_to_joint_vel_callback
        )

        self.get_logger().info('Jacobian Velocity Kinematics Node started')
        self.get_logger().info(f'Robot parameters: L0={self.L0:.6f}, L1={self.L1:.6f}')
        self.get_logger().info(f'                  L2V={self.L2V:.6f}, L2H={self.L2H:.6f}')
        self.get_logger().info(f'                  L3={self.L3:.6f}, L4={self.L4:.6f}')
        self.get_logger().info(f'                  offset={math.degrees(self.offset):.2f}°')

    def dh_transform(self, a, alpha, d, theta):
        """
        Compute standard DH transformation matrix.
        
        Args:
            a: link length
            alpha: link twist
            d: link offset
            theta: joint angle
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,      ca,      d],
            [0,        0,       0,      1.0]
        ])

    def compute_jacobian(self, q):
        """
        Compute the manipulator Jacobian matrix for a given joint configuration.
        
        The Jacobian J is a 6x4 matrix that relates joint velocities to 
        end-effector velocities:
            V = J * q_dot
        
        where:
            V = [vx, vy, vz, wx, wy, wz]^T (6x1)
            q_dot = [θ1_dot, θ2_dot, θ3_dot, θ4_dot]^T (4x1)
            J = Jacobian matrix (6x4)
        
        Args:
            q: Joint configuration [θ1, θ2, θ3, θ4] in radians
            
        Returns:
            J: 6x4 Jacobian matrix
        """
        q1, q2, q3, q4 = q

        # ====================================================================
        # Step 1: Compute DH transformation matrices
        # ====================================================================
        # Match DH convention in openmx_fwd.py
        
        # Link 1: Base rotation joint
        A1 = self.dh_transform(
            a=0,
            alpha=-math.pi/2.0,
            d=self.L0 + self.L1,
            theta=q1
        )
        
        # Link 2: Shoulder joint (with offset correction)
        A2 = self.dh_transform(
            a=self.a2,
            alpha=0,
            d=0,
            theta=q2 - (math.pi/2.0 - self.offset)
        )
        
        # Link 3: Elbow joint (with offset correction)
        A3 = self.dh_transform(
            a=self.a3,
            alpha=0,
            d=0,
            theta=q3 + (math.pi/2.0 - self.offset)
        )
        
        # Link 4: Wrist joint
        A4 = self.dh_transform(
            a=self.a4,
            alpha=0,
            d=0,
            theta=q4
        )

        # ====================================================================
        # Step 2: Compute cumulative transformations
        # ====================================================================
        H1 = A1
        H2 = H1 @ A2
        H3 = H2 @ A3
        H4 = H3 @ A4

        # ====================================================================
        # Step 3: Extract end-effector position and joint positions
        # ====================================================================
        o_n = H4[0:3, 3]  # End-effector position
        o1 = H1[0:3, 3]   # Joint 2 position
        o2 = H2[0:3, 3]   # Joint 3 position
        o3 = H3[0:3, 3]   # Joint 4 position

        # ====================================================================
        # Step 4: Extract z-axes (rotation axes) for each joint
        # ====================================================================
        # For revolute joints, the z-axis is the axis of rotation
        z_0 = np.array([0, 0, 1])  # Base frame z-axis
        z_1 = H1[0:3, 2]           # Joint 1 z-axis
        z_2 = H2[0:3, 2]           # Joint 2 z-axis
        z_3 = H3[0:3, 2]           # Joint 3 z-axis

        # ====================================================================
        # Step 5: Compute Jacobian columns
        # ====================================================================
        # For revolute joints:
        #   J_linear = z_{i-1} x (o_n - o_{i-1})
        #   J_angular = z_{i-1}
        
        # Joint 1: Base rotation
        J_v1 = np.cross(z_0, o_n)
        J_w1 = z_0
        
        # Joint 2: Shoulder pitch
        J_v2 = np.cross(z_1, o_n - o1)
        J_w2 = z_1
        
        # Joint 3: Elbow pitch
        J_v3 = np.cross(z_2, o_n - o2)
        J_w3 = z_2
        
        # Joint 4: Wrist pitch
        J_v4 = np.cross(z_3, o_n - o3)
        J_w4 = z_3

        # ====================================================================
        # Step 6: Assemble the Jacobian matrix (6×4)
        # ====================================================================
        # Each column: [J_v; J_w] (linear velocity; angular velocity)
        J = np.column_stack([
            np.concatenate([J_v1, J_w1]),
            np.concatenate([J_v2, J_w2]),
            np.concatenate([J_v3, J_w3]),
            np.concatenate([J_v4, J_w4])
        ])

        return J

    def joint_to_ee_vel_callback(self, request, response):
        """
        Service callback for forward velocity kinematics.
        
        Converts joint velocities to end-effector velocities using:
            V = J(q) * q_dot
        
        Args:
            request: Contains q (joint positions) and q_dot (joint velocities)
            response: Populated with V (EE linear + angular velocity)
            
        Returns:
            response with success flag and computed velocities
        """
        try:
            # Extract joint configuration and velocities from request
            q = np.array([request.q1, request.q2, request.q3, request.q4])
            q_dot = np.array([request.q1_dot, request.q2_dot, 
                             request.q3_dot, request.q4_dot])

            self.get_logger().info(
                f"Forward velocity request:\n"
                f"  Joint positions (rad): {np.array2string(q, precision=4)}\n"
                f"  Joint velocities (rad/s): {np.array2string(q_dot, precision=4)}"
            )

            # Compute Jacobian at current configuration
            J = self.compute_jacobian(q)

            # Forward velocity kinematics: V = J * q_dot
            V = J @ q_dot

            # Populate response
            response.vx = float(V[0])
            response.vy = float(V[1])
            response.vz = float(V[2])
            response.wx = float(V[3])
            response.wy = float(V[4])
            response.wz = float(V[5])
            response.success = True
            response.message = "Forward velocity kinematics computed successfully"

            self.get_logger().info(
                f"  EE linear velocity (m/s): [{V[0]:.4f}, {V[1]:.4f}, {V[2]:.4f}]\n"
                f"  EE angular velocity (rad/s): [{V[3]:.4f}, {V[4]:.4f}, {V[5]:.4f}]"
            )

        except Exception as e:
            response.success = False
            response.message = f"Forward velocity kinematics failed: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def ee_to_joint_vel_callback(self, request, response):
        """
        Service callback for inverse velocity kinematics.
        
        Converts end-effector velocities to joint velocities using:
            q_dot = J^+ * V
        where J^+ = (J^T * J)^(-1) * J^T is the Moore-Penrose pseudoinverse
        
        This provides the minimum-norm solution for the overdetermined system
        (6 EE velocity DOF mapped to 4 joint DOF).
        
        Args:
            request: Contains q (joint positions) and V (EE velocity)
            response: Populated with q_dot (joint velocities)
            
        Returns:
            response with success flag and computed joint velocities
        """
        try:
            # Extract joint configuration and desired EE velocity from request
            q = np.array([request.q1, request.q2, request.q3, request.q4])
            V = np.array([request.vx, request.vy, request.vz, 
                         request.wx, request.wy, request.wz])

            self.get_logger().info(
                f"Inverse velocity request:\n"
                f"  Joint positions (rad): {np.array2string(q, precision=4)}\n"
                f"  EE linear velocity (m/s): [{V[0]:.4f}, {V[1]:.4f}, {V[2]:.4f}]\n"
                f"  EE angular velocity (rad/s): [{V[3]:.4f}, {V[4]:.4f}, {V[5]:.4f}]"
            )

            # Compute Jacobian at current configuration
            J = self.compute_jacobian(q)

            # ================================================================
            # Inverse velocity kinematics using pseudoinverse
            # ================================================================
            # For overdetermined system (6×4 Jacobian), use:
            #   q_dot = (J^T * J)^(-1) * J^T * V
            # This is the Moore-Penrose right pseudoinverse
            
            # Check for singularities by examining J^T * J
            JTJ = J.T @ J
            det_JTJ = np.linalg.det(JTJ)
            
            if abs(det_JTJ) < 1e-6:
                raise ValueError(
                    f"Near-singular configuration detected (det(J^T*J) = {det_JTJ:.2e}). "
                    "Inverse velocity solution may be unreliable."
                )

            # Compute pseudoinverse: J^+ = (J^T * J)^(-1) * J^T
            J_pseudoinv = np.linalg.inv(JTJ) @ J.T

            # Inverse velocity kinematics: q_dot = J^+ * V
            q_dot = J_pseudoinv @ V

            # Populate response
            response.q1_dot = float(q_dot[0])
            response.q2_dot = float(q_dot[1])
            response.q3_dot = float(q_dot[2])
            response.q4_dot = float(q_dot[3])
            response.success = True
            response.message = "Inverse velocity kinematics computed successfully"

            self.get_logger().info(
                f"  Joint velocities (rad/s): {np.array2string(q_dot, precision=4)}\n"
                f"  Singularity measure (det(J^T*J)): {det_JTJ:.4e}"
            )

            # ================================================================
            # Verification (optional): Check if J * q_dot ≈ V
            # ================================================================
            V_check = J @ q_dot
            velocity_error = np.linalg.norm(V - V_check)
            
            if velocity_error > 1e-3:
                self.get_logger().warn(
                    f"  Velocity verification error: {velocity_error:.6f} m/s\n"
                    f"  This may indicate numerical issues or singularity"
                )
            else:
                self.get_logger().info(
                    f"  Velocity solution verified (error: {velocity_error:.2e} m/s)"
                )

        except Exception as e:
            response.success = False
            response.message = f"Inverse velocity kinematics failed: {str(e)}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    """
    Main function to run the Jacobian velocity kinematics node.
    """
    rclpy.init(args=args)
    jacobian_node = JacobianVelocityKinematics()

    try:
        rclpy.spin(jacobian_node)
    except KeyboardInterrupt:
        pass
    finally:
        jacobian_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()