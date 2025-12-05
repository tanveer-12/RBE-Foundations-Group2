"""
================================================================================
INVERSE KINEMATICS MODULE - 4-DOF Robotic Manipulator
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

This module implements INVERSE KINEMATICS for a 4-degree-of-freedom (4-DOF) 
robotic manipulator. It computes the joint angles required to position the 
end-effector at a desired pose (position + orientation) in 3D space.

KEY FEATURES:
    - Analytic/geometric closed-form IK solution (fast, deterministic)
    - Wrist position decoupling for simplified computation
    - ROS2 service interface for synchronous IK requests
    - Quaternion-based orientation handling
    - Comprehensive error handling for unreachable poses
    - Degrees of freedom: position (x,y,z) + pitch orientation

ROBOT CONFIGURATION:
    - 4 revolute joints (1 base rotation + 3 planar arm joints)
    - Solution method: Geometric approach with 2R planar manipulator
    - Joint limits and singularities handled via error responses

================================================================================
TECHNICAL OVERVIEW
================================================================================

SOLUTION METHOD:
    Uses analytical/geometric inverse kinematics approach:
    1. Wrist Position Decoupling: Calculate wrist center from TCP
    2. Base Joint (θ1): Solve using atan2 projection onto XY plane
    3. 2R Planar IK (θ2, θ3): Apply law of cosines for shoulder & elbow
    4. Wrist Joint (θ4): Calculate from desired pitch constraint

COORDINATE SYSTEM:
    - Base frame: Z-axis pointing upward, origin at base
    - Input coordinates: millimeters
    - Output joint angles: degrees

ROBOT PARAMETERS:
    l1   = 96.326 mm   (Base height to shoulder joint)
    l2   = 130.23 mm   (Shoulder to elbow link length)
    l3   = 124.0 mm    (Elbow to wrist link length)
    l4   = 133.4 mm    (Wrist to end-effector length)
    offset = 10.62°    (Mechanical offset angle for link geometry)

ALGORITHM STEPS:
    1. Extract target pose from service request (position + quaternion)
    2. Convert quaternion to Euler angles to extract pitch
    3. Calculate wrist center by backtracking along end-effector axis
    4. Solve θ1 from XY projection (base rotation)
    5. Solve θ2, θ3 using 2R planar manipulator geometry
    6. Solve θ4 to achieve desired end-effector pitch
    7. Return joint angles in degrees via service response

ROS2 SERVICE:
    Service:
        - /inverse_kinematics (openmx_interfaces/InverseKinematics)
    
    Request Fields:
        - pose (geometry_msgs/Pose): Target end-effector pose
          · position: x, y, z (meters)
          · orientation: quaternion (qx, qy, qz, qw)
    
    Response Fields:
        - success (bool): True if solution found
        - message (string): Status message or error description
        - theta1, theta2, theta3, theta4 (float): Joint angles (degrees)

ERROR HANDLING:
    - Out of reach targets (distance exceeds workspace)
    - Singular configurations (arm fully extended/retracted)
    - Mathematical domain errors (acos/asin out of range)
    - Invalid quaternion inputs

================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    from .openmx_params import get_robot_params
except ImportError:
    from openmx_params import get_robot_params

from openmx_interfaces.srv import InverseKinematics

class RobotInverseKinematics(Node):

    def __init__(self):
        super().__init__('robot_inv_kin')

        # Create service server for inverse kinematics
        self.srv = self.create_service(
            InverseKinematics,
            'inverse_kinematics',
            self.inverse_kinematics_callback
        )

        # Get robot parameters
        params = get_robot_params()
        self.L0 = params['L0']
        self.L1 = params['L1']
        self.L2 = params['L2']
        self.L3 = params['L3']
        self.L4 = params['L4']
        
        self.get_logger().info('Inverse Kinematics Service Server started')

    # ==========================================================
    #                    CALLBACK: EE POSE
    # ==========================================================
    def inverse_kinematics_callback(self, request, response):

        # Extract position from request
        x_c = request.pose.position.x * 1000.00
        y_c = request.pose.position.y * 1000.00
        z_c = request.pose.position.z * 1000.00
        
        # Extract orientation from quaternion
        qx = request.pose.orientation.x
        qy = request.pose.orientation.y
        qz = request.pose.orientation.z
        qw = request.pose.orientation.w

        # Quaternion → Euler angles (roll, pitch, yaw)
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        self.get_logger().info(f"Pitch = {pitch:.4f} rad")

        response = self.analytic_ik([x_c, y_c, z_c], qx, qy, qz, qw, pitch, response)

        # Log based on response.success

        # Logs IK success with formatted output showing:
        # - Target position (x, y, z in mm)
        # - Target orientation (quaternion qx,qy,qz,qw + pitch)
        # - Computed joint angles (θ1, θ2, θ3, θ4 in degrees)
        # Right-aligned values with borders for readability

        # Log results based on success/failure
        if response.success:
            self.get_logger().info(
                f"x_c: {x_c},\ny_c: {y_c},\nz_c {z_c}, {qx}, {qy}, {qz}, {qw}"
                f"IK Joint angles (deg) → "
                f"θ1={response.theta1:.3f}, "
                f"θ2={response.theta2:.3f}, "
                f"θ3={response.theta3:.3f}, "
                f"θ4={response.theta4:.3f}"
            )
            
        else:
            self.get_logger().error(f"IK failed: {response.message}")

        return response

    # ==========================================================
    #                 ANALYTIC IK IMPLEMENTATION
    # ==========================================================
    def analytic_ik(self, target, qx, qy, qz, qw, pitch, response):
        """
        Calculate inverse kinematics using geometric/analytic method for a 4-DOF manipulator.
        
        This method computes joint angles needed to position the end-effector at a target pose.
        The solution uses wrist position decoupling and geometric relationships between links.
        
        Args:
            target (list): Target position [x, y, z] in millimeters (base frame)
            qx, qy, qz, qw (float): End-effector orientation as quaternion components
            pitch (float): Desired pitch angle of the end-effector (radians)
            response (object): ROS service response object to populate with results
            
        Returns:
            response: Populated response object with joint angles (degrees) or error message
            
        Robot Configuration:
            - 4-DOF serial manipulator
            - Joint 1: Base rotation (revolute, vertical axis)
            - Joint 2: Shoulder pitch (revolute)
            - Joint 3: Elbow pitch (revolute)
            - Joint 4: Wrist pitch (revolute)
        """
        # Extract target coordinates from input list
        x_c, y_c, z_c = target

        # ========================================================================
        # Link lengths and offsets (millimeters)
        # ========================================================================
        l1 = 96.326
        l2 = 130.23
        l3 = 124.0
        l4 = 133.4
        offset = math.radians(10.62)
        M_PI = math.pi
        
        # ========================================================================
        # Step 1: Wrist Position Decoupling
        # ========================================================================
        # Calculate wrist center position by backtracking from TCP along end-effector axis
        # Uses rotation matrix elements derived from quaternion (R_03 matrix, 3rd column)
        # Formula: P_wrist = P_tcp - l4 * R_03[:,2] (Z-axis of end-effector frame)
        # calculate the wrist position backtracking from the quaternion defined rotation matrix
        
        x_wc = x_c - l4*(1-2*(qy**2+qz**2)) # R[0,2]: rotation matrix element
        y_wc = y_c - l4*(2*(qx*qy+qz*qw))   # R[1,2]: rotation matrix element
        z_wc = z_c - l4*(2*(qx*qz-qy*qw))   # R[2,2]: rotation matrix element
        
        try:
            # ====================================================================
            # Step 2: Joint 1 - Base Rotation
            # ====================================================================
            # θ1 is the azimuthal angle in the XY plane
            # Projects the target onto the horizontal plane
            # θ1
            theta1 = math.atan2(y_c, x_c)


            # ====================================================================
            # Step 3: 2R Planar Manipulator Solution (Joints 2 & 3)
            # ====================================================================
            # Project the 3D problem into a 2D plane containing the arm
            
            # Radial distance: horizontal distance from base axis to wrist
            R = math.sqrt(x_wc**2 + y_wc**2)
            
            # Vertical distance: height of wrist above shoulder joint
            H = z_wc - l1
            
            # Law of cosines parameter for triangle formed by l2, l3, and distance to wrist
            # S = cos(θ3 + offset - π/2), used to find elbow angle
            S = (R**2+H**2-l2**2-l3**2) / (2*l2*l3)
            
            # ====================================================================
            # Step 4: Joint 2 - Shoulder Angle
            # ====================================================================
            # phi: angle from horizontal to line connecting shoulder to wrist
            phi = math.atan2(R,H)


            # psi: interior angle at shoulder in the triangle (shoulder-elbow-wrist)
            # Derived using law of cosines
            psi = math.acos((l2**2+R**2+H**2-l3**2) / (2*l2*math.sqrt(R**2+H**2)))
            
            # θ2: shoulder joint angle accounting for mechanical offset
            # Subtracting psi from phi gives the angle from horizontal
            theta2 = phi - psi - offset
            
            # ====================================================================
            # Step 5: Joint 3 - Elbow Angle
            # ====================================================================
            # θ3: elbow joint angle
            # acos(S) gives the supplementary angle; adjustments account for mechanical offset
            theta3 = offset - M_PI/2.0 + math.acos(S)

            # ====================================================================
            # Step 6: Joint 4 - Wrist Orientation
            # ====================================================================
            # θ4: wrist pitch angle to achieve desired end-effector pitch
            # The pitch is distributed across the chain: pitch = θ2 + θ3 + θ4
            theta4 = pitch - theta2 - theta3

            # ====================================================================
            # Step 7: Populate Response
            # ====================================================================
            # Convert all joint angles from radians to degrees for ROS message
            response.success = True
            response.message = "IK solution found"
            
            # Solution
            response.theta1 = math.degrees(theta1)
            response.theta2 = math.degrees(theta2)
            response.theta3 = math.degrees(theta3)
            response.theta4 = math.degrees(theta4)

        except Exception as e:
            # ====================================================================
            # Error Handling
            # ====================================================================
            # Common failure cases:
            # - Target out of reach (sqrt or acos of invalid values)
            # - Singularities (division by zero, atan2 undefined)
            # - Mathematical domain errors (acos/asin input outside [-1, 1])
            response.success = False
            response.message = f"IK computation failed: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    openmx_inv = RobotInverseKinematics()

    try:
        rclpy.spin(openmx_inv)
    except KeyboardInterrupt:
        pass
    finally:
        openmx_inv.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()