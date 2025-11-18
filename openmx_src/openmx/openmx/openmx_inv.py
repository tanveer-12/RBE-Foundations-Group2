#import dependencies
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

        # self.srv = self.create_service(
        #     InverseKinematics,
        #     'inverse_kinematics',
        #     self.inverse_kinematics_callback
        # )

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

        # Compute IK
        # self.analytic_ik([x_c, y_c, z_c], qx, qy, qz, qw, pitch, response)

        # self.analytic_ik([x_c, y_c, z_c], qx, qy, qz, qw, pitch, response)

        # Compute IK (fills response directly)
        response = self.analytic_ik([x_c, y_c, z_c], qx, qy, qz, qw, pitch, response)

        # Log based on response.success
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


        # # self.joint_pub.publish(js)
        # theta1_deg = math.degrees(q[0])
        # theta2_deg = math.degrees(q[1])
        # theta3_deg = math.degrees(q[2])
        # theta4_deg = math.degrees(q[3])

        # self.get_logger().info(
        #     f"IK Joint angles (deg)→ "
        #     f"θ1={theta1_deg:.3f}, "
        #     f"θ2={theta2_deg:.3f}, "
        #     f"θ3={theta3_deg:.3f}, "
        #     f"θ4={theta4_deg:.3f}"
        # )

    # ==========================================================
    #                 ANALYTIC IK IMPLEMENTATION
    # ==========================================================
    def analytic_ik(self, target, qx, qy, qz, qw, pitch, response):
        """
        target = [x, y, z] in mm
        """

        x_c, y_c, z_c = target

        # Constants in mm
        l1 = 96.326
        l2 = 130.23
        l3 = 124.0
        l4 = 133.4
        offset = math.radians(10.62)
        M_PI = math.pi
        
        
        # calculate the wrist position backtracking from the quaternion defined rotation matrix
        x_wc = x_c - l4*(1-2*(qy**2+qz**2))
        y_wc = y_c - l4*(2*(qx*qy+qz*qw))
        z_wc = z_c - l4*(2*(qx*qz-qy*qw))
        
        try:
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
            
            theta3 = offset - M_PI/2.0 + math.acos(S)

            # θ4 (using pitch)
            theta4 = pitch - theta2 - theta3

            # Convert to degrees
            response.success = True
            response.message = "IK solution found"
            
            # Solution 1 (elbow down)
            response.theta1 = math.degrees(theta1)
            response.theta2 = math.degrees(theta2)
            response.theta3 = math.degrees(theta3)
            response.theta4 = math.degrees(theta4)

        except Exception as e:
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
