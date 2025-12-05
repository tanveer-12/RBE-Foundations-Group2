#import dependencies
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import numpy as np

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
        self.L2V = params.get('L2V', 0.128)  # Vertical component
        self.L2H = params.get('L2H', 0.024)  # Horizontal component
        self.L3 = params['L3']
        self.L4 = params['L4']
        
        # Calculate offset angle for L-shaped link (matches forward kinematics)
        self.theta_0 = math.atan2(self.L2H, self.L2V)  # ~10.62 degrees
        
        # Link 2 effective length (hypotenuse of L-shaped segment)
        self.L2 = math.sqrt(self.L2V**2 + self.L2H**2)
        
        self.get_logger().info('Inverse Kinematics Service Server started')
        self.get_logger().info(f'Robot parameters: L0={self.L0:.6f}, L1={self.L1:.6f}, L2V={self.L2V:.6f}, L2H={self.L2H:.6f}')
        self.get_logger().info(f'Computed: L2={self.L2:.6f}, theta_0={math.degrees(self.theta_0):.2f}°, L3={self.L3:.6f}, L4={self.L4:.6f}')
        
    
    def inverse_kinematics_callback(self, request, response):
        
        # Pose
        x4 = request.pose.position.x
        y4 = request.pose.position.y
        z4 = request.pose.position.z

        qx = request.pose.orientation.x
        qy = request.pose.orientation.y
        qz = request.pose.orientation.z
        qw = request.pose.orientation.w

        _, pitch, _ = self.quaternion_to_euler(qx, qy, qz, qw)

        self.get_logger().info(
            f"IK Request: x={x4:.4f}, y={y4:.4f}, z={z4:.4f}, pitch={math.degrees(pitch):.3f}°"
        )

        try:

            offset = math.atan2(24,128)

            # Wrist Center
            x_wc = x4 - self.L4 * ( 1 - 2 * (qy**2 + qz**2))
            y_wc = y4 - self.L4 * (2 * (qx*qy + qz*qw)) 
            z_wc = z4 - self.L4 * (2 *(qx*qz - qy*qw))

            #Theta 1
            theta1 = math.atan2(y4,x4)

            #Radial
            R = math.sqrt(x_wc**2 + y_wc**2)

            #Vert distance
            H = z_wc - (self.L0 + self.L1)

            S = (R**2 + H**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)

            phi = math.atan2(H,R)

            psi = math.acos((self.L2**2 + R**2 + H**2 - self.L3**2) / (2 * self.L2 * math.sqrt(R**2 + H**2)))

            #Theta 2
            theta2 = math.pi/2 - (phi - psi) - offset

            theta3 = math.acos(S) + math.pi/2 - offset

            theta4 = pitch - theta2 - theta3

            response.theta1 = math.degrees(theta1)
            response.theta2 = math.degrees(theta2)
            response.theta3 = math.degrees(theta3)
            response.theta4 = math.degrees(theta4)

            self.get_logger().info(
                f"Solution 1 (elbow down): θ1={response.theta1:.2f}°, θ2={response.theta2:.2f}°, "
                f"θ3={response.theta3:.2f}°, θ4={response.theta4:.2f}°"
            )

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