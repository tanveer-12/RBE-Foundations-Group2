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
        
    """
    def inverse_kinematics_callback(self, request, response):
        
        Service callback for inverse kinematics
        Takes a Pose request and returns joint angles
        
        # Extract position from request
        x4 = request.pose.position.x
        y4 = request.pose.position.y
        z4 = request.pose.position.z
        
        # Extract orientation from quaternion
        qx = request.pose.orientation.x
        qy = request.pose.orientation.y
        qz = request.pose.orientation.z
        qw = request.pose.orientation.w
        
        # Convert quaternion to Euler angles to extract pitch
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        self.get_logger().info(
            f"IK Service called: x={x4:.4f}, y={y4:.4f}, z={z4:.4f}, pitch={math.degrees(pitch):.2f}°"
        )

        # Compute inverse kinematics
        try:
            # Theta 1: Base rotation
            theta_1 = math.atan2(y4, x4)

            # Project to 2D plane
            r4 = math.sqrt(x4**2 + y4**2)
            s4 = z4 - (self.L0 + self.L1)

            # Account for end-effector orientation
            r3 = r4 - self.L4 * math.cos(pitch)
            s3 = s4 - self.L4 * math.sin(pitch)

            # Check reachability
            d_sq = r3**2 + s3**2
            d = math.sqrt(d_sq)
            reach_max = self.L2 + self.L3
            reach_min = abs(self.L2 - self.L3)

            if d > reach_max or d < reach_min:
                response.success = False
                response.message = f"Target unreachable! Distance={d:.4f}, Valid range=[{reach_min:.4f}, {reach_max:.4f}]"
                self.get_logger().error(response.message)
                return response
            
            # Theta 3: Elbow angle
            cos_theta3 = (d_sq - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)

            if abs(cos_theta3) > 1.0:
                response.success = False
                response.message = f"IK failed: cos(θ3) = {cos_theta3:.4f} (out of range)"
                self.get_logger().error(response.message)
                return response
            
            offset = math.pi/2.0 - self.theta_0

            # Two solutions for theta3
            theta_3_down = math.acos(cos_theta3) + offset
            theta_3_up = offset - math.acos(cos_theta3)

            # Theta 2: Shoulder angle
            beta = math.atan2(s3, r3)

            # Using law of cosines for gamma
            gamma = math.acos((self.L2**2 + d_sq - self.L3**2) / (2 * self.L2 * d))
            
            theta_2_down = offset - (beta - gamma)
            theta_2_up = offset - (beta + gamma)

            # Theta 4: Wrist angle
            theta_4_down = pitch - theta_2_down - theta_3_down
            theta_4_up = pitch - theta_2_up - theta_3_up

            # Convert to degrees
            response.success = True
            response.message = "IK solution found"
            
            
            # Solution 1 (elbow down)
            response.theta1 = math.degrees(theta_1)
            response.theta2 = math.degrees(theta_2_down)
            response.theta3 = math.degrees(theta_3_down)
            response.theta4 = math.degrees(theta_4_down)
            
            # Solution 2 (elbow up)
            response.theta1_alt = math.degrees(theta_1)
            response.theta2_alt = math.degrees(theta_2_up)
            response.theta3_alt = math.degrees(theta_3_up)
            response.theta4_alt = math.degrees(theta_4_up)
            
            # Solution 1 (elbow down)
            response.theta1 = math.degrees(theta_1)
            response.theta2 = math.degrees(theta_2_down)
            response.theta3 = math.degrees(theta_3_down_adjusted)
            response.theta4 = math.degrees(theta_4_down)
            
            # Solution 2 (elbow up)
            response.theta1_alt = math.degrees(theta_1)
            response.theta2_alt = math.degrees(theta_2_up)
            response.theta3_alt = math.degrees(theta_3_up_adjusted)
            response.theta4_alt = math.degrees(theta_4_up)
            
            self.get_logger().info(
                f"Solution 1 (elbow down): θ1={response.theta1:.2f}°, θ2={response.theta2:.2f}°, "
                f"θ3={response.theta3:.2f}°, θ4={response.theta4:.2f}°"
            )
            self.get_logger().info(
                f"Solution 2 (elbow up): θ1={response.theta1_alt:.2f}°, θ2={response.theta2_alt:.2f}°, "
                f"θ3={response.theta3_alt:.2f}°, θ4={response.theta4_alt:.2f}°"
            )

        except Exception as e:
            response.success = False
            response.message = f"IK computation failed: {str(e)}"
            self.get_logger().error(response.message)

        return response
    """
    
    def inverse_kinematics_callback(self, request, response):
        """
        Inverse kinematics callback that returns servo angles (degrees).
        Chooses the solution closest to servo-zero so FK(0,0,0,0) -> IK(...) returns (0,0,0,0).
        """
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
            """
            # --- Wrist center in plane ---
            r4 = math.hypot(x4, y4)
            s4 = z4 - (self.L0 + self.L1)

            r3 = r4 - self.L4 * math.cos(pitch)
            s3 = s4 - self.L4 * math.sin(pitch)

            # base rotation (kinematic)
            theta1_kin = math.atan2(y4, x4)

            # distance to wrist center
            d_sq = r3 * r3 + s3 * s3
            d = math.sqrt(d_sq)

            # reach check
            reach_max = self.L2 + self.L3
            reach_min = abs(self.L2 - self.L3)
            if d > reach_max + 1e-9 or d < reach_min - 1e-9:
                response.success = False
                response.message = f"Target unreachable: d={d:.6f}, allowed=[{reach_min:.6f},{reach_max:.6f}]"
                self.get_logger().error(response.message)
                return response

            # --- Kinematic theta3 (two branches) ---
            D = (d_sq - self.L2**2 - self.L3**2) / (2.0 * self.L2 * self.L3)
            D = max(-1.0, min(1.0, D))  # clamp
            # kinematic solutions for theta3
            theta3_kin_down = +math.acos(D)   # elbow-down (convention)
            theta3_kin_up   = -math.acos(D)   # elbow-up

            # --- Kinematic theta2 ---
            beta = math.atan2(s3, r3)
            phi_down = math.atan2(self.L3 * math.sin(theta3_kin_down),
                                self.L2 + self.L3 * math.cos(theta3_kin_down))
            phi_up = math.atan2(self.L3 * math.sin(theta3_kin_up),
                                self.L2 + self.L3 * math.cos(theta3_kin_up))

            theta2_kin_down = beta - phi_down
            theta2_kin_up   = beta - phi_up

            # --- Kinematic theta4 (wrist) ---
            theta4_kin_down = pitch - (theta2_kin_down + theta3_kin_down)
            theta4_kin_up   = pitch - (theta2_kin_up + theta3_kin_up)

            # --- Convert kinematic -> servo (your DH mapping) ---
            offset = math.pi / 2.0 - self.theta_0

            # servo angles in radians
            s_down = [
                theta1_kin,
                theta2_kin_down + offset,
                theta3_kin_down - offset,
                theta4_kin_down
            ]
            s_up = [
                theta1_kin,
                theta2_kin_up + offset,
                theta3_kin_up - offset,
                theta4_kin_up
            ]

            # Evaluate which solution is closer to servo-zero
            def servo_norm(sol):
                return math.sqrt(sum((float(v))**2 for v in sol))

            norm_down = servo_norm(s_down)
            norm_up = servo_norm(s_up)

            # Choose primary = the one closer to zero servo vector (so FK(0,0,0,0) maps to primary)
            if norm_down <= norm_up:
                primary = s_down
                alt = s_up
                chosen = 'down'
            else:
                primary = s_up
                alt = s_down
                chosen = 'up'

            # Optional: tiny tolerance snap to exact zero where appropriate (helps exact match)
            # If a servo angle is within 1e-6 rad of zero, force exact zero (avoid tiny numerical residuals).
            def snap_zero(sol):
                return [0.0 if abs(x) < 1e-6 else x for x in sol]

            primary = snap_zero(primary)
            alt = snap_zero(alt)

            # Fill response in degrees (servo frame)
            response.theta1 = math.degrees(primary[0])
            response.theta2 = math.degrees(primary[1])
            response.theta3 = math.degrees(primary[2])
            response.theta4 = math.degrees(primary[3])

            response.theta1_alt = math.degrees(alt[0])
            response.theta2_alt = math.degrees(alt[1])
            response.theta3_alt = math.degrees(alt[2])
            response.theta4_alt = math.degrees(alt[3])

            response.success = True
            response.message = f"IK solution found (primary={chosen})"

            self.get_logger().info(
                f"Primary ({chosen}) servo°: θ1={response.theta1:.6f}, θ2={response.theta2:.6f}, "
                f"θ3={response.theta3:.6f}, θ4={response.theta4:.6f}"
            )
            self.get_logger().info(
                f"Alt servo°: θ1_alt={response.theta1_alt:.6f}, θ2_alt={response.theta2_alt:.6f}, "
                f"θ3_alt={response.theta3_alt:.6f}, θ4_alt={response.theta4_alt:.6f}"
            )

            # --- Optional verification (uncomment to log FK of returned servo solution) ---
            # fk_primary = self.forward_kinematics_from_servo(primary[0], primary[1], primary[2], primary[3])
            # self.get_logger().info(f"FK(primary) -> x={fk_primary[0]:.6f}, y={fk_primary[1]:.6f}, z={fk_primary[2]:.6f}, pitch={math.degrees(fk_primary[3]):.6f}°")

            return response
            """

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