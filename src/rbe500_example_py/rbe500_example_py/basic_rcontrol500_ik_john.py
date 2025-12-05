############## John's code ##################
# #import dependencies
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose
# import math

# try:
#     from .openmx_params import get_robot_params
# except ImportError:
#     from openmx_params import get_robot_params

# from openmx_interfaces.srv import InverseKinematics

# class RobotInverseKinematics(Node):

#     def __init__(self):
#         super().__init__('robot_inv_kin')

#         # Create service server for inverse kinematics
#         self.srv = self.create_service(
#             InverseKinematics,
#             'inverse_kinematics',
#             self.inverse_kinematics_callback
#         )

#         # Get robot parameters
#         params = get_robot_params()
#         self.L0 = params['L0']
#         self.L1 = params['L1']
#         self.L2 = params['L2']
#         self.L3 = params['L3']
#         self.L4 = params['L4']
        
#         self.get_logger().info('Inverse Kinematics Service Server started')

#     def inverse_kinematics_callback(self, request, response):
#         """
#         Service callback for inverse kinematics
#         Takes a Pose request and returns joint angles
#         """
#         # Extract position from request
#         x4 = request.pose.position.x
#         y4 = request.pose.position.y
#         z4 = request.pose.position.z
        
#         # Extract pitch from quaternion
#         qx = request.pose.orientation.x
#         qy = request.pose.orientation.y
#         qz = request.pose.orientation.z
#         qw = request.pose.orientation.w
        
#         pitch = math.atan2(2*(qw*qy - qz*qx), 1 - 2*(qx**2 + qy**2))
        
#         self.get_logger().info(
#             f"IK Service called: x={x4:.4f}, y={y4:.4f}, z={z4:.4f}, pitch={math.degrees(pitch):.2f}°"
#         )

#         # Compute inverse kinematics
#         try:
#             # Theta 1: Base rotation
#             theta_1 = math.atan2(y4, x4)

#             # Project to 2D plane
#             r4 = math.sqrt(x4**2 + y4**2)
#             s4 = z4 - (self.L0 + self.L1)

#             # Account for end-effector orientation
#             r3 = r4 - self.L4 * math.cos(pitch)
#             s3 = s4 - self.L4 * math.sin(pitch)

#             # Check reachability
#             d_sq = r3**2 + s3**2
#             d = math.sqrt(d_sq)
#             reach_max = self.L2 + self.L3
#             reach_min = abs(self.L2 - self.L3)

#             if d > reach_max or d < reach_min:
#                 response.success = False
#                 response.message = f"Target unreachable! Distance={d:.4f}, Valid range=[{reach_min:.4f}, {reach_max:.4f}]"
#                 self.get_logger().error(response.message)
#                 return response
            
#             # Theta 3
#             cos_theta3 = (d_sq - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)

#             if abs(cos_theta3) > 1.0:
#                 response.success = False
#                 response.message = f"IK failed: cos(θ3) = {cos_theta3:.4f} (out of range)"
#                 self.get_logger().error(response.message)
#                 return response
            
#             # Two solutions for theta3
#             theta_3_down = math.acos(cos_theta3)
#             theta_3_up = -theta_3_down

#             # Theta 2
#             beta = math.atan2(s3, r3)

#             # Using law of cosines for gamma
#             gamma = math.acos((self.L2**2 + d_sq - self.L3**2) / (2 * self.L2 * d))
#             theta_2_down = beta - gamma
#             theta_2_up = beta + gamma

#             # Theta 4: Wrist angle
#             theta_4_down = pitch - theta_2_down - theta_3_down
#             theta_4_up = pitch - theta_2_up - theta_3_up

#             # Convert to degrees
#             response.success = True
#             response.message = "IK solution found"
            
#             # Solution 1 (elbow down)
#             response.theta1 = math.degrees(theta_1)
#             response.theta2 = math.degrees(theta_2_down)
#             response.theta3 = math.degrees(theta_3_down)
#             response.theta4 = math.degrees(theta_4_down)
            
#             # Solution 2 (elbow up)
#             response.theta1_alt = math.degrees(theta_1)
#             response.theta2_alt = math.degrees(theta_2_up)
#             response.theta3_alt = math.degrees(theta_3_up)
#             response.theta4_alt = math.degrees(theta_4_up)

#             self.get_logger().info(
#                 f"Solution 1: theta1={response.theta1:.2f}°, theta2={response.theta2:.2f}°, "
#                 f"theta3={response.theta3:.2f}°, theta4={response.theta4:.2f}°"
#             )
#             self.get_logger().info(
#                 f"Solution 2: theta1={response.theta1_alt:.2f}°, theta2={response.theta2_alt:.2f}°, "
#                 f"theta3={response.theta3_alt:.2f}°, theta4={response.theta4_alt:.2f}°"
#             )

#         except Exception as e:
#             response.success = False
#             response.message = f"IK computation failed: {str(e)}"
#             self.get_logger().error(response.message)

#         return response


# def main(args=None):        #define main() function, initialize rclpy and this node
#     rclpy.init(args=args)

#     openmx_inv = RobotInverseKinematics()

#     rclpy.spin(openmx_inv)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     openmx_inv.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

###### Original #############
# import math
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose
# from openmx_interfaces.srv import InverseKinematics


# class IKNode(Node):
#     def __init__(self):
#         super().__init__("openmx_exact_ik")

#         self.srv = self.create_service(
#             InverseKinematics,
#             "inverse_kinematics",
#             self.inverse_callback
#         )

#         # SAME link lengths as FK
#         self.L0 = 36.076
#         self.L1 = 60.25
#         self.L2 = 130.23
#         self.L3 = 124.0
#         self.L4 = 133.4

#         self.offset = math.radians(79.38)

#         self.get_logger().info("Exact IK Service started (matches FK exactly).")

#     def inverse_callback(self, request, response):
#         # EE position
#         x4 = request.pose.position.x
#         y4 = request.pose.position.y
#         z4 = request.pose.position.z

#         # Extract pitch from quaternion (tool rotates around y-axis)
#         qx = request.pose.orientation.x
#         qy = request.pose.orientation.y
#         qz = request.pose.orientation.z
#         qw = request.pose.orientation.w

#         pitch = math.atan2(2*(qw*qy - qz*qx), 1 - 2*(qx*qx + qy*qy))

#         # Base rotation
#         q1 = math.atan2(y4, x4)

#         # Wrist position
#         x3 = x4 - self.L4 * math.cos(q1) * math.cos(pitch)
#         y3 = y4 - self.L4 * math.sin(q1) * math.cos(pitch)
#         z3 = z4 - self.L4 * math.sin(pitch)

#         # Planar distance (subtract vertical offset)
#         r = math.sqrt(x3**2 + y3**2) - self.L1
#         s = z3 - self.L0

#         # Distance to wrist
#         d2 = r*r + s*s
#         d = math.sqrt(d2)

#         # Solve q3_actual
#         cos_q3 = (d2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
#         cos_q3 = max(-1.0, min(1.0, cos_q3))  # clamp
#         q3_actual = math.acos(cos_q3)

#         # Solve q2_actual
#         beta = math.atan2(s, r)
#         gamma = math.acos(
#             (self.L2**2 + d2 - self.L3**2) / (2 * self.L2 * d)
#         )
#         q2_actual = beta - gamma

#         # Convert to real q2, q3 (your FK mapping)
#         q2 = q2_actual + self.offset
#         q3 = q3_actual - self.offset

#         # q4 from tool pitch
#         q4 = pitch - q2_actual - q3_actual

#         # Convert to degrees
#         response.theta1 = math.degrees(q1)
#         response.theta2 = math.degrees(q2)
#         response.theta3 = math.degrees(q3)
#         response.theta4 = math.degrees(q4)

#         response.success = True
#         response.message = "Exact IK solution found"

#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node = IKNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

####################### Updated fix ##########################
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray


class IKNode(Node):

    def __init__(self):
        super().__init__('openmx_ik_node')

        # ------------ SUBSCRIBE TO EE POSE -------------
        self.pose_sub = self.create_subscription(
            Pose,
            '/fk_output',
            self.ik_callback,
            10
        )

        # ------------ PUBLISH JOINT ANGLES -------------
        self.joint_pub = self.create_publisher(
            Float32MultiArray,
            '/ik_output',
            10
        )

        # ------------ ROBOT PARAMETERS (MATCH FK!) ------------
        self.L0 = 36.076
        self.L1 = 60.25
        self.L2 = 130.23
        self.L3 = 124.0
        self.L4 = 133.4

        self.offset = math.radians(79.38)

        self.get_logger().info("IK Subscriber Node Started...")

    # ----------------------------------------------------------
    #                   IK CALLBACK
    # ----------------------------------------------------------
    def ik_callback(self, msg):

        # -----------------------------
        # 1) Extract EE pose
        # -----------------------------
        x4 = msg.position.x
        y4 = msg.position.y
        z4 = msg.position.z

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        self.get_logger().info(f"x, y, z:\n{x4}, {y4}, {z4}, \nqx, qy, qz, qw:\n{qx}, {qy}, {qz}, {qw}")

        # -----------------------------
        # 2) Convert quaternion → R
        # -----------------------------
        R = np.array([
            [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),       1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
        ])

        p = np.array([x4, y4, z4])

        # -----------------------------
        # 3) Compute wrist center
        # -----------------------------
        tool_z = R[:,1]          # FIXED: correct tool axis from URDF
        p3 = p - tool_z * self.L4
        # p3 = p - R[:,2] * self.L4   # subtract tool length along EE z-axis

        x3, y3, z3 = p3

        # -----------------------------
        # 4) Solve q1
        # -----------------------------
        q1 = math.atan2(y3, x3)

        # -----------------------------
        # 5) Planar projection for q2/q3
        # -----------------------------
        r = math.sqrt(x3**2 + y3**2) - self.L1
        s = z3 - self.L0

        d2 = r*r + s*s
        d = math.sqrt(d2)

        # -----------------------------
        # 6) Solve q3_actual
        # -----------------------------
        cos_q3 = (d2 - self.L2**2 - self.L3**2) / (2*self.L2*self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))  # clamp
        q3_actual = math.acos(cos_q3)

        # -----------------------------
        # 7) Solve q2_actual
        # -----------------------------
        beta  = math.atan2(s, r)

        val = (self.L2**2 + d2 - self.L3**2) / (2*self.L2*d)
        val = max(-1.0, min(1.0, val))
        gamma = math.acos(val)

        # gamma = math.acos(
        #     (self.L2**2 + d2 - self.L3**2) / (2*self.L2*d)
        # )
        q2_actual = beta - gamma

        # -----------------------------
        # 8) Convert into FK angles
        # -----------------------------
        q2 = q2_actual + self.offset
        q3 = q3_actual - self.offset

        # -----------------------------
        # 9) Solve q4 (rotation of last link)
        # -----------------------------
        # From R = Rz(q1)*Rz(q2_actual)*Rz(q3_actual)*Rz(q4)
        q234 = math.atan2(R[1,2], R[0,2])
        q4 = q234 - q2_actual - q3_actual

        # -----------------------------
        # 10) Publish
        # -----------------------------
        out = Float32MultiArray()
        out.data = [
            math.degrees(q1),
            math.degrees(q2),
            math.degrees(q3),
            math.degrees(q4)
        ]


        self.joint_pub.publish(out)


        # self.get_logger().info(
        #     f"IK -> q1={out.data[0]:.2f}, q2={out.data[1]:.2f}, "
        #     f"x, y, z:\n{x4}, {y4}, {z4}, \nqx, qy, qz, qw:\n{qx}, {qy}, {qz}, {qw}",
        #     f"q3={out.data[2]:.2f}, q4={out.data[3]:.2f}"
        # )

        self.get_logger().info(
            f"IK → q1={out.data[0]:.2f}, q2={out.data[1]:.2f}, "
            f"q3={out.data[2]:.2f}, q4={out.data[3]:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
