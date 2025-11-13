#import dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Twist
import math
import numpy as np

try:
    from .openmx_params import get_robot_params
except ImportError:
    from openmx_params import get_robot_params


class RobotForwardKinematics(Node):

    def __init__(self):     #class constructor
        super().__init__('robot_fwd_kin')

        #Forward kinematics subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'topicFWD',
            self.listener_callback,
            10)
        self.subscription  #prevent unused variable warning

        # Forward kinematics publisher for end effector pose
        self.pose_publisher = self.create_publisher(
            Pose,
            'end_effector_pose',
            10)
        
        # Get robot parameters
        params = get_robot_params()
        self.L0 = params['L0']
        self.L1 = params['L1']
        self.L2 = params['L2']
        self.L3 = params['L3']
        self.L4 = params['L4']

        self.get_logger().info('Forward Kinematics Node started')


#Forward Kinematics
    def listener_callback(self, msg):
        if len(msg.data) != 4:  #check for 4 values input. If not 4, shows error.
            self.get_logger().error("Need 4 input values in degrees (theta_1, theta_2, theta_3, theta_4)")
            return
        
        theta_1_deg, theta_2_deg, theta_3_deg, theta_4_deg = msg.data
        self.get_logger().info(
            f"Joint angles received (degrees): theta1 = {theta_1_deg}, theta2 = {theta_2_deg}, theta3 = {theta_3_deg}, theta4 = {theta_4_deg}."
            )
        joint_angles_rad = [math.radians(theta_1_deg), 
                            math.radians(theta_2_deg), 
                            math.radians(theta_3_deg),
                            math.radians(theta_4_deg)]      #convert joint angles from degrees to radians
        
        dh_par = self.build_dh_par(joint_angles_rad)        #run build_dh_par() function

        T, A_matrices = self.end_effector_pose(dh_par)      #run end_effector_pose() function

        for i, A_i in enumerate(A_matrices, start=1):
            self.get_logger().info(f'Transformation A_{i}: \n{A_i}')    #publish A_1, A_2, A_3 to console

        self.get_logger().info(f'End effector pose (Homegeneous transformation): \n{T}')    #publish T to console
        
        self.publish_end_effector_pose(T)


    def build_dh_par(self, joint_angles_rad):       #Define the DH parameters for this specific robot
        #Each row is [a, alpha, d, theta]
        
        theta_1, theta_2, theta_3, theta_4 = joint_angles_rad
        
        dh_par = [
            [0, math.pi/2.0, self.L0 + self.L1, theta_1],
            [self.L2, 0, 0, theta_2],
            [self.L3, 0, 0, theta_3],
            [self.L4, 0, 0, theta_4]
        ]

        return  dh_par


    def dh_transform(self, a, alpha, d, theta):     #function to calculate generic A_i from DH parameters
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


    def end_effector_pose(self, dh_par):        #function to calculate A_1, A_2, A_3, and T
        T = np.eye(4)
        A_matrices = []

        for i, (a, alpha, d, theta) in enumerate(dh_par, start=1):
            A_i = self.dh_transform(a, alpha, d, theta)
            A_matrices.append(A_i)
            T = T @ A_i
        
        return T, A_matrices
    
    def publish_end_effector_pose(self, T):
        
        pose_msg = Pose()
        
        # Set position
        pose_msg.position.x = float(T[0, 3])
        pose_msg.position.y = float(T[1, 3])
        pose_msg.position.z = float(T[2, 3])
        
        # Convert rotation to quaternion
        rotation_matrix = T[:3, :3]
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]
        
        # Publish
        self.pose_publisher.publish(pose_msg)
        
        self.get_logger().info(
            f'Published end effector pose: '
            f'pos=[{pose_msg.position.x:.4f}, {pose_msg.position.y:.4f}, {pose_msg.position.z:.4f}], '
            f'quat=[{quaternion[0]:.4f}, {quaternion[1]:.4f}, {quaternion[2]:.4f}, {quaternion[3]:.4f}]'
        )


    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a 3x3 rotation matrix to a quaternion [x, y, z, w]
        Uses the Shepperd method for numerical stability
        """
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]

    

def main(args=None):        #define main() function, initialize rclpy and this node
    rclpy.init(args=args)

    openmx_fwd = RobotForwardKinematics()

    rclpy.spin(openmx_fwd)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    openmx_fwd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
