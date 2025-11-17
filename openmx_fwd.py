#import dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import numpy as np

try:
    from .openmx_params import get_robot_params
except ImportError:
    from openmx_params import get_robot_params


class RobotForwardKinematics(Node):

    def __init__(self):
        super().__init__('robot_fwd_kin')

        # Forward kinematics subscriber (original)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'topicFWD',
            self.listener_callback,
            10)
        self.subscription

        # Joint states subscriber (from basic_robot_control)
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.joint_states_subscription

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
        self.get_logger().info('Listening to joint_states and topicFWD')

    def listener_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().error("Need 4 input values in degrees (theta_1, theta_2, theta_3, theta_4)")
            return
        
        theta_1_deg, theta_2_deg, theta_3_deg, theta_4_deg = msg.data
        self.get_logger().info(
            f"Joint angles received (degrees): θ1={theta_1_deg:.2f}, θ2={theta_2_deg:.2f}, "
            f"θ3={theta_3_deg:.2f}, θ4={theta_4_deg:.2f}"
        )
        
        joint_angles_rad = [math.radians(theta_1_deg), 
                            math.radians(theta_2_deg), 
                            math.radians(theta_3_deg),
                            math.radians(theta_4_deg)]
        
        dh_par = self.build_dh_par(joint_angles_rad)
        T, A_matrices = self.end_effector_pose(dh_par)

        for i, A_i in enumerate(A_matrices, start=1):
            self.get_logger().info(f'Transformation A_{i}:\n{np.array2string(A_i, precision=4, suppress_small=True)}')

        pos = T[:3, 3]
        self.get_logger().info(f'End effector position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}')
        
        # For this robot, pitch = θ2 + θ3 + θ4
        pitch_deg = theta_2_deg + theta_3_deg + theta_4_deg
        
        self.get_logger().info(f'End effector pitch: {pitch_deg:.2f}°')
        
        # Pass the pitch angle to the publisher
        self.publish_end_effector_pose(T, math.radians(pitch_deg))

    def joint_states_callback(self, msg):
        """
        Callback for joint_states topic published by the robot controller
        Extracts the first 4 joint positions (joint1-joint4) and computes forward kinematics
        """
        if len(msg.position) < 4:
            self.get_logger().error(f"Expected at least 4 joint positions, got {len(msg.position)}")
            return
        
        # Extract first 4 joint angles (assuming they correspond to joint1-joint4)
        # joint_states typically publishes positions in radians
        joint_angles_rad = list(msg.position[:4])
        
        # Convert to degrees for logging
        joint_angles_deg = [math.degrees(angle) for angle in joint_angles_rad]
        
        self.get_logger().info(
            f"Joint states received (degrees): θ1={joint_angles_deg[0]:.2f}, θ2={joint_angles_deg[1]:.2f}, "
            f"θ3={joint_angles_deg[2]:.2f}, θ4={joint_angles_deg[3]:.2f}"
        )
        
        dh_par = self.build_dh_par(joint_angles_rad)
        T, A_matrices = self.end_effector_pose(dh_par)

        for i, A_i in enumerate(A_matrices, start=1):
            self.get_logger().info(f'Transformation A_{i}:\n{np.array2string(A_i, precision=4, suppress_small=True)}')

        pos = T[:3, 3]
        self.get_logger().info(f'End effector position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}')
        
        # For this robot, pitch = θ2 + θ3 + θ4
        pitch_rad = joint_angles_rad[1] + joint_angles_rad[2] + joint_angles_rad[3]
        pitch_deg = math.degrees(pitch_rad)
        
        self.get_logger().info(f'End effector pitch: {pitch_deg:.2f}°')
        
        # Pass the pitch angle to the publisher
        self.publish_end_effector_pose(T, pitch_rad)

    def build_dh_par(self, joint_angles_rad):
        theta_1, theta_2, theta_3, theta_4 = joint_angles_rad
        
        dh_par = [
            [0, math.pi/2.0, self.L0 + self.L1, theta_1],
            [self.L2, 0, 0, theta_2],
            [self.L3, 0, 0, theta_3],
            [self.L4, 0, 0, theta_4]
        ]
        return dh_par

    def dh_transform(self, a, alpha, d, theta):
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

    def end_effector_pose(self, dh_par):
        T = np.eye(4)
        A_matrices = []

        for a, alpha, d, theta in dh_par:
            A_i = self.dh_transform(a, alpha, d, theta)
            A_matrices.append(A_i)
            T = T @ A_i
        
        return T, A_matrices
    
    def publish_end_effector_pose(self, T, pitch_rad):
        """
        Publish end effector pose
        
        Args:
            T: 4x4 transformation matrix
            pitch_rad: pitch angle in radians (θ2 + θ3 + θ4)
        """
        pose_msg = Pose()
        
        # Set position
        pose_msg.position.x = float(T[0, 3])
        pose_msg.position.y = float(T[1, 3])
        pose_msg.position.z = float(T[2, 3])
        
        # For orientation, we encode the pitch directly
        # Since θ1 is base rotation and doesn't affect pitch,
        # we create a quaternion that represents:
        # - Rotation around Z by θ1 (from position x,y)
        # - Pitch angle = θ2 + θ3 + θ4
        
        # Extract θ1 from position
        theta_1 = math.atan2(pose_msg.position.y, pose_msg.position.x)
        
        # Create quaternion representing orientation
        # Roll = 0, Pitch = pitch_rad, Yaw = theta_1
        quaternion = self.euler_to_quaternion(0.0, pitch_rad, theta_1)
        
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]
        
        # Publish
        self.pose_publisher.publish(pose_msg)
        
        self.get_logger().info(
            f'Published end effector pose: '
            f'pos=[{pose_msg.position.x:.4f}, {pose_msg.position.y:.4f}, {pose_msg.position.z:.4f}], '
            f'pitch={math.degrees(pitch_rad):.2f}°, '
            f'quat=[{quaternion[0]:.4f}, {quaternion[1]:.4f}, {quaternion[2]:.4f}, {quaternion[3]:.4f}]'
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion [x, y, z, w]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]



def main(args=None):
    rclpy.init(args=args)
    openmx_fwd = RobotForwardKinematics()

    try:
        rclpy.spin(openmx_fwd)
    except KeyboardInterrupt:
        pass
    finally:
        openmx_fwd.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()