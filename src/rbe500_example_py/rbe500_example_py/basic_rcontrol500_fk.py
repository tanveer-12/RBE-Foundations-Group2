import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


class FKNode(Node):
    def __init__(self):
        super().__init__('openmanipulator_fk')

        # --- SUBSCRIBE TO JOINT STATES ---
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        print(self.joint_sub)

        # --- PUBLISH END-EFFECTOR POSE ---
        self.pose_pub = self.create_publisher(Pose, '/fk_output', 10)

        self.get_logger().info("FK Node started. Waiting for /joint_states...")

    # ------------------------------------------------------------------
    # JOINT CALLBACK
    # ------------------------------------------------------------------
    def joint_callback(self, msg):
        """
        Joint order for OpenManipulator-X:
        ['joint1', 'joint2', 'joint3', 'joint4']
        """

        if len(msg.position) < 4:
            self.get_logger().warn("Received joint_states with < 4 joints!")
            return

        q1 = msg.position[0]
        q2 = msg.position[1]
        q3 = msg.position[2]
        q4 = msg.position[3]

        # --- Forward Kinematics (your matrices inserted directly) ---
        offset = math.radians(79.38)
        # offset = math.radians(0)

        A1 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 36.076],
            [0, 0, 0, 1]
        ])

        A2 = np.array([
            [math.cos(q1), 0, -math.sin(q1), 0],
            [math.sin(q1), 0,  math.cos(q1), 0],
            [0, -1, 0, 60.25],
            [0, 0, 0, 1]
        ])

        A3 = np.array([
            [math.cos(-offset+q2), -math.sin(-offset+q2), 0, 130.23*math.cos(-offset+q2)],
            [math.sin(-offset+q2),  math.cos(-offset+q2), 0, 130.23*math.sin(-offset+q2)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        A4 = np.array([
            [math.cos(offset+q3), -math.sin(offset+q3), 0, 124*math.cos(offset+q3)],
            [math.sin(offset+q3),  math.cos(offset+q3), 0, 124*math.sin(offset+q3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        A5 = np.array([
            [math.cos(q4), -math.sin(q4), 0, 133.4*math.cos(q4)],
            [math.sin(q4),  math.cos(q4), 0, 133.4*math.sin(q4)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        H = A1 @ A2 @ A3 @ A4 @ A5

        # Extract end-effector XYZ
        x = float(H[0, 3])
        y = float(H[1, 3])
        z = float(H[2, 3])

        # Extract rotation matrix
        R = H[0:3, 0:3]

        # Convert to quaternion
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

        # Publish Pose
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # # Publish Pose message
        # pose_msg = Pose()
        # pose_msg.position.x = x
        # pose_msg.position.y = y
        # pose_msg.position.z = z
        # pose_msg.orientation.qx = qx
        # pose_msg.orientation.qy = qy
        # pose_msg.orientation.qz = qz
        # pose_msg.orientation.w = w

        # self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f"EE Pose -> "
            f"x:{x:.2f}, y:{y:.2f}, z:{z:.2f},\n"
            f"qx:{qx:.4f}, qy:{qy:.4f}, qz:{qz:.4f}, qw:{qw:.4f}"
        )

    def rotation_matrix_to_quaternion(self, R):
        t = np.trace(R)
        if t > 0:
            S = math.sqrt(t + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
