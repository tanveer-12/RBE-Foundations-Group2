import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray


class IKNode(Node):
    def __init__(self):
        super().__init__("openmanipulator_ik")

        # Subscribe to end-effector pose
        self.pose_sub = self.create_subscription(
            Pose,
            "/fk_output",
            self.pose_callback,
            10
        )

        # Publish joint angles
        self.joint_pub = self.create_publisher(
            Float32MultiArray,
            "/ik_output",
            10
        )

        self.get_logger().info("IK Node started. Waiting for /fk_output...")

    # -------------------------------------------------------------
    # CALLBACK: receives x,y,z → computes q1,q2,q3,q4
    # -------------------------------------------------------------
    def pose_callback(self, msg):

        # Extract EE position
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # ------------------------------
        # YOUR EXACT IK EQUATIONS
        # ------------------------------
        A = math.sqrt(x**2 + y**2)
        B = z + 37.074
        R1 = 130.23
        R2 = 124
        alpha = math.radians(10.62)
        M = math.sqrt(A**2 + B**2)
        K = (A**2 + B**2 + R1**2 - R2**2) / (2 * R1)
        phi = math.atan2(B, A)

        # IK formulas (your version)
        ratio = K / M
        ratio = max(-1.0, min(1.0, ratio))   # clamp for numerical stability

        u = math.asin(ratio) - phi
        
        # u = math.asin(K / M) - phi
        v = math.atan2(R1 * math.cos(u) - B, A - R1 * math.sin(u))

        q2 = u - alpha
        q3 = v - u + alpha
        q4 = math.radians(90) - q2 - q3
        q1 = math.atan2(y, x)

        # Publish joint angles (in degrees or radians? → using degrees since your FK expects degrees)
        out = Float32MultiArray()
        out.data = [
            math.degrees(q1),
            math.degrees(q2),
            math.degrees(q3),
            math.degrees(q4)
        ]

        self.joint_pub.publish(out)

        # Print for debugging
        self.get_logger().info(
            f"IK → q1:{math.degrees(q1):.2f}, q2:{math.degrees(q2):.2f}, "
            f"q3:{math.degrees(q3):.2f}, q4:{math.degrees(q4):.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
