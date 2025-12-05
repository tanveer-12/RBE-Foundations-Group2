#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys

from openmx_interfaces.srv import InverseKinematics



class IKClient(Node):

    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(InverseKinematics, 'inverse_kinematics')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for inverse_kinematics service...')

    def send_request(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        request = InverseKinematics.Request()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        self.get_logger().info(f'Sending IK request: pos=[{x}, {y}, {z}]')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    ik_client = IKClient()
    
    # Parse command line arguments
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        # Default test values
        x, y, z = 0.2, 0.0, 0.3
        ik_client.get_logger().info(f'Using default position: [{x}, {y}, {z}]')
    
    response = ik_client.send_request(x, y, z)
    
    if response.success:
        ik_client.get_logger().info(f'\n{response.message}')
        ik_client.get_logger().info(
            f'Solution 1: theta1={response.theta1:.2f}°, theta2={response.theta2:.2f}°, '
            f'theta3={response.theta3:.2f}°, theta4={response.theta4:.2f}°'
        )
        ik_client.get_logger().info(
            f'Solution 2: theta1={response.theta1_alt:.2f}°, theta2={response.theta2_alt:.2f}°, '
            f'theta3={response.theta3_alt:.2f}°, theta4={response.theta4_alt:.2f}°'
        )
    else:
        ik_client.get_logger().error(f'IK failed: {response.message}')
    
    ik_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()