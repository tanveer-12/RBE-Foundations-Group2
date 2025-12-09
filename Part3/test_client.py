#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from open_manipulator_pd_control.srv import SetJointPosition
from std_srvs.srv import SetBool
import sys


class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        
        # Service clients
        self.position_client = self.create_client(
            SetJointPosition,
            'set_actuator4_position'
        )
        
        self.logging_client = self.create_client(
            SetBool,
            'start_logging'
        )
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.position_client.wait_for_service()
        self.logging_client.wait_for_service()
        self.get_logger().info('Services available!')
    
    def set_position(self, position):
        """Set reference position"""
        request = SetJointPosition.Request()
        request.position = position
        
        future = self.position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Response: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def start_logging(self):
        """Start data logging"""
        request = SetBool.Request()
        request.data = True
        
        future = self.logging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Logging: {future.result().message}')
    
    def stop_logging(self):
        """Stop data logging"""
        request = SetBool.Request()
        request.data = False
        
        future = self.logging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Logging: {future.result().message}')


def main(args=None):
    rclpy.init(args=args)
    
    client = TestClient()
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Test position: python3 test_client.py position <value_in_radians>")
        print("  Start logging: python3 test_client.py start_log")
        print("  Stop logging:  python3 test_client.py stop_log")
        return
    
    command = sys.argv[1]
    
    if command == 'position' and len(sys.argv) == 3:
        position = float(sys.argv[2])
        client.set_position(position)
    elif command == 'start_log':
        client.start_logging()
    elif command == 'stop_log':
        client.stop_logging()
    else:
        print("Invalid command")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
