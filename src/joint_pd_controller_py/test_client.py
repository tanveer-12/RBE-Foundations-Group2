#!/usr/bin/env python3

"""
Test client for PD controller - matches team member's interface
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from open_manipulator_msgs.srv import SetJointPosition
import sys
import time

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        
        # Create clients for services
        self.set_position_client = self.create_client(
            SetJointPosition,
            'set_actuator4_position'
        )
        
        self.start_logging_client = self.create_client(
            SetBool,
            'start_logging'
        )
        
        # Wait for services
        while not self.set_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_actuator4_position service...')
        
        while not self.start_logging_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for start_logging service...')
    
    def start_log(self):
        """Start data logging"""
        request = SetBool.Request()
        request.data = True
        future = self.start_logging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info('Logging started successfully')
        else:
            self.get_logger().error(f'Failed to start logging: {response.message}')
    
    def stop_log(self):
        """Stop data logging"""
        request = SetBool.Request()
        request.data = False
        future = self.start_logging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info('Logging stopped successfully')
        else:
            self.get_logger().error(f'Failed to stop logging: {response.message}')
    
    def set_position(self, position):
        """Set joint4 position"""
        request = SetJointPosition.Request()
        request.joint_position.joint_name = ['joint4']
        request.joint_position.position = [position]
        request.path_time = 0.01  # Small path time
        
        future = self.set_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.is_moving:
            self.get_logger().info(f'Position set to {position} rad successfully')
        else:
            self.get_logger().error(f'Failed to set position: {response.message}')


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_client.py <command> [args]")
        print("Commands: start_log, stop_log, position <value>")
        return
    
    command = sys.argv[1]
    
    rclpy.init()
    client = TestClient()
    
    if command == 'start_log':
        client.start_log()
    elif command == 'stop_log':
        client.stop_log()
    elif command == 'position':
        if len(sys.argv) < 3:
            print("Usage: python3 test_client.py position <value>")
            return
        try:
            position = float(sys.argv[2])
            client.set_position(position)
        except ValueError:
            print("Invalid position value")
    else:
        print(f"Unknown command: {command}")
        print("Commands: start_log, stop_log, position <value>")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()