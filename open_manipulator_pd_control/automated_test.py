#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from std_srvs.srv import SetBool
import time


class AutomatedTest(Node):
    def __init__(self):
        super().__init__('automated_test')
        
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
        self.position_client.wait_for_service(timeout_sec=10.0)
        self.logging_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services available!')
    
    def set_position(self, position):
        """Set reference position"""
        request = SetJointPosition.Request()
        request.position = position
        
        future = self.position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Set position: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def control_logging(self, start):
        """Start or stop data logging"""
        request = SetBool.Request()
        request.data = start
        
        future = self.logging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info(f'Logging: {future.result().message}')
    
    def run_test(self, test_positions, duration=10.0):
        """
        Run automated test for multiple reference positions
        
        Args:
            test_positions: List of reference positions in radians
            duration: Duration to log each test (seconds)
        """
        for idx, position in enumerate(test_positions):
            self.get_logger().info(f'\n{"="*50}')
            self.get_logger().info(f'Test {idx+1}/{len(test_positions)}: Position = {position} rad')
            self.get_logger().info(f'{"="*50}')
            
            # Set position
            if not self.set_position(position):
                self.get_logger().error(f'Failed to set position for test {idx+1}')
                continue
            
            # Small delay to let the system respond
            time.sleep(0.5)
            
            # Start logging
            self.control_logging(True)
            
            # Wait for specified duration
            self.get_logger().info(f'Logging data for {duration} seconds...')
            time.sleep(duration)
            
            # Stop logging
            self.control_logging(False)
            
            # Wait before next test
            if idx < len(test_positions) - 1:
                self.get_logger().info('Waiting 2 seconds before next test...')
                time.sleep(2.0)
        
        self.get_logger().info('\nAll tests completed!')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = AutomatedTest()
    
    # Define three different test positions (in radians)
    # Adjust these based on your robot's joint limits
    # For OpenManipulator-X joint 4, typical range is approximately [-1.8, 1.8] rad
    test_positions = [
        0.0,      # Test 1: Neutral position
        0.5,      # Test 2: Positive position (≈28.6°)
        -0.5      # Test 3: Negative position (≈-28.6°)
    ]
    
    test_node.get_logger().info('Starting automated PD controller test')
    test_node.get_logger().info(f'Test positions: {test_positions} rad')
    
    try:
        # Run test with 10 second duration per position
        test_node.run_test(test_positions, duration=10.0)
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
