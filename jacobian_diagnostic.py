#!/usr/bin/env python3
"""
================================================================================
JACOBIAN DIRECTION DIAGNOSTIC TOOL
================================================================================

This script tests the Jacobian velocity mapping to identify direction errors.

Tests performed:
1. Pure +X velocity command → Expected robot motion
2. Pure +Y velocity command → Expected robot motion  
3. Pure +Z velocity command → Expected robot motion
4. Combined motions

This helps identify if there's a sign error, axis swap, or frame mismatch.
================================================================================
"""

import rclpy
from rclpy.node import Node
from openmx_interfaces.srv import EEToJointVelocity
from geometry_msgs.msg import Twist
import time

class JacobianDiagnostic(Node):
    def __init__(self):
        super().__init__('jacobian_diagnostic')
        
        # Service client
        self.jacobian_client = self.create_client(
            EEToJointVelocity,
            'ee_to_joint_velocity'
        )
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_cartesian',
            10
        )
        
        # Wait for service
        while not self.jacobian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Jacobian service...')
        
        self.get_logger().info('Jacobian Diagnostic Tool Ready')
        self.get_logger().info('=' * 70)

    def test_direction(self, name, vx, vy, vz, wx, wy, wz, expected_motion):
        """Test a specific velocity command and report results."""
        
        self.get_logger().info('')
        self.get_logger().info(f'TEST: {name}')
        self.get_logger().info(f'  Command: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f} m/s')
        self.get_logger().info(f'  Expected motion: {expected_motion}')
        
        # Call Jacobian service at home position
        request = EEToJointVelocity.Request()
        request.q1 = 0.0
        request.q2 = 0.0
        request.q3 = 0.0
        request.q4 = 0.0
        request.vx = vx
        request.vy = vy
        request.vz = vz
        request.wx = wx
        request.wy = wy
        request.wz = wz
        
        future = self.jacobian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'  Joint velocities: '
                    f'q1_dot={response.q1_dot:7.4f}, '
                    f'q2_dot={response.q2_dot:7.4f}, '
                    f'q3_dot={response.q3_dot:7.4f}, '
                    f'q4_dot={response.q4_dot:7.4f} rad/s'
                )
                
                # Send command to robot
                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.linear.z = vz
                twist.angular.x = wx
                twist.angular.y = wy
                twist.angular.z = wz
                
                self.get_logger().info(f'  Publishing command...')
                for _ in range(3):
                    self.cmd_pub.publish(twist)
                    time.sleep(0.1)
                
                self.get_logger().info(f'  OBSERVE ROBOT MOTION NOW!')
                self.get_logger().info(f'  Does it match expected motion? {expected_motion}')
                time.sleep(3.0)  # Wait for observation
                
                # Stop
                twist = Twist()
                self.cmd_pub.publish(twist)
                
            else:
                self.get_logger().error(f'  Jacobian failed: {response.message}')
        else:
            self.get_logger().error(f'  Service call timeout')

    def run_diagnostic(self):
        """Run complete diagnostic test sequence."""
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('STARTING JACOBIAN DIRECTION DIAGNOSTIC')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('IMPORTANT: Watch the robot carefully for each test!')
        self.get_logger().info('Compare actual motion to expected motion.')
        self.get_logger().info('')
        
        input('Press ENTER to start tests...')
        
        # Test 1: +X direction (forward from robot's perspective)
        self.test_direction(
            name='Pure +X velocity',
            vx=0.05, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0,
            expected_motion='Move FORWARD (away from base)'
        )
        
        # Test 2: -X direction (backward)
        self.test_direction(
            name='Pure -X velocity',
            vx=-0.05, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0,
            expected_motion='Move BACKWARD (toward base)'
        )
        
        # Test 3: +Y direction (left from robot's perspective)
        self.test_direction(
            name='Pure +Y velocity',
            vx=0.0, vy=0.05, vz=0.0, wx=0.0, wy=0.0, wz=0.0,
            expected_motion='Move LEFT (robot\'s left)'
        )
        
        # Test 4: -Y direction (right)
        self.test_direction(
            name='Pure -Y velocity',
            vx=0.0, vy=-0.05, vz=0.0, wx=0.0, wy=0.0, wz=0.0,
            expected_motion='Move RIGHT (robot\'s right)'
        )
        
        # Test 5: +Z direction (up)
        self.test_direction(
            name='Pure +Z velocity',
            vx=0.0, vy=0.0, vz=0.05, wx=0.0, wy=0.0, wz=0.0,
            expected_motion='Move UP (away from ground)'
        )
        
        # Test 6: -Z direction (down)
        self.test_direction(
            name='Pure -Z velocity',
            vx=0.0, vy=0.0, vz=-0.05, wx=0.0, wy=0.0, wz=0.0,
            expected_motion='Move DOWN (toward ground)'
        )
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('DIAGNOSTIC COMPLETE')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('ANALYSIS:')
        self.get_logger().info('If any direction is WRONG, check for:')
        self.get_logger().info('  1. Sign error in Jacobian computation')
        self.get_logger().info('  2. Axis swap (X<->Y, Y<->Z, etc.)')
        self.get_logger().info('  3. Frame convention mismatch')
        self.get_logger().info('  4. DH parameter errors')
        self.get_logger().info('')

def main():
    rclpy.init()
    
    diagnostic = JacobianDiagnostic()
    
    try:
        diagnostic.run_diagnostic()
    except KeyboardInterrupt:
        pass
    
    diagnostic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
