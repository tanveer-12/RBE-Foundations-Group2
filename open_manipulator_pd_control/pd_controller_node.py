#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
import numpy as np
import time
from datetime import datetime

# Custom service for setting reference position
from open_manipulator_msgs.srv import SetJointPosition


class PDControllerNode(Node):
    def __init__(self):
        super().__init__('pd_controller_node')
        
        # Declare parameters for PD gains
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('control_frequency', 100.0)  # Hz
        self.declare_parameter('actuator_index', 3)  # Index 3 = Actuator 4 (0-indexed)
        
        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.actuator_idx = self.get_parameter('actuator_index').value
        
        # Control variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.reference_position = 0.0
        self.previous_position = 0.0
        self.control_active = False
        
        # Data logging
        self.logging_active = False
        self.log_data = []
        self.start_time = None
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Effort publisher (for actuator 4)
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        
        # Service to set reference position
        self.set_position_srv = self.create_service(
            SetJointPosition,
            'set_actuator4_position',
            self.set_position_callback
        )
        
        # Service to start/stop logging
        self.start_logging_srv = self.create_service(
            SetBool,
            'start_logging',
            self.start_logging_callback
        )
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_loop
        )
        
        self.get_logger().info('PD Controller Node initialized')
        self.get_logger().info(f'Kp: {self.kp}, Kd: {self.kd}')
        self.get_logger().info(f'Control frequency: {self.control_freq} Hz')
        
    def joint_state_callback(self, msg):
        """Callback to receive joint states from the robot"""
        try:
            # Assuming joint_states message has joints in order
            # joint_1, joint_2, joint_3, joint_4, gripper
            if len(msg.position) > self.actuator_idx:
                self.previous_position = self.current_position
                self.current_position = msg.position[self.actuator_idx]
                
                # Calculate velocity (numerical differentiation)
                if len(msg.velocity) > self.actuator_idx:
                    self.current_velocity = msg.velocity[self.actuator_idx]
                else:
                    # Estimate velocity if not provided
                    dt = 1.0 / self.control_freq
                    self.current_velocity = (self.current_position - self.previous_position) / dt
                    
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {str(e)}')
    
    def set_position_callback(self, request, response):
        """Service callback to set reference position for actuator 4"""
        self.reference_position = request.position
        self.control_active = True
        response.success = True
        response.message = f'Reference position set to {self.reference_position} rad'
        self.get_logger().info(response.message)
        return response
    
    def start_logging_callback(self, request, response):
        """Service callback to start/stop data logging"""
        if request.data:
            # Start logging
            self.logging_active = True
            self.log_data = []
            self.start_time = time.time()
            response.success = True
            response.message = 'Data logging started'
            self.get_logger().info('Data logging started')
        else:
            # Stop logging and save data
            self.logging_active = False
            self.save_log_data()
            response.success = True
            response.message = f'Data logging stopped. Saved {len(self.log_data)} samples'
            self.get_logger().info(response.message)
        
        return response
    
    def control_loop(self):
        """Main PD control loop"""
        if not self.control_active:
            return
        
        # Calculate error
        position_error = self.reference_position - self.current_position
        
        # PD control law: u = Kp * e + Kd * de/dt
        # Velocity error (derivative term)
        velocity_error = -self.current_velocity  # Target velocity is 0
        
        # Calculate control effort
        effort = self.kp * position_error + self.kd * velocity_error
        
        # Saturation limits (adjust based on your robot's specifications)
        max_effort = 2.0  # Maximum current/effort in appropriate units
        effort = np.clip(effort, -max_effort, max_effort)
        
        # Publish effort command
        effort_msg = Float64MultiArray()
        # Send effort only to actuator 4, others will maintain their positions
        effort_msg.data = [0.0, 0.0, 0.0, effort, 0.0]  # 5 actuators total
        self.effort_pub.publish(effort_msg)
        
        # Log data if logging is active
        if self.logging_active and self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            self.log_data.append({
                'time': elapsed_time,
                'reference': self.reference_position,
                'current': self.current_position,
                'error': position_error,
                'effort': effort
            })
        
        # Optional: Print status periodically
        # if len(self.log_data) % 100 == 0:
        #     self.get_logger().info(
        #         f'Error: {position_error:.4f} rad, Effort: {effort:.4f}'
        #     )
    
    def save_log_data(self):
        """Save logged data to a text file"""
        if not self.log_data:
            self.get_logger().warn('No data to save')
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'pd_control_log_{timestamp}.txt'
        
        with open(filename, 'w') as f:
            # Write header
            f.write('# PD Controller Data Log\n')
            f.write(f'# Kp: {self.kp}, Kd: {self.kd}\n')
            f.write('# Time(s), Reference(rad), Current(rad), Error(rad), Effort\n')
            
            # Write data
            for data_point in self.log_data:
                f.write(f"{data_point['time']:.4f}, "
                       f"{data_point['reference']:.6f}, "
                       f"{data_point['current']:.6f}, "
                       f"{data_point['error']:.6f}, "
                       f"{data_point['effort']:.6f}\n")
        
        self.get_logger().info(f'Data saved to {filename}')
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.logging_active:
            self.save_log_data()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PDControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
