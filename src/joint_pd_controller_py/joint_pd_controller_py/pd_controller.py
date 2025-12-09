#!/usr/bin/env python3

"""
PD Controller for Actuator 4 (joint4) on OpenMANIPULATOR-X
Implements a PD controller to control the position of Actuator 4 (joint4) using effort/current control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
from std_srvs.srv import SetBool
import time
from datetime import datetime
import math
import numpy as np

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')
        
        # Control variables
        self.current_positions = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}
        self.current_velocities = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}
        self.desired_position = 0.0
        self.kp = 0.1  # Start with small Kp as per assignment
        self.kd = 0.0  # Start with zero Kd as per assignment
        self.previous_position = 0.0
        self.control_active = False
        
        # OpenMANIPULATOR-X specific
        self.actuator_4_name = 'joint4'  # The actuator to control
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Data logging - 10 seconds with 0.01s sampling as per assignment
        self.logging_active = False
        self.log_data = []
        self.start_time = None
        
        # Create subscribers and services
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Service for setting position reference
        self.position_service = self.create_service(
            SetJointPosition,
            'set_actuator4_position',
            self.position_service_callback
        )
        
        # Service to start/stop logging (to match team member's interface)
        self.start_logging_srv = self.create_service(
            SetBool,
            'start_logging',
            self.start_logging_callback
        )
        
        # Timer for PD control loop (100Hz - same as OpenMANIPULATOR-X)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('PD Controller initialized for OpenMANIPULATOR-X joint4')
        self.get_logger().info(f'Initial Kp: {self.kp}, Kd: {self.kd}')

    def joint_state_callback(self, msg):
        """Callback for joint state messages - gets current position of all joints"""
        try:
            # Update all joint positions and velocities
            for i, name in enumerate(msg.name):
                if name in self.current_positions:
                    if name == self.actuator_4_name:
                        self.previous_position = self.current_positions[name]
                        self.current_positions[name] = msg.position[i]
                        
                        # Calculate velocity if available, otherwise estimate
                        if i < len(msg.velocity):
                            self.current_velocities[name] = msg.velocity[i]
                        else:
                            # Estimate velocity if not provided
                            dt = 0.01  # 100Hz control loop
                            if dt > 0:
                                self.current_velocities[name] = (self.current_positions[name] - self.previous_position) / dt
                    else:
                        self.current_positions[name] = msg.position[i]
                        if i < len(msg.velocity):
                            self.current_velocities[name] = msg.velocity[i]
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')

    def position_service_callback(self, request, response):
        """Callback for position service requests"""
        try:
            # Extract desired position for joint4 from the request
            for i, joint_name in enumerate(request.joint_name):
                if joint_name == self.actuator_4_name:
                    self.desired_position = request.position[i]
                    self.control_active = True
                    response.is_moving = True
                    response.message = f'Reference position set to {self.desired_position} rad'
                    self.get_logger().info(response.message)
                    return response
            
            self.get_logger().warn(
                f'Actuator {self.actuator_4_name} not found in request'
            )
            response.is_moving = False
            response.message = f'{self.actuator_4_name} not found in request'
            return response
        except Exception as e:
            self.get_logger().error(f'Error in position_service_callback: {e}')
            response.is_moving = False
            response.message = f'Error: {str(e)}'
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
        """Main control loop - runs at 100Hz"""
        if not self.control_active:
            return
        
        try:
            # Get current position and velocity of joint4
            current_pos = self.current_positions[self.actuator_4_name]
            current_vel = self.current_velocities[self.actuator_4_name]
            
            # Calculate error
            position_error = self.desired_position - current_pos
            
            # PD control law: u = Kp * e + Kd * de/dt
            # Velocity error (derivative term) - target velocity is 0
            velocity_error = -current_vel
            
            # Calculate control effort
            effort = self.kp * position_error + self.kd * velocity_error
            
            # Saturation limits
            max_effort = 2.0  # Maximum current/effort in appropriate units
            effort = np.clip(effort, -max_effort, max_effort)
            
            # Log data if logging is active
            if self.logging_active and self.start_time is not None:
                elapsed_time = time.time() - self.start_time
                self.log_data.append({
                    'time': elapsed_time,
                    'reference': self.desired_position,
                    'current': current_pos,
                    'error': position_error,
                    'effort': effort
                })
                
                # Print status every second for monitoring
                if len(self.log_data) % 100 == 0:
                    self.get_logger().info(
                        f'Time: {elapsed_time:.2f}s, Error: {position_error:.4f} rad, Effort: {effort:.4f}'
                    )
            
        except Exception as e:
            self.get_logger().error(f'Error in control_loop: {e}')

    def save_log_data(self):
        """Save logged data to a text file (matching team member's format)"""
        if not self.log_data:
            self.get_logger().warn('No data to save')
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'pd_control_log_{timestamp}.txt'
        
        with open(filename, 'w') as f:
            # Write header (matching team member's format)
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

    def set_gains(self, kp, kd):
        """Method to set PD gains externally"""
        self.kp = kp
        self.kd = kd
        self.get_logger().info(f'Updated gains - Kp: {self.kp}, Kd: {self.kd}')


def main():
    """Main function to run the PD controller"""
    rclpy.init()
    
    pd_controller = PDController()
    
    try:
        rclpy.spin(pd_controller)
    except KeyboardInterrupt:
        pd_controller.get_logger().info('PD Controller interrupted by user')
        # Save any remaining data if logging was active
        if pd_controller.logging_active:
            pd_controller.save_log_data()
    finally:
        pd_controller.destroy_node()
        rclpy.shutdown()