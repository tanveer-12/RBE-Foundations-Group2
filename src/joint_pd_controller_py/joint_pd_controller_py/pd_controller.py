#!/usr/bin/env python3

"""
PD Controller for Actuator 4 (joint4) on OpenMANIPULATOR-X
Implements a PD controller to control the position of Actuator 4 (joint4) using effort/current control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
import time
import csv
from datetime import datetime
import math

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')
        
        # Control variables
        self.current_positions = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}
        self.desired_position = 0.0
        self.kp = 0.1  # Start with small Kp as per assignment
        self.kd = 0.0  # Start with zero Kd as per assignment
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        
        # OpenMANIPULATOR-X specific
        self.actuator_4_name = 'joint4'  # The actuator to control
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Data logging - 10 seconds with 0.01s sampling as per assignment
        self.log_start_time = self.get_clock().now().nanoseconds / 1e9
        self.log_duration = 10.0  # seconds
        self.log_filename = f'pd_controller_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.log_data = []
        
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
        
        # Client for sending joint position commands (this is how we "publish" effort)
        self.joint_position_client = self.create_client(
            SetJointPosition,
            '/goal_joint_space_path'
        )
        
        # Wait for service to be available
        while not self.joint_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /goal_joint_space_path service...')
        
        # Timer for PD control loop (100Hz - same as OpenMANIPULATOR-X)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        # Timer for data logging (100Hz to match sampling requirement of 0.01s)
        self.logging_timer = self.create_timer(0.01, self.log_data_callback)
        
        self.get_logger().info('PD Controller initialized for OpenMANIPULATOR-X joint4')
        self.get_logger().info(f'Initial Kp: {self.kp}, Kd: {self.kd}')
        self.get_logger().info(f'Controlling: {self.actuator_4_name}')
        self.get_logger().info(f'Log file: {self.log_filename}')
        self.get_logger().info('Logging data for 10 seconds at 0.01s intervals as per assignment')

    def joint_state_callback(self, msg):
        """Callback for joint state messages - gets current position of all joints"""
        try:
            # Update all joint positions
            for i, name in enumerate(msg.name):
                if name in self.current_positions:
                    self.current_positions[name] = msg.position[i]
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')

    def position_service_callback(self, request, response):
        """Callback for position service requests"""
        try:
            # Extract desired position for joint4 from the request
            for i, joint_name in enumerate(request.joint_name):
                if joint_name == self.actuator_4_name:
                    self.desired_position = request.position[i]
                    self.get_logger().info(
                        f'Set desired position for {self.actuator_4_name} to {self.desired_position}'
                    )
                    response.is_moving = True
                    return response
            
            self.get_logger().warn(
                f'Actuator {self.actuator_4_name} not found in request'
            )
            response.is_moving = False
            return response
        except Exception as e:
            self.get_logger().error(f'Error in position_service_callback: {e}')
            response.is_moving = False
            return response

    def compute_pd_control(self, current_pos, desired_pos):
        """Compute PD control effort"""
        try:
            # Calculate error
            error = desired_pos - current_pos
            
            # Get current time for derivative calculation
            current_time = self.get_clock().now()
            dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
            
            if dt <= 0:
                dt = 0.001  # Prevent division by zero
            
            # Calculate derivative of error
            error_derivative = (error - self.last_error) / dt
            
            # PD control law: u = Kp * error + Kd * error_derivative
            control_effort = self.kp * error + self.kd * error_derivative
            
            # Limit the control effort to prevent excessive forces
            control_effort = max(-0.1, min(0.1, control_effort))  # Small limit for safety
            
            # Update for next iteration
            self.last_error = error
            self.last_time = current_time
            
            return control_effort
        except Exception as e:
            self.get_logger().error(f'Error in compute_pd_control: {e}')
            return 0.0

    def control_loop(self):
        """Main control loop - runs at 100Hz"""
        try:
            # Get current position of joint4
            current_joint4_pos = self.current_positions[self.actuator_4_name]
            
            # Compute control effort using PD
            control_effort = self.compute_pd_control(current_joint4_pos, self.desired_position)
            
            # Calculate new desired position for joint4 based on PD control
            new_joint4_pos = current_joint4_pos + control_effort
            
            # Ensure the new position is within reasonable limits
            new_joint4_pos = max(-math.pi, min(math.pi, new_joint4_pos))
            
            # Prepare joint position command for ALL joints
            # Keep joints 1-3 at their current positions (maintain rigidly)
            # Only move joint4 based on our PD control
            from open_manipulator_msgs.msg import JointPosition
            joint_pos_msg = JointPosition()
            joint_pos_msg.joint_name = self.joint_names
            joint_pos_msg.position = [
                self.current_positions['joint1'],  # Keep joint1 at current position
                self.current_positions['joint2'],  # Keep joint2 at current position  
                self.current_positions['joint3'],  # Keep joint3 at current position
                new_joint4_pos                    # Move joint4 based on PD control
            ]
            
            # Create request for OpenMANIPULATOR-X service
            request = SetJointPosition.Request()
            request.joint_position = joint_pos_msg
            request.path_time = 0.01  # 10ms path time to match control frequency
            
            # Send the command to the OpenMANIPULATOR-X controller
            # This is how we "publish" our control effort to the robot
            future = self.joint_position_client.call_async(request)
            
        except Exception as e:
            self.get_logger().error(f'Error in control_loop: {e}')

    def log_data_callback(self):
        """Callback to save log data at 100Hz (0.01s sampling) for 10 seconds"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = current_time - self.log_start_time
            
            # Stop logging after 10 seconds as per assignment
            if elapsed_time > self.log_duration:
                self.get_logger().info('Completed 10 seconds of data logging')
                # Stop the logging timer
                self.logging_timer.cancel()
                return
            
            # Write to CSV file
            with open(self.log_filename, 'a', newline='') as csvfile:
                fieldnames = ['time', 'current_position', 'desired_position', 'error', 'effort_command']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header only if file is empty
                if csvfile.tell() == 0:
                    writer.writeheader()
                
                # Get current joint4 position
                current_joint4_pos = self.current_positions[self.actuator_4_name]
                
                # Calculate current control effort
                control_effort = self.compute_pd_control(current_joint4_pos, self.desired_position)
                
                # Write current data
                error = self.desired_position - current_joint4_pos
                
                data_point = {
                    'time': elapsed_time,
                    'current_position': current_joint4_pos,
                    'desired_position': self.desired_position,
                    'error': error,
                    'effort_command': control_effort
                }
                
                writer.writerow(data_point)
                self.log_data.append(data_point)
                
        except Exception as e:
            self.get_logger().error(f'Error in log_data_callback: {e}')

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
    finally:
        # Close log file
        if hasattr(pd_controller, 'log_filename') and pd_controller.log_filename:
            pd_controller.get_logger().info(f'Log saved to: {pd_controller.log_filename}')
        
        pd_controller.destroy_node()
        rclpy.shutdown()