#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from open_manipulator_pd_control.srv import SetJointPosition
import numpy as np
import time
from datetime import datetime

# Direct Dynamixel control
from dynamixel_sdk import *

class PDControllerDirect(Node):
    def __init__(self):
        super().__init__('pd_controller_direct')
        
        # Parameters
        self.declare_parameter('kp', 5.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('actuator_index', 3)
        
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
        
        # Initialize Dynamixel
        self.init_dynamixel()
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Services
        self.set_position_srv = self.create_service(
            SetJointPosition,
            'set_actuator4_position',
            self.set_position_callback
        )
        
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
        
        self.get_logger().info('PD Controller (Direct Dynamixel) initialized')
        self.get_logger().info(f'Kp: {self.kp}, Kd: {self.kd}')
        
    def init_dynamixel(self):
        """Initialize direct Dynamixel communication"""
        # Dynamixel settings
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 1000000
        self.PROTOCOL_VERSION = 2.0
        self.DXL_ID = 14  # Joint 4
        
        # Control table addresses
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_PRESENT_CURRENT = 126
        
        # Initialize port handler and packet handler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        # Open port
        if not self.portHandler.openPort():
            self.get_logger().error(f"Failed to open port {self.DEVICENAME}")
            raise Exception("Cannot open port")
        
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error(f"Failed to set baudrate to {self.BAUDRATE}")
            raise Exception("Cannot set baudrate")
        
        # Enable torque
        self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 1
        )
        
        self.get_logger().info(f"Dynamixel ID {self.DXL_ID} initialized")
    
    def joint_state_callback(self, msg):
        """Receive joint states"""
        try:
            if len(msg.position) > self.actuator_idx:
                self.previous_position = self.current_position
                self.current_position = msg.position[self.actuator_idx]
                
                if len(msg.velocity) > self.actuator_idx:
                    self.current_velocity = msg.velocity[self.actuator_idx]
                else:
                    dt = 1.0 / self.control_freq
                    self.current_velocity = (self.current_position - self.previous_position) / dt
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {str(e)}')
    
    def set_position_callback(self, request, response):
        """Service to set reference position"""
        self.reference_position = request.position
        self.control_active = True
        response.success = True
        response.message = f'Reference position set to {self.reference_position} rad'
        self.get_logger().info(response.message)
        return response
    
    def start_logging_callback(self, request, response):
        """Service to start/stop logging"""
        if request.data:
            self.logging_active = True
            self.log_data = []
            self.start_time = time.time()
            response.success = True
            response.message = 'Data logging started'
        else:
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
        velocity_error = -self.current_velocity
        
        # PD control law
        effort = self.kp * position_error + self.kd * velocity_error
        
        # Convert effort to Dynamixel current value
        # XM430 current range: approximately -1193 to 1193 (maps to ±2.69A)
        # Scale your effort (typically -2.0 to 2.0) to this range
        max_effort = 2.0
        effort = np.clip(effort, -max_effort, max_effort)
        
        # Convert to Dynamixel units (adjust scaling as needed)
        # This scaling may need tuning based on your robot
        dynamixel_current = int(effort * 300)  # Scale factor: adjust as needed
        dynamixel_current = np.clip(dynamixel_current, -1193, 1193)
        
        # Send current command to Dynamixel
        try:
            # Convert to unsigned 16-bit (Dynamixel uses 2's complement)
            if dynamixel_current < 0:
                dynamixel_current_unsigned = (1 << 16) + dynamixel_current
            else:
                dynamixel_current_unsigned = dynamixel_current
            
            self.packetHandler.write2ByteTxRx(
                self.portHandler,
                self.DXL_ID,
                self.ADDR_GOAL_CURRENT,
                dynamixel_current_unsigned
            )
        except Exception as e:
            self.get_logger().error(f'Error sending current command: {str(e)}')
        
        # Log data if active
        if self.logging_active and self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            self.log_data.append({
                'time': elapsed_time,
                'reference': self.reference_position,
                'current': self.current_position,
                'error': position_error,
                'effort': effort
            })
    
    def save_log_data(self):
        """Save logged data"""
        if not self.log_data:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'pd_control_log_{timestamp}.txt'
        
        with open(filename, 'w') as f:
            f.write('# PD Controller Data Log (Direct Dynamixel Control)\n')
            f.write(f'# Kp: {self.kp}, Kd: {self.kd}\n')
            f.write('# Time(s), Reference(rad), Current(rad), Error(rad), Effort\n')
            
            for data_point in self.log_data:
                f.write(f"{data_point['time']:.4f}, "
                       f"{data_point['reference']:.6f}, "
                       f"{data_point['current']:.6f}, "
                       f"{data_point['error']:.6f}, "
                       f"{data_point['effort']:.6f}\n")
        
        self.get_logger().info(f'Data saved to {filename}')
    
    def destroy_node(self):
        """Cleanup"""
        if self.logging_active:
            self.save_log_data()
        
        # Stop motor and close port
        try:
            self.packetHandler.write2ByteTxRx(
                self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, 0
            )
            self.portHandler.closePort()
        except:
            pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PDControllerDirect()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()