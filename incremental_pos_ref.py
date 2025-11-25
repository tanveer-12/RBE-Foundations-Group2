"""
================================================================================
INCREMENTAL POSITION CONTROLLER - Velocity Control via Position References
================================================================================

PROJECT: 4-DOF Robotic Manipulator Velocity Control
FRAMEWORK: ROS2 (Robot Operating System 2)
LANGUAGE: Python 3

TEAM MEMBERS (GROUP 2):
    - Rajdeep Banerjee
    - John Hoang Do
    - Hunter Sanford
    - Tanveer Kaur

================================================================================
EXECUTIVE SUMMARY
================================================================================

This module implements VELOCITY CONTROL through INCREMENTAL POSITION UPDATES.
Rather than sending velocity commands directly, it integrates desired velocities
into position references and sends these to the robot's position controllers.

KEY CONCEPT:
    Position-controlled robots (like OpenManipulator-X) don't accept velocity
    commands directly. Instead, we use numerical integration to convert velocities
    into small position increments, creating the illusion of velocity control.

MATHEMATICAL FOUNDATION:
    q_ref = q_ref_old + delta_q · delta_t

    where:
        q_ref_old = current position reference [rad]
        delta_q = desired joint velocities [rad/s]
        delta_t = sampling time / control period [s]
        q_ref = next position reference [rad]

INTEGRATION WITH JACOBIAN:
    1. User specifies desired EE velocity: V_desired
    2. Jacobian service computes: delta_q = J⁺(q) · V_desired
    3. This node integrates: q_ref = q_ref_old + delta_q · delta_t
    4. Send q_ref_new to position controllers
    5. Robot moves, creating velocity-like behavior

================================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import time

# Import service for Jacobian inverse kinematics
from openmx_interfaces.srv import EEToJointVelocity

from open_manipulator_msgs.srv import SetJointPosition


class IncrementalPositionController(Node):
    """
    Velocity controller using incremental position updates.
    
    This node implements velocity control for position-controlled robots by:
    1. Receiving desired Cartesian velocities (Twist messages)
    2. Computing required joint velocities via Jacobian inverse
    3. Integrating velocities to create position references
    4. Sending position references to robot at fixed rate
    
    Mathematical operation:
        q_ref(t + Δt) = q_ref(t) + q̇_desired · Δt
    
    This creates smooth velocity-like motion on position-controlled hardware.
    """

    def __init__(self):
        super().__init__('incremental_position_controller')
        
        # ====================================================================
        # Control Parameters
        # ====================================================================
        
        # Control loop frequency (Hz)
        self.declare_parameter('control_rate', 100.0)
        self.control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.control_rate  # Sampling time
        
        # Safety limits
        self.declare_parameter('max_joint_velocity', 2.0)  # rad/s
        self.declare_parameter('max_position_increment', 0.05)  # rad per step
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_position_increment = self.get_parameter('max_position_increment').value
        
        # Velocity scaling (safety factor)
        self.declare_parameter('velocity_scale', 1.0)
        self.velocity_scale = self.get_parameter('velocity_scale').value
        
        # Joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # ====================================================================
        # State Variables
        # ====================================================================
        
        # Current joint state from robot
        self.current_q = None  # Current joint positions [rad]
        self.current_q_dot = None  # Current joint velocities [rad/s]
        
        # Position reference (what we command to robot)
        self.q_ref = None  # Reference positions [rad]
        
        # Desired velocity (from user commands)
        self.desired_ee_velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.desired_joint_velocity = np.zeros(4)  # [θ̇1, θ̇2, θ̇3, θ̇4]
        
        # Control mode
        self.control_active = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 1.0  # Stop if no command for 1 second
        
        # ====================================================================
        # ROS2 Communication
        # ====================================================================
        
        # Subscribe to joint states (robot feedback)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to desired Cartesian velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_cartesian',
            self.cmd_vel_callback,
            10
        )
        
        # Service client for Jacobian inverse kinematics
        self.jacobian_client = self.create_client(
            EEToJointVelocity,
            'ee_to_joint_velocity'
        )
        
        # Wait for Jacobian service
        while not self.jacobian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Jacobian service...')
        
        # Setup robot command interface (varies by robot)
        self.setup_robot_interface()
        
        # Control timer - runs at fixed rate
        self.control_timer = self.create_timer(
            self.dt,
            self.control_loop
        )
        
        # ====================================================================
        # Logging
        # ====================================================================
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Incremental Position Controller Started')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Control rate: {self.control_rate} Hz (delta_t = {self.dt*1000:.1f} ms)')
        self.get_logger().info(f'Max joint velocity: {self.max_joint_velocity} rad/s')
        self.get_logger().info(f'Max position increment: {self.max_position_increment} rad')
        self.get_logger().info(f'Velocity scale: {self.velocity_scale}')
        self.get_logger().info('')
        self.get_logger().info('Theory: q_ref = q_ref_old + delta_q · delta_t')
        self.get_logger().info('        Converts velocities to position increments')
        self.get_logger().info('')
        self.get_logger().info('Send Twist messages to /cmd_vel_cartesian to control robot')
        self.get_logger().info('=' * 70)

    def setup_robot_interface(self):
        """
        Setup interface for sending commands to robot.
        Different robots have different interfaces.
        """
        if USING_OPEN_MANIPULATOR:
            # OpenManipulator-X uses SetJointPosition service
            self.robot_cmd_client = self.create_client(
                SetJointPosition,
                'goal_joint_space_path'
            )
            
            self.get_logger().info('Using OpenManipulator SetJointPosition service')
            
            while not self.robot_cmd_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for robot command service...')
        else:
            # Generic robots use JointTrajectory messages
            self.robot_cmd_pub = self.create_publisher(
                JointTrajectory,
                '/joint_trajectory_controller/joint_trajectory',
                10
            )
            
            self.get_logger().info('Using JointTrajectory topic')

    def joint_state_callback(self, msg):
        """
        Callback to receive current robot state.
        
        Initializes q_ref on first reception.
        """
        try:
            if len(msg.position) >= 4:
                self.current_q = np.array(list(msg.position[:4]))
                
                # Initialize reference to current position
                if self.q_ref is None:
                    self.q_ref = self.current_q.copy()
                    self.get_logger().info(
                        f'Initialized q_ref to current position: '
                        f'{np.array2string(self.q_ref, precision=3)}'
                    )
            
            if len(msg.velocity) >= 4:
                self.current_q_dot = np.array(list(msg.velocity[:4]))
                
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {str(e)}')

    def cmd_vel_callback(self, msg):
        """
        Callback to receive desired Cartesian velocity commands.
        
        Args:
            msg (Twist): Desired end-effector velocity
                         linear: [vx, vy, vz] m/s
                         angular: [wx, wy, wz] rad/s
        """
        # Store desired EE velocity
        self.desired_ee_velocity = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ])
        
        # Apply velocity scaling
        self.desired_ee_velocity *= self.velocity_scale
        
        # Update command timestamp
        self.last_cmd_time = self.get_clock().now()
        
        # Activate control if we have valid state
        if self.current_q is not None and self.q_ref is not None:
            self.control_active = True
        
        # Log command
        if np.linalg.norm(self.desired_ee_velocity) > 1e-6:
            self.get_logger().info(
                f'Received velocity command: '
                f'linear=[{msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}] m/s, '
                f'angular=[{msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f}] rad/s'
            )

    def compute_joint_velocities(self):
        """
        Compute desired joint velocities from desired EE velocity.
        
        Uses Jacobian inverse kinematics service:
            q_dot = J⁺(q) · V
        
        Returns:
            np.array: Desired joint velocities [rad/s] or None if failed
        """
        # Check if velocity is near zero
        if np.linalg.norm(self.desired_ee_velocity) < 1e-6:
            return np.zeros(4)
        
        # Call Jacobian service
        request = EEToJointVelocity.Request()
        
        # Use current reference position for Jacobian computation
        request.q1, request.q2, request.q3, request.q4 = self.q_ref
        request.vx, request.vy, request.vz = self.desired_ee_velocity[:3]
        request.wx, request.wy, request.wz = self.desired_ee_velocity[3:]
        
        try:
            # Synchronous call (blocks until response)
            future = self.jacobian_client.call_async(request)
            
            # Wait for result with timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.01)
            
            if future.done():
                response = future.result()
                
                if response.success:
                    q_dot = np.array([
                        response.q1_dot,
                        response.q2_dot,
                        response.q3_dot,
                        response.q4_dot
                    ])
                    
                    return q_dot
                else:
                    self.get_logger().warn(f'Jacobian failed: {response.message}')
                    return None
            else:
                self.get_logger().warn('Jacobian service call timeout')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling Jacobian service: {str(e)}')
            return None

    def apply_safety_limits(self, q_dot):
        """
        Apply safety limits to joint velocities.
        
        Args:
            q_dot: Desired joint velocities [rad/s]
            
        Returns:
            np.array: Limited joint velocities [rad/s]
        """
        # Limit individual joint velocities
        q_dot_limited = np.clip(
            q_dot,
            -self.max_joint_velocity,
            self.max_joint_velocity
        )
        
        # Limit position increment per step
        delta_q = q_dot_limited * self.dt
        delta_q_limited = np.clip(
            delta_q,
            -self.max_position_increment,
            self.max_position_increment
        )
        
        # Recompute velocity from limited increment
        q_dot_final = delta_q_limited / self.dt
        
        # Log if limits applied
        if not np.allclose(q_dot, q_dot_final):
            self.get_logger().warn(
                f'Applied velocity limits: '
                f'{np.array2string(q_dot, precision=3)} → '
                f'{np.array2string(q_dot_final, precision=3)}'
            )
        
        return q_dot_final

    def integrate_velocity(self, q_dot):
        """
        Integrate joint velocities to compute new position reference.
        
        Implements: q_ref = q_ref_old + delta_q · delta_t
        
        This is the CORE of velocity control via position updates!
        
        Args:
            q_dot: Joint velocities [rad/s]
            
        Returns:
            np.array: New position reference [rad]
        """
        # Apply safety limits
        q_dot_safe = self.apply_safety_limits(q_dot)
        
        # Euler forward integration
        # This is where velocity becomes position!
        delta_q = q_dot_safe * self.dt
        q_ref_new = self.q_ref + delta_q
        
        # Log integration
        self.get_logger().debug(
            f'Integration: q_ref += q_dot·Δt\n'
            f'  q_dot = {np.array2string(q_dot_safe, precision=4)} rad/s\n'
            f'  delta_t = {self.dt:.4f} s\n'
            f'  delta_q = {np.array2string(delta_q, precision=4)} rad\n'
            f'  q_ref: {np.array2string(self.q_ref, precision=4)} → '
            f'{np.array2string(q_ref_new, precision=4)}'
        )
        
        return q_ref_new

    def send_position_command(self, q_ref):
        """
        Send position reference to robot.
        
        Args:
            q_ref: Desired joint positions [rad]
        """
        if USING_OPEN_MANIPULATOR:
            # Use OpenManipulator service
            request = SetJointPosition.Request()
            request.joint_position.joint_name = self.joint_names
            request.joint_position.position = q_ref.tolist()
            request.path_time = self.dt * 2.0  # Give robot 2 cycles to reach target
            
            # Async call (non-blocking)
            self.robot_cmd_client.call_async(request)
            
        else:
            # Use JointTrajectory message
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = q_ref.tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(self.dt * 1e9)
            
            traj_msg.points = [point]
            self.robot_cmd_pub.publish(traj_msg)

    def control_loop(self):
        """
        Main control loop - called at fixed rate.
        
        Implements the complete velocity control pipeline:
        1. Check if control should be active
        2. Compute joint velocities via Jacobian
        3. Integrate to get position reference
        4. Send to robot
        """
        # Check if we have valid state
        if self.current_q is None or self.q_ref is None:
            return
        
        # Check command timeout
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout:
            if self.control_active:
                self.get_logger().info('Command timeout - stopping')
                self.control_active = False
                self.desired_ee_velocity = np.zeros(6)
        
        # If not active, just track current position
        if not self.control_active:
            # Update reference to current position (prevents jumps on reactivation)
            self.q_ref = self.current_q.copy()
            return
        
        # ====================================================================
        # VELOCITY CONTROL PIPELINE
        # ====================================================================
        
        # Step 1: Compute joint velocities from desired EE velocity
        #         q_dot = J⁺(q) · V
        q_dot_desired = self.compute_joint_velocities()
        
        if q_dot_desired is None:
            # Jacobian failed - stop motion
            self.get_logger().error('Failed to compute joint velocities - stopping')
            self.desired_ee_velocity = np.zeros(6)
            self.control_active = False
            return
        
        # Store for monitoring
        self.desired_joint_velocity = q_dot_desired
        
        # Step 2: Integrate velocities to position reference
        #         q_ref = q_ref_old + delta_q · delta_t
        q_ref_new = self.integrate_velocity(q_dot_desired)
        
        # Step 3: Update reference
        self.q_ref = q_ref_new
        
        # Step 4: Send to robot
        self.send_position_command(self.q_ref)
        
        # ====================================================================
        # MONITORING
        # ====================================================================
        
        # Compute tracking error
        position_error = np.linalg.norm(self.q_ref - self.current_q)
        
        # Log periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 50 == 0:  # Every 1 second at 50 Hz
            self.get_logger().info(
                f'Control active | '
                f'q_dot=[{self.desired_joint_velocity[0]:.3f}, '
                f'{self.desired_joint_velocity[1]:.3f}, '
                f'{self.desired_joint_velocity[2]:.3f}, '
                f'{self.desired_joint_velocity[3]:.3f}] rad/s | '
                f'error={position_error:.4f} rad'
            )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        controller = IncrementalPositionController()
        
        # Spin to handle callbacks
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        pass
    
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()