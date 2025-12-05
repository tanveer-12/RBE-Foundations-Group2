#!/usr/bin/env python3
"""
================================================================================
CARTESIAN TRAJECTORY RECORDER AND PLOTTER
================================================================================

Records the robot's Cartesian position over time while commanding a velocity,
then plots the trajectory.

Usage:
    python3 record_trajectory.py
    
The script will:
1. Start recording positions from /end_effector_pose
2. Command +Y velocity (0.05 m/s) for specified duration
3. Save data to CSV
4. Generate plots (X vs t, Y vs t, Z vs t, and 3D trajectory)

================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import csv
from datetime import datetime


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        # Recording parameters
        self.declare_parameter('duration', 5.0)  # seconds
        self.declare_parameter('velocity_y', 0.05)  # m/s
        self.declare_parameter('output_file', 'trajectory_data.csv')
        
        self.duration = self.get_parameter('duration').value
        self.velocity_y = self.get_parameter('velocity_y').value
        self.output_file = self.get_parameter('output_file').value
        
        # Data storage
        self.timestamps = []  # Time in seconds
        self.positions_x = []
        self.positions_y = []
        self.positions_z = []
        
        self.start_time = None
        self.recording = False
        
        # Subscriber to end-effector pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/end_effector_pose',
            self.pose_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_cartesian',
            10
        )
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Cartesian Trajectory Recorder')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Duration: {self.duration} seconds')
        self.get_logger().info(f'Commanded Y velocity: {self.velocity_y} m/s')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info('=' * 70)
        
    def pose_callback(self, msg):
        """Record pose data during recording period."""
        if not self.recording:
            return
        
        # Record timestamp
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Check if duration exceeded
        if elapsed > self.duration:
            self.stop_recording()
            return
        
        # Record position
        self.timestamps.append(elapsed)
        self.positions_x.append(msg.position.x)
        self.positions_y.append(msg.position.y)
        self.positions_z.append(msg.position.z)
        
        # Log periodically
        if len(self.timestamps) % 10 == 0:
            self.get_logger().info(
                f't={elapsed:.2f}s: '
                f'x={msg.position.x:.4f}, '
                f'y={msg.position.y:.4f}, '
                f'z={msg.position.z:.4f}'
            )
    
    def start_recording(self):
        """Start recording trajectory."""
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('STARTING RECORDING')
        self.get_logger().info('=' * 70)
        
        self.recording = True
        self.start_time = time.time()
        
        # Start commanding velocity
        self.command_velocity()
        
    def stop_recording(self):
        """Stop recording and process data."""
        if not self.recording:
            return
            
        self.recording = False
        
        # Stop robot
        self.command_stop()
        
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('RECORDING COMPLETE')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Recorded {len(self.timestamps)} data points')
        
        # Save and plot
        self.save_data()
        self.plot_trajectory()
        
    def command_velocity(self):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = self.velocity_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # Publish continuously at 10 Hz
        self.cmd_timer = self.create_timer(0.1, lambda: self.cmd_pub.publish(twist))
        
        self.get_logger().info(f'Commanding +Y velocity: {self.velocity_y} m/s')
        
    def command_stop(self):
        """Stop the robot."""
        twist = Twist()  # All zeros
        self.cmd_pub.publish(twist)
        
        if hasattr(self, 'cmd_timer'):
            self.cmd_timer.cancel()
        
        self.get_logger().info('Robot stopped')
        
    def save_data(self):
        """Save trajectory data to CSV."""
        try:
            with open(self.output_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time (s)', 'X (m)', 'Y (m)', 'Z (m)'])
                
                for i in range(len(self.timestamps)):
                    writer.writerow([
                        self.timestamps[i],
                        self.positions_x[i],
                        self.positions_y[i],
                        self.positions_z[i]
                    ])
            
            self.get_logger().info(f'✓ Data saved to: {self.output_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save data: {str(e)}')
    
    def plot_trajectory(self):
        """Generate plots of the trajectory."""
        if len(self.timestamps) == 0:
            self.get_logger().error('No data to plot!')
            return
        
        try:
            # Convert to numpy arrays
            t = np.array(self.timestamps)
            x = np.array(self.positions_x)
            y = np.array(self.positions_y)
            z = np.array(self.positions_z)
            
            # Create figure with subplots
            fig = plt.figure(figsize=(15, 10))
            
            # Plot 1: X vs Time
            ax1 = plt.subplot(2, 3, 1)
            ax1.plot(t, x, 'b-', linewidth=2)
            ax1.set_xlabel('Time (s)', fontsize=12)
            ax1.set_ylabel('X Position (m)', fontsize=12)
            ax1.set_title('X Position vs Time', fontsize=14, fontweight='bold')
            ax1.grid(True, alpha=0.3)
            
            # Plot 2: Y vs Time
            ax2 = plt.subplot(2, 3, 2)
            ax2.plot(t, y, 'r-', linewidth=2)
            ax2.set_xlabel('Time (s)', fontsize=12)
            ax2.set_ylabel('Y Position (m)', fontsize=12)
            ax2.set_title('Y Position vs Time (Commanded +Y)', fontsize=14, fontweight='bold')
            ax2.grid(True, alpha=0.3)
            
            # Plot 3: Z vs Time
            ax3 = plt.subplot(2, 3, 3)
            ax3.plot(t, z, 'g-', linewidth=2)
            ax3.set_xlabel('Time (s)', fontsize=12)
            ax3.set_ylabel('Z Position (m)', fontsize=12)
            ax3.set_title('Z Position vs Time', fontsize=14, fontweight='bold')
            ax3.grid(True, alpha=0.3)
            
            # Plot 4: XY trajectory (top view)
            ax4 = plt.subplot(2, 3, 4)
            ax4.plot(x, y, 'b-', linewidth=2, label='Trajectory')
            ax4.plot(x[0], y[0], 'go', markersize=10, label='Start')
            ax4.plot(x[-1], y[-1], 'ro', markersize=10, label='End')
            ax4.set_xlabel('X Position (m)', fontsize=12)
            ax4.set_ylabel('Y Position (m)', fontsize=12)
            ax4.set_title('XY Trajectory (Top View)', fontsize=14, fontweight='bold')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            ax4.axis('equal')
            
            # Plot 5: 3D Trajectory
            ax5 = plt.subplot(2, 3, 5, projection='3d')
            ax5.plot(x, y, z, 'b-', linewidth=2, label='Trajectory')
            ax5.scatter(x[0], y[0], z[0], c='g', s=100, marker='o', label='Start')
            ax5.scatter(x[-1], y[-1], z[-1], c='r', s=100, marker='o', label='End')
            ax5.set_xlabel('X (m)', fontsize=10)
            ax5.set_ylabel('Y (m)', fontsize=10)
            ax5.set_zlabel('Z (m)', fontsize=10)
            ax5.set_title('3D Trajectory', fontsize=14, fontweight='bold')
            ax5.legend()
            
            # Plot 6: Velocity analysis (numerical derivative)
            ax6 = plt.subplot(2, 3, 6)
            if len(t) > 1:
                # Compute velocities
                dt = np.diff(t)
                vx = np.diff(x) / dt
                vy = np.diff(y) / dt
                vz = np.diff(z) / dt
                t_vel = t[1:]
                
                ax6.plot(t_vel, vy, 'r-', linewidth=2, label='Y velocity')
                ax6.axhline(y=self.velocity_y, color='k', linestyle='--', 
                           label=f'Commanded ({self.velocity_y} m/s)')
                ax6.set_xlabel('Time (s)', fontsize=12)
                ax6.set_ylabel('Velocity (m/s)', fontsize=12)
                ax6.set_title('Y Velocity vs Time', fontsize=14, fontweight='bold')
                ax6.legend()
                ax6.grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            # Save figure
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            plot_filename = f'trajectory_plot_{timestamp}.png'
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            
            self.get_logger().info(f'✓ Plot saved to: {plot_filename}')
            
            # Show plot
            plt.show()
            
            # Print statistics
            self.print_statistics(t, x, y, z)
            
        except Exception as e:
            self.get_logger().error(f'Failed to generate plots: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def print_statistics(self, t, x, y, z):
        """Print trajectory statistics."""
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('TRAJECTORY STATISTICS')
        self.get_logger().info('=' * 70)
        
        # Displacement
        dx = x[-1] - x[0]
        dy = y[-1] - y[0]
        dz = z[-1] - z[0]
        total_dist = np.sqrt(dx**2 + dy**2 + dz**2)
        
        self.get_logger().info(f'Start position: x={x[0]:.4f}, y={y[0]:.4f}, z={z[0]:.4f}')
        self.get_logger().info(f'End position:   x={x[-1]:.4f}, y={y[-1]:.4f}, z={z[-1]:.4f}')
        self.get_logger().info(f'Displacement:   Δx={dx:.4f}, Δy={dy:.4f}, Δz={dz:.4f}')
        self.get_logger().info(f'Total distance: {total_dist:.4f} m')
        
        # Average velocity
        if len(t) > 1:
            avg_vy = dy / (t[-1] - t[0])
            self.get_logger().info(f'\nCommanded Y velocity: {self.velocity_y:.4f} m/s')
            self.get_logger().info(f'Achieved Y velocity:  {avg_vy:.4f} m/s')
            self.get_logger().info(f'Error: {abs(avg_vy - self.velocity_y):.4f} m/s '
                                  f'({abs(avg_vy - self.velocity_y)/self.velocity_y*100:.1f}%)')
        
        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    
    recorder = TrajectoryRecorder()
    
    try:
        # Wait for first pose to be received
        recorder.get_logger().info('\nWaiting for /end_effector_pose topic...')
        time.sleep(2.0)
        
        # Start recording
        recorder.start_recording()
        
        # Spin until recording is complete
        while recorder.recording and rclpy.ok():
            rclpy.spin_once(recorder, timeout_sec=0.1)
        
        # Keep node alive briefly to finish processing
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        recorder.get_logger().info('Interrupted by user')
        recorder.stop_recording()
    
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()