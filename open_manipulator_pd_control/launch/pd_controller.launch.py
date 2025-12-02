from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments with default values
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='5.0',
        description='Proportional gain for PD controller'
    )
    
    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.5',
        description='Derivative gain for PD controller'
    )
    
    control_freq_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100.0',
        description='Control loop frequency in Hz'
    )
    
    actuator_idx_arg = DeclareLaunchArgument(
        'actuator_index',
        default_value='3',
        description='Actuator index (3 for Actuator 4, 0-indexed)'
    )
    
    # PD Controller Node
    pd_controller_node = Node(
        package='open_manipulator_pd_control',
        executable='pd_controller_node.py',
        name='pd_controller',
        output='screen',
        parameters=[{
            'kp': LaunchConfiguration('kp'),
            'kd': LaunchConfiguration('kd'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'actuator_index': LaunchConfiguration('actuator_index'),
        }]
    )
    
    return LaunchDescription([
        kp_arg,
        kd_arg,
        control_freq_arg,
        actuator_idx_arg,
        pd_controller_node
    ])
