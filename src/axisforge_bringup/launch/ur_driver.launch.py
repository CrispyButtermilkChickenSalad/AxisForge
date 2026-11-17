"""Supervise the UR dashboard status/stop client alongside the standard driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Keep the upstream driver's arguments, controller configuration and manual
    # External Control startup. A transient primary-interface startup timeout
    # must not silently remove the dashboard emergency-stop/status service.
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
        launch_arguments={'launch_dashboard_client': 'false',
                          'robot_ip': LaunchConfiguration('robot_ip'),
                          'ur_type': LaunchConfiguration('ur_type')}.items(),
    )
    dashboard = Node(
        package='ur_robot_driver', executable='dashboard_client', name='dashboard_client',
        output='screen', respawn=True, respawn_delay=2.,
        condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
        parameters=[{'robot_ip': LaunchConfiguration('robot_ip')},
                    {'receive_timeout': 20.0}, {'autoconnect': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip'),
        DeclareLaunchArgument('ur_type', default_value='ur16e'),
        driver, dashboard,
    ])
