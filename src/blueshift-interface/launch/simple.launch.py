import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ld = LaunchDescription()

    web_interface_share_directory = get_package_share_directory('blueshift-interface')

    start_web_bridge_file = os.path.join(
        web_interface_share_directory,
        'ros2-web-bridge',
        'bin',
        'rosbridge.js'
    )

    # ROS2-Web-Bridge Launcher
    start_web_bridge = Node(
        name='ros2_web_bridge',
        executable='nvm',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            'exec',
            '12',
            f'"node {start_web_bridge_file}"'
        ],
        cwd=web_interface_share_directory
    )
    ld.add_action(start_web_bridge)

    # Web Interface Launcher
    start_interface_file = os.path.join(
        web_interface_share_directory,
        'build',
        'index.js')

    start_web_interface = Node(
        name='blueshift_interface',
        executable='node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            start_interface_file
        ],
        cwd=web_interface_share_directory
    )
    ld.add_action(start_web_interface)


    return ld