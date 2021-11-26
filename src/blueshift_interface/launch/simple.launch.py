import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ld = LaunchDescription()

    web_interface_share_directory = get_package_share_directory('blueshift_interface')
    
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

    # ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    rosbridge_websocket_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch/rosbridge_websocket_launch.xml'
                )
            )
        )
    ld.add_action(rosbridge_websocket_launch)

    return ld