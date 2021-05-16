import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('tritonkart_navigation')
    log_dir = get_package_share_directory('tritonkart_logging')

    map_file_path = os.path.join(
        log_dir, 
        'path',
        'gps_waypoints.csv'
    )


    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time'
    )

    map_path = LaunchConfiguration('map_path')

    map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value=map_file_path
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        use_sim_time_cmd,
        map_path_cmd,
        Node(
            package='tritonkart_navigation',
            executable='nav2_client.py',
            name='racing_client',
            output='screen',
            parameters=[
                    # {'use_sim_time': use_sim_time},
                    {'map_path': map_file_path},
            ]
        )
    ])

