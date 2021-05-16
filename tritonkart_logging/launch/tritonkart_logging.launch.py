import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    navigation_dir = get_package_share_directory('tritonkart_navigation')

    freq = LaunchConfiguration('frequency')

    freq_cmd = DeclareLaunchArgument(
        'frequency',
        default_value='1',
        description='Frequency at which to record GPS coordinates'
    )

    save_path = LaunchConfiguration('save_path')

    save_path_cmd = DeclareLaunchArgument(
        'save_path',
        default_value=os.path.join(navigation_dir, 'path', 'gps_waypoints.csv'),
        description='Use simulator time'
    )

    imu_topic = LaunchConfiguration('imu_topic')

    imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/imu_raw',
        description='Raw IMU topic'
    )

    gps_topic = LaunchConfiguration('gps_topic')

    gps_topic_cmd = DeclareLaunchArgument(
        'gps_topic',
        default_value='/lgsvl/gnss_odom',
        description='Raw GPS topic'
    )

    remappings_log = {
        '/gps': gps_topic,
        '/imu': imu_topic,
    }

    log_node = Node(
        package='tritonkart_logging',
        executable='gps_logger.py',
        name='gps_logger_node',
        output='screen',
        parameters=[{'frequency': freq, 'save_path': save_path}],
        remappings=remappings_log.items()
    )

    ld = LaunchDescription()
    ld.add_action(freq_cmd)
    ld.add_action(save_path_cmd)
    ld.add_action(imu_topic_cmd)
    ld.add_action(gps_topic_cmd)
    ld.add_action(log_node)

    return ld

