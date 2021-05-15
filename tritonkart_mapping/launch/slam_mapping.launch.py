import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_mapping_rviz = LaunchConfiguration('use_mapping_rviz')

    use_mapping_rviz_cmd = DeclareLaunchArgument(
        'use_mapping_rviz',
        default_value='false',
        description='Pull up Rviz when calling this launch file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time'
    )

    scan_topic = LaunchConfiguration('scan_topic')

    scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/lidar/scan',
        description='Lidar Scan topic'
    )

    mapping_dir = get_package_share_directory('tritonkart_mapping')

    parameters_file_path = os.path.join(
        mapping_dir,
        'config', 
        'mapping.yaml'
    )

    rviz_config_file = os.path.join(
        mapping_dir, 
        'rviz', 
        'mapping.rviz'
    )

    remappings_map = {
        'tf': '/tf',
        'map': '/map',
        "/scan": scan_topic
    }

    map_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_node',
        output='screen',
        parameters=[parameters_file_path, {'use_sim_time': use_sim_time}],
        remappings=remappings_map.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_file],
        parameters= [
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_mapping_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(use_mapping_rviz_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(scan_topic_cmd)
    ld.add_action(map_node)
    ld.add_action(rviz_node)

    return ld

