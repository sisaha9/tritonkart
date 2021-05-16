import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time'
    )

    odom0_topic = LaunchConfiguration('odom0_topic')

    odom0_topic_cmd = DeclareLaunchArgument(
        'odom0_topic',
        default_value='/lgsvl/gnss_odom',
        description='Raw Odometry topic'
    )

    imu0_topic = LaunchConfiguration('imu0_topic')

    imu0_topic_cmd = DeclareLaunchArgument(
        'imu0_topic',
        default_value='/imu/imu_raw',
        description='Raw IMU topic'
    )


    mapping_dir = get_package_share_directory('tritonkart_mapping')

    parameters_file_path = os.path.join(
        mapping_dir,
        'config', 
        'mapping.yaml'
    )


    remappings_rl = {
        'odometry/filtered':'/odometry/local'
    }

    rl_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_filter_node',
        parameters=[parameters_file_path, {'use_sim_time': use_sim_time, 'odom0': odom0_topic, 'imu0':imu0_topic}],
        remappings=remappings_rl.items()
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(odom0_topic_cmd)
    ld.add_action(imu0_topic_cmd)
    ld.add_action(rl_node)

    return ld

