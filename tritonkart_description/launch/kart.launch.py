from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    use_kart_rviz = LaunchConfiguration('use_kart_rviz')

    use_kart_rviz_cmd = DeclareLaunchArgument(
        'use_kart_rviz',
        default_value='false',
        description='Pull up Rviz when calling this launch file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time'
    )

    desc_dir = get_package_share_directory('tritonkart_description')

    urdf_file_path = os.path.join(
        desc_dir,
        'urdf',
        'kart.urdf'
    )

    with open(urdf_file_path, 'r') as infile:
        urdf_content = infile.read()

    param_file_path = os.path.join(
        desc_dir,
        'config',
        'kart.yaml'
    )

    rviz_config_file = os.path.join(desc_dir, 'rviz', 'kart.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[
            param_file_path, 
            {'robot_description':urdf_content,
            'use_sim_time': use_sim_time}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_kart_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(use_kart_rviz_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld