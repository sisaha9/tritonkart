import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav_dir = get_package_share_directory('tritonkart_navigation')
    map_dir = get_package_share_directory('tritonkart_mapping')

    map_file_path = os.path.join(
        map_dir, 
        'maps',
        'map.yaml'
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

    use_navigation_rviz = LaunchConfiguration('use_navigation_rviz')

    use_navigation_rviz_cmd = DeclareLaunchArgument(
        'use_navigation_rviz',
        default_value='false'
    )
    
    parameters_file_path = os.path.join(
        nav_dir,
        'config',
        'nav2_navigation.yaml'
    )

    
    
    xml_file_path = os.path.join(
        nav_dir,
        'config', 
        'nav_bt.xml'
    )

    rviz_config_file = os.path.join(
        nav_dir,
        'rviz',
        'nav2.rviz')

    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator']

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        use_sim_time_cmd,
        map_path_cmd,
        use_navigation_rviz_cmd,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                    {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file_path},
            ]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time},]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time},]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                  parameters_file_path,
                  {'use_sim_time': use_sim_time},
                  {'default_bt_xml_filename': xml_file_path}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav_node',
            output='screen',
            arguments = ['-d', rviz_config_file],
            parameters = [{'use_sim_time': use_sim_time}],
            condition= IfCondition(use_navigation_rviz)
            )
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'autostart': True},
        #                 {'node_names': lifecycle_nodes}]
        # ),
    ])

