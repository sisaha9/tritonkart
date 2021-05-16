import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    # Get the launch directories
    desc_dir = os.path.join(get_package_share_directory('tritonkart_description'), 'launch')
    logging_dir = os.path.join(get_package_share_directory('tritonkart_logging'), 'launch')
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_kart_rviz = LaunchConfiguration('use_kart_rviz')
    freq = LaunchConfiguration('frequency')
    save_path = LaunchConfiguration('save_path')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_topic = LaunchConfiguration('gps_topic')



    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time'
    )

    use_kart_rviz_cmd = DeclareLaunchArgument(
        'use_kart_rviz',
        default_value='false',
        description='Pull up Rviz of just kart'
    )

    freq_cmd = DeclareLaunchArgument(
        'frequency',
        default_value='1',
        description='Frequency at which to record GPS coordinates'
    )

    save_path_cmd = DeclareLaunchArgument(
        'save_path',
        default_value=os.path.join(logging_dir, 'path', 'gps_waypoints.csv'),
        description='Use simulator time'
    )

    imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/imu_raw',
        description='Raw IMU topic'
    )

    gps_topic_cmd = DeclareLaunchArgument(
        'gps_topic',
        default_value='/lgsvl/gnss_odom',
        description='Raw GPS topic'
    )

    lifecycle_nodes = [
        'robot_state_publisher_node',
        'gps_logger_node',
    ]
    

    # Specify the actions
    bringup_cmd_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_dir, 'kart.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_kart_rviz': use_kart_rviz}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(logging_dir, 'logging.launch.py')),
            launch_arguments={'frequency': freq,
                              'save_path': save_path,
                              'imu_topic': imu_topic,
                              'gps_topic': gps_topic}.items()),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_logging',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(use_sim_time_cmd)
    ld.add_action(use_kart_rviz_cmd)
    ld.add_action(freq_cmd)
    ld.add_action(save_path_cmd)
    ld.add_action(imu_topic_cmd)
    ld.add_action(gps_topic_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld