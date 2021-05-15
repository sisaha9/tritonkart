
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
    utilities_dir = os.path.join(get_package_share_directory('tritonkart_utilities'), 'launch')
    mapping_dir = os.path.join(get_package_share_directory('tritonkart_mapping'), 'launch')
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    in_cloud = LaunchConfiguration('in_cloud')
    out_scan = LaunchConfiguration('out_scan')
    in_comp = LaunchConfiguration('in_comp')
    out_cam_raw = LaunchConfiguration('out_cam_raw')
    use_scan = LaunchConfiguration('use_scan')
    use_cam = LaunchConfiguration('use_cam')
    odom0_topic = LaunchConfiguration('odom0_topic')
    imu0_topic = LaunchConfiguration('imu0_topic')
    use_mapping_rviz = LaunchConfiguration('use_mapping_rviz')
    use_kart_rviz = LaunchConfiguration('use_kart_rviz')
    scan_topic = LaunchConfiguration('scan_topic')



    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time')

    in_cloud_cmd = DeclareLaunchArgument(
        'in_cloud',
        default_value='/lidar/points_raw',
        description='Input PointCloud topic'
    )

    out_scan_cmd = DeclareLaunchArgument(
        'out_scan',
        default_value='/lidar/scan',
        description='Output Scan topic')

    in_comp_cmd = DeclareLaunchArgument(
        'in_comp',
        default_value='/color_camera/compressed',
        description='Input Compressed camera topic'
    )

    out_cam_raw_cmd = DeclareLaunchArgument(
        'out_cam_raw',
        default_value='/color_camera/image_raw',
        description='Output Raw camera topic')

    use_scan_cmd = DeclareLaunchArgument(
        'use_scan',
        default_value='true',
        description='Use Scan conversion')

    use_cam_cmd = DeclareLaunchArgument(
        'use_cam',
        default_value='true',
        description='Use Cam conversion')

    odom0_topic_cmd = DeclareLaunchArgument(
        'odom0_topic',
        default_value='/gnss/odom',
        description='Raw Odometry topic'
    )

    imu0_topic_cmd = DeclareLaunchArgument(
        'imu0_topic',
        default_value='/imu/imu_raw',
        description='Raw IMU topic'
    )

    use_kart_rviz_cmd = DeclareLaunchArgument(
        'use_kart_rviz',
        default_value='false',
        description='Pull up Rviz of just kart'
    )

    use_mapping_rviz_cmd = DeclareLaunchArgument(
        'use_mapping_rviz',
        default_value='true',
        description='Pull up Rviz of mapping config'
    )

    scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/lidar/scan',
        description='Lidar Scan topic'
    )

    lifecycle_nodes = [
        'robot_state_publisher_node',
        'ekf_local_filter_node',
        'cloud_to_laser_node',
        'image_conversion_node',
        'slam_toolbox_node',
        'rviz2_node'
    ]
    

    # Specify the actions
    bringup_cmd_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_dir, 'kart.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_kart_rviz': use_kart_rviz}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(utilities_dir, 'start_utilities.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'in_cloud': in_cloud,
                              'out_scan': out_scan,
                              'in_comp': in_comp,
                              'out_cam_raw': out_cam_raw,
                              'use_scan': use_scan,
                              'use_cam': use_cam}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping_dir, 'ekf_node.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'odom0_topic': odom0_topic,
                              'imu0_topic': imu0_topic}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping_dir, 'slam_mapping.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_mapping_rviz': use_mapping_rviz,
                              'scan_topic': scan_topic}.items()),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'node_names': lifecycle_nodes}])
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(use_sim_time_cmd)
    ld.add_action(in_cloud_cmd)
    ld.add_action(out_scan_cmd)
    ld.add_action(in_comp_cmd)
    ld.add_action(out_cam_raw_cmd)
    ld.add_action(use_scan_cmd)
    ld.add_action(use_cam_cmd)
    ld.add_action(odom0_topic_cmd)
    ld.add_action(imu0_topic_cmd)
    ld.add_action(use_kart_rviz_cmd)
    ld.add_action(use_mapping_rviz_cmd)
    ld.add_action(scan_topic_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld