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
    localization_dir = os.path.join(get_package_share_directory('tritonkart_localization'), 'launch')
    navigation_dir = os.path.join(get_package_share_directory('tritonkart_navigation'), 'launch')
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    in_cloud = LaunchConfiguration('in_cloud')
    out_scan = LaunchConfiguration('out_scan')
    in_comp = LaunchConfiguration('in_comp')
    out_cam_raw = LaunchConfiguration('out_cam_raw')
    use_scan = LaunchConfiguration('use_scan')
    use_cam = LaunchConfiguration('use_cam')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    # use_mapping_rviz = LaunchConfiguration('use_mapping_rviz')
    use_kart_rviz = LaunchConfiguration('use_kart_rviz')
    scan_topic = LaunchConfiguration('scan_topic')

    clock_topic = LaunchConfiguration('clock_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    svl_topic = LaunchConfiguration('clock_topic')


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

    odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic',
        default_value='/lgsvl/gnss_odom',
        description='Raw Odometry topic'
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

    use_kart_rviz_cmd = DeclareLaunchArgument(
        'use_kart_rviz',
        default_value='false',
        description='Pull up Rviz of just kart'
    )

    use_navigation_rviz_cmd = DeclareLaunchArgument(
        'use_navigation_rviz',
        default_value='false',
        description='Pull up Rviz of navigation config'
    )

    scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/lidar/scan',
        description='Lidar Scan topic'
    )

    clock_topic_cmd = DeclareLaunchArgument(
        'clock_topic',
        default_value='/clock',
        description='Input Time topic'
    )

    

    twist_topic_cmd = DeclareLaunchArgument(
        'twist_topic',
        default_value='',
        description='Input Twist Topic'
    )
    

    svl_topic_cmd = DeclareLaunchArgument(
        'svl_topic',
        default_value='/lgsvl/vehicle_control_cmd',
        description='Output Vehicle topic'
    )

    lifecycle_nodes = [
        'robot_state_publisher_node',
        'cloud_to_laser_node',
        'image_conversion_node',
        'svl_node',
        'ekf_local_filter_node',
        'ekf_global_filter_node',
        'navsat_transform_node',
        'map_server',
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'racing_client',
    ]
    

    # Specify the actions
    bringup_cmd_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_dir, 'kart.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_kart_rviz': use_kart_rviz}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(utilities_dir, 'start_utilities_sim.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'in_cloud': in_cloud,
                              'out_scan': out_scan,
                              'in_comp': in_comp,
                              'out_cam_raw': out_cam_raw,
                              'use_scan': use_scan,
                              'use_cam': use_cam,
                              'clock_topic': clock_topic,
                              'twist_topic': twist_topic,
                              'svl_topic': svl_topic}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization_dir, 'gps_localization.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'odom_topic': odom_topic,
                              'imu_topic': imu_topic,
                              'gps_topic': gps_topic}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation_dir, 'nav2.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation_dir, 'client.launch.py')),
            launch_arguments=[]),

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
    ld.add_action(odom_topic_cmd)
    ld.add_action(imu_topic_cmd)
    ld.add_action(gps_topic_cmd)
    ld.add_action(use_kart_rviz_cmd)
    ld.add_action(use_navigation_rviz_cmd)
    ld.add_action(scan_topic_cmd)
    ld.add_action(clock_topic_cmd)
    ld.add_action(twist_topic_cmd)
    ld.add_action(svl_topic_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld