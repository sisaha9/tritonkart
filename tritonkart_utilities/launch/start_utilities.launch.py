from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    in_cloud = LaunchConfiguration('in_cloud')

    in_cloud_cmd = DeclareLaunchArgument(
        'in_cloud',
        default_value='/lidar/points_raw',
        description='Input PointCloud topic'
    )

    out_scan = LaunchConfiguration('out_scan')

    out_scan_cmd = DeclareLaunchArgument(
        'out_scan',
        default_value='/lidar/scan',
        description='Output Scan topic')

    in_comp = LaunchConfiguration('in_comp')

    in_comp_cmd = DeclareLaunchArgument(
        'in_comp',
        default_value='/color_camera/compressed',
        description='Input Compressed camera topic'
    )

    out_cam_raw = LaunchConfiguration('out_cam_raw')

    out_cam_raw_cmd = DeclareLaunchArgument(
        'out_cam_raw',
        default_value='/color_camera/image_raw',
        description='Output Raw camera topic')

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulator time'
    )

    use_scan = LaunchConfiguration('use_scan')

    use_scan_cmd = DeclareLaunchArgument(
        'use_scan',
        default_value='true',
        description='Use Scan conversion')

    use_cam = LaunchConfiguration('use_cam')

    use_cam_cmd = DeclareLaunchArgument(
        'use_cam',
        default_value='true',
        description='Use Cam conversion')

    utilities_dir = get_package_share_directory('tritonkart_utilities')

    parameters_file_path = os.path.join(
        utilities_dir,
        'config', 
        'utilities.yaml'
    )

    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    parameters_file_path = RewrittenYaml(
        source_file = parameters_file_path,
        root_key='',
        param_rewrites = param_substitutions,
        convert_types = True
    )



    scan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', in_cloud),
                    ('scan', out_scan)],
        parameters=[parameters_file_path],
        name='cloud_to_laser_node',
        condition=IfCondition(use_scan)
    )
    
    cam_node = Node(
        package='image_transport',
        executable='republish',
        name='image_conversion_node',
        arguments=['compressed', 'raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[
            ('/in/compressed', in_comp),
            ('/out', out_cam_raw)
        ],
        condition=IfCondition(use_cam)
    )

    ld = LaunchDescription()

    ld.add_action(in_cloud_cmd)
    ld.add_action(out_scan_cmd)
    ld.add_action(in_comp_cmd)
    ld.add_action(out_cam_raw_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(use_scan_cmd)
    ld.add_action(use_cam_cmd)
    ld.add_action(scan_node)
    ld.add_action(cam_node)

    return ld
