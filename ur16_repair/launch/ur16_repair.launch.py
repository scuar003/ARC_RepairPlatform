import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Resolve the parameter file path from the launch configuration.
    params_file_path = LaunchConfiguration('params_file').perform(context)
    # Load parameters from the YAML file.
    with open(params_file_path, 'r') as f:
        params = yaml.safe_load(f)
    
    # Extract global parameters.
    global_params = params.get('global', {})

    # Merge global parameters with the UR control node parameters.
    ur_node_params = params.get('ur_control', {}).get('ros__parameters', {})
    merged_ur_params = dict(global_params, **ur_node_params)
    ur_type = merged_ur_params.get('ur_type', 'ur16e')
    robot_ip = merged_ur_params.get('robot_ip', '192.168.56.101')
    launch_rviz = merged_ur_params.get('launch_rviz', True)

    # Include the UR control launch file with merged parameters.
    ur_robot_driver_share_dir = get_package_share_directory('ur_robot_driver')
    ur_control_launch_file = os.path.join(ur_robot_driver_share_dir, 'launch', 'ur_control.launch.py')
    include_ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_launch_file),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': str(launch_rviz).lower()  # expects 'true' or 'false'
        }.items()
    )

    voxel_params = params.get('voxel_mapper', {}).get('ros__parameters', {})
    merge_voxel_params = dict(global_params, **voxel_params)

    # Build the rest of the launch description.
    nodes = [
        Node (
            package='rmw_zenoh_cpp',
            executable='rmw_zenohd',
            output='screen'
        ),
        include_ur_control,
        Node (
            package='ur16_repair',
            executable='supervisor',
            output='screen',
            parameters=[global_params]
        ),
        Node (
            package='ur16_repair',
            executable='repair_server',
            output='screen',
            parameters=[global_params]
        ),
        Node (
            package='ur16_repair',
            executable='renderer',
            output='screen',
            parameters=[global_params]
        ),
        Node (
            package='ur16_repair',
            executable='laser_to_cloud',
            output='log'
        ),
        Node (
            package='ur16_repair',
            executable='voxel_mapper',
            output='log',
            parameters=[{'ros__parameters': merge_voxel_params}]
        ),
        # Static transforms.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camara',
            output='screen',
            arguments=['0.042', '0.0', '0.036', '1.570', '3.19', '-0.02', 'camara_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_tool0',
            output='screen',
            arguments=['-0.13', '-0.03', '0.04', '0', '1.57', '3.14', 'tool0', 'lase']
        )
    ]
    
    return nodes

def generate_launch_description():
    # Default parameter file inside the package.
    default_params_file = os.path.join(
        get_package_share_directory('ur16_repair'),
        'config',
        'params.yaml'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Absolute path to external parameter file'
    )
    
    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])
