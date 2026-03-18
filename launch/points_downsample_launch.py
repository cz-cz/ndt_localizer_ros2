import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_localizer')

    # Declare arguments
    sync_arg = DeclareLaunchArgument(
        'sync',
        default_value='false',
        description='Whether to sync topics'
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='voxel_grid_filter',
        description='Node name'
    )
    points_topic_arg = DeclareLaunchArgument(
        'points_topic',
        default_value='/os1_points',
        description='Input points topic'
    )
    output_log_arg = DeclareLaunchArgument(
        'output_log',
        default_value='false',
        description='Whether to output log'
    )
    leaf_size_arg = DeclareLaunchArgument(
        'leaf_size',
        default_value='3.0',
        description='Voxel leaf size'
    )

    # Determine points topic based on sync condition
    points_topic = PythonExpression([
        "'/sync_drivers/points_raw' if 'true' == '", LaunchConfiguration('sync'), "' else '", LaunchConfiguration('points_topic'), "'"
    ])

    # Voxel Grid Filter Node
    voxel_grid_filter_node = Node(
        package='ndt_localizer',
        executable='voxel_grid_filter',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            {'points_topic': points_topic},
            {'output_log': LaunchConfiguration('output_log')},
            {'leaf_size': LaunchConfiguration('leaf_size')}
        ]
    )

    return LaunchDescription([
        sync_arg,
        node_name_arg,
        points_topic_arg,
        output_log_arg,
        leaf_size_arg,
        voxel_grid_filter_node
    ])