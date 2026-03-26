import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_localizer')

    # Static TF publisher: base_link to ouster
    # localizer_to_base_link_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["--x", "0", "--y", "0", "--z", "0",
    #             "--roll", "0", "--pitch", "0", "--yaw", "0",
    #             "--frame-id", "base_link", "--child-frame-id", "ouster"],
    #     output="screen"
    # )

    # Static TF publisher: map to world
    # world_to_map_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_to_map',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    # )
    livox_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "livox_frame"],
        output="screen"
    )
    # map_to_base_link_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["--x", "0", "--y", "0", "--z", "0",
    #             "--roll", "0", "--pitch", "0", "--yaw", "0",
    #             "--frame-id", "map", "--child-frame-id", "base_link"],
    #     output="screen"
    # )
    return LaunchDescription([
        # localizer_to_base_link_tf,
        # world_to_map_tf,
        livox_to_base_link_tf,
        # map_to_base_link_tf
    ])