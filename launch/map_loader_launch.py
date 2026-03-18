import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_localizer')

    # Declare arguments
    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll angle for map transform'
    )
    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Pitch angle for map transform'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw angle for map transform'
    )
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X translation for map transform'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y translation for map transform'
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Z translation for map transform'
    )
    pcd_path_arg = DeclareLaunchArgument(
        'pcd_path',
        default_value=os.path.join(pkg_share, 'map', '202603061-floor-processed-downsample22.pcd'),
        description='Path to PCD map file'
    )
    print(os.path.join(pkg_share, 'map', '202603061-floor-processed-downsample22.pcd'))
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/points_map',
        description='Topic to publish map point cloud'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'cfgs', 'rock-auto.rviz')],
        output='screen'
    )

    # Map Loader Node
    map_loader_node = Node(
        package='ndt_localizer',
        executable='map_loader',
        name='map_loader',
        output='screen',
        parameters=[
            {'pcd_path': LaunchConfiguration('pcd_path')},
            {'map_topic': LaunchConfiguration('map_topic')},
            {'roll': LaunchConfiguration('roll')},
            {'pitch': LaunchConfiguration('pitch')},
            {'yaw': LaunchConfiguration('yaw')},
            {'x': LaunchConfiguration('x')},
            {'y': LaunchConfiguration('y')},
            {'z': LaunchConfiguration('z')}
        ]
    )

    return LaunchDescription([
        roll_arg,
        pitch_arg,
        yaw_arg,
        x_arg,
        y_arg,
        z_arg,
        pcd_path_arg,
        map_topic_arg,
        rviz_node,
        map_loader_node
    ])