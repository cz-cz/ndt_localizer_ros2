import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_localizer')

    # Declare arguments
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='/base_link',
        description='Base frame'
    )
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='vehicle_model',
        description='Topic name for robot state'
    )
    offset_x_arg = DeclareLaunchArgument(
        'offset_x',
        default_value='1.2',
        description='X offset'
    )
    offset_y_arg = DeclareLaunchArgument(
        'offset_y',
        default_value='0.0',
        description='Y offset'
    )
    offset_z_arg = DeclareLaunchArgument(
        'offset_z',
        default_value='0.0',
        description='Z offset'
    )
    offset_roll_arg = DeclareLaunchArgument(
        'offset_roll',
        default_value='0.0',
        description='Roll offset (degree)'
    )
    offset_pitch_arg = DeclareLaunchArgument(
        'offset_pitch',
        default_value='0.0',
        description='Pitch offset (degree)'
    )
    offset_yaw_arg = DeclareLaunchArgument(
        'offset_yaw',
        default_value='0.0',
        description='Yaw offset (degree)'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share, 'urdf', 'lexus.urdf'),
        description='Path to URDF model'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='False',
        description='Whether to use GUI'
    )

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_gui': LaunchConfiguration('gui')}
        ]
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['cat ', LaunchConfiguration('model_path')])}
        ]
    )

    return LaunchDescription([
        base_frame_arg,
        topic_name_arg,
        offset_x_arg,
        offset_y_arg,
        offset_z_arg,
        offset_roll_arg,
        offset_pitch_arg,
        offset_yaw_arg,
        model_path_arg,
        gui_arg,
        joint_state_publisher_node,
        robot_state_publisher_node
    ])