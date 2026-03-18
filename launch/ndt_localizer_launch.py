import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_localizer')

    # Declare arguments
    input_sensor_points_topic_arg = DeclareLaunchArgument(
        'input_sensor_points_topic',
        # default_value='/livox/points',
        default_value='/mapping/LaserframeXYZI',
        description='Sensor points topic'
    )
    input_initial_pose_topic_arg = DeclareLaunchArgument(
        'input_initial_pose_topic',
        default_value='/ekf_pose_with_covariance',
        description='Initial position topic to align'
    )
    input_map_points_topic_arg = DeclareLaunchArgument(
        'input_map_points_topic',
        default_value='/points_map',
        description='Map points topic'
    )
    output_pose_topic_arg = DeclareLaunchArgument(
        'output_pose_topic',
        default_value='ndt_pose',
        description='Estimated self position'
    )
    output_diagnostics_topic_arg = DeclareLaunchArgument(
        'output_diagnostics_topic',
        default_value='diagnostics',
        description='Diagnostic topic'
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Vehicle reference frame'
    )
    trans_epsilon_arg = DeclareLaunchArgument(
        'trans_epsilon',
        default_value='0.05',
        description='The maximum difference between two consecutive transformations in order to consider convergence'
    )
    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='0.1',
        description='The newton line search maximum step length'
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.3',
        description='The ND voxel grid resolution'
    )
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='30',
        description='The number of iterations required to calculate alignment'
    )
    converged_param_transform_probability_arg = DeclareLaunchArgument(
        'converged_param_transform_probability',
        default_value='3.0',
        description='Convergence parameter for transform probability'
    )

    # Include other launch files
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'static_tf_launch.py'))
    )

    map_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'map_loader_launch.py'))
    )

    points_downsample_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'points_downsample_launch.py'))
    )

    # lexus_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'lexus_launch.py'))
    # )

    # NDT Localizer Node
    ndt_localizer_node = Node(
        package='ndt_localizer',
        executable='ndt_localizer_node',
        name='ndt_localizer_node',
        output='screen',
        remappings=[
            ('filtered_points', LaunchConfiguration('input_sensor_points_topic')),
            ('initialpose', LaunchConfiguration('input_initial_pose_topic')),
            ('points_map', LaunchConfiguration('input_map_points_topic')),
            ('ndt_pose', LaunchConfiguration('output_pose_topic')),
            ('diagnostics', LaunchConfiguration('output_diagnostics_topic'))
        ],
        parameters=[
            {'base_frame': LaunchConfiguration('base_frame')},
            {'trans_epsilon': LaunchConfiguration('trans_epsilon')},
            {'step_size': LaunchConfiguration('step_size')},
            {'resolution': LaunchConfiguration('resolution')},
            {'max_iterations': LaunchConfiguration('max_iterations')},
            {'converged_param_transform_probability': LaunchConfiguration('converged_param_transform_probability')}
        ]
    )

    return LaunchDescription([
        input_sensor_points_topic_arg,
        input_initial_pose_topic_arg,
        input_map_points_topic_arg,
        output_pose_topic_arg,
        output_diagnostics_topic_arg,
        base_frame_arg,
        trans_epsilon_arg,
        step_size_arg,
        resolution_arg,
        max_iterations_arg,
        converged_param_transform_probability_arg,
        static_tf_launch,
        map_loader_launch,
        points_downsample_launch,
        ndt_localizer_node,
        # lexus_launch
    ])