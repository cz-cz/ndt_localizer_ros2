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
        default_value='/livox/points',
        # default_value='/mapping/LaserframeXYZI',
        description='Sensor points topic'
    )
    input_initial_pose_topic_arg = DeclareLaunchArgument(
        'input_initial_pose_topic',
        default_value='/initialpose',
        description='Initial position topic to align, will be used as GICP initial pose'
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
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry reference frame'
    )
    trans_epsilon_arg = DeclareLaunchArgument(
        'trans_epsilon',
        default_value='0.01',
        description='The maximum difference between two consecutive transformations in order to consider convergence'
    )
    # GICP 参数，替换了原来的NDT的step_size和resolution
    max_corr_dist_arg = DeclareLaunchArgument(
        'max_correspondence_distance',
        default_value='1.0',
        description='Maximum distance between corresponding points for GICP'
    )
    fitness_epsilon_arg = DeclareLaunchArgument(
        'euclidean_fitness_epsilon',
        default_value='0.01',
        description='Maximum error threshold for GICP convergence'
    )
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='30',
        description='The number of iterations required to calculate alignment'
    )
    converged_param_fitness_score_arg = DeclareLaunchArgument(
        'converged_param_fitness_score',
        default_value='1.0',
        description='Convergence parameter: maximum allowed fitness score (error)'
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
    # GICP Localizer Node (replaced original NDT node)
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
            {'odom_frame': LaunchConfiguration('odom_frame')},
            {'trans_epsilon': LaunchConfiguration('trans_epsilon')},
            {'max_correspondence_distance': LaunchConfiguration('max_correspondence_distance')},
            {'euclidean_fitness_epsilon': LaunchConfiguration('euclidean_fitness_epsilon')},
            {'max_iterations': LaunchConfiguration('max_iterations')},
            {'converged_param_fitness_score': LaunchConfiguration('converged_param_fitness_score')}
        ]
    )
    return LaunchDescription([
        input_sensor_points_topic_arg,
        input_initial_pose_topic_arg,
        input_map_points_topic_arg,
        output_pose_topic_arg,
        output_diagnostics_topic_arg,
        base_frame_arg,
        odom_frame_arg,
        trans_epsilon_arg,
        max_corr_dist_arg,
        fitness_epsilon_arg,
        max_iterations_arg,
        converged_param_fitness_score_arg,
        static_tf_launch,
        map_loader_launch,
        points_downsample_launch,
        ndt_localizer_node
    ])
