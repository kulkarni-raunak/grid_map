import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    grid_map_demos_share = get_package_share_directory('grid_map_demos')

    include_janus_sim_arg = DeclareLaunchArgument(
        'include_janus_sim',
        default_value='false',
        description='Include janus_bringup_sim.launch.py before starting mapping pipeline.',
    )
    use_mapper_arg = DeclareLaunchArgument(
        'use_mapper',
        default_value='true',
        description='Run the live grid_map_pcl mapper on the PointCloud2 topic and publish a GridMap.',
    )
    use_filters_arg = DeclareLaunchArgument(
        'use_filters',
        default_value='false',
        description='Run grid_map_filters demo pipeline over the input GridMap.',
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    pointcloud_topic_arg = DeclareLaunchArgument('pointcloud_topic', default_value='/janus/velodyne_points')
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='',
        description='Frame id for the published GridMap. Empty uses the incoming PointCloud2 frame.',
    )
    raw_map_topic_arg = DeclareLaunchArgument('raw_map_topic', default_value='/elevation_map')
    filtered_map_topic_arg = DeclareLaunchArgument('filtered_map_topic', default_value='/elevation_map_filtered')

    janus_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('janus_navigation'),
                'launch',
                'sim',
                'janus_bringup_sim.launch.py',
            )
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_janus_sim')),
    )

    filter_chain_config = os.path.join(
        grid_map_demos_share,
        'config',
        'janus_elevation_filter_chain.yaml',
    )
    visualization_filtered_config = os.path.join(
        grid_map_demos_share,
        'config',
        'janus_elevation_visualization_filtered.yaml',
    )
    visualization_raw_config = os.path.join(
        grid_map_demos_share,
        'config',
        'janus_elevation_visualization_raw.yaml',
    )
    mapper_config = os.path.join(
        grid_map_demos_share,
        'config',
        'janus_grid_map_pcl_parameters.yaml',
    )
    mapper_node = Node(
        package='grid_map_pcl',
        executable='grid_map_pcl_pointcloud_to_gridmap_node',
        name='grid_map_pcl_mapper',
        output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'grid_map_topic': LaunchConfiguration('raw_map_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'pcl_config_file': mapper_config,
            },
        ],
        condition=IfCondition(LaunchConfiguration('use_mapper')),
    )
    filter_node = Node(
        package='grid_map_demos',
        executable='filters_demo',
        name='grid_map_filters',
        output='screen',
        parameters=[
            filter_chain_config,
            {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'input_topic': LaunchConfiguration('raw_map_topic'),
                'output_topic': LaunchConfiguration('filtered_map_topic'),
                'filter_chain_parameter_name': 'filters',
            },
        ],
        condition=IfCondition(LaunchConfiguration('use_filters')),
    )

    # Visualization if filters are enabled.
    visualization_filtered = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[
            visualization_filtered_config,
            {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'grid_map_topic': LaunchConfiguration('filtered_map_topic'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('use_filters')),
    )

    # Visualization directly from raw map if filters are disabled.
    visualization_raw = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[
            visualization_raw_config,
            {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'grid_map_topic': LaunchConfiguration('raw_map_topic'),
            },
        ],
        condition=UnlessCondition(LaunchConfiguration('use_filters')),
    )

    return LaunchDescription([
        include_janus_sim_arg,
        use_mapper_arg,
        use_filters_arg,
        use_sim_time_arg,
        pointcloud_topic_arg,
        map_frame_arg,
        raw_map_topic_arg,
        filtered_map_topic_arg,
        janus_sim_launch,
        mapper_node,
        filter_node,
        visualization_filtered,
        visualization_raw,
    ])
