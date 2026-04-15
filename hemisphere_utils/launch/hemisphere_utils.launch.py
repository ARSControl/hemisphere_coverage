import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hemisphere_coverage_dir = get_package_share_directory('hemisphere_coverage')
    hemisphere_coverage_ros_params = os.path.join(
        hemisphere_coverage_dir, 'config', 'hemisphere_config.yaml'
    )
    params_file = LaunchConfiguration('params_file', default=hemisphere_coverage_ros_params)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    hemisphere_utils_node = Node(
        package='hemisphere_utils',
        executable='hemisphere_utils',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time_cfg, value_type=bool),
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(hemisphere_utils_node)
    return ld
