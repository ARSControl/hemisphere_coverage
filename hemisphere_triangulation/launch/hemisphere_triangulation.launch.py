import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hemisphere_triangulation_dir = get_package_share_directory('hemisphere_triangulation')
    hemisphere_triangulation_ros_params = os.path.join(
        hemisphere_triangulation_dir, 'config', 'hemisphere_triangulation.yaml'
    )
    params_file = LaunchConfiguration('params_file', default=hemisphere_triangulation_ros_params)

    uav_name = EnvironmentVariable('UAV_NAME')
    uav_name_param = LaunchConfiguration('uav_name', default=uav_name)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    hemisphere_triangulation_node = Node(
        package='hemisphere_triangulation',
        executable='hemisphere_triangulation',
        output='screen',
        namespace=uav_name_param,
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time_cfg, value_type=bool),
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(hemisphere_triangulation_node)
    return ld
