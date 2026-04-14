import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ------------------------------------------------------------
    # 1. Locate config file and define a default LaunchConfiguration
    # ------------------------------------------------------------
    hemisphere_coverage_dir = get_package_share_directory('hemisphere_coverage')
    hemisphere_coverage_ros_params = os.path.join(
        hemisphere_coverage_dir, 'config', 'hemisphere_config.yaml'
    )
    params_file = LaunchConfiguration('params_file', default=hemisphere_coverage_ros_params)

    # ------------------------------------------------------------
    # 2. Read environment variables for UAV name, ID, run_type, etc.
    # ------------------------------------------------------------
    uav_name = EnvironmentVariable('UAV_NAME')
    uav_id = EnvironmentVariable('UAV_ID')
    uav_name_param = LaunchConfiguration('uav_name', default=uav_name)
    uav_id_param = LaunchConfiguration('uav_id', default=uav_id)

    # ------------------------------------------------------------
    # 3. Declare launch arguments for active parameters
    # ------------------------------------------------------------
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Command prefix for launching nodes, e.g., "gdb -ex run --args"',
    )

    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='10.0',
        description='Radius parameter'
    )
    geometric_arg = DeclareLaunchArgument(
        'geometric',
        default_value='1',
        description='Geometric parameter'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ------------------------------------------------------------
    # 4. LaunchConfigurations to capture each argument
    # ------------------------------------------------------------
    prefix = LaunchConfiguration('prefix')
    radius = LaunchConfiguration('radius')
    geometric = LaunchConfiguration('geometric')
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------
    # 5. Create the node, passing the YAML plus overrides as ROS parameters
    # ------------------------------------------------------------
    hemisphere_coverage_node = Node(
        package='hemisphere_coverage',
        executable='hemisphere_coverage',
        output='screen',
        prefix=prefix,
        namespace=uav_name_param,
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time_cfg, value_type=bool),
                'uav_name': ParameterValue(uav_name_param, value_type=str),
                'uav_id': ParameterValue(uav_id_param, value_type=int),
                'radius': ParameterValue(radius, value_type=float),
                'geometric': ParameterValue(geometric, value_type=int)
            }
        ],
    )

    # ------------------------------------------------------------
    # 6. Add everything to the LaunchDescription
    # ------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(prefix_arg)
    ld.add_action(radius_arg)
    ld.add_action(geometric_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(hemisphere_coverage_node)

    return ld
