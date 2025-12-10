import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

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
    run_type = EnvironmentVariable('RUN_TYPE')
    uav_id = EnvironmentVariable('UAV_ID')
    uav_name_param = LaunchConfiguration('uav_name', default=uav_name)
    uav_id_param = LaunchConfiguration('uav_id', default=uav_id)

    # ------------------------------------------------------------
    # 3. Decide if we are in simulation mode (default to true, override via arg)
    sim = LaunchConfiguration('simulation', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ------------------------------------------------------------
    # 4. Declare launch arguments for extra parameters
    # ------------------------------------------------------------
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Command prefix for launching nodes, e.g., "gdb -ex run --args"',
    )

    deployment_arg = DeclareLaunchArgument(
        'deployment',
        default_value='0',
        description='Usage of deployment parameters'
    )

    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='10.0',
        description='Radius parameter'
    )
    neighbors_arg = DeclareLaunchArgument(
        'neighbors',
        default_value='10',
        description='neighbors'
    )
    geometric_arg = DeclareLaunchArgument(
        'geometric',
        default_value='1',
        description='Geometric parameter'
    )
    gaussian_arg = DeclareLaunchArgument(
        'gaussian',
        default_value='[1,1,1,0.5]',
        description='Gaussian parameter'
    )
    sim_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Simulation mode flag'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ------------------------------------------------------------
    # 5. LaunchConfigurations to capture each argument
    # ------------------------------------------------------------
    prefix = LaunchConfiguration('prefix')
    deployment = LaunchConfiguration('deployment')
    radius = LaunchConfiguration('radius')
    neighbors = LaunchConfiguration('neighbors')
    geometric = LaunchConfiguration('geometric')
    gaussian = LaunchConfiguration('gaussian')
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------
    # 6. Create the node, passing the YAML plus overrides as ROS parameters
    #    The dictionary at the end will override matching keys from the YAML file.
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
                'use_sim_time': use_sim_time_cfg,
                'deployment': deployment,
                'radius': radius,
                'neighbors': neighbors,
                'geometric': geometric,
                'gaussian': gaussian
            }
        ],
    )

    # ------------------------------------------------------------
    # 7. Add everything to the LaunchDescription
    # ------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(prefix_arg)
    ld.add_action(deployment_arg)
    ld.add_action(radius_arg)
    ld.add_action(neighbors_arg)
    ld.add_action(type_arg)
    ld.add_action(geometric_arg)
    ld.add_action(gaussian_arg)
    ld.add_action(sim_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(hemisphere_coverage_node)

    return ld
