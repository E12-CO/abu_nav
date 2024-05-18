# Launch file for Area 1 -> Area 3 navigator for ABU R0b0c0n 2O24
import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'abu_nav'

    nav_param_dir = launch.substitutions.LaunchConfiguration(
        'nav_param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'params',
            'abu_params.yaml'))
    
    abu_nav_instant = launch_ros.actions.Node(
        package='abu_nav',
        executable='abu_nav_node',
        output='screen',
        parameters=[nav_param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'nav_param_dir',
            default_value=nav_param_dir,
            description='Full path to main parameter file to load'),
        abu_nav_instant,
    ])
