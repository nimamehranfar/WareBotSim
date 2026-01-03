import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    warebotsim_share = get_package_share_directory('warebotsim')
    nav2_share = get_package_share_directory('nav2_bringup')

    bringup_launch = os.path.join(
        nav2_share, 'launch', 'bringup_launch.py'
    )

    map_yaml = os.path.join(
        warebotsim_share, 'maps', 'warehouse.yaml'
    )

    params_yaml = os.path.join(
        warebotsim_share, 'config', 'nav2_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam', 'False'),          # MUST be a STRING
                ('map', map_yaml),
                ('params_file', params_yaml),
                ('autostart', 'true'),
            ],
        ),
    ])
