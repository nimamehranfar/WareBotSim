import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world')

    default_world = os.path.join(
        os.path.expanduser('~'),
        'WareBotSim',
        'src',
        'warebotsim',
        'worlds',
        'warehouse_world.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock (Gazebo) if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the Gazebo world SDF'
        ),

        # --- Gazebo Sim (Ionic) ---
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # --- Gazebo ↔ ROS bridge ---
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            respawn=True,
            respawn_delay=1.0,
        ),

        # Delay a bit so /odom and /scan exist
        TimerAction(
            period=2.0,
            actions=[
                # odom -> base_link
                Node(
                    package='warebotsim',
                    executable='state_publisher',
                    name='state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    respawn=True,
                    respawn_delay=1.0,
                ),

                # base_link -> lidar_link (+ compatibility)
                Node(
                    package='warebotsim',
                    executable='static_frames',
                    name='static_frames',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    respawn=True,
                    respawn_delay=1.0,
                ),
            ],
        ),
    ])
