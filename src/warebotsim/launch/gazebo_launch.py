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
        # Start early so /clock is available quickly.
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
            parameters=[{'use_sim_time': True, 'use_nav2': True}],
            respawn=True,
            respawn_delay=1.0,
        ),

        # --- TF from odometry ---
        # Delay a bit so the bridge is already up and publishing /odom.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='warebotsim',
                    executable='state_publisher',
                    name='state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    respawn=True,
                    respawn_delay=1.0,
                ),
            ],
        ),

        # --- Semantic + static TFs + order service/action client ---
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='warebotsim',
                    executable='order_manager',
                    name='order_manager',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    respawn=True,
                    respawn_delay=1.0,
                ),
            ],
        ),

        # --- Robot action server (your project already has it) ---
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='warebotsim',
                    executable='fulfill_order_server',
                    name='fulfill_order_server',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    respawn=True,
                    respawn_delay=1.0,
                ),
            ],
        ),
    ])
