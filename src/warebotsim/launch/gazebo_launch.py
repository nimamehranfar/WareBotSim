from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # --- Gazebo Sim ---
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '/home/' + __import__('os').getlogin() +
                '/WareBotSim/src/warebotsim/worlds/warehouse_world.sdf'
            ],
            output='screen'
        ),

        # --- Gazebo ↔ ROS bridge ---
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ],
            output='screen'
        ),

        # --- TF from odometry ---
        Node(
            package='warebotsim',
            executable='state_publisher',
            name='state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # --- Semantic + static TFs ---
        Node(
            package='warebotsim',
            executable='order_manager',
            name='order_manager',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
