from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = FileContent(
        PathJoinSubstitution([FindPackageShare('warebotsim'), 'jackal.urdf']))
    robot_description = ParameterValue(urdf, value_type=str)

    warehouse_urdf = FileContent(
        PathJoinSubstitution([
            FindPackageShare('warebotsim'),
            'warehouse.urdf'
        ])
    )
    warehouse_description = ParameterValue(warehouse_urdf, value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='robot',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
            arguments=[urdf]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='warehouse_state_publisher',
            namespace='warehouse',
            output='screen',
            parameters=[{
                'robot_description': warehouse_description
            }],
        ),
        #Node(
         #   package='tf2_ros',
          #  executable='static_transform_publisher',
           # name='world_to_odom',
            #arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
            #output='screen',
        #),
        # Node(
          #  package='joint_state_publisher_gui',
           # executable='joint_state_publisher_gui',
            #name='joint_state_publisher_gui',
            #output='screen',
        #),
        Node(
            package='warebotsim',
            executable='state_publisher',
            name='state_publisher',
            namespace='robot',
            output='screen'),
    ])