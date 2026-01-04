import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    pkg_share = get_package_share_directory("warebotsim")

    default_params = os.path.join(
        pkg_share,
        "config",
        "nav2_params.yaml",
    )

    # Resolve the installed map file path
    map_file_path = os.path.join(
        pkg_share,
        "maps",
        "warehouse.yaml"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the Nav2 YAML parameters file",
    )

    nav2_container = ComposableNodeContainer(
        name="nav2_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[
                    params_file,
                    {'yaml_filename': map_file_path}
                ],
            ),
            ComposableNode(
                package="nav2_amcl",
                plugin="nav2_amcl::AmclNode",
                name="amcl",
                parameters=[params_file],
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[params_file],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[params_file],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[params_file],
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[params_file],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[params_file],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[params_file],
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            nav2_container,
        ]
    )