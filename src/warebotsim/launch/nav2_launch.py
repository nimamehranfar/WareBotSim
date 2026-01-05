import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
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

    map_file_path = os.path.join(
        pkg_share,
        "maps",
        "warehouse.yaml"
    )

    # Resolve Behavior Tree XML Paths (Using standard one now that behavior server will work)
    nav2_bt_pkg_share = get_package_share_directory("nav2_bt_navigator")
    nav_to_pose_xml = os.path.join(
        nav2_bt_pkg_share,
        "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    nav_through_poses_xml = os.path.join(
        nav2_bt_pkg_share,
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml"
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

    # --- FIX: Run behavior_server as a standalone node to avoid class-loader errors ---
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
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
                parameters=[params_file, {'yaml_filename': map_file_path}],
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
            # behavior_server removed from here and moved to standalone Node above
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[
                    params_file,
                    {
                        "default_nav_to_pose_bt_xml": nav_to_pose_xml,
                        "default_nav_through_poses_bt_xml": nav_through_poses_xml,
                    }
                ],
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
            behavior_server_node, # Added standalone node
            nav2_container,
        ]
    )