# WareBotSim

Warehouse robot simulation and navigation project (ROS 2 Kilted + Gazebo Sim).

## What this project demonstrates (ROS 2 concepts)

- **Nodes**: separate executables for state publishing, order management, and order execution.
- **Topics**: `/scan`, `/odom`, `/tf`, `/tf_static`, `/cmd_vel`, `/clock`.
- **Services**: `/warehouse/create_order` (custom service) creates a new order.
- **Actions**: `/robot/fulfill_order` (custom action) executes a full pick-and-deliver workflow.
- **Parameters**: Nav2 parameters (controller, costmaps, AMCL, etc.) and behavior parameters for the action server.
- **TF**: static semantic frames for shelves and delivery points (`shelf_*`, `delivery_*`), plus robot frames.

## Repository layout

```
.
├─ src/
│  ├─ warebotsim/                      # main package (Python)
│  │  ├─ warebotsim/                   # Python module
│  │  │  ├─ fulfill_order_server.py    # action server: executes an order end-to-end
│  │  │  ├─ order_manager.py           # service server + action client + semantic TF publisher
│  │  │  ├─ state_publisher.py         # publishes TF from odometry (robot pose)
│  │  │  └─ ...
│  │  ├─ launch/
│  │  │  ├─ gazebo_launch.py           # starts Gazebo + bridge + project nodes
│  │  │  ├─ nav2_launch.py             # starts Nav2 (map_server, AMCL, planner, controller, BT navigator, etc.)
│  │  │  ├─ gazebo_mapping_launch.py   # optional mapping setup (SLAM)
│  │  │  └─ demo_launch.py             # optional demo entrypoint
│  │  ├─ config/
│  │  │  └─ nav2_params.yaml           # Nav2 configuration
│  │  ├─ maps/
│  │  │  ├─ warehouse.yaml             # map metadata used by Nav2 map_server
│  │  │  └─ *.pgm                      # map image
│  │  ├─ worlds/
│  │  │  └─ warehouse_world.sdf         # Gazebo world (walls, floor, shelves, delivery points)
│  │  ├─ models/                       # robot/world models
│  │  └─ scripts/                      # helper scripts (optional)
│  └─ warebotsim_interfaces/           # custom interfaces package
│     ├─ action/FulfillOrder.action
│     ├─ srv/CreateOrder.srv
│     └─ ...
```

## Runtime architecture

1. **Gazebo Sim** runs the world and robot model.
2. **ros_gz_bridge** bridges Gazebo topics to ROS 2 (`/scan`, `/odom`, `/cmd_vel`, `/clock`).
3. **state_publisher** publishes robot TF (map/odom/base_link chain as configured).
4. **order_manager**
   - publishes static TF frames for `shelf_*` and `delivery_*` (semantic locations)
   - serves `/warehouse/create_order`
   - sends an action goal to `/robot/fulfill_order`
   - logs coarse progress and robot yaw during execution
5. **fulfill_order_server**
   - receives the goal
   - runs each phase (Nav2 navigation + package teleportation + rotate-in-place)
6. **Nav2** handles global/local planning and execution (`NavigateToPose` action).

## Commands (fill in any local paths if needed)

### 1) Build and source
```bash
cd ~/WareBotSim
colcon build --symlink-install
source install/setup.bash
```

### 2) Start Gazebo + project nodes (bridge, TF, service/action)
```bash
ros2 launch warebotsim gazebo_launch.py
```

### 3) Start Nav2 (map + localization + navigation)
```bash
ros2 launch warebotsim nav2_launch.py
```

### 4) Create an order (service)
```bash
ros2 service call /warehouse/create_order warebotsim_interfaces/srv/CreateOrder "{shelf_id: 1, delivery_id: 1, package_id: ''}"
```

### 5) (Optional) Send the action directly
```bash
ros2 action send_goal /robot/fulfill_order warebotsim_interfaces/action/FulfillOrder "{order_id: 1, shelf_id: 1, delivery_id: 1, package_id: 'pkg_001'}"
```

## How to verify which map Nav2 is using (RViz / CLI)

Nav2’s `map_server` loads the map file specified by its `yaml_filename` parameter.

- Check the parameter:
```bash
ros2 param get /map_server yaml_filename
```

- Confirm the map topic is being published:
```bash
ros2 topic echo /map --once
ros2 topic hz /map
```

If RViz shows an empty view:
- Set **Fixed Frame** to `map`
- Add a **Map** display and set topic to `/map`
- Add **TF** display to confirm the transform tree exists
- Confirm Nav2 nodes are running and activated (lifecycle):
```bash
ros2 node list | grep nav2
ros2 lifecycle get /map_server
```

## Notes on phase progress

Progress is reported as a monotonic value from 0.0 to 1.0 and only advances at phase milestones.
This keeps progress from bouncing due to navigation feedback distance changes.
