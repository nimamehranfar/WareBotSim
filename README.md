# WareBotSim

A comprehensive warehouse robotics simulation project demonstrating autonomous navigation, task planning, and manipulation using ROS 2 Kilted and Gazebo Sim.

## Project Overview

WareBotSim simulates a warehouse environment where a mobile robot autonomously picks packages from shelves and delivers them to designated stations. The project demonstrates core ROS 2 concepts including nodes, topics, services, actions, parameters, and transforms.

## Key Features

- **Autonomous Navigation**: Nav2-based path planning and obstacle avoidance
- **Task Management**: Service-based order creation and action-based execution
- **Simulation**: Gazebo Sim with realistic physics and sensor models
- **Transform Management**: Complete TF tree with semantic location frames
- **Progress Tracking**: Real-time feedback during order fulfillment

## ROS 2 Concepts Demonstrated

### Nodes
- `state_publisher`: Publishes robot odometry as TF transforms
- `order_manager`: Manages orders and publishes semantic location frames
- `fulfill_order_server`: Executes pick-and-deliver workflows
- `static_frames`: Publishes static robot frame transforms

### Topics
- `/scan`: Laser scan data from lidar sensor
- `/odom`: Robot odometry from Gazebo
- `/cmd_vel`: Velocity commands to robot
- `/tf` and `/tf_static`: Transform data
- `/clock`: Simulation time from Gazebo

### Services
- `/warehouse/create_order`: Creates new delivery orders (custom `CreateOrder` service)

### Actions
- `/robot/fulfill_order`: Executes complete pick-and-deliver workflow (custom `FulfillOrder` action)
- `/navigate_to_pose`: Nav2 navigation action

### Parameters
- Nav2 configuration (costmaps, planners, controllers)
- Robot behavior (approach distances, speeds, tolerances)
- Sensor configuration (lidar range, update rates)

### Transforms
- Robot frames: `base_link`, `base_footprint`, `lidar_link`
- World frames: `map`, `odom`
- Semantic frames: `shelf_1`, `shelf_2`, `shelf_3`, `delivery_1`, `delivery_2`, `delivery_3`

## Repository Structure

```
WareBotSim/
├── src/
│   ├── warebotsim/                    # Main ROS 2 package (Python)
│   │   ├── warebotsim/                # Python module
│   │   │   ├── fulfill_order_server.py    # Action server for order execution
│   │   │   ├── order_manager.py           # Service server and TF publisher
│   │   │   ├── state_publisher.py         # Odometry to TF conversion
│   │   │   └── static_frames.py           # Static frame publisher
│   │   ├── launch/
│   │   │   ├── gazebo_launch.py           # Main simulation launcher
│   │   │   ├── nav2_launch.py             # Nav2 stack launcher
│   │   │   ├── gazebo_mapping_launch.py   # SLAM mapping mode
│   │   │   └── demo_launch.py             # Demo configuration
│   │   ├── config/
│   │   │   ├── nav2_params.yaml           # Nav2 configuration
│   │   │   └── slam_toolbox_online.yaml   # SLAM configuration
│   │   ├── maps/
│   │   │   ├── warehouse.yaml             # Map metadata
│   │   │   └── warehouse.pgm              # Map image
│   │   ├── worlds/
│   │   │   └── warehouse_world.sdf        # Gazebo world definition
│   │   ├── models/
│   │   │   └── warebotsim/
│   │   │       └── model.sdf              # Robot model definition
│   │   ├── meshes/                        # Visual meshes for robot
│   │   ├── scripts/                       # Helper scripts
│   │   ├── urdf/                          # URDF files (legacy)
│   │   ├── package.xml                    # Package manifest
│   │   ├── setup.py                       # Python package setup
│   │   └── setup.cfg                      # Install configuration
│   └── warebotsim_interfaces/         # Custom ROS 2 interfaces
│       ├── action/
│       │   └── FulfillOrder.action        # Order execution action
│       ├── srv/
│       │   └── CreateOrder.srv            # Order creation service
│       ├── CMakeLists.txt
│       └── package.xml
├── .gitignore
└── README.md
```

## File Descriptions

### Core Python Nodes

**`fulfill_order_server.py`**
- Action server implementing complete order workflow
- Phases: spawn package → navigate to shelf → pick → retreat → rotate → navigate to delivery → place → retreat → rotate
- Uses Nav2 for long-distance navigation and cmd_vel for precise maneuvers
- Manages package teleportation via Gazebo services
- Individual shelf approach offsets to handle geometric variations

**`order_manager.py`**
- Service server for order creation
- Action client to trigger order fulfillment
- Publishes static TF frames for semantic locations (shelves, deliveries)
- Logs order progress with monotonic feedback

**`state_publisher.py`**
- Subscribes to `/odom` from Gazebo
- Publishes TF transform: `odom → base_link`
- Ensures consistent frame naming for Nav2

**`static_frames.py`**
- Publishes static robot frames: `base_link → lidar_link`, `base_link → base_footprint`
- Provides compatibility frames for legacy components

### Launch Files

**`gazebo_launch.py`**
- Starts Gazebo Sim with warehouse world
- Launches ros_gz_bridge for topic translation
- Starts state_publisher, order_manager, and fulfill_order_server nodes
- Configures delays to ensure proper initialization order

**`nav2_launch.py`**
- Launches complete Nav2 stack:
  - Map server (loads pre-generated map)
  - AMCL (localization)
  - Controller server (local planning)
  - Planner server (global planning)
  - BT Navigator (behavior trees)
  - Behavior server (recovery behaviors)
  - Lifecycle managers
- Uses composable node container for efficiency

**`gazebo_mapping_launch.py`**
- Alternative launch for creating new maps
- Includes SLAM Toolbox configuration
- Used during map generation phase

### Configuration Files

**`nav2_params.yaml`**
- AMCL parameters (particle filter, motion model)
- Controller parameters (DWB local planner, goal tolerances)
- Planner parameters (NavFn global planner)
- Costmap parameters (static layer, obstacle layer, inflation)
- Behavior tree configuration
- Recovery behavior configuration

**`slam_toolbox_online.yaml`**
- SLAM parameters for map generation
- Scan matching configuration
- Map resolution and range settings

### World and Model Files

**`warehouse_world.sdf`**
- Defines warehouse environment: floor, walls, shelves, delivery stations
- Includes visual details (floor markings, shelf structure)
- Sets up lighting and scene configuration
- Defines collision geometries for navigation

**`models/warebotsim/model.sdf`**
- Jackal-based differential drive robot
- Includes:
  - Base chassis with wheels
  - Lidar sensor (raised to 0.75m for clear view)
  - Pickup tray with guard rails (prevents package sliding)
  - Visual meshes for realistic appearance
- Gazebo plugins:
  - Differential drive controller
  - GPU lidar sensor

### Interface Definitions

**`FulfillOrder.action`**
```
# Goal
int32 order_id
int32 shelf_id
int32 delivery_id
string package_id
---
# Result
bool success
string message
---
# Feedback
string stage
float32 progress
```

**`CreateOrder.srv`**
```
# Request
int32 shelf_id
int32 delivery_id
string package_id
---
# Response
bool accepted
int32 order_id
string message
```

## Setup and Installation

### Prerequisites
- Ubuntu 22.04 or later
- ROS 2 Kilted
- Gazebo Sim (Ionic)
- Nav2 navigation stack
- SLAM Toolbox (optional, for mapping)

### Build Instructions

1. Clone the repository:
```bash
cd ~/
git clone <repository_url> WareBotSim
cd WareBotSim
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build --symlink-install
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Running the Complete System

1. **Terminal 1: Start Gazebo and core nodes**
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 launch warebotsim gazebo_launch.py
```

This starts:
- Gazebo Sim with warehouse world
- Robot spawned at origin
- ros_gz_bridge for ROS-Gazebo communication
- state_publisher for odometry TF
- order_manager for order handling
- fulfill_order_server for task execution

2. **Terminal 2: Start Nav2 navigation stack**
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 launch warebotsim nav2_launch.py
```

This starts:
- Map server with pre-generated warehouse map
- AMCL for localization
- Global and local planners
- Controller server
- Behavior tree navigator
- Recovery behaviors

3. **Terminal 3: Create an order**
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 service call /warehouse/create_order warebotsim_interfaces/srv/CreateOrder "{shelf_id: 1, delivery_id: 1, package_id: ''}"
```

Valid IDs:
- `shelf_id`: 1, 2, or 3
- `delivery_id`: 1, 2, or 3
- `package_id`: optional custom name (auto-generated if empty)

### Monitoring Progress

**View order feedback:**
```bash
ros2 topic echo /robot/fulfill_order/_action/feedback
```

**View robot position:**
```bash
ros2 topic echo /odom
```

**View TF tree:**
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Check node status:**
```bash
ros2 node list
ros2 node info /order_manager
```

**Check action status:**
```bash
ros2 action list
ros2 action info /robot/fulfill_order
```

### Visualization with RViz2

1. **Start RViz2:**
```bash
rviz2
```

2. **Configure display:**
   - Set **Fixed Frame** to `map`
   - Add **Map** display:
     - Topic: `/map`
     - Color scheme: map
   - Add **RobotModel** display:
     - Description Topic: `/robot_description` (if published)
   - Add **TF** display:
     - Show all frames
     - Enable axes and names
   - Add **LaserScan** display:
     - Topic: `/scan`
     - Size: 0.05
     - Color: by intensity
   - Add **Path** display (optional):
     - Topic: `/plan` (global path)
     - Topic: `/local_plan` (local path)

3. **Verify map is loaded:**
```bash
ros2 topic echo /map --once
ros2 topic hz /map
ros2 param get /map_server yaml_filename
```

If map doesn't appear:
- Check that Nav2 launched successfully
- Verify lifecycle manager activated nodes:
  ```bash
  ros2 lifecycle get /map_server
  ```
  Should show `active [3]`
- Check map file path in `nav2_params.yaml`
- Verify map file exists:
  ```bash
  ls ~/WareBotSim/install/warebotsim/share/warebotsim/maps/
  ```

### Creating a New Map (Optional)

If you need to regenerate the map:

1. **Terminal 1: Launch mapping mode**
```bash
ros2 launch warebotsim gazebo_mapping_launch.py
```

2. **Terminal 2: Start SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=~/WareBotSim/install/warebotsim/share/warebotsim/config/slam_toolbox_online.yaml \
  use_sim_time:=true
```

3. **Terminal 3: Drive the robot**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

Or use the automated scan script:
```bash
bash ~/WareBotSim/src/warebotsim/scripts/auto_scan.sh
```

4. **Save the map**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/WareBotSim/src/warebotsim/maps/warehouse
```

## Troubleshooting

### Robot doesn't move
- Check Nav2 is running: `ros2 node list | grep nav2`
- Verify goal tolerance parameters in `nav2_params.yaml`
- Check for costmap inflation issues: `ros2 topic echo /local_costmap/costmap`

### Package falls off robot during turns
- Guard rails are already installed in `model.sdf`
- Check package attachment offset parameters in fulfill_order_server
- Verify rotation speed is not too high (parameter: `rotation_angular_speed`)

### Shelf_2 approach distance issue
- `shelf_2` has individual offset parameter: `shelf_2_approach_dx: -1.45`
- Adjust only this parameter without affecting shelf_1 or shelf_3
- Each shelf now has independent offset control

### Map not showing in RViz2
- Verify map_server is active: `ros2 lifecycle get /map_server`
- Check map topic: `ros2 topic echo /map --once`
- Confirm Fixed Frame is set to `map` in RViz2
- Restart Nav2 if necessary

### Progress feedback not updating
- Order manager logs progress at phase boundaries only
- Check feedback topic: `ros2 topic echo /robot/fulfill_order/_action/feedback`
- Progress values: 5% → 30% → 40% → 45% → 50% → 80% → 90% → 95% → 100%

### Rotation/retreat micro-adjustments
- These phases now use direct cmd_vel control, not Nav2
- Adjust parameters:
  - `rotation_angular_speed`: rotation speed (rad/s)
  - `rotation_tolerance`: acceptable yaw error (rad)
  - `retreat_linear_speed`: backup speed (m/s)
  - `post_pick_retreat_distance`: distance after pickup (m)
  - `post_drop_retreat_distance`: distance after delivery (m)

## Parameters Reference

### fulfill_order_server Parameters

```yaml
shelf_1_approach_dx: -1.25      # Approach offset for shelf 1 (meters)
shelf_2_approach_dx: -1.45      # Approach offset for shelf 2 (meters)
shelf_3_approach_dx: -1.25      # Approach offset for shelf 3 (meters)
delivery_approach_dx: 0.45      # Approach offset for deliveries (meters)

post_pick_retreat_distance: 0.90    # Retreat distance after pickup (meters)
post_drop_retreat_distance: 0.90    # Retreat distance after delivery (meters)
retreat_linear_speed: 0.25          # Speed during retreat (m/s)

rotation_angular_speed: 0.40        # Rotation speed (rad/s)
rotation_tolerance: 0.10            # Yaw tolerance for rotation completion (rad)

package_mount_x: 0.0            # Package offset in robot frame (meters)
package_mount_y: 0.0
package_mount_z: 0.30

spawn_on_shelf_dx: -0.30        # Package spawn offset from shelf center (meters)
spawn_on_shelf_z: 1.20          # Package spawn height (meters)
place_on_delivery_z: 1.20       # Package placement height (meters)
```

### order_manager Parameters

```yaml
shelf_ids: [1, 2, 3]            # Available shelf IDs
delivery_ids: [1, 2, 3]         # Available delivery station IDs
```

## Development Notes

### Order Execution Phases

1. **Spawn Package (5%)**: Create package on shelf
2. **Navigate to Shelf (30%)**: Drive to shelf approach point using Nav2
3. **Pick Package (40%)**: Attach package to robot
4. **Retreat from Shelf (45%)**: Back away using cmd_vel
5. **Rotate to Delivery (50%)**: Turn 180° using cmd_vel
6. **Navigate to Delivery (80%)**: Drive to delivery point using Nav2
7. **Place Package (90%)**: Teleport package to delivery station
8. **Retreat from Delivery (95%)**: Back away using cmd_vel
9. **Rotate to Ready (100%)**: Face shelves using cmd_vel

### Coordinate System

- **Origin**: Center of warehouse
- **+X**: East (towards shelves)
- **+Y**: North
- **+Z**: Up
- **Shelves**: Located at positive X
- **Deliveries**: Located at negative X

### Key Design Decisions

- **Individual shelf offsets**: Each shelf has independent approach distance to handle geometric variations
- **cmd_vel for precision**: Retreat and rotation use direct velocity control to avoid Nav2 micro-adjustments
- **Simulation time awareness**: Motion durations calculated from distance and speed, executed with simulation time
- **Monotonic progress**: Feedback only updates at phase boundaries to prevent non-monotonic values
- **Raised lidar**: Sensor at 0.75m ensures clear view over carried packages
- **Guard rails**: Pickup tray prevents package sliding during motion

## Contributing

When modifying the code:
- Maintain consistent coding style
- Keep parameter names descriptive
- Update this README for significant changes
- Test with all shelf/delivery combinations

## License

[Add your license information here]

## Authors

[Add author information here]

## Acknowledgments

- Built on ROS 2 Kilted
- Uses Nav2 navigation stack
- Gazebo Sim for physics simulation
- SLAM Toolbox for mapping