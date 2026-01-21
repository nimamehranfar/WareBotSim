# WareBotSim (ROS 2 Kilted + Gazebo Sim + Nav2 + SLAM Toolbox)

WareBotSim is a warehouse robotics simulation where a Jackal-like differential-drive robot autonomously **picks up** packages from shelves and **delivers** them to delivery stations inside a Gazebo Sim world. The project demonstrates a complete robotics pipeline: simulation → sensing → navigation → task execution.

---

## Repository layout (what matters)
- `src/warebotsim/` — main Python package (nodes, launch, configs, models, worlds)
- `src/warebotsim_interfaces/` — custom ROS interfaces (**service + action**) used by the project
- `src/warebotsim/worlds/warehouse_world.sdf` — Gazebo world (warehouse)
- `src/warebotsim/models/warebotsim/model.sdf` — robot model (Jackal-like)

---

## Main runtime components
### ROS nodes (from `warebotsim`)
- **`order_manager`**
  - Provides the order API (`/warehouse/create_order`)
  - Dispatches the robot action (`/robot/fulfill_order`)
  - Publishes semantic TF frames for `shelf_*` and `delivery_*` into the `map` frame
- **`fulfill_order_server`**
  - Action server for `/robot/fulfill_order`
  - Uses Nav2 action `/navigate_to_pose` for navigation
  - Publishes `/cmd_vel` for post-pick / post-drop retreat and rotate motions
  - Spawns and places packages using Gazebo services (`/world/warehouse_world/create`, `/world/warehouse_world/set_pose`)
- **`state_publisher`**
  - Subscribes to `/odom` and broadcasts the dynamic TF transform `odom → base_link`
- **`static_frames`** (mainly used for mapping launch)
  - Publishes static TF frames (e.g., `base_link → lidar_link`, `base_footprint → base_link`)

### Custom interfaces (from `warebotsim_interfaces`)
- **Service:** `warebotsim_interfaces/srv/CreateOrder`
  - Called by the user to request delivery (shelf → delivery)
- **Action:** `warebotsim_interfaces/action/FulfillOrder`
  - Long-running robot task with feedback and result

---

## Quick prerequisites
- ROS 2 **Kilted**
- Gazebo Sim (gz) installed and working
- Nav2 installed
- SLAM Toolbox installed
- Build toolchain (`colcon`)

This repository uses `use_sim_time` with Gazebo.

---

# RUN: Main project (autonomous pickup + delivery)

## Terminal 1 — Build + start Gazebo + project nodes
**Use case:** Start the warehouse world and the ROS 2 nodes (`order_manager`, `fulfill_order_server`, `state_publisher`, bridge).
```bash
cd ~/WareBotSim
colcon build --symlink-install
source install/setup.bash
ros2 launch warebotsim gazebo_launch.py
```

## Terminal 2 — Spawn the robot into the Gazebo world
**Use case:** Insert the Jackal-like robot model into the running Gazebo world.
```bash
cd ~/WareBotSim
source install/setup.bash

gz service -s /world/warehouse_world/create   --reqtype gz.msgs.EntityFactory   --reptype gz.msgs.Boolean   --timeout 3000   --req 'sdf_filename:"/home/'"$USER"'/WareBotSim/src/warebotsim/models/warebotsim/model.sdf", name:"jackal"'
```

## Terminal 3 — Start Nav2 (navigation)
**Use case:** Enable autonomous navigation using the saved map + Nav2 stack.
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 launch warebotsim nav2_launch.py
```

## Terminal 4 — Create an order (shelf → delivery)
**Use case:** Trigger a full pickup + delivery run.
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 service call /warehouse/create_order warebotsim_interfaces/srv/CreateOrder "{shelf_id: 1, delivery_id: 2}"
```

### What should happen
1. Package spawns on the requested shelf.
2. Robot navigates near the shelf (approach offset).
3. Package attaches to robot (mount point).
4. Robot navigates near the delivery station (approach offset).
5. Package is placed at the delivery station (drop).
6. Robot retreats/rotates to clear the station.

---

# RUN: Mapping (SLAM Toolbox)

## Recommended mapping launch (minimal nodes)
**Use case:** Mapping session without running order logic.
### Terminal 1 — Build + start Gazebo mapping launch
```bash
cd ~/WareBotSim
colcon build --symlink-install
source install/setup.bash
ros2 launch warebotsim gazebo_mapping_launch.py
```

### Terminal 2 — Spawn the robot
```bash
cd ~/WareBotSim
source install/setup.bash

gz service -s /world/warehouse_world/create   --reqtype gz.msgs.EntityFactory   --reptype gz.msgs.Boolean   --timeout 3000   --req 'sdf_filename:"/home/'"$USER"'/WareBotSim/src/warebotsim/models/warebotsim/model.sdf", name:"jackal"'
```

### Terminal 3 — Drive the robot (teleop)
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Terminal 4 — Start SLAM Toolbox
```bash
cd ~/WareBotSim
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

### Terminal 5 — RViz2 (optional visualization)
```bash
cd ~/WareBotSim
source install/setup.bash
rviz2
```

# Runtime tuning: change fulfill_order_server parameters

`fulfill_order_server` declares runtime ROS parameters that can be changed while the node is running.

## List all parameters on the node
```bash
ros2 param list /fulfill_order_server
```

## Read a parameter value
```bash
ros2 param get /fulfill_order_server shelf_approach_dx
```

## Set a parameter value (takes effect for the next step/order)
```bash
ros2 param set /fulfill_order_server shelf_approach_dx -1.10
ros2 param set /fulfill_order_server shelf_2_approach_dx -0.75
ros2 param set /fulfill_order_server delivery_approach_dx 0.60
```

### Parameters available on `fulfill_order_server`
**Approach offsets (meters)**
- `shelf_approach_dx` (default: `-1.25`)
- `shelf_2_approach_dx` (default: `-0.85`)
- `delivery_approach_dx` (default: `0.55`)

**Retreat / speed**
- `post_pick_retreat_distance` (default: `0.90`)
- `post_drop_retreat_distance` (default: `0.90`)
- `retreat_linear_speed` (default: `0.25`)

**Rotation**
- `rotation_angular_speed` (default: `0.50`)
- `rotation_tolerance` (default: `0.10`)

**Package mount offset in `base_link`**
- `package_mount_x` (default: `0.0`)
- `package_mount_y` (default: `0.0`)
- `package_mount_z` (default: `0.30`)

**Spawn / place offsets**
- `spawn_on_shelf_dx` (default: `-0.30`)
- `spawn_on_shelf_z` (default: `1.20`)
- `place_on_delivery_z` (default: `1.20`)

Example: slow down rotation for smoother behavior
```bash
ros2 param set /fulfill_order_server rotation_angular_speed 0.30
```

Example: increase post-drop retreat distance
```bash
ros2 param set /fulfill_order_server post_drop_retreat_distance 1.10
```

---

# Helpful checks (quick debugging)
```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /cmd_vel -v
ros2 action list
ros2 action info /robot/fulfill_order
```

---


## Authors
Nima Mehranfar
