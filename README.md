# Hybrid Multi-Robot Fleet Manager for ROS 2 + Gazebo + Nav2

This repository contains a research-oriented multi-robot stack built around three layers:

- centralized discrete coordination with CBS
- hybrid fleet orchestration over landmarks
- decentralized execution with Nav2 in each robot namespace

The project is intended for simulation experiments, algorithm evaluation, and thesis work, not as a production fleet system.

## Architecture

The current execution pipeline is:

`task_manager -> /fleet/tasks -> fleet_manager -> CBS grid plan -> LM route -> Nav2 goals -> robot`

Main roles:

- `task_manager`: CLI task sender
- `hybrid_fleet_manager`: central coordination node and monitor
- `fleet_msgs`: custom ROS interfaces for fleet topics
- `gazebo_sim`: multi-robot Gazebo and Nav2 bringup

Key principle:

- CBS plans on discrete grid cells
- Fleet Manager maps the solution to landmark-level execution
- Nav2 executes one landmark goal at a time in continuous world coordinates

## Repository Layout

```text
fleet_msgs/
  msg/
  srv/

gazebo_sim/
  config/
  launch/
  maps/
  world/

hybrid_fleet_manager/
  config/
  launch/
  scripts/
    planning/
    runtime/
    utils/
    execution_state.py
    landmark_router.py
    task_state.py
    visualization_snapshot.py
  src/
  test/
  tools/

task_manager/
fleet_experiments/
mapf/
multi_agent_path_planning/
```

## Packages

### `fleet_msgs`

Custom interfaces used by the fleet stack.

Topics:

- `/fleet/tasks` -> `fleet_msgs/msg/FleetTask`
- `/fleet/task_status` -> `fleet_msgs/msg/FleetTaskStatus`
- `/fleet/visualization` -> `fleet_msgs/msg/FleetVisualization`

Services defined in the package:

- `fleet_msgs/srv/SubmitFleetTask`
- `fleet_msgs/srv/CancelFleetTask`

### `hybrid_fleet_manager`

Implements the hybrid coordination layer:

- receives tasks
- tracks robot landmark occupancy
- blocks occupied landmarks and nearby cells
- runs centralized CBS on the grid
- converts the result into a landmark route
- sends Nav2 goals per landmark
- replans on timeout, off-path, or blocked conditions

### `task_manager`

Simple CLI publisher for sending tasks and waiting for terminal task status.

### `gazebo_sim`

Brings up:

- Gazebo world
- multiple robots
- per-robot namespaced Nav2
- sensor topic namespacing

## Current Landmark Map

Landmarks are defined in [hybrid_fleet_manager/config/landmarks.yaml](hybrid_fleet_manager/config/landmarks.yaml).

Current default map:

```text
LM1  LM2  LM3
LM4  LM5  LM6
LM7  LM8  LM9
```

World coordinates:

- `LM1=(-2, 2)`
- `LM2=(0, 2)`
- `LM3=(2, 2)`
- `LM4=(-2, 0)`
- `LM5=(0, 0)`
- `LM6=(2, 0)`
- `LM7=(-2, -2)`
- `LM8=(0, -2)`
- `LM9=(2, -2)`

## Requirements

Current workflow assumes:

- Ubuntu 22.04
- ROS 2 Jazzy
- Gazebo Sim
- Nav2

Useful tools:

- `colcon`
- `rosdep`
- `pytest`

## Build

From the repository root:

```bash
cd ~/go2_ros2_sim_py
rosdep update
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

If you only changed the fleet stack:

```bash
cd ~/go2_ros2_sim_py
colcon build --packages-select fleet_msgs hybrid_fleet_manager task_manager gazebo_sim --symlink-install
source install/local_setup.bash
```

## Run the System

### 1. Start Gazebo and robots

```bash
cd ~/go2_ros2_sim_py
source install/local_setup.bash
ros2 launch gazebo_sim gazebo_multi_nav2_world.launch.py
```

Robot spawn positions are configured in [gazebo_sim/config/robots.yaml](gazebo_sim/config/robots.yaml).

### 2. Start Fleet Manager and monitor

```bash
cd ~/go2_ros2_sim_py
source install/local_setup.bash
ros2 launch hybrid_fleet_manager fleet_manager.launch.py
```

This launch file starts:

- `fleet_manager`
- `grid_monitor`

### 3. Send tasks

Single goal:

```bash
ros2 run task_manager task_manager --robot robot1 --goal LM9
```

Multiple goals:

```bash
ros2 run task_manager task_manager --robot robot1 --goal LM5 --goal LM8 --goal LM9
```

Wait for terminal status between goals:

```bash
ros2 run task_manager task_manager --robot robot1 --goal LM3 --goal LM1 --wait-status
```

Cancel current task:

```bash
ros2 run task_manager task_manager --robot robot1 --cancel
```

## Fleet Topics

Inspect topic types:

```bash
ros2 topic info /fleet/tasks
ros2 topic info /fleet/task_status
ros2 topic info /fleet/visualization
```

Inspect interfaces:

```bash
ros2 interface show fleet_msgs/msg/FleetTask
ros2 interface show fleet_msgs/msg/FleetTaskStatus
ros2 interface show fleet_msgs/msg/FleetVisualization
```

Echo task status:

```bash
ros2 topic echo /fleet/task_status
```

## Nav2 Namespacing

The shared Nav2 template is:

- [gazebo_sim/config/nav2_params.yaml](gazebo_sim/config/nav2_params.yaml)

It uses `<robot_namespace>` placeholders. During launch, temporary per-robot Nav2 configs are generated under `/tmp`.

Expected namespaced topics include:

- `/robot1/scan`
- `/robot2/scan`
- `/robot1/odometry/filtered`
- `/robot2/odometry/filtered`

## Fleet Manager Behavior

Current implemented behavior:

- task queue and task status publishing
- current landmark tracking for each robot
- goal occupancy checks
- occupied landmark blocking for planning
- centralized CBS planning on the occupancy grid
- landmark route extraction for execution
- per-landmark Nav2 goal dispatch
- waypoint timeout handling
- off-path detection and requeue
- visualization snapshot publishing for monitor tools

Main runtime parameters are in [hybrid_fleet_manager/config/fleet_manager_params.yaml](hybrid_fleet_manager/config/fleet_manager_params.yaml).

Important parameters:

- `waypoint_timeout_sec`
- `waypoint_min_spacing_m`
- `goal_occupied_radius_m`
- `occupied_landmark_radius_m`
- `occupied_landmark_block_radius_m`
- `landmark_capture_radius_m`
- `cbs_low_level_max_time`
- `cbs_max_high_level_nodes`
- `cbs_max_planning_time_sec`

## Grid Monitor

Run separately if needed:

```bash
cd ~/go2_ros2_sim_py
source install/local_setup.bash
ros2 run hybrid_fleet_manager grid_monitor
```

It visualizes:

- robots
- landmark graph
- global planned path
- current local execution path

It also saves a PNG snapshot to:

```text
/root/ws/src/output/fleet_live_monitor.png
```

## CBS Planner

Hybrid Fleet Manager uses:

- [hybrid_fleet_manager/scripts/planning/cbs_planner.py](hybrid_fleet_manager/scripts/planning/cbs_planner.py)

Current support:

- vertex conflicts
- edge conflicts
- constrained low-level A*
- global blocked cells
- reserved vertex constraints
- reserved edge constraints
- bounded high-level search
- planning timeout

Related tests:

- [hybrid_fleet_manager/test/test_cbs_planner.py](hybrid_fleet_manager/test/test_cbs_planner.py)
- [hybrid_fleet_manager/test/test_cbs_scenarios.py](hybrid_fleet_manager/test/test_cbs_scenarios.py)

## Tests

Run the current fleet and planner tests:

```bash
pytest -q \
  hybrid_fleet_manager/test/test_cbs_planner.py \
  hybrid_fleet_manager/test/test_cbs_scenarios.py \
  hybrid_fleet_manager/test/test_fleet_manager_logic.py
```

Run the CBS scenario demo:

```bash
PYTHONPATH=. python3 hybrid_fleet_manager/tools/run_cbs_scenarios.py
```

## Important Config Files

- [gazebo_sim/config/robots.yaml](gazebo_sim/config/robots.yaml)
- [gazebo_sim/config/nav2_params.yaml](gazebo_sim/config/nav2_params.yaml)
- [hybrid_fleet_manager/config/landmarks.yaml](hybrid_fleet_manager/config/landmarks.yaml)
- [hybrid_fleet_manager/config/fleet_manager_params.yaml](hybrid_fleet_manager/config/fleet_manager_params.yaml)
- [fleet_msgs/msg/FleetTask.msg](fleet_msgs/msg/FleetTask.msg)
- [fleet_msgs/msg/FleetTaskStatus.msg](fleet_msgs/msg/FleetTaskStatus.msg)
- [fleet_msgs/msg/FleetVisualization.msg](fleet_msgs/msg/FleetVisualization.msg)
