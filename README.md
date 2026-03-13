# Hybrid Multi-Robot Fleet Manager for ROS2 + Gazebo + Nav2

This repository contains a hybrid fleet coordination stack for multiple mobile robots in Gazebo:

- centralized discrete planning with MAPF / CBS on a grid
- decentralized continuous execution with Nav2 inside each robot namespace
- a lightweight Fleet Manager that converts discrete plans into Nav2 waypoint chains

The project is intended as a research and thesis platform rather than a production fleet product.

## Architecture

The execution pipeline is:

`task -> FleetManager -> CBS planner -> grid path -> world waypoints -> Nav2 -> robot`

Main components:

- `gazebo_sim`: multi-robot Gazebo + Nav2 bringup
- `hybrid_fleet_manager`: centralized hybrid fleet logic and visualizer
- `task_manager`: CLI task publisher for testing

Important principle:

- CBS operates only on grid coordinates
- Nav2 operates only on world coordinates
- the Fleet Manager is the bridge between them

## Repository Layout

```text
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
  src/
  test/
  tools/

task_manager/
```

## Requirements

Tested in the current project workflow with:

- Ubuntu 22.04
- ROS2 Jazzy
- Gazebo Sim
- Nav2

You also need standard ROS2 development tools:

- `colcon`
- `rosdep`
- `pytest`

## Build

Clone the repository and build:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone git@github.com:abutalipovvv/mapf_ros2.git .
cd ~/ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

## Multi-Robot Simulation

Robot spawn positions are configured in [robots.yaml](/home/kaisar/go2_ros2_sim_py/gazebo_sim/config/robots.yaml).

Example:

```yaml
robots:
  - name: robot1
    x_pose: '0.0'
    y_pose: '0.0'
  - name: robot2
    x_pose: '-2.0'
    y_pose: '0.0'
```

Launch Gazebo + multiple robots + Nav2:

```bash
source ~/ws/install/local_setup.bash
ros2 launch gazebo_sim gazebo_multi_nav2_world.launch.py
```

## Nav2 Namespace Substitution

`gazebo_sim/config/nav2_params.yaml` is a single shared Nav2 template.

It uses the placeholder:

```yaml
<robot_namespace>
```

Example:

```yaml
scan_topic: <robot_namespace>/scan
odom_topic: <robot_namespace>/odometry/filtered
```

During `gazebo_multi_nav2_world.launch.py`, a temporary per-robot Nav2 params file is generated automatically, for example:

- `/tmp/tb3_multi_nav2/robot1_nav2_params.yaml`
- `/tmp/tb3_multi_nav2/robot2_nav2_params.yaml`

So you keep one readable config file, but each robot still receives absolute namespaced topics such as:

- `/robot1/scan`
- `/robot2/scan`

## Fleet Manager

Fleet Manager parameters live in [fleet_manager_params.yaml](/home/kaisar/go2_ros2_sim_py/hybrid_fleet_manager/config/fleet_manager_params.yaml).

Current core functionality:

- receives tasks from `/fleet/tasks`
- reads robot poses from AMCL
- converts world poses to grid cells
- runs centralized CBS planning
- converts discrete plans into waypoint chains
- sends goals through each robot's Nav2 `navigate_to_pose`
- blocks tasks if the goal is occupied by another robot
- treats idle / finished robots as static blocked cells for planning
- replans on timeout / off-path / aborted execution

Start the Fleet Manager:

```bash
source ~/ws/install/local_setup.bash
ros2 run hybrid_fleet_manager fleet_manager --ros-args --params-file ~/ws/src/hybrid_fleet_manager/config/fleet_manager_params.yaml
```

If you prefer the launch file:

```bash
source ~/ws/install/local_setup.bash
ros2 launch hybrid_fleet_manager fleet_manager.launch.py
```

## Task Manager

Landmarks are stored in [landmarks.yaml](/home/kaisar/go2_ros2_sim_py/hybrid_fleet_manager/config/landmarks.yaml).

Current demo landmarks:

- `LM1` ... `LM9`

Example single task:

```bash
ros2 run task_manager task_manager --robot robot1 --goal LM9
```

Example sequential task list:

```bash
ros2 run task_manager task_manager --robot robot1 --goal LM5 --goal LM8 --goal LM9
```

Example waiting for terminal status between goals:

```bash
ros2 run task_manager task_manager --robot robot1 --goal LM3 --goal LM1 --wait-status
```

Cancel active task:

```bash
ros2 run task_manager task_manager --robot robot1 --cancel
```

Monitor task status:

```bash
ros2 topic echo /fleet/task_status
```

## Visualizer

The simple monitor visualizes:

- landmarks
- robots
- the landmark graph
- the active global plan on that graph

Run it with:

```bash
source ~/ws/install/local_setup.bash
ros2 run hybrid_fleet_manager grid_monitor
```

It also saves a PNG snapshot to:

```text
/root/ws/src/output/fleet_live_monitor.png
```

## CBS Planner

CBS implementation lives in:

- [cbs_planner.py](/home/kaisar/go2_ros2_sim_py/hybrid_fleet_manager/scripts/planning/cbs_planner.py)

It currently supports:

- vertex conflicts
- edge conflicts
- per-agent constraints
- constrained low-level A*
- high-level CBS search limits
- static blocked cells for non-moving robots

## Tests

Run the current planner and FleetManager logic tests:

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

- [robots.yaml](/home/kaisar/go2_ros2_sim_py/gazebo_sim/config/robots.yaml): available robots and spawn poses
- [nav2_params.yaml](/home/kaisar/go2_ros2_sim_py/gazebo_sim/config/nav2_params.yaml): shared Nav2 template with `<robot_namespace>` placeholders
- [landmarks.yaml](/home/kaisar/go2_ros2_sim_py/hybrid_fleet_manager/config/landmarks.yaml): named task goals
- [fleet_manager_params.yaml](/home/kaisar/go2_ros2_sim_py/hybrid_fleet_manager/config/fleet_manager_params.yaml): fleet planning and runtime parameters

## Current Status

The current system is already suitable as a thesis research platform for:

- conflict-free centralized planning on a discrete grid
- hybrid execution with Nav2
- multi-robot Gazebo experiments
- blocked goal handling
- conflict scenario testing and visualization

The next work should focus mainly on experiments, evaluation and comparison, not on adding product-style management features.
