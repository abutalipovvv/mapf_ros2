# Minimal Run Matrix

This is the simplified execution plan for the thesis.

## Scenarios

Run only these 4:

1. `single_robot`
2. `head_on_horizontal`
3. `crossing_center`
4. `same_goal_blocked`

## Modes

For each scenario, run:

1. `discrete_cbs`
2. `nav2_only`
3. `hybrid`

## What to Write Down

After each run, record only:

- `success`
- `completion_time_sec`
- `conflict_handling`
- `comment`

Examples of `conflict_handling`:

- `no_conflict`
- `resolved`
- `waiting`
- `blocked_goal`
- `deadlock`

Examples of `comment`:

- `smooth`
- `small detour`
- `stopped near goal`
- `needed replan`

## Phase A. Discrete CBS

Run:

```bash
PYTHONPATH=./fleet_experiments:./hybrid_fleet_manager \
python3 -m fleet_experiments.run_experiment --mode discrete_cbs --scenario <scenario> --runs 3
```

Use this phase mainly to show:

- the planner found a valid conflict-free solution
- the planner resolved the conflict logically

## Phase B. Nav2-Only

Do not launch `fleet_manager`.

Start Gazebo and Nav2:

```bash
source install/local_setup.bash
ros2 launch gazebo_sim gazebo_multi_nav2_world.launch.py
```

Print the direct Nav2 goal commands:

```bash
PYTHONPATH=./fleet_experiments:./hybrid_fleet_manager \
python3 -m fleet_experiments.run_experiment --mode nav2_only --scenario <scenario> --status pending
```

Then manually run the printed commands.

After the run, save the result:

```bash
PYTHONPATH=./fleet_experiments:./hybrid_fleet_manager \
python3 -m fleet_experiments.run_experiment \
  --mode nav2_only \
  --scenario <scenario> \
  --status completed \
  --execution-time-sec 20.0 \
  --success-rate 1.0 \
  --notes "conflict_handling=waiting; comment=smooth"
```

## Phase C. Hybrid

Start Gazebo and Nav2:

```bash
source install/local_setup.bash
ros2 launch gazebo_sim gazebo_multi_nav2_world.launch.py
```

In another terminal:

```bash
source install/local_setup.bash
ros2 launch hybrid_fleet_manager fleet_manager.launch.py
```

Print the task commands:

```bash
PYTHONPATH=./fleet_experiments:./hybrid_fleet_manager \
python3 -m fleet_experiments.run_experiment --mode hybrid --scenario <scenario> --status pending
```

Then manually run the printed commands.

After the run, save the result:

```bash
PYTHONPATH=./fleet_experiments:./hybrid_fleet_manager \
python3 -m fleet_experiments.run_experiment \
  --mode hybrid \
  --scenario <scenario> \
  --status completed \
  --execution-time-sec 16.0 \
  --success-rate 1.0 \
  --notes "conflict_handling=resolved; comment=needed replan"
```

## Final Table

Make one table:

- `Scenario`
- `Mode`
- `Success`
- `Completion time, s`
- `Conflict handling`
- `Comment`

That is enough for a clear and defendable thesis evaluation.
