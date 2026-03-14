# Minimal Thesis Experiment Protocol

This protocol is intentionally simple.

You compare only three modes:

- `discrete_cbs`
- `nav2_only`
- `hybrid`

And you evaluate them only on a small fixed scenario set based on the same landmarks.

## 1. Core Scenarios

Use only these 4 scenarios:

- `single_robot`
- `head_on_horizontal`
- `crossing_center`
- `same_goal_blocked`

They are enough to show:

- normal navigation
- head-on conflict
- crossing conflict
- shared goal conflict

## 2. What to Measure

Use only 4 criteria.

### 1. Success

Did the robots complete the scenario?

Values:

- `success`
- `failure`

### 2. Completion Time

How long did the scenario take from start to finish?

This is the main quantitative comparison for Gazebo runs.

### 3. Conflict Handling

Did the system handle the conflict correctly?

Examples:

- `no conflict`
- `conflict resolved`
- `waiting`
- `deadlock`
- `goal blocked`

### 4. Path Quality

Was the motion reasonable?

Examples:

- `short`
- `detour`
- `oscillation`
- `stopped`

This criterion can be written as a short comment instead of a strict number.

## 3. What Each Mode Proves

### `discrete_cbs`

Shows that centralized planning can generate conflict-free discrete solutions.

You mainly use it to show:

- conflicts are resolved at planning level
- the paths are valid and coordinated

### `nav2_only`

Shows decentralized execution without global multi-robot coordination.

You mainly use it to show:

- robots can complete simple tasks
- conflicts are harder to handle reliably without centralized coordination

### `hybrid`

Shows your thesis contribution:

- conflict resolution is done centrally
- execution is done practically through Nav2

This is the most important mode for the defense.

## 4. Minimal Number of Runs

You do not need a huge benchmark.

Recommended:

- `discrete_cbs`: 3 runs per scenario
- `nav2_only`: 3 runs per scenario
- `hybrid`: 3 runs per scenario

That is enough for a master's thesis demonstration if the scenarios are well chosen and clearly explained.

## 5. Minimal Result Table

You can build one simple table with these columns:

- `Scenario`
- `Mode`
- `Success`
- `Completion time, s`
- `Conflict handling`
- `Comment`

This table is easy to explain during defense.

## 6. Main Thesis Claim

With this minimal protocol, you can defend the following claim:

- pure decentralized Nav2 execution is sufficient for simple navigation tasks
- centralized CBS resolves conflicts correctly in discrete space
- the hybrid system transfers this centralized coordination into practical execution in Gazebo with Nav2

That is enough. You do not need to pretend this is an industrial benchmark.
