# MFJA manipulation

## Prerequisite

Having completed [tutorial 2](../tutorial_2/README.md) and
[tutorial 3](../tutorial_3/README.md).

The exercise uses the TX2-60L, its gripper, the Room 315 collision meshes, the
Stäubli table, and one box from the MFJA 3rd floor repository.
Viser displays the room shell, glass panels, Stäubli table and adjacent rails.
HPP checks the panels, table and rails for collision.

## Overview

The exercise has four parts:

1. plan one collision-free arm trajectory and display it with Viser;
2. plan a classic pick and place on the Stäubli table;
3. turn the graph path into `hpp-exec` execution segments;
4. execute the segments on the Stäubli after the instructor preflight.

`init.py` builds the scene. `exercise.py` contains the parts to complete.
`solution.py` is the reference implementation used by `execute.py`.

## Installation

Follow the [student installation guide](./INSTALL.md). It uses the portable
installer from the MFJA 3rd floor repository to prepare HPP, the ROS packages
and the Viser Python environment.

## Starting the exercise

Load the exercise environment:

```bash
export MFJA_WORK_DIR="${MFJA_WORK_DIR:-$HOME/mfja}"
source "$MFJA_WORK_DIR/setup.bash"
ros2 run mfja_staubli_manipulation_demos room315_check_setup.sh
```

From this directory, start the student file:

```bash
./run.sh
```

The useful variables are:

- `robot`, `problem`, and `graph`;
- `q_start`: the initial arm posture with the box on the table;
- `q_init`: the working posture with the box at the pickup pose;
- `q_goal`: the same arm posture with the box at the placement pose;
- `display()`: create the Viser viewer.

## Part 1: Plan an arm trajectory

Complete the first part of `exercise.py`. The box stays on the table, so plan
on the free-state loop transition `"Loop | f"`.

Try first, then use this API help:

```python
planner = TransitionPlanner(problem)
transition = graph.getTransition("Loop | f")
planner.setTransition(transition)
success, p_arm, report = planner.directPath(q_start, q_init, True)
```

`True` asks HPP to validate the complete path. If `success` is false, inspect
`report` and correct the path.

Visualize the result:

```python
v = display()
v(q_start)
v(q_init)
v.loadPath(p_arm)
```

## Part 2: Plan the pick and place

The constraint graph uses these SRDF elements:

| Role | Name |
|---|---|
| gripper | `staubli/tool0_gripper` |
| handle | `box/top_handle` |
| object contact | `box/bottom_surface` |
| support contact | `staubli_table/drop_zone` |

The arm part of `q_init` and `q_goal` is identical. Only the box pose changes.
Generate the four pickup waypoints, a transfer, then the four release
waypoints:

```text
staubli/tool0_gripper > box/top_handle | f_01
staubli/tool0_gripper > box/top_handle | f_12
staubli/tool0_gripper > box/top_handle | f_23
staubli/tool0_gripper > box/top_handle | f_34
Loop | 0-0
staubli/tool0_gripper < box/top_handle | 0-0_43
staubli/tool0_gripper < box/top_handle | 0-0_32
staubli/tool0_gripper < box/top_handle | 0-0_21
staubli/tool0_gripper < box/top_handle | 0-0_10
```

For each pickup transition, project a target and validate it:

```python
transition = graph.getTransition(transition_name)
success, target, error = graph.generateTargetConfig(
    transition, source, initializer
)
valid, report = transition.pathValidation().validateConfiguration(target)
```

The first `initializer` is a random configuration whose box pose is copied
from `source`. Initialize each following projection with the previous
waypoint. Retry when projection or collision validation fails.

Plan each movement on its transition:

```python
planner.setTransition(transition)
success, path, report = planner.directPath(source, target, True)
```

If the direct path fails, use `TransitionPlanner.planPath`:

```python
goals = np.zeros((1, robot.configSize()), order="F")
goals[0, :] = target
path = planner.planPath(source, goals, True)
```

Append the nine paths to a single path vector:

```python
p_pick_place = hpp_path.Vector(robot.configSize(), robot.numberDof())
for path in pick_paths:
    p_pick_place.appendPath(path)
```

Display it with `v.loadPath(p_pick_place)`.

## Part 3: Build execution segments

Preserve transition semantics through time parameterization:

```python
semantic = EnforceTransitionSemantic(problem)
p_semantic = semantic.optimize(p_pick_place)

optimizer = SimpleTimeParameterization(problem)
optimizer.order = 2
optimizer.safety = 0.5
optimizer.maxAcceleration = 0.5
p_timed = optimizer.optimize(p_semantic)
```

Ask `hpp-exec` to expose the graph segments:

```python
configs, times, segments = segments_from_graph(p_timed, graph)
print_segments(segments)
segments_by_name = segments_by_transition(segments)
```

Identify the segments that carry the gripper actions. In this graph the gripper
closes before `f_23` and opens before `0-0_21`:

```python
GRASP_TRANSITION = "staubli/tool0_gripper > box/top_handle | f_23"
RELEASE_TRANSITION = "staubli/tool0_gripper < box/top_handle | 0-0_21"

assert len(segments_by_name[GRASP_TRANSITION]) == 1
assert len(segments_by_name[RELEASE_TRANSITION]) == 1
```

`execute.py` maps the Stäubli gripper actions to these transition names.

Run the reference plan to compare your result:

```bash
./run.sh python3 -i solution.py
```

Then use Viser:

```python
v = display()
v.loadPath(p_arm_timed)
v.loadPath(p_timed)
```

## Part 4: Execute on the real robot

**Instructor procedure. Begin after physical commissioning and approval.**

Physical commissioning requires measured tool, workpiece, handle and cell
geometry, conservative collision margins, payload data and reduced speed
limits.

Before enabling the robot, the instructor must:

1. update the teaching geometry with the measured hardware geometry;
2. set `Q_ARM_START`, the pickup pose and the placement pose from the real
   cell, then compute the full plan;
3. verify every path in Viser and repeat the collision checks with conservative
   margins;
4. test the pneumatic gripper unloaded, verify module 2 and pin 0, and leave
   the gripper open;
5. confirm `/joint_states` and
   `/manipulator_controller/joint_trajectory_action`, clear the cell, select
   reduced speed and obtain operator approval.

The MFJA 3rd floor repository provides the Stäubli hardware launch. Use the
cell's commissioned ROS domain in each terminal. On the authorized robot
computer, start the launch with the controller IP:

```bash
export MFJA_WORK_DIR="${MFJA_WORK_DIR:-$HOME/mfja}"
source "$MFJA_WORK_DIR/setup.bash"
read -r -p "Cell ROS domain ID: " ROS_DOMAIN_ID
export ROS_DOMAIN_ID
read -r -p "Staubli controller IP: " ROBOT_IP
ros2 launch mfja_staubli_manipulation_demos \
  room_315_staubli_hardware.launch.py robot_ip:="$ROBOT_IP"
```

Read the measured configuration from a second terminal:

```bash
export MFJA_WORK_DIR="${MFJA_WORK_DIR:-$HOME/mfja}"
source "$MFJA_WORK_DIR/setup.bash"
read -r -p "Cell ROS domain ID: " ROS_DOMAIN_ID
export ROS_DOMAIN_ID
ros2 run mfja_staubli_demos room315_read_configuration.py
```

Copy the measured arm position into `Q_ARM_START`, update the measured pickup
and placement poses, then compute and review the complete plan. After the
instructor preflight, run the tutorial executor from a third terminal:

```bash
export MFJA_WORK_DIR="${MFJA_WORK_DIR:-$HOME/mfja}"
source "$MFJA_WORK_DIR/setup.bash"
read -r -p "Cell ROS domain ID: " ROS_DOMAIN_ID
export ROS_DOMAIN_ID
./run.sh python3 execute.py --confirm-real
```

`execute.py` accepts execution when the measured joints match `q_init` within
0.03 rad. It sends the arm segments through `hpp-exec.execute_segments` and
uses the Stäubli IO service for the grasp and release actions.

## Optional Gazebo demonstration

The reference trajectory can also be displayed and replayed in Gazebo. The
launcher, executor and commands are kept in the
[`gazebo`](./gazebo/README.md) directory.

## API documentation

- [HPP documentation](https://gepetto.github.io/doc/hpp-doc/doxygen-html/index.html)
- [hpp-exec documentation](https://gepetto.github.io/doc/hpp-exec/doxygen-html/index.html)
