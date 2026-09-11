# Pick and place on the Stäubli robot

## Prerequisite

Install HPP, `hpp-exec`, the MFJA models and the Stäubli driver using the
[MFJA installation guide](https://github.com/psardin001/mfja_3rd_floor_gz/blob/main/INSTALL.md).
Use the tutorial checkout containing `tutorial_10`.

## Overview

MFJA is an academic facility located in Toulouse, France hosting a robotics platform that includes a Stäubli robot.
We use the pick-and-place problem to move a gear between two placements
on the plate. First, send the arm trajectory with no gripper commands. Then
add actions to open and close the gripper during execution.

## Terminal 1: Starting the driver

Install the matching VAL3 programs on the CS9 controller and start `ros_server`
from the teach pendant as described in the installation guide.
Check the tool, the plate pose in `environment.py`
and the gripper wiring with the instructor before executing.

In a terminal with the HPP and MFJA environments loaded:

```bash
source "$HOME/mfja-gears/setup.bash"
export ROS_DOMAIN_ID=7
read -r -p "Controller IP: " ROBOT_IP
ros2 launch mfja_staubli_manipulation_demos room_315_staubli_hardware.launch.py \
  robot_ip:="$ROBOT_IP" joint_config:="$MFJA_ROOT/installation/staubli_gears.yaml" \
  enable_io:=true
```

This starts joint feedback, the trajectory action and the gripper IO service.
The configuration sets `default_velocity_ratio` to `0.005` for position commands.

## Terminal 2: Planning

Load the same environments in a second terminal, then run:

```bash
source "$HOME/mfja-gears/setup.bash"
export ROS_DOMAIN_ID=7
cd "$HPP_TUTORIAL_DIR/tutorial_10"
python -i init.py
```

The script reads the arm position from `/joint_states` and solves the existing
`mfja/pick_and_place.py` problem in the MFJA cell. It moves the gear from
`gear_plate/placement_1` to `gear_plate/placement_2` and returns the arm to its
initial position. RandomShortcut optimizes the path, EnforceTransitionSemantic
labels its transitions, and TOPPRA time-parameterizes it.

Inspect the path before execution:

```python
v = display()
v.loadPath(p_timed)
```

## Sending the arm trajectory

For this first run, leave the gripper open and empty, and remove the gear from
the plate. Keep the robot at the position used for planning.

`p_timed(t)` gives the robot configuration at time `t` in seconds. Sample it
at regular intervals, include the subpath endpoints stored in `path_times`,
and send the resulting waypoints to the trajectory action:

```python
import numpy as np
from hpp_exec import send_trajectory

times = np.linspace(0.0, p_timed.length(), 101).tolist()
times = sorted(set(times + path_times))
configs = []
for t in times:
    q, success = p_timed(t)
    assert success
    configs.append(np.array(q))

check_initial_position(configs)
send_trajectory(
    configs, times,
    joint_names=ARM_JOINT_NAMES,
    joint_indices=list(range(6)),
    controller_topic=TRAJECTORY_ACTION,
    positions_only=True,
    wait_for_completion=lambda node, result: wait_for_motion(node, result, configs),
)
```

The first six configuration values are the arm joints; the remaining seven
describe the gear pose. `positions_only=True` sends joint positions with empty
velocity fields. The driver uses its configured speed, so the real duration
can differ from the TOPPRA duration. `wait_for_motion` waits for measured arrival
and the controller's completion signal.

The script validates the timed path. `check_initial_position` checks that the
arm is still at its planned start before sending. Ctrl+C requests cancellation
while keeping ROS available.

## Adding the gripper actions

Place the gear at `gear_plate/placement_1`, with the gripper open and empty.
Restart `python -i init.py` to plan from the current arm position and inspect
the new path.

`init.py` provides `open_gripper()` and `close_gripper()`. They call
`staubli_io.set_gripper` through `/io_interface/write_single_io`, using valve
output 0, then wait 0.5 seconds for the gripper.

Sample the timed path and split it into segments labelled with the constraint
graph transitions:

```python
from hpp_exec import segments_from_graph, segments_by_transition, print_segments

configs, times, segments = segments_from_graph(p_timed, graph)
segments_by_name = segments_by_transition(segments)
print_segments(segments)
```

`segments` lists the movements in execution order. `segments_by_name[name]`
lists the segments belonging to a given transition. Each segment has
`pre_actions` and `post_actions` lists, called before and after its arm motion.
Append a function to a list, for example `segment.pre_actions.append(open_gripper)`;
pass the function itself so it is called during execution.

Add the actions to `segment.pre_actions`: open before the first segment, close
before lifting the gear, and open before retreating after placement.
Use these transitions to find the grasp and release segments:

```python
GRASP_TRANSITION = "staubli/tool0_gripper > gear_42/stud | f_23"
RELEASE_TRANSITION = "staubli/tool0_gripper < gear_42/stud | 0-0_21"
```

Check the actions with `print_segments(segments)`, then execute:

```python
from hpp_exec import execute_segments

check_initial_position(configs)
execute_segments(
    segments, configs, times,
    joint_names=ARM_JOINT_NAMES,
    joint_indices=list(range(6)),
    controller_topic=TRAJECTORY_ACTION,
    positions_only=True,
    wait_for_completion=wait_for_motion,
)
```
