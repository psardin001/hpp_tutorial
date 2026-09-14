# Exercise 10: Pick and place on the Stäubli robot

This folder contains the MFJA planning exercises, execution scripts, scene
configuration and gripper helpers.

## Planning

Install HPP, TOPPRA and the MFJA model/configuration packages. Set
`ROS_PACKAGE_PATH` to their parent directories so HPP can resolve `package://`
resources. Planning needs neither ROS Python modules nor a running ROS system.
The [MFJA installation guide](https://github.com/psardin001/mfja_3rd_floor_gz/blob/main/INSTALL.md)
provides the complete planning and execution environment.

```bash
source "$HOME/mfja-gears/setup.bash"
cd "$HPP_TUTORIAL_DIR/exercise_10"
python -i pick_and_place.py
```

`pick_and_place.py` builds the MFJA scene and constraint graph, moves the gear
from `gear_plate/placement_1` to `gear_plate/placement_2`, and returns the arm
to its starting configuration. It uses the configured default arm posture,
solves with `ManipulationPlanner`, optimizes with `RandomShortcut`, restores
transition labels and applies TOPPRA. Each timed subpath is collision-validated.

Inspect the result in Python:

```python
v = display()
v.loadPath(p_timed)
```

Importing the planner builds the problem. Call `solve()` to plan, or
`solve(q_start)` to use six arm joint angles in radians. It returns the raw path,
optimized path, timed path and timed subpath boundaries.

## Robot driver

Install the matching VAL3 programs on the CS9 controller and start `ros_server`
from the teach pendant as described in the installation guide. Check the tool,
the plate pose in `environment.py` and the gripper wiring with the instructor.

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
The driver configuration sets `default_velocity_ratio` to `0.005`.

## Execution scripts

Both scripts start with `from pick_and_place import *`, read the current arm
position from `/joint_states`, and call `solve(q_start)` before sending motion.
They check that the arm is still at the planned start and wait for measured
arrival and controller completion. ROS topics, tolerances and gripper settings
come from `two_gears_execution.yaml`.

These commands send robot motion when run. In a second terminal, load the same
environment, set `ROS_DOMAIN_ID=7`, and enter `exercise_10`.

For an arm-only run, leave the gripper open and empty and remove the gear:

```bash
python pick_and_place_execute_without_actions.py
```

This samples the timed path, including every subpath boundary, and calls
`send_trajectory`. It sends no gripper commands and uses no pre/post-actions.

For the complete pick-and-place cycle, put the gear at `gear_plate/placement_1`
and leave the gripper empty:

```bash
python pick_and_place_execute_with_actions.py
```

This uses `segments_from_graph` and `execute_segments`. The pre-actions open
before the first movement, close before lifting the grasped gear (`f_23`),
and open before retreating from the placement (`0-0_21`). Gripper commands use
valve output 0 and wait 0.5 seconds for settling.

Both examples use `positions_only=True`: the driver controls movement speed,
so actual duration can differ from TOPPRA timing. Ctrl+C requests cancellation.
The execution scripts have not been physically validated.

## Other MFJA exercises

```bash
python -i one_gear.py
# Or:
python -i two_gears.py
```

Display their timed path with `Viewer(robot)` and `v.loadPath(p2)` after
initializing the viewer and setting its problem and graph.

The existing `one_gear_execute.py`, `two_gears_execute.py`, `calibrate_gears.py`,
`environment.py`, `staubli_io.py`, `tools.py` and execution YAML are also here.
After calibration and driver setup, the gear execution commands are:

```bash
python one_gear_execute.py --execute --all
# Or:
python two_gears_execute.py --execute --all
```

Omit `--all` to select the next segment interactively. These gear examples send
TOPPRA velocities and operate the gripper between confirmed stops.
