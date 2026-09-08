# Two gears: segment-by-segment Stäubli validation

This is a commissioning procedure, **not physically validated**. Only the
operator runs hardware commands. Keep the cell clear, use the site's approved
reduced-speed procedure and have the protective stop available.

## 1. Check the physical scene

`two_gears_execution.yaml` contains reference poses for the gear plate and
gear support, with `calibrated: false`. These are not measurements.

Measure both fixture origins relative to the Stäubli base and replace
`gear_plate_pose` and `gear_support_pose`: metres, then quaternion **x y z w**.
Use the origins in `gear_plate.urdf` and `gear_support.urdf`, not the centres of
the gears. Their SRDF frames define the two starting slots and two destination
slots; the script computes gear poses from these frames. Verify the real gears,
slots, tool/TCP and gripper match those models. Set `calibrated: true` only after
checking them, then create a new plan.

The execution planner reads `room_315_only.world` and
`robots_room_315_only.yaml` from the supplied MFJA checkout. It transforms the
world's rigid collision fixtures into the robot-base frame and saves that scene
with the plan. It does not use the older `room315_cell.urdf`. Check the actual
table, rails and surrounding fixed equipment against the preview: a world-file
pose is not a measurement of a movable object. Sun and ground plane are omitted.
The inherited planner also disables gripper-to-gear collisions; inspect finger
clearances yourself.

## 2. Prepare the local software

In each terminal, activate an environment providing HPP, TOPPRA, Viser and
ROS 2 Jazzy with Python 3.12. Set these variables to your source checkouts
and ROS workspace:

```bash
export HPP_TUTORIAL_DIR=/path/to/hpp_tutorial
export MFJA_ROOT=/path/to/mfja_3rd_floor_gz
export MFJA_WS=/path/to/mfja_ws
```

Use the patched `hpp-manipulation` (short transitions), `hpp-exec` (nested
path segmentation), and `hpp-python` with `InterpolatedPath.interpolationPoints`,
the current MFJA gear models, and this exercise branch.
The planner uses `tools.Toppra` to stop at geometric junctions, with a velocity
scale of 0.5 and an acceleration setting of 0.45 rad/s². Regenerate saved plans
created before this timing change. These are planning settings; the VAL3
execution behavior is described below.
The local `Staubli_ROS2` driver changes preserve fractional timestamps, respect
requested speeds below the fallback and mark the final point even when no
velocities are provided. Rebuild the changed driver before hardware bringup:

```bash
cd "$MFJA_WS"
CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --merge-install --symlink-install --executor sequential \
  --base-paths "$MFJA_ROOT/Staubli_ROS2" \
  --packages-select industrial_robot_client --cmake-force-configure
source "$MFJA_WS/install/setup.bash"
```

Make a local copy of `Staubli_ROS2/staubli_val3_driver/config/tx2_60l_streaming.yaml`.
Keep its joint names and commissioned velocity limits. Under
`joint_trajectory_interface.ros__parameters`, add `default_velocity_ratio` with
the positive ratio approved for commissioning (at most 1). The default is 0.1;
this fallback is used for zero endpoint velocities and missing velocities.
It is **not a speed cap**. Apply the controller's commissioned reduced-speed
settings as well.

The installed VAL3 program must match the local driver protocol: velocities
above 2 mark the final point and cause an unblended stop. VAL3 otherwise uses
blended `movej`, not TOPPRA time tracking. The preview duration, Cartesian path
and acceleration bounds are therefore not guarantees about physical motion.
Verify low-speed tracking and stopping before attempting contact or insertion.

## 3. Read the starting joints and save one plan

In a **ROS terminal**, with the authorized controller prepared, bring up the
local hardware stack. Replace both placeholders with commissioned values:

```bash
source /opt/ros/jazzy/setup.bash
source "$MFJA_WS/install/setup.bash"
ros2 launch mfja_staubli_manipulation_demos room_315_staubli_hardware.launch.py \
  robot_ip:=CONTROLLER_IP joint_config:=/absolute/path/to/commissioned.yaml
```

In another sourced ROS terminal, check the endpoints and read the current pose:

```bash
ros2 action info /manipulator_controller/joint_trajectory_action
ros2 service type /io_interface/write_single_io
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
```

Require one physical joint-state publisher, the expected trajectory server and
`staubli_msgs/srv/WriteSingleIO`. Record positions in **joint_1 … joint_6 order,
in radians**. Do not assume the message's array order. Both gears must be in their
plate slots and the gripper empty. This runner does not recover a partly finished
exercise or move the arm to its starting pose.

In a separate **HPP terminal**, plan from those six measured values:

```bash
cd "$HPP_TUTORIAL_DIR/mfja"
python two_gears_execute.py plan /tmp/two-gears.json \
  --mfja-root "$MFJA_ROOT" \
  --q-start J1 J2 J3 J4 J5 J6
python two_gears_execute.py inspect /tmp/two-gears.json
python two_gears_execute.py view /tmp/two-gears.json --segment 0
```

Replace `J1 … J6`, then open the printed Viser URL and press Enter to play.
Preview **every** segment by changing its index. Reuse this saved JSON for all
execution; `view` and `execute` never replan. If fixtures or model files change,
replan and review again.

## 4. Execute one segment, then inspect

First verify gripper wiring/polarity with an empty gripper using the site's
commissioning procedure. This script assumes **VALVE_OUT, pin 0, true = open**.
The pin/service and settling delay are in the YAML. An IO acknowledgement is
not confirmation of a successful grasp.

In the ROS terminal, select the same source checkout and saved plan:

```bash
cd "$HPP_TUTORIAL_DIR/mfja"
python two_gears_execute.py execute /tmp/two-gears.json --segment 0
```

This command **operates the gripper and moves the robot**. It checks the starting
joints, sets the displayed gripper state, sends only the selected segment through
`hpp-exec`, then checks the endpoint. The default joint tolerance is 0.03 rad;
that check does not establish insertion accuracy or object position.

For the usual 13-segment plan, expect:

| Segment | Gripper before motion | Check before advancing |
|---|---|---|
| 0–1 | open | Approach gear 1; fingers aligned before closing |
| 2 | close | Gear 1 grasped and lifted clear |
| 3–4 | close | Transfer, then one descent; gear 1 seated before opening |
| 5 | open | Local withdrawal; gear 1 stays on its support |
| 6–7 | open | Approach gear 2; fingers aligned before closing |
| 8 | close | Gear 2 grasped and lifted clear |
| 9–10 | close | Transfer, then one descent; gear 2 seated before opening |
| 11–12 | open | Local withdrawal, then departure; both gears remain placed |

Use the saved plan's printed transition names as the authority. If its order
differs, review it before motion. Before **each** command, compare the real gear
locations and gripper state to the preview. After it returns, confirm the robot
is stationary, check seating/clearance, and only then run the next index manually.
Never use a shell loop for this first validation.

On a failed grasp, unexpected movement, timeout or endpoint mismatch, stop using
the site's procedure. A timeout requests cancellation but does not prove the
robot has stopped. Do not advance or blindly replay a segment: opening may drop
a carried gear. Inspect the physical state and recover under operator control;
start a new plan only once both gears are back in the modeled initial slots.

## Offline checks

These do not connect to a controller:

```bash
cd "$HPP_TUTORIAL_DIR"
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover \
  -s mfja -p test_two_gears_execute.py
```
