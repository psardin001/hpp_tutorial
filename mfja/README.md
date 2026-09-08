# MFJA gear transfer

## Prerequisite

Complete [tutorial 2](../tutorial_2/README.md) before this exercise. Tutorial 3
provides additional explanations about manufacturing applications, waypoint
states and security margins.

Install and source the
[MFJA 3rd floor repository](https://github.com/psardin001/mfja_3rd_floor_gz).
This exercise only needs its `mfja_3rd_floor_description` package; it does not
start ROS, Gazebo or a robot controller.

## Goal

Use the Stäubli TX2-60L to transfer a 42 mm gear from the first location on the
gear plate to the gear support.

The scene contains four models:

| Model | Root joint | Role |
|---|---|---|
| `staubli` | anchor | manipulator |
| `gear_plate` | anchor | initial fixture |
| `gear_support` | anchor | destination fixture |
| `gear_42` | free-flyer | manipulated object |

The constraint graph allows only these grasps:

| Gripper | Handle |
|---|---|
| `staubli/tool0_gripper` | `gear_42/stud` |
| `gear_support/gear_42` | `gear_42/gear_support` |

`exercise.py` contains the initialized scene and the three results to compute.
`script.py` is the complete reference example from the `hpp-training` branch.

## Start the exercise

From a terminal containing HPP and the sourced MFJA description package, run:

```bash
cd mfja
python3 -i exercise.py
```

The script builds `robot`, `problem`, `graph` and the neutral configuration `q`.
Complete the remaining sections to produce `q1`, `q2` and `p`.

## Build the initial configuration

The gear starts on `gear_plate/placement_1`. Create a grasp constraint between
that fixture and the `gear_42/placement` handle, then project the neutral
configuration:

```python
g = robot.grippers()["gear_plate/placement_1"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_1 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q1, status = cp.solver().solve(q)
```

## Build the goal configuration

Repeat the projection with gripper `gear_support/gear_42` and handle
`gear_42/gear_support`. Store the result in `q2`.

The fixed gear plate and gear support are modeled as grippers because a grasp
constraint expresses the exact pose of the gear in each fixture.

## Solve the manipulation problem

Set the initial configuration, goal configuration and constraint graph on the
problem, then let `ManipulationPlanner` find the sequence of graph transitions:

```python
problem.initConfig(q1)
problem.addGoalConfig(q2)
problem.constraintGraph(graph)
planner = ManipulationPlanner(problem)
planner.maxIterations(1000)
p = planner.solve()
```

Run the complete example with:

```bash
python3 -i script.py
```

## Display the result

In the interactive Python terminal, create a Viser viewer and load the path:

```python
from pyhpp_viser import Viewer

v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v(q1)
v(q2)
v.loadPath(p)
```

Open <http://localhost:8000> in a browser.

## API documentation

For the two-gear hardware exercise, follow the
[segment-by-segment validation guide](STAUBLI_EXECUTION.md).

- [HPP documentation](https://gepetto.github.io/doc/hpp-doc/doxygen-html/index.html)
