# MFJA gears

Install HPP, the MFJA models and the Stäubli driver using the
[gears installation guide](https://github.com/psardin001/mfja_3rd_floor_gz/blob/main/INSTALL.md).

```bash
source "$HOME/mfja-gears/setup.bash"
cd "$HPP_TUTORIAL_DIR/mfja"
python -i one_gear.py
# Or:
python -i two_gears.py
```

In Python:

```python
v = Viewer(robot)
v.initViewer(open=True, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v.loadPath(p2)
```

The installation guide also contains the controller setup and execution commands.

After calibrating the scene and starting the driver, place the gears at their
initial locations and leave the gripper empty. To plan from the measured arm
position and execute the complete cycle:

```bash
python one_gear_execute.py --execute --all
# Or:
python two_gears_execute.py --execute --all
```

Omit `--all` to choose the next segment, the remaining cycle, or quit. Both
commands send TOPPRA joint velocities and handle the gripper between confirmed
stops. The Stäubli driver must support explicit ratios below its fallback speed;
actual movement durations still depend on the controller.
