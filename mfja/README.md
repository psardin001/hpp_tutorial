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
