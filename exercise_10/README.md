# Exercise 10: MFJA pick and place

- `pick_and_place.py`: build and solve the HPP planning problem.
- `pick_and_place_execute_without_actions.py`: plan from measured robot joints
  and send the arm trajectory.
- `pick_and_place_execute_with_actions.py`: plan from measured robot joints
  and execute with gripper actions.
- `one_gear.py`, `two_gears.py`: gear-transfer planning examples.
- `one_gear_execute.py`, `two_gears_execute.py`: execute the gear-transfer plans.
- `environment.py`: load the MFJA scene and initial arm configuration.
- `staubli_io.py`: read robot feedback and control the gripper.
- `tools.py`: smooth and time-parameterize paths.
- `two_gears_execution.yaml`: execution topics, tolerances and gripper settings.
- `test_manipulation_timing.py`: manipulation timing tests.
- `Makefile`: dependency build targets.
