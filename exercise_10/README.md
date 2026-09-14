# Exercise 10: MFJA pick and place

## Objective

The objective of this exercise is to use the notions explained in the various tutorials in order
to plan and execute manipulation motions of increasing complexity.

## Instructions

  1. In script pick_and_place.py, fill in with the appropriate python code the parts
     designated by comment `#TODO:`

  2. in a script called `pick_and_place_execute_without_actions.py`, write the code necessary
     to execute the planned motion without controlling the gripper. For that, take inspiration from
     tutorial_7.

  3. In a script called `pick_and_place_execute_with_actions.py`, write the code necessary to
     execute the planned motion with appropriate control of the gripper. You will need the
     following methods:

```python
    from staubli_io import set_gripper

    def open_gripper():
        return set_gripper(node, execution_config, "open")

    def close_gripper():
        return set_gripper(node, execution_config, "close")

```
     
