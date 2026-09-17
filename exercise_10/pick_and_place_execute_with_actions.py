"""Plan from the measured arm position and execute with gripper actions."""

# ruff: noqa: F401
from pathlib import Path

import numpy as np
import yaml
from hpp_exec import (
    execute_segments,
    print_segments,
    segments_by_transition,
    segments_from_graph,
)
from pick_and_place import *  # noqa: F403
from staubli_io import (
    JOINT_NAMES,
    check_position,
    read_joints,
    robot_connection,
    set_gripper,
    wait_reached,
)

execution_config = yaml.safe_load(
    Path(__file__).with_name("two_gears_execution.yaml").read_text()
)["execution"]


def wait_for_motion(node, result, configs):
    return wait_reached(node, execution_config, result, np.asarray(configs)[:, :6])
