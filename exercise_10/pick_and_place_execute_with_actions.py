"""Plan from the measured arm position and execute with gripper actions."""

# ruff: noqa: F405
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


GRASP_TRANSITION = "staubli/tool0_gripper > gear_42/stud | f_23"
RELEASE_TRANSITION = "staubli/tool0_gripper < gear_42/stud | 0-0_21"


def execute():
    with robot_connection() as node:
        q_start = read_joints(node, execution_config)
        _, _, p_timed, path_times = solve(q_start)
        sample_times = sorted(
            set(np.linspace(0.0, p_timed.length(), 101).tolist() + path_times)
        )
        configs, times, segments = segments_from_graph(
            p_timed, graph, sample_params=sample_times
        )
        by_transition = segments_by_transition(segments)

        def open_gripper():
            return set_gripper(node, execution_config, "open")

        def close_gripper():
            return set_gripper(node, execution_config, "close")

        segments[0].pre_actions.append(open_gripper)
        by_transition[GRASP_TRANSITION][0].pre_actions.append(close_gripper)
        for segment in by_transition[RELEASE_TRANSITION]:
            segment.pre_actions.append(open_gripper)
        print_segments(segments)

        check_position(node, execution_config, configs[0][:6])
        success = execute_segments(
            segments,
            configs,
            times,
            joint_names=JOINT_NAMES,
            joint_indices=list(range(6)),
            controller_topic=execution_config["trajectory_action"],
            positions_only=True,
            wait_for_completion=wait_for_motion,
        )
        if not success:
            raise RuntimeError("Pick-and-place execution failed")


if __name__ == "__main__":
    execute()
