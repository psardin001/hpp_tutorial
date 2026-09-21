"""Plan from the measured arm position and execute with gripper actions."""

from pathlib import Path

import yaml
from hpp_exec import (
    execute_segments,
    print_segments,
    segments_by_transition,
    segments_from_graph,
)
from pick_and_place import graph, solve
from staubli_io import JOINT_NAMES, read_joints, robot_connection, set_gripper

execution_config = yaml.safe_load(
    Path(__file__).with_name("two_gears_execution.yaml").read_text()
)["execution"]


def execute():
    with robot_connection() as node:
        q_start = read_joints(node, execution_config)
        _, _, p_timed, _ = solve(q_start)
        configs, times, segments = segments_from_graph(p_timed, graph)
        segments_by_name = segments_by_transition(segments)

        def open_gripper():
            return set_gripper(node, execution_config, "open")

        def close_gripper():
            return set_gripper(node, execution_config, "close")

        GRASP_TRANSITION = "staubli/tool0_gripper > gear_42/stud | f_23"
        RELEASE_TRANSITION = "staubli/tool0_gripper < gear_42/stud | 0-0_21"

        segments[0].pre_actions.append(open_gripper)
        for segment in segments_by_name[GRASP_TRANSITION]:
            segment.pre_actions.append(close_gripper)

        for segment in segments_by_name[RELEASE_TRANSITION]:
            segment.pre_actions.append(open_gripper)
        print_segments(segments)

        execute_segments(
            segments,
            configs,
            times,
            joint_names=JOINT_NAMES,
            joint_indices=list(range(6)),
            controller_topic=execution_config["trajectory_action"],
            positions_only=True,
        )


if __name__ == "__main__":
    execute()
