"""Plan from the measured arm position and send the arm trajectory."""

from pathlib import Path

import yaml
from hpp_exec import segments_from_graph, send_trajectory
from pick_and_place import graph, solve
from staubli_io import JOINT_NAMES, read_joints, robot_connection

execution_config = yaml.safe_load(
    Path(__file__).with_name("two_gears_execution.yaml").read_text()
)["execution"]


def execute():
    with robot_connection() as node:
        q_start = read_joints(node, execution_config)
        _, _, p_timed, _ = solve(q_start)
        configs, times, _ = segments_from_graph(p_timed, graph)

        send_trajectory(
            configs,
            times,
            joint_names=JOINT_NAMES,
            joint_indices=list(range(6)),
            controller_topic=execution_config["trajectory_action"],
            positions_only=True,
        )


if __name__ == "__main__":
    execute()
