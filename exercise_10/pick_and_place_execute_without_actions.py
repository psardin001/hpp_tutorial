"""Plan from the measured arm position and send the arm trajectory."""

# ruff: noqa: F405
from pathlib import Path

import numpy as np
import yaml
from hpp_exec import send_trajectory
from pick_and_place import *  # noqa: F403
from staubli_io import (
    JOINT_NAMES,
    check_position,
    read_joints,
    robot_connection,
    wait_reached,
)

execution_config = yaml.safe_load(
    Path(__file__).with_name("two_gears_execution.yaml").read_text()
)["execution"]


def wait_for_motion(node, result, configs):
    return wait_reached(node, execution_config, result, np.asarray(configs)[:, :6])


def execute():
    with robot_connection() as node:
        q_start = read_joints(node, execution_config)
        _, _, p_timed, path_times = solve(q_start)
        times = sorted(
            set(np.linspace(0.0, p_timed.length(), 101).tolist() + path_times)
        )
        configs = []
        for t in times:
            q, success = p_timed(t)
            if not success:
                raise RuntimeError(f"Could not sample the path at {t}")
            configs.append(np.array(q))

        check_position(node, execution_config, configs[0][:6])
        success = send_trajectory(
            configs,
            times,
            joint_names=JOINT_NAMES,
            joint_indices=list(range(6)),
            controller_topic=execution_config["trajectory_action"],
            positions_only=True,
            wait_for_completion=lambda sender, result: wait_for_motion(
                sender, result, configs
            ),
        )
        if not success:
            raise RuntimeError("Arm trajectory failed")


if __name__ == "__main__":
    execute()
