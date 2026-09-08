"""Send one saved two-gear segment through hpp-exec."""

import argparse
import json
from pathlib import Path

import numpy as np
from staubli_io import JOINT_NAMES, check_position, set_gripper


def validate_plan(plan):
    configs = np.asarray(plan["configurations"])
    times = np.asarray(plan["times"])
    if times.ndim != 1 or configs.shape != (len(times), 6) or len(times) < 2:
        raise ValueError("Expected six joint positions per sample")
    if (
        not np.isfinite(configs).all()
        or not np.isfinite(times).all()
        or not (np.diff(times) > 0).all()
    ):
        raise ValueError("Plan samples must be finite and times strictly increasing")
    if len(plan["gripper"]) != len(plan["segments"]) or any(
        mode not in ("open", "close") for mode in plan["gripper"]
    ):
        raise ValueError("Expected one gripper state per segment")
    previous_end = 1
    for item in plan["segments"]:
        if item["start_index"] != previous_end - 1 or not item[
            "start_index"
        ] + 1 < item["end_index"] <= len(times):
            raise ValueError("Segments must cover the path with shared endpoints")
        previous_end = item["end_index"]
    if previous_end != len(times):
        raise ValueError("Segments do not cover the path")


def execute(plan, index):
    validate_plan(plan)
    if not 0 <= index < len(plan["segments"]):
        raise ValueError("Segment index out of range")
    if plan["config"]["calibrated"] is not True:
        raise RuntimeError(
            "Measure the fixture poses, mark the configuration calibrated, then replan"
        )

    import rclpy
    from hpp_exec import Segment, execute_segments
    from rclpy.node import Node

    configs = np.asarray(plan["configurations"])
    segments = [Segment(**item) for item in plan["segments"]]
    segment = segments[index]
    config = plan["config"]["execution"]
    mode = plan["gripper"][index]
    print(f"Segment {index}: {segment.transition_name}; gripper: {mode}")
    rclpy.init()
    node = Node("two_gears_execution")
    try:
        check_position(node, config, configs[segment.start_index])
        segment.pre_actions.append(lambda: set_gripper(node, config, mode))
        if not execute_segments(
            [segment],
            configs,
            plan["times"],
            JOINT_NAMES,
            controller_topic=config["trajectory_action"],
        ):
            raise RuntimeError(
                "Execution failed; inspect the physical state before proceeding"
            )
        check_position(node, config, configs[segment.end_index - 1])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    parser.add_argument("--segment", type=int, required=True)
    args = parser.parse_args()
    execute(json.loads(args.file.read_text()), args.segment)


if __name__ == "__main__":
    main()
