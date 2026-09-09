"""Export a planned path and send one saved segment through hpp-exec."""

import argparse
import json
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import yaml
from hpp_exec import print_segments, segments_from_graph
from pyhpp.core.path import Vector
from staubli_io import JOINT_NAMES, check_position, set_gripper


def validate_path(path, graph):
    """Validate each timed subpath with its manipulation transition."""
    flat = Vector(path.outputSize(), path.outputDerivativeSize())
    path.flatten(flat)
    start = 0.0
    for i in range(flat.numberPaths()):
        leaf = flat.pathAtRank(i)
        edge = graph.transitionAtParam(path, start + leaf.length() / 2)
        valid, _, report = edge.pathValidation().validate(leaf, False)
        if not valid:
            raise RuntimeError(f"Invalid timed subpath {i} ({edge.name()}): {report}")
        start += leaf.length()


def export_plan(file, robot, graph, path, config=None, q_start=None):
    """Save arm trajectories between stops, combining graph transitions."""
    validate_path(path, graph)
    if config is None:
        config = yaml.safe_load(
            Path(__file__).with_name("two_gears_execution.yaml").read_text()
        )
    configs, times, segments = segments_from_graph(path, graph)
    indices = [robot.rankInConfiguration["staubli/" + name] for name in JOINT_NAMES]
    joints = np.asarray(configs)[:, indices]
    if q_start is not None and not np.allclose(joints[0], q_start, atol=1e-6, rtol=0):
        raise RuntimeError("Planning changed the requested arm start configuration")
    flat = Vector(path.outputSize(), path.outputDerivativeSize())
    path.flatten(flat)
    boundaries = np.cumsum(
        [flat.pathAtRank(i).length() for i in range(flat.numberPaths())]
    )
    for t in boundaries[:-1]:
        if np.linalg.norm(path.derivative(float(t), 1), np.inf) > 1e-6:
            continue
        for i, segment in enumerate(segments):
            if segment.start_time + 1e-9 < t < segment.end_time - 1e-9:
                index = int(np.argmin(np.abs(np.asarray(times) - t)))
                state = str(graph.getStateFromConfiguration(configs[index]))
                segments[i : i + 1] = [
                    replace(
                        segment,
                        end_index=index + 1,
                        end_time=float(t),
                        actual_state_after=state,
                    ),
                    replace(
                        segment,
                        start_index=index,
                        start_time=float(t),
                        actual_state_before=state,
                    ),
                ]
                break
    gripper = []
    groups = []
    previous = None
    for segment in segments:
        state = str(
            graph.getContainingNode(graph.getTransition(segment.transition_name))
        )
        # Preserve geometric stops within a manipulation state.
        if (
            state == previous
            and np.linalg.norm(path.derivative(segment.start_time, 1), np.inf) > 1e-6
        ):
            last = groups[-1]
            last.end_index = segment.end_index
            last.end_time = segment.end_time
            last.state_after = segment.state_after
            last.actual_state_after = segment.actual_state_after
            last.transition_name += " -> " + segment.transition_name
        else:
            groups.append(segment)
            gripper.append(
                "close" if "staubli/tool0_gripper grasps gear_42_" in state else "open"
            )
        previous = state
    plan = dict(
        config=config,
        configurations=joints.tolist(),
        times=times,
        segments=[asdict(segment) for segment in groups],
        gripper=gripper,
    )
    Path(file).write_text(json.dumps(plan, indent=2, allow_nan=False) + "\n")
    print_segments(groups)
    return groups


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
