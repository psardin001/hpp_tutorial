"""Export a gear plan, execute it all, or advance through its stopped segments."""

import argparse
import json
from dataclasses import asdict, replace
from functools import partial
from pathlib import Path

import numpy as np
import yaml
from hpp_exec import Segment, execute_segments, segments_from_graph
from pyhpp.core.path import Vector
from staubli_io import (
    JOINT_NAMES,
    check_position,
    check_speed,
    check_status,
    robot_connection,
    set_gripper,
    wait_reached,
)


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
                "close" if "staubli/tool0_gripper grasps gear_42" in state else "open"
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


def execute_next(plan, index=0, count=1):
    """Execute the next count segments and return the next index (zero-based)."""
    validate_plan(plan)
    if not 0 <= index <= len(plan["segments"]) or count < 0:
        raise ValueError("Segment index out of range")
    end = min(index + count, len(plan["segments"]))
    if index == end:
        return index
    if plan["config"]["calibrated"] is not True:
        raise RuntimeError("Valider la calibration avant l'exécution")

    configs = np.asarray(plan["configurations"])
    config = plan["config"]["execution"]
    with robot_connection() as node:

        def prepare(segment, number):
            check_speed(node)
            check_status(node)
            points = configs[segment.start_index : segment.end_index]
            current = check_position(node, config, points[0])
            if np.max(np.abs(points - current)) < 1e-5:
                # A single-point segment runs its actions with no arm trajectory.
                segment.end_index = segment.start_index + 1
            print(f"Segment {number + 1}/{len(plan['segments'])}", flush=True)
            return True

        segments = [Segment(**item) for item in plan["segments"][index:end]]
        previous_mode = None
        for i, segment in enumerate(segments, index):
            segment.pre_actions = [partial(prepare, segment, i)]
            mode = plan["gripper"][i]
            if mode != previous_mode:
                segment.pre_actions.append(partial(set_gripper, node, config, mode))
                previous_mode = mode
            segment.post_actions = [
                lambda target=configs[segment.end_index - 1]: (
                    check_position(node, config, target) is not None
                )
            ]
        if not execute_segments(
            segments,
            configs,
            plan["times"],
            JOINT_NAMES,
            controller_topic=config["trajectory_action"],
            positions_only=True,
            wait_for_completion=lambda sender, result, points: wait_reached(
                sender, config, result, points
            ),
        ):
            raise RuntimeError("Échec de l'exécution")
    return end


def execute_all(plan, start=0):
    """Execute the plan from start through its final segment."""
    return execute_next(plan, start, len(plan["segments"]) - start)


def prompt_execution(plan):
    index = 0
    while index < len(plan["segments"]):
        answer = input(
            f"Segment {index + 1} : Entrée = suivant, a = tout, q = quitter : "
        ).strip()
        if answer == "a":
            execute_all(plan, index)
            return
        if answer:
            return
        index = execute_next(plan, index)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--segment", type=int, help="indice du segment (à partir de 0)")
    mode.add_argument("--all", action="store_true", help="exécuter tout le plan")
    args = parser.parse_args()
    plan = json.loads(args.file.read_text())
    if args.all:
        execute_all(plan)
    elif args.segment is not None:
        execute_next(plan, args.segment)
    else:
        prompt_execution(plan)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError) as error:
        raise SystemExit(str(error)) from error
    except (KeyboardInterrupt, EOFError):
        raise SystemExit(
            "Exécution interrompue. Vérifier l'arrêt au pendant."
        ) from None
