"""Save, inspect, preview and execute the two-gear path one segment at a time."""

import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import yaml

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


def build_scene(plan, directory):
    from staubli_scene import placement
    from two_gears import build_problem

    cell = Path(directory) / "room315.urdf"
    cell.write_text(plan["room_urdf"])
    srdf = cell.with_suffix(".srdf")
    srdf.write_text('<robot name="room315"/>')
    config = plan["config"]
    return build_problem(
        plan["q_start"],
        config["gear_plate_pose"],
        config["gear_support_pose"],
        (str(cell), str(srdf), placement(plan["robot_world_pose"]).inverse()),
    )


def make_plan(config, q_start, mfja_root):
    from hpp_exec import segments_from_graph
    from staubli_scene import read_room
    from two_gears import solve

    room, base = read_room(mfja_root)
    plan = dict(
        config=config, q_start=list(q_start), room_urdf=room, robot_world_pose=base
    )
    with TemporaryDirectory(prefix="two-gears-") as directory:
        robot, graph, problem = build_scene(plan, directory)
        path = solve(problem)
        configs, times, segments = segments_from_graph(path, graph)
        indices = [robot.rankInConfiguration["staubli/" + name] for name in JOINT_NAMES]
        plan.update(
            configurations=[q.tolist() for q in configs],
            times=times,
            joint_indices=indices,
            segments=[],
        )
        for segment in segments:
            state = str(
                graph.getContainingNode(graph.getTransition(segment.transition_name))
            )
            item = asdict(segment)
            item.pop("pre_actions")
            item.pop("post_actions")
            item["containing_state"] = state
            item["gripper"] = (
                "close" if "staubli/tool0_gripper grasps gear_42_" in state else "open"
            )
            plan["segments"].append(item)
    validate_plan(plan)
    if not np.allclose(
        np.asarray(plan["configurations"])[0, indices], q_start, atol=1e-6
    ):
        raise RuntimeError("Planning changed the requested arm start configuration")
    return plan


def validate_plan(plan):
    configs = np.asarray(plan["configurations"])
    times = np.asarray(plan["times"])
    if configs.ndim != 2 or len(configs) != len(times) or len(times) < 2:
        raise ValueError("Invalid plan samples")
    if (
        not np.isfinite(configs).all()
        or not np.isfinite(times).all()
        or not (np.diff(times) > 0).all()
    ):
        raise ValueError("Plan samples must be finite and times strictly increasing")
    joints = plan["joint_indices"]
    if (
        len(joints) != 6
        or len(set(joints)) != 6
        or any(not isinstance(i, int) or not 0 <= i < configs.shape[1] for i in joints)
    ):
        raise ValueError("Expected six distinct arm configuration indices")
    previous_end = 1
    for item in plan["segments"]:
        if item["start_index"] != previous_end - 1 or not item[
            "start_index"
        ] + 1 < item["end_index"] <= len(times):
            raise ValueError("Segments must cover the path with shared endpoints")
        if item["gripper"] not in ("open", "close"):
            raise ValueError("Invalid gripper state")
        previous_end = item["end_index"]
    if previous_end != len(times):
        raise ValueError("Segments do not cover the path")


def inspect_plan(plan):
    print(
        f"{len(plan['segments'])} segments; TOPPRA duration {plan['times'][-1]:.2f} s"
    )
    for i, item in enumerate(plan["segments"]):
        print(f"{i:2d}  {item['gripper']:5s}  {item['transition_name']}")


def preview(plan, index):
    from pyhpp_viser import Viewer

    item = plan["segments"][index]
    with TemporaryDirectory(prefix="two-gears-") as directory:
        robot, graph, problem = build_scene(plan, directory)
        viewer = Viewer(robot)
        viewer.initViewer(open=False, loadModel=True)
        viewer.setProblem(problem)
        viewer.setGraph(graph)
        configs = plan["configurations"][item["start_index"] : item["end_index"]]
        times = plan["times"][item["start_index"] : item["end_index"]]
        viewer(np.asarray(configs[0]))
        input("Open the Viser URL, then press Enter to play this saved segment. ")
        for i, configuration in enumerate(configs):
            if i:
                time.sleep(times[i] - times[i - 1])
            viewer(np.asarray(configuration))
        input("Press Enter to close the preview. ")


def set_gripper(node, config, mode):
    import rclpy
    from staubli_msgs.msg import IOModule, ServiceReturnCode
    from staubli_msgs.srv import WriteSingleIO

    client = node.create_client(WriteSingleIO, config["gripper_service"])
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Gripper service unavailable")
        request = WriteSingleIO.Request()
        request.module.id = IOModule.VALVE_OUT
        request.pin = config["gripper_pin"]
        request.state = mode == "open"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or response.code.val != ServiceReturnCode.SUCCESS:
            raise RuntimeError("Gripper command failed")
        time.sleep(config["gripper_settle_s"])
    finally:
        node.destroy_client(client)
    return True


def check_position(node, config, target):
    from hpp_exec import read_current_configuration

    current = read_current_configuration(
        node,
        JOINT_NAMES,
        topic=config["joint_state_topic"],
        timeout_sec=10.0,
        require_single_publisher=True,
    )
    if current is None or not np.isfinite(current).all():
        raise RuntimeError("No valid joint state received")
    error = float(np.max(np.abs(current - target)))
    if error > config["joint_tolerance_rad"]:
        raise RuntimeError(
            f"Arm differs from the expected configuration by {error:.4f} rad"
        )


def execute(plan, index):
    if plan["config"]["calibrated"] is not True:
        raise RuntimeError(
            "Measure the fixture poses, mark the configuration calibrated, then replan"
        )
    import rclpy
    from hpp_exec import Segment, execute_segments
    from rclpy.node import Node

    item = plan["segments"][index]
    config = plan["config"]["execution"]
    configurations = [np.asarray(q) for q in plan["configurations"]]
    joints = plan["joint_indices"]
    print(f"Expected physical state: {item['containing_state']}")
    print(f"Gripper: {item['gripper']}; segment: {item['transition_name']}")
    rclpy.init()
    node = Node("two_gears_execution")
    try:
        check_position(node, config, configurations[item["start_index"]][joints])
        segment = Segment(
            item["start_index"],
            item["end_index"],
            transition_name=item["transition_name"],
        )
        segment.pre_actions.append(lambda: set_gripper(node, config, item["gripper"]))
        if not execute_segments(
            [segment],
            configurations,
            plan["times"],
            JOINT_NAMES,
            joint_indices=joints,
            controller_topic=config["trajectory_action"],
        ):
            raise RuntimeError(
                "Execution failed; inspect the physical state before proceeding"
            )
        check_position(node, config, configurations[item["end_index"] - 1][joints])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan")
    plan.add_argument("file", type=Path)
    plan.add_argument("--q-start", type=float, nargs=6, required=True)
    plan.add_argument("--mfja-root", type=Path, required=True)
    plan.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("two_gears_execution.yaml"),
    )
    for name in ("inspect", "view", "execute"):
        command = commands.add_parser(name)
        command.add_argument("file", type=Path)
        if name != "inspect":
            command.add_argument("--segment", type=int, required=True)
    args = parser.parse_args()
    if args.command == "plan":
        config = yaml.safe_load(args.config.read_text())
        plan = make_plan(config, args.q_start, args.mfja_root)
        args.file.write_text(json.dumps(plan, indent=2, allow_nan=False) + "\n")
    else:
        plan = json.loads(args.file.read_text())
        validate_plan(plan)
    inspect_plan(plan)
    if args.command in ("view", "execute"):
        if not 0 <= args.segment < len(plan["segments"]):
            raise ValueError("Segment index out of range")
        if args.command == "view":
            preview(plan, args.segment)
        else:
            execute(plan, args.segment)


if __name__ == "__main__":
    main()
