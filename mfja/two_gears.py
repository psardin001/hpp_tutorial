"""Plan, validate and export the two-gear manipulation segments."""

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import yaml
from hpp_exec import print_segments, segments_from_graph
from pinocchio import SE3, XYZQUATToSE3, neutral
from pyhpp.constraints import ComparisonType, ComparisonTypes, Implicit, Transformation
from pyhpp.core import ConfigProjector, Progressive, ProgressiveProjector
from pyhpp.core.path import Vector
from pyhpp.manipulation import (
    Device,
    Graph,
    GraphPathValidation,
    GraphRandomShortcut,
    Problem,
    StatesPathFinder,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from staubli_io import JOINT_NAMES
from staubli_scene import placement, read_room
from tools import Toppra


def build_problem(q_start, plate_pose, support_pose, room=None):
    robot = Device("mfja")

    # Load Staubli robot
    urdf_filename = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
    srdf_filename = "package://mfja_3rd_floor_description/srdf/staubli_tx2_60l.srdf"

    urdf.loadModel(
        robot, 0, "staubli", "anchor", urdf_filename, srdf_filename, SE3.Identity()
    )

    # Load gear plate
    urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_plate.urdf"
    srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_plate.srdf"
    pose = XYZQUATToSE3(np.asarray(plate_pose))

    urdf.loadModel(robot, 0, "gear_plate", "anchor", urdf_filename, srdf_filename, pose)

    # Load gear support
    urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_support.urdf"
    srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_support.srdf"
    pose = XYZQUATToSE3(np.asarray(support_pose))

    urdf.loadModel(
        robot, 0, "gear_support", "anchor", urdf_filename, srdf_filename, pose
    )

    # Load 2 instances of 42 mm gear
    urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_42.urdf"
    srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_42.srdf"

    urdf.loadModel(
        robot, 0, "gear_42_1", "freeflyer", urdf_filename, srdf_filename, SE3.Identity()
    )
    robot.setJointBounds(
        "gear_42_1/root_joint",
        [
            -1.0,
            1.0,
            -1.0,
            1.0,
            -0.2,
            1.5,
        ],
    )
    urdf.loadModel(
        robot, 0, "gear_42_2", "freeflyer", urdf_filename, srdf_filename, SE3.Identity()
    )
    robot.setJointBounds(
        "gear_42_2/root_joint",
        [
            -1.0,
            1.0,
            -1.0,
            1.0,
            -0.2,
            1.5,
        ],
    )

    if room is not None:
        urdf.loadModel(robot, 0, "room315", "anchor", room[0], room[1], room[2])

    problem = Problem(robot)
    problem.pathValidation(GraphPathValidation(Progressive(robot, 0.001)))
    problem.pathValidationFactory(GraphPathValidation(Progressive(robot, 0.001)))
    problem.pathProjector(
        ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.2)
    )

    graph = Graph("robot", robot, problem)
    factory = ConstraintGraphFactory(graph)
    graph.maxIterations(40)
    graph.errorThreshold(1e-5)

    factory.setGrippers(
        ["staubli/tool0_gripper", "gear_support/gear_42_1", "gear_support/gear_42_2"]
    )
    objects = ["gear_42_1", "gear_42_2"]
    handlesPerObject = [
        ["gear_42_1/stud", "gear_42_1/gear_support"],
        ["gear_42_2/stud", "gear_42_2/gear_support"],
    ]
    contactsPerObject = [["gear_42_1/bottom"], ["gear_42_2/bottom"]]
    factory.setObjects(objects, handlesPerObject, contactsPerObject)
    factory.environmentContacts(["gear_plate/top"])
    factory.setPossibleGrasps(
        {
            "staubli/tool0_gripper": ["gear_42_1/stud", "gear_42_2/stud"],
            "gear_support/gear_42_1": ["gear_42_1/gear_support"],
            "gear_support/gear_42_2": ["gear_42_2/gear_support"],
        }
    )
    factory.generate()
    # Force linear motion when placing gear_42_1 on support
    h = robot.handles()["gear_42_1/gear_support"]
    f = Transformation(
        "vertical gear_42_1",
        robot,
        h.getParentJointId(),
        h.localPosition,
        SE3.Identity(),
        [True, True, False, True, True, True],
    )
    cts = ComparisonTypes()
    cts[:] = [
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
    ]
    vertical_gear_42_1 = Implicit(f, cts, [True, True, True, True, True])
    transition = graph.getTransition(
        "gear_support/gear_42_1 > gear_42_1/gear_support | 0-0_12"
    )
    graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_1])
    transition = graph.getTransition(
        "gear_support/gear_42_1 < gear_42_1/gear_support | 0-0:1-1_21"
    )
    graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_1])

    # Force linear motion when placing gear_42_2 on support
    h = robot.handles()["gear_42_2/gear_support"]
    f = Transformation(
        "vertical gear_42_2",
        robot,
        h.getParentJointId(),
        h.localPosition,
        SE3.Identity(),
        [True, True, False, True, True, True],
    )
    cts = ComparisonTypes()
    cts[:] = [
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
    ]
    vertical_gear_42_2 = Implicit(f, cts, [True, True, True, True, True])
    transition = graph.getTransition(
        "gear_support/gear_42_2 > gear_42_2/gear_support | 0-2:1-1_12"
    )
    graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_2])
    transition = graph.getTransition(
        "gear_support/gear_42_2 < gear_42_2/gear_support | 0-2:1-1:2-3_21"
    )
    graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_2])

    # Keep release projections near their placement configurations.
    for name in (
        "staubli/tool0_gripper < gear_42_1/stud | 0-0:1-1_21",
        "staubli/tool0_gripper < gear_42_2/stud | 0-2:1-1:2-3_21",
    ):
        graph.setShort(graph.getTransition(name), True)

    # Deactive collision checking between gripper and gears for all transitions
    for transition in graph.getTransitions():
        graph.setSecurityMarginForTransition(
            transition, "staubli/joint_6", "gear_42_1/root_joint", float("-inf")
        )
        graph.setSecurityMarginForTransition(
            transition, "staubli/joint_6", "gear_42_2/root_joint", float("-inf")
        )

    graph.initialize()

    q = neutral(robot.model())
    for i, value in enumerate(q_start, 1):
        q[robot.rankInConfiguration[f"staubli/joint_{i}"]] = value

    # Build initial configuration where
    #     - gear_42_1 is placed on gripper gear_placement/placement_1
    #     - gear_42_2 is placed on gripper gear_placement/placement_2
    cp = ConfigProjector(robot, "solver", 1e-5, 40)
    g = robot.grippers()["gear_plate/placement_1"]
    h = robot.handles()["gear_42_1/placement"]
    grasp = h.createGrasp(g, "gear_plate/placement_1 grasps gear_42_1/placement")
    cp.add(grasp, 0)
    g = robot.grippers()["gear_plate/placement_2"]
    h = robot.handles()["gear_42_2/placement"]
    grasp = h.createGrasp(g, "gear_plate/placement_2 grasps gear_42_2/placement")
    cp.add(grasp, 0)
    q1, status = cp.solver().solve(q)
    if not status:
        raise RuntimeError("Initial gear placement projection failed")

    # Build goal configuration where
    #     - gear_42_1 is placed on gripper gear_support/gear_42_1
    #     - gear_42_2 is placed on gripper gear_support/gear_42_2
    cp = ConfigProjector(robot, "solver", 1e-5, 40)
    g = robot.grippers()["gear_support/gear_42_1"]
    h = robot.handles()["gear_42_1/gear_support"]
    grasp = h.createGrasp(g, "gear_support/gear_42_1 grasps gear_42_1/gear_support")
    cp.add(grasp, 0)
    g = robot.grippers()["gear_support/gear_42_2"]
    h = robot.handles()["gear_42_2/gear_support"]
    grasp = h.createGrasp(g, "gear_support/gear_42_2 grasps gear_42_2/gear_support")
    cp.add(grasp, 0)
    q2, status = cp.solver().solve(q)
    if not status:
        raise RuntimeError("Goal gear placement projection failed")

    # Solving a manipulation problem between q1 and q2
    problem.initConfig(q1)
    problem.addGoalConfig(q2)
    problem.constraintGraph(graph)
    return robot, graph, problem


def solve(problem):
    problem.setParameter("StatesPathFinder/maxDepth", 6)
    problem.setParameter("StatesPathFinder/nTriesUntilBacktrack", 5)
    manipulationPlanner = StatesPathFinder(problem)
    manipulationPlanner.maxIterations(1000)
    p = manipulationPlanner.solve()

    # Optimize the path
    opt1 = GraphRandomShortcut(problem)
    p1 = opt1.optimize(p)

    toppra = Toppra(problem)
    toppra.velocityScale = 0.5
    toppra.N = 100
    toppra.selectJoints([f"staubli/joint_{i}" for i in range(1, 7)])
    # Reserve 10% of the 0.5 rad/s² limit for numerical projection effects.
    toppra.accelerationLimits = np.array(6 * [0.45])
    p2 = toppra.optimize(p1)
    validate_path(p2, problem.constraintGraph())
    return p2


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


def export_plan(file, robot, graph, path, config, q_start):
    """Save hpp-exec segments and the six arm joints for the ROS process."""
    configs, times, segments = segments_from_graph(path, graph)
    indices = [robot.rankInConfiguration["staubli/" + name] for name in JOINT_NAMES]
    joints = np.asarray(configs)[:, indices]
    if not np.allclose(joints[0], q_start, atol=1e-6, rtol=0):
        raise RuntimeError("Planning changed the requested arm start configuration")
    gripper = []
    for segment in segments:
        state = str(
            graph.getContainingNode(graph.getTransition(segment.transition_name))
        )
        gripper.append(
            "close" if "staubli/tool0_gripper grasps gear_42_" in state else "open"
        )
    plan = dict(
        config=config,
        configurations=joints.tolist(),
        times=times,
        segments=[asdict(segment) for segment in segments],
        gripper=gripper,
    )
    file.write_text(json.dumps(plan, indent=2, allow_nan=False) + "\n")
    print_segments(segments)
    return segments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    parser.add_argument("--q-start", type=float, nargs=6, required=True)
    parser.add_argument("--mfja-root", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("two_gears_execution.yaml"),
    )
    parser.add_argument("--view", action="store_true")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    room, base = read_room(args.mfja_root)
    with TemporaryDirectory(prefix="two-gears-") as directory:
        cell = Path(directory) / "room315.urdf"
        cell.write_text(room)
        srdf = cell.with_suffix(".srdf")
        srdf.write_text('<robot name="room315"/>')
        robot, graph, problem = build_problem(
            args.q_start,
            config["gear_plate_pose"],
            config["gear_support_pose"],
            (str(cell), str(srdf), placement(base).inverse()),
        )
        path = solve(problem)
        export_plan(args.file, robot, graph, path, config, args.q_start)
        if args.view:
            from pyhpp_viser import Viewer

            viewer = Viewer(robot)
            viewer.initViewer(open=False, loadModel=True)
            viewer.setProblem(problem)
            viewer.setGraph(graph)
            viewer.loadPath(path)
            input("Preview the planned path in Viser, then press Enter to close. ")


if __name__ == "__main__":
    main()
