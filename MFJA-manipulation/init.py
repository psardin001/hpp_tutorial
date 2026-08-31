"""Room 315 Stäubli scene for the pick-and-place exercise."""

import sys

import coal
import numpy as np
import pinocchio as pin
from pyhpp.manipulation import Device, Graph, Problem, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins

# Make Viser use the Coal geometry classes loaded by ROS Jazzy.
sys.modules["hppfcl"] = coal

JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
Q_ARM_INITIAL = np.deg2rad([0.0, 50.0, 70.0, 0.0, 55.0, 0.0])
Q_ARM_START = Q_ARM_INITIAL + np.array([0.15, -0.10, 0.10, 0.0, -0.10, 0.0])

ROOM315_ROBOT_POSE = (-15.03, -6.0, 1.0, 0.0, 0.0, 0.0)
TABLE_DROP_ZONE_WORLD_POSE = (-14.65, -5.84, 1.003, 0.0, 0.0, 0.0)
BOX_START_WORLD_POSE = (-14.75, -5.84, 1.033, 0.0, 0.0, 0.0)
BOX_GOAL_WORLD_POSE = (-14.55, -5.84, 1.033, 0.0, 0.0, 0.0)

GRIPPER = "staubli/tool0_gripper"
BOX_HANDLE = "box/top_handle"
BOX_CONTACT = "box/bottom_surface"
TABLE_CONTACT = "staubli_table/drop_zone"

ROBOT_URDF = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
ROBOT_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/staubli_tx2_60l_manipulation.srdf"
)
CELL_URDF = "package://mfja_3rd_floor_description/urdf/room315_cell.urdf"
CELL_SRDF = "package://mfja_3rd_floor_description/urdf/room315_cell.srdf"
TABLE_URDF = (
    "package://mfja_staubli_manipulation_demos/hpp/room315_staubli_table_drop_zone.urdf"
)
TABLE_SRDF = (
    "package://mfja_staubli_manipulation_demos/hpp/room315_staubli_table_drop_zone.srdf"
)
BOX_URDF = "package://mfja_staubli_manipulation_demos/hpp/room315_payload_box.urdf"
BOX_SRDF = "package://mfja_staubli_manipulation_demos/hpp/room315_payload_box.srdf"


def se3_from_pose(pose):
    x, y, z, roll, pitch, yaw = pose
    return pin.SE3(
        pin.rpy.rpyToMatrix(roll, pitch, yaw),
        np.array([x, y, z]),
    )


ROBOT_IN_WORLD = se3_from_pose(ROOM315_ROBOT_POSE)


def configuration_with_box(robot, q_arm, box_world_pose):
    box_in_robot = ROBOT_IN_WORLD.inverse() * se3_from_pose(box_world_pose)
    return np.r_[
        q_arm,
        box_in_robot.translation,
        pin.Quaternion(box_in_robot.rotation).coeffs(),
    ]


def project_free(problem, graph, configuration, label):
    success, projected, error = graph.applyStateConstraints(
        graph.getState("free"), configuration
    )
    if not success:
        raise RuntimeError(f"failed to project {label}: {error:.3g}")
    projected = np.asarray(projected).flatten()
    valid, report = problem.isConfigValid(projected)
    if not valid:
        raise RuntimeError(f"{label} is invalid: {report}")
    return projected


def build_problem():
    robot = Device("room315_staubli_manipulation")
    urdf.loadModel(
        robot, 0, "staubli", "anchor", ROBOT_URDF, ROBOT_SRDF, pin.SE3.Identity()
    )
    urdf.loadModel(
        robot,
        0,
        "room315",
        "anchor",
        CELL_URDF,
        CELL_SRDF,
        ROBOT_IN_WORLD.inverse(),
    )
    urdf.loadModel(
        robot,
        0,
        "staubli_table",
        "anchor",
        TABLE_URDF,
        TABLE_SRDF,
        ROBOT_IN_WORLD.inverse() * se3_from_pose(TABLE_DROP_ZONE_WORLD_POSE),
    )
    urdf.loadModel(robot, 0, "box", "freeflyer", BOX_URDF, BOX_SRDF, pin.SE3.Identity())
    robot.setJointBounds(
        "box/root_joint",
        [
            -1.2,
            1.2,
            -1.0,
            1.2,
            -0.4,
            0.8,
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
            -float("inf"),
            float("inf"),
        ],
    )

    problem = Problem(robot)
    problem.addConfigValidation("CollisionValidation")
    problem.addConfigValidation("JointBoundValidation")

    graph = Graph("room315_staubli_pick_place", robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(1e-5)

    factory = ConstraintGraphFactory(graph)
    factory.setGrippers([GRIPPER])
    factory.setObjects(["box"], [[BOX_HANDLE]], [[BOX_CONTACT]])
    factory.environmentContacts([TABLE_CONTACT])
    factory.generate()

    margins = SecurityMargins(
        problem,
        factory,
        ["staubli", "box", "room315", "staubli_table"],
        robot,
    )
    margins.setSecurityMarginBetween("box", "room315", 0.03)
    margins.setSecurityMarginBetween("staubli", "room315", 0.02)
    margins.apply()

    graph.initialize()
    problem.constraintGraph(graph)
    return robot, problem, graph


def display():
    from pyhpp_viser import Viewer

    viewer = Viewer(robot)
    viewer.initViewer(open=False, loadModel=True)
    viewer.setProblem(problem)
    viewer.setGraph(graph)
    return viewer


robot, problem, graph = build_problem()
q_init = project_free(
    problem,
    graph,
    configuration_with_box(robot, Q_ARM_START, BOX_START_WORLD_POSE),
    "initial configuration",
)
q_goal = project_free(
    problem,
    graph,
    configuration_with_box(robot, Q_ARM_START, BOX_GOAL_WORLD_POSE),
    "goal configuration",
)
q_start = project_free(
    problem,
    graph,
    configuration_with_box(robot, Q_ARM_INITIAL, BOX_START_WORLD_POSE),
    "initial arm configuration",
)

print("Room 315 Stäubli scene initialized")
print(f"configuration size: {robot.configSize()}")
print(f"gripper: {GRIPPER}")
print(f"handle: {BOX_HANDLE}")
print(f"support: {TABLE_CONTACT}")
