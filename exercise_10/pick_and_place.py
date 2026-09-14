import numpy as np
from environment import initial_configuration, load_scene, set_environment_margins
from pinocchio import SE3
from pyhpp.core import ConfigProjector, Progressive
from pyhpp.core.path import Vector
from pyhpp.manipulation import (
    Device,
    EnforceTransitionSemantic,
    Graph,
    GraphPathValidation,
    GraphRandomShortcut,
    ManipulationPlanner,
    Problem,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp_viser import Viewer
from tools import SplineToppra


def display():
    v = Viewer(robot)
    v.initViewer(open=False, loadModel=True)
    v.setProblem(problem)
    v.setGraph(graph)
    return v


robot = Device("mfja")

# Load Staubli robot in MFJA room 315 environment
load_scene(robot)

# Keep the arm facing the fixtures with the elbow in its initial posture.
robot.setJointBounds("staubli/joint_1", np.deg2rad([-90.0, 90.0]).tolist())
robot.setJointBounds("staubli/joint_3", [0.0, robot.model().upperPositionLimit[2]])

# Load 42 mm gear
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_42.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_42.srdf"

urdf.loadModel(
    robot, 0, "gear_42", "freeflyer", urdf_filename, srdf_filename, SE3.Identity()
)

robot.setJointBounds(
    "gear_42/root_joint",
    [
        -1.0,
        1.0,
        -1.0,
        1.0,
        -0.2,
        1.5,
    ],
)

problem = Problem(robot)
problem.pathValidation(GraphPathValidation(Progressive(robot, 0.001)))
problem.pathValidationFactory(GraphPathValidation(Progressive(robot, 0.001)))

graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)
graph.maxIterations(40)
graph.errorThreshold(1e-5)

factory.setGrippers(["staubli/tool0_gripper"])
objects = ["gear_42"]
handlesPerObject = [["gear_42/stud"]]
contactsPerObject = [["gear_42/bottom"]]
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.environmentContacts(["gear_plate/top"])
factory.generate()

# Deactive collision checking between gripper and gear_42 when grasped
for tr in [
    "staubli/tool0_gripper > gear_42/stud | f_12",
    "staubli/tool0_gripper < gear_42/stud | 0-0_21",
    "staubli/tool0_gripper > gear_42/stud | f_23",
    "staubli/tool0_gripper < gear_42/stud | 0-0_32",
    "staubli/tool0_gripper > gear_42/stud | f_34",
    "staubli/tool0_gripper < gear_42/stud | 0-0_43",
    "Loop | 0-0",
]:
    transition = graph.getTransition(tr)
    graph.setSecurityMarginForTransition(
        transition, "staubli/joint_6", "gear_42/root_joint", float("-inf")
    )

set_environment_margins(graph, margin=0.005)
# Initialize the graph once constructed
graph.initialize()

q = initial_configuration(robot)

# Read initial configuration robot in global variable q_start if defined
q[:6] = globals().get("q_start", q[:6])

# Build a configuration where gear_42 is placed on gripper gear_placement/placement_1
g = robot.grippers()["gear_plate/placement_1"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_1 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q1, status = cp.solver().solve(q)
assert status

# Build a configuration where gear_42 is placed on gripper gear_placement/placement_2
g = robot.grippers()["gear_plate/placement_2"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_2 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q2, status = cp.solver().solve(q)
assert status


def solve(q_start=None):
    """Plan, optimize and time the transfer, returning to the starting arm pose."""
    # Solving a manipulation problem between q1 and q2
    start, goal = q1.copy(), q2.copy()
    if q_start is not None:
        start[:6] = q_start
        goal[:6] = q_start
    problem.initConfig(start)
    problem.resetGoalConfigs()
    problem.addGoalConfig(goal)
    problem.constraintGraph(graph)
    manipulationPlanner = ManipulationPlanner(problem)
    manipulationPlanner.maxIterations(1000)
    p = manipulationPlanner.solve()

    # Optimize the path
    opt1 = GraphRandomShortcut(problem)
    opt1.maxIterations(1000)
    p1 = opt1.optimize(p)

    semantic = EnforceTransitionSemantic(problem)
    p1 = semantic.optimize(p1)

    toppra = SplineToppra(problem, graph)
    toppra.singleSplineTransitions = (
        "gear_support/gear_42_1 > gear_42/gear_support | 0-0_12",
    )
    toppra.velocityScale = 0.5
    toppra.N = 100
    toppra.selectJoints([f"staubli/joint_{i}" for i in range(1, 7)])
    # Reserve 10% of the 0.5 rad/s² limit for numerical projection effects.
    toppra.accelerationLimits = np.array(6 * [0.45])
    p_timed = toppra.optimize(p1)

    # Validate each transition and keep its endpoints for sampling.
    flat_path = Vector(p_timed.outputSize(), p_timed.outputDerivativeSize())
    p_timed.flatten(flat_path)
    path_times = [0.0]
    for i in range(flat_path.numberPaths()):
        leaf = flat_path.pathAtRank(i)
        transition = graph.transitionAtParam(
            p_timed, path_times[-1] + leaf.length() / 2
        )
        valid, _, report = transition.pathValidation().validate(leaf, False)
        if not valid:
            raise RuntimeError(f"Invalid timed subpath {i}: {report}")
        path_times.append(path_times[-1] + leaf.length())
    return p, p1, p_timed, path_times


if __name__ == "__main__":
    p, p1, p_timed, path_times = solve()
    p2 = p_timed
