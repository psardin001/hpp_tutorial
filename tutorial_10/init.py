import numpy as np
import rclpy
from environment import load_scene
from pinocchio import SE3, neutral
from pyhpp.core import ConfigProjector, Progressive, RandomShortcut
from pyhpp.core.path import Vector
from pyhpp.manipulation import (
    Device,
    EnforceTransitionSemantic,
    Graph,
    GraphPathValidation,
    ManipulationPlanner,
    Problem,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp_viser import Viewer
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from staubli_io import check_position, read_joints, set_gripper, wait_reached
from tools import Toppra

ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
TRAJECTORY_ACTION = "/manipulator_controller/joint_trajectory_action"
execution_config = {
    "joint_state_topic": "/joint_states",
    "joint_tolerance_rad": 0.002,
    "gripper_service": "/io_interface/write_single_io",
    "gripper_pin": 0,
    "gripper_settle_s": 0.5,
}

if not rclpy.ok():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
staubli_node = Node("tutorial_10")


def open_gripper():
    return set_gripper(staubli_node, execution_config, "open")


def close_gripper():
    return set_gripper(staubli_node, execution_config, "close")


def wait_for_motion(node, result, configs):
    return wait_reached(node, execution_config, result, np.asarray(configs)[:, :6])


def check_initial_position(configs):
    return check_position(staubli_node, execution_config, np.asarray(configs[0])[:6])


def display():
    v = Viewer(robot)
    v.initViewer(open=False, loadModel=True)
    v.setProblem(problem)
    v.setGraph(graph)
    return v


# Pick-and-place problem from mfja/pick_and_place.py.
robot = Device("mfja")

load_scene(robot)

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

# Allow contact between the gripper and gear_42 during grasping.
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

graph.initialize()

q = neutral(robot.model())
q[:6] = read_joints(staubli_node, execution_config)

# Place gear_42 at gear_plate/placement_1.
g = robot.grippers()["gear_plate/placement_1"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_1 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q1, status = cp.solver().solve(q)
assert status

# Place gear_42 at gear_plate/placement_2.
g = robot.grippers()["gear_plate/placement_2"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_2 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q2, status = cp.solver().solve(q)
assert status

# Solving a manipulation problem between q1 and q2
problem.initConfig(q1)
problem.addGoalConfig(q2)
problem.constraintGraph(graph)
manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(1000)
p = manipulationPlanner.solve()

# Optimize the path
opt1 = RandomShortcut(problem)
opt1.maxIterations(1000)
p1 = opt1.optimize(p)

semantic = EnforceTransitionSemantic(problem)
p1 = semantic.optimize(p1)

toppra = Toppra(problem)
toppra.velocityScale = 0.5
toppra.N = 100
toppra.selectJoints([f"staubli/joint_{i}" for i in range(1, 7)])
toppra.accelerationLimits = np.array(6 * [0.5])
p_timed = toppra.optimize(p1)

# Validate each transition and keep its endpoints for sampling.
flat_path = Vector(p_timed.outputSize(), p_timed.outputDerivativeSize())
p_timed.flatten(flat_path)
path_times = [0.0]
for i in range(flat_path.numberPaths()):
    leaf = flat_path.pathAtRank(i)
    transition = graph.transitionAtParam(p_timed, path_times[-1] + leaf.length() / 2)
    valid, _, report = transition.pathValidation().validate(leaf, False)
    if not valid:
        raise RuntimeError(f"Invalid timed subpath {i}: {report}")
    path_times.append(path_times[-1] + leaf.length())
