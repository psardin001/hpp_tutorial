import numpy as np
from environment import initial_configuration, load_scene, set_environment_margins
from pinocchio import SE3
from pyhpp.constraints import (
    ComparisonType,
    ComparisonTypes,
    Implicit,
    RelativeTransformationR3xSO3,
    Transformation,
)
from pyhpp.core import ConfigProjector, Progressive, ProgressiveProjector
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
from pyhpp_viser import Viewer  # noqa: F401
from tools import SplineToppra

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
problem.pathProjector(
    ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.2)
)

graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)
graph.maxIterations(40)
graph.errorThreshold(1e-5)

factory.setGrippers(["staubli/tool0_gripper", "gear_support/gear_42_1"])
objects = ["gear_42"]
handlesPerObject = [["gear_42/stud", "gear_42/gear_support"]]
contactsPerObject = [["gear_42/bottom"]]
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.environmentContacts(["gear_plate/top"])
factory.setPossibleGrasps(
    {
        "staubli/tool0_gripper": ["gear_42/stud"],
        "gear_support/gear_42_1": ["gear_42/gear_support"],
    }
)
factory.generate()
# Force linear motion when placing gear_42 on support
h = robot.handles()["gear_42/gear_support"]
f = Transformation(
    "vertical gear_42",
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
vertical_gear_42 = Implicit(f, cts, [True, True, True, True, True])
transition = graph.getTransition(
    "gear_support/gear_42_1 > gear_42/gear_support | 0-0_12"
)
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42])
transition = graph.getTransition(
    "gear_support/gear_42_1 < gear_42/gear_support | 0-0:1-1_21"
)
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42])

# Guide the gripper along the stud approach axis, keeping its orientation.
g = robot.grippers()["staubli/tool0_gripper"]
h = robot.handles()["gear_42/stud"]
f = RelativeTransformationR3xSO3(
    "axial gripper",
    robot,
    g.getParentJointId(),
    h.getParentJointId(),
    g.localPosition,
    h.localPosition,
    [True, True, False, True, True, True],
)
axial_gripper = Implicit(f, cts, [True, True, True, True, True])
for name in (
    "staubli/tool0_gripper > gear_42/stud | f_12",
    "staubli/tool0_gripper < gear_42/stud | 0-0_21",
    "staubli/tool0_gripper < gear_42/stud | 0-0:1-1_21",
    "staubli/tool0_gripper > gear_42/stud | 1-1_12",
):
    transition = graph.getTransition(name)
    graph.addNumericalConstraintsToTransition(transition, [axial_gripper])
    graph.setShort(transition, True)

# Deactive collision checking between gripper and gear_42 when grasped
for tr in [
    "staubli/tool0_gripper > gear_42/stud | f_12",
    "staubli/tool0_gripper < gear_42/stud | 0-0_21",
    "staubli/tool0_gripper > gear_42/stud | f_23",
    "staubli/tool0_gripper < gear_42/stud | 0-0_32",
    "staubli/tool0_gripper > gear_42/stud | f_34",
    "staubli/tool0_gripper < gear_42/stud | 0-0_43",
    "Loop | 0-0",
    "gear_support/gear_42_1 > gear_42/gear_support | 0-0_01",
    "gear_support/gear_42_1 < gear_42/gear_support | 0-0:1-1_10",
    "gear_support/gear_42_1 > gear_42/gear_support | 0-0_12",
    "gear_support/gear_42_1 < gear_42/gear_support | 0-0:1-1_21",
    "staubli/tool0_gripper < gear_42/stud | 0-0:1-1_21",
    "staubli/tool0_gripper > gear_42/stud | 1-1_12",
]:
    transition = graph.getTransition(tr)
    graph.setSecurityMarginForTransition(
        transition, "staubli/joint_6", "gear_42/root_joint", float("-inf")
    )

set_environment_margins(graph)
graph.initialize()

q = initial_configuration(robot)
q[:6] = globals().get("q_start", q[:6])

# Build a configuration where gear_42 is placed on gripper gear_placement/placement_1
g = robot.grippers()["gear_plate/placement_1"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_1 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q1, status = cp.solver().solve(q)

# Build a configuration where gear_42 is placed on gripper gear_support/gear_42_1

g = robot.grippers()["gear_support/gear_42_1"]
h = robot.handles()["gear_42/gear_support"]
grasp = h.createGrasp(g, "gear_support/gear_42_1 grasps gear_42/gear_support")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q2, status = cp.solver().solve(q)

# Solving a manipulation problem between q1 and q2
problem.initConfig(q1)
problem.addGoalConfig(q2)
problem.constraintGraph(graph)
manipulationPlanner = StatesPathFinder(problem)
manipulationPlanner.maxIterations(1000)
p = manipulationPlanner.solve()

# Optimize the path
opt1 = GraphRandomShortcut(problem)
opt1.maxIterations(1000)
p1 = opt1.optimize(p)

toppra = SplineToppra(problem, graph)
toppra.velocityScale = 0.5
toppra.N = 100
toppra.selectJoints([f"staubli/joint_{i}" for i in range(1, 7)])
# Reserve 10% of the 0.5 rad/s² limit for numerical projection effects.
toppra.accelerationLimits = np.array(6 * [0.45])
p2 = toppra.optimize(p1)
