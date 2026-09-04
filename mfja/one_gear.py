import numpy as np
from pinocchio import SE3, neutral
from pyhpp.constraints import (ComparisonType, ComparisonTypes, Implicit, Transformation)
from pyhpp.core import (ConfigProjector, Discretized, Progressive, RandomShortcut)
from pyhpp.manipulation import (Device, Graph, GraphPathValidation,
                                Problem, ManipulationPlanner, SplineGradientBased_bezier3, urdf)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp_toppra import Toppra
from pyhpp_viser import Viewer


robot = Device("mfja")

# Load Staubli robot
urdf_filename = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/staubli_tx2_60l.srdf"

urdf.loadModel(robot, 0, "staubli", "anchor", urdf_filename, srdf_filename, SE3.Identity())

# Load gear plate
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_plate.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_plate.srdf"
pose = SE3.Identity()
pose.translation = np.array([.6, .15, 0.])

urdf.loadModel(robot, 0, "gear_plate", "anchor", urdf_filename, srdf_filename,
               pose)

# Load gear support
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_support.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_support.srdf"
pose.translation = np.array([.6, -.15, 0.])

urdf.loadModel(robot, 0, "gear_support", "anchor", urdf_filename, srdf_filename,
               pose)

# Load 42 mm gear
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_42.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_42.srdf"

urdf.loadModel(robot, 0, "gear_42", "freeflyer", urdf_filename, srdf_filename, SE3.Identity())

robot.setJointBounds("gear_42/root_joint", [-1., 1.,
    -1., 1.,
    -0.2, 1.5,])

problem = Problem(robot)
problem.pathValidation(GraphPathValidation(Progressive(robot, .001)))
problem.pathValidationFactory(GraphPathValidation(Progressive(robot, .001)))

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
factory.setPossibleGrasps({"staubli/tool0_gripper": ["gear_42/stud"],
                           "gear_support/gear_42_1": ["gear_42/gear_support"]})
factory.generate()
# Force linear motion when placing gear_42 on support
h = robot.handles()["gear_42/gear_support"]
f = Transformation("vertical gear_42", robot, h.getParentJointId(), h.localPosition,
                   SE3.Identity(), [True, True, False, True, True, True])
cts = ComparisonTypes()
cts[:] = [ComparisonType.Equality, ComparisonType.Equality]
vertical_gear_42 = Implicit(f, cts, [True, True, True, True, True])
transition = graph.getTransition("gear_support/gear_42_1 > gear_42/gear_support | 0-0_12")
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42])
transition = graph.getTransition("gear_support/gear_42_1 < gear_42/gear_support | 0-0:1-1_21")
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42])

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
        "staubli/tool0_gripper > gear_42/stud | 1-1_12"
        ]:
    transition = graph.getTransition(tr)
    graph.setSecurityMarginForTransition(transition, "staubli/joint_6", "gear_42/root_joint",
                                         float("-inf"))

graph.initialize()

q = neutral(robot.model())

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
manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(1000)
p = manipulationPlanner.solve()

# Optimize the path
opt1 = RandomShortcut(problem)
opt1.maxIterations(1000)
p1 = opt1.optimize(p)

toppra = Toppra(problem)
toppra.velocityScale = 0.5
toppra.N = 100
toppra.selectJoints([f"staubli/joint_{i}" for i in range(1,7)])
toppra.accelerationLimits = np.array(6 * [0.5])
p2 = toppra.optimize(p1)
