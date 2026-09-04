import numpy as np
from pinocchio import SE3, neutral
from pyhpp.constraints import (ComparisonType, ComparisonTypes, Implicit, Transformation)
from pyhpp.core import (ConfigProjector, Discretized, ProgressiveProjector, Progressive,
                        RandomShortcut)
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

# Load 2 instances of 42 mm gear
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_42.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_42.srdf"

urdf.loadModel(robot, 0, "gear_42_1", "freeflyer", urdf_filename, srdf_filename,
               SE3.Identity())
robot.setJointBounds("gear_42_1/root_joint", [-1., 1.,
                                              -1., 1.,
                                              -0.2, 1.5,])
urdf.loadModel(robot, 0, "gear_42_2", "freeflyer", urdf_filename, srdf_filename,
               SE3.Identity())
robot.setJointBounds("gear_42_2/root_joint", [-1., 1.,
                                              -1., 1.,
                                              -0.2, 1.5,])

problem = Problem(robot)
problem.pathValidation(GraphPathValidation(Progressive(robot, .001)))
problem.pathValidationFactory(GraphPathValidation(Progressive(robot, .001)))
problem.pathProjector(ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.2))

graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)
graph.maxIterations(40)
graph.errorThreshold(1e-5)

factory.setGrippers(["staubli/tool0_gripper", "gear_support/gear_42_1", "gear_support/gear_42_2"])
objects = ["gear_42_1", "gear_42_2"]
handlesPerObject = [["gear_42_1/stud", "gear_42_1/gear_support"],
                    ["gear_42_2/stud", "gear_42_2/gear_support"]]
contactsPerObject = [["gear_42_1/bottom"], ["gear_42_2/bottom"]]
factory.setObjects(objects, handlesPerObject, contactsPerObject)
factory.environmentContacts(["gear_plate/top"])
factory.setPossibleGrasps({"staubli/tool0_gripper": ["gear_42_1/stud", "gear_42_2/stud"],
                           "gear_support/gear_42_1": ["gear_42_1/gear_support"],
                           "gear_support/gear_42_2": ["gear_42_2/gear_support"]})
factory.generate()
# Force linear motion when placing gear_42_1 on support
h = robot.handles()["gear_42_1/gear_support"]
f = Transformation("vertical gear_42_1", robot, h.getParentJointId(), h.localPosition,
                   SE3.Identity(), [True, True, False, True, True, True])
cts = ComparisonTypes()
cts[:] = [ComparisonType.Equality, ComparisonType.Equality, ComparisonType.Equality,
          ComparisonType.Equality, ComparisonType.Equality]
vertical_gear_42_1 = Implicit(f, cts, [True, True, True, True, True])
transition = graph.getTransition("gear_support/gear_42_1 > gear_42_1/gear_support | 0-0_12")
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_1])
transition = graph.getTransition("gear_support/gear_42_1 < gear_42_1/gear_support | 0-0:1-1_21")
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_1])

# Force linear motion when placing gear_42_2 on support
h = robot.handles()["gear_42_2/gear_support"]
f = Transformation("vertical gear_42_2", robot, h.getParentJointId(), h.localPosition,
                   SE3.Identity(), [True, True, False, True, True, True])
cts = ComparisonTypes()
cts[:] = [ComparisonType.Equality, ComparisonType.Equality, ComparisonType.Equality,
          ComparisonType.Equality, ComparisonType.Equality]
vertical_gear_42_2 = Implicit(f, cts, [True, True, True, True, True])
transition = graph.getTransition("gear_support/gear_42_2 > gear_42_2/gear_support | 0-2:1-1_12")
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_2])
transition = graph.getTransition("gear_support/gear_42_2 < gear_42_2/gear_support | 0-2:1-1:2-3_21")
graph.addNumericalConstraintsToTransition(transition, [vertical_gear_42_2])

# Deactive collision checking between gripper and gears for all transitions
for transition in graph.getTransitions():
    graph.setSecurityMarginForTransition(transition, "staubli/joint_6", "gear_42_1/root_joint",
                                         float("-inf"))
    graph.setSecurityMarginForTransition(transition, "staubli/joint_6", "gear_42_2/root_joint",
                                         float("-inf"))

graph.initialize()

q = neutral(robot.model())

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

# Solving a manipulation problem between q1 and q2
problem.initConfig(q1)
problem.addGoalConfig(q2)
problem.constraintGraph(graph)
manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(1000)
#manipulationPlanner.roadmap().addNode(q3)
p = manipulationPlanner.solve()

# Optimize the path
opt1 = RandomShortcut(problem)
opt1.maxIterations(5000)
p1 = opt1.optimize(p)

toppra = Toppra(problem)
toppra.velocityScale = 0.5
toppra.N = 100
toppra.selectJoints([f"staubli/joint_{i}" for i in range(1,7)])
toppra.accelerationLimits = np.array(6 * [0.5])
p2 = toppra.optimize(p1)
