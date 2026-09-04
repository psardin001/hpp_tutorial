"""Plan the MFJA gear transfer from the plate to the support."""

import numpy as np
from pinocchio import SE3, neutral
from pyhpp.core import ConfigProjector
from pyhpp.manipulation import Device, Graph, ManipulationPlanner, Problem, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory

robot = Device("mfja")

# Load Stäubli robot
urdf_filename = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/staubli_tx2_60l.srdf"
urdf.loadModel(
    robot, 0, "staubli", "anchor", urdf_filename, srdf_filename, SE3.Identity()
)

# Load gear plate
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_plate.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_plate.srdf"
pose = SE3.Identity()
pose.translation = np.array([0.6, 0.15, 0.0])
urdf.loadModel(robot, 0, "gear_plate", "anchor", urdf_filename, srdf_filename, pose)

# Load gear support
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_support.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_support.srdf"
pose.translation = np.array([0.6, -0.15, 0.0])
urdf.loadModel(robot, 0, "gear_support", "anchor", urdf_filename, srdf_filename, pose)

# Load 42 mm gear
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_42.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_42.srdf"
urdf.loadModel(
    robot,
    0,
    "gear_42",
    "freeflyer",
    urdf_filename,
    srdf_filename,
    SE3.Identity(),
)
robot.setJointBounds(
    "gear_42/root_joint",
    [-1.0, 1.0, -1.0, 1.0, -0.2, 1.5],
)

problem = Problem(robot)
graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)
graph.maxIterations(40)
graph.errorThreshold(1e-5)

factory.setGrippers(["staubli/tool0_gripper", "gear_support/gear_42"])
objects = ["gear_42"]
handles_per_object = [["gear_42/stud", "gear_42/gear_support"]]
contacts_per_object = [["gear_42/bottom"]]
factory.setObjects(objects, handles_per_object, contacts_per_object)
factory.environmentContacts(["gear_plate/top"])
factory.setPossibleGrasps(
    {
        "staubli/tool0_gripper": ["gear_42/stud"],
        "gear_support/gear_42": ["gear_42/gear_support"],
    }
)
factory.generate()
graph.initialize()

q = neutral(robot.model())

# Build a configuration where gear_42 is placed on gear_plate/placement_1.
g = robot.grippers()["gear_plate/placement_1"]
h = robot.handles()["gear_42/placement"]
grasp = h.createGrasp(g, "gear_plate/placement_1 grasps gear_42/placement")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q1, status = cp.solver().solve(q)

# Build a configuration where gear_42 is placed on gear_support/gear_42.
g = robot.grippers()["gear_support/gear_42"]
h = robot.handles()["gear_42/gear_support"]
grasp = h.createGrasp(g, "gear_support/gear_42 grasps gear_42/gear_support")
cp = ConfigProjector(robot, "solver", 1e-5, 40)
cp.add(grasp, 0)
q2, status = cp.solver().solve(q)

# Solve a manipulation problem between q1 and q2.
problem.initConfig(q1)
problem.addGoalConfig(q2)
problem.constraintGraph(graph)
planner = ManipulationPlanner(problem)
planner.maxIterations(1000)
p = planner.solve()
