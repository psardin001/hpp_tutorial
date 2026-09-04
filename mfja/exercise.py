"""Starting point for the MFJA gear-transfer exercise."""

import numpy as np
from pinocchio import SE3, neutral
from pyhpp.core import ConfigProjector  # noqa: F401
from pyhpp.manipulation import (  # noqa: F401
    Device,
    Graph,
    ManipulationPlanner,
    Problem,
    urdf,
)
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory

robot = Device("mfja")

# Load the Stäubli robot.
urdf_filename = "package://mfja_3rd_floor_description/urdf/staubli_tx2_60l.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/staubli_tx2_60l.srdf"
urdf.loadModel(
    robot, 0, "staubli", "anchor", urdf_filename, srdf_filename, SE3.Identity()
)

# Load the gear plate.
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_plate.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_plate.srdf"
pose = SE3.Identity()
pose.translation = np.array([0.6, 0.15, 0.0])
urdf.loadModel(robot, 0, "gear_plate", "anchor", urdf_filename, srdf_filename, pose)

# Load the gear support.
urdf_filename = "package://mfja_3rd_floor_description/urdf/gear_support.urdf"
srdf_filename = "package://mfja_3rd_floor_description/srdf/gear_support.srdf"
pose.translation = np.array([0.6, -0.15, 0.0])
urdf.loadModel(robot, 0, "gear_support", "anchor", urdf_filename, srdf_filename, pose)

# Load the 42 mm gear.
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

# Exercise 1: project q on the placement constraint between
# gear_plate/placement_1 and gear_42/placement. Store the result in q1.
q1 = None

# Exercise 2: project q on the placement constraint between
# gear_support/gear_42 and gear_42/gear_support. Store the result in q2.
q2 = None

# Exercise 3: configure the problem and solve it with ManipulationPlanner.
# Store the resulting path in p.
p = None
