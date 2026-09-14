"""Load the fixed MFJA cell and gear fixtures in the Staubli base frame."""

from pathlib import Path

import numpy as np
import yaml
from pinocchio import SE3, neutral
from pinocchio.rpy import rpyToMatrix
from pyhpp.manipulation import urdf
from pyhpp.tools.xacro import retrieve_resource


def load_scene(robot):
    """Use the Room 315 robot placement and the MFJA table work surface."""
    robots = yaml.safe_load(
        Path(
            retrieve_resource(
                "package://mfja_robot_control_config/config/robots_room_315_only.yaml"
            )
        ).read_text()
    )
    staubli = next(item for item in robots["robots"] if item["name"] == "staubli1")
    world_from_base = SE3(
        rpyToMatrix(0.0, 0.0, staubli["yaw"]),
        np.array([staubli[key] for key in ("x_pose", "y_pose", "z_pose")]),
    )
    scene = yaml.safe_load(
        Path(
            retrieve_resource(
                "package://mfja_staubli_manipulation_demos/config/room315_pick_place.yaml"
            )
        ).read_text()
    )["scene"]
    table_z = scene["table_drop_zone_pose"][2] - staubli["z_pose"]
    package = "package://mfja_3rd_floor_description"
    urdf.loadModel(
        robot,
        0,
        "staubli",
        "anchor",
        f"{package}/urdf/staubli_tx2_60l.urdf",
        f"{package}/srdf/staubli_tx2_60l.srdf",
        SE3.Identity(),
    )
    urdf.loadModel(
        robot,
        0,
        "room315",
        "anchor",
        f"{package}/urdf/room315_environment.urdf",
        "",
        world_from_base.inverse(),
    )
    for name, x, y in (
        ("gear_plate", 0.52453, -0.1815),
        ("gear_support", 0.58753, 0.039),
    ):
        pose = SE3(np.eye(3), np.array([x, y, table_z]))
        urdf.loadModel(
            robot,
            0,
            name,
            "anchor",
            f"{package}/urdf/{name}.urdf",
            f"{package}/srdf/{name}.srdf",
            pose,
        )


def initial_configuration(robot):
    """Start with the arm posture configured for the MFJA pick-and-place demo."""
    planning = yaml.safe_load(
        Path(
            retrieve_resource(
                "package://mfja_staubli_manipulation_demos/config/room315_pick_place.yaml"
            )
        ).read_text()
    )["planning"]
    q = neutral(robot.model())
    q[:6] = planning["default_configuration"]
    return q


def set_environment_margins(graph, margin=0.002):
    """Set the arm/fixture security margin in metres for every transition."""
    for transition in graph.getTransitions():
        for i in range(1, 7):
            graph.setSecurityMarginForTransition(
                transition, f"staubli/joint_{i}", "universe", margin
            )
