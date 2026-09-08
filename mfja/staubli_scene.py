"""Read the Room 315 fixture poses used by the MFJA world."""

from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml


def placement(values):
    x, y, z, roll, pitch, yaw = values
    return pin.SE3(pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z]))


def read_room(mfja_root):
    """Convert the world's fixed mesh/box fixtures into one anchored URDF."""
    root = Path(mfja_root)
    description = root / "mfja_3rd_floor_description"
    robots = yaml.safe_load(
        (
            root / "mfja_robot_control_config/config/robots_room_315_only.yaml"
        ).read_text()
    )["robots"]
    robot = next(item for item in robots if item["name"] == "staubli1")
    base_pose = [robot[key] for key in ("x_pose", "y_pose", "z_pose")]
    base_pose += [0.0, 0.0, robot["yaw"]]
    world = ET.parse(description / "worlds/room_315_only.world")
    urdf = ET.Element("robot", name="room315")
    ET.SubElement(urdf, "link", name="world")
    for include in world.findall("./world/include"):
        model_name = include.findtext("uri").removeprefix("model://")
        if model_name in ("sun", "ground_plane"):
            continue
        model = ET.parse(description / "models" / model_name / "model.sdf").find(
            "model"
        )
        # The Room 315 fixture models are single rigid links.
        if len(model.findall("link")) != 1 or model.findall("joint"):
            raise ValueError(f"Expected a rigid fixture: {model_name}")
        link = model.find("link")
        pose = pin.SE3.Identity()
        for element in (include, model, link):
            pose = pose * placement(
                [float(x) for x in element.findtext("pose", "0 0 0 0 0 0").split()]
            )
        name = include.findtext("name")
        target = ET.SubElement(urdf, "link", name=name)
        joint = ET.SubElement(urdf, "joint", name=name + "_joint", type="fixed")
        ET.SubElement(joint, "parent", link="world")
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(map(str, pose.translation)),
            rpy=" ".join(map(str, pin.rpy.matrixToRpy(pose.rotation))),
        )
        for collision in link.findall("collision"):
            item = ET.SubElement(target, "collision")
            values = collision.findtext("pose", "0 0 0 0 0 0").split()
            ET.SubElement(
                item, "origin", xyz=" ".join(values[:3]), rpy=" ".join(values[3:])
            )
            geometry = ET.SubElement(item, "geometry")
            shape = collision.find("geometry")[0]
            if shape.tag == "mesh":
                uri = shape.findtext("uri").replace(
                    "model://", "package://mfja_3rd_floor_description/models/"
                )
                ET.SubElement(
                    geometry,
                    "mesh",
                    filename=uri,
                    scale=shape.findtext("scale", "1 1 1"),
                )
            elif shape.tag == "box":
                ET.SubElement(geometry, "box", size=shape.findtext("size"))
            else:
                raise ValueError(f"Unsupported fixture geometry: {shape.tag}")
            visual = deepcopy(item)
            visual.tag = "visual"
            target.append(visual)
    return ET.tostring(urdf, encoding="unicode"), base_pose
