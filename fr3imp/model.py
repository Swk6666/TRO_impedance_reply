"""Build independent plant and nominal models without editing the vendor asset.

Only the plant contains friction, damping, uncertain inertial parameters,
contacts, and the instrumented tool. Both models use identical kinematics.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DT = 0.001
TORQUE_LIMIT = np.array([87., 87., 87., 87., 12., 12., 12.])
HOME = np.array([0., 0., 0., -1.57079, 0., 1.57079, -0.7853])


@dataclass(frozen=True)
class PlantParameters:
    friction_scale: float = 1.0
    damping_scale: float = 1.0
    mass_error: float = 0.10
    tool_mass: float = 0.4
    nominal_tool_mass: float = 0.0
    timestep: float = .00025
    contact_timeconstant: float = .004
    contact_impedance: tuple = (.95,.99,.001)
    contact_friction: float = .25


def build_model(parameters=PlantParameters(), *, nominal=False,
                environment=None, visuals=True) -> mujoco.MjModel:
    root = ET.parse(ROOT / "franka_fr3/fr3.xml").getroot()
    if not visuals:
        # Rendering meshes have no collision role and articulated bodies have
        # explicit inertias. Remove only visuals, retaining every collision
        # mesh, to reduce memory during multi-process experiments.
        for body in root.findall('.//worldbody//body'):
            for geom in list(body.findall('geom')):
                if geom.get('class')=='visual':body.remove(geom)
        used={g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
        assets=root.find('asset')
        for mesh in list(assets.findall('mesh')):
            name=mesh.get('name',Path(mesh.get('file')).stem)
            if name not in used:assets.remove(mesh)
    root.find("compiler").set("meshdir", str(ROOT / "franka_fr3/assets"))
    option = root.find("option")
    option.set("timestep", str(parameters.timestep))
    option.set("cone", "elliptic")
    option.set("iterations", "80")
    option.set("tolerance", "1e-10")
    default_joint = root.find("default/default/joint")
    default_joint.set("damping", "0")
    default_joint.set("frictionloss", "0")
    for i in range(7):
        joint = root.find(f".//worldbody//joint[@name='fr3_joint{i+1}']")
        joint.set("damping", str(0.0 if nominal else .21 * parameters.damping_scale))
        friction = [1.137, 1.137, 1.137, 1.137, .763, .44, .248][i]
        joint.set("frictionloss", str(0.0 if nominal else friction * parameters.friction_scale))
        body = root.find(f".//body[@name='fr3_link{i+1}']")
        if not nominal:
            inertial = body.find("inertial")
            scale = 1 + parameters.mass_error * np.sin(i + 1.)
            inertial.set("mass", str(float(inertial.get("mass")) * scale))
            inertial.set("diaginertia", " ".join(str(float(v)*scale) for v in inertial.get("diaginertia").split()))

    actuator = root.find("actuator")
    actuator.clear()
    for i, limit in enumerate(TORQUE_LIMIT, 1):
        ET.SubElement(actuator, "motor", name=f"motor{i}", joint=f"fr3_joint{i}",
                      gear="1", ctrllimited="true", ctrlrange=f"{-limit} {limit}")
    root.find("keyframe/key").set("ctrl", "0 0 0 0 0 0 0")

    # Physical wrist -> welded tool -> TCP. The F/T sensor measures the
    # transmitted wrench, not a joint-actuator torque or qfrc_constraint.
    link7 = root.find(".//body[@name='fr3_link7']")
    tool = ET.SubElement(link7, "body", name="tool", pos="0 0 0.107")
    mass = parameters.nominal_tool_mass if nominal else parameters.tool_mass
    if mass > 0:
        ET.SubElement(tool, "inertial", pos="0 0 0.06", mass=str(mass),
                      diaginertia=f"{mass*.0015} {mass*.0015} {mass*.0001}")
    ET.SubElement(tool, "site", name="wrist", pos="0 0 0", size="0.005")
    ET.SubElement(tool, "site", name="tcp", pos="0 0 0.12", size="0.008", rgba="1 0.2 0.1 1")
    ET.SubElement(tool, "geom", name="tool_shaft", type="capsule", size="0.008",
                  fromto="0 0 0 0 0 0.11", mass="0", contype="0", conaffinity="0")
    ET.SubElement(tool, "geom", name="tool_tip", type="sphere", size="0.01", pos="0 0 0.12",
                  mass="0", contype="0" if nominal else "2", conaffinity="0" if nominal else "4",
                  friction=f"{parameters.contact_friction} 0.005 0.0001", solref=f"{parameters.contact_timeconstant} 1", solimp=" ".join(map(str,parameters.contact_impedance)))
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "force", name="wrist_force", site="wrist")
    ET.SubElement(sensors, "torque", name="wrist_torque", site="wrist")

    if not nominal:
        world = root.find("worldbody")
        ET.SubElement(world, "light", pos="1 -1 2", dir="-0.5 0.5 -1", directional="true")
        ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 .1",
                      pos="0 0 -0.02", rgba="0.25 0.28 0.32 1", contype="4", conaffinity="3")
        for spec in environment or []:
            attrs = {k: str(v) for k, v in spec.items()}
            defaults = dict(type="box", contype="4", conaffinity="2", friction=f"{parameters.contact_friction} 0.005 0.0001",
                            solref=f"{parameters.contact_timeconstant} 1", solimp=" ".join(map(str,parameters.contact_impedance)), rgba="0.7 0.75 0.8 1")
            defaults.update(attrs)
            ET.SubElement(world, "geom", **defaults)
    else:
        # No collisions or contacts in the controller's forward model.
        for geom in root.iter("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def home_data(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return data
