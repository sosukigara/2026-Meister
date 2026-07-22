#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
spawn_robot.py — URDF→SDF converter + Gazebo robot spawner for Meister Robot.

Workaround: ros_gz_sim create fails because Gazebo transport is broken
(gz topic/service/world -l all return empty).  This script converts the
URDF from /robot_description into a proper Gazebo SDF model (model.config
+ model.sdf) and modifies a world file to include the robot, so Gazebo
auto-loads it at startup.

Modes
-----
pregenerate (default)
    Run BEFORE Gazebo starts.  Reads URDF from /robot_description ROS 2
    topic, generates model files under the package's models/ directory,
    and creates a copy of the named world with the robot <include> tag
    added at the configured spawn pose.  Gazebo is then launched with
    this modified world.

    Usage:
        ros2 run meister_robot spawn_robot.py --mode pregenerate [--world maze]

spawn
    Run AFTER Gazebo is running.  Attempts to spawn the robot via direct
    Gazebo transport (gz service ...).  Since GZ transport is known to
    be broken, this mode will likely fail and fall back to instructions.

    Usage:
        ros2 run meister_robot spawn_robot.py --mode spawn

Environment
-----------
GZ_SIM_RESOURCE_PATH
    Must include the parent directory of the model (e.g. the models/
    directory that contains meister_robot/).  The script prints the
    required export command.
"""

import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import threading

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default spawn pose matching meister_slam_nav.launch.py
DEFAULT_SPAWN_POSE = (1.5, 1.0, 0.3, 0.0, 0.0, 0.0)

# Model name registered in model.config
ROBOT_NAME = "meister_robot"

# The source root under which we find models/, worlds/, etc.
# Normally src/meister_robot/ in the workspace or the install share.
_PACKAGE_NAME = "meister_robot"


def _get_package_src_root() -> str:
    """Return the package source root directory.

    Prefers the workspace source tree; falls back to install share.
    """
    # 1) Workspace source via AMENT_PREFIX_PATH
    ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in ament_prefix.split(":"):
        share = os.path.join(prefix, "share", _PACKAGE_NAME)
        if os.path.isdir(share):
            return share

    # 2) Colcon default install
    for base in ("/home/so/Meister/install", "/home/so/Meister"):
        share = os.path.join(base, "share", _PACKAGE_NAME)
        if os.path.isdir(share):
            return share

    # 3) Source tree (development)
    for candidate in (
        "/home/so/Meister/src/meister_robot",
    ):
        if os.path.isdir(candidate):
            return candidate

    # 4) ament_index fallback
    try:
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory(_PACKAGE_NAME)
    except Exception:
        pass

    # 5) Last resort: working directory
    return os.getcwd()


def _resolve_world_source(world_name: str) -> str | None:
    """Find a .world file by base name, checking rosnav first."""
    # Rosnav worlds (original source)
    rosnav_worlds = (
        "/home/so/Meister/src/rosnav/worlds",
        "/home/so/Meister/install/diff_drive_robot/share/diff_drive_robot/worlds",
    )
    for d in rosnav_worlds:
        p = os.path.join(d, f"{world_name}.world")
        if os.path.isfile(p):
            return p

    # Our own worlds
    src_root = _get_package_src_root()
    for d in (src_root, os.path.join(src_root, "worlds")):
        p = os.path.join(d, f"{world_name}.world")
        if os.path.isfile(p):
            return p

    return None


# ---------------------------------------------------------------------------
# URDF → SDF conversion helpers
# ---------------------------------------------------------------------------

_URDF_JOINT_TYPE_MAP = {
    "continuous": "revolute",
    "revolute": "revolute",
    "prismatic": "prismatic",
    "fixed": "fixed",
    "floating": "fixed",
    "planar": "prismatic",
}


def _attr(elem: ET.Element | None, key: str, default: str = "0 0 0") -> str:
    """Get attribute or return default."""
    if elem is None:
        return default
    return elem.get(key, default)


def _origin_to_pose(origin: ET.Element | None) -> str:
    """Convert URDF <origin xyz=... rpy=.../> to SDF pose string."""
    xyz = _attr(origin, "xyz", "0 0 0")
    rpy = _attr(origin, "rpy", "0 0 0")
    return f"{xyz} {rpy}"


def _geom_to_sdf(geom: ET.Element, parent: ET.Element) -> None:
    """Convert URDF geometry to SDF geometry (writes child elements)."""
    for child in geom:
        tag = child.tag
        if tag == "box":
            s = child.get("size", "1 1 1")
            box = ET.SubElement(parent, "box")
            box_sz = ET.SubElement(box, "size")
            box_sz.text = s
        elif tag == "cylinder":
            cyl = ET.SubElement(parent, "cylinder")
            r = ET.SubElement(cyl, "radius")
            r.text = child.get("radius", "0.1")
            l = ET.SubElement(cyl, "length")
            l.text = child.get("length", "0.1")
        elif tag == "sphere":
            sph = ET.SubElement(parent, "sphere")
            r = ET.SubElement(sph, "radius")
            r.text = child.get("radius", "0.1")
        elif tag == "mesh":
            mesh = ET.SubElement(parent, "mesh")
            uri = ET.SubElement(mesh, "uri")
            uri.text = child.get("filename", "")
            sc = child.get("scale")
            if sc:
                s = ET.SubElement(mesh, "scale")
                s.text = sc


def _material_to_sdf(mat: ET.Element | None, parent: ET.Element) -> None:
    """Convert URDF material to SDF material."""
    if mat is None:
        return
    sdf_mat = ET.SubElement(parent, "material")
    color = mat.find("color")
    if color is not None:
        rgba = color.get("rgba", "0.5 0.5 0.5 1.0")
        ambient = ET.SubElement(sdf_mat, "ambient")
        ambient.text = rgba
        diffuse = ET.SubElement(sdf_mat, "diffuse")
        diffuse.text = rgba


def _build_sdf_link(urdf_link: ET.Element, sdf_model: ET.Element,
                    link_friction: float | None = None) -> None:
    """Convert a URDF <link> to an SDF <link> under sdf_model.

    Args:
        urdf_link: The URDF <link> element.
        sdf_model: Parent SDF <model> element.
        link_friction: Optional mu friction coefficient for this link.
    """
    link_name = urdf_link.get("name", "unnamed_link")
    sdf_link = ET.SubElement(sdf_model, "link")
    sdf_link.set("name", link_name)

    # Pose (identity — joints handle positioning)
    pose_elem = ET.SubElement(sdf_link, "pose")
    pose_elem.text = "0 0 0 0 0 0"

    # ── Inertial ────────────────────────────────────────────────────
    inertial = urdf_link.find("inertial")
    if inertial is not None:
        sdf_inertial = ET.SubElement(sdf_link, "inertial")
        origin = inertial.find("origin")
        if origin is not None:
            sdf_origin = ET.SubElement(sdf_inertial, "pose")
            sdf_origin.text = _origin_to_pose(origin)
        else:
            sdf_origin = ET.SubElement(sdf_inertial, "pose")
            sdf_origin.text = "0 0 0 0 0 0"

        mass = inertial.find("mass")
        if mass is not None:
            m = ET.SubElement(sdf_inertial, "mass")
            m.text = mass.get("value", "0.0")

        inertia = inertial.find("inertia")
        if inertia is not None:
            sdf_inertia = ET.SubElement(sdf_inertial, "inertia")
            for axis in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                val = inertia.get(axis, "0")
                elem = ET.SubElement(sdf_inertia, axis)
                elem.text = val
        else:
            # Default tiny inertia
            _add_tiny_inertia(sdf_inertial)
    else:
        # URDF link without inertial — give it a tiny one
        sdf_inertial = ET.SubElement(sdf_link, "inertial")
        sdf_origin = ET.SubElement(sdf_inertial, "pose")
        sdf_origin.text = "0 0 0 0 0 0"
        m = ET.SubElement(sdf_inertial, "mass")
        m.text = "0.001"
        _add_tiny_inertia(sdf_inertial)

    # ── Visual ──────────────────────────────────────────────────────
    for vis in urdf_link.findall("visual"):
        sdf_vis = ET.SubElement(sdf_link, "visual")
        sdf_vis.set("name", vis.get("name", "visual"))
        origin = vis.find("origin")
        if origin is not None:
            p = ET.SubElement(sdf_vis, "pose")
            p.text = _origin_to_pose(origin)
        geom = vis.find("geometry")
        if geom is not None and len(geom) > 0:
            g = ET.SubElement(sdf_vis, "geometry")
            _geom_to_sdf(geom, g)
        _material_to_sdf(vis.find("material"), sdf_vis)

    # ── Collision ───────────────────────────────────────────────────
    for col in urdf_link.findall("collision"):
        sdf_col = ET.SubElement(sdf_link, "collision")
        sdf_col.set("name", col.get("name", "collision"))
        origin = col.find("origin")
        if origin is not None:
            p = ET.SubElement(sdf_col, "pose")
            p.text = _origin_to_pose(origin)
        geom = col.find("geometry")
        if geom is not None and len(geom) > 0:
            g = ET.SubElement(sdf_col, "geometry")
            _geom_to_sdf(geom, g)

        # Friction
        if link_friction is not None:
            surface = ET.SubElement(sdf_col, "surface")
            friction = ET.SubElement(surface, "friction")
            ode = ET.SubElement(friction, "ode")
            mu = ET.SubElement(ode, "mu")
            mu.text = f"{link_friction}"
            mu2 = ET.SubElement(ode, "mu2")
            mu2.text = f"{link_friction}"


def _add_tiny_inertia(parent: ET.Element) -> None:
    """Add a minimal inertia block (avoids physics engine warnings)."""
    inert = ET.SubElement(parent, "inertia")
    for axis in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        e = ET.SubElement(inert, axis)
        e.text = "1e-06" if axis in ("ixx", "iyy", "izz") else "0"


def _build_sdf_joint(urdf_joint: ET.Element, sdf_model: ET.Element) -> None:
    """Convert a URDF <joint> to an SDF <joint> under sdf_model."""
    joint_name = urdf_joint.get("name", "unnamed_joint")
    joint_type = urdf_joint.get("type", "fixed")
    sdf_type = _URDF_JOINT_TYPE_MAP.get(joint_type, "fixed")

    sdf_joint = ET.SubElement(sdf_model, "joint")
    sdf_joint.set("name", joint_name)
    sdf_joint.set("type", sdf_type)

    parent = urdf_joint.find("parent")
    if parent is not None:
        p = ET.SubElement(sdf_joint, "parent")
        p.text = parent.get("link", "")
    child = urdf_joint.find("child")
    if child is not None:
        c = ET.SubElement(sdf_joint, "child")
        c.text = child.get("link", "")

    origin = urdf_joint.find("origin")
    pose_elem = ET.SubElement(sdf_joint, "pose")
    pose_elem.text = _origin_to_pose(origin)

    # Axis (for revolute joints)
    if sdf_type == "revolute" or sdf_type == "prismatic":
        axis_elem = urdf_joint.find("axis")
        if axis_elem is not None:
            sdf_axis = ET.SubElement(sdf_joint, "axis")
            xyz = ET.SubElement(sdf_axis, "xyz")
            xyz.text = axis_elem.get("xyz", "0 0 1")


def _build_sdf_plugins(urdf_root: ET.Element, sdf_model: ET.Element,
                       link_sensors: dict[str, list[ET.Element]]) -> None:
    """Append plugin elements from URDF <gazebo> extensions to SDF model.

    Also attaches sensor elements to their respective SDF links.
    """
    for gazebo in urdf_root.findall("gazebo"):
        ref = gazebo.get("reference", "")

        # ── Sensors (attach to links) ────────────────────────────────
        sensor = gazebo.find("sensor")
        if sensor is not None and ref:
            # Find the SDF link and append the sensor
            sdf_link = sdf_model.find(f"link[@name='{ref}']")
            if sdf_link is not None:
                # Convert sensor element from URDF gazebo wrapper
                sdf_sensor = _copy_element(sensor)
                # Remove pose if present (URDF sensor pose might exist)
                sdf_link.append(sdf_sensor)

        # ── Plugin blocks ────────────────────────────────────────────
        plugin = gazebo.find("plugin")
        if plugin is not None and not ref:
            sdf_plugin = _copy_element(plugin)
            sdf_model.append(sdf_plugin)


def _copy_element(elem: ET.Element) -> ET.Element:
    """Deep-copy an XML element, preserving all attributes and children."""
    return ET.fromstring(ET.tostring(elem))


# ---------------------------------------------------------------------------
# Main converter
# ---------------------------------------------------------------------------


def convert_urdf_to_sdf_xml(urdf_xml: str, model_name: str = ROBOT_NAME) -> str:
    """Parse a URDF XML string and return the equivalent SDF model XML string.

    Handles links, joints, sensors, plugins, and friction from
    <gazebo reference="..."> extensions.

    Returns a pretty-printed SDF document (a single <model> inside <sdf>).
    """
    # Parse URDF
    try:
        urdf_root = ET.fromstring(urdf_xml)
    except ET.ParseError as e:
        raise ValueError(f"Failed to parse URDF XML: {e}") from e

    if urdf_root.tag != "robot":
        raise ValueError(f"Expected <robot> root, got <{urdf_root.tag}>")

    # Build SDF document
    sdf_root = ET.Element("sdf", version="1.8")
    sdf_model = ET.SubElement(sdf_root, "model")
    sdf_model.set("name", model_name)

    # Default model pose
    model_pose = ET.SubElement(sdf_model, "pose")
    model_pose.text = "0 0 0 0 0 0"

    # ── Collect friction and sensor data from URDF <gazebo> extensions ──
    link_frictions: dict[str, float] = {}
    link_sensors: dict[str, list[ET.Element]] = {}

    for gazebo in urdf_root.findall("gazebo"):
        ref = gazebo.get("reference", "")
        if not ref:
            continue

        # Friction
        mu1 = gazebo.find("mu1")
        mu2 = gazebo.find("mu2")
        if mu1 is not None and mu2 is not None and mu1.text and mu2.text:
            # Use mu1 as primary friction
            link_frictions[ref] = float(mu1.text)

        # Sensor storage (will be attached to link later)
        sensor = gazebo.find("sensor")
        if sensor is not None:
            link_sensors.setdefault(ref, []).append(sensor)

    # ── Convert links ───────────────────────────────────────────────
    for link_elem in urdf_root.findall("link"):
        name = link_elem.get("name", "")
        friction = link_frictions.get(name)
        _build_sdf_link(link_elem, sdf_model, link_friction=friction)

    # ── Convert joints ──────────────────────────────────────────────
    for joint_elem in urdf_root.findall("joint"):
        _build_sdf_joint(joint_elem, sdf_model)

    # ── Plugins and sensors ─────────────────────────────────────────
    _build_sdf_plugins(urdf_root, sdf_model, link_sensors)

    # ── Pretty print ────────────────────────────────────────────────
    _indent_xml(sdf_root)
    return ET.tostring(sdf_root, encoding="unicode", xml_declaration=True)


def _indent_xml(elem: ET.Element, level: int = 0) -> None:
    """Add indentation whitespace for pretty-printing."""
    indent = "  "
    # Don't indent children of geometry (they're inline)
    if elem.tag in ("geometry", "box", "cylinder", "sphere",
                    "material", "inertia", "collision", "visual",
                    "pose", "inertial", "mass", "axis", "plane"):
        # Single-element children — keep inline
        for child in elem:
            _indent_xml(child, level)
        return

    i = "\n" + indent * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        last_child = None
        for child in elem:
            _indent_xml(child, level + 1)
            last_child = child
        if last_child is not None and (not last_child.tail or not last_child.tail.strip()):
            last_child.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


# ---------------------------------------------------------------------------
# Model file creation
# ---------------------------------------------------------------------------


def create_model_config(model_dir: str, model_name: str = ROBOT_NAME) -> str:
    """Write model.config and return its path."""
    path = os.path.join(model_dir, "model.config")
    content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Meister Robot Team</name>
    <email>s25089@tokyo.kosen-ac.jp</email>
  </author>
  <description>
    {model_name} — mecanum-wheel robot with 4 mecanum wheels and 2D LiDAR.
    Auto-generated from URDF by spawn_robot.py.
  </description>
</model>
"""
    with open(path, "w") as f:
        f.write(content)
    return path


def write_model_sdf(urdf_xml: str, model_dir: str,
                    model_name: str = ROBOT_NAME) -> str:
    """Convert URDF to SDF, write model.sdf, return its path."""
    sdf_xml = convert_urdf_to_sdf_xml(urdf_xml, model_name)
    path = os.path.join(model_dir, "model.sdf")
    with open(path, "w") as f:
        f.write(sdf_xml)
    return path


# ---------------------------------------------------------------------------
# World file modification
# ---------------------------------------------------------------------------


def create_world_with_robot(
    source_world_path: str,
    output_path: str,
    model_name: str = ROBOT_NAME,
    pose: tuple[float, float, float, float, float, float] = DEFAULT_SPAWN_POSE,
) -> str:
    """Copy a world file and add the robot <include> before </world>.

    The model is referenced via ``model://<model_name>``, which Gazebo
    resolves from ``GZ_SIM_RESOURCE_PATH``.
    """
    with open(source_world_path) as f:
        world_text = f.read()

    pose_str = f"{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}"
    robot_include = f"""
    <!-- Robot inserted by spawn_robot.py -->
    <include>
      <uri>model://{model_name}</uri>
      <pose>{pose_str}</pose>
      <name>{model_name}</name>
    </include>
"""

    # Insert before closing </world>
    modified = world_text.replace("</world>", f"{robot_include}\n  </world>", 1)

    with open(output_path, "w") as f:
        f.write(modified)

    return output_path


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------


class SpawnRobot(Node):
    """ROS 2 node that reads URDF and spawns the robot in Gazebo."""

    def __init__(self):
        super().__init__("spawn_robot")
        self._urdf_received: str | None = None

    # ── URDF reader ─────────────────────────────────────────────────

    def get_robot_description(self, timeout: float = 15.0) -> str | None:
        """Subscribe to /robot_description and return the URDF string.

        Blocks for up to *timeout* seconds waiting for a message.
        """
        self._urdf_received = None
        received_event = threading.Event()

        def _urdf_cb(msg: String) -> None:
            self._urdf_received = msg.data
            received_event.set()

        sub = self.create_subscription(
            String,
            "/robot_description",
            _urdf_cb,
            1,
        )

        self.get_logger().info(
            f"Waiting for /robot_description topic (timeout={timeout}s)..."
        )

        try:
            if received_event.wait(timeout=timeout):
                urdf = self._urdf_received or ""
                self.get_logger().info(
                    f"Received URDF ({len(urdf)} bytes)."
                )
                return urdf

            self.get_logger().error("Timed out waiting for /robot_description.")
            return None
        finally:
            self.destroy_subscription(sub)

    # ── Gazebo readiness check ──────────────────────────────────────

    def wait_for_gazebo(self, timeout: float = 60.0) -> bool:
        """Wait for Gazebo by polling for /clock messages.

        Returns True if Gazebo appears to be running.
        """
        self.get_logger().info(
            f"Waiting for Gazebo (/clock topic, timeout={timeout}s)..."
        )

        clock_received = threading.Event()

        sub = self.create_subscription(
            Clock,
            "/clock",
            lambda msg: clock_received.set(),
            10,
        )

        try:
            ok = clock_received.wait(timeout=timeout)
            if ok:
                self.get_logger().info("Gazebo is running (received /clock).")
            else:
                self.get_logger().warn(
                    "Gazebo readiness check timed out. "
                    "Proceeding anyway..."
                )
            return ok
        finally:
            self.destroy_subscription(sub)

    # ── Gazebo transport spawn (attempt) ────────────────────────────

    def try_gz_spawn(self, model_dir: str) -> bool:
        """Try to spawn the robot via ``gz service`` Gazebo transport.

        This is expected to FAIL since Gazebo transport is broken.
        Returns True on success, False on failure.
        """
        # Ensure GZ_SIM_RESOURCE_PATH includes our model
        models_parent = os.path.dirname(model_dir)
        env = os.environ.copy()
        existing = env.get("GZ_SIM_RESOURCE_PATH", "")
        if models_parent not in existing:
            env["GZ_SIM_RESOURCE_PATH"] = f"{models_parent}:{existing}" if existing else models_parent

        model_sdf = os.path.join(model_dir, "model.sdf")
        if not os.path.isfile(model_sdf):
            self.get_logger().error(f"Model SDF not found: {model_sdf}")
            return False

        # Build spawn command
        # gz service ... /world/<name>/create --reqtype ... --reptype ...
        service = "/world/maze/create"
        req_type = "gz.msgs.EntityFactory"
        rep_type = "gz.msgs.Boolean"

        # Construct request XML
        pose = DEFAULT_SPAWN_POSE
        pose_str = f"{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}"

        # Read SDF content for inline spawn
        with open(model_sdf) as f:
            sdf_content = f.read()

        req_xml = f"""\
sdf: '{sdf_content}'
pose: {{
  position: {{ x: {pose[0]}, y: {pose[1]}, z: {pose[2]} }}
  orientation: {{ x: {pose[3]}, y: {pose[4]}, z: {pose[5]}, w: 1.0 }}
}}
name: '{ROBOT_NAME}'
allow_renaming: false
"""

        cmd = [
            "gz", "service", service,
            "--reqtype", req_type,
            "--reptype", rep_type,
            "--timeout", "5000",
            "--req", req_xml,
        ]

        self.get_logger().info(f"Attempting gz spawn: {' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10, env=env
            )
            self.get_logger().info(
                f"gz service stdout: {result.stdout.strip()}"
            )
            self.get_logger().info(
                f"gz service stderr: {result.stderr.strip()}"
            )
            if result.returncode == 0:
                self.get_logger().info("Robot spawned via gz service!")
                return True
            self.get_logger().warn(
                f"gz service returned code {result.returncode}: "
                f"{result.stderr.strip()}"
            )
        except FileNotFoundError:
            self.get_logger().warn("'gz' command not found in PATH.")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("gz service timed out (transport broken?).")
        except Exception as e:
            self.get_logger().warn(f"gz spawn error: {e}")

        return False

    # ── Instructions printer ────────────────────────────────────────

    @staticmethod
    def print_instructions(source_world: str, output_world: str,
                           model_dir: str) -> None:
        """Print setup instructions for the user."""
        models_parent = os.path.dirname(model_dir)
        print()
        print("=" * 72)
        print("  SPAWN_ROBOT — SETUP INSTRUCTIONS")
        print("=" * 72)
        print()
        print("  Model files created:")
        print(f"    {model_dir}")
        print(f"    {os.path.join(model_dir, 'model.config')}")
        print(f"    {os.path.join(model_dir, 'model.sdf')}")
        print()
        print("  Modified world:")
        print(f"    {output_world}")
        print()
        print("  1. Set the resource path (add to your .bashrc or launch):")
        print()
        print(f'    export GZ_SIM_RESOURCE_PATH=${{GZ_SIM_RESOURCE_PATH}}:"{models_parent}"')
        print()
        print("  2. Launch Gazebo with the modified world:")
        print()
        print(f"    gz sim -r -s {output_world}")
        print()
        print("  3. Or use the ROS 2 launch (which includes ros_gz_bridge, etc.):")
        print()
        print(f'    export GZ_SIM_RESOURCE_PATH=${{GZ_SIM_RESOURCE_PATH}}:"{models_parent}"')
        print(f"    ros2 launch meister_robot meister_slam_nav.launch.py \\")
        print(f"             world_name:=maze_with_robot")
        print()
        print("  NOTE: The ros_gz_sim create node in the launch file will still")
        print("  fail (transport broken), but the robot is already loaded via the")
        print("  world file. You can also remove the create node or let it fail")
        print("  harmlessly.")
        print("=" * 72)
        print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="URDF→SDF converter + Gazebo robot spawner for Meister Robot"
    )
    parser.add_argument(
        "--mode", choices=["pregenerate", "spawn"], default="pregenerate",
        help=(
            "pregenerate: read URDF, create SDF model + world (run before Gazebo). "
            "spawn: try gz service (run after Gazebo)."
        ),
    )
    parser.add_argument(
        "--world", default="maze",
        help="Base world name (default: maze). Creates <world>_with_robot.world.",
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="Output directory for model files (default: models/ in package source).",
    )
    parser.add_argument(
        "--spawn-x", type=float, default=DEFAULT_SPAWN_POSE[0],
    )
    parser.add_argument(
        "--spawn-y", type=float, default=DEFAULT_SPAWN_POSE[1],
    )
    parser.add_argument(
        "--spawn-z", type=float, default=DEFAULT_SPAWN_POSE[2],
    )
    parser.add_argument(
        "--spawn-yaw", type=float, default=DEFAULT_SPAWN_POSE[4],
    )
    parser.add_argument(
        "--name", default=ROBOT_NAME,
        help="Robot model name (default: meister_robot).",
    )

    args = parser.parse_args()

    # ── Resolve paths ───────────────────────────────────────────
    src_root = _get_package_src_root()
    world_source = _resolve_world_source(args.world)

    if args.output_dir:
        model_dir = args.output_dir
    else:
        # Use models/ directory under package source
        model_dir = os.path.join(src_root, "models", args.name)

    world_basename = f"{args.world}_with_robot.world"
    world_output = os.path.join(src_root, "worlds", world_basename)

    spawn_pose = (
        args.spawn_x, args.spawn_y, args.spawn_z,
        0.0, args.spawn_yaw, 0.0,
    )

    # ── Initialize ROS ──────────────────────────────────────────
    rclpy.init(args=sys.argv[1:] if hasattr(sys, 'argv') else [])
    node = SpawnRobot()

    if args.mode == "pregenerate":
        _run_pregenerate(node, model_dir, world_source, world_output,
                         spawn_pose, args.name)
    elif args.mode == "spawn":
        _run_spawn(node, model_dir, args.name)

    rclpy.shutdown()


def _run_pregenerate(node, model_dir, world_source, world_output,
                     spawn_pose, model_name):
    """Pregenerate mode: read URDF, create model + world."""
    node.get_logger().info(f"Pregenerate mode — model dir: {model_dir}")

    # 1) Read robot description
    urdf_xml = node.get_robot_description(timeout=15.0)
    if urdf_xml is None:
        node.get_logger().error(
            "Could not read /robot_description. "
            "Make sure robot_state_publisher is running."
        )
        # Fall back to reading from known file
        urdf_path = "/tmp/meister_robot.urdf"
        if os.path.isfile(urdf_path):
            node.get_logger().info(f"Fallback: reading URDF from {urdf_path}")
            with open(urdf_path) as f:
                urdf_xml = f.read()
        else:
            node.get_logger().error("No URDF source available. Exiting.")
            sys.exit(1)

    # 2) Create model directory
    os.makedirs(model_dir, exist_ok=True)

    # 3) Write model.config
    config_path = create_model_config(model_dir, model_name)
    node.get_logger().info(f"Created {config_path}")

    # 4) Convert URDF → SDF and write model.sdf
    sdf_path = write_model_sdf(urdf_xml, model_dir, model_name)
    node.get_logger().info(f"Created {sdf_path}")

    # 5) Create modified world
    if world_source and os.path.isfile(world_source):
        world_path = create_world_with_robot(
            world_source, world_output, model_name, spawn_pose,
        )
        node.get_logger().info(f"Created world: {world_path}")
    else:
        node.get_logger().warn(
            f"Source world '{world_source}' not found. "
            "Skipping world modification."
        )
        world_path = None

    # 6) Print instructions
    node.print_instructions(world_source or "", world_output, model_dir)

    node.get_logger().info("Done.")


def _run_spawn(node, model_dir, model_name):
    """Spawn mode: try to spawn the robot in a running Gazebo."""
    # Wait for Gazebo
    node.wait_for_gazebo(timeout=30.0)

    # Ensure model exists
    model_sdf = os.path.join(model_dir, "model.sdf")
    if not os.path.isfile(model_sdf):
        node.get_logger().info(
            "Model SDF not found — attempting to read URDF and convert..."
        )
        urdf_xml = node.get_robot_description(timeout=10.0)
        if urdf_xml is None:
            node.get_logger().error("No URDF available. Exiting.")
            sys.exit(1)
        os.makedirs(model_dir, exist_ok=True)
        create_model_config(model_dir, model_name)
        write_model_sdf(urdf_xml, model_dir, model_name)

    # Try gz spawn
    ok = node.try_gz_spawn(model_dir)
    if not ok:
        print()
        print("╔═══════════════════════════════════════════════════════════════╗")
        print("║  Gazebo transport spawn FAILED (expected — known issue).    ║")
        print("║                                                             ║")
        print("║  Workaround: use pregenerate mode instead:                  ║")
        print("║    ros2 run meister_robot spawn_robot.py --mode pregenerate ║")
        print("║                                                             ║")
        print("║  This creates a world file with the robot embedded.         ║")
        print("╚═══════════════════════════════════════════════════════════════╝")
        print()

    node.get_logger().info("Done.")


if __name__ == "__main__":
    main()
