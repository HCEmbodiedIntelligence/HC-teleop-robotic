"""Validated immutable robot packages and compilation for the C++ runtime."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
import threading
import uuid
import xml.etree.ElementTree as ET
import zipfile

import yaml


class ConfigurationError(ValueError):
    pass


def atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".pending-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def within(root: Path, relative: str, *, base: Path | None = None) -> Path:
    root = root.resolve()
    path = ((base or root) / relative).resolve()
    if not path.is_relative_to(root):
        raise ConfigurationError(f"Resource escapes package: {relative}")
    if not path.is_file():
        raise ConfigurationError(f"Missing resource: {relative}")
    return path


def read_yaml(path: Path) -> dict:
    if path.stat().st_size > 4 * 1024 * 1024:
        raise ConfigurationError("Configuration exceeds 4 MiB")
    try:
        value = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"Invalid YAML: {path.name}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigurationError(f"Expected mapping: {path.name}")
    return value


def finite(value, label: str) -> float:
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise ConfigurationError(f"Invalid number: {label}") from exc
    if not math.isfinite(result):
        raise ConfigurationError(f"Non-finite number: {label}")
    return result


def package_hash(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ConfigurationError("Symbolic links are not permitted in packages")
        if not path.is_file() or path.suffix == ".zip" or ".before-" in path.name:
            continue
        digest.update(path.relative_to(root).as_posix().encode() + b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def _compile_package(root: Path, robot_id: str | None = None) -> dict:
    root = root.resolve()
    profile = (
        read_yaml(within(root, "profile.yaml"))
        if (root / "profile.yaml").exists()
        else {}
    )
    teleop = read_yaml(within(root, "arm_teleop.yaml"))
    robot = teleop.get("robot", {})
    if not isinstance(robot, dict):
        raise ConfigurationError("robot must be a mapping")
    urdf = within(root, robot.get("urdf_path", profile.get("urdf", "robot.urdf")))
    text = urdf.read_text()
    if len(text) > 16 * 1024 * 1024 or "<!DOCTYPE" in text or "<!ENTITY" in text:
        raise ConfigurationError("URDF size or XML entity declaration is not allowed")
    try:
        model = ET.fromstring(text)
    except ET.ParseError as exc:
        raise ConfigurationError(f"Invalid URDF XML: {exc}") from exc
    if model.tag != "robot":
        raise ConfigurationError("URDF root must be robot")
    links = [node.attrib.get("name", "") for node in model.findall("link")]
    if (
        not links
        or any(not name.strip() for name in links)
        or len(links) != len(set(links))
    ):
        raise ConfigurationError("URDF links must be nonempty and unique")
    joints = {}
    parent_for_child = {}
    for node in model.findall("joint"):
        name = node.attrib.get("name", "")
        if not name or name in joints:
            raise ConfigurationError(f"Duplicate or empty joint: {name}")
        parent, child = node.find("parent"), node.find("child")
        if parent is None or child is None:
            raise ConfigurationError(f"Missing joint links: {name}")
        p, c = parent.attrib.get("link"), child.attrib.get("link")
        if p not in links or c not in links or c in parent_for_child:
            raise ConfigurationError(f"Invalid parent/child: {name}")
        parent_for_child[c] = p
        joints[name] = node
        for vector_node, field in (
            (node.find("axis"), "xyz"),
            (node.find("origin"), "xyz"),
            (node.find("origin"), "rpy"),
        ):
            if vector_node is not None and field in vector_node.attrib:
                values = [finite(v, name) for v in vector_node.attrib[field].split()]
                if len(values) != 3 or (
                    vector_node.tag == "axis"
                    and node.attrib.get("type") != "fixed"
                    and sum(v * v for v in values) < 1e-12
                ):
                    raise ConfigurationError(f"Invalid axis/origin: {name}")
    roots = set(links) - set(parent_for_child)
    if len(roots) != 1:
        raise ConfigurationError("URDF must have exactly one root")
    for child in links:
        seen = set()
        while child in parent_for_child:
            if child in seen:
                raise ConfigurationError("URDF contains a cycle")
            seen.add(child)
            child = parent_for_child[child]
    for mesh in model.findall(".//mesh"):
        resource = mesh.attrib.get("filename", "")
        if resource.startswith("package://"):
            resource = resource[len("package://") :].split("/", 1)[-1]
            within(root, resource)
        else:
            within(root, resource, base=urdf.parent)
        if "scale" in mesh.attrib:
            scale = [finite(v, "mesh scale") for v in mesh.attrib["scale"].split()]
            if len(scale) != 3 or any(abs(value) < 1e-12 for value in scale):
                raise ConfigurationError("Invalid mesh scale")
    groups = {}
    if not isinstance(teleop.get("arms", {}), dict):
        raise ConfigurationError("arms must be a mapping")
    for side in ("right", "left"):
        arm = teleop.get("arms", {}).get(side)
        if arm is not None:
            if not isinstance(arm, dict):
                raise ConfigurationError(f"{side} arm must be a mapping")
            for key in ("base_link", "ee_link"):
                if arm.get(key) not in links:
                    raise ConfigurationError(f"Unknown {side} {key}")
            arm_names = arm.get("joint_names")
            if (
                not isinstance(arm_names, list)
                or not arm_names
                or any(not isinstance(name, str) or not name for name in arm_names)
            ):
                raise ConfigurationError(
                    f"{side} joint_names must be a nonempty list of names"
                )
            groups[f"{side}_arm"] = list(arm_names)
            if len(groups[f"{side}_arm"]) != len(set(groups[f"{side}_arm"])):
                raise ConfigurationError(f"Duplicate joint in {side} arm")
            if arm.get("generic_task_base_link", arm["base_link"]) not in links:
                raise ConfigurationError(f"Unknown {side} task base")
    body = teleop.get("body", {})
    if body.get("enabled") and body.get("waist_joint_names"):
        groups["body"] = list(body["waist_joint_names"])
    if body.get("neck_tracking_enabled"):
        groups.setdefault("body", []).extend(body.get("neck_joint_names", []))
    names = [name for values in groups.values() for name in values]
    if not names or len(names) > 64 or len(names) != len(set(names)):
        raise ConfigurationError(
            "Joint groups must be unique, nonempty, and at most 64 joints"
        )
    initial = robot.get("initial_joints", {})
    solver_path = root / "controller_v23.yml"
    solver = read_yaml(solver_path) if solver_path.exists() else {}
    free = solver.get("model", {}).get("free_joints", [])
    if len(free) != len(set(free)) or any(name not in joints for name in free):
        raise ConfigurationError(
            "Solver free_joints contain unknown or duplicate joints"
        )
    velocities = solver.get("limit", {}).get("velocity", [])
    if velocities and len(free) != len(velocities):
        raise ConfigurationError("Solver velocity and free_joints lengths differ")
    velocity_map = {
        name: finite(value, "velocity")
        if joints[name].attrib.get("type") == "prismatic"
        else math.radians(finite(value, "velocity"))
        for name, value in zip(free, velocities)
    }
    acceleration = finite(
        solver.get("limit", {}).get("acceleration_rad_s2", 2.0), "acceleration"
    )
    if acceleration <= 0:
        raise ConfigurationError("Acceleration must be positive")
    lower, upper, velocity, home, units, model_velocity = [], [], [], [], [], []
    for name in names:
        node = joints.get(name)
        if (
            node is None
            or node.attrib.get("type") not in ("revolute", "continuous", "prismatic")
            or node.find("mimic") is not None
        ):
            raise ConfigurationError(f"Not an independently actuated joint: {name}")
        limit = node.find("limit")
        limits = limit.attrib if limit is not None else {}
        lo = finite(limits.get("lower", -math.pi), f"{name} lower")
        hi = finite(limits.get("upper", math.pi), f"{name} upper")
        speed = min(
            velocity_map.get(name, 0.6), finite(limits.get("velocity", 1.0), name)
        )
        zero = finite(initial.get(name, 0), f"{name} home")
        if lo >= hi or speed <= 0 or not lo <= zero <= hi:
            raise ConfigurationError(f"Invalid limits or home pose: {name}")
        lower.append(lo)
        upper.append(hi)
        velocity.append(speed)
        home.append(zero)
        model_velocity.append(finite(limits.get("velocity", 1.0), name))
        units.append("m" if node.attrib["type"] == "prismatic" else "rad")
    control = teleop.get("control", {})
    mapping = control.get("axis_mapping", [[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
    if len(mapping) != 3 or any(len(row) != 3 for row in mapping):
        raise ConfigurationError("axis_mapping must be 3x3")
    for i in range(3):
        for j in range(3):
            dot = sum(
                finite(mapping[i][k], "axis_mapping")
                * finite(mapping[j][k], "axis_mapping")
                for k in range(3)
            )
            if abs(dot - (1 if i == j else 0)) > 1e-5:
                raise ConfigurationError("axis_mapping must be orthonormal")
    for key in ("position_scale", "target_filter_alpha"):
        value = finite(control.get(key, 1), key)
        if value <= 0 or (key.endswith("alpha") and value > 1):
            raise ConfigurationError(f"Invalid {key}")
    for side in ("left", "right"):
        hand = teleop.get("grippers", {}).get(side, {})
        if hand.get("sim_joint") and hand["sim_joint"] not in joints:
            raise ConfigurationError(f"Unknown {side} gripper simulation joint")
        fingers = hand.get("finger_joint_names", [])
        if fingers:
            if len(fingers) != len(set(fingers)) or any(
                len(hand.get(key, [])) != len(fingers)
                for key in ("finger_open", "finger_closed")
            ):
                raise ConfigurationError(f"Invalid {side} finger calibration lengths")
            for value in hand["finger_open"] + hand["finger_closed"]:
                finite(value, "finger calibration")
    middleware = (
        read_yaml(root / "middleware.yaml")
        if (root / "middleware.yaml").exists()
        else {}
    )
    return {
        "schema": "hc-platform-config/v1",
        "robot_id": robot_id or profile.get("id", root.name),
        "config_hash": package_hash(root),
        "package_root": str(root),
        "urdf": str(urdf),
        "urdf_path": str(urdf),
        "joint_names": names,
        "joint_units": units,
        "command_interface_version": "hc-control-ipc/v1",
        "lower": lower,
        "upper": upper,
        "velocity": velocity,
        "model_velocity_limits": model_velocity,
        "acceleration": [acceleration] * len(names),
        "home": home,
        "groups": groups,
        "control_hz": 200,
        "feedback_timeout_ms": 200,
        "command_timeout_ms": 150,
        "lease_timeout_ms": 1000,
        "handover_timeout_ms": 1000,
        "home_velocity": control.get("home_joint_velocity", 0.5),
        "home_acceleration": control.get("home_joint_acceleration", 1.0),
        "home_tolerance": control.get("home_tolerance", 0.02),
        "home_joint_names": control.get("home_joint_names", names),
        "teleop": teleop,
        "arms": teleop.get("arms", {}),
        "grippers": teleop.get("grippers", {}),
        "initial_positions": dict(zip(names, home)),
        "hardware": {
            "adapter": "x1"
            if profile.get("id", root.name).lower() == "x1"
            else "unconfigured",
            "contract": "legacy-model-topic",
            "allowed_groups": ["right_arm", "right_gripper"],
            "feedback_timeout_s": 0.2,
            "ready_timeout_s": 0.5,
        },
        "control_backend": control.get("backend", "v23"),
        "sim": {
            "base_position": robot.get("base_position", [0, 0, 0]),
            "base_orientation": robot.get("base_orientation", [0, 0, 0, 1]),
        },
        "allowed_groups": list(groups),
        "enable_grippers": bool(teleop.get("grippers")),
        "enable_base": bool(body.get("enabled") and body.get("base_command_topic")),
        "base_velocity_limit": [0.1, 0.1, 0.2],
        "camera": middleware.get("camera", {}),
        "topics": middleware.get("ros", {}).get("subscriptions", []),
        "hardware_contract": "hc_x1_standard_v1",
        "verification": {"simulation": "not_verified", "hardware": "not_verified"},
    }


def compile_package(root: Path, robot_id: str | None = None) -> dict:
    try:
        return _compile_package(root, robot_id)
    except (AttributeError, KeyError, TypeError, OverflowError) as exc:
        raise ConfigurationError(f"Malformed robot configuration: {exc}") from exc


class PackageRegistry:
    def __init__(self, builtin_root: Path, state_root: Path):
        self.builtin_root = builtin_root.resolve()
        self.root = state_root / "packages"
        self.root.mkdir(parents=True, exist_ok=True)
        self.active_path = state_root / "active_config.json"
        self._cache = {}
        self._cache_lock = threading.Lock()

    @contextmanager
    def _lock(self):
        # Separate open descriptions make flock serialize threads and other
        # registry instances/processes as well.
        with (self.root / ".registry.lock").open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    @staticmethod
    def validate_id(robot_id: str):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", robot_id):
            raise ConfigurationError("Invalid robot ID")

    def resolve(self, robot_id: str) -> Path:
        self.validate_id(robot_id)
        for root in (self.root, self.builtin_root):
            path = (root / robot_id).resolve()
            if path.is_relative_to(root) and path.is_dir():
                return path
        raise ConfigurationError(f"Unknown robot: {robot_id}")

    def compile(self, robot_id: str, *, refresh: bool = False) -> dict:
        with self._cache_lock:
            if refresh or robot_id not in self._cache:
                self._cache[robot_id] = compile_package(self.resolve(robot_id), robot_id)
            return json.loads(json.dumps(self._cache[robot_id]))

    def list(self) -> list[dict]:
        identifiers = {
            p.name
            for root in (self.builtin_root, self.root)
            for p in root.iterdir()
            if p.is_dir() and not p.name.startswith(".")
        }
        result = []
        for identifier in sorted(identifiers):
            try:
                config = self.compile(identifier)
                result.append(
                    {
                        "id": identifier,
                        "valid": True,
                        "config_hash": config["config_hash"],
                        "groups": config["groups"],
                        "verification": config["verification"],
                        "builtin": not (self.root / identifier).exists(),
                    }
                )
            except (ValueError, KeyError, OSError, TypeError) as exc:
                result.append({"id": identifier, "valid": False, "errors": [str(exc)]})
        return result

    def import_zip(self, archive: Path, robot_id: str) -> dict:
        self.validate_id(robot_id)
        # Never use shutil.move's existing-directory nesting semantics for
        # immutable installations.
        with self._lock():
            try:
                return self._import_zip_locked(archive, robot_id)
            except zipfile.BadZipFile as exc:
                raise ConfigurationError("Invalid robot ZIP archive") from exc

    def _import_zip_locked(self, archive: Path, robot_id: str) -> dict:
        if (self.root / robot_id).exists() or (self.builtin_root / robot_id).exists():
            raise ConfigurationError(
                "Robot ID exists; import as a new immutable version"
            )
        with tempfile.TemporaryDirectory(prefix=".import-", dir=self.root) as temporary:
            staging = Path(temporary)
            with zipfile.ZipFile(archive) as package:
                items = package.infolist()
                if len(items) > 10000 or sum(p.file_size for p in items) > 1024**3:
                    raise ConfigurationError("Package exceeds extraction budget")
                for item in items:
                    destination = (staging / item.filename).resolve()
                    mode = item.external_attr >> 16
                    if not destination.is_relative_to(staging) or stat.S_ISLNK(mode):
                        raise ConfigurationError("Unsafe archive path or symlink")
                    if item.is_dir():
                        destination.mkdir(parents=True, exist_ok=True)
                    else:
                        destination.parent.mkdir(parents=True, exist_ok=True)
                        with (
                            package.open(item) as source,
                            destination.open("wb") as target,
                        ):
                            shutil.copyfileobj(source, target, 1024 * 1024)
            candidates = (
                [staging]
                if (staging / "arm_teleop.yaml").exists()
                else [p.parent for p in staging.rglob("arm_teleop.yaml")]
            )
            if len(candidates) != 1:
                raise ConfigurationError("Package must contain one arm_teleop.yaml")
            compile_package(candidates[0], robot_id)
            candidates[0].rename(self.root / robot_id)
        self._cache.pop(robot_id, None)
        return self.compile(robot_id)

    def export_zip(self, robot_id: str, output: Path) -> Path:
        root = self.resolve(robot_id)
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(root.rglob("*")):
                if path.is_file() and not path.is_symlink() and path.suffix != ".zip":
                    archive.write(path, path.relative_to(root))
        return output

    def delete(self, robot_id: str, referenced_hashes=()) -> None:
        with self._lock():
            path = self.resolve(robot_id)
            config = self.compile(robot_id)
            active = (
                json.loads(self.active_path.read_text())
                if self.active_path.exists()
                else {}
            )
            if (
                active.get("robot_id") == robot_id
                or config["config_hash"] in referenced_hashes
            ):
                raise ConfigurationError("Package is active or referenced")
            if not path.is_relative_to(self.root):
                raise ConfigurationError(
                    "Bundled package cannot be deleted; imported versions are removable"
                )
            tombstone = self.root / f".deleted-{robot_id}-{uuid.uuid4().hex[:8]}"
            path.rename(tombstone)
            shutil.rmtree(tombstone)
            with self._cache_lock:
                self._cache.pop(robot_id, None)
