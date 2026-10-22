from __future__ import annotations

import math
from itertools import combinations
from typing import Mapping, Sequence


Aabb = tuple[Sequence[float], Sequence[float]]


def aabb_distance(first: Aabb, second: Aabb) -> float:
    """Return the Euclidean separation between two axis-aligned boxes."""
    squared = 0.0
    for axis in range(3):
        gap = max(
            float(second[0][axis]) - float(first[1][axis]),
            float(first[0][axis]) - float(second[1][axis]),
            0.0,
        )
        squared += gap * gap
    return math.sqrt(squared)


def descendants(parents: Mapping[int, int], root: int) -> set[int]:
    result: set[int] = set()
    for link in parents:
        current = link
        while current >= 0:
            if current == root:
                result.add(link)
                break
            current = parents.get(current, -1)
    result.discard(root)
    return result


def tree_distance(parents: Mapping[int, int], first: int, second: int) -> int:
    first_ancestors: dict[int, int] = {}
    current = first
    distance = 0
    while current >= 0:
        first_ancestors[current] = distance
        current = parents.get(current, -1)
        distance += 1

    current = second
    distance = 0
    while current >= 0:
        if current in first_ancestors:
            return first_ancestors[current] + distance
        current = parents.get(current, -1)
        distance += 1
    return 1_000_000


def build_protected_pairs(
    collision_links: set[int],
    parents: Mapping[int, int],
    right_arm_root: int,
    left_arm_root: int,
    ignore_graph_distance: int = 2,
    *,
    check_intra_arm: bool = True,
    check_inter_arm: bool = True,
    check_arm_body: bool = True,
) -> tuple[set[int], set[int], list[tuple[int, int]]]:
    """Build arm/body, inter-arm and non-adjacent intra-arm collision pairs."""
    right_links = descendants(parents, right_arm_root) & collision_links
    left_links = descendants(parents, left_arm_root) & collision_links
    moving_links = right_links | left_links
    pairs: list[tuple[int, int]] = []

    for first, second in combinations(sorted(collision_links), 2):
        if first not in moving_links and second not in moving_links:
            continue
        same_arm = (
            first in right_links and second in right_links
        ) or (
            first in left_links and second in left_links
        )
        inter_arm = (
            first in right_links and second in left_links
        ) or (
            first in left_links and second in right_links
        )
        arm_and_body = (first in moving_links) != (second in moving_links)
        if same_arm and not check_intra_arm:
            continue
        if inter_arm and not check_inter_arm:
            continue
        if arm_and_body and not check_arm_body:
            continue
        if (same_arm or arm_and_body) and tree_distance(
            parents, first, second
        ) <= ignore_graph_distance:
            continue
        pairs.append((first, second))

    return right_links, left_links, pairs


def calibrated_clearance(
    requested: float,
    home_distance: float | None,
    tolerance: float,
) -> float:
    """Preserve a valid close-clearance home pose without ignoring the pair."""
    if home_distance is None or home_distance >= requested:
        return requested
    if home_distance <= 0.0:
        return 0.0
    return max(0.0, min(requested, home_distance - tolerance))


def collision_deficit(collision: tuple[str, str, float, float]) -> float:
    """Return how far a collision result lies inside its required clearance."""
    return float(collision[3]) - float(collision[2])


def improves_collision_clearance(
    current: tuple[str, str, float, float] | None,
    candidate: tuple[str, str, float, float],
    epsilon: float,
) -> bool:
    """Allow an unsafe pose only when it monotonically moves toward safety."""
    if current is None:
        return False
    return collision_deficit(candidate) <= collision_deficit(current) - epsilon


class ModelCollisionGuard:
    """Profile-calibrated collision check shared by every selected source.

    Geometry is evaluated in model SI units after public conversion. A subset
    command is overlaid on current measured joints, never on a stale command.
    The existing clearance and monotonic escape policies are used unchanged.
    """

    def __init__(self, api):
        import pybullet as bullet

        self.bullet = bullet
        self.client = bullet.connect(bullet.DIRECT)
        if self.client < 0:
            raise RuntimeError("cannot create shared collision model")
        self.robot = bullet.loadURDF(str(api.urdf_path), useFixedBase=True,
                                    physicsClientId=self.client)
        self.control = api.teleop["control"]
        self.minimum = float(self.control.get("collision_min_distance", .02))
        self.epsilon = float(self.control.get("collision_escape_epsilon", 1e-5))
        self.joints, self.links, parents = {}, {}, {}
        self.names = {-1: bullet.getBodyInfo(self.robot, physicsClientId=self.client)[0].decode()}
        self.links[self.names[-1]] = -1
        for index in range(bullet.getNumJoints(self.robot, physicsClientId=self.client)):
            info = bullet.getJointInfo(self.robot, index, physicsClientId=self.client)
            if info[2] != bullet.JOINT_FIXED:
                self.joints[info[1].decode()] = index
            self.names[index] = info[12].decode()
            self.links[self.names[index]] = index
            parents[index] = int(info[16])
        self.home = {n: float(api.teleop["robot"]["initial_joints"].get(n, 0.)) for n in self.joints}
        self.required = {n for group in api.groups.values() for n in group}
        self._apply(self.home)
        shapes = {i for i in self.names if bullet.getCollisionShapeData(self.robot, i, physicsClientId=self.client)}
        roots = [self.links[api.teleop["arms"][side]["generic_task_base_link"]] for side in ("right", "left")]
        _, _, self.pairs = build_protected_pairs(shapes, parents, *roots,
            int(self.control.get("collision_ignore_graph_distance", 2)),
            check_intra_arm=bool(self.control.get("collision_check_intra_arm", False)),
            check_inter_arm=bool(self.control.get("collision_check_inter_arm", True)),
            check_arm_body=bool(self.control.get("collision_check_arm_body", True)))
        self.clearances = {}
        for pair in self.pairs:
            distance = self._distance(pair, self.minimum)
            self.clearances[pair] = calibrated_clearance(self.minimum, distance,
                float(self.control.get("collision_home_tolerance", .008)))

    def _apply(self, values):
        for name, value in values.items():
            if name in self.joints:
                self.bullet.resetJointState(self.robot, self.joints[name], float(value), physicsClientId=self.client)

    def _distance(self, pair, clearance):
        boxes = [self.bullet.getAABB(self.robot, i, physicsClientId=self.client) for i in pair]
        if aabb_distance(*boxes) > clearance:
            return None
        points = self.bullet.getClosestPoints(self.robot, self.robot, clearance,
            linkIndexA=pair[0], linkIndexB=pair[1], physicsClientId=self.client)
        return min(float(p[8]) for p in points) if points else None

    def _collision(self, values):
        self._apply(values)
        worst = None
        for pair, clearance in self.clearances.items():
            distance = self._distance(pair, clearance)
            if distance is not None and distance < clearance:
                item = (self.names[pair[0]], self.names[pair[1]], distance, clearance)
                if worst is None or collision_deficit(item) > collision_deficit(worst):
                    worst = item
        return worst

    def check(self, measured, command):
        if not self.required <= measured.keys():
            raise ValueError("collision check requires complete arm/body feedback")
        state = {**self.home, **measured}
        current = self._collision(state)
        candidate = self._collision({**state, **command})
        allowed = candidate is None or improves_collision_clearance(current, candidate, self.epsilon)
        return allowed, candidate, current is not None

    def close(self):
        if self.client >= 0:
            self.bullet.disconnect(self.client)
            self.client = -1
