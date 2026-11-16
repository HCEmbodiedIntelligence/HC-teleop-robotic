"""Optional bounded self-collision path checks in an isolated PyBullet world.

This validates the configured URDF geometry, not unmodelled surroundings or
vendor-driver motion. A timeout, missing dependency or incomplete configuration
never turns into a successful collision result.
"""

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class CollisionResult:
    safe: bool
    reason: str
    minimum_distance: float
    samples: int
    checked_pairs: int
    elapsed_ms: float
    pair: tuple = ()


class CollisionChecker:
    def __init__(self, model, config):
        import pybullet as bullet

        self.bullet = bullet
        self.model = model
        self.config = config
        self.options = config.get("teleop", {}).get("control", {})
        self.margin = float(self.options.get("collision_min_distance", 0.02))
        self.step = float(self.options.get("collision_path_step_rad", 0.03))
        if (
            not math.isfinite(self.margin)
            or self.margin < 0
            or not math.isfinite(self.step)
            or self.step <= 0
        ):
            raise ValueError("invalid collision margin/path step")
        self.client = bullet.connect(bullet.DIRECT)
        if self.client < 0:
            raise RuntimeError("collision checker PyBullet connection failed")
        try:
            flags = getattr(bullet, "URDF_IGNORE_VISUAL_SHAPES", 0)
            self.robot = bullet.loadURDF(
                str(model.path),
                useFixedBase=True,
                flags=flags,
                physicsClientId=self.client,
            )
            self.indices = {}
            links = {model.root: -1}
            for index in range(
                bullet.getNumJoints(self.robot, physicsClientId=self.client)
            ):
                info = bullet.getJointInfo(
                    self.robot, index, physicsClientId=self.client
                )
                self.indices[info[1].decode()] = index
                links[info[12].decode()] = index
            self.link_indices = links
            collision_links = [
                name
                for name, index in links.items()
                if bullet.getCollisionShapeData(
                    self.robot, index, physicsClientId=self.client
                )
            ]
            arms = config.get("arms", config.get("teleop", {}).get("arms", {}))
            self.sides = {}
            for name in collision_links:
                ancestors = set(model.chain(name))
                self.sides[name] = next(
                    (
                        side
                        for side, arm in arms.items()
                        if ancestors.intersection(arm["joint_names"])
                    ),
                    None,
                )
            ignored = {
                frozenset(pair)
                for pair in self.options.get("collision_ignore_pairs", [])
            }
            if any(
                len(pair) != 2 or not pair.issubset(set(model.links))
                for pair in ignored
            ):
                raise ValueError("invalid collision ignored link pair")
            self.pairs = []
            distance = int(self.options.get("collision_ignore_graph_distance", 2))
            for i, a in enumerate(collision_links):
                for b in collision_links[i + 1 :]:
                    if frozenset((a, b)) in ignored or self._distance(a, b) <= distance:
                        continue
                    sa, sb = self.sides[a], self.sides[b]
                    if sa and sb:
                        enabled = (
                            self.options.get("collision_check_intra_arm", False)
                            if sa == sb
                            else self.options.get("collision_check_inter_arm", True)
                        )
                    elif sa or sb:
                        enabled = self.options.get("collision_check_arm_body", True)
                    else:
                        enabled = self.options.get("collision_check_body_body", False)
                    if enabled:
                        self.pairs.append((a, b))
            if not self.pairs:
                raise ValueError(
                    "collision enabled but no geometric pairs are configured"
                )
            self.required_names = set(config.get("joint_names", []))
        except BaseException:
            self.close()
            raise

    def _distance(self, a, b):
        def ancestors(link):
            result = {link: 0}
            count = 0
            while link in self.model.parents:
                link = self.model.joints[self.model.parents[link]].parent
                count += 1
                result[link] = count
            return result

        aa, bb = ancestors(a), ancestors(b)
        return min(aa[name] + bb[name] for name in aa.keys() & bb.keys())

    def _reset(self, positions):
        for name, index in self.indices.items():
            joint = self.model.joints[name]
            if joint.kind != "fixed":
                self.bullet.resetJointState(
                    self.robot,
                    index,
                    self.model._value(name, positions, set()),
                    physicsClientId=self.client,
                )
        self.bullet.performCollisionDetection(physicsClientId=self.client)

    def _separation(self, pair):
        points = self.bullet.getClosestPoints(
            self.robot,
            self.robot,
            distance=self.margin + 0.05,
            linkIndexA=self.link_indices[pair[0]],
            linkIndexB=self.link_indices[pair[1]],
            physicsClientId=self.client,
        )
        return min((float(point[8]) for point in points), default=self.margin + 0.05)

    def check_path(self, current, target, deadline_ns):
        started = time.monotonic_ns()

        def result(safe, reason, minimum=math.inf, samples=0, pairs=0, pair=()):
            # JSON statuses cannot contain Infinity.
            return CollisionResult(
                safe,
                reason,
                float(minimum if math.isfinite(minimum) else self.margin + 0.05),
                samples,
                pairs,
                (time.monotonic_ns() - started) / 1e6,
                pair,
            )

        if not self.required_names.issubset(current):
            return result(False, "COLLISION_FEEDBACK_INCOMPLETE")
        if any(name not in self.model.joints for name in target) or not all(
            math.isfinite(float(value))
            for value in list(current.values()) + list(target.values())
        ):
            return result(False, "COLLISION_INVALID_STATE")
        goal = dict(current)
        goal.update(target)
        changed = [
            name
            for name in goal
            if abs(goal[name] - current.get(name, goal[name])) > 1e-12
        ]
        samples = max(
            1,
            math.ceil(
                max((abs(goal[name] - current[name]) for name in changed), default=0)
                / self.step
            ),
        )
        if samples > 64:
            return result(False, "COLLISION_PATH_TOO_LONG", samples=samples)
        if time.monotonic_ns() >= deadline_ns:
            return result(False, "COLLISION_DEADLINE")
        self._reset(current)
        baseline = {}
        checked = 0
        minimum = math.inf
        for pair in self.pairs:
            if time.monotonic_ns() >= deadline_ns:
                return result(False, "COLLISION_DEADLINE", minimum, 0, checked)
            baseline[pair] = self._separation(pair)
            checked += 1
        escape = float(self.options.get("collision_escape_epsilon", 0))
        for sample in range(1, samples + 1):
            if time.monotonic_ns() >= deadline_ns:
                return result(False, "COLLISION_DEADLINE", minimum, sample - 1, checked)
            alpha = sample / samples
            positions = dict(current)
            positions.update(
                {
                    name: current[name] + alpha * (goal[name] - current[name])
                    for name in changed
                }
            )
            self._reset(positions)
            for pair in self.pairs:
                if time.monotonic_ns() >= deadline_ns:
                    return result(False, "COLLISION_DEADLINE", minimum, sample, checked)
                separation = self._separation(pair)
                checked += 1
                minimum = min(minimum, separation)
                initial = baseline[pair]
                if initial >= self.margin:
                    blocked = separation < self.margin - 1e-7
                elif escape > 0:
                    blocked = separation < initial - 1e-7 or (
                        sample == samples and separation < initial + escape
                    )
                else:
                    blocked = separation < self.margin - 1e-7
                if blocked:
                    return result(
                        False, "COLLISION_BLOCKED", minimum, sample, checked, pair
                    )
        return result(True, "CLEAR", minimum, samples, checked)

    def close(self):
        if getattr(self, "client", None) is not None:
            self.bullet.disconnect(physicsClientId=self.client)
            self.client = None
