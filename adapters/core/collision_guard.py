"""Self-collision clearance guard and clutch-release latch for arm teleop."""

from __future__ import annotations

from contextlib import nullcontext
import math
import os
from typing import Any

import pybullet as bullet
from std_msgs.msg import Bool

from .collision_safety import (
    _native_collision,
    aabb_distance,
    build_protected_pairs,
    calibrated_clearance,
    improves_collision_clearance,
)


def initialize_collision_safety(node: Any) -> None:
    collision_links = {
        index
        for index in set(node.parent_by_link) | {-1}
        if bullet.getCollisionShapeData(
            node.robot_id, index, physicsClientId=node.physics_client
        )
    }
    right_root = node.generic_task_base_indices["right"]
    left_root = node.generic_task_base_indices["left"]
    _, _, node.collision_pairs = build_protected_pairs(
        collision_links,
        node.parent_by_link,
        right_root,
        left_root,
        int(node.control.get("collision_ignore_graph_distance", 2)),
        check_intra_arm=node.collision_check_intra_arm,
        check_inter_arm=node.collision_check_inter_arm,
        check_arm_body=node.collision_check_arm_body,
    )
    node.collision_links = {
        index for pair in node.collision_pairs for index in pair
    }
    node._cpp_collision_guard = None
    urdf_path = getattr(node, "urdf_path", None)
    use_native = (
        _native_collision is not None
        and urdf_path is not None
        and os.environ.get("HC_NATIVE_EXT", "1") != "0"
    )
    adjusted = 0
    if use_native:
        try:
            pair_names = [
                (node.link_name_by_index[a], node.link_name_by_index[b])
                for a, b in node.collision_pairs
            ]
            guard = _native_collision.CppCollisionGuard(
                str(urdf_path),
                dict(getattr(node, "initial_joints", {})),
                pair_names,
                float(node.collision_min_distance),
                float(node.collision_home_tolerance),
                float(node.collision_escape_epsilon),
            )
            node._cpp_collision_guard = guard
            for name_a, name_b, clearance in guard.pair_clearances():
                pair = (node.link_by_name[name_a], node.link_by_name[name_b])
                node.collision_clearances[pair] = float(clearance)
                if clearance < node.collision_min_distance:
                    adjusted += 1
        except Exception:
            node._cpp_collision_guard = None

    if node._cpp_collision_guard is None:
        aabbs = {
            index: bullet.getAABB(
                node.robot_id, index, physicsClientId=node.physics_client
            )
            for index in node.collision_links
        }
        adjusted = 0
        for pair in node.collision_pairs:
            home_distance = None
            if aabb_distance(aabbs[pair[0]], aabbs[pair[1]]) <= (
                node.collision_min_distance
            ):
                points = bullet.getClosestPoints(
                    node.robot_id,
                    node.robot_id,
                    node.collision_min_distance,
                    linkIndexA=pair[0],
                    linkIndexB=pair[1],
                    physicsClientId=node.physics_client,
                )
                if points:
                    home_distance = min(float(point[8]) for point in points)
            clearance = calibrated_clearance(
                node.collision_min_distance,
                home_distance,
                node.collision_home_tolerance,
            )
            node.collision_clearances[pair] = clearance
            if clearance < node.collision_min_distance:
                adjusted += 1
    backend = "cpp" if node._cpp_collision_guard is not None else "pybullet"
    node.get_logger().info(
        "collision avoidance ready: "
        f"{len(node.collision_pairs)} protected pairs, "
        f"backend={backend}, "
        f"minimum_distance={node.collision_min_distance:.3f} m, "
        f"home_adjusted_pairs={adjusted}, "
        f"intra_arm={node.collision_check_intra_arm}, "
        f"inter_arm={node.collision_check_inter_arm}, "
        f"arm_body={node.collision_check_arm_body}"
    )


def collision_for_command(
    node: Any,
    command: dict[str, float],
) -> tuple[str, str, float, float] | None:
    if not node.collision_enabled or not node.collision_pairs:
        return None
    cpp_guard = getattr(node, "_cpp_collision_guard", None)
    if cpp_guard is not None:
        base_state = getattr(node, "joint_state", None)
        if base_state:
            return cpp_guard.evaluate_overlay(base_state, command)
        return cpp_guard.evaluate(command)
    previous: list[tuple[int, float]] = []
    updated: set[int] = set()
    with (getattr(node, "_model_lock", None) or nullcontext()):
        try:
            for name, value in command.items():
                index = node.joint_by_name.get(name)
                if index is None or index not in node.dof_by_joint or index in updated:
                    continue
                previous.append(
                    (
                        index,
                        float(
                            bullet.getJointState(
                                node.robot_id,
                                index,
                                physicsClientId=node.physics_client,
                            )[0]
                        ),
                    )
                )
                updated.add(index)
                bullet.resetJointState(
                    node.robot_id,
                    index,
                    float(value),
                    physicsClientId=node.physics_client,
                )

            aabbs = {
                index: bullet.getAABB(
                    node.robot_id, index, physicsClientId=node.physics_client
                )
                for index in node.collision_links
            }
            closest: tuple[str, str, float, float] | None = None
            for pair in node.collision_pairs:
                clearance = node.collision_clearances[pair]
                if aabb_distance(aabbs[pair[0]], aabbs[pair[1]]) > clearance:
                    continue
                points = bullet.getClosestPoints(
                    node.robot_id,
                    node.robot_id,
                    clearance,
                    linkIndexA=pair[0],
                    linkIndexB=pair[1],
                    physicsClientId=node.physics_client,
                )
                if not points:
                    continue
                distance = min(float(point[8]) for point in points)
                if distance >= clearance:
                    continue
                candidate = (
                    node.link_name_by_index[pair[0]],
                    node.link_name_by_index[pair[1]],
                    distance,
                    clearance,
                )
                if closest is None or distance - clearance < closest[2] - closest[3]:
                    closest = candidate
            return closest
        finally:
            for index, value in previous:
                bullet.resetJointState(
                    node.robot_id,
                    index,
                    value,
                    physicsClientId=node.physics_client,
                )


def block_colliding_command(
    node: Any,
    collision: tuple[str, str, float, float],
    now: float,
    *,
    escape_mode: bool = False,
) -> None:
    first, second, distance, clearance = collision
    node.collision_blocked = True
    node.collision_pair = f"{first}<->{second}"
    node.collision_distance = distance
    node.collision_rejections += 1
    node.collision_escape_mode = escape_mode
    if node.collision_latch_until_release and not escape_mode:
        node.collision_wait_right_release |= node.homing or any(
            arm.active for arm in node.arms.values()
        )
        node.collision_wait_left_release |= node.body.active
    if node.homing and node.external_ik:
        node.solver_rearm.homing_complete()
    node.homing = False
    if not escape_mode:
        node._release_all(send_base_zero=False)
    if now - node.last_collision_reset >= 0.2:
        node.solver_reset_pub.publish(Bool(data=True))
        node.last_collision_reset = now
    if now - node.last_collision_log >= 1.0:
        if escape_mode:
            node.get_logger().warning(
                "arm feedback is inside the protected zone: "
                f"{node.collision_pair}, distance={distance:.4f} m, "
                f"required={clearance:.4f} m; only commands that increase "
                "clearance are allowed"
            )
        else:
            node.get_logger().warning(
                "collision avoidance blocked arm command: "
                f"{node.collision_pair}, distance={distance:.4f} m, "
                f"required={clearance:.4f} m; holding last safe command, "
                "release the active Grip once to resume"
            )
        node.last_collision_log = now


def collision_command_allowed(
    node: Any,
    command: dict[str, float],
    now: float,
) -> bool:
    candidate = node._collision_for_command(command)
    if candidate is None:
        node._clear_collision_block()
        return True

    current = (
        node._collision_for_command(node.joint_state)
        if node.joint_state
        else None
    )
    if improves_collision_clearance(
        current,
        candidate,
        node.collision_escape_epsilon,
    ):
        node.collision_blocked = False
        node.collision_escape_mode = True
        node.collision_pair = f"{candidate[0]}<->{candidate[1]}"
        node.collision_distance = candidate[2]
        node.collision_wait_right_release = False
        node.collision_wait_left_release = False
        return True

    node._block_colliding_command(
        candidate,
        now,
        escape_mode=current is not None,
    )
    return False


def clear_collision_block(node: Any) -> None:
    node.collision_blocked = False
    node.collision_pair = ""
    node.collision_distance = math.inf
    node.collision_escape_mode = False
    node.collision_wait_right_release = False
    node.collision_wait_left_release = False


def apply_collision_latch(
    node: Any,
    base_requested: bool,
    arms_requested: bool,
) -> tuple[bool, bool]:
    if node.collision_escape_mode:
        return base_requested, arms_requested
    if not node.collision_blocked or not node.collision_latch_until_release:
        return base_requested, arms_requested

    if node.collision_wait_right_release and not arms_requested:
        node.collision_wait_right_release = False
    if node.collision_wait_left_release and not base_requested:
        node.collision_wait_left_release = False

    if not (node.collision_wait_right_release or node.collision_wait_left_release):
        node._clear_collision_block()
        node.get_logger().info(
            "collision hold cleared after Grip release; teleop can re-engage"
        )
        return base_requested, arms_requested

    if node.collision_wait_right_release:
        arms_requested = False
    if node.collision_wait_left_release:
        base_requested = False
    return base_requested, arms_requested

