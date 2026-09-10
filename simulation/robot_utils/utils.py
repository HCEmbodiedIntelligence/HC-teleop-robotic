"""Small pose and PyBullet visualisation helpers."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
import pybullet as p


Pose = Sequence[Sequence[float]]


def debug_draw_pose(
    pose: Pose,
    replace_line_ids: Optional[Sequence[int]] = None,
    line_width: float = 5.0,
    line_length: float = 0.3,
    physics_client_id: int = 0,
) -> list[int]:
    """Draw or update an XYZ frame and return its three PyBullet line IDs."""
    axes = np.identity(3) * line_length
    colors = np.identity(3)
    line_ids = [] if replace_line_ids is None else list(replace_line_ids)

    if replace_line_ids is not None and len(replace_line_ids) != 3:
        raise ValueError("replace_line_ids must contain exactly three line IDs")

    for axis_index in range(3):
        line_end, _ = p.multiplyTransforms(
            pose[0], pose[1], axes[axis_index], [0.0, 0.0, 0.0, 1.0]
        )
        kwargs = {
            "lineFromXYZ": pose[0],
            "lineToXYZ": line_end,
            "lineColorRGB": colors[axis_index],
            "lineWidth": line_width,
            "lifeTime": 0,
            "physicsClientId": physics_client_id,
        }
        if replace_line_ids is None:
            line_ids.append(p.addUserDebugLine(**kwargs))
        else:
            kwargs["replaceItemUniqueId"] = replace_line_ids[axis_index]
            p.addUserDebugLine(**kwargs)
    return line_ids


def multiply_transforms(pose1: Pose, pose2: Pose):
    """Compose two ``[position, quaternion]`` transforms."""
    return p.multiplyTransforms(pose1[0], pose1[1], pose2[0], pose2[1])


def invert_transform(pose: Pose):
    """Invert a ``[position, quaternion]`` transform."""
    return p.invertTransform(pose[0], pose[1])


def pose_msg_to_list(msg):
    """Convert a geometry_msgs/Pose-like object into PyBullet pose form."""
    return [
        [msg.position.x, msg.position.y, msg.position.z],
        [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ],
    ]
