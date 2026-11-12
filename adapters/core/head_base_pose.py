"""Planar headset displacement -> measured base pose servo (no ROS dependency)."""
import math
import numpy as np
from .arm_teleop_math import mapped_relative_yaw, quaternion_to_matrix


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate(xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * xy[0] - s * xy[1], s * xy[0] + c * xy[1]])


class HeadBasePoseFollower:
    def __init__(self, config, axis_mapping, yaw_direction=1.0):
        self.config = config
        self.mapping = np.asarray(axis_mapping, dtype=float)
        self.yaw_direction = float(yaw_direction)
        if not math.isfinite(self.yaw_direction):
            raise ValueError('body.yaw_direction must be finite')
        for key, default in [('position_scale', 1.), ('yaw_scale', 1.),
                             ('position_gain', 1.5), ('yaw_gain', 1.5),
                             ('position_tolerance', .01), ('yaw_tolerance', .03),
                             ('max_displacement', .5), ('max_yaw', 1.57),
                             ('feedback_timeout', .3)]:
            value = float(config.get(key, default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'body.head_pose_base.{key} must be positive and finite')
            setattr(self, key, value)
        if self.mapping.shape != (3, 3) or not np.isfinite(self.mapping).all():
            raise ValueError('head pose axis mapping must be a finite 3x3 matrix')
        self.reset()

    def reset(self):
        self.head_reference = None
        self.base_reference = None
        self.target = None

    def update(self, head, base, limits):
        if any(not math.isfinite(v) or v <= 0 for v in limits):
            self.reset()
            return (0., 0., 0.)
        position, orientation = (np.asarray(v, dtype=float) for v in head)
        base = np.asarray(base, dtype=float)
        if not all(np.isfinite(v).all() for v in (position, orientation, base)):
            self.reset()
            return (0., 0., 0.)
        if self.head_reference is None:
            self.head_reference = (position.copy(), orientation.copy())
            self.base_reference = base.copy()
        ref_pos, ref_quat = self.head_reference
        # Project into the initial headset's horizontal forward/left frame.
        facing = self.mapping @ (quaternion_to_matrix(ref_quat) @ np.array([0., 0., 1.]))
        ref_yaw = math.atan2(facing[1], facing[0])
        displacement = rotate((self.mapping @ (position - ref_pos))[:2], -ref_yaw)
        displacement *= self.position_scale
        length = float(np.linalg.norm(displacement))
        if length > self.max_displacement:
            displacement *= self.max_displacement / length
        target_xy = self.base_reference[:2] + rotate(displacement, self.base_reference[2])
        yaw_delta = mapped_relative_yaw(orientation, ref_quat, self.mapping)
        yaw_delta = float(np.clip(yaw_delta * self.yaw_direction * self.yaw_scale,
                                  -self.max_yaw, self.max_yaw))
        target_yaw = wrap(self.base_reference[2] + yaw_delta)
        self.target = [float(target_xy[0]), float(target_xy[1]), target_yaw]
        # World position error -> current command-local forward and lateral axes.
        error = rotate(target_xy - base[:2], -base[2])
        distance = float(np.linalg.norm(error))
        velocity = np.zeros(2) if distance <= self.position_tolerance else (
            error * self.position_gain * (1. - self.position_tolerance / distance))
        yaw_error = wrap(target_yaw - base[2])
        yaw_velocity = math.copysign(max(abs(yaw_error) - self.yaw_tolerance, 0.), yaw_error) * self.yaw_gain
        # Preserve translation direction when either axis reaches its speed limit.
        scale = max(1., abs(velocity[0]) / limits[0], abs(velocity[1]) / limits[1])
        return (float(np.clip(yaw_velocity, -limits[2], limits[2])),
                float(velocity[0] / scale), float(velocity[1] / scale))


class HeadBaseVelocityFollower(HeadBasePoseFollower):
    """Follow relative headset motion using command integration, not odometry.

    The caller must pass the same velocity limits enforced by the driver and
    reset on release, stale input or output inhibition. Slippage and dropped
    commands cannot be observed; the estimate is discarded at every clutch.
    """

    def reset(self):
        super().reset()
        self.estimate = np.zeros(3)
        self.previous_command = (0., 0., 0.)
        self.previous_time = None

    def update_velocity(self, head, now, limits, timeout):
        if self.previous_time is not None:
            dt = now - self.previous_time
            if not math.isfinite(dt) or dt < 0. or dt > timeout:
                self.reset()
            else:
                yaw, forward, lateral = self.previous_command
                # Midpoint heading reduces integration error during combined motion.
                self.estimate[:2] += rotate(
                    (forward, lateral), self.estimate[2] + yaw * dt / 2.) * dt
                self.estimate[2] = wrap(self.estimate[2] + yaw * dt)
        command = super().update(head, self.estimate, limits)
        self.previous_time = now
        self.previous_command = command
        return command
