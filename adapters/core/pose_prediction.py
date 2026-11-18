"""Bounded, fading prediction of short VR pose gaps; never changes freshness."""
from __future__ import annotations

import math
import numpy as np


def _cap(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector * min(1.0, limit / max(norm, 1e-12))


def _multiply(a, b):
    return np.r_[a[3]*b[:3] + b[3]*a[:3] + np.cross(a[:3], b[:3]),
                 a[3]*b[3] - np.dot(a[:3], b[:3])]


def _rotation_vector(a, b):
    q = _multiply(a, np.r_[-b[:3], b[3]])
    if q[3] < 0:
        q = -q
    size = float(np.linalg.norm(q[:3]))
    return q[:3] * (2 * math.atan2(size, max(0.0, q[3])) / max(size, 1e-12))


def _rotate(q, vector):
    angle = float(np.linalg.norm(vector))
    delta = np.r_[vector * (math.sin(angle / 2) / max(angle, 1e-12)), math.cos(angle / 2)]
    result = _multiply(delta, q)
    return result / np.linalg.norm(result)


class BoundedPosePredictor:
    """Predict at most 100 ms/20 mm/0.04 rad, then hold the bounded pose.

    Sample-clock intervals estimate velocity, receive age bounds prediction.
    25 ms grace avoids extrapolating between ordinary 90 Hz packets. Prediction
    velocity fades with a 40 ms time constant; late samples reconcile over 40 ms.
    """
    grace = 0.025
    decay = 0.04
    max_distance = 0.02
    max_angle = 0.04

    def __init__(self):
        self.reset()

    def reset(self):
        self.pose = None
        self.stamp = None
        self.sample_time = None
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.correction = np.zeros(3)
        self.angular_correction = np.zeros(3)

    def observe(self, pose, stamp, sample_time=None):
        position, quaternion = (np.asarray(v, dtype=float).copy() for v in pose)
        if (not math.isfinite(stamp) or not np.all(np.isfinite(position))
                or not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-8):
            self.reset()
            return
        quaternion /= np.linalg.norm(quaternion)
        if sample_time is not None and not math.isfinite(sample_time):
            self.reset()
            return
        if self.stamp is not None and stamp <= self.stamp:
            self.reset()
        old = self.pose
        age = stamp - self.stamp if self.stamp is not None else 0.0
        if old is not None:
            if age > self.grace:
                predicted = self.predict(stamp)
                self.correction = _cap(predicted[0] - position, self.max_distance)
                self.angular_correction = _cap(_rotation_vector(predicted[1], quaternion), self.max_angle)
            else:
                self.correction *= math.exp(-age / self.decay)
                self.angular_correction *= math.exp(-age / self.decay)
        dt = sample_time - self.sample_time if sample_time is not None and self.sample_time is not None else age
        # A repeated/out-of-order sample or a long outage must not seed a catch-up velocity.
        if old is not None and 0.002 <= dt <= 0.10 and age <= 0.10:
            self.velocity = _cap((position - old[0]) / dt, 0.4)
            self.angular_velocity = _cap(_rotation_vector(quaternion, old[1]) / dt, 0.8)
        else:
            self.velocity[:] = 0.0
            self.angular_velocity[:] = 0.0
        self.pose = (position, quaternion)
        self.stamp, self.sample_time = stamp, sample_time

    def predict(self, now, *, horizon=0.10):
        if self.pose is None:
            return None
        if not math.isfinite(horizon) or not 0 < horizon < 0.15:
            raise ValueError("prediction horizon must be in (0, 0.15)")
        age = min(max(0.0, now - self.stamp), horizon)
        integrated = self.decay * (1 - math.exp(-max(0.0, age - self.grace) / self.decay))
        correction_decay = math.exp(-age / self.decay)
        displacement = _cap(self.velocity * integrated + self.correction * correction_decay, self.max_distance)
        rotation = _cap(self.angular_velocity * integrated + self.angular_correction * correction_decay, self.max_angle)
        return self.pose[0] + displacement, _rotate(self.pose[1], rotation)
