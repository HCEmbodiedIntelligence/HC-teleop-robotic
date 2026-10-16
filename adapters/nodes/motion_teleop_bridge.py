#!/usr/bin/env python3
"""Bridge a selected profile external-IK adapter to motion ServoP/ServoJ channels.

The state/cache implementation is deliberately independent of ROS.  Receiving a
motion result alone never grants authority to move an arm: a fresh enabled
teleoperation status, hardware heartbeat and forwarded target are all required.
"""
from __future__ import annotations

import json
import math
import time

SIDES = ("right", "left")
ARM_NAMES = {side: tuple(f"Joint{i}_{'R' if side == 'right' else 'L'}" for i in range(1, 8)) for side in SIDES}


def joint_values(names, positions):
    if not names or len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("invalid joint names/positions")
    values = dict(zip(names, map(float, positions)))
    if any(not name or not math.isfinite(value) for name, value in values.items()):
        raise ValueError("invalid joint value")
    return values


def valid_stamp(stamp, wall, max_age=0.3):
    return math.isfinite(stamp) and stamp > 0 and -0.2 <= wall - stamp <= max_age


class MotionCache:
    def __init__(self, arm_names=None):
        self.arm_names = arm_names or ARM_NAMES
        flattened = [n for names in self.arm_names.values() for n in names]
        if not flattened or len(set(flattened)) != len(flattened):
            raise ValueError("arm joint groups must be nonempty and disjoint")
        self.sides = tuple(self.arm_names)
        self.feedback_times = {}
        self.feedback_stamp = {}
        self.feedback = {}
        self.feedback_time = float('-inf')
        self.status = {}
        self.status_time = float('-inf')
        self.hardware = False
        self.hardware_time = float('-inf')
        self.estopped = False
        self.targets = {}
        self.lease = {}
        self.cutoff = {side: 0.0 for side in self.sides}
        self.reset_pending = False
        self.reset_wall = 0.0
        self.guard_until = 0.0

    def clear(self, wall):
        self.targets.clear()
        self.lease.clear()
        self.cutoff = {side: wall for side in self.sides}

    def update_status(self, status, now, wall):
        if not isinstance(status, dict):
            raise ValueError("status must be an object")
        if not isinstance(status.get('arms', {}), dict) or any(
                not isinstance(arm, dict) for arm in status.get('arms', {}).values()):
            raise ValueError('invalid arm status')
        self.status, self.status_time = status, now
        for side in self.sides:
            if not self.allowed(side, now):
                self.targets.pop(side, None)
                self.lease.pop(side, None)
                self.cutoff[side] = wall

    def allowed(self, side, now, homing=False):
        status = self.status
        return (not self.estopped and now >= self.guard_until and not self.reset_pending
                and self.hardware and now - self.hardware_time <= 0.5
                and now - self.status_time <= 0.6 and status.get('enabled') is True
                and now - self.feedback_times.get(side, float("-inf")) <= 0.3
                and all(name in self.feedback for name in self.arm_names[side])
                and (status.get('mode') == 'homing' if homing else
                     status.get('mode') != 'homing' and
                     status.get('arms', {}).get(side, {}).get('active') is True))

    def feedback_received(self, names, positions, stamp, now, wall):
        values = joint_values(names, positions)
        present = [side for side, names in self.arm_names.items() if all(n in values for n in names)]
        if not present:
            raise ValueError('feedback contains no complete configured arm')
        if not valid_stamp(stamp, wall):
            raise ValueError("stale feedback header")
        if self.reset_pending and stamp <= self.reset_wall:
            raise ValueError('feedback predates reset')
        if any(stamp <= self.feedback_stamp.get(side, 0) for side in present):
            raise ValueError('duplicate or reordered feedback')
        self.feedback.update(values)
        self.feedback_time = now
        for side in present:
            self.feedback_times[side] = now
            self.feedback_stamp[side] = stamp
        if self.reset_pending:
            self.reset_pending = False
            self.clear(wall)
            self.guard_until = now + 0.1
            return True
        return False

    def reset(self, wall):
        self.clear(wall)
        self.reset_pending = True
        self.reset_wall = wall

    def grant(self, side, now, wall, homing=False):
        if not self.allowed(side, now, homing):
            return False
        if side not in self.lease:
            self.cutoff[side] = wall
        self.lease[side] = (now, homing)
        return True

    def accept_result(self, names, positions, stamp, now, wall):
        values = joint_values(names, positions)
        if not valid_stamp(stamp, wall, 0.1):
            return
        for side in self.sides:
            lease = self.lease.get(side)
            required = self.arm_names[side]
            if (lease is not None and now - lease[0] <= 0.2
                    and self.allowed(side, now, lease[1]) and stamp > self.cutoff[side]
                    and all(name in values for name in required)):
                self.targets[side] = ({name: values[name] for name in required}, now)

    def output(self, now, wall):
        if now - self.feedback_time > 0.3 or self.reset_pending:
            self.clear(wall)
            return None
        result = dict(self.feedback)
        for side, names in self.arm_names.items():
            if now - self.feedback_times.get(side, float('-inf')) > .3:
                for name in names:
                    result.pop(name, None)
        for side in self.sides:
            lease = self.lease.get(side)
            target = self.targets.get(side)
            if (lease and target and self.allowed(side, now, lease[1])
                    and now - lease[0] <= 0.2 and now - target[1] <= 0.1):
                result.update(target[0])
            elif (lease and (not self.allowed(side, now, lease[1]) or now - lease[0] > 0.2)):
                self.targets.pop(side, None)
                self.lease.pop(side, None)
                self.cutoff[side] = wall
            elif target:
                # A late/missing result requires measured hold, but must not
                # revoke a fresh target lease and advance its acceptance cutoff.
                # Otherwise every timer tick rejects the in-flight SDK result.
                self.targets.pop(side, None)
        return result


def profile_settings(path):
    import yaml
    from pathlib import Path
    profile = yaml.safe_load(Path(path).read_text())
    arms = profile['arms']
    names = {side: tuple(arm['joint_names']) for side, arm in arms.items()}
    if not names or any(not joints for joints in names.values()):
        raise ValueError('profile requires nonempty joint groups')
    control = profile.get('control', {})
    channels = yaml.safe_load((Path(path).parent / 'motion/channels.yaml').read_text())['channels']
    endpoints = {}
    for side, arm in arms.items():
        for kind in ('servo_p', 'servo_j'):
            matches = [c for c in channels if c['kind'] == kind and c['group'] == side + '_arm']
            if len(matches) != 1 or matches[0]['base_frame'] != arm['base_link'] or matches[0]['tip_frame'] != arm['ee_link']:
                raise ValueError('motion channel frame/group does not match profile')
            endpoints[(side, kind)] = matches[0]['endpoint']
    bases = {arm['base_link'] for arm in arms.values()}
    if len(bases) != 1:
        raise ValueError('PoseArray requires a common base frame for configured arms')
    return names, bases.pop(), endpoints, control


def main():
    import os
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from middleware.core.robot_api import selected_profile
    import yaml
    config = os.environ.get('HC_ARM_TELEOP_CONFIG')
    if config:
        profile = Path(config).resolve().parent
    else:
        middleware_config = root / 'middleware/config.yaml'
        profile = selected_profile(yaml.safe_load(middleware_config.read_text()), middleware_config.parent)
    from adapters.nodes.unified_motion_teleop import main as run
    run(profile)


if __name__ == '__main__':
    main()
