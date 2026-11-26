"""Map a side's calibrated native gripper positions to its recorded open/closed convention."""
from math import isfinite


class BinaryGrip:
    def __init__(self, name, native_names, opened, closed, *, native_unit, threshold=0.5, marker=None):
        self.name = name
        self.native_names = tuple(native_names)
        self.opened = tuple(opened)
        self.closed = tuple(closed)
        self.marker = marker
        self.native_unit = native_unit
        self.threshold = float(threshold)
        if (len(self.native_names) != len(self.opened) or len(self.opened) != len(self.closed)
                or len(set(self.native_names)) != len(self.native_names)
                or not all(isfinite(v) for v in (*self.opened, *self.closed))
                or not 0 <= self.threshold <= 1 or marker in self.native_names):
            raise ValueError(f'invalid gripper calibration: {name}')
        # Compare a complete measured posture with the halfway (or configured threshold)
        # posture along the calibrated closing direction. Fixed linkage axes carry no signal.
        self._moving = [(n, a + (b-a)*self.threshold, b-a)
                        for n, a, b in zip(self.native_names, self.opened, self.closed) if a != b]
        if not self._moving:
            raise ValueError(f'gripper calibration has no moving joints: {name}')
        self._layouts = [None, None]
        self.command_names = self.native_names + ((marker,) if marker else ())
        self._commands = ((self.opened + (0.0,), self.closed + (1.0,)) if marker
                          else (self.opened, self.closed))

    @classmethod
    def from_profile(cls, side, grippers, native_names, native_unit):
        hand = grippers[side]
        indices = [hand['finger_joint_names'].index(n) for n in native_names]
        return cls(side + '_grip', native_names, [hand['finger_open'][i] for i in indices],
                   [hand['finger_closed'][i] for i in indices], native_unit=native_unit,
                   threshold=grippers.get('binary_trigger_threshold', 0.5),
                   marker=hand.get('binary_state_joint'))

    @classmethod
    def from_description(cls, public_name, calibration):
        return cls(public_name, **calibration)

    def describe(self):
        return dict(native_names=list(self.native_names), opened=list(self.opened), closed=list(self.closed),
                    native_unit=self.native_unit, threshold=self.threshold, marker=self.marker)

    def encode(self, bit):
        if bit not in (0.0, 1.0):
            raise ValueError(f'{self.name} must be 0 (open) or 1 (closed)')
        return self._commands[int(bit)]

    def _layout(self, names, use_marker):
        if len(set(names)) != len(names):
            return -1, ()
        if use_marker and self.marker and self.marker in names:
            return names.index(self.marker), ()
        indices = {n: i for i, n in enumerate(names)}
        if any(n not in indices for n, _, _ in self._moving):
            return -1, ()
        return -1, tuple((indices[n], midpoint, span) for n, midpoint, span in self._moving)

    def decode(self, names, positions, *, use_marker=False):
        if len(names) != len(positions):
            return None
        cached = self._layouts[use_marker]
        if cached is None or cached[0] != names:
            # rclpy exposes names as a list; snapshot only when the incoming layout changes.
            key = names.copy() if isinstance(names, list) else tuple(names)
            cached = (key, self._layout(names, use_marker))
            self._layouts[use_marker] = cached
        marker, moving = cached[1]
        if marker >= 0:
            value = positions[marker]
            return float(value) if value in (0.0, 1.0) else None
        if not moving:
            return None
        score = 0.0
        for index, midpoint, span in moving:
            value = positions[index]
            if not isfinite(value):
                return None
            score += (value - midpoint) * span
        return float(score >= 0)
