"""Integer ROS epoch grid shared by simultaneous online and offline recordings."""

NSEC = 1_000_000_000
JOINT_MAX_AGE_NS = 200_000_000
CAMERA_MAX_AGE_NS = 250_000_000


def epoch_grid(start_ns, end_ns, fps):
    """Yield epoch-aligned sample times inside the closed capture window."""
    if fps <= 0:
        raise ValueError('recording fps must be positive')
    half = fps // 2
    first_index = -((half - start_ns * fps) // NSEC)
    last_index = ((end_ns + 1) * fps - half - 1) // NSEC
    for index in range(first_index, last_index + 1):
        yield (index * NSEC + half) // fps
