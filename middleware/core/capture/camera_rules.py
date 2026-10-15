"""Shared timing contract for online association and platform depth evidence."""


def uses_rgb_reference(alignment):
    return alignment.get('camera_timing_reference') == 'per_camera_rgb'


def limit_ns(alignment, source, key):
    return round(source.get(key, alignment[key]) * 1e6)


def depth_reference(row, ident, cfg):
    """A depth image is paired to its own RGB; the RGB is checked against the grid."""
    alignment, source = cfg['alignment'], cfg['sources'][ident]
    if uses_rgb_reference(alignment):
        target = row.get('images', {}).get(ident, {}).get('rgb', {}).get('exposure_midpoint_ns')
        return target, limit_ns(alignment, source, 'rgbd_max_midpoint_skew_ms')
    return row.get('association_time_ns') or row['target_time_ns'], limit_ns(alignment, source, 'camera_max_error_ms')
