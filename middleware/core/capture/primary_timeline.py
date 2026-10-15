"""Identity and limits for association on actual primary-camera exposures."""

LEGACY_PRIMARY_ACTION_POLICY = 'recorder-next-state/primary-camera/v2'
PRIMARY_ACTION_POLICY = 'recorder-next-state/primary-camera/v3'


def uses_primary_timeline(alignment):
    return alignment.get('timeline_mode', 'fixed_grid') == 'primary_camera'


def match_limit_ns(cfg):
    # Half a nominal frame is a matching window, not a synchronization guarantee.
    return round(cfg['alignment'].get('camera_match_max_skew_ms',
                 500 / cfg['dataset']['fps']) * 1_000_000)


def frame_identity(source_id, group):
    rgb = group['rgb']
    return {'source_id': source_id, 'source_seq': group['source_seq'],
            'rgb_source_seq': rgb['source_seq'], 'clock_id': rgb['clock_id'],
            'clock_epoch': rgb['clock_epoch'], 'capture_time_ns': rgb['capture_time_ns']}


def consecutive_frames(before, after, fps):
    return bool(before and after and
        all(before[key] == after[key] for key in ('source_id', 'clock_id', 'clock_epoch')) and
        after['rgb_source_seq'] == before['rgb_source_seq'] + 1 and
        0 < after['capture_time_ns'] - before['capture_time_ns'] <= round(1.5e9 / fps))


def next_observation(before, after, alignment):
    """Next recorded observation may skip camera counters, never clock epochs."""
    return bool(before and after and
        all(before[key] == after[key] for key in ('source_id', 'clock_id', 'clock_epoch')) and
        after['rgb_source_seq'] > before['rgb_source_seq'] and
        0 < after['capture_time_ns'] - before['capture_time_ns'] <=
        round(alignment.get('next_observation_max_gap_ms', 1000) * 1_000_000))
