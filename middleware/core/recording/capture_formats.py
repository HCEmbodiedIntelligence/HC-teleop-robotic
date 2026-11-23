"""The saved recording choice and the independently uploaded artifacts it creates."""

SOURCE_FORMATS = {'mcap': 'MCAP', 'lerobot': 'LEROBOT_V3', 'mcap_mp4': 'CAPTURE_BUNDLE'}


def artifact_formats(save_format):
    return ('mcap', 'lerobot') if save_format == 'both' else (save_format,)


def upload_formats(job):
    # Older both recordings uploaded MCAP only. Never change a saved choice.
    return artifact_formats(job.get('upload_format') or ('mcap' if job['format'] == 'both' else job['format']))


def job_artifact_formats(job):
    return tuple(dict.fromkeys((*upload_formats(job), *artifact_formats(job['format']))))


def artifact_identity(job, fmt):
    # Preserve the ID of the independently selected primary upload, also on replay.
    return job['recording_id'] + ('-' + fmt if fmt != upload_formats(job)[0] else '')


def artifact_snapshot(job, fmt):
    if fmt == upload_formats(job)[0]:
        return job.get('capture_snapshot')
    return job.get('capture_snapshots', {}).get(SOURCE_FORMATS[fmt])


def lerobot_snapshot(metadata):
    import json
    snapshots = json.loads(metadata.get('capture_snapshots') or '{}')
    if snapshots.get('LEROBOT_V3'):
        return snapshots['LEROBOT_V3']
    snapshot = json.loads(metadata.get('capture_snapshot') or 'null')
    if snapshot:
        from humanoid_upload.capture_plan import validate_snapshot
        validate_snapshot(snapshot)
    return snapshot if snapshot and snapshot['source_format'] == 'LEROBOT_V3' else None
