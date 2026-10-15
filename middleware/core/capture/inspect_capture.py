"""Inspect sealed capture bytes and report dimensions, timing and loss counters."""
import argparse
import json
from pathlib import Path

import av
import numpy as np

from .catalog import read_json
from .completion import canonical, inventory, validate_tables
from .profile import schema_check
import hashlib


def distribution(values):
    a = np.asarray(values)
    return {'count': len(values), 'max': float(a.max()) if len(a) else None,
            **{f'p{p}': float(np.percentile(a, p)) if len(a) else None for p in (50, 95, 99)}}


def inspect(root, session=None):
    root = Path(root)
    if root.name.startswith('.export-') or list(root.rglob('FAILED.json')):
        raise ValueError('Incomplete export')
    marker = read_json(root / 'export-complete.json')
    schema_check(marker, 'export-complete')
    context = read_json(root / 'capture-context.json')
    schema_check(context, 'capture-context')
    if marker['assets'] != inventory(root) or marker['content_sha256'] != hashlib.sha256(canonical(marker['assets'])).hexdigest():
        raise ValueError('Asset integrity failure')
    validate_tables(root, context)
    mappings = [json.loads(line) for line in (root/'source_mapping.jsonl').read_text().splitlines()]
    errors = {c['id']: [] for c in context['profile']['cameras']}
    ages = []
    for m in mappings:
        schema_check(m, 'source-mapping')
        ages.extend(m['target_ns'] - t for t in m['action_source_ns'])
        for ident, camera in m['cameras'].items():
            errors[ident].append(abs(camera['capture_ns'] - m['target_ns']))
    videos = {}
    for camera in context['profile']['cameras']:
        from .holobrain_fields import image_key
        paths = sorted((root / 'videos' / image_key(camera['id'])).rglob('*.mp4'))
        count = 0
        for path in paths:
            with av.open(str(path)) as stream:
                count += sum(1 for _ in stream.decode(video=0))
        videos[camera['id']] = {'frames': count, 'files': len(paths), 'absolute_grid_error_ns': distribution(errors[camera['id']])}
    info = read_json(root / 'meta/info.json')
    report = {'valid': True, 'synthetic': context['synthetic'], 'export_id': marker['export_id'],
              'content_sha256': marker['content_sha256'], 'assets': len(marker['assets']),
              'frames': info['total_frames'], 'episodes': context['episodes'],
              'dimensions': {k: info['features'][k]['shape'] for k in ('observation.state', 'action')},
              'cameras': videos, 'action_age_ns': distribution(ages), 'physical_sync': context['physical_sync'],
              'selected_invalid_frames': sum(not all(m['action_valid'] + m['state_valid']) for m in mappings)}
    if session is not None:
        quality = read_json(Path(session) / 'quality.json')
        report['session_counters'] = quality['counters']
        report['missing_dropped_invalid_counts'] = {key: value for key, value in quality['counters'].items()
                                                    if any(word in key for word in ('missing', 'reject', 'gap', 'invalid', 'duplicate', 'late', 'repair'))}
        report['source_counts'] = quality['source_counts']
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--session', type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect(args.bundle, args.session), indent=2))
