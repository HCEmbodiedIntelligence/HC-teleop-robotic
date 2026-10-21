"""Lightweight LeRobot dataset inspection, without importing torch/LeRobot."""
from __future__ import annotations

import json
import tempfile
import zipfile
from datetime import datetime
from pathlib import Path
from urllib.parse import quote


def resolve_dataset(directory: Path, identifier: str) -> Path:
    """Keep file IDs distinct from dataset IDs, rejecting traversal and symlinks."""
    prefix = next((p for p in ('lerobot:', 'bundle:') if identifier.startswith(p)), '')
    is_dataset = bool(prefix)
    name = identifier[len(prefix):] if prefix else identifier
    if not name or name in ('.', '..') or '/' in name or '\\' in name or name.startswith('.'):
        raise ValueError('invalid recording identifier')
    parent = directory / ('bundles' if prefix == 'bundle:' else 'lerobot') if is_dataset else directory
    target = parent / name
    if parent.is_symlink() or target.is_symlink() or target.resolve().parent != parent.resolve():
        raise ValueError('recording path escapes storage directory')
    if is_dataset:
        if not target.is_dir() or not (target / ('session.json' if prefix == 'bundle:' else 'meta/info.json')).is_file():
            raise FileNotFoundError(f'dataset not found: {name}')
    elif not target.is_file() or target.suffix not in ('.mcap', '.jsonl'):
        raise FileNotFoundError(f'recording not found: {name}')
    return target


def dataset_files(target: Path) -> list[Path]:
    files = []
    for path in target.rglob('*'):
        if path.is_symlink():
            raise ValueError('dataset contains symlinks')
        if path.is_file():
            files.append(path)
    return sorted(files)


def inspect_dataset(target: Path) -> dict:
    info = json.loads((target / 'meta/info.json').read_text())
    files = dataset_files(target)
    frames, fps = int(info['total_frames']), float(info['fps'])
    if frames < 0 or fps <= 0:
        raise ValueError('invalid dataset frame count or FPS')
    identifier = 'lerobot:' + target.name
    features = info.get('features', {})
    cameras = [key for key, value in features.items() if value.get('dtype') in ('video', 'image')]
    size = sum(path.stat().st_size for path in files)
    stamp = max(path.stat().st_mtime for path in files)
    videos = []
    for path in files:
        if path.suffix == '.mp4' and path.relative_to(target).parts[0] == 'videos':
            relative = path.relative_to(target).as_posix()
            videos.append({'feature': path.relative_to(target).parts[1], 'path': relative,
                           'url': f'/api/recordings/{quote(identifier, safe="")}/video?path={quote(relative, safe="")}'})
    sync_path = target / 'meta/teleop_sync.json'
    sync = json.loads(sync_path.read_text()) if sync_path.exists() else {}
    return {
        'id': identifier, 'filename': target.name, 'format': 'LeRobot',
        'size_bytes': size, 'size_human': human_size(size),
        'created_at': datetime.fromtimestamp(target.stat().st_ctime).strftime('%Y-%m-%d %H:%M:%S'),
        'modified_at': datetime.fromtimestamp(stamp).strftime('%Y-%m-%d %H:%M:%S'),
        'timestamp': stamp, 'is_current': False, 'marked': False,
        'frames': frames, 'total_episodes': int(info['total_episodes']), 'fps': fps,
        'camera_count': len(cameras), 'feature_count': len(features), 'task': sync.get('task', ''),
        'duration_sec': frames / fps, 'duration_human': f'{frames / fps:.1f}s',
        'profile_id': info.get('robot_type', ''), 'profile_display_name': info.get('robot_type', ''),
        'message_count': frames, 'topic_count': len(features), 'avg_rate_hz': fps,
        'channels': [{'topic': key, 'type': f"{value.get('dtype', '')} {value.get('shape', [])}",
                      'count': frames, 'rate_hz': fps} for key, value in features.items()],
        'videos': videos,
    }


def human_size(size: int) -> str:
    for unit in ('B', 'KB', 'MB', 'GB'):
        if size < 1024 or unit == 'GB':
            return f'{size:.1f} {unit}'
        size /= 1024


def archive_dataset(target: Path) -> Path:
    """Use a temporary ZIP on disk so downloads do not hold video bytes in RAM."""
    files = dataset_files(target)
    with tempfile.NamedTemporaryFile(prefix='hc-lerobot-', suffix='.zip', delete=False) as handle:
        archive = Path(handle.name)
    try:
        with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_STORED) as output:
            for path in files:
                output.write(path, Path(target.name) / path.relative_to(target))
        return archive
    except BaseException:
        archive.unlink(missing_ok=True)
        raise


def dataset_video(target: Path, relative: str) -> Path:
    candidate = target / relative
    if candidate.suffix != '.mp4' or candidate not in dataset_files(target) or not candidate.resolve().is_relative_to((target / 'videos').resolve()):
        raise FileNotFoundError('dataset video not found')
    return candidate


def pending_datasets(directory: Path, existing_ids: set) -> list[dict]:
    from .export_progress import read_progress
    entries = []
    for path in directory.glob('.*.progress.json'):
        if path.is_symlink():
            continue
        name = path.name[1:-len('.progress.json')]
        identifier = 'lerobot:' + name
        if identifier in existing_ids or not name or name.startswith('.'):
            continue
        progress = read_progress(path)
        if progress.get('state') == 'exporting' and progress.get('pid'):
            import os
            try:
                os.kill(int(progress['pid']), 0)
            except ProcessLookupError:
                progress = {**progress, 'state': 'error', 'stage': '转换中断', 'error': '转换进程已退出，原始 MCAP 已保留'}
            except (PermissionError, ValueError):
                pass
        if progress.get('state') not in ('exporting', 'error'):
            continue
        entries.append({'id': identifier, 'filename': name, 'format': 'LeRobot',
                        'export_state': progress['state'], 'export_progress': progress,
                        'export_error': progress.get('error', ''), 'is_current': True,
                        'size_bytes': 0, 'size_human': '--', 'duration_human': '--',
                        'created_at': datetime.fromtimestamp(path.stat().st_ctime).strftime('%Y-%m-%d %H:%M:%S'),
                        'timestamp': path.stat().st_mtime, 'channels': [], 'videos': [],
                        'message_count': 0, 'topic_count': 0})
    return entries


def inspect_bundle(target: Path) -> dict:
    from .topic_recorder import _inspect_mcap_file
    session = json.loads((target / 'session.json').read_text())
    files = dataset_files(target)
    stamp = max(p.stat().st_mtime for p in files)
    size = sum(p.stat().st_size for p in files)
    identifier = 'bundle:' + target.name
    return {**_inspect_mcap_file(target / 'raw.mcap'), 'id': identifier, 'filename': target.name,
            'format': 'CAPTURE_BUNDLE', 'timestamp': stamp, 'is_current': False,
            'size_bytes': size, 'size_human': human_size(size),
            'created_at': session['capture_started_at'], 'modified_at': session['capture_ended_at'],
            'videos': [{'feature': video.get('topic', video['path']), 'path': video['path'],
                        'url': f'/api/recordings/{quote(identifier, safe="")}/video?path={quote(video["path"], safe="")}'}
                       for video in session['videos']]}
