"""Low-priority archive worker. No ROS, control locks or dashboard event loop."""
from __future__ import annotations

import ctypes
import json
import os
from pathlib import Path
import signal
import stat
import sys
import time
import zipfile
from datetime import datetime

from .recording_datasets import dataset_files, resolve_dataset, validate_download_selection
from .takeover_log import takeover_file

CHUNK_SIZE = 1024 * 1024


def file_identity(value):
    return [value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns]


def build_download(directory: Path, identifiers: list[str], output: Path, *, single=False, progress=None):
    """Copy in bounded chunks so a single large MCAP has byte-level progress."""
    validate_download_selection(identifiers)
    if single and len(identifiers) != 1:
        raise ValueError('single download requires exactly one dataset')
    progress = progress or (lambda **fields: None)
    progress(state='scanning', stage='正在读取文件列表', percent=0)
    plan = []
    raw = False
    for identifier in identifiers:
        target = resolve_dataset(directory, identifier)
        if target.is_file():
            companion = takeover_file(target)
            if single and companion is None:
                raw = True
            entries = [(target, Path('mcap') / target.name)]
            if companion:
                entries.append((companion, Path('mcap') / companion.name))
        else:
            prefix = (Path(target.name) if single else
                      Path('lerobot' if identifier.startswith('lerobot:') else 'capture_bundle') / target.name)
            entries = [(path, prefix / path.relative_to(target)) for path in dataset_files(target)]
        for path, name in entries:
            info = path.stat(follow_symlinks=False)
            if not stat.S_ISREG(info.st_mode) or path.is_symlink():
                raise ValueError('dataset contains a non-regular file')
            plan.append((path, name.as_posix(), info))
    total = sum(info.st_size for _, _, info in plan)
    progress(state='packing', stage='正在打包', bytes_total=total, bytes_done=0,
             files_total=len(plan), files_done=0, current_file='', percent=0)
    if raw:
        path, _, info = plan[0]
        progress(bytes_done=total, files_done=len(plan), percent=100)
        return dict(path=str(path), owned=False, filename=path.name, archive_size=info.st_size,
                    content_type='application/octet-stream', identity=file_identity(info))
    try:
        done = 0
        with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
            for number, (path, name, info) in enumerate(plan):
                progress(current_file=name[:500])
                # Reject replacement between planning and opening, including symlinks.
                fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
                with os.fdopen(fd, 'rb') as source:
                    if file_identity(os.fstat(source.fileno())) != file_identity(info):
                        raise ValueError('source changed while preparing download: ' + path.name)
                    date = datetime.fromtimestamp(info.st_mtime)
                    stamp = (min(2107, max(1980, date.year)), date.month, date.day, date.hour, date.minute, date.second)
                    entry = zipfile.ZipInfo(name, date_time=stamp)
                    entry.external_attr = (info.st_mode & 0xFFFF) << 16
                    entry.file_size = info.st_size
                    copied = 0
                    with archive.open(entry, 'w', force_zip64=True) as destination:
                        while chunk := source.read(CHUNK_SIZE):
                            destination.write(chunk)
                            copied += len(chunk)
                            done += len(chunk)
                            progress(bytes_done=done, percent=min(99.9, done * 100 / total) if total else 0)
                    if copied != info.st_size or file_identity(os.fstat(source.fileno())) != file_identity(info):
                        raise ValueError('source changed while packaging download: ' + path.name)
                progress(files_done=number + 1, percent=min(99.9, done * 100 / total) if total else
                         (number + 1) * 99.9 / max(1, len(plan)))
            progress(stage='正在完成 ZIP 索引', current_file='')
        filename = (resolve_dataset(directory, identifiers[0]).name + '.zip' if single else
                    'hc-datasets-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.zip')
        return dict(path=str(output), owned=True, filename=filename,
                    archive_size=output.stat().st_size, content_type='application/zip',
                    identity=file_identity(output.stat()))
    except BaseException:
        output.unlink(missing_ok=True)
        raise


def main():
    try:
        os.nice(10)
        # Best effort Linux idle I/O class; failure never changes process priority elsewhere.
        libc = ctypes.CDLL(None, use_errno=True)
        if hasattr(libc, 'ioprio_set'):
            libc.ioprio_set(1, 0, 3 << 13)
    except (OSError, AttributeError):
        pass
    def stopped(_signal, _frame):
        raise InterruptedError('download cancelled')
    signal.signal(signal.SIGTERM, stopped)
    config = json.loads(sys.stdin.readline())
    last = 0.0
    pending = {}
    def emit(force=False, **fields):
        nonlocal last
        pending.update(fields)
        now = time.monotonic()
        if force or now - last >= .1:
            print(json.dumps(pending, ensure_ascii=False), flush=True)
            pending.clear()
            last = now
    try:
        result = build_download(Path(config['directory']), config['filenames'], Path(config['output']),
                                single=config.get('single', False), progress=lambda **fields: emit(**fields))
        emit(force=True, state='ready', stage='打包完成，等待下载', percent=100, **result)
    except InterruptedError:
        emit(force=True, state='cancelled', stage='已取消')
    except Exception as exception:
        emit(force=True, state='error', stage='打包失败', error=str(exception)[:2000])
        sys.exit(1)


if __name__ == '__main__':
    main()
