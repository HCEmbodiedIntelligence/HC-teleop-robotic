"""Append-only local staging: encoding backlogs consume disk, not frame slots.

Each record contains JSON metadata and the exact source bytes with a checksum.
The spool is retained on failure and removed only after all media and MCAP close
successfully. It is never a committed upload asset.
"""
from collections import deque
import hashlib
import json
import os
import shutil
import struct
import threading


class MediaSpool:
    def __init__(self, path, *, sync=True, reserve_bytes=1024**3):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.writer = path.open('xb', buffering=0)
        self.reader = path.open('rb', buffering=0)
        self.sync, self.reserve_bytes = sync, reserve_bytes
        self.condition = threading.Condition()
        self.items = deque()
        self.closed = False
        self.bytes = self.peak_bytes = self.peak_items = self.rejected = 0
        self.disk_bytes = self.submitted = self.consumed = 0

    def offer(self, item, size):
        data, raw = item
        metadata = json.dumps(data, allow_nan=False, separators=(',', ':')).encode()
        header = struct.pack('<QQ', len(metadata), len(raw))
        checksum = hashlib.sha256(metadata + raw).digest()
        length = len(header) + len(checksum) + len(metadata) + len(raw)
        if self.closed:
            self.rejected += 1
            return False
        if shutil.disk_usage(self.path.parent).free < length + self.reserve_bytes:
            raise OSError('media spool disk reserve exhausted; source data retained')
        offset = self.writer.tell()
        # One producer; publish only after complete writes. No encoder work or
        # capacity waiting takes place on this path.
        for value in (header, checksum, metadata, raw):
            remaining = memoryview(value)
            while remaining:
                written = self.writer.write(remaining)
                if not written:
                    raise OSError('media spool short write')
                remaining = remaining[written:]
        with self.condition:
            self.items.append((offset, length))
            self.bytes += length
            self.disk_bytes += length
            self.submitted += 1
            self.peak_bytes = max(self.peak_bytes, self.bytes)
            self.peak_items = max(self.peak_items, len(self.items))
            self.condition.notify()
        return True

    def get(self, timeout=.05):
        with self.condition:
            if not self.items and not self.closed:
                self.condition.wait(timeout)
            if not self.items:
                return None
            offset, length = self.items.popleft()
            self.bytes -= length
        self.reader.seek(offset)
        record = self.reader.read(length)
        if len(record) != length:
            raise OSError('media spool truncated record')
        meta_size, raw_size = struct.unpack('<QQ', record[:16])
        payload = record[48:]
        if len(payload) != meta_size + raw_size or hashlib.sha256(payload).digest() != record[16:48]:
            raise ValueError('media spool checksum mismatch')
        self.consumed += 1
        return json.loads(payload[:meta_size]), payload[meta_size:]

    def close(self):
        if not self.closed:
            if self.sync:
                os.fsync(self.writer.fileno())
            self.writer.close()
            with self.condition:
                self.closed = True
                self.condition.notify_all()

    def release(self, *, success=False):
        self.reader.close()
        if success:
            if not self.closed or self.items or self.submitted != self.consumed:
                raise RuntimeError('cannot remove undrained media spool')
            self.path.unlink()

    def status(self):
        with self.condition:
            return {'items': len(self.items), 'bytes': self.bytes,
                    'peak_items': self.peak_items, 'peak_bytes': self.peak_bytes,
                    'rejected': self.rejected, 'closed': self.closed,
                    'storage': 'disk', 'disk_bytes': self.disk_bytes,
                    'submitted': self.submitted, 'consumed': self.consumed}
