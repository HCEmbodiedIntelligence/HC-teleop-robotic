"""Skip source pre-hashing while retaining completed-package integrity checks."""
import hashlib
import json
from pathlib import Path

import pytest
from mcap.writer import Writer

from humanoid_upload.common import UploadError, canonical
from middleware.core import video_bundle


@pytest.mark.parametrize('legacy_source_hash', [False, True])
def test_no_source_hash_or_retry_rescan_but_output_tampering_is_rejected(tmp_path, monkeypatch, legacy_source_hash):
    source = tmp_path/'source.mcap'
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        channel = writer.register_channel('/event', 'json', 0)
        writer.add_message(channel, 1, b'{"value":1}', 1)
        writer.finish()
    job = dict(recording_id='source-identity', started_at='2026-10-22T00:00:00Z',
               ended_at='2026-10-22T00:00:01Z')
    original = video_bundle.checksum
    def output_checksum(path, limiter=None):
        assert Path(path) != source, 'source CRC64 is not used by recovery or upload'
        return original(path, limiter)
    monkeypatch.setattr(video_bundle, 'checksum', output_checksum)
    target = video_bundle.prepare(source, tmp_path/'bundle', job)
    session = json.loads((target/'session.json').read_text())
    assert 'source_sha256' not in session
    if legacy_source_hash:
        session['source_sha256'] = 'a'*64
        manifest = json.loads((target/'manifest.json').read_text())
        manifest['session_sha256'] = hashlib.sha256(canonical(session)).hexdigest()
        (target/'session.json').write_text(json.dumps(session))
        (target/'manifest.json').write_text(json.dumps(manifest))
    # A retry reuses the sealed bundle without reading the potentially huge source.
    original_open = Path.open
    def no_source_read(path, *args, **kwargs):
        assert path != source, 'completed package retries must not rescan the source'
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', no_source_read)
    assert video_bundle.prepare(source, target, job) == target
    with (target/'raw.mcap').open('ab') as stream:
        stream.write(b'tampered-output')
    with pytest.raises(UploadError, match='ASSET_INTEGRITY'):
        video_bundle.prepare(source, target, job)


def test_multi_chunk_capture_is_transformed_before_reading_whole_source(tmp_path, monkeypatch):
    from mcap.reader import make_reader
    from mcap.writer import CompressionType
    source = tmp_path/'many-chunks.mcap'
    with source.open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE, chunk_size=256*1024)
        writer.start()
        channel = writer.register_channel('/sensor', 'binary', 0)
        writer.add_metadata('before', {'value': 'first'})
        for index in range(48):
            writer.add_message(channel, index+1, bytes([index])*128*1024, index+1, index)
        writer.add_metadata('after', {'value': 'last'})
        writer.finish()
    inputs, first_output_position = [], []
    original_open, original_add = Path.open, Writer.add_message
    def track_open(path, *args, **kwargs):
        stream = original_open(path, *args, **kwargs)
        if path == source:
            inputs.append(stream)
        return stream
    def track_add(writer, *args, **kwargs):
        if not first_output_position:
            first_output_position.append(inputs[0].tell())
        return original_add(writer, *args, **kwargs)
    with monkeypatch.context() as patch:
        patch.setattr(Path, 'open', track_open)
        patch.setattr(Writer, 'add_message', track_add)
        target = video_bundle.prepare(source, tmp_path/'bundle', dict(recording_id='streaming',
            started_at='2026-10-22T00:00:00Z', ended_at='2026-10-22T00:00:01Z'), read_bytes_per_second=1e9)
    assert first_output_position[0] < source.stat().st_size/4
    with (target/'raw.mcap').open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        assert {m.name: m.metadata for m in reader.iter_metadata()} == {
            'before': {'value': 'first'}, 'after': {'value': 'last'}}
        rows = list(reader.iter_messages())
    assert len(rows) == 48
    assert [m.sequence for _, _, m in rows] == list(range(48))
    assert all(m.data == bytes([m.sequence])*128*1024 for _, _, m in rows)
