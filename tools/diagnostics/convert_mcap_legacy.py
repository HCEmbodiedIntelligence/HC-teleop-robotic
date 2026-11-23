"""Lossless event-wire conversion to ROS2/CDR; never overwrite the source.

Run: python3 -m tools.diagnostics.convert_mcap_legacy SOURCE DESTINATION
JSON event text is preserved verbatim inside std_msgs/msg/String. All original
ROS payloads, message order, sequence numbers, timestamps and metadata survive.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import tempfile

from mcap.records import Attachment, Channel, Footer, Header, Message, Metadata, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import Writer
from middleware.core.recording.recording_cdr import write_string

STRING_SCHEMA = ('std_msgs/msg/String', 'ros2msg', b'string data\n')


def _schema(record, normalize):
    if record.encoding == 'jsonschema' and normalize:
        return STRING_SCHEMA
    if record.encoding != 'ros2msg':
        raise ValueError(f'unsupported schema encoding: {record.name}: {record.encoding}')
    return record.name, record.encoding, record.data


def _payload(record, channel, schema, normalize):
    if channel.message_encoding == 'json' and normalize:
        if schema.encoding != 'jsonschema':
            raise ValueError('JSON channel does not have a JSON schema')
        text = record.data.decode('utf-8')
        json.loads(text)  # Validate without normalizing whitespace, floats or integers.
        return write_string(text), True
    if channel.message_encoding != 'cdr' or schema.encoding != 'ros2msg':
        raise ValueError(f'unsupported channel encoding: {channel.topic}')
    return record.data, False


def _fingerprint(path, normalize):
    schemas, channels = {}, {}
    schema_defs, channel_defs, metadata, attachments = [], [], [], []
    messages = converted = 0
    digest = hashlib.sha256()
    header = None
    footer = False
    with Path(path).open('rb') as stream:
        for record in StreamReader(stream, validate_crcs=True).records:
            if isinstance(record, Header):
                header = (record.profile, record.library)
            elif isinstance(record, Schema) and record.id not in schemas:
                schemas[record.id] = (len(schema_defs), record)
                schema_defs.append(_schema(record, normalize))
            elif isinstance(record, Channel) and record.id not in channels:
                schema_index, schema = schemas[record.schema_id]
                if record.message_encoding not in ('cdr', 'json'):
                    raise ValueError(f'unsupported channel: {record.topic}')
                if record.message_encoding == 'json' and not normalize:
                    raise ValueError('destination is not entirely CDR')
                if (record.message_encoding == 'json') != (schema.encoding == 'jsonschema'):
                    raise ValueError('channel/schema encoding mismatch')
                channels[record.id] = (len(channel_defs), record)
                channel_defs.append((record.topic, 'cdr', schema_index, sorted(record.metadata.items())))
            elif isinstance(record, Message):
                index, channel = channels[record.channel_id]
                raw, changed = _payload(record, channel, schemas[channel.schema_id][1], normalize)
                digest.update(struct.pack('<IQQIQ', index, record.log_time, record.publish_time,
                                          record.sequence, len(raw)))
                digest.update(raw)
                messages += 1
                converted += changed
            elif isinstance(record, Metadata):
                metadata.append((record.name, sorted(record.metadata.items())))
            elif isinstance(record, Attachment):
                attachments.append((record.create_time, record.log_time, record.name,
                                    record.media_type, len(record.data), hashlib.sha256(record.data).hexdigest()))
            elif isinstance(record, Footer):
                footer = True
    if not header or not footer:
        raise ValueError('incomplete MCAP header/footer')
    return dict(header=header, schemas=schema_defs, channels=channel_defs,
                metadata=metadata, attachments=attachments, messages=messages,
                message_digest=digest.hexdigest(), converted_messages=converted)


def verify(source: Path, destination: Path) -> dict:
    """CRC-check and compare normalized messages and all retained evidence."""
    before, after = _fingerprint(source, True), _fingerprint(destination, False)
    for key in before:
        if key != 'converted_messages' and before[key] != after[key]:
            raise ValueError(f'converted MCAP differs in {key}')
    return dict(verified=True, messages=after['messages'], channels=len(after['channels']),
                schemas=len(after['schemas']), converted_messages=before['converted_messages'],
                message_digest=after['message_digest'])


def _identity(path):
    value = path.stat()
    return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns


def convert(source: Path, destination: Path) -> dict:
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(destination)
    original = _identity(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.'+destination.name+'.', suffix='.partial',
                                     dir=destination.parent)
    temporary = Path(temporary)
    schemas, schema_ids, channel_ids, channels = {}, {}, {}, {}
    started = ended = False
    try:
        with source.open('rb') as stream, os.fdopen(fd, 'wb') as output:
            writer = Writer(output, enable_data_crcs=True)
            for record in StreamReader(stream, validate_crcs=True).records:
                if isinstance(record, Header):
                    if started:
                        raise ValueError('multiple MCAP headers')
                    writer.start(profile=record.profile, library=record.library)
                    started = True
                elif isinstance(record, Schema) and record.id not in schemas:
                    schemas[record.id] = record
                    schema_ids[record.id] = writer.register_schema(*_schema(record, True))
                elif isinstance(record, Channel) and record.id not in channel_ids:
                    schema = schemas[record.schema_id]
                    if (record.message_encoding == 'json') != (schema.encoding == 'jsonschema'):
                        raise ValueError('channel/schema encoding mismatch')
                    if record.message_encoding not in ('json', 'cdr'):
                        raise ValueError('unsupported channel encoding')
                    channels[record.id] = record
                    channel_ids[record.id] = writer.register_channel(
                        record.topic, 'cdr', schema_ids[record.schema_id], record.metadata)
                elif isinstance(record, Message):
                    channel = channels[record.channel_id]
                    raw, _ = _payload(record, channel, schemas[channel.schema_id], True)
                    writer.add_message(channel_ids[record.channel_id], record.log_time, raw,
                                       record.publish_time, record.sequence)
                elif isinstance(record, Metadata):
                    writer.add_metadata(record.name, record.metadata)
                elif isinstance(record, Attachment):
                    writer.add_attachment(record.create_time, record.log_time, record.name,
                                          record.media_type, record.data)
                elif isinstance(record, Footer):
                    ended = True
            if not started or not ended:
                raise ValueError('incomplete MCAP header/footer')
            writer.finish()
            output.flush()
            os.fsync(output.fileno())
        report = verify(source, temporary)
        if _identity(source) != original:
            raise RuntimeError('source changed during conversion; refusing to install')
        os.chmod(temporary, stat.S_IMODE(source.stat().st_mode))
        os.link(temporary, destination)  # Atomic, fails rather than overwriting a racing destination.
        return dict(report, source=str(source), destination=str(destination), source_bytes=original[2],
                    destination_bytes=destination.stat().st_size)
    finally:
        temporary.unlink(missing_ok=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    print(json.dumps(convert(args.source, args.destination), ensure_ascii=False))
