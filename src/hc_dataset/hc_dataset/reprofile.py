"""
Convert legacy HC recordings to the channel profile of a reference MCAP.

Schemas, channels, QoS metadata and the MCAP Header come from the reference;
dynamic robot and camera payloads come from the source recording.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any, Callable, Dict, Iterable, Iterator, Mapping, Optional, Tuple
from urllib.parse import unquote, urlparse

from mcap.reader import make_reader
from mcap.records import Channel, Message, Schema
from mcap.writer import CompressionType, Writer
from mcap_ros2._dynamic import serialize_dynamic
from mcap_ros2.decoder import DecoderFactory


PROFILE_NAME = "hc_tj_description/v1"

# Direct CDR-compatible copies. ROS 2 accepts both ``pkg/Type`` and
# ``pkg/msg/Type`` schema spellings; the serialized payload is identical.
DIRECT_TOPIC_MAP = {
    "/hc_teleop/joint_states": "io_teleop/joint_states",
    "/hc_teleop/actual_ee_poses": "/io_teleop/state_ee_poses",
    "/hc_teleop/target_ee_poses": "io_teleop/target_ee_poses",
    "/hc_teleop/camera_head/color/compressed": "/io_teleop/camera_head/color",
    "/hc_teleop/camera_head/depth/compressed": "/io_teleop/camera_head/depth",
}

SOURCE_JOINT_COMMAND = "/hc_teleop/joint_cmd_arm"
SOURCE_LEFT_HAND = "/hc_teleop/joint_cmd_finger_left"
SOURCE_RIGHT_HAND = "/hc_teleop/joint_cmd_finger_right"
SOURCE_BASE_TARGET = "/hc_teleop/target_base_move"

TARGET_BASE = "io_teleop/target_base_move"
TARGET_COMMAND = "io_teleop/joint_cmd"
TARGET_JOINT_STATE = "io_teleop/joint_states"
TARGET_WRENCH = "/io_teleop/state_ee_wrenches"
TARGET_VIBRATION = "/io_teleop/vibration_feedback_exo_during_sync"
TARGET_HAND_TRIGGER = "/io_teleop/hand_trigger"
TARGET_HAND_STATE = "/io_teleop/hand_joint_states"
TARGET_GRIPPER = "/io_teleop/gripper_state"
TARGET_LEFT_FINGER = "/io_teleop/joint_cmd_finger_left"
TARGET_RIGHT_FINGER = "/io_teleop/joint_cmd_finger_right"

STATIC_REFERENCE_TOPICS = (
    "io_teleop/robot_info",
    "/io_teleop/camera_head/extrinsics",
    "/io_teleop/camera_head/calibration",
)
STATIC_REFERENCE_TOPIC_SET = frozenset(STATIC_REFERENCE_TOPICS)

REQUIRED_TARGET_TOPICS = frozenset(
    {
        "io_teleop/robot_info",
        TARGET_BASE,
        TARGET_COMMAND,
        TARGET_JOINT_STATE,
        "io_teleop/target_ee_poses",
        "/io_teleop/state_ee_poses",
        TARGET_WRENCH,
        TARGET_VIBRATION,
        TARGET_HAND_TRIGGER,
        TARGET_HAND_STATE,
        TARGET_GRIPPER,
        TARGET_LEFT_FINGER,
        TARGET_RIGHT_FINGER,
        "/io_teleop/camera_head/extrinsics",
        "/io_teleop/camera_head/calibration",
        "/io_teleop/camera_head/color",
        "/io_teleop/camera_head/depth",
    }
)


class ReprofileError(RuntimeError):
    """Raised when an input cannot satisfy the requested output profile."""


def _parse_sftp_source(value: str) -> Tuple[str, str, Optional[int]]:
    parsed = urlparse(value)
    if parsed.scheme != "sftp":
        raise ReprofileError(f"unsupported source URL scheme: {parsed.scheme}")
    if not parsed.hostname or not parsed.path:
        raise ReprofileError("SFTP source must include a host and absolute path")
    if parsed.password is not None:
        raise ReprofileError(
            "do not put an SFTP password in the URL; use HC_MCAP_SFTP_PASSWORD"
        )
    if parsed.query or parsed.fragment:
        raise ReprofileError("SFTP source must not contain a query or fragment")
    host = parsed.hostname
    if ":" in host:
        host = f"[{host}]"
    authority = f"{parsed.username}@{host}" if parsed.username else host
    return authority, unquote(parsed.path), parsed.port


@contextmanager
def _materialize_source(value: str) -> Iterator[Path]:
    if not value.startswith("sftp://"):
        yield Path(value)
        return

    authority, remote_path, port = _parse_sftp_source(value)
    with tempfile.TemporaryDirectory(prefix="hc-mcap-sftp-") as directory:
        local_path = Path(directory) / Path(remote_path).name
        command = [
            "scp",
            "-q",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            "ConnectTimeout=10",
        ]
        if port is not None:
            command.extend(["-P", str(port)])
        command.extend([f"{authority}:{remote_path}", str(local_path)])

        environment = os.environ.copy()
        password = environment.get("HC_MCAP_SFTP_PASSWORD")
        if password:
            sshpass = shutil.which("sshpass")
            if sshpass is None:
                raise ReprofileError(
                    "HC_MCAP_SFTP_PASSWORD is set but sshpass is not installed"
                )
            environment["SSHPASS"] = password
            command = [sshpass, "-e", *command]
        try:
            subprocess.run(command, check=True, env=environment)
        except (OSError, subprocess.CalledProcessError) as error:
            raise ReprofileError(f"failed to download SFTP source: {value}") from error
        yield local_path


def _normal_ros_type(name: str) -> str:
    return name.replace("/msg/", "/")


def _stamp_ns(message: Any, fallback: int) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _set_stamp(message: Any, timestamp_ns: int) -> None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return
    stamp.sec, stamp.nanosec = divmod(int(timestamp_ns), 1_000_000_000)


def _field_map(message: Any) -> Dict[str, float]:
    return {
        str(name): float(position)
        for name, position in zip(message.name, message.position)
    }


class _ReferenceProfile:
    def __init__(self, path: Path):
        self.path = path
        self.header: Any = None
        self.schemas: Dict[int, Schema] = {}
        self.channels: Dict[int, Channel] = {}
        self.channels_by_topic: Dict[str, Channel] = {}
        self.first_messages: Dict[str, Message] = {}
        self.templates: Dict[str, Any] = {}
        self.metadata: list[Any] = []
        self._encoders: Dict[int, Callable[[Any], bytes]] = {}
        self._decoders: Dict[int, Callable[[bytes], Any]] = {}

        decoder_factory = DecoderFactory()
        with path.open("rb") as stream:
            reader = make_reader(stream)
            self.header = reader.get_header()
            summary = reader.get_summary()
            if summary is None:
                raise ReprofileError(f"reference MCAP has no summary: {path}")
            self.schemas = dict(summary.schemas)
            self.channels = dict(summary.channels)
            self.channels_by_topic = {
                channel.topic: channel for channel in self.channels.values()
            }
            self.metadata = list(reader.iter_metadata())

            missing = sorted(REQUIRED_TARGET_TOPICS - self.channels_by_topic.keys())
            if missing:
                raise ReprofileError(
                    "reference is not an hc_tj_description profile; missing topics: "
                    + ", ".join(missing)
                )

            for schema, channel, message in reader.iter_messages(log_time_order=True):
                if channel.topic in self.first_messages:
                    continue
                self.first_messages[channel.topic] = message
                if schema is not None:
                    decoder = decoder_factory.decoder_for(
                        channel.message_encoding, schema
                    )
                    if decoder is not None:
                        self._decoders[schema.id] = decoder
                        self.templates[channel.topic] = decoder(message.data)

        missing_samples = sorted(REQUIRED_TARGET_TOPICS - self.first_messages.keys())
        if missing_samples:
            raise ReprofileError(
                "reference topics have no template messages: " + ", ".join(missing_samples)
            )

    def schema_for_topic(self, topic: str) -> Schema:
        channel = self.channels_by_topic[topic]
        return self.schemas[channel.schema_id]

    def encode(self, topic: str, message: Any) -> bytes:
        schema = self.schema_for_topic(topic)
        encoder = self._encoders.get(schema.id)
        if encoder is None:
            encoders = serialize_dynamic(schema.name, schema.data.decode())
            try:
                encoder = encoders[schema.name]
            except KeyError as error:
                raise ReprofileError(
                    f"cannot create encoder for reference schema {schema.name}"
                ) from error
            self._encoders[schema.id] = encoder
        return encoder(message)

    def clone_template(self, topic: str) -> Any:
        # mcap_ros2 0.5.x dynamic classes use slots and do not implement a
        # working deepcopy protocol. Re-decoding is deterministic and gives a
        # fully independent message object.
        channel = self.channels_by_topic[topic]
        return self._decoders[channel.schema_id](self.first_messages[topic].data)


class _Output:
    def __init__(self, path: Path, profile: _ReferenceProfile):
        self.writer = Writer(str(path), compression=CompressionType.ZSTD)
        self.writer.start(
            profile=profile.header.profile,
            library=profile.header.library,
        )
        schema_ids: Dict[int, int] = {}
        for old_id, schema in sorted(profile.schemas.items()):
            schema_ids[old_id] = self.writer.register_schema(
                schema.name, schema.encoding, schema.data
            )
        self.channel_ids: Dict[str, int] = {}
        for _, channel in sorted(profile.channels.items()):
            self.channel_ids[channel.topic] = self.writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_ids[channel.schema_id],
                metadata=dict(channel.metadata),
            )
        for item in profile.metadata:
            self.writer.add_metadata(item.name, dict(item.metadata))
        self.counts = {topic: 0 for topic in self.channel_ids}

    def add(
        self,
        topic: str,
        timestamp_ns: int,
        data: bytes,
        *,
        publish_time_ns: Optional[int] = None,
        sequence: int = 0,
    ) -> None:
        self.writer.add_message(
            channel_id=self.channel_ids[topic],
            log_time=int(timestamp_ns),
            publish_time=int(
                timestamp_ns if publish_time_ns is None else publish_time_ns
            ),
            sequence=int(sequence),
            data=data,
        )
        self.counts[topic] += 1

    def finish(self) -> None:
        self.writer.finish()


def _validate_direct_type(
    source_schema: Optional[Schema], target_schema: Schema, source_topic: str
) -> None:
    if source_schema is None:
        raise ReprofileError(f"source topic has no schema: {source_topic}")
    if _normal_ros_type(source_schema.name) != _normal_ros_type(target_schema.name):
        raise ReprofileError(
            f"incompatible mapping for {source_topic}: "
            f"{source_schema.name} -> {target_schema.name}"
        )


def _zero_array(profile: _ReferenceProfile, topic: str) -> bytes:
    message = profile.clone_template(topic)
    message.data = [0.0] * len(message.data)
    return profile.encode(topic, message)


def _joint_message(
    profile: _ReferenceProfile,
    topic: str,
    values: Mapping[str, float],
    timestamp_ns: int,
) -> Any:
    output = profile.clone_template(topic)
    _set_stamp(output, timestamp_ns)
    output.position = [float(values.get(str(name), 0.0)) for name in output.name]
    output.velocity = []
    output.effort = []
    return output


def convert_mcap(
    source_path: Path,
    reference_path: Path,
    output_path: Path,
    *,
    auxiliary_rate_hz: float = 10.0,
    overwrite: bool = False,
) -> Dict[str, Any]:
    """Convert one HC recording to the channel contract of ``reference_path``."""
    source_path = source_path.expanduser().resolve()
    reference_path = reference_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if not source_path.is_file():
        raise ReprofileError(f"source MCAP does not exist: {source_path}")
    if not reference_path.is_file():
        raise ReprofileError(f"reference MCAP does not exist: {reference_path}")
    if output_path in {source_path, reference_path}:
        raise ReprofileError("output must not overwrite an input MCAP")
    if output_path.exists() and not overwrite:
        raise ReprofileError(f"output already exists (use --overwrite): {output_path}")
    if not auxiliary_rate_hz > 0.0:
        raise ReprofileError("auxiliary_rate_hz must be positive")

    profile = _ReferenceProfile(reference_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = tempfile.NamedTemporaryFile(
        prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent, delete=False
    )
    temporary_path = Path(temporary.name)
    temporary.close()

    source_decoder = DecoderFactory()
    decoder_cache: Dict[int, Callable[[bytes], Any]] = {}
    latest_joint_state: Dict[str, float] = {}
    latest_left_hand: Dict[str, float] = {}
    latest_right_hand: Dict[str, float] = {}
    latest_base = _zero_array(profile, TARGET_BASE)
    zero_wrench = _zero_array(profile, TARGET_WRENCH)
    zero_vibration = _zero_array(profile, TARGET_VIBRATION)
    auxiliary_period_ns = int(round(1_000_000_000 / auxiliary_rate_hz))
    next_auxiliary_ns: Optional[int] = None
    source_counts: Dict[str, int] = {}

    def decode(schema: Optional[Schema], channel: Channel, message: Message) -> Any:
        if schema is None:
            raise ReprofileError(f"source topic has no schema: {channel.topic}")
        decoder = decoder_cache.get(schema.id)
        if decoder is None:
            value = source_decoder.decoder_for(channel.message_encoding, schema)
            if value is None:
                raise ReprofileError(
                    f"cannot decode {channel.topic} ({schema.name}/{channel.message_encoding})"
                )
            decoder = value
            decoder_cache[schema.id] = decoder
        return decoder(message.data)

    try:
        with source_path.open("rb") as source_stream:
            source_reader = make_reader(source_stream)
            source_summary = source_reader.get_summary()
            if source_summary is None or source_summary.statistics is None:
                raise ReprofileError(f"source MCAP has no statistics: {source_path}")
            source_topics = {
                channel.topic for channel in source_summary.channels.values()
            }
            start_time = int(source_summary.statistics.message_start_time)
            end_time = int(source_summary.statistics.message_end_time)
            sink = _Output(temporary_path, profile)

            for topic in STATIC_REFERENCE_TOPICS:
                sample = profile.first_messages[topic]
                sink.add(topic, start_time, sample.data, publish_time_ns=start_time)
            if (
                TARGET_BASE not in source_topics
                and SOURCE_BASE_TARGET not in source_topics
            ):
                # Short/aborted recordings occasionally contain no base
                # sample at all. Keep every target channel populated with an
                # explicit neutral command instead of emitting an empty topic.
                sink.add(
                    TARGET_BASE,
                    start_time,
                    latest_base,
                    publish_time_ns=start_time,
                )

            for schema, channel, message in source_reader.iter_messages(
                log_time_order=True
            ):
                topic = channel.topic
                source_counts[topic] = source_counts.get(topic, 0) + 1

                # A previously converted recording may already use the target
                # topic names. Keep conversion idempotent by copying those
                # payloads directly while still rebuilding Header, schemas,
                # channels and QoS metadata from the selected reference.
                target = None
                if (
                    topic in profile.channels_by_topic
                    and topic not in STATIC_REFERENCE_TOPIC_SET
                ):
                    target = topic
                else:
                    target = DIRECT_TOPIC_MAP.get(topic)
                if target is not None:
                    _validate_direct_type(schema, profile.schema_for_topic(target), topic)
                    sink.add(
                        target,
                        message.log_time,
                        message.data,
                        publish_time_ns=message.publish_time,
                        sequence=message.sequence,
                    )

                if topic in {SOURCE_BASE_TARGET, TARGET_BASE}:
                    _validate_direct_type(
                        schema, profile.schema_for_topic(TARGET_BASE), topic
                    )
                    latest_base = message.data

                if topic == "/hc_teleop/target_ee_poses":
                    sink.add(
                        TARGET_BASE,
                        message.log_time,
                        latest_base,
                        publish_time_ns=message.publish_time,
                        sequence=message.sequence,
                    )

                if topic in {SOURCE_LEFT_HAND, TARGET_LEFT_FINGER}:
                    latest_left_hand = _field_map(decode(schema, channel, message))
                elif topic in {SOURCE_RIGHT_HAND, TARGET_RIGHT_FINGER}:
                    latest_right_hand = _field_map(decode(schema, channel, message))

                if topic in {"/hc_teleop/joint_states", TARGET_JOINT_STATE}:
                    decoded_state = decode(schema, channel, message)
                    latest_joint_state = _field_map(decoded_state)
                    if TARGET_WRENCH not in source_topics:
                        sink.add(
                            TARGET_WRENCH,
                            message.log_time,
                            zero_wrench,
                            publish_time_ns=message.publish_time,
                            sequence=message.sequence,
                        )

                    if next_auxiliary_ns is None:
                        next_auxiliary_ns = message.log_time
                    if message.log_time >= next_auxiliary_ns:
                        sample_time = _stamp_ns(decoded_state, message.log_time)
                        trigger = profile.clone_template(TARGET_HAND_TRIGGER)
                        _set_stamp(trigger, sample_time)
                        # The source has no VR trigger channel. Preserve the
                        # reference shape and explicitly emit neutral values.
                        for pose in trigger.poses:
                            pose.position.x = pose.position.y = pose.position.z = 0.0
                            pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0
                            pose.orientation.w = 1.0
                        if TARGET_HAND_TRIGGER not in source_topics:
                            sink.add(
                                TARGET_HAND_TRIGGER,
                                message.log_time,
                                profile.encode(TARGET_HAND_TRIGGER, trigger),
                            )
                        if TARGET_VIBRATION not in source_topics:
                            sink.add(
                                TARGET_VIBRATION, message.log_time, zero_vibration
                            )

                        if latest_left_hand and latest_right_hand:
                            hand_values = dict(latest_right_hand)
                            hand_values.update(latest_left_hand)
                            for target_topic in (
                                TARGET_HAND_STATE,
                                TARGET_LEFT_FINGER,
                                TARGET_RIGHT_FINGER,
                            ):
                                if target_topic in source_topics:
                                    continue
                                derived = _joint_message(
                                    profile, target_topic, hand_values, sample_time
                                )
                                sink.add(
                                    target_topic,
                                    message.log_time,
                                    profile.encode(target_topic, derived),
                                )

                            if TARGET_GRIPPER not in source_topics:
                                gripper_values = {
                                    "R_hand": latest_right_hand.get("R_ban", 0.0),
                                    "L_hand": latest_left_hand.get("L_ban", 0.0),
                                }
                                gripper = _joint_message(
                                    profile,
                                    TARGET_GRIPPER,
                                    gripper_values,
                                    sample_time,
                                )
                                sink.add(
                                    TARGET_GRIPPER,
                                    message.log_time,
                                    profile.encode(TARGET_GRIPPER, gripper),
                                )
                        while next_auxiliary_ns <= message.log_time:
                            next_auxiliary_ns += auxiliary_period_ns

                if topic == SOURCE_JOINT_COMMAND:
                    decoded_command = decode(schema, channel, message)
                    command_values = dict(latest_joint_state)
                    command_values.update(_field_map(decoded_command))
                    command_time = _stamp_ns(decoded_command, message.log_time)
                    command = _joint_message(
                        profile, TARGET_COMMAND, command_values, command_time
                    )
                    sink.add(
                        TARGET_COMMAND,
                        message.log_time,
                        profile.encode(TARGET_COMMAND, command),
                        publish_time_ns=message.publish_time,
                        sequence=message.sequence,
                    )

            sink.finish()

        if output_path.exists():
            output_path.unlink()
        os.replace(temporary_path, output_path)

        validation = inspect_compatibility(output_path, reference_path)
        if not validation["compatible"]:
            raise ReprofileError(
                "output failed profile validation: "
                + "; ".join(validation["differences"])
            )
        return {
            "profile": PROFILE_NAME,
            "source": str(source_path),
            "reference": str(reference_path),
            "output": str(output_path),
            "source_start_time_ns": start_time,
            "source_end_time_ns": end_time,
            "source_counts": source_counts,
            "output_counts": sink.counts,
            "compatibility": validation,
        }
    except Exception:
        if temporary_path.exists():
            temporary_path.unlink()
        raise


def _structure(path: Path) -> Tuple[Any, Dict[str, Tuple[Any, ...]]]:
    with path.open("rb") as stream:
        reader = make_reader(stream)
        header = reader.get_header()
        summary = reader.get_summary()
        if summary is None:
            raise ReprofileError(f"MCAP has no summary: {path}")
        result = {}
        for channel in summary.channels.values():
            schema = summary.schemas[channel.schema_id]
            result[channel.topic] = (
                schema.name,
                schema.encoding,
                schema.data,
                channel.message_encoding,
                dict(channel.metadata),
            )
        return header, result


def inspect_compatibility(candidate: Path, reference: Path) -> Dict[str, Any]:
    """Compare Header, topic, schema and channel metadata with a reference."""
    candidate_header, candidate_channels = _structure(candidate)
    reference_header, reference_channels = _structure(reference)
    differences = []
    if candidate_header.profile != reference_header.profile:
        differences.append(
            f"profile {candidate_header.profile!r} != {reference_header.profile!r}"
        )
    if candidate_header.library != reference_header.library:
        differences.append(
            f"library {candidate_header.library!r} != {reference_header.library!r}"
        )
    candidate_topics = set(candidate_channels)
    reference_topics = set(reference_channels)
    if candidate_topics != reference_topics:
        differences.append(
            f"topics differ: missing={sorted(reference_topics - candidate_topics)} "
            f"extra={sorted(candidate_topics - reference_topics)}"
        )
    for topic in sorted(candidate_topics & reference_topics):
        if candidate_channels[topic] != reference_channels[topic]:
            differences.append(f"channel/schema differs: {topic}")
    return {
        "compatible": not differences,
        "header": {
            "profile": candidate_header.profile,
            "library": candidate_header.library,
        },
        "topic_count": len(candidate_channels),
        "differences": differences,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Reprofile a legacy HC MCAP using the exact Header, schemas and "
            "channels of an hc_tj_description reference MCAP."
        )
    )
    parser.add_argument(
        "source", help="local path or sftp://user@host/path source MCAP"
    )
    parser.add_argument("reference", type=Path, help="reference/profile MCAP")
    parser.add_argument("output", type=Path, help="output MCAP")
    parser.add_argument(
        "--auxiliary-rate",
        type=float,
        default=10.0,
        help="derived hand/trigger/vibration rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--overwrite", action="store_true", help="replace an existing output file"
    )
    parser.add_argument(
        "--json", action="store_true", help="print the complete conversion report"
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        with _materialize_source(args.source) as source_path:
            report = convert_mcap(
                source_path,
                args.reference,
                args.output,
                auxiliary_rate_hz=args.auxiliary_rate,
                overwrite=args.overwrite,
            )
            report["source"] = args.source
    except (OSError, ReprofileError, ValueError) as error:
        print(f"hc_mcap_reprofile: error: {error}", file=os.sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        compatibility = report["compatibility"]
        print(f"output: {report['output']}")
        print(
            "profile: "
            f"{compatibility['header']['profile']} / "
            f"{compatibility['header']['library']}"
        )
        print(f"topics: {compatibility['topic_count']} (exact reference match)")
        print(f"messages: {sum(report['output_counts'].values())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
