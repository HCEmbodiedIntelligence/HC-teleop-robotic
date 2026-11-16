"""Read ROS CDR and JSON MCAP without importing a ROS runtime."""

import base64
import io
import json

from .errors import DataError


def field(value, key, default=None):
    return (
        value.get(key, default)
        if isinstance(value, dict)
        else getattr(value, key, default)
    )


def iter_samples(path, topics=None):
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        raise DataError(
            "MCAP_DECODER_UNAVAILABLE", "Install mcap and mcap-ros2-support"
        ) from None
    factory = DecoderFactory()
    with open(path, "rb") as stream:
        for schema, channel, message in make_reader(
            stream, validate_crcs=True
        ).iter_messages(topics=topics):
            if channel.message_encoding == "json":
                payload = json.loads(message.data)
            elif channel.message_encoding == "cdr":
                decode = factory.decoder_for("cdr", schema)
                if decode is None:
                    raise DataError("MCAP_SCHEMA_UNSUPPORTED", channel.topic)
                payload = decode(message.data)
                if schema.name in {"std_msgs/msg/String", "std_msgs/String"}:
                    try:
                        payload = json.loads(payload.data)
                    except ValueError:
                        continue
            else:
                continue
            received_mono = None
            if (
                isinstance(payload, dict)
                and payload.get("schema_version") == "hc.sample/v1"
            ):
                received_mono = payload.get("received_monotonic_ns")
                payload = payload["payload"]
            yield {
                "timestamp_ns": message.log_time,
                "source_ns": message.publish_time,
                "topic": channel.topic,
                "ros_type": schema.name if schema else "",
                "payload": payload,
                "received_monotonic_ns": received_mono,
            }


def decode_image(payload):
    try:
        from PIL import Image
    except ImportError:
        raise DataError("IMAGE_DECODER_UNAVAILABLE", "Install Pillow") from None
    raw = field(payload, "data_base64")
    if raw is not None:
        raw = base64.b64decode(raw, validate=True)
    else:
        raw = field(payload, "data")
        raw = (
            base64.b64decode(raw, validate=True)
            if isinstance(raw, str)
            else bytes(raw or b"")
        )
    encoding = field(payload, "encoding", field(payload, "format", ""))
    width, height = field(payload, "width"), field(payload, "height")
    if (
        not width
        or not height
        or "jpeg" in encoding
        or "jpg" in encoding
        or "png" in encoding
    ):
        image = Image.open(io.BytesIO(raw))
        image.load()
        return image.convert("RGB")
    if not 1 <= int(width) <= 8192 or not 1 <= int(height) <= 8192:
        raise DataError("IMAGE_DIMENSIONS_INVALID")
    formats = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "mono8": ("L", "L", 1),
    }
    if encoding not in formats:
        raise DataError("IMAGE_ENCODING_UNSUPPORTED", str(encoding))
    mode, raw_mode, channels = formats[encoding]
    stride = int(field(payload, "step", int(width) * channels))
    if stride < width * channels or len(raw) < stride * height:
        raise DataError("IMAGE_BUFFER_INVALID")
    return Image.frombytes(
        mode, (int(width), int(height)), raw, "raw", raw_mode, stride, 1
    ).convert("RGB")


def is_image(sample):
    value = sample["payload"]
    return sample["ros_type"] in {
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/CompressedImage",
    } or (
        isinstance(value, dict)
        and ("data_base64" in value or "data" in value)
        and ("encoding" in value or "format" in value)
    )
