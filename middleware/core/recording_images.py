"""Index live RGB compressed messages without decoding or retaining camera payloads."""
from io import BytesIO

from .capture_window import header_stamp_ns
from .recording_cdr import Reader


def compressed_fields(raw):
    reader = Reader(raw)
    reader.offset = 12
    frame_id = reader.string()
    image_format = reader.string()
    if 'depth' in image_format.lower():
        raise ValueError('Depth images are not RGB observations')
    size = reader.uint()
    end = reader.offset + size
    if not size or end > len(raw):
        raise ValueError('Invalid ROS CompressedImage data length')
    payload = memoryview(raw)[reader.offset:end]
    stamp = header_stamp_ns(raw, 'sensor_msgs/msg/CompressedImage')
    return frame_id, stamp, payload


def rgb_shape(payload):
    """Read the encoded dimensions just like the live Pillow RGB decoder."""
    from PIL import Image
    # Standard ROS JPEG and PNG dimensions are available in the first few KB.
    # Large EXIF/ICC records may push the JPEG frame header beyond that prefix.
    try:
        with Image.open(BytesIO(payload[:4096])) as image:
            return [image.height, image.width, 3]
    except OSError:
        if len(payload) <= 4096:
            raise
    with Image.open(BytesIO(payload)) as image:
        return [image.height, image.width, 3]


def stage_selected_images(source, staging, topics_by_feature, episodes):
    """Replace selected camera ordinals with shared references to the original encoded images."""
    from mcap.reader import make_reader
    selected = {key: {} for key in topics_by_feature}
    for episode in episodes:
        for row in episode:
            values = row['values']
            for key, paths in selected.items():
                ordinal = values[key]
                if isinstance(ordinal, int):
                    path = paths.get(ordinal)
                    if path is None:
                        path = str(staging / key / f'{ordinal:09d}.image')
                        paths[ordinal] = path
                    values[key] = path
    wanted = {topics_by_feature[key]: paths for key, paths in selected.items() if paths}
    counts = {key: len(paths) for key, paths in selected.items()}
    if not wanted:
        return counts
    ordinal_by_topic = dict.fromkeys(wanted, 0)
    with source.open('rb') as stream:
        for schema, channel, msg in make_reader(stream).iter_messages(topics=list(wanted), log_time_order=True):
            topic = channel.topic
            if channel.message_encoding != 'cdr' or schema is None or schema.name != 'sensor_msgs/msg/CompressedImage':
                continue
            ordinal = ordinal_by_topic[topic]
            ordinal_by_topic[topic] += 1
            path = wanted[topic].pop(ordinal, None)
            if path is not None:
                _, _, payload = compressed_fields(msg.data)
                with open(path, 'wb') as output:
                    output.write(payload)
    if any(wanted.values()):
        raise ValueError('Source MCAP changed during camera selection')
    return counts
