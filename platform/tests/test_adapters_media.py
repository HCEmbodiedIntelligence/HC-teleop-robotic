import asyncio
from fractions import Fraction
import json
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from hc_platform.camera_metadata import CameraMetadata
from hc_platform.media import Media, Rate, create_media_app


def config(backend="x264"):
    return {
        "camera": {
            "encoder_backend": backend,
            "streams": [
                {
                    "id": "head",
                    "topic": "/image",
                    "type": "sensor_msgs/msg/CompressedImage",
                    "width": 160,
                    "height": 120,
                    "fps": 12,
                    "enabled": True,
                }
            ],
        },
        "topics": [
            {
                "topic": "/record_only",
                "type": "sensor_msgs/msg/Image",
                "outputs": ["record"],
                "min_hz": 10,
            }
        ],
    }


def image_message():
    import cv2

    image = np.zeros((240, 320, 3), np.uint8)
    image[:, :, 1] = 160
    image[20:80, 40:150, 2] = 220
    ok, data = cv2.imencode(".jpg", image)
    assert ok
    return NS(data=data.tobytes(), format="jpeg")


def test_record_only_topics_are_measured_without_reading_payload():
    media = Media(config())

    class Untouchable:
        @property
        def data(self):
            raise AssertionError("record-only stats must not touch the payload")

    for _ in range(3):
        media.observe(Untouchable(), media.topic_config[0])
    result = media.status()
    assert result["topic_metrics"]["/record_only"]["received"] == 3
    assert result["topic_metrics"]["/record_only"]["age_ms"] is not None
    assert result["observations"] == {}
    now = time.monotonic()
    rate = Rate()
    for stamp in (now - 0.2, now - 0.1, now):
        rate.tick(stamp)
    assert rate.status(now)["actual_hz"] == pytest.approx(10)
    assert rate.status(now + 3)["actual_hz"] == 0
    asyncio.run(media.close())


def test_camera_resize_raw_layout_and_never_claim_encoder_before_use():
    media = Media(config("nvenc"))
    media.receive("head", image_message())
    image, frame = media.snapshot("head")
    assert image.shape == (120, 160, 3)
    assert media.status()["streams"][0]["encoder_actual"] is None
    media.receive("head", NS(data=b"x", encoding="rgb8", width=10, height=10, step=30))
    with pytest.raises(ValueError, match="INVALID_RAW_IMAGE_LAYOUT"):
        media.snapshot("head")
    asyncio.run(media.close())


def transform(parent, child, translation):
    return NS(
        header=NS(frame_id=parent),
        child_frame_id=child,
        transform=NS(
            translation=NS(x=translation[0], y=translation[1], z=translation[2]),
            rotation=NS(x=0, y=0, z=0, w=1),
        ),
    )


def test_metadata_preserves_provenance_and_resolves_camera_info_tf():
    metadata = CameraMetadata()
    metadata.json_message(
        "head",
        "extrinsics",
        "/model",
        json.dumps({"source": "model_fk", "calibrated": True}),
    )
    entry = metadata.status("head")["selected"]["extrinsics"]
    assert entry["calibrated"] is False and entry["calibration_verified"] is False
    metadata.json_message("head", "extrinsics", "/bad", '{"x":NaN}')
    assert "/bad" in metadata.status("head")["errors"]
    metadata.json_message("head", "extrinsics", "/huge_float", '{"x":1e999}')
    assert "/huge_float" in metadata.status("head")["errors"]
    info = NS(
        k=[100, 0, 50, 0, 100, 50, 0, 0, 1],
        p=[100, 0, 50, 0, 0, 100, 50, 0, 0, 0, 1, 0],
        r=[1, 0, 0, 0, 1, 0, 0, 0, 1],
        d=[],
        width=100,
        height=100,
        distortion_model="plumb_bob",
        header=NS(frame_id="optical", stamp=NS(sec=1, nanosec=2)),
    )
    metadata.camera_info("head", "/camera_info", info)
    metadata.tf_message(
        NS(
            transforms=[
                transform("base", "mount", [1, 0, 0]),
                transform("mount", "optical", [0, 2, 0]),
            ]
        ),
        True,
    )
    result = metadata.status("head", "base")
    assert result["selected"]["intrinsics"]["calibrated"] is True
    assert result["selected"]["intrinsics"]["physical_calibration_verified"] is False
    tf = next(source for source in result["sources"] if source["source"] == "tf")
    assert np.array(tf["payload"]["T_parent_child"])[:3, 3] == pytest.approx([1, 2, 0])
    assert tf["calibrated"] is None and tf["source_topic"] == "/tf_static"


@pytest.mark.parametrize("backend", ["x264", "nvenc", "v4l2", "vaapi"])
def test_real_encoder_reports_actual_success_or_actual_fallback(backend):
    av = pytest.importorskip("av")
    from hc_platform.video_encoding import AdaptiveH264Encoder

    statuses = []
    encoder = AdaptiveH264Encoder(backend, 12, on_status=statuses.append)
    assert statuses[-1]["actual"] is None
    packets = []
    for index in range(4):
        frame = av.VideoFrame.from_ndarray(
            np.full((120, 160, 3), index * 20, np.uint8), format="bgr24"
        )
        frame.pts = index * 7500
        frame.time_base = Fraction(1, 90000)
        packets.extend(encoder.encode(frame)[0])
    assert packets and statuses[-1]["actual"] in (
        "libx264",
        "h264_nvenc",
        "h264_v4l2m2m",
    )
    assert statuses[-1]["frames_encoded"] > 0
    if backend == "x264":
        assert statuses[-1]["fallback_reason"] is None
    if backend == "vaapi":
        assert "not implemented" in statuses[-1]["fallback_reason"]
    if backend in ("nvenc", "v4l2") and statuses[-1]["actual"] == "libx264":
        assert statuses[-1]["fallback_reason"]


@pytest.mark.parametrize("backend", ["x264", "nvenc"])
def test_real_webrtc_loopback_resolution_fps_repair_and_receiver_session(backend):
    pytest.importorskip("aiortc")

    async def run():
        import httpx
        from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration
        from hc_platform.video_encoding import negotiated_repair

        app = create_media_app(config(backend))
        media = app.state.media
        client_peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        track_future = asyncio.get_running_loop().create_future()

        @client_peer.on("track")
        def got_track(track):
            if not track_future.done():
                track_future.set_result(track)

        async def feed():
            message = image_message()
            while True:
                media.receive("head", message)
                await asyncio.sleep(0.025)

        feed_task = asyncio.create_task(feed())
        client_peer.addTransceiver("video", direction="recvonly")
        try:
            offer = await client_peer.createOffer()
            await client_peer.setLocalDescription(offer)
            transport = httpx.ASGITransport(app=app)
            async with httpx.AsyncClient(
                transport=transport, base_url="http://media"
            ) as client:
                response = await client.post(
                    "/offer",
                    json={
                        "sdp": client_peer.localDescription.sdp,
                        "type": "offer",
                        "camera_ids": ["head"],
                    },
                )
                assert response.status_code == 200, response.text
                answer = response.json()
                repair = negotiated_repair(answer["sdp"])
                assert (
                    repair["rtx"]
                    and repair["nack"]
                    and repair["pli"]
                    and not repair["fec"]
                )
                await client_peer.setRemoteDescription(
                    RTCSessionDescription(answer["sdp"], answer["type"])
                )
                track = await asyncio.wait_for(track_future, 5)
                frames = [await asyncio.wait_for(track.recv(), 8) for _ in range(8)]
                assert all(
                    (frame.width, frame.height) == (160, 120) for frame in frames
                )
                timestamps = [float(frame.pts * frame.time_base) for frame in frames]
                # The sender intentionally skips missed slots after a slow
                # first hardware-encoder frame instead of building latency.
                # PTS must remain on the configured 12 Hz clock, without bursts.
                intervals = np.diff(timestamps) * 12
                assert np.all(intervals >= 0.99)
                assert intervals == pytest.approx(np.round(intervals), abs=0.024)
                assert timestamps[-1] - timestamps[0] < 2
                status = (await client.get("/status")).json()
                stream = status["streams"][0]
                assert stream["state"] == "running" and stream["capture_fps"] > 20
                assert 8 < stream["webrtc_send_fps"] < 16
                assert (
                    stream["webrtc_send_fps_basis"]
                    == "complete_frame_dispatched_to_dtls_transport"
                )
                assert stream["encoder_actual"] in ("libx264", "h264_nvenc")
                report = {
                    "session_id": answer["telemetry_session_id"],
                    "sequence": 0,
                    "sample_ms": 1000,
                    "stats_id": "incoming-video",
                    "packets_received": 20,
                    "packets_lost": -1,
                    "frames_decoded": 8,
                }
                assert (
                    await client.post("/receiver/head", json=report)
                ).status_code == 200
                assert (
                    await client.post("/receiver/head", json=report)
                ).status_code == 409
                assert (
                    await client.post(
                        "/receiver/other", json=dict(report, sequence=1, sample_ms=1100)
                    )
                ).status_code == 409
                assert (
                    await client.post(
                        "/receiver/head", json=dict(report, session_id="wrong")
                    )
                ).status_code == 409
                assert (
                    await client.post(
                        "/receiver/head", json=dict(report, session_id={})
                    )
                ).status_code == 400
                assert (
                    await client.post(
                        "/receiver/head",
                        json=dict(report, sequence=1, sample_ms=1100, frames_decoded=9),
                    )
                ).status_code == 200
                latest = (await client.get("/status")).json()["streams"][0][
                    "receivers"
                ][0]
                assert latest["decoded_fps"] == pytest.approx(10)
        finally:
            feed_task.cancel()
            await asyncio.gather(feed_task, return_exceptions=True)
            await client_peer.close()
            await media.close()

    asyncio.run(run())


def test_ros_record_only_camera_metadata_and_missing_type_are_isolated(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    monkeypatch.setenv("ROS_DOMAIN_ID", "197")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    settings = config()
    settings["camera"]["streams"][0]["metadata"] = {"parent_frame": "base"}
    settings["topics"].append(
        {
            "topic": "/missing_type",
            "type": "no_such_hc_package/msg/Missing",
            "outputs": ["record"],
        }
    )
    settings["camera"]["streams"].append(
        {
            "id": "bad",
            "topic": "/bad_camera",
            "type": "no_such_hc_package/msg/Image",
            "width": 160,
            "height": 120,
            "fps": 12,
        }
    )
    media = Media(settings)
    media.start()
    context = rclpy.context.Context()
    rclpy.init(context=context)
    producer = rclpy.create_node("media_test_producer", context=context)
    try:
        assert media.reason is None
        assert media.status()["ros_available"]
        assert media.status()["configuration_errors"]
        pub_record = producer.create_publisher(Image, "/record_only", 1)
        pub_image = producer.create_publisher(CompressedImage, "/image", 1)
        pub_info = producer.create_publisher(
            CameraInfo, "/observation/cameras/head/camera_info", 1
        )
        pub_json = producer.create_publisher(
            String, "/observation/cameras/head/calibration", 1
        )
        pub_tf = producer.create_publisher(
            TFMessage,
            "/tf_static",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        image = Image()
        image.width = 320
        image.height = 240
        image.step = 320 * 3
        image.encoding = "bgr8"
        image.data = bytes(320 * 240 * 3)
        compressed = CompressedImage()
        compressed.format = "jpeg"
        compressed.data = image_message().data
        info = CameraInfo()
        info.width = 320
        info.height = 240
        info.header.frame_id = "optical"
        info.k = [200.0, 0.0, 160.0, 0.0, 200.0, 120.0, 0.0, 0.0, 1.0]
        calibration = String()
        calibration.data = json.dumps(
            {
                "source": "hardware_factory_intrinsics",
                "calibrated": True,
                "K": list(info.k),
            }
        )
        transform_msg = TransformStamped()
        transform_msg.header.frame_id = "base"
        transform_msg.child_frame_id = "optical"
        transform_msg.transform.rotation.w = 1.0
        transform_msg.transform.translation.z = 0.3
        tf = TFMessage()
        tf.transforms = [transform_msg]
        for _ in range(36):
            pub_record.publish(image)
            pub_image.publish(compressed)
            pub_info.publish(info)
            pub_json.publish(calibration)
            pub_tf.publish(tf)
            time.sleep(0.04)
        result = media.status()
        assert result["topic_metrics"]["/record_only"]["received"] > 10
        assert result["topic_metrics"]["/record_only"]["actual_hz"] > 10
        assert result["topic_metrics"]["/missing_type"]["state"] == "error"
        head = result["streams"][0]
        assert head["capture_fps"] > 10 and head["state"] == "running"
        assert "intrinsics" in head["metadata"]["selected"]
        assert (
            head["metadata"]["selected"]["calibration"]["source"]
            == "hardware_factory_intrinsics"
        )
        assert any(source["source"] == "tf" for source in head["metadata"]["sources"])
        assert result["streams"][1]["state"] == "error"
    finally:
        producer.destroy_node()
        context.try_shutdown()
        asyncio.run(media.close())
