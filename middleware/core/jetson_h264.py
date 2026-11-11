"""Jetson's native GStreamer/V4L2 encoder; optional dependencies load lazily.

One input frame is submitted per synchronous encode call. Output is one Annex-B
access unit, consumed by aiortc's existing H.264 packetizer. No gst-launch shell
or automatic system package installation is involved.
"""
from pathlib import Path
import re
import threading


INSTALL_GUIDANCE = (
    "Use the Python interpreter with JetPack's system GI bindings. On the Jetson, "
    "install matching JetPack/L4T packages: python3-gi, gir1.2-gstreamer-1.0, "
    "gir1.2-gst-plugins-base-1.0, gstreamer1.0-plugins-base, "
    "gstreamer1.0-plugins-bad, nvidia-l4t-gstreamer. Check gst-inspect-1.0 "
    "nvv4l2h264enc and nvvidconv; restore the matching JetPack driver/device "
    "exposure if encoder startup fails. No packages were installed automatically."
)


def jetson_platform():
    def read(path):
        try:
            return Path(path).read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            return ''
    model = read('/proc/device-tree/model')
    compatible = read('/proc/device-tree/compatible')
    detected = 'nvidia,tegra' in compatible.lower() or 'jetson' in model.lower()
    return {'is_jetson': detected, 'model': model or None,
            'compatible': compatible or None,
            'encoder_devices': [str(p) for pattern in ('/dev/nvhost-msenc*', '/dev/nvhost-nvenc*', '/dev/v4l2-nvenc*')
                                for p in Path('/').glob(pattern.lstrip('/'))],
            'hardware_note': ('Jetson Orin Nano has no hardware video encoder; use x264.'
                              if 'orin nano' in model.lower() else None)}


def _load_gst():
    try:
        import gi
        gi.require_version('Gst', '1.0')
        gi.require_version('GstVideo', '1.0')
        from gi.repository import Gst, GstVideo
        Gst.init(None)
        return Gst, GstVideo
    except (ImportError, ValueError) as exc:
        raise RuntimeError(f'Jetson GStreamer/GI unavailable: {exc}. {INSTALL_GUIDANCE}') from exc


def annexb_nals(data):
    return [nal for nal in re.split(b'\x00\x00\x00?\x01', data) if nal]


def validate_access_unit(data, require_idr=False, require_headers=False):
    if not data.startswith((b'\x00\x00\x01', b'\x00\x00\x00\x01')):
        raise RuntimeError('Jetson encoder did not return Annex-B H.264')
    nals = annexb_nals(data)
    types = {nal[0] & 31 for nal in nals}
    if not types.intersection({1, 5}):
        raise RuntimeError('Jetson output contains no video slice')
    if require_idr and 5 not in types:
        raise RuntimeError('Jetson did not produce the requested recovery IDR')
    if require_headers and not {7, 8}.issubset(types):
        raise RuntimeError('Jetson first access unit lacks SPS/PPS')
    for nal in nals:
        if nal[0] & 31 == 7:
            if len(nal) < 4 or nal[1] != 66 or not nal[2] & 0x40 or nal[3] > 31:
                raise RuntimeError('Jetson SPS is incompatible with negotiated H.264 Constrained Baseline level 3.1')
    return types


def pack_i420(frame, width, height, info):
    """Copy PyAV's compact ndarray into GstVideoInfo's padded plane layout."""
    raw = frame.to_ndarray(format='yuv420p').tobytes()
    if len(raw) != width * height * 3 // 2:
        raise RuntimeError('Jetson raw I420 frame is not tightly packed')
    if (info.size == len(raw) and list(info.stride[:3]) == [width, width // 2, width // 2]
            and list(info.offset[:3]) == [0, width * height, width * height * 5 // 4]):
        return raw
    packed = bytearray(info.size)
    source_offset = 0
    for plane, (columns, rows) in enumerate(((width, height), (width // 2, height // 2), (width // 2, height // 2))):
        stride, destination_offset = info.stride[plane], info.offset[plane]
        if stride < columns or destination_offset + (rows - 1) * stride + columns > len(packed):
            raise RuntimeError('GStreamer returned an invalid I420 plane layout')
        for row in range(rows):
            begin = source_offset + row * columns
            destination = destination_offset + row * stride
            packed[destination:destination + columns] = raw[begin:begin + columns]
        source_offset += columns * rows
    return bytes(packed)


class JetsonH264Codec:
    """Small CodecContext-compatible facade, owned by one RTP encoder.

    appsrc is nonblocking with a one-frame byte limit, appsink holds at most
    two access units, and the caller permits exactly one outstanding input.
    A timeout makes this instance unusable; close it and fall back immediately.
    """
    def __init__(self, width, height, bitrate, fps=30, timeout_seconds=0.5, startup_timeout_seconds=3.0):
        self.width, self.height, self.bit_rate = int(width), int(height), int(bitrate)
        self.fps = int(fps)
        macroblocks = ((self.width + 15) // 16) * ((self.height + 15) // 16)
        if (self.width < 2 or self.height < 2 or self.width % 2 or self.height % 2
                or self.fps < 1 or macroblocks > 3600 or macroblocks * self.fps > 108000):
            raise ValueError('Jetson H.264 output must have even dimensions and fit level 3.1')
        self.timeout_seconds = float(timeout_seconds)
        self.startup_timeout_seconds = float(startup_timeout_seconds)
        if not 0 < self.timeout_seconds <= 10 or not 0 < self.startup_timeout_seconds <= 10:
            raise ValueError('Jetson encoding timeout must be between 0 and 10 seconds')
        self.platform = jetson_platform()
        if not self.platform['is_jetson']:
            raise RuntimeError('Jetson GStreamer backend unavailable: device tree does not identify a Jetson. '
                               'A desktop NVIDIA GPU uses the nvenc backend. ' + INSTALL_GUIDANCE)
        if self.platform.get('hardware_note') and not self.platform.get('encoder_devices'):
            raise RuntimeError(self.platform['hardware_note'] + ' Installing JetPack cannot add the missing encoder.')
        self.Gst, self.GstVideo = _load_gst()
        self.pipeline = None
        self.closed = False
        self.frames = 0
        self._last_pts = None
        self._lock = threading.RLock()
        required = ('appsrc', 'nvvidconv', 'nvv4l2h264enc', 'h264parse', 'appsink')
        missing = [name for name in required if not self.Gst.ElementFactory.find(name)]
        if missing:
            raise RuntimeError(f'Jetson GStreamer plugins missing: {", ".join(missing)}. {INSTALL_GUIDANCE}')
        try:
            raw_caps = self.Gst.Caps.from_string(
                f'video/x-raw,format=I420,width={self.width},height={self.height},framerate={self.fps}/1')
            self.video_info = self.GstVideo.VideoInfo.new_from_caps(raw_caps)
            if self.video_info is None:
                raise RuntimeError('GStreamer could not describe the raw I420 layout')
            # profile=0 is NVIDIA's Baseline enum; level is negotiated through
            # caps, not the desktop FFmpeg NVENC option. On tested L4T 36.4,
            # vbv-size is a bit budget (default 4,000,000), not a frame count.
            # A tiny value such as 7 triggers NVENC BlockSide error 0x4.
            pipeline = (
                'appsrc name=source is-live=true format=time block=false do-timestamp=false '
                f'max-bytes={self.video_info.size} '
                f'caps="video/x-raw,format=I420,width={self.width},height={self.height},framerate={self.fps}/1" ! '
                'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
                'nvv4l2h264enc name=encoder profile=0 control-rate=1 '
                f'bitrate={self.bit_rate} preset-level=1 insert-sps-pps=true '
                'iframeinterval=300 idrinterval=300 ! '
                'video/x-h264,profile=constrained-baseline,level=(string)3.1,stream-format=byte-stream,alignment=au ! '
                'h264parse config-interval=-1 ! '
                'video/x-h264,stream-format=byte-stream,alignment=au ! '
                'appsink name=sink sync=false async=false max-buffers=2 drop=false enable-last-sample=false'
            )
            self.pipeline = self.Gst.parse_launch(pipeline)
            self.source = self.pipeline.get_by_name('source')
            self.encoder = self.pipeline.get_by_name('encoder')
            self.sink = self.pipeline.get_by_name('sink')
            # Modern JetPack exposes profile=1 for Constrained Baseline. Older
            # versions expose only Baseline (0); the caps/SPS check must still
            # prove constraint_set1 so both advertised 42001f and 42e01f offers
            # can decode this stream. Never silently emit Main/High/unconstrained.
            profile = self.encoder.find_property('profile')
            enum_values = getattr(getattr(profile, 'enum_class', None), '__enum_values__', {})
            if 1 in enum_values:
                self.encoder.set_property('profile', 1)
            # These properties differ between JetPack versions. Baseline caps
            # already exclude B slices; only apply optional low-latency knobs
            # if this encoder actually exports them.
            for name, value in (('num-B-Frames', 0), ('maxperf-enable', True),
                                ('vbv-size', max(1, self.bit_rate // 4)), ('poc-type', 2)):
                if self.encoder.find_property(name):
                    self.encoder.set_property(name, value)
            self.bus = self.pipeline.get_bus()
            result = self.pipeline.set_state(self.Gst.State.PLAYING)
            if result == self.Gst.StateChangeReturn.FAILURE:
                self._raise_bus_error('Jetson pipeline could not enter PLAYING')
        except Exception as exc:
            self.close()
            raise RuntimeError(f'Jetson GStreamer initialization failed: {exc}. {INSTALL_GUIDANCE}') from exc

    def _raise_bus_error(self, default):
        message = self.bus.timed_pop_filtered(0, self.Gst.MessageType.ERROR)
        if message:
            error, debug = message.parse_error()
            raise RuntimeError(f'{default}: {error}; {debug or ""}. {INSTALL_GUIDANCE}')
        raise RuntimeError(f'{default}. {INSTALL_GUIDANCE}')

    def set_bitrate(self, bitrate):
        """Apply in PLAYING only when the plugin advertises runtime mutability.

        Otherwise the owner uses its bounded rebuild policy. Readback is a
        configuration acknowledgement, not a claim about observed network rate.
        """
        with self._lock:
            if self.closed:
                raise RuntimeError('Jetson encoder is closed')
            prop = self.encoder.find_property('bitrate')
            if not prop or not int(prop.flags) & int(self.Gst.PARAM_MUTABLE_PLAYING):
                return False
            self.encoder.set_property('bitrate', int(bitrate))
            if self.encoder.get_property('bitrate') != int(bitrate):
                raise RuntimeError('Jetson encoder rejected bitrate update')
            self.bit_rate = int(bitrate)
            return True

    def encode(self, frame):
        with self._lock:
            if self.closed:
                raise RuntimeError('Jetson encoder is closed')
            try:
                if frame.width != self.width or frame.height != self.height:
                    raise ValueError('Jetson frame dimensions changed without reconfiguration')
                if self.bus.have_pending():
                    error = self.bus.timed_pop_filtered(0, self.Gst.MessageType.ERROR)
                    if error:
                        detail, debug = error.parse_error()
                        raise RuntimeError(f'Jetson encoder bus error: {detail}; {debug or ""}. {INSTALL_GUIDANCE}')
                force = getattr(frame.pict_type, 'name', None) == 'I' or int(frame.pict_type) == 1
                if force and self.frames:
                    try:
                        # NVIDIA's action is required on tested L4T 36.4: the
                        # generic event is accepted but does not produce IDR.
                        self.encoder.emit('force-IDR')
                    except TypeError as exc:
                        # Only older plugins without this action use the
                        # standard event. Real action/driver failures propagate.
                        if 'unknown signal' not in str(exc).lower():
                            raise
                        event = self.GstVideo.video_event_new_downstream_force_key_unit(
                            self.Gst.CLOCK_TIME_NONE, self.Gst.CLOCK_TIME_NONE, self.Gst.CLOCK_TIME_NONE, True, self.frames)
                        if not self.encoder.get_static_pad('sink').send_event(event):
                            raise RuntimeError('Jetson rejected force-key-unit request')
                raw = pack_i420(frame, self.width, self.height, self.video_info)
                buffer = self.Gst.Buffer.new_allocate(None, len(raw), None)
                buffer.fill(0, raw)
                pts = int(frame.pts * frame.time_base * self.Gst.SECOND) if frame.pts is not None and frame.time_base else self.frames * self.Gst.SECOND // self.fps
                if self._last_pts is not None and pts <= self._last_pts:
                    raise ValueError('Jetson input PTS must increase')
                buffer.pts = pts
                buffer.dts = self.Gst.CLOCK_TIME_NONE
                buffer.duration = self.Gst.SECOND // self.fps
                result = self.source.emit('push-buffer', buffer)
                if result != self.Gst.FlowReturn.OK:
                    raise RuntimeError(f'Jetson appsrc push failed: {result}')
                timeout = self.startup_timeout_seconds if not self.frames else self.timeout_seconds
                sample = self.sink.emit('try-pull-sample', int(timeout * self.Gst.SECOND))
                if sample is None:
                    self._raise_bus_error(f'Jetson hardware encoder produced no frame within {timeout:g}s')
                output = sample.get_buffer()
                if output.pts not in (pts, self.Gst.CLOCK_TIME_NONE):
                    raise RuntimeError('Jetson output PTS does not match the submitted frame')
                data = bytes(output.extract_dup(0, output.get_size()))
                validate_access_unit(data, require_idr=force or not self.frames, require_headers=not self.frames)
                self._last_pts = pts
                self.frames += 1
                return [data]
            except Exception:
                self.close()
                raise

    def close(self):
        with getattr(self, '_lock', threading.RLock()):
            if getattr(self, 'closed', True):
                return
            self.closed = True
            pipeline = self.pipeline
            self.pipeline = None
            if pipeline is not None:
                pipeline.set_state(self.Gst.State.NULL)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
