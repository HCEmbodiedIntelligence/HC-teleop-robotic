"""The reference LeRobot single-channel depth video encoding (HEVC gray12le)."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any, cast

import av
import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True)
class DepthProfile:
    """Immutable quantization contract. Distances are metres, never sensor counts.

    Source scale is explicit per input frame. Dtype does not establish units.
    Changing any field produces a different profile identity.
    """

    depth_min: float = 0.01
    depth_max: float = 10.0
    shift: float = 3.5
    use_log: bool = True
    version: str = "hc-depth/1"

    def __post_init__(self) -> None:
        if (
            not all(math.isfinite(x) for x in (self.depth_min, self.depth_max, self.shift))
            or not 0 < self.depth_min < self.depth_max
            or (self.use_log and self.depth_min + self.shift <= 0)
        ):
            raise ValueError("invalid depth quantization range/shift")

    def metadata(self) -> dict[str, Any]:
        return {
            "is_depth_map": True,
            "depth_unit": "mm",
            "video.codec": "hevc",
            "video.pix_fmt": "gray12le",
            "video.channels": 1,
            "video.depth_min": self.depth_min,
            "video.depth_max": self.depth_max,
            "video.shift": self.shift,
            "video.use_log": self.use_log,
            "video.video_backend": "pyav",
            "has_audio": False,
        }

    def document(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "parameter_unit": "m",
            "input_unit": "mm",
            "source_scale_unit": "m/source_unit",
            "bits": 12,
            "qmax": 4095,
            "invalid_fill_code": 0,
            "invalid_requires_hc_mask": True,
            "container": "mp4",
            "codec": "hevc",
            "pixel_format": "gray12le",
            "x265_lossless": 1,
            "official_reader_version": "0.6.1",
        }

    @property
    def sha256(self) -> str:
        body = json.dumps(self.document(), sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(body.encode()).hexdigest()


DEFAULT_PROFILE = DepthProfile()
DEPTH_INFO = DEFAULT_PROFILE.metadata()
ENCODER_PROFILE = "hc-hevc-gray12-lossless/2"


def dequantize(
    quantized: ArrayLike, profile: DepthProfile = DEFAULT_PROFILE, *, output_unit: str = "m"
) -> NDArray[np.float32]:
    """Official v0.6.1 float32 inverse; mm output includes rounding to whole mm.

    This returns a 2-D plane. Apply the separate HC validity mask before geometry.
    Code zero alone is a valid depth_min measurement, NOT an invalid indicator.
    """
    if output_unit not in {"mm", "m"}:
        raise ValueError("depth output_unit must be m or mm")
    array = np.asarray(quantized)
    if not np.all(np.isfinite(array) & (array >= 0) & (array <= 4095)):
        raise ValueError("depth codes outside 12-bit range")
    if profile.use_log:
        offset = math.log(profile.depth_min + profile.shift)
        scale = (math.log(profile.depth_max + profile.shift) - offset) / 4095
    else:
        offset = profile.depth_min
        scale = (profile.depth_max - offset) / 4095
    result = np.empty(array.shape, dtype=np.float32)
    np.multiply(array, scale, out=result)
    np.add(result, offset, out=result)
    if profile.use_log:
        np.exp(result, out=result)
        result -= profile.shift
    np.clip(result, profile.depth_min, profile.depth_max, out=result)
    if output_unit == "mm":
        result *= 1000
        np.rint(result, out=result)
    return result


@dataclass(frozen=True)
class QuantizedDepth:
    codes: NDArray[np.uint16]
    valid: NDArray[np.bool_]
    saturated_low: NDArray[np.bool_]
    saturated_high: NDArray[np.bool_]
    physical_m: NDArray[np.float64]

    @property
    def flags(self) -> NDArray[np.uint8]:
        # PNG preserves these flags losslessly, independently of HEVC.
        return cast(
            NDArray[np.uint8],
            self.valid.astype(np.uint8)
            | (self.saturated_low.astype(np.uint8) << 1)
            | (self.saturated_high.astype(np.uint8) << 2),
        )

    def error_report(self, profile: DepthProfile = DEFAULT_PROFILE) -> dict[str, Any]:
        interior = self.valid & ~self.saturated_low & ~self.saturated_high
        decoded = dequantize(self.codes, profile).astype(np.float64)
        errors = np.abs(decoded[interior] - self.physical_m[interior])
        # Maximum distance from a code centre to either half-code boundary.
        # Add only the documented float32 calculation tolerance, not clipping error.
        if profile.use_log:
            delta = (
                math.log((profile.depth_max + profile.shift) / (profile.depth_min + profile.shift))
                / 4095
            )
            bound = (decoded[interior] + profile.shift) * math.expm1(delta / 2)
        else:
            bound = np.full(errors.shape, (profile.depth_max - profile.depth_min) / 8190)
        tolerance_m = 2e-5
        if np.any(errors > bound + tolerance_m):
            raise ValueError("depth quantization exceeds half-cell error bound")
        clipped = self.saturated_low | self.saturated_high
        clipped_errors = np.abs(decoded[clipped] - self.physical_m[clipped])
        percentiles = np.quantile(errors, [0.5, 0.95, 0.99]) if errors.size else [None]*3
        return {
            "pixels": self.valid.size,
            "valid_pixels": int(self.valid.sum()),
            "invalid_pixels": int((~self.valid).sum()),
            "saturated_low_pixels": int(self.saturated_low.sum()),
            "saturated_high_pixels": int(self.saturated_high.sum()),
            "unclipped_pixels": int(interior.sum()),
            "unclipped_max_error_m": float(errors.max()) if errors.size else None,
            "unclipped_p50_error_m": float(percentiles[0]) if errors.size else None,
            "unclipped_p95_error_m": float(percentiles[1]) if errors.size else None,
            "unclipped_p99_error_m": float(percentiles[2]) if errors.size else None,
            "clipped_max_error_m": float(clipped_errors.max()) if clipped_errors.size else None,
            "half_cell_bound_passed": True,
            "float32_tolerance_m": tolerance_m,
            "mm_reader_additional_rounding_bound_m": 0.0005,
        }


def quantize(
    array: ArrayLike,
    scale: float | None,
    profile: DepthProfile = DEFAULT_PROFILE,
    *,
    valid_mask: ArrayLike | None = None,
    invalid_values: Sequence[float] = (),
) -> QuantizedDepth:
    if scale is None or not math.isfinite(scale) or scale <= 0:
        raise ValueError("explicit finite positive source depth_scale (metres/unit) required")
    source = np.asarray(array)
    if source.ndim != 2 or source.dtype.kind not in "uif":
        raise ValueError("depth must be a numeric two-dimensional single-channel plane")
    physical = source.astype(np.float64) * scale
    valid = np.isfinite(physical) & (physical > 0)
    for sentinel in invalid_values:
        valid &= source != sentinel
    if valid_mask is not None:
        mask = np.asarray(valid_mask)
        if mask.shape != source.shape or mask.dtype != np.bool_:
            raise ValueError("sensor validity mask must be boolean and match depth shape")
        valid &= mask
    low, high = valid & (physical < profile.depth_min), valid & (physical > profile.depth_max)
    # Determine validity/saturation BEFORE filling or clipping. The source remains untouched.
    # Use the official explicit-mm float32 quantization domain, including its rounding.
    minimum, maximum, shift = (
        np.float32(x * 1000) for x in (profile.depth_min, profile.depth_max, profile.shift)
    )
    with np.errstate(over="ignore", invalid="ignore"):
        mm = source.astype(np.float32) * np.float32(scale * 1000)
    mm = np.where(valid, np.clip(mm, minimum, maximum), minimum)
    if profile.use_log:
        norm = (np.log(mm + shift) - math.log(float(minimum + shift))) / (
            math.log(float(maximum + shift)) - math.log(float(minimum + shift))
        )
    else:
        norm = (mm - minimum) / (maximum - minimum)
    packed = np.rint(norm * 4095).clip(0, 4095).astype(np.uint16)
    packed[~valid] = 0
    return QuantizedDepth(packed, valid, low, high, physical)


def codes(
    array: ArrayLike, scale: float | None, profile: DepthProfile = DEFAULT_PROFILE
) -> NDArray[np.uint16]:
    return quantize(array, scale, profile).codes


class DepthVideo:
    def __init__(
        self,
        path: Path,
        fps: float | Fraction,
        shape: Sequence[int],
        profile: DepthProfile = DEFAULT_PROFILE,
    ) -> None:
        if min(shape[:2]) < 16:
            raise ValueError("HEVC depth video requires width and height of at least 16 pixels")
        if fps <= 0:
            raise ValueError("depth fps must be positive")
        self.profile, self.shape, self.closed = profile, tuple(shape[:2]), False
        path.parent.mkdir(parents=True, exist_ok=True)
        self.container = av.open(str(path), "w")
        self.stream = cast(
            av.video.stream.VideoStream,
            self.container.add_stream("libx265", rate=Fraction(str(fps))),
        )
        self.stream.height, self.stream.width = shape[:2]
        self.stream.pix_fmt = "gray12le"
        self.stream.codec_context.thread_count = 1
        self.encoder_options = {
            # Let x265 select the monochrome 12-bit RExt profile, as the official
            # LeRobot encoder does. --profile main12 rejects gray12le/i400.
            "preset": "ultrafast",
            "x265-params": (
                # Inter prediction in the pinned build reproduced code changes
                # after zero frames at multiple sizes. Neither rd=3 nor weightp=0
                # reliably fixed it. Use independent pictures, retain gray12le,
                # and still verify every decoded code before accepting output.
                "lossless=1:keyint=1:bframes=0:open-gop=0:repeat-headers=1:"
                "pools=none:frame-threads=1:log-level=error"
            ),
        }
        self.stream.options = dict(self.encoder_options)
        self.fps, self.frames = fps, 0

    def encoding_metadata(self) -> dict[str, Any]:
        configuration = {
            "encoder_profile": ENCODER_PROFILE,
            "pyav_version": av.__version__,
            "ffmpeg_libraries": av.library_versions,
            "encoder_options": dict(self.encoder_options),
        }
        encoded = json.dumps(configuration, sort_keys=True, separators=(",", ":"))
        return {
            **configuration,
            "encoder_configuration_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
        }

    def write(self, array: ArrayLike, scale: float | None) -> NDArray[np.uint16]:
        quantized = codes(array, scale, self.profile)
        self.write_codes(quantized)
        return quantized

    def write_codes(self, quantized: NDArray[np.uint16]) -> None:
        if self.closed:
            raise ValueError("depth video is closed")
        if (
            quantized.shape != self.shape
            or quantized.dtype != np.uint16
            or np.any(quantized > 4095)
        ):
            raise ValueError("depth code shape/dtype/range does not match the stream")
        frame = av.VideoFrame(quantized.shape[1], quantized.shape[0], "gray12le")
        plane = frame.planes[0]
        padded = np.zeros((quantized.shape[0], plane.line_size // 2), dtype="<u2")
        padded[:, : quantized.shape[1]] = quantized
        plane.update(padded.tobytes())
        frame.pts, frame.time_base = self.frames, 1 / Fraction(str(self.fps))
        for packet in self.stream.encode(frame):
            self.container.mux(packet)
        self.frames += 1

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            if self.frames:
                for packet in self.stream.encode(None):
                    self.container.mux(packet)
        finally:
            self.container.close()
