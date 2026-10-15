"""hc-depth-source/1: declared source units, independent of depth quantization.

No unit inference from an array dtype. Unknown/invalid declarations may be stored
with original capture bytes, but must not enter a complete physical depth package.
The platform carries a hash-recorded copy; it does not import this ROS workspace.
"""

import hashlib
import json
import math

import numpy as np

SCHEMA = "hc-depth-source/1"
ENCODINGS = {"16UC1": ("u", 2), "mono16": ("u", 2), "32FC1": ("f", 4)}
FIELDS = {
    "schema_version",
    "encoding",
    "unit",
    "scale_m_per_unit",
    "invalid_values",
    "invalid_nonfinite",
    "registered_to_rgb",
    "registration_evidence",
    "evidence",
}


def validate_profile(profile):
    if not isinstance(profile, dict) or set(profile) != FIELDS:
        raise ValueError("depth source profile fields differ from hc-depth-source/1")
    scale = profile["scale_m_per_unit"]
    if (
        profile["schema_version"] != SCHEMA
        or not isinstance(profile["encoding"], str)
        or profile["encoding"] not in ENCODINGS
        or not isinstance(profile["unit"], str)
        or profile["unit"] not in {"m", "mm", "source_unit"}
        or isinstance(scale, bool)
        or not isinstance(scale, (int, float))
        or not math.isfinite(scale)
        or scale <= 0
    ):
        raise ValueError("depth source encoding, unit or scale is invalid")
    if profile["unit"] in {"m", "mm"} and scale != {"m": 1.0, "mm": 0.001}[profile["unit"]]:
        raise ValueError("depth source unit disagrees with scale")
    invalid = profile["invalid_values"]
    if (
        not isinstance(invalid, list)
        or len(invalid) > 16
        or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in invalid
        )
        or profile["invalid_nonfinite"] is not True
        or profile["registered_to_rgb"] is not None
        and type(profile["registered_to_rgb"]) is not bool
    ):
        raise ValueError("depth invalid values or registration declaration is invalid")
    evidence = profile["evidence"]
    if (
        not isinstance(evidence, dict)
        or set(evidence) != {"kind", "reference", "version"}
        or not isinstance(evidence["kind"], str)
        or evidence["kind"]
        not in {"driver_contract", "explicit_configuration", "synthetic_fixture"}
        or any(
            not isinstance(evidence[k], str) or not 1 <= len(evidence[k]) <= 2048
            for k in ("reference", "version")
        )
    ):
        raise ValueError("depth scale requires a versioned source evidence reference")
    registration = profile["registration_evidence"]
    if (
        not isinstance(registration, str)
        or len(registration) > 2048
        or profile["registered_to_rgb"] is not None
        and not registration
    ):
        raise ValueError("known registration state requires evidence")
    return profile


def profile_hash(profile):
    return hashlib.sha256(
        json.dumps(
            validate_profile(profile), sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()


def inspect_depth(metadata, array):
    """Return bounded status without changing original declarations or pixels."""
    profile = metadata.get("source_profile")
    if profile is None:
        return {
            "status": "UNKNOWN",
            "errors": ["depth_source_profile_missing"],
            "profile_sha256": None,
        }
    try:
        validate_profile(profile)
    except (ValueError, TypeError, KeyError):
        return {
            "status": "INVALID",
            "errors": ["depth_source_profile_invalid"],
            "profile_sha256": None,
        }
    expected = profile_hash(profile)
    pixels = np.asarray(array)
    errors = []
    if metadata.get("contract_errors"):
        errors.append("depth_upstream_contract_invalid")
    if (
        pixels.ndim != 2
        or (pixels.dtype.kind, pixels.dtype.itemsize) != ENCODINGS[profile["encoding"]]
    ):
        errors.append("depth_source_dtype_mismatch")
    if metadata.get("encoding") != profile["encoding"]:
        errors.append("depth_source_encoding_mismatch")
    if (
        isinstance(metadata.get("depth_scale"), bool)
        or metadata.get("depth_scale") != profile["scale_m_per_unit"]
    ):
        errors.append("depth_source_scale_mismatch")
    if metadata.get("shape") != list(pixels.shape) or metadata.get("dtype") != pixels.dtype.str:
        errors.append("depth_source_layout_mismatch")
    if metadata.get("source_profile_sha256", expected) != expected:
        errors.append("depth_source_profile_hash_mismatch")
    if profile["invalid_values"] and (
        isinstance(metadata.get("invalid_value"), bool)
        or metadata.get("invalid_value") != profile["invalid_values"][0]
    ):
        errors.append("depth_source_invalid_value_mismatch")
    return {
        "status": "INVALID" if errors else "DECLARED",
        "errors": errors,
        "profile_sha256": expected,
    }


def require_depth_source(metadata, array):
    result = inspect_depth(metadata, array)
    if result["status"] != "DECLARED":
        raise ValueError("DEPTH_SOURCE_CONTRACT_REQUIRED: " + ",".join(result["errors"]))
    return metadata["source_profile"]


def ros_image_metadata(image, profile):
    """Metadata only; layout decoding remains in the ROS adapter, with no rescale."""
    value = {
        "encoding": image.encoding,
        "is_bigendian": bool(image.is_bigendian),
        "row_step": image.step,
        "source_profile": profile,
        "depth_scale": None,
        "invalid_value": None,
    }
    if isinstance(profile, dict):
        value["depth_scale"] = profile.get("scale_m_per_unit")
        invalid = profile.get("invalid_values")
        if isinstance(invalid, list) and invalid:
            value["invalid_value"] = invalid[0]
    return value
