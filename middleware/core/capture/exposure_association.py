"""Associate measured exposure times without inventing clock accuracy.

Kept equivalent in the robot recorder and the platform's raw recording reader.
A fixed-rate grid selects frames; measured primary RGB time selects numeric data.
"""
import math


def midpoint_time(image):
    value = image.get("exposure_midpoint_ns")
    if (image.get("timestamp_semantics") != "exposure_midpoint"
            or type(value) is not int or value <= 0
            or image.get("capture_time_ns") != value
            or not image.get("timestamp_evidence")):
        return None
    return value


def exposure_errors(image, source):
    if not source.get("require_exposure_metadata"):
        return []
    errors = []
    if midpoint_time(image) is None:
        errors.append("exposure_midpoint_evidence_missing")
    actual = image.get("actual_exposure_us")
    if (isinstance(actual, bool) or not isinstance(actual, (int, float))
            or not math.isfinite(actual) or actual <= 0
            or not image.get("exposure_evidence")):
        errors.append("exposure_metadata_missing_or_invalid")
    elif actual > source["exposure"]["max_actual_us"]:
        errors.append("exposure_limit_exceeded")
    return errors


def numeric_target(row, cfg):
    if cfg["alignment"].get("state_reference") != "primary_exposure_midpoint":
        return row["target_time_ns"]
    primary = cfg["alignment"]["primary_camera"]
    image = row["images"].get(primary, {}).get("rgb", {})
    target = midpoint_time(image)
    row.update(association_time_ns=target, association_camera=primary,
               state_time_basis="primary_rgb_exposure_midpoint",
               grid_to_exposure_ns=None if target is None else target - row["target_time_ns"])
    return target


def associate_exposure_states(aligner, row, deadline, epoch):
    if aligner.cfg["alignment"].get("state_reference") != "primary_exposure_midpoint":
        return
    from .named import aligned
    numeric_target(row, aligner.cfg)
    states = {ident: source for ident, source in aligner.cfg["sources"].items()
              if source["kind"] == "state"}
    samples = {ident: aligner.cache.eligible(ident, deadline) for ident in states}
    for camera, group in row["images"].items():
        associations = group["state_at_exposure"] = {}
        for stream in (("rgb", "depth") if aligner.cfg["sources"][camera].get("depth_enabled", True) else ("rgb",)):
            target = midpoint_time(group[stream])
            result = {"time_ns": target, "state": {}, "valid": True, "invalid_reasons": []}
            associations[stream] = result
            if target is None:
                result["invalid_reasons"].append("exposure_midpoint_evidence_missing")
            else:
                for ident, source in states.items():
                    if "named_axes" in source:
                        value, errors = aligned(samples[ident], target, source,
                                                aligner.cfg["alignment"], epoch)
                    else:
                        value, errors = aligner.state(samples[ident], target, source)
                    result["state"][ident] = value
                    if source["required"]:
                        result["invalid_reasons"].extend(ident + ":" + error for error in errors)
            result["valid"] = not result["invalid_reasons"]
            if group["required"] and not result["valid"]:
                row["invalid_reasons"].extend(
                    camera + ":" + stream + ":" + error for error in result["invalid_reasons"])

