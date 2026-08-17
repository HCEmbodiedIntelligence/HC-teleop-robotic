#!/usr/bin/env python3
"""Summarize an HC-TJ teleop diagnostic CSV and identify likely jitter sources."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from teleop_diagnostics import ARM_JOINTS, POSE_FIELDS, SIDES


def number(row: dict[str, str], field: str) -> float | None:
    try:
        value = float(row.get(field, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def unique_rows(
    rows: Iterable[dict[str, str]], key: str
) -> list[dict[str, str]]:
    result: list[dict[str, str]] = []
    previous = None
    for row in rows:
        sequence = row.get(f"{key}_seq", "")
        if sequence and sequence != previous:
            result.append(row)
            previous = sequence
    return result


def percentile(values: Iterable[float], level: float = 95.0) -> float:
    array = np.asarray(list(values), dtype=float)
    return float(np.percentile(array, level)) if array.size else float("nan")


def pose_steps(rows: list[dict[str, str]], key: str) -> tuple[float, float]:
    samples = unique_rows(rows, key)
    positions: list[np.ndarray] = []
    quaternions: list[np.ndarray] = []
    for row in samples:
        values = [number(row, f"{key}_{field}") for field in POSE_FIELDS]
        if any(value is None for value in values):
            continue
        positions.append(np.asarray(values[:3], dtype=float))
        quaternion = np.asarray(values[3:], dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if norm > 1e-9:
            quaternions.append(quaternion / norm)
        else:
            positions.pop()
    if len(positions) < 2:
        return float("nan"), float("nan")
    position_step = np.linalg.norm(np.diff(np.asarray(positions), axis=0), axis=1)
    quaternion_array = np.asarray(quaternions)
    dots = np.abs(np.sum(quaternion_array[1:] * quaternion_array[:-1], axis=1))
    angle_step = 2.0 * np.arccos(np.clip(dots, -1.0, 1.0))
    return percentile(position_step) * 1000.0, percentile(angle_step) * 180.0 / math.pi


def tracking_error(
    rows: list[dict[str, str]], side: str
) -> tuple[float, float]:
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    for row in rows:
        target = [number(row, f"target_{side}_{field}") for field in POSE_FIELDS]
        actual = [number(row, f"actual_{side}_{field}") for field in POSE_FIELDS]
        if any(value is None for value in target + actual):
            continue
        position_errors.append(
            float(np.linalg.norm(np.asarray(target[:3]) - np.asarray(actual[:3])))
        )
        target_q = np.asarray(target[3:], dtype=float)
        actual_q = np.asarray(actual[3:], dtype=float)
        target_q /= np.linalg.norm(target_q)
        actual_q /= np.linalg.norm(actual_q)
        dot = abs(float(np.dot(target_q, actual_q)))
        orientation_errors.append(2.0 * math.acos(min(1.0, dot)))
    return (
        percentile(position_errors) * 1000.0,
        percentile(orientation_errors) * 180.0 / math.pi,
    )


def joint_metrics(
    rows: list[dict[str, str]], source: str
) -> list[tuple[str, float, float]]:
    samples = unique_rows(rows, source)
    result = []
    for name in ARM_JOINTS:
        values = [number(row, f"{source}_{name}") for row in samples]
        array = np.asarray([value for value in values if value is not None], dtype=float)
        if array.size < 2:
            continue
        result.append(
            (
                name,
                float(np.ptp(array)) * 180.0 / math.pi,
                percentile(np.abs(np.diff(array))) * 180.0 / math.pi,
            )
        )
    return result


def source_rate(rows: list[dict[str, str]], key: str) -> float:
    samples = unique_rows(rows, key)
    if len(samples) < 2:
        return float("nan")
    first_sequence = number(samples[0], f"{key}_seq")
    last_sequence = number(samples[-1], f"{key}_seq")
    first_time = number(samples[0], "elapsed_s")
    last_time = number(samples[-1], "elapsed_s")
    if None in (first_sequence, last_sequence, first_time, last_time) or last_time <= first_time:
        return float("nan")
    return (last_sequence - first_sequence) / (last_time - first_time)


def show(value: float, unit: str, digits: int = 3) -> str:
    return "无数据" if not math.isfinite(value) else f"{value:.{digits}f} {unit}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--start", type=float, default=0.0, help="只分析该秒数之后")
    parser.add_argument("--end", type=float, help="只分析该秒数之前")
    args = parser.parse_args()
    with args.log.expanduser().open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("日志为空")
    rows = [
        row
        for row in rows
        if (number(row, "elapsed_s") or 0.0) >= args.start
        and (args.end is None or (number(row, "elapsed_s") or 0.0) <= args.end)
    ]
    if not rows:
        raise SystemExit("所选时间范围内没有日志样本")

    duration = (number(rows[-1], "elapsed_s") or 0.0) - (number(rows[0], "elapsed_s") or 0.0)
    active = [row for row in rows if row.get("right_clutch") == "1"]
    print(f"日志: {args.log.resolve()}")
    print(f"时长: {duration:.2f} s；总样本: {len(rows)}；右臂离合样本: {len(active)}")
    if len(active) < 3:
        print("结论: 没有足够的右 Grip 控制数据；按住右 Grip、保持手柄静止 5–10 秒后再分析。")
        return

    print("\n末端链路（活跃时相邻消息 P95）:")
    controller_position_steps = []
    target_position_steps = []
    actual_position_steps = []
    for side in SIDES:
        controller_pos, controller_rot = pose_steps(active, f"controller_{side}")
        target_pos, target_rot = pose_steps(active, f"target_{side}")
        actual_pos, actual_rot = pose_steps(active, f"actual_{side}")
        position_error, orientation_error = tracking_error(active, side)
        controller_position_steps.append(controller_pos)
        target_position_steps.append(target_pos)
        actual_position_steps.append(actual_pos)
        print(
            f"  {side}: 手柄 {show(controller_pos, 'mm')} / {show(controller_rot, 'deg')}；"
            f"目标 {show(target_pos, 'mm')} / {show(target_rot, 'deg')}；"
            f"实际 {show(actual_pos, 'mm')} / {show(actual_rot, 'deg')}；"
            f"跟踪误差 {show(position_error, 'mm')} / {show(orientation_error, 'deg')}"
        )

    command_metrics = joint_metrics(active, "joint_command")
    feedback_metrics = joint_metrics(active, "joint_state")
    print("\n关节变化最大的命令（范围 / 相邻消息 P95）:")
    for name, value_range, step in sorted(
        command_metrics, key=lambda item: item[2], reverse=True
    )[:6]:
        print(f"  {name}: {value_range:.3f} deg / {step:.3f} deg")

    if "ik_right_converged" in active[0]:
        print("\nIK 安全状态:")
        for side in SIDES:
            rejected = sum(row.get(f"ik_{side}_converged") == "0" for row in active)
            seeds = sorted(
                {
                    row.get(f"ik_{side}_seed", "")
                    for row in active
                    if row.get(f"ik_{side}_seed", "")
                }
            )
            total = max(
                (
                    int(float(row[f"ik_{side}_rejection_total"]))
                    for row in active
                    if row.get(f"ik_{side}_rejection_total")
                ),
                default=0,
            )
            print(
                f"  {side}: 拒绝状态样本 {rejected}/{len(active)}；"
                f"累计拒绝 {total}；选解 {','.join(seeds) or '无数据'}"
            )

    command_step = max((item[2] for item in command_metrics), default=float("nan"))
    feedback_step = max((item[2] for item in feedback_metrics), default=float("nan"))
    controller_step = max(
        (value for value in controller_position_steps if math.isfinite(value)),
        default=float("nan"),
    )
    target_step = max(
        (value for value in target_position_steps if math.isfinite(value)),
        default=float("nan"),
    )
    actual_step = max(
        (value for value in actual_position_steps if math.isfinite(value)),
        default=float("nan"),
    )
    print("\n时序:")
    print(
        "  手柄/目标/实际/命令/反馈消息率: "
        f"{source_rate(active, 'controller_right'):.1f} / "
        f"{source_rate(active, 'target_right'):.1f} / "
        f"{source_rate(active, 'actual_right'):.1f} / "
        f"{source_rate(active, 'joint_command'):.1f} / "
        f"{source_rate(active, 'joint_state'):.1f} Hz"
    )

    reasons = []
    if math.isfinite(controller_step) and controller_step > 0.3:
        reasons.append("手柄存在主动运动或追踪跳变；用 --start/--end 选择静止保持区间可进一步区分")
    if (
        math.isfinite(controller_step)
        and math.isfinite(target_step)
        and controller_step <= 0.3
        and target_step > max(0.5, controller_step * 3.0)
    ):
        reasons.append("坐标/目标生成环节放大了手柄增量")
    if math.isfinite(target_step) and target_step <= 0.5 and math.isfinite(command_step) and command_step > 0.2:
        reasons.append("目标基本稳定但关节命令变化明显，属于冗余 IK 漂移或近奇异位形放大")
    if math.isfinite(command_step) and math.isfinite(feedback_step) and feedback_step > command_step * 1.5 + 0.1:
        reasons.append("关节反馈比命令更抖，问题位于仿真/执行器跟踪环节")
    if math.isfinite(actual_step) and math.isfinite(target_step) and actual_step > target_step * 2.0 + 0.5:
        reasons.append("实际末端变化显著大于目标，IK 或关节执行放大了抖动")
    if not reasons:
        reasons.append("未发现单一明显放大环节，请延长静止离合采样并观察最大关节")
    print("\n自动判断:")
    for reason in reasons:
        print(f"  - {reason}")


if __name__ == "__main__":
    main()
