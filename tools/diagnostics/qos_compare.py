#!/usr/bin/env python3
"""Compare exported, matched DSCP-off/on camera sample arrays without changing QoS."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

# These are measured sample values, not cumulative counters.
GAUGES = ('webrtc_rtt_ms', 'webrtc_fraction_lost_pct',
          'webrtc_submit_to_transport_ms', 'webrtc_submit_to_transport_p95_ms',
          'webrtc_submit_to_transport_p99_ms', 'receiver_jitter_buffer_ms',
          'receiver_decode_ms', 'receiver_display_fps', 'receiver_frames_per_second', 'receiver_received_fps', 'receiver_decoded_fps',
          'receiver_loss_pct', 'receiver_freeze_count', 'receiver_frames_dropped',
          'receiver_fec_packets_received', 'receiver_fec_packets_discarded',
          'receiver_render_gap_p95_ms', 'receiver_render_gap_p99_ms',
          'receiver_receive_to_render_p95_ms', 'receiver_receive_to_render_p99_ms',
          'receiver_processing_p95_ms', 'receiver_processing_p99_ms',
          'receiver_freeze_duration_ms', 'receiver_packet_loss_pct',
          'control_gap_p95_ms', 'control_gap_p99_ms')


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * q
    lo, hi = math.floor(position), math.ceil(position)
    return round(values[lo] + (values[hi] - values[lo]) * (position - lo), 3)


def samples_from(document):
    samples = document.get('samples', []) if isinstance(document, dict) else document
    if not isinstance(samples, list) or not all(isinstance(item, dict) for item in samples):
        raise ValueError('Expected a JSON array of camera samples or {"samples": [...]}')
    result = []
    for item in samples:
        camera = item.get('camera', item)
        streams = camera.get('streams') if isinstance(camera, dict) else None
        if streams is not None:
            if not isinstance(streams, list):
                raise ValueError('streams must be an array')
            result.extend(streams)
        else:
            result.append(camera)
    if not all(isinstance(item, dict) for item in result):
        raise ValueError('Each camera sample must be an object')
    return result


def summarize(document):
    samples = samples_from(document)
    summary = {}
    for field in GAUGES:
        values = [float(sample[field]) for sample in samples if number(sample.get(field))
                  and not (field.startswith('receiver_') and (sample.get('receiver_stale') is True
                           or sample.get('receiver_reported') is False))
                  and not (field == 'webrtc_fraction_lost_pct' and sample.get('webrtc_feedback_received') is False)]
        if values:
            summary[field] = {'samples': len(values), 'mean': round(sum(values) / len(values), 3),
                              'p50': percentile(values, 0.5), 'p95': percentile(values, 0.95),
                              'p99': percentile(values, 0.99)}
    loss = [sample['webrtc_fraction_lost_pct'] for sample in samples
            if number(sample.get('webrtc_fraction_lost_pct')) and
            sample.get('webrtc_feedback_received') is not False]
    # RTCP intervals are correlated and not packet-level burst observations.
    qos = [sample['media_qos'] for sample in samples if isinstance(sample.get('media_qos'), dict)]
    return {'sample_count': len(samples), 'metrics': summary,
            'media_qos': {'requested_dscp': sorted({str(item.get('requested_dscp')) for item in qos}),
                          'states': sorted({str(item.get('state')) for item in qos}),
                          'wmm_verified': False},
            'loss_intervals_with_loss': sum(value > 0 for value in loss),
            'loss_intervals_observed': len(loss)}


def compare_one(baseline, treatment):
    off, on = summarize(baseline), summarize(treatment)
    deltas = {}
    for field in off['metrics'].keys() & on['metrics'].keys():
        deltas[field] = {stat: round(on['metrics'][field][stat] - off['metrics'][field][stat], 3)
                         for stat in ('mean', 'p50', 'p95', 'p99')}
    observed = min(off['loss_intervals_observed'], on['loss_intervals_observed'])
    lossy = off['loss_intervals_with_loss'] + on['loss_intervals_with_loss']
    reason = ('数据不足：需要两组有效 RTCP 丢包反馈和接收端冻结观测。' if observed < 30 else
              '观测到丢包，建议采集 RTP 序号/丢包连续长度，再评估自适应 FEC 与带宽成本。' if lossy else
              '现有窗口未观察到丢包，暂无启用 FEC 的测量依据。')
    return {'schema_version': 1, 'baseline': off, 'treatment': on,
            'delta_treatment_minus_baseline': deltas,
            'fec': {'changes_configuration': False,
                    'baseline_observed_states': sorted({s.get('webrtc_fec', {}).get('state', 'unknown') for s in samples_from(baseline)}),
                    'treatment_observed_states': sorted({s.get('webrtc_fec', {}).get('state', 'unknown') for s in samples_from(treatment)}),
                    'decision': 'requires_packet_level_measurement' if lossy and observed >= 30 else 'insufficient_evidence',
                    'reason': reason},
            'warnings': ['仅比较观测值，不能证明 DSCP 到 WMM 映射已生效；需抓包/AP 队列统计验证。',
                         '两组应使用相同设备、码率、场景、时长和交叉流量，建议交替多轮采样。',
                         'RTCP 丢包率不是突发长度；不能据此设定固定 FEC 冗余比例。',
                         '样本分位数反映采样值分布；若输入已是窗口 p95，不能解释为全体帧的 p95。',
                         'receiver_freeze_duration_ms/freezes/dropped 是接收端滚动窗口值，不可累加为整段总量。',
                         'null/缺失表示未观测；本工具不估算头显显示或端到端延迟。']}


def group_samples(document, session=None):
    groups = {}
    for sample in samples_from(document):
        if session is not None and sample.get('receiver_session_id') != session:
            continue
        camera_id = str(sample.get('id', 'main'))
        groups.setdefault(camera_id, []).append(sample)
    for camera_id, samples in groups.items():
        sessions = {sample['receiver_session_id'] for sample in samples
                    if sample.get('receiver_session_id') and sample.get('receiver_reported') is not False
                    and sample.get('receiver_stale') is not True}
        if len(sessions) > 1:
            raise ValueError(f'Camera {camera_id} contains multiple receiver sessions; select a session explicitly')
    return groups


def compare(baseline, treatment, camera_id=None, baseline_session=None, treatment_session=None):
    off = group_samples(baseline, baseline_session)
    on = group_samples(treatment, treatment_session)
    common = sorted(off.keys() & on.keys())
    if camera_id is not None:
        common = [key for key in common if key == camera_id]
    if not common:
        raise ValueError('No matching camera IDs/sessions in the two recordings')
    cameras = {}
    for key in common:
        report = compare_one(off[key], on[key])
        report['baseline_receiver_sessions'] = sorted({sample['receiver_session_id'] for sample in off[key]
                                                       if sample.get('receiver_session_id')})
        report['treatment_receiver_sessions'] = sorted({sample['receiver_session_id'] for sample in on[key]
                                                        if sample.get('receiver_session_id')})
        cameras[key] = report
    return {'schema_version': 1, 'cameras': cameras,
            'unmatched_camera_ids': sorted(off.keys() ^ on.keys())}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('treatment', type=Path)
    parser.add_argument('--camera-id')
    parser.add_argument('--baseline-session')
    parser.add_argument('--treatment-session')
    args = parser.parse_args(argv)
    try:
        report = compare(json.loads(args.baseline.read_text()), json.loads(args.treatment.read_text()),
                         args.camera_id, args.baseline_session, args.treatment_session)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
