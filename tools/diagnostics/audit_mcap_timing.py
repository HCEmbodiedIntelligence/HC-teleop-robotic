"""Audit real MCAP samples without interpolating, retiming or dropping frames.

Run: /usr/bin/python3 -m tools.diagnostics.audit_mcap_timing FILE --output report.json
Action log_time is its final publication time when publication capture is enabled.
Sensor log_time is reception time; source continuity and RGBD pairing use header.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import struct

from mcap.reader import make_reader

from middleware.core.recording.recording_cdr import read_event

ACTION = '/action/joint_states'
PUBLISHED_ACTION = '/internal/recording/published_action'
ACTION_PROVENANCE = '/events/recording/action_provenance'
VR_TARGET = '/control/vr/joint_states'
COLOR = '/observation/cameras/head/color/compressed'
DEPTH = '/observation/cameras/head/depth/compressed'
TOPICS = (ACTION, '/control/solver/joint_targets', '/control/joint_cmd',
          '/observation/joint_states', COLOR, DEPTH)
HEADER_TYPES = {'sensor_msgs/msg/JointState', 'sensor_msgs/msg/CompressedImage',
                'sensor_msgs/msg/Image', 'geometry_msgs/msg/PoseStamped',
                'geometry_msgs/msg/PoseArray', 'geometry_msgs/msg/TwistStamped'}


def header_stamp(data: bytes) -> int | None:
    """Read the unchanged standard ROS Header from plain CDR, not a guessed clock."""
    if len(data) < 12 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        return None
    sec, nanosec = struct.unpack_from(('<' if data[1] else '>') + 'iI', data, 4)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    stamp = sec * 1_000_000_000 + nanosec
    return stamp if stamp else None


def clock_summary(stamps: list[int], threshold_ns: int) -> dict:
    ordered = sorted(stamps)
    gaps = [dict(before_ns=a, after_ns=b, gap_ms=(b-a)/1e6)
            for a, b in zip(ordered, ordered[1:]) if b-a > threshold_ns]
    return dict(count=len(stamps), first_ns=ordered[0] if ordered else None,
                last_ns=ordered[-1] if ordered else None,
                duplicates=len(stamps)-len(set(stamps)),
                out_of_order=sum(b < a for a, b in zip(stamps, stamps[1:])),
                first_interval_ms=(ordered[1]-ordered[0])/1e6 if len(ordered)>1 else None,
                last_interval_ms=(ordered[-1]-ordered[-2])/1e6 if len(ordered)>1 else None,
                max_gap_ms=max(((b-a)/1e6 for a, b in zip(ordered, ordered[1:])), default=None),
                gaps_gt_threshold=len(gaps), gaps=gaps)


def audit(path: str | Path, threshold_ms: float = 40.) -> dict:
    threshold_ns = round(threshold_ms*1e6)
    if threshold_ns <= 0:
        raise ValueError('threshold_ms must be positive')
    rows = defaultdict(lambda: dict(log=[], header=[], invalid_headers=0))
    metadata = {}
    publication_metadata = False
    provenance, invalid_provenance = [], 0
    with Path(path).open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        for record in reader.iter_metadata():
            metadata[record.name] = dict(record.metadata)
        for schema, channel, message in reader.iter_messages(topics=TOPICS+(VR_TARGET, PUBLISHED_ACTION, ACTION_PROVENANCE), log_time_order=False):
            if channel.topic == ACTION_PROVENANCE:
                try:
                    value = read_event(message.data, channel.message_encoding, schema.name if schema else '')
                    if (value.get('schema_version') != 'hc-action-provenance/v1'
                            or value.get('action_log_time_ns') != message.log_time
                            or value.get('source') not in {'published_action', 'feedback_hold'}
                            or value.get('sent_to_robot') is not (value['source'] == 'published_action')
                            or (value['source'] == 'feedback_hold' and value.get('publication_ns') is not None)):
                        raise ValueError('inconsistent label provenance')
                    provenance.append(value)
                except (ValueError, TypeError, AttributeError, KeyError):
                    invalid_provenance += 1
                continue
            row = rows[channel.topic]
            row['log'].append(message.log_time)
            stamp = header_stamp(message.data) if schema.name in HEADER_TYPES else None
            if stamp is None:
                row['invalid_headers'] += 1
            else:
                row['header'].append(stamp)
    report = dict(path=str(Path(path).resolve()), threshold_ms=threshold_ms,
                  metadata=metadata, topics={}, failures=[], unverified=[])
    labelled = 'hc_teleop.action_labels' in metadata
    source, action = rows[VR_TARGET], rows[PUBLISHED_ACTION if labelled else ACTION]
    if labelled:
        report['real_publication'] = dict(topic=PUBLISHED_ACTION,
            **{clock: clock_summary(action[clock], threshold_ns) for clock in ('log', 'header')})
        if not action['log']:
            report['failures'].append(f'{PUBLISHED_ACTION}: no successful publication evidence')
        elif report['real_publication']['log']['gaps_gt_threshold']:
            report['failures'].append(f'{PUBLISHED_ACTION}: real publication gaps > {threshold_ms:g} ms')
        report['action_label_policy'] = metadata['hc_teleop.action_labels']
        expected_labels = Counter(p['action_log_time_ns'] for p in provenance
                                  if p.get('included_in_action_labels') is True)
        expected_publications = Counter(p['publication_ns'] for p in provenance
                                        if p['source'] == 'published_action')
        label_missing = sum((Counter(rows[ACTION]['log'])-expected_labels).values())
        claim_missing = sum((expected_labels-Counter(rows[ACTION]['log'])).values())
        publication_mismatch = sum((Counter(action['log'])-expected_publications).values()) + sum(
            (expected_publications-Counter(action['log'])).values())
        report['label_provenance'] = dict(
            sources=dict(Counter(p['source'] for p in provenance)),
            invalid=invalid_provenance, unmarked_action_frames=label_missing,
            markers_without_action=claim_missing, publication_marker_mismatch=publication_mismatch)
        if invalid_provenance or label_missing or claim_missing or publication_mismatch:
            report['failures'].append('action labels: missing or inconsistent per-frame provenance')
    source_counts, action_counts = Counter(source['header']), Counter(action['header'])
    report['control_pipeline'] = dict(
        vr_target_available=bool(source['log']),
        vr_target={clock:clock_summary(source[clock], threshold_ns) for clock in ('log', 'header')},
        exact_vr_action_header_matches=sum((source_counts & action_counts).values()),
        vr_headers_without_action=sum((source_counts-action_counts).values()),
        action_headers_without_vr=sum((action_counts-source_counts).values()),
        action_evidence_topic=PUBLISHED_ACTION if labelled else ACTION,
        interpretation='Observed original-header correspondence only. Source selection, stop, home and latest-target coalescing can intentionally omit targets; unmatched counts alone are not proof of data loss.')
    for topic in TOPICS:
        row = rows[topic]
        result = {clock: clock_summary(row[clock], threshold_ns) for clock in ('log', 'header')}
        result['invalid_headers'] = row['invalid_headers']
        report['topics'][topic] = result
        if not row['log']:
            report['failures'].append(f'{topic}: no samples')
        elif len(row['log']) < 2:
            report['unverified'].append(f'{topic}: fewer than two samples')
        if row['invalid_headers']:
            report['failures'].append(f'{topic}: invalid or unsupported header stamps')
        if result['header']['gaps_gt_threshold']:
            report['failures'].append(f'{topic}: source timestamp gaps > {threshold_ms:g} ms')
        if result['header']['duplicates'] or result['header']['out_of_order']:
            report['failures'].append(f'{topic}: duplicate or out-of-order source stamps')
    if report['topics'][ACTION]['log']['gaps_gt_threshold']:
        report['failures'].append(f'{ACTION}: {"recorded label" if labelled else "publication/reception"} gaps > {threshold_ms:g} ms')
    color, depth = Counter(rows[COLOR]['header']), Counter(rows[DEPTH]['header'])
    color_missing, depth_missing = color-depth, depth-color
    report['rgbd'] = dict(paired_exact=sum((color & depth).values()),
                          color_without_depth=sum(color_missing.values()),
                          depth_without_color=sum(depth_missing.values()),
                          color_unmatched_ns=sorted(color_missing.elements()),
                          depth_unmatched_ns=sorted(depth_missing.elements()))
    if color_missing or depth_missing:
        report['failures'].append('RGBD: unpaired source timestamps; ordinal pairing would be wrong')
    window = metadata.get('hc_teleop.capture_window')
    if not window:
        report['unverified'].append('exact recording admission window was not persisted')
    else:
        try:
            start, stop = int(window['window_start_ns']), int(window['stop_requested_ns'])
            admission = int(window['admission_start_ns'])
            closed = int(window['raw_ingress_closed_ns'])
            if not 0 < start <= admission <= stop <= closed:
                raise ValueError('invalid boundary ordering')
        except (ValueError, TypeError, KeyError) as exc:
            report['failures'].append(f'invalid capture window: {exc}')
        else:
            publication_metadata = (window.get('schema_version') == 'hc-capture-window/v1'
                                    and (window.get('action_at_publication') == 'true'
                                         or labelled and window.get('real_publication_topic') == PUBLISHED_ACTION
                                         and window.get('real_action_at_publication') == 'true'))
            report['window_coverage'] = {}
            for topic in TOPICS:
                # An action already reserved before stop may finish publishing
                # later: keep and display this real evidence instead of erasing it.
                clock = 'log' if topic == ACTION else 'header'
                stamps = rows[topic][clock]
                in_window = sorted(x for x in stamps if start <= x <= stop)
                coverage = dict(clock=clock,
                                before_window=sum(x < start for x in stamps),
                                after_window=sum(x > stop for x in stamps),
                                head_gap_ms=(in_window[0]-start)/1e6 if in_window else None,
                                tail_gap_ms=(stop-in_window[-1])/1e6 if in_window else None)
                report['window_coverage'][topic] = coverage
                if not in_window:
                    report['failures'].append(f'{topic}: no samples in capture window')
                elif coverage['head_gap_ms'] > threshold_ms or coverage['tail_gap_ms'] > threshold_ms:
                    report['failures'].append(f'{topic}: recording window head/tail gap > {threshold_ms:g} ms')
                if topic != ACTION and (coverage['before_window'] or coverage['after_window']):
                    report['failures'].append(f'{topic}: source samples outside capture window')
    if not publication_metadata:
        report['unverified'].append('action log_time publication provenance requires confirming recorder implementation')
    report['result'] = 'FAIL' if report['failures'] else 'UNVERIFIED' if report['unverified'] else 'PASS'
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path)
    parser.add_argument('--threshold-ms', type=float, default=40.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = audit(args.path, args.threshold_ms)
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(encoded+'\n')
    print(json.dumps({key: result[key] for key in ('result', 'rgbd', 'failures', 'unverified')},
                     ensure_ascii=False, indent=2))
    return 1 if result['result'] != 'PASS' else 0


if __name__ == '__main__':
    raise SystemExit(main())
