"""Inspect recorded MCAP topic quality: head/tail coverage, inter-frame gaps, and timing jitter.

Run:
  /usr/bin/python3 tools/diagnostics/check_mcap_quality.py [MCAP_FILE]
  /usr/bin/python3 -m tools.diagnostics.check_mcap_quality --latest --gap-ms 40 --head-tail-ms 40
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import struct
import sys
from typing import Any, Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[2]
for candidate in (PROJECT_ROOT, PROJECT_ROOT / '.deps'):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

from mcap.reader import make_reader

from middleware.core.capture_window import WINDOW_METADATA, _HEADER_TYPES
from tools.diagnostics.audit_mcap_timing import (
    ACTION,
    ACTION_PROVENANCE,
    HEADER_TYPES,
    PUBLISHED_ACTION,
    clock_summary,
    header_stamp,
)

ALL_HEADER_TYPES = HEADER_TYPES | _HEADER_TYPES
INHERENT_LOW_RATE_TOPICS = {
    '/action/base/command',
    '/action/base/velocity',
    '/observation/hands/joint_states',
    '/observation/hands/vibration_feedback',
    '/status/base',
    '/status/control/ownership',
    '/status/teleop',
    '/tf',
}
INHERENT_LOW_RATE_SUFFIXES = ('/extrinsics', '/calibration')
INHERENT_LOW_RATE_PREFIXES = ('/events/',)


def is_inherent_low_rate(topic: str) -> bool:
    """Return True for status/metadata/event topics whose normal period exceeds 40 ms."""
    return (
        topic in INHERENT_LOW_RATE_TOPICS
        or topic.endswith(INHERENT_LOW_RATE_SUFFIXES)
        or topic.startswith(INHERENT_LOW_RATE_PREFIXES)
    )


def find_latest_mcap(directory: Path | None = None) -> Path:
    """Return the most recently modified MCAP recording under ``directory``."""
    root = directory or (PROJECT_ROOT / 'runtime' / 'topic_recordings')
    if not root.exists():
        raise FileNotFoundError(f'Recording directory not found: {root}')
    candidates = [p for p in root.glob('*.mcap') if p.is_file()]
    if not candidates:
        candidates = [p for p in root.rglob('*.mcap') if p.is_file()]
    if not candidates:
        raise FileNotFoundError(f'No .mcap files found under: {root}')
    return max(candidates, key=lambda item: (item.stat().st_mtime_ns, item.name))


def _decode_vrdata_timing(data: bytes) -> dict[str, Any] | None:
    """Extract sequence, vr_timestamp, and hc_received_ns from CDR std_msgs/String /vrdata."""
    if len(data) < 12 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        return None
    endian = '<' if data[1] else '>'
    length = struct.unpack_from(endian + 'I', data, 4)[0]
    if length <= 1 or 8 + length - 1 > len(data):
        return None
    try:
        payload = json.loads(data[8:8 + length - 1].decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    return {
        'sequence': payload.get('sequence'),
        'vr_timestamp': payload.get('vr_timestamp'),
        'hc_received_ns': payload.get('hc_received_ns'),
    }


def _percentile(sorted_values: list[float], ratio: float) -> float | None:
    if not sorted_values:
        return None
    idx = min(len(sorted_values) - 1, max(0, int(len(sorted_values) * ratio)))
    return sorted_values[idx]


def _clock_metrics(
    stamps: list[int],
    threshold_ns: int,
    window: dict[str, int] | None,
    *,
    max_gap_samples: int,
) -> dict[str, Any]:
    summary = clock_summary(stamps, threshold_ns)
    ordered = sorted(stamps)
    intervals_ms = [(b - a) / 1e6 for a, b in zip(ordered, ordered[1:])]
    sorted_intervals = sorted(intervals_ms)
    span_s = (ordered[-1] - ordered[0]) / 1e9 if len(ordered) >= 2 else 0.0
    hz = (len(ordered) - 1) / span_s if span_s > 0 else None

    gap_details = []
    for idx, (a, b) in enumerate(zip(ordered, ordered[1:])):
        if b - a > threshold_ns:
            offset_base = window['admission_start_ns'] if window else ordered[0]
            gap_details.append({
                'index': idx,
                'before_ns': a,
                'after_ns': b,
                'offset_ms': round((a - offset_base) / 1e6, 3),
                'gap_ms': round((b - a) / 1e6, 3),
            })

    metrics: dict[str, Any] = {
        'count': summary['count'],
        'hz': round(hz, 2) if hz is not None else None,
        'duplicates': summary['duplicates'],
        'out_of_order': summary['out_of_order'],
        'min_ms': round(sorted_intervals[0], 3) if sorted_intervals else None,
        'avg_ms': round(sum(intervals_ms) / len(intervals_ms), 3) if intervals_ms else None,
        'p50_ms': round(_percentile(sorted_intervals, 0.50), 3) if sorted_intervals else None,
        'p95_ms': round(_percentile(sorted_intervals, 0.95), 3) if sorted_intervals else None,
        'p99_ms': round(_percentile(sorted_intervals, 0.99), 3) if sorted_intervals else None,
        'max_ms': round(sorted_intervals[-1], 3) if sorted_intervals else None,
        'first_interval_ms': round(summary['first_interval_ms'], 3) if summary['first_interval_ms'] is not None else None,
        'last_interval_ms': round(summary['last_interval_ms'], 3) if summary['last_interval_ms'] is not None else None,
        'gaps_gt_threshold': len(gap_details),
        'gap_samples': gap_details[:max_gap_samples] if max_gap_samples > 0 else gap_details,
        'first_ns': summary['first_ns'],
        'last_ns': summary['last_ns'],
    }
    if window and ordered:
        w_start = window['window_start_ns']
        adm_start = window['admission_start_ns']
        stop_req = window['stop_requested_ns']
        metrics.update({
            'before_window': sum(x < w_start for x in ordered),
            'after_stop': sum(x > stop_req for x in ordered),
            'head_vs_window_ms': round((ordered[0] - w_start) / 1e6, 3),
            'head_vs_admission_ms': round((ordered[0] - adm_start) / 1e6, 3),
            'tail_vs_stop_ms': round((stop_req - ordered[-1]) / 1e6, 3),
        })
    return metrics


def _resolve_window(metadata: dict[str, dict[str, str]], summary: Any) -> tuple[dict[str, Any] | None, str]:
    raw_window = metadata.get(WINDOW_METADATA)
    if raw_window:
        try:
            start = int(raw_window['window_start_ns'])
            adm = int(raw_window['admission_start_ns'])
            stop = int(raw_window['stop_requested_ns'])
            closed = int(raw_window.get('raw_ingress_closed_ns', stop))
            if not (0 < start <= adm <= stop <= closed):
                raise ValueError('invalid capture_window boundary ordering')
            buffered = json.loads(raw_window.get('buffered_topics', '[]'))
            return {
                'window_start_ns': start,
                'admission_start_ns': adm,
                'stop_requested_ns': stop,
                'raw_ingress_closed_ns': closed,
                'buffered_topics': set(buffered) if isinstance(buffered, list) else set(),
            }, 'capture_window'
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None, 'invalid_capture_window'

    stats = getattr(summary, 'statistics', None)
    if stats and stats.message_count and stats.message_start_time and stats.message_end_time:
        return {
            'window_start_ns': int(stats.message_start_time),
            'admission_start_ns': int(stats.message_start_time),
            'stop_requested_ns': int(stats.message_end_time),
            'raw_ingress_closed_ns': int(stats.message_end_time),
            'buffered_topics': set(),
        }, 'mcap_summary_fallback'
    return None, 'unavailable'


def _vrdata_gap_attribution(vr_entries: list[tuple[int, dict[str, Any]]], threshold_ns: int) -> list[dict[str, Any]]:
    """Attribute /vrdata log-time gaps to headset generation vs UDP/Wi-Fi arrival vs writer."""
    ordered = sorted(vr_entries, key=lambda item: item[0])
    results = []
    for (log_a, meta_a), (log_b, meta_b) in zip(ordered, ordered[1:]):
        if log_b - log_a <= threshold_ns:
            continue
        seq_a, seq_b = meta_a.get('sequence'), meta_b.get('sequence')
        ts_a, ts_b = meta_a.get('vr_timestamp'), meta_b.get('vr_timestamp')
        rx_a, rx_b = meta_a.get('hc_received_ns'), meta_b.get('hc_received_ns')
        results.append({
            'log_gap_ms': round((log_b - log_a) / 1e6, 2),
            'seq_before': seq_a,
            'seq_after': seq_b,
            'seq_delta': (seq_b - seq_a) if isinstance(seq_a, int) and isinstance(seq_b, int) else None,
            'vr_ts_gap_ms': round((ts_b - ts_a) * 1000, 2) if isinstance(ts_a, (int, float)) and isinstance(ts_b, (int, float)) else None,
            'hc_rx_gap_ms': round((rx_b - rx_a) / 1e6, 2) if isinstance(rx_a, int) and isinstance(rx_b, int) else None,
            'rx_to_mcap_ms': round((log_b - rx_b) / 1e6, 2) if isinstance(rx_b, int) else None,
        })
    return results


def _rgbd_pairing(rows: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    pairs = []
    for topic in sorted(rows):
        if '/color' not in topic or not rows[topic]['log']:
            continue
        depth_topic = topic.replace('/color', '/depth', 1)
        if depth_topic not in rows or not rows[depth_topic]['log']:
            continue
        color = Counter(rows[topic]['header'])
        depth = Counter(rows[depth_topic]['header'])
        color_missing = sum((color - depth).values())
        depth_missing = sum((depth - color).values())
        pairs.append({
            'color_topic': topic,
            'depth_topic': depth_topic,
            'paired_exact': sum((color & depth).values()),
            'color_without_depth': color_missing,
            'depth_without_color': depth_missing,
        })
    return pairs


def inspect_mcap(
    path: str | Path,
    *,
    gap_ms: float = 40.0,
    head_tail_ms: float = 40.0,
    clock_mode: str = 'auto',
    topic_filter: Sequence[str] = (),
    max_gap_samples: int = 5,
    strict: bool = False,
) -> dict[str, Any]:
    """Inspect all channels inside ``path`` for head/tail coverage and frame-interval delays."""
    if gap_ms <= 0 or head_tail_ms <= 0:
        raise ValueError('gap_ms and head_tail_ms must be positive')
    if clock_mode not in {'auto', 'header', 'log'}:
        raise ValueError("clock_mode must be 'auto', 'header', or 'log'")

    target = Path(path)
    threshold_ns = round(gap_ms * 1e6)
    rows: dict[str, dict[str, Any]] = defaultdict(lambda: {
        'schema': '',
        'encoding': '',
        'log': [],
        'header': [],
        'invalid_headers': 0,
    })
    metadata: dict[str, dict[str, str]] = {}
    vr_entries: list[tuple[int, dict[str, Any]]] = []
    summary = None

    with target.open('rb') as stream:
        reader = make_reader(stream, validate_crcs=True)
        summary = reader.get_summary()
        if summary:
            for ch in summary.channels.values():
                sch = summary.schemas.get(ch.schema_id)
                rows[ch.topic]['schema'] = sch.name if sch else ''
                rows[ch.topic]['encoding'] = ch.message_encoding
        for record in reader.iter_metadata():
            metadata[record.name] = dict(record.metadata)
        for schema, channel, message in reader.iter_messages(log_time_order=False):
            if channel.topic == ACTION_PROVENANCE:
                continue
            row = rows[channel.topic]
            if schema and not row['schema']:
                row['schema'] = schema.name
            row['log'].append(message.log_time)
            if row['schema'] in ALL_HEADER_TYPES:
                stamp = header_stamp(message.data)
                if stamp is None:
                    row['invalid_headers'] += 1
                else:
                    row['header'].append(stamp)
            if channel.topic == '/vrdata':
                decoded = _decode_vrdata_timing(message.data)
                if decoded is not None:
                    vr_entries.append((message.log_time, decoded))

    window, window_source = _resolve_window(metadata, summary)
    buffered_topics = window['buffered_topics'] if window else set()
    selected = set(topic_filter)

    topics_report: dict[str, Any] = {}
    empty_topics: list[str] = []
    failures: list[str] = []
    warnings: list[str] = []

    if window_source == 'invalid_capture_window':
        failures.append('capture_window metadata is malformed')
    elif window_source != 'capture_window':
        warnings.append(f'capture_window unavailable ({window_source}); head/tail uses MCAP summary bounds')

    for topic in sorted(rows):
        if selected and topic not in selected:
            continue
        row = rows[topic]
        if not row['log']:
            empty_topics.append(topic)
            if strict:
                failures.append(f'{topic}: 0 messages')
            continue

        has_header = row['schema'] in ALL_HEADER_TYPES
        if clock_mode == 'auto':
            primary_clock = 'log' if (topic in {ACTION, PUBLISHED_ACTION} or not has_header) else 'header'
        elif clock_mode == 'header':
            primary_clock = 'header' if has_header else 'log'
        else:
            primary_clock = 'log'

        clocks: dict[str, Any] = {'log': _clock_metrics(row['log'], threshold_ns, window, max_gap_samples=max_gap_samples)}
        if has_header and row['header']:
            clocks['header'] = _clock_metrics(row['header'], threshold_ns, window, max_gap_samples=max_gap_samples)

        primary = clocks.get(primary_clock) or clocks['log']
        prerolled = (topic in buffered_topics or topic in {ACTION, PUBLISHED_ACTION})
        low_rate = is_inherent_low_rate(topic)

        issues: list[str] = []
        if row['invalid_headers']:
            issues.append(f"invalid_headers={row['invalid_headers']}")
        if primary['duplicates']:
            issues.append(f"duplicates={primary['duplicates']}")
        if primary['out_of_order']:
            issues.append(f"out_of_order={primary['out_of_order']}")
        if primary['gaps_gt_threshold']:
            issues.append(f"gaps>{gap_ms:g}ms={primary['gaps_gt_threshold']} (max={primary['max_ms']}ms)")
        if primary_clock == 'header' and clocks['log']['gaps_gt_threshold'] > primary['gaps_gt_threshold']:
            issues.append(f"log_gaps>{gap_ms:g}ms={clocks['log']['gaps_gt_threshold']} (max={clocks['log']['max_ms']}ms)")

        if window and primary['count'] > 0:
            head_ms = primary['head_vs_window_ms'] if (prerolled and window_source == 'capture_window') else primary['head_vs_admission_ms']
            tail_ms = primary['tail_vs_stop_ms']
            if head_ms is not None and head_ms > head_tail_ms:
                ref = 'window_start' if (prerolled and window_source == 'capture_window') else 'admission_start'
                issues.append(f'head_gap_{ref}={head_ms:.2f}ms>{head_tail_ms:g}ms')
            if tail_ms is not None and tail_ms > head_tail_ms:
                issues.append(f'tail_gap={tail_ms:.2f}ms>{head_tail_ms:g}ms')
            if primary_clock == 'header' and topic not in {ACTION, PUBLISHED_ACTION}:
                if primary.get('before_window') or primary.get('after_stop'):
                    issues.append(f"outside_window=(before:{primary['before_window']},after:{primary['after_stop']})")

        if not issues:
            status = 'PASS'
        elif low_rate and not strict:
            status = 'LOW_RATE'
            warnings.append(f'{topic} (inherent low-rate): ' + '; '.join(issues))
        else:
            status = 'FAIL'
            failures.append(f'{topic}: ' + '; '.join(issues))

        entry: dict[str, Any] = {
            'schema': row['schema'],
            'primary_clock': primary_clock,
            'prerolled': prerolled,
            'inherent_low_rate': low_rate,
            'invalid_headers': row['invalid_headers'],
            'status': status,
            'issues': issues,
            'clocks': clocks,
        }
        if topic == '/vrdata' and vr_entries:
            attribution = _vrdata_gap_attribution(vr_entries, threshold_ns)
            if attribution:
                entry['vr_gap_attribution'] = attribution[:max_gap_samples] if max_gap_samples > 0 else attribution
        topics_report[topic] = entry

    if not topics_report:
        failures.append('no recorded messages found in MCAP')

    rgbd = _rgbd_pairing(rows)
    for pair in rgbd:
        if pair['color_without_depth'] or pair['depth_without_color']:
            failures.append(
                f"RGBD unpaired ({pair['color_topic']} vs {pair['depth_topic']}): "
                f"color_only={pair['color_without_depth']}, depth_only={pair['depth_without_color']}"
            )

    if empty_topics and not strict:
        warnings.append(f'{len(empty_topics)} registered channel(s) have 0 messages')

    result = 'FAIL' if failures else 'WARN' if warnings else 'PASS'
    return {
        'path': str(target.resolve()),
        'size_bytes': target.stat().st_size,
        'gap_threshold_ms': gap_ms,
        'head_tail_threshold_ms': head_tail_ms,
        'clock_mode': clock_mode,
        'window_source': window_source,
        'window': ({k: v for k, v in window.items() if k != 'buffered_topics'}
                   | {'buffered_topics': sorted(buffered_topics),
                      'duration_ms': round((window['stop_requested_ns'] - window['window_start_ns']) / 1e6, 2),
                      'admission_duration_ms': round((window['stop_requested_ns'] - window['admission_start_ns']) / 1e6, 2)})
        if window else None,
        'result': result,
        'topics': topics_report,
        'empty_topics': empty_topics,
        'rgbd': rgbd,
        'failures': failures,
        'warnings': warnings,
    }


def _fmt_ms(value: float | None) -> str:
    return '-' if value is None else f'{value:.2f}'


def format_report(report: dict[str, Any], *, include_empty: bool = False) -> str:
    """Format a human-readable quality summary for terminal inspection."""
    lines = [
        f"MCAP File : {report['path']} ({report['size_bytes']:,} bytes)",
        f"Result    : {report['result']} (gap>{report['gap_threshold_ms']:g}ms, head/tail>{report['head_tail_threshold_ms']:g}ms)",
    ]
    win = report.get('window')
    if win:
        lines.append(
            f"Window    : {report['window_source']} | total={win['duration_ms']:.2f}ms "
            f"(adm={win['admission_duration_ms']:.2f}ms, preroll={(win['admission_start_ns']-win['window_start_ns'])/1e6:.2f}ms)"
        )
    else:
        lines.append(f"Window    : {report['window_source']}")

    lines.append('')
    header = (
        f"{'Topic':42s} {'Clk':6s} {'N':>5s} {'Hz':>6s} "
        f"{'HeadWin':>9s} {'HeadAdm':>9s} {'TailStop':>9s} "
        f"{'p50/p95/max(ms)':>21s} {'>Gap':>5s} {'Dup/OOO':>7s} {'Status'}"
    )
    lines.append(header)
    lines.append('-' * len(header))

    for topic, info in report['topics'].items():
        clk = info['primary_clock']
        m = info['clocks'][clk]
        p_triple = f"{_fmt_ms(m['p50_ms'])}/{_fmt_ms(m['p95_ms'])}/{_fmt_ms(m['max_ms'])}"
        dup_ooo = f"{m['duplicates']}/{m['out_of_order']}"
        hz_txt = '-' if m['hz'] is None else f"{m['hz']:.1f}"
        preroll_mark = '*' if info['prerolled'] else ' '
        lines.append(
            f"{(topic + preroll_mark)[:42]:42s} {clk:6s} {m['count']:5d} {hz_txt:>6s} "
            f"{_fmt_ms(m.get('head_vs_window_ms')):>9s} {_fmt_ms(m.get('head_vs_admission_ms')):>9s} "
            f"{_fmt_ms(m.get('tail_vs_stop_ms')):>9s} {p_triple:>21s} "
            f"{m['gaps_gt_threshold']:5d} {dup_ooo:>7s} {info['status']}"
        )

    lines.append("(* = prerolled/buffered from window_start_ns)")

    gap_sections = []
    for topic, info in report['topics'].items():
        m = info['clocks'][info['primary_clock']]
        samples = m.get('gap_samples') or []
        if samples:
            detail = ', '.join(f"+{s['offset_ms']:.1f}ms:{s['gap_ms']:.2f}ms" for s in samples)
            if m['gaps_gt_threshold'] > len(samples):
                detail += f" ... (+{m['gaps_gt_threshold'] - len(samples)} more)"
            extra = ''
            if 'vr_gap_attribution' in info:
                vr_txt = '; '.join(
                    f"seqΔ={g['seq_delta']} vr={g['vr_ts_gap_ms']}ms rx={g['hc_rx_gap_ms']}ms rx->mcap={g['rx_to_mcap_ms']}ms"
                    for g in info['vr_gap_attribution']
                )
                extra = f"\n    vr_attribution: {vr_txt}"
            gap_sections.append(f"  {topic} [{info['primary_clock']}]: {detail}{extra}")
        elif info['primary_clock'] == 'header' and info['clocks']['log']['gaps_gt_threshold']:
            lm = info['clocks']['log']
            detail = ', '.join(f"+{s['offset_ms']:.1f}ms:{s['gap_ms']:.2f}ms" for s in (lm.get('gap_samples') or []))
            gap_sections.append(f"  {topic} [log only, header OK]: {detail}")

    if gap_sections:
        lines.append('')
        lines.append(f"Inter-frame gaps > {report['gap_threshold_ms']:g} ms:")
        lines.extend(gap_sections)

    if report['rgbd']:
        lines.append('')
        lines.append('RGBD pairing:')
        for pair in report['rgbd']:
            lines.append(
                f"  {pair['color_topic']} <-> {pair['depth_topic']}: "
                f"paired={pair['paired_exact']}, color_only={pair['color_without_depth']}, depth_only={pair['depth_without_color']}"
            )

    if include_empty and report['empty_topics']:
        lines.append('')
        lines.append(f"Empty channels ({len(report['empty_topics'])}): " + ', '.join(report['empty_topics']))
    elif report['empty_topics']:
        lines.append('')
        lines.append(f"Empty channels ({len(report['empty_topics'])}): pass --include-empty to list")

    if report['failures']:
        lines.append('')
        lines.append('Failures:')
        lines.extend(f'  - {item}' for item in report['failures'])
    if report['warnings']:
        lines.append('')
        lines.append('Warnings:')
        lines.extend(f'  - {item}' for item in report['warnings'])

    return '\n'.join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', nargs='?', type=Path, help='MCAP file path (defaults to latest in runtime/topic_recordings)')
    parser.add_argument('--latest', action='store_true', help='Inspect the latest MCAP in runtime/topic_recordings')
    parser.add_argument('--gap-ms', type=float, default=40.0, help='Consecutive frame gap threshold in ms (default: 40)')
    parser.add_argument('--head-tail-ms', type=float, default=40.0, help='Head/tail window gap threshold in ms (default: 40)')
    parser.add_argument('--clock', choices=('auto', 'header', 'log'), default='auto', help='Primary timestamp clock (default: auto)')
    parser.add_argument('--topic', action='append', default=[], dest='topics', help='Filter to specific topic(s), repeatable')
    parser.add_argument('--max-gap-samples', type=int, default=5, help='Max gap examples shown per topic (0 = all, default: 5)')
    parser.add_argument('--include-empty', action='store_true', help='List channels with 0 messages')
    parser.add_argument('--strict', action='store_true', help='Fail on low-rate topics > gap-ms and 0-message channels')
    parser.add_argument('--json', action='store_true', help='Print full JSON report to stdout')
    parser.add_argument('--json-output', type=Path, help='Write full JSON report to path')
    args = parser.parse_args(argv)

    target = find_latest_mcap() if (args.latest or args.path is None) else args.path
    report = inspect_mcap(
        target,
        gap_ms=args.gap_ms,
        head_tail_ms=args.head_tail_ms,
        clock_mode=args.clock,
        topic_filter=args.topics,
        max_gap_samples=args.max_gap_samples,
        strict=args.strict,
    )
    encoded = json.dumps(report, ensure_ascii=False, indent=2)
    if args.json_output:
        args.json_output.write_text(encoded + '\n', encoding='utf-8')
    print(encoded if args.json else format_report(report, include_empty=args.include_empty))
    return 1 if report['result'] == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
