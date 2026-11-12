#!/usr/bin/env python3
"""Read-only Linux Wi-Fi preflight. JSON is written to stdout; no AP is changed."""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from datetime import datetime, timezone


def run_iw(arguments, errors):
    try:
        result = subprocess.run(['iw', *arguments], capture_output=True, text=True,
                                timeout=12, check=False)
        if result.returncode:
            errors.append({'command': ['iw', *arguments], 'error': result.stderr.strip()})
        return result.stdout if result.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired) as exc:
        errors.append({'command': ['iw', *arguments], 'error': str(exc)})
        return ''


def parse_survey(text):
    result = {}
    current = None
    for line in text.splitlines():
        match = re.search(r'frequency:\s*(\d+)', line)
        if match:
            current = result.setdefault(int(match[1]), {})
        elif current is not None:
            match = re.search(r'channel (active|busy|receive|transmit) time:\s*(\d+)', line)
            if match:
                current[match[1]] = int(match[2])
            match = re.search(r'noise:\s*(-?\d+)', line)
            if match:
                current['noise_dbm'] = int(match[1])
    return result


def busy_rates(before, after):
    """Only fresh same-channel counter deltas qualify as measured airtime."""
    result = {}
    for frequency, sample in after.items():
        previous = before.get(frequency, {})
        active = sample.get('active', 0) - previous.get('active', 0)
        busy = sample.get('busy', 0) - previous.get('busy', 0)
        valid = ('active' in previous and 'busy' in previous and
                 'active' in sample and 'busy' in sample and active > 0 and 0 <= busy <= active)
        result[frequency] = {'busy_pct': round(100 * busy / active, 2) if valid else None,
                             'observed_ms': active if valid else None,
                             'noise_dbm': sample.get('noise_dbm')}
    return result


def parse_scan(text):
    access_points = []
    current = None
    for line in text.splitlines():
        if re.match(r'^BSS\s', line):
            current = {'frequency_mhz': None, 'signal_dbm': None}
            access_points.append(current)
        elif current is not None:
            match = re.search(r'freq:\s*(\d+)', line)
            if match:
                current['frequency_mhz'] = int(match[1])
            match = re.search(r'signal:\s*(-?[\d.]+)', line)
            if match:
                current['signal_dbm'] = float(match[1])
    return [ap for ap in access_points if ap['frequency_mhz'] is not None]


def parse_phy(text):
    result = {}
    for line in text.splitlines():
        match = re.search(r'\*\s*(\d+) MHz\s*\[(\d+)\](.*)', line)
        if match:
            flags = match[3].lower()
            result[int(match[1])] = {'channel': int(match[2]),
                                     'disabled': 'disabled' in flags,
                                     'no_ir': 'no ir' in flags or 'no-ir' in flags,
                                     'dfs': 'radar' in flags}
    return result


def channel_groups(phy, width):
    """5 GHz 20/40/80 MHz blocks; avoid inventing device/regdomain support."""
    blocks = ((36, 40, 44, 48), (52, 56, 60, 64), (100, 104, 108, 112),
              (116, 120, 124, 128), (132, 136, 140, 144), (149, 153, 157, 161))
    size = width // 20
    groups = []
    for block in blocks:
        for start in range(0, 4, size):
            channels = block[start:start + size]
            frequencies = [5000 + 5 * channel for channel in channels]
            if all(frequency in phy for frequency in frequencies):
                groups.append((channels, frequencies))
    if width == 20 and 5825 in phy:
        groups.append(((165,), [5825]))
    return groups


def make_report(info, link, phy_text, reg_text, scan_text, before, after, width, active_scan, errors):
    phy = parse_phy(phy_text)
    aps = parse_scan(scan_text)
    measurements = busy_rates(parse_survey(before), parse_survey(after))
    current = re.search(r'freq:\s*(\d+)', link)
    current_frequency = int(current[1]) if current else None
    candidates = []
    for channels, frequencies in channel_groups(phy, width):
        flags = [phy[f] for f in frequencies]
        disabled = any(flag['disabled'] for flag in flags)
        dfs = any(flag['dfs'] for flag in flags)
        no_ir = any(flag['no_ir'] for flag in flags)
        # Neighbor widths may be unavailable; include nearby primaries conservatively.
        nearby = [ap for ap in aps if min(abs(ap['frequency_mhz'] - f) for f in frequencies) < 80]
        candidate = {'channels': list(channels), 'width_mhz': width,
                     'current_group': current_frequency in frequencies,
                     'disabled': disabled, 'dfs': dfs, 'no_ir': no_ir,
                     'observed_nearby_bss': len(nearby),
                     'strongest_nearby_dbm': max((ap['signal_dbm'] for ap in nearby
                                               if ap['signal_dbm'] is not None), default=None),
                     'per_20mhz_measurements': {str(f): measurements.get(f, {'busy_pct': None})
                                               for f in frequencies},
                     'recommendation_eligible': not (disabled or dfs or no_ir)}
        candidates.append(candidate)
    eligible = sorted((item for item in candidates if item['recommendation_eligible']),
                      key=lambda item: (item['observed_nearby_bss'], item['strongest_nearby_dbm'] or -110))
    suggested = eligible[0]['channels'] if eligible and scan_text.strip() else None
    warnings = [
        '测量仅代表本机位置，不能代表头显；有线连接需 AP/头显侧遥测。',
        '邻居 AP 数不等于空口忙碌率；未测量字段为 null，不代表空闲。',
        '候选按指定带宽分组；邻居带宽未知，邻近 BSS 计数保守估计，不能保证最佳信道。',
        'DFS/no-IR 候选仅展示，需核实 AP、头显及监管规则；未自动修改信道。',
        '本机支持不代表 AP/头显支持；切换前必须复测。',
    ]
    if not active_scan:
        warnings.append('使用缓存扫描结果，可能过时；低置信度建议需在遥操作前主动扫描确认。')
    return {'schema_version': 1, 'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'connected' if current_frequency else 'unavailable_or_disconnected',
            'interface_info': info.strip(), 'link': link.strip(), 'regulatory': reg_text.strip(),
            'current_frequency_mhz': current_frequency,
            'current_airtime': measurements.get(current_frequency, {'busy_pct': None}),
            'scan_mode': 'active' if active_scan else 'cached',
            'candidate_confidence': 'low', 'suggested_trial_channels': suggested,
            'candidates': candidates, 'warnings': warnings, 'errors': errors}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', help='Wireless interface; default: first iw interface')
    parser.add_argument('--width', type=int, choices=(20, 40, 80), default=80)
    parser.add_argument('--sample-seconds', type=float, default=1.0)
    parser.add_argument('--active-scan', action='store_true',
                        help='Only before teleoperation: scanning can interrupt an active link')
    args = parser.parse_args(argv)
    if not 0.1 <= args.sample_seconds <= 10:
        parser.error('--sample-seconds must be between 0.1 and 10')
    errors = []
    interfaces = run_iw(['dev'], errors)
    match = re.search(r'Interface\s+(\S+)', interfaces)
    interface = args.interface or (match[1] if match else None)
    if interface is None:
        print(json.dumps({'schema_version': 1, 'status': 'no_wireless_interface',
                          'warnings': ['没有可测量无线网卡；请使用 AP/头显侧遥测。'], 'errors': errors}, ensure_ascii=False))
        return 0
    info = run_iw(['dev', interface, 'info'], errors)
    link = run_iw(['dev', interface, 'link'], errors)
    wiphy = re.search(r'wiphy\s+(\d+)', info)
    phy = run_iw(['phy', 'phy' + wiphy[1], 'info'], errors) if wiphy else ''
    reg = run_iw(['reg', 'get'], errors)
    if args.active_scan:
        print('警告：主动扫描可能打断无线传输；仅在遥操作开始前运行。', file=sys.stderr)
        scan = run_iw(['dev', interface, 'scan'], errors)
    else:
        scan = run_iw(['dev', interface, 'scan', 'dump'], errors)
    before = run_iw(['dev', interface, 'survey', 'dump'], errors)
    time.sleep(args.sample_seconds)
    after = run_iw(['dev', interface, 'survey', 'dump'], errors)
    print(json.dumps(make_report(info, link, phy, reg, scan, before, after,
                                 args.width, args.active_scan, errors), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
