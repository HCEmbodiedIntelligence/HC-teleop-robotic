#!/usr/bin/env python3
"""Capture bounded status samples for offline QoS A/B comparison."""
import argparse
import json
import time
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:7876/api/camera/status')
    parser.add_argument('--seconds', type=int, default=60)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 3600:
        parser.error('--seconds must be 1..3600')
    samples = []
    for _ in range(args.seconds):
        started = time.monotonic()
        with urlopen(args.url, timeout=3) as response:
            data = json.load(response)
        samples.append({'captured_at': time.time(), **data})
        time.sleep(max(0, 1 - (time.monotonic() - started)))
    with open(args.output, 'w') as output:
        json.dump({'samples': samples}, output, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
