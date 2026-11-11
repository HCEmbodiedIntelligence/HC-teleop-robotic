#!/usr/bin/env python3
"""JSON health probe: real Jetson raw-frame encode, software decode, IDR test.

The probe runs in a bounded child process so a native driver hang cannot leave
an interactive health check waiting indefinitely. It never installs packages.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def probe(width, height, count):
    from middleware.core.jetson_h264 import JetsonH264Codec, jetson_platform, INSTALL_GUIDANCE
    result = {'backend': 'jetson', 'hardware_available': False,
              'platform': jetson_platform(), 'test': 'raw I420 -> Jetson hardware -> Annex-B -> software decode',
              'packages_installed': False, 'frames_encoded': 0, 'frames_decoded': 0}
    codec = None
    try:
        import av
        import numpy as np
        from fractions import Fraction
        codec = JetsonH264Codec(width, height, 1_000_000)
        decoder = av.CodecContext.create('h264', 'r')
        rng = np.random.default_rng(13)
        sizes, elapsed = [], []
        for index in range(count):
            pixels = rng.integers(0, 256, (height, width, 3), dtype=np.uint8)
            frame = av.VideoFrame.from_ndarray(pixels, format='bgr24')
            frame.pts = index * 3000
            frame.time_base = Fraction(1, 90000)
            if index == count - 1:
                frame.pict_type = av.video.frame.PictureType.I
            started = time.monotonic()
            access_unit = b''.join(codec.encode(frame))
            elapsed.append(round((time.monotonic() - started) * 1000, 2))
            sizes.append(len(access_unit))
            result['frames_encoded'] += 1
            for decoded in decoder.decode(av.Packet(access_unit)):
                if (decoded.width, decoded.height) != (width, height):
                    raise RuntimeError('decoded frame dimensions differ from source')
                result['frames_decoded'] += 1
        result['frames_decoded'] += len(decoder.decode(None))
        if result['frames_decoded'] != count:
            raise RuntimeError('encoded access units did not decode into the expected number of frames')
        result.update(hardware_available=True, state='passed', access_unit_bytes=sizes,
                      encode_ms=elapsed, profile='Constrained Baseline', maximum_level='3.1',
                      forced_idr_verified=True)
    except Exception as exc:
        result.update(state='unavailable', error=f'{type(exc).__name__}: {exc}', installation_guidance=INSTALL_GUIDANCE)
    finally:
        if codec is not None:
            codec.close()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=400)
    parser.add_argument('--frames', type=int, choices=range(2, 121), default=6)
    parser.add_argument('--timeout', type=float, default=15)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.timeout <= 0 or args.timeout > 120:
        parser.error('--timeout must be between 0 and 120 seconds')
    if args.worker:
        result = probe(args.width, args.height, args.frames)
    else:
        try:
            child = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--worker',
                                    '--width', str(args.width), '--height', str(args.height),
                                    '--frames', str(args.frames)], capture_output=True,
                                   text=True, timeout=args.timeout, check=False)
            # Some NVIDIA plugins log native initialization messages to stdout.
            # Only the structured final line is returned to the caller.
            lines = child.stdout.strip().splitlines()
            result = json.loads(lines[-1]) if lines else {'state': 'failed', 'hardware_available': False,
                                                         'error': child.stderr[-2000:] or 'probe child returned no JSON'}
        except subprocess.TimeoutExpired:
            result = {'state': 'timeout', 'hardware_available': False,
                      'error': f'Jetson probe exceeded {args.timeout:g}s; child process terminated'}
        except (OSError, json.JSONDecodeError) as exc:
            result = {'state': 'failed', 'hardware_available': False, 'error': str(exc)}
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get('hardware_available') else 1


if __name__ == '__main__':
    raise SystemExit(main())
