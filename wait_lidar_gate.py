#!/usr/bin/env python3
"""Wait for the LD06 trigger sequence before starting align_once.launch.py.

Sequence:
  1. front and left are both below --threshold-mm continuously for --hold-s.
  2. after that, front and left are both above --threshold-mm.
  3. sleep --post-delay-s, then exit 0.

This uses the same direct serial LD06 parser and direction estimator as
r2bot_bringup/scripts/align_to_obstacles.py when source:=serial.
"""
import argparse
import os
import sys
import time
from typing import Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BRINGUP_SCRIPTS = os.path.join(SCRIPT_DIR, 'src', 'r2bot_bringup', 'scripts')
if BRINGUP_SCRIPTS not in sys.path:
    sys.path.insert(0, BRINGUP_SCRIPTS)

from direction import DirectionEstimator, LD06Reader, format_distance  # noqa: E402


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Wait for LD06 front/left trigger before alignment')
    parser.add_argument('--port', default='/dev/lidar_ld06', help='LD06 serial device path')
    parser.add_argument('--baudrate', type=int, default=230400, help='LD06 serial baudrate')
    parser.add_argument('--timeout', type=float, default=0.1, help='serial read timeout in seconds')
    parser.add_argument('--offset', type=float, default=0.0, help='angle offset in degrees')
    parser.add_argument('--window', type=float, default=10.0, help='half window in degrees for direction sectors')
    parser.add_argument('--min-intensity', type=int, default=0, help='minimum point intensity')
    parser.add_argument('--max-distance-mm', type=int, default=12000, help='maximum valid distance')
    parser.add_argument('--threshold-mm', type=int, default=20, help='front/left trigger threshold')
    parser.add_argument('--hold-s', type=float, default=2.0, help='seconds below threshold before arming')
    parser.add_argument('--post-delay-s', type=float, default=10.0, help='seconds to wait after release')
    parser.add_argument('--near-grace-s', type=float, default=1.0, help='keep counting brief N/A readings after a near hit')
    parser.add_argument('--status-interval-s', type=float, default=0.5, help='seconds between status logs')
    return parser


def is_near(value: Optional[int], threshold_mm: int) -> bool:
    return value is not None and value < threshold_mm


def below_threshold(front: Optional[int], left: Optional[int], threshold_mm: int, near_grace_active: bool) -> bool:
    if front is not None and left is not None:
        return front < threshold_mm and left < threshold_mm
    return near_grace_active and (front is None or front < threshold_mm) and (left is None or left < threshold_mm)


def above_threshold(front: Optional[int], left: Optional[int], threshold_mm: int) -> bool:
    return front is not None and left is not None and front > threshold_mm and left > threshold_mm


def main() -> int:
    args = build_argparser().parse_args()
    estimator = DirectionEstimator(
        window_deg=args.window,
        min_intensity=args.min_intensity,
        min_distance_mm=1,
        max_distance_mm=args.max_distance_mm,
    )

    print(
        f'等待雷达触发: front 和 left 都 < {args.threshold_mm} mm 持续 {args.hold_s:.1f}s，'
        f'然后都 > {args.threshold_mm} mm 后等待 {args.post_delay_s:.1f}s；短暂 N/A 容忍 {args.near_grace_s:.1f}s',
        flush=True,
    )
    print(
        f'读取 {args.port}，方向规则: front=0 deg, right=90 deg, back=180 deg, left=270 deg, window=+/-{args.window} deg',
        flush=True,
    )

    low_since: Optional[float] = None
    near_until = 0.0
    armed = False
    last_status = 0.0

    reader = None
    try:
        reader = LD06Reader(
            port=args.port,
            baudrate=args.baudrate,
            timeout=args.timeout,
            angle_offset_deg=args.offset,
        )
        for points in reader.frames():
            distances = estimator.update(points)
            if distances is None:
                continue

            now = time.monotonic()
            front = distances.get('front')
            left = distances.get('left')

            if is_near(front, args.threshold_mm) or is_near(left, args.threshold_mm):
                near_until = now + args.near_grace_s
            near_grace_active = now <= near_until

            if now - last_status >= args.status_interval_s:
                state = 'armed_wait_release' if armed else 'wait_low_hold'
                print(
                    f'gate state={state} front={format_distance(front)} left={format_distance(left)} near_grace={near_grace_active}',
                    flush=True,
                )
                last_status = now

            if not armed:
                if below_threshold(front, left, args.threshold_mm, near_grace_active):
                    if low_since is None:
                        low_since = now
                        print('front/left 已低于阈值，开始计时', flush=True)
                    elif now - low_since >= args.hold_s:
                        armed = True
                        print('低于阈值持续时间已满足，等待 front/left 都重新大于阈值', flush=True)
                else:
                    if low_since is not None:
                        print('低于阈值计时中断，重新等待', flush=True)
                    low_since = None
                continue

            if above_threshold(front, left, args.threshold_mm):
                print(f'front/left 已大于阈值，等待 {args.post_delay_s:.1f}s 后启动对齐', flush=True)
                time.sleep(args.post_delay_s)
                return 0
    except Exception as exc:
        print(f'雷达触发脚本失败: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if reader is not None:
            reader.close()

    return 1


if __name__ == '__main__':
    raise SystemExit(main())
