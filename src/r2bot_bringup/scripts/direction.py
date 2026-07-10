#!/usr/bin/env python3
import argparse
import signal
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import serial


FRAME_HEADER = 0x54
FRAME_VER_LEN = 0x2C
POINTS_PER_FRAME = 12
FRAME_SIZE = 47

CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25,
    0x8B, 0xC6, 0x11, 0x5C, 0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 0x1F, 0x52, 0x85, 0xC8,
    0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93,
    0x3D, 0x70, 0xA7, 0xEA, 0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 0x97, 0xDA, 0x0D, 0x40,
    0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04,
    0xAA, 0xE7, 0x30, 0x7D, 0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 0x7C, 0x31, 0xE6, 0xAB,
    0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0,
    0x5E, 0x13, 0xC4, 0x89, 0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 0xCA, 0x87, 0x50, 0x1D,
    0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67,
    0xC9, 0x84, 0x53, 0x1E, 0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7, 0x5D, 0x10, 0xC7, 0x8A,
    0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1,
    0x7F, 0x32, 0xE5, 0xA8,
]


@dataclass
class ScanPoint:
    angle_deg: float
    distance_mm: int
    intensity: int


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc = CRC_TABLE[(crc ^ value) & 0xFF]
    return crc


def normalize_angle(angle_deg: float) -> float:
    return angle_deg % 360.0


def angular_distance(a_deg: float, b_deg: float) -> float:
    diff = abs(normalize_angle(a_deg - b_deg))
    return min(diff, 360.0 - diff)


def parse_frame(frame: bytes, angle_offset_deg: float) -> List[ScanPoint]:
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"invalid frame length: {len(frame)}")
    if frame[0] != FRAME_HEADER or frame[1] != FRAME_VER_LEN:
        raise ValueError("invalid frame header")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("crc mismatch")

    start_angle = int.from_bytes(frame[4:6], byteorder="little") / 100.0
    end_angle = int.from_bytes(frame[42:44], byteorder="little") / 100.0

    if end_angle < start_angle:
        end_angle += 360.0
    step = (end_angle - start_angle) / (POINTS_PER_FRAME - 1)

    points: List[ScanPoint] = []
    for index in range(POINTS_PER_FRAME):
        base = 6 + index * 3
        distance_mm = int.from_bytes(frame[base:base + 2], byteorder="little")
        intensity = frame[base + 2]
        raw_angle = start_angle + step * index
        angle_deg = normalize_angle(raw_angle + angle_offset_deg)
        points.append(ScanPoint(angle_deg=angle_deg, distance_mm=distance_mm, intensity=intensity))
    return points


class LD06Reader:
    def __init__(self, port: str, baudrate: int, timeout: float, angle_offset_deg: float):
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._buffer = bytearray()
        self._angle_offset_deg = angle_offset_deg

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def frames(self) -> Iterable[List[ScanPoint]]:
        while True:
            chunk = self._serial.read(256)
            if not chunk:
                continue
            self._buffer.extend(chunk)

            while True:
                header_index = self._buffer.find(bytes([FRAME_HEADER]))
                if header_index < 0:
                    self._buffer.clear()
                    break
                if header_index > 0:
                    del self._buffer[:header_index]
                if len(self._buffer) < FRAME_SIZE:
                    break

                candidate = bytes(self._buffer[:FRAME_SIZE])
                try:
                    points = parse_frame(candidate, self._angle_offset_deg)
                except ValueError:
                    del self._buffer[0]
                    continue

                del self._buffer[:FRAME_SIZE]
                yield points


class DirectionEstimator:
    def __init__(self, window_deg: float, min_intensity: int, min_distance_mm: int, max_distance_mm: int):
        self._window_deg = window_deg
        self._min_intensity = min_intensity
        self._min_distance_mm = min_distance_mm
        self._max_distance_mm = max_distance_mm
        self._bins_mm: List[Optional[int]] = [None] * 360
        self._last_frame_start_angle: Optional[float] = None

    def update(self, points: Iterable[ScanPoint]) -> Optional[Dict[str, Optional[int]]]:
        points = list(points)
        if not points:
            return None

        frame_start = points[0].angle_deg
        scan_completed = False
        if self._last_frame_start_angle is not None:
            # A wrap from a large angle back to a small angle marks a new full turn.
            scan_completed = frame_start + 5.0 < self._last_frame_start_angle
        self._last_frame_start_angle = frame_start

        for point in points:
            if point.distance_mm < self._min_distance_mm or point.distance_mm > self._max_distance_mm:
                continue
            if point.intensity < self._min_intensity:
                continue

            index = int(round(normalize_angle(point.angle_deg))) % 360
            current = self._bins_mm[index]
            if current is None or point.distance_mm < current:
                self._bins_mm[index] = point.distance_mm

        if not scan_completed:
            return None

        result = {
            "front": self._sector_min(0.0),
            "right": self._sector_min(90.0),
            "back": self._sector_min(180.0),
            "left": self._sector_min(270.0),
        }
        self._bins_mm = [None] * 360
        return result

    def _sector_min(self, center_deg: float) -> Optional[int]:
        matches = [
            distance_mm
            for angle_deg, distance_mm in enumerate(self._bins_mm)
            if distance_mm is not None and angular_distance(angle_deg, center_deg) <= self._window_deg
        ]
        if not matches:
            return None
        return min(matches)


def format_distance(distance_mm: Optional[int]) -> str:
    if distance_mm is None:
        return "N/A"
    return f"{distance_mm} mm ({distance_mm / 1000.0:.3f} m)"


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read LD06 data from serial and print obstacle distance in four directions."
    )
    parser.add_argument("--port", default="/dev/lidar_ld06", help="serial device path")
    parser.add_argument("--baudrate", type=int, default=230400, help="serial baudrate")
    parser.add_argument("--timeout", type=float, default=0.1, help="serial read timeout in seconds")
    parser.add_argument(
        "--offset",
        type=float,
        default=0.0,
        help="angle offset in degrees applied to the lidar front direction",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=10.0,
        help="half window size in degrees for each direction sector",
    )
    parser.add_argument(
        "--min-intensity",
        type=int,
        default=0,
        help="ignore points with intensity lower than this value",
    )
    parser.add_argument(
        "--min-distance-mm",
        type=int,
        default=1,
        help="ignore points closer than this distance",
    )
    parser.add_argument(
        "--max-distance-mm",
        type=int,
        default=12000,
        help="ignore points farther than this distance",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="minimum seconds between printed outputs",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()

    running = True

    def stop_handler(signum, frame) -> None:  # type: ignore[override]
        del signum, frame
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    reader = None
    try:
        reader = LD06Reader(
            port=args.port,
            baudrate=args.baudrate,
            timeout=args.timeout,
            angle_offset_deg=args.offset,
        )
        estimator = DirectionEstimator(
            window_deg=args.window,
            min_intensity=args.min_intensity,
            min_distance_mm=args.min_distance_mm,
            max_distance_mm=args.max_distance_mm,
        )

        print(
            f"Reading LD06 from {args.port} at {args.baudrate} bps; "
            f"front=0 deg, right=90 deg, back=180 deg, left=270 deg, offset={args.offset} deg",
            flush=True,
        )
        last_print_time = 0.0
        for points in reader.frames():
            if not running:
                break
            distances = estimator.update(points)
            if distances is None:
                continue
            now = time.monotonic()
            if now - last_print_time < args.interval:
                continue
            last_print_time = now
            print(
                "front={front} | right={right} | back={back} | left={left}".format(
                    front=format_distance(distances["front"]),
                    right=format_distance(distances["right"]),
                    back=format_distance(distances["back"]),
                    left=format_distance(distances["left"]),
                ),
                flush=True,
            )
    except serial.SerialException as exc:
        print(f"Failed to open/read serial port {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        if reader is not None:
            reader.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
