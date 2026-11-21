#!/usr/bin/env python3
"""Parallel stream-downsample interleaved sc16 I/Q files to cu8 with FIR anti-aliasing."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import os
from pathlib import Path

import numpy as np
from scipy.signal import firwin, lfilter

_IN_MMAP: np.memmap | None = None
_OUT_FD: int = -1
_B: np.ndarray | None = None
_DECIMATION: int = 4
_TAPS: int = 63
_GAIN: np.float32 = np.float32(1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Parallel downsample of interleaved sc16 I/Q to cu8."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--decimation", type=int, default=4)
    parser.add_argument("--taps", type=int, default=63)
    parser.add_argument("--cutoff-ratio", type=float, default=0.8)
    parser.add_argument("--sc16-scale", type=float, default=2048.0)
    parser.add_argument("--chunk-samples", type=int, default=4_000_000)
    parser.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    return parser.parse_args()


def _init_worker(
    in_path: str,
    out_path: str,
    b: np.ndarray,
    decimation: int,
    taps: int,
    gain: float,
) -> None:
    global _IN_MMAP, _OUT_FD, _B, _DECIMATION, _TAPS, _GAIN
    _IN_MMAP = np.memmap(in_path, dtype="<i2", mode="r").reshape(-1, 2)
    _OUT_FD = os.open(out_path, os.O_WRONLY)
    _B = b
    _DECIMATION = decimation
    _TAPS = taps
    _GAIN = np.float32(gain)


def _process_chunk(task: tuple[int, int]) -> int:
    start, count = task
    overlap = _TAPS - 1
    read_start = max(0, start - overlap)
    pad_left = overlap - (start - read_start)

    raw = _IN_MMAP[read_start : start + count]
    if pad_left > 0:
        iq = np.empty((pad_left + len(raw), 2), dtype=np.float32)
        iq[:pad_left] = 0.0
        iq[pad_left:] = raw
    else:
        iq = raw.astype(np.float32)

    filtered = lfilter(_B, [1.0], iq, axis=0)
    valid = filtered[overlap:]
    decimated = valid[:: _DECIMATION]

    cu8 = np.clip(np.rint(decimated * _GAIN + np.float32(127.5)), 0, 255).astype(
        np.uint8
    )
    out_offset_bytes = (start // _DECIMATION) * 2
    data = cu8.tobytes()

    written = 0
    while written < len(data):
        written += os.pwrite(_OUT_FD, data[written:], out_offset_bytes + written)

    return len(cu8)


def main() -> int:
    args = parse_args()
    if args.decimation < 2:
        raise SystemExit("--decimation must be >= 2")
    if args.taps < 3 or args.taps % 2 == 0:
        raise SystemExit("--taps must be an odd integer >= 3")
    if not (0.0 < args.cutoff_ratio < 1.0):
        raise SystemExit("--cutoff-ratio must be between 0 and 1")
    if args.sc16_scale <= 0:
        raise SystemExit("--sc16-scale must be positive")
    if args.workers < 1:
        raise SystemExit("--workers must be >= 1")

    chunk_samples = (args.chunk_samples // args.decimation) * args.decimation
    if chunk_samples <= 0:
        raise SystemExit("--chunk-samples must be >= --decimation")

    file_size = args.input.stat().st_size
    if file_size % 4 != 0:
        raise SystemExit("Input file size is not a multiple of 4 bytes (sc16)")
    total_samples = file_size // 4
    total_out_samples = (total_samples + args.decimation - 1) // args.decimation

    b = firwin(args.taps, cutoff=args.cutoff_ratio / args.decimation).astype(np.float32)
    gain = 127.5 / args.sc16_scale

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as fout:
        fout.truncate(total_out_samples * 2)

    tasks = [
        (start, min(chunk_samples, total_samples - start))
        for start in range(0, total_samples, chunk_samples)
    ]

    written_samples = 0
    with ProcessPoolExecutor(
        max_workers=args.workers,
        initializer=_init_worker,
        initargs=(
            str(args.input),
            str(args.output),
            b,
            args.decimation,
            args.taps,
            gain,
        ),
    ) as pool:
        for count in pool.map(_process_chunk, tasks, chunksize=1):
            written_samples += count

    print(f"input_samples={total_samples} output_samples={written_samples}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())