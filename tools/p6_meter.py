#!/usr/bin/env python3

import argparse
import curses
import glob
import json
import subprocess
import sys
import tempfile
import time
from collections import deque
from pathlib import Path


WINDOW_S = 5.0


def clamp(value: float, minimum: float = 0.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


class EventFollower:
    def __init__(self, out_dir: Path) -> None:
        self.pattern = str(out_dir / "events_*.jsonl")
        self.offsets: dict[str, int] = {}

    def read_new(self) -> list[dict]:
        events = []
        now = time.monotonic()
        for path in sorted(glob.glob(self.pattern)):
            try:
                with open(path, encoding="utf-8") as stream:
                    offset = self.offsets.get(path, 0)
                    stream.seek(offset)
                    while line := stream.readline():
                        end = stream.tell()
                        try:
                            event = json.loads(line)
                        except json.JSONDecodeError:
                            break
                        event["_received"] = now
                        events.append(event)
                        offset = end
                    self.offsets[path] = offset
            except FileNotFoundError:
                continue
        return events


def number(event: dict, field: str, default: float = 0.0) -> float:
    value = event.get(field)
    if isinstance(value, (int, float)):
        return float(value)
    return default


def event_metrics(event: dict) -> dict[str, float]:
    quality = clamp(number(event, "q") / 255.0)
    mean_margin = clamp(number(event, "mean_margin"))
    min_margin = clamp(number(event, "min_margin"))
    lock = clamp(number(event, "lock"))
    snr_db = number(event, "p6snr_db", number(event, "snr_db"))
    snr = clamp((snr_db + 5.0) / 30.0)
    overall = (
        0.35 * quality
        + 0.25 * mean_margin
        + 0.15 * min_margin
        + 0.15 * lock
        + 0.10 * snr
    )
    return {
        "overall": overall,
        "viterbi": quality,
        "mean_margin": mean_margin,
        "min_margin": min_margin,
        "lock": lock,
        "snr": snr,
        "snr_db": snr_db,
    }


class Meter:
    def __init__(self) -> None:
        self.events: deque[tuple[float, dict]] = deque()
        self.maximums: dict[str, float] = {}

    def add(self, events: list[dict]) -> None:
        for event in events:
            self.events.append((event["_received"], event))

    def snapshot(self) -> dict:
        cutoff = time.monotonic() - WINDOW_S
        while self.events and self.events[0][0] < cutoff:
            self.events.popleft()

        p6 = []
        ssr_count = 0
        accepted_count = 0
        repeated_count = 0
        for _, event in self.events:
            event_type = event.get("type")
            if event_type == "SSR":
                ssr_count += 1
            elif event_type in ("P6", "P6_REJECTED"):
                p6.append(event_metrics(event))
                if event_type == "P6":
                    accepted_count += 1
                    if event.get("status") in ("repeated", "confirmed"):
                        repeated_count += 1

        values = {
            key: max((frame[key] for frame in p6), default=0.0)
            for key in (
                "overall",
                "viterbi",
                "mean_margin",
                "min_margin",
                "lock",
                "snr",
            )
        }
        values["snr_db"] = max(
            (frame["snr_db"] for frame in p6), default=-999.0
        )
        values["ssr"] = clamp(ssr_count / 10.0)
        for key, value in values.items():
            if key != "snr_db":
                self.maximums[key] = max(self.maximums.get(key, 0.0), value)
        return {
            **values,
            "p6_count": len(p6),
            "accepted_count": accepted_count,
            "repeated_count": repeated_count,
            "ssr_count": ssr_count,
        }

    def reset(self) -> None:
        self.maximums.clear()


def bar(value: float, width: int) -> str:
    filled = round(clamp(value) * width)
    return "#" * filled + "-" * (width - filled)


def draw_line(
    screen, row: int, label: str, value: float, maximum: float,
    value_text: str, width: int, new_max: bool = False,
) -> int:
    bar_width = max(10, min(28, width - 52))
    relative = 100.0 * value / maximum if maximum > 0 else 0.0
    suffix = " NEW MAX" if new_max else ""
    text = (
        f"{label:<14} [{bar(value, bar_width)}] "
        f"{value_text:<9} {relative:5.0f}% max{suffix}"
    )
    screen.addnstr(row, 0, text, max(0, width - 1))
    return row + 1


def draw(screen, meter: Meter, process: subprocess.Popen, dump_args: list[str]) -> None:
    snapshot = meter.snapshot()
    height, width = screen.getmaxyx()
    if height < 15 or width < 72:
        screen.erase()
        screen.addstr(0, 0, "Terminal too small (need at least 72x15).")
        screen.refresh()
        return

    screen.erase()
    screen.addnstr(0, 0, "p6_meter - live 1030 MHz decoder quality", width - 1)
    screen.addnstr(
        1, 0,
        f"Last {WINDOW_S:g}s | r reset maximum | q quit | PID {process.pid}",
        width - 1,
    )
    screen.hline(2, 0, "-", width - 1)
    row = 4
    metrics = (
        ("Overall", "overall", f"{snapshot['overall'] * 100:5.1f}%"),
        ("Viterbi", "viterbi", f"{snapshot['viterbi'] * 100:5.1f}%"),
        ("Mean margin", "mean_margin", f"{snapshot['mean_margin']:5.3f}"),
        ("Min margin", "min_margin", f"{snapshot['min_margin']:5.3f}"),
        ("SPR lock", "lock", f"{snapshot['lock']:5.3f}"),
        ("P6 SNR", "snr", "n/a" if snapshot["snr_db"] <= -900 else
         f"{snapshot['snr_db']:+5.1f}dB"),
        ("SSR activity", "ssr", f"{snapshot['ssr_count'] / WINDOW_S:5.1f}/s"),
    )
    for label, key, value_text in metrics:
        value = snapshot[key]
        maximum = meter.maximums.get(key, 0.0)
        row = draw_line(
            screen, row, label, value, maximum, value_text, width,
            maximum > 0.0 and abs(value - maximum) < 1e-12,
        )

    row += 1
    screen.addnstr(
        row, 0,
        f"P6 candidates: {snapshot['p6_count']}  "
        f"accepted: {snapshot['accepted_count']}  "
        f"repeated: {snapshot['repeated_count']}  "
        f"SSR: {snapshot['ssr_count']}",
        width - 1,
    )
    row += 2
    if process.poll() is not None:
        screen.addnstr(
            row, 0,
            f"dump1030 exited with code {process.returncode}; press q",
            width - 1,
        )
    elif not snapshot["p6_count"] and not snapshot["ssr_count"]:
        screen.addnstr(row, 0, "Waiting for decoder events ...", width - 1)
    else:
        screen.addnstr(row, 0, "NEW MAX highlights the current session peak.", width - 1)
    row += 1
    screen.addnstr(row, 0, "dump1030: " + " ".join(dump_args), width - 1)
    screen.refresh()


def run_screen(
    screen, process: subprocess.Popen, follower: EventFollower,
    meter: Meter, dump_args: list[str],
) -> None:
    curses.curs_set(0)
    screen.nodelay(True)
    while True:
        meter.add(follower.read_new())
        draw(screen, meter, process, dump_args)
        deadline = time.monotonic() + 0.20
        while time.monotonic() < deadline:
            key = screen.getch()
            if key in (ord("q"), ord("Q")):
                return
            if key in (ord("r"), ord("R")):
                meter.reset()
            time.sleep(0.02)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live 1030 MHz decoder-quality meter for antenna positioning."
    )
    parser.add_argument("--binary", help="dump1030 binary")
    parser.add_argument("--device", default="0", help="RTL-SDR device index")
    parser.add_argument("--gain", default="max", help="RTL-SDR gain")
    parser.add_argument("--chan", default="chan.txt", help="DPSK channel file")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    binary = (
        Path(args.binary).resolve()
        if args.binary
        else script_dir.parent / "dump1030"
    )
    channel = Path(args.chan).resolve()
    if not binary.is_file():
        print(f"dump1030 not found: {binary}", file=sys.stderr)
        return 2
    if not channel.is_file():
        print(f"channel file not found: {channel}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="p6-meter-") as temp:
        out_dir = Path(temp)
        dump_args = [
            str(binary),
            "--device", args.device,
            "--gain", args.gain,
            "--chan", str(channel),
            "--uplink-decode",
            "--continuous",
            "--p6-diagnostics",
            "--out-dir", str(out_dir),
        ]
        stderr_path = out_dir / "dump1030.stderr"
        with stderr_path.open("wb") as stderr:
            try:
                process = subprocess.Popen(dump_args, stdout=subprocess.DEVNULL, stderr=stderr)
            except OSError as exc:
                print(f"could not start dump1030: {exc}", file=sys.stderr)
                return 2

            try:
                curses.wrapper(
                    run_screen, process, EventFollower(out_dir), Meter(), dump_args
                )
            except curses.error as exc:
                print(f"terminal UI failed: {exc}", file=sys.stderr)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()

        if process.returncode not in (0, -15):
            error = stderr_path.read_text(errors="replace").strip()
            print(f"dump1030 exited with code {process.returncode}", file=sys.stderr)
            if error:
                print(error, file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
