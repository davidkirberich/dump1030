#!/usr/bin/env python3

import argparse
import curses
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from collections import Counter, deque
from pathlib import Path


WINDOW_S = 5.0


def clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def parse_raw_message(line: str) -> dict | None:
    text = line.strip()
    if len(text) < 16 or text[0] != "*" or text[-1] != ";":
        return None
    payload = text[1:-1]
    if len(payload) not in (14, 28):
        return None
    try:
        data = bytes.fromhex(payload)
    except ValueError:
        return None

    df = (data[0] >> 3) & 0x1F
    address = None
    if df in (17, 18):
        address = data[1:4].hex().upper()
    return {
        "received": time.monotonic(),
        "df": df,
        "address": address,
        "bits": len(data) * 8,
    }


class RawReader(threading.Thread):
    def __init__(self, stream) -> None:
        super().__init__(daemon=True)
        self.stream = stream
        self.messages: deque[dict] = deque()
        self.lock = threading.Lock()

    def run(self) -> None:
        for line in self.stream:
            message = parse_raw_message(line)
            if message is None:
                continue
            with self.lock:
                self.messages.append(message)

    def drain(self) -> list[dict]:
        with self.lock:
            messages = list(self.messages)
            self.messages.clear()
        return messages


class Meter:
    def __init__(self) -> None:
        self.messages: deque[dict] = deque()
        self.maximums: dict[str, float] = {}

    def add(self, messages: list[dict]) -> None:
        self.messages.extend(messages)

    def snapshot(self) -> dict:
        cutoff = time.monotonic() - WINDOW_S
        while self.messages and self.messages[0]["received"] < cutoff:
            self.messages.popleft()

        dfs = Counter(message["df"] for message in self.messages)
        addresses = Counter(
            message["address"]
            for message in self.messages
            if message["address"] is not None
        )
        repeated_addresses = sum(count >= 2 for count in addresses.values())
        count = len(self.messages)
        values = {
            "overall": clamp(
                0.60 * clamp(count / 25.0)
                + 0.25 * clamp(len(addresses) / 8.0)
                + 0.15 * clamp(repeated_addresses / 4.0)
            ),
            "messages": clamp(count / 25.0),
            "aircraft": clamp(len(addresses) / 8.0),
            "repeated": clamp(repeated_addresses / 4.0),
            "df_mix": clamp(len(dfs) / 6.0),
        }
        for key, value in values.items():
            self.maximums[key] = max(self.maximums.get(key, 0.0), value)
        return {
            **values,
            "count": count,
            "rate": count / WINDOW_S,
            "aircraft_count": len(addresses),
            "repeated_count": repeated_addresses,
            "df_counts": dfs,
        }

    def reset(self) -> None:
        self.maximums.clear()


def bar(value: float, width: int) -> str:
    filled = round(clamp(value) * width)
    return "#" * filled + "-" * (width - filled)


def draw_line(
    screen, row: int, label: str, value: float, maximum: float,
    value_text: str, width: int,
) -> int:
    bar_width = max(10, min(28, width - 52))
    relative = 100.0 * value / maximum if maximum > 0 else 0.0
    suffix = " NEW MAX" if maximum > 0 and abs(value - maximum) < 1e-12 else ""
    screen.addnstr(
        row, 0,
        f"{label:<14} [{bar(value, bar_width)}] "
        f"{value_text:<9} {relative:5.0f}% max{suffix}",
        width - 1,
    )
    return row + 1


def draw(screen, meter: Meter, process: subprocess.Popen, command: list[str]) -> None:
    snapshot = meter.snapshot()
    height, width = screen.getmaxyx()
    if height < 14 or width < 72:
        screen.erase()
        screen.addstr(0, 0, "Terminal too small (need at least 72x14).")
        screen.refresh()
        return

    screen.erase()
    screen.addnstr(0, 0, "mode-s-meter - live 1090 MHz reception quality", width - 1)
    screen.addnstr(
        1, 0,
        f"Last {WINDOW_S:g}s | r reset maximum | q quit | PID {process.pid}",
        width - 1,
    )
    screen.hline(2, 0, "-", width - 1)
    row = 4
    metrics = (
        ("Overall", "overall", f"{snapshot['overall'] * 100:5.1f}%"),
        ("Messages", "messages", f"{snapshot['rate']:5.1f}/s"),
        ("ICAO aircraft", "aircraft", f"{snapshot['aircraft_count']:5d}"),
        ("Repeated ICAO", "repeated", f"{snapshot['repeated_count']:5d}"),
        ("DF diversity", "df_mix", f"{len(snapshot['df_counts']):5d}"),
    )
    for label, key, value_text in metrics:
        row = draw_line(
            screen, row, label, snapshot[key],
            meter.maximums.get(key, 0.0), value_text, width,
        )

    row += 1
    df_text = " ".join(
        f"DF{df}:{count}"
        for df, count in sorted(snapshot["df_counts"].items())
    ) or "no decoded frames"
    screen.addnstr(
        row, 0,
        f"Frames: {snapshot['count']}  ICAO: {snapshot['aircraft_count']}  {df_text}",
        width - 1,
    )
    row += 2
    if process.poll() is not None:
        screen.addnstr(
            row, 0,
            f"dump1090 exited with code {process.returncode}; press q",
            width - 1,
        )
    elif snapshot["count"] == 0:
        screen.addnstr(row, 0, "Waiting for decoded Mode-S frames ...", width - 1)
    else:
        screen.addnstr(row, 0, "NEW MAX highlights the current five-second peak.", width - 1)
    row += 1
    screen.addnstr(row, 0, "decoder: " + " ".join(command), width - 1)
    screen.refresh()


def run_screen(
    screen, process: subprocess.Popen, reader: RawReader, meter: Meter,
    command: list[str],
) -> None:
    curses.curs_set(0)
    screen.nodelay(True)
    while True:
        meter.add(reader.drain())
        draw(screen, meter, process, command)
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
        description="Live 1090 MHz Mode-S reception meter for antenna positioning."
    )
    parser.add_argument(
        "--binary",
        help="dump1090-compatible binary",
    )
    parser.add_argument("--device", default="0", help="RTL-SDR device index")
    parser.add_argument("--gain", help="RTL-SDR gain in dB")
    parser.add_argument("--ppm", help="RTL-SDR frequency correction")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    binary = args.binary
    if binary is None:
        binary = shutil.which("dump1090-fa") or shutil.which("dump1090")
        if binary is None:
            for candidate in (
                script_dir / "dump1090",
                script_dir.parent / "dump1090" / "dump1090",
            ):
                if candidate.is_file():
                    binary = str(candidate)
                    break
    resolved_binary = shutil.which(binary) if binary else None
    if resolved_binary is None and binary is not None:
        candidate = Path(binary).expanduser().resolve()
        if candidate.is_file():
            resolved_binary = str(candidate)
    if resolved_binary is None:
        requested = binary or "dump1090"
        print(f"dump1090 binary not found: {requested}", file=sys.stderr)
        return 2

    command = [resolved_binary, "--device-index", args.device, "--raw", "--quiet"]
    if args.gain is not None:
        command += ["--gain", args.gain]
    if args.ppm is not None:
        command += ["--ppm", args.ppm]

    with tempfile.TemporaryFile() as stderr:
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=stderr,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            print(f"could not start dump1090: {exc}", file=sys.stderr)
            return 2
        assert process.stdout is not None
        reader = RawReader(process.stdout)
        reader.start()
        try:
            curses.wrapper(run_screen, process, reader, Meter(), command)
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
            stderr.seek(0)
            error = stderr.read().decode(errors="replace").strip()
            print(f"dump1090 exited with code {process.returncode}", file=sys.stderr)
            if error:
                print(error, file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
