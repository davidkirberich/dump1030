#!/usr/bin/env python3

import argparse
import curses
import glob
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Set, Tuple


@dataclass
class Sighting:
    address: str
    count: int
    q: int
    qmax: int
    uf: int
    payload: str
    last_seen: float


@dataclass
class Interrogator:
    kind: str
    code: int
    count: int
    qmax: int
    q: int
    first_seen: float
    last_seen: float
    ufs: Set[int]
    targets: Set[str]
    payload: str


class JsonlFollower:
    def __init__(self, pattern: str) -> None:
        self.pattern = pattern
        self.offsets: Dict[str, int] = {}
        self.known_files: Set[str] = set()

    def read_new(self) -> List[dict]:
        events: List[dict] = []
        for path in sorted(glob.glob(self.pattern)):
            try:
                with open(path, "r", encoding="utf-8") as stream:
                    self.known_files.add(path)
                    offset = self.offsets.get(path, 0)
                    stream.seek(offset)
                    while line := stream.readline():
                        line_end = stream.tell()
                        try:
                            events.append(json.loads(line))
                        except json.JSONDecodeError:
                            offset = line_end - len(line)
                            break
                        offset = line_end
                    self.offsets[path] = offset
            except FileNotFoundError:
                continue
        return events


def valid_address(address: object) -> bool:
    if not isinstance(address, str) or len(address) != 6:
        return False
    try:
        value = int(address, 16)
    except ValueError:
        return False
    return value not in (0, 0xFFFFFF)


def update_views(
    events: List[dict],
    sightings: Dict[str, Sighting],
    interrogators: Dict[str, Interrogator],
    uf_counts: Dict[int, int],
) -> Tuple[int, int, int]:
    confirmed_events = 0
    p6_events = 0
    interrogator_events = 0
    now = time.time()
    for event in events:
        if event.get("type") != "P6":
            continue
        p6_events += 1
        uf = int(event.get("uf", -1))
        uf_counts[uf] = uf_counts.get(uf, 0) + 1

        code = event.get("ii")
        kind = "II"
        if code is None:
            code = event.get("si")
            kind = "SI"
        if isinstance(code, int):
            interrogator_events += 1
            key = f"{kind}{code:02d}"
            quality = int(event.get("q", 0))
            current_interrogator = interrogators.get(key)
            if current_interrogator is None:
                interrogators[key] = Interrogator(
                    kind=kind,
                    code=code,
                    count=1,
                    qmax=quality,
                    q=quality,
                    first_seen=now,
                    last_seen=now,
                    ufs={uf},
                    targets=set(),
                    payload=str(event.get("hex", "")),
                )
                current_interrogator = interrogators[key]
            else:
                current_interrogator.count += 1
                current_interrogator.q = quality
                current_interrogator.qmax = max(current_interrogator.qmax, quality)
                current_interrogator.last_seen = now
                current_interrogator.ufs.add(uf)
                current_interrogator.payload = str(event.get("hex", ""))
            address = event.get("aa")
            if valid_address(address):
                current_interrogator.targets.add(address)

        address = event.get("aa")
        if not valid_address(address):
            continue

        confirmed_events += 1
        quality = int(event.get("q", 0))
        reported_count = max(2, int(event.get("aa_cnt", 1)))
        current = sightings.get(address)
        if current is None:
            sightings[address] = Sighting(
                address=address,
                count=reported_count,
                q=quality,
                qmax=quality,
                uf=int(event.get("uf", -1)),
                payload=str(event.get("hex", "")),
                last_seen=now,
            )
        else:
            current.count = max(current.count, reported_count)
            current.q = quality
            current.qmax = max(current.qmax, quality)
            current.uf = int(event.get("uf", -1))
            current.payload = str(event.get("hex", ""))
            current.last_seen = now
    return p6_events, confirmed_events, interrogator_events


def age_text(seconds: int) -> str:
    if seconds < 60:
        return f"{seconds}s"
    if seconds < 3600:
        return f"{seconds // 60}m{seconds % 60:02d}s"
    return f"{seconds // 3600}h{(seconds % 3600) // 60:02d}m"


class Monitor:
    def __init__(self, screen: "curses._CursesWindow", follower: JsonlFollower,
                 refresh: float) -> None:
        self.screen = screen
        self.follower = follower
        self.refresh = refresh
        self.sightings: Dict[str, Sighting] = {}
        self.interrogators: Dict[str, Interrogator] = {}
        self.uf_counts: Dict[int, int] = {}
        self.view = 1
        self.paused = False
        self.running = True
        self.filter_text = ""
        self.selection = 0
        self.scroll = 0
        self.message = ""
        self.confirmed_events = 0
        self.p6_events = 0
        self.interrogator_events = 0
        self.events_since_rate = 0
        self.rate_started = time.monotonic()
        self.event_rate = 0.0

    def rows(self) -> List[Sighting]:
        needle = self.filter_text.upper()
        rows = [
            sighting for sighting in self.sightings.values()
            if not needle
            or needle in sighting.address
            or needle in sighting.payload
        ]
        return sorted(rows, key=lambda sighting: (-sighting.last_seen, sighting.address))

    def interrogator_rows(self) -> List[Interrogator]:
        needle = self.filter_text.upper()
        rows = [
            interrogator for interrogator in self.interrogators.values()
            if not needle
            or needle in f"{interrogator.kind}{interrogator.code:02d}"
            or any(needle in target for target in interrogator.targets)
        ]
        return sorted(
            rows,
            key=lambda interrogator: (-interrogator.last_seen, interrogator.kind, interrogator.code),
        )

    def poll(self) -> None:
        events = self.follower.read_new()
        new_p6, new_confirmed, new_interrogators = update_views(
            events, self.sightings, self.interrogators, self.uf_counts
        )
        self.p6_events += new_p6
        self.confirmed_events += new_confirmed
        self.interrogator_events += new_interrogators
        self.events_since_rate += new_p6

        elapsed = time.monotonic() - self.rate_started
        if elapsed >= 1.0:
            self.event_rate = self.events_since_rate / elapsed
            self.events_since_rate = 0
            self.rate_started = time.monotonic()

    def add_line(self, y: int, text: str, attr: int = 0) -> None:
        height, width = self.screen.getmaxyx()
        if y < 0 or y >= height or width <= 0:
            return
        self.screen.addnstr(y, 0, text.ljust(width - 1), width - 1, attr)

    def render_aircraft(self, visible_rows: int, header_attr: int) -> None:
        rows = self.rows()
        self.selection = min(self.selection, max(0, len(rows) - 1))
        if self.selection < self.scroll:
            self.scroll = self.selection
        elif self.selection >= self.scroll + visible_rows:
            self.scroll = max(0, self.selection - visible_rows + 1)

        self.add_line(
            3,
            f"{'ICAO':<7}{'SEEN':>6}  {'Q':>3}  {'QMAX':>4}  {'UF':>3}  "
            f"{'AGE':>7}  PAYLOAD",
            header_attr,
        )
        now = time.time()
        for index, sighting in enumerate(
            rows[self.scroll:self.scroll + visible_rows], start=self.scroll
        ):
            line = (
                f"{sighting.address:<7}{sighting.count:>6}  "
                f"{sighting.q:>3}  {sighting.qmax:>4}  {sighting.uf:>3}  "
                f"{age_text(max(0, int(now - sighting.last_seen))):>7}  "
                f"{sighting.payload}"
            )
            self.add_line(
                4 + index - self.scroll,
                line,
                curses.A_REVERSE if index == self.selection else 0,
            )
        if not rows:
            if self.p6_events == 0:
                self.add_line(4, " waiting for P6 events")
            else:
                self.add_line(
                    4, " P6 received; waiting for a repeated ICAO address"
                )
        return

    def render_interrogators(self, visible_rows: int, header_attr: int) -> None:
        rows = self.interrogator_rows()
        self.selection = min(self.selection, max(0, len(rows) - 1))
        if self.selection < self.scroll:
            self.scroll = self.selection
        elif self.selection >= self.scroll + visible_rows:
            self.scroll = max(0, self.selection - visible_rows + 1)

        self.add_line(
            3,
            f"{'ID':<6}{'SEEN':>7}  {'Q':>3}  {'QMAX':>4}  {'UFs':<11}"
            f"{'AGE':>7}  TARGETS / LAST PAYLOAD",
            header_attr,
        )
        now = time.time()
        for index, interrogator in enumerate(
            rows[self.scroll:self.scroll + visible_rows], start=self.scroll
        ):
            ufs = ",".join(str(uf) for uf in sorted(interrogator.ufs))
            targets = ",".join(sorted(interrogator.targets)) or "-"
            line = (
                f"{interrogator.kind}{interrogator.code:02d}  "
                f"{interrogator.count:>6}  {interrogator.q:>3}  "
                f"{interrogator.qmax:>4}  {ufs:<11}"
                f"{age_text(max(0, int(now - interrogator.last_seen))):>7}  "
                f"{targets} / {interrogator.payload}"
            )
            self.add_line(
                4 + index - self.scroll,
                line,
                curses.A_REVERSE if index == self.selection else 0,
            )
        if not rows:
            if self.p6_events == 0:
                self.add_line(4, " waiting for P6 events")
            else:
                self.add_line(4, " no II/SI fields in the accepted P6 traffic")
                uf_summary = " ".join(
                    f"UF{uf}:{count}" for uf, count in sorted(self.uf_counts.items())
                )
                self.add_line(5, f" observed formats: {uf_summary}")
        return

    def render(self) -> None:
        height, width = self.screen.getmaxyx()
        self.screen.erase()
        if curses.has_colors():
            title_attr = curses.color_pair(1)
            header_attr = curses.color_pair(2)
        else:
            title_attr = curses.A_BOLD
            header_attr = curses.A_REVERSE

        visible_rows = max(0, height - 7)
        mode = "PAUSED" if self.paused else "LIVE"
        filter_label = self.filter_text or "-"
        view_name = "Aircrafts" if self.view == 1 else "Interrogators"
        self.add_line(
            0,
            f" dump1030 top  [{mode}]  {view_name}  "
            f"aircrafts {len(self.sightings)}  interrogators {len(self.interrogators)}  "
            f"P6 {self.p6_events} ({self.event_rate:.1f}/s)  "
            f"files {len(self.follower.known_files)}",
            title_attr,
        )
        self.add_line(
            1,
            f" source: {self.follower.pattern}  |  filter: {filter_label}",
            title_attr,
        )
        if self.view == 1:
            self.render_aircraft(visible_rows, header_attr)
            rows = self.rows()
        else:
            self.render_interrogators(visible_rows, header_attr)
            rows = self.interrogator_rows()

        footer_y = max(4, height - 3)
        self.add_line(
            footer_y,
            " 1 aircrafts  2 interrogators  q/:q quit  space/p pause  "
            "j/k or arrows select  / filter  "
            "c clear filter  :w [file] export",
            header_attr,
        )
        self.add_line(height - 2, self.message[:max(0, width - 1)])
        if rows:
            self.add_line(
                height - 1,
                f" {self.selection + 1}/{len(rows)}  "
                f"(Enter copies no data; select and then freeze for PuTTY copy)"
            )
        self.screen.refresh()

    def input_line(self, prompt: str) -> Tuple[bool, str]:
        height, width = self.screen.getmaxyx()
        value = ""
        curses.curs_set(1)
        while self.running:
            self.add_line(height - 2, prompt + value)
            self.screen.refresh()
            try:
                key = self.screen.get_wch()
            except curses.error:
                continue
            if key in ("\n", "\r"):
                curses.curs_set(0)
                return True, value
            if key == "\x1b":
                curses.curs_set(0)
                return False, value
            if key in ("\b", "\x7f", curses.KEY_BACKSPACE):
                value = value[:-1]
            elif isinstance(key, str) and key.isprintable() and len(prompt + value) < width - 2:
                value += key
        curses.curs_set(0)
        return False, value

    def export(self, path: str) -> None:
        if self.view == 1:
            target = path or "p6_sightings.tsv"
            with open(target, "w", encoding="utf-8") as stream:
                stream.write("ICAO\tSEEN\tQ\tQMAX\tUF\tPAYLOAD\n")
                for sighting in self.rows():
                    stream.write(
                        f"{sighting.address}\t{sighting.count}\t{sighting.q}\t"
                        f"{sighting.qmax}\t{sighting.uf}\t{sighting.payload}\n"
                    )
            self.message = f"exported {len(self.rows())} addresses to {target}"
            return

        target = path or "p6_interrogators.tsv"
        with open(target, "w", encoding="utf-8") as stream:
            stream.write("ID\tSEEN\tQ\tQMAX\tUFs\tTARGETS\tPAYLOAD\n")
            for interrogator in self.interrogator_rows():
                ufs = ",".join(str(uf) for uf in sorted(interrogator.ufs))
                targets = ",".join(sorted(interrogator.targets))
                stream.write(
                    f"{interrogator.kind}{interrogator.code:02d}\t"
                    f"{interrogator.count}\t{interrogator.q}\t"
                    f"{interrogator.qmax}\t{ufs}\t{targets}\t"
                    f"{interrogator.payload}\n"
                )
        self.message = f"exported {len(self.interrogator_rows())} interrogators to {target}"

    def command(self) -> None:
        accepted, value = self.input_line(":")
        if not accepted:
            self.message = ""
            return
        parts = value.strip().split(maxsplit=1)
        if not parts:
            return
        command = parts[0].lower()
        argument = parts[1] if len(parts) > 1 else ""
        if command in ("q", "quit"):
            self.running = False
        elif command in ("w", "write"):
            try:
                self.export(argument)
            except OSError as error:
                self.message = f"export failed: {error}"
        elif command in ("p", "pause"):
            self.paused = True
            self.message = "paused - screen is stable for selection"
        elif command == "resume":
            self.paused = False
            self.message = ""
        else:
            self.message = f"unknown command: {command}"

    def handle_key(self, key: object) -> None:
        if key in ("q", "\x03"):
            self.running = False
        elif key == ":":
            self.command()
        elif key == "/":
            accepted, value = self.input_line("/")
            if accepted:
                self.filter_text = value.upper()
                self.selection = self.scroll = 0
        elif key == "c":
            self.filter_text = ""
            self.message = ""
        elif key in ("1", "2"):
            self.view = int(key)
            self.selection = self.scroll = 0
            self.filter_text = ""
            self.message = ""
        elif key in (" ", "p"):
            self.paused = not self.paused
            self.message = "paused - screen is stable for selection" if self.paused else ""
        elif key in ("j", curses.KEY_DOWN):
            length = len(self.rows()) if self.view == 1 else len(self.interrogator_rows())
            self.selection = min(self.selection + 1, max(0, length - 1))
        elif key in ("k", curses.KEY_UP):
            self.selection = max(0, self.selection - 1)
        elif key == curses.KEY_RESIZE:
            curses.resize_term(0, 0)

    def run(self) -> None:
        self.screen.timeout(max(1, int(self.refresh * 1000)))
        next_poll = 0.0
        render_needed = True
        while self.running:
            now = time.monotonic()
            if not self.paused and now >= next_poll:
                self.poll()
                next_poll = now + self.refresh
                render_needed = True
            if render_needed:
                self.render()
                render_needed = False
            try:
                self.handle_key(self.screen.get_wch())
                render_needed = True
            except curses.error:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive live monitor for dump1030 P6 ICAO sightings."
    )
    parser.add_argument(
        "pattern",
        nargs="?",
        default="captured/events_*.jsonl",
        help="JSONL file or glob (default: captured/events_*.jsonl)",
    )
    parser.add_argument(
        "--refresh",
        type=float,
        default=1.0,
        help="display refresh interval in seconds (default: 1)",
    )
    return parser.parse_args()


def run_tui(screen: "curses._CursesWindow", args: argparse.Namespace) -> int:
    curses.curs_set(0)
    if curses.has_colors():
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_GREEN)
    Monitor(screen, JsonlFollower(args.pattern), args.refresh).run()
    return 0


def main() -> int:
    args = parse_args()
    if args.refresh <= 0:
        print("error: --refresh must be greater than zero", file=sys.stderr)
        return 2
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("error: monitor_p6 requires an interactive terminal", file=sys.stderr)
        return 2
    try:
        return curses.wrapper(run_tui, args)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        os._exit(0)
