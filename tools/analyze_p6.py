#!/usr/bin/env python3
"""Summarize dump1030 JSONL captures without external dependencies."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import glob
import json
import math
from pathlib import Path
import statistics
from typing import Iterable


QUALITY_FIELDS = (
    "q",
    "lock",
    "cov",
    "rho",
    "p6snr_db",
    "pre_snr_db",
    "p6_p1_ratio",
    "viterbi_metric",
    "path_metric_gap",
    "min_margin",
    "df_hz",
    "spr_us",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze dump1030 JSONL event files."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        help="JSONL file or glob; quoted globs are expanded by this script",
    )
    parser.add_argument(
        "--top", type=int, default=15, help="number of top entries to print"
    )
    parser.add_argument(
        "--json", dest="json_path", type=Path, help="write summary as JSON"
    )
    parser.add_argument(
        "--min-q", type=int, default=0, help="minimum accepted P6 quality"
    )
    args = parser.parse_args()
    if args.top < 1:
        parser.error("--top must be positive")
    return args


def expand_paths(patterns: Iterable[str]) -> list[Path]:
    paths: set[Path] = set()
    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            paths.update(Path(match) for match in matches if Path(match).is_file())
        elif Path(pattern).is_file():
            paths.add(Path(pattern))
    return sorted(paths)


def load_events(paths: list[Path]) -> tuple[list[dict], Counter]:
    events: list[dict] = []
    errors: Counter = Counter()
    for path in paths:
        with path.open(errors="replace") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    errors["invalid_json"] += 1
                    continue
                if not isinstance(event, dict):
                    errors["non_object"] += 1
                    continue
                event["_source"] = str(path)
                event["_line"] = line_number
                events.append(event)
    events.sort(
        key=lambda event: (
            numeric(event.get("mono"), math.inf),
            numeric(event.get("smp"), math.inf),
            event["_source"],
            event["_line"],
        )
    )
    return events, errors


def numeric(value: object, fallback: float = math.nan) -> float:
    return float(value) if isinstance(value, (int, float)) else fallback


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def stats(values: Iterable[object]) -> dict | None:
    clean = [numeric(value) for value in values]
    clean = [value for value in clean if math.isfinite(value)]
    if not clean:
        return None
    return {
        "count": len(clean),
        "min": min(clean),
        "p10": percentile(clean, 0.10),
        "median": statistics.median(clean),
        "p90": percentile(clean, 0.90),
        "max": max(clean),
        "mean": statistics.fmean(clean),
    }


def valid_address(value: object) -> str | None:
    if not isinstance(value, str) or len(value) != 6:
        return None
    try:
        address = int(value, 16)
    except ValueError:
        return None
    return value.upper() if address not in (0, 0xFFFFFF) else None


def summarize(events: list[dict], errors: Counter, min_q: int, top: int) -> dict:
    accepted = [
        event
        for event in events
        if event.get("type") == "P6"
        and numeric(event.get("q"), -1) >= min_q
    ]
    rejected = [event for event in events if event.get("type") == "P6_REJECTED"]
    ssr = [event for event in events if event.get("type") == "SSR"]

    times = [numeric(event.get("mono")) for event in accepted]
    times = [value for value in times if math.isfinite(value)]
    duration = max(times) - min(times) if len(times) >= 2 else 0.0

    by_address: dict[str, list[dict]] = defaultdict(list)
    for event in accepted:
        address = valid_address(event.get("aa"))
        if address:
            by_address[address].append(event)

    address_rows = []
    for address, members in by_address.items():
        member_times = [
            numeric(event.get("mono"))
            for event in members
            if math.isfinite(numeric(event.get("mono")))
        ]
        gaps = [
            later - earlier
            for earlier, later in zip(member_times, member_times[1:])
        ]
        address_rows.append(
            {
                "aa": address,
                "count": len(members),
                "max_aa_cnt": max(
                    int(numeric(event.get("aa_cnt"), 0)) for event in members
                ),
                "mean_q": statistics.fmean(
                    numeric(event.get("q"), 0) for event in members
                ),
                "max_q": max(numeric(event.get("q"), 0) for event in members),
                "span_s": (
                    max(member_times) - min(member_times)
                    if len(member_times) >= 2
                    else 0.0
                ),
                "gap_median_s": statistics.median(gaps) if gaps else None,
            }
        )
    address_rows.sort(key=lambda row: (-row["count"], -row["mean_q"], row["aa"]))

    by_sample_rate = Counter(
        int(rate)
        for rate in (numeric(event.get("sample_rate")) for event in events)
        if math.isfinite(rate)
    )

    payload_counts = Counter(
        event.get("hex") for event in accepted if isinstance(event.get("hex"), str)
    )
    repeated_payloads = [
        {"hex": payload, "count": count}
        for payload, count in payload_counts.most_common()
        if count > 1
    ]

    summary = {
        "events": len(events),
        "accepted_p6": len(accepted),
        "rejected_p6": len(rejected),
        "ssr": len(ssr),
        "parse_errors": dict(errors),
        "duration_s": duration,
        "accepted_p6_per_min": (
            len(accepted) * 60.0 / duration if duration > 0 else None
        ),
        "sample_rates": dict(sorted(by_sample_rate.items())),
        "uf": dict(sorted(Counter(event.get("uf") for event in accepted).items())),
        "nbits": dict(
            sorted(Counter(event.get("nbits") for event in accepted).items())
        ),
        "status": dict(
            sorted(Counter(event.get("status") for event in accepted).items())
        ),
        "mode5": dict(
            sorted(Counter(bool(event.get("mode5")) for event in accepted).items())
        ),
        "reject_reasons": dict(
            Counter(event.get("reason", "unknown") for event in rejected).most_common()
        ),
        "ssr_subtypes": dict(
            Counter(event.get("subtype", "unknown") for event in ssr).most_common()
        ),
        "quality": {
            field: stats(event.get(field) for event in accepted)
            for field in QUALITY_FIELDS
        },
        "addresses": address_rows[:top],
        "valid_address_events": sum(row["count"] for row in address_rows),
        "invalid_or_missing_address_events": (
            len(accepted) - sum(row["count"] for row in address_rows)
        ),
        "repeated_payloads": repeated_payloads[:top],
    }
    return summary


def format_number(value: object, decimals: int = 3) -> str:
    return "n/a" if value is None or not math.isfinite(numeric(value)) else (
        f"{numeric(value):.{decimals}f}"
    )


def print_counter(title: str, values: dict) -> None:
    print(f"\n{title}")
    if not values:
        print("  none")
        return
    for key, value in values.items():
        print(f"  {key}: {value}")


def print_report(summary: dict) -> None:
    print("dump1030 JSONL analysis")
    print(f"Events: {summary['events']}")
    print(f"Accepted P6: {summary['accepted_p6']}")
    print(f"Rejected P6: {summary['rejected_p6']}")
    print(f"SSR events: {summary['ssr']}")
    print(f"Duration: {format_number(summary['duration_s'])} s")
    print(f"Accepted P6 rate: {format_number(summary['accepted_p6_per_min'])}/min")
    print(f"Parse errors: {sum(summary['parse_errors'].values())}")

    print_counter("Sample rates", summary["sample_rates"])
    print_counter("UF distribution", summary["uf"])
    print_counter("Frame lengths", summary["nbits"])
    print_counter("P6 status", summary["status"])
    print_counter("Rejections", summary["reject_reasons"])
    print_counter("SSR subtypes", summary["ssr_subtypes"])

    print("\nP6 quality")
    for field, values in summary["quality"].items():
        if values is None:
            continue
        print(
            f"  {field:16s} "
            f"median={format_number(values['median'])} "
            f"p10={format_number(values['p10'])} "
            f"p90={format_number(values['p90'])} "
            f"range={format_number(values['min'])}..{format_number(values['max'])}"
        )

    print(
        "\nAddresses "
        f"(valid={summary['valid_address_events']}, "
        f"invalid/missing={summary['invalid_or_missing_address_events']})"
    )
    if summary["addresses"]:
        print("  AA      count  max_cnt  mean_Q  max_Q  span_s  gap_median_s")
        for row in summary["addresses"]:
            print(
                f"  {row['aa']}  {row['count']:5d}  {row['max_aa_cnt']:7d}  "
                f"{row['mean_q']:6.1f}  {row['max_q']:6.0f}  "
                f"{row['span_s']:7.1f}  "
                f"{format_number(row['gap_median_s']):>12s}"
            )
    else:
        print("  none")

    print("\nRepeated payloads")
    if summary["repeated_payloads"]:
        for row in summary["repeated_payloads"]:
            print(f"  {row['hex']}  x{row['count']}")
    else:
        print("  none")


def main() -> int:
    args = parse_args()
    paths = expand_paths(args.inputs)
    if not paths:
        raise SystemExit("No input files found.")
    events, errors = load_events(paths)
    summary = summarize(events, errors, args.min_q, args.top)
    print(f"Files: {len(paths)}")
    for path in paths:
        print(f"  {path}")
    print_report(summary)
    if args.json_path:
        args.json_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n"
        )
        print(f"\nJSON summary: {args.json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
