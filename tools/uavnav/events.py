"""Reader for the JSONL event log written by uavnav_core's JsonlSink.

Standard library only. Library use:

    records = load(path)                      # strict: ValueError on a malformed line
    records = load(path, skip_malformed=True) # tolerate torn lines
    count_by_reason(records)                  # {(component, event, reason): n}
    timeline(records)                         # ["1.000 comp event A->B REASON", ...]

CLI:

    python3 -m tools.uavnav.events <file.jsonl> [--counts|--timeline] [--skip-malformed]
"""
import argparse
import json
import sys
from typing import Dict, List, Optional, Tuple

_STRING_KEYS = ("component", "event", "state_before", "state_after", "reason")
_REQUIRED_KEYS = ("t_steady_ns",) + _STRING_KEYS + ("values",)


def _reject_constant(name: str):
    # json.loads accepts NaN/Infinity by default; the writer never emits them (it writes null).
    raise ValueError(f"non-standard JSON constant {name}")


def _parse_line(raw: bytes) -> dict:
    """Parse and validate one non-blank physical line. Raises ValueError with a bare message."""
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"invalid UTF-8: {exc}") from None
    try:
        record = json.loads(text, parse_constant=_reject_constant)
    except ValueError as exc:  # JSONDecodeError is a ValueError
        raise ValueError(f"invalid JSON: {exc}") from None
    if not isinstance(record, dict):
        raise ValueError(f"expected a JSON object, got {type(record).__name__}")
    for key in _REQUIRED_KEYS:
        if key not in record:
            raise ValueError(f"missing required key {key!r}")
    t = record["t_steady_ns"]
    if not isinstance(t, int) or isinstance(t, bool):
        raise ValueError("key 't_steady_ns' must be an integer")
    for key in _STRING_KEYS:
        if not isinstance(record[key], str):
            raise ValueError(f"key {key!r} must be a string")
    if not isinstance(record["values"], dict):
        raise ValueError("key 'values' must be an object")
    return record


def load(path, *, skip_malformed: bool = False,
         skipped: Optional[List[Tuple[int, str]]] = None) -> List[dict]:
    """Read a JSONL event log.

    Blank or whitespace-only lines are ignored (a failed write may leave one). A malformed
    line raises ValueError(f"{path}:{line_no}: ...") with the 1-based physical line number
    (blank lines count). With skip_malformed=True such lines are skipped instead; if
    `skipped` is a list, each one is appended to it as (line_no, message).
    """
    records: List[dict] = []
    with open(path, "rb") as f:
        data = f.read()
    # Split on b"\n" only: raw U+2028 and friends may legally appear inside JSON strings.
    for line_no, raw in enumerate(data.split(b"\n"), start=1):
        if not raw.strip():
            continue
        try:
            records.append(_parse_line(raw))
        except ValueError as exc:
            if skip_malformed:
                if skipped is not None:
                    skipped.append((line_no, str(exc)))
                continue
            raise ValueError(f"{path}:{line_no}: {exc}") from None
    return records


def count_by_reason(records) -> Dict[Tuple[str, str, str], int]:
    counts: Dict[Tuple[str, str, str], int] = {}
    for r in records:
        key = (r["component"], r["event"], r["reason"])
        counts[key] = counts.get(key, 0) + 1
    return counts


def timeline(records) -> List[str]:
    """One line per record, stable-sorted by t_steady_ns."""
    lines = []
    for r in sorted(records, key=lambda rec: rec["t_steady_ns"]):
        t_s = r["t_steady_ns"] / 1e9
        lines.append(f"{t_s:.3f} {r['component']} {r['event']} "
                     f"{r['state_before']}->{r['state_after']} {r['reason']}")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="python3 -m tools.uavnav.events",
                                     description="Inspect a uavnav JSONL event log.")
    parser.add_argument("file", help="path to a .jsonl event log")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--counts", action="store_true",
                      help="count records by (component, event, reason)")
    mode.add_argument("--timeline", action="store_true",
                      help="print one line per record sorted by time (default)")
    parser.add_argument("--skip-malformed", action="store_true",
                        help="skip malformed lines instead of failing")
    args = parser.parse_args(argv)

    skipped: List[Tuple[int, str]] = []
    try:
        records = load(args.file, skip_malformed=args.skip_malformed, skipped=skipped)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"{args.file}: cannot read: {exc.strerror or exc}", file=sys.stderr)
        return 1

    if args.counts:
        for (component, event, reason), n in sorted(count_by_reason(records).items()):
            print(f"{n:>6} {component} {event} {reason}")
    else:
        for line in timeline(records):
            print(line)
    if args.skip_malformed:
        print(f"skipped {len(skipped)} malformed line(s)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
