"""Find processes that forbid a SITL start: a colcon build/test, or another SITL. Standard library only.

Used by tools/uavnav/sitl_hover.sh before it starts anything. Matching is on the process argv
(/proc/<pid>/cmdline, NUL-separated), anchored on argv[0] (and the script argument of an
interpreter), never on a substring of the whole command line: a shell whose `bash -c` string merely
contains "colcon build" or "px4" does not match, while `/path/bin/px4 -d`, `./bin/px4` or a renamed
path to the same binary name does. Every process in the caller's ancestor chain (this process, the
shell, make, ...) is excluded.

CLI:

    python3 tools/uavnav/sitl_guard.py
    -> one line per match "<category> <pid> <argv0> [argv1 ...]" (category colcon or sitl);
       exit 0 none, 3 at least one match, 2 /proc unreadable
"""
import os
import re
import sys

_PYTHON = re.compile(r"^python[0-9.]*$")
_RUBY = re.compile(r"^ruby[0-9.]*$")
_SITL_BINARIES = {"px4", "MicroXRCEAgent", "parameter_bridge", "gz_lidar_bridge"}
_SITL_SCRIPTS = {"sitl_probe.py"}


def _base(arg: str) -> str:
    return arg.rsplit("/", 1)[-1]


def classify(argv: list) -> str | None:
    """'colcon', 'sitl' or None for one process argv."""
    if not argv:
        return None
    exe = _base(argv[0])
    rest = argv[1:]
    # colcon build|test, run directly or through its interpreter (python3 /usr/bin/colcon build ...)
    if exe == "colcon" and rest[:1] and rest[0] in ("build", "test"):
        return "colcon"
    if _PYTHON.match(exe) and len(rest) >= 2 and _base(rest[0]) == "colcon" and rest[1] in ("build", "test"):
        return "colcon"
    if exe in _SITL_BINARIES:
        return "sitl"
    # Gazebo: `gz sim ...` or its ruby launcher `ruby /usr/bin/gz sim ...`
    if exe == "gz" and rest[:1] == ["sim"]:
        return "sitl"
    if _RUBY.match(exe) and len(rest) >= 2 and _base(rest[0]) == "gz" and rest[1] == "sim":
        return "sitl"
    if _PYTHON.match(exe) and rest[:1] and _base(rest[0]) in _SITL_SCRIPTS:
        return "sitl"
    return None


def read_processes(proc="/proc") -> dict:
    """{pid: (ppid, argv)} of every readable process."""
    processes = {}
    for name in os.listdir(proc):
        if not name.isdigit():
            continue
        try:
            with open(f"{proc}/{name}/cmdline", "rb") as f:
                raw = f.read()
            with open(f"{proc}/{name}/stat", "rb") as f:
                stat = f.read().decode("utf-8", "replace")
        except OSError:
            continue  # exited meanwhile, or not ours to read
        argv = [a.decode("utf-8", "replace") for a in raw.split(b"\0") if a]
        ppid = int(stat[stat.rindex(")") + 2:].split()[1])  # fields after "(comm)": state ppid ...
        processes[int(name)] = (ppid, argv)
    return processes


def ancestors(processes: dict, pid: int) -> set:
    """pid and every ancestor up to (and including) pid 1."""
    chain = set()
    while pid > 0 and pid not in chain:
        chain.add(pid)
        pid = processes.get(pid, (0, []))[0]
    return chain


def find(processes: dict, self_pid: int) -> list:
    """[(category, pid, argv)] of the matching processes outside self_pid's ancestor chain, by pid."""
    excluded = ancestors(processes, self_pid)
    found = []
    for pid in sorted(processes):
        if pid in excluded:
            continue
        category = classify(processes[pid][1])
        if category:
            found.append((category, pid, processes[pid][1]))
    return found


def main() -> int:
    try:
        processes = read_processes()
    except OSError as exc:
        print(f"sitl_guard: cannot read /proc: {exc}", file=sys.stderr)
        return 2
    found = find(processes, os.getpid())
    for category, pid, argv in found:
        print(f"{category} {pid} {' '.join(argv)[:200]}")
    return 3 if found else 0


if __name__ == "__main__":
    sys.exit(main())
