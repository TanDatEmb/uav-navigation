#!/usr/bin/env python3
"""Compile/run audit speed sweep with A's governor and isolated build metadata."""
import json
import shlex
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "validation/build"
BACKEND = BUILD / "navigation_planning_backend"
SOURCE = ROOT / "tests/h5_speed_grid_oracle.cpp"
OBJECT = ROOT / "validation/h5_speed_grid_oracle.o"
BINARY = ROOT / "validation/h5_speed_grid_oracle"

entries = json.loads((BUILD / "compile_commands.json").read_text())
entry = next(x for x in entries if x["file"].endswith("/test_planner_config.cpp"))
compile_args = shlex.split(entry["command"])
compiler = compile_args[0]
source_path = Path(entry["file"]).resolve()
flags = []
i = 1
while i < len(compile_args):
    arg = compile_args[i]
    if arg in ("-I", "-isystem", "-D", "-U") and i + 1 < len(compile_args):
        flags.extend((arg, compile_args[i + 1]))
        i += 2
        continue
    if arg.startswith(("-I", "-D", "-U")):
        flags.append(arg)
    i += 1

compile_command = [compiler, *flags, "-std=c++20", "-O3", "-DNDEBUG",
                   "-c", str(SOURCE), "-o", str(OBJECT)]
compile = subprocess.run(compile_command, text=True, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
(ROOT / "validation/logs/h5_speed_grid_oracle_compile.log").write_text(
    json.dumps(compile_command) + "\n" + compile.stdout)
if compile.returncode:
    raise SystemExit(f"compile exit={compile.returncode}\n{compile.stdout}")

link_file = BACKEND / "CMakeFiles/test_planner_config.dir/link.txt"
link_args = shlex.split(link_file.read_text())
out_index = link_args.index("-o")
link_tail = link_args[out_index + 2:]
link_command = [compiler, "-O3", "-DNDEBUG", str(OBJECT), "-o", str(BINARY),
                *link_tail]
link = subprocess.run(link_command, cwd=BACKEND, text=True,
                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
(ROOT / "validation/logs/h5_speed_grid_oracle_link.log").write_text(
    json.dumps(link_command) + "\n" + link.stdout)
if link.returncode:
    raise SystemExit(f"link exit={link.returncode}\n{link.stdout}")

run = subprocess.run([str(BINARY)], text=True, stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT)
(ROOT / "validation/logs/h5_speed_grid_oracle_run.log").write_text(run.stdout)
print(run.stdout, end="")
print(f"run_exit={run.returncode}")
raise SystemExit(run.returncode)
