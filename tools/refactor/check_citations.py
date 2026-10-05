#!/usr/bin/env python3
"""Compatibility entrypoint for strict repository documentation validation."""
from pathlib import Path
import runpy

runpy.run_path(str(Path(__file__).resolve().parents[1] / "check_documentation.py"), run_name="__main__")
