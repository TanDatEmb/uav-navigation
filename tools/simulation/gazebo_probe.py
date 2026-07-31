#!/usr/bin/env python3
"""Bounded Gazebo Transport progression probe using short CLI samples."""
from __future__ import annotations

import re
import subprocess
from typing import Any


def _run(arguments: list[str], timeout: float = 1.0) -> str:
    try:
        result = subprocess.run(arguments, capture_output=True, text=True,
                                timeout=timeout, check=False)
        return result.stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _progress(topic: str, timeout: float = 1.0) -> bool:
    """Observe one message while discarding its body (safe for packed clouds)."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=timeout, check=False)
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


class GazeboProbe:
    def __init__(self, topics: dict[str, str]):
        self.topics = topics
        self.sequences = {name: 0 for name in topics}

    def sample(self) -> dict[str, Any]:
        listed = set(_run(["gz", "topic", "-l"], 5.0).splitlines())
        result: dict[str, Any] = {}
        for name, topic in self.topics.items():
            exists = topic in listed
            result[name + "_exists"] = exists
            if not exists:
                continue
            # One message only, bounded output; binary point cloud is never dumped.
            if name in ("pointcloud",):
                info = _run(["gz", "topic", "-i", "-t", topic], 3.0)
                result[name + "_publisher"] = "Publisher" in info
                if result[name + "_publisher"] and _progress(topic, 3.0):
                    self.sequences[name] += 1
                continue
            text = _run(["gz", "topic", "-e", "-n", "1", "-t", topic], 3.0)
            if text:
                self.sequences[name] += 1
            if name == "raw_scan":
                tokens = re.findall(r"ranges:\\s*([-+\\w.]+)", text)
                finite = sum(1 for token in tokens if token.lower() not in ("inf", "+inf", "-inf", "nan"))
                result["raw_scan_sample_count"] = len(tokens)
                result["raw_scan_inf_count"] = len(tokens)-finite
                result["raw_scan_finite_ratio"] = finite/len(tokens) if tokens else None
            if name == "stats":
                match = re.search(r"real_time_factor:\\s*([-+\\d.eE]+)", text)
                if match:
                    result["real_time_factor"] = float(match.group(1))
                result["sim_paused"] = bool(re.search(r"paused:\\s*true", text))
        result.update({name + "_sequence": sequence for name, sequence in self.sequences.items()})
        return result
