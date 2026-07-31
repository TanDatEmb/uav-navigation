#!/usr/bin/env python3
"""ROS graph metadata helper kept separate from callback metrics."""
from __future__ import annotations

from typing import Any


def sample_graph(node: Any, topics: dict[str, str]) -> dict[str, dict[str, Any]]:
    names_types = dict(node.get_topic_names_and_types())
    result = {}
    for name, topic in topics.items():
        result[name] = {
            "publisher_count": node.count_publishers(topic),
            "subscriber_count": node.count_subscribers(topic),
            "message_types": names_types.get(topic, []),
            "qos_compatibility": "runtime_subscription_created",
        }
    return result
