import bisect
import json
import sqlite3
import statistics
import sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

KEYS = {'command_store_publish_us', 'command_transport_publish_us',
        'command_transition_lock_wait_us', 'planning_scheduling_gap_us',
        'execution_owner_publish_lock_wait_us'}

def quantile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

def summary(values):
    return {'n': len(values), 'p50': quantile(values, .5),
            'p95': quantile(values, .95), 'p99': quantile(values, .99),
            'max': max(values) if values else None}

def load_topic(connection, name):
    row = connection.execute('select id,type from topics where name=?', (name,)).fetchone()
    if not row:
        return []
    message_type = get_message(row[1])
    return [(timestamp, deserialize_message(blob, message_type))
            for timestamp, blob in connection.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp',
                (row[0],))]

def analyze(directory):
    directory = Path(directory)
    report = json.loads((directory / 'report.json').read_text())
    metadata = json.loads((directory / 'metadata.json').read_text())
    conn = sqlite3.connect(next((directory / 'rosbag').glob('*.db3')))
    command = load_topic(conn, '/navigation/navigation_command')
    admission = load_topic(conn, '/navigation/command_admission')
    px4 = load_topic(conn, '/fmu/in/trajectory_setpoint')
    diag = load_topic(conn, '/navigation/diagnostics')
    values = {name: [] for name in KEYS}
    keys_matching = set()
    for _, message in diag:
        for status in message.status:
            for item in status.values:
                if item.key in values:
                    try:
                        values[item.key].append(float(item.value))
                    except ValueError:
                        pass
                if any(term in item.key.lower() for term in ('lease', 'identity', 'boundary_rejection', 'continuity')):
                    keys_matching.add(item.key)
    command_by_id = {}
    for timestamp, message in command:
        key = (message.mode_activation_id, message.mission_id,
               message.request_id, message.bundle_generation, message.sample_id)
        command_by_id[key] = timestamp
    px4_timestamps = [timestamp for timestamp, _ in px4]
    handoffs = []
    prior = None
    for index, (timestamp, message) in enumerate(admission):
        if prior is not None and message.request_id != prior[1].request_id:
            old_timestamp, old_message = prior
            new_timestamp, new_message = timestamp, message
            old_key = (old_message.mode_activation_id, old_message.mission_id,
                       old_message.request_id, old_message.bundle_generation,
                       old_message.sample_id)
            new_key = (new_message.mode_activation_id, new_message.mission_id,
                       new_message.request_id, new_message.bundle_generation,
                       new_message.sample_id)
            def nearest_setpoint(t):
                i = bisect.bisect_left(px4_timestamps, t)
                candidates = px4_timestamps[max(0, i-1):min(len(px4_timestamps), i+2)]
                if not candidates:
                    return None
                result = min(candidates, key=lambda x: abs(x-t))
                return result if abs(result-t) <= 50_000_000 else None
            old_px4 = nearest_setpoint(old_timestamp)
            new_px4 = nearest_setpoint(new_timestamp)
            handoffs.append({
                'old_request': old_message.request_id,
                'new_request': new_message.request_id,
                'admission_gap_ms': (new_timestamp-old_timestamp)/1e6,
                'core_command_gap_ms': ((command_by_id[new_key]-command_by_id[old_key])/1e6
                                        if old_key in command_by_id and new_key in command_by_id else None),
                'nearest_px4_setpoint_gap_ms': ((new_px4-old_px4)/1e6
                                                  if old_px4 is not None and new_px4 is not None else None),
            })
        prior = (timestamp, message)
    result = {
        'session': directory.name,
        'source_head': metadata.get('build_provenance', {}).get('manifest', {}).get('source', {}).get('git_head'),
        'tracking_mode': metadata.get('tracking_experiment', {}).get('mode'),
        'mission_complete': report['mission_outcome']['acceptance']['mission_complete_observed'],
        'accepted': report['mission_outcome']['acceptance']['waypoint_acceptance_indices'],
        'handoffs': handoffs,
        'metrics_us': {key: summary(data) for key, data in values.items()},
        'diagnostic_keys_for_failure_review': sorted(keys_matching),
    }
    return result

if __name__ == '__main__':
    for name in sys.argv[1:]:
        print(json.dumps(analyze(name), sort_keys=True))
