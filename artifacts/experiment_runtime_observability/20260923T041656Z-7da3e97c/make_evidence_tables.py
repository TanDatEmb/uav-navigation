#!/usr/bin/env python3
"""Derive compact audit tables from immutable normalized local bags.

This script is local experiment orchestration, not product code. It copies no
unobserved event into a timeline. Sequence gaps remain visible in raw manifests.
"""
import csv
import json
from pathlib import Path
from collections import defaultdict

root = Path('.artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c')
out = Path('artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c')
completed = ['C','W1-fixed','O1-F1','O1-F2'] + [f'O1-R{i:02}' for i in [3,4,5,6,9,10]]
world = ['W1-fixed','W2','W3','W4','W5']
hold = ['C','W1-fixed','W2','W3','W4','W5']

def table(path):
    return list(csv.DictReader(path.open()))

def write(path, rows, fields):
    with path.open('w', newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore',
                              lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)

o1=[]
for run in completed:
    path=root/f'run-{run}-normalized/O1_PASS_THROUGH_EVENTS.csv'
    if not path.exists(): continue
    groups=defaultdict(list)
    for row in table(path):
        groups[(row['mission_hash'],row['waypoint_index'],row['request_id'])].append(row)
    for identity, rows in groups.items():
        crossings=[r for r in rows if int(r['flags']) & 2 and int(r['flags']) & (1<<10)]
        if not crossings:
            for row in rows:
                flags=int(row['flags'])
                if not flags&(1<<9): continue
                o1.append(dict(run=run,classification='MEASURED_STOP_ACCEPTED',mission_hash=row['mission_hash'],localization_epoch=row['localization_epoch'],route_revision=row['route_revision'],waypoint_index=row['waypoint_index'],request_id=row['request_id'],first_ready_ros_ns='',cross_ros_ns=row['ros_now_ns'],ready_lead_ms='',diagnostic_sequence=row['diagnostic_sequence'],phase=row['phase'],outcome=row['outcome'],flags=row['flags'],crossing_error_m=row['crossing_error_m'],sample_gap_s=row['sample_gap_s'],previous_sample_sequence=row['previous_sample_sequence'],current_sample_sequence=row['current_sample_sequence'],previous_sample_stamp_ns=row['previous_sample_stamp_ns'],current_sample_stamp_ns=row['current_sample_stamp_ns'],continuation_bundle_generation=row['bundle_generation'],continuation_sample_id=row['sample_id'],waypoint_accepted=True,source_file=str(path.resolve())))
            continue
        ready=next((r for r in rows if int(r['flags']) & (1<<3)),None)
        for row in crossings:
            flags=int(row['flags'])
            o1.append(dict(run=run,classification=('READY_BEFORE_CROSS+CROSS_AND_READY_SAME_UPDATE' if ready and int(ready['ros_now_ns'])<int(row['ros_now_ns']) else 'CROSS_AND_READY_SAME_UPDATE' if flags&(1<<7) else 'CROSS_BEFORE_READY'),mission_hash=row['mission_hash'],localization_epoch=row['localization_epoch'],route_revision=row['route_revision'],waypoint_index=row['waypoint_index'],request_id=row['request_id'],first_ready_ros_ns=ready['ros_now_ns'] if ready else '',cross_ros_ns=row['ros_now_ns'],ready_lead_ms=(int(row['ros_now_ns'])-int(ready['ros_now_ns']))/1e6 if ready else '',diagnostic_sequence=row['diagnostic_sequence'],phase=row['phase'],outcome=row['outcome'],flags=row['flags'],crossing_error_m=row['crossing_error_m'],sample_gap_s=row['sample_gap_s'],previous_sample_sequence=row['previous_sample_sequence'],current_sample_sequence=row['current_sample_sequence'],previous_sample_stamp_ns=row['previous_sample_stamp_ns'],current_sample_stamp_ns=row['current_sample_stamp_ns'],continuation_bundle_generation=row['bundle_generation'],continuation_sample_id=row['sample_id'],waypoint_accepted=bool(flags&(1<<9)),source_file=str(path.resolve())))
write(out/'O1_PASS_THROUGH_EVENTS.csv',o1,['run','classification','mission_hash','localization_epoch','route_revision','waypoint_index','request_id','first_ready_ros_ns','cross_ros_ns','ready_lead_ms','diagnostic_sequence','phase','outcome','flags','crossing_error_m','sample_gap_s','previous_sample_sequence','current_sample_sequence','previous_sample_stamp_ns','current_sample_stamp_ns','continuation_bundle_generation','continuation_sample_id','waypoint_accepted','source_file'])

source=root/'run-C-normalized/O2_PUBLISH_RECEIVE.csv'
(out/'O2_PUBLISH_RECEIVE.csv').write_bytes(source.read_bytes().replace(b'\r\n', b'\n'))

o3=[]
for run in hold:
    base=root/f'run-{run}-normalized'
    for row in table(base/'O3_HOLD_PROTOCOL_EVENTS.csv'):
        phase=int(row['phase'])
        if phase not in [1,2,3,4,6,7,8] and not (phase==5 and (int(row['reason'])==4 or int(row['flags'])&(1<<4) or int(row['flags'])&(1<<5))): continue
        o3.append(dict(run=run,origin='AUDIT',kind={1:'HOLD_REQUEST',2:'SCHEDULE_ATTEMPT',3:'RETRY_TICK',4:'CALLBACK',5:'VEHICLE_STATUS',6:'EXECUTOR_DEACTIVATE',7:'FAILSAFE_DEFERRED',8:'EXECUTOR_ACTIVATE'}[phase],bag_observer_ns=row['bag_observer_ns'],ros_now_ns=row['ros_now_ns'],px4_source_stamp_ns=row['source_stamp_ns'],diagnostic_sequence=row['diagnostic_sequence'],attempt_generation=row['attempt_generation'],outcome=row['outcome'],nav_state=row['reason'] if phase==5 else '',executor_in_charge=row['aux_stamp_ns'] if phase==5 else '',flags=row['flags'],retry_deadline_steady_ns=row['retry_deadline_steady_ns'],raw_command='',raw_result='',source_file=str((base/'O3_HOLD_PROTOCOL_EVENTS.csv').resolve())))
    for line in (base/'px4_protocol_raw.jsonl').open():
        row=json.loads(line); topic=row['topic']; msg=row['message']
        if topic.endswith('vehicle_command_mode_executor') and msg.get('command')==100001 and msg.get('param1')==4:
            kind='RAW_HOLD_COMMAND'
        elif topic.endswith('vehicle_command_ack') and msg.get('command')==100001:
            kind='RAW_MODE_COMMAND_ACK'
        elif topic.endswith('mode_completed'):
            kind='RAW_MODE_COMPLETED'
        else: continue
        o3.append(dict(run=run,origin='PX4_RAW',kind=kind,bag_observer_ns=row['bag_observer_ns'],ros_now_ns='',px4_source_stamp_ns=int(msg.get('timestamp',0))*1000,diagnostic_sequence='',attempt_generation='',outcome='',nav_state=msg.get('nav_state',''),executor_in_charge='',flags='',retry_deadline_steady_ns='',raw_command=f"{msg.get('command','')}:{msg.get('param1','')}",raw_result=msg.get('result',''),source_file=str((base/'px4_protocol_raw.jsonl').resolve())))
o3.sort(key=lambda r:(r['run'],int(r['bag_observer_ns'])))
write(out/'O3_HOLD_PROTOCOL_EVENTS.csv',o3,['run','origin','kind','bag_observer_ns','ros_now_ns','px4_source_stamp_ns','diagnostic_sequence','attempt_generation','outcome','nav_state','executor_in_charge','flags','retry_deadline_steady_ns','raw_command','raw_result','source_file'])

sessions={'W1-fixed':'external-mode-check-20260923T063510-38964','W2':'external-mode-check-20260923T063803-42512','W3':'external-mode-check-20260923T064024-45983','W4':'external-mode-check-20260923T064238-49492','W5':'external-mode-check-20260923T064439-52901'}
o4=[]
for run in world:
    base=root/f'run-{run}-normalized'
    session=Path('/home/letandat/Dev/uav-navigation/.artifacts/runtime')/sessions[run]
    for line in (session/'audit_world_fault.jsonl').open():
        r=json.loads(line)
        if r['kind'] not in ['FAULT_START','FAULT_END']:continue
        o4.append(dict(run=run,kind=r['kind'],ros_now_ns=r['ros_now_ns'],source_stamp_ns=r.get('input_source_stamp_ns',''),bag_observer_ns='',diagnostic_sequence='',world_generation='',world_revision='',active_bundle_generation='',outcome='',flags='',reason='',dropped_input_count=r['dropped_count'],source_file=str((session/'audit_world_fault.jsonl').resolve())))
    w=table(base/'O4_WORLD_EVENTS.csv')
    for phase,kind in [(3,'WORLD_STALE_REJECT'),(4,'COMMAND_SUSPEND'),(8,'COMMAND_RESUME')]:
        selected=[r for r in w if int(r['phase'])==phase]
        for r in selected[:1]+selected[-1:] if len(selected)>1 else selected:
            o4.append(dict(run=run,kind=kind,ros_now_ns=r['ros_now_ns'],source_stamp_ns=r['source_stamp_ns'],bag_observer_ns=r['bag_observer_ns'],diagnostic_sequence=r['diagnostic_sequence'],world_generation=r['world_generation'],world_revision=r['world_revision'],active_bundle_generation=r['bundle_generation'],outcome=r['outcome'],flags=r['flags'],reason=r['reason'],dropped_input_count='',source_file=str((base/'O4_WORLD_EVENTS.csv').resolve())))
    for phase,kind in [(2,'FIRST_FRESH_WORLD_AFTER_FAULT'),(5,'RECERT_START_AFTER_FAULT'),(6,'RECERT_END_AFTER_FAULT'),(7,'WORLD_COMMIT_AFTER_FAULT')]:
        end=next((r['ros_now_ns'] for r in o4 if r['run']==run and r['kind']=='FAULT_END'),None)
        r=next((r for r in w if end and int(r['phase'])==phase and int(r['ros_now_ns'])>int(end)),None)
        if r:o4.append(dict(run=run,kind=kind,ros_now_ns=r['ros_now_ns'],source_stamp_ns=r['source_stamp_ns'],bag_observer_ns=r['bag_observer_ns'],diagnostic_sequence=r['diagnostic_sequence'],world_generation=r['world_generation'],world_revision=r['world_revision'],active_bundle_generation=r['bundle_generation'],outcome=r['outcome'],flags=r['flags'],reason=r['reason'],dropped_input_count='',source_file=str((base/'O4_WORLD_EVENTS.csv').resolve())))
    for line in (base/'audit_events.jsonl').open():
        r=json.loads(line)
        if r['event_type']!=6:continue
        o4.append(dict(run=run,kind='ADAPTER_LEASE_DISPOSITION',ros_now_ns=r['ros_now_ns'],source_stamp_ns=r['source_stamp_ns'],bag_observer_ns=r['bag_observer_ns'],diagnostic_sequence=r['diagnostic_sequence'],world_generation='',world_revision='',active_bundle_generation=r['bundle_generation'],outcome=r['outcome'],flags=r['flags'],reason=r['reason'],dropped_input_count='',source_file=str((base/'audit_events.jsonl').resolve())))
    for r in table(base/'O3_HOLD_PROTOCOL_EVENTS.csv'):
        if int(r['phase']) in (1,2) or (int(r['phase'])==5 and int(r['reason'])==4 and any(x['run']==run and x['kind']=='ADAPTER_LEASE_DISPOSITION' for x in o4) and int(r['ros_now_ns'])>next(int(x['ros_now_ns']) for x in o4 if x['run']==run and x['kind']=='ADAPTER_LEASE_DISPOSITION')):
            o4.append(dict(run=run,kind='HOLD_'+{1:'REQUEST',2:'SCHEDULE_ATTEMPT',5:'AUTO_LOITER_STATUS'}[int(r['phase'])],ros_now_ns=r['ros_now_ns'],source_stamp_ns=r['source_stamp_ns'],bag_observer_ns=r['bag_observer_ns'],diagnostic_sequence=r['diagnostic_sequence'],world_generation='',world_revision='',active_bundle_generation='',outcome=r['outcome'],flags=r['flags'],reason=r['reason'],dropped_input_count='',source_file=str((base/'O3_HOLD_PROTOCOL_EVENTS.csv').resolve())))
o4.sort(key=lambda r:(r['run'],int(r['ros_now_ns'])))
write(out/'O4_STALE_WORLD_EVENTS.csv',o4,['run','kind','ros_now_ns','source_stamp_ns','bag_observer_ns','diagnostic_sequence','world_generation','world_revision','active_bundle_generation','outcome','flags','reason','dropped_input_count','source_file'])
print('rows',len(o1),sum(1 for r in table(out/'O2_PUBLISH_RECEIVE.csv')),len(o3),len(o4))
