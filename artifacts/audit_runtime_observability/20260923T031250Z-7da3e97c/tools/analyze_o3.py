"""Normalize observed Hold-adjacent events, without inventing request correlations."""
import csv
import json
from pathlib import Path
import re
import sys

TARGET='7da3e97cb399c2e39d62cfe60213a45e8a92300e'


def main():
    prior,runs,out=map(Path,sys.argv[1:4])
    with prior.open() as f: rows=list(csv.DictReader(f))
    fields=list(rows[0])
    focus=[Path(p) for p in sys.argv[4:]]
    for run in focus:
        meta_path=run/'metadata.json'
        if not meta_path.exists():continue
        meta=json.loads(meta_path.read_text())
        if meta.get('repo_commit')!=TARGET or meta.get('repo_dirty'):continue
        base={k:'' for k in fields};base['run']=run.name
        for path in sorted((run/'logs').glob('px4_navigation_external_mode_node*')):
            for line in path.read_text(errors='replace').splitlines():
                if not any(s in line for s in ('PX4 Hold handover','executor deactivated','Avoidance Mission completed')):continue
                match=re.search(r'\[(\d+\.\d+)\]',line)
                event=('hold_request_log' if 'requesting PX4 Hold' in line else
                       'hold_completion_callback_log' if 'PX4 Hold handover completed' in line else
                       'hold_attempt_failure_log' if 'PX4 Hold handover attempt' in line else
                       'executor_deactivated_log' if 'executor deactivated' in line else 'mission_completed_log')
                rows.append({**base,'source':'adapter_log','event':event,'log_wall_s':match.group(1) if match else '',
                             'result':'Deactivated' if 'result=Deactivated' in line else 'Success' if 'result=Success' in line else '',
                             'detail':line[-240:]})
        for path,kind in ((run/'execution_timeline.jsonl','vehicle_status'),(run/'planning_timeline.jsonl','event')):
            if not path.exists():continue
            for line in path.open():
                rec=json.loads(line)['record'];p=rec.get('payload',{})
                if rec.get('kind')!=kind:continue
                if kind=='event' and p.get('name') not in ('px4_hold_handover_requested','external_mode_exit_observed','external_mode_entered'):continue
                rows.append({**base,'source':'runner_vehicle_status_change' if kind=='vehicle_status' else 'runner_event',
                             'event':'vehicle_status_change' if kind=='vehicle_status' else p['name'],
                             'sim_time_ns':rec.get('sim_time_ns',''),
                             'observer_steady_ns':rec.get('observer_record_steady_ns',''),
                             'px4_boot_us':p.get('timestamp_us','') if kind=='vehicle_status' else '',
                             'nav_state':p.get('nav_state','') if kind=='vehicle_status' else '',
                             'executor_in_charge':p.get('executor_in_charge','') if kind=='vehicle_status' else '',
                             'failsafe':p.get('failsafe','') if kind=='vehicle_status' else '',
                             'detail':json.dumps(p.get('detail',{}),sort_keys=True) if kind=='event' else ''})
    for run in list(sorted(runs.glob('*20260922*')))+focus:
        if not (run/'metadata.json').exists(): continue
        meta=json.loads((run/'metadata.json').read_text())
        if meta.get('repo_commit')!=TARGET or meta.get('repo_dirty'): continue
        path=run/'planning_timeline.jsonl'
        if not path.exists(): continue
        for line in path.open():
            rec=json.loads(line)['record']
            if rec.get('kind')!='command_ack': continue
            p=rec.get('payload',{})
            row={k:'' for k in fields}
            row.update(run=run.name,source='runner_command_ack_unattributed',event='vehicle_command_ack_observed',
                       sim_time_ns=rec.get('sim_time_ns',''),observer_steady_ns=rec.get('observer_record_steady_ns',''),
                       result=p.get('result',''),detail=f"command={p.get('command','')}; no request/target identity")
            rows.append(row)
    with out.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader();w.writerows(rows)
    print('rows',len(rows),'runs',len({r['run'] for r in rows}),
          'unattributed_ack',sum(r['event']=='vehicle_command_ack_observed' for r in rows))


if __name__=='__main__':main()
