"""Collapse pinned RouteProgress sample opportunities to episode-level evidence."""
import csv
import math
from collections import defaultdict
from pathlib import Path
import sys


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def main():
    previous,out,progress_path,raw_root=map(Path,sys.argv[1:5])
    selected_run=sys.argv[5]
    grouped=defaultdict(list)
    for row in read(previous):
        grouped[(row['run'],row['epoch'],row['waypoint'],row['request'])].append(row)
    progress=read(progress_path)
    output=[]
    for (run,epoch,waypoint,request),rs in grouped.items():
        rs.sort(key=lambda r:int(r['odom_current_source_ns']))
        first=rs[0];t0=int(first['odom_current_source_ns'])
        ready=int(first['first_future_continuation_source_ns']) if first['first_future_continuation_source_ns'] else 0
        observed_before_ready=bool(ready and ready>t0 and first['latest_command_continuation']!='1')
        crossings_before_ready=sum(int(x['odom_current_source_ns'])<ready for x in rs) if observed_before_ready else 0
        before=[p for p in progress if run==selected_run and p['epoch']==epoch and p['waypoint']==waypoint and p['request']==request]
        reentry=any(any(int(a['odom_current_source_ns'])<int(p['source_ns'])<int(b['odom_current_source_ns'])
                        and p['inside_ball']=='0' for p in before)
                    for a,b in zip(rs,rs[1:]))
        if first['classification']=='CONTINUATION_PUBLISHED_BEFORE_CROSSING':
            classification='READY_BEFORE_CROSSING_OPPORTUNITY'
        elif waypoint=='0':
            classification='INITIAL_HANDOFF_EXCEPTION_OR_INSUFFICIENT_TRACE'
        elif observed_before_ready:
            classification='CURRENT_IN_BALL_BEFORE_READY;REENTRY_ACCEPTED' if reentry else 'CURRENT_IN_BALL_BEFORE_READY'
        else:
            classification='INITIAL_HANDOFF_OR_INSUFFICIENT_TRACE'
        arc_delta=''
        path_distance=''
        if before and observed_before_ready:
            a=min(before,key=lambda p:abs(int(p['source_ns'])-t0))
            b=min(before,key=lambda p:abs(int(p['source_ns'])-ready))
            if int(a['source_ns'])==t0 and abs(int(b['source_ns'])-ready)<=20000000:
                arc_delta=float(b['progress_arc_m'])-float(a['progress_arc_m'])
            odom_path=raw_root/run/'odom_monitor.csv'
            if odom_path.exists():
                samples=[s for s in read(odom_path) if s['epoch']==epoch and t0<=int(s['source_ns'])<=ready]
                if samples and int(samples[0]['source_ns'])==t0 and abs(int(samples[-1]['source_ns'])-ready)<=20000000:
                    path_distance=sum(math.dist([float(a[k]) for k in ('x','y','z')],
                                                [float(b[k]) for k in ('x','y','z')])
                                      for a,b in zip(samples,samples[1:]))
        output.append(dict(run=run,localization_epoch=epoch,waypoint_index=waypoint,request_id=request,
            sample_opportunities=len(rs),first_source_ns=t0,first_sequence=first['odom_sequence'],
            first_previous_source_ns=first['odom_prev_source_ns'],first_geometry='CURRENT_IN_BALL' if first['inside_ball']=='1' else 'TWO_SAMPLE_SEGMENT',
            first_crossing_error_m=first['crossing_error_m'],first_continuation_published=first['latest_command_continuation'],
            first_ready_source_ns=ready or '',source_delay_ms=(ready-t0)/1e6 if observed_before_ready else '',
            source_samples_before_ready=crossings_before_ready if observed_before_ready else 0,
            reentry_observed=reentry,accepted_source_ns=first['mission_accept_source_ns'],
            route_arc_delta_before_ready_m=arc_delta,measured_path_distance_before_ready_m=path_distance,
            classification=classification,
            adapter_receive_ns='UNOBSERVABLE_WITH_CURRENT_TARGET',mission_update_ns='UNOBSERVABLE_WITH_CURRENT_TARGET',
            exact_callback_continuation='UNOBSERVABLE_WITH_CURRENT_TARGET',lost_crossing='NOT_REPRODUCED'))
    output.sort(key=lambda r:(r['run'],int(r['first_source_ns'])))
    with out.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(output[0]),lineterminator='\n');w.writeheader();w.writerows(output)
    print('episodes',len(output),'ready_before',sum(x['classification']=='READY_BEFORE_CROSSING_OPPORTUNITY' for x in output),
          'before_ready_non_initial',sum('BEFORE_READY' in x['classification'] for x in output),'reentry',sum(x['reentry_observed'] for x in output),
          'two_sample_first',sum(x['first_geometry']=='TWO_SAMPLE_SEGMENT' for x in output))


if __name__=='__main__':main()
