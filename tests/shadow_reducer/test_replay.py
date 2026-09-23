import json
import tempfile
import unittest
from pathlib import Path
from tools.shadow_reducer.replay import replay_run


class ReplayTests(unittest.TestCase):
    def test_minimal_normalized_stream(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); base=dict(process_incarnation=1, producer_id=2, bag_observer_ns=1,
                                  diagnostic_sequence=1, event_name='MISSION_GATE',
                                  mission_hash=5, route_revision=1, localization_epoch=7,
                                  waypoint_index=1, request_id=2, bundle_generation=3,
                                  phase=2, flags=sum(1<<n for n in (0,1,2,3,7,8,9,10)),
                                  outcome=1, crossing_error_m=0.1,
                                  previous_sample_stamp_ns=10, current_sample_stamp_ns=20,
                                  previous_sample_sequence=1, current_sample_sequence=2,
                                  world_generation=1, world_revision=1,
                                  goal_epoch=1, sample_id=1, ros_now_ns=20,
                                  source_stamp_ns=20, reason=0)
            (p/'audit_events.jsonl').write_text(json.dumps(base)+'\n')
            (p/'trace_integrity.json').write_text(json.dumps(dict(command_pairs=[dict(status='AMBIGUOUS_OR_UNPAIRED', first_use='NOT_OBSERVED')],
                                                                  queue_drops_observed=0, sequence_gaps=[])))
            r=replay_run(p)
            self.assertEqual(r['counts']['pass_MATCH'],1)
            self.assertEqual(r['counts']['command_INSUFFICIENT_TRACE'],1)

    def test_fenced_old_receive_is_rejected_not_resurrected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            common=dict(process_incarnation=1,producer_id=2,mission_hash=5,route_revision=1,
                        localization_epoch=7,waypoint_index=1,request_id=2,bundle_generation=3,
                        world_generation=1,world_revision=1,goal_epoch=1,sample_id=1,
                        source_stamp_ns=10,reason=0,flags=0,outcome=0)
            rows=[dict(common,event_name='COMMAND_PUBLISH',phase=0,bag_observer_ns=1,diagnostic_sequence=1,ros_now_ns=10),
                  dict(common,event_name='HOLD_TRANSFER',phase=1,bag_observer_ns=2,diagnostic_sequence=2,ros_now_ns=11,attempt_generation=1),
                  dict(common,event_name='COMMAND_RECEIVE',phase=0,bag_observer_ns=3,diagnostic_sequence=3,ros_now_ns=12,outcome=2,reason=2)]
            (p/'audit_events.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows))
            (p/'trace_integrity.json').write_text(json.dumps(dict(command_pairs=[],queue_drops_observed=0,sequence_gaps=[])))
            r=replay_run(p)
            self.assertEqual(r['counts']['receive_rejected_after_fence'],1)
            self.assertEqual(r['issues'],[])

if __name__=='__main__': unittest.main()
