"""Audit-only parser/model checks; not product or flight tests."""
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parents[2]


def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


o2=module('analyze_o2')
hold=module('hold_order_model')
world=module('world_lease_model')


class CommandIdentityTests(unittest.TestCase):
    def test_scope_and_reset(self):
        p=dict(runtime_instance_id='run-a',localization_epoch=1,goal_epoch=2,request_id=3,bundle_generation=4,sample_id=1)
        self.assertEqual(o2.key(p),o2.key(dict(p)))
        for field,new in [('runtime_instance_id','run-b'),('localization_epoch',2),('goal_epoch',3),('request_id',4),('bundle_generation',5),('sample_id',2)]:
            self.assertNotEqual(o2.key(p),o2.key(dict(p,**{field:new})))
        self.assertIsNone(o2.key(dict(p,sample_id=None)))
        self.assertIsNone(o2.paired_duration_ms(100,'ros_sim',120,'host_steady'))
        self.assertIsNone(o2.paired_duration_ms(120,'host_steady',100,'host_steady'))

    def test_duplicates_missing_and_clock_rejection(self):
        with tempfile.TemporaryDirectory() as td:
            run=Path(td);(run/'metadata.json').write_text(json.dumps({'repo_commit':o2.TARGET,'repo_dirty':False}))
            (run/'report.json').write_text(json.dumps({'infrastructure':{'classification':'VALID'}}))
            base=dict(runtime_instance_id='r',localization_epoch=1,goal_epoch=1,request_id=1,bundle_generation=1,sample_id=1)
            p=[{'record':{'kind':'pva_command','payload':dict(base,execution_authorization_steady_ns=100),'observer_record_steady_ns':110}}]*2
            (run/'planning_timeline.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in p))
            traces=[]
            for sample,start in [(1,90),(1,120),(2,130)]:
                traces.append({'record':{'kind':'px4_input_trace','payload':dict(base,sample_id=sample,setpoint_kind='tracking',trace_values={'update_start_steady_ns':str(start)})}})
            (run/'execution_timeline.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in traces))
            r=o2.analyze_run(run)[0]
            self.assertEqual((r['duplicate_command_keys'],r['unmatched_tracking_traces'],r['invalid_clock_pairs'],r['n']),(1,1,1,1))


class HoldOrderingTests(unittest.TestCase):
    def test_focused_protocol_bag_has_correlated_request_ack_status(self):
        with (ROOT/'O3_FOCUSED_PROTOCOL_EVENTS.csv').open() as f:
            rows=list(csv.DictReader(f))
        req=next(x for x in rows if x['topic']=='/fmu/in/vehicle_command_mode_executor' and x['command']=='100001' and x['param1']=='4.0')
        ack=next(x for x in rows if x['topic']=='/fmu/out/vehicle_command_ack' and x['command']=='100001'
                 and x['target_component']==req['source_component'] and int(x['bag_observer_wall_ns'])>int(req['bag_observer_wall_ns']))
        status=next(x for x in rows if x['topic']=='/fmu/out/vehicle_status_v1' and x['nav_state']=='4'
                    and int(x['bag_observer_wall_ns'])>int(ack['bag_observer_wall_ns']))
        self.assertEqual(ack['result'],'0')
        self.assertGreater(int(status['px4_boot_us']),int(ack['px4_boot_us']))
        self.assertEqual(status['executor_in_charge'],'1')
        self.assertFalse(any(x['topic']=='/fmu/out/mode_completed' and x['nav_state']=='4' for x in rows))

    def test_completion_cannot_confirm(self):
        self.assertEqual(hold.classify([{'kind':'request','steady_ns':10},{'kind':'ack','steady_ns':11},{'kind':'completion','steady_ns':12}]),'UNKNOWN_NO_POST_REQUEST_AUTHORITY_WITNESS')

    def test_out_of_order_duplicate_and_preexisting_status(self):
        events=[{'kind':'status','steady_ns':5,'boot_us':10,'fresh':True,'nav_state':'AUTO_LOITER'},
                {'kind':'request','steady_ns':10},
                {'kind':'status','steady_ns':20,'boot_us':20,'fresh':True,'nav_state':'AUTO_LOITER'},
                {'kind':'status','steady_ns':21,'boot_us':20,'fresh':True,'nav_state':'AUTO_LOITER'}]
        self.assertEqual(hold.classify(list(reversed(events))),'OBSERVED_POST_REQUEST_LOITER_CAUSE_UNCONFIRMED')
        self.assertEqual(hold.classify(events[:2]),'UNKNOWN_NO_POST_REQUEST_AUTHORITY_WITNESS')
        self.assertEqual(hold.classify(events+[{'kind':'status','steady_ns':22,'boot_us':22,'fresh':True,'operator':True}]),'EXTERNAL_OPERATOR_TAKEOVER')

    def test_reset_missing_and_takeover(self):
        self.assertEqual(hold.classify([{'kind':'request','steady_ns':None}]),'UNKNOWN_MISSING_CLOCK')
        self.assertEqual(hold.classify([{'kind':'request','steady_ns':1},{'kind':'status','steady_ns':2,'boot_us':3,'fresh':True,'failsafe':True}]),'EXTERNAL_FAILSAFE_TAKEOVER')


class WorldLeaseTests(unittest.TestCase):
    def test_missing_receipt_and_handover(self):
        base=[dict(kind='suspend',steady_ns=10,bundle=3),dict(kind='recertified',steady_ns=30,bundle=3),dict(kind='resume',steady_ns=40,bundle=3)]
        self.assertEqual(world.classify(base),'INSUFFICIENT_TRACE')
        paired=base+[dict(kind='last_adapter_receive',steady_ns=5,bundle=3)]
        self.assertEqual(world.classify(paired,lease_ns=100),'RECERTIFIED_AND_RESUMED_WITHIN_LEASE_WINDOW')
        self.assertEqual(world.classify(paired+[dict(kind='adapter_handover',steady_ns=35,bundle=3)],lease_ns=100),'RESUME_AFTER_OBSERVED_HANDOVER')
        self.assertEqual(world.classify(paired,lease_ns=30),'RESUME_AFTER_LEASE_WINDOW_NO_HANDOVER_WITNESS')
        self.assertEqual(world.classify(base+[dict(kind='last_adapter_receive',steady_ns=5,bundle=4)]),'IDENTITY_CONFLICT')


class CrossingGoldenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.binary=Path(cls.tmp.name)/'route_progress_replay'
        subprocess.run(['g++','-std=c++20','-O2','-I',str(REPO/'src/contracts/navigation_mission/include'),
                        '-I',str(REPO/'src/mapping/navigation_world_model/include'),'-I','/usr/include/eigen3',
                        str(ROOT/'tools/route_progress_replay.cpp'),str(REPO/'src/contracts/navigation_mission/src/mission.cpp'),
                        str(REPO/'src/contracts/navigation_mission/src/route_progress.cpp'),'-lyaml-cpp','-o',str(cls.binary)],check=True)

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def replay(self,positions,epochs=None):
        epochs=epochs or [1]*len(positions)
        with tempfile.TemporaryDirectory() as td:
            td=Path(td)
            (td/'mission.yaml').write_text('''mission:\n  version: 1\n  id: golden\n  frame: lio_odom\n  waypoints:\n    - {id: start, position: [0, 0, 3], acceptance_radius_m: 0.5, behavior: pass_through}\n    - {id: gate, position: [10, 0, 3], acceptance_radius_m: 0.5, behavior: pass_through}\n    - {id: finish, position: [20, 0, 3], acceptance_radius_m: 0.5, behavior: stop}\n''')
            (td/'goal.csv').write_text('bag_ns,source_ns,mission,waypoint,request,route_revision,route_progress_valid,behavior\n1,1000000000,golden,1,1,1,True,0\n')
            with (td/'odom.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['bag_ns','source_ns','epoch','sequence','x','y','z','vx','vy','vz'])
                for i,x in enumerate(positions):w.writerow([i+1,1000000000+i*50000000,epochs[i],i+1,x,0,3,0,0,0])
            p=subprocess.run([str(self.binary),str(td/'mission.yaml'),str(td/'odom.csv'),str(td/'goal.csv')],text=True,capture_output=True,check=True)
            return list(csv.DictReader(p.stdout.splitlines()))

    def test_forward_skip_reverse_and_in_ball(self):
        self.assertNotEqual(self.replay([9.2,10.8])[1]['error_m'],'')
        self.assertEqual(self.replay([10.8,9.2])[1]['error_m'],'')
        self.assertNotEqual(self.replay([9.8])[0]['error_m'],'')

    def test_localization_reset_breaks_two_sample_crossing(self):
        self.assertEqual(self.replay([9.2,10.8],[1,2])[1]['error_m'],'')


if __name__=='__main__':unittest.main()
