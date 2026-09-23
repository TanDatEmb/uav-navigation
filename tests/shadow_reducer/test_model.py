import unittest
from tools.shadow_reducer.model import *


def cursor(segment, stamp, seq):
    return RouteCursor(segment, 0.5, stamp, seq)


def cross(route, waypoint=1):
    return Crossing(route, waypoint, cursor(waypoint - 1, 10, 1),
                    cursor(waypoint, 20, 2), 0.2, Quality.OBSERVED)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.r = RouteIdentity(1, 1, 1)
        self.m = MissionProgressShadow()

    def test_pt1_ready_then_cross(self):
        self.m.reset_route(self.r); self.m.continuation(1); self.m.observe_crossing(cross(self.r))
        self.assertTrue(self.m.accept_pass(1))

    def test_pt2_cross_then_ready(self):
        self.m.observe_crossing(cross(self.r)); self.assertFalse(self.m.accept_pass(1))
        self.m.continuation(1); self.assertTrue(self.m.accept_pass(1))

    def test_pt3_leave_ball_then_ready(self):
        self.m.observe_crossing(cross(self.r))
        self.m.measured = cursor(2, 30, 3)  # route-relative progress survives geometry re-entry
        self.m.continuation(1); self.assertTrue(self.m.accept_pass(1))

    def test_pt4_localization_reset(self):
        self.m.observe_crossing(cross(self.r)); self.m.reset_route(RouteIdentity(1, 1, 2))
        self.m.continuation(1); self.assertFalse(self.m.accept_pass(1))

    def test_pt5_route_revision(self):
        self.m.observe_crossing(cross(self.r)); self.m.reset_route(RouteIdentity(1, 2, 1))
        self.m.continuation(1); self.assertFalse(self.m.accept_pass(1))

    def test_pt6_reverse_segment(self):
        self.m.observe_crossing(cross(self.r)); self.m.reverse_to(cursor(0, 30, 3))
        self.m.continuation(1); self.assertFalse(self.m.accept_pass(1))

    def test_self_intersection_and_duplicate(self):
        self.m.observe_crossing(cross(self.r)); self.m.continuation(1)
        self.assertTrue(self.m.accept_pass(1)); self.assertFalse(self.m.accept_pass(1))
        self.assertEqual(self.m.measured.segment, 1)

    def test_stop_needs_measured_stop(self):
        self.assertFalse(self.m.accept_stop(1, False)); self.assertTrue(self.m.accept_stop(1, True))


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.c = ContextKey(1, 2, 3, 4, 5)
        self.e = ExecutionAuthorityShadow(context=self.c)
        self.a = Trajectory(1, 'MAIN', self.c, 100)
        self.b = Trajectory(2, 'MAIN', self.c, 100)

    def test_ex1_atomic_cutover(self):
        self.assertTrue(self.e.stage(self.a)); self.assertTrue(self.e.activate())
        self.assertTrue(self.e.stage(self.b)); self.assertEqual(self.e.active, self.a)
        self.assertTrue(self.e.activate()); self.assertEqual(self.e.active, self.b)

    def test_ex2_stale_context(self):
        old = Trajectory(3, 'MAIN', ContextKey(1, 2, 3, 3, 5))
        self.assertFalse(self.e.stage(old))

    def test_ex3_stop_late_nominal(self):
        self.e.stage(self.a); self.e.activate(); self.e.safety_stop()
        self.assertFalse(self.e.stage(self.b)); self.assertFalse(self.e.activate())

    def test_ex4_ex5_measured_stop(self):
        self.e.stage(self.a); self.e.activate(); self.e.safety_stop()
        self.assertEqual(self.e.phase, ExecutionPhase.COMMITTED_STOPPING)
        self.e.measured_stop(); self.assertEqual(self.e.phase, ExecutionPhase.STOPPED)
        self.e.restart_new_session(self.c); self.assertEqual(self.e.phase, ExecutionPhase.NONE)

    def test_wl1_wl2_certificate_expiry(self):
        self.e.stage(self.a); self.e.activate(); self.e.world(ContextKey(1,2,3,4,6))
        self.assertTrue(self.e.can_publish(1,99)); self.assertFalse(self.e.can_publish(1,100))
        self.e.safety_stop(); self.assertFalse(self.e.can_publish(1,101))

    def test_wl3_wl4_fence(self):
        self.e.stage(self.a); self.e.activate(); self.e.release()
        self.e.world(ContextKey(1,2,3,4,6))
        self.assertFalse(self.e.stage(self.b)); self.assertFalse(self.e.can_publish(1,10))
        self.assertEqual(self.e.fenced_generation, 1)


class HoldTests(unittest.TestCase):
    def setUp(self):
        self.p = Px4AuthorityShadow()
        self.p.request_hold(1, 10)

    def test_ack_then_status(self):
        self.p.ack(11, 0); self.assertFalse(self.p.hold.confirmed())
        self.p.vehicle_status(12, 4, 12); self.assertTrue(self.p.hold.confirmed())

    def test_status_then_completion(self):
        self.p.vehicle_status(12,4,12); self.p.completed(13,0)
        self.assertTrue(self.p.hold.confirmed())

    def test_completion_then_status(self):
        self.p.completed(11,0); self.assertTrue(self.p.hold.retry_required())
        self.p.vehicle_status(12,4,12); self.assertTrue(self.p.hold.confirmed())

    def test_completion_no_status(self):
        self.p.completed(11,0); self.assertFalse(self.p.hold.confirmed())
        self.assertTrue(self.p.hold.retry_required())

    def test_deactivated_is_not_hold(self):
        self.p.completed(11,2); self.assertFalse(self.p.hold.confirmed())

    def test_takeover(self):
        self.p.takeover('OPERATOR'); self.assertFalse(self.p.hold.confirmed())
        self.assertFalse(self.p.hold.retry_required())

    def test_failsafe(self):
        self.p.takeover('FAILSAFE'); self.assertFalse(self.p.hold.confirmed())

    def test_no_response_and_stale_status(self):
        self.p.vehicle_status(1,4,1); self.assertFalse(self.p.hold.confirmed())
        self.assertTrue(self.p.hold.retry_required())

    def test_lease(self):
        p=Px4AuthorityShadow(); p.reference((1,2),10)
        self.assertFalse(p.lease_expired(100_000_009)); self.assertTrue(p.lease_expired(100_000_010))
        p.request_hold(1,100_000_010); p.reference((2,3),100_000_011)
        self.assertIsNone(p.last_reference_key)


class TraceTests(unittest.TestCase):
    def test_gap(self):
        s=ShadowReducer(); s.trace_gap()
        self.assertEqual(s.compare(True,True),Verdict.INSUFFICIENT)
        self.assertEqual(s.compare(True,False),Verdict.INSUFFICIENT)

    def test_unknown(self):
        self.assertEqual(ShadowReducer().compare(None,True),Verdict.INSUFFICIENT)


class AdditionalAdversarialTests(unittest.TestCase):
    def test_old_session_result_cannot_rearm(self):
        c=ContextKey(1,2,3,4,5); e=ExecutionAuthorityShadow(context=c)
        a=Trajectory(7,'MAIN',c); self.assertTrue(e.stage(a)); self.assertTrue(e.activate())
        e.release(); e.world(ContextKey(1,2,3,4,6))
        self.assertFalse(e.stage(a)); self.assertFalse(e.activate())

    def test_crossing_gap_and_out_of_order_are_discarded(self):
        r=RouteIdentity(1,1,1); m=MissionProgressShadow(route=r)
        long_gap=Crossing(r,1,cursor(0,1,1),cursor(1,300_000_002,2),0.1,Quality.OBSERVED)
        m.observe_crossing(long_gap); self.assertIsNone(m.crossing)
        reversed_stamp=Crossing(r,1,cursor(0,10,2),cursor(1,9,1),0.1,Quality.OBSERVED)
        m.observe_crossing(reversed_stamp); self.assertIsNone(m.crossing)

    def test_reverse_within_same_segment_invalidates(self):
        r=RouteIdentity(1,1,1); m=MissionProgressShadow(); m.observe_crossing(cross(r))
        m.reverse_to(RouteCursor(1,0.1,30,3)); m.continuation(1)
        self.assertFalse(m.accept_pass(1))

    def test_continuation_wrong_request(self):
        r=RouteIdentity(1,1,1); m=MissionProgressShadow();m.reset_route(r);m.set_gate(1,2)
        m.observe_crossing(Crossing(r,1,cursor(0,10,1),cursor(1,20,2),0.1,Quality.OBSERVED,'SEGMENT',2))
        m.continuation(1,3);self.assertFalse(m.accept_pass(1))

    def test_duplicate_status_not_new_confirmation(self):
        p=Px4AuthorityShadow();p.vehicle_status(10,4,10);p.request_hold(1,11)
        p.vehicle_status(10,4,12);self.assertFalse(p.hold.confirmed())
        p.vehicle_status(13,4,13);self.assertTrue(p.hold.confirmed())

    def test_queue_drop_forces_insufficient(self):
        s=ShadowReducer();s.trace_gap();self.assertEqual(s.compare(False,False),Verdict.INSUFFICIENT)
    def test_hold_status_lost_on_new_non_hold_status(self):
        p=Px4AuthorityShadow();p.request_hold(1,10);p.vehicle_status(11,4,11)
        self.assertTrue(p.hold.confirmed());p.vehicle_status(12,26,12)
        self.assertFalse(p.hold.confirmed());self.assertEqual(p.hold.takeover,'NON_HOLD_STATUS_UNKNOWN_CAUSE')
    def test_retry_deadline_is_independent(self):
        p=Px4AuthorityShadow();p.request_hold(1,10);p.hold.retry_deadline_ns=100
        self.assertFalse(p.hold.retry_due(99));self.assertTrue(p.hold.retry_due(100))
        p.vehicle_status(11,4,11);self.assertFalse(p.hold.retry_due(101))
    def test_recoverable_braking_requires_continuity_certificate(self):
        c=ContextKey(1,2,3,4,5);e=ExecutionAuthorityShadow(context=c)
        a=Trajectory(1,'MAIN',c,100);e.stage(a);e.activate()
        self.assertTrue(e.recoverable_brake())
        self.assertFalse(e.recover_from_braking(Trajectory(2,'MAIN',c,100),continuity_proven=False,now_ns=20))
        self.assertTrue(e.recover_from_braking(Trajectory(2,'MAIN',c,100),continuity_proven=True,now_ns=20))
        e.safety_stop();self.assertFalse(e.recover_from_braking(a,continuity_proven=True,now_ns=30))
    def test_px4_local_witness_reset_and_order(self):
        p=Px4AuthorityShadow()
        w=LocalBoundaryWitness(1,10,11,9,11,2,(0.,0.,0.),(0.,0.,0.),(1.,2.,3.),0.,'VALID',10)
        p.observe_local(w);p.observe_local(LocalBoundaryWitness(1,9,12,9,12,2,(9.,9.,9.),(0.,0.,0.),(1.,2.,3.),0.,'VALID',9))
        self.assertEqual(p.local,w);p.localization_reset();self.assertIsNone(p.local)
    def test_crossing_evidence_quality_and_error(self):
        r=RouteIdentity(1,1,1);m=MissionProgressShadow()
        m.observe_crossing(Crossing(r,1,cursor(0,10,1),cursor(1,20,2),0.1,Quality.TRACE_GAP))
        self.assertIsNone(m.crossing)
        m.observe_crossing(Crossing(r,1,cursor(0,10,1),cursor(1,20,2),float('nan'),Quality.OBSERVED))
        self.assertIsNone(m.crossing)
        m.observe_crossing(Crossing(r,1,cursor(0,10,1),cursor(1,20,2),0.1,Quality.OBSERVED))
        self.assertEqual(m.crossing.error_m,0.1)

class ContextIdentityTests(unittest.TestCase):
    def test_goal_and_dynamics_identity_reject_stale_result(self):
        c=ContextKey(1,2,3,4,5,goal_epoch=10,waypoint=1,request_id=2,dynamics_hash=99)
        e=ExecutionAuthorityShadow(context=c)
        self.assertFalse(e.stage(Trajectory(1,'MAIN',ContextKey(1,2,3,4,5,goal_epoch=11,waypoint=1,request_id=2,dynamics_hash=99))))
        self.assertFalse(e.stage(Trajectory(1,'MAIN',ContextKey(1,2,3,4,5,goal_epoch=10,waypoint=1,request_id=2,dynamics_hash=100))))
        self.assertTrue(e.stage(Trajectory(1,'MAIN',c)))


if __name__ == "__main__": unittest.main()
