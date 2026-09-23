"""Diagnostic-only semantic reducer. Never imported by product runtime."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Optional


class Quality(str, Enum):
    OBSERVED = "Observed"
    INFERRED = "InferredFromSourceInvariant"
    UNKNOWN = "Unknown"
    TRACE_GAP = "TraceGap"


class Verdict(str, Enum):
    MATCH = "MATCH"
    DIVERGENCE = "DIVERGENCE"
    INSUFFICIENT = "INSUFFICIENT_TRACE"
    UNKNOWN_POLICY = "EXTERNAL_CONTRACT_UNKNOWN"


@dataclass(frozen=True)
class RouteIdentity:
    mission: int
    revision: int
    localization: int


@dataclass(frozen=True)
class RouteCursor:
    # Segment plus within-segment u disambiguates self-intersections. The
    # existing trace lacks u; replay marks the route branch inferred.
    segment: int
    u: Optional[float]
    source_stamp_ns: int
    sample_sequence: int


@dataclass(frozen=True)
class Crossing:
    route: RouteIdentity
    waypoint: int
    before: RouteCursor
    after: RouteCursor
    error_m: float
    quality: Quality
    kind: str = "SEGMENT"  # SEGMENT or BALL; source checks differ
    request_id: int = 0


@dataclass(frozen=True)
class ContinuationWitness:
    route: RouteIdentity
    waypoint: int
    request_id: int
    bundle_generation: int


@dataclass
class MissionProgressShadow:
    route: Optional[RouteIdentity] = None
    measured: Optional[RouteCursor] = None
    crossing: Optional[Crossing] = None
    accepted_waypoint: int = -1
    gate: Optional[tuple[int, int]] = None  # current waypoint and request
    continuation_witness: Optional[ContinuationWitness] = None

    def reset_route(self, route: RouteIdentity) -> None:
        if route != self.route:
            self.route = route
            self.measured = None
            self.crossing = None
            self.accepted_waypoint = -1
            self.gate = None
            self.continuation_witness = None

    def set_gate(self, waypoint: int, request_id: int) -> None:
        new_gate = (waypoint, request_id)
        if self.gate is not None and self.gate != new_gate:
            self.crossing = None
            self.continuation_witness = None
        self.gate = new_gate

    def observe_crossing(self, crossing: Crossing) -> None:
        self.reset_route(crossing.route)
        if crossing.quality not in (Quality.OBSERVED, Quality.INFERRED) or not math.isfinite(crossing.error_m) or crossing.error_m < 0:
            return
        if crossing.kind not in ("SEGMENT", "BALL"):
            return
        if crossing.kind == "SEGMENT":
            gap = crossing.after.source_stamp_ns - crossing.before.source_stamp_ns
            if gap <= 0 or gap > 250_000_000:
                return
            if crossing.after.sample_sequence <= crossing.before.sample_sequence:
                return
        if self.measured and crossing.after.source_stamp_ns < self.measured.source_stamp_ns:
            return
        if self.gate is not None and self.gate != (crossing.waypoint, crossing.request_id):
            return
        self.measured = crossing.after
        self.crossing = crossing

    def reverse_to(self, cursor: RouteCursor) -> None:
        if self.measured and (cursor.segment < self.measured.segment or
                              (cursor.segment == self.measured.segment and
                               cursor.u is not None and self.measured.u is not None and
                               cursor.u < self.measured.u)):
            self.crossing = None  # reverse past route segment: prior crossing cannot authorize later gate
        self.measured = cursor

    def continuation(self, waypoint: int, request_id: int = 0, bundle_generation: int = 0) -> None:
        if self.route is not None:
            self.continuation_witness = ContinuationWitness(self.route, waypoint, request_id, bundle_generation)

    def accept_pass(self, waypoint: int) -> bool:
        witness = self.crossing
        if not witness or witness.waypoint != waypoint or witness.route != self.route:
            return False
        c = self.continuation_witness
        if (c is None or c.route != self.route or c.waypoint != waypoint or
                c.request_id != witness.request_id or waypoint <= self.accepted_waypoint):
            return False
        self.accepted_waypoint = waypoint
        self.crossing = None
        self.continuation_witness = None
        return True

    def accept_stop(self, waypoint: int, measured_stop: bool) -> bool:
        if not measured_stop or waypoint <= self.accepted_waypoint:
            return False
        self.accepted_waypoint = waypoint
        self.crossing = None
        self.continuation_witness = None
        return True


@dataclass(frozen=True)
class ContextKey:
    localization: int
    mission: int
    route: int
    world_generation: int
    world_revision: int
    goal_epoch: int = 0
    waypoint: int = 0
    request_id: int = 0
    dynamics_hash: int = 0


@dataclass(frozen=True)
class Trajectory:
    generation: int
    role: str
    context: ContextKey
    certificate_expiry_ns: Optional[int] = None


class ExecutionPhase(str, Enum):
    NONE = "NoExecution"
    TRACKING = "Tracking"
    RECOVERABLE_BRAKING = "RecoverableBraking"
    COMMITTED_STOPPING = "CommittedStopping"
    STOPPED = "Stopped"
    RELEASED = "Released"


@dataclass
class ExecutionAuthorityShadow:
    phase: ExecutionPhase = ExecutionPhase.NONE
    active: Optional[Trajectory] = None
    staged: Optional[Trajectory] = None
    context: Optional[ContextKey] = None
    fenced_generation: Optional[int] = None

    def stage(self, trajectory: Trajectory) -> bool:
        if self.phase in (ExecutionPhase.RELEASED, ExecutionPhase.COMMITTED_STOPPING) or (self.context and trajectory.context != self.context):
            return False
        self.staged = trajectory
        return True

    def activate(self) -> bool:
        if not self.staged or self.phase in (ExecutionPhase.RELEASED, ExecutionPhase.COMMITTED_STOPPING):
            return False
        if self.context and self.staged.context != self.context:
            self.staged = None
            return False
        self.active, self.staged = self.staged, None
        self.phase = ExecutionPhase.TRACKING
        return True

    def world(self, context: ContextKey) -> None:
        self.context = context
        if self.staged and self.staged.context != context:
            self.staged = None
        # World refresh cannot resurrect RELEASED, nor extend an old certificate.

    def recoverable_brake(self) -> bool:
        if self.phase != ExecutionPhase.TRACKING or self.active is None:
            return False
        self.phase = ExecutionPhase.RECOVERABLE_BRAKING
        return True

    def recover_from_braking(self, trajectory: Trajectory, *, continuity_proven: bool,
                             now_ns: int) -> bool:
        # `continuity_proven` is an event certificate, never a stored latch.
        if self.phase != ExecutionPhase.RECOVERABLE_BRAKING or not continuity_proven:
            return False
        if self.context != trajectory.context or (trajectory.certificate_expiry_ns is not None and
                                                 now_ns >= trajectory.certificate_expiry_ns):
            return False
        self.active = trajectory
        self.staged = None
        self.phase = ExecutionPhase.TRACKING
        return True

    def safety_stop(self, trajectory: Optional[Trajectory] = None) -> None:
        if self.phase == ExecutionPhase.RELEASED:
            return
        self.active = trajectory or self.active
        self.staged = None
        self.phase = ExecutionPhase.COMMITTED_STOPPING

    def measured_stop(self) -> None:
        if self.phase == ExecutionPhase.COMMITTED_STOPPING:
            self.phase = ExecutionPhase.STOPPED

    def release(self) -> None:
        self.fenced_generation = self.active.generation if self.active else self.fenced_generation
        self.active = None
        self.staged = None
        self.phase = ExecutionPhase.RELEASED

    def restart_new_session(self, context: ContextKey) -> None:
        self.active = None
        self.staged = None
        self.context = context
        self.phase = ExecutionPhase.NONE
        self.fenced_generation = None

    def can_publish(self, generation: int, now_ns: int) -> bool:
        if self.phase not in (ExecutionPhase.TRACKING, ExecutionPhase.RECOVERABLE_BRAKING):
            return False
        a = self.active
        return bool(a and a.generation == generation and
                    (a.certificate_expiry_ns is None or now_ns < a.certificate_expiry_ns))


@dataclass(frozen=True)
class ProtocolWitness:
    stamp_ns: int
    value: int
    quality: Quality = Quality.OBSERVED


@dataclass
class HoldTransfer:
    request_stamp_ns: int
    request_id: int
    ack: Optional[ProtocolWitness] = None
    completion: Optional[ProtocolWitness] = None
    hold_status: Optional[ProtocolWitness] = None
    takeover: Optional[str] = None
    prior_status_source_stamp_ns: Optional[int] = None
    retry_deadline_ns: Optional[int] = None

    def confirmed(self) -> bool:
        # A fresh AUTO_LOITER witness after the request is required. ACK and
        # completion are independent, and neither alone grants confirmation.
        return bool(self.hold_status and self.hold_status.stamp_ns > 0 and
                    (self.prior_status_source_stamp_ns is None or
                     self.hold_status.stamp_ns > self.prior_status_source_stamp_ns) and
                    self.hold_status.value == 4 and not self.takeover)

    def retry_required(self) -> bool:
        return not self.confirmed() and self.takeover is None

    def retry_due(self, now_steady_ns: int) -> bool:
        return self.retry_required() and (self.retry_deadline_ns is None or
                                          now_steady_ns >= self.retry_deadline_ns)


@dataclass(frozen=True)
class LocalBoundaryWitness:
    localization_epoch: int
    state_source_stamp_ns: int
    state_receive_stamp_ns: int
    px4_source_stamp_ns: int
    px4_receive_stamp_ns: int
    frame_generation: int
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    translation_ned: tuple[float, float, float]
    heading_ned: float
    health_state: str
    health_source_stamp_ns: int


@dataclass
class Px4AuthorityShadow:
    local: Optional[LocalBoundaryWitness] = None
    last_reference_key: Optional[tuple[int, ...]] = None
    lease_deadline_ns: Optional[int] = None
    hold: Optional[HoldTransfer] = None
    status: Optional[ProtocolWitness] = None

    def observe_local(self, witness: LocalBoundaryWitness) -> None:
        if self.local is None or witness.state_source_stamp_ns > self.local.state_source_stamp_ns:
            self.local = witness

    def localization_reset(self) -> None:
        self.local = None
        self.last_reference_key = None
        self.lease_deadline_ns = None

    def reference(self, key: tuple[int, ...], receive_ns: int, lease_ns: int = 100_000_000) -> None:
        if self.hold is not None:
            return
        self.last_reference_key = key
        self.lease_deadline_ns = receive_ns + lease_ns

    def lease_expired(self, now_ns: int) -> bool:
        return self.lease_deadline_ns is not None and now_ns >= self.lease_deadline_ns

    def request_hold(self, request_id: int, now_ns: int) -> None:
        self.hold = HoldTransfer(now_ns, request_id,
                                 prior_status_source_stamp_ns=self.status.stamp_ns if self.status else None)
        self.last_reference_key = None
        self.lease_deadline_ns = None

    def ack(self, stamp_ns: int, result: int) -> None:
        if self.hold:
            self.hold.ack = ProtocolWitness(stamp_ns, result)

    def completed(self, stamp_ns: int, result: int) -> None:
        if self.hold:
            self.hold.completion = ProtocolWitness(stamp_ns, result)

    def vehicle_status(self, stamp_ns: int, nav_state: int, receive_ns: int) -> None:
        if self.status and stamp_ns < self.status.stamp_ns:
            return
        self.status = ProtocolWitness(stamp_ns, nav_state)
        if self.hold and self.hold.hold_status and nav_state != 4 and receive_ns >= self.hold.request_stamp_ns:
            self.hold.hold_status = None
            self.hold.takeover = "NON_HOLD_STATUS_UNKNOWN_CAUSE"
        if self.hold and nav_state == 4 and receive_ns >= self.hold.request_stamp_ns:
            if stamp_ns > 0 and (self.hold.prior_status_source_stamp_ns is None or
                                 stamp_ns > self.hold.prior_status_source_stamp_ns):
                self.hold.hold_status = ProtocolWitness(stamp_ns, nav_state)

    def takeover(self, reason: str) -> None:
        if self.hold:
            self.hold.takeover = reason


@dataclass
class ShadowReducer:
    mission: MissionProgressShadow = field(default_factory=MissionProgressShadow)
    execution: ExecutionAuthorityShadow = field(default_factory=ExecutionAuthorityShadow)
    px4: Px4AuthorityShadow = field(default_factory=Px4AuthorityShadow)
    evidence: Quality = Quality.OBSERVED

    def trace_gap(self) -> None:
        self.evidence = Quality.TRACE_GAP

    def compare(self, predicted: Optional[bool], observed: bool) -> Verdict:
        if self.evidence == Quality.TRACE_GAP or predicted is None:
            return Verdict.INSUFFICIENT
        return Verdict.MATCH if predicted == observed else Verdict.DIVERGENCE
