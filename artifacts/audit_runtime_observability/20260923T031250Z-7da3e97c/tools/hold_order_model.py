"""Audit-only epistemic Hold classifier; never grants product authority."""


def classify(events):
    """Classify recorded witness ordering; UNKNOWN is deliberate missing evidence."""
    if any(e.get('steady_ns') is None for e in events):
        return 'UNKNOWN_MISSING_CLOCK'
    events=sorted(events,key=lambda e:e['steady_ns'])
    request=None
    last_status_boot=-1
    outcome='UNKNOWN_NO_POST_REQUEST_AUTHORITY_WITNESS'
    for e in events:
        kind=e['kind']
        if kind=='request':
            request=e
            outcome='UNKNOWN_NO_POST_REQUEST_AUTHORITY_WITNESS'
        elif kind=='status':
            boot=e.get('boot_us')
            if boot is None or boot<=last_status_boot:
                continue
            last_status_boot=boot
            if request and e['steady_ns']>request['steady_ns'] and e.get('fresh'):
                if e.get('failsafe'):
                    outcome='EXTERNAL_FAILSAFE_TAKEOVER'
                elif e.get('operator'):
                    outcome='EXTERNAL_OPERATOR_TAKEOVER'
                elif e.get('nav_state')=='AUTO_LOITER':
                    outcome='OBSERVED_POST_REQUEST_LOITER_CAUSE_UNCONFIRMED'
        elif kind=='shutdown':
            return 'EXECUTOR_SHUTDOWN'
    return outcome
