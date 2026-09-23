"""Audit-only classifier: it refuses to infer adapter lease from producer silence."""


def classify(events, lease_ns=100_000_000):
    by_kind={e['kind']:e for e in events}
    needed=('suspend','last_adapter_receive','recertified','resume')
    if any(k not in by_kind for k in needed):
        return 'INSUFFICIENT_TRACE'
    suspend,received,recert,resume=(by_kind[k] for k in needed)
    ids={e.get('bundle') for e in (suspend,received,recert,resume)}
    if len(ids)!=1 or None in ids:
        return 'IDENTITY_CONFLICT'
    if any(e.get('steady_ns') is None for e in (suspend,received,recert,resume)):
        return 'INSUFFICIENT_TRACE'
    if recert['steady_ns']>resume['steady_ns'] or received['steady_ns']>suspend['steady_ns']:
        return 'ORDER_CONFLICT'
    handover=by_kind.get('adapter_handover')
    if handover and handover['steady_ns']<=resume['steady_ns']:
        return 'RESUME_AFTER_OBSERVED_HANDOVER'
    if resume['steady_ns']-received['steady_ns']>=lease_ns:
        return 'RESUME_AFTER_LEASE_WINDOW_NO_HANDOVER_WITNESS'
    return 'RECERTIFIED_AND_RESUMED_WITHIN_LEASE_WINDOW'
