# Repro: heading-rebind role partition end (suffix_duration = committed - start_tt)
# vs candidate.position.getTotalDuration() (sum of re-sliced pieces), which the
# validator compares with exact '=='. Models Trajectory::getPartialTrajectoryByTime
# (first piece d1 - t0, interior pieces unchanged, last piece local_end = end_TT - cum_start).
import random
random.seed(1)
N=20000; mismatch=0
for _ in range(N):
    n=random.randint(2,8)
    d=[random.uniform(0.05,1.5) for _ in range(n)]
    committed=0.0
    for x in d: committed+=x
    cum=[0.0]
    for x in d: cum.append(cum[-1]+x)
    start_tt=random.uniform(1e-3, d[0]-1e-3)       # inside first piece
    suffix=committed-start_tt
    pieces=[d[0]-start_tt]+d[1:-1]+[min(max(committed-cum[n-1],0.0),d[-1])]  # last piece local end (clamped as locateSliceEnd)
    total=0.0
    for x in pieces: total+=x
    end=min(suffix, committed-start_tt)              # role.end_tt - start_tt for final role
    if end!=total: mismatch+=1
print(f"{mismatch}/{N} rebinds: final role end_tt != position.getTotalDuration() (exact compare)")
