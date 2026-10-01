# Replicates navigation_common::secondsToRosTime (time.hpp:132-148, truncation of double fraction)
# vs secondsToNanoseconds (time.hpp:71-84, round(long double product)).
# Exact rational product is used as a stand-in for the long-double product (ulp <= 0.125 ns at 1.7e18 ns).
from fractions import Fraction
import math
def to_ros_time_trunc(s):
    whole=int(s); frac=s-float(whole); ns=int(frac*1e9); return whole*1_000_000_000+ns
def to_ns_round(s):
    return round(Fraction(s)*10**9)
for s in [12.345678901, 100.000000007, 1727000000.123456789, 0.1+0.2, 0.000000001]:
    print(f"{s!r:>25} secondsToRosTime={to_ros_time_trunc(s)} secondsToNanoseconds={to_ns_round(s)} diff_ns={to_ns_round(s)-to_ros_time_trunc(s)}")
