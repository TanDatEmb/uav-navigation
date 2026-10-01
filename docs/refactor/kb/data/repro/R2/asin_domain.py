# Reproduce: backup mapIntervalToInf asin argument leaves [-1,1] for ts at the validated interval edge.
import math, random
random.seed(1)
bad=0; n=100000; ex=None
for _ in range(n):
    t0=random.uniform(0,20); te=t0+random.uniform(0.01,5)
    for ts in (t0,te):
        r=(ts-(t0+te)/2)/((te-t0)/2)
        if abs(r)>1.0:
            bad+=1
            if ex is None: ex=(t0,te,ts,r)
print("edge samples with |arg|>1 (asin->NaN):",bad,"of",2*n); print("example",repr(ex))
