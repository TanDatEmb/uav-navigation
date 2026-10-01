import math
def near(v,f):
    o=sorted(v); return o[min(len(o)-1,max(0,round((len(o)-1)*f)))]
def lin(v,f):
    v=sorted(x for x in v if math.isfinite(x)); i=(len(v)-1)*f; lo,hi=math.floor(i),math.ceil(i)
    return v[lo] if lo==hi else v[lo]+(v[hi]-v[lo])*(i-lo)
# 11 samples: one outlier tail
v=[0.1]*10+[5.0]
print('n=11 p95 near',near(v,.95),'lin',lin(v,.95))
v=[0.1]*30+[5.0]
print('n=31 p95 near',near(v,.95),'lin',lin(v,.95))
print('n=2 p50 near',near([1,2],.5),' n=4 p50 near',near([1,2,3,4],.5))
print('nan sort',near([float("nan"),3,1,2],.95), near([3,float("nan"),1,2],.95))
