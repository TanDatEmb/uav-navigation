import math
import numpy as np
lg=lambda p: math.log(p/(1-p))
# production planner.yaml rog_map/raycasting values
p=dict(p_min=0.12,p_miss=0.49,p_free=0.499,p_occ=0.85,p_hit=0.9,p_max=0.98)
l={k.replace('p_','l_'):lg(np.float32(v)) for k,v in p.items()}
print({k:round(v,4) for k,v in l.items()})
unk=(l['l_free']+l['l_occ'])/2
def misses_to_free(v):
    n=0
    while not v< l['l_free']:
        v=max(v+l['l_miss'],l['l_min']); n+=1
    return n
print('initial unknown value',round(unk,4),'misses->KNOWN_FREE:',misses_to_free(unk))
print('slide-reset value 0.0 misses->KNOWN_FREE:',misses_to_free(0.0))
print('hits from unk->OCC:',math.ceil((l['l_occ']-unk)/l['l_hit']) , 'from 0:', math.ceil(l['l_occ']/l['l_hit']))
# virtual ceiling clip formula (prob_map.cpp:880-882)
o=np.array([0.,0.,1.]); pt=np.array([10.,0.,5.]); ceil=3.0
dz=pt[2]-o[2]; pc=ceil-o[2]
d=pt-o
code=o+d/np.linalg.norm(d)*pc/dz
correct=o+d*(pc/dz)
print('code clip point',code.round(3),'len',round(np.linalg.norm(code-o),3))
print('correct clip  ',correct.round(3),'len',round(np.linalg.norm(correct-o),3))
