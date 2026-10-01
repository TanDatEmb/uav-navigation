# Verifies minimumSnapStopPiece terminal conditions and the steady-cruise extrema
# claimed in backup_braking.hpp:164-167 (v_peak=v, a_peak=15v/(8T), j_peak=10v/(sqrt3 T^2)).
import numpy as np, random
def coeffs(p,v,a,j,T):
    t2=T*T;t3=t2*T;t4=t3*T;t5=t4*T
    c6=(-t2*j/12-T*a/2-v)/t5
    c5=(3*t2*j+16*T*a+30*v)/(10*t4)
    c4=(-3*t2*j-12*T*a-20*v)/(8*t3)
    return [0,c6,c5,c4,j/6,a/2,v,p]  # descending t^7..1
worst=0
for _ in range(2000):
    p,v,a,j=[random.uniform(-3,3) for _ in range(4)]; T=random.uniform(0.2,5)
    P=np.poly1d(coeffs(p,v,a,j,T))
    d1,d2,d3=P.deriv(1),P.deriv(2),P.deriv(3)
    worst=max(worst,abs(d1(T)),abs(d2(T)),abs(d3(T)), abs(d1(0)-v),abs(d2(0)-a),abs(d3(0)-j))
print("max terminal/initial residual:",worst)
v=2.0;T=1.3
P=np.poly1d(coeffs(0,v,0,0,T)); ts=np.linspace(0,T,200001)
print("vpeak",max(abs(P.deriv(1)(ts))),"expect",v)
print("apeak",max(abs(P.deriv(2)(ts))),"expect",15*v/(8*T))
print("jpeak",max(abs(P.deriv(3)(ts))),"expect",10*v/(np.sqrt(3)*T*T))
