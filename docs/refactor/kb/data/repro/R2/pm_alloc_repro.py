# Transliteration of geometry_utils.h:53-183 simplePMTimeAllocator (Case 2 / Case 3) to check kinematics.
import math
def solve_q(a,b,c):
    d=b*b-4*a*c
    sc=max(1.0,abs(b*b),abs(4*a*c))
    if d<0 and d>-1e-12*sc: d=0.0
    if d<0: return float('nan')
    return (-b+math.sqrt(d))/(2*a)
def pm(amax,vmax,v0,D,s):
    t_to_vmax=vmax/amax; dis_to_vmax=0.5*amax*t_to_vmax**2
    t_to_v0=v0/amax; dis_to_v0=0.5*amax*t_to_v0**2
    dec_time=(vmax-v0)/amax; dec_dis=0.5*(vmax+v0)*dec_time
    if D<=dis_to_v0:
        disc=v0*v0-2*amax*s; t=(v0-math.sqrt(max(disc,0)))/amax; return t,v0-amax*t,'case1'
    if D<=dis_to_vmax+dec_dis:
        a=2*amax; b=-(amax-v0); c=-(v0*v0/amax+2*D)
        t_acc=solve_q(a,b,c); dis_acc=0.5*amax*t_acc**2; vpk=amax*t_acc
        if s<=dis_acc:
            t=math.sqrt(2*s/amax); return t,amax*t,'case2a'
        rem=s-dis_acc; t2=solve_q(-amax,2*vpk,-2*rem); return t_acc+t2, vpk-t2*amax,'case2b'
    if s<dis_to_vmax:
        t=math.sqrt(2*s/amax); return t,amax*t,'case3.1'
    if s<D-dec_dis:
        return t_to_vmax+(s-dis_to_vmax)/vmax, vmax,'case3.2'
    const=D-dec_dis-dis_to_vmax; rem=s-dis_to_vmax-const
    td=solve_q(-amax,2*vmax,-2*rem); return t_to_vmax+const/vmax+td, vmax-td*amax,'case3.3'
for (amax,vmax,v0,D) in [(5,10,0,10),(3,2,0,5.608921464952064),(5,5,0,30),(5,5,3,30),(2,5,4,3)]:
    t,v,c=pm(amax,vmax,v0,D,D)
    print(f"a={amax} vmax={vmax} v0={v0} D={D}: {c} total={t:.4f}s terminal_v={v:.4f} m/s")
print("analytic rest-to-rest triangle a=5,D=10: total=%.4f terminal_v=0"%(2*math.sqrt(10/5)))
t,v,c=pm(5,5,3,30,0.5); print("a=5 vmax=5 v0=3, s=0.5m:",c,"t=%.4f v=%.4f (v0=3 ignored: exact t=%.4f)"%(t,v,(-3+math.sqrt(9+2*5*0.5))/5))
