# Check yaw_traj_opt.cpp:82-84 stopping displacement = argmin_D jerk energy of quintic (v0,a0)->(D,0,0).
import numpy as np
def coeffs(v0,a0,D,T):
    M=[];b=[]
    def row(k,tt):
        r=[0.0]*6
        for i in range(k,6):
            f=1.0
            for j in range(k): f*= (i-j)
            r[i]=f*tt**(i-k)
        return r
    M=[row(0,0),row(1,0),row(2,0),row(0,T),row(1,T),row(2,T)]; b=[0,v0,a0,D,0,0]
    return np.linalg.solve(np.array(M),np.array(b))
def J(c,T):
    ts=np.linspace(0,T,20001); jer=6*c[3]+24*c[4]*ts+60*c[5]*ts**2
    return np.trapezoid(jer**2,ts)
for v0,a0,T in [(1.0,0.0,2.0),(0.5,0.8,1.5),(-1.2,0.3,3.0)]:
    Ds=np.linspace(-5,5,4001); best=Ds[np.argmin([J(coeffs(v0,a0,D,T),T) for D in Ds])]
    print(v0,a0,T,"argmin D=%.4f formula=%.4f"%(best,0.5*v0*T+a0*T*T/12))
