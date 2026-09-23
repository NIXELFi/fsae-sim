import json, numpy as np
from scipy.optimize import fsolve
d=json.load(open(r'C:/Users/nick5/sdm26-assetto-corsa/data/sdm26_team_data.json',encoding='utf-8'))['hardpoints']
def P(ax,k): return np.array(d[ax][k],float)
def rot(p,axis_a,axis_b,th):
    k=(axis_b-axis_a); k/=np.linalg.norm(k); v=p-axis_a
    return axis_a+v*np.cos(th)+np.cross(k,v)*np.sin(th)+k*np.dot(k,v)*(1-np.cos(th))
def solve(axle, pushrod_on):
    UF,UA,LF,LA=P(axle,'CHAS_UppFor'),P(axle,'CHAS_UppAft'),P(axle,'CHAS_LowFor'),P(axle,'CHAS_LowAft')
    UB,LB,WC=P(axle,'UPRI_UppPnt'),P(axle,'UPRI_LowPnt'),P(axle,'wheel_centre')
    PP=P(axle,'NSMA_PPAttPnt'); RP=P(axle,'CHAS_RocPiv'); RR=P(axle,'ROCK_RodPnt'); RC=P(axle,'ROCK_CoiPnt'); CA=P(axle,'CHAS_AttPnt')
    TC,TU=P(axle,'CHAS_TiePnt'),P(axle,'UPRI_TiePnt')
    # rocker axis: assume perpendicular to plane of pivot, rod pt, coil pt
    n=np.cross(RR-RP,RC-RP); n/=np.linalg.norm(n)
    Lpr=np.linalg.norm(RR-PP); Lub=None
    arm_is_upper = pushrod_on.startswith('upper')
    # upright rigid body defined by UB,LB,TU; wheel centre relative
    def state(thL):
        LBn=rot(LB,LF,LA,thL)
        # find thU such that |UB-LB| const, and tie rod length const -> upright rotation; simplify: solve thU for ball-joint distance
        dUL=np.linalg.norm(UB-LB)
        f=lambda th: np.linalg.norm(rot(UB,UF,UA,th[0])-LBn)-dUL
        thU=fsolve(f,[thL])[0]; UBn=rot(UB,UF,UA,thU)
        # upright orientation: rotate about UB-LB axis to hold tie rod length
        Ltie=np.linalg.norm(TU-TC)
        # build upright frame
        def place(phi):
            # rigid transform mapping (UB,LB) -> (UBn,LBn) plus spin phi about new axis
            a0=(LB-UB)/np.linalg.norm(LB-UB); a1=(LBn-UBn)/np.linalg.norm(LBn-UBn)
            v=np.cross(a0,a1); s=np.linalg.norm(v); c=np.dot(a0,a1)
            if s<1e-12: Rm=np.eye(3)
            else:
                vx=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]); Rm=np.eye(3)+vx+vx@vx*((1-c)/s**2)
            def T(p):
                q=UBn+Rm@(p-UB)
                return rot(q,UBn,LBn,phi)
            return T
        g=lambda ph: np.linalg.norm(place(ph[0])(TU)-TC)-Ltie
        phi=fsolve(g,[0.0])[0]; T=place(phi)
        WCn=T(WC)
        PPn = rot(PP,UF,UA,thU) if arm_is_upper else rot(PP,LF,LA,thL)
        # rocker angle to keep pushrod length
        h=lambda ps: np.linalg.norm(rot(RR,RP,RP+n,ps[0])-PPn)-Lpr
        ps=fsolve(h,[0.0])[0]; RCn=rot(RC,RP,RP+n,ps)
        return WCn[2], np.linalg.norm(RCn-CA)
    z0,c0=state(0.0)
    out=[]
    for th in np.radians([-1.0,-0.5,0.5,1.0]):
        z,c=state(th); out.append((z-z0, c-c0))
    out=np.array(out)
    mr=np.polyfit(out[:,0],out[:,1],1)[0]
    return mr, out
for axle in ['front','rear']:
    mr,out=solve(axle,d[axle]['pushrod_on'])
    print(axle,'d(spring length)/d(wheel centre z) = %.3f  (|MR| spring/wheel)'%mr, ' -> wheel/spring %.3f'%(1/abs(mr)))
