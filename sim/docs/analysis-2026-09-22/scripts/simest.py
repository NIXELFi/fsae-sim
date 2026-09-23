from common import *
s=pd.read_csv('simfit/sim_drive.csv')
def est(W=40, ayth=0.05, rth=0.04, sth=3, fc=2, extra=None):
    rows=[]
    for run,d in s.groupby((s.t//100).astype(int)):
        ay=lp(d.ay_g.values,fc); r=lp(d.r_rad_s.values,fc); dl=lp(d.steer_deg.values,fc); v=lp(d.v.values,fc)
        ss=(sd(ay,W)<ayth)&(sd(r,W)<rth)&(sd(dl*5.27,W)<sth)&(np.abs(ay)>0.2)&(np.abs(r)>0.15)&(np.sign(ay)==np.sign(r))&(np.sign(dl)==np.sign(ay))
        idx=np.flatnonzero(ss)
        for k in idx[::5]:
            a=abs(ay[k]); R=a*9.81/r[k]**2
            rows.append(dict(ay=a,R=R,Rt=v[k]/abs(r[k]),delta=abs(dl[k]),side=np.sign(ay[k])))
    df=pd.DataFrame(rows); df=df[df.ay<1.0]
    y=df.delta-np.degrees(1.53/df.R); K=np.linalg.lstsq(np.c_[df.ay,df.side],y,rcond=None)[0][0]
    yt=df.delta-np.degrees(1.53/df.Rt); Kt=np.linalg.lstsq(np.c_[df.ay,df.side],yt,rcond=None)[0][0]
    return len(df), K, Kt, np.median(df.R/df.Rt)
if __name__=='__main__':
    for W,a,r_,st in [(40,.05,.04,3),(80,.03,.02,2),(100,.02,.015,1.5),(150,.02,.015,1.5)]:
        print('W',W,'n %d K(gyro R) %.2f K(true R) %.2f  R_est/R_true %.3f'%est(W,a,r_,st))
