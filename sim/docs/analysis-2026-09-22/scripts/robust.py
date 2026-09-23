from common import *
from scipy.stats import theilslopes
rng=np.random.default_rng(1)
def huber(X,y,c=1.345,it=50):
    b=np.linalg.lstsq(X,y,rcond=None)[0]
    for _ in range(it):
        r=y-X@b; s=np.median(np.abs(r-np.median(r)))/0.6745+1e-9; u=np.abs(r)/(c*s); w=np.where(u<=1,1,1/u)
        b=np.linalg.lstsq(X*np.sqrt(w)[:,None],y*np.sqrt(w),rcond=None)[0]
    return b
def seg_points(df):
    return df.groupby('seg').agg(ay=('ay','median'),us=('us','median'),LR=('LR','median'),delta=('delta','median'),side=('side','first'),n=('ay','size')).reset_index()
def estimates(df,nb=1000):
    sp=seg_points(df)
    X=np.c_[sp.ay,np.ones(len(sp)),sp.side]; kh=huber(X,sp.us.values)
    ts=theilslopes(sp.us,sp.ay)[0]
    Xf=np.c_[sp.LR,sp.ay,np.ones(len(sp)),sp.side]; kf=huber(Xf,sp.delta.values)
    bs=[];bf=[];bt=[]
    for _ in range(nb):
        s=sp.iloc[rng.integers(0,len(sp),len(sp))]
        bs.append(huber(np.c_[s.ay,np.ones(len(s)),s.side],s.us.values)[0]); bt.append(theilslopes(s.us,s.ay)[0])
        bf.append(huber(np.c_[s.LR,s.ay,np.ones(len(s)),s.side],s.delta.values)[:2])
    bf=np.array(bf)
    return dict(nseg=len(sp),K_huber=kh[0],K_huber_ci=np.percentile(bs,[2.5,97.5]).tolist(),int_huber=kh[1],K_theilsen=ts,K_ts_ci=np.percentile(bt,[2.5,97.5]).tolist(),
                c_free=kf[0],c_free_ci=np.percentile(bf[:,0],[2.5,97.5]).tolist(),K_free=kf[1],K_free_ci=np.percentile(bf[:,1],[2.5,97.5]).tolist())
if __name__=='__main__':
    import json
    real=pd.read_csv('us_points.csv'); real['us']=real.delta-real.LR
    r1=estimates(real); r2=estimates(real[real.ay<=1.0])
    # sim drive through identical pipeline
    s=pd.read_csv('simfit/sim_drive.csv'); rows=[]
    for run,d in s.groupby((s.t//100).astype(int)):
        ay=lp(d.ay_g.values); r=lp(d.r_rad_s.values); dl=lp(d.steer_deg.values)
        ss=(sd(ay,40)<0.05)&(sd(r,40)<0.04)&(sd(dl*5.27,40)<3)&(np.abs(ay)>0.2)&(np.abs(r)>0.15)&(np.sign(ay)==np.sign(r))&(np.sign(dl)==np.sign(ay))
        idx=np.flatnonzero(ss); seg=np.cumsum(np.r_[1,np.diff(idx)>1]) if len(idx) else []
        for i,k in list(enumerate(idx))[::5]:
            a=abs(ay[k]); R=a*9.81/r[k]**2
            rows.append(dict(seg=f'{run}:{seg[i]}',ay=a,R=R,LR=np.degrees(1.53/R),delta=abs(dl[k]),side=np.sign(ay[k])))
    sim=pd.DataFrame(rows); sim['us']=sim.delta-sim.LR
    r3=estimates(sim,300); r4=estimates(sim[sim.ay<=1.0],300)
    for lab,r in [('real all',r1),('real ay<=1',r2),('sim-drive all',r3),('sim-drive ay<=1',r4)]:
        print(lab, {k:(np.round(v,3) if not isinstance(v,list) else np.round(v,2).tolist()) for k,v in r.items()})
    json.dump(dict(real_all=r1,real_le1=r2,sim_all=r3,sim_le1=r4),open('us_robust.json','w'),indent=1,default=float)
