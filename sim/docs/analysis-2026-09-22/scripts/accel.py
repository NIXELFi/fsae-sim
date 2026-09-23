import pandas as pd, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def load(f):
    d=pd.read_csv(f,skiprows=[0,2],low_memory=False).apply(pd.to_numeric,errors='coerce')
    d['t']=d['File Time']; return d
runs=[]
for f in ['logs/SDM26 (5.3.1) Accel.csv','logs/SDM26 (5.3.2) Accel.csv']:
    d=load(f); t=d.t.values; tps=d['TPS (Main)'].values; v=d['GP Speed 1 - TransSpeed'].values
    hi=tps>70; e=np.flatnonzero(np.diff(hi.astype(int)))
    for i in range(len(e)-1):
        a,b=e[i]+1,e[i+1]
        if hi[a] and t[b]-t[a]>2 and np.nanmax(v[a:b])>50: runs.append((f,a,b))
print(len(runs))
for f,a,b in runs:
    d=load(f) if 'dcache' not in globals() or dcache[0]!=f else dcache[1]
    globals()['dcache']=(f,d)
    t=d.t.values; v=d['GP Speed 1 - TransSpeed'].values/3.6; rpm=d['Engine Speed'].values; g=d['Gear'].values
    # start: last sample before a where v<0.5 up to... find first index >= a-3000 where v>0.3
    lo=max(a-5000,0); i0=lo+np.flatnonzero(v[lo:b]>0.3)[0]
    # refine: movement start = when v first exceeds 0.3 m/s
    tt=t[i0:b+3000]-t[i0]; vv=v[i0:b+3000]
    # update rate of speed channel
    ch=np.flatnonzero(np.diff(vv)!=0); upd=np.median(np.diff(tt[ch])) if len(ch)>3 else np.nan
    x=np.concatenate([[0],np.cumsum(0.5*(vv[1:]+vv[:-1])*np.diff(tt))])
    k=np.searchsorted(x,75)
    res=dict(file=f.split('/')[-1],t_start=t[i0],upd=upd,t75=tt[k] if k<len(tt) else np.nan,v75=vv[k]*3.6 if k<len(tt) else np.nan,vmax=np.nanmax(vv)*3.6,rpm_launch=np.nanmax(rpm[max(i0-500,0):i0]))
    # shift times: gear changes
    gg=g[i0:b+3000]; sh=np.flatnonzero(np.diff(gg)!=0)
    res['shifts']=[(round(tt[s],2),int(gg[s]),int(gg[s+1]),int(rpm[i0+s])) for s in sh]
    # rpm/kph per gear during run
    rr=rpm[i0:b+3000]; kk=vv*3.6
    res['rpm_per_kph']={int(G):round(np.nanmedian((rr/kk)[(gg==G)&(kk>15)]),1) for G in np.unique(gg) if ((gg==G)&(kk>15)).sum()>50}
    print(res)
    runs_out=globals().setdefault('R',[]); runs_out.append((res,tt,vv,rr,gg,x))
import pickle; pickle.dump([(r[0],r[1],r[2],r[3],r[4],r[5]) for r in R],open('accel_runs.pkl','wb'))
