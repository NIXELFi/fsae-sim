from common import *
import json
rng=np.random.default_rng(0)
rows=[]
for f in FILES:
    o=load(f,fc=2)
    if o.ay.abs().max()<0.5: continue
    W=40
    moving=(o.v>3)
    straight=(np.abs(o.ay)<0.04)&(np.abs(o.r)<0.03)&(np.abs(o.rim)<30)&moving
    off=np.median(o.rim[straight]) if straight.sum()>50 else 0.0
    rim=o.rim.values-off
    dl=road(rim)
    ay=o.ay.values; r=o.r.values
    ss=(sd(ay,W)<0.05)&(sd(r,W)<0.04)&(sd(rim,W)<3)&(np.abs(ay)>0.2)&(np.abs(o.rim.values)<115)&(np.abs(r)>0.15)&(np.sign(ay)==-np.sign(r))&(np.sign(dl)==np.sign(ay))
    idx=np.flatnonzero(ss)
    # segment id for block bootstrap
    seg=np.cumsum(np.r_[1,np.diff(idx)>1])
    keep=idx[::5]; sg=seg[::5]
    a=np.abs(ay[keep]); R=a*9.81/r[keep]**2; d=np.abs(dl[keep])
    for i,k in enumerate(keep):
        rows.append(dict(file=name(f),seg=f'{name(f)}:{sg[i]}',ay=a[i],R=R[i],LR=np.degrees(L/R[i]),delta=d[i],v_gyro=a[i]*9.81/abs(r[k]),v_gp=o.v.values[k],side=np.sign(ay[k]),off=off))
df=pd.DataFrame(rows); df.to_csv('us_points.csv',index=False)
print('points',len(df),'segments',df.seg.nunique(),'files',df.file.nunique())
print(df.groupby('file').agg(n=('ay','size'),segs=('seg','nunique'),ay_med=('ay','median'),R_med=('R','median'),v_med=('v_gyro','median'),off=('off','first')).round(2))
def fit(d, free_c=True):
    X=[d.ay.values, d.side.values]
    if free_c: X=[d.LR.values]+X
    X=np.c_[tuple(X)]
    y=d.delta.values if free_c else d.delta.values-d.LR.values
    co=np.linalg.lstsq(X,y,rcond=None)[0]
    return co
def boot(d,free_c,n=1000):
    segs=d.seg.unique(); g={s:d[d.seg==s] for s in segs}; out=[]
    for _ in range(n):
        pick=rng.choice(segs,len(segs)); dd=pd.concat([g[s] for s in pick]); out.append(fit(dd,free_c))
    return np.array(out)
res={}
for lab,sub in [('all',df),('R<15m',df[df.R<15]),('ay 0.2-1.0',df[df.ay<1.0])]:
    for free_c in [False,True]:
        co=fit(sub,free_c); b=boot(sub,free_c,500)
        if free_c: txt='c(L/R scale) %.3f [%.3f,%.3f]  K %.2f [%.2f,%.2f] deg/g'%(co[0],*np.percentile(b[:,0],[2.5,97.5]),co[1],*np.percentile(b[:,1],[2.5,97.5]))
        else: txt='K %.2f [%.2f,%.2f] deg/g (L/R fixed at gyro scale)'%(co[0],*np.percentile(b[:,0],[2.5,97.5]))
        print(lab,'n',len(sub),'free_c' if free_c else 'fixed','->',txt)
        res[f'{lab}|{"free" if free_c else "fixed"}']=dict(n=len(sub),coef=co.tolist(),ci=np.percentile(b,[2.5,97.5],axis=0).tolist())
# per file K with fixed c
for fn,sub in df.groupby('file'):
    if sub.seg.nunique()>=4: print(fn, 'K fixed %.2f'%fit(sub,False)[0], ' free: c %.2f K %.2f'%tuple(fit(sub,True)[:2]))
json.dump(res,open('us_results.json','w'),indent=1)
