from common import *
MRF,MRR=1.143,1.054   # spring(damper) travel per wheel travel, from hardpoints == Ride Roll 1.14 / sqrt(249.833/225)
res=[]
import json
pooled={'ay':[],'rf':[],'rr':[],'file':[]}
for f in FILES:
    o=load(f,fc=3)
    if o.ay.abs().max()<0.5: continue
    dF=(o.FLSHOCK-o.FRSHOCK).values; dR=(o.RLSHOCK-o.RRSHOCK).values
    rf=np.degrees(np.arctan(dF/MRF/(TF*1000))); rr=np.degrees(np.arctan(dR/MRR/(TR*1000)))
    W=30
    moving=np.abs(o.ay.values)>0.05
    ss=(sd(o.ay.values,W)<0.04)&(sd(rf,W)<0.05)&(np.abs(o.ay.values)<2.5)
    # exclude kerb/impact: high vertical content in shocks
    idx=np.flatnonzero(ss)[::5]
    ay=o.ay.values[idx]
    out={'file':name(f),'n':len(idx)}
    for lab,x in [('front',rf[idx]),('rear',rr[idx])]:
        A=np.c_[ay,np.ones_like(ay)]; co,resid,*_=np.linalg.lstsq(A,x,rcond=None)
        out[lab]=co[0]; out[lab+'_r']=np.corrcoef(ay,x)[0,1]
        # linearity: slope over |ay|<0.6 vs >0.6
        for rng,m in [('lo',np.abs(ay)<0.6),('hi',np.abs(ay)>=0.6)]:
            if m.sum()>20:
                out[lab+'_'+rng]=np.polyfit(ay[m],x[m],1)[0]
    out['ay99']=np.percentile(np.abs(ay),99)
    res.append(out)
    pooled['ay']+=list(ay); pooled['rf']+=list(rf[idx]-np.median(rf[idx][np.abs(ay)<0.05]) if (np.abs(ay)<0.05).sum()>10 else rf[idx]); pooled['rr']+=list(rr[idx]); pooled['file']+=[name(f)]*len(idx)
df=pd.DataFrame(res); pd.set_option('display.width',250)
print(df.round(3).to_string())
json.dump(res,open('roll_results.json','w'),indent=1)
