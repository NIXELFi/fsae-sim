from common import *
import json, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from scipy import stats
TILT=1.016
MRF,MRR=1.143,1.054
roll=pd.DataFrame(json.load(open('roll_results.json')))
main=roll[roll.ay99>0.5]
out={}
for ax_ in ['front','rear']:
    x=np.abs(main[ax_].values)*TILT; m=x.mean(); se=x.std(ddof=1)/np.sqrt(len(x)); t=stats.t.ppf(.975,len(x)-1)
    out[ax_]=(m,m-t*se,m+t*se,x.min(),x.max(),len(x))
    print('%s spring roll %.3f deg/g  95%% CI [%.3f, %.3f]  file range %.3f-%.3f  n files %d'%((ax_,)+out[ax_]))
avg=(np.abs(main.front)+np.abs(main.rear)).values/2*TILT
print('axle-mean %.3f CI +-%.3f'%(avg.mean(), stats.t.ppf(.975,len(avg)-1)*avg.std(ddof=1)/np.sqrt(len(avg))))
# plot pooled
fig,axs=plt.subplots(1,2,figsize=(12,4.6),sharey=True)
for f in FILES:
    o=load(f,fc=3)
    if o.ay.abs().max()<0.5 or np.percentile(np.abs(o.ay),99)<0.5: continue
    dF=(o.FLSHOCK-o.FRSHOCK).values; dR=(o.RLSHOCK-o.RRSHOCK).values
    rf=np.degrees(np.arctan(dF/MRF/(TF*1000))); rr=-np.degrees(np.arctan(dR/MRR/(TR*1000)))
    ss=(sd(o.ay.values,30)<0.04)&(sd(rf,30)<0.05)
    idx=np.flatnonzero(ss)[::25]; ay=o.ay.values[idx]/TILT
    for a,x in [(axs[0],rf[idx]),(axs[1],rr[idx])]:
        c=np.polyfit(ay,x,1); a.scatter(ay,x-c[1],s=2,alpha=.25,color='#4a78b5')
xx=np.linspace(-1.4,1.4,10)
for a,k,lab in [(axs[0],out['front'][0],'front'),(axs[1],out['rear'][0],'rear')]:
    a.plot(xx,k*xx,color='#c0392b',lw=2,label='logs, springs only: %.2f deg/g'%k)
    simk=0.402 if lab=='front' else 0.373
    a.plot(xx,simk*xx,color='#2c3e50',lw=2,ls='--',label='sim DT, springs only: %.2f deg/g'%simk)
    a.plot(xx,0.668*xx,color='#7f8c8d',lw=1.5,ls=':',label='sim DT, body incl. tyres: 0.67 deg/g')
    a.set_title(f'{lab} axle: roll from shock pots (quasi-steady, offsets removed)'); a.set_xlabel('lateral acceleration (g)'); a.grid(alpha=.3); a.legend(fontsize=8,loc='upper left')
axs[0].set_ylabel('suspension roll (deg)')
plt.tight_layout(); plt.savefig('png/roll_fit.png',dpi=110)
# understeer plot
df=pd.read_csv('us_points.csv'); df['us']=df.delta-df.LR
sw=pd.read_csv('simfit/sweep.csv',comment='#'); sw=sw[sw.model=='dt']; sw['us']=sw.delta_deg-np.degrees(1.53/sw.R)
fig,a=plt.subplots(figsize=(8,5))
a.scatter(df.ay,df.us,s=4,alpha=.3,color='#4a78b5',label='logs: quasi-steady points (R = ay/r^2 from yaw gyro)')
b=pd.cut(df.ay,np.arange(0.2,1.45,0.1)); g=df.groupby(b,observed=True).us.agg(['median','size']); cen=[i.mid for i in g.index]
a.plot(cen,g['median'],'o-',color='#c0392b',label='logs: bin median')
for v,ls in [(8,'--'),(10,'-.'),(12,':')]:
    d=sw[(sw.v_set==v)&(sw.ay_g<=1.3)]; a.plot(d.ay_g,d.us,ls=ls,color='#2c3e50',label=f'sim DT steady state, {v} m/s')
a.set_xlabel('lateral acceleration (g)'); a.set_ylabel('road steer - L/R (deg)'); a.set_xlim(0.15,1.45); a.set_ylim(-4,8); a.grid(alpha=.3); a.legend(fontsize=8)
a.set_title('Understeer: steer beyond Ackermann vs lateral g')
plt.tight_layout(); plt.savefig('png/understeer.png',dpi=110)
print(g.round(2))
