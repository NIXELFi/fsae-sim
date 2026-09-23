"""Fit the 2026 CFD ride-height aero map to planes (AeroRideMap in vehicle.rs).

Reads the team workbook export ($SDM26_TEAM_DATA, default
~/sdm26-assetto-corsa/data/sdm26_team_data.json), aero_maps.bw_2026_lbf:
'Ride Height Data (BW)', a 5 x 5 grid [rear RH][front RH], -1..+1 in.
Drops the (front -1, rear +1) cell (front wing in the ground, flagged bad in
the sheet) and least-squares fits front DF, rear DF and drag, as planes and
as quadratics, printing coefficients and rms residual (lbf).
"""
import os
import json, numpy as np
d=json.load(open(os.path.expanduser(os.environ.get('SDM26_TEAM_DATA','~/sdm26-assetto-corsa/data/sdm26_team_data.json'))));am=d['aero_maps'];bw=am['bw_2026_lbf']
rows=bw['rear_rh_rows']; cols=am['ride_height_delta_in']
tot=np.array(bw['total_df']); pf=np.array(bw['pct_front'])/100; dr=np.array(bw['total_drag'])
F=tot*pf; R=tot*(1-pf)
X=[];Y={'F':[],'R':[],'D':[]}
for i,r in enumerate(rows):
  for j,f in enumerate(cols):
    if f==-1 and r==1: continue
    X.append([1,f,r,f*f,f*r,r*r]); Y['F'].append(F[i,j]); Y['R'].append(R[i,j]); Y['D'].append(dr[i,j])
X=np.array(X)
for deg,cols_ in [('lin',3),('quad',6)]:
  print(deg)
  for k in 'FRD':
    c,res,*_=np.linalg.lstsq(X[:,:cols_],np.array(Y[k]),rcond=None); pred=X[:,:cols_]@c
    rms=np.sqrt(np.mean((pred-np.array(Y[k]))**2)); print(' ',k,np.round(c,3),'rms',round(rms,2),'at0',round(c[0],2))
