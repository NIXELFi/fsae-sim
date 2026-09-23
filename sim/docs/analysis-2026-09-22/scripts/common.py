import pandas as pd, glob, numpy as np
from scipy.signal import butter, filtfilt, medfilt
from scipy.ndimage import uniform_filter1d as uf
from scipy.interpolate import PchipInterpolator
L=1.53; TF=1.207; TR=1.194
ROAD=[0.0000,0.9486,1.9018,2.8643,3.8407,4.8355,5.8534,6.8987,7.9756,9.0882,10.2396,11.4326,12.6689,13.9491,15.2722,16.6359,18.0360,19.4671,20.9221,22.3935,23.8733,25.3539,26.8282,28.2906,29.7370,31.1647,32.5728,33.9618,35.3334,36.6907,38.0376,39.3790,40.7206,42.0688,43.4308,44.8146,46.2288]
_p=PchipInterpolator(np.arange(37)*5.0,ROAD)
def road(rim): return np.sign(rim)*_p(np.minimum(np.abs(rim),180))
def lp(x,fc=2,fs=100):
    b,a=butter(2,fc/(fs/2)); return filtfilt(b,a,x)
def sd(x,W): return np.sqrt(np.maximum(uf(x*x,W)-uf(x,W)**2,0))
def load(f,fc=2):
    d=pd.read_csv(f); d=d[d.IMU_Z_ACCEL>0.3].reset_index(drop=True)
    o=pd.DataFrame({'t':d.TS.values})
    o['ay']=lp(d.IMU_X_ACCEL.values,fc); o['ax']=lp(d.IMU_Y_ACCEL.values,fc)
    o['ey']=-lp(d.ENG_IMU_Y.values,fc)
    o['r']=np.radians(lp(d.IMU_Z_GYRO.values,fc)/1000)
    o['rim']=lp(d.STEERING.values,fc)
    o['v']=lp(medfilt(d.GP_SPEED.values,21),1)/3.6
    o['vraw']=d.GP_SPEED.values/3.6
    for c in ['FLSHOCK','FRSHOCK','RLSHOCK','RRSHOCK']: o[c]=lp(d[c].values,fc)
    o['ecu_on']=(d.ENGINE_SPEED.values>0)
    return o
FILES=sorted(glob.glob('logs/4-1*.csv'))
def name(f): return f.split('csv__')[-1].replace('.csv','') if 'csv__' in f else f
