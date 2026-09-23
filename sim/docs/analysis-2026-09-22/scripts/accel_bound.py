# Upper bound on 75 m: point mass, rear-traction limit with load transfer + aero, constant wheel power (no shifts).
import math
m=267; g=9.81; wb=1.53; h=0.2845; wdf=0.485; rho=1.162; cda=1.267; cla=3.132; ff=0.524; crr=0.015
def run(P, mu, extra_mass=0.0):
    v=0.0;x=0.0;t=0.0;dt=1e-3
    while x<75:
        q=0.5*rho*v*v
        # rear load with transfer a: Fzr = m g (1-wdf) + q cla (1-ff) + m a h / wb ; traction = mu Fzr
        # solve a = min(mu*Fzr(a), P/v) - drag
        drag=q*cda+crr*(m*g+q*cla)
        Fz0=m*g*(1-wdf)+q*cla*(1-ff)
        Ft=mu*Fz0/(1-mu*m*h/wb/ m * m / m) if False else None
        # traction: F = mu*(Fz0 + m*a*h/wb), a=(F-drag)/m -> F = mu*(Fz0 + (F-drag)*h/wb) -> F(1-mu h/wb)=mu(Fz0 - drag h/wb)
        F=mu*(Fz0-drag*h/wb)/(1-mu*h/wb)
        if v>0.1: F=min(F,P/v)
        a=(F-drag)/(m+extra_mass)
        v+=a*dt;x+=v*dt;t+=dt
    return t, v*3.6
for P in [46e3, 40e3, 55e3]:
  for mu in [1.3,1.5,1.7]:
    for em in [0,40]:
      t,vt=run(P,mu,em); print(f"P {P/1e3:.0f} kW mu {mu} +{em} kg: 75 m {t:.2f} s, trap {vt:.0f} km/h")
