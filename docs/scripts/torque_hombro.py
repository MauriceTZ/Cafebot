"""Estimación del torque en el hombro del brazo para alcanzar cada fila del árbol.
Supuestos: masas de servos/eslabones y largo de la garra aproximados (ver docs/brazo.md).
Correr: python3 docs/scripts/torque_hombro.py"""
import numpy as np, itertools
L1,L2,L3,L4=0.107,0.107,0.067,0.032
ZB=0.0671; BASE=0.36; g=9.81
mS=0.0572; mL=0.020; mG=0.060  # servo, link bracket, gripper+jaw extra
# ball center height: support top + 17mm
rows={'baja':0.390+0.017,'media':0.540+0.017,'alta':0.690+0.017}
GRIP=0.045 # grip point beyond L4 end (jaw centre) - assumption
def fk(q):
    t1,t12,t123=q[0],q[0]+q[1],q[0]+q[1]+q[2]
    p0=np.array([0,ZB]); p1=p0+L1*np.array([np.sin(t1),np.cos(t1)])
    p2=p1+L2*np.array([np.sin(t12),np.cos(t12)]); p3=p2+L3*np.array([np.sin(t123),np.cos(t123)])
    return p0,p1,p2,p3
def torque(q):
    p0,p1,p2,p3=fk(q)
    # wrist horizontal: grip point = p3 + (L4+GRIP) forward
    tip=p3+np.array([L4+GRIP,0])
    pts=[(p1,mS+mL),(p2,mS+mL),(p3,mS+mL),(p3+np.array([(L4+GRIP)/2,0]),mS+mG+0.012)]
    return g*sum(m*(p[0]-p0[0]) for p,m in pts), tip
qs=np.radians(np.arange(-60,121,2))
for name,zb in rows.items():
  for d in (0.08,0.12,0.16):
    best=None
    for a in qs:
      for b in qs:
        # solve q3 so that ... brute force too
        for c in qs:
          t,tip=torque((a,b,c))
          if abs(tip[0]-d)<0.006 and abs(tip[1]+BASE-zb)<0.006:
            if best is None or abs(t)<abs(best[0]): best=(t,a,b,c)
    print(name, 'd=%.2f'%d, 'z_rel=%.0fmm'%((zb-BASE-ZB)*1000), 'min|tau_hombro|= %s'%('INALCANZABLE' if best is None else '%.2f N·m'%best[0]))
# horizontal stretched
t,_=torque((np.pi/2,0,0)); print('estirado horizontal: %.2f N·m'%t)
