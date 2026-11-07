import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r9'
PD=dict(inertia=[4,4,4,2,.4,.4],damping=[200,200,200,20,2,4],stiffness=[2000,2000,2000,0,200,10])
cf=[]
for w in (3.,6.,10.,14.):
  for c_ in ('impedance','admittance','smc','hinf'):
    cf.append(dict(name=f"D_w{int(w)}_{c_}",exp='D',ctrl=c_,fric_vec=[3,3,3,3,w,w,w],sensor=dict(delay_samples=3),imp=PD,ctl=dict(nric_gain_scale=1.),kw=dict(operator_roll_hold=False,operator_stiffness_scale=1.,operator_torque_scale=[1.,1.,1.])))
res=run_all(cf,S,workers=16);res.sort(key=lambda r:r['name'])
for r in res:
    for k in ('pitch_eq','pitch_floor','pitch_gap'):r[k]*=100
print('pitch values x100');table(res,['pitch_eq','pitch_floor','pitch_gap','yaw_eq','yaw_floor','yaw_gap','roll2','sat','taupk'])
