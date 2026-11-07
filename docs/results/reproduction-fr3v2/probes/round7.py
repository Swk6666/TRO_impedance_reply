import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r7'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
PC=dict(inertia=[4,4,10,.4,.4,.4],damping=[80,80,200,80,80,80],stiffness=[400,400,0,400,400,400])
PD=dict(inertia=[4,4,4,2,.4,.4],damping=[200,200,200,20,2,4],stiffness=[2000,2000,2000,0,200,10])
AR=dict(full=[19.,29.,4.4,.25,.96,.96,.96],half=[9.,14.,2.2,.25,.48,.48,.48],low=[4.,6.,1.,.25,.25,.25,.25])
CB=dict(contact_timeconstant=.02,contact_dampratio=.4,contact_impedance=(.95,.99,.001))
CC=dict(contact_timeconstant=.04,contact_dampratio=.3,contact_impedance=(.9,.95,.001))
cf=[]
for an,av in AR.items():
  for fm in (6.,10.,13.):
    for dly in (0,3):
      fv=[fm,fm,fm,fm,fm*.3,fm*.3,fm*.3]
      t=f"{an}_f{int(fm)}_d{dly}";sn=dict(delay_samples=dly);b=dict(arm_vec=av,fric_vec=fv,sensor=sn)
      for c_ in ('impedance','admittance','hinf'):
        cf.append(dict(name=f"A_{t}_{c_}",exp='A',ctrl=c_,ctl=dict(ref_gain_scale=1.),**b))
      for c_ in ('impedance','admittance','bidirectional','smc'):
        cf.append(dict(name=f"B_{t}_{c_}",exp='B',ctrl=c_,plant=CB,imp=PB,ctl=dict(ref_gain_scale=1.),**b))
      for a in (.25,.5,1.):
        cf.append(dict(name=f"F_{t}_a{a}",exp='B',ctrl='hinf',plant=CB,imp=PB,ctl=dict(ref_gain_scale=a),**b))
      for c_ in ('admittance','bidirectional','hinf'):
        cf.append(dict(name=f"C_{t}_{c_}",exp='C',ctrl=c_,plant=CC,imp=PC,ctl=dict(ref_gain_scale=1.),**b))
      for c_ in ('impedance','admittance','smc','hinf'):
        cf.append(dict(name=f"D_{t}_{c_}",exp='D',ctrl=c_,imp=PD,ctl=dict(ref_gain_scale=1.,nric_gain_scale=1.),kw=dict(operator_roll_hold=False,operator_stiffness_scale=1.,operator_torque_scale=[1.,1.,1.]),**b))
res=run_all(cf,S,workers=30);res.sort(key=lambda r:r['name'])
print('status',{s:sum(r['status']==s for r in res) for s in set(r['status'] for r in res)})
for pre,keys in (('A',['pos_rms','pos_peak','ref','d','sat','tau','taupk']),('F',['ref','ref_std','d','d_std','eq','eq_std','floor','gap','Fpk','sat','tau','taupk']),('B',['pos_rms','pos_peak','ref','eq','floor','gap','Fpk','sat','tau','taupk','F_hf_rms','F_hf_f','F_window_std','beta_c']),('C',['ref','eq','gap','Fpk','sat','taupk','fz_rms','fz_pk','F_hf_rms','F_hf_f','beta_c']),('D',['pitch_eq','pitch_floor','pitch_gap','yaw_eq','yaw_floor','yaw_gap','roll2','sat','taupk'])):
    print('\n==',pre);
    for r in res:
        if pre=='D' and r['name'][0]=='D':
            for k in ('pitch_eq','pitch_floor','pitch_gap'):r[k]=r[k]*10
    table([r for r in res if r['name'][0]==pre],keys)
