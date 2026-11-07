import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r4'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
PC=dict(inertia=[4,4,10,.4,.4,.4],damping=[80,80,200,80,80,80],stiffness=[400,400,0,400,400,400])
PD=dict(inertia=[4,4,4,2,.4,.4],damping=[200,200,200,20,2,4],stiffness=[2000,2000,2000,0,200,10])
lz3=dict(contact_timeconstant=.02,contact_dampratio=.1,contact_impedance=(.95,.99,.001))
cf=[]
for f in (3.,4.):
  P=dict(friction_scale=f); d3=dict(delay_samples=3); t=f"f{int(f)}"
  for c_ in ('impedance','admittance','hinf'):
    cf.append(dict(name=f"A_{t}_{c_}",exp='A',ctrl=c_,plant=P,sensor=d3))
  for c_ in ('impedance','admittance','bidirectional','smc','hinf'):
    cf.append(dict(name=f"B_{t}_{c_}",exp='B',ctrl=c_,plant={**P,**lz3},imp=PB,sensor=d3))
  for a in (.0625,.125,.25):
    cf.append(dict(name=f"F_{t}_a{a}",exp='B',ctrl='hinf',plant={**P,**lz3},imp=PB,sensor=d3,ctl=dict(ref_gain_scale=a)))
    cf.append(dict(name=f"F_nodelay_{t}_a{a}",exp='B',ctrl='hinf',plant=P,imp=PB,ctl=dict(ref_gain_scale=a)))
  for cn,ct in dict(cur=dict(contact_timeconstant=.04,contact_dampratio=1.,contact_impedance=(.5,.5,.001)),
                    z1=dict(contact_timeconstant=.04,contact_dampratio=.2,contact_impedance=(.9,.95,.001)),
                    z2=dict(contact_timeconstant=.02,contact_dampratio=.2,contact_impedance=(.9,.95,.001)),
                    z3=dict(contact_timeconstant=.02,contact_dampratio=.1,contact_impedance=(.95,.99,.001))).items():
    for ln,imp in dict(Lp=PC,Lx=None).items():
      for c_ in ('admittance','bidirectional','hinf'):
        cf.append(dict(name=f"C_{t}_{cn}_{ln}_{c_}",exp='C',ctrl=c_,plant={**P,**ct},imp=imp,sensor=d3))
  for hold in (True,False):
    for c_ in ('impedance','admittance','smc','hinf'):
      cf.append(dict(name=f"D_{t}_fr3_h{int(hold)}_{c_}",exp='D',ctrl=c_,plant=P,sensor=d3,kw=dict(operator_roll_hold=hold)))
      cf.append(dict(name=f"D_{t}_pap_h{int(hold)}_{c_}",exp='D',ctrl=c_,plant=P,sensor=d3,imp=PD,ctl=dict(nric_gain_scale=1.),
                     kw=dict(operator_roll_hold=hold,operator_stiffness_scale=1.,operator_torque_scale=[1.,1.,1.])))
res=run_all(cf,S,workers=28)
res.sort(key=lambda r:r['name'])
for pre,keys in (('A',['pos_rms','pos_peak','ref','d','gap','sat','taupk']),('B',['pos_rms','pos_peak','ref','eq','floor','gap','Fpk','sat','taupk','F_hf_rms','F_hf_f','F_window_std','loss','beta_c','beta_f']),
  ('F',['ref','ref_std','d','eq','eq_std','floor','gap','Fpk','sat']),('C',['pos_rms','ref','eq','gap','Fpk','sat','taupk','fz_rms','fz_pk','F_hf_rms','F_hf_f','beta_c','beta_f']),('D',['pitch_eq','pitch_floor','pitch_gap','yaw_eq','yaw_floor','yaw_gap','roll2','sat','taupk','ref'])):
    print('\n==',pre);table([r for r in res if r['name'][0]==pre],keys)
