import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r5'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
PC=dict(inertia=[4,4,10,.4,.4,.4],damping=[80,80,200,80,80,80],stiffness=[400,400,0,400,400,400])
cf=[];P=dict(friction_scale=3.5)
for tc in (.02,.01):
  for z in (.1,.2,.4,.7):
    for d in (1,2,3):
      pl={**P,'contact_timeconstant':tc,'contact_dampratio':z,'contact_impedance':(.95,.99,.001)}
      for c_ in ('admittance','bidirectional','smc','hinf'):
        cf.append(dict(name=f"B_tc{tc}_z{z}_d{d}_{c_[:4]}",exp='B',ctrl=c_,plant=pl,imp=PB,sensor=dict(delay_samples=d)))
for z,si in ((.3,(.9,.95,.001)),(.5,(.9,.95,.001)),(.7,(.9,.95,.001)),(.3,(.7,.8,.001)),(.5,(.7,.8,.001))):
    for d in (1,2,3):
      pl={**P,'contact_timeconstant':.04,'contact_dampratio':z,'contact_impedance':si}
      for c_ in ('admittance','bidirectional','hinf'):
        cf.append(dict(name=f"C_z{z}_si{si[0]}_d{d}_{c_[:4]}",exp='C',ctrl=c_,plant=pl,imp=PC,sensor=dict(delay_samples=d)))
res=run_all(cf,S,workers=28);res.sort(key=lambda r:r['name'])
print('== B');table([r for r in res if r['name'][0]=='B'],['pos_rms','ref','eq','floor','gap','Fpk','sat','taupk','F_hf_rms','F_hf_f','F_window_std','beta_c'])
print('== C');table([r for r in res if r['name'][0]=='C'],['pos_rms','ref','eq','gap','Fpk','sat','taupk','fz_rms','fz_pk','F_hf_rms','F_hf_f','beta_c'])
