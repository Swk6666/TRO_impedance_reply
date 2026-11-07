import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
from fr3imp.actuation import ActuatorParameters
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r3'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
CT=dict(soft=(.02,1.,(.5,.5,.001)),rig=(.004,1.,(.95,.99,.001)),lz1=(.01,.2,(.95,.99,.001)),lz2=(.005,.2,(.95,.99,.001)),lz3=(.02,.1,(.95,.99,.001)))
cf=[]
f=4.
for cn,(tc,z,si) in CT.items():
  pl=dict(friction_scale=f,contact_timeconstant=tc,contact_dampratio=z,contact_impedance=si)
  for dly in (0,3):
    for cut in (100.,40.):
      for arm in (1.,1.5):
        kw=dict(actuator_parameters=ActuatorParameters(cutoff_hz=cut))
        base=dict(exp='B',plant=pl,imp=PB,arm=arm,sensor=dict(delay_samples=dly),kw=kw)
        tag=f"{cn}_d{dly}_c{int(cut)}_a{arm}"
        cf.append(dict(name="adm_Gp_"+tag,ctrl='admittance',**base))
        cf.append(dict(name="adm_Gm_"+tag,ctrl='admittance',G=GM,**base))
        cf.append(dict(name="hin_"+tag,ctrl='hinf',ctl=dict(ref_gain_scale=.25),**base))
res=run_all(cf,S)
res.sort(key=lambda r:(r['name'][4:],r['name']) )
table(res,['pos_rms','ref','eq','gap','Fpk','sat','taupk','F_hf_rms','F_hf_f','F_window_std','loss'])
