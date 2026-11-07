import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r2'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
PC=dict(inertia=[4,4,10,.4,.4,.4],damping=[80,80,200,80,80,80],stiffness=[400,400,0,400,400,400])
CT=dict(soft=dict(contact_timeconstant=.02,contact_impedance=(.5,.5,.001)),mid=dict(contact_timeconstant=.004,contact_impedance=(.5,.5,.001)),rigid=dict(contact_timeconstant=.004,contact_impedance=(.95,.99,.001)))
G0=[1200,1200,600,400,400,400,400.]
Gs=dict(Gp=None,Gm=GM,Gm2=[2*g for g in GM],Gm05=[.5*g for g in GM],Gw=[1200,1200,600,400,80,80,60.])
cf=[]
for f in (1,10):
  for cn,ct in CT.items():
    pl=dict(friction_scale=float(f),**ct)
    for gn,G in Gs.items():
        cf.append(dict(name=f"B_adm_f{f}_{cn}_{gn}",exp='B',ctrl='admittance',plant=pl,imp=PB,G=G))
    for a in (.25,1.):
        cf.append(dict(name=f"B_hinf_f{f}_{cn}_a{a}",exp='B',ctrl='hinf',plant=pl,imp=PB,ctl=dict(ref_gain_scale=a)))
    for c_ in ('smc','bidirectional','impedance'):
        cf.append(dict(name=f"B_{c_[:5]}_f{f}_{cn}",exp='B',ctrl=c_,plant=pl,imp=PB,**(dict(G=GM) if c_=='bidirectional' else {})))
for f in (1,10):
  for cn,ct in dict(soft=dict(contact_timeconstant=.04,contact_impedance=(.5,.5,.001)),mid=dict(contact_timeconstant=.01,contact_impedance=(.5,.5,.001))).items():
    pl=dict(friction_scale=float(f),**ct)
    for ln,imp in dict(Lp=PC).items():
        for gn,G in dict(Gp=None,Gm=GM,Gw=Gs['Gw']).items():
            cf.append(dict(name=f"C_adm_f{f}_{cn}_{ln}_{gn}",exp='C',ctrl='admittance',plant=pl,imp=imp,G=G))
            cf.append(dict(name=f"C_bid_f{f}_{cn}_{ln}_{gn}",exp='C',ctrl='bidirectional',plant=pl,imp=imp,G=G))
        for a in (.25,1.):
            cf.append(dict(name=f"C_hinf_f{f}_{cn}_{ln}_a{a}",exp='C',ctrl='hinf',plant=pl,imp=imp,ctl=dict(ref_gain_scale=a)))
res=run_all(cf,S)
res.sort(key=lambda r:r['name'])
table(res,['pos_rms','ref','d','eq','floor','gap','Fpk','sat','tau','taupk','F_hf_rms','F_peak_f','fz_rms','fz_pk','beta_c','beta_f'])
