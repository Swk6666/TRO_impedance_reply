import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r1'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
cf=[]
for f in (1,3,6,10):
    for ct,ctl in (('impedance',None),('admittance',None),('hinf',dict(ref_gain_scale=.25)),('hinf',dict(ref_gain_scale=1.))):
        cf.append(dict(name=f"A_f{f}_{ct}"+(f"_a{ctl['ref_gain_scale']}" if ctl else ''),exp='A',ctrl=ct,plant=dict(friction_scale=float(f)),ctl=ctl))
    for a in (.0625,.125,.25,.5,1.):
        cf.append(dict(name=f"F_paperL_f{f}_a{a}",exp='B',ctrl='hinf',plant=dict(friction_scale=float(f)),ctl=dict(ref_gain_scale=a),imp=PB))
        if a<=.25:cf.append(dict(name=f"F_fr3L_f{f}_a{a}",exp='B',ctrl='hinf',plant=dict(friction_scale=float(f)),ctl=dict(ref_gain_scale=a)))
res=run_all(cf,S)
res.sort(key=lambda r:r['name'])
table(res,['pos_rms','pos_peak','alpha','ref','ref_std','d','d_std','eq','eq_std','floor','gap','Fpk','sat','tau','taupk'])
