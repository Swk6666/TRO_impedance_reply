import sys;sys.path.insert(0,'/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad')
from probe_lib import *
S='/tmp/claude-1000/-home-swk--claude-spares-spare-oNkjVp/67af89ca-a008-4677-a734-40676580a091/scratchpad/r6'
PB=dict(inertia=[10,10,10,1,1,1],damping=[80,80,80,10,10,10],stiffness=[400,400,400,50,50,50])
PC=dict(inertia=[4,4,10,.4,.4,.4],damping=[80,80,200,80,80,80],stiffness=[400,400,0,400,400,400])
PD=dict(inertia=[4,4,4,2,.4,.4],damping=[200,200,200,20,2,4],stiffness=[2000,2000,2000,0,200,10])
F=dict(friction_scale=3.5);d3=dict(delay_samples=3)
CB=dict(contact_timeconstant=.02,contact_dampratio=.4,contact_impedance=(.95,.99,.001))
CC=dict(contact_timeconstant=.04,contact_dampratio=.3,contact_impedance=(.9,.95,.001))
CC8=dict(contact_timeconstant=.04,contact_dampratio=.3,contact_impedance=(.8,.9,.001))
cf=[]
for seed in (100,101,102):
    for c_ in ('impedance','admittance','hinf'):
        cf.append(dict(name=f"A_s{seed}_{c_}",exp='A',ctrl=c_,plant=F,sensor=d3,seed=seed))
    for c_ in ('impedance','admittance','bidirectional','smc','hinf'):
        cf.append(dict(name=f"B_s{seed}_{c_}",exp='B',ctrl=c_,plant={**F,**CB},imp=PB,sensor=d3,seed=seed))
    for a in (.0625,.125,.25):
        cf.append(dict(name=f"F_s{seed}_a{a}",exp='B',ctrl='hinf',plant={**F,**CB},imp=PB,sensor=d3,ctl=dict(ref_gain_scale=a),seed=seed))
    for c_ in ('admittance','bidirectional','hinf'):
        cf.append(dict(name=f"C_s{seed}_{c_}",exp='C',ctrl=c_,plant={**F,**CC},imp=PC,sensor=d3,seed=seed))
        cf.append(dict(name=f"Csi8_s{seed}_{c_}",exp='C',ctrl=c_,plant={**F,**CC8},imp=PC,sensor=d3,seed=seed))
# refinement
for c_ in ('admittance','bidirectional','smc','hinf'):
    cf.append(dict(name=f"Bp8_{c_}",exp='B',ctrl=c_,plant={**F,**CB},imp=PB,sensor=d3,kw=dict(physics_substeps=8)))
for c_ in ('admittance','bidirectional','hinf'):
    cf.append(dict(name=f"Cp8_{c_}",exp='C',ctrl=c_,plant={**F,**CC},imp=PC,sensor=d3,kw=dict(physics_substeps=8)))
# adverse: bandwidth-matched G
cf.append(dict(name="Gm_B_admittance",exp='B',ctrl='admittance',plant={**F,**CB},imp=PB,sensor=d3,G=GM))
cf.append(dict(name="Gm_B_bidirectional",exp='B',ctrl='bidirectional',plant={**F,**CB},imp=PB,sensor=d3,G=GM))
cf.append(dict(name="Gm_C_admittance",exp='C',ctrl='admittance',plant={**F,**CC},imp=PC,sensor=d3,G=GM))
cf.append(dict(name="Gm_C_bidirectional",exp='C',ctrl='bidirectional',plant={**F,**CC},imp=PC,sensor=d3,G=GM))
cf.append(dict(name="Gm_D_admittance",exp='D',ctrl='admittance',plant=F,sensor=d3,imp=PD,G=GM,ctl=dict(nric_gain_scale=1.),kw=dict(operator_roll_hold=False,operator_stiffness_scale=1.,operator_torque_scale=[1.,1.,1.])))
# adverse: no delay at candidate contact
for c_ in ('admittance','hinf'):
    cf.append(dict(name=f"d0_B_{c_}",exp='B',ctrl=c_,plant={**F,**CB},imp=PB))
    cf.append(dict(name=f"d0_C_{c_}",exp='C',ctrl=c_,plant={**F,**CC},imp=PC))
for tr in range(30):
    for c_ in ('impedance','admittance','smc','hinf'):
        cf.append(dict(name=f"D_t{tr:02d}_{c_}",exp='D',ctrl=c_,plant=F,sensor=d3,imp=PD,trial=tr,seed=100+tr,ctl=dict(nric_gain_scale=1.),
                       kw=dict(operator_roll_hold=False,operator_stiffness_scale=1.,operator_torque_scale=[1.,1.,1.])))
res=run_all(cf,S,workers=30);res.sort(key=lambda r:r['name'])
print('status',{s:sum(r['status']==s for r in res) for s in set(r['status'] for r in res)},[ (r['name'],r['why']) for r in res if r['status']!='completed'])
K1=['pos_rms','pos_peak','ref','ref_std','eq','eq_std','floor','gap','Fpk','sat','taupk','F_hf_rms','F_hf_f','F_window_std','fz_rms','fz_pk','beta_c']
table([r for r in res if not r['name'].startswith('D_')],K1)
print('\nD 30 trials: mean +- sd')
for c_ in ('admittance','impedance','smc','hinf'):
    rs=[r for r in res if r['name'].startswith('D_t') and r['name'].endswith(c_)]
    a=lambda k:(np.mean([r[k] for r in rs]),np.std([r[k] for r in rs],ddof=1))
    print(c_.ljust(11),'complete',sum(r['status']=='completed' for r in rs),' '.join(f"{k}={a(k)[0]:.4f}±{a(k)[1]:.4f}" for k in ('pitch_eq','pitch_floor','pitch_gap','yaw_eq','yaw_floor','yaw_gap','roll2','sat','taupk')))
