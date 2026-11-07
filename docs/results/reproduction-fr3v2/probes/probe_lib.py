import json,os,sys,numpy as np,io,contextlib
from dataclasses import replace
from multiprocessing import Pool
sys.path.insert(0,'/home/swk/swk/TRO_impedance_reply')
import fr3imp.controllers as C
from fr3imp.experiment import run_experiment,default_plant_parameters
from fr3imp.evaluation import ideal_impedance_error
from fr3imp.dynamics import euler_rate_map
GM=[80,90,140,380,40,43,30.]   # joint PID gain giving paper's -3 dB bandwidth on FR3 (home/ABF average)
PAPER={k:v for k,v in dict(A=None).items()}
_orig=C.Admittance.__init__
def run(c):
    name=c['name'];exp=c['exp'];ctrl=c['ctrl'];out=c.get('out')
    G=c.get('G')
    def patched(self,*a,**k):
        _orig(self,*a,**k)
        if G is not None:self.G=np.array(G,float)
    C.Admittance.__init__=patched
    import fr3imp.experiment as X
    from fr3imp.model import build_model as _bm
    arm=c.get('arm',1.);fv=c.get('fric_vec')
    def bm(parameters,*,nominal=False,environment=None,visuals=True):
        m=_bm(parameters,nominal=nominal,environment=environment,visuals=visuals)
        if c.get('arm_vec') is not None:m.dof_armature[:7]=np.array(c['arm_vec'])
        if not nominal:
            m.dof_armature[:7]*=arm
            if fv is not None:m.dof_frictionloss[:7]=np.array([1.137,1.137,1.137,1.137,.763,.44,.248])*np.array(fv)
        return m
    X.build_model=bm
    pp=default_plant_parameters(exp,'fr3','fr3','external')
    pp=replace(pp,**c.get('plant',{}))
    kw=dict(c.get('kw',{}))
    if 'sensor' in c:
        from fr3imp.sensors import SensorParameters
        kw['sensor_parameters']=SensorParameters(tool_calibration_mass=pp.tool_mass,**c['sensor'])
    with contextlib.redirect_stdout(io.StringIO()):
        s=run_experiment(exp,ctrl,seed=c.get('seed',100),trial=c.get('trial',0),output=f"{out}/{name}",plant_parameters=pp,
             impedance_overrides=c.get('imp'),control_overrides=c.get('ctl'),duration=c.get('duration'),**kw)
    import pathlib
    folder=pathlib.Path(out)/name/f"{exp}_{ctrl}_trial{c.get('trial',0):02d}_seed{c.get('seed',100)}"
    l=np.load(folder/'trajectory.npz');imp=s['impedance'];L,D,K=[np.array(imp[k]) for k in('inertia','damping','stiffness')]
    t=l['time'];e=(l['desired']-l['pose']).astype(float);e[:,3:]=(e[:,3:]+np.pi)%(2*np.pi)-np.pi
    Ft=l['force_true'].astype(float);Fm=l['force_measured'].astype(float);Fd=l['force_desired'].astype(float)
    def gen(F):
        F=F.copy()
        for i,p in enumerate(l['pose']):F[i,3:]=euler_rate_map(p[3:]).T@F[i,3:]
        return F
    gt,gm,gd=gen(Ft),gen(Fm),gen(Fd)
    dyn=ideal_impedance_error(t,gd-gt,L,D,K,e[0],np.zeros(6))
    nz=K>0
    eqm=e.copy();eqm[:,nz]+=(gm-gd)[:,nz]/K[nz]
    flo=dyn.copy();flo[:,nz]+=(gt-gd)[:,nz]/K[nz]
    n=lambda x:np.linalg.norm(x,axis=1)
    # non-force axes only for position gap (z in C has K=0)
    ax=[i for i in range(3) if K[i]>0]
    sn=n(l['reference_error'][:,:3])
    alpha=s['controller_parameters'].get('alpha')
    m=dict(name=name,status=s['status'],T=float(s['simulated_duration_s']),why=s['termination_reason'],
      pos_rms=float(np.sqrt((n(e[:,:3])**2).mean())*1e3),pos_peak=float(n(e[:,:3]).max()*1e3),
      ref=float(sn.mean()),ref_std=float(sn.std()),eq=float(n(eqm[:,ax]).mean()*1e3),eq_std=float(n(eqm[:,ax]).std()*1e3),
      floor=float(n(flo[:,ax]).mean()*1e3),gap=float(n((e-dyn)[:,ax]).mean()*1e3),
      Fpk=float(n(Ft[:,:3]).max()),sat=float(l['saturated'].mean()*100),tau=float(n(l['torque_command']).mean()),taupk=float(n(l['torque_command']).max()),
      alpha=alpha)
    if alpha:m['d']=float(alpha*1100*sn.mean());m['d_std']=float(alpha*1100*sn.std())
    if exp=='C':
        m['fz_rms']=float(np.sqrt(((Fm[:,2]-Fd[:,2])**2).mean()));m['fz_pk']=float(Fm[:,2].max())
    if exp=='D':
        for ax_,lab in [(4,'pitch'),(5,'yaw')]:
            m[lab+'_eq']=float(abs(eqm[:,ax_]).mean());m[lab+'_gap']=float(abs((e-dyn)[:,ax_]).mean());m[lab+'_floor']=float(abs(flo[:,ax_]).mean())
        m['roll2']=float(l['pose'][t>=23,3].mean()*180/np.pi) if (t>=23).any() else None
    if 'beta' in l and ctrl=='bidirectional':
        c_=n(l['force_contact'][:,:3])>1
        m['beta_c']=float(l['beta'][c_].mean()) if c_.any() else None;m['beta_f']=float(l['beta'][~c_].mean())
    # chatter band fraction of measured force in contact
    c_=n(l['force_contact'][:,:3])>1
    if c_.sum()>500:
        x=Fm[c_][:,:3];x=x-x.mean(0);X=(abs(np.fft.rfft(x,axis=0))**2).sum(1);f=np.fft.rfftfreq(len(x),.001)
        hi=X[f>=20].sum();m['F_hf_rms']=float(np.sqrt(hi/X.sum())*x.std(0).sum());m['F_peak_f']=float(f[1:][np.argmax(X[1:])])
    if c_.sum()>500:
        hb=f>=5
        m['F_hf_f']=float(f[hb][np.argmax(X[hb])])
        i0,i1=np.where(c_)[0][[0,-1]]
        m['F_window_std']=float(np.linalg.norm(Fm[i0:i1,:3]-np.apply_along_axis(lambda a:np.convolve(a,np.ones(201)/201,'same'),0,Fm[i0:i1,:3]),axis=1)[300:-300].std()) if i1-i0>700 else None
        m['loss']=int(np.sum(np.diff(c_[i0:i1].astype(int))==-1))
    import shutil
    if not c.get('keep'):shutil.rmtree(folder,ignore_errors=True)
    return m
def run_all(cfgs,out,workers=26):
    for c in cfgs:c['out']=out
    os.makedirs(out,exist_ok=True)
    with Pool(workers) as p:res=p.map(run,cfgs,chunksize=1)
    json.dump(res,open(out+'/res.json','w'),indent=1);return res
def table(res,keys):
    print(' | '.join(['name'.ljust(38)]+[k.rjust(8) for k in keys]))
    for r in res:
        def fm(v):
            if v is None:return '-'.rjust(8)
            return (f'{v:8.3f}' if isinstance(v,float) else str(v).rjust(8))
        print(' | '.join([r['name'].ljust(38)]+[fm(r.get(k)) for k in keys])+('' if r['status']=='completed' else f"  !!{r['T']:.1f}s {r['why']}"))
