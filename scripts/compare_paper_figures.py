"""Compare original paper figures with saved FR3 recordings, without simulation.

Requires the supplied PDF, Poppler pdftoppm, and the complete 134-run FR3 main
matrix (30 D trials per controller). E is excluded by AGENTS.md.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse,hashlib,json,subprocess,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'docs/results/paper-comparison')
parser.add_argument('--runs',type=Path,default=ROOT/'outputs/reproduction-fr3/main')
parser.add_argument('--pdf',type=Path,default=ROOT/'Nonlinear_mathcal_H_infty__Inverse-Optimal_Impedance_Control_for_Robotic_Systems.pdf')
args=parser.parse_args()
OUT=args.output.resolve();PDF=args.pdf.resolve();RUNS=args.runs.resolve()
if not PDF.is_file():parser.error(f'Paper PDF not found: {PDF}')
if not (RUNS/'manifest.json').is_file():parser.error('Generate the FR3 main matrix first; see docs/results/paper-comparison/REPORT.md')
manifest=json.loads((RUNS/'manifest.json').read_text())
if manifest.get('profile')!='fr3' or manifest.get('d_trials')!=30 or manifest.get('errors'):
    parser.error('Comparison requires a complete fr3 profile matrix with 30 D trials and no worker errors')
(OUT/'source').mkdir(parents=True,exist_ok=True)

def source_path(path):
    """Store portable repository-relative paths whenever the input is local."""
    try:return str(path.relative_to(ROOT))
    except ValueError:return str(path)

def sha256(path):
    with path.open('rb') as file:return hashlib.file_digest(file,'sha256').hexdigest()

plt.rcParams.update({'font.size':9,'axes.titlesize':10,'axes.labelsize':9,'legend.fontsize':8,'lines.linewidth':.9,'figure.facecolor':'white','savefig.facecolor':'white'})
COL={'impedance':'#7e2f8e','admittance':'#77ac30','bidirectional':'#0072bd','smc':'#edb120','hinf':'#d95319'}
NAMES={'impedance':'Impedance','admittance':'Admittance','bidirectional':'Bidirectional','smc':'Sliding mode','hinf':r'$H_\infty$'}
AXCOL=['#d95319','#77ac30','#0072bd']
GROUPS={'A':['impedance','admittance','hinf'],'B':['impedance','admittance','smc','bidirectional','hinf'],'C':['admittance','bidirectional','hinf'],'D':['impedance','admittance','smc','hinf'],'F':['hinf_0.25','hinf_0.5','hinf_1']}
CROPS={'A':(13,(312,54,566,175),'5'),'B':(14,(47,54,301,519),'6'),
       'B_control':(14,(312,54,566,175),'7'),'C':(15,(312,54,566,493),'9'),
       'D_full':(17,(47,54,551,450),'11'),'D_motion':(17,(47,158,551,343),'11, pitch/yaw rows'),
       'D':(17,(47,486,301,582),'12'),'F':(19,(47,378,301,475),'16')}

def crop(item):
    name,(page,box,number)=item
    x,y,r,b=box;scale=300/72
    command=['pdftoppm','-f',str(page),'-l',str(page),'-r','300','-x',str(round(x*scale)),'-y',str(round(y*scale)),
             '-W',str(round((r-x)*scale)),'-H',str(round((b-y)*scale)),'-singlefile','-png',str(PDF),str(OUT/'source'/('paper_'+name))]
    subprocess.run(command,check=True)

with ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(crop,CROPS.items()))

cache={};provenance={};computed={}
for e,controllers in GROUPS.items():
    for c in controllers:
        folder=RUNS/f'{e}_{c}_trial00_seed100'
        s=json.loads((folder/'summary.json').read_text())
        with np.load(folder/'trajectory.npz') as f:
            a={k:f[k].astype(float) for k in ('time','pose','desired','force_measured','force_true','reference_error','torque_command','torque_applied','beta','force_desired','v_measured')}
        assert s['profile']=='fr3' and s['status']=='completed'
        assert s['source_sha256']==manifest['source_sha256'], 'Mixed runtime versions'
        a['error']=a['desired']-a['pose'];a['error'][:,3:]=(a['error'][:,3:]+np.pi)%(2*np.pi)-np.pi
        p=a['pose'];fm=a['force_measured'];cp,sp,cy,sy=np.cos(p[:,4]),np.sin(p[:,4]),np.cos(p[:,5]),np.sin(p[:,5])
        generalized=fm.copy()
        generalized[:,3]=cy*cp*fm[:,3]+sy*cp*fm[:,4]-sp*fm[:,5]
        generalized[:,4]=-sy*fm[:,3]+cy*fm[:,4]
        a['generalized_measured']=generalized
        k=np.asarray(s['impedance']['stiffness']);a['static_ref']=np.divide(a['force_desired']-generalized,k,out=np.zeros_like(generalized),where=k!=0)
        cache[e,c]=(s,a)
        provenance[e+'_'+c]=dict(summary=source_path(folder/'summary.json'),trajectory=source_path(folder/'trajectory.npz'),trajectory_sha256=sha256(folder/'trajectory.npz'),source_sha256=s['source_sha256'],effective_alpha=s['controller_parameters'].get('alpha'),impedance=s['impedance'])
        if e in ('A','B'):computed[e+'_'+c]=dict(s['metrics'])
        if c=='bidirectional':
            window=(a['time']>=13)&(a['time']<(19 if e=='C' else 18))
            computed[e+'_beta']=dict(window_s=[13,19 if e=='C' else 18],mean=float(a['beta'][window].mean()),fraction_above_0_9=float((a['beta'][window]>.9).mean()),max=float(a['beta'][window].max()))
        if e=='C':
            computed[e+'_'+c]=dict(measured_force_rmse_N=float(np.sqrt(np.mean((fm[:,2]-a['force_desired'][:,2])**2))),measured_force_min_N=float(fm[:,2].min()),measured_force_max_N=float(fm[:,2].max()),joint_speed_norm_mean_rad_s=float(np.linalg.norm(a['v_measured'],axis=1).mean()),joint_speed_norm_max_rad_s=float(np.linalg.norm(a['v_measured'],axis=1).max()))
        if e=='F':
            eq=np.linalg.norm((a['error']-a['static_ref'])[:,:3],axis=1);ref=np.linalg.norm(a['reference_error'][:,:3],axis=1)
            computed[e+'_'+c]=dict(ref_mean=float(ref.mean()),ref_sd=float(ref.std()),measured_equilibrium_mean_m=float(eq.mean()),measured_equilibrium_sd_m=float(eq.std()),physical_equilibrium_mean_m=s['metrics']['equilibrium_position_error_mean_m'],physical_equilibrium_sd_m=s['metrics']['equilibrium_position_error_std_m'])

def axes_time(ax,limit=30):
    ax.grid(alpha=.16);ax.set(xlim=(0,limit),xlabel='Time [s]')

def finish(fig,name):
    fig.savefig(OUT/f'fr3_{name}.png',dpi=180)
    plt.close(fig)

def pair(name,title,note=''):
    left=Image.open(OUT/'source'/f'paper_{name}.png');right=Image.open(OUT/f'fr3_{name}.png')
    lh=left.height/left.width;rh=right.height/right.width
    content=max(lh,rh)*7
    fig=plt.figure(figsize=(14.5,content+.75))
    # Images are never stretched or normalized: preserve their aspect ratios.
    for start,im in [(.015,left),(.515,right)]:
        ax=fig.add_axes([start,.40/(content+.75),.47,content/(content+.75)])
        ax.imshow(im);ax.axis('off')
    fig.text(.25,.978,f'Paper Fig. {CROPS[name][2]} | Indy7 experiment',ha='center',va='top',fontsize=13,weight='bold')
    fig.text(.75,.978,title,ha='center',va='top',fontsize=13,weight='bold')
    if note:fig.text(.5,.012,note,ha='center',va='bottom',fontsize=9,color='#444444')
    fig.savefig(OUT/f'{name}_compare.png',dpi=180)
    plt.close(fig)

# A: exactly the two error norms in Fig. 5, with the source axis limits.
fig,ax=plt.subplots(1,2,figsize=(8,3.6),layout='constrained')
acol={'impedance':'#77ac30','admittance':'#0072bd','hinf':'#d95319'}
for c in GROUPS['A']:
    s,a=cache['A',c]
    for i,sl in enumerate((slice(0,3),slice(3,6))):ax[i].plot(a['time'],np.linalg.norm(a['error'][:,sl],axis=1),color=acol[c],label=NAMES[c])
for axis,lim,label in zip(ax,[(0,.06),(0,.045)],['Position error norm [m]','Orientation error norm [rad]']):
    axes_time(axis);axis.set(ylim=lim,ylabel=label)
fig.legend(*ax[0].get_legend_handles_labels(),loc='outside upper center',ncol=3)
finish(fig,'A');pair('A','FR3 simulation | same axis ranges','Actual saved errors; no amplitude fitting. H-infinity base alpha = 0.25 (paper: 1).')

# B: match the panels of Fig. 6. Display-relative TCP positions are labelled.
fig=plt.figure(figsize=(7.8,13.5),layout='constrained');gs=fig.add_gridspec(4,2)
ax3=fig.add_subplot(gs[0,0],projection='3d');xy=fig.add_subplot(gs[0,1])
for c in GROUPS['B']:
    s,a=cache['B',c];p=a['pose'][:,:3]-a['desired'][0,:3]
    ax3.plot(p[:,0],p[:,1],p[:,2],color=COL[c],label=NAMES[c]);xy.plot(p[:,1],p[:,0],color=COL[c])
p=a['desired'][:,:3]-a['desired'][0,:3]
ax3.plot(*p.T,'k:',label='Desired');xy.plot(p[:,1],p[:,0],'k:')
ax3.set(xlabel='x - x0 [m]',ylabel='y - y0 [m]',zlabel='z - z0 [m]');ax3.tick_params(labelsize=7);ax3.view_init(25,-65)
xy.set(xlabel='y - y0 [m]',ylabel='x - x0 [m]',aspect='equal');xy.invert_xaxis();xy.grid(alpha=.15)
limits={'impedance':(-.1,.08),'admittance':(-.4,.8),'smc':(-.08,.08),'bidirectional':(-.3,.2),'hinf':(-.15,.2)}
for i,c in enumerate(GROUPS['B']):
    axis=fig.add_subplot(gs[1+i//2,i%2]);s,a=cache['B',c]
    for j,col in enumerate(AXCOL):
        axis.plot(a['time'],a['error'][:,j],color=col,label=f'error {"xyz"[j]}')
        axis.plot(a['time'],a['static_ref'][:,j],color=col,ls=':',lw=.75,label=f'-F{ "xyz"[j]}/K (F/T)')
    axes_time(axis);axis.set(title=NAMES[c],ylabel='Position error [m]',ylim=limits[c]);axis.axvspan(13,18,color='gray',alpha=.05)
    if c=='admittance':axis.text(.98,.95,f'Completed {s["simulated_duration_s"]:g} s\n{s["metrics"]["saturation_fraction"]:.2%} saturation',transform=axis.transAxes,ha='right',va='top',fontsize=8)
    if c=='impedance':axis.legend(ncol=2,fontsize=6.5,loc='lower left')
axis=fig.add_subplot(gs[3,1]);s,a=cache['B','bidirectional'];axis.plot(a['time'],a['beta'],color='#444444',lw=.5);axes_time(axis);axis.set(ylabel='beta',ylim=(0,1.05),title='Bidirectional modulation')
fig.legend(*ax3.get_legend_handles_labels(),loc='outside upper center',ncol=3,fontsize=9)
finish(fig,'B');pair('B','FR3 simulation | Fig. 6 observables','Positions are relative to each robot\'s initial TCP; errors and F/K are unscaled. Dotted references use measured F/T.')

fig,ax=plt.subplots(1,2,figsize=(8,3.6),layout='constrained')
for c,col in [('smc','#0072bd'),('hinf','#d95319')]:
    s,a=cache['B',c]
    ax[0].plot(a['time'],np.linalg.norm(a['reference_error'][:,:3],axis=1),color=col,label=NAMES[c])
    ax[1].plot(a['time'],np.linalg.norm(a['torque_command'],axis=1),color=col,label=NAMES[c]+' command')
    ax[1].plot(a['time'],np.linalg.norm(a['torque_applied'],axis=1),color=col,ls=':',lw=.7)
for axis,lim,label in zip(ax,[(0,.5),(0,350)],['Translational REF norm [m/s]','Joint torque norm [Nm]']):axes_time(axis);axis.set(ylim=lim,ylabel=label)
fig.legend(*ax[0].get_legend_handles_labels(),loc='outside upper center',ncol=2)
finish(fig,'B_control');pair('B_control','FR3 | same axis ranges','Torque: solid = controller command; dotted = applied after 100 Hz filter and limits. Different robots / torque capacities.')

# C: all eight Fig. 9 observables. The third-row right panel is JOINT SPEED, not REF.
fig,ax=plt.subplots(4,2,figsize=(7.8,12.8),layout='constrained')
for c in GROUPS['C']:
    s,a=cache['C',c];p=a['pose'][:,:3]-a['desired'][0,:3];col=COL[c];t=a['time']
    ax[0,0].plot(p[:,1],p[:,0],color=col,label=NAMES[c]);ax[0,1].plot(p[:,1],p[:,2],color=col)
    for j in range(2):
        ax[1,j].plot(t,a['error'][:,j],color=col);ax[1,j].plot(t,a['static_ref'][:,j],color=col,ls=':',lw=.7)
    ax[2,0].plot(t,a['force_measured'][:,2],color=col,lw=.65)
    ax[2,1].plot(t,np.linalg.norm(a['v_measured'],axis=1),color=col)
    ax[3,0].plot(t,np.linalg.norm(a['error'][:,3:],axis=1),color=col)
    ax[3,0].plot(t,np.linalg.norm(a['static_ref'][:,3:],axis=1),color=col,ls=':',lw=.7)
    if c=='bidirectional':ax[3,1].plot(t,a['beta'],color='#444444',lw=.5)
p=a['desired'][:,:3]-a['desired'][0,:3]
ax[0,0].plot(p[:,1],p[:,0],'k:',label='Desired');ax[0,1].plot(p[:,1],p[:,2],'k:')
for j,label in [(0,'x - x0 [m]'),(1,'z - z0 [m]')]:ax[0,j].set(xlabel='y - y0 [m]',ylabel=label);ax[0,j].invert_xaxis();ax[0,j].grid(alpha=.15)
ax[0,0].set_aspect('equal')
for row in range(1,4):
    for axis in ax[row]:axes_time(axis);axis.axvspan(13,19,color='gray',alpha=.05)
ax[1,0].set(ylabel='x error [m]',ylim=(-.06,.08));ax[1,1].set(ylabel='y error [m]',ylim=(-.08,.04))
ax[2,0].plot(t,a['force_desired'][:,2],'k:',lw=1.2);ax[2,0].set(ylabel='Measured Fz [N]',ylim=(-20,70),title='Force range expanded to retain peaks')
ax[2,1].set(ylabel='Joint speed norm [rad/s]',ylim=(0,.75),title='7-joint measured velocity; all peaks retained')
ax[3,0].set(ylabel='Orientation error / static ref. [rad]',ylim=(0,.02));ax[3,1].set(ylabel='beta',ylim=(0,1.05))
for axis in (ax[1,0],ax[1,1],ax[3,0]):
    low,high=axis.get_ylim()
    data=np.concatenate([line.get_ydata() for line in axis.get_lines()])
    span=float(data.max()-data.min())
    new_low=min(low,float(data.min())-.02*span);new_high=max(high,float(data.max())+.02*span)
    if data.min()<low or data.max()>high:
        axis.set(ylim=(new_low,new_high),title='Expanded range: all samples retained')
fig.legend(*ax[0,0].get_legend_handles_labels(),loc='outside upper center',ncol=4,fontsize=9)
finish(fig,'C');pair('C','FR3 simulation | Fig. 9 observables','Joint-speed norms use 6 axes in the paper and 7 in FR3. Ranges expanded where needed; all force/speed peaks retained.')

# D: representative pitch/yaw rows, plus statistical comparison over all 30 runs.
fig,ax=plt.subplots(2,4,figsize=(13,4.9),layout='constrained',sharex=True,sharey='row')
for j,c in enumerate(GROUPS['D']):
    s,a=cache['D',c]
    for i,k,col in [(0,4,'#77ac30'),(1,5,'#0072bd')]:
        ax[i,j].plot(a['time'],a['error'][:,k],color='#555555',label='Task error')
        ax[i,j].plot(a['time'],a['static_ref'][:,k],color=col,ls=':',label='-moment/K (F/T)')
        axes_time(ax[i,j],35);ax[i,j].axvspan(13,20,color='gray',alpha=.06)
    ax[0,j].set_title(NAMES[c])
ax[0,0].set(ylabel='Pitch error [rad]',ylim=(-.3,.4));ax[1,0].set(ylabel='Yaw error [rad]',ylim=(-1.5,1.5))
ax[0,0].legend(fontsize=7)
finish(fig,'D_motion');pair('D_motion','FR3 | representative trial 0','Both panels: columns = impedance, admittance, sliding mode, H-infinity. Different inputs / roll direction; no time alignment.')

stats={};d_sources={};order=['admittance','impedance','smc','hinf'];bars=['#0072bd','#d95319','#edb120','#7e2f8e']
for c in order:
    values=[]
    for folder in sorted(RUNS.glob(f'D_{c}_trial*')):
        s=json.loads((folder/'summary.json').read_text());assert s['status']=='completed'
        assert s['profile']=='fr3' and s['source_sha256']==manifest['source_sha256']
        d_sources[folder.name]=dict(summary=source_path(folder/'summary.json'),trajectory_sha256=sha256(folder/'trajectory.npz'))
        with np.load(folder/'trajectory.npz') as a:
            p=a['pose'].astype(float);f=a['force_measured'].astype(float)
        pitch=-p[:,4]+(-np.sin(p[:,5])*f[:,3]+np.cos(p[:,5])*f[:,4])/s['impedance']['stiffness'][4]
        yaw=-p[:,5]+f[:,5]/s['impedance']['stiffness'][5]
        values.append([np.abs(pitch).mean(),np.abs(yaw).mean()])
    values=np.array(values);assert len(values)==30
    stats[c]=dict(trials=30,mean_rad=values.mean(0).tolist(),sample_sd_rad=values.std(0,ddof=1).tolist())
computed['D_FT_statistics']=stats
fig,ax=plt.subplots(1,2,figsize=(8,3.4),layout='constrained')
for j,(axis,label,limit) in enumerate(zip(ax,['Pitch equilibrium discrepancy [rad]','Yaw equilibrium discrepancy [rad]'],[.045,.45])):
    mean=[stats[c]['mean_rad'][j] for c in order];sd=[stats[c]['sample_sd_rad'][j] for c in order]
    axis.bar(range(4),mean,yerr=sd,color=bars,capsize=3)
    for i,v in enumerate(mean):axis.text(i,v+limit*.02,f'{v:.4f}',ha='center',va='bottom',fontsize=8)
    axis.set(xticks=range(4),xticklabels=['Adm','Imp','SMC',r'$H_\infty$'],ylabel=label,ylim=(0,limit));axis.grid(axis='y',alpha=.15)
finish(fig,'D');pair('D','FR3 | 30 virtual-operator trials','Same radian scales; measured F/T used on the right. Error bars = sample SD across trials, not human confidence intervals.')

# F: paper-equivalent F/T statistic; preserve previously reported true-force values in metrics.
fig,ax=plt.subplots(1,2,figsize=(8,3.4),layout='constrained')
for j,(axis,mk,sk,label,limits) in enumerate(zip(ax,['ref_mean','measured_equilibrium_mean_m'],['ref_sd','measured_equilibrium_sd_m'],['Translational REF norm [m/s]','F/T equilibrium discrepancy [m]'],[(0,.5),(-.002,.016)])):
    for i,c in enumerate(GROUPS['F']):
        m=computed['F_'+c];v,sd=m[mk],m[sk]
        axis.bar(i,v,yerr=sd,color=bars[i],capsize=3)
        text=(f'{v:.3f} +/- {sd:.3f}' if j==0 else f'{v:.6f}\n+/- {sd:.6f}')
        axis.text(i,v+sd+.02*(limits[1]-limits[0]),text,ha='center',va='bottom',fontsize=7.3)
    axis.set(xticks=range(3),xticklabels=['C1\na=0.0625','C2\na=0.125','C3\na=0.25'],ylabel=label,ylim=limits);axis.grid(axis='y',alpha=.15)
finish(fig,'F');pair('F','FR3 | same scales and F/T metric','Paper alpha = 0.25 / 0.5 / 1; FR3 effective alpha = 0.0625 / 0.125 / 0.25. FR3 bars = temporal mean +/- SD.')

computed['F_paper_printed']={'ref_mean_m_s':[.379,.186,.095],'ref_printed_plusminus_m_s':[.117,.054,.026],'equilibrium_mean_m':[.00799,.00441,.00310],'equilibrium_printed_plusminus_m':[.00783,.00508,.00443]}
computed['conventions']={'selection':'fr3 profile, representative seed100 and D trial0; D statistics use all30 trials per controller','error':'desired pose minus actual pose','static_reference':'(F_des - measured F/T generalized force)/K on nonzero stiffness axes','no_change':'No controller rerun, curve fitting, amplitude scaling or time warping','differences_from_prior_report':'Prior equilibrium summaries used force_true; this comparison uses force_measured to match the paper. Both F statistics are retained here.','internal_REF':'REF includes integrator states; a large value is not an actual robot velocity.'}
(OUT/'comparison_metrics.json').write_text(json.dumps(computed,indent=2))
(OUT/'sources.json').write_text(json.dumps(dict(pdf=source_path(PDF),pdf_sha256=sha256(PDF),script=source_path(Path(__file__).resolve()),script_sha256=sha256(Path(__file__).resolve()),runtime_source_sha256=manifest['source_sha256'],crops_pdf_points=CROPS,recordings=provenance,d_statistics_recordings=d_sources),indent=2))
comparisons=[OUT/f'{name}_compare.png' for name in ('A','B','B_control','C','D','D_motion','F')]
bundle_files=comparisons+[OUT/'source'/f'paper_{name}.png' for name in CROPS]+[OUT/'comparison_metrics.json',OUT/'sources.json']
with zipfile.ZipFile(OUT/'paper_vs_fr3.zip','w',compression=zipfile.ZIP_DEFLATED) as bundle:
    for path in bundle_files:bundle.write(path,str(path.relative_to(OUT)))
print(json.dumps({'output':str(OUT),'comparisons':[p.name for p in comparisons],'d_trials_per_controller':30},indent=2))
