"""Validate frozen C audit logs, publish all metrics and source-backed figures."""
import argparse
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch
from scipy.integrate import cumulative_trapezoid

COLORS={'admittance':'#77a92f','bidirectional':'#0677ba','hinf':'#d45420'}
LABELS={'admittance':'Admittance','bidirectional':'Bidirectional','hinf':'H-infinity'}


def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))


def metrics(a,s,paper):
    t=a['time'];f=a['force_true'][:,2];error=f-a['force_desired'][:,2]
    obstacle=(t>=13)&(t<19);free=(t>=1)&(t<5);after=t>=1
    eq=a['desired'][:,:2]-a['pose'][:,:2]+a['force_true'][:,:2]/400
    # Integrate the desired impedance equation, avoiding acceleration differences.
    e=a['desired'][:,:2]-a['pose'][:,:2]
    w=np.pi/12;vdes=np.c_[.1*w*np.cos(2*w*t),.15*w*np.cos(w*t)]
    edot=vdes-a['velocity'][:,:2]
    ri=4*(edot-edot[0])+80*(e-e[0])+cumulative_trapezoid(400*e+a['force_true'][:,:2],t,axis=0,initial=0)
    Pe=a['power_environment'];Pc=a['power_compensation'];beta=a['beta']
    Pg=a['power_generation'] if s['controller']=='bidirectional' else Pe
    residual=(1-beta)*(Pg-beta*Pc)
    corrected=residual-(1-beta)*a['tank_mu']
    active=t[a['threshold_contact']>0]
    sample_times=paper['force_'+s['controller']][:,0]
    sampled=np.interp(sample_times,t,f)-10-3*np.sin(sample_times)
    result=dict(force_rmse_N=rms(error),force_measured_rmse_N=rms(a['force_measured'][:,2]-a['force_desired'][:,2]),
        force_rmse_free_1_5_N=rms(error[free]),force_rmse_obstacle_13_19_N=rms(error[obstacle]),
        force_rmse_after_1s_N=rms(error[after]),
        plot_sampled_force_rmse_N=float(np.sqrt(np.trapezoid(sampled**2,sample_times)/(sample_times[-1]-sample_times[0]))),
        force_range_N=[float(f.min()),float(f.max())],saturation_fraction=float(a['saturated'].mean()),
        saturation_by_joint=np.mean(np.abs(a['torque_filtered'])>np.array([87]*4+[12]*3),axis=0).tolist(),
        board_contact_after_1s=float(a['board_contact'][after].mean()),
        threshold_contact_bounds_s=[float(active[0]),float(active[-1])] if len(active) else None,
        xy_equilibrium_gap_rms_13_19_mm=float(np.sqrt(np.mean(np.sum(eq[obstacle]**2,axis=1)))*1000),
        xy_equilibrium_gap_bias_13_19_mm=float(np.linalg.norm(np.mean(eq[obstacle],axis=0))*1000),
        integrated_impedance_residual_xy_rms_Ns=float(np.sqrt(np.mean(np.sum(ri**2,axis=1)))),
        beta_mean_13_19=float(beta[obstacle].mean()),beta_above_09_13_19=float(np.mean(beta[obstacle]>.9)),
        beta_mean=float(beta.mean()),positive_environment_work_J=float(np.trapezoid(np.maximum(Pe,0),t)),
        positive_exact_environment_work_J=float(np.trapezoid(np.maximum(a['power_environment_exact'],0),t)),
        positive_modulated_work_J=float(np.trapezoid(np.maximum(residual,0),t)),
        tank_accounted_residual_max_W=float(corrected.max()),tank_min_J=float(a['tank'].min()),
        compensation_transmitted_fraction=float(np.linalg.norm((1-beta[:,None])*a['compensation_torque'])/max(1e-15,np.linalg.norm(a['compensation_torque']))))
    for label,mask in [('free',free),('obstacle',obstacle)]:
        freq,power=welch(error[mask],fs=1000,nperseg=1000)
        result[f'force_high_frequency_rms_{label}_N']=float(np.sqrt(np.sum(power[freq>=20])*(freq[1]-freq[0])))
    return result,eq,ri


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=Path('outputs/c-mechanism'))
    parser.add_argument('--paper',type=Path,default=Path('outputs/paper-c-digitized'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/c-mechanism'))
    args=parser.parse_args();manifest=json.loads((args.input/'manifest.json').read_text())
    assert not manifest['errors'],manifest['errors']
    assert set(manifest['expected_cases'])==set(manifest['completed'])
    args.output.mkdir(parents=True,exist_ok=True)
    paper=dict(np.load(args.paper/'curves.npz'));paper_meta=json.loads((args.paper/'metrics.json').read_text())
    records={};selected={};hashes={};compact={};all_valid=True
    for name,s in sorted(manifest['completed'].items()):
        folder=args.input/name/f"C_{s['controller']}_trial00_seed{s['seed']}"
        assert json.loads((folder/'summary.json').read_text())==s
        assert s['source_sha256']==manifest['source_sha256']
        hashes[name]=hashlib.sha256((folder/'trajectory.npz').read_bytes()).hexdigest()
        if s['status']!='completed':
            records[name]={'status':s['status'],'termination_reason':s['termination_reason']};all_valid=False;continue
        assert s['completed_control_steps']==round(30/s['control_period_s'])
        assert s['force_samples']==30000
        a={k:v.astype(float) for k,v in np.load(folder/'trajectory.npz').items()}
        assert all(len(v)==30000 and np.isfinite(v).all() for v in a.values()),name
        np.testing.assert_allclose(a['time'],np.arange(30000)*.001,atol=1e-12)
        limits=np.array([87]*4+[12]*3)
        np.testing.assert_array_equal(np.any(abs(a['torque_filtered'])>limits,axis=1),a['saturated'])
        np.testing.assert_allclose(np.clip(a['torque_filtered'],-limits,limits),a['torque_applied'])
        m,eq,ri=metrics(a,s,paper)
        np.testing.assert_allclose(m['force_rmse_N'],s['metrics']['force_tracking_rmse_N'],rtol=2e-6)
        records[name]=m
        if name.startswith('filter_on_') and name.endswith('s100') or name in ('beta_0','beta_1'):
            selected[name]=a;selected[name]['eq']=eq
            for key in ('time','force_true','force_desired','beta','power_environment','power_compensation','power_generation','tank','saturated','pose','desired'):
                compact[f'{name}__{key}']=a[key].astype(np.float64 if key=='time' else np.float32)
    aggregates={}
    for mode in ('on','off'):
        for c in COLORS:
            group=[records[f'filter_{mode}_{c}_s{seed}'] for seed in (100,101,102)]
            aggregates[f'filter_{mode}_{c}']={k:dict(mean=float(np.mean([r[k] for r in group])),min=float(min(r[k] for r in group)),max=float(max(r[k] for r in group)))
                for k in ('force_rmse_N','saturation_fraction','xy_equilibrium_gap_bias_13_19_mm')}
    payload=dict(source_sha256=manifest['source_sha256'],all_runs_completed=all_valid,
        full_paper_performance_reproduced=False,
        definitions=dict(force_rmse='Unfiltered true world Fz minus desired Fz, all 1 kHz logs; wrist measurement RMSE separately provided.',
            plot_sampled='Simulated Fz interpolated at the PDF force-polyline vertex times, trapezoid-weighted. Does not restore unpublished paper samples.',
            equilibrium_gap='norm(xi_xy + Fxy/400), descriptive quasi-static diagnostic, not the complete dynamic impedance equation.',
            integrated_impedance='L*(edot-edot0)+D*(e-e0)+integral(K*e+F-Fdes)dt, true Cartesian x/y, units N s.',
            high_frequency='Welch power in >=20 Hz band at 1 kHz; analysis only, no force filter in the feedback loop.',
            energy='Source-rule observed power. Algebraic cancellation is not a passivity proof for filtered, saturated, noisy, sampled, redundant robot dynamics.'),
        paper=paper_meta,aggregates=aggregates,metrics=records,runs=manifest['completed'],trajectory_sha256=hashes)
    (args.output/'metrics.json').write_text(json.dumps(payload,indent=2))
    np.savez_compressed(args.output/'selected_curves.npz',**compact)
    np.savez_compressed(args.output/'paper_curves.npz',**paper)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for c,col in COLORS.items():
        p=paper['force_'+c];axes[0,0].plot(*p.T,color=col,lw=.55,label=LABELS[c])
        a=selected[f'filter_on_{c}_s100'];t=a['time'];axes[0,1].plot(t,a['force_true'][:,2],color=col,lw=.4,label=LABELS[c])
        tx=np.arange(.01,30,.01)
        gap=np.c_[*[np.interp(tx,*paper[f'error_{ax}_{c}'].T)-np.interp(tx,*paper[f'equilibrium_{ax}_{c}'].T) for ax in 'xy']]
        axes[1,0].plot(tx,1000*np.linalg.norm(gap,axis=1),color=col,lw=.55)
        axes[1,1].plot(t,1000*np.linalg.norm(a['eq'],axis=1),color=col,lw=.45)
    for j,title in enumerate(('Paper Fig. 9: exported vector curves (~100 Hz)','FR3: 1 kHz, original gains, command filter 100 Hz')):
        axes[0,j].set_title(title);axes[0,j].plot(np.arange(0,30,.01),10+3*np.sin(np.arange(0,30,.01)),'k--',lw=1,label='Desired')
        axes[0,j].set(ylabel='Normal force [N]',ylim=(-5,55));axes[0,j].legend(fontsize=8,ncol=2)
        axes[1,j].set(ylabel='XY quasi-static gap norm [mm]')
        axes[2,j].set(ylabel='Beta',ylim=(-.05,1.05),xlabel='Time [s]')
    gap_peak=max(np.max(line.get_ydata()) for ax in axes[1] for line in ax.lines)
    for ax in axes[1]:ax.set_ylim(0,np.ceil(gap_peak/20)*20)
    a=selected['filter_on_bidirectional_s100']
    for j,(bt,bv) in enumerate((paper['beta'].T,(a['time'],a['beta']))):
        axes[2,j].plot(bt,bv,color='#555555',lw=.4,alpha=.35,label='Exported polyline' if j==0 else '1 kHz samples')
        centers=np.arange(.25,30,.5);means=[]
        for center in centers:
            grid=np.linspace(center-.25,center+.25,501)
            means.append(np.trapezoid(np.interp(grid,bt,bv),grid)/.5)
        axes[2,j].plot(centers,means,color='#b75413',lw=1.6,label='0.5 s mean of displayed curve')
        axes[2,j].legend(fontsize=8,loc='upper right')
    for ax in axes.flat:ax.axvspan(13,19,color='#a7c7d7',alpha=.12);ax.set_xlim(0,30)
    fig.savefig(args.output/'C_paper_comparison.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for c,col in COLORS.items():
        vals=[aggregates[f'filter_{mode}_{c}']['force_rmse_N']['mean'] for mode in ('on','off')]
        off=list(COLORS).index(c)*.23-.23
        axes[0,0].bar(np.arange(2)+off,vals,width=.23,color=col,label=LABELS[c])
        vals=[100*aggregates[f'filter_{mode}_{c}']['saturation_fraction']['mean'] for mode in ('on','off')]
        axes[0,1].bar(np.arange(2)+off,vals,width=.23,color=col)
    axes[0,0].set(ylabel='Force RMSE [N]',title='Three paired noise seeds; original gains');axes[0,0].legend(fontsize=8)
    axes[0,1].set(ylabel='Clipped command samples [%]',title='Same 1 kHz clock and environment')
    for ax in axes[0]:ax.set_xticks([0,1],['100 Hz filter ON','Filter OFF'])
    names=['beta_0','filter_on_bidirectional_s100','beta_1'];labs=['Fixed beta=0','Adaptive beta','Fixed beta=1']
    axes[1,0].bar(['Beta=0','Adaptive','Beta=1','Beta=1 ideal'],
        [records[n]['force_rmse_N'] for n in [*names,'beta_1_ideal']],
        color=['#999999',COLORS['bidirectional'],'#777777','#77a92f'])
    axes[1,0].set(ylabel='Force RMSE [N]',title='Fixed beta runs are diagnostic ablations')
    for n,label in zip(names,labs):
        a=selected[n];t=a['time'];Pe=a['power_generation'];Pc=a['power_compensation'];b=a['beta']
        axes[1,1].plot(t,cumulative_trapezoid(np.maximum((1-b)*(Pe-b*Pc),0),t,initial=0),label=label)
    axes[1,1].set(xlabel='Time [s]',ylabel='Observed positive residual work [J]',title='Observer term; not total physical energy');axes[1,1].legend(fontsize=8)
    fig.savefig(args.output/'C_mechanism.png',dpi=160);plt.close(fig)
    print(json.dumps({'runs':len(records),'all_completed':all_valid,'aggregates':aggregates},indent=2))


if __name__=='__main__':main()
