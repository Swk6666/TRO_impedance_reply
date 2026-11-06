"""Validate saved runs and render standalone scientific comparison figures.

All calculations use saved arrays; failed/short runs remain in the inventory.
Run: python -m fr3imp.analyze outputs/suite --output docs/results
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .scenarios import COMPARISONS
from .dynamics import euler_rate_map

COLORS={'impedance':'#b85a17','admittance':'#2479ad','bidirectional':'#aa4488',
        'smc':'#977412','hinf':'#17845a','hinf_0.25':'#7764ad',
        'hinf_0.5':'#2479ad','hinf_1':'#17845a'}
LABELS={'impedance':'Impedance','admittance':'Admittance','bidirectional':'Bidirectional',
        'smc':'Sliding mode','hinf':r'$H_\infty$','hinf_0.25':r'$\alpha=0.25$',
        'hinf_0.5':r'$\alpha=0.5$','hinf_1':r'$\alpha=1$'}
STYLES={'impedance':':','admittance':'--','bidirectional':'-.','smc':'--',
        'hinf':'-','hinf_0.25':':','hinf_0.5':'--','hinf_1':'-'}


def read_runs(root):
    manifest=json.loads((root/'manifest.json').read_text())
    expected={(e,c,i) for e in manifest['experiments'] for c in COMPARISONS[e]
              for i in range(manifest['d_trials'] if e=='D' else 1)}
    runs=[];seen=set();checks=[]
    for entry in manifest['completed']:
        key=(entry['experiment'],entry['controller'],entry['trial'])
        if key in seen:raise ValueError(f'Duplicate run {key}')
        seen.add(key)
        folder=root/f'{key[0]}_{key[1]}_trial{key[2]:02d}_seed{entry["seed"]}'
        summary=json.loads((folder/'summary.json').read_text())
        if summary != entry:raise ValueError(f'Manifest/summary disagreement {folder}')
        if summary['source_sha256'] != manifest['source_sha256']:
            raise ValueError(f'Mixed runtime versions {folder}')
        with np.load(folder/'trajectory.npz') as file:
            # Plotting needs only representative trajectories; all runs are
            # independently checked before discarding their larger arrays.
            arrays={k:file[k] for k in file.files}
        t=arrays['time'];n=len(t)
        if not n:raise ValueError(f'No logged samples {folder}')
        if any(len(a)!=n or not np.isfinite(a).all() for a in arrays.values()):
            raise ValueError(f'Invalid arrays {folder}')
        np.testing.assert_allclose(np.diff(t),.001,atol=1e-10,rtol=0)
        if summary['status']=='completed':
            np.testing.assert_allclose(summary['simulated_duration_s'],summary['requested_duration_s'],atol=1e-8)
            if n != round(summary['requested_duration_s']/.001):
                raise ValueError(f'Incomplete sampling {folder}')
        # Deliberately independent of metrics_from_log (different expression).
        squared=(arrays['pose'][:,:3].astype(float)-arrays['desired'][:,:3])**2
        recomputed=np.sqrt(squared.sum()/n)
        np.testing.assert_allclose(recomputed,summary['metrics']['position_rmse_m'],rtol=2e-6,atol=1e-10)
        peak=np.linalg.norm(arrays['force_true'][:,:3].astype(float),axis=1).max()
        np.testing.assert_allclose(peak,summary['metrics']['force_peak_N'],rtol=2e-6,atol=1e-8)
        checks.append(dict(run=folder.name,samples=n,position_rmse_recomputed_m=float(recomputed),
                           force_peak_recomputed_N=float(peak)))
        runs.append((summary,arrays if key[2]==0 else None))
    if seen!=expected:raise ValueError(f'Missing/unexpected runs: {expected-seen}, {seen-expected}')
    if manifest['errors']:raise ValueError(f'Worker errors: {manifest["errors"]}')
    return manifest,runs,checks


def line(ax,t,y,controller,**kwargs):
    ax.plot(t,y,color=COLORS[controller],ls=STYLES[controller],lw=1.15,
            label=LABELS[controller],**kwargs)


def make_figure(experiment,runs,output):
    selected=[(s,a) for s,a in runs if s['experiment']==experiment and a is not None]
    fig,axes=plt.subplots(2,2,figsize=(11.5,7.2),layout='constrained')
    ax=axes.ravel()
    for summary,a in selected:
        c=summary['controller'];t=a['time'];skip=max(1,len(t)//6000);sl=slice(None,None,skip)
        err=a['desired']-a['pose']
        if experiment in 'ABCF':
            line(ax[0],a['pose'][sl,0]*1000,a['pose'][sl,1]*1000,c)
            line(ax[1],t[sl],np.linalg.norm(err[sl,:3],axis=1)*1000,c)
            if experiment=='C':
                line(ax[2],t[sl],a['force_true'][sl,2],c)
            elif experiment in 'BF':
                line(ax[2],t[sl],a['force_true'][sl,0],c)
            else:
                line(ax[2],t[sl],np.linalg.norm(a['reference_error'][sl,:3],axis=1),c)
            line(ax[3],t[sl],np.linalg.norm(a['reference_error'][sl,3:],axis=1),c)
        elif experiment=='D':
            for i,axis in enumerate((3,4,5)):
                line(ax[i],t[sl],np.rad2deg(a['pose'][sl,axis]),c)
            line(ax[3],t[sl],a['force_true'][sl,3],c)
        else:
            line(ax[0],t[sl],a['pose'][sl,0]*1000,c)
            line(ax[1],t[sl],a['pose'][sl,2]*1000,c)
            line(ax[2],t[sl],a['force_true'][sl,2],c)
            line(ax[3],t[sl],np.linalg.norm(a['reference_error'][sl,:3],axis=1),c)
        if summary['status']!='completed':
            for axis in ax[1:]:axis.axvline(t[-1],color=COLORS[c],ls=':',alpha=.5)
    first=selected[0][1]
    if experiment in 'ABCF':
        ax[0].plot(first['desired'][:,0]*1000,first['desired'][:,1]*1000,'k--',lw=.9,label='Desired')
        ax[0].set(xlabel='World x [mm]',ylabel='World y [mm]',aspect='equal')
        ax[1].set(ylabel='Position error norm [mm]')
        ax[2].set(ylabel=('Normal force Fz [N]' if experiment=='C' else
                         'Wall force Fx [N]' if experiment in 'BF' else 'Translational REF norm [m/s]'))
        ax[3].set(ylabel='Rotational REF norm [rad/s]')
        if experiment=='C':ax[2].plot(first['time'],first['force_desired'][:,2],'k--',lw=1,label='Desired force')
    elif experiment=='D':
        for axis,label in zip(ax,['Roll [deg]','Pitch [deg]','Yaw [deg]','Applied roll moment [Nm]']):axis.set(ylabel=label)
        ax[0].axhline(90,color='black',ls=':',lw=.8,label='90 degree target')
        for axis in ax:axis.axvspan(11,20,color='#999999',alpha=.10)
    else:
        for i,k in [(0,0),(1,2)]:
            ax[i].plot(first['time'],first['desired'][:,k]*1000,'k--',lw=1,label='Desired')
        for axis,label in zip(ax,['World x [mm]','World z [mm]','Total true Fz [N]','Translational REF norm [m/s]']):axis.set(ylabel=label)
    for i,axis in enumerate(ax):
        axis.grid(alpha=.18)
        if experiment not in 'ABCF' or i:axis.set(xlabel='Time [s]',xlim=(0,35 if experiment=='D' else 30))
    ax[0].legend(fontsize=9,loc='best')
    if experiment=='C':ax[2].legend(fontsize=9)
    title={'A':'Free motion','B':'Rigid obstacle','C':'Hybrid force/motion',
           'D':'Rotation with simulated operator (representative trial 0)',
           'E':'Grinding-equivalent contact and vibration','F':'REF gain sweep'}[experiment]
    fig.suptitle(f'{experiment}: {title} — FR3 / MuJoCo',fontsize=14)
    fig.savefig(output/f'{experiment}.png',dpi=160)
    plt.close(fig)


def make_d_statistics(runs,output):
    fig,axes=plt.subplots(1,3,figsize=(12,4.3),layout='constrained')
    controllers=COMPARISONS['D']
    for ax,metric,title,scale in zip(axes,
        ['pitch_equilibrium_error_mae_rad','yaw_equilibrium_error_mae_rad','maximum_roll_rad'],
        ['Pitch equilibrium discrepancy','Yaw equilibrium discrepancy','Maximum roll'],[180/np.pi]*3):
        for i,c in enumerate(controllers):
            values=np.array([s['metrics'][metric] for s,_ in runs if s['experiment']=='D' and s['controller']==c])*scale
            if not len(values):continue
            std=values.std(ddof=1) if len(values)>1 else 0.
            ax.bar(i,values.mean(),yerr=std,color=COLORS[c],width=.65,capsize=4)
            ax.scatter(i+np.linspace(-.14,.14,len(values)),values,color='black',s=8,alpha=.4)
        ax.set_xticks(range(4),[LABELS[c] for c in controllers],rotation=20,ha='right')
        ax.set(title=title,ylabel='Degrees',ylim=(0,None));ax.grid(axis='y',alpha=.2)
    fig.suptitle('D: virtual-operator trials; bars = mean, whiskers = sample SD (not confidence intervals)')
    fig.savefig(output/'D_statistics.png',dpi=160);plt.close(fig)


def make_d_tracking(runs,output):
    fig,axes=plt.subplots(2,4,figsize=(14,6),sharex=True,sharey='row',layout='constrained')
    for col,c in enumerate(COMPARISONS['D']):
        a=next(a for s,a in runs if s['experiment']=='D' and s['controller']==c and s['trial']==0)
        sl=slice(None,None,5);t=a['time'][sl];p=a['pose'][sl]
        generalized=np.array([euler_rate_map(q[3:]).T@f[3:] for q,f in zip(p,a['force_true'][sl])])
        for row,axis,K in [(0,4,200),(1,5,10)]:
            axes[row,col].plot(t,np.rad2deg(p[:,axis]),color=COLORS[c],label='Actual angle')
            axes[row,col].plot(t,np.rad2deg(generalized[:,axis-3]/K),'k--',lw=.9,label='Static F/K reference')
            axes[row,col].axvspan(11,20,color='#999999',alpha=.1)
            axes[row,col].grid(alpha=.2)
            axes[row,col].set(xlim=(0,35),xlabel='Time [s]')
        axes[0,col].set_title(LABELS[c])
    axes[0,0].set_ylabel('Pitch [deg]');axes[1,0].set_ylabel('Yaw [deg]')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('D: actual angles and static equilibrium reference (dynamic phases also include inertia/damping)')
    fig.savefig(output/'D_tracking.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,default=Path('docs/results'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    manifest,runs,checks=read_runs(args.root)
    for name in manifest['experiments']:make_figure(name,runs,args.output)
    if 'D' in manifest['experiments']:
        make_d_statistics(runs,args.output)
        make_d_tracking(runs,args.output)
    payload=dict(source_sha256=manifest['source_sha256'],input_root=str(args.root),
                 expected_runs=manifest['expected_runs'],status_counts=dict(Counter(s['status'] for s,_ in runs)),
                 validation='All saved runs checked for membership, length, time spacing, finite arrays; position RMSE and peak force independently recomputed.',
                 checks=checks,runs=[s for s,_ in runs])
    (args.output/'metrics.json').write_text(json.dumps(payload,indent=2))
    rows=['# FR3 仿真结果','',f'运行数：{len(runs)}；状态：{payload["status_counts"]}。',
          f'控制代码 SHA-256：`{manifest["source_sha256"]}`。','',
          '来源为本项目生成的 FR3/MuJoCo 轨迹；不是论文 Indy7 实验数据。D 的重复是虚拟操作者参数与噪声种子变化，不是真人样本。',
          '提前终止运行的指标仅涵盖终止前，不能与完整运行直接排名。完整参数和逐运行检查见 [metrics.json](metrics.json)。','',
          '|组|控制器|次数|完整完成|位置误差 RMS (mm)|参考误差 RMS (m/s)|采样峰值力 (N)|饱和样本比例|',
          '|---|---|---:|---:|---:|---:|---:|---:|']
    for e in manifest['experiments']:
        for c in COMPARISONS[e]:
            ss=[s for s,_ in runs if s['experiment']==e and s['controller']==c]
            def mean(k):return float(np.mean([s['metrics'][k] for s in ss]))
            rows.append(f'|{e}|{c}|{len(ss)}|{sum(s["status"]=="completed" for s in ss)}|{mean("position_rmse_m")*1000:.3f}|{mean("reference_error_position_rms_m_s"):.4f}|{max(s["metrics"]["force_peak_N"] for s in ss):.2f}|{mean("saturation_fraction"):.2%}|')
    rows+=['','位置误差在接触任务中包含期望的柔顺位移，不宜单独作为阻抗好坏的排名。REF 使用控制器收到的带噪测量；静态平衡偏差也包含动态阶段的惯性和阻尼效应。','',
           '图中轨迹来自 trial 0，每隔最多 5–6 ms 取点绘图；指标使用保存的全部 1 kHz 样本，未平滑。力峰值是已保存样本的最大值，不是所有 4 kHz 控制时刻或物理子步的全局最大值；饱和比例也是 1 kHz 样本估计。','']
    for e in manifest['experiments']:rows += [f'## {e}',f'![{e}]({e}.png)','']
    if 'D' in manifest['experiments']:rows += ['![D statistics](D_statistics.png)','', 'D 统计图的误差棒是运行间标准差，不是置信区间。每三个运行共享同一虚拟操作者配置，不能视为 30 个独立真人。','', '![D tracking](D_tracking.png)']
    (args.output/'RESULTS.md').write_text('\n'.join(rows)+'\n')
    print(json.dumps(dict(runs=len(runs),status_counts=payload['status_counts'],output=str(args.output))))


if __name__=='__main__':main()
