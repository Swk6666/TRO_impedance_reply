"""Validate the active reproduction scope and report supported/missing effects."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter,sosfiltfilt,cont2discrete,dlsim

from fr3imp.analyze import (read_runs,make_figure,make_d_statistics,make_d_tracking,
                           COLORS,LABELS)
from fr3imp.evaluation import impedance_metrics
from fr3imp.model import TORQUE_LIMIT
from fr3imp.scenarios import Scenario,COMPARISONS
from validate_reproduction import cases


def extra_metrics(s,a):
    t=a['time'];sat=a['saturated'].astype(bool)
    edges=np.diff(np.r_[False,sat,False].astype(int))
    widths=np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)
    result=dict(longest_saturation_s=float(widths.max()*.001) if len(widths) else 0.)
    if s['experiment'] in ('B','F') and len(t)>100:
        ripple=sosfiltfilt(butter(3,20,fs=1000,output='sos',btype='high'),a['force_true'][:,:3],axis=0)
        mask=(t>=13)&(t<18)
        if mask.any():result['wall_force_ripple_rms_N']=float(np.sqrt(np.mean(np.sum(ripple[mask]**2,axis=1))))
    if s['experiment']=='C' and len(t)>100:
        result['measured_force_tracking_rmse_N']=float(np.sqrt(np.mean((a['force_measured'][:,2]-a['force_desired'][:,2])**2)))
        ripple=sosfiltfilt(butter(3,20,fs=1000,output='sos',btype='high'),a['force_true'][:,2])
        for label,lo,hi in [('free_board',1,12),('threshold',13,19)]:
            mask=(t>=lo)&(t<hi)
            if mask.any():result[label+'_force_ripple_rms_N']=float(np.sqrt(np.mean(ripple[mask]**2)))
    if s['experiment']=='D':
        # Independently solve the two rotational second-order systems. This
        # avoids judging transient response only against the static F/K line.
        p=a['pose'].astype(float);f=a['force_true'].astype(float)
        generalized=np.column_stack((-np.sin(p[:,5])*f[:,3]+np.cos(p[:,5])*f[:,4],f[:,5]))
        for j,(axis,label) in enumerate([(4,'pitch'),(5,'yaw')]):
            L,D,K=(s['impedance'][k][axis] for k in ('inertia','damping','stiffness'))
            sys=cont2discrete(tuple(np.asarray(x,dtype=float) for x in
                ([[0,1],[-K/L,-D/L]],[[0],[1/L]],[[1,0]],[[0]])),.001)
            _,desired,_=dlsim(sys,-generalized[:,j],x0=[-p[0,axis],-a['velocity'][0,axis]])
            gap=-p[:,axis]-desired[:,0]
            result[label+'_dynamic_rmse_deg']=float(np.rad2deg(np.sqrt(np.mean(gap**2))))
    return result


def check_record(s,folder,version):
    if s['source_sha256']!=version:raise ValueError(f'Mixed source versions: {folder}')
    with np.load(folder/'trajectory.npz') as f:a={k:f[k] for k in f.files}
    t=a['time'];n=len(t)
    if not n or any(len(v)!=n or not np.isfinite(v).all() for v in a.values()):
        raise ValueError(f'Invalid/empty arrays: {folder}')
    np.testing.assert_allclose(np.diff(t),.001,rtol=0,atol=1e-10)
    if s['status']=='completed':
        if n!=round(s['requested_duration_s']*1000):raise ValueError(f'Short run: {folder}')
        if s['force_samples']!=n:raise ValueError(f'Wrong F/T clock: {folder}')
        if s['completed_control_steps']!=n:raise ValueError(f'Wrong control clock: {folder}')
    # Independent checks against arrays, not a re-run of the summary function.
    error=a['pose'][:,:3].astype(float)-a['desired'][:,:3]
    np.testing.assert_allclose(np.linalg.norm(error)/np.sqrt(n),s['metrics']['position_rmse_m'],rtol=3e-6,atol=1e-9)
    peak=np.sqrt((a['force_true'][:,:3].astype(float)**2).sum(1)).max()
    np.testing.assert_allclose(peak,s['metrics']['force_peak_N'],rtol=3e-6,atol=1e-8)
    np.testing.assert_allclose(a['torque_applied'],np.clip(a['torque_filtered'],-TORQUE_LIMIT,TORQUE_LIMIT),atol=1e-5)
    np.testing.assert_allclose(a['q'][0],s['initial_q'],atol=1e-7)
    if s['controller'] in ('admittance','bidirectional'):
        np.testing.assert_allclose(a['proxy_q'][0],s['initial_q'],atol=1e-7)
    surely_clipped=np.any(abs(a['torque_filtered'])>TORQUE_LIMIT+1e-5,axis=1)
    if not np.all(a['saturated'][surely_clipped]==1):raise ValueError(f'Missing saturation flags: {folder}')
    if s.get('tank_depletion_count',0):raise ValueError(f'Energy-tank depletion: {folder}')
    record=dict(summary=s,trajectory_sha256=hashlib.sha256((folder/'trajectory.npz').read_bytes()).hexdigest(),
                samples=n,diagnostics=extra_metrics(s,a))
    return record,a


def interaction_figure(e,selected,output):
    columns=3 if e=='C' else 2
    fig,axes=plt.subplots(len(COMPARISONS[e]),columns,figsize=(12,2.3*len(COMPARISONS[e])),
                          squeeze=False,sharex=True,layout='constrained')
    for row,c in enumerate(COMPARISONS[e]):
        s,a=selected[e,c];scenario=Scenario(e,a['desired'][0],variant=s['scenario_variant'],impedance_overrides=s.get('impedance'))
        _,dynamic,equilibrium=impedance_metrics(a,scenario)
        error=a['desired']-a['pose'];t=a['time']
        for col,axis in enumerate((0,1)):
            ax=axes[row,col]
            ax.plot(t,(equilibrium[:,axis]-error[:,axis])*-1000,color='#888888',lw=.45,label='Static -F/K')
            ax.plot(t,dynamic[:,axis]*1000,color='#3182bd',ls='--',lw=1,label='Full desired dynamics')
            ax.plot(t,error[:,axis]*1000,color=COLORS[c],lw=.9,label='Actual error')
            ax.set(ylabel=f'{LABELS[c]}\n'+('x' if col==0 else 'y')+' error [mm]')
            ax.axvspan(13,19 if e=='C' else 18,color='black',alpha=.04)
        if e=='C':
            ax=axes[row,2]
            ax.plot(t,a['force_measured'][:,2],color='#bbbbbb',lw=.4,label='Measured Fz')
            ax.plot(t,a['force_true'][:,2],color=COLORS[c],lw=.6,label='Physical Fz')
            ax.plot(t,a['force_desired'][:,2],'k--',lw=1,label='Desired Fz')
            ax.set(ylabel='Normal force [N]')
        for ax in axes[row]:ax.grid(alpha=.15);ax.set(xlim=(0,30),xlabel='Time [s]')
    for ax in axes[0]:ax.legend(fontsize=8)
    fig.suptitle(f'{e}: physical impedance response, including the intended compliant displacement')
    fig.savefig(output/f'{e}_impedance.png',dpi=150);plt.close(fig)


def paper_c_figure(selected,paper_file,output):
    if not paper_file.exists():return
    paper=np.load(paper_file)
    # Share a complete range; the FR3 chatter peaks exceed the source figure's
    # original 60 N upper limit and must not disappear beyond the plot border.
    force_arrays=[selected['C',c][1][k][:,2] for c in COMPARISONS['C'] for k in ('force_true','force_measured')]
    force_arrays += [paper['force_'+c][:,1] for c in COMPARISONS['C']]
    lower=10*np.floor(min(-10.,min(float(v.min()) for v in force_arrays))/10)
    upper=10*np.ceil(max(60.,max(float(v.max()) for v in force_arrays))/10)
    fig,axes=plt.subplots(2,3,figsize=(13,6),sharex=True,sharey=True,layout='constrained')
    for col,c in enumerate(COMPARISONS['C']):
        _,a=selected['C',c];p=paper['force_'+c]
        axes[0,col].plot(p[:,0],p[:,1],color=COLORS[c],lw=.6)
        axes[0,col].plot(p[:,0],10+3*np.sin(p[:,0]),'k--',lw=1)
        axes[1,col].plot(a['time'],a['force_true'][:,2],color='#bbbbbb',lw=.4,label='Physical force')
        axes[1,col].plot(a['time'],a['force_measured'][:,2],color=COLORS[c],lw=.6,label='Measured F/T')
        axes[1,col].plot(a['time'],a['force_desired'][:,2],'k--',lw=1)
        axes[0,col].set_title(LABELS[c])
        for row in range(2):
            axes[row,col].axvspan(13,19,color='black',alpha=.05)
            axes[row,col].set(xlim=(0,30),ylim=(lower,upper),xlabel='Time [s]');axes[row,col].grid(alpha=.15)
    axes[0,0].set_ylabel('Paper plotted F/T force [N]')
    axes[1,0].set_ylabel('FR3 measured F/T force [N]')
    axes[1,0].legend(fontsize=8,loc='upper right')
    fig.suptitle('C: source drawing versus simulation (different robots and sampling; not author raw logs)')
    fig.savefig(output/'C_paper.png',dpi=160);plt.close(fig)


def gain_figure(selected,output):
    fr3=selected['F','hinf_1'][0].get('profile')=='fr3'
    fig,axes=plt.subplots(1,2,figsize=(10,4.4),layout='constrained')
    for ax,metric,title,unit in zip(axes,
        ['reference_error_position','equilibrium_position_error'],
        ['Translational REF norm','Quasi-static equilibrium discrepancy'],['m/s','m']):
        lower_bounds=[0.]
        for i,c in enumerate(COMPARISONS['F']):
            m=selected['F',c][0]['metrics'];suffix='m_s' if unit=='m/s' else 'm'
            mean=m[f'{metric}_mean_{suffix}'];std=m[f'{metric}_std_{suffix}']
            lower_bounds.append(mean-std)
            ax.bar(i,mean,yerr=std,color=COLORS[c],capsize=4)
            ax.text(i,mean+std,f'{mean:.5f}',ha='center',va='bottom',fontsize=9)
        labels=[f"{c.split('_')[1]}\n(eff. {selected['F',c][0]['controller_parameters']['alpha']:g})" for c in COMPARISONS['F']] if fr3 else ['0.25','0.5','1']
        ax.set(xticks=range(3),xticklabels=labels,xlabel='Gain relative to FR3 base (effective alpha)' if fr3 else 'REF gain alpha',ylabel=unit,title=title,ylim=(min(lower_bounds)*1.1,None))
        ax.margins(y=.2);ax.grid(axis='y',alpha=.15)
    fig.suptitle('F: mean and temporal SD; '+('FR3 base alpha=0.25' if fr3 else 'paper gain parameters'))
    fig.savefig(output/'F_gain.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/reproduction-fr3'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/reproduction-fr3'))
    parser.add_argument('--paper-c',type=Path,default=Path('docs/results/c-mechanism/paper_curves.npz'))
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    manifest,runs,checks=read_runs(args.root/'main');version=manifest['source_sha256']
    experiments=manifest['experiments'];has_grinding='E' in experiments
    profile=manifest.get('profile','paper')
    if not set('ABCDF').issubset(experiments):raise ValueError('This report requires A/B/C/D/F')
    selected={(s['experiment'],s['controller']):(s,a) for s,a in runs if a is not None}
    records={}
    for s,_ in runs:
        name=f"{s['experiment']}_{s['controller']}_trial{s['trial']:02d}_seed{s['seed']}"
        records[name],_=check_record(s,args.root/'main'/name,version)
    sensitivity=json.loads((args.root/'sensitivity/manifest.json').read_text())
    if sensitivity['errors'] or set(sensitivity['completed'])!=set(cases(experiments,profile)):raise ValueError('Incomplete sensitivity matrix')
    extras={}
    for name,s in sensitivity['completed'].items():
        folder=args.root/'sensitivity'/name/f"{s['experiment']}_{s['controller']}_trial00_seed{s['seed']}"
        if json.loads((folder/'summary.json').read_text())!=s:raise ValueError(f'Manifest mismatch: {folder}')
        extras[name],_=check_record(s,folder,version)
    for e in experiments:make_figure(e,runs,out)
    make_d_statistics(runs,out);make_d_tracking(runs,out)
    for e in 'BC':interaction_figure(e,selected,out)
    paper_c_figure(selected,args.paper_c,out);gain_figure(selected,out)
    payload=dict(source_sha256=version,main=records,sensitivity=extras,independent_checks=checks)
    (out/'metrics.json').write_text(json.dumps(payload,indent=2))
    def metric(e,c,k):return selected[e,c][0]['metrics'][k]
    def fmt(e,c,k,scale=1):return f'{metric(e,c,k)*scale:.3f}'
    states=Counter(r['summary']['status'] for r in [*records.values(),*extras.values()])
    rows=['# FR3 '+('A–F' if has_grinding else 'A、B、C、D、F')+'：1 kHz 修订与验证','',
          f'正式矩阵 {len(records)} 次（D 每控制器 {manifest["d_trials"]} 次），固定敏感性矩阵 {len(extras)} 次；状态：{dict(states)}。',
          f'运行时代码 SHA256：`{version}`。','',
          '**这些是 FR3 仿真结果。A/F 的主要趋势已出现；B–E 仍有明确的论文差距，不能称为完整复现原机性能。**',
          '用户确认将工具质量、接触材料和命令滤波作为明确仿真假设保留。默认控制、关节测量和 F/T 都是 1 kHz，物理 250 μs，公共命令低通 100 Hz。D 是虚拟操作者，E 使用内部偏心转子和旋转圆盘接触，不是真人/材料去除实验。',
          '[E 的 36 次机制验证](../grinding/REPORT.md) · [B/C/D 的 48 次惯量与握持对照](../portability/REPORT.md) · [上一版 202 次记录](../reproduction/REPORT.md)。','',
          '## 结果与剩余差距','',
          '|组|当前证据|与论文的差距|','|---|---|---|',
          f'|A|普通阻抗/导纳/H∞ 位置 RMS：{fmt("A","impedance","position_rmse_m",1000)} / {fmt("A","admittance","position_rmse_m",1000)} / {fmt("A","hinf","position_rmse_m",1000)} mm；低增益下两种鲁棒控制显著优于普通阻抗|FR3 与 Indy7 的绝对误差不相同|',
          '|B|斜墙、约 13–18 s 接触；H∞/SMC 稳定适应，普通阻抗偏差大，双向调制损失阻抗精度|普通导纳没有复现论文的失稳急停；不能把它写成失败|',
          f'|C|导纳/双向/H∞ 的法向力 RMS：{fmt("C","admittance","force_tracking_rmse_N")} / {fmt("C","bidirectional","force_tracking_rmse_N")} / {fmt("C","hinf","force_tracking_rmse_N")} N|普通导纳主要在挡块段抖振，没有源图的全程抖振；幅度与误差比仍不同|',
          '|D|恢复正负旋转、握持/释放、约 90° 翻转，30 次统计；H∞ 保持期望旋转响应|普通导纳和 H∞ 的差别仍小于论文；普通阻抗受限幅和滚转漂移影响|',
          (f'|E|H∞ 接触段 x RMS {fmt("E","hinf","tangential_tracking_rmse_m",1000)} mm，双向 {fmt("E","bidirectional","tangential_tracking_rmse_m",1000)} mm；H∞ 接触丢失 {fmt("E","hinf","contact_loss_fraction_after_6s",100)}%；SMC REF 更大|内部转子产生自由空间测力波动；双向 β/高力与 SMC 明显反弹尚未匹配|' if has_grinding else '|E|依用户指示暂缓|不纳入当前复现及完成条件|'),
          f'|F|α=.25/.5/1 的 REF 均值：{fmt("F","hinf_0.25","reference_error_position_mean_m_s")} / {fmt("F","hinf_0.5","reference_error_position_mean_m_s")} / {fmt("F","hinf_1","reference_error_position_mean_m_s")} m/s，接近翻倍增益、误差减半|绝对幅值随机器人摩擦与惯量变化|','',
          '## 修复了什么','',
          '- 新 `paper` 配置统一接入已验证的 100 Hz 命令接口、物理外力能量端口和 RK4 阶段耦合；保留 `legacy` 重跑旧配置。论文 α=1、NRIC G 和 Λ/D/K 保留，F 单独改变 α。',
          '- 从 PDF 矢量图恢复 A/B/F 的 260×260 mm 轨迹和斜墙，C 保留 100×300 mm 轨迹与转角挡块。A/B/F 改用可达的 FR3 起始姿态，真实/名义/零空间状态一致，避免直接扩大轨迹导致伸直奇异。',
          '- B/E/F 共用有限接触柔顺性，约 26 N / 0.7 mm，明显高于任务刚度；并非原机材料标定。C 为更软笔尖。所有控制器使用同一环境。',
          '- D 使用显式有限刚度虚拟握持，物理力矩与欧拉广义力矩保持功率对偶；E 恢复约 70 mm 接近行程及表面以下 8 mm 目标。',
          '- E 的 0.4 kg 点接触工具改为 1.8 kg 工具、100 mm 旋转圆盘和内部偏心转子。自由空间真实外力与工具产生的 F/T 振动分开，摩擦作用于圆盘，显式校验耗散功率与摩擦上限。名义控制器仍只有七轴，完整八轴状态另存供回放。',
          '- 增加独立的完整阻抗微分方程响应、接触状态与持续限幅指标。F 使用论文的均值±时间标准差；接触下的位置偏移不再单独作为控制好坏判据。力图保留 1 kHz 样本，避免绘图抽样掩盖振荡。','',
          '## 正式结果','',
          '|组|控制器|位置 RMS / mm|动态阻抗位置偏差 / mm|REF 均值 / m·s⁻¹|采样峰值力 / N|限幅比例|',
          '|---|---|---:|---:|---:|---:|---:|']
    if not has_grinding:
        rows=[r for r in rows if not r.startswith(('- E 的','[E 的'))]
        rows=[r.replace('B–E 仍有明确的论文差距','B–D 仍有明确的论文差距')
              .replace('D 是虚拟操作者，E 使用内部偏心转子和旋转圆盘接触，不是真人/材料去除实验。','D 是虚拟操作者；E 依 AGENTS.md 暂缓。')
              .replace('；E 恢复约 70 mm 接近行程及表面以下 8 mm 目标。','。') for r in rows]
        rows.insert(2,'当前范围由 [AGENTS.md](../../../AGENTS.md) 明确为 A/B/C/D/F；仅汇总该范围，旧 E 记录保留。')
    if profile=='fr3':
        from report_fr3_profile import overview,write_evidence
        rows=overview(manifest,records,extras,selected)
        write_evidence(records,extras,selected,out)
    for e in experiments:
        for c in COMPARISONS[e]:
            ss=[s for s,_ in runs if s['experiment']==e and s['controller']==c and s['status']=='completed']
            if not ss:
                rows.append(f'|{e}|{c}|无完整运行|—|—|—|—|');continue
            def mean(k):return np.mean([s['metrics'][k] for s in ss])
            rows.append(f'|{e}|{c}|{mean("position_rmse_m")*1000:.3f}|{mean("dynamic_impedance_position_rmse_m")*1000:.3f}|{mean("reference_error_position_mean_m_s"):.5f}|{max(s["metrics"]["force_peak_N"] for s in ss):.2f}|{mean("saturation_fraction"):.3%}|')
    rows+=['','动态偏差由 1 kHz 记录的真实外力零阶保持驱动 `Λ ë + D ė + K e = Fdes − Fext` 独立积分得到，包含惯性/阻尼，而静态 F/K 关系仅用于描述。角度力矩采用功率对偶变换。峰值只涵盖保存的 1 kHz 样本。REF 包含误差积分，不是末端实际速度；普通阻抗不使用此反馈，不能把其积分积累当作机器人高速运动。',
           'C 表格评价真实接触力；论文对照图则使用 F/T 测量通道，避免把测量值与物理真值混为一谈。测量包含噪声和工具惯性，相关 RMS 单独保存在 metrics.json 的 diagnostics。','',
           '## 固定敏感性检查','',
           '|案例|状态|位置 RMS / mm|REF 均值 / m·s⁻¹|限幅比例|最长连续限幅 / ms|',
           '|---|---|---:|---:|---:|---:|']
    for name,r in sorted(extras.items()):
        s=r['summary'];m=s['metrics']
        rows.append(f'|{name}|{s["status"]}|{m["position_rmse_m"]*1000:.3f}|{m["reference_error_position_mean_m_s"]:.5f}|{m["saturation_fraction"]:.3%}|{r["diagnostics"]["longest_saturation_s"]*1000:.1f}|')
    deltas=[]
    for name,r in extras.items():
        if name.startswith('physics8'):
            s=r['summary'];e=s['experiment'];c=s['controller']
            key='force_tracking_rmse_N' if e=='C' else 'reference_error_position_mean_m_s'
            deltas.append((name,(s['metrics'][key]/metric(e,c,key)-1)*100))
    largest=max(deltas,key=lambda item:abs(item[1]))
    rows+=['','固定种子 100/101/102，物理子步 4→8、虚拟积分 4→8、旧硬接触、命令滤波关闭均保留。关闭滤波可能产生持续力矩限幅；运行完成不能作为该配置可用的证明。没有按排名选择种子或删去负面配置。',
           f'物理步长减半时，以 C 的力 RMS、B/F 的 REF 均值比较，最大相对变化为 {largest[0]} 的 {largest[1]:+.2f}%。这只是所列指标的变化量，不代表非线性振荡逐样本一致或严格的无源性证明。','',
           '## 图与复核','',
           '[完整配置、指标、原始轨迹哈希](metrics.json) · [A/B/F 图示几何恢复证据](../reproduction/paper_ab_geometry.json) · [实现与假设](../../IMPLEMENTATION.md) · [此前 C 机制审计](../../C_MECHANISM_AUDIT.md)','']
    for name in ['A','B','B_impedance','C','C_paper','C_impedance','D_tracking','D_statistics']+(['E'] if has_grinding else [])+['F_gain']:
        rows += [f'![{name}]({name}.png)','']
    rows+=['## MuJoCo 回放','',
           '[A 自由轨迹](A_hinf.mp4) · [B 斜墙接触](B_hinf.mp4) · [C 白板与挡块](C_hinf.mp4) · [D 旋转交互](D_hinf.mp4) · [E 旋转圆盘](E_hinf.mp4) · [F 基准增益](F_hinf.mp4)（均为 H∞，2 倍速；控制器对比见上方图表）。','',
           '回放使用保存的真实关节轨迹；绿色为期望路径，橙色为实际路径。显示用半透明墙不改变实验碰撞几何。','',
           '![B 实际场景](scene_B.png)','', '![C 实际场景](scene_C.png)','', '![E 实际场景](scene_E.png)','',
           '## 重跑','',
           '```bash','OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_reproduction.py --resume',
           'OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/report_reproduction.py','```','',
           '时序、有限值、矩阵完整性、位置 RMS、峰值力和实际限幅已逐运行核对。单元/集成测试见 `VALIDATION.txt`；E 测试依当前范围默认跳过。D 的误差棒是虚拟操作者重复间样本标准差，不是真人统计置信区间。敏感性运行的提前终止保留，不能把部分时长指标与完整 30 s 直接等同。']
    if not has_grinding:
        rows=[r.replace(' · [E 旋转圆盘](E_hinf.mp4)','') for r in rows if r!='![E 实际场景](scene_E.png)']
    (out/'REPORT.md').write_text('\n'.join(rows))
    print(json.dumps(dict(main=len(records),sensitivity=len(extras),status=dict(states),output=str(out)),indent=2))


if __name__=='__main__':main()
