"""Report D under the Fig. 11 replay operator, against the paper and the fr3v2 grasp model.

The claims were written before this matrix finished, but after single-trial probes
of the same input, so they are not independent of what was already seen.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from fr3imp.model import ROOT
from fr3imp.scenarios import REPLAY_MIRROR,Scenario
from validate_d_operator import cases

ORDER=('admittance','impedance','smc','hinf')
NAMES=dict(admittance='导纳',impedance='普通阻抗',smc='SMC',hinf='H∞')
COLOR=dict(admittance='#0072bd',impedance='#d95319',smc='#edb120',hinf='#7e2f8e')
# Read off Fig. 12: means over 30 human trials.
PAPER=dict(pitch=dict(admittance=.0217,impedance=.0407,smc=.017,hinf=.0024),
           yaw=dict(admittance=.175,impedance=.41,smc=.126,hinf=.118))
SCALE=.6
CLAIMS=[('complete','四种控制器 120 次全部跑完'),
        ('impedance_worst','pitch 和 yaw 的平衡偏差都是普通阻抗最大'),
        ('smc_above_hinf','SMC 的 pitch 偏差至少是 H∞ 的 1.5 倍'),
        ('hinf_pitch_level','H∞ 的 pitch 偏差在论文值 × 输入比例的 ±30% 内'),
        ('hinf_yaw_level','H∞ 的 yaw 偏差在论文值 × 输入比例的 ±30% 内'),
        ('impedance_yaw_level','普通阻抗的 yaw 偏差在论文值 × 输入比例的 ±30% 内'),
        ('roll_free','无滚转支撑时，H∞ 第二阶段滚转都在 70°–110°'),
        ('admittance_above_hinf','导纳的 pitch 偏差至少是 H∞ 的 2 倍（论文约 9 倍）'),
        ('smc_paper_ratio','SMC 的 pitch 偏差至少是 H∞ 的 4 倍（论文约 7 倍）'),
        ('impedance_paper_ratio','普通阻抗的 pitch 偏差至少是 H∞ 的 10 倍（论文约 17 倍）'),
        ('hinf_yaw_best','H∞ 的 yaw 偏差最小')]


def equilibrium(summary,folder):
    """Time-average |e + m/K| on pitch and yaw, from the measured wrench."""
    with np.load(folder/'trajectory.npz') as a:
        p=a['pose'].astype(float);f=a['force_measured'].astype(float);d=a['desired'].astype(float)
    k=summary['impedance']['stiffness']
    pitch=d[:,4]-p[:,4]+(-np.sin(p[:,5])*f[:,3]+np.cos(p[:,5])*f[:,4])/k[4]
    yaw=d[:,5]-p[:,5]+f[:,5]/k[5]
    return float(np.abs(pitch).mean()),float(np.abs(yaw).mean())


def load(root):
    manifest=json.loads((root/'manifest.json').read_text())
    if manifest['errors'] or set(manifest['completed'])!=set(cases()):raise ValueError('Incomplete D matrix')
    records={}
    for name,s in manifest['completed'].items():
        if s['source_sha256']!=manifest['source_sha256']:raise ValueError(f'Mixed source versions: {name}')
        folder=root/name.split('/')[0]/f"D_{s['controller']}_trial{s['trial']:02d}_seed{s['seed']}"
        if json.loads((folder/'summary.json').read_text())!=s:raise ValueError(f'Manifest mismatch: {folder}')
        with np.load(folder/'trajectory.npz') as a:
            if not all(np.isfinite(a[k]).all() for k in a.files):raise ValueError(f'Non-finite arrays: {folder}')
            n=len(a['time'])
        if s['status']=='completed' and n!=round(s['requested_duration_s']*1000):raise ValueError(f'Short run: {folder}')
        records[name]=dict(summary=s,folder=folder,equilibrium=equilibrium(s,folder))
    return manifest,records


def statistics(records):
    result={}
    for c in ORDER:
        rows=[r for name,r in records.items() if name.startswith(f'main/D_{c}_')]
        done=[r for r in rows if r['summary']['status']=='completed']
        eq=np.array([r['equilibrium'] for r in done])
        metric=lambda k,scale=1:[r['summary']['metrics'][k]*scale for r in done]
        result[c]=dict(completed=len(done),total=len(rows),pitch=eq[:,0].mean(),pitch_sd=eq[:,0].std(ddof=1),
                       yaw=eq[:,1].mean(),yaw_sd=eq[:,1].std(ddof=1),
                       pitch_gap=float(np.mean(metric('pitch_impedance_gap_mae_rad'))),
                       yaw_gap=float(np.mean(metric('yaw_impedance_gap_mae_rad'))),
                       saturation=float(np.mean(metric('saturation_fraction',100))),
                       roll=metric('phase2_roll_mean_rad',180/np.pi))
    return result


def evaluate(stats):
    p={c:stats[c]['pitch'] for c in ORDER};y={c:stats[c]['yaw'] for c in ORDER}
    near=lambda value,paper:abs(value/(SCALE*paper)-1)<=.3
    roll=stats['hinf']['roll']
    return dict(complete=bool(all(stats[c]['completed']==30 for c in ORDER)),
                impedance_worst=max(p,key=p.get)=='impedance' and max(y,key=y.get)=='impedance',
                smc_above_hinf=bool(p['smc']>=1.5*p['hinf']),
                hinf_pitch_level=bool(near(p['hinf'],PAPER['pitch']['hinf'])),
                hinf_yaw_level=bool(near(y['hinf'],PAPER['yaw']['hinf'])),
                impedance_yaw_level=bool(near(y['impedance'],PAPER['yaw']['impedance'])),
                roll_free=bool(len(roll)==30 and min(roll)>=70 and max(roll)<=110),
                admittance_above_hinf=bool(p['admittance']>=2*p['hinf']),
                smc_paper_ratio=bool(p['smc']>=4*p['hinf']),
                impedance_paper_ratio=bool(p['impedance']>=10*p['hinf']),
                hinf_yaw_best=min(y,key=y.get)=='hinf')


def grasp_statistics(root):
    """The same statistic for the fr3v2 grasp operator, from its saved 30 trials."""
    result={}
    for c in ORDER:
        values=[]
        for folder in sorted((root/'main').glob(f'D_{c}_trial*')):
            values.append(equilibrium(json.loads((folder/'summary.json').read_text()),folder))
        result[c]=dict(zip(('pitch','yaw'),np.mean(values,axis=0).tolist()))
    return result


def digitization_check():
    """Fig. 11 is one trial per controller; it should still sit close to Fig. 12."""
    with np.load(ROOT/'fr3imp/data/paper_d_curves.npz') as curves:
        result={}
        for c in ORDER:
            result[c]={}
            for axis,k in (('pitch',200.),('yaw',10.)):
                e,m=curves[f'{c}_{axis}_error'],curves[f'{c}_{axis}_moment']
                t=np.arange(0,min(e[:,0].max(),m[:,0].max()),.001)
                result[c][axis]=float(np.abs(np.interp(t,e[:,0],e[:,1])+np.interp(t,m[:,0],m[:,1])/k).mean())
    return result


def bar_figure(stats,grasp,out):
    fig,axes=plt.subplots(2,2,figsize=(10,6.4),layout='constrained')
    x=np.arange(4)
    for row,(axis,label) in enumerate((('pitch','Pitch'),('yaw','Yaw'))):
        paper=[PAPER[axis][c]*SCALE for c in ORDER];ours=[stats[c][axis] for c in ORDER];old=[grasp[c][axis] for c in ORDER]
        ax=axes[row,0]
        ax.bar(x-.27,paper,.27,color='#888888',label=f'Paper x {SCALE}')
        ax.bar(x,old,.27,color='#c6c6c6',label='fr3v2 grasp operator')
        ax.bar(x+.27,ours,.27,yerr=[stats[c][axis+'_sd'] for c in ORDER],color=[COLOR[c] for c in ORDER],capsize=3,label='Fig. 11 replay')
        ax.set(xticks=x,xticklabels=['Adm','Imp','SMC','H-inf'],ylabel=f'{label} equilibrium discrepancy [rad]');ax.grid(axis='y',alpha=.15);ax.legend(fontsize=8)
        ax=axes[row,1]
        hinf=dict(paper=PAPER[axis]['hinf'],old=grasp['hinf'][axis],ours=stats['hinf'][axis])
        ax.bar(x-.27,[PAPER[axis][c]/hinf['paper'] for c in ORDER],.27,color='#888888',label='Paper')
        ax.bar(x,[grasp[c][axis]/hinf['old'] for c in ORDER],.27,color='#c6c6c6',label='fr3v2 grasp operator')
        ax.bar(x+.27,[stats[c][axis]/hinf['ours'] for c in ORDER],.27,color=[COLOR[c] for c in ORDER],label='Fig. 11 replay')
        ax.axhline(1,color='k',lw=.7)
        ax.set(xticks=x,xticklabels=['Adm','Imp','SMC','H-inf'],ylabel=f'{label}: ratio to H-inf');ax.grid(axis='y',alpha=.15);ax.legend(fontsize=8)
    fig.suptitle('D: equilibrium discrepancy. Left: amplitude. Right: scale-free ratio to H-infinity')
    fig.savefig(out/'D_operator_bars.png',dpi=160);plt.close(fig)


def trace_figure(records,out):
    with np.load(ROOT/'fr3imp/data/paper_d_curves.npz') as curves:
        paper={k:curves[k] for k in curves.files}
    r=records['main/D_hinf_trial00']
    with np.load(r['folder']/'trajectory.npz') as a:
        t=a['time'];e=a['desired'].astype(float)-a['pose'].astype(float);f=a['force_measured'].astype(float);p=a['pose'].astype(float)
    k=r['summary']['impedance']['stiffness']
    moment=np.c_[-np.sin(p[:,5])*f[:,3]+np.cos(p[:,5])*f[:,4],f[:,5]]
    scale=r['summary']['operator_replay']['scale']*Scenario('D',np.zeros(6),0).operator_scale
    fig,axes=plt.subplots(2,2,figsize=(12,5.6),sharex=True,layout='constrained')
    for col,(axis,index,mirror,stiff) in enumerate((('pitch',4,REPLAY_MIRROR[1],k[4]),('yaw',5,REPLAY_MIRROR[2],k[5]))):
        pe,pm=paper[f'hinf_{axis}_error'],paper[f'hinf_{axis}_moment']
        axes[0,col].plot(pm[:,0],mirror*-pm[:,1]/stiff*scale,color='#888888',lw=.8,label=f'Paper -m/K x {scale:.3g}')
        axes[0,col].plot(t,-moment[:,col]/stiff,color=COLOR['hinf'],lw=.7,label='Sim -m/K (measured)')
        axes[0,col].set(ylabel=f'{axis} -m/K [rad]');axes[0,col].legend(fontsize=8)
        axes[1,col].plot(pe[:,0],mirror*pe[:,1]*scale,color='#888888',lw=.8,label=f'Paper error x {scale:.3g}')
        axes[1,col].plot(t,e[:,index],color=COLOR['hinf'],lw=.7,label='Sim error')
        axes[1,col].set(ylabel=f'{axis} error [rad]',xlabel='Time [s]',xlim=(0,30));axes[1,col].legend(fontsize=8)
    for ax in axes.ravel():ax.grid(alpha=.15)
    fig.suptitle('D, H-infinity: paper trial versus simulation given that trial\'s moments')
    fig.savefig(out/'D_operator_trace.png',dpi=160);plt.close(fig)


def controller_figure(records,out):
    with np.load(ROOT/'fr3imp/data/paper_d_curves.npz') as curves:
        paper={k:curves[k] for k in curves.files}
    fig,axes=plt.subplots(2,4,figsize=(14,5.2),sharex=True,layout='constrained')
    for col,c in enumerate(('impedance','admittance','smc','hinf')):
        with np.load(records[f'main/D_{c}_trial00']['folder']/'trajectory.npz') as a:
            t=a['time'];e=a['desired'].astype(float)-a['pose'].astype(float)
        axes[0,col].plot(t,e[:,4],color=COLOR[c],lw=.6);axes[0,col].set_title(NAMES_EN[c])
        axes[1,col].plot(t,e[:,5],color=COLOR[c],lw=.6)
        axes[1,col].set(xlabel='Time [s]',xlim=(0,30))
        for row,(axis,mirror) in enumerate((('pitch',REPLAY_MIRROR[1]),('yaw',REPLAY_MIRROR[2]))):
            trace=paper[f'{c}_{axis}_error'];axes[row,col].plot(trace[:,0],mirror*trace[:,1],color='#aaaaaa',lw=.5,zorder=0)
    axes[0,0].set_ylabel('Pitch error [rad]');axes[1,0].set_ylabel('Yaw error [rad]')
    for ax in axes.ravel():ax.grid(alpha=.15)
    fig.suptitle('D: simulation with H-infinity trial moments (color) over each paper trial, unscaled and with its own moments (gray)')
    fig.savefig(out/'D_operator_controllers.png',dpi=160);plt.close(fig)


OPERATOR_SCALES=[Scenario('D',np.zeros(6),trial).operator_scale for trial in range(30)]
NAMES_EN=dict(admittance='Admittance',impedance='Impedance',smc='Sliding mode',hinf='H-infinity')


def sensitivity_rows(records):
    rows=['|配置|H∞ 状态|pitch：导纳 / 普通阻抗 / SMC / H∞|yaw：同上|限幅 % ：同上|','|---|---|---:|---:|---:|']
    def cell(label,c,index):
        r=records.get(f'{label}/D_{c}')
        return '—' if r is None else f'{r["equilibrium"][index]:.4f}'
    def sat(label,c):
        r=records.get(f'{label}/D_{c}')
        return '—' if r is None else f'{r["summary"]["metrics"]["saturation_fraction"]*100:.2f}'
    rows_main={c:records[f'main/D_{c}_trial00'] for c in ORDER}
    rows.append('|trial 0（基准）|'+rows_main['hinf']['summary']['status']+'|'+' / '.join(f'{rows_main[c]["equilibrium"][0]:.4f}' for c in ORDER)+'|'
                +' / '.join(f'{rows_main[c]["equilibrium"][1]:.3f}' for c in ORDER)+'|'+' / '.join(f'{rows_main[c]["summary"]["metrics"]["saturation_fraction"]*100:.2f}' for c in ORDER)+'|')
    labels=[*(f'scale{s:g}' for s in (.3,.4,.5,.7,.8,1.)),*(f'grip{g:g}' for g in (10.,30.,60.,120.,300.)),'own_column','delay0','matched_G','friction1','friction2','friction3.5','physics8']
    for label in labels:
        h=records.get(f'{label}/D_hinf')
        state='—' if h is None else h['summary']['status']+(f" ({h['summary']['simulated_duration_s']:.1f} s)" if h['summary']['status']!='completed' else '')
        rows.append(f'|{label}|{state}|'+' / '.join(cell(label,c,0) for c in ORDER)+'|'+' / '.join(cell(label,c,1) for c in ORDER)+'|'+' / '.join(sat(label,c) for c in ORDER)+'|')
    return rows


def report(manifest,records,stats,grasp,checks,digitized,out):
    labels=dict(CLAIMS)
    failed=[labels[k] for k,_ in CLAIMS if not checks[k]]
    states=sum(r['summary']['status']=='completed' for r in records.values())
    def triple(values,digits=4):return ' / '.join(f'{v:.{digits}f}' for v in values)
    rows=['# D：用论文 Fig. 11 的实测力矩作为操作者输入','',
          '当前范围见 [AGENTS.md](../../../AGENTS.md)。这里只改 D 的操作者，其余沿用 `fr3v2`。[`fr3v2` 原有的 D 结果](../reproduction-fr3v2/REPORT.md)和默认操作者都没有改动。','',
          f'共 {len(records)} 次运行（4 种控制器各 30 次 + 固定敏感性检查），其中 {states} 次跑完，其余提前终止并保留。运行时代码 SHA256：`{manifest["source_sha256"]}`。','',
          '**这是 FR3/MuJoCo 仿真，输入取自论文图中的一次试验，不是真人交互，也不是作者的原始日志。**','',
          '## 结论','',*CONCLUSION,'',
          f'矩阵运行前写定的 {len(CLAIMS)} 条判据通过 {sum(checks.values())} 条。未通过：','',*[f'- {x}' for x in failed],'',
          '## 操作者模型','',*MODEL,'',
          '## 平衡偏差','',
          f'全部为测量 F/T 口径，时间平均后取 30 次均值。“论文 × {SCALE}”只用于比较量级：各次试验还乘以操作者系数（{min(OPERATOR_SCALES):.2f}–{max(OPERATOR_SCALES):.2f}，均值 {np.mean(OPERATOR_SCALES):.2f}），实际平均输入约为论文的 {SCALE*np.mean(OPERATOR_SCALES):.2f} 倍。与量级相比，相对 H∞ 的倍数不受这个比例影响，更适合比较。','',
          '|指标（导纳 / 普通阻抗 / SMC / H∞）|论文|论文 × .6|`fr3v2` 原操作者|Fig. 11 回放|','|---|---:|---:|---:|---:|']
    for axis,name in (('pitch','pitch'),('yaw','yaw')):
        paper=[PAPER[axis][c] for c in ORDER];old=[grasp[c][axis] for c in ORDER];new=[stats[c][axis] for c in ORDER]
        d=4 if axis=='pitch' else 3
        rows.append(f'|{name} / rad|{triple(paper,d)}|{triple([x*SCALE for x in paper],d)}|{triple(old,d)}|{triple(new,d)}|')
        rows.append(f'|{name} 相对 H∞ 的倍数|{triple([x/paper[3] for x in paper],2)}|—|{triple([x/old[3] for x in old],2)}|{triple([x/new[3] for x in new],2)}|')
    rows+=['','![D 平衡偏差：幅值与相对 H∞ 的倍数](D_operator_bars.png)','',
           '|控制器|完整/总数|pitch / rad|yaw / rad|pitch 阻抗偏差 / rad|yaw 阻抗偏差 / rad|第二阶段 roll / °|限幅|','|---|---:|---:|---:|---:|---:|---:|---:|']
    for c in ORDER:
        s=stats[c]
        rows.append(f'|{NAMES[c]}|{s["completed"]}/{s["total"]}|{s["pitch"]:.4f} ± {s["pitch_sd"]:.4f}|{s["yaw"]:.4f} ± {s["yaw_sd"]:.4f}|{s["pitch_gap"]:.4f}|{s["yaw_gap"]:.4f}|{np.mean(s["roll"]):.1f}（{min(s["roll"]):.1f}–{max(s["roll"]):.1f}）|{s["saturation"]:.2f}%|')
    rows+=['','± 是 30 次之间的样本标准差。这 30 次共用同一条力矩曲线，只是幅值和噪声不同，所以离散程度明显小于真人试验，不能当作置信区间。','',
           '普通阻抗的 12.70% 限幅不是偏差的原因。在 trial 0 上，它全部落在 13.67–17.5 s 的滚转引导段，是第 5 关节达到 12 Nm 限幅；这段的 yaw 偏差（.216 rad）并不高于其余时段（.255 rad）。','',
           '![H∞：论文试验与仿真](D_operator_trace.png)','',
           '灰线是论文 H∞ 这一次试验按比例缩放并镜像后的结果，彩线是仿真。','',
           '![四种控制器：仿真与论文](D_operator_controllers.png)','',
           '灰线是论文四次试验，未缩放，且各用各的力矩；彩线是仿真，四种控制器用同一条 H∞ 力矩，因此两者只能看形态。','',
           '## 判据','',
           '判据写在这个矩阵跑完之前，但写在单次探索之后，阈值并非独立于已看到的结果。','',
           '|判据|结果|','|---|---:|',*[f'|{labels[k]}|{"✓" if checks[k] else "✗"}|' for k,_ in CLAIMS],'',
           '## 敏感性','',
           '每行除标注外都是 trial 0、种子 100。`scale*` 只跑 H∞，用来确定幅值上限。`grip*` 是握持刚度（yaw，pitch 为 3 倍）。`own_column` 让每种控制器用论文中自己那一列的力矩。','',
           *sensitivity_rows(records),'',*SENSITIVITY,'',
           '## 数字化核对','',
           'Fig. 11 是每种控制器的一次代表性试验，从 PDF 矢量线恢复。下表用它算出的偏差与 Fig. 12 的 30 次均值对比，两者接近，说明坐标标定可信。矢量线可能被导出工具简化，短促尖峰可能被削低。','',
           '|控制器|pitch：数字化 / Fig. 12|yaw：数字化 / Fig. 12|','|---|---:|---:|']
    for c in ORDER:
        rows.append(f'|{NAMES[c]}|{digitized[c]["pitch"]:.4f} / {PAPER["pitch"][c]:.4f}|{digitized[c]["yaw"]:.3f} / {PAPER["yaw"][c]:.3f}|')
    rows+=['','## 重跑','',
           '重新数字化需要 Poppler 的 `pdftocairo`。','',
           '```bash','.venv/bin/python scripts/digitize_paper_d.py',
           'OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_d_operator.py --workers 8 --resume',
           'OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/report_d_operator.py','```','',
           '[机器可读结果](metrics.json) · [测试记录](VALIDATION.txt) · [数字化曲线](../../../fr3imp/data/paper_d_curves.npz)','',
           '加入这个模型改变了运行时代码哈希，因此早先的 `fr3`、`fr3v2` 记录要回放时，请使用各自输出目录里的 `source/` 快照。']
    (out/'REPORT.md').write_text('\n'.join(rows)+'\n')


CONCLUSION=['换成论文实测输入后，D 的趋势比原操作者更接近论文：yaw 上普通阻抗与 H∞ 的倍数几乎相同，pitch 上 SMC 也开始明显差于 H∞。但论文最强的 pitch 优势仍只复现了一部分，**导纳的恶化完全没有出现**。这个差距已经不能归咎于操作者。','',
            '- **yaw：** 普通阻抗是 H∞ 的 3.45 倍，论文 3.47 倍（原操作者 6.21 倍）。SMC 1.31 倍，论文 1.07 倍。H∞ 与普通阻抗的幅值也与按比例缩放的论文值接近。不过普通阻抗的倍数不是定值：trial 0 上摩擦 ×1 时 2.41 倍、×3 时 3.25 倍，加入握持后超过 4 倍，所以与 3.47 几乎相同有一定巧合。',
            '- **pitch：** SMC 是 H∞ 的 1.90 倍（原操作者 .98 倍），普通阻抗 5.56 倍（原操作者 3.32 倍）。方向对了，但比论文的 7.08 倍和 16.96 倍小得多。',
            '- **导纳：** pitch 为 H∞ 的 .87 倍（论文 9.04 倍），yaw 为 .95 倍（论文 1.48 倍）。它在两个轴上都略优于 H∞，这与论文相反。',
            '- **为什么操作者解释不了导纳：** 同样的力矩下，仿真导纳的 pitch 阻抗偏差只有 .0002 rad，几乎就是理想阻抗响应。用论文导纳那一列自己的力矩时，yaw 偏差只有 .0225 rad，而论文 Fig. 12 按 .6 折算后约 .105 rad。论文中该次试验的 yaw 误差峰值（1.33 rad）约为 `m/K` 峰值（.385 rad）的 3.5 倍，而线性的临界阻尼响应不会超过 `m/K` 的峰值。这个差距的后续诊断见[导纳差距报告](../d-admittance/REPORT.md)：论文导纳不符合它写明的方程，差距更可能来自实现，不太可能来自仿真对象。',
            '- **稳健性有新证据：** 关节摩擦从 ×1 增加到 ×3.5（trial 0），H∞ 的 pitch 偏差只从 .0013 变到 .0015 rad，SMC 从 .0014 变到 .0031，普通阻抗从 .0043 变到 .0084。SMC 与 H∞ 的差距随摩擦拉开，而摩擦 ×1 时两者几乎相同，所以这一差距依赖 ×3 这个未标定的假设。']
MODEL=['- **力矩：** 使用从 Fig. 11(d) 数字化的 H∞ 这次试验的 `m_x`、`m_y`、`m_z`，以开环方式加到四种控制器上，因此每次试验的输入对所有控制器完全相同。没有人为添加事件，交叉轴力矩和不规则时序都来自原图。',
       '- **幅值：** 乘以 .6。论文 pitch 力矩峰值 18 Nm，超过 FR3 腕部 12 Nm 的限幅。这个比例只根据 H∞ 的运行确定：在系数 1.02 的 trial 0 上，.7 能跑完，.8 就提前终止（第 5 关节超限）。操作者系数理论上限 1.26，.6 × 1.26 = .76 仍低于失败的 .8，所以取 .6；实际 30 次的最高系数是 1.07。这样 trial 0 的 pitch 峰值约 11 Nm，比论文小，这是 FR3 力矩能力决定的，不是论文条件。选择时没有看控制器之间的差距。',
       '- **滚转：** K=0，开环力矩到不了 90°，所以 13–17.5 s 改用闭环引导。之后没有被动支撑，第二阶段滚转均值约 83°，论文四次试验均值约 72°–82°。',
       '- **方向：** 关于 x-z 平面镜像，滚转仍为 +90°，与原操作者一致；论文的滚转方向相反。这一点没有单独做敏感性检查。',
       '- **重复：** 沿用 10 组操作者系数 × 3 次的方案，但只有幅值和噪声变化。这不等于十名真人的差异，论文中真人还会根据手感调整施力。']
SENSITIVITY=['- **紧握没有引发导纳失稳。** 握持刚度从 10 加到 300 Nm/rad，导纳的 pitch 偏差反而从 .0013 降到 .0010 rad，限幅也没有上升。论文提到的紧握失稳没有出现。这个握持模型只在操作者施力时生效，且跟随的是论文的运动轨迹，因此不能排除其他握持方式。',
             '- **F/T 延迟和 G 对 D 影响很小。** 把延迟设为 0，或把 NRIC G 换成带宽匹配值（导纳 pitch .0017 rad），都没有改变结论。这与 B/C 不同，在 B/C 中这两项决定导纳是否抖振。可能是因为 D 中没有硬接触，但这里没有单独验证。',
             '- **各控制器用自己那一列的力矩时**，导纳和 SMC 的输入变小，偏差也更小。这说明论文中导纳和 SMC 的大偏差并非因为人对它们施力更大。',
             '- **物理步长减半**（子步 4 → 8）没有改变结果。']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/d-operator'))
    parser.add_argument('--grasp-root',type=Path,default=Path('outputs/reproduction-fr3v2'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/d-operator'))
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    manifest,records=load(args.root)
    stats=statistics(records);grasp=grasp_statistics(args.grasp_root);checks=evaluate(stats);digitized=digitization_check()
    bar_figure(stats,grasp,out);trace_figure(records,out);controller_figure(records,out)
    report(manifest,records,stats,grasp,checks,digitized,out)
    payload=dict(source_sha256=manifest['source_sha256'],claims=dict(CLAIMS),checks=checks,paper_fig12=PAPER,input_scale=SCALE,
                 replay=stats,grasp=grasp,digitized_fig11=digitized,
                 sensitivity={name:dict(status=r['summary']['status'],simulated_duration_s=r['summary']['simulated_duration_s'],
                                        termination_reason=r['summary']['termination_reason'],pitch_rad=r['equilibrium'][0],yaw_rad=r['equilibrium'][1],
                                        saturation_fraction=r['summary']['metrics']['saturation_fraction'])
                              for name,r in records.items() if not name.startswith('main/')})
    (out/'metrics.json').write_text(json.dumps(payload,indent=2))
    print(json.dumps(dict(runs=len(records),checks=checks),indent=2))


if __name__=='__main__':main()
