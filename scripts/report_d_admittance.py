"""Identify W in Lambda*x'' + D*x' + K*x = (I + W)*m from Fig. 11 and from the candidate runs.

Needs outputs/d-admittance (validate_d_admittance.py), outputs/paper-c-digitized
(digitize_paper_c.py) and, for the regression check, outputs/d-operator.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import cont2discrete

from fr3imp.dynamics import euler_rate_map
from fr3imp.experiment import source_hash
from report_d_operator import PAPER,SCALE
from validate_d_admittance import cases

L=np.array([2.,.4,.4]);D=np.array([20.,2.,4.]);K=np.array([0.,200.,10.])
AXES=('roll','pitch','yaw')
ORDER=('admittance','impedance','smc','hinf')
NAMES=dict(admittance='导纳',impedance='普通阻抗',smc='SMC',hinf='H∞')
NAMES_EN=dict(admittance='Admittance',impedance='Impedance',smc='Sliding mode',hinf='H-infinity')
STEP=.002
PAPER_WINDOWS=dict(phase1=(0.,13.),phase2=(17.5,35.))
SIM_WINDOWS=dict(phase1=(0.,13.),phase2=(17.5,30.))
GRAY,LIGHT,ACCENT='#6b7280','#c4c4c4','#0072B2'

CANDIDATES=(('servo_g0.3','软内环 G×.3'),('servo_g0.1','软内环 G×.1'),('servo_g0.03','软内环 G×.03'),
            ('ft_delay20','F/T 延迟 20 ms'),('force_gain4','代理力矩恒定 ×4'),('feedthrough','未抵消力通路'))


def response(axis,moment):
    A,B,*_=cont2discrete((np.array([[0,1],[-K[axis]/L[axis],-D[axis]/L[axis]]]),np.array([[0],[1/L[axis]]]),
                          np.array([[1,0]]),np.array([[0]])),STEP,method='zoh')
    state=np.zeros(2);output=np.zeros(len(moment));b=B[:,0]
    for n,value in enumerate(moment):
        output[n]=state[0];state=A@state+b*value
    return output


def identify(time,x,m,window):
    """Per axis, regress x - G[m_i] on G[m_j]; roll has no restoring force, so it also gets a drift."""
    active=(time>=window[0])&(time<window[1])
    W=np.zeros((3,3));ideal=np.zeros(3);fitted=np.zeros(3);prediction=np.full_like(x,np.nan)
    for i in range(3):
        base=response(i,m[:,i])
        columns=[response(i,m[:,j]) for j in range(3)]+[np.ones(len(time))]
        if K[i]==0:columns.append(time)
        X=np.column_stack(columns)
        c=np.linalg.lstsq(X[active],(x[:,i]-base)[active],rcond=None)[0]
        W[i]=c[:3]
        prediction[active,i]=base[active]+X[active]@c
        total=np.sum((x[active,i]-x[active,i].mean())**2)
        ideal[i]=1-np.sum((x[active,i]-base[active])**2)/total
        fitted[i]=1-np.sum((x[active,i]-prediction[active,i])**2)/total
    return dict(W=W,ideal_r2=ideal,fit_r2=fitted,prediction=prediction)


def paper_series(curves,column):
    time=np.arange(0,35,STEP)
    x=np.column_stack([-np.interp(time,*curves[f'{column}_{a}_error'].T) for a in AXES])
    m=np.column_stack([np.interp(time,*curves[f'{column}_{a}_moment'].T) for a in AXES])
    return time,x,m


def run_files(root,label,controller,experiment='D'):
    folder=root/label/f'{experiment}_{controller}_trial00_seed100'
    return (folder,json.loads((folder/'summary.json').read_text()),json.loads((folder/'variant.json').read_text()))


def sim_series(folder):
    with np.load(folder/'trajectory.npz') as saved:
        time=saved['time'].astype(float);pose=saved['pose'].astype(float);desired=saved['desired'].astype(float)
        force=saved['force_measured'].astype(float);saturated=float(np.mean(saved['saturated']))
    m=np.array([euler_rate_map(p).T@f for p,f in zip(pose[:,3:],force[:,3:])])
    return time,pose[:,3:]-desired[:,3:],m,saturated


def discrepancy(x,m):
    return np.abs(-x[:,1:]+m[:,1:]/K[1:]).mean(axis=0)


def sim_row(root,label,controller):
    folder,summary,variant=run_files(root,label,controller)
    time,x,m,saturated=sim_series(folder)
    row=dict(status=summary['status'],duration=summary['simulated_duration_s'],reason=summary.get('termination_reason'),
             pitch=float(discrepancy(x,m)[0]),yaw=float(discrepancy(x,m)[1]),saturation=saturated,
             peak_moment=np.abs(m).max(axis=0).tolist(),phases={})
    resampled=(time[::2],x[::2],m[::2])
    for name,window in SIM_WINDOWS.items():
        if time[-1]<window[0]+3:continue
        fit=identify(*resampled,window)
        row['phases'][name]=dict(W=np.diag(fit['W']).tolist(),W_full=fit['W'].tolist(),ideal_r2=fit['ideal_r2'].tolist(),fit_r2=fit['fit_r2'].tolist(),
                                 predicted=(L*np.diag(np.array(variant['task_inverse_inertia'][name])[3:,3:])).tolist(),
                                 predicted_full=(L[:,None]*np.array(variant['task_inverse_inertia'][name])[3:,3:]).tolist())
    return row


def paper_identification(curves):
    result={}
    for column in ORDER:
        time,x,m=paper_series(curves,column)
        result[column]=dict(peak_moment=np.abs(m).max(axis=0).tolist(),phases={})
        for name,window in PAPER_WINDOWS.items():
            fit=identify(time,x,m,window)
            result[column]['phases'][name]=dict(W=np.diag(fit['W']).tolist(),W_full=fit['W'].tolist(),ideal_r2=fit['ideal_r2'].tolist(),fit_r2=fit['fit_r2'].tolist())
    return result


def force_bins(time,force):
    return [float(force[(time>=t)&(time<t+5)].mean()) for t in range(0,30,5)]


def c_results(root,c_curves):
    paper=dict(np.load(c_curves))
    result=dict(paper={},sim={})
    grid=np.arange(.01,30,STEP);desired=10+3*np.sin(grid)
    result['paper_desired_mean']=float(desired.mean())
    for c in ('admittance','hinf'):
        force=np.interp(grid,*paper['force_'+c].T)
        result['paper'][c]=dict(mean=float(force.mean()),std=float(force.std()),bins=force_bins(grid,force))
    for label in ('baseline','feedthrough'):
        for c in ('admittance','hinf'):
            folder,summary,variant=run_files(root,label,c,'C')
            with np.load(folder/'trajectory.npz') as saved:
                time=saved['time'].astype(float);force=saved['force_measured'][:,2].astype(float)
            inverse=np.array(variant['task_inverse_inertia']['all'])
            result['sim'][f'{label}/{c}']=dict(status=summary['status'],mean=float(force.mean()),std=float(force.std()),
                                                bins=force_bins(time,force),mass_z=float(1/inverse[2,2]),
                                                predicted_ratio=float(1/(1+10*inverse[2,2])))
    return result


def ba_results(root):
    rows={}
    for experiment,controllers in (('B',ORDER),('A',('admittance','impedance'))):
        for label in ('baseline','feedthrough'):
            for c in controllers:
                folder,summary,_=run_files(root,label,c,experiment)
                with np.load(folder/'trajectory.npz') as saved:
                    force=np.linalg.norm(saved['force_true'][:,:3].astype(float),axis=1)
                    error=saved['pose'][:,:3].astype(float)-saved['desired'][:,:3].astype(float)
                rows[f'{experiment}/{label}/{c}']=dict(status=summary['status'],peak_force=float(force.max()),
                    equilibrium=summary['metrics'].get('equilibrium_position_error_measured_mean_m'),
                    position_rms=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))))
    return rows


def regression_check(root,reference):
    """The diagnostic class with its terms switched off must equal the plain controller."""
    with np.load(root/'baseline/D_admittance_trial00_seed100/trajectory.npz') as a,\
         np.load(reference/'main/D_admittance_trial00_seed100/trajectory.npz') as b:
        return float(np.abs(a['pose']-b['pose']).max())


def fit_figure(curves,out):
    fig,axes=plt.subplots(2,2,figsize=(11,5.6),sharex=True,layout='constrained')
    for col,column in enumerate(('hinf','admittance')):
        time,x,m=paper_series(curves,column)
        for row,axis in enumerate((1,2)):
            ax=axes[row,col]
            ax.plot(time,x[:,axis],color=GRAY,lw=2.2,label='Paper Fig. 11')
            ax.plot(time,response(axis,m[:,axis]),color='#D55E00',lw=1.1,label='Stated equation (W = 0)')
            for number,window in enumerate(PAPER_WINDOWS.values()):
                ax.plot(time,identify(time,x,m,window)['prediction'][:,axis],color=ACCENT,lw=1.1,ls='--',
                        label='Fitted W' if number==0 else None)
            ax.axhline(0,color='#dddddd',lw=.8);ax.grid(alpha=.2)
            ax.set_ylabel(f'{AXES[axis]} deviation [rad]')
            if row==0:ax.set_title(NAMES_EN[column])
    axes[1,0].set_xlabel('Time [s]');axes[1,1].set_xlabel('Time [s]')
    axes[0,0].legend(loc='upper right',fontsize=8)
    fig.savefig(out,dpi=160);plt.close(fig)


def ratio_figure(rows,out):
    fig,axes=plt.subplots(1,2,figsize=(10,3.6),layout='constrained')
    x=np.arange(4)
    for ax,axis in zip(axes,('pitch','yaw')):
        paper=[PAPER[axis][c]/PAPER[axis]['hinf'] for c in ORDER]
        base=[rows['baseline_own'][c][axis]/rows['baseline_own']['hinf'][axis] for c in ORDER]
        feed=[rows['feedthrough_own'][c][axis]/rows['feedthrough_own']['hinf'][axis] for c in ORDER]
        for offset,values,color,label in ((-.27,paper,GRAY,'Paper'),(0,base,LIGHT,'FR3, paper equations'),
                                          (.27,feed,ACCENT,'FR3, force term not cancelled')):
            bars=ax.bar(x+offset,values,.25,color=color,label=label)
            ax.bar_label(bars,fmt='%.1f',fontsize=7,padding=1)
        ax.axhline(1,color='black',lw=.8)
        ax.set_xticks(x,[NAMES_EN[c] for c in ORDER]);ax.set_ylabel(f'{axis.capitalize()}: ratio to H-infinity');ax.grid(axis='y',alpha=.2)
    axes[0].legend(fontsize=8)
    fig.savefig(out,dpi=160);plt.close(fig)


def force_figure(c,out):
    fig,ax=plt.subplots(figsize=(7,3.4),layout='constrained')
    centers=np.arange(2.5,30,5)
    series=((c['paper']['hinf']['bins'],GRAY,'--','Paper H-infinity'),(c['sim']['baseline/admittance']['bins'],LIGHT,'-','FR3 admittance, paper equations'),
            (c['paper']['admittance']['bins'],GRAY,'-','Paper admittance'),(c['sim']['feedthrough/admittance']['bins'],ACCENT,'-','FR3 admittance, force term not cancelled'))
    for values,color,style,label in series:
        ax.plot(centers,values,color=color,ls=style,lw=2,marker='o',ms=5,label=label)
    ax.axhline(10,color='black',lw=.8);ax.text(13.6,10.25,'Mean desired force',fontsize=8)
    ax.set_xlabel('Time [s], 5 s bins');ax.set_ylabel('Mean normal force [N]');ax.set_ylim(0,13);ax.grid(alpha=.2)
    ax.legend(fontsize=8,loc='lower right',ncols=2)
    fig.savefig(out,dpi=160);plt.close(fig)


INDY7_FINDINGS=['''- **量级通过。** 不计转子时，隐含的 `Mx⁻¹` 与模型同一量级，`conventional_cold` 在第一阶段 pitch、yaw 和第二阶段 pitch 上相差不到 5%。''',
                '- **计入转子时 D 和 C 对不上。** 四组参数在 D 都需要 c≈2.8–6.7，在 C 却只需要 c≈.75–1.21，范围完全不重叠。不计转子时两者有重叠：D 为 .39–2.17（`conventional_cold` 为 .95–1.68），C 为 .56–.90。所以这个形式只有在名义模型不含转子惯量时才说得通。',
                '- **不利之处：结果依赖参数组。** 四组辨识值的 Izz6 从 .065 到 .157，差 2.4 倍。只有 `conventional_cold` 有姿态同时满足四项 ±20%（346 个中的 43 个），其余七个模型都没有。从八个模型里挑最好的一个有选择偏差。',
                '- **不利之处：有两项对不上。** 即使用 `conventional_cold`，第二阶段 yaw 也需要 c=1.68，第二阶段 roll 需要 c=.49。roll 的力矩太小，但 yaw 没有这个理由。Fig. 10 的姿态只知道大致形状，这也可能是原因。',
                '- **关键假设没有验证。** 论文没有说明名义模型 `M̂` 是否包含转子惯量。如果包含，`c=1` 的形式就不成立。参数也来自另一个实验室的另一台 Indy7。论文写明 F/T 传感器和工具未建模，所以这里也没有加入它们。']
CONCLUSION=['''**论文导纳的 D 响应不符合它写明的导纳方程，所以把仿真对象做得更逼真，不太可能弥合这个差距。** 用论文曲线辨识后发现，H∞ 符合 `Λẍ+Dẋ+Kx=m`，而导纳相当于多了一项力矩通路 `(I+W)m`。这一差距更可能来自导纳的实现，不太可能来自操作者或 FR3 与 Indy7 的差异。''','',
            '在四种控制器上只改一处，即去掉方程 (54) 和名义输入里的 `-JᵀF`，就同时得到了以下结果：',
            '',
            '- pitch 的相对排序和量级接近论文：普通阻抗 15.35 倍（论文 16.96）、导纳 14.39 倍（9.04）、SMC 4.34 倍（7.08）。',
            '- H∞ 几乎不受影响（pitch .0014 rad，W≈0），也和论文一致。',
            '- C 中导纳的平均力从 10.09 N 降到 3.54 N，H∞ 仍是 10.06 N。论文为 5.9 N 和 10.07 N，每 5 s 的起伏形状也相近。',
            '',
            '与公开的 Indy7 模型核对后，这个解释通过了量级检验，但前提是名义模型不含转子惯量，而这一点未知；计入转子时，D 和 C 需要的系数互相矛盾（见[核对](#与-indy7-模型核对)）。结果还依赖所用的辨识参数组，所以这只是支持，不是确认。',
            '',
            '**这不等于证明作者的代码缺了这一项。** 论文公式里有这一项，作者代码未公开。这里只能说，这个形式足以解释证据，而其他已测试的候选都不能。我也没有把它加入 `fr3v2`：它不是论文的控制器，加入后导纳虽然更差，却不再是对论文方法的公平验证。']
EVIDENCE=['''- **直接从论文数据看：** Fig. 11 中 H∞ 的 |W|≤.15，按论文方程的 R² 已有 .98–1.00。导纳按论文方程的 R² 只有 −35–.47，加入 W 后升到 .88–.99。''',
          '- **yaw 的峰值：** yaw 是临界阻尼，线性响应不会超过 `m/K` 的峰值，而论文导纳约为 3.5 倍（误差 1.33 rad，`m/K` .385 rad）。论文把这种过冲归因于名义模型的惯性和阻尼，但仿真同样用 RK4 积分名义模型，W≈0，说明按写明的参数积分不足以产生这个量级。',
          '- **W 跟着物理轴走：** Λ、D、K 在两个阶段都不变，但滚转 90° 后，论文的 W 从 pitch 4.44、yaw 2.61 变成 pitch 2.51、yaw 4.20，几乎互换。`W=ΛMx⁻¹` 预测了这种互换，恒定增益不能。',
          '- **roll 只是旁证：** roll 的 Λ 是其他两轴的 5 倍，形式上 W 应当更大，第二阶段确实是 7.65，但第一阶段只有 1.59。两个阶段的 roll 力矩有效值都只有约 .55 Nm，不足以下定论。另一方面，转过约 90° 时（8–20 s），导纳的净冲量只有 1.83 Nm·s，H∞ 为 31.47 Nm·s，符合“所需力矩大幅减小”，但导纳这个净冲量的符号与运动相反，这一点没有解释。',
          '- **C 的平均力不能来自抖振：** 论文方程在 z 向为 `Λz̈+Dż=F−Fd`，K=0，z 又贴着白板不会漂走，所以长时间平均力必须等于 Fd，与抖振无关。仿真按论文方程即使抖振（标准差 10 N），平均力仍是 10.09 N（Fd 10.08 N）。论文导纳只有 5.9 N，所以不是抖振造成的。这是我在运行前就写下的预测。']
EXCLUDED=['''- **软内环和关节柔性：** 串联柔度会在 `m/K` 上叠加一个与 K 无关的偏差。要达到论文的响应，yaw 需要约 3 Nm/rad、pitch 需要约 65 Nm/rad 的串联刚度，两者差 20 倍，而论文腕部内环的等效刚度约为 12000 Nm/rad（`G·Kp`）。仿真把 G 降到 ×.03 时 W 也只有 .17。没有在仿真中单独建模关节柔性，这一项的排除依靠这个估算。''',
          '- **F/T 延迟和滤波：** 延迟 20 ms 后 W 仍为 0。论文也写明力信号未滤波。',
          '- **恒定力矩增益：** W 在三个轴和两个阶段都约为 3，没有互换。',
          '- **操作者：** 公式右侧本来就是测得的力矩，所以人的施力方式不影响从 m 到 x 的映射。',
          '- **关节摩擦（针对普通阻抗）：** 仿真基线的 pitch W 为负（−.83、−.41），摩擦在这里只会减小偏差。论文普通阻抗 pitch 的 W 为 +6.96、+3.67，方向相反。']
OPEN=['''- **系数和 Mx 分不开。** 数据只能给出 `W=c·Λ·Mx⁻¹`。取 c=1 时，论文 pitch、yaw 对应的 Mx 约为 .09–.16 kg·m²。与 Indy7 模型的核对见上一节，能否确定仍取决于作者的代码或日志。''',
      '- **绝对值不一致。** FR3 的 Mx 与 Indy7 不同：z 向只有 4.2 kg，因此导纳的绝对偏差和力偏差不会与论文相同。导纳 pitch .0202 rad、yaw .211 rad，高于论文折算值 .0130 和 .105；力比例 .35，也低于论文 .59。',
      '- **yaw 倍数偏差较大。** 导纳 2.81 倍（论文 1.48），SMC .83 倍（1.07），排序中 SMC 与 H∞ 互换。',
      '- **交叉项不能作证据。** 理论上 `Mx⁻¹` 应当对称，但论文 W 的交叉项并不对称。仿真里已知真值时，这个方法同样恢复不了交叉项，所以这一项既不支持也不否定假设。',
      '- **B 的急停和紧握失稳仍未解释。** B 中未抵消力通路没有引发急停（导纳峰值 106.8 N，基线 101.6 N），H∞ 的接触峰值反而从 72.0 N 降到 43.8 N。A 不受影响，这与论文一致。F 没有测。',
      '- **用 H∞ 的力矩时会提前终止。** 导纳 3.0 s 终止，普通阻抗 22.8 s 终止，所以倍数只按各用自己那一列的力矩计算。s=.3 的导纳也在 23.0 s 终止。这与论文中导纳的力矩明显更小（yaw 3.9 Nm，H∞ 12.2 Nm）一致。',
      '- **证据范围。** D 只有每种控制器的一次代表性试验（数字化结果与 Fig. 12 的 30 次均值接近），C 的矢量曲线约 10 ms 一点，可能丢失短促尖峰。候选列表在运行前写定，但是在看过论文辨识结果之后写的。']
MEANING=['''- **对这次复现：** D 中导纳的 ×9 不应当作复现目标，也不宜在仿真里调出来。按论文公式实现时，导纳在 FR3 上接近理想响应，这就是当前的如实结果。''',
         '- **对论文方法的优越性：** H∞ 的两项优势保持成立：在实测力矩下响应符合设定的阻抗；D 和 C 的结果在这个力项和摩擦变化下都基本不变。但“H∞ 明显优于按论文公式实现的导纳”这一点，D 的这组数据不能支持，也没有否定，因为差距可能来自实现而不是因果结构。仿真中按论文公式实现的导纳略优于 H∞，但这个对象没有关节柔性等非理想因素。']


def fmt(values,digits=2):
    return ' / '.join(f'{v:.{digits}f}' for v in values)


def report(payload,root):
    p=payload['paper'];rows=payload['rows'];c=payload['c'];out=[]
    add=out.append
    add('# D：论文导纳差距的原因');add('')
    add('当前范围见 [AGENTS.md](../../../AGENTS.md)。这是诊断报告，`fr3v2`、默认操作者和控制器代码都没有改动。接在 [D 操作者报告](../d-operator/REPORT.md)之后。');add('')
    add(f'共 {len(cases())} 次运行（trial 0，种子 100）。运行时代码 SHA256：`{payload["source_sha256"]}`。诊断版导纳关闭改动后，与 [D 操作者矩阵](../d-operator/REPORT.md)中同一次运行的姿态最大差为 {payload["regression_pose_difference"]:.1e}。');add('')
    add('**这是 FR3/MuJoCo 仿真加上从论文 PDF 恢复的曲线，不是作者的原始日志或代码。**');add('')
    add('## 结论');add('');out.extend(CONCLUSION);add('')
    add('![各控制器的 pitch 和 yaw 倍数](D_ratios.png)');add('')
    add('每种控制器都用论文中自己那一列的力矩（×.6），这与论文的条件一致。');add('')
    add('## 证据');add('');out.extend(EVIDENCE);add('')
    add('![论文 Fig. 11 与方程、拟合的对比](D_paper_fit.png)');add('')
    add('H∞ 与论文方程重合；导纳偏离方程，加入拟合的 W 后才吻合（13–17.5 s 的滚转过渡不参与拟合）。');add('')
    add('|论文 Fig. 11|峰值力矩 / Nm（roll / pitch / yaw）|第一阶段 W（roll / pitch / yaw）|R²：方程 → 拟合|第二阶段 W|R²：方程 → 拟合|');add('|---|---:|---:|---:|---:|---:|')
    for column in ORDER:
        a=p[column]['phases']['phase1'];b=p[column]['phases']['phase2']
        add(f'|{NAMES[column]}|{fmt(p[column]["peak_moment"],1)}|{fmt(a["W"])}|{fmt(a["ideal_r2"])} → {fmt(a["fit_r2"])}|{fmt(b["W"])}|{fmt(b["ideal_r2"])} → {fmt(b["fit_r2"])}|')
    add('');add('导纳 roll 的力矩很小（两个阶段有效值约 .55 Nm），这一项的 W 不可靠。普通阻抗 yaw 被摩擦卡住，线性拟合 R² 只有约 .4。');add('')
    add('## 候选机理');add('')
    add('输入均为 Fig. 11 的 H∞ 力矩 × .6，只看导纳。`W` 依次为 roll / pitch / yaw。');add('')
    add('|候选|状态|第一阶段 W|第二阶段 W|pitch / rad|yaw / rad|');add('|---|---|---:|---:|---:|---:|')
    def line(name,row):
        status='跑完' if row['status']=='completed' else f"终止（{row['duration']:.1f} s）"
        a=row['phases'].get('phase1');b=row['phases'].get('phase2')
        return f"|{name}|{status}|{fmt(a['W']) if a else '—'}|{fmt(b['W']) if b else '—'}|{row['pitch']:.4f}|{row['yaw']:.3f}|"
    add(line('基线（论文方程）',rows['baseline']['admittance']))
    for label,name in CANDIDATES:
        add(line(name,rows[label]['admittance']))
    add('');add('未抵消力通路这一行只有 3 s 数据，W 不可用。');add('');own=rows['feedthrough_own']['admittance']
    add(f"用导纳自己那一列的力矩时，未抵消力通路的 W 为第一阶段 {fmt(own['phases']['phase1']['W'])}（该运行的 `ΛMx⁻¹` 为 {fmt(own['phases']['phase1']['predicted'])}），第二阶段 {fmt(own['phases']['phase2']['W'])}（{fmt(own['phases']['phase2']['predicted'])}）。对角项基本恢复了已知真值，说明这个辨识方法可靠。");add('')
    add('排除依据：');add('');out.extend(EXCLUDED);add('')
    add('## 四种控制器');add('')
    add('|控制器|论文倍数 pitch / yaw|FR3 论文方程|FR3 未抵消力通路|FR3 未抵消力通路：pitch / rad，yaw / rad，限幅|');add('|---|---:|---:|---:|---:|')
    for column in ORDER:
        base=rows['baseline_own'][column];feed=rows['feedthrough_own'][column]
        refb=rows['baseline_own']['hinf'];reff=rows['feedthrough_own']['hinf']
        add(f"|{NAMES[column]}|{PAPER['pitch'][column]/PAPER['pitch']['hinf']:.2f} / {PAPER['yaw'][column]/PAPER['yaw']['hinf']:.2f}|{base['pitch']/refb['pitch']:.2f} / {base['yaw']/refb['yaw']:.2f}|{feed['pitch']/reff['pitch']:.2f} / {feed['yaw']/reff['yaw']:.2f}|{feed['pitch']:.4f}，{feed['yaw']:.3f}，{feed['saturation']*100:.2f}%|")
    add('');add(f'论文折算到 × {SCALE} 后，导纳 pitch .0130 rad、yaw .105 rad。H∞ 这一行用 H∞ 那一列的力矩。');add('')
    add('## C 和 B');add('')
    add('![C 的平均法向力](D_force_c.png)');add('')
    add('|C 平均力 / N|平均|标准差|每 5 s 平均|');add('|---|---:|---:|---:|')
    add(f"|论文导纳|{c['paper']['admittance']['mean']:.2f}|{c['paper']['admittance']['std']:.2f}|{fmt(c['paper']['admittance']['bins'],1)}|")
    add(f"|论文 H∞|{c['paper']['hinf']['mean']:.2f}|{c['paper']['hinf']['std']:.2f}|{fmt(c['paper']['hinf']['bins'],1)}|")
    for key,name in (('baseline/admittance','FR3 导纳，论文方程'),('feedthrough/admittance','FR3 导纳，未抵消力通路'),('baseline/hinf','FR3 H∞，论文方程'),('feedthrough/hinf','FR3 H∞，未抵消力通路')):
        add(f"|{name}|{c['sim'][key]['mean']:.2f}|{c['sim'][key]['std']:.2f}|{fmt(c['sim'][key]['bins'],1)}|")
    feed=c['sim']['feedthrough/admittance']
    add('');add(f"期望力平均 {c['paper_desired_mean']:.2f} N。FR3 的 z 向 Mz={feed['mass_z']:.1f} kg，预测比例 {feed['predicted_ratio']:.2f}，实际 {feed['mean']/c['paper_desired_mean']:.2f}。论文曲线来自约 10 ms 一点的矢量线。");add('')
    add('|B 或 A|论文方程：峰值力 / N，平衡偏差 / mm|未抵消力通路：峰值力 / N，平衡偏差 / mm|');add('|---|---:|---:|')
    ba=payload['ba']
    for column in ORDER:
        x=ba[f'B/baseline/{column}'];y=ba[f'B/feedthrough/{column}']
        add(f"|B {NAMES[column]}|{x['peak_force']:.1f}，{x['equilibrium']*1000:.1f}|{y['peak_force']:.1f}，{y['equilibrium']*1000:.1f}|")
    add('');add(f"A 是自由运动，不受影响：导纳位置 RMS {ba['A/baseline/admittance']['position_rms']*1000:.2f} → {ba['A/feedthrough/admittance']['position_rms']*1000:.2f} mm，普通阻抗 {ba['A/baseline/impedance']['position_rms']*1000:.2f} → {ba['A/feedthrough/impedance']['position_rms']*1000:.2f} mm。");add('')
    add('## 与 Indy7 模型核对');add('')
    indy=payload['indy7'];obs=indy['observed_inverse_inertia']
    add(f'用 [Tadese 等 2022](https://pmc.ncbi.nlm.nih.gov/articles/PMC9783800/) 的 DH 表和基参数计算 `J M⁻¹ Jᵀ`，不需要 URDF。A2 的数值已对照原文。A1 的线性组合用随机全参数验证，与 DH 表一致（M 的最大差约 5e-4，来自表中常数的舍入）。D 的姿态按 Fig. 10 取工具朝下、再绕 x 滚转 −90°，并扫描 TCP 位置、工具长度和 ±15° 倾角，共 {indy["poses_used"]} 个。');add('')
    add('|`Mx⁻¹` 对角项|第一阶段 pitch|第一阶段 yaw|第二阶段 pitch|第二阶段 yaw|第二阶段 roll|四项 ±20% 的姿态数|C 的 Mz / kg|');add('|---|---:|---:|---:|---:|---:|---:|---:|')
    add(f"|**论文导纳隐含值**|**{obs['phase1']['pitch']:.2f}**|**{obs['phase1']['yaw']:.2f}**|**{obs['phase2']['pitch']:.2f}**|**{obs['phase2']['yaw']:.2f}**|**{obs['phase2']['roll']:.2f}**|—|**{payload['implied_mass_z'][0]:.0f}–{payload['implied_mass_z'][1]:.0f}**|")
    for label,entry in indy['models'].items():
        name,kind=label.split('/');kind='不计转子' if kind=='no_rotor' else '计入转子'
        cell=lambda a,b:f"{entry[a][b]['median']:.2f}"
        add(f"|{name}，{kind}|{cell('phase1','pitch')}|{cell('phase1','yaw')}|{cell('phase2','pitch')}|{cell('phase2','yaw')}|{cell('phase2','roll')}|{entry['poses_within_20_percent']}|{entry['mass_z_c']['median']:.1f}|")
    add('');add('模型列为姿态扫描的中位数。Mz 沿 C 的轨迹计算，高度未公开，取 .1–.2 m；关节 1、2 的转子惯量在基参数中分不开，所以“不计转子”只去掉关节 3–6 的项。');add('')
    add('|参数组|D 需要的 c（四项，不含 roll）|C 需要的 c|');add('|---|---:|---:|')
    for label,entry in indy['models'].items():
        name,kind=label.split('/');kind='不计转子' if kind=='no_rotor' else '计入转子'
        need=[v for k,v in entry['coefficient_needed'].items() if not k.endswith('roll')]
        low,high=payload['c_needed'][label]
        add(f'|{name}，{kind}|{fmt(need)}|{low:.2f}–{high:.2f}|')
    add('');add(f"C 的 c 由力比例 {payload['force_ratio_range'][0]:.2f}–{payload['force_ratio_range'][1]:.2f} 算出（高值是全程，低值去掉接触门槛所在的 15–20 s），并按仿真标定：FR3 已知 c=1 时，同样的算法得到 {payload['sim_apparent_c']:.2f}。");add('')
    out.extend(INDY7_FINDINGS);add('')
    add('## 不确定之处');add('');out.extend(OPEN);add('')
    add('## 对复现的含义');add('');out.extend(MEANING);add('')
    add('## 重跑');add('');add('```bash')
    add('.venv/bin/python scripts/digitize_paper_c.py')
    add('OPENBLAS_NUM_THREADS=1 PYTHONPATH=scripts .venv/bin/python scripts/validate_d_admittance.py --workers 8 --resume')
    add('OPENBLAS_NUM_THREADS=1 PYTHONPATH=scripts .venv/bin/python scripts/check_indy7_inertia.py')
    add('OPENBLAS_NUM_THREADS=1 PYTHONPATH=scripts .venv/bin/python scripts/report_d_admittance.py')
    add('```');add('');add('[机器可读结果](metrics.json) · [测试记录](VALIDATION.txt)');add('')
    return '\n'.join(out)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/d-admittance'))
    parser.add_argument('--reference',type=Path,default=Path('outputs/d-operator'))
    parser.add_argument('--c-curves',type=Path,default=Path('outputs/paper-c-digitized/curves.npz'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/d-admittance'))
    args=parser.parse_args()
    indy=json.loads((args.output/'indy7_mx.json').read_text())
    manifest=json.loads((args.root/'manifest.json').read_text())
    if manifest['errors'] or set(manifest['completed'])!=set(cases()):raise ValueError('Incomplete candidate matrix')
    args.output.mkdir(parents=True,exist_ok=True)
    curves=dict(np.load('fr3imp/data/paper_d_curves.npz'))
    rows={}
    for name in cases():
        label,job=name.split('/')
        experiment,controller=job.split('_')
        if experiment=='D':rows.setdefault(label,{})[controller]=sim_row(args.root,label,controller)
    # H-infinity has no separate own-column run: its own column is the default input.
    rows['baseline_own']['hinf']=rows['baseline']['hinf']
    rows['feedthrough_own']['hinf']=rows['feedthrough']['hinf']
    payload=dict(source_sha256=manifest['source_sha256'],paper=paper_identification(curves),rows=rows,
                 c=c_results(args.root,args.c_curves),ba=ba_results(args.root),
                 regression_pose_difference=regression_check(args.root,args.reference))
    payload['indy7']=indy
    c=payload['c'];feed=c['sim']['feedthrough/admittance']
    payload['sim_apparent_c']=(1/(feed['mean']/c['paper_desired_mean'])-1)*feed['mass_z']/10
    bins=c['paper']['admittance']['bins']
    ratios=(float(np.mean([bins[i] for i in (0,1,4,5)]))/c['paper_desired_mean'],c['paper']['admittance']['mean']/c['paper_desired_mean'])
    payload['force_ratio_range']=list(ratios)
    payload['implied_mass_z']=[10/(1/r-1) for r in ratios]
    payload['c_needed']={label:[(1/ratios[1]-1)*entry['mass_z_c']['median']/10/payload['sim_apparent_c'],
                                (1/ratios[0]-1)*entry['mass_z_c']['median']/10/payload['sim_apparent_c']] for label,entry in indy['models'].items()}
    fit_figure(curves,args.output/'D_paper_fit.png')
    ratio_figure(rows,args.output/'D_ratios.png')
    force_figure(payload['c'],args.output/'D_force_c.png')
    (args.output/'metrics.json').write_text(json.dumps(payload,indent=2,ensure_ascii=False))
    (args.output/'REPORT.md').write_text(report(payload,args.root))
    print(json.dumps(dict(runs=len(cases()),regression=payload['regression_pose_difference']),indent=2))


if __name__=='__main__':
    main()
