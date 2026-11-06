"""Report physically generated grinder vibration, including failed assumptions."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from fr3imp.analyze import COLORS,LABELS
from report_reproduction import check_record
from validate_grinding import cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/grinding'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/grinding'))
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((args.root/'manifest.json').read_text())
    if manifest['errors'] or set(manifest['completed'])!=set(cases()):raise ValueError('Incomplete grinder matrix')
    records={};selected={}
    for name,s in sorted(manifest['completed'].items()):
        folder=args.root/name/f"E_{s['controller']}_trial00_seed{s['seed']}"
        r,a=check_record(s,folder,manifest['source_sha256'])
        if a['q_measured'].shape[1]!=7 or a['plant_q'].shape[1]!=8:raise ValueError('Wrong arm/rotor separation')
        free=a['time']<3;contact=a['time']>=6
        np.testing.assert_array_equal(a['force_true'][free],0.)
        if np.any(a['grinding_power']>1e-6):raise ValueError('Friction injects energy')
        bound=s['plant']['contact_friction']*a['grinding_normal_load']
        if np.any(np.linalg.norm(a['grinding_wrench'][:,:3],axis=1)>bound+1e-5):raise ValueError('Friction exceeds Coulomb bound')
        diag=r['diagnostics']
        diag['free_measured_force_rms_N']=float(np.sqrt(np.mean(np.sum(a['force_measured'][free,:3]**2,axis=1))))
        diag['rotor_speed_mean_rpm']=float(np.mean(a['plant_qvel'][:,-1])*60/(2*np.pi))
        diag['rotor_motor_peak_Nm']=float(np.max(np.abs(a['rotor_motor_torque'])))
        if contact.any():
            for key in ('force_true','force_measured'):
                diag[key+'_normal_mean_N']=float(np.mean(a[key][contact,2]))
                diag[key+'_normal_std_N']=float(np.std(a[key][contact,2]))
                diag[key+'_normal_peak_N']=float(np.max(a[key][contact,2]))
            diag['grinding_mean_power_W']=float(np.mean(a['grinding_power'][contact]))
            lost=(a['environment_contact']==0)&contact
            edges=np.diff(np.r_[False,lost,False].astype(int));lengths=np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)
            diag['longest_contact_loss_ms']=float(max(lengths,default=0))
        records[name]=r
        if name.startswith('main_'):selected[s['controller']]=(s,a)
    payload=dict(source_sha256=manifest['source_sha256'],runs=records)
    (out/'metrics.json').write_text(json.dumps(payload,indent=2))
    controllers=['bidirectional','smc','hinf']
    fig,axes=plt.subplots(3,3,figsize=(13,8),layout='constrained',sharex=True)
    for row,c in enumerate(controllers):
        s,a=selected[c];t=a['time'];color=COLORS[c]
        axes[row,0].plot(t,1000*(a['pose'][:,0]-a['desired'][0,0]),color=color,lw=.7,label=LABELS[c])
        axes[row,0].plot(t,1000*(a['desired'][:,0]-a['desired'][0,0]),'k--',lw=.8,label='Desired')
        axes[row,1].plot(t,a['force_true'][:,2],color='#aaaaaa',lw=.5,label='Physical contact')
        axes[row,1].plot(t,a['force_measured'][:,2],color=color,lw=.5,label='Measured F/T')
        axes[row,2].plot(t,np.linalg.norm(a['reference_error'][:,:3],axis=1),color=color,lw=.6)
        for col,title in enumerate(['x displacement [mm]','Normal force [N]','REF norm [m/s]']):
            axes[row,col].set(xlabel='Time [s]',ylabel=title,xlim=(0,30));axes[row,col].grid(alpha=.15)
        axes[row,0].legend(fontsize=8);axes[row,1].legend(fontsize=8)
    fig.suptitle('E: internal rotor and rotating-disk contact, identical plant for all controllers')
    fig.savefig(out/'E_rotor.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,6),layout='constrained')
    for c in controllers:
        _,a=selected[c];t=a['time'];color=COLORS[c];mask=(t>=1)&(t<=1.06)
        axes[0,0].plot(t[mask]-1,a['force_measured'][mask,2],color=color,lw=.8,label=LABELS[c])
        axes[0,1].plot(t,a['beta'],color=color,lw=.5)
        if c=='bidirectional':
            axes[0,1].plot(t[99:],np.convolve(a['beta'],np.ones(100)/100,mode='valid'),
                           color='black',lw=1.1,label='100 ms mean')
            axes[0,1].legend(fontsize=8)
        axes[1,0].plot(t,a['grinding_power'],color=color,lw=.4)
        axes[1,1].plot(t,a['plant_qvel'][:,-1]*60/(2*np.pi),color=color,lw=.5)
    axes[0,0].axhline(0,color='k',ls='--',label='True external force')
    axes[0,0].set(xlabel='Time from 1 s [s]',ylabel='Free-space measured Fz [N]');axes[0,0].legend(fontsize=8)
    axes[0,1].set(xlabel='Time [s]',ylabel='Energy-flow modulation beta',ylim=(-.05,1.05))
    axes[1,0].set(xlabel='Time [s]',ylabel='Sliding friction power [W]')
    axes[1,1].set(xlabel='Time [s]',ylabel='Rotor speed [rpm]')
    for ax in axes.flat:ax.grid(alpha=.15)
    fig.suptitle('E mechanism checks: sensor vibration without free-space external force; dissipative grinding')
    fig.savefig(out/'E_mechanism.png',dpi=160);plt.close(fig)
    states=Counter(s['status'] for s in manifest['completed'].values())
    rows=['# E：内部转子与旋转圆盘接触','',
          f'固定 36 次运行：{dict(states)}。运行时代码 SHA256：`{manifest["source_sha256"]}`。','',
          '此轮补齐工具振动的物理来源，保留所有未完成的敏感性运行。尚未复现论文中双向导纳 100–150 N、β 接近 1，以及 SMC 明显反弹的完整组合。','',
          '## 模型与来源','',
          '- [Bosch 原厂资料](https://www.bosch-pt.com.my/my/en/products/gws-7-100-et-06013885L0)给出 GWS 7-100 ET 的 1.8 kg、100 mm 圆盘和 2,800–9,300 rpm。仿真采用总质量 1.8 kg、9,000 rpm；转子质量 0.1 kg、偏心 0.1 mm、工具 COM 60 mm、摩擦系数 .25 均是未经标定的假设。',
          '- 真实系统新增第八个转子自由度，名义控制器仍只接收 FR3 的七轴测量。转速伺服连续运行，限矩 .5 Nm；七轴控制与 F/T 仍为 1 kHz。静态工具重力单独校准，动态惯性保留在测量中。',
          '- 自由空间真实外力严格为零，腕部 F/T 的振动由转子反作用产生。固定工具测试验证 `m e ω²` 和 TCP 力矩变换。旧的外加正弦力/额外 1.5 N 白噪声通过 `--grinder-model external` 保留。',
          '- 圆盘使用 MuJoCo 法向接触，加显式耗散滑动摩擦 `F_t = −μ F_n v_t / sqrt(v_t²+.01²)`，每个物理子步更新，施加在转子实际接触点。逐记录核对摩擦功率非正及 `|F_t| ≤ μ F_n`，并测试接触功率与广义力功率一致。',
          '- 约 47 m/s 轮缘速度下，直接使用软摩擦锥曾在第一次接触产生约 1.5 kN 法向力；这属于被诊断的数值异常，未当作算法效果。新模型仍是光滑工件接触，不包含材料去除、磨粒或热过程。','',
          '## 全部结果','',
          '|案例|状态 / 时长|x RMS / mm|REF 均值 / m/s|真实峰值 / N|接触丢失 / %|最长丢失 / ms|',
          '|---|---|---:|---:|---:|---:|---:|']
    for name,r in records.items():
        s=r['summary'];m=s['metrics'];d=r['diagnostics']
        rows.append(f'|{name}|{s["status"]} / {s["simulated_duration_s"]:.3f} s|{m.get("tangential_tracking_rmse_m",float("nan"))*1000:.3f}|{m["reference_error_position_mean_m_s"]:.5f}|{m["force_peak_N"]:.2f}|{m.get("contact_loss_fraction_after_6s",float("nan"))*100:.4f}|{d.get("longest_contact_loss_ms",float("nan")):.0f}|')
    rows+=['','中止案例的指标仅覆盖中止前，不能与完整 30 s 的统计直接等同。接触丢失按几何接触活动标志统计，1 ms 缺口不能直接称为可见反弹。峰值只覆盖 1 kHz 样本。','',
           '## 积分细化','',
           '|控制器|子步|REF 均值 / m/s|接触期物理 Fz 标准差 / N|平均滑动功率 / W|',
           '|---|---:|---:|---:|---:|']
    for c in controllers:
        for label,n in [('main',4),('physics8',8),('physics16',16)]:
            r=records[label+'_'+c];s=r['summary'];d=r['diagnostics']
            rows.append(f'|{c}|{n}|{s["metrics"]["reference_error_position_mean_m_s"]:.6f}|{d["force_true_normal_std_N"]:.4f}|{d["grinding_mean_power_W"]:.3f}|')
    rows+=['','RMS/均值与逐采样峰值分开报告；接触峰值对种子和碰撞切换敏感，不能称为逐样本收敛。',
           '', '![E 对比](E_rotor.png)','','![内部振动与功率](E_mechanism.png)','',
           '[完整配置、轨迹哈希与指标](metrics.json)','',
           '重跑：`OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_grinding.py --workers 8`，随后运行 `scripts/report_grinding.py`。历史记录的可运行源码在对应输出目录 `source/`；以该目录为工作目录调用项目 Python，即可回放同版本记录。']
    (out/'REPORT.md').write_text('\n'.join(rows))
    print(json.dumps(dict(runs=len(records),states=dict(states),output=str(out)),ensure_ascii=False))


if __name__=='__main__':main()
