"""Audit the fixed inertia/grasp probes without promoting failures to defaults."""
import argparse
from collections import Counter
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from fr3imp.analyze import COLORS,LABELS
from fr3imp.evaluation import impedance_metrics
from fr3imp.scenarios import Scenario
from report_reproduction import check_record
from validate_portability import cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/portability'))
    parser.add_argument('--output',type=Path,default=Path('docs/results/portability'))
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((args.root/'manifest.json').read_text());jobs=cases()
    if manifest['errors'] or set(manifest['completed'])!=set(jobs):raise ValueError('Incomplete portability matrix')
    records={};selected={}
    for name,s in sorted(manifest['completed'].items()):
        expected=jobs[name]
        if s['impedance']!=expected['impedance_overrides']:raise ValueError('Impedance mismatch')
        folder=args.root/name/f"{s['experiment']}_{s['controller']}_trial00_seed100"
        record,a=check_record(s,folder,manifest['source_sha256'])
        scenario=Scenario(s['experiment'],a['desired'][0],variant=s['scenario_variant'],
            initial_q=s['initial_q'],impedance_overrides=s['impedance'],
            operator_stiffness_scale=s['operator_stiffness_scale'])
        independent,_,_=impedance_metrics(a,scenario)
        for key in ['dynamic_impedance_position_rmse_m','dynamic_impedance_rotation_rmse_rad']:
            np.testing.assert_allclose(independent[key],s['metrics'][key],rtol=1e-10,atol=1e-12)
        records[name]=record
        if name.startswith(('literal_B','inertia_eighth_B')):selected[name]=(s,a)
    fig,axes=plt.subplots(2,3,figsize=(13,6),layout='constrained',sharex=True)
    for row,label in enumerate(['literal','inertia_eighth']):
        for c in ['admittance','hinf']:
            s,a=selected[f'{label}_B_{c}'];t=a['time'];color=COLORS[c]
            axes[row,0].plot(t,np.linalg.norm(a['force_contact'][:,:3],axis=1),color=color,lw=.55,label=LABELS[c])
            axes[row,1].plot(t,a['torque_applied'][:,6],color=color,lw=.55)
            axes[row,2].plot(t,np.linalg.norm(a['reference_error'][:,:3],axis=1),color=color,lw=.55)
        axes[row,0].set_ylabel(('Literal inertia' if row==0 else 'Inertia / 8')+'\nContact force [N]')
        axes[row,1].set_ylabel('Joint 7 torque [Nm]');axes[row,2].set_ylabel('REF norm [m/s]')
        axes[row,0].legend(fontsize=8)
        for ax in axes[row]:ax.set(xlim=(0,30),xlabel='Time [s]');ax.grid(alpha=.15)
    fig.suptitle('B: lower rendered inertia exposes admittance chatter; stiffness and damping ratio fixed')
    fig.savefig(out/'B_inertia.png',dpi=160);plt.close(fig)
    states=Counter(s['status'] for s in manifest['completed'].values())
    rows=['# 跨机器人惯量与握持对照','',
          f'固定 48 次运行：{dict(states)}；代码 SHA256：`{manifest["source_sha256"]}`。','',
          '**B 的低惯量对照出现了论文相似的导纳抖振 / H∞ 稳定差异，但统一缩放不能解决全部实验。C、D 的负面结果保留，未替换默认论文参数。**','',
          '## 为什么检查惯量比例','',
          '当前 FR3 名义模型 home 处第七轴质量矩阵对角元约 0.07412 kg·m²。另一项 [Indy7 参数辨识研究](https://pmc.ncbi.nlm.nih.gov/articles/PMC9783800/)附录表 A1/A2 给出末轴转子/齿轮惯量约 0.5405、连杆轴惯量约 0.0705 kg·m²（作者提出方法、低温列），合计约 0.611。该研究不是本篇 TRO 的同一台实物标定，关节惯量也不能直接替代整个 Cartesian 惯量矩阵。此数量级差异仅用于提出可检验假设。',
          '', '矩阵固定包含：B/C/D 的原参数、Λ×1/4、Λ×1/8；缩放时 D×sqrt(scale)、K 不变，保持每轴阻尼比。D 另含虚拟握持刚度 ×4/×8，以及 Λ×1/8 配合握持 ×4。所有比较控制器使用完全相同的任务/握持变化，控制器反馈增益保持原值。它们是机制对照，不是原论文参数或已标定的动力学等价变换。','',
          '## 全部结果','',
          '|案例|状态 / 时长|位置阻抗动态偏差 / mm|旋转阻抗动态偏差 / deg|限幅 / %|物理峰值力 / N|',
          '|---|---|---:|---:|---:|---:|']
    for name,r in records.items():
        s=r['summary'];m=s['metrics']
        rows.append(f'|{name}|{s["status"]} / {s["simulated_duration_s"]:.3f} s|{m["dynamic_impedance_position_rmse_m"]*1000:.3f}|{m["dynamic_impedance_rotation_rmse_rad"]*180/np.pi:.3f}|{m["saturation_fraction"]*100:.3f}|{m["force_peak_N"]:.2f}|')
    rows+=['','中止前指标不能当作完整时长统计。运行完成也不表示稳定：例如持续限幅必须单独检查。动态偏差来自完整 Λ/D/K 方程，不把期望柔顺位移本身判为跟踪失败。','',
           '## 可支持的结论','',
           '- B：Λ×1/8 时，普通导纳限幅约 7.2%，H∞ 无限幅；接触峰值约 74.6 N 对 30.2 N。原参数下普通导纳主要保持稳定。说明 FR3 上可通过降低共同的期望惯量检验论文强调的低惯量交互机制；没有将有限抖振写成论文同等急停。',
           '- C：统一降低包括法向与旋转轴的惯量，会同时损害 H∞，不能作为有效的全局移植规则。后续应独立处理运动轴、力控轴与采样接口。',
           '- D：较硬握持暴露了 1 kHz / 100 Hz 命令接口下的限制，部分 H∞ 设置提前终止；原参数的导纳动态误差仍很小。此结果不支持“降低惯量即可复现论文中的全部优劣关系”。','',
           '![B 惯量对照](B_inertia.png)','',
           '[完整配置、原始轨迹哈希与指标](metrics.json)','',
           '```bash','OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_portability.py --workers 8',
           'OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/report_portability.py',
           '# 单独复核 B 的低惯量对照；显式参数不会更改默认值',
           '.venv/bin/python -m fr3imp --experiment B --controller hinf --inertia-scale .125 --output outputs/B-low-inertia',
           '```']
    (out/'REPORT.md').write_text('\n'.join(rows))
    (out/'metrics.json').write_text(json.dumps(dict(source_sha256=manifest['source_sha256'],runs=records),indent=2))
    print(json.dumps(dict(runs=len(records),states=dict(states),output=str(out)),ensure_ascii=False))


if __name__=='__main__':main()
