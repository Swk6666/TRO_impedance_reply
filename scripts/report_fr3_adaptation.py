"""Keep the complete pre-profile adaptation record, including negative cases."""
from collections import Counter
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    out=ROOT/'docs/results/fr3-adaptation';out.mkdir(parents=True,exist_ok=True)
    names=['portability','axis-adaptation','torque-scaling','torque-scaling-roll','fr3-candidate']
    groups={name:json.loads((ROOT/'outputs'/name/'manifest.json').read_text()) for name in names}
    records={}
    for name,m in groups.items():
        if m['errors']:raise ValueError((name,m['errors']))
        for case,s in m['completed'].items():
            if s['experiment']=='E':raise ValueError('Deferred experiment outside adaptation scope')
            if s['source_sha256']!=m['source_sha256']:raise ValueError((name,case,'source mismatch'))
            records[name+'/'+case]=s
    states=Counter(s['status'] for s in records.values())
    rows=['# FR3 参数适配审计（不含 E）','',
          f'五个诊断批次共 {len(records)} 次，状态 {dict(states)}。全部结果均保留，包含速度越界和任务参数不适用的情况。本文汇总的是选定 `fr3` 配置前的诊断，不替代 [最终 134+51 次验证](../reproduction-fr3/REPORT.md)。','',
          '## 从原参数到明确适配','',
          '1. `paper` 的 A/F 已有主要趋势，但 B 普通导纳未失稳，C 抖振主要局限在挡块，D 的若干差异不明显。机械臂惯量、减速传动、摩擦、力矩限制与采样不同，原参数不能直接保证相同闭环表现。',
          '2. 全轴降低虚拟惯量使 B 出现接触抖振差异，却使 C 的 H∞ 限幅、D 的强握持过载。没有将这种统一缩放作为默认规则。',
          '3. C 仅缩放 x/y 运动惯量和阻尼，就能在平板阶段出现导纳抖振，保留 z 力控制阻抗和角轴参数；这是按轴隔离机制的证据。',
          '4. D 只降低增益、只降低惯量或额外延迟一次 F/T 均未解决强握持问题。将旋转 Λ/D/K、yaw/pitch 交互力矩与相关反馈尺度一起调整后，H∞ 不再限幅。全量滚转引导恢复约 90° 翻转；因普通阻抗释放后仍漂移，所有控制器统一加入第二阶段被动滚转支撑。这一附加条件与论文的自由滚转释放不同，已明确披露。',
          '5. 统一降低 NRIC G 的候选使 B/C 普通导纳变得稳定。因此最终 A/B/C 保留其原 G，仅 D 与旋转尺度一起调整。不能从最终有限配置得出导纳算法必然失稳或某算法总是最优。',
          '6. F 在较高绝对增益时，REF 仍近似反比下降，但静态偏差接近仿真噪声/接触误差平台，不再严格单调。以 FR3 基准 α=.25 扫描相对倍率 .25/.5/1，才同时观察到 REF 和静态偏差下降；实际有效 α=.0625/.125/.25 在图表和日志中明示。','',
          '这些是公开保留选择过程的仿真适配，未根据论文图像改写力或关节日志。候选参数经过主矩阵和新的种子/步长检查；没有把所有探索性配置算作成功复现。','',
          '## 批次与源码','',
          '|批次|数量|状态|运行时代码 SHA256|','|---|---:|---|---|']
    for name,m in groups.items():
        rows.append(f'|[{name}](../../../outputs/{name}/manifest.json)|{len(m["completed"])}|{dict(Counter(s["status"] for s in m["completed"].values()))}|`{m["source_sha256"]}`|')
    rows+=['','每批 `outputs/<批次>/source/` 保留对应源码；其配置不可与后续默认参数混用。`metrics.json` 保存下列各次完整摘要。原始 NPZ 仍位于对应 outputs 目录。','',
           '## 完整运行清单','',
           '列出的误差来自各自实际运行时长；提前终止与完整 30/35 s 运行不可直接等同。D 改变了旋转任务，因此跨不同旋转刚度批次的静态误差也不可直接排名。','',
           '|批次 / 案例|状态 / 时长 s|C 力 RMS / N|动态位置偏差 / mm|限幅比例|终止原因|',
           '|---|---|---:|---:|---:|---|']
    for name,s in sorted(records.items()):
        m=s['metrics'];force=f'{m["force_tracking_rmse_N"]:.3f}' if 'force_tracking_rmse_N' in m else '—'
        rows.append(f'|{name}|{s["status"]} / {s["simulated_duration_s"]:.3f}|{force}|{m["dynamic_impedance_position_rmse_m"]*1000:.3f}|{m["saturation_fraction"]:.3%}|{s["termination_reason"] or "—"}|')
    (out/'REPORT.md').write_text('\n'.join(rows))
    (out/'metrics.json').write_text(json.dumps(dict(status=dict(states),runs=records),indent=2))
    print(dict(runs=len(records),status=dict(states),output=str(out)))


if __name__=='__main__':main()
