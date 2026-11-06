"""Evidence and prose for the explicitly adapted FR3 profile (A/B/C/D/F)."""
import json
from collections import Counter
import numpy as np


def overview(manifest,records,extras,selected):
    def metric(e,c,k):return selected[e,c][0]['metrics'][k]
    def values(e,controllers,key,scale=1,digits=3):
        return ' / '.join(f'{metric(e,c,key)*scale:.{digits}f}' for c in controllers)
    states=Counter(r['summary']['status'] for r in [*records.values(),*extras.values()])
    rows=['# FR3 适配复现：A、B、C、D、F','',
          '当前范围见 [AGENTS.md](../../../AGENTS.md)。E 砂轮机按用户要求暂缓，既有代码和历史记录保留。','',
          f'`fr3` 正式矩阵 {len(records)} 次（D 每控制器 {manifest["d_trials"]} 次），固定敏感性矩阵 {len(extras)} 次；状态：{dict(states)}。',
          f'运行时代码 SHA256：`{manifest["source_sha256"]}`。','',
          '**本报告是参数已明确调整的 FR3/MuJoCo 仿真，不能称为论文原参数或真实机械臂复现。** `paper` 保留论文 Λ/D/K 和反馈增益，[此前的原参数对照](../reproduction-active/REPORT.md)表明，直接移植时 B/C/D 的若干现象不会出现。',
          '用户已确认未知工具质量、材料与滤波采用仿真假设。所有对比使用相同工具、物理环境、采样率、任务和虚拟操作者规律。没有改写记录中的实际轨迹、注入专属扰动或删去失败配置。','',
          '## 与论文相似的主要效果','',
          '|组|当前证据|仍需区分的边界|','|---|---|---|',
          f'|A|普通阻抗 / 导纳 / H∞ 位置 RMS：{values("A",["impedance","admittance","hinf"],"position_rmse_m",1000)} mm；两种鲁棒反馈显著改善低任务刚度下的跟踪|绝对误差属于 FR3 仿真|',
          f'|B|普通导纳接触时抖振并有 {metric("B","admittance","saturation_fraction"):.2%} 采样限幅；H∞ 无限幅，动态阻抗位置偏差 {metric("B","hinf","dynamic_impedance_position_rmse_m")*1000:.3f} mm；双向调制偏差 {metric("B","bidirectional","dynamic_impedance_position_rmse_m")*1000:.3f} mm|导纳仍完成 30 s，没有伪称复现原机急停|',
          f'|C|导纳 / 双向 / H∞ 真实法向力 RMS：{values("C",["admittance","bidirectional","hinf"],"force_tracking_rmse_N")} N；导纳在平板绘制阶段也出现抖振，H∞ 保持力跟踪|非原工具材料；振幅与源图不能逐点等同|',
          '|D|先后在约 0° 与 90° 滚转姿态下执行 yaw/pitch 交互；H∞ 动态响应接近期望阻抗，导纳有短时限幅，滑模瞬态偏差更大；下表含全部重复|虚拟手在第二阶段继续提供滚转支撑；这是显式附加假设，未复现真人握持感受或每一项统计排名|',
          f'|F|相对增益 .25 / .5 / 1 的 REF 均值：{values("F",["hinf_0.25","hinf_0.5","hinf_1"],"reference_error_position_mean_m_s",digits=5)} m/s；增益翻倍时接近减半；静态偏差均值 {values("F",["hinf_0.25","hinf_0.5","hinf_1"],"equilibrium_position_error_mean_m",1000)} mm|有效 α 为 .0625 / .125 / .25；不是论文的绝对增益。静态偏差降幅较小，其时间标准差未随增益下降|','',
          '[机器可读效果检查](effect_checks.json)同时覆盖种子 100/101/102、物理步长细化和主矩阵；这些检查不是稳定性或无源性证明。','',
          '## 适配参数与选择依据','',
          '|范围|相对 `paper` 的变化|','|---|---|',
          '|全部活动实验|H∞ 基准反馈 ×.25；F 再乘 .25/.5/1。100 Hz 公共命令低通、1 kHz 控制/F/T、250 μs 物理步长；无力信号低通|',
          '|A|Λ/D/K 不变；NRIC G 不变|',
          '|B、F|全部任务轴 Λ×.125、D×sqrt(.125)、K 不变；B 的 NRIC G 不变|',
          '|C|仅 x/y 的 Λ×.25、D×.5；力轴 z 与三个角轴的 Λ/D/K 不变；NRIC G 不变|',
          '|D|三个角轴的 Λ/D/K 同乘 .25；NRIC G×.25；平移阻抗不变。yaw/pitch 虚拟手弹簧×4，输出广义力矩×.25（上限 3 Nm）；滚转引导保持原量级并延长至试验结束|',
          '|SMC 与普通阻抗|原控制律增益保持；使用与该实验所有控制器相同的任务阻抗和外部环境|','',
          'D 的 NRIC 比例与旋转任务比例一起调整；其余导纳保留论文 G。比较的是各完整配置，不能据此断言算法在任意等增益优化下的排名。所有实际 Λ/D/K、控制增益、交互参数均保存在每次 `config.json` 和本报告 `metrics.json`。',
          '选择过程保留了不利证据：[48 次统一惯量/握持对照](../portability/REPORT.md)显示，统一降低惯量会损害 C/D；轴向对照才定位到 C 的平面运动与接触耦合。另一个统一将 NRIC G 降为 .25 的候选使 B/C 导纳恢复稳定，因此不能把“导纳必然不稳定”作为结论。完整候选记录和源码快照见 [适配审计](../fr3-adaptation/REPORT.md)。','',
          '## D：30 次虚拟操作者重复','',
          '每控制器使用同样 10 组握持参数 × 3 个噪声种子。下表只统计完整时长运行，并显示完整/总数；均值 ± 样本标准差不是真人置信区间。动态偏差独立积分该运行实际 Λ/D/K，避免把过渡过程只与静态 F/K 比较。','',
          '|控制器|完整/总数|pitch 动态 RMS / °|yaw 动态 RMS / °|第二阶段平均 roll / °|限幅比例|',
          '|---|---:|---:|---:|---:|---:|']
    for c in ('impedance','admittance','smc','hinf'):
        all_rows=[r for r in records.values() if r['summary']['experiment']=='D' and r['summary']['controller']==c]
        done=[r for r in all_rows if r['summary']['status']=='completed']
        def aggregate(xs):
            return f'{np.mean(xs):.3f} ± {np.std(xs,ddof=1) if len(xs)>1 else 0:.3f}' if xs else '—'
        pitch=aggregate([r['diagnostics']['pitch_dynamic_rmse_deg'] for r in done])
        yaw=aggregate([r['diagnostics']['yaw_dynamic_rmse_deg'] for r in done])
        roll=aggregate([np.rad2deg(r['summary']['metrics']['phase2_roll_mean_rad']) for r in done])
        sat=np.mean([r['summary']['metrics']['saturation_fraction'] for r in done]) if done else float('nan')
        rows.append(f'|{c}|{len(done)}/{len(all_rows)}|{pitch}|{yaw}|{roll}|{sat:.3%}|')
    rows+=['','D 的静态误差排名并非所有指标均由 H∞ 最小；导纳的 pitch 动态偏差也可能更小。H∞ 的优势证据是两个姿态下较小的整体动态偏差、无力矩限幅及稳定的恢复过程，不把静态 F/K 或某一张图当成全局优越性证明。','',
           '## 正式结果','',
           '下表 D 为完整运行均值，其余为主种子 100。失败/提前终止和完成数量仍保存在上表与机器可读清单中。','',
           '|组|控制器|位置 RMS / mm|动态阻抗位置偏差 / mm|REF 均值 / m·s⁻¹|采样峰值力 / N|限幅比例|',
           '|---|---|---:|---:|---:|---:|---:|']
    return rows


def write_evidence(records,extras,selected,out):
    def main(e,c):return next(r for r in records.values() if r['summary']['experiment']==e and r['summary']['controller']==c and r['summary']['trial']==0)
    groups={'main':{k:main(*k) for k in selected}}
    for label in ('seed101','seed102','physics8'):
        groups[label]={(r['summary']['experiment'],r['summary']['controller']):r for name,r in extras.items() if name.startswith(label+'_')}
    checks=[]
    def check(group,name,passed,values):checks.append(dict(group=group,effect=name,passed=bool(passed),values=values))
    for label,group in groups.items():
        def m(e,c,k):return group[e,c]['summary']['metrics'][k]
        if ('A','hinf') in group:
            ratios={c:m('A',c,'position_rmse_m')/m('A','impedance','position_rmse_m') for c in ('admittance','hinf')}
            check(label,'A robust tracking materially improves over ordinary impedance',max(ratios.values())<.2,ratios)
        if ('B','hinf') in group:
            ripples={c:group['B',c]['diagnostics']['wall_force_ripple_rms_N'] for c in ('admittance','hinf')}
            saturation={c:m('B',c,'saturation_fraction') for c in ('admittance','hinf')}
            check(label,'B admittance contact chatter; H-infinity avoids saturation',ripples['admittance']>5*ripples['hinf'] and saturation['admittance']>.01 and saturation['hinf']==0,dict(force_ripple_rms_N=ripples,saturation_fraction=saturation))
        if ('C','hinf') in group:
            rms={c:m('C',c,'force_tracking_rmse_N') for c in ('admittance','bidirectional','hinf')}
            ripple={c:group['C',c]['diagnostics']['free_board_force_ripple_rms_N'] for c in ('admittance','hinf')}
            check(label,'C full-board chatter contrast and improved force tracking',rms['hinf']<rms['bidirectional']<rms['admittance'] and ripple['admittance']>5*ripple['hinf'] and m('C','hinf','saturation_fraction')==0,dict(force_rmse_N=rms,free_board_ripple_N=ripple))
        if ('F','hinf_1') in group:
            controllers=['hinf_0.25','hinf_0.5','hinf_1']
            ref=np.array([m('F',c,'reference_error_position_mean_m_s') for c in controllers])
            eq=np.array([m('F',c,'equilibrium_position_error_mean_m') for c in controllers])
            ratio=ref[:-1]/ref[1:]
            check(label,'F approximate inverse gain/REF trend and decreasing equilibrium discrepancy',np.all((ratio>1.7)&(ratio<2.2)) and np.all(np.diff(eq)<0),dict(ref_mean_m_s=ref.tolist(),equilibrium_mean_m=eq.tolist(),adjacent_ref_ratios=ratio.tolist()))
    d=[r for r in records.values() if r['summary']['experiment']=='D' and r['summary']['controller']=='hinf']
    roll=[np.rad2deg(r['summary']['metrics']['phase2_roll_mean_rad']) for r in d if r['summary']['status']=='completed']
    check('D all trials','D H-infinity completes both poses without saturation',all(r['summary']['status']=='completed' and r['summary']['metrics']['saturation_fraction']==0 for r in d) and len(roll)==len(d) and all(abs(x-90)<2 for x in roll),dict(trials=len(d),phase2_roll_range_deg=[min(roll),max(roll)] if roll else []))
    # Equality is checked from recorded settings, rather than assumed from code.
    common=('plant','sensors','impedance','initial_q','actuator','operator_stiffness_scale','operator_torque_scale','operator_roll_hold')
    task_groups={}
    for r in records.values():
        s=r['summary'];task_groups.setdefault((s['experiment'],s['trial'],s['seed']),[]).append(s)
    for key,group in task_groups.items():
        for field in common:
            if any(s[field]!=group[0][field] for s in group[1:]):raise ValueError(f'Unmatched comparison {key}: {field}')
    result=dict(description='Declared qualitative FR3 adaptation checks, not certification or exact paper reproduction.',
                source_sha256=next(iter(records.values()))['summary']['source_sha256'],
                common_task_settings_checked=list(common),main_status=dict(Counter(r['summary']['status'] for r in records.values())),
                checks=checks,all_qualitative_checks_passed=all(c['passed'] for c in checks))
    (out/'effect_checks.json').write_text(json.dumps(result,indent=2))
