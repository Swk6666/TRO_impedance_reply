"""Compare saved runs at different physics steps, keeping control timing fixed."""
import argparse
import json
from pathlib import Path

from .analyze import read_runs


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('baseline',type=Path)
    parser.add_argument('refined',type=Path)
    parser.add_argument('--output',type=Path,default=Path('docs/results'))
    args=parser.parse_args()
    bmanifest,baseline,_=read_runs(args.baseline)
    rmanifest,refined,_=read_runs(args.refined)
    if bmanifest['source_sha256']!=rmanifest['source_sha256']:
        raise ValueError('Step comparison requires identical runtime code')
    index={(s['experiment'],s['controller'],s['trial'],s['seed']):s for s,_ in baseline}
    pairs=[]
    for r,_ in refined:
        key=(r['experiment'],r['controller'],r['trial'],r['seed'])
        b=index[key]
        for k in ['sensors','impedance','ideal','control_period_s','requested_duration_s','mujoco_version']:
            if b[k]!=r[k]:raise ValueError(f'Unmatched {k} for {key}')
        bp=b['plant'].copy();rp=r['plant'].copy()
        bdt=bp.pop('timestep');rdt=rp.pop('timestep')
        if bp!=rp or rdt>=bdt:raise ValueError(f'Invalid refinement for {key}')
        pairs.append(dict(experiment=key[0],controller=key[1],trial=key[2],seed=key[3],
                          baseline_step_s=bdt,refined_step_s=rdt,
                          baseline_status=b['status'],refined_status=r['status'],
                          baseline=b['metrics'],refined=r['metrics']))
    pairs.sort(key=lambda x:(x['experiment'],x['controller']))
    args.output.mkdir(parents=True,exist_ok=True)
    payload=dict(source_sha256=bmanifest['source_sha256'],pairs=pairs)
    (args.output/'sensitivity.json').write_text(json.dumps(payload,indent=2))
    lines=['# 物理步长敏感性','',
           '以相同代码、种子、物理参数和反馈参数对比 0.25 ms 与 0.125 ms 物理积分。控制保持 4 kHz，F/T 保持 1 kHz。每种设置一个匹配种子；这是敏感性检查，不是严格的数值收敛证明。',
           '逐项指标及状态见 [sensitivity.json](sensitivity.json)。已独立核对两个矩阵的完整性与关键指标。','',
           '|组|控制器|位置 RMS：原步长→半步长 (mm)|平移 REF RMS：原→半 (m/s)|采样峰值力：原→半 (N)|',
           '|---|---|---:|---:|---:|']
    for p in pairs:
        b,r=p['baseline'],p['refined']
        lines.append(f'|{p["experiment"]}|{p["controller"]}|{b["position_rmse_m"]*1000:.4f} → {r["position_rmse_m"]*1000:.4f}|{b["reference_error_position_rms_m_s"]:.5f} → {r["reference_error_position_rms_m_s"]:.5f}|{b["force_peak_N"]:.2f} → {r["force_peak_N"]:.2f}|')
    lines+=['','## C 组法向力误差','', '|控制器|原步长 RMS (N)|半步长 RMS (N)|','|---|---:|---:|']
    for p in pairs:
        if p['experiment']=='C':
            lines.append(f'|{p["controller"]}|{p["baseline"]["force_tracking_rmse_N"]:.4f}|{p["refined"]["force_tracking_rmse_N"]:.4f}|')
    lines+=['','C 的明显接触振荡在减半步长后仍存在。尤其双向能量流控制的轨迹/REF 对步长有可见敏感性，不能把整组结果称为已经数值收敛。峰值来自 1 kHz 保存样本，不是所有物理子步的峰值。']
    (args.output/'SENSITIVITY.md').write_text('\n'.join(lines)+'\n')
    print(f'Validated {len(pairs)} matched step-refinement pairs')


if __name__=='__main__':main()
