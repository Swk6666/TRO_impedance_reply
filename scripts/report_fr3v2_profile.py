"""Evidence and prose for the fr3v2 profile (A/B/C/D/F).

The checks below were written before the 282-run matrix finished, from the paper's
claims. Several were expected to fail; they stay in the report when they do.
"""
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path

import numpy as np

from scipy.signal import welch

from fr3imp.evaluation import impedance_metrics
from fr3imp.scenarios import Scenario,COMPARISONS

FR3_ROOT=Path('outputs/reproduction-fr3/main')
PAPER_C=Path('docs/results/c-mechanism/paper_curves.npz')
D_ORDER=('admittance','impedance','smc','hinf')
# Read from the paper: F is printed text in Fig. 16; A and D are read off Figs. 5 and 12.
PAPER=dict(A_impedance_peak_mm=58.,F_ref=[.379,.186,.095],F_eq_mm=[7.99,4.41,3.10],F_eq_std_mm=[7.83,5.08,4.43],
           F_alpha=[.25,.5,1.],D_pitch=[.0217,.0407,.017,.0024],D_yaw=[.175,.41,.126,.118])
GAIN=1100.  # Translational entry of K*gamma + 1/gamma^2; effective gain is alpha times this.
SENSITIVITY_GROUPS=('delay0','delay1','delay2','delay5','matched_G','friction2','friction3.5',
                    'contact_zeta1','contact_soft','contact_stiff')
CLAIMS=[('A_tracking','A：导纳与 H∞ 的位置 RMS 都低于普通阻抗的 20%'),
        ('A_impedance_peak_scale','A：普通阻抗峰值误差落在论文约 58 mm 的 0.5–2 倍内'),
        ('B_impedance_contacts','B：普通阻抗也接触到墙（不是靠大偏差错过）'),
        ('B_chatter_contrast','B：导纳接触力高频波动至少是 H∞ 的 3 倍'),
        ('B_saturation_contrast','B：导纳限幅超过 1%，H∞ 低于 0.5%'),
        ('B_equilibrium_order','B：平衡偏差普通阻抗最大、H∞ 最小'),
        ('B_bidirectional_gap','B：双向调制的阻抗偏差至少是 H∞ 的 3 倍'),
        ('B_ref_below_smc','B（Fig. 7）：H∞ 的 REF 比 SMC 至少低 10%'),
        ('B_admittance_stop','B：导纳在接触后失稳终止（原机急停）'),
        ('C_force_order','C：法向力 RMS 为 H∞ < 双向 < 导纳'),
        ('C_reduction','C：H∞ 法向力 RMS 不超过导纳的一半'),
        ('C_chatter','C：平板段导纳力波动至少是 H∞ 的 2 倍'),
        ('F_ref_inverse','F：增益翻倍时 REF 接近减半（相邻比 1.7–2.2）'),
        ('F_disturbance_constant','F：增益 × REF 在三档增益下基本不变（最大/最小 < 1.3）'),
        ('F_eq_mean','F：平衡偏差均值随增益单调下降，C1→C3 至少降 30%'),
        ('F_eq_std','F：平衡偏差时间标准差随增益单调下降，C1→C3 至少降 20%'),
        ('D_complete','D：四种控制器 120 次全部跑完'),
        ('D_impedance_worst','D：pitch 与 yaw 平衡偏差都是普通阻抗最大'),
        ('D_pitch_hinf_best','D（Fig. 12）：H∞ 的 pitch 偏差比其他控制器都至少低 50%'),
        ('D_yaw_hinf_best','D（Fig. 12）：H∞ 的 yaw 偏差最小'),
        ('D_roll_free','D：无被动支撑时 H∞ 第二阶段滚转仍在 90°±2°')]


def fz_rms_from_paper():
    curves=np.load(PAPER_C)
    return {c:float(np.sqrt(np.mean((curves['force_'+c][:,1]-10-3*np.sin(curves['force_'+c][:,0]))**2)))
            for c in COMPARISONS['C']}


def force_band(a,e,window):
    """Measured-force spectrum inside a contact window; the bands are post hoc, see the report."""
    mask=(a['time']>=window[0])&(a['time']<window[1])
    force=a['force_measured'][mask,2] if e=='C' else np.linalg.norm(a['force_measured'][mask,:3],axis=1)
    f,power=welch(force-force.mean(),1000,nperseg=2048)
    def rms(low,high):return float(np.sqrt(power[(f>=low)&(f<high)].sum()*(f[1]-f[0])))
    oscillation=(f>=2)&(f<500)
    return dict(band_2_100_N=rms(2,100),band_100_500_N=rms(100,500),peak_hz=float(f[oscillation][power[oscillation].argmax()]))


def paper_force_band():
    """Digitized Fig. 9, about 100 Hz: nothing above roughly 50 Hz is visible in it."""
    curves=np.load(PAPER_C);result={}
    for c in COMPARISONS['C']:
        grid=np.arange(1,12,.002)
        f,power=welch(np.interp(grid,curves['force_'+c][:,0],curves['force_'+c][:,1])-curves['force_'+c][:,1].mean(),500,nperseg=1024)
        band=(f>=2)&(f<50)
        result[c]=dict(band_2_50_N=float(np.sqrt(power[band].sum()*(f[1]-f[0]))),peak_hz=float(f[band][power[band].argmax()]))
    return result


def recompute(folder):
    """One definition for both profiles: old fr3 summaries lack the newer keys."""
    from report_reproduction import extra_metrics
    s=json.loads((folder/'summary.json').read_text())
    with np.load(folder/'trajectory.npz') as f:a={k:f[k] for k in f.files}
    scenario=Scenario(s['experiment'],a['desired'][0].astype(float),s['trial'],variant=s['scenario_variant'],
                      initial_q=s['initial_q'],impedance_overrides=s['impedance'])
    metrics={**s['metrics'],**impedance_metrics(a,scenario)[0]}
    diagnostics=extra_metrics(s,a)
    if s['experiment'] in ('B','C'):diagnostics['force_band']=force_band(a,s['experiment'],(13,18) if s['experiment']=='B' else (1,12))
    return (s['experiment'],s['controller'],s['trial']),dict(status=s['status'],metrics=metrics,
        diagnostics=diagnostics,alpha=s['controller_parameters'].get('alpha'))


def indicators(root):
    folders=sorted(p.parent for p in root.glob('*/summary.json'))
    with ProcessPoolExecutor(max_workers=16) as pool:records=dict(pool.map(recompute,folders))
    def m(e,c,k):return records[e,c,0]['metrics'][k]
    def d(e,c,k):return records[e,c,0]['diagnostics'][k]
    def trials(c,k):return [r['metrics'][k] for key,r in records.items() if key[0]=='D' and key[1]==c]
    F=COMPARISONS['F']
    return dict(
        A_position_rms_mm={c:m('A',c,'position_rmse_m')*1e3 for c in COMPARISONS['A']},
        A_impedance_peak_mm=records['A','impedance',0]['diagnostics']['position_error_peak_m']*1e3,
        B_saturation_pct={c:m('B',c,'saturation_fraction')*100 for c in COMPARISONS['B']},
        B_ripple_N={c:d('B',c,'wall_force_ripple_rms_N') for c in COMPARISONS['B']},
        B_ripple_peak_hz={c:d('B',c,'wall_force_ripple_peak_hz') for c in COMPARISONS['B']},
        B_equilibrium_mm={c:m('B',c,'equilibrium_position_error_measured_mean_m')*1e3 for c in COMPARISONS['B']},
        B_gap_mm={c:m('B',c,'dynamic_impedance_position_gap_mean_m')*1e3 for c in COMPARISONS['B']},
        B_ref={c:m('B',c,'reference_error_position_mean_m_s') for c in COMPARISONS['B']},
        C_measured_fz_rms_N={c:m('C',c,'force_tracking_measured_rmse_N') for c in COMPARISONS['C']},
        C_saturation_pct={c:m('C',c,'saturation_fraction')*100 for c in COMPARISONS['C']},
        B_force_band={c:d('B',c,'force_band') for c in COMPARISONS['B']},
        B_force_peak_N={c:m('B',c,'force_peak_N') for c in COMPARISONS['B']},
        C_force_band={c:d('C',c,'force_band') for c in COMPARISONS['C']},
        C_board_contact={c:m('C',c,'board_contact_fraction_after_1s') for c in COMPARISONS['C']},
        C_gap_mm={c:m('C',c,'dynamic_impedance_position_gap_mean_m')*1e3 for c in COMPARISONS['C']},
        D_pitch_rad={c:float(np.mean(trials(c,'pitch_equilibrium_error_measured_mae_rad'))) for c in D_ORDER},
        D_yaw_rad={c:float(np.mean(trials(c,'yaw_equilibrium_error_measured_mae_rad'))) for c in D_ORDER},
        D_pitch_gap_rad={c:float(np.mean(trials(c,'pitch_impedance_gap_mae_rad'))) for c in D_ORDER},
        D_roll2_deg={c:float(np.rad2deg(np.mean(trials(c,'phase2_roll_mean_rad')))) for c in D_ORDER},
        D_saturation_pct={c:float(np.mean(trials(c,'saturation_fraction'))*100) for c in D_ORDER},
        F_alpha=[records['F',c,0]['alpha'] for c in F],
        F_ref=[m('F',c,'reference_error_position_mean_m_s') for c in F],
        F_eq_mm=[m('F',c,'equilibrium_position_error_measured_mean_m')*1e3 for c in F],
        F_eq_std_mm=[m('F',c,'equilibrium_position_error_measured_std_m')*1e3 for c in F],
        F_disturbance_N=[records['F',c,0]['alpha']*GAIN*m('F',c,'reference_error_position_mean_m_s') for c in F])


def evaluate(group):
    """Claims supported by the runs in `group`; a missing run omits the claim."""
    def m(e,c,k):return group[e,c]['summary']['metrics'][k]
    def d(e,c,k):return group[e,c]['diagnostics'][k]
    def has(e,*names):return all((e,c) in group for c in names)
    result={}
    def claim(name,passed,**values):result[name]=dict(passed=bool(passed),values=values)
    if has('A','impedance','admittance','hinf'):
        ratios={c:m('A',c,'position_rmse_m')/m('A','impedance','position_rmse_m') for c in ('admittance','hinf')}
        claim('A_tracking',max(ratios.values())<.2,ratios=ratios)
        peak=d('A','impedance','position_error_peak_m')*1e3
        claim('A_impedance_peak_scale',.5*PAPER['A_impedance_peak_mm']<peak<2*PAPER['A_impedance_peak_mm'],peak_mm=peak)
    if has('B','impedance'):
        claim('B_impedance_contacts',m('B','impedance','contact_fraction')>0,contact_fraction=m('B','impedance','contact_fraction'))
    if has('B','admittance','hinf'):
        ripple={c:d('B',c,'wall_force_ripple_rms_N') for c in ('admittance','hinf')}
        claim('B_chatter_contrast',ripple['admittance']>=3*ripple['hinf'],ripple_N=ripple)
        sat={c:m('B',c,'saturation_fraction') for c in ('admittance','hinf')}
        claim('B_saturation_contrast',sat['admittance']>.01 and sat['hinf']<.005,saturation=sat)
        claim('B_admittance_stop',group['B','admittance']['summary']['status']!='completed',
              status=group['B','admittance']['summary']['status'])
    if has('B',*COMPARISONS['B']):
        eq={c:m('B',c,'equilibrium_position_error_measured_mean_m') for c in COMPARISONS['B']}
        claim('B_equilibrium_order',max(eq,key=eq.get)=='impedance' and min(eq,key=eq.get)=='hinf',equilibrium_m=eq)
    if has('B','bidirectional','hinf'):
        gap={c:m('B',c,'dynamic_impedance_position_gap_mean_m') for c in ('bidirectional','hinf')}
        claim('B_bidirectional_gap',gap['bidirectional']>=3*gap['hinf'],gap_m=gap)
    if has('B','smc','hinf'):
        ref={c:m('B',c,'reference_error_position_mean_m_s') for c in ('smc','hinf')}
        claim('B_ref_below_smc',ref['hinf']<.9*ref['smc'],ref_m_s=ref)
    if has('C',*COMPARISONS['C']):
        rms={c:m('C',c,'force_tracking_measured_rmse_N') for c in COMPARISONS['C']}
        claim('C_force_order',rms['hinf']<rms['bidirectional']<rms['admittance'],measured_rms_N=rms)
        claim('C_reduction',rms['hinf']<=.5*rms['admittance'],measured_rms_N=rms)
        ripple={c:d('C',c,'free_board_force_ripple_rms_N') for c in ('admittance','hinf')}
        claim('C_chatter',ripple['admittance']>=2*ripple['hinf'],ripple_N=ripple)
    if has('F',*COMPARISONS['F']):
        F=COMPARISONS['F']
        ref=np.array([m('F',c,'reference_error_position_mean_m_s') for c in F])
        alpha=np.array([group['F',c]['summary']['controller_parameters']['alpha'] for c in F])
        eq=np.array([m('F',c,'equilibrium_position_error_measured_mean_m') for c in F])
        std=np.array([m('F',c,'equilibrium_position_error_measured_std_m') for c in F])
        ratio=ref[:-1]/ref[1:];disturbance=alpha*GAIN*ref
        claim('F_ref_inverse',np.all((ratio>1.7)&(ratio<2.2)),adjacent_ratios=ratio.tolist())
        claim('F_disturbance_constant',disturbance.max()/disturbance.min()<1.3,gain_times_ref_N=disturbance.tolist())
        claim('F_eq_mean',np.all(np.diff(eq)<0) and eq[-1]<=.7*eq[0],equilibrium_mean_m=eq.tolist())
        claim('F_eq_std',np.all(np.diff(std)<0) and std[-1]<=.8*std[0],equilibrium_std_m=std.tolist())
    return result


def evaluate_d(records):
    """The D claims need all trials, so they use the main matrix only."""
    def values(c,k):return [r['summary']['metrics'][k] for r in records.values()
                            if r['summary']['experiment']=='D' and r['summary']['controller']==c and r['summary']['status']=='completed']
    d=[r for r in records.values() if r['summary']['experiment']=='D']
    done=Counter(r['summary']['controller'] for r in d if r['summary']['status']=='completed')
    result=dict(D_complete=dict(passed=len(d)==120 and sum(done.values())==120,values=dict(completed=dict(done))))
    pitch={c:float(np.mean(values(c,'pitch_equilibrium_error_measured_mae_rad'))) for c in D_ORDER}
    yaw={c:float(np.mean(values(c,'yaw_equilibrium_error_measured_mae_rad'))) for c in D_ORDER}
    others=[c for c in D_ORDER if c!='hinf']
    result['D_impedance_worst']=dict(passed=max(pitch,key=pitch.get)=='impedance' and max(yaw,key=yaw.get)=='impedance',values=dict(pitch_rad=pitch,yaw_rad=yaw))
    result['D_pitch_hinf_best']=dict(passed=all(pitch['hinf']<=.5*pitch[c] for c in others),values=dict(pitch_rad=pitch))
    result['D_yaw_hinf_best']=dict(passed=min(yaw,key=yaw.get)=='hinf',values=dict(yaw_rad=yaw))
    roll=np.rad2deg(values('hinf','phase2_roll_mean_rad'))
    result['D_roll_free']=dict(passed=len(roll)==30 and bool(np.all(abs(roll-90)<2)),values=dict(range_deg=[float(roll.min()),float(roll.max())]))
    return result


def check_groups(records,extras,selected):
    def main(e,c):return next(r for r in records.values() if r['summary']['experiment']==e and r['summary']['controller']==c and r['summary']['trial']==0)
    groups={'main':{k:main(*k) for k in selected}}
    for name,r in extras.items():
        label=name.split(f"_{r['summary']['experiment']}_{r['summary']['controller']}")[0]
        groups.setdefault(label,{})[r['summary']['experiment'],r['summary']['controller']]=r
    # G only changes the NRIC controllers; every other result is the main run's.
    groups['matched_G']={**{k:v for k,v in groups['main'].items() if k[0]!='D'},**groups['matched_G']}
    return groups


def write_evidence(records,extras,selected,out,root):
    groups=check_groups(records,extras,selected)
    outcome={label:evaluate(group) for label,group in groups.items()}
    outcome['main'].update(evaluate_d(records))
    common=('plant','sensors','impedance','initial_q','actuator','operator_stiffness_scale','operator_torque_scale','operator_roll_hold')
    task_groups={}
    for r in records.values():
        s=r['summary'];task_groups.setdefault((s['experiment'],s['trial'],s['seed']),[]).append(s)
    for key,group in task_groups.items():
        for field in common:
            if any(s[field]!=group[0][field] for s in group[1:]):raise ValueError(f'Unmatched comparison {key}: {field}')
    (out/'effect_checks.json').write_text(json.dumps(dict(
        description='Claims declared before the matrix finished; a failed claim means that paper effect is not reproduced here.',
        source_sha256=next(iter(records.values()))['summary']['source_sha256'],
        claims=dict(CLAIMS),common_task_settings_checked=list(common),
        main_status=dict(Counter(r['summary']['status'] for r in records.values())),groups=outcome),indent=2))
    comparison=dict(paper=dict(**PAPER,C_digitized_fz_rms_N=fz_rms_from_paper(),C_digitized_force_band=paper_force_band(),F_disturbance_N=[a*GAIN*r for a,r in zip(PAPER['F_alpha'],PAPER['F_ref'])]),
                    fr3=indicators(FR3_ROOT),fr3v2=indicators(root/'main'))
    (out/'fr3_vs_fr3v2.json').write_text(json.dumps(comparison,indent=2))


def triple(values,order,scale=1,digits=2):
    if isinstance(values,dict):values=[values[c] for c in order]
    return ' / '.join(f'{v*scale:.{digits}f}' for v in values)


def comparison_rows(c):
    p,a,b=c['paper'],c['fr3'],c['fr3v2']
    A=COMPARISONS['A'];B5=['admittance','bidirectional','smc','hinf'];Cc=COMPARISONS['C']
    def band(x,key,order,digits=1):return ' / '.join(f'{x[c][key]:.{digits}f}' for c in order)
    rows=['|指标|论文|`fr3`|`fr3v2`|','|---|---:|---:|---:|',
          f'|A 普通阻抗峰值误差 / mm|≈{p["A_impedance_peak_mm"]:.0f}|{a["A_impedance_peak_mm"]:.1f}|{b["A_impedance_peak_mm"]:.1f}|',
          f'|A 位置 RMS / mm（阻抗/导纳/H∞）|—|{triple(a["A_position_rms_mm"],A)}|{triple(b["A_position_rms_mm"],A)}|',
          f'|B 限幅 / %（导纳/双向/SMC/H∞）|原机急停|{triple(a["B_saturation_pct"],B5)}|{triple(b["B_saturation_pct"],B5)}|',
          f'|B 13–18 s 力波动 / N（同上）|—|{triple(a["B_ripple_N"],B5)}|{triple(b["B_ripple_N"],B5)}|',
          f'|B 13–18 s 力 2–100 Hz 波动 / N（同上）|—|{band(a["B_force_band"],"band_2_100_N",B5)}|{band(b["B_force_band"],"band_2_100_N",B5)}|',
          f'|B 导纳力谱峰 / Hz|—|{a["B_force_band"]["admittance"]["peak_hz"]:.0f}|{b["B_force_band"]["admittance"]["peak_hz"]:.0f}|',
          f'|B 峰值力 / N（同上）|—|{triple(a["B_force_peak_N"],B5,digits=0)}|{triple(b["B_force_peak_N"],B5,digits=0)}|',
          f'|B 平衡偏差 / mm（同上）|—|{triple(a["B_equilibrium_mm"],B5)}|{triple(b["B_equilibrium_mm"],B5)}|',
          f'|B 阻抗偏差 / mm（同上）|—|{triple(a["B_gap_mm"],B5)}|{triple(b["B_gap_mm"],B5)}|',
          f'|B REF / m·s⁻¹（SMC/H∞）|SMC 更大|{triple(a["B_ref"],["smc","hinf"],digits=3)}|{triple(b["B_ref"],["smc","hinf"],digits=3)}|',
          f'|C 测量法向力 RMS / N（导纳/双向/H∞）|{triple(p["C_digitized_fz_rms_N"],Cc)}（数字化）|{triple(a["C_measured_fz_rms_N"],Cc)}|{triple(b["C_measured_fz_rms_N"],Cc)}|',
          f'|C 阻抗偏差 / mm（同上）|—|{triple(a["C_gap_mm"],Cc)}|{triple(b["C_gap_mm"],Cc)}|',
          f'|C 平板段 2–100 Hz 力波动 / N（同上）|{band(p["C_digitized_force_band"],"band_2_50_N",Cc)}（数字化，2–50 Hz）|{band(a["C_force_band"],"band_2_100_N",Cc)}|{band(b["C_force_band"],"band_2_100_N",Cc)}|',
          f'|C 平板段 >100 Hz 力波动 / N（同上）|—|{band(a["C_force_band"],"band_100_500_N",Cc)}|{band(b["C_force_band"],"band_100_500_N",Cc)}|',
          f'|C 导纳力谱峰 / Hz|{p["C_digitized_force_band"]["admittance"]["peak_hz"]:.0f}（数字化）|{a["C_force_band"]["admittance"]["peak_hz"]:.0f}|{b["C_force_band"]["admittance"]["peak_hz"]:.0f}|',
          f'|C 板面接触比例（同上）|—|{triple(a["C_board_contact"],Cc)}|{triple(b["C_board_contact"],Cc)}|',
          f'|D pitch 平衡偏差 / rad（导纳/阻抗/SMC/H∞）|{triple(p["D_pitch"],D_ORDER,digits=4)}|{triple(a["D_pitch_rad"],D_ORDER,digits=4)}|{triple(b["D_pitch_rad"],D_ORDER,digits=4)}|',
          f'|D yaw 平衡偏差 / rad（同上）|{triple(p["D_yaw"],D_ORDER,digits=3)}|{triple(a["D_yaw_rad"],D_ORDER,digits=3)}|{triple(b["D_yaw_rad"],D_ORDER,digits=3)}|',
          f'|D 第二阶段 roll / °（同上）|自由顺应|{triple(a["D_roll2_deg"],D_ORDER,digits=1)}|{triple(b["D_roll2_deg"],D_ORDER,digits=1)}|',
          f'|F 有效 α（C1/C2/C3）|{triple(p["F_alpha"],None,digits=4)}|{triple(a["F_alpha"],None,digits=4)}|{triple(b["F_alpha"],None,digits=4)}|',
          f'|F REF / m·s⁻¹|{triple(p["F_ref"],None,digits=3)}|{triple(a["F_ref"],None,digits=3)}|{triple(b["F_ref"],None,digits=3)}|',
          f'|F 增益 × REF / N|{triple(p["F_disturbance_N"],None,digits=0)}|{triple(a["F_disturbance_N"],None,digits=1)}|{triple(b["F_disturbance_N"],None,digits=1)}|',
          f'|F 平衡偏差均值 / mm|{triple(p["F_eq_mm"],None)}|{triple(a["F_eq_mm"],None)}|{triple(b["F_eq_mm"],None)}|',
          f'|F 平衡偏差标准差 / mm|{triple(p["F_eq_std_mm"],None)}|{triple(a["F_eq_std_mm"],None)}|{triple(b["F_eq_std_mm"],None)}|']
    return rows


def claim_rows(outcome):
    columns=['main','seed101','seed102','physics8',*SENSITIVITY_GROUPS]
    rows=['|判据|'+'|'.join(columns)+'|','|---|'+'---:|'*len(columns)]
    for name,label in CLAIMS:
        cells=[('✓' if outcome[g][name]['passed'] else '✗') if name in outcome.get(g,{}) else '' for g in columns]
        rows.append(f'|{label}|'+'|'.join(cells)+'|')
    return rows


def assumption_rows(groups):
    rows=['|配置|B 导纳限幅|B 力波动 导纳 / H∞ / N|C 测量法向力 RMS 导纳 / 双向 / H∞ / N|F REF C1 / C3|F 平衡偏差 C1 / C3 / mm|','|---|---:|---:|---:|---:|---:|']
    for label in ('main','delay0','delay1','delay2','delay5','matched_G','friction2','friction3.5','contact_zeta1','contact_soft','contact_stiff','c_solimp8','rigid_contact'):
        g=groups[label]
        def m(e,c,k):return g[e,c]['summary']['metrics'][k]
        b=f'{m("B","admittance","saturation_fraction"):.2%}' if ('B','admittance') in g else '—'
        ripple=f'{g["B","admittance"]["diagnostics"]["wall_force_ripple_rms_N"]:.1f} / {g["B","hinf"]["diagnostics"]["wall_force_ripple_rms_N"]:.1f}' if ('B','hinf') in g and ('B','admittance') in g else '—'
        c=' / '.join(f'{m("C",x,"force_tracking_measured_rmse_N"):.2f}' for x in COMPARISONS['C']) if all(('C',x) in g for x in COMPARISONS['C']) else '—'
        if ('F','hinf_1') in g:
            f_ref=f'{m("F","hinf_0.25","reference_error_position_mean_m_s"):.3f} / {m("F","hinf_1","reference_error_position_mean_m_s"):.3f}'
            f_eq=f'{m("F","hinf_0.25","equilibrium_position_error_measured_mean_m")*1e3:.2f} / {m("F","hinf_1","equilibrium_position_error_measured_mean_m")*1e3:.2f}'
        else:f_ref=f_eq='—'
        rows.append(f'|{label}|{b}|{ripple}|{c}|{f_ref}|{f_eq}|')
    return rows


def overview(manifest,records,extras,selected,out):
    evidence=json.loads((out/'effect_checks.json').read_text())
    comparison=json.loads((out/'fr3_vs_fr3v2.json').read_text())
    groups=check_groups(records,extras,selected)
    states=Counter(r['summary']['status'] for r in [*records.values(),*extras.values()])
    main=evidence['groups']['main']
    passed=[n for n,_ in CLAIMS if main[n]['passed']];failed=[n for n,_ in CLAIMS if not main[n]['passed']]
    labels=dict(CLAIMS)
    rows=['# `fr3v2`：恢复论文阻抗参数后的 A、B、C、D、F','',
          '当前范围见 [AGENTS.md](../../../AGENTS.md)。E 砂轮机按用户要求暂缓，既有代码和历史记录保留。','',
          f'`fr3v2` 正式矩阵 {len(records)} 次（D 每控制器 {manifest["d_trials"]} 次），固定敏感性矩阵 {len(extras)} 次；状态：{dict(states)}。',
          f'运行时代码 SHA256：`{manifest["source_sha256"]}`。[`fr3` 的结果](../reproduction-fr3/REPORT.md)保留作对照。','',
          '**这是 FR3/MuJoCo 仿真，不是真实机械臂或论文原机复现。** 摩擦、F/T 延迟和接触参数都未经实物标定；选择方式见“这些假设如何选出来”。','',
          '## 结论','',
          *CONCLUSION,'',
          f'预先声明的 {len(CLAIMS)} 条判据中，主矩阵通过 {len(passed)} 条，未通过 {len(failed)} 条：','',
          *[f'- {labels[n]}' for n in failed],'',
          '## 配置','',
          '|项目|`paper`|`fr3`|`fr3v2`|','|---|---|---|---|',
          '|Λ/D/K|论文值|B/F 全轴 Λ×.125、D×√.125；C 的 x/y 缩放；D 角轴 ×.25|**论文值**|',
          '|NRIC G|论文值|D ×.25，其余论文值|**论文值**|',
          '|H∞ 有效 α|1；F 为 .25/.5/1|.25；F 为 .0625/.125/.25|.25；F 为 .0625/.125/.25|',
          '|D 虚拟操作者|原尺度|弹簧×4、力矩×.25、第二阶段滚转支撑|**原尺度，无滚转支撑**|',
          '|关节摩擦|模型值×1|×1|**×3**|',
          '|F/T 延迟|0|0|**3 ms（3 个采样）**|',
          '|B/F 接触 solref；solimp|(.02,1)；(.5,.5,.001)|同左|**(.02,.4)；(.95,.99,.001)**|',
          '|C 接触 solref；solimp|(.04,1)；(.5,.5,.001)|同左|**(.04,.3)；(.9,.95,.001)**|','',
          '两版都用 1 kHz 控制/F/T、250 μs 物理步长、100 Hz 公共命令低通、A/B/F 相同起始姿态。同一实验内所有控制器共享以上全部设置，已从每次记录逐项核对。','',
          '## 论文、`fr3`、`fr3v2` 对照','',
          '三列由同一段代码从保存的轨迹重算，`fr3` 没有重跑。平衡偏差和 C 的法向力用测量 F/T，与论文口径一致。D 为 30 次均值，其余为种子 100。',''
          ,*comparison_rows(comparison),'',
          '论文 A、D 的数值从原图读取，C 从矢量图数字化，只有 F 的数值是原图印出的。',
          f'增益 × REF 是 H∞ 反馈实际要抵消的等效任务空间扰动。论文三档约 {comparison["paper"]["F_disturbance_N"][0]:.0f} N，`fr3` 约 {comparison["fr3"]["F_disturbance_N"][2]:.0f} N，`fr3v2` 约 {comparison["fr3v2"]["F_disturbance_N"][2]:.0f} N，仍只有论文的约五分之一。','',
          '## 判据与稳健性','',
          '判据在矩阵跑完前写定。✓/✗ 是该组运行上的结果，空白表示该组没有所需运行。D 的判据要用全部重复，只在主矩阵评估。[机器可读结果](effect_checks.json)。','',
          *claim_rows(evidence['groups']),'',
          '## 平台假设的影响','',
          *assumption_rows(groups),'',
          *ASSUMPTIONS,'',
          '## 这些假设如何选出来','',
          *SELECTION,'',
          '## D：30 次虚拟操作者重复','',
          '每控制器使用同样 10 组握持参数 × 3 个噪声种子，均值 ± 样本标准差不是真人置信区间。','',
          '|控制器|完整/总数|pitch 平衡偏差 / rad|yaw 平衡偏差 / rad|pitch 阻抗偏差 / rad|第二阶段 roll / °|限幅|',
          '|---|---:|---:|---:|---:|---:|---:|']
    for c in D_ORDER:
        all_rows=[r for r in records.values() if r['summary']['experiment']=='D' and r['summary']['controller']==c]
        done=[r['summary']['metrics'] for r in all_rows if r['summary']['status']=='completed']
        def agg(k,scale=1,digits=4):
            x=[m[k]*scale for m in done];return f'{np.mean(x):.{digits}f} ± {np.std(x,ddof=1):.{digits}f}'
        rows.append(f'|{c}|{len(done)}/{len(all_rows)}|{agg("pitch_equilibrium_error_measured_mae_rad")}|{agg("yaw_equilibrium_error_measured_mae_rad")}|{agg("pitch_impedance_gap_mae_rad")}|{agg("phase2_roll_mean_rad",180/np.pi,2)}|{np.mean([m["saturation_fraction"] for m in done]):.2%}|')
    rows+=['','## 正式结果','',
           '下表 D 为完整运行均值，其余为主种子 100。表中动态阻抗位置偏差沿用旧口径，包含 C 的 z 轴（K=0，积分参考会漂移），所以 C 的这一列不能直接比较；K>0 轴的结果见前面的“B/C 阻抗偏差”和 `metrics.json` 的 `dynamic_impedance_position_rmse_stiff_axes_m`。','',
           '|组|控制器|位置 RMS / mm|动态阻抗位置偏差 / mm|REF 均值 / m·s⁻¹|采样峰值力 / N|限幅比例|',
           '|---|---|---:|---:|---:|---:|---:|']
    return rows


CONCLUSION=['`fr3v2` 用论文的任务参数复现了 A、F 的主要趋势和 D 的 yaw 趋势。B/C 中导纳差于 H∞ 的现象也出现了，但这个差异依赖高增益内环和 F/T 延迟同时存在。论文最强的 D pitch 优势仍未复现。它也不是全面好于 `fr3`：C 的 H∞ 力跟踪和 B 的 H∞/SMC REF 差异都变差了。','',
            '**比 `fr3` 更接近论文：**','',
            '- 任务条件：Λ/D/K、NRIC G、D 虚拟操作者全部回到论文值，D 不再需要滚转支撑。H∞ 和导纳在第二阶段仍保持约 90°，普通阻抗掉到约 82.6°。',
            '- A：普通阻抗峰值误差 73 mm（论文约 58 mm，`fr3` 33 mm），H∞ 的小幅误差凸起也出现在相近位置。',
            '- F：REF 为 .311 / .160 / .081 m/s（论文 .379 / .186 / .095）。平衡偏差 C1→C3 下降 48%（论文 61%，`fr3` 4.3%）。',
            '- D：普通阻抗 yaw 平衡偏差 .417 rad（论文 .41，`fr3` .229）。',
            '- B/C：导纳振荡从 `fr3` 的 134/151 Hz 降到 18/11 Hz。C 平板段 2–100 Hz 力波动为 6.1 / 1.9 / 1.0 N，从论文图数字化的 2–50 Hz 结果约为 4.7 / 1.2 / .8 N，主频约 10 Hz。数字化约 100 Hz，只能看到 50 Hz 以下。','',
            '**比 `fr3` 更差：**','',
            '- C：H∞ 法向力 RMS 为 3.14 N（`fr3` .63，论文约 1.32）。板面接触比例从 1.00 降到 .90，测量力在 0–15 N 间高频波动，而论文 H∞ 曲线贴近目标。',
            '- B：H∞ 和 SMC 的 REF 都是 .081 m/s，不再有差异。`fr3` 是 .030 / .094，论文也是 SMC 明显更大。',
            '- B：H∞ 不再是零限幅（.09%），接触峰值力从 30 N 升到 72 N，13–18 s 力波动从 2.0 N 升到 5.6 N。','',
            '**两版都没有复现：**','',
            '- D pitch：H∞ .0023 rad 与论文 .0024 相当，但导纳 .0024 和 SMC .0022 也在同一水平（论文 .0217 和 .017），H∞ 没有优势。',
            '- D yaw：导纳 .0653 rad 略小于 H∞ .0671，论文是 H∞ 最小。',
            '- B：导纳跑完 30 s，没有原机的失稳急停；论文 Fig. 7 中 SMC 的明显力矩抖振也没有出现（SMC 命令力矩范数峰值 77 Nm，论文约 340 Nm）。',
            '- F：平衡偏差的时间标准差不随增益下降（8.00 / 7.42 / 7.61 mm，论文 7.83 / 5.08 / 4.43）。',
            '- 量级：增益 × REF 约 22 N，只有论文约 104 N 的五分之一。']
ASSUMPTIONS=['- **H∞ 对 F/T 延迟几乎不敏感，这一点不依赖延迟的具体取值。** 延迟从 0 到 5 ms，H∞ 在 B 的力波动为 5.8→5.6 N，在 C 的法向力 RMS 为 3.11→3.18 N。同样条件下，导纳在 C 从 3.25 N 升到 16.61 N，在 B 的限幅从 .10% 升到 4.13%。B 的限幅在 1 ms 和 2 ms 之间跳变（.23%→4.02%）。',
             '- **导纳的劣势依赖论文原 G。** `matched_G` 把 NRIC G 换成在 FR3 上给出论文内环 −3 dB 带宽的估计值后，导纳在 B 的限幅只有 .24%，力波动 6.2 N（H∞ 5.6 N），在 C 的 RMS 为 3.14 N，与 H∞ 相同；代价是 A 的位置 RMS 从 .39 mm 升到 .82 mm。论文 G 是为 Indy7 设的，同样的数值放到 FR3 上，估算内环带宽为 75–234 Hz（J1–3）和 600 Hz 以上（腕部），而论文约为 13–23 Hz 和 63–69 Hz。所以能说的只是“高增益内环加 F/T 延迟时，导纳比 H∞ 敏感”，不能说导纳本身不稳定，也不能当成原机失稳的机理。',
             '- **C 的 H∞ 退化主要来自接触变硬，不是欠阻尼。** 接触时间常数翻倍后 H∞ 为 1.65 N，放宽 solimp 后为 1.91 N，而把 ζ 改回 1 仍有 3.05 N。这两个更软的接触也会减弱导纳的抖振（5.55 N、4.98 N），未改作默认值。',
             '- **摩擦有窄窗口。** ×2 时 F 的 REF 只有 .212 / .055 m/s。×3.5 时普通阻抗在 B 中碰不到墙，H∞ 的 REF 则低于 SMC，判据结果只有这两项变化。',
             '- **B 的抖振对比对接触参数敏感。** 力波动判据在 ζ=1 和更硬的接触下不成立，限幅判据在更软的接触下不成立。种子 101/102 和物理步长减半时，所有判据结果都与主矩阵相同。',
             '- 关闭命令滤波时 A 的导纳有 96.7% 的样本限幅，因此 100 Hz 接口仍是必要假设，不是无害的细节。']
SELECTION=['- 摩擦 ×3 是看 F 的 REF 量级和“普通阻抗能碰到墙”选的。平衡偏差没有用来拟合，但它也随摩擦变化。',
           '- **F/T 延迟和接触参数是在观察控制器差异是否拉开时选的，存在选择偏差。** 它们没有实物依据，延迟也可能偏大或偏小。',
           '- 判据在正式矩阵跑完前写定，但写在探索之后。它只能防止事后改标准，不能防止参数对结果的拟合。',
           '- `C：平板段导纳力波动` 沿用了 `fr3` 判据的 20 Hz 高通，错过了 11 Hz 的主频，所以在所有组都是 ✗。判据不改，结果保留。对照表里的 2–100 Hz 频带和力谱峰是看过结果后才加的，属于事后口径。',
           '- 探索共 9 轮 1152 次运行，脚本和结果存放在 [probes](probes)。它们通过替换 `build_model` 和 `Admittance.__init__` 运行在改动前的代码上，只作过程记录，不能直接用当前代码重跑。',
           '- 探索中否定的方向：只加腕部摩擦（D 的 H∞ 限幅 3–8%，普通阻抗滚转掉到 15–17°）；接近 Indy7 的转子惯量（A 的普通阻抗峰值 63.7 mm 更接近论文，但 C 的导纳触发 400 N 截止，H∞ 的 C/F 变差，D 有 34 次提前终止）；×3.5 及以上的摩擦（普通阻抗碰不到墙）；4 kHz 或更细步长。',
           '- 所有比较中，物理环境和测量条件对同一实验的控制器相同。这只保证比较条件一致，不能证明这些条件接近原机。']
