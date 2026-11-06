"""Independently validate C records, then create figures and machine-readable evidence."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=Path('outputs/c-validation'))
    parser.add_argument('--output', type=Path, default=Path('docs/results/c-1khz'))
    args = parser.parse_args()
    manifest = json.loads((args.input/'manifest.json').read_text())
    assert not manifest['errors'], manifest['errors']
    assert set(manifest['completed']) == set(manifest['expected_cases'])
    metrics = {}; logs = {}
    for name, summary in sorted(manifest['completed'].items()):
        folder = args.input/name/f'C_{summary["controller"]}_trial00_seed{summary["seed"]}'
        assert json.loads((folder/'summary.json').read_text()) == summary
        assert summary['source_sha256'] == manifest['source_sha256']
        assert summary['status'] == 'completed', (name, summary['termination_reason'])
        assert summary['completed_control_steps'] == round(30/summary['control_period_s'])
        assert summary['force_samples'] == 30000
        with np.load(folder/'trajectory.npz') as file:
            a = {key: file[key].astype(float) for key in file.files}
        assert all(len(value) == 30000 and np.isfinite(value).all() for value in a.values())
        np.testing.assert_allclose(a['time'], np.arange(30000)*.001, atol=1e-12)
        residual = a['force_true'][:, 2] - a['force_desired'][:, 2]
        rms = np.linalg.norm(residual)/np.sqrt(len(residual))
        np.testing.assert_allclose(rms, summary['metrics']['force_tracking_rmse_N'], rtol=2e-6)
        torque_limits = np.array([87]*4+[12]*3)
        sat = np.any(np.abs(a['torque_command']) > torque_limits, axis=1)
        np.testing.assert_array_equal(sat, a['saturated'])
        np.testing.assert_allclose(np.clip(a['torque_command'], -torque_limits, torque_limits), a['torque_applied'])
        after = a['time'] >= 1
        obstacle = a['threshold_contact'] > 0
        active = a['time'][obstacle]
        penetration = (a['desired'][:, 2] - a['pose'][:, 2])*1000
        metrics[name] = dict(
            force_rmse_N=float(rms),
            force_rmse_after_1s_N=float(np.sqrt(np.mean(residual[after]**2))),
            force_rmse_at_threshold_N=float(np.sqrt(np.mean(residual[obstacle]**2))) if obstacle.any() else None,
            force_measured_rmse_N=float(np.sqrt(np.mean((a['force_measured'][:, 2]-a['force_desired'][:, 2])**2))),
            force_z_min_N=float(a['force_true'][:, 2].min()),
            force_z_max_N=float(a['force_true'][:, 2].max()),
            force_z_min_after_1s_N=float(a['force_true'][after, 2].min()),
            saturation_fraction=float(sat.mean()),
            board_contact_fraction_after_1s=float(a['board_contact'][after].mean()),
            threshold_contact_start_s=float(active[0]) if len(active) else None,
            threshold_contact_end_s=float(active[-1]) if len(active) else None,
            xy_position_rmse_mm=float(np.sqrt(np.mean(np.sum((a['pose'][:, :2]-a['desired'][:, :2])**2, axis=1)))*1000),
            median_board_penetration_mm=float(np.median(penetration[after])),
            max_board_penetration_mm=float(np.max(penetration)),
        )
        if name in ('legacy_1khz','gain_only','contact_only','tuned','admittance','bidirectional'):
            logs[name] = a
    args.output.mkdir(parents=True, exist_ok=True)
    payload = dict(source_sha256=manifest['source_sha256'],
                   definitions=dict(force_rmse='True world Fz minus desired Fz, all 30000 samples, no filtering',
                                    after_1s='Exclude only t < 1 s; full-duration error is also reported',
                                    board_contact='Active MuJoCo tip-whiteboard constraint, not a force threshold',
                                    peaks='1 kHz samples, not all internal physics substeps'),
                   metrics=metrics, runs=manifest['completed'])
    (args.output/'metrics.json').write_text(json.dumps(payload, indent=2))
    a = logs['tuned']; old = logs['legacy_1khz']; t = a['time']; p0 = a['desired'][0, :2]
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig, ax = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    ax[0,0].plot(t, old['force_true'][:,2], color='#a8afb9', lw=.4, label='Original settings at 1 kHz')
    ax[0,0].plot(t, a['force_true'][:,2], color='#168465', lw=.6, label='FR3 C adaptation')
    ax[0,0].plot(t, a['force_desired'][:,2], 'k--', lw=1, label='Desired')
    ax[0,0].set(title='Normal force: complete 30 s', ylabel='Fz [N]', xlabel='Time [s]')
    ax[0,0].legend(fontsize=8)
    w = (t >= 13) & (t <= 17)
    ax[0,1].plot(t[w], a['force_true'][w,2], color='#168465', lw=.65, label='True force')
    ax[0,1].plot(t[w], a['force_measured'][w,2], color='#c48723', alpha=.55, lw=.5, label='Noisy wrist F/T')
    ax[0,1].plot(t[w], a['force_desired'][w,2], 'k--', label='Desired')
    ax[0,1].set(title='Threshold interaction: unsmoothed 1 kHz samples', xlabel='Time [s]', ylabel='Fz [N]')
    ax[0,1].legend(fontsize=8)
    ax[1,0].plot((a['desired'][:,0]-p0[0])*1000, (a['desired'][:,1]-p0[1])*1000, 'k--', label='Desired path')
    for name, color, label in [('admittance','#377fb0','Admittance'),('bidirectional','#b35393','Bidirectional'),('tuned','#168465','H-infinity')]:
        b = logs[name]
        ax[1,0].plot((b['pose'][:,0]-p0[0])*1000, (b['pose'][:,1]-p0[1])*1000, lw=1, color=color, label=label)
        ax[1,1].plot(t, b['force_true'][:,2], color=color, lw=.5, alpha=.8, label=label)
    ax[1,0].add_patch(Rectangle((72,-60),12,120,facecolor='#bbbbbb',alpha=.5,label='Threshold solid'))
    ax[1,0].axvline(62, color='#999999', ls=':', lw=1, label='Rigid tip center limit')
    ax[1,0].set(title='Motion adapts to friction and the threshold', xlabel='Relative x [mm]', ylabel='Relative y [mm]', aspect='equal', xlim=(-5,85), ylim=(-55,55))
    ax[1,0].legend(fontsize=7,loc='lower left')
    ax[1,1].plot(t, a['force_desired'][:,2], 'k--', lw=1, label='Desired')
    ax[1,1].set(title='Three controllers with 1 kHz gain adaptation', xlabel='Time [s]', ylabel='Fz [N]')
    ax[1,1].legend(fontsize=8)
    fig.savefig(args.output/'C_1khz.png', dpi=170)
    plt.close(fig)
    print(json.dumps(metrics, indent=2))
    print(f'Validated {len(metrics)} complete runs and {len(metrics)*30000} logged samples')


if __name__ == '__main__':
    main()
