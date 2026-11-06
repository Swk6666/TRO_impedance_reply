"""Reproduce C ablations and robustness checks at an explicit 1 kHz clock.

Run from repo root: OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/validate_c.py
Raw trajectories remain in outputs; the report command checks every sample.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
import json
from pathlib import Path
from fr3imp.experiment import run_experiment, source_hash
from fr3imp.model import PlantParameters
from fr3imp.sensors import SensorParameters


def cases():
    soft = PlantParameters(contact_timeconstant=.04, contact_impedance=(.5, .5, .001))
    quiet = SensorParameters(q_std=0, v_std=0, force_std=0, moment_std=0)
    result = {
        'legacy_1khz': dict(c_profile='legacy'),
        'gain_only': dict(c_profile='legacy', controller='hinf_0.25'),
        'contact_only': dict(controller='hinf_1'),
        'tuned': {},
        'legacy_no_noise': dict(c_profile='legacy', sensor_parameters=quiet),
        'legacy_no_joint_friction': dict(c_profile='legacy', plant_parameters=PlantParameters(friction_scale=0)),
        'legacy_no_contact_friction': dict(c_profile='legacy', plant_parameters=PlantParameters(contact_friction=0)),
        'legacy_4khz': dict(c_profile='legacy', control_period=.00025, physics_substeps=1),
        'admittance': dict(controller='admittance'),
        'bidirectional': dict(controller='bidirectional'),
        'physics_500us': dict(physics_substeps=2),
        'physics_125us': dict(physics_substeps=8),
        'noise_x2': dict(sensor_parameters=SensorParameters(q_std=2e-5, v_std=1e-3, force_std=.2, moment_std=.01)),
        'delay_1ms': dict(sensor_parameters=SensorParameters(delay_samples=1)),
        'friction_x1p5': dict(plant_parameters=replace(soft, friction_scale=1.5, contact_friction=.375)),
        'contact_30ms': dict(plant_parameters=replace(soft, contact_timeconstant=.03)),
        'contact_50ms': dict(plant_parameters=replace(soft, contact_timeconstant=.05)),
    }
    result.update({f'seed_{seed}': dict(seed=seed) for seed in range(101, 105)})
    return result


def run_case(name, settings, root):
    kwargs = dict(experiment='C', controller='hinf', seed=100, output=str(root/name))
    kwargs.update(settings)
    return name, run_experiment(**kwargs,profile='legacy')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/c-validation'))
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = dict(source_sha256=source_hash(), expected_cases=list(cases()), completed={}, errors={})
    with ProcessPoolExecutor(args.workers) as pool:
        futures = {pool.submit(run_case, n, kw, args.output): n for n, kw in cases().items()}
        for future in as_completed(futures):
            try:
                name, result = future.result()
                manifest['completed'][name] = result
                print(name, result['status'], result['metrics']['force_tracking_rmse_N'], flush=True)
            except Exception as exc:
                manifest['errors'][futures[future]] = repr(exc)
            (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    if manifest['errors']:
        raise RuntimeError(manifest['errors'])


if __name__ == '__main__':
    main()
