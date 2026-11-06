"""D: scale rotational impedance, operator moments and feedback in torque units.

Unlike reducing inertia alone, scaling rotational L/D/K together preserves its
natural frequency and damping ratio. The operator's complete torque law is
scaled after clipping. The same task and input law apply to all four methods.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json
from pathlib import Path

import numpy as np

from fr3imp.experiment import run_experiment,source_hash
from fr3imp.scenarios import COMPARISONS,PARAMETERS
from source_snapshot import capture_source


def cases():
    result={}
    for scale in (.125,.25):
        values=np.asarray(PARAMETERS['D'],dtype=float).copy();values[:,3:]*=scale
        impedance=dict(zip(('inertia','damping','stiffness'),values.tolist()))
        for grasp in (1.,4.):
            for c in COMPARISONS['D']:
                # The FR3/Indy7 wrist inertia ratio motivates the scale order,
                # but does not identify an exact whole-arm equivalence.
                feedback=dict(alpha=scale) if c=='hinf' else (dict(nric_gain_scale=scale) if c=='admittance' else {})
                result[f'torque{scale}_grasp{grasp}_{c}']=dict(experiment='D',controller=c,
                    seed=100,profile='paper',impedance_overrides=impedance,
                    operator_stiffness_scale=grasp,operator_torque_scale=scale,control_overrides=feedback)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/torque-scaling'))
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);capture_source(args.output)
    jobs=cases();manifest=dict(source_sha256=source_hash(),expected_cases=list(jobs),completed={},errors={})
    def save():
        path=args.output/'manifest.tmp';path.write_text(json.dumps(manifest,indent=2));path.replace(args.output/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_experiment,**job,output=args.output/name):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as exc:manifest['errors'][name]=repr(exc)
            save();print(f'TORQUE SCALING {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
