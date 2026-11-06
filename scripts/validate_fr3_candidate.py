"""Prospective joint FR3 adaptation, restricted to A/B/C/D/F.

Do not promote this configuration solely on execution completion. Compare its
physical response, saturation and paper signatures with the literal baseline.
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
    jobs={}
    for e in 'ABCDF':
        L,D,K=np.asarray(PARAMETERS[e],dtype=float).copy()
        settings={}
        if e in 'BF':L*=.125;D*=np.sqrt(.125)
        if e=='C':L[:2]*=.25;D[:2]*=.5
        if e=='D':
            L[3:]*=.25;D[3:]*=.25;K[3:]*=.25
            settings=dict(operator_stiffness_scale=4.,operator_torque_scale=[1.,.25,.25],operator_roll_hold=True)
        impedance=dict(inertia=L.tolist(),damping=D.tolist(),stiffness=K.tolist())
        for c in COMPARISONS[e]:
            actual=c
            feedback={}
            if c.startswith('hinf'):
                relative=float(c.split('_')[1]) if '_' in c else 1.
                if e=='F':actual=f'hinf_{relative*.25:g}'
                else:feedback=dict(alpha=.25)
            elif c in ('admittance','bidirectional'):feedback=dict(nric_gain_scale=.25)
            jobs[f'{e}_{c}']=dict(experiment=e,controller=actual,seed=100,profile='paper',
                impedance_overrides=impedance,control_overrides=feedback,**settings)
    return jobs


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/fr3-candidate'))
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);capture_source(args.output)
    jobs=cases();manifest=dict(source_sha256=source_hash(),jobs=jobs,completed={},errors={})
    def save():
        path=args.output/'manifest.tmp';path.write_text(json.dumps(manifest,indent=2));path.replace(args.output/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_experiment,**job,output=args.output/name):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as exc:manifest['errors'][name]=repr(exc)
            save();print(f'FR3 CANDIDATE {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
