"""Fixed B/C/D inertia and grasp contrasts, separate from literal paper gains.

Lower inertia while preserving stiffness and damping ratio; apply the identical
change to every controller. These are mechanism probes, not an identified Indy7
equivalence or a replacement for the literal-parameter comparison.
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
    def add(label,e,inertia_scale=1.,grasp_scale=1.):
        L,D,K=np.asarray(PARAMETERS[e],dtype=float)
        override=dict(inertia=(L*inertia_scale).tolist(),damping=(D*np.sqrt(inertia_scale)).tolist(),stiffness=K.tolist())
        for c in COMPARISONS[e]:
            result[f'{label}_{e}_{c}']=dict(experiment=e,controller=c,seed=100,profile='paper',
                impedance_overrides=override,operator_stiffness_scale=grasp_scale)
    for e in 'BCD':
        add('literal',e)
        add('inertia_quarter',e,.25)
        add('inertia_eighth',e,.125)
    add('grasp4','D',grasp_scale=4.)
    add('grasp8','D',grasp_scale=8.)
    add('inertia_eighth_grasp4','D',.125,4.)
    return result


def run_case(name,job,root,resume):
    folder=root/name/f"{job['experiment']}_{job['controller']}_trial00_seed100"
    path=folder/'summary.json'
    if resume and path.exists():
        old=json.loads(path.read_text())
        if (old['source_sha256']==source_hash() and old['impedance']==job['impedance_overrides']
                and old['operator_stiffness_scale']==job['operator_stiffness_scale']
                and old['requested_duration_s']==(35 if job['experiment']=='D' else 30)
                and (folder/'trajectory.npz').exists()):return old
    return run_experiment(**job,output=root/name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/portability'))
    parser.add_argument('--workers',type=int,default=8)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    capture_source(args.output)
    jobs=cases();manifest=dict(source_sha256=source_hash(),expected_cases=list(jobs),completed={},errors={})
    def save():
        path=args.output/'manifest.tmp';path.write_text(json.dumps(manifest,indent=2));path.replace(args.output/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_case,name,job,args.output,args.resume):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as exc:manifest['errors'][name]=repr(exc)
            save();print(f'PORTABILITY {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
