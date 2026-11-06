"""Active-scope probes: separate Cartesian axes and respect sampled gain limits.

E is intentionally absent. Keep all cases, including failures. These are
explicit adaptations, separate from the literal paper-parameter baseline.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json
from pathlib import Path

import numpy as np

from fr3imp.experiment import run_experiment,source_hash
from fr3imp.scenarios import COMPARISONS,PARAMETERS
from fr3imp.sensors import SensorParameters
from source_snapshot import capture_source


def scaled(e,axes,scale):
    L,D,K=np.array(PARAMETERS[e],dtype=float)
    L[axes]*=scale;D[axes]*=np.sqrt(scale)
    return dict(inertia=L.tolist(),damping=D.tolist(),stiffness=K.tolist())


def cases():
    result={}
    def add(label,e,c,**settings):
        result[f'{label}_{e}_{c}']=dict(experiment=e,controller=c,seed=100,profile='paper')|settings
    # B: check the previously observed /8 effect across seeds and physics rate.
    for label,settings in [('seed101',dict(seed=101)),('seed102',dict(seed=102)),
                           ('physics8',dict(physics_substeps=8))]:
        for c in COMPARISONS['B']:
            add('low_inertia_'+label,'B',c,impedance_overrides=scaled('B',slice(None),.125),**settings)
    # F must use the exact same B scenario and impedance when testing its gain.
    for c in COMPARISONS['F']:
        add('low_inertia','F',c,impedance_overrides=scaled('F',slice(None),.125))
        add('low_inertia_physics8','F',c,impedance_overrides=scaled('F',slice(None),.125),physics_substeps=8)
    # C: change only motion-controlled x/y; leave normal-force and angular axes.
    for label,scale in [('xy_quarter',.25),('xy_eighth',.125)]:
        for c in COMPARISONS['C']:
            add(label,'C',c,impedance_overrides=scaled('C',slice(0,2),scale))
    for alpha in (.25,.5):add('alpha'+str(alpha),'C','hinf',control_overrides=dict(alpha=alpha))
    for c in ('admittance','bidirectional'):
        add('nric_gain_point1','C',c,control_overrides=dict(nric_gain_scale=.1))
    # D: preserve stiff translation while changing angular rendering only.
    for label,scale in [('rotation_quarter',.25),('rotation_eighth',.125)]:
        for c in COMPARISONS['D']:
            add(label,'D',c,impedance_overrides=scaled('D',slice(3,6),scale))
    for alpha in (.25,.5):
        add('alpha'+str(alpha),'D','hinf',control_overrides=dict(alpha=alpha))
        add('grasp4_alpha'+str(alpha),'D','hinf',operator_stiffness_scale=4.,control_overrides=dict(alpha=alpha))
    # Same one-sample F/T delay for all four methods in this grasp contrast.
    for c in COMPARISONS['D']:
        overrides=dict(alpha=.25) if c=='hinf' else (dict(nric_gain_scale=.1) if c=='admittance' else {})
        add('grasp4_delay1','D',c,operator_stiffness_scale=4.,
            sensor_parameters=SensorParameters(delay_samples=1),control_overrides=overrides)
    return result


def run_case(name,job,root):
    return run_experiment(**job,output=root/name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/axis-adaptation'))
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);capture_source(args.output)
    jobs=cases();manifest=dict(source_sha256=source_hash(),expected_cases=list(jobs),completed={},errors={})
    def save():
        path=args.output/'manifest.tmp';path.write_text(json.dumps(manifest,indent=2));path.replace(args.output/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_case,name,job,args.output):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as exc:manifest['errors'][name]=repr(exc)
            save();print(f'AXIS ADAPTATION {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
