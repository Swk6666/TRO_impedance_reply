"""Fixed E mechanism checks; retain all outcomes, including terminated runs."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
import json
from pathlib import Path

from fr3imp.experiment import default_plant_parameters, run_experiment, source_hash
from fr3imp.scenarios import COMPARISONS
from source_snapshot import capture_source


def cases():
    p=default_plant_parameters('E')
    variants={
        'main':{},
        'physics8':dict(physics_substeps=8),
        'physics16':dict(physics_substeps=16),
        'seed101':dict(seed=101),
        'seed102':dict(seed=102),
        'balanced':dict(plant_parameters=replace(p,rotor_eccentricity=0.)),
        'half_imbalance':dict(plant_parameters=replace(p,rotor_eccentricity=.00005)),
        'double_imbalance':dict(plant_parameters=replace(p,rotor_eccentricity=.0002)),
        '6000rpm':dict(plant_parameters=replace(p,rotor_frequency_hz=100.)),
        'fixed_point':dict(plant_parameters=replace(p,grinder_contact='point')),
        'frictionless':dict(plant_parameters=replace(p,contact_friction=0.)),
        'rigid_contact':dict(plant_parameters=replace(p,contact_timeconstant=.004,
                                                    contact_impedance=(.95,.99,.001))),
    }
    return {f'{name}_{c}':dict(experiment='E',controller=c,seed=100,profile='paper')|kw
            for name,kw in variants.items() for c in COMPARISONS['E']}


def run_case(name,job,root,resume):
    folder=root/name/f"E_{job['controller']}_trial00_seed{job['seed']}"
    path=folder/'summary.json'
    if resume and path.exists():
        old=json.loads(path.read_text())
        plant=asdict(job.get('plant_parameters',default_plant_parameters('E')))
        plant['timestep']=.001/job.get('physics_substeps',4)
        if (old['source_sha256']==source_hash() and old['requested_duration_s']==30
                and old['seed']==job['seed'] and old['grinder_model']=='rotor'
                and json.dumps(old['plant'],sort_keys=True)==json.dumps(plant,sort_keys=True)
                and (folder/'trajectory.npz').exists()):return old
    return run_experiment(**job,output=root/name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/grinding'))
    parser.add_argument('--workers',type=int,default=8)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    jobs=cases();manifest=dict(source_sha256=source_hash(),expected_cases=list(jobs),completed={},errors={})
    capture_source(args.output)
    def save():
        path=args.output/'manifest.tmp';path.write_text(json.dumps(manifest,indent=2));path.replace(args.output/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_case,name,job,args.output,args.resume):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as exc:manifest['errors'][name]=repr(exc)
            save()
            print(f'GRINDING {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
