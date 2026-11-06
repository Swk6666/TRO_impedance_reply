"""Rebuild A/B/C/D/F at 1 kHz, with fixed seeds and explicit sensitivity cases.

E is deferred by AGENTS.md. Main: 134 runs including 30 D trials. Additional
cases keep failed/saturated configurations, including the historical rigid
contact and bypassed command filter. --resume requires matching runtime code.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import asdict,replace
import json
from pathlib import Path
import shutil
import subprocess
import sys

from fr3imp.actuation import ActuatorParameters
from fr3imp.experiment import run_experiment,source_hash,default_plant_parameters
from fr3imp.model import ROOT,PlantParameters
from fr3imp.scenarios import COMPARISONS
from source_snapshot import capture_source


def cases(experiments='ABCDF',profile='fr3'):
    selected=set(experiments)
    result={}
    def add(label,e,c,**kwargs):
        if e not in selected:return
        result[f'{label}_{e}_{c}']=dict(experiment=e,controller=c,seed=100,profile=profile,**kwargs)
    for seed in (101,102):
        for e in 'ABCEF':
            if e not in selected:continue
            for c in COMPARISONS[e]:
                key=f'seed{seed}_{e}_{c}'
                result[key]=dict(experiment=e,controller=c,seed=seed,profile=profile)
    for e in 'BCEF':
        for c in COMPARISONS[e]:add('physics8',e,c,physics_substeps=8)
    for e in 'BCE':add('virtual8',e,'bidirectional',control_overrides=dict(virtual_substeps=8))
    add('virtual8','C','admittance',control_overrides=dict(virtual_substeps=8))
    for e in 'BE':
        p=replace(default_plant_parameters(e,profile),contact_timeconstant=.004,contact_impedance=(.95,.99,.001))
        for c in COMPARISONS[e]:add('rigid_contact',e,c,plant_parameters=p)
    for e,c in [('A','admittance'),('B','hinf'),('C','hinf'),('D','hinf'),('E','hinf')]:
        add('filter_off',e,c,actuator_parameters=ActuatorParameters())
    return result


def run_case(name,job,root,resume):
    folder=root/name/f"{job['experiment']}_{job['controller']}_trial00_seed{job['seed']}"
    path=folder/'summary.json'
    if resume and path.exists():
        old=json.loads(path.read_text())
        required=dict(source_sha256=source_hash(),profile=job['profile'],experiment=job['experiment'],
            controller=job['controller'],seed=job['seed'],trial=0,control_period_s=.001,
            requested_duration_s=35. if job['experiment']=='D' else 30.,
            physics_substeps=job.get('physics_substeps',4))
        matches=all(old.get(k)==v for k,v in required.items())
        if 'actuator_parameters' in job:matches &= old['actuator']==asdict(job['actuator_parameters'])
        for k,v in job.get('control_overrides',{}).items():matches &= old['control_settings'].get(k)==v
        if 'plant_parameters' in job:
            expected=asdict(job['plant_parameters']);expected['timestep']=.001/required['physics_substeps']
            matches &= json.dumps(old['plant'],sort_keys=True)==json.dumps(expected,sort_keys=True)
        if matches and (folder/'trajectory.npz').exists():return old
    return run_experiment(**job,output=root/name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/reproduction-fr3'))
    parser.add_argument('--profile',choices=['fr3','paper'],default='fr3')
    parser.add_argument('--experiments',default='ABCDF',help='E is deferred by AGENTS.md')
    parser.add_argument('--workers',type=int,default=8)
    parser.add_argument('--d-trials',type=int,default=30)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--skip-main',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    capture_source(args.output)
    if not args.skip_main:
        command=[sys.executable,'-m','fr3imp.suite','--output',str(args.output/'main'),
                 '--profile',args.profile,'--workers',str(args.workers),'--d-trials',str(args.d_trials),
                 '--experiments',args.experiments]
        if args.resume:command+=['--resume']
        subprocess.run(command,check=True)
    jobs=cases(args.experiments,args.profile);root=args.output/'sensitivity';root.mkdir(exist_ok=True)
    manifest=dict(source_sha256=source_hash(),profile=args.profile,expected_cases=list(jobs),completed={},errors={})
    def save():
        path=root/'manifest.tmp';path.write_text(json.dumps(manifest,indent=2));path.replace(root/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_case,name,job,root,args.resume):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as exc:manifest['errors'][name]=repr(exc)
            save()
            print(f'SENSITIVITY {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
