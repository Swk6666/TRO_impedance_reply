"""Run the paper's comparison matrix in isolated worker processes."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json
from pathlib import Path
import numpy as np

from .experiment import run_experiment,source_hash
from .actuation import ActuatorParameters
from .scenarios import COMPARISONS
from .profiles import profile_parameters


def run_job(kwargs):
    return run_experiment(**kwargs)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--experiments',default='ABCDF',help='E is deferred by AGENTS.md')
    parser.add_argument('--d-trials',type=int,default=30)
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--output',default='outputs/suite')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--ideal',action='store_true')
    parser.add_argument('--physics-substeps',type=int,default=4)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--control-period',type=float,choices=[.001,.00025],default=.001)
    parser.add_argument('--c-profile',choices=['fr3','legacy'],default='fr3')
    parser.add_argument('--profile',choices=['fr3','paper','legacy'],default='fr3')
    parser.add_argument('--inertia-scale',type=float,default=1.)
    parser.add_argument('--operator-stiffness-scale',type=float,default=None)
    parser.add_argument('--grinder-model',choices=['rotor','external'],default=None)
    parser.add_argument('--command-cutoff',type=float,default=None)
    args=parser.parse_args()
    if args.d_trials<1 or args.workers<1 or args.physics_substeps<1:
        parser.error('trial, worker and physics-substep counts must be positive')
    if not np.isfinite(args.inertia_scale) or args.inertia_scale<=0:
        parser.error('inertia scale must be finite and positive')
    if not args.experiments or any(e not in COMPARISONS for e in args.experiments) or len(set(args.experiments))!=len(args.experiments):
        parser.error('experiments must be unique letters drawn from ABCDEF')
    if args.profile=='fr3' and 'E' in args.experiments:
        parser.error('E is deferred by AGENTS.md; the fr3 profile covers A/B/C/D/F')
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    jobs=[];completed=[]
    version=source_hash()
    for name in args.experiments:
        configuration=profile_parameters(name,args.profile,args.c_profile)
        L,D,K=(np.asarray(configuration['impedance'][k]) for k in ('inertia','damping','stiffness'))
        impedance=dict(inertia=(L*args.inertia_scale).tolist(),damping=(D*np.sqrt(args.inertia_scale)).tolist(),stiffness=K.tolist())
        operator_stiffness=args.operator_stiffness_scale if args.operator_stiffness_scale is not None else configuration['operator_stiffness_scale']
        for controller in COMPARISONS[name]:
            for trial in range(args.d_trials if name=='D' else 1):
                job=dict(experiment=name,controller=controller,trial=trial,seed=100+trial,
                         duration=args.duration,ideal=args.ideal,output=str(root),physics_substeps=args.physics_substeps,control_period=args.control_period,c_profile=args.c_profile,profile=args.profile,
                         impedance_overrides=impedance,operator_stiffness_scale=operator_stiffness,
                         grinder_model=args.grinder_model)
                if args.command_cutoff is not None:job['actuator_parameters']=ActuatorParameters(cutoff_hz=args.command_cutoff)
                folder=root/f'{name}_{controller}_trial{trial:02d}_seed{100+trial}'
                old=folder/'summary.json'
                if args.resume and old.exists():
                    summary=json.loads(old.read_text())
                    expected_duration=args.duration or (35 if name=='D' else 30)
                    if (summary['source_sha256']==version and summary['ideal']==args.ideal
                        and (folder/'trajectory.npz').exists()
                        and summary['requested_duration_s']==expected_duration
                        and summary['physics_substeps']==args.physics_substeps
                        and summary['control_period_s']==args.control_period
                        and summary.get('profile')==args.profile
                        and summary.get('impedance')==impedance
                        and summary.get('operator_stiffness_scale',1.)==operator_stiffness
                        and summary.get('operator_torque_scale')==configuration['operator_torque_scale']
                        and summary.get('operator_roll_hold')==configuration['operator_roll_hold']
                        and summary.get('control_settings')==configuration['control_settings']
                        and (name!='E' or summary.get('grinder_model')==(args.grinder_model or ('rotor' if args.profile=='paper' else 'external')))
                        and summary.get('actuator',{}).get('cutoff_hz')==(args.command_cutoff if args.command_cutoff is not None else (0 if args.profile=='legacy' else 100))
                        and summary.get('c_profile')==(args.c_profile if name=='C' else None)):
                        completed.append(summary);continue
                jobs.append(job)
    manifest=dict(source_sha256=version,experiments=args.experiments,d_trials=args.d_trials,profile=args.profile,
                  physics_substeps=args.physics_substeps,expected_runs=len(jobs)+len(completed),completed=completed,errors=[])
    def save():
        tmp=root/'manifest.tmp'
        tmp.write_text(json.dumps(manifest,indent=2))
        tmp.replace(root/'manifest.json')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures={executor.submit(run_job,job):job for job in jobs}
        for future in as_completed(futures):
            job=futures[future]
            try:manifest['completed'].append(future.result())
            except Exception as exc:
                manifest['errors'].append(dict(job=job,error=repr(exc)))
                print('ERROR',job,repr(exc),flush=True)
            save()
            print(f'PROGRESS {len(manifest["completed"])+len(manifest["errors"])}/{manifest["expected_runs"]}',flush=True)
    print('SUITE FINISHED',root/'manifest.json',flush=True)


if __name__=='__main__':main()
