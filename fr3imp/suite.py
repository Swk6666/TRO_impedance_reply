"""Run the paper's comparison matrix in isolated worker processes."""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
import json
from pathlib import Path

from .experiment import run_experiment,source_hash
from .scenarios import COMPARISONS


def run_job(kwargs):
    return run_experiment(**kwargs)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--experiments',default='ABCDEF')
    parser.add_argument('--d-trials',type=int,default=30)
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--output',default='outputs/suite')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--ideal',action='store_true')
    parser.add_argument('--physics-substeps',type=int,default=1)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    if args.d_trials<1 or args.workers<1 or args.physics_substeps<1:
        parser.error('trial, worker and physics-substep counts must be positive')
    if not args.experiments or any(e not in COMPARISONS for e in args.experiments) or len(set(args.experiments))!=len(args.experiments):
        parser.error('experiments must be unique letters drawn from ABCDEF')
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    jobs=[];completed=[]
    version=source_hash()
    for name in args.experiments:
        for controller in COMPARISONS[name]:
            for trial in range(args.d_trials if name=='D' else 1):
                job=dict(experiment=name,controller=controller,trial=trial,seed=100+trial,
                         duration=args.duration,ideal=args.ideal,output=str(root),physics_substeps=args.physics_substeps)
                folder=root/f'{name}_{controller}_trial{trial:02d}_seed{100+trial}'
                old=folder/'summary.json'
                if args.resume and old.exists():
                    summary=json.loads(old.read_text())
                    expected_duration=args.duration or (35 if name=='D' else 30)
                    if (summary['source_sha256']==version and summary['ideal']==args.ideal
                        and summary['requested_duration_s']==expected_duration
                        and summary['physics_substeps']==args.physics_substeps):
                        completed.append(summary);continue
                jobs.append(job)
    manifest=dict(source_sha256=version,experiments=args.experiments,d_trials=args.d_trials,
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
