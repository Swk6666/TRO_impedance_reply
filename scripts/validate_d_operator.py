"""Run D with the Fig. 11 replay operator; the fr3v2 grasp-model records stay as they are.

Main: 4 controllers x 30 trials (10 operator scales x 3 repeats). Fixed checks:
input amplitude (H-infinity only, including configurations that fail), firm grip,
which column supplies the moments, F/T delay, NRIC gain, friction and step size.
--resume requires matching runtime code.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import asdict,replace
import json
from pathlib import Path

from fr3imp.experiment import run_experiment,source_hash,default_plant_parameters,default_sensor_parameters
from fr3imp.scenarios import COMPARISONS
from source_snapshot import capture_source
from validate_reproduction import BANDWIDTH_MATCHED_G

CONTROLLERS=COMPARISONS['D']


def cases(d_trials=30):
    common=dict(experiment='D',profile='fr3v2',operator_model='fig11_replay')
    result={f'main/D_{c}_trial{trial:02d}':dict(common,controller=c,trial=trial,seed=100+trial)
            for c in CONTROLLERS for trial in range(d_trials)}
    def add(label,c,**kwargs):
        kwargs={'operator_replay':dict(),**kwargs}
        result[f'{label}/D_{c}']=dict(common,controller=c,trial=0,seed=100,**kwargs)
    for scale in (.3,.4,.5,.7,.8,1.):add(f'scale{scale:g}','hinf',operator_replay=dict(scale=scale))
    for c in CONTROLLERS:
        for grip in (10.,30.,60.,120.,300.):add(f'grip{grip:g}',c,operator_replay=dict(grip=grip))
        add('own_column',c,operator_replay=dict(column=c))
        add('delay0',c,sensor_parameters=replace(default_sensor_parameters('fr3v2',default_plant_parameters('D','fr3v2')),delay_samples=0))
        add('physics8',c,physics_substeps=8)
        for friction in (1.,2.,3.5):
            add(f'friction{friction:g}',c,plant_parameters=replace(default_plant_parameters('D','fr3v2'),friction_scale=friction))
    add('matched_G','admittance',control_overrides=dict(nric_gain_vector=BANDWIDTH_MATCHED_G))
    return result


def run_case(name,job,root,resume):
    label,_=name.split('/')
    folder=root/label/f"D_{job['controller']}_trial{job['trial']:02d}_seed{job['seed']}"
    path=folder/'summary.json'
    if resume and path.exists():
        old=json.loads(path.read_text())
        expected=dict(column='hinf',scale=.6,grip=0.,**job.get('operator_replay',{}))
        if (old.get('source_sha256')==source_hash() and old.get('operator_replay')==expected
            and (folder/'trajectory.npz').exists()
            and old.get('physics_substeps')==job.get('physics_substeps',4)
            and all(old['control_settings'].get(k)==v for k,v in job.get('control_overrides',{}).items())
            and ('sensor_parameters' not in job or old['sensors']==json.loads(json.dumps(asdict(job['sensor_parameters']))))
            and ('plant_parameters' not in job or old['plant']['friction_scale']==job['plant_parameters'].friction_scale)):
            return old
    return run_experiment(**job,output=root/label)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/d-operator'))
    parser.add_argument('--workers',type=int,default=8)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    capture_source(args.output)
    jobs=cases()
    manifest=dict(source_sha256=source_hash(),expected_cases=list(jobs),completed={},errors={})
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
            print(f'D {len(manifest["completed"])+len(manifest["errors"])}/{len(jobs)} {name}',flush=True)
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
