"""Fixed C mechanism audit. No optimizer selects gains or a favorable seed.

FR3 runs use 1 kHz control AND 1 kHz F/T. A named 4 kHz diagnostic is
reported separately. Failed cases remain in the manifest. Source matching
permits reuse of exploratory runs only when every configuration field agrees.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import asdict,replace
import hashlib
import json
from pathlib import Path
import shutil
from fr3imp.actuation import ActuatorParameters
from fr3imp.experiment import run_experiment,source_hash
from fr3imp.model import PlantParameters,ROOT
from fr3imp.sensors import SensorParameters

CONTROLLERS=('hinf','admittance','bidirectional')
SOFT=PlantParameters(contact_timeconstant=.04,contact_impedance=(.5,.5,.001))
CONTROL=dict(alpha=1.,nric_gain_scale=1.,energy_port='physical',coupling_update='stages',virtual_substeps=4)


def cases():
    result={}
    def add(name,c,seed=100,**kwargs):
        config=dict(experiment='C',controller=c,seed=seed,scenario_variant='paper_figure',
                    plant_parameters=SOFT,control_overrides=CONTROL.copy(),
                    actuator_parameters=ActuatorParameters(cutoff_hz=100))
        config.update(kwargs);result[name]=config
    for fc,label in [(100,'on'),(0,'off')]:
        for seed in (100,101,102):
            for c in CONTROLLERS:
                add(f'filter_{label}_{c}_s{seed}',c,seed,actuator_parameters=ActuatorParameters(cutoff_hz=fc))
    for c in CONTROLLERS:
        add(f'adapted_off_{c}',c,control_overrides={**CONTROL,'alpha':.25,'nric_gain_scale':.1},actuator_parameters=ActuatorParameters())
        add(f'no_friction_{c}',c,plant_parameters=replace(SOFT,friction_scale=0))
        add(f'no_noise_{c}',c,sensor_parameters=SensorParameters(q_std=0,v_std=0,force_std=0,moment_std=0))
        add(f'physics_125us_{c}',c,physics_substeps=8)
        add(f'softer_contact_{c}',c,plant_parameters=replace(SOFT,contact_timeconstant=.1))
        add(f'reference_4khz_{c}',c,control_period=.00025,actuator_parameters=ActuatorParameters())
    for beta in (0.,1.):
        add(f'beta_{int(beta)}','bidirectional',control_overrides={**CONTROL,'beta_override':beta})
        add(f'beta_{int(beta)}_ideal','bidirectional',ideal=True,
            control_overrides={**CONTROL,'beta_override':beta})
    add('proxy_8steps','bidirectional',control_overrides={**CONTROL,'virtual_substeps':8})
    for port in ('net','physical_exact'):
        add(f'port_{port}','bidirectional',control_overrides={**CONTROL,'energy_port':port})
    return result


def expected(settings):
    dt=settings.get('control_period',.001);sub=settings.get('physics_substeps',4)
    ideal=settings.get('ideal',False)
    plant=replace(settings['plant_parameters'],timestep=dt/sub)
    sensor=settings.get('sensor_parameters',SensorParameters())
    if ideal:
        plant=replace(plant,friction_scale=0,damping_scale=0,mass_error=0,tool_mass=0,nominal_tool_mass=0)
        sensor=SensorParameters(q_std=0,v_std=0,force_std=0,moment_std=0,tool_calibration_mass=0)
    return dict(experiment='C',controller=settings['controller'],seed=settings['seed'],trial=0,
        requested_duration_s=30.,ideal=ideal,c_profile='fr3',scenario_variant='paper_figure',
        control_period_s=dt,physics_substeps=sub,
        plant=asdict(plant),sensors=asdict(sensor),
        actuator=asdict(settings['actuator_parameters']),control_settings=settings['control_overrides'],
        source_sha256=source_hash())


def matches(summary,settings):
    # JSON normalizes tuples and lists, as they are semantically identical here.
    return all(json.dumps(summary.get(k),sort_keys=True)==json.dumps(v,sort_keys=True)
               for k,v in expected(settings).items())


def run_case(name,settings,root,caches):
    destination=root/name/f"C_{settings['controller']}_trial00_seed{settings['seed']}"
    for candidate in [destination/'summary.json',*caches]:
        if not candidate.exists():continue
        summary=json.loads(candidate.read_text())
        if matches(summary,settings) and summary['status']=='completed' and summary['force_samples']==30000:
            if candidate.parent!=destination:
                shutil.copytree(candidate.parent,destination,dirs_exist_ok=True)
            return name,summary,str(candidate)
    result=run_experiment(**settings,output=root/name,profile='legacy')
    return name,result,None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/c-mechanism'))
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--reuse',type=Path,action='append',default=[])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    manifest_path=args.output/'manifest.json';all_cases=cases();sha=source_hash()
    if manifest_path.exists():
        old=json.loads(manifest_path.read_text())
        if old['source_sha256']!=sha:raise RuntimeError('Use a fresh output directory for changed runtime code')
    source=args.output/'source';source.mkdir(exist_ok=True)
    for p in (ROOT/'fr3imp').glob('*.py'):shutil.copy2(p,source/p.name)
    caches=[p for root in args.reuse for p in root.rglob('summary.json')]
    manifest=dict(source_sha256=sha,expected_cases=list(all_cases),completed={},errors={},reused={},
                  protocol='C mechanism audit v1: shared scene/environment, original gains, paired seeds, filter on/off, causal and integration checks',
                  configurations={n:expected(kw) for n,kw in all_cases.items()},
                  runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    manifest_path.write_text(json.dumps(manifest,indent=2))
    with ProcessPoolExecutor(args.workers) as pool:
        futures={pool.submit(run_case,n,kw,args.output,caches):n for n,kw in all_cases.items()}
        for f in as_completed(futures):
            try:
                name,result,reused=f.result();manifest['completed'][name]=result
                if reused:manifest['reused'][name]=reused
                print(name,result['status'],result['metrics'].get('force_tracking_rmse_N'),flush=True)
            except Exception as exc:manifest['errors'][futures[f]]=repr(exc)
            manifest_path.write_text(json.dumps(manifest,indent=2))
    if manifest['errors']:raise RuntimeError(manifest['errors'])


if __name__=='__main__':main()
