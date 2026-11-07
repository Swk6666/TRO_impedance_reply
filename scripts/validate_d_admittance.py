"""Test candidate causes of the paper's D admittance response on FR3.

Fig. 11 implies Lambda*x'' + D*x' + K*x = (I + W)*m for admittance, with W != 0,
while H-infinity has W = 0. Candidates, fixed before running:
- servo_g*: soft inner loop, the series-compliance family (G x .3, .1, .03);
- ft_delay20: 20 ms F/T delay;
- force_gain4: constant moment gain in the proxy, a control that cannot follow the axes;
- feedthrough: the -J^T*F term of (54) and of the nominal input is left out, so
  W = Lambda*Mx^-1. This is NOT the paper's controller; it asks whether that one
  structural change is sufficient, and it is also applied to B, C and A because
  the same term would act there.
The variants live here, not in fr3imp, so the runtime code and its hash stay as they are.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

import fr3imp.controllers as controllers
from fr3imp.dynamics import angle_error
import fr3imp.experiment as experiment
from fr3imp.experiment import run_experiment,source_hash,default_plant_parameters,default_sensor_parameters
from source_snapshot import capture_source

BASE_MAKE=controllers.make_controller
D_WINDOWS=dict(phase1=(0.,13.),phase2=(17.5,30.))


class DiagnosticAdmittance(controllers.Admittance):
    feedthrough=0.;force_gain=1.

    def nominal_acceleration(self,t,q,v,wrench):
        n=self.proxy.evaluate(q,v)
        ref=self.scenario.reference(t)
        e=angle_error(ref.p,n.p)
        measured=n.generalized_wrench(wrench)
        F=self.force_gain*measured-n.generalized_wrench(ref.force)
        aref=ref.a+(self.D*(ref.v-n.pdot)+self.K*e+F)/self.L
        acc=n.Jplus@(aref-n.Jdot@v)+n.null@(12*(self.posture-q)-4*v)
        acc=acc+self.feedthrough*np.linalg.solve(n.M,n.J.T@measured)
        torque=n.M@acc+n.bias-n.J.T@measured
        return acc,torque,n


class Feedthrough:
    def step(self,t,state,wrench):
        output=super().step(t,state,wrench)
        output.torque=output.torque+state.J.T@state.generalized_wrench(wrench)
        return output


class HInfinityFeedthrough(Feedthrough,controllers.HInfinity):pass
class ImpedanceFeedthrough(Feedthrough,controllers.Impedance):pass
class SlidingModeFeedthrough(Feedthrough,controllers.SlidingMode):pass


def variant_maker(variant,captured):
    def make(name,dynamics,scenario,control_period=controllers.DT,**settings):
        controller=BASE_MAKE(name,dynamics,scenario,control_period,**settings)
        captured['dynamics']=dynamics
        if variant in ('feedthrough','force_gain4','baseline') and name=='admittance':
            controller.__class__=DiagnosticAdmittance
            controller.feedthrough=1. if variant=='feedthrough' else 0.
            controller.force_gain=4. if variant=='force_gain4' else 1.
        elif variant=='feedthrough':
            controller.__class__=dict(hinf=HInfinityFeedthrough,impedance=ImpedanceFeedthrough,
                                      smc=SlidingModeFeedthrough)[name]
        return controller
    return make


def cases():
    result={}
    def add(label,variant,experiment_name,controller,**kwargs):
        if experiment_name=='D':
            kwargs={'operator_model':'fig11_replay','operator_replay':{},**kwargs}
        result[f'{label}/{experiment_name}_{controller}']=dict(variant=variant,experiment=experiment_name,
            controller=controller,trial=0,seed=100,profile='fr3v2',**kwargs)
    for c in ('admittance','impedance','smc','hinf'):
        add('baseline','baseline','D',c)
        add('feedthrough','feedthrough','D',c)
    for c in ('admittance','impedance','smc'):
        add('baseline_own','baseline','D',c,operator_replay=dict(column=c))
        add('feedthrough_own','feedthrough','D',c,operator_replay=dict(column=c))
    add('feedthrough_scale.3','feedthrough','D','admittance',operator_replay=dict(scale=.3))
    for gain in (.3,.1,.03):
        add(f'servo_g{gain:g}','plain','D','admittance',control_overrides=dict(nric_gain_scale=gain))
    plant=default_plant_parameters('D','fr3v2')
    add('ft_delay20','plain','D','admittance',sensor_parameters=replace(default_sensor_parameters('fr3v2',plant),delay_samples=20))
    add('force_gain4','force_gain4','D','admittance')
    for c in ('admittance','hinf'):
        add('baseline','baseline','C',c)
        add('feedthrough','feedthrough','C',c)
    for c in ('admittance','impedance','smc','hinf'):
        add('baseline','baseline','B',c)
        add('feedthrough','feedthrough','B',c)
    for c in ('admittance','impedance'):
        add('baseline','baseline','A',c)
        add('feedthrough','feedthrough','A',c)
    return result


def task_inverse_inertia(dynamics,folder,windows):
    """Mean rotational and translational Mx^-1 at the poses the run visited."""
    with np.load(folder/'trajectory.npz') as saved:
        time=saved['time'].astype(float);q=saved['q'].astype(float)
    result={}
    for label,(low,high) in windows.items():
        samples=q[(time>=low)&(time<high)][::500]
        result[label]=np.mean([np.linalg.inv(dynamics.evaluate(x,np.zeros(7)).inertia) for x in samples],axis=0).tolist()
    return result


def run_case(name,job,root,resume):
    label,_=name.split('/')
    job=dict(job);variant=job.pop('variant')
    folder=root/label/f"{job['experiment']}_{job['controller']}_trial00_seed100"
    record=folder/'variant.json'
    if resume and record.exists():
        saved=json.loads(record.read_text())
        if saved['source_sha256']==source_hash() and saved['variant']==variant:
            return saved
    captured={}
    experiment.make_controller=variant_maker(variant,captured)
    summary=run_experiment(**job,output=root/label)
    windows=D_WINDOWS if job['experiment']=='D' else dict(all=(0.,summary['simulated_duration_s']+1.))
    saved=dict(source_sha256=source_hash(),variant=variant,status=summary['status'],
               simulated_duration_s=summary['simulated_duration_s'],termination_reason=summary.get('termination_reason'),
               task_inverse_inertia=task_inverse_inertia(captured['dynamics'],folder,windows))
    record.write_text(json.dumps(saved,indent=2))
    return saved


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/d-admittance'))
    parser.add_argument('--workers',type=int,default=8)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    capture_source(args.output)
    jobs=cases()
    manifest=dict(source_sha256=source_hash(),expected_cases=list(jobs),completed={},errors={})
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_case,name,job,args.output,args.resume):name for name,job in jobs.items()}
        for future in as_completed(futures):
            name=futures[future]
            try:manifest['completed'][name]=future.result()
            except Exception as error:manifest['errors'][name]=repr(error)
            print(name,manifest['completed'].get(name,{}).get('status','error'),flush=True)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    if manifest['errors']:raise SystemExit(json.dumps(manifest['errors'],indent=2))


if __name__=='__main__':
    main()
