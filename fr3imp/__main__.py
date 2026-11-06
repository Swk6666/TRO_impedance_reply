import argparse
import json
import numpy as np
from .actuation import ActuatorParameters
from .experiment import run_experiment
from .profiles import profile_parameters


def main():
    parser=argparse.ArgumentParser(description='FR3 paper reproduction')
    parser.add_argument('--experiment',choices=list('ABCDEF'),default='A')
    parser.add_argument('--controller',default='hinf',choices=['hinf','impedance','admittance','bidirectional','smc','hinf_0.25','hinf_0.5','hinf_1'])
    parser.add_argument('--duration',type=float)
    parser.add_argument('--trial',type=int,default=0)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--output',default='outputs')
    parser.add_argument('--ideal',action='store_true')
    parser.add_argument('--physics-substeps',type=int,default=4)
    parser.add_argument('--control-period',type=float,choices=[.001,.00025],default=.001)
    parser.add_argument('--c-profile',choices=['fr3','legacy'],default='fr3')
    parser.add_argument('--profile',choices=['fr3','paper','legacy'],default='fr3')
    parser.add_argument('--grinder-model',choices=['rotor','external'],default=None,
                        help='E: internal eccentric rotor (paper default) or historical applied-force approximation')
    parser.add_argument('--inertia-scale',type=float,default=1.,
                        help='Explicit adaptation: scale task inertia and scale damping by its square root; keep stiffness')
    parser.add_argument('--operator-stiffness-scale',type=float,default=None,
                        help='D: scale virtual yaw/pitch grasp stiffness (not roll)')
    parser.add_argument('--command-cutoff',type=float,default=None,help='Override command low-pass cutoff in Hz; 0 bypasses it')
    args=parser.parse_args()
    kwargs=vars(args);cutoff=kwargs.pop('command_cutoff');scale=kwargs.pop('inertia_scale')
    if not np.isfinite(scale) or scale<=0:parser.error('inertia scale must be finite and positive')
    if scale!=1.:
        p=profile_parameters(args.experiment,args.profile,args.c_profile)['impedance']
        L,D,K=(np.asarray(p[k]) for k in ('inertia','damping','stiffness'))
        kwargs['impedance_overrides']=dict(inertia=(L*scale).tolist(),damping=(D*np.sqrt(scale)).tolist(),stiffness=K.tolist())
    if cutoff is not None:kwargs['actuator_parameters']=ActuatorParameters(cutoff_hz=cutoff)
    result=run_experiment(**kwargs)
    print(json.dumps(result['metrics'],indent=2))


if __name__=='__main__':main()
