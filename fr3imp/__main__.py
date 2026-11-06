import argparse
import json
from .experiment import run_experiment


def main():
    parser=argparse.ArgumentParser(description='FR3 paper reproduction')
    parser.add_argument('--experiment',choices=list('ABCDEF'),default='A')
    parser.add_argument('--controller',default='hinf',choices=['hinf','impedance','admittance','bidirectional','smc','hinf_0.25','hinf_0.5','hinf_1'])
    parser.add_argument('--duration',type=float)
    parser.add_argument('--trial',type=int,default=0)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--output',default='outputs')
    parser.add_argument('--ideal',action='store_true')
    parser.add_argument('--physics-substeps',type=int,default=1)
    args=parser.parse_args()
    result=run_experiment(**vars(args))
    print(json.dumps(result['metrics'],indent=2))


if __name__=='__main__':main()
