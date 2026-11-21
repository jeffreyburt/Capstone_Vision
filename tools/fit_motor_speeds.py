"""Fit monotonic TB6612 speed curves from measure_motor_speeds recordings."""
import argparse
import json
from pathlib import Path
import statistics


def fit(recording):
    runs=recording['runs']
    original=[r for r in runs if r['channel']!='TB6612']
    if {r['channel'] for r in original}!={'M2','M3','M4'}:
        raise ValueError('Need measured reference motors M2, M3, M4')
    reference=statistics.median(r['comparable_count_rate']/abs(r['command']) for r in original)
    result={'reference_counts_per_command':reference,
            'minimum_count_rate':reference*2,'maximum_count_rate':reference*5,'count_convention':'sum of both GPIO phases, both edges (x4 magnitude)',
            'encoder_resolution_assumption':'same encoder resolution and gearbox ratio on all four motors',
            'calibration':'lifted-wheel measurements 2026-11-21'}
    for direction,sign in [('positive',1),('negative',-1)]:
        measured=sorted((r['applied_duty_percent']/100,r['comparable_count_rate']) for r in runs
                        if r['channel']=='TB6612' and r['speed']*sign>0)
        if len(measured)<4:raise ValueError('Need multiple speed measurements in both directions')
        # Pool adjacent non-monotonic rate observations instead of fitting negative slopes.
        blocks=[]
        for duty,rate in measured:
            blocks.append([duty,rate,1])
            while len(blocks)>1 and blocks[-2][1]/blocks[-2][2]>=blocks[-1][1]/blocks[-1][2]:
                b=blocks.pop();a=blocks.pop();blocks.append([a[i]+b[i] for i in range(3)])
        result[direction]=[[rate/n,duty/n] for duty,rate,n in blocks]
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('recording',type=Path);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    result=fit(json.loads(args.recording.read_text()))
    args.output.write_text(json.dumps({'tb6612_scale':1.0,'tb6612_calibration':result},indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
