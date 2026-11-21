"""Record lifted-wheel Robot 1 speed sweeps through dashboard diagnostics.

This moves installed wheels. Run only with every wheel lifted. Saves every raw sample,
restores the previous dashboard trim, and leaves click driving disarmed.
"""
import argparse
import json
from pathlib import Path
import time
import urllib.request


def api(base, action='', data=None):
    request=urllib.request.Request(base+action,data=None if data is None else json.dumps(data).encode(),
        headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=5) as response:
        return json.load(response)


def steady_rate(state):
    samples=[s for s in state['samples'] if s.get('state')=='running' and s.get('elapsed',0)>=.75]
    if len(samples)<5:
        raise ValueError('Not enough steady samples')
    first,last=samples[0],samples[-1]
    rates=[(v-z)/(last['elapsed']-first['elapsed']) for v,z in zip(last['counts'],first['counts'])]
    index=int(state['channel'][1])-1 if state['channel']!='TB6612' else None
    return rates, abs(rates[index]) if index is not None else sum(abs(v) for v in rates)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheels-lifted',action='store_true',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--base',default='http://localhost:8090/api/motors')
    args=parser.parse_args()
    previous=api(args.base)['settings']
    result={'previous_settings':previous,'tb6612_test_scale':.2,'runs':[]}
    def save():args.output.write_text(json.dumps(result,indent=2)+'\n')
    save()
    try:
        api(args.base,'/settings',{'tb6612_scale':.2})
        plan=[(c,s) for c in ('M2','M3','M4') for s in (10,30,50,-30)]
        plan += [('TB6612',s*d) for d in (1,-1) for s in (10,15,20,25,30,40,50)]
        for channel,speed in plan:
            api(args.base,'/start',dict(robot_id=1,channel=channel,speed=speed,seconds=3,tb6612_open_loop=True))
            deadline=time.monotonic()+15
            while True:
                time.sleep(.2)
                state=api(args.base)
                if not state['running']:break
                if time.monotonic()>deadline:raise RuntimeError('Test timed out')
            if state['state']!='finished':raise RuntimeError(state)
            rates,total=steady_rate(state)
            state['steady_rates']=rates;state['comparable_count_rate']=total
            result['runs'].append(state);save()
            print(f'{channel} {speed:+}% command={state["command"]} duty={state.get("applied_duty_percent")} rate={total:.1f} channels={[round(v,1) for v in rates]}',flush=True)
            time.sleep(.4)
    finally:
        state=api(args.base,'/stop',{})
        for _ in range(30):
            if not state['running']:break
            time.sleep(.2);state=api(args.base)
        api(args.base,'/settings',previous)
        save()

if __name__=='__main__':main()
