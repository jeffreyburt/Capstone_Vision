"""Brief TB6612FNG channel-A test with quadrature feedback (BCM pin numbering).

AIN1=17 (header 11), AIN2=27 (13), PWMA=18 (12), STBY=22 (15).
Encoder A=23 (16), B=24 (18). Driver logic VCC and encoder VCC use 3.3V;
VM uses the motor supply; Pi, encoder and driver share ground. Motor goes to AO1/AO2.

Run on the robot: python3 tb6612_encoder_test.py
Counts are gpiozero complete quadrature steps, not Hiwonder's raw x4 edge counts.
"""
import argparse
import math
import time
from contextlib import ExitStack


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--in1',type=int,default=17)
    parser.add_argument('--in2',type=int,default=27)
    parser.add_argument('--pwm',type=int,default=18)
    parser.add_argument('--stby',type=int,default=22)
    parser.add_argument('--encoder-a',type=int,default=23)
    parser.add_argument('--encoder-b',type=int,default=24)
    parser.add_argument('--duty',type=float,default=.25,help='PWM duty, 0..1')
    parser.add_argument('--seconds',type=float,default=3.)
    parser.add_argument('--reverse',action='store_true')
    args=parser.parse_args(argv)
    pins=[args.in1,args.in2,args.pwm,args.stby,args.encoder_a,args.encoder_b]
    if len(set(pins))!=len(pins) or any(not 0<=p<=27 for p in pins):
        parser.error('Use distinct BCM GPIO numbers 0..27')
    if not math.isfinite(args.duty) or not 0<=args.duty<=1 or not math.isfinite(args.seconds) or args.seconds<=0:
        parser.error('Need duty 0..1 and positive finite seconds')
    from gpiozero import OutputDevice,PWMOutputDevice,RotaryEncoder
    from gpiozero.pins.lgpio import LGPIOFactory
    with ExitStack() as stack:
        factory=LGPIOFactory()
        stack.callback(factory.close)
        standby=stack.enter_context(OutputDevice(args.stby,initial_value=False,pin_factory=factory))
        in1=stack.enter_context(OutputDevice(args.in1,initial_value=False,pin_factory=factory))
        in2=stack.enter_context(OutputDevice(args.in2,initial_value=False,pin_factory=factory))
        pwm=stack.enter_context(PWMOutputDevice(args.pwm,frequency=1000,initial_value=0,pin_factory=factory))
        encoder=stack.enter_context(RotaryEncoder(args.encoder_a,args.encoder_b,max_steps=0,bounce_time=None,pin_factory=factory))
        print(f'BCM IN1={args.in1}, IN2={args.in2}, PWM={args.pwm}, STBY={args.stby}; encoder={args.encoder_a}/{args.encoder_b}',flush=True)
        started=time.monotonic();initial=encoder.steps;last=initial;last_t=started
        try:
            in1.value=not args.reverse
            in2.value=args.reverse
            standby.on()
            pwm.value=args.duty
            print(f'Running {args.duty*100:g}% PWM for {args.seconds:g}s',flush=True)
            while time.monotonic()-started<args.seconds:
                time.sleep(min(.25,max(0,args.seconds-(time.monotonic()-started))))
                now=time.monotonic();count=encoder.steps
                print(f'{now-started:4.2f}s  count={count-initial:+d}  rate={(count-last)/max(now-last_t,.001):+.0f} steps/s',flush=True)
                last,last_t=count,now
        finally:
            pwm.off();standby.off();in1.off();in2.off()
            print('Motor output off',flush=True)
        time.sleep(.2)
        print(f'Total encoder steps: {encoder.steps-initial:+d}',flush=True)
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted; motor output released')
