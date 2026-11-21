"""Measured TB6612 feed-forward curve and small encoder speed correction.

Both GPIO phases' rising/falling edges sum to quadrature x4 count magnitude.
The reference scale assumes the same encoder resolution and gear ratio as the other motors.
"""
import math


def interpolate(points, target):
    if not points:
        raise ValueError('Speed calibration has no measured points')
    if target <= points[0][0]:
        return points[0][1]
    for (x0,y0),(x1,y1) in zip(points,points[1:]):
        if target <= x1:
            return y0+(y1-y0)*(target-x0)/(x1-x0)
    return points[-1][1]


class SpeedMatch:
    def __init__(self, profile, command, trim=1):
        direction='positive' if command>0 else 'negative'
        self.points=profile[direction]
        if len(self.points)<2 or any(len(p)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p) for p in self.points):
            raise ValueError('Invalid measured speed curve')
        if any(a[0]>=b[0] for a,b in zip(self.points,self.points[1:])) or any(x<0 or not 0<=y<=1 for x,y in self.points):
            raise ValueError('Speed curve must have increasing rates and bounded PWM duty')
        self.target=abs(command)*profile['reference_counts_per_command']*trim
        if not 0<self.target<=self.points[-1][0]:
            raise ValueError('Requested speed is outside the measured calibration range')
        minimum=profile.get('minimum_count_rate',0)
        maximum=profile.get('maximum_count_rate',self.points[-1][0])
        if not minimum<=self.target<=maximum:
            raise ValueError(f'Requested {self.target:.0f} counts/s is outside the reliable measured range {minimum:.0f}–{maximum:.0f} counts/s')
        self.feedforward=interpolate(self.points,self.target)
        self.maximum=max(y for _,y in self.points)
        self.integral=0
        self.duty=self.feedforward

    def update(self, measured_rate, dt):
        error=self.target-measured_rate
        self.integral=max(-.015,min(.015,self.integral+error*dt*.00008))
        self.duty=max(0,min(self.maximum,self.feedforward+error*.00002+self.integral))
        return self.duty


class PrecisionPWM:
    """Use lgpio's fractional duty support; gpiozero's lgpio backend truncates to integer %."""
    def __init__(self, pin=18, frequency=1000, chip=0):
        import lgpio
        self.gpio=lgpio
        self.pin=pin
        self.frequency=frequency
        self.handle=lgpio.gpiochip_open(chip)
        self._value=0
        try:
            lgpio.gpio_claim_output(self.handle,pin,0)
        except BaseException:
            lgpio.gpiochip_close(self.handle)
            raise

    @property
    def value(self):return self._value

    @value.setter
    def value(self,value):
        if not math.isfinite(value) or not 0<=value<=1:
            raise ValueError('PWM duty must be 0..1')
        self.gpio.tx_pwm(self.handle,self.pin,self.frequency,value*100)
        self._value=value

    def off(self):
        self.gpio.tx_pwm(self.handle,self.pin,self.frequency,0)
        self.gpio.gpio_write(self.handle,self.pin,0)
        self._value=0

    def __enter__(self):return self

    def __exit__(self,*args):
        try:self.off()
        finally:
            try:self.gpio.gpio_free(self.handle,self.pin)
            finally:self.gpio.gpiochip_close(self.handle)
