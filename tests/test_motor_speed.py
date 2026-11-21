import pytest
from robot.motor_speed import SpeedMatch, interpolate


def profile():
    return dict(reference_counts_per_command=100,
                positive=[[0,.02],[200,.03],[400,.05],[600,.08]],
                negative=[[0,.02],[200,.035],[400,.055],[600,.085]])


def test_direction_curves_and_reference_units():
    assert SpeedMatch(profile(),2).feedforward==.03
    assert SpeedMatch(profile(),-2).feedforward==.035
    assert SpeedMatch(profile(),2,.5).target==100
    assert interpolate([[0,.02],[200,.03]],100)==pytest.approx(.025)
    with pytest.raises(ValueError,match='outside'):
        SpeedMatch(profile(),7)


def test_speed_correction_reduces_error_with_load_and_is_bounded():
    match=SpeedMatch(profile(),3)
    # Independent simple motor plant: slower than the no-load calibration curve.
    rate=lambda duty:max(0,(duty-.012)*7000)
    initial=abs(rate(match.duty)-match.target)
    for _ in range(100):
        match.update(rate(match.duty),.1)
    assert abs(rate(match.duty)-match.target)<initial*.3
    for _ in range(1000):
        match.update(0,.1)
    assert 0<=match.duty<=match.maximum


def test_reject_bad_measurement_curve():
    p=profile();p['positive']=[[100,.03],[100,.04]]
    with pytest.raises(ValueError):SpeedMatch(p,1)


def test_pwm_preserves_fractional_percent_and_releases_output(monkeypatch):
    import sys
    from unittest.mock import Mock
    from robot.motor_speed import PrecisionPWM
    gpio=Mock();gpio.gpiochip_open.return_value=7
    monkeypatch.setitem(sys.modules,'lgpio',gpio)
    with PrecisionPWM() as pwm:
        pwm.value=.02735
        gpio.tx_pwm.assert_called_with(7,18,1000,pytest.approx(2.735))
        assert pwm.value==.02735
    gpio.gpio_write.assert_called_with(7,18,0)
    gpio.gpio_free.assert_called_once_with(7,18)
    gpio.gpiochip_close.assert_called_once_with(7)


def test_saved_trim_preserves_measured_curve(tmp_path):
    import json
    from monitor.motors import MotorTests
    manager=MotorTests(None);manager.settings_path=tmp_path/'motor_speed.json'
    manager.settings_path.write_text(json.dumps({'tb6612_scale':1,'tb6612_calibration':profile()}))
    manager.save_settings({'tb6612_scale':.8})
    assert manager.settings()['tb6612_calibration']==profile()


def test_low_speed_stall_range_is_rejected():
    p=profile();p['minimum_count_rate']=200;p['maximum_count_rate']=500
    with pytest.raises(ValueError,match='reliable measured range'):
        SpeedMatch(p,1)
    assert SpeedMatch(p,2).target==200


def test_fit_pools_inconsistent_raw_measurements():
    from tools.fit_motor_speeds import fit
    runs=[dict(channel=c,command=3,comparable_count_rate=300) for c in ('M2','M3','M4')]
    for sign in (1,-1):
        runs.extend(dict(channel='TB6612',speed=sign*20,applied_duty_percent=duty,comparable_count_rate=rate)
                    for duty,rate in [(2,0),(4,250),(5,240),(8,600)])
    p=fit({'runs':runs})
    assert p['reference_counts_per_command']==100
    assert p['minimum_count_rate']==200
    assert p['positive']==[[0,.02],[245,.045],[600,.08]]
