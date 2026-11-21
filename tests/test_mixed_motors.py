from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from robot.mixed_motors import MixedMotors,TB6612Motor
from robot.drive_server import DriveServer,DriveParams,PrintMotors


def profile():
    return dict(reference_counts_per_command=100,minimum_count_rate=200,maximum_count_rate=500,
                positive=[[0,.02],[200,.03],[500,.07]],negative=[[0,.02],[200,.035],[500,.08]])


def make():
    board=PrintMotors();tb=Mock();tb.telemetry.return_value={}
    return MixedMotors(board,tb,profile()),board,tb


def test_replacement_routes_to_tb_only_with_all_other_wheels_preserved():
    mixed,board,tb=make()
    mixed.set_motor_speed_counts((3,4,-2,0))
    tb.set.assert_called_once_with(3)
    assert board.values==(0,4,-2,0)
    assert mixed.last_values==[3,4,-2,0]
    assert mixed.telemetry()['hiwonder_commands'][0]==0
    mixed.stop();tb.stop.assert_called_once()
    assert board.values==(0,0,0,0)


def test_low_speed_stops_whole_vector_not_just_replacement():
    mixed,board,tb=make()
    mixed.set_motor_speed_counts((1,5,3,2))
    tb.set.assert_not_called()
    assert board.values==(0,0,0,0)
    assert mixed.last_values==[0]*4
    assert 'whole chassis' in mixed.guard
    mixed.set_motor_speed_counts((0,3,2,-3)) # Exact mecanum diagonal: FR legitimately stationary.
    assert board.values==(0,3,2,-3)
    tb.set.assert_called_once_with(0)
    assert mixed.guard is None


def test_runtime_fault_latches_and_does_not_retry_motor_on_next_command():
    mixed,board,tb=make();tb.set.side_effect=RuntimeError('encoder lost')
    mixed.set_motor_speed_counts((3,3,3,3))
    assert mixed.fault=='encoder lost'
    assert board.values==(0,0,0,0)
    mixed.set_motor_speed_counts((4,4,4,4))
    assert tb.set.call_count==1
    assert mixed.last_values==[0]*4


def test_driver_watchdog_stops_both_backends_and_reports_effective_zero():
    mixed,board,tb=make()
    p=DriveParams([3,1,4,2],[1,1,-1,1],max_counts=5)
    server=DriveServer(mixed,p,1,host='127.0.0.1',port=0,log=lambda _:None)
    try:
        server.target=(.03,0,0);server.last_cmd=1
        server.tick(1)
        assert tb.set.called
        server.tick(2)
        assert mixed.last_values==[0]*4
        assert board.values==(0,0,0,0)
        assert server._status_reply(1,2)['mixed_drive']['backend']=='hiwonder+tb6612'
    finally:server.sock.close()


def test_pwm_and_bridge_are_off_during_direction_reversal():
    now=[1.]
    motor=TB6612Motor.__new__(TB6612Motor)
    motor.profile=profile();motor.clock=lambda:now[0]
    motor.command=3;motor.matcher=SimpleNamespace(integral=0)
    motor.pwm=SimpleNamespace(value=.04,off=lambda:setattr(motor.pwm,'value',0))
    motor.standby=Mock();motor.in1=Mock();motor.in2=Mock()
    motor.history=[];motor.edges=[100,100];motor.rate=300
    motor.set(-3)
    assert motor.pwm.value==0
    motor.standby.off.assert_called_once()
    assert motor.started==pytest.approx(1.1)
    now[0]=1.05;motor.set(-3)
    assert motor.pwm.value==0


def test_disconnected_encoder_phase_is_not_mistaken_for_a_slower_motor():
    from collections import deque
    now=[0.]
    motor=TB6612Motor.__new__(TB6612Motor)
    motor.profile=profile();motor.clock=lambda:now[0]
    motor.command=0;motor.matcher=None;motor.edges=[0,0]
    motor.history=deque();motor.rate=None;motor.previous=0
    motor.pwm=SimpleNamespace(value=0,off=lambda:setattr(motor.pwm,'value',0))
    motor.standby=Mock();motor.in1=Mock();motor.in2=Mock()
    motor.set(3)
    now[0]=.8;motor.edges=[80,0]
    with pytest.raises(RuntimeError,match='phases disagree'):
        motor.set(3)
