"""V6.4 provenance, pickup, held-input and callback-race regressions."""
from dataclasses import replace
import json
import pytest
from test_gripper import config, worker, FakeDriver, engage, drive
from ur_bus_servo_gripper.gripper_mapping import Follow, normalize
from ur_bus_servo_gripper.trigger_input import TriggerInput


def observation(t=100., seq=1, event=1, raw=.35, event_time=100., **changes):
    d = dict(schema=1, session='test', device='KN0', stamp_ns=int(t*1e9),
             observation_seq=seq, event_seq=event, event_time=event_time,
             receipt_ns=int(event_time*1e9), event_age_s=t-event_time,
             raw=raw, imu_seq=seq, imu_age_s=.001, pose=True, optical=True,
             state='GENUINE_FRESH_INPUT', rejected_events=0, loss_seq=0)
    d.update(changes)
    return d


def parse(parser, d, fresh=True):
    return parser.parse(json.dumps(d), d['stamp_ns'], fresh)


@pytest.mark.parametrize('reverse,invert', [(False,False),(True,False),(False,True),(True,True)])
def test_validated_crossing_without_exact_match(config, reverse, invert):
    c = replace(config, invert=invert, deadzone=.02, trigger_min=.1, trigger_max=.9,
                open_ticks=2300 if reverse else 1000, closed_ticks=1000 if reverse else 2300)
    f = Follow(c)
    def raw(x):
        x = .02 + x*.96
        return .1 + .8*(1-x if invert else x)
    assert f.step(normalize(raw(.35),c),1650,.04,(1,100.)) is None
    assert f.step(normalize(raw(.55),c),1650,.04,(2,100.04)) is None
    assert f.active and f.pickup_method == 'crossing'
    assert f.reference == f.last_sent == 1650
    assert abs(f.step(.8,1650,.04)-1650) <= 4


@pytest.mark.parametrize('sample', [(2,100.5),(1,100.04),(0,99.),(3,100.04)])
def test_crossing_rejects_stale_reordered_or_missing_sample(config, sample):
    f=Follow(config)
    f.step(.35,1650,.04,(1,100.))
    f.step(.55,1650,.04,sample)
    assert not f.active


def test_pickup_stationary_and_loss_reset(config):
    f=Follow(config)
    for i in range(20):
        assert f.step(.5,1650,.04,(i+1,100.+i*.04)) is None
    assert not f.active
    f.reset();f.step(.35,1650,.04,(30,101.));f.reset()
    f.step(.55,1650,.04,(31,101.04))
    assert not f.active
    f.step(.5,1650,.04,(32,101.08))
    assert f.active and f.pickup_method=='proximity'


@pytest.mark.parametrize('raw', [float('nan'),float('inf'),-.1,1.1])
def test_invalid_pickup_values(config,raw):
    with pytest.raises(ValueError):Follow(config).step(raw,1650,.04,(1,100.))


def test_timer_publication_does_not_refresh_event():
    p=TriggerInput()
    a=parse(p,observation())
    b=parse(p,observation(t=110.,seq=2,state='VALID_HELD_INPUT'))
    assert a['new_event'] and b['valid'] and not b['new_event']
    assert b['event_time']==a['event_time'] and b['event_age_s']==10.
    c=parse(p,observation(t=110.01,seq=3,state='VALID_HELD_INPUT',event_age_s=0.))
    assert not c['valid'] and 'age refreshed' in c['rejection']


@pytest.mark.parametrize('changes', [dict(raw=0.),dict(event_time=101.),dict(receipt_ns=100000000001),
    dict(observation_seq=1),dict(imu_seq=0),dict(imu_age_s=.3),dict(pose=False),dict(raw=True)])
def test_malformed_or_replayed_held_input(changes):
    p=TriggerInput();parse(p,observation())
    assert not parse(p,observation(t=100.04,seq=2,**changes))['valid']


def test_stale_dds_and_no_synthetic_zero():
    p=TriggerInput()
    assert not parse(p,observation(),False)['valid']
    a=parse(p,observation(raw=.7))
    b=parse(p,observation(t=100.04,seq=2,raw=.7,pose=False,state='TEMPORARILY_UNVERIFIED'))
    c=parse(p,observation(t=100.08,seq=3,raw=.7,state='VALID_HELD_INPUT'))
    assert a['valid'] and not b['valid'] and c['valid'] and c['raw']==.7
    assert not c['new_event']


def test_genuine_zero_is_not_signal_loss(worker):
    engage(worker);drive(worker,0.)
    assert worker.signal_lost_at is None
    assert worker.snapshot()['follow_state']=='FOLLOWING'


def deliver(worker, parser, seq, raw=.35, event=1, event_time=100., **changes):
    worker.now[0] += .04
    d=parse(parser,observation(t=worker.now[0],seq=seq,raw=raw,event=event,event_time=event_time,**changes))
    worker.observe_trigger(d);worker.update('heartbeat',True);worker.step()
    return d


def test_coherent_worker_crossing_and_held_short_recovery(worker):
    p=TriggerInput();t=worker.now[0]
    deliver(worker,p,1,event_time=t)
    deliver(worker,p,2,raw=.55,event=2,event_time=t+.04)
    assert worker.follow.active and worker.follow.pickup_method=='crossing'
    assert not FakeDriver.writes
    deliver(worker,p,3,raw=.7,event=3,event_time=t+.08)
    count=len(FakeDriver.writes)
    deliver(worker,p,4,raw=.7,event=3,event_time=t+.08,pose=False,state='TEMPORARILY_UNVERIFIED')
    assert len(FakeDriver.writes)==count
    for seq in range(5,13):
        deliver(worker,p,seq,raw=.7,event=3,event_time=t+.08,state='VALID_HELD_INPUT')
    assert worker.follow.active and worker.snapshot()['recovery_count']==1
    assert FakeDriver.writes[count]>=1650  # no false release/opening
    assert max(FakeDriver.writes)<=1670


def test_no_verification_from_repeated_cached_device_observations(worker):
    engage(worker)
    p=TriggerInput();t=worker.now[0]
    deliver(worker,p,1,raw=.7,event_time=t,pose=False,state='TEMPORARILY_UNVERIFIED')
    count=len(FakeDriver.writes)
    for seq in range(2,10):
        deliver(worker,p,seq,raw=.7,event_time=t,imu_seq=2,state='VALID_HELD_INPUT')
    assert len(FakeDriver.writes)==count
    assert worker.snapshot()['follow_state']!='FOLLOWING'


def test_history_is_bounded(worker):
    for i in range(100):
        worker.follow_state='SIGNAL_PAUSED' if i%2 else 'PICKUP_REQUIRED'
        worker.recovery_fields()
    assert len(worker.follow_history)==32


def test_lost_invalid_message_cannot_hide_interruption(worker):
    p=TriggerInput();t=worker.now[0]
    deliver(worker,p,1,event_time=t)
    # The invalid publication was dropped by DDS; the next healthy observation
    # carries its loss generation and cannot complete a crossing across it.
    deliver(worker,p,3,raw=.55,event=2,event_time=t+.04,loss_seq=1)
    assert not worker.follow.active and not FakeDriver.writes
    assert worker.signal_lost_at is not None


def test_stop_between_position_ack_and_torque_enable(config):
    from ur_bus_servo_gripper.st3215_driver import Driver
    calls=[];allowed=[True]
    class Packet:
        def WritePosEx(self,*args):
            calls.append('position');allowed[0]=False
            return 0,0
        def write1ByteTxRx(self,*args):
            calls.append('torque');return 0,0
    driver=Driver(config,sdk=object());driver.servo_id=7;driver.packet=Packet()
    driver.write_permitted=lambda:allowed[0]
    assert driver.write(1650) is False
    assert calls==['position'] and not driver.torque_enabled
    assert driver.write(1650) is False
    assert calls==['position']
