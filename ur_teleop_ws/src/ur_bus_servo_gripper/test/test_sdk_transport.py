"""Exercise actual pinned sms_sts packets against a pseudo-terminal, never USB."""
from dataclasses import replace
import os
import pty
import select
import threading
import time
import json
import pytest
from ur_bus_servo_gripper.gripper_config import Config
from ur_bus_servo_gripper.st3215_driver import Driver


class Bus:
    def __init__(self):
        self.master,self.slave=pty.openpty()
        self.path=os.ttyname(self.slave)
        self.stop=threading.Event();self.instructions=[];self.respond=True
        self.mode='normal';self.delay=0.
        self.memory=bytearray(100)
        self.memory[3:5]=(777).to_bytes(2,'little')
        self.memory[11:13]=(4095).to_bytes(2,'little')
        self.memory[30]=1
        self.memory[56:58]=(1650).to_bytes(2,'little')
        self.memory[62]=120;self.memory[63]=25
        self.thread=threading.Thread(target=self.run);self.thread.start()
    def run(self):
        buffer=b''
        while not self.stop.is_set():
            if not select.select([self.master],[],[],.01)[0]:continue
            buffer+=os.read(self.master,256)
            while len(buffer)>=4 and len(buffer)>=buffer[3]+4:
                length=buffer[3]+4;packet,buffer=buffer[:length],buffer[length:]
                servo,op=packet[2],packet[4];self.instructions.append((servo,op,packet[5:-1]))
                if servo!=7 or not self.respond:continue
                data=b''
                if op==2:
                    address,count=packet[5:7];data=self.memory[address:address+count]
                elif op==3:
                    address=packet[5];self.memory[address:address+len(packet[6:-1])]=packet[6:-1]
                if self.delay: time.sleep(self.delay)
                if self.mode=='short': data=data[:-1]
                if self.mode=='long': data=data+b'\x00'
                body=bytes([servo if self.mode!='wrong_id' else 8,len(data)+2,
                            4 if self.mode=='motor' else 0])+data
                response=b'\xff\xff'+body+bytes([((~sum(body))&255) ^ (1 if self.mode=='corrupt' else 0)])
                if self.mode=='incomplete': response=response[:-2]
                os.write(self.master,response)
    def close(self):
        self.stop.set();self.thread.join();os.close(self.master);os.close(self.slave)


@pytest.fixture
def bus():
    b=Bus()
    yield b
    b.close()


def config(bus,**kwargs):
    return replace(Config(enabled=True,port=bus.path,servo_id=7,open_ticks=1000,
                          closed_ticks=2300,min_ticks=990,max_ticks=2310),**kwargs)


def test_real_sdk_read_only_then_authorized_write(bus):
    driver=Driver(config(bus))
    try:
        assert driver.connect().ticks==1650
        assert all(op in (1,2) for _,op,_ in bus.instructions)
        assert driver.identity['mode']==0
        driver.write(1654)
        writes=[bytes(data) for _,op,data in bus.instructions if op==3]
        assert writes[0]==bytes([41,5,118,6,0,0,100,0])
        assert writes[1]==bytes([40,1])
        assert not any(data[0]<40 or data[0]==55 for data in writes)
        bus.respond=False
        with pytest.raises(IOError,match='communication'):driver.read()
    finally:driver.close()


def test_wrong_id_and_reconnect_never_moves(bus):
    wrong=Driver(config(bus,servo_id=8))
    with pytest.raises(IOError):wrong.connect()
    driver=Driver(config(bus))
    driver.connect();driver.close();driver.connect();driver.close()
    assert all(op in (1,2) for _,op,_ in bus.instructions)


def test_wrong_mode_refused(bus):
    bus.memory[33]=3
    with pytest.raises(IOError,match='single-turn'):Driver(config(bus)).connect()
    assert all(op in (1,2) for _,op,_ in bus.instructions)


def test_exclusive_owner(bus):
    driver=Driver(config(bus));driver.connect()
    try:
        with pytest.raises(OSError):Driver(config(bus)).connect()
    finally:driver.close()


@pytest.mark.parametrize('mode,category', [('short','invalid_responses'),
    ('long','invalid_responses'), ('corrupt','rx_corruption'),
    ('incomplete','rx_timeouts'), ('wrong_id','rx_timeouts'), ('motor','motor_errors')])
def test_malformed_packets_are_not_temperatures(bus,mode,category):
    driver=Driver(config(bus));driver.connect()
    bus.mode=mode
    try:
        with pytest.raises(IOError):driver.read()
        if mode == 'incomplete':
            assert sum(driver.diagnostics.counters[k] for k in ('rx_timeouts','rx_corruption')) == 1
        else:
            assert driver.diagnostics.counters[category] == 1
        assert driver.diagnostics.counters['consecutive_failures'] == 1
    finally:driver.close()


def test_valid_checksum_high_temperature_is_valid_data_not_corruption(bus):
    driver=Driver(config(bus));driver.connect()
    bus.memory[63]=107
    try:
        assert driver.read().temperature_c==107
        assert driver.read_temperature()[0]==107
        assert driver.diagnostics.counters['rx_corruption']==0
    finally:driver.close()


def test_deadline_and_cleanup_with_late_response(bus):
    driver=Driver(config(bus));driver.connect()
    bus.delay=.06
    start=time.monotonic()
    try:
        with pytest.raises(IOError):driver.read_temperature()
        assert time.monotonic()-start < .15
        assert driver.diagnostics.counters['rx_timeouts']==1
    finally:driver.close()
    assert not driver.port.is_open


def test_mock_latency_measurement(bus, capsys):
    driver=Driver(config(bus));driver.connect()
    try:
        for _ in range(100):
            driver.read();driver.read_temperature();driver.write(1650)
        baseline=driver.diagnostics.snapshot()
        bus.delay=.008
        driver.read();driver.read_temperature();driver.write(1650)
        result=driver.diagnostics.snapshot()
        print('PTY_TIMINGS '+json.dumps(dict(baseline=baseline,with_injected_latency=result)))
        assert result['counters']['rx_timeouts']==0
        assert result['timings']['feedback']['max_ms']>=8
    finally:driver.close()
    output=capsys.readouterr().out
    with open('/tmp/teleop_v62_pty_timings.json','w') as stream:
        stream.write(output.removeprefix('PTY_TIMINGS '))


def test_worker_shutdown_pending_real_sdk_read(bus,tmp_path):
    from ur_bus_servo_gripper.gripper_worker import Worker
    from ur_bus_servo_gripper.gripper_config import save
    path=tmp_path/'g.json';save(path,config(bus))
    w=Worker(path);w.request('connect');w.start()
    deadline=time.monotonic()+2
    while not w.snapshot().get('feedback_valid') and time.monotonic()<deadline:
        time.sleep(.005)
    assert w.snapshot().get('feedback_valid')
    bus.delay=.06
    # Wait until the next read has actually been transmitted.
    count=len(bus.instructions)
    while len(bus.instructions)==count and time.monotonic()<deadline:time.sleep(.001)
    start=time.monotonic();w.close()
    assert time.monotonic()-start < .3
    assert not w.thread.is_alive()
    assert all(op in (1,2) for _,op,_ in bus.instructions)


def test_real_sdk_worker_confirmation_is_read_only(bus,tmp_path):
    from ur_bus_servo_gripper.gripper_worker import Worker
    from ur_bus_servo_gripper.gripper_config import save
    path=tmp_path/'g.json';save(path,config(bus))
    w=Worker(path)
    try:
        w.request('connect');w.step()
        bus.memory[63]=107;w.step()
        assert w.temperature.state=='TEMPERATURE_UNVERIFIED'
        assert w.last_temperature_event['feedback']['raw_bytes'][7]==107
        bus.memory[63]=35;w.step()
        bus.memory[63]=36;w.step()
        assert w.temperature.state=='RECOVERY_REQUIRED'
        assert w.temperature.confirmations==[35,36]
        assert all(op in (1,2) for _,op,_ in bus.instructions)
        assert any(op==2 and bytes(data)==bytes([63,1]) for _,op,data in bus.instructions)
    finally:w.disconnect()


@pytest.mark.parametrize('result', [None, ([],0), ([0]*14,0,0), ([0]*16,0,0),
    ([0]*7+[256]+[0]*7,0,0), ([0]*15,'0',0), ([0]*15,0,'0')])
def test_invalid_sdk_result_never_becomes_feedback(result):
    class Packet:
        def readTxRx(self,*args):return result
    driver=Driver(Config(),sdk=object());driver.packet=Packet();driver.servo_id=7
    with pytest.raises(IOError):driver.read()
    assert driver.diagnostics.counters['invalid_responses']==1


@pytest.mark.parametrize('communication,category', [(-6,'rx_timeouts'),(-7,'rx_corruption'),
    (-1,'other_errors'),(-2,'other_errors'),(-3,'other_errors'),(-4,'other_errors'),(-9,'other_errors')])
def test_sdk_communication_categories(communication,category):
    class Packet:
        def readTxRx(self,*args):return [],communication,0
    driver=Driver(Config(),sdk=object());driver.packet=Packet();driver.servo_id=7
    with pytest.raises(IOError):driver.read_temperature()
    assert driver.diagnostics.counters[category]==1
    assert driver.diagnostics.last['communication']==communication


def test_continuous_other_id_packets_cannot_evade_deadline(bus):
    driver=Driver(config(bus));driver.connect()
    # Infinite valid-checksum packets for another ID exercise the SDK branch
    # that does not itself check timeout. Only a pseudo-terminal is open.
    body=bytes([8,17,0])+bytes(15)
    packet=b'\xff\xff'+body+bytes([(~sum(body))&255])
    offset=[0]
    original=driver.port.ser.read
    def endless(count):
        start=offset[0];offset[0]+=count
        return bytes(packet[i%len(packet)] for i in range(start,start+count))
    driver.port.ser.read=endless
    errors=[]
    def receive():
        try:driver.read()
        except Exception as exc:errors.append(exc)
    thread=threading.Thread(target=receive)
    thread.start();thread.join(.2)
    finished=not thread.is_alive()
    driver.port.ser.read=original
    thread.join(.2)
    driver.close()
    assert finished and errors
    assert driver.diagnostics.counters['rx_timeouts']==1


def test_discovery_survives_missing_ids_and_releases_sdk_busy_flag(bus):
    driver=Driver(config(bus,servo_id=-1,enabled=False))
    try:
        assert driver.connect().ticks==1650
        assert driver.servo_id==7
        assert not driver.port.is_using
        assert driver.diagnostics.counters['rx_timeouts']==253
        assert all(op in (1,2) for _,op,_ in bus.instructions)
    finally:driver.close()
