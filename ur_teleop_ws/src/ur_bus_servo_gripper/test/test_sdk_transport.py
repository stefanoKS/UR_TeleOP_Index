"""Exercise actual pinned sms_sts packets against a pseudo-terminal, never USB."""
from dataclasses import replace
import os
import pty
import select
import threading
import pytest
from ur_bus_servo_gripper.gripper_config import Config
from ur_bus_servo_gripper.st3215_driver import Driver


class Bus:
    def __init__(self):
        self.master,self.slave=pty.openpty()
        self.path=os.ttyname(self.slave)
        self.stop=threading.Event();self.instructions=[];self.respond=True
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
                body=bytes([servo,len(data)+2,0])+data
                os.write(self.master,b'\xff\xff'+body+bytes([(~sum(body))&255]))
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
