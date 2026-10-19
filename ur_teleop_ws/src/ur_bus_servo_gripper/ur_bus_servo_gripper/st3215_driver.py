"""Single-owner ST-series SDK adapter. No EEPROM or mode writes; torque enable only on authorized motion."""
from dataclasses import dataclass
from glob import glob
import time


@dataclass(frozen=True)
class Feedback:
    ticks: int
    speed_raw: int
    load_raw: int
    voltage_v: float
    temperature_c: int
    current_raw: int
    motor_status: int
    moving: bool
    received_at: float


def ports():
    return sorted(glob('/dev/serial/by-id/*'))


def checked(result):
    *values, communication, error = result
    if communication != 0 or error != 0:
        raise IOError(f'ST3215 communication={communication}, motor_error=0x{error:02x}')
    return values[0] if len(values) == 1 else values


def signed(value, bit):
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


class Driver:
    def __init__(self, config, sdk=None):
        if sdk is None:
            import scservo_sdk as sdk
        self.config, self.sdk = config, sdk
        self.port = self.packet = None
        self.servo_id = None
        self.identity = {}
        self.torque_enabled = False

    def connect(self, cancelled=lambda: False):
        paths = ports()
        path = self.config.port
        if not path:
            if len(paths) != 1:
                raise IOError('Select exactly one USB adapter using Gripper Setup')
            path = paths[0]
        # Use SDK packet implementation with bounded exclusive Linux transport.
        # Its stock clearPort uses unbounded tcdrain; replace with input flush.
        import serial
        base = self.sdk.PortHandler

        class BoundedPort(base):
            def setupPort(self, baud):
                if self.is_open:
                    self.closePort()
                self.ser = serial.Serial(self.port_name, baudrate=baud, timeout=0,
                                         write_timeout=.02, exclusive=True)
                self.is_open = True
                self.ser.reset_input_buffer()
                self.tx_time_per_byte = 10000. / baud
                return True

            def clearPort(self):
                self.ser.reset_input_buffer()

            def getCurrentTime(self):
                return time.monotonic() * 1000.

            def setPacketTimeout(self, length):
                self.setPacketTimeoutMillis(20.)

        self.port = BoundedPort(path)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(1000000):
                raise IOError('Cannot open adapter at 1000000 baud')
            self.packet = self.sdk.sms_sts(self.port)
            ids = []
            candidates = range(254) if self.config.servo_id == -1 else [self.config.servo_id]
            for servo_id in candidates:
                if cancelled():
                    raise IOError('Discovery cancelled')
                model, result, error = self.packet.ping(servo_id)
                if result == 0:
                    checked((model, result, error))
                    ids.append((servo_id, model))
                elif self.config.servo_id != -1:
                    checked((model, result, error))
            if len(ids) != 1:
                raise IOError(f'Expected one servo, found IDs {[i[0] for i in ids]}')
            self.servo_id, model = ids[0]
            mode = checked(self.packet.read1ByteTxRx(self.servo_id, 33))
            resolution = checked(self.packet.read1ByteTxRx(self.servo_id, 30))
            low = checked(self.packet.read2ByteTxRx(self.servo_id, 9))
            high = checked(self.packet.read2ByteTxRx(self.servo_id, 11))
            phase = checked(self.packet.read1ByteTxRx(self.servo_id, 18))
            if mode != 0 or resolution != 1 or not 0 <= low < high <= 4095:
                raise IOError('Requires reviewed single-turn mode 0, resolution 1, nonzero angle range')
            if self.config.enabled and not low <= self.config.min_ticks <= self.config.max_ticks <= high:
                raise IOError('Calibration exceeds servo angle limits')
            torque = checked(self.packet.read1ByteTxRx(self.servo_id, 40))
            if torque not in (0, 1):
                raise IOError('Unsupported torque state; engineering review required')
            self.torque_enabled = torque == 1
            self.identity = dict(port=path, servo_id=self.servo_id, model=model,
                                 mode=mode, resolution=resolution, phase=phase,
                                 firmware_min=low, firmware_max=high)
            return self.read()
        except Exception:
            self.close()
            raise

    def read(self):
        data = checked(self.packet.readTxRx(self.servo_id, 56, 15))
        if len(data) != 15:
            raise IOError('Incomplete motor feedback')
        word = lambda i: data[i] | data[i + 1] << 8
        ticks = signed(word(0), 15)
        if not 0 <= ticks <= 4095:
            raise IOError('Encoder outside single-turn domain')
        return Feedback(ticks, signed(word(2), 15), signed(word(4), 10),
                        data[6] / 10., data[7], signed(word(13), 15), data[9],
                        bool(data[10]), time.monotonic())

    def write(self, ticks, calibration=False):
        c = self.config.validate(calibrated=True)
        if type(ticks) is not int or not c.min_ticks <= ticks <= c.max_ticks:
            raise ValueError('Target outside reviewed travel')
        checked(self.packet.WritePosEx(self.servo_id, ticks,
                min(c.speed, 50) if calibration else c.speed, min(c.acceleration, 5) if calibration else c.acceleration))
        if not self.torque_enabled:
            # Only after a deliberate bounded target, never on discovery/save.
            checked(self.packet.write1ByteTxRx(self.servo_id, 40, 1))
            self.torque_enabled = True

    def close(self):
        if self.port is not None and self.port.is_open:
            self.port.closePort()
        self.packet = None
