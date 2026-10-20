"""Single-owner ST-series SDK adapter. No EEPROM or mode writes; torque enable only on authorized motion."""
from dataclasses import dataclass
from glob import glob
import time
from .serial_diagnostics import Diagnostics, TransportError
from .faults import DeviceConfigurationFault, GripperFault, MechanicalFault


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
    raw_bytes: tuple = ()
    transaction_ms: float = 0.


def ports():
    return sorted(glob('/dev/serial/by-id/*'))


def friendly_error(error):
    """Turn transport errors into actionable operator diagnostics."""
    text = str(error)
    lower = text.lower()
    if isinstance(error, ImportError) or 'scservo_sdk' in lower and 'no module' in lower:
        return ('ST3215 SDK missing for /usr/bin/python3; run '
                'ur_teleop/scripts/install_gripper_sdk.py')
    if 'permission denied' in lower or 'errno 13' in lower:
        return ('Permission denied opening the serial adapter; check dialout group/udev '
                'policy and log in again (do not relax permissions globally)')
    if 'busy' in lower or 'errno 16' in lower:
        return 'Serial port already in use by another process; stop the other owner first'
    if 'no such file' in lower or 'errno 2' in lower or 'not found' in lower:
        return 'USB adapter not found at the configured path; check USB cable and /dev/serial/by-id'
    return text


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
            try:
                import scservo_sdk as sdk
            except ImportError as error:
                raise IOError(friendly_error(error)) from error
        self.config, self.sdk = config, sdk
        self.port = self.packet = None
        self.servo_id = None
        self.identity = {}
        self.torque_enabled = False
        self.diagnostics = Diagnostics()
        self.received_at = None

    def connect(self, cancelled=lambda: False):
        paths = ports()
        path = self.config.port
        if not path:
            if len(paths) != 1:
                raise IOError('USB adapter not found' if not paths else
                              'Multiple USB serial adapters found; select one /dev/serial/by-id path in Gripper Setup')
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

            def readPort(self, length):
                # The SDK checks timeouts only on some parser branches. Bound
                # noise/wrong-ID streams too, including a late valid packet.
                if self.getTimeSinceStart() > self.packet_timeout:
                    raise TransportError("rx_timeouts", "ST3215 communication=-6; receive deadline", -6)
                return super().readPort(length)

            def setPacketTimeout(self, length):
                self.setPacketTimeoutMillis(20.)

        self.port = BoundedPort(path)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(1000000):
                raise IOError('Cannot open adapter at 1000000 baud')
            class StrictPacket(self.sdk.sms_sts):
                def txRxPacket(packet_self, txpacket):
                    response, result, error = super().txRxPacket(txpacket)
                    if result == 0:
                        port = packet_self.portHandler
                        if port.getTimeSinceStart() > port.packet_timeout:
                            raise TransportError('rx_timeouts', 'ST3215 communication=-6; late response', -6)
                        expected = txpacket[6] + 6 if txpacket[4] == 2 else 6
                        if (response is None or len(response) != expected or
                                response[3] != expected - 4 or response[2] != txpacket[2]):
                            raise TransportError('invalid_responses', 'Invalid motor response length/ID')
                    return response, result, error
            self.packet = StrictPacket(self.port)
            ids = []
            candidates = range(254) if self.config.servo_id == -1 else [self.config.servo_id]
            for servo_id in candidates:
                if cancelled():
                    raise IOError('Discovery cancelled')
                try:
                    model = self.transaction('ping', lambda: self.packet.ping(servo_id))
                except TransportError as exc:
                    if (self.config.servo_id == -1 and exc.motor_error == 0 and
                            exc.communication in (-6, -7)):
                        continue
                    raise
                ids.append((servo_id, model))
            if len(ids) != 1:
                if not ids:
                    raise IOError('No ST3215 servo responded; check motor power, baud rate and servo ID')
                raise IOError('Multiple servo IDs discovered: '
                              + ', '.join(str(i[0]) for i in ids)
                              + '; configure the intended ID in Gripper Setup')
            self.servo_id, model = ids[0]
            mode = self.transaction('configuration', lambda: self.packet.read1ByteTxRx(self.servo_id, 33))
            resolution = self.transaction('configuration', lambda: self.packet.read1ByteTxRx(self.servo_id, 30))
            low = self.transaction('configuration', lambda: self.packet.read2ByteTxRx(self.servo_id, 9))
            high = self.transaction('configuration', lambda: self.packet.read2ByteTxRx(self.servo_id, 11))
            phase = self.transaction('configuration', lambda: self.packet.read1ByteTxRx(self.servo_id, 18))
            if mode != 0 or resolution != 1 or not 0 <= low < high <= 4095:
                raise DeviceConfigurationFault(
                    'Requires reviewed single-turn mode 0, resolution 1, nonzero angle range'
                    f' (detected mode={mode}, resolution={resolution}, range={low}..{high})')
            if self.config.enabled and not low <= self.config.min_ticks <= self.config.max_ticks <= high:
                raise DeviceConfigurationFault('Calibration exceeds servo angle limits')
            torque = self.transaction('configuration', lambda: self.packet.read1ByteTxRx(self.servo_id, 40))
            if torque not in (0, 1):
                raise DeviceConfigurationFault('Unsupported torque state; engineering review required')
            self.torque_enabled = torque == 1
            self.identity = dict(port=path, servo_id=self.servo_id, model=model,
                                 mode=mode, resolution=resolution, phase=phase,
                                 firmware_min=low, firmware_max=high)
            return self.read()
        except Exception as error:
            self.close()
            if isinstance(error, (GripperFault, IOError)) and str(error):
                raise
            raise IOError(friendly_error(error)) from error

    def transaction(self, kind, call, length=None):
        started = time.monotonic()
        communication = motor_error = None
        category = None
        try:
            result = call()
            self.received_at = time.monotonic()
            if not isinstance(result, (tuple, list)) or len(result) < 2:
                raise TransportError('invalid_responses', 'Invalid SDK result')
            *values, communication, motor_error = result
            if type(communication) is not int or type(motor_error) is not int:
                raise TransportError('invalid_responses', 'Invalid SDK result codes')
            if motor_error:
                category = 'motor_errors'
            elif communication:
                category = {-6: 'rx_timeouts', -7: 'rx_corruption'}.get(communication, 'other_errors')
            if category:
                raise TransportError(category,
                    f'ST3215 communication={communication}, motor_error=0x{motor_error:02x}',
                    communication, motor_error)
            data = values[0] if len(values) == 1 else values
            if length is not None and (not isinstance(data, (list, tuple, bytes)) or
                    len(data) != length or any(type(v) is not int or not 0 <= v <= 255 for v in data)):
                raise TransportError('invalid_responses', 'Incomplete/invalid motor feedback')
            return data
        except Exception as exc:
            category = getattr(exc, 'category', 'other_errors')
            communication = getattr(exc, 'communication', communication)
            motor_error = getattr(exc, 'motor_error', motor_error)
            raise
        finally:
            # SDK exceptions can bypass rxPacket's ownership-flag cleanup.
            # This adapter is the sole serial caller, including ID discovery.
            if self.port is not None:
                self.port.is_using = False
            self.diagnostics.transaction(kind, time.monotonic()-started,
                                         communication, motor_error, category)

    def read_temperature(self):
        # readTxRx retains the length information discarded by read1ByteTxRx.
        data = self.transaction('temperature', lambda: self.packet.readTxRx(self.servo_id, 63, 1), 1)
        return data[0], self.received_at

    def read(self):
        data = self.transaction('feedback', lambda: self.packet.readTxRx(self.servo_id, 56, 15), 15)
        word = lambda i: data[i] | data[i + 1] << 8
        ticks = signed(word(0), 15)
        if not 0 <= ticks <= 4095:
            self.diagnostics.counters['invalid_responses'] += 1
            raise TransportError('invalid_responses', 'Encoder outside single-turn domain')
        if data[9]:
            self.diagnostics.counters['motor_errors'] += 1
        return Feedback(ticks, signed(word(2), 15), signed(word(4), 10),
                        data[6] / 10., data[7], signed(word(13), 15), data[9],
                        bool(data[10]), self.received_at, tuple(data),
                        self.diagnostics.last['duration_ms'])

    def write(self, ticks, calibration=False):
        c = self.config.validate(calibrated=True)
        if type(ticks) is not int or not c.min_ticks <= ticks <= c.max_ticks:
            raise MechanicalFault('Target outside reviewed travel')
        self.transaction("write", lambda: self.packet.WritePosEx(self.servo_id, ticks,
                min(c.speed, 50) if calibration else c.speed, min(c.acceleration, 5) if calibration else c.acceleration))
        if not self.torque_enabled:
            # Only after a deliberate bounded target, never on discovery/save.
            self.transaction("torque", lambda: self.packet.write1ByteTxRx(self.servo_id, 40, 1))
            self.torque_enabled = True

    def close(self):
        if self.port is not None and self.port.is_open:
            self.port.closePort()
        self.packet = None
