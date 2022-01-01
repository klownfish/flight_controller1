################################
#GENERATED FILE DO NOT EDIT
################################

from enum import Enum
import struct

def scaledFloat_to_uint(value, scale):
    return value * scale

def uint_to_scaledFloat(value, scale):
    return value / scale

def packedFloat_to_uint(value, minValue, maxValue, size):
    intMax = (1 << size * 8) - 1
    if(value < minValue):
      return 0
    if(value > maxValue):
      return intMax
    ratio = (value - minValue) / (maxValue - minValue)
    return 1 + ((intMax - 2)) * ratio
  
def uint_to_packedFloat(value, minValue, maxValue, size):
    intMax = (1 << size * 8) - 1
    if(value <= 0):
      return minValue - 1.0
    if(value >= intMax):
      return maxValue + 1.0
    ratio = (value - 1) / (intMax - 2)
    return ratio * (maxValue - minValue) + minValue

class state(Enum):
    debug = 0
    sleeping = 1
    ready = 2
    powered_flight = 3
    passive_flight = 4
    falling = 5
    landed = 6
class fix_type(Enum):
    none = 0
    fix2D = 1
    fix3D = 2
class nodes(Enum):
    everyone = 0
    rocket = 1
    ground = 2
    flash = 3
    launchpad = 4
class fields(Enum):
    ms_since_boot = 0
    flash_address = 1
    ax = 2
    ay = 3
    az = 4
    roll = 5
    pitch = 6
    yaw = 7
    altitude = 8
    satellites = 9
    voltage = 10
    state = 11
    rssi = 12
    longitude = 13
    latitude = 14
    pressure = 15
    temperature = 16
    gx = 17
    gy = 18
    gz = 19
    mx = 20
    my = 21
    mz = 22
    angle_x = 23
    angle_y = 24
    pdop = 25
    n_satellites = 26
    fix_type = 27
    armed = 28
    channel = 29
    this_to_42 = 30
    is_enabled = 31
class messages(Enum):
    handshake = 0
    telemetry = 1
    position = 2
    bmp = 3
    mpu = 4
    bmi_accel = 5
    bmi_gyro = 6
    tvc_angle = 7
    battery_voltage = 8
    state = 9
    rssi = 10
    gps = 11
    ms_since_boot = 12
    arm_pyro = 13
    enable_pyro = 14
    wipe_flash = 15
    play_music = 16
    set_logging = 17
    dump_flash = 18
    set_state = 19
class categories(Enum):
    none = 0
class handshake_from_everyone_to_everyone:
    def __init__(self):
        self._sender = nodes.everyone
        self._receiver = nodes.everyone
        self._message = messages.handshake
        self._category = categories.none
        self._id = 0
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class telemetry_from_rocket_to_ground:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.ground
        self._message = messages.telemetry
        self._category = categories.none
        self._id = 1
        self._size = 46
        self._ms_since_boot = 0
        self._flash_address = 0
        self._ax = 0
        self._ay = 0
        self._az = 0
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        self._altitude = 0
        self._satellites = 0
        self._voltage = 0
        self._state = 0
        self._rssi = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_ms_since_boot(self, value):
        self._ms_since_boot = value
    def set_flash_address(self, value):
        self._flash_address = value
    def set_ax(self, value):
        self._ax = value
    def set_ay(self, value):
        self._ay = value
    def set_az(self, value):
        self._az = value
    def set_roll(self, value):
        self._roll = value
    def set_pitch(self, value):
        self._pitch = value
    def set_yaw(self, value):
        self._yaw = value
    def set_altitude(self, value):
        self._altitude = value
    def set_satellites(self, value):
        self._satellites = value
    def set_voltage(self, value):
        self._voltage = value
    def set_state(self, value):
        self._state = value.value
    def set_rssi(self, value):
        self._rssi = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<L", self._ms_since_boot)
        buf += struct.pack("<L", self._flash_address)
        buf += struct.pack("<f", self._ax)
        buf += struct.pack("<f", self._ay)
        buf += struct.pack("<f", self._az)
        buf += struct.pack("<f", self._roll)
        buf += struct.pack("<f", self._pitch)
        buf += struct.pack("<f", self._yaw)
        buf += struct.pack("<f", self._altitude)
        buf += struct.pack("<B", self._satellites)
        buf += struct.pack("<f", self._voltage)
        buf += struct.pack("<B", self._state)
        buf += struct.pack("<f", self._rssi)
        return buf
    def get_ms_since_boot(self):
        return self._ms_since_boot
    def get_flash_address(self):
        return self._flash_address
    def get_ax(self):
        return self._ax
    def get_ay(self):
        return self._ay
    def get_az(self):
        return self._az
    def get_roll(self):
        return self._roll
    def get_pitch(self):
        return self._pitch
    def get_yaw(self):
        return self._yaw
    def get_altitude(self):
        return self._altitude
    def get_satellites(self):
        return self._satellites
    def get_voltage(self):
        return self._voltage
    def get_state(self):
        return state(self._state)
    def get_rssi(self):
        return self._rssi
    def get_all_data(self):
        data = []
        data.append((fields.ms_since_boot, self.get_ms_since_boot()))
        data.append((fields.flash_address, self.get_flash_address()))
        data.append((fields.ax, self.get_ax()))
        data.append((fields.ay, self.get_ay()))
        data.append((fields.az, self.get_az()))
        data.append((fields.roll, self.get_roll()))
        data.append((fields.pitch, self.get_pitch()))
        data.append((fields.yaw, self.get_yaw()))
        data.append((fields.altitude, self.get_altitude()))
        data.append((fields.satellites, self.get_satellites()))
        data.append((fields.voltage, self.get_voltage()))
        data.append((fields.state, self.get_state()))
        data.append((fields.rssi, self.get_rssi()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._ms_since_boot = struct.unpack_from("<L", buf, index)[0]
        index += 4
        self._flash_address = struct.unpack_from("<L", buf, index)[0]
        index += 4
        self._ax = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._ay = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._az = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._roll = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._pitch = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._yaw = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._altitude = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._satellites = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._voltage = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._state = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._rssi = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class position_from_rocket_to_ground:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.ground
        self._message = messages.position
        self._category = categories.none
        self._id = 2
        self._size = 8
        self._longitude = 0
        self._latitude = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_longitude(self, value):
        self._longitude = value
    def set_latitude(self, value):
        self._latitude = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._longitude)
        buf += struct.pack("<f", self._latitude)
        return buf
    def get_longitude(self):
        return self._longitude
    def get_latitude(self):
        return self._latitude
    def get_all_data(self):
        data = []
        data.append((fields.longitude, self.get_longitude()))
        data.append((fields.latitude, self.get_latitude()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._longitude = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._latitude = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class bmp_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.bmp
        self._category = categories.none
        self._id = 3
        self._size = 8
        self._pressure = 0
        self._temperature = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_pressure(self, value):
        self._pressure = value
    def set_temperature(self, value):
        self._temperature = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._pressure)
        buf += struct.pack("<f", self._temperature)
        return buf
    def get_pressure(self):
        return self._pressure
    def get_temperature(self):
        return self._temperature
    def get_all_data(self):
        data = []
        data.append((fields.pressure, self.get_pressure()))
        data.append((fields.temperature, self.get_temperature()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._pressure = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._temperature = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class mpu_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.mpu
        self._category = categories.none
        self._id = 4
        self._size = 36
        self._ax = 0
        self._ay = 0
        self._az = 0
        self._gx = 0
        self._gy = 0
        self._gz = 0
        self._mx = 0
        self._my = 0
        self._mz = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_ax(self, value):
        self._ax = value
    def set_ay(self, value):
        self._ay = value
    def set_az(self, value):
        self._az = value
    def set_gx(self, value):
        self._gx = value
    def set_gy(self, value):
        self._gy = value
    def set_gz(self, value):
        self._gz = value
    def set_mx(self, value):
        self._mx = value
    def set_my(self, value):
        self._my = value
    def set_mz(self, value):
        self._mz = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._ax)
        buf += struct.pack("<f", self._ay)
        buf += struct.pack("<f", self._az)
        buf += struct.pack("<f", self._gx)
        buf += struct.pack("<f", self._gy)
        buf += struct.pack("<f", self._gz)
        buf += struct.pack("<f", self._mx)
        buf += struct.pack("<f", self._my)
        buf += struct.pack("<f", self._mz)
        return buf
    def get_ax(self):
        return self._ax
    def get_ay(self):
        return self._ay
    def get_az(self):
        return self._az
    def get_gx(self):
        return self._gx
    def get_gy(self):
        return self._gy
    def get_gz(self):
        return self._gz
    def get_mx(self):
        return self._mx
    def get_my(self):
        return self._my
    def get_mz(self):
        return self._mz
    def get_all_data(self):
        data = []
        data.append((fields.ax, self.get_ax()))
        data.append((fields.ay, self.get_ay()))
        data.append((fields.az, self.get_az()))
        data.append((fields.gx, self.get_gx()))
        data.append((fields.gy, self.get_gy()))
        data.append((fields.gz, self.get_gz()))
        data.append((fields.mx, self.get_mx()))
        data.append((fields.my, self.get_my()))
        data.append((fields.mz, self.get_mz()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._ax = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._ay = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._az = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._gx = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._gy = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._gz = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._mx = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._my = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._mz = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class bmi_accel_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.bmi_accel
        self._category = categories.none
        self._id = 5
        self._size = 12
        self._ax = 0
        self._ay = 0
        self._az = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_ax(self, value):
        self._ax = value
    def set_ay(self, value):
        self._ay = value
    def set_az(self, value):
        self._az = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._ax)
        buf += struct.pack("<f", self._ay)
        buf += struct.pack("<f", self._az)
        return buf
    def get_ax(self):
        return self._ax
    def get_ay(self):
        return self._ay
    def get_az(self):
        return self._az
    def get_all_data(self):
        data = []
        data.append((fields.ax, self.get_ax()))
        data.append((fields.ay, self.get_ay()))
        data.append((fields.az, self.get_az()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._ax = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._ay = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._az = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class bmi_gyro_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.bmi_gyro
        self._category = categories.none
        self._id = 6
        self._size = 12
        self._gx = 0
        self._gy = 0
        self._gz = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_gx(self, value):
        self._gx = value
    def set_gy(self, value):
        self._gy = value
    def set_gz(self, value):
        self._gz = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._gx)
        buf += struct.pack("<f", self._gy)
        buf += struct.pack("<f", self._gz)
        return buf
    def get_gx(self):
        return self._gx
    def get_gy(self):
        return self._gy
    def get_gz(self):
        return self._gz
    def get_all_data(self):
        data = []
        data.append((fields.gx, self.get_gx()))
        data.append((fields.gy, self.get_gy()))
        data.append((fields.gz, self.get_gz()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._gx = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._gy = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._gz = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class tvc_angle_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.tvc_angle
        self._category = categories.none
        self._id = 7
        self._size = 8
        self._angle_x = 0
        self._angle_y = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_angle_x(self, value):
        self._angle_x = value
    def set_angle_y(self, value):
        self._angle_y = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._angle_x)
        buf += struct.pack("<f", self._angle_y)
        return buf
    def get_angle_x(self):
        return self._angle_x
    def get_angle_y(self):
        return self._angle_y
    def get_all_data(self):
        data = []
        data.append((fields.angle_x, self.get_angle_x()))
        data.append((fields.angle_y, self.get_angle_y()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._angle_x = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._angle_y = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class battery_voltage_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.battery_voltage
        self._category = categories.none
        self._id = 8
        self._size = 4
        self._voltage = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_voltage(self, value):
        self._voltage = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<f", self._voltage)
        return buf
    def get_voltage(self):
        return self._voltage
    def get_all_data(self):
        data = []
        data.append((fields.voltage, self.get_voltage()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._voltage = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class state_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.state
        self._category = categories.none
        self._id = 9
        self._size = 1
        self._state = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_state(self, value):
        self._state = value.value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._state)
        return buf
    def get_state(self):
        return state(self._state)
    def get_all_data(self):
        data = []
        data.append((fields.state, self.get_state()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._state = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class rssi_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.rssi
        self._category = categories.none
        self._id = 10
        self._size = 2
        self._rssi = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_rssi(self, value):
        self._rssi = scaledFloat_to_uint(value, 100)
    def build_buf(self):
        buf = b""
        buf += struct.pack("<h", self._rssi)
        return buf
    def get_rssi(self):
        return uint_to_scaledFloat(self._rssi, 100)
    def get_all_data(self):
        data = []
        data.append((fields.rssi, self.get_rssi()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._rssi = struct.unpack_from("<h", buf, index)[0]
        index += 2
        return
class gps_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.gps
        self._category = categories.none
        self._id = 11
        self._size = 16
        self._pdop = 0
        self._n_satellites = 0
        self._fix_type = 0
        self._altitude = 0
        self._latitude = 0
        self._longitude = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_pdop(self, value):
        self._pdop = scaledFloat_to_uint(value, 100)
    def set_n_satellites(self, value):
        self._n_satellites = value
    def set_fix_type(self, value):
        self._fix_type = value.value
    def set_altitude(self, value):
        self._altitude = value
    def set_latitude(self, value):
        self._latitude = value
    def set_longitude(self, value):
        self._longitude = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<H", self._pdop)
        buf += struct.pack("<B", self._n_satellites)
        buf += struct.pack("<B", self._fix_type)
        buf += struct.pack("<f", self._altitude)
        buf += struct.pack("<f", self._latitude)
        buf += struct.pack("<f", self._longitude)
        return buf
    def get_pdop(self):
        return uint_to_scaledFloat(self._pdop, 100)
    def get_n_satellites(self):
        return self._n_satellites
    def get_fix_type(self):
        return fix_type(self._fix_type)
    def get_altitude(self):
        return self._altitude
    def get_latitude(self):
        return self._latitude
    def get_longitude(self):
        return self._longitude
    def get_all_data(self):
        data = []
        data.append((fields.pdop, self.get_pdop()))
        data.append((fields.n_satellites, self.get_n_satellites()))
        data.append((fields.fix_type, self.get_fix_type()))
        data.append((fields.altitude, self.get_altitude()))
        data.append((fields.latitude, self.get_latitude()))
        data.append((fields.longitude, self.get_longitude()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._pdop = struct.unpack_from("<H", buf, index)[0]
        index += 2
        self._n_satellites = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._fix_type = struct.unpack_from("<B", buf, index)[0]
        index += 1
        self._altitude = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._latitude = struct.unpack_from("<f", buf, index)[0]
        index += 4
        self._longitude = struct.unpack_from("<f", buf, index)[0]
        index += 4
        return
class ms_since_boot_from_rocket_to_flash:
    def __init__(self):
        self._sender = nodes.rocket
        self._receiver = nodes.flash
        self._message = messages.ms_since_boot
        self._category = categories.none
        self._id = 12
        self._size = 4
        self._ms_since_boot = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_ms_since_boot(self, value):
        self._ms_since_boot = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<L", self._ms_since_boot)
        return buf
    def get_ms_since_boot(self):
        return self._ms_since_boot
    def get_all_data(self):
        data = []
        data.append((fields.ms_since_boot, self.get_ms_since_boot()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._ms_since_boot = struct.unpack_from("<L", buf, index)[0]
        index += 4
        return
class arm_pyro_from_ground_to_launchpad:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.launchpad
        self._message = messages.arm_pyro
        self._category = categories.none
        self._id = 13
        self._size = 1
        self._bit_field = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_armed(self, value):
        self._bit_field =  value * (self._bit_field | (1 << 0)) + (not value) * (self._bit_field & ~(1 << 0))
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._bit_field)
        return buf
    def get_armed(self):
        return self._bit_field & (1 << 0)
    def get_all_data(self):
        data = []
        data.append((fields.armed, self.get_armed()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._bit_field = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class enable_pyro_from_ground_to_launchpad:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.launchpad
        self._message = messages.enable_pyro
        self._category = categories.none
        self._id = 14
        self._size = 1
        self._channel = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_channel(self, value):
        self._channel = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._channel)
        return buf
    def get_channel(self):
        return self._channel
    def get_all_data(self):
        data = []
        data.append((fields.channel, self.get_channel()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._channel = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class wipe_flash_from_ground_to_rocket:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.rocket
        self._message = messages.wipe_flash
        self._category = categories.none
        self._id = 15
        self._size = 1
        self._this_to_42 = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_this_to_42(self, value):
        self._this_to_42 = value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._this_to_42)
        return buf
    def get_this_to_42(self):
        return self._this_to_42
    def get_all_data(self):
        data = []
        data.append((fields.this_to_42, self.get_this_to_42()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._this_to_42 = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class play_music_from_ground_to_rocket:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.rocket
        self._message = messages.play_music
        self._category = categories.none
        self._id = 16
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class set_logging_from_ground_to_rocket:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.rocket
        self._message = messages.set_logging
        self._category = categories.none
        self._id = 17
        self._size = 1
        self._bit_field = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_is_enabled(self, value):
        self._bit_field =  value * (self._bit_field | (1 << 0)) + (not value) * (self._bit_field & ~(1 << 0))
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._bit_field)
        return buf
    def get_is_enabled(self):
        return self._bit_field & (1 << 0)
    def get_all_data(self):
        data = []
        data.append((fields.is_enabled, self.get_is_enabled()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._bit_field = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
class dump_flash_from_ground_to_rocket:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.rocket
        self._message = messages.dump_flash
        self._category = categories.none
        self._id = 18
        self._size = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def build_buf(self):
        buf = b""
        return buf
    def get_all_data(self):
        data = []
        return data
    def parse_buf(self, buf):
        index = 0
        return
class set_state_from_ground_to_rocket:
    def __init__(self):
        self._sender = nodes.ground
        self._receiver = nodes.rocket
        self._message = messages.set_state
        self._category = categories.none
        self._id = 19
        self._size = 1
        self._state = 0
    def get_sender(self):
        return self._sender
    def get_receiver(self):
        return self._receiver
    def get_message(self):
        return self._message
    def get_id(self):
        return self._id
    def get_size(self):
        return self._size
    def get_category(self):
        return self._category
    def set_state(self, value):
        self._state = value.value
    def build_buf(self):
        buf = b""
        buf += struct.pack("<B", self._state)
        return buf
    def get_state(self):
        return state(self._state)
    def get_all_data(self):
        data = []
        data.append((fields.state, self.get_state()))
        return data
    def parse_buf(self, buf):
        index = 0
        self._state = struct.unpack_from("<B", buf, index)[0]
        index += 1
        return
def id_to_message_class(id):
    if id == 0:
        receiver = handshake_from_everyone_to_everyone()
        return receiver
    if id == 1:
        receiver = telemetry_from_rocket_to_ground()
        return receiver
    if id == 2:
        receiver = position_from_rocket_to_ground()
        return receiver
    if id == 3:
        receiver = bmp_from_rocket_to_flash()
        return receiver
    if id == 4:
        receiver = mpu_from_rocket_to_flash()
        return receiver
    if id == 5:
        receiver = bmi_accel_from_rocket_to_flash()
        return receiver
    if id == 6:
        receiver = bmi_gyro_from_rocket_to_flash()
        return receiver
    if id == 7:
        receiver = tvc_angle_from_rocket_to_flash()
        return receiver
    if id == 8:
        receiver = battery_voltage_from_rocket_to_flash()
        return receiver
    if id == 9:
        receiver = state_from_rocket_to_flash()
        return receiver
    if id == 10:
        receiver = rssi_from_rocket_to_flash()
        return receiver
    if id == 11:
        receiver = gps_from_rocket_to_flash()
        return receiver
    if id == 12:
        receiver = ms_since_boot_from_rocket_to_flash()
        return receiver
    if id == 13:
        receiver = arm_pyro_from_ground_to_launchpad()
        return receiver
    if id == 14:
        receiver = enable_pyro_from_ground_to_launchpad()
        return receiver
    if id == 15:
        receiver = wipe_flash_from_ground_to_rocket()
        return receiver
    if id == 16:
        receiver = play_music_from_ground_to_rocket()
        return receiver
    if id == 17:
        receiver = set_logging_from_ground_to_rocket()
        return receiver
    if id == 18:
        receiver = dump_flash_from_ground_to_rocket()
        return receiver
    if id == 19:
        receiver = set_state_from_ground_to_rocket()
        return receiver
def is_specifier(sender, name, field):
    if (messages.telemetry == name and nodes.rocket == sender):
        if (fields.ms_since_boot == field):
            return False
        if (fields.flash_address == field):
            return False
        if (fields.ax == field):
            return False
        if (fields.ay == field):
            return False
        if (fields.az == field):
            return False
        if (fields.roll == field):
            return False
        if (fields.pitch == field):
            return False
        if (fields.yaw == field):
            return False
        if (fields.altitude == field):
            return False
        if (fields.satellites == field):
            return False
        if (fields.voltage == field):
            return False
        if (fields.state == field):
            return False
        if (fields.rssi == field):
            return False
    if (messages.position == name and nodes.rocket == sender):
        if (fields.longitude == field):
            return False
        if (fields.latitude == field):
            return False
    if (messages.bmp == name and nodes.rocket == sender):
        if (fields.pressure == field):
            return False
        if (fields.temperature == field):
            return False
    if (messages.mpu == name and nodes.rocket == sender):
        if (fields.ax == field):
            return False
        if (fields.ay == field):
            return False
        if (fields.az == field):
            return False
        if (fields.gx == field):
            return False
        if (fields.gy == field):
            return False
        if (fields.gz == field):
            return False
        if (fields.mx == field):
            return False
        if (fields.my == field):
            return False
        if (fields.mz == field):
            return False
    if (messages.bmi_accel == name and nodes.rocket == sender):
        if (fields.ax == field):
            return False
        if (fields.ay == field):
            return False
        if (fields.az == field):
            return False
    if (messages.bmi_gyro == name and nodes.rocket == sender):
        if (fields.gx == field):
            return False
        if (fields.gy == field):
            return False
        if (fields.gz == field):
            return False
    if (messages.tvc_angle == name and nodes.rocket == sender):
        if (fields.angle_x == field):
            return False
        if (fields.angle_y == field):
            return False
    if (messages.battery_voltage == name and nodes.rocket == sender):
        if (fields.voltage == field):
            return False
    if (messages.state == name and nodes.rocket == sender):
        if (fields.state == field):
            return False
    if (messages.rssi == name and nodes.rocket == sender):
        if (fields.rssi == field):
            return False
    if (messages.gps == name and nodes.rocket == sender):
        if (fields.pdop == field):
            return False
        if (fields.n_satellites == field):
            return False
        if (fields.fix_type == field):
            return False
        if (fields.altitude == field):
            return False
        if (fields.latitude == field):
            return False
        if (fields.longitude == field):
            return False
    if (messages.ms_since_boot == name and nodes.rocket == sender):
        if (fields.ms_since_boot == field):
            return False
    if (messages.enable_pyro == name and nodes.ground == sender):
        if (fields.channel == field):
            return False
    if (messages.wipe_flash == name and nodes.ground == sender):
        if (fields.this_to_42 == field):
            return False
    if (messages.set_state == name and nodes.ground == sender):
        if (fields.state == field):
            return False
    return False
def is_extended_id(id):
    return
