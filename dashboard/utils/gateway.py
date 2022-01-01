import serial 
import serial.tools.list_ports
from datetime import datetime
import os
import sys
import time
from threading import Thread
from collections import defaultdict
import json
import utils.rocket as protocol
from utils.definitions import *
from utils.data_handling import (TimeSeries, write_data_db)

LIVE = 0
BACKUP_TIMESTAMP = 1
FLASH_TIMESTAMP = 2

####
#a wrapper around the Serial class. It backups all communication to the ./data/ 
#directory. It can also find and open a connection to a device 
###
#init(device)
#   device can be, "gateway", "flight_controller", "RFD"
#
#write(*args) - copy of pyserial's write
#read(*args) - copy of pyserial's read
#open_serial() - open the serial connection
class SerialWrapper():
    def __init__(self, device):
        self.ser = serial.Serial(timeout=1)
        self.device = device
        #get file name
        now = datetime.now()
        time = now.strftime("%Y-%m-%d-%H-%M-%S")
        file_name = device + "-" + time
        #create file and directory
        try:
            dir_name = "data/" 
            os.mkdir(dir_name)
        except:
            pass
        try:
            self.backup = open(dir_name + file_name, "w+b")
        except:
            print("could not create the file: " + dir_name + file_name)
            sys.exit()

    #copy of the pyserial write with added backup
    def write(self, *args):
        self.backup.write(*args)
        self.ser.write(*args)

    #copy of the pyserial read with added backup
    def read(self, *args):
        buf = self.ser.read(*args)
        self.backup.write(buf)
        return buf

    #gets a list with all ports
    def get_safe_devices(self):
        safeStrings = [
            "usb",
            "arduino",
            "ch340"
        ]
        safe_devices = []
        devices = serial.tools.list_ports.comports()
        for d in devices:
            flag = False
            for substring in safeStrings:
                if substring in d.description.lower():
                    flag = True
            if flag:
                safe_devices.append(d)
        return safe_devices

    #returns true or false if a port contatins a device
    def init_device(self):
        if self.device == "rocket":
            msg = protocol.handshake_from_everyone_to_everyone()
            reply = protocol.handshake_from_everyone_to_everyone()
            handshake = bytes(SEPARATOR + [msg.get_id()]) 
            response = bytes(SEPARATOR + [reply.get_id()])
            print(handshake)
            self.ser.write(handshake)
            self.ser.read_all()
            time.sleep(0.4)
            buff = self.ser.read_all()
            print(buff)
            return response in buff
        else:
            return False
    
    #timestamp the backupfile
    def timestamp(self, time):
        time = int(time)
        msg = protocol.local_timestamp_from_local_to_local()
        msg.set_local_timestamp(time)
        buf = []
        buf += SEPARATOR
        buf += [msg.get_id()]
        buf += msg.build_buf()
        self.backup.write(bytes(buf))

    #start and open serial
    def open_serial(self):
        baudrates ={
            "rocket": 115200
        }
        self.ser.baudrate = baudrates[self.device]

        ports = self.get_safe_devices()
        for v in ports:
            self.ser.port = v.device
            self.ser.open()
            print(self.device + ": Testing " + str(v))
            if self.init_device():
                print(self.device + ": Succesfully connected")
                return True
            print(self.device + ": Did not respond")
            self.ser.close()
        else:
            print(self.device + ": opening serial failed")
            return False    

#####
#spawns a thread that reads and parses data from a stream.
###
#init(self, *args, influx = False, device = "None")
#   influx is the influx client to write to
#   device is the SerialWrapper device
#
#stop() - stops the reading thread
#pause() - pauses the reader thread
#resume() - resumes the reader thread
#redirect_data() - overwrite the data array to e.g. combine the output from two readers
#serial_is_active() - if the serial connection is open
#serial_is_active() - if a readable connection is open
#open_backup_file(path) - open a file created by this very program
#open_flash_file(path) - open a file dumped from the flash drive
#open_serial() - open a serial connection
#seconds_since_last_message() - get the time since the last message
class SerialReader():
    def __init__(self, *args, influx = False, device = "None"):
        self.influx = influx
        self.device = device
        #stream is for reading
        #serial is for writing
        self.serial_is_active = False
        self.stream_is_active = False
        self.ser = None
        self.stream = None
        self.client = False
        self.last_message_time = time.time()
        self.last_timestamp = 0
        self.time_sync_state = 0
        self.start_time = None
        self.pause = False
        self.exit = False
        self.decoders = {}
        #use defaultdict to let the front-end use uninitialized data
        self.data = defaultdict(lambda: defaultdict(lambda: defaultdict(TimeSeries)))

        t = Thread(target = self.reader_thread)
        t.start()

    #stops the thread
    def stop(self):
        self.exit = True

    #if the serial port is open
    def is_serial_open(self):
        return self.serial_is_active
    
    #if the is reading either from file or serial
    def is_stream_open(self):
        return self.stream_is_active

    #pause the reader thread
    def pause(self):
        self.paused = True
    
    #resume the reader thread
    def resume(self):
        self.paused = False    

    #tell the class to write into another data dict
    def redirect_data(self, data):
        self.data = data

    #opens a file dumped from the flash chip
    def open_flash_file(self, path):
        if self.is_serial_open():
            return
        self.time_sync_state = FLASH_TIMESTAMP
        self.start_time = None
        self.stream = open(path, "r+b")
        self.stream_is_active = True

    #opens the serial connection
    def open_serial(self):
        if self.is_serial_open():
            return
        self.time_sync_state = LIVE
        self.ser = SerialWrapper(self.device)
        result = self.ser.open_serial()
        if result:
            self.serial_is_active = True
            self.stream = self.ser
            self.stream_is_active = True
            return True
        return False

    #gets the seconds since the last message
    def seconds_since_last_message(self):
        return time.time() - self.last_message_time

    #get the current time using either the computer clock or the last timestamp
    def decide_on_time(self, name, value):
        self.last_message_time = time.time()
        if self.time_sync_state == LIVE:
            if self.start_time == None:
                    self.start_time = time.time()
            return time.time() - self.start_time

        if self.time_sync_state == FLASH_TIMESTAMP:
            if name == "ms_since_boot":
                self.last_timestamp = value
            if self.last_timestamp == None:
                return 0 #can't tell a proper time
            else:
                return self.last_timestamp / 1000 #convert to seconds

    def get_current_time(self):
        return self.decide_on_time("", 0)
    
    def reader_thread(self):
        while not self.exit:
            if not self.stream_is_active or self.pause:
                time.sleep(0.1) # Don't overload the computer
                continue
            separator = self.stream.read(1)
            #test for frame separator, read one byte at a time so it aligns itself
            if separator == b"":
                continue
            if separator[0] != SEPARATOR[0] or self.stream.read(1)[0] != SEPARATOR[1]:
                print(self.device + ": Invalid Separator: " + str(separator))
                continue
            frame_id = self.stream.read(1)[0]

            #we have two different protocol definitions so decide on which one ot use
            #try to get fc decoder
            fc_decoder = protocol.id_to_message_class(frame_id)
            if fc_decoder:
                self.read_fc_data(fc_decoder)
            else:
                print("invalid id: ", frame_id)
                

    def read_fc_data(self, decoder):
        length = decoder.get_size()
        buf = self.stream.read(length)
        if len(buf) != length:
            print("invalid length")
            return
        decoder.parse_buf(buf)
        decoded_data = decoder.get_all_data()
        source = decoder.get_sender()
        message = decoder.get_message()
        sensor_index = None
        if len(decoded_data) == 0:
            current_time = self.decide_on_time("", 0)
            self.data[source.name][message.name]["value"].x.append(current_time)
            self.data[source.name][message.name]["value"].y.append(1)
        for single_data in decoded_data:
            (field, value) = single_data
            name = field.name
            if name == "sensor_index":
                sensor_index = value
                continue
            #add the sensor index to the measurement so e.g. temperature becomes temperature2
            if sensor_index != None:
                name += str(sensor_index)
                sensor_index = None
            current_time = self.decide_on_time(name, value)
            print(source.name, message.name, name, value)
            self.data[source.name][message.name][name].x.append(current_time)
            self.data[source.name][message.name][name].y.append(value)

class Gateway(SerialReader):
    def __init__(self, **kwargs):
        super().__init__(device = "rocket", **kwargs)

    def _send_message(self, msg):
        if not self.serial_is_active:
            return
        buf = b"\n\r"
        buf += bytes([msg.get_id()])
        buf += msg.build_buf()
        self.ser.write(buf)

    def enter_sleep(self):
        msg = protocol.set_state_from_ground_to_rocket()
        msg.set_state(protocol.state.sleeping)
        self._send_message(msg)
        
    def get_ready(self):
        msg = protocol.set_state_from_ground_to_rocket()
        msg.set_state(protocol.state.ready)
        self._send_message(msg)

    
    def play_music(self):
        msg = protocol.play_music_from_ground_to_rocket()
        self._send_message(msg)
    
    def wipe_flash(self):
        msg = protocol.wipe_flash_from_ground_to_rocket()
        msg.set_this_to_42(42)
        self._send_message(msg)
    
    def dump_flash(self):
        self.pause()
        time.sleep(2)
        self.resume()
    
    def enable_logging(self):
        msg = protocol.set_logging_from_ground_to_rocket()
        msg.set_is_enabled(True)
        self._send_message(msg)

    def disable_logging(self):
        msg = protocol.set_logging_from_ground_to_rocket()
        msg.set_is_enabled(False)
        self._send_message(msg)

    def arm(self):
        msg = protocol.arm_pyro_from_ground_to_launchpad()
        msg.set_armed(True)
        self._send_message(msg)
    
    def disarm(self):
        msg = protocol.arm_pyro_from_ground_to_launchpad()
        msg.set_armed(False)
        self._send_message(msg)
    
    def enable_pyro1(self):
        msg = protocol.enable_pyro_from_ground_to_launchpad()
        msg.set_channel(1)
        self._send_message(msg)