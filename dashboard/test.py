import serial
from utils.definitions import *

def send_message(ser, msg):
        #the time doesn't matter, just skip it
        start = b"$" + bytes([len(msg) + 5]) + b"0000"
        combined_message = start + msg        
        checksum = 0
        for char in combined_message:
            checksum ^= char
        combined_message += bytes([checksum])
        ser.write(combined_message)
        print(combined_message)


ser = serial.Serial()
ser.baudrate = 115200
ser.port = "/dev/ttyACM3"
ser.open()

send_message(ser, )
while True:
    print(ser.read())
