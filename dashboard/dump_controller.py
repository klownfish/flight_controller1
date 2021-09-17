import serial.tools.list_ports
import serial
import sys
from utils.definitions import *

def get_safe_devices():
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

if not sys.argv[1]:
    print("please specify where to save")
    sys.exit()
try:
    f = open(sys.argv[1], "wb")
except:
    print("could not open file")

ser = serial.Serial(baudrate=BAUD, timeout=1)
devices = get_safe_devices()
for device in devices:
    ser.port = device.device
    ser.open()
    send_message(ser, bytes([IS_CONTROLLER]))
    if ser.read(1)[0] == 1:
        break

ser.read_all()
send_message(ser, bytes([CONTROLLER_DUMP_FLASH]))
print("dumping...")
length = 0
while True:
    byte = ser.read(1)
    if byte == b"":
        break
    length += 1
    print(byte)
    f.write(byte)

print(length)
print("done! dumped to ", sys.argv[1])
print("open the file with the dashboard to parse")