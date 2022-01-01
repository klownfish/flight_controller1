import tkinter as tk
from utils.gateway import Gateway
from utils.data_handling import init_db
from utils.widgets import *

def main():
    client = init_db(True)
    gw = Gateway(influx = client)

    root = tk.Tk()
    padding = {"padx": 5, "pady": 5}
    settings = tk.Frame(root)

    open_serial = tk.Button(settings, text="open serial", command = gw.open_serial, **padding)
    open_flash = ButtonFile(settings, text="open file", command = gw.open_flash_file, **padding)
    
    sleep = tk.Button(settings, text="sleep", command = gw.enter_sleep, **padding)
    get_ready = tk.Button(settings, text="get ready!", command = gw.get_ready, **padding)
    state = EnumLastValue(settings, "state: ", gw.data["rocket"]["telemetry"]["state"])
    voltage = TextLastValue(settings, "voltage: ", gw.data["rocket"]["telemetry"]["voltage"])
    rssi = TextLastValue(settings, "rssi rocket: ", gw.data["rocket"]["telemetry"]["rssi"])
    rssi_relay = TextLastValue(settings, "rssi relay: ", gw.data["relay"]["rssi"]["rssi"])
    rssi_launchpad = TextLastValue(settings, "rssi launchpad: ", gw.data["launchpad"]["rssi"]["rssi"])
    wipe_flash = tk.Button(settings, text = "wipe flash", command= gw.wipe_flash, **padding)
    flash_index = FlashUsed(settings, gw, **padding)
    enable_logging = tk.Button(settings, text = "enable logging", command= gw.enable_logging, **padding)
    disable_logging = tk.Button(settings, text = "disable logging", command= gw.disable_logging, **padding)
    play_music = tk.Button(settings, text = "play music", command= gw.play_music, **padding)
    arm = tk.Button(settings, text = "arm", command= gw.arm, **padding)
    disarm = tk.Button(settings, text = "disarm", command= gw.disarm, **padding)
    pyro1 = tk.Button(settings, text = "enable pyro", command= gw.enable_pyro1, **padding)
    n_satellites = TextLastValue(settings, "satellites: ", gw.data["rocket"]["telemetry"]["satellites"])
    latitude = TextLastValue(settings, "latitude: ", gw.data["rocket"]["position"]["latitude"])
    longitude = TextLastValue(settings, "longitude: ", gw.data["rocket"]["position"]["longitude"])
    altitude_text = TextLastValue(settings, "altitude: ", gw.data["rocket"]["telemetry"]["altitude"])

    altitude = AltitudeGraph(root, gw)
    gyro = GyroGraph(root, gw)
    acceleration = AccelerationGraph(root, gw)
    
    index = 0

    open_serial.grid(column = 0, row = index)
    index += 1

    open_flash.grid(column = 0, row = index)
    index += 1

    sleep.grid(column = 0, row = index)
    index += 1

    get_ready.grid(column = 0, row = index)
    index += 1

    wipe_flash.grid(column = 0, row = index)
    index += 1

    enable_logging.grid(column = 0, row = index)
    index += 1
    
    disable_logging.grid(column = 0, row = index)
    index += 1

    play_music.grid(column= 0, row = index)
    index += 1


    arm.grid(column = 0, row = index)
    index += 1

    disarm.grid(column = 0, row = index)
    index += 1

    pyro1.grid(column = 0, row = index)
    index += 1

    tindex = 0
    state.grid(column = 1, row = tindex)
    tindex += 1

    voltage.grid(column = 1, row = tindex)
    tindex += 1

    rssi.grid(column = 1, row = tindex)
    tindex += 1

    rssi_relay.grid(column = 1, row = tindex)
    tindex += 1
    
    flash_index.grid(column = 1, row = tindex)
    tindex += 1

    n_satellites.grid(column = 1, row = tindex)
    tindex += 1

    latitude.grid(column = 1, row = tindex)
    tindex += 1

    longitude.grid(column = 1, row = tindex)
    tindex += 1

    altitude_text.grid(column = 1, row = tindex)
    tindex += 1
    
    altitude.widget.grid(column = 0, row = 0, **padding)
    gyro.widget.grid(column = 0, row = 1, **padding)
    acceleration.widget.grid(column = 1, row = 0, **padding)
    settings.grid(column = 1,row = 1)
    def on_close():
        gw.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

main()