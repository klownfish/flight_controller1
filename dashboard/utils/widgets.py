import tkinter as tk
from tkinter.filedialog import askopenfilename
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from time import time

REFRESH = 50

VIEWRANGE = 30
##################
#Class to display TimeSeries
##################
#
#__init__(root, database, datalists)
#root - TKinter root window
#clock - the clock that will be used
#time_serise - a list with TimeSeries to display
class GenericGraph():
    def __init__(self, root, clock, time_series, width = 9, height = 5):
        self.clock = clock
        self.time_series = time_series
        self.fig = plt.Figure(figsize=(width, height), dpi=100, tight_layout=True)
        self.ax = self.fig.add_subplot(111)    
        self.lines = []
        #create a line for every series
        for _ in time_series:
            line, = self.ax.plot([], [])
            self.lines.append(line)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.widget = self.canvas.get_tk_widget()
        self.ani = animation.FuncAnimation(
            self.fig, self.update, interval=REFRESH)
    
    def update(self, _): 
        relative_time = self.clock()
        self.ax.set_xlim(
            relative_time - VIEWRANGE, relative_time)
        for i in range(len(self.time_series)):
            self.lines[i].set_data(self.time_series[i].x, self.time_series[i].y)

        return self.lines,

###################
#class to display the latest value from a TimeSeries
###################
#
#__init__(self, root, text, value)
#root - tkinter root
#text - displayed before the value
#value - the TimeSeries that the value will be taken from
class TextLastValue(tk.Label):
    def __init__(self, root, text, value, **kwargs):
        self.text = text
        self.stringVar = tk.StringVar()
        self.stringVar.set(text)
        self.value = value
        self.root = root
        self.update()
        super().__init__(root, textvariable = self.stringVar)

    def update(self):
        self.root.after(REFRESH, self.update)    
        if len(self.value.y) == 0:
            return
        self.stringVar.set(self.text + '%.2f' % self.value.y[-1])

class EnumLastValue(tk.Label):
    def __init__(self, root, text, value, **kwargs):
        self.text = text
        self.stringVar = tk.StringVar()
        self.stringVar.set(text)
        self.value = value
        self.root = root
        self.update()
        super().__init__(root, textvariable = self.stringVar)

    def update(self):
        self.root.after(REFRESH, self.update)    
        if len(self.value.y) == 0:
            return
        self.stringVar.set(self.text + self.value.y[-1].name)

    
class ButtonFile(tk.Button):
    def __init__(self, root, **kwargs):
        self.on_click = kwargs["command"]
        kwargs["command"] = self.on_click2
        super().__init__(root, **kwargs)

    def on_click2(self):
        path = askopenfilename()
        self.on_click(path)

#specific widgets
class GyroGraph(GenericGraph):
    def __init__(self, root, gw):
        dataLists = [
            gw.data["rocket"]["estimate"]["gx"],
            gw.data["rocket"]["estimate"]["gy"],
            gw.data["rocket"]["estimate"]["gz"]
        ]
        super().__init__(root, gw.get_current_time, dataLists)
        self.ax.set_ylim(-6.28, 6.28)
        self.ax.set_title("rotation - radians/s")
        self.ax.axhline(0, color='gray')

class AccelerationGraph(GenericGraph):
    def __init__(self, root, gw):
        dataLists = [
            gw.data["rocket"]["telemetry"]["ax"],
            gw.data["rocket"]["telemetry"]["ay"],
            gw.data["rocket"]["telemetry"]["az"],
        ]
        super().__init__(root, gw.get_current_time, dataLists)
        self.ax.set_ylim(-5, 40)
        self.ax.set_title("acceleration - m/s²")
        self.ax.axhline(0, color='gray')

class AltitudeGraph(GenericGraph):
    def __init__(self, root, gw):
        super().__init__(root, gw.get_current_time, [gw.data["rocket"]["telemetry"]["altitude"]])
        self.ax.set_ylim(-10, 150)
        self.ax.set_title("altitude - m")

class FlashUsed(tk.Label):
    def __init__(self, root, gw, **kwargs):
        self.stringVar = tk.StringVar()
        self.stringVar.set("flash used:")
        self.root = root
        self.value = gw.data["rocket"]["telemetry"]["flash_address"]
        self.update()
        super().__init__(root, textvariable = self.stringVar)

    def update(self):
        self.root.after(REFRESH, self.update)    
        if len(self.value.y) == 0:
            return
        # 128MiBit
        self.stringVar.set('flash used: %.2f' % (100 * self.value.y[-1] /  16777216) + "%")