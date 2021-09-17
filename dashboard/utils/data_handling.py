import influxdb
from influxdb import InfluxDBClient
from utils.definitions import INFLUX_NAME

import datetime

class TimeSeries():
    def __init__(self):
        self.x = []
        self.y = []

class Data():
    def __init__(measurement, value):
        self.measurement = measurement
        self.value = value

def init_db(reset = False):
    try:
        client = InfluxDBClient(host='localhost', port='8086')
        client.create_database(INFLUX_NAME)
        client.switch_database(INFLUX_NAME)
        return client
    except:
        print("COULD NOT CONNECT TO INFLUX. NO DATA WILL BE WRITTEN TO THE SERVER")
        return 0

def write_data_db(sender, field, value, client, time = False):
    points = []
    
    point = {
        "measurement": "rocket",
        "fields": {
            name: value,
        }
    }
    if time:
        point["time"] = time
    points.append(point)
    try:
        if client:
            client.write_points(points)
    except:
        print("could not write to the influx server")
    return 0