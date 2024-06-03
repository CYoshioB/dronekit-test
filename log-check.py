from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
#import RPi.GPIO as GPIO
import time
from datetime import datetime
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc

# Global values to be updated everytime callback function is invoked
global id, iso_date, lat, lon

"""
def location_callback(self, attr_name, newVal):
    # Logic to check which ID is recognized
    if GPIO.input(1) == 1 and GPIO.input(2) == 0:
        id = 22
    elif GPIO.input(2) == 1 and GPIO.input(1) == 0:
        id = 23
    else:
        return  # If no Aruco is recognized, break out of the callback
    # Update current time and coordinates after Aruco is recognized
    iso_date = datetime.now().isoformat()
    lat = newVal.lat
    lon = newVal.lon
    # Print out the updated values
    print("RTXDC_24")
    print("CSULB_UAV_WaterBlast!_%d_%sZ_%.6f_%.6f" % (id, iso_date, lat, lon))

vehicle.add_attribute_listener('location.global_frame', location_callback)
"""

# Dummy Values
id = 23
iso_date = datetime.now().isoformat()
lat = 35.678901234
lon = 125.1029384756

print("RTXDC_24")
print("CSULB_UAV_WaterBlast!_%d_%sZ_%.6f_%.6f" % (id, iso_date, lat, lon))