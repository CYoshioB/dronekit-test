from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import RPi.GPIO as GPIO
from datetime import datetime
import time
import math
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def heading_adjusted_goto(theta, north, east, down):
   # Transform absolute NED coordinates into XYZ relative to angle from North
   nPrime = north*math.cos(math.radians(-theta)) + east*math.sin(math.radians(-theta))
   ePrime = east*math.cos(math.radians(-theta)) - north*math.sin(math.radians(-theta))
   dPrime = down

   # Send Mavlink msg to go to adjusted local coordinates
   goto_position_target_local_ned(nPrime, ePrime, dPrime)

# Global variables for field conditions
DURATION = 8
nLENGTH = 9.144 / 21 # 0.4572m, which is 1.5ft
eLENGTH = 9.144      # Equivalent to 30 ft
HEADING = vehicle.heading

# Globals that are updated everytime callback function is invoked
global id, iso_date, lat, lon

# Global position callback for logging water blasts
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
    print("")

# Initialize the callback function
vehicle.add_attribute_listener('location.global_frame', location_callback)

# Arm and take off to value of ALTITUDE
ALTITUDE = 2
arm_and_takeoff(ALTITUDE)

print("Begin initial zig-zag flight pattern")
heading_adjusted_goto(HEADING, nLENGTH * 1, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 2, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 3, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 4, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 5, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 6, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 7, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 8, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 9, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 10, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 11, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 12, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 13, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 14, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 15, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 16, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 17, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 18, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 19, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 20, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 21, eLENGTH, -ALTITUDE)
time.sleep(DURATION)

print("Initial pattern complete.")
heading_adjusted_goto(HEADING, nLENGTH * 21, 0, -ALTITUDE)
time.sleep(5)

print("Now begin reverse flight pattern.")
heading_adjusted_goto(HEADING, nLENGTH * 20, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 19, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 18, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 17, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 16, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 15, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 14, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 13, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 12, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 11, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 10, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 9, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 8, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 7, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 6, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 5, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 4, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 3, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 2, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 1, 0, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 0, eLENGTH, -ALTITUDE)
time.sleep(DURATION)
heading_adjusted_goto(HEADING, nLENGTH * 0, 0, -ALTITUDE)
time.sleep(DURATION)

print("Flight Routine complete.")
time.sleep(5)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
print("Close vehicle object")
vehicle.close()
print("Completed")
