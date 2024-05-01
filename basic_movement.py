from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect') #default='127.0.0.1:14550'
args = parser.parse_args()
connection_string = args.connect
if not connection_string:
	import dronekit_sitl
	sitl = dronekit_sitl.start_default()
	connection_string =sitl.connection_string()

# Connect to the Vehicle
# If using dronekit-sitl:
vehicle = connect(connection_string, wait_ready = True)
# If using physical hardware:
# vehicle = connect(args.connect, baud=921600, wait_ready=True)
# 921600 is the baudrate that you have set in the mission plannar or qgc
print('Connecting to vehicle on: %s' % args.connect)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)
    
def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see: 
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
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


# Initialize the takeoff sequence to 15m
arm_and_takeoff(4)

print("Take off complete")

# Hover for 5 seconds
time.sleep(5)

print("Set airspeed to 10m/s (max).")
vehicle.airspeed = 10

print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
DURATION = 20 #Set duration for each segment.

print("North 50m, East 0m, 10m altitude for %s seconds" % DURATION)
goto_position_target_local_ned(50,0,-10)
print("Point ROI at current location (home position)") 
# NOTE that this has to be called after the goto command as first "move" command of a particular type
# "resets" ROI/YAW commands
set_roi(vehicle.location.global_relative_frame)
time.sleep(DURATION)

print("North 50m, East 50m, 10m altitude")
goto_position_target_local_ned(50,50,-10)
time.sleep(DURATION)

print("Point ROI at current location")
set_roi(vehicle.location.global_relative_frame)

print("North 0m, East 50m, 10m altitude")
goto_position_target_local_ned(0,50,-10)
time.sleep(DURATION)

print("North 0m, East 0m, 10m altitude")
goto_position_target_local_ned(0,0,-10)
time.sleep(DURATION)

print("Now let's land at home")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
