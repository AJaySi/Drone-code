import os
import argparse
import time
import math
import socket
from time import sleep

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, APIException, Command
import dronekit_sitl
from pymavlink import mavutil


# Helper function to connect to a vehicle on a given IP.
def connectVehicle():
	"""
	Returns a connection to a vehicle on a given IP.
	Returns a sitl object.
	TBD: Take flags as simulated or real vehicle.
	"""
	# Get command line inputs from the user.
	# This is needed in case of connecting to the real vehicle.
	cmdopts = argparse.ArgumentParser(description="Parameters to connect plane")
	cmdopts.add_argument("--connect")
	args = cmdopts.parse_args()
	conn_str = args.connect
	sitl = ''

	# Connect sitl to default localhost.
	# This will also launch SITL and connect to it on 127.0.0.1:14551	
	if not conn_str:
		sitl = dronekit_sitl.start_default()
		conn_str = sitl.connection_string()

	# Now, we have connection IP, Connect to the Vehicle.
	print("Connecting to vehicle on: {}".format(conn_str))

	try:
		vehicle = connect(conn_str, wait_ready=True)
	# Bad TCP connection
	except socket.error:
		print("No server exists!")
	# Bad TTY connection
	except exceptions.OSError as e:
		print("No serial exists!")
	# API Error
	except APIException:
		print("Timeout!")
	# Other error
	except:
		print("Some other error!")
	else:
		print("Connected successfully.")
		return vehicle, sitl

# Util function to get/set the home location.
def set_home_loc(home_loc=''):

	present_loc = LocationGlobal(18.60277104448721,73.76351987984208, 10)
	vehicle.home_location = present_loc

	# If home_loc is null, then print/log the current home location.
	if not home_loc:
		# Get Vehicle Home location - will be `None` until first set by autopilot
		while not vehicle.home_location:
			cmds = vehicle.commands
			cmds.download()
			cmds.wait_ready()
			if not vehicle.home_location:
				print ("Waiting for home location ...")

	# We have a home location, so print it!
	print ("Home location: {}".format(vehicle.home_location))
	print("Vehicle global frmae: {}".format(vehicle.location.global_frame))
	print ("Location in Global frame: {}".format(vehicle.location.global_frame))
        print ("Location in global relative frame: {}".format(vehicle.location.global_relative_frame))
        print ("Location in Local frame: {}".format(vehicle.location.local_frame))


#Launch sequence
# Accepts altitude in meters to takeoff to.
def arm_takeoff(tgt_alt):
	
	print("Poll on Vehicle.is_armable until the vehicle is ready to arm.")
	while not vehicle.is_armable:
		print("Waiting for vehicle to initialise..")
		sleep(1)

	# Set the mode to GUIDED
	print("Set and wait until the Vehicle.mode is GUIDED.")
	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode != "GUIDED":
		print("Waiting to enter/confirm GUIDED mode..")
		sleep(1)

	# Arm the vehicle for takeoff.
	print("Set Vehicle.armed to True and poll until the vehicle is armed.")
	vehicle.armed = True
	# Check if the state is set to True/Armed
	while not vehicle.armed:
		print("Waiting for arm confirmation")
		sleep(1)

	print("Vehicle Ready for takeoff !")
	print("Simple_takeoff to achieve altitude of {} meters.".format(tgt_alt))
	vehicle.simple_takeoff(tgt_alt)
	while True:
		print("Climbing to Altitude: {}".format(vehicle.location.global_relative_frame.alt))
		#Break and return from function just below target altitude. Error margin of 5%.
		if vehicle.location.global_relative_frame.alt>=.95*tgt_alt:
			print("Reached target altitude")
			break
		time.sleep(1)

	print("95% of target height achieved.")
	return None


# Common function to get the telemetry details.
# TBD: Need to log it in a log.
def telemetry():

	# Get some vehicle attributes (state)
	print("Get some vehicle attribute values:")
	print("GPS: {}".format(vehicle.gps_0))
	print("Attitude: {}".format(vehicle.attitude))
	print("Velocity:{}".format(vehicle.velocity))
	print("Battery: {}".format(vehicle.battery))
	print("Last Heartbeat: {}".format(vehicle.last_heartbeat))
	print("Is Armable?: {}".format(vehicle.is_armable))
	print("System status: {}".format(vehicle.system_status.state))
	print("Mode: {}".format(vehicle.mode.name))


# Check the battery percentage and land if less than 9.9 
def battery_check():
    if (vehicle.battery < 9.9):
        print ("ERROR: Battery Low. Landing")
        land()
    else:
        print ("Battery: %s".format(vehicle.battery))


# Common function to calculate the distance between 2 GPS co-ordinates
def get_dist_meters(target_loc, destination_loc):
	"""
	Returns the ground distance in metres between two Location objects.
	
 	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = target_loc.lat - destination_loc.lat
	dlon = target_loc.lon - destination_loc.lon

	# Calculate the hypotoneus to get distance inbetween
	return(math.sqrt((dlat * dlat) + (dlon * dlon))*1.13195e5)


# Helper function to move drone to target location
def goto(targetloc):
	# Get the target distance in meters
	target_dist = get_dist_meters(targetloc, vehicle.home_location)
	print("Flying to destination {} at a distance of {}".format(targetloc, target_dist))

	
	vehicle.simple_goto(targetloc)
	# Wait till the drone completes the travel
	while vehicle.mode.name == "GUIDED":
		
		currentDist = get_dist_meters(targetloc, vehicle.location.global_relative_frame)
		print("Distance Covered is {} meters {} and TARGET: {}".format(currentDist, vehicle.location.global_relative_frame, targetloc))
		# Accuracy of 1% within the target location
		if currentDist < target_dist * 0.01:
			print("Waypoint Reached, Current distance to target: {}m".format(currentDist))
			time.sleep(2)
			break
		# Check distance to destination, every second
		time.sleep(1)
	
	# Return once at target/destination
	return None


# Common function to land the plane.
def land():
	print("Preparing to LAND the vehicle")
	vehicle.mode = VehicleMode("LAND")
	
	# Wait till plane enters the LAND mode
	while vehicle.mode != "LAND":
		print("Waiting to enter 'LAND' flight Mode")
	
	#while not vehicle.location.global_relative_frame.alt==0:
        #	if vehicle.location.global_relative_frame.alt < 2:
        #    		set_velocity_body(vehicle,0,0,0.1)
	
	print("Vehicle landing Completed.")



# Function to move the vehicle with velocity components in LOCAL Frame
def send_local_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFEST_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # vx, vy, vz velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    #time.sleep(1)
    vehivle.flush()


# Function to move vehicle with velocity components in GLOBAL referance, True North.
def send_global_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    #for x in range(0,duration):
    vehicle.send_mavlink(msg)
    #time.sleep(1)
    vehicle.flush()  


"""
Convenience functions for sending immediate/guided mode commands to control the Copter.
The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.
The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""

def condition_yaw(yawdegrees, relative):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour. 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        yawdegrees,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)
    #vehicle.flush()


# Required init for condition_yaw function.
def dummy_condition_yaw_init():
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    Basically, get the present location of drone, needed for yaw_condition to work properly.
    """
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    
    aLocation = LocationGlobalRelative(lat, lon, alt)
    
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
	

def velocity_flight():
	cntr = 0
	# fly forward for 5 seconds
	while cntr < 5:
		send_local_velocity(5,0,0)
		time.sleep(1)
		cntr += 1
		print("Moving North, relative to drone front..")
		
	sleep(1)
	
	# fly south, in local referance
	cntr = 0
	while cntr < 5:
		send_global_velocity(0,-5,0)
		time.sleep(1)
		cntr += 1
		print("Moving West, realtive to drone front..")
	
	sleep(1)
	
	cntr = 0
	# fly forward for 5 seconds
	while cntr < 5:
		send_global_velocity(5,0,0)
		time.sleep(1)
		cntr += 1
		print("Moving True North, relative to drone front..")
		
	sleep(1)
	
	# fly south, in local referance
	cntr = 0
	while cntr < 5:
		send_global_velocity(0,-5,0)
		time.sleep(1)
		cntr += 1
		print("Moving True West, realtive to drone front..")
	
	sleep(1)
	
	while True:
		sleep(1)
