import sys
sys.path.append("..")
#from .. import drone_utils

# This is a simple mission examples, based of well defined waypoints of GPS
# co-ordinates. One can edit the GPS co-ordinates below to make other missions.
# The basic flow of the code remains same for these kinds of flight missions.
def gps_based_mission():
	print("Starting Simple Mission of 4 pre-defined waypoints.")
	wp_now = vehicle.location.global_relative_frame
	mission1=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wp_now.lat,wp_now.lon,wp_now.alt)
	# Edit longitudes, latitudes, as per the starting location.
	mission2=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,88.602954920916062,13.76333133005619,10)
	# Edit longitudes, latitudes, as per the starting location.
	mission3=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,88.60296599879532, 23.7644932092553, 15)
	mission4=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0)
	
	# DOwnload the current list of drone commands
	print("Wait until downloading of current drone commmands complete..")
	drone_cmds = vehicle.commands
	drone_cmds.download()
	drone_cmds.wait_ready()
	
	# Clear the current list of commands
	print("Clear current list of commands: {}".format(drone_cmds))
	drone_cmds.clear()
	
	# Add mission specifc commands
	print("Add mission specific commands")
	drone_cmds.add(mission1)
	drone_cmds.add(mission2)
	drone_cmds.add(mission3)
	drone_cmds.add(mission4)
	
	print("Uploading mission commands to vehicle..")
	vehicle.commands.upload()
	
	arm_takeoff(10)
	
	# Looop, to wait till the flight mode is AUTO
	vehicle.mode = "AUTO"
	while vehicle.mode != "AUTO":
		print("Waiting for mission to Start..")
	
	# Check, if current altitude is near the desired altitude.
	while vehicle.location.global_relative_frame.alt > 2:
		print("Waiting for mission to complete..")
		battery_check()
		telemetry()
		sleep(2)
	print("-------Mission Completed---------\n")


# For Debug
gps_based_mission()
