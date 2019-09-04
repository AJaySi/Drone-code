import drone_utils

# Connect to SITL
print("Connecting to the SITL vehicle")
vehicle, sitl = connectVehicle()

set_home_loc()
arm_takeoff(10)
dummy_condition_yaw_init()
sleep(2)

condition_yaw(30, 1)
print("Yaw 30 degrees Relative to current position")
sleep(7)

print("Yawing to True North")
condition_yaw(0, 0)

print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl_obj.stop()
print("Mission Completed")

