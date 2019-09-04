# Standard import
import os
import argparse
import builtins
import time
import math
import socket
from time import sleep

# Library imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import dronekit_sitl


# Helper function to connect to a vehicle on a given IP.
def connectVehicle():
	"""
	Returns a connection to a vehicle on a given IP.
	Inputs:
	  conn_str: IP and port address to connect to.
                    Ex: --connect :14550" listen UDP.
          Default is to connect to SITL on local host.
	"""
	# Get command line inputs from the user.
	# This is needed in case of connecting to the real vehicle.
	cmdopts = argparse.ArgumentParser(description="Parameters to connect plane")
	cmdopts.add_argument("--connect")
	args = cmdopts.parse_args()
	conn_str = args.connect

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
		return vehicle

#vehicle_obj = connectVehicle()
