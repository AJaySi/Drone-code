----
For details on installing and getting started refer to file install.README
----

# Introduction

This is a repo for drone programming utils and libraries, mostly personal use.

It covers mainly Ardupilot, MAVlink and dronekit stack for copter.
For simulation, dronekit-sitl is used.

The 'drone_utils.py' contains common functions to craft autonomous fligt missions
based on GPS co-ordinates and velocities. As an example, simple_mission.py, 
gps_mission.py and velocity_based_mission.py are provided.

----

##Running Examples:

1). If on linux, fire the setup.sh script. address the failures, if any.
1). If on Windows, do a pip3 install requirements.txt

2). # What does sim_vehicle.py do? 
(Script Location: ardupilot/Tools/autotest/sim_vehicle.py)

Launches a simulated drone with four main steps:
1. Detects what vehicle to build for (Copter, Plane, Rover etc)
2. Compiles all source code into an executable
3. Launches the simulated drone by running the SITL executable
4. Launches MAVProxy to communicate with/command the SITL drone

Command: sim_vehicle.py --map --console -v ArduCopter

In the MAV terminal/window : try the following Commands:

mode GUIDED
arm throttle
take 10

#Enter to get the prompt back. 
Use the map and right click to choose 'fly here'. 
Check the equivalent command in the MAV windows.

Telemetry is in the thrid window.
After pre-requisite of installing as per install.README, execute as follows:

----

## examples folder

This repo has some handy scripts in examples folder. Have a look for using it boilerplate-template.
