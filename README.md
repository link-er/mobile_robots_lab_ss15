Mobile Robots lab, Bonn University, Summer Semester 2015
=============

##ardrone_moveit
Package, that was generated with the help of moveit_setup_assistant by urdf-file of the drone. Files config/ardrone_sensors.yaml, launch/ardrone_moveit_sensor_manager.launch.xml and launch/sensor_manager.launch.xml are modified in order to get point_cloud to the moveit space

##drone_lab
Contains several files for mini-project and sources of the experiments with drone.

1 - ardrone.urdf - description of the drone for moveit_setup

2 - calibration_front.cfg - config file for front camera of the Parrot AR.Drone type 1.0 for using with lsd_slam package

3 - path.txt - one of the generated trajectories from moveit planner

4 - pc.ply and pc1.ply - two example point clouds to play with

executables:

1 - analyze_image - trying to hover over a color spot

2 - detect-direction - trying to find the direction from the triangle and hover on it

3 - fly_and_show - simple displaying two images from front and bottom cameras

4 - land - send command for landing

5 - record_video - recording video from bottom camera, writes file to "/home/ropra/ros_world/LAB_SS15_GR05/out.avi" - just change it in case of need

6 - reset - send command for resetting drone

7 - takeoff - just get up and hover for a few seconds and then land

All the executables should be run with simple rosrun and roscore and ardrone_driver already started.

##experiment_03
Package for AMCL using for navigation the roomba robot to the desired position.
Should be run with launch file.

##llab_01
Package contains sources for wall following and legs following behaviour of roomba robot. scanner.cpp and smooth_derivative.cpp are intermediate programs just for getting info from laser scanner with some filters.
Should be run with launch files.

##Other sources
There is also a master's thesis about ardrone and the report for the mini-project of manipulating vehicle with the help of moveit.
