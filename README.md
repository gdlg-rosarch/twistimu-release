TwistIMU
========

Package Summary
---------------
ROS Node that convert IMU data to a Twist for manual robot drive.

- Maintainer status: maintained
- Maintainer: Gérald Lelong (gerald.lelong AT easymov DOT fr)
- Author: Gérald Lelong
- License: BSD
- Source: git git@gitlab.com:easymov/twistimu.git (branch: master)

Overview
--------

Use this node to drive your robot with an IMU device.
Try this android app : https://github.com/chadrockey/android_sensors_driver to use an android phone as controller.

X axis command velocity, Y axis command direction.

Quick start
-----------

Clone this repository into the source directory of a valid catkin workspace
then build it and don't forget to source the setup file in the devel directory of your catkin workspace.

`roslaunch twistimu turtle.launch`

`rosbag play bags/android_imu.bag`

Node
----

### twistimu.py ###

#### Subscribed Topics ####

_~imu_ (sensors_msgs/IMU)
 > IMU input

#### Published Topics ####

_~cmd_ (geometry_msgs/Twist)
 > Command output

#### Parameters  (Dynamic Reconfigure) ####

- _pitch_min_ [double]: angle where direction is full left
- _pitch_neutral_ [double]: angle where direction is straight
- _pitch_max_ [double]: angle where direction is full right
- _roll_min_ [double]: angle where output speed is full backward
- _roll_neutral_ [double]: angle where output speed is neutral
- _roll_max_ [double]: angle where output speed is full forward
- _speed_min_ [double]: maximum allowed backward speed
- _speed_neutral_ [double]: stop speed
- _speed_max_ [double]: maximum allowed forward speed
- _roll_dead_zone_ [double]: neutral speed zone size
- _pitch_dead_zone_ [double]: straight direction zone size
- _max_acceleration_ [double]: maximum allowed acceleration
- _track_ [double]: distance between the centerline of two roadwheels on the same axle

### Launch file ###

`roslaunch twistimu turtle.launch`

Publish to _/android/imu_ and control a turtle.