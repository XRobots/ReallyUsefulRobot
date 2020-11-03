# ReallyUsefulRobot

## Hardware

CAD files are provided in STEP/STP format. This is the whole assembly as a solid format so you can easily edit it. You will need to load it into some CAD software and edit/export individual STLs for printing.

Tyres are printed in TPU, I used Ninjaflex. The rest in PLA printed with a combination of 0.5mm and 1.2mm nozzles.

Motors are 6374 150Kv BLDC motors from the ODrive store: https://odriverobotics.com/shop
Encoders and cables are the 8192 CPR encoders also from the ODrive store.
I used a 56V ODrive 3.6, the robot runs from a 24V LiPo, but it can be 25V+ when it's charged.
Drive belts are 590mm T5 12mm wide PU belts with steel tensioners made by Synchroflex.

Other electronics so far include:

-Teensy 4.1
-NVIDIA Xavier NX
-RPLIDAR A2
-24v and 11.1v lipos, switches and wiring etc

## Software

Obviously this isn't a comprehensive guide to ROS, you will need to undestand some core concepts before building this. The Robot runs ROS Melodic on Ubuntu 18.04. This is because the NVIDIA Xavier NX runs 18.04 by default.

On the robot:

The arduino code runs on the Teensy 4.1. This talks to the wheel encoders. It subscribres to the cmd_vel topic and publishes Odom, and TF for the robot's motions. The RPLIDAR A2 node publishes the scan topic: https://github.com/robopeak/rplidar_ros

The Teensy also talks to the ODrive on Serial1. In order to get the ROS Serial library running on the Teensy 4.1, and get both the ROS Serial library and the ODrive library to run together I had to make various modifications. I removed mentions of the Stream Operator which were in the original ODrive Arduino example: https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino/examples/ I also had to modify the ROS Serial library to add Teensy 4.1 support, and increase buffers so that the Odom message could be published over Serial. I've included both modified libraries as zip files.

The 'rur' and 'rur_description' ROS packages are installed on the robot, and everything is launched with the 'rur_bringup.launch' file. This launches the ROS Serial node to talk tot he Teensy 4.1, starts the laser, and also uses the Robot & Joint State Publishers to publish the transform in the URDF file so the laser is linked to the robot base in the right place.

On my remote workstation:

The 'rur_navigation' ROS package is installed on the remote workstation (although it could be run locally). You will also need to install the ROS navigation stack and slam_gmapping. You will also need to run rosdep to install any other dependencies: http://wiki.ros.org/rosdep

https://github.com/ros-planning/navigation/tree/melodic-devel
https://github.com/ros-perception/slam_gmapping/tree/melodic-devel

There is no launch file for gmapping, so having launched the rur_bringup package on the robot and launched Gmapping ( rosrun gmapping slam_gmapping scan:=scan ) you'll have to open Rviz and manually add/connect the topics for TF/Laser/Map etc. You can use the map server/saver command to save the map ( rosrun map_server map_saver -f ~/map )

After the map is complete you can launch the rur_navigation package which will open Rviz automatically. Use: roslaunch rur_navigation rur_navigation.launch map_file:=$HOME/map.yaml   to connect to the map you saved.




