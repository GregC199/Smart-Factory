#!/bin/bash

cd gazebo_plugin
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch agv_ppot turtlebot3_AGV_PPot.launch
