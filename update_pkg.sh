#!/usr/bin/env bash

cp -r prius_description /tmp/workspace/src
cp -r prius_msgs /tmp/workspace/src
cp -r car_demo /tmp/workspace/src
# cd /tmp/workspace && source /opt/ros/kinetic/setup.bash&& catkin_make

source /opt/ros/kinetic/setup.bash && source /tmp/workspace/devel/setup.bash
roslaunch car_demo demo.launch